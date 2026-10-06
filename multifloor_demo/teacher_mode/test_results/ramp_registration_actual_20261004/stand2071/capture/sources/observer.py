#!/usr/bin/env python3
"""Read-only ROS79 actual full-cloud/SLAM/accelerometer collector. No publishers."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
from capture import CaptureWriter,finish,atomic_json
from cloud import metadata_from_message


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def stamp_ns(stamp): return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def verify_freeze():
    data=json.loads((HERE/'freeze.json').read_text())
    for name,expected in data['source_hashes'].items():
        if digest(HERE/name)!=expected: raise RuntimeError('Registration source changed: '+name)
    for name,ref in data.get('source_contract_refs',{}).items():
        if digest(ref['path'])!=ref['sha256']:raise RuntimeError('Registration source/API contract changed: '+name)
    return data


def environment(manifest_path=None):
    names={k:v for k,v in os.environ.items() if k.startswith(('ROS_','RMW_','FAST','GZ_'))
           or k in ('SKIP_DEFAULT_XML','CYCLONEDDS_URI','LD_LIBRARY_PATH','AMENT_PREFIX_PATH')}
    if os.environ.get('ROS_DOMAIN_ID')!='79': raise RuntimeError('Root must explicitly launch this observer in ROS_DOMAIN_ID=79')
    if manifest_path is not None:
        manifest_path=manifest_path.resolve(); data=json.loads(manifest_path.read_text())
        if data.get('schema')!='cloud_transport_experiment/v1': raise ValueError('unexpected environment manifest schema')
        for key,value in data['environment'].items():
            if os.environ.get(key)!=str(value): raise ValueError('Observer environment differs from frozen transport manifest: '+key)
        for key in data.get('remove_environment_keys',[]):
            if key not in data['environment'] and key in os.environ: raise ValueError('Observer inherited forbidden transport environment key: '+key)
        if digest(data['xml_path'])!=data['xml_sha256']: raise ValueError('transport XML changed')
        for path,expected in data.get('source_hashes',{}).items():
            if digest(path)!=expected:raise ValueError('transport source changed: '+path)
        names['environment_manifest']={'path':str(manifest_path),'sha256':digest(manifest_path),'contents':data}
    return names


def source_run_contract(run):
    """Record producer configuration only. Never reads world/SDF/Actor geometry."""
    run=Path(run).resolve();names=('sensor_contract.json','navigation_fastlivo.yaml','navigation_camera.yaml')
    sources={str(run/n):digest(run/n)for n in names}
    sensors=json.loads((run/'sensor_contract.json').read_text())
    if sensors['imu']['frame']!='imu_link':raise ValueError('unsupported actual IMU frame; new explicit contract needed')
    return {'run':str(run),'source_hashes':sources,'imu_frame':'imu_link',
        'imu_orientation_metadata_used':False,'world_or_actor_geometry_read':False,
        'producer_execution_identity':'Root-owned run runtime_manifest/process tree must independently confirm execution; config hash alone is not proof'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--duration-s',type=float,default=60)
    parser.add_argument('--wall-timeout-s',type=float,default=180)
    parser.add_argument('--environment-manifest',type=Path)
    parser.add_argument('--source-run',type=Path,required=True)
    args,ros_args=parser.parse_known_args()
    if not 1<=args.duration_s<=600 or not args.duration_s<args.wall_timeout_s<=1800: parser.error('invalid bounded observation duration')
    frozen=verify_freeze(); env=environment(args.environment_manifest);producer=source_run_contract(args.source_run)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,HistoryPolicy,ReliabilityPolicy,DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2,Imu
    from rclpy.utilities import get_rmw_implementation_identifier
    from rclpy.parameter import Parameter
    writer=CaptureWriter(args.output.resolve())
    base={'schema':'actual_ramp_cloud_capture/v1','started_monotonic_wall':time.monotonic(),
          'duration_sim_s':args.duration_s,'wall_timeout_s':args.wall_timeout_s,'source_freeze':frozen,
          'freeze_sha256':digest(HERE/'freeze.json'),'environment':env,'topics':{
          'full_cloud':'/cloud_registered_full','body_odom':'/demo/slam/body_odom',
          'imu_slam_odom':'/aft_mapped_to_init','imu':'/livox/imu','clock':'/clock'},
          'qos':{'history':'KEEP_LAST','depth':1,'reliability':'BEST_EFFORT'},
          'limits':{'queue_records':512,'queue_bytes':64*1024**2,'archive_bytes':8*1024**3,'frame_bytes':32*1024**2},
          'navigation_inputs':['actual full registered cloud','actual SLAM poses','actual accelerometer/gyro'],
          'truth_topics_subscribed':[],'publishers':[],'service_clients':[]}
    base['producer_configuration']=producer
    class Observer(Node):
        class LocalParameterEvents:
            # rclpy Node unconditionally requests /parameter_events. Suppress
            # only this internal bookkeeping channel locally, without creating
            # any DDS Publisher or changing the imported Node class globally.
            def publish(self,message):pass
        def create_publisher(self,msg_type,topic,*args,**kwargs):
            if topic=='/parameter_events' and msg_type.__name__=='ParameterEvent':return self.LocalParameterEvents()
            raise RuntimeError('Read-only collector forbids all DDS publishers')
        def __init__(self):
            super().__init__('actual_ramp_registration_observer',enable_rosout=False,start_parameter_services=False,
                enable_logger_service=False,parameter_overrides=[Parameter('use_sim_time',value=False),
                    Parameter('start_type_description_service',value=False)])
            self.clock_ns=None;self.clock_wall=None;self.first_clock_ns=None;self.sequence=0;self.failure=None
            self.subscriptions_held=[]
        def start_subscriptions(self):
            qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            for name,topic in base['topics'].items():
                typ=Clock if name=='clock' else PointCloud2 if name=='full_cloud' else Imu if name=='imu' else Odometry
                def callback_factory(stream):
                    def callback(message,info):self.receive(stream,message,info)
                    return callback
                self.subscriptions_held.append(self.create_subscription(typ,topic,callback_factory(name),qos))
        def receive(self,name,msg,info=None):
            wall=time.monotonic();self.sequence+=1
            if name=='clock':
                ns=stamp_ns(msg.clock)
                if self.clock_ns is not None and ns<self.clock_ns: self.failure='actual simulation clock reversed';return
                self.clock_ns=ns;self.clock_wall=wall
                if self.first_clock_ns is None:self.first_clock_ns=ns
                # 200Hz clock does not need disk rows; original clock at every sensor receipt is retained.
                return
            row={'schema':'actual_ramp_sensor_receipt/v1','sequence':self.sequence,'stream':name,
                 'source_topic':base['topics'][name],'original_stamp_ns':stamp_ns(msg.header.stamp),
                 'frame_id':msg.header.frame_id,'received_monotonic_wall':wall,
                 'received_ros_clock_ns':self.clock_ns,'received_clock_wall_age_s':None if self.clock_wall is None else wall-self.clock_wall}
            if info is not None:
                row['DDS_message_info']={k:int(getattr(info,k))if getattr(info,k,None)is not None else None
                    for k in ('source_timestamp','received_timestamp','publication_sequence_number','reception_sequence_number')}
                gid=getattr(info,'publisher_gid',None)
                row['DDS_message_info']['publisher_gid_hex']=None if gid is None else bytes(gid).hex()
            if name=='full_cloud':
                row.update(metadata_from_message(msg));raw=bytes(msg.data)
            elif name=='imu':
                a=msg.linear_acceleration;g=msg.angular_velocity
                row.update(linear_acceleration=[a.x,a.y,a.z],angular_velocity=[g.x,g.y,g.z],imu_orientation_field_used=False);raw=None
            else:
                p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist.linear
                row.update(child_frame_id=msg.child_frame_id,position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],
                           body_linear_velocity=[v.x,v.y,v.z],pose_covariance=list(msg.pose.covariance),twist_covariance=list(msg.twist.covariance));raw=None
            try:writer.append(row,raw)
            except Exception as exc:self.failure=f'{type(exc).__name__}: {exc}'
    node=None;failure=None;reason='duration_complete'
    try:
        (writer.output/'sources').mkdir()
        for name in frozen['source_hashes']:shutil.copyfile(HERE/name,writer.output/'sources'/name)
        shutil.copyfile(HERE/'freeze.json',writer.output/'sources/freeze.json')
        (writer.output/'sources/dependencies').mkdir()
        for name,ref in frozen.get('source_contract_refs',{}).items():shutil.copyfile(ref['path'],writer.output/'sources/dependencies'/name)
        (writer.output/'producer_configuration').mkdir()
        for path in producer['source_hashes']:shutil.copyfile(path,writer.output/'producer_configuration'/Path(path).name)
        rclpy.init(args=ros_args)
        node=Observer();node.start_subscriptions();base['rmw_implementation_identifier']=get_rmw_implementation_identifier()
        base['actual_node_interface_counts']={'publishers':len(list(node.publishers)),
            'services':len(list(node.services)),'clients':len(list(node.clients)),'subscriptions':len(list(node.subscriptions))}
        if any(base['actual_node_interface_counts'][n]for n in ('publishers','services','clients')):
            raise RuntimeError('Unexpected writable ROS interface on read-only collector')
        base['pid']=os.getpid();(writer.output/'process_maps.txt').write_text(Path('/proc/self/maps').read_text())
        atomic_json(writer.output/'observer_started.json',base)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if node.failure or writer.error:raise RuntimeError(node.failure or writer.error)
            now=time.monotonic()
            if now-base['started_monotonic_wall']>args.wall_timeout_s:raise RuntimeError('bounded observer wall deadline exceeded')
            if node.clock_wall is not None and now-node.clock_wall>3:raise RuntimeError('actual clock stalled over 3 wall seconds')
            if node.clock_ns is not None and node.clock_ns-node.first_clock_ns>=round(args.duration_s*1e9):break
        else:reason='context_shutdown'
    except KeyboardInterrupt:reason='interrupted'
    except Exception as exc:
        if not rclpy.ok() and type(exc).__name__ in ('ExternalShutdownException','RCLError'):reason='context_shutdown'
        else:failure=f'{type(exc).__name__}: {exc}'
    finally:
        if node is not None:
            base['last_actual_clock_ns']=node.clock_ns;base['first_actual_clock_ns']=node.first_clock_ns
            try:node.destroy_node()
            except Exception as exc:failure=f'{failure or ""}; destroy_node: {exc}'
        if rclpy.ok():
            try:rclpy.shutdown()
            except Exception as exc:failure=f'{failure or ""}; shutdown: {exc}'
        try:
            verify_freeze();environment(args.environment_manifest)
            if source_run_contract(args.source_run)!=producer:raise RuntimeError('actual producer configuration changed during capture')
        except Exception as exc:failure=f'{failure or ""}; source integrity: {exc}'
        finish(writer,base,failure=failure,stop_reason=reason)


if __name__=='__main__':main()
