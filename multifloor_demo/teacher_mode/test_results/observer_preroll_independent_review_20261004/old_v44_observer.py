#!/usr/bin/env python3
"""New scoped experiment only: physical box mover or passive sensor evidence.

Default is prepare-only. The mover's sole service can change moving_obstacle;
neither role publishes navigation, robot poses, joints, flags or point clouds.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode')
sys.path.insert(0,str(ROOT/'navigation'))
from runtime_io import EvidenceWriter,latest_sensor_qos


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Tail:
    def __init__(self,path):self.path=Path(path);self.offset=0;self.partial=b'';self.latest=None
    def poll(self):
        if not self.path.is_file():return self.latest
        with self.path.open('rb')as stream:
            if self.path.stat().st_size<self.offset:raise RuntimeError('Evidence stream was truncated: '+str(self.path))
            stream.seek(self.offset);block=stream.read(2*1024*1024);self.offset=stream.tell()
        parts=(self.partial+block).split(b'\n');self.partial=parts.pop()
        if len(self.partial)>2*1024*1024:raise RuntimeError('Evidence line exceeds bounded input size')
        for line in parts:
            if not line.strip():continue
            data=json.loads(line)
            if data.get('kind')=='observer_end':continue
            self.latest=data
        return self.latest


def fresh(stamp_s,wall_s,sim_now,wall_now):
    return all(isinstance(x,(int,float))and math.isfinite(x)for x in(stamp_s,wall_s,sim_now,wall_now))and -.05<=sim_now-stamp_s<.3 and 0<=wall_now-wall_s<.3


class Trigger:
    def __init__(self,protocol):self.rule=protocol['trigger'];self.first=None;self.last=None;self.along=None;self.samples=0;self.missed_window=False
    def observe(self,pose,telemetry,nav,anchor,sim,wall):
        if not all(isinstance(x,dict)for x in(pose,telemetry,nav,anchor)):self.first=None;self.samples=0;return False
        stamp=pose.get('stamp_ns',-1)/1e9;v=pose.get('body_velocity',[]);p=pose.get('position',[])
        envelope=telemetry.get('navigation_envelope',{});command=telemetry.get('requested',[])
        valid=(pose.get('frame_id')=='camera_init'and pose.get('child_frame_id')=='demo_slam_body'
            and fresh(stamp,pose.get('received_monotonic_wall'),sim,wall)
            and fresh(telemetry.get('world_sim_time'),envelope.get('read_monotonic_wall'),sim,wall)
            and nav.get('state')=='running'and nav.get('waypoint_index')==self.rule['waypoint_index']
            and 0<=wall-nav.get('monotonic_wall',-math.inf)<.3
            and envelope.get('read_status')=='accepted'and envelope.get('healthy')is True
            and len(v)==3 and len(p)==3 and len(command)==3
            and all(isinstance(x,(int,float))and math.isfinite(x)for x in v+p+command)
            and v[0]>=self.rule['slam_body_forward_min_mps']and command[0]>=self.rule['accepted_forward_min_mps']
            and anchor.get('source')=='/demo/slam/body_odom'and anchor.get('ground_truth_navigation_used')is False)
        if valid:
            dx=p[0]-anchor['origin'][0];dy=p[1]-anchor['origin'][1]
            self.along=math.cos(anchor['yaw'])*dx+math.sin(anchor['yaw'])*dy
            if self.along>self.rule['slam_along_max_m']:self.missed_window=True
            valid=self.rule['slam_along_min_m']<=self.along<=self.rule['slam_along_max_m']
        if not valid:self.first=None;self.last=None;self.samples=0;return False
        if self.last is None or stamp>self.last:
            if self.last is not None and stamp-self.last>.200000001:self.first=None;self.samples=0
            if self.first is None:self.first=stamp
            self.last=stamp;self.samples+=1
        return self.samples>=self.rule['min_distinct_slam_poses']and self.last-self.first>=self.rule['continuous_sim_s']-1e-9


class Program:
    def __init__(self,protocol):
        self.protocol=protocol;self.phase='waiting';self.start=None;self.hold_start=None;self.leave_start=None
        self.last_sim=None;self.failure=None;self.clear_start=None;self.target=list(protocol['initial_position_world'])
    def step(self,sim,trigger,actual_pose,wall):
        if self.last_sim is not None and sim<self.last_sim:self.failure='Actual ROS clock moved backward'
        self.last_sim=sim
        if self.failure:self.phase='failed';return self.target
        initial=self.protocol['initial_position_world'];blocked=self.protocol['blocked_position_world']
        tolerance=self.protocol['actual_pose_position_tolerance_m']
        actual_valid=(isinstance(actual_pose,dict)and actual_pose.get('status')=='actual_observed'
            and actual_pose.get('model')=='moving_obstacle'and actual_pose.get('navigation_input')is False
            and len(actual_pose.get('position',[]))==3
            and fresh(actual_pose.get('stamp_ns',-1)/1e9,actual_pose.get('received_monotonic_wall'),sim,wall))
        def at(position):return actual_valid and math.dist(actual_pose['position'],position)<=tolerance
        span=abs(blocked[1]-initial[1])/self.protocol['motion_speed_mps']
        if self.phase=='waiting'and trigger:
            if not at(initial):self.failure='Actual model initial pose missing/stale/mismatched';self.phase='failed';return self.target
            self.start=sim;self.phase='entering'
        if self.phase=='entering':
            alpha=max(0.,min(1.,(sim-self.start)/span));self.target=[initial[0],initial[1]+alpha*(blocked[1]-initial[1]),initial[2]]
            if alpha==1 and at(blocked):self.phase='blocking';self.hold_start=sim
            elif sim-self.start>span+2.:
                self.failure='Requested entry never confirmed by actual model pose';self.phase='failed'
        if self.phase=='blocking':
            self.target=list(blocked)
            if not at(blocked):self.failure='Actual blocking pose missing/stale/changed';self.phase='failed'
            elif sim-self.hold_start>=self.protocol['blocking_duration_sim_s']:
                self.phase='leaving';self.leave_start=sim
        if self.phase=='leaving':
            alpha=max(0.,min(1.,(sim-self.leave_start)/span));self.target=[initial[0],blocked[1]+alpha*(initial[1]-blocked[1]),initial[2]]
            if alpha==1 and at(initial):self.phase='clear';self.clear_start=sim
            elif sim-self.leave_start>span+2.:
                self.failure='Requested withdrawal never confirmed by actual model pose';self.phase='failed'
        return self.target


def verify_scope(run,protocol_path):
    if run.parent!=(ROOT/'runs').resolve():raise RuntimeError('Only a new independent Teacher run is eligible')
    scope_path=run/'dynamic_scope.json';scope=json.loads(scope_path.read_text())
    if (scope.get('schema')!='teacher_dynamic_scope/v1'or scope.get('status')!='experimental_unverified'
        or scope.get('allowed')is not True or scope.get('navigation_ground_truth_used')is not False
        or scope.get('experiment')!='finite_flat_dynamic_stop_resume_v1'or scope.get('run_dir')!=str(run)):
        raise RuntimeError('Missing explicit new dynamic experiment scope; old excluded profile is insufficient')
    from scoped_profile import verify_scope as verify_navigation_scope
    navreceipt=verify_navigation_scope(run/'navigation_scope.json')
    if scope.get('navigation_experiment')!=navreceipt['experiment']or scope.get('navigation_max_yaw_rate_radps')!=navreceipt['profile']['max_yaw_rate_radps']:
        raise RuntimeError('Explicit dynamic navigation variant does not match parent scope')
    parent=json.loads((run/'navigation_scope.json').read_text())
    if 'dynamic_obstacle_success_claim'in parent.get('excluded_scenarios',[]):
        raise RuntimeError('Parent navigation scope still excludes dynamic obstacle validation')
    refs=scope.get('references',{})
    mandatory=[scope_path.parent/'world.sdf',protocol_path,Path(__file__),run/'navigation_scope.json']
    if any(str(p.resolve())not in refs for p in mandatory):raise RuntimeError('Dynamic scope omits mandatory world/protocol/runtime/parent source')
    for name,digest in refs.items():
        if sha(name)!=digest:raise RuntimeError('Dynamic experiment source changed: '+name)
    if scope.get('protocol_sha256')!=sha(protocol_path):raise RuntimeError('Dynamic protocol mismatch')
    basis=scope.get('finite_navigation_basis',[])
    if len(basis)!=3:raise RuntimeError('Exactly three actual finite navigation repeat receipts required')
    for item in basis:
        summary=Path(item['path']);data=json.loads(summary.read_text())
        if sha(summary)!=item['sha256']or data.get('status')!='passed'or data.get('levels',{}).get('finite_flat_navigation')!='passed':raise RuntimeError('A finite navigation basis is not passed/preserved')
    if (os.environ.get('ROS_DOMAIN_ID')!='79'or os.environ.get('GZ_PARTITION')!='teacher_'+run.name
        or os.environ.get('DEMO_RUN_DIR')!=str(run)):raise RuntimeError('Wrong owned ROS/run/partition environment')
    protocol=json.loads(protocol_path.read_text());w=ET.parse(run/'world.sdf').getroot().find('world')
    if w.get('name')!='teacher_demo':raise RuntimeError('Unknown world')
    model=w.find("model[@name='moving_obstacle']")
    if model is None or model.findtext('static')!='true':raise RuntimeError('Original collidable obstacle missing')
    actual=[float(x)for x in model.findtext('pose').split()][:3]
    size=[float(x)for x in model.findtext('link/collision/geometry/box/size').split()]
    if actual!=protocol['initial_position_world']or size!=protocol['box_size_m']:raise RuntimeError('Actual asset differs from prospective obstacle fixture')
    return protocol,scope


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path)
    p.add_argument('--protocol',type=Path,default=None)
    p.add_argument('--role',choices=('mover','sensor_observer'),default='mover');p.add_argument('--execute',action='store_true')
    args,ros=p.parse_known_args()
    args.protocol=args.protocol or (args.run/'dynamic_protocol.json'if args.run else Path(__file__).with_name('protocol.json'))
    if not args.execute:
        print(json.dumps({'status':'prepared_unverified','starts_ros':False,'writes_commands':False,
            'role':args.role,'required_new_scope':'teacher_dynamic_scope/v1','protocol_sha256':sha(args.protocol)}));return
    if args.run is None:p.error('--execute requires --run and a frozen new dynamic scope')
    run=args.run.resolve();protocol,scope=verify_scope(run,args.protocol)
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from sensor_msgs.msg import PointCloud2,Image,CameraInfo
    from sensor_msgs_py import point_cloud2
    import numpy as np
    from PIL import Image as PILImage
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity
    rclpy.init(args=ros)
    class Fixture(Node):
        def __init__(self):
            super().__init__('teacher_dynamic_'+args.role)
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.evidence=EvidenceWriter();self.program=Program(protocol);self.trigger=Trigger(protocol)
            self.actual=Tail(run/'obstacle_actual_pose.jsonl');self.poses=Tail(run/'navigation_slam_poses.jsonl')
            self.telemetry=Tail(run/'telemetry.jsonl');self.nav=Tail(run/'navigation_status.jsonl')
            self.pending=None;self.sent=None;self.sent_sim=None;self.last_send=-math.inf;self.last_ns=None;self.last_rgb={}
            self.directory=run/'dynamic_sensor_evidence';self.directory.mkdir(exist_ok=True)
            self.counts={};self.last_phase=None
            if args.role=='mover':self.client=self.create_client(SetEntityPose,protocol['service'])
            else:
                for name,typ,topic in [('raw_lidar',PointCloud2,'/demo/teacher/raw_lidar'),('cloud',PointCloud2,'/cloud_registered_full'),
                    ('vehicle',Image,'/demo/camera'),('overview',Image,'/demo/teacher/overview'),
                    ('vehicle_info',CameraInfo,'/demo/camera_info'),('overview_info',CameraInfo,'/demo/teacher/overview_info')]:
                    self.create_subscription(typ,topic,lambda m,n=name:self.sensor(n,m),latest_sensor_qos())
            self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def sensor(self,name,msg):
            received=time.monotonic();stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            self.counts[name]=self.counts.get(name,0)+1
            row={'source':name,'stamp_ns':stamp,'received_monotonic_wall':received,'frame_id':msg.header.frame_id,
                'navigation_input':False,'source_scope':'actual ROS sensor payload; passive independent fixture evidence'}
            if name.endswith('info'):
                row.update(width=msg.width,height=msg.height,K=list(msg.k),D=list(msg.d),distortion_model=msg.distortion_model)
            else:
                raw=bytes(msg.data);row.update(width=msg.width,height=msg.height,data_sha256=hashlib.sha256(raw).hexdigest())
                if name in ('raw_lidar','cloud'):
                    row.update(scope='hash of raw PointCloud2 data; saved XYZ only from actual decoded fields',
                        point_step=msg.point_step,row_step=msg.row_step,is_bigendian=msg.is_bigendian,
                        fields=[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count}for f in msg.fields])
                    state_path=run/'dynamic_obstacle_state.json'
                    state=json.loads(state_path.read_text())if state_path.exists()else{}
                    row.update(fixture_phase=state.get('phase'),fixture_state_sim_time=state.get('sim_time'),
                        fixture_state_monotonic_wall=state.get('monotonic_wall'))
                    if state.get('phase')in ('entering','blocking','leaving')or (state.get('phase')=='clear'and
                            state.get('clear_start_sim_s')is not None and stamp/1e9-state['clear_start_sim_s']<=5.):
                        xyz=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True).copy()
                        target=self.directory/f'{name}_{stamp}.npz';row.update(xyz_file=target.name,decoded_points=len(xyz),skip_nans=True)
                        self.evidence.enqueue(lambda t=target,x=xyz:np.savez_compressed(t,xyz=x))
                elif stamp-self.last_rgb.get(name,-10**18)>=1_000_000_000:
                    if msg.encoding not in ('rgb8','bgr8'):raise RuntimeError('Unsupported actual RGB encoding')
                    image=np.frombuffer(raw,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
                    if msg.encoding=='bgr8':image=image[:,:,::-1]
                    image=image.copy();target=self.directory/f'{name}_{stamp}.jpg';row['snapshot_file']=target.name
                    self.evidence.enqueue(lambda t=target,x=image:PILImage.fromarray(x).save(t,quality=92))
                    self.last_rgb[name]=stamp
            self.evidence.append(self.directory/'sensor_inputs.jsonl',row)
        def tick(self):
            sim=self.get_clock().now().nanoseconds/1e9;wall=time.monotonic()
            if args.role=='sensor_observer':
                if self.evidence.error:raise RuntimeError(self.evidence.error)
                return
            if self.evidence.error:self.program.failure=self.evidence.error
            actual=self.actual.poll();pose=self.poses.poll();telemetry=self.telemetry.poll();nav=self.nav.poll()
            anchor_path=run/'navigation_anchor.json';anchor=json.loads(anchor_path.read_text())if anchor_path.exists()else None
            trigger=self.trigger.observe(pose,telemetry,nav,anchor,sim,wall)
            if self.program.phase=='waiting'and self.trigger.missed_window:
                self.program.failure='Early actual SLAM trigger window was missed; refusing late obstacle entry'
            target=self.program.step(sim,trigger,actual,wall)
            if self.pending is not None and self.pending.done():
                error=None
                try:success=self.pending.result().success
                except Exception as exc:success=False;error=str(exc)
                self.evidence.append(run/'obstacle_motion_history.jsonl',{'kind':'service_reply','monotonic_wall':wall,'sim_time':sim,
                    'success':success,'error':error,'requested_position':self.sent,'request_sim_time':self.sent_sim,
                    'scope':'command acknowledgement only; not actual model pose'})
                if not success:self.program.failure='Gazebo rejected obstacle pose service'
                self.pending=None
            if self.pending is not None and sim-self.sent_sim>1.:
                self.program.failure='Obstacle service reply exceeded 1sim second'
            status={'schema':1,'phase':self.program.phase,'failure':self.program.failure,'monotonic_wall':wall,'sim_time':sim,
                'target_position':target,'actual_model_pose':actual,'trigger_slam_along_m':self.trigger.along,
                'trigger_first_slam_stamp_s':self.trigger.first,'trigger_last_slam_stamp_s':self.trigger.last,
                'clear_start_sim_s':self.program.clear_start,'trigger_distinct_pose_samples':self.trigger.samples,
                'trigger_window_missed':self.trigger.missed_window,
                'model':'moving_obstacle','navigation_ground_truth_used':False,'navigation_commands_published':False,
                'scope':'physical fixture state, not a navigation success receipt'}
            self.evidence.atomic(run/'dynamic_obstacle_state.json',status)
            if self.last_phase!=self.program.phase:
                self.evidence.append(run/'obstacle_motion_history.jsonl',{'kind':'phase_change',**status});self.last_phase=self.program.phase
            if self.program.phase in ('waiting','failed','clear'):return
            if self.pending is None and sim-self.last_send>=1/protocol['request_rate_sim_hz']-1e-9:
                if not self.client.service_is_ready():self.program.failure='Owned Gazebo obstacle service is not ready';return
                req=SetEntityPose.Request();req.entity.name='moving_obstacle';req.entity.type=Entity.MODEL
                req.pose.position.x,req.pose.position.y,req.pose.position.z=target;req.pose.orientation.w=1.
                self.evidence.append(run/'obstacle_motion_history.jsonl',{'kind':'service_request','monotonic_wall':wall,'sim_time':sim,
                    'service':protocol['service'],'model':'moving_obstacle','requested_position':target})
                self.sent=target.copy();self.sent_sim=sim;self.last_send=sim;self.pending=self.client.call_async(req)
    node=Fixture()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        # Freeze the box at its last physical location. Never sweep it through a
        # parked robot, never send robot commands during fixture cleanup.
        node.evidence.append(run/'obstacle_motion_history.jsonl',{'kind':'fixture_end','role':args.role,'phase':node.program.phase,
            'failure':node.program.failure,'counts':node.counts,'navigation_validation':'unverified'})
        try:
            node.evidence.close()
            if args.role=='sensor_observer':
                evidence_files=sorted(node.directory.glob('*.npz'))+sorted(node.directory.glob('*.jpg'))
                (run/'dynamic_sensor_evidence_manifest.json').write_text(json.dumps({'schema':1,
                    'status':'recorded_unverified','queue_error':node.evidence.error,'counts':node.counts,
                    'source':'Actual ROS XYZ and actual RGB only; not generated geometry/images',
                    'navigation_input':False,'files':{str(f.relative_to(run)):sha(f)for f in evidence_files}},
                    indent=2,ensure_ascii=False)+'\n')
        finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
