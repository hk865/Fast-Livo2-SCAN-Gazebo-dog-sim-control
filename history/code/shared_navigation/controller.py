#!/usr/bin/env python3
"""Teacher-specific finite wrapper around existing measured-pose Go2 SCAN.

The shared controller consumes SLAM, actual registered clouds and checked SCAN
trajectories. This wrapper narrows scope, fixes duplicate-stamp acceptance and
requires measured stopping before obstacle recovery. It owns no joint actuator.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parent;DEMO=ROOT.parent
sys.path.insert(0,str(DEMO/'navigation'))
spec=importlib.util.spec_from_file_location('teacher_measured_scan_base',DEMO/'navigation/controller.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
from scoped_profile import verify_scope
from control_core import rotation_xyzw,body_tilt
from runtime_io import EvidenceWriter,latest_sensor_qos
from teacher_transition import TeacherHeadingGate
from std_msgs.msg import String


class RecordedPublisher:
    def __init__(self,publisher,run,node):self.publisher=publisher;self.run=run;self.node=node
    def publish(self,msg):
        data=json.loads(msg.data)
        data.update(mode='teacher',scope=self.node.profile['experiment'],navigation_ground_truth_used=False,
            policy_observations='Gazebo privileged Teacher inputs; navigation localization is sensor SLAM',
            navigation_validation='unverified',acceptance_is_global=False)
        data['teacher_transition']=self.node.heading_gate.evidence(self.node.get_clock().now().nanoseconds/1e9,time.monotonic())
        self.publisher.publish(String(data=json.dumps(data,ensure_ascii=False,allow_nan=False)))
        self.node.evidence.append(self.run/'navigation_status.jsonl',{**data,'monotonic_wall':time.monotonic(),
            'ros_sim_time':self.node.get_clock().now().nanoseconds/1e9})
        self.node.evidence.atomic(self.run/'navigation_status.json',data)


class TeacherNavigation(base.Navigation):
    def __init__(self,run):
        self.run=run;receipt=verify_scope(run/'navigation_scope.json');self.profile=receipt['profile']
        super().__init__()
        self.evidence=EvidenceWriter();self.gyro_body=None;self.gyro_stamp=None;self.gyro_wall=None
        self.heading_gate=TeacherHeadingGate(self.profile)
        self.status_pub=RecordedPublisher(self.status_pub,run,self)
        self.bridge_updated=-math.inf
    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        if topic in ('/demo/slam/body_odom','/cloud_registered_full','/livox/imu'):
            qos_profile=latest_sensor_qos()
        if topic=='/demo/control/safety':qos_profile=1
        return super().create_subscription(msg_type,topic,callback,qos_profile,*args,**kwargs)
    def declare_parameter(self,name,value=None,*args,**kwargs):
        if name=='scenario_path':value=str(self.run/'navigation_scenario.json')
        if name=='max_speed':value=self.profile['max_speed_mps']
        if name in ('pose_timeout','cloud_timeout'):value=self.profile['pose_cloud_timeout_s']
        return super().declare_parameter(name,value,*args,**kwargs)
    def on_bridge_safety(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        if (data.get('mode')!='teacher' or data.get('source')!='scan_slam'
            or data.get('state')not in ('hold','ready','failed')):return
        try:
            if not -.05<=time.monotonic()-float(data['monotonic_wall'])<.3:return
        except (ValueError,TypeError,KeyError):return
        self.bridge_updated=time.monotonic();self.bridge_safety=data
        if self.state=='running'and self.apply_tilt_guard(self.bridge_updated,self.get_clock().now().nanoseconds/1e9):
            self.reset_region_arrival('protected')
            # Do not feed bridge hold -> repeated zero -> bridge status back
            # into another zero. A moving command still stops immediately.
            if any(self.command):self.publish_command()
            if self.state=='failed':self.publish_status()
    def on_imu(self,msg):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9;q=msg.orientation
        accepted=self.raw_imu.update([q.x,q.y,q.z,q.w],self.stamp(msg)/1e9,now,ros_now,
                                     msg.orientation_covariance[0]>=0)
        if accepted:
            self.counts['imu']+=1;v=msg.angular_velocity
            gyro=self.imu_body_rotation@np.array([v.x,v.y,v.z])
            if np.isfinite(gyro).all():
                self.gyro_body=gyro.tolist();self.gyro_stamp=self.stamp(msg)/1e9;self.gyro_wall=now
        if self.state=='running'and self.apply_tilt_guard(now,ros_now):
            self.reset_region_arrival('protected')
            # Protection still stops a moving robot in this very callback.
            # Already-zero held commands use the unchanged 20Hz timer heartbeat.
            if any(self.command):self.publish_command()
            if self.state=='failed':self.publish_status()
    def apply_tilt_guard(self,now,ros_now):
        # A communication hold is not measured tilt and must not reset a
        # parked Teacher's heading/path every time a fresh cloud is delayed.
        # Physical IMU/SLAM tilt protections retain the original thresholds.
        slam_tilt=body_tilt(self.rotation);fresh=self.raw_imu.fresh(now,ros_now)
        sources=[('SLAM',slam_tilt)]+([('raw_imu',self.raw_imu.tilt)]if fresh else [])
        self.max_tilt=max(self.max_tilt,*(tilt for _,tilt in sources))
        severe=[name for name,tilt in sources if tilt>=.50]
        if self.bridge_safety.get('state')=='failed':severe.append('execution_bridge')
        if self.evidence.error:severe.append('evidence_writer')
        if severe:
            self.state='failed';self.tilt_source='+'.join(severe);self.message=self.tilt_source+'保护已锁定，停车'
            return True
        high=[name for name,tilt in sources if tilt>=.30]
        if high:
            if not self.tilt_hold:self.tilt_stops+=1
            self.tilt_hold=True;self.tilt_source='+'.join(high);self.tilt_clear_since=None
        if not fresh:
            self.message='原始IMU超时，停车';self.tilt_clear_since=None;return True
        if self.tilt_hold:
            if all(tilt<.18 for _,tilt in sources)and now-self.pose_updated<self.pose_timeout:
                if self.tilt_clear_since is None:self.tilt_clear_since=ros_now
                if ros_now-self.tilt_clear_since>=.8:
                    self.tilt_hold=False;self.tilt_source=None;self.heading_gate.reset()
                    self.samples=None;self.last_reference=-math.inf
            else:self.tilt_clear_since=None
            self.message='实际倾角保护，停车等待稳定';return True
        if now-self.bridge_updated>.30 or self.bridge_safety.get('state')!='ready':
            self.message='Teacher 执行命令门或 SLAM 传感器预热未就绪，停车'
            return True
        self.tilt_source=None;return False
    def on_odom(self,msg):
        stamp=self.stamp(msg);ns=self.get_clock().now().nanoseconds
        received=time.monotonic();q=msg.pose.pose.orientation
        v=msg.twist.twist.linear;w=msg.twist.twist.angular;p=msg.pose.pose.position
        if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body' or stamp<=self.pose_stamp
            or not -.05e9<=ns-stamp<self.pose_timeout*1e9
            or not .98<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.02
            or not all(math.isfinite(x)for x in (p.x,p.y,p.z,v.x,v.y,v.z,w.x,w.y,w.z))):return
        super().on_odom(msg)
        if self.gyro_body is not None and received-self.gyro_wall<.3:
            self.heading_gate.observe(stamp/1e9,received,[v.x,v.y,v.z],w.z,self.gyro_body,self.gyro_stamp,not any(self.command))
        row={'stamp_ns':stamp,'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
            'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w],
            'body_velocity':[v.x,v.y,v.z],'body_angular_velocity':[w.x,w.y,w.z],
            'received_monotonic_wall':received,'callback_ros_clock_ns':ns,'sim_age_at_callback_s':(ns-stamp)/1e9,
            'body_gyro':self.gyro_body,'gyro_stamp_s':self.gyro_stamp,'zero_velocity_command':not any(self.command),
            'measured_stop_evidence':self.heading_gate.evidence(ns/1e9,received)}
        self.evidence.append(self.run/'navigation_slam_poses.jsonl',row)
    def on_cloud(self,msg):
        stamp=self.stamp(msg);ns=self.get_clock().now().nanoseconds
        if stamp<=self.cloud_stamp or not -.05e9<=ns-stamp<self.cloud_timeout*1e9:return
        super().on_cloud(msg)
        if self.cloud_stamp==stamp and self.cloud_input_context is not None:
            c=self.cloud_input_context
            row={'stamp_ns':stamp,'frame_id':msg.header.frame_id,'input_points':int(msg.width*msg.height),
                'filtered_points':c['filtered_points'],'self_filtered_points':c['self_filtered_points'],
                'nearest_slam_pose_stamp_ns':c['filtering_body_stamp_ns'],
                'received_monotonic_wall':time.monotonic(),'callback_ros_clock_ns':ns,'sim_age_at_callback_s':(ns-stamp)/1e9,
                'filtered_xyz_float64_sha256':hashlib.sha256(np.asarray(self.cloud,dtype='<f8').tobytes()).hexdigest()}
            self.evidence.append(self.run/'navigation_cloud_history.jsonl',row)
    def accept_spline(self,msg,metadata):
        before=self.replans;super().accept_spline(msg,metadata)
        if self.replans==before or self.samples is None:return
        directory=self.run/'navigation_trajectories'
        array_path=directory/f'{self.replans:06d}_trajectory_{self.active_trajectory_id}.npz'
        coefficients=np.array([[p.x,p.y,p.z]for p in msg.pos_pts]);knots=np.array(msg.knots);samples=self.samples.copy()
        document={'trajectory_id':self.active_trajectory_id,'order':int(msg.order),'metadata':metadata,
            'reference_stamp':self.trajectory_association.reference_stamp,'request_id':self.request_id,
            'waypoint_index':self.waypoint_index,'accepted_slam_stamp_ns':self.pose_stamp,
            'array_file':array_path.name,
            'source':'actual SCAN committed B-spline and associated metadata; sensor SLAM feedback'}
        def archive():
            directory.mkdir(exist_ok=True);np.savez_compressed(array_path,coefficients=coefficients,knots=knots,samples=samples)
            document['array_sha256']=hashlib.sha256(array_path.read_bytes()).hexdigest()
            array_path.with_suffix('.json').write_text(json.dumps(document,allow_nan=False,ensure_ascii=False)+'\n')
        self.evidence.enqueue(archive)
    def on_request(self,msg):
        try:
            requested=json.loads(msg.data);frozen=json.loads((self.run/'navigation_request.json').read_text())
            if requested!=frozen:raise ValueError('Only this run frozen SLAM-relative route is eligible')
        except (OSError,ValueError,TypeError)as error:
            self.last_rejected=str(error);self.publish_status();return
        super().on_request(msg)
    def control(self,now):
        # Shared recovery already requires one clear second and a new checked
        # trajectory; additionally hold it until measured body motion is small.
        if self.evidence.error:
            self.state='failed';self.message=self.evidence.error;self.publish_command();return
        if self.obstacle_hold and not self.heading_gate.stopped(self.get_clock().now().nanoseconds/1e9,now):
            self.obstacle_clear_since=None
        super().control(now)
        if self.heading_gate.phase in ('pre_turn','settle'):
            self.message='Teacher 等待实际SLAM机身速度和IMU角速确认停车，保持零速度'


def main():
    import rclpy
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();os.environ['DEMO_RUN_DIR']=str(run)
    rclpy.init(args=ros);node=TeacherNavigation(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publish_command()
        node.event_archive.close();node.evidence.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
