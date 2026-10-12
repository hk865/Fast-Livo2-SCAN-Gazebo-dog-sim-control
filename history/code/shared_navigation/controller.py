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
from control_core import rotation_xyzw
from std_msgs.msg import String


class RecordedPublisher:
    def __init__(self,publisher,run,node):self.publisher=publisher;self.run=run;self.node=node
    def publish(self,msg):
        data=json.loads(msg.data)
        data.update(mode='teacher',scope='flat_relative_roundtrip_v1',navigation_ground_truth_used=False,
            policy_observations='Gazebo privileged Teacher inputs; navigation localization is sensor SLAM',
            navigation_validation='unverified',acceptance_is_global=False)
        self.publisher.publish(String(data=json.dumps(data,ensure_ascii=False,allow_nan=False)))
        with (self.run/'navigation_status.jsonl').open('a')as stream:
            stream.write(json.dumps({**data,'monotonic_wall':time.monotonic(),
                'ros_sim_time':self.node.get_clock().now().nanoseconds/1e9},allow_nan=False,ensure_ascii=False)+'\n')
        tmp=self.run/'navigation_status.tmp';tmp.write_text(json.dumps(data,allow_nan=False,ensure_ascii=False));tmp.replace(self.run/'navigation_status.json')


class TeacherNavigation(base.Navigation):
    def __init__(self,run):
        self.run=run;receipt=verify_scope(run/'navigation_scope.json');self.profile=receipt['profile']
        super().__init__()
        self.status_pub=RecordedPublisher(self.status_pub,run,self)
        self.bridge_updated=-math.inf
    def declare_parameter(self,name,value=None,*args,**kwargs):
        if name=='scenario_path':value=str(self.run/'navigation_scenario.json')
        if name=='max_speed':value=self.profile['max_speed_mps']
        if name in ('pose_timeout','cloud_timeout'):value=self.profile['pose_cloud_timeout_s']
        return super().declare_parameter(name,value,*args,**kwargs)
    def on_bridge_safety(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        if data.get('mode')!='teacher' or data.get('source')!='scan_slam':return
        self.bridge_updated=time.monotonic();super().on_bridge_safety(msg)
    def apply_tilt_guard(self,now,ros_now):
        protected=super().apply_tilt_guard(now,ros_now)
        if now-self.bridge_updated>.30 or self.bridge_safety.get('state')!='ready':
            self.message='Teacher 执行命令门或 SLAM 传感器预热未就绪，停车'
            return True
        return protected
    def on_odom(self,msg):
        stamp=self.stamp(msg);ns=self.get_clock().now().nanoseconds
        q=msg.pose.pose.orientation
        if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body' or stamp<=self.pose_stamp
            or not -.05e9<=ns-stamp<self.pose_timeout*1e9
            or not .98<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.02):return
        super().on_odom(msg)
        p=msg.pose.pose.position;v=msg.twist.twist.linear;w=msg.twist.twist.angular
        row={'stamp_ns':stamp,'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
            'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w],
            'body_velocity':[v.x,v.y,v.z],'body_angular_velocity':[w.x,w.y,w.z]}
        with(self.run/'navigation_slam_poses.jsonl').open('a')as stream:stream.write(json.dumps(row,allow_nan=False)+'\n')
    def on_cloud(self,msg):
        stamp=self.stamp(msg);ns=self.get_clock().now().nanoseconds
        if stamp<=self.cloud_stamp or not -.05e9<=ns-stamp<self.cloud_timeout*1e9:return
        super().on_cloud(msg)
        if self.cloud_stamp==stamp and self.cloud_input_context is not None:
            c=self.cloud_input_context
            row={'stamp_ns':stamp,'frame_id':msg.header.frame_id,'input_points':int(msg.width*msg.height),
                'filtered_points':c['filtered_points'],'self_filtered_points':c['self_filtered_points'],
                'nearest_slam_pose_stamp_ns':c['filtering_body_stamp_ns'],
                'filtered_xyz_float64_sha256':hashlib.sha256(np.asarray(self.cloud,dtype='<f8').tobytes()).hexdigest()}
            with(self.run/'navigation_cloud_history.jsonl').open('a')as stream:stream.write(json.dumps(row)+'\n')
    def accept_spline(self,msg,metadata):
        before=self.replans;super().accept_spline(msg,metadata)
        if self.replans==before or self.samples is None:return
        directory=self.run/'navigation_trajectories';directory.mkdir(exist_ok=True)
        array_path=directory/f'{self.replans:06d}_trajectory_{self.active_trajectory_id}.npz'
        np.savez_compressed(array_path,coefficients=np.array([[p.x,p.y,p.z]for p in msg.pos_pts]),
            knots=np.array(msg.knots),samples=self.samples)
        document={'trajectory_id':self.active_trajectory_id,'order':int(msg.order),'metadata':metadata,
            'reference_stamp':self.trajectory_association.reference_stamp,'request_id':self.request_id,
            'waypoint_index':self.waypoint_index,'accepted_slam_stamp_ns':self.pose_stamp,
            'array_sha256':hashlib.sha256(array_path.read_bytes()).hexdigest(),'array_file':array_path.name,
            'source':'actual SCAN committed B-spline and associated metadata; sensor SLAM feedback'}
        array_path.with_suffix('.json').write_text(json.dumps(document,allow_nan=False,ensure_ascii=False)+'\n')
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
        if self.obstacle_hold:
            stopped=False
            if len(self.pose_history)>2:
                last=self.pose_history[-1]
                candidates=[r for r in self.pose_history if .4e9<=last[0]-r[0]<=.65e9]
                if candidates:
                    first=candidates[-1]
                    stopped=(np.linalg.norm(last[1]-first[1])/((last[0]-first[0])/1e9)<.03)
            if not stopped:self.obstacle_clear_since=None
        super().control(now)


def main():
    import rclpy
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();os.environ['DEMO_RUN_DIR']=str(run)
    rclpy.init(args=ros);node=TeacherNavigation(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publish_command()
        node.event_archive.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
