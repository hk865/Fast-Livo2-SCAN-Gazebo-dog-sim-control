#!/usr/bin/env python3
"""Warm up actual sensor SLAM before permitting a bounded route request."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import time

TOPIC='/demo/teacher/navigation/sensor_health'


def atomic(path,data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,allow_nan=False,ensure_ascii=False)+'\n');tmp.replace(path)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo,Imu,PointCloud2
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock as ClockMessage
    from std_msgs.msg import String
    import numpy as np
    import yaml
    from sensor_msgs_py import point_cloud2
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args,ros=parser.parse_known_args();run=args.run.resolve()
    profile=json.loads((run/'navigation_profile.json').read_text())
    actual_camera_rate=json.loads((run/'sensor_contract.json').read_text())['camera']['rate_hz']
    calib=yaml.safe_load((run/'navigation_camera.yaml').read_text())['/**']['ros__parameters']
    rclpy.init(args=ros)
    class Gate(Node):
        def __init__(self):
            super().__init__('teacher_navigation_sensor_gate')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.clock_ns=None;self.clock_wall=None;self.stamps={};self.walls={};self.counts={};self.errors={}
            self.info=None;self.image=None;self.good_since=None;self.failure=None;self.last_report=None
            self.pub=self.create_publisher(String,TOPIC,10)
            for name,typ,topic in [('imu',Imu,'/livox/imu'),('raw_lidar',PointCloud2,'/demo/teacher/raw_lidar'),
                ('image',Image,'/demo/camera'),('camera_info',CameraInfo,'/demo/camera_info'),
                ('body_odom',Odometry,'/demo/slam/body_odom'),('cloud',PointCloud2,'/cloud_registered_full')]:
                self.create_subscription(typ,topic,lambda m,n=name:self.receive(n,m),qos_profile_sensor_data)
            self.create_subscription(ClockMessage,'/clock',self.clock,qos_profile_sensor_data)
            self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def clock(self,msg):
            ns=msg.clock.sec*1_000_000_000+msg.clock.nanosec
            if self.clock_ns is not None and ns<self.clock_ns:self.failure='Simulation clock moved backward'
            if self.clock_ns is None or ns>self.clock_ns:self.clock_ns=ns;self.clock_wall=time.monotonic()
        def receive(self,name,msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if stamp<=self.stamps.get(name,-1):return
            error=None
            if name=='body_odom':
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body'
                    or not all(math.isfinite(v)for v in values) or not .98<=sum(v*v for v in values[3:])<=1.02):
                    error='Invalid measured SLAM body pose/frame'
            elif name=='cloud':
                if msg.header.frame_id!='camera_init' or msg.width*msg.height<20:error='Registered cloud is empty or in wrong frame'
                else:
                    try:
                        xyz=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)
                        if len(xyz)<20 or not np.isfinite(xyz).all():error='Registered cloud XYZ invalid'
                    except (ValueError,TypeError):error='Registered cloud cannot be decoded'
            elif name=='image':
                if msg.header.frame_id!='demo_camera_optical_frame' or msg.width!=640 or msg.height!=480 or not msg.data:
                    error='Vehicle image frame/dimensions invalid'
                else:self.image=(stamp,msg.header.frame_id,msg.width,msg.height)
            elif name=='camera_info':
                expected=[calib['cam_fx'],0.,calib['cam_cx'],0.,calib['cam_fy'],calib['cam_cy'],0.,0.,1.]
                if (msg.header.frame_id!='demo_camera_optical_frame' or msg.width!=640 or msg.height!=480
                    or msg.distortion_model!='plumb_bob' or len(msg.d)<4 or any(abs(v)>1e-12 for v in msg.d)
                    # Actual Gazebo float32 K quantization is 1.065e-5 px;
                    # this 2e-5 px tolerance does not change geometry bounds.
                    or not np.allclose(msg.k,expected,rtol=0,atol=2e-5)):
                    error='Actual vehicle CameraInfo does not match pinhole K/zero D'
                else:self.info=(stamp,msg.header.frame_id,msg.width,msg.height)
            elif name=='raw_lidar':
                if msg.header.frame_id!='velodyne' or msg.width*msg.height<20:error='Raw actual LiDAR invalid'
            elif name=='imu':
                values=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
                if not all(math.isfinite(v)for v in values):error='Actual IMU nonfinite'
            if error:self.errors[name]=error;return
            self.errors.pop(name,None);self.stamps[name]=stamp;self.walls[name]=time.monotonic()
            self.counts[name]=self.counts.get(name,0)+1
        def graph(self):
            expected={'/aft_mapped_to_init':'laserMapping','/demo/slam/body_odom':'demo_slam_odom_adapter',
                '/cloud_registered_full':'laserMapping','/demo/slam/lidar_filtered':'demo_slam_self_echo_filter',
                '/demo/teacher/slam/image':'teacher_slam_actual_sensor_relay',
                '/demo/teacher/slam/imu':'teacher_slam_actual_sensor_relay'}
            results={}
            for topic,name in expected.items():
                infos=self.get_publishers_info_by_topic(topic)
                results[topic]={'expected':name,'publishers':[i.node_namespace+'/'+i.node_name for i in infos],
                    'passed':len(infos)==1 and infos[0].node_name==name and infos[0].node_namespace=='/'}
            return results
        def tick(self):
            now=time.monotonic();reasons=[];ages={};ns=self.clock_ns
            if self.failure:reasons.append(self.failure)
            if self.clock_wall is None or now-self.clock_wall>=.3:reasons.append('Actual simulation clock stale')
            for name in ('imu','raw_lidar','image','camera_info','body_odom','cloud'):
                wall_age=None if name not in self.walls else now-self.walls[name]
                sim_age=None if ns is None or name not in self.stamps else (ns-self.stamps[name])/1e9
                ages[name]={'wall_s':wall_age,'sim_s':sim_age,'stamp_ns':self.stamps.get(name)}
                # 2 Hz RGB cannot use the 300ms motion feedback watchdog.
                wall_limit,sim_limit=((2.,.75)if actual_camera_rate<10 else(.3,.3))if name in ('image','camera_info')else(.3,.3)
                if wall_age is None or sim_age is None or not 0<=wall_age<wall_limit or not -.05<=sim_age<sim_limit:
                    reasons.append(name+' missing/stale')
                if name in self.errors:reasons.append(self.errors[name])
            if self.info is None or self.image is None or self.info[1:]!=self.image[1:] or abs(self.info[0]-self.image[0])>50_000_000:
                reasons.append('Vehicle image and actual CameraInfo are not paired within 50ms')
            graph=self.graph()
            for topic,value in graph.items():
                if not value['passed']:reasons.append('Unexpected actual SLAM publisher chain: '+topic)
            if reasons:self.good_since=None
            elif self.good_since is None:self.good_since=ns
            warmed=not reasons and ns is not None and self.good_since is not None and (ns-self.good_since)/1e9>=profile['sensor_warmup_sim_s']
            if not warmed and not reasons:reasons.append('Waiting for continuous actual sensor SLAM warmup')
            status={'schema':1,'mode':'teacher','state':'failed'if self.failure else'ready'if warmed else'hold',
                'ready':warmed,'reason':'; '.join(reasons)if reasons else'Actual vehicle RGB/K, IMU, LiDAR and sensor SLAM healthy',
                'monotonic_wall':now,'ros_sim_time_ns':ns,'good_since_sim_ns':self.good_since,'ages':ages,
                'counts':self.counts,'publisher_graph':graph,'ground_truth_navigation_used':False,
                'overview_used_for_vio':False,'camera_intrinsics_scale':'Original640x480 K; FAST-LIVO2 internally applies scale=0.5'}
            atomic(run/'navigation_sensor_gate.json',status)
            with(run/'navigation_sensor_gate_history.jsonl').open('a')as stream:
                stream.write(json.dumps(status,allow_nan=False,ensure_ascii=False)+'\n')
            self.pub.publish(String(data=json.dumps(status,allow_nan=False)));self.last_report=status
    node=Gate()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if node.last_report:
            status={**node.last_report,'ready':False,'state':'hold','reason':'Sensor SLAM gate stopped','monotonic_wall':time.monotonic()}
            atomic(run/'navigation_sensor_gate.json',status)
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
