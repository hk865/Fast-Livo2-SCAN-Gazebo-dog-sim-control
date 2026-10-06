#!/usr/bin/env python3
"""Physical IMU-heading turn A/B fixture. Truth is recorded, never controlled."""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2,Imu,Image
from std_msgs.msg import String
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT.parent/'scripts'))
from self_echo_filter import filter_records
from evaluate_run import compare_trajectory,read_poses


def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))


class Probe(Node):
    def __init__(self,run):
        super().__init__('demo_physical_slam_turn_probe')
        self.run=run;self.clock=0.;self.phase='waiting_sensors';self.counts=Counter()
        self.pose_file=(run/'pose_audit.jsonl').open('w');self.sensor_file=(run/'sensor_audit.jsonl').open('w')
        self.last_imu=None;self.last_body=None;self.body_since=None;self.initial_heading=None
        self.cmd=self.create_publisher(Twist,'/demo/cmd_vel',10)
        self.create_subscription(Clock,'/clock',lambda m:setattr(self,'clock',m.clock.sec+m.clock.nanosec*1e-9),10)
        self.lidar=json.loads((ROOT.parent/'simulation/scenario.json').read_text())['sensors']['lidar']
        for name,topic,typ in [('imu','/livox/imu',Imu),('lidar','/livox/lidar',PointCloud2),
            ('camera','/camera/image_color',Image),('body','/demo/slam/body_odom',Odometry),
            ('truth','/demo/ground_truth',Odometry),('filtered','/demo/slam/lidar_filtered',PointCloud2),
            ('full','/cloud_registered_full',PointCloud2),('rgb','/cloud_registered',PointCloud2)]:
            self.create_subscription(typ,topic,lambda m,n=name:self.receive(n,m),qos_profile_sensor_data)

    def receive(self,name,msg):
        ts=stamp(msg);self.counts[name]+=1;record={'source':name,'stamp':ts,'received_sim_clock':self.clock}
        if name=='imu':
            self.last_imu=msg;record.update({'q':[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w],
                'omega':[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                'acc':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]})
        elif name in ('body','truth'):
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            self.pose_file.write(json.dumps({'source':'slam' if name=='body' else 'truth','stamp':ts,
                'p':[p.x,p.y,p.z],'q':[q.x,q.y,q.z,q.w],'stage':self.phase})+'\n')
            if name=='body':
                self.last_body=msg
                if self.body_since is None:self.body_since=self.clock
        elif name=='lidar':
            _,removed,total=filter_records(msg,self.lidar)
            record.update({'points':total,'self_points':removed})
            if self.counts[name] in (1,100,200,300,400,500):
                (self.run/f'raw_lidar.{self.counts[name]:04}.bin').write_bytes(bytes(msg.data))
                (self.run/f'raw_lidar.{self.counts[name]:04}.json').write_text(json.dumps({
                    'stamp':ts,'frame_id':msg.header.frame_id,'width':msg.width,'height':msg.height,
                    'point_step':msg.point_step,'row_step':msg.row_step,'is_bigendian':msg.is_bigendian,
                    'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in msg.fields]}))
        elif name=='camera':record['latest_imu_stamp_at_camera_receive']=None if self.last_imu is None else stamp(self.last_imu)
        elif name in ('filtered','full','rgb'):record['points']=msg.width*msg.height
        self.sensor_file.write(json.dumps(record)+'\n')

    def command(self,vx=0.,omega=0.):
        msg=Twist();msg.linear.x=vx;msg.angular.z=omega;self.cmd.publish(msg)

    def flush(self):self.pose_file.flush();self.sensor_file.flush()
    def close(self):self.flush();self.pose_file.close();self.sensor_file.close()


def run(args):
    directory=Path(args.output_dir).resolve()
    if directory.exists():raise ValueError('turn probe must use a new directory')
    directory.mkdir(parents=True)
    config=yaml.safe_load((ROOT/'fastlivo.yaml').read_text());params=config['/**']['ros__parameters']
    params['common']['require_complete_imu']=args.strict_imu
    params['debug']={'sensor_sync_diagnostics':True}
    params['common']['lid_topic']='/demo/slam/lidar_filtered' if args.self_filter else '/livox/lidar'
    (directory/'fastlivo_probe.yaml').write_text(yaml.safe_dump(config))
    env=os.environ.copy();env.update({'DEMO_TURN_RUN':str(directory),'DEMO_TURN_FILTER':'1' if args.self_filter else '0',
        'GZ_PARTITION':'go2_slam_turn_audit','GZ_IP':'127.0.0.1','QT_QPA_PLATFORM':'offscreen',
        'GZ_SIM_SYSTEM_PLUGIN_PATH':'/opt/ros/jazzy/lib:'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH',''),
        'ROS_LOG_DIR':str(directory/'ros_logs')})
    log=(directory/'stack.log').open('w')
    launch=subprocess.Popen(['ros2','launch',str(ROOT/'tests/turn_probe.launch.py')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();probe=Probe(directory);wall_deadline=time.monotonic()+240
    phases=[('turn1',math.pi/2,None),('drive1',math.pi/2,12.),('settle1',None,3.),
            ('turn2',math.pi,None),('drive2',math.pi,12.),('settle2',None,4.)]
    index=-1;phase_started=None;aligned_since=None;success=False
    try:
        while time.monotonic()<wall_deadline and launch.poll() is None:
            rclpy.spin_once(probe,timeout_sec=.025)
            if index<0:
                probe.command()
                if probe.body_since is not None and probe.clock-probe.body_since>=5 and probe.last_imu:
                    probe.initial_heading=yaw(probe.last_imu.orientation);index=0;phase_started=probe.clock;probe.phase='exploring'
                continue
            name,heading,duration=phases[index]
            if name.startswith('turn'):
                error=wrap(probe.initial_heading+heading-yaw(probe.last_imu.orientation))
                if abs(error)<.05:
                    probe.command();aligned_since=aligned_since or probe.clock
                    if probe.clock-aligned_since>.7:index+=1;phase_started=probe.clock;aligned_since=None
                else:probe.command(omega=max(-.12,min(.12,.6*error)));aligned_since=None
                if probe.clock-phase_started>40:break
            elif name.startswith('drive'):
                error=wrap(probe.initial_heading+heading-yaw(probe.last_imu.orientation))
                probe.command(.1,max(-.04,min(.04,.5*error)))
                if probe.clock-phase_started>=duration:index+=1;phase_started=probe.clock
            else:
                probe.command()
                if probe.clock-phase_started>=duration:index+=1;phase_started=probe.clock
            if index>=len(phases):success=True;break
            probe.flush()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):probe.command();rclpy.spin_once(probe,timeout_sec=.02)
        probe.close();probe.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if launch.poll() is None:
            try:os.killpg(launch.pid,signal.SIGINT)
            except ProcessLookupError:pass
        try:launch.wait(timeout=35)
        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
        log.close()
    for name in ('mat_pre.txt','mat_out.txt','imu.txt'):
        source=ROOT/'ros2_ws/src/fast_livo2_core/Log'/name
        if source.exists():shutil.copyfile(source,directory/('fastlivo_'+name))
    samples,errors=read_poses(directory/'pose_audit.jsonl');trajectory,*_=compare_trajectory(samples)
    report={'test_kind':'actual Gazebo physical IMU-heading turns and translation; truth recorded only',
        'self_filter':args.self_filter,'strict_imu':args.strict_imu,'sequence_completed':success,
        'counts':dict(probe.counts),'trajectory':trajectory,'pose_parse_errors':errors}
    (directory/'probe_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if success else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',required=True)
    parser.add_argument('--self-filter',action='store_true');parser.add_argument('--strict-imu',action='store_true')
    raise SystemExit(run(parser.parse_args()))
