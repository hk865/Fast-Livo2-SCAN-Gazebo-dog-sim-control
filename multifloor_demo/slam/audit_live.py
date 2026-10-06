#!/usr/bin/env python3
"""Read-only sensor/SLAM audit; independent truth is used only for this report."""
import argparse
import json
import time
from collections import Counter, defaultdict, deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation, Slerp

from geometry import cloud_records


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def pose(msg):
    p,q=msg.pose.pose.position,msg.pose.pose.orientation
    return np.array([p.x,p.y,p.z]), np.array([q.x,q.y,q.z,q.w])


class Audit(Node):
    def __init__(self):
        super().__init__('demo_slam_read_only_audit')
        self.counts=Counter();self.duplicates=Counter();self.backwards=Counter()
        self.stamps=defaultdict(list);self.samples=defaultdict(list)
        self.frames={};self.rgb_sample=None;self.rgb_colors=set();self.rgb_points=0
        for name,topic,typ in [('imu','/livox/imu',Imu),('lidar','/livox/lidar',PointCloud2),
                ('camera','/camera/image_color',Image),('raw','/aft_mapped_to_init',Odometry),
                ('body','/demo/slam/body_odom',Odometry),('lidar_odom','/demo/slam/lidar_odom',Odometry),
                ('full_cloud','/cloud_registered_full',PointCloud2),('colored_cloud','/cloud_registered',PointCloud2),
                ('ground_truth','/demo/ground_truth',Odometry)]:
            self.create_subscription(typ,topic,lambda msg,n=name:self.receive(n,msg),qos_profile_sensor_data)

    def receive(self,name,msg):
        ts=stamp(msg)
        previous=self.stamps[name][-1] if self.stamps[name] else None
        self.counts[name]+=1;self.frames[name]=msg.header.frame_id
        if previous is not None:
            if previous==ts:self.duplicates[name]+=1
            elif ts<previous:self.backwards[name]+=1
        self.stamps[name].append(ts)
        if name in ('body','ground_truth','lidar_odom'):
            p,q=pose(msg);self.samples[name].append((ts,p,q))
        if name=='colored_cloud':
            try:
                xyz,rgb=cloud_records(msg)
                self.rgb_points+=len(xyz)
                self.rgb_colors.update(map(tuple,rgb[::max(1,len(rgb)//1000)].tolist()))
            except ValueError:
                self.counts['invalid_color_format']+=1

    def report(self):
        rates={}
        for name,times in self.stamps.items():
            unique=np.unique(times)
            rates[name]={'messages':len(times),'unique_stamps':len(unique),
                'first_stamp':times[0],'last_stamp':times[-1],
                'unique_hz_sim':None if len(unique)<2 else (len(unique)-1)/(unique[-1]-unique[0]),
                'duplicate_stamps':self.duplicates[name],'backwards_stamps':self.backwards[name]}
        result={'source':'real running ROS topics; read only', 'ground_truth_use':'audit comparison only, never fed to SLAM/navigation',
            'rates':rates,'frames':self.frames,'rgb_observed_samples':self.rgb_points,
            'rgb_unique_sampled_colors':len(self.rgb_colors),'body_pose_samples':len(self.samples['body'])}
        body=self.samples['body'];truth=self.samples['ground_truth']
        if body and truth:
            gt_unique={t:(p,q) for t,p,q in truth};tt=np.array(sorted(gt_unique));
            pp=np.array([gt_unique[t][0] for t in tt]);qq=np.array([gt_unique[t][1] for t in tt])
            matched=[]
            for t,p,q in body:
                i=np.searchsorted(tt,t)
                if i==0 or i>=len(tt) or tt[i]-tt[i-1]>.15:continue
                frac=(t-tt[i-1])/(tt[i]-tt[i-1]);gtp=pp[i-1]*(1-frac)+pp[i]*frac
                gtq=Slerp(tt[i-1:i+1],Rotation.from_quat(qq[i-1:i+1]))([t]).as_quat()[0]
                matched.append((t,p,q,gtp,gtq))
            if matched:
                _,sp,sq,gp,gq=matched[0]
                r=Rotation.from_quat(gq).as_matrix()@Rotation.from_quat(sq).as_matrix().T
                offset=gp-r@sp
                errors=[float(np.linalg.norm(r@p+offset-gp)) for _,p,q,gp,gq in matched]
                result['trajectory_comparison']={'alignment':'single fixed SE(3) alignment from first time-matched pair, evaluation only',
                    'matched_samples':len(matched),'max_error_m':max(errors),'rmse_m':float(np.sqrt(np.mean(np.square(errors)))),
                    'final_error_m':errors[-1], 'slam_displacement_m':float(np.linalg.norm(matched[-1][1]-sp)),
                    'truth_displacement_m':float(np.linalg.norm(matched[-1][3]-gp)),
                    'first_stamp':matched[0][0],'last_stamp':matched[-1][0]}
        required=['imu','lidar','camera','raw','body','lidar_odom','full_cloud','colored_cloud']
        result['all_sensor_slam_topics_received']=all(self.counts[n]>0 for n in required)
        result['unique_forward_sensor_time']=all(not self.duplicates[n] and not self.backwards[n] for n in ['imu','lidar','camera'])
        return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=20);parser.add_argument('--output',required=True)
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args);node=Audit()
    deadline=time.monotonic()+args.duration
    try:
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    result=node.report();Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
