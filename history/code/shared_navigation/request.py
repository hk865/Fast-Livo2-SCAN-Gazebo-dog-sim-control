#!/usr/bin/env python3
"""Freeze the finite out/return regions once from warmed measured SLAM."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String,Bool
    from scoped_profile import verify_scope
    from sensor_gate import atomic,TOPIC
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();receipt=verify_scope(run/'navigation_scope.json');profile=receipt['profile']
    if (run/'navigation_request.json').exists()or(run/'navigation_anchor.json').exists():
        raise RuntimeError('Choose a new run; a previously anchored route cannot be reused')
    rclpy.init(args=ros)
    class Request(Node):
        def __init__(self):
            super().__init__('teacher_navigation_request')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.pose=None;self.pose_wall=None;self.pose_stamp=None;self.health={};self.health_wall=None
            self.request=None;self.anchor=None;self.started_wall=time.monotonic();self.last_emit=-math.inf;self.failed=False
            self.status_pub=self.create_publisher(String,'/demo/teacher/navigation/request_status',10)
            self.pub=self.create_publisher(String,'/demo/navigation/request',10)
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.anchor_pub=self.create_publisher(String,'/demo/teacher/navigation/anchor',latched)
            self.stop_pub=self.create_publisher(Bool,'/demo/navigation/stop',10)
            self.create_subscription(Odometry,'/demo/slam/body_odom',self.odom,qos_profile_sensor_data)
            self.create_subscription(String,TOPIC,self.sensor,10)
            self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def sensor(self,msg):
            try:data=json.loads(msg.data)
            except (ValueError,TypeError):return
            if data.get('ground_truth_navigation_used')is not False:return
            self.health=data;self.health_wall=time.monotonic()
        def odom(self,msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            now=self.get_clock().now().nanoseconds;p=msg.pose.pose.position;q=msg.pose.pose.orientation
            if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body'
                or self.pose_stamp is not None and stamp<=self.pose_stamp
                or not -.05e9<=now-stamp<.3e9
                or not all(math.isfinite(v)for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                or not .98<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.02):return
            self.pose=msg;self.pose_wall=time.monotonic();self.pose_stamp=stamp
        def freeze(self):
            p=self.pose.pose.pose.position;q=self.pose.pose.pose.orientation
            # Anchor is measured body yaw in camera_init, not simulator world
            # yaw/position, GPS, IMU yaw injection or a truth-based SE3 alignment.
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            origin=[p.x,p.y,p.z];d=profile['distance_m']
            self.anchor={'schema':1,'experiment':profile['experiment'],'run_dir':str(run),
                'origin':origin,'yaw':yaw,'pose_stamp_ns':self.pose_stamp,'frame_id':'camera_init',
                'source':'/demo/slam/body_odom','ground_truth_navigation_used':False,
                'frozen_once':True,'monotonic_wall':time.monotonic()}
            arrival={'type':'disc_prism','radius_m':profile['arrival_radius_m'],
                'height_half_span_m':profile['arrival_height_half_span_m'],'dwell_sim_s':profile['dwell_sim_s'],
                'control_band':{'type':'disc_prism','radius_m':profile['arrival_control_radius_m'],
                               'height_half_span_m':profile['arrival_height_half_span_m']}}
            self.request={'schema_version':2,'request_id':run.name+'_flat_roundtrip','frame_id':'camera_init',
                'goals':[{'goal_id':'out_1m','center':[p.x+d*math.cos(yaw),p.y+d*math.sin(yaw),p.z],
                          'arrival':arrival,'timeout_sim_s':profile['goal_timeout_sim_s']},
                         {'goal_id':'return_origin','center':origin,'arrival':arrival,
                          'timeout_sim_s':profile['goal_timeout_sim_s']}]}
            atomic(run/'navigation_anchor.json',self.anchor);atomic(run/'navigation_request.json',self.request)
            self.anchor_pub.publish(String(data=json.dumps(self.anchor)))
        def tick(self):
            now=time.monotonic()
            ready=(self.pose_wall is not None and now-self.pose_wall<.3 and self.health_wall is not None
                and now-self.health_wall<.3 and self.health.get('ready')is True)
            if self.request is None and ready:self.freeze()
            if self.request is not None and now-self.last_emit>=1.:
                # Immutable idempotent resends handle DDS discovery. Anchor is
                # never re-estimated from later motion or simulator state.
                self.anchor_pub.publish(String(data=json.dumps(self.anchor)))
                self.pub.publish(String(data=json.dumps(self.request)))
                self.last_emit=now
            if self.request is None and now-self.started_wall>100.:
                self.failed=True;self.stop_pub.publish(Bool(data=True))
            state='failed'if self.failed else'route_frozen'if self.request else'waiting_actual_sensor_slam'
            self.status_pub.publish(String(data=json.dumps({'state':state,'ready':ready,
                'navigation_validation':'unverified','request_id':None if self.request is None else self.request['request_id'],
                'source':'SLAM pose and warmed real sensor chain','ground_truth_navigation_used':False})))
    node=Request()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.stop_pub.publish(Bool(data=True))
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
