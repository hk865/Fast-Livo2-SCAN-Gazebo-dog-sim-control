#!/usr/bin/env python3
"""Freeze the finite out/return regions once from warmed measured SLAM."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import time
import sys
sys.path.append(str(Path(__file__).resolve().parent.parent))
from runtime_io import EvidenceWriter,latest_sensor_qos


class StartupDeadline:
    """Simulation-time startup budget plus finite wall cap for absent clocks."""
    def __init__(self,profile,wall):
        self.wall_start=wall;self.sim_start=None;self.last_sim=None;self.profile=profile;self.failed=None
    def observe(self,sim_ns,wall):
        if self.last_sim is not None and sim_ns<self.last_sim:self.failed='Actual ROS simulation clock moved backward'
        self.last_sim=sim_ns
        if sim_ns>0 and self.sim_start is None:self.sim_start=sim_ns
        if wall-self.wall_start>=self.profile['request_startup_timeout_wall_s']:
            self.failed='Sensor SLAM startup wall safety deadline exceeded'
        if self.sim_start is not None and (sim_ns-self.sim_start)/1e9>=self.profile['request_startup_timeout_sim_s']:
            self.failed='Sensor SLAM startup simulation deadline exceeded'
        return self.failed


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String,Bool
    from pid_scope import verify_scope
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
            self.deadline=StartupDeadline(profile,self.started_wall);self.evidence=EvidenceWriter()
            self.status_pub=self.create_publisher(String,'/demo/teacher/navigation/request_status',10)
            self.pub=self.create_publisher(String,'/demo/navigation/request',10)
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.anchor_pub=self.create_publisher(String,'/demo/teacher/navigation/anchor',latched)
            self.stop_pub=self.create_publisher(Bool,'/demo/navigation/stop',10)
            self.create_subscription(Odometry,'/demo/slam/body_odom',self.odom,latest_sensor_qos())
            self.create_subscription(String,TOPIC,self.sensor,1)
            self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def sensor(self,msg):
            try:data=json.loads(msg.data)
            except (ValueError,TypeError):return
            if (data.get('ground_truth_navigation_used')is not False or data.get('mode')!=profile['controller_kind']
                or not -.05<=time.monotonic()-float(data.get('monotonic_wall',-math.inf))<.3):return
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
            from route import build_request
            self.request=build_request(run.name,self.anchor,profile,arrival)
            atomic(run/'navigation_anchor.json',self.anchor);atomic(run/'navigation_request.json',self.request)
            self.anchor_pub.publish(String(data=json.dumps(self.anchor)))
        def tick(self):
            now=time.monotonic();sim_ns=self.get_clock().now().nanoseconds
            if self.evidence.error:self.failed=True
            if self.request is None and self.deadline.observe(sim_ns,now):self.failed=True
            ready=(self.pose_wall is not None and now-self.pose_wall<.3 and self.health_wall is not None
                and now-self.health_wall<.3 and self.health.get('ready')is True
                and self.pose_stamp is not None and -.05e9<=sim_ns-self.pose_stamp<.3e9
                and -.05<=now-float(self.health.get('monotonic_wall',-math.inf))<.3)
            if not self.failed and self.request is None and ready:self.freeze()
            if not self.failed and self.request is not None and now-self.last_emit>=1.:
                # Immutable idempotent resends handle DDS discovery. Anchor is
                # never re-estimated from later motion or simulator state.
                self.anchor_pub.publish(String(data=json.dumps(self.anchor)))
                self.pub.publish(String(data=json.dumps(self.request)))
                self.last_emit=now
            if self.failed:self.stop_pub.publish(Bool(data=True))
            state='failed'if self.failed else'route_frozen'if self.request else'waiting_actual_sensor_slam'
            status={'state':state,'ready':ready,'monotonic_wall':now,'ros_sim_time_ns':sim_ns,
                'startup_first_sim_ns':self.deadline.sim_start,'startup_failure':self.deadline.failed,
                'startup_wall_elapsed_s':now-self.started_wall,
                'navigation_validation':'unverified','request_id':None if self.request is None else self.request['request_id'],
                'source':'SLAM pose and warmed real sensor chain','ground_truth_navigation_used':False}
            self.status_pub.publish(String(data=json.dumps(status)))
            self.evidence.append(run/'navigation_request_history.jsonl',status)
    node=Request()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.stop_pub.publish(Bool(data=True))
        node.evidence.close()
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
