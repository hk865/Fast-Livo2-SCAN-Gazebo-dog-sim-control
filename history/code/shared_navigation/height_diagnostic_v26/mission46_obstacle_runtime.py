#!/usr/bin/env python3
"""Original dynamic crossing in the owned Teacher Gazebo world only.

Only moving_obstacle SetEntityPose is called. Actual SLAM SceneTrigger owns
activation; robot pose/ground truth is never subscribed or modified. Service
responses, not requested targets, determine reported visible position/phase.
Terminal missions keep spinning and freeze in place until owned cleanup.
"""
from __future__ import annotations
import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.append(str(HERE.parent))
from mission46_profile import validate_profile
from simulation.obstacle_trigger import SceneTrigger
from runtime_io import EvidenceWriter,latest_sensor_qos


def target_at(elapsed_s):
    if elapsed_s<0:raise ValueError('Obstacle simulation time moved backward')
    if elapsed_s<8:return [1.,4.-.25*elapsed_s,.6],'entering'
    if elapsed_s<28:return [1.,2.,.6],'blocking'
    if elapsed_s<36:return [1.,2.-.25*(elapsed_s-28),.6],'leaving'
    return [1.,0.,.6],'clear'


def build_node(run):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String,Bool
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity

    class Obstacle(Node):
        def __init__(self):
            super().__init__('teacher_mission46_obstacle')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.writer=EvidenceWriter();self.enabled=False;self.started=None;self.trigger=SceneTrigger()
            self.state={};self.state_wall=-math.inf;self.pose=None;self.pose_wall=-math.inf
            self.pending=None;self.sent=None;self.successes=0;self.failures=0;self.sequence=0
            self.position=[1.,4.,.6];self.visible_phase='parked';self.request_id=None;self.armed_once=False
            self.failure=None;self.last_sim=None;self.last_send_sim=None
            self.client=self.create_client(SetEntityPose,'/world/teacher_demo/set_pose')
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub=self.create_publisher(String,'/demo/obstacle/state',latched)
            self.create_subscription(Bool,'/demo/obstacle/enable',self.enable,10)
            self.create_subscription(String,'/demo/mission/state',self.mission,10)
            self.create_subscription(Odometry,'/demo/slam/body_odom',self.odom,latest_sensor_qos())
            self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def enable(self,msg):
            self.enabled=bool(msg.data)
            if not self.enabled:self.started=None
        def mission(self,msg):
            try:
                state=json.loads(msg.data)
                if state.get('run_id')!=run.name:return
                self.state=state;self.state_wall=time.monotonic();self.trigger.observe(state)
                if state.get('stage') in {'completed','failed','stopped'}:
                    self.enabled=False;self.started=None
            except (ValueError,TypeError):self.trigger.error='invalid actual mission state'
        def odom(self,msg):
            now=int(self.get_clock().now().nanoseconds);stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            p=msg.pose.pose.position
            if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body'
                    or not 0<=now-stamp<=300_000_000 or not all(math.isfinite(v) for v in (p.x,p.y,p.z))):return
            wall=time.monotonic();self.pose_wall=wall
            self.pose=dict(position=[p.x,p.y,p.z],stamp_ns=int(stamp),received_wall_s=wall,
                source='/demo/slam/body_odom',frame_id='camera_init',ground_truth_used=False)
        def record(self,row):
            self.sequence+=1
            self.writer.append(run/'mission46_obstacle_calls.jsonl',dict(sequence=self.sequence,
                run_id=run.name,request_id=self.request_id,navigation_ground_truth_used=False,**row))
        def tick(self):
            ns=int(self.get_clock().now().nanoseconds);now=time.monotonic()
            if self.writer.error:self.failure=self.writer.error
            if self.last_sim is not None and ns<self.last_sim:self.failure='actual obstacle clock moved backward'
            self.last_sim=ns
            # Service ACK handling is required even if enable has just become
            # false. It does not send a replacement parking sweep.
            if self.pending is not None and self.pending.done():
                try:
                    response=self.pending.result();success=bool(response.success)
                    if success:
                        self.position=list(self.sent['position']);self.visible_phase=self.sent['phase'];self.successes+=1
                    else:self.failures+=1;self.failure='Actual SetEntityPose service rejected moving_obstacle'
                    self.record(dict(operation='SetEntityPose_response',success=success,response_sim_ns=ns,
                        response_monotonic_wall=now,**self.sent))
                except Exception as e:
                    self.failures+=1;self.failure=type(e).__name__+': '+str(e)
                    self.record(dict(operation='SetEntityPose_response',success=False,error=self.failure,
                        response_sim_ns=ns,response_monotonic_wall=now,**self.sent))
                self.pending=None;self.sent=None
            fresh=(self.pose is not None and 0<=now-self.pose_wall<.3 and
                   0<=ns-self.pose['stamp_ns']<=300_000_000 and 0<=now-self.state_wall<.3
                   and self.state.get('stage')=='navigating' and self.state.get('navigation_ground_truth_used')is False)
            if self.enabled and self.started is None and not self.armed_once and fresh and self.trigger.nearby(self.pose['position'][:2]):
                self.started=ns;self.request_id=self.state['current_request'];self.armed_once=True
                self.record(dict(operation='trigger_armed',start_sim_ns=ns,start_monotonic_wall=now,
                    mission_state=copy.deepcopy(self.state),actual_slam_pose=copy.deepcopy(self.pose),
                    trigger_point=copy.deepcopy(self.trigger.point),radius_m=1.4))
            # Stale mission/body freezes obstacle motion until source returns.
            # Any actual pause is recorded; the verifier cannot call it the
            # original unmodified duration without the real ACK geometry.
            move=self.enabled and self.started is not None and fresh and self.failure is None
            if move and self.pending is None and self.client.service_is_ready() and (self.last_send_sim is None or ns>self.last_send_sim):
                target,phase=target_at((ns-self.started)/1e9)
                req=SetEntityPose.Request();req.entity.name='moving_obstacle';req.entity.type=Entity.MODEL
                req.pose.position.x,req.pose.position.y,req.pose.position.z=target;req.pose.orientation.w=1.
                self.sent=dict(entity='moving_obstacle',service='/world/teacher_demo/set_pose',position=target,
                    phase=phase,request_sim_ns=ns,request_monotonic_wall=now)
                self.record(dict(operation='SetEntityPose_request',**self.sent))
                self.pending=self.client.call_async(req);self.last_send_sim=ns
            data=dict(run_id=run.name,request_id=self.request_id,active=self.enabled and self.started is not None,
                enabled=self.enabled,position=list(self.position),radius=.36,size=[.5,.5,1.2],
                visible_phase=self.visible_phase,gazebo_updates=self.successes,failed_updates=self.failures,
                sim_ns=ns,stamp=ns/1e9,monotonic_wall=now,source='actual Teacher world SetEntityPose execution ACK',
                actual_pose_info_observer_collected=False,failure=self.failure,
                trigger_navigation_ready=self.trigger.navigation_ready,trigger_waypoint_index=self.trigger.navigation_index,
                trigger_position=self.trigger.point,trigger_error=self.trigger.error,mission_stage=self.trigger.stage,
                navigation_ground_truth_used=False,robot_pose_service_called=False)
            self.writer.atomic(run/'mission46_obstacle_state.json',data)
            self.writer.append(run/'mission46_obstacle_state_history.jsonl',data)
            if self.context.ok():self.pub.publish(String(data=json.dumps(data,allow_nan=False)))
        def close(self):
            self.enabled=False;self.started=None;self.writer.close()
    return Obstacle()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve()
    from pid_scope import verify_scope
    profile=verify_scope(run/'navigation_scope.json')['profile'];validate_profile(profile)
    if (run/'mission46_obstacle_calls.jsonl').exists():raise RuntimeError('Choose a new actual mission obstacle run')
    import rclpy
    rclpy.init(args=ros);node=build_node(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        try:node.close()
        finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
