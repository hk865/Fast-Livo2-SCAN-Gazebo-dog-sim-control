#!/usr/bin/env python3
"""SLAM pure-turn drift stop/replan controller; region and safety contracts remain.

Only measured raw SLAM position causes this additional STOP for schema2 goals.
Legacy waypoints retain original Navigation behavior without arming this guard.
No truth, velocity compensation or body feedback. Original region, safety and
deadline methods run unchanged. Dependencies resolve from the navigation folder.
"""
from __future__ import annotations
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from collections import deque
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'navigation'))
from controller import Navigation, rclpy, _ros, String
from control_core import follow_trajectory
from turn_drift import TurnDriftSupervisor


class StatusProxy:
    """Add diagnostics to the existing publisher, with no additional ROS node."""
    def __init__(self,publisher,owner):self.publisher,self.owner=publisher,owner
    def publish(self,msg):
        value=json.loads(msg.data)
        value['turn_drift_guard']=self.owner.drift_status()
        self.publisher.publish(String(data=json.dumps(value,ensure_ascii=False)))


class NavigationDrift(Navigation):
    def __init__(self):
        super().__init__()
        self.drift=None
        self.drift_pending=False
        self.drift_stop_clock_ns=None
        self.drift_stop_wall=None
        self.drift_stop_count=None
        self.drift_bridge_wall=-math.inf
        self.drift_bridge_sequence=0
        self.drift_consumed_bridge_sequence=0
        self.drift_log=deque(maxlen=20000)
        self.drift_log_dropped=0
        self.drift_events=0
        self.status_pub=StatusProxy(self.status_pub,self)

    def drift_record(self,event,**value):
        if len(self.drift_log)==self.drift_log.maxlen:self.drift_log_dropped+=1
        self.drift_log.append(dict(event=event,callback_clock_ns=self.get_clock().now().nanoseconds,
            callback_wall_ns=time.monotonic_ns(),request_id=self.request_id,
            waypoint_index=self.waypoint_index,**value))

    def drift_status(self):
        return dict(candidate='displacement_015_02',enabled=True,pending=self.drift_pending,
            state='inactive' if self.drift is None else self.drift.state,
            limits=dict(TurnDriftSupervisor.LIMITS,planar_drift_m=.15,persistence_ns=200000000,
                        min_raw_observations=3,require_path_offset=False),
            interruptions=self.drift_events,stop_clock_ns=self.drift_stop_clock_ns,
            stop_adapter_count=self.drift_stop_count,log_dropped=self.drift_log_dropped,
            latest=None if not self.drift_log else self.drift_log[-1])

    def drift_context(self):
        if (self.state!='running' or self.waypoint_index>=len(self.goals)
                or self.goals[self.waypoint_index].legacy
                or self.samples is None or self.active_trajectory_id is None
                or self.trajectory_association.reference_stamp is None
                or self.trajectory_association.accepted is None
                or self.trajectory_association.accepted['traj_id']!=self.active_trajectory_id):return None
        return (self.request_id,self.goals[self.waypoint_index].goal_id,self.active_trajectory_id,
                tuple(self.trajectory_association.reference_stamp))

    def drift_protected(self,now):
        clock=self.get_clock().now().nanoseconds/1e9
        return (self.state!='running' or self.tilt_hold or self.obstacle_hold
            or self.bridge_safety.get('state')!='ready'
            or now-self.drift_bridge_wall>.25
            or now-self.pose_updated>self.pose_timeout
            or now-self.cloud_updated>self.cloud_timeout
            or not self.raw_imu.fresh(now,clock))

    def publish_command(self,velocity=None,yaw_rate=0.):
        # An interruption never traverses the acceleration limiter: exact zero
        # is sent using the original implementation before changing bookkeeping.
        if self.drift_pending:
            return super().publish_command()
        result=super().publish_command(velocity,yaw_rate)
        context=self.drift_context()
        planned_pure=(velocity is not None and np.array_equal(np.asarray(velocity),np.zeros(2))
                      and self.command[0]==0. and self.command[1]==0. and self.command[2]!=0.)
        if (planned_pure and self.heading_gate.phase=='align' and context is not None
                and self.region_raw_pose is not None and self.heading_gate.heading is not None):
            # A legitimate new checked source starts a new observation window;
            # it does not trigger an extra source-change stop condition.
            if self.drift is None or self.drift.state!='observing' or self.drift.context!=context:
                self.drift=TurnDriftSupervisor(require_path_offset=False,profile='displacement_015_02')
                self.drift.begin(context,int(self.pose_stamp),self.region_raw_pose,
                                 self.heading_gate.heading,self.waypoints[self.waypoint_index])
                self.drift_record('arm',context=context,raw_anchor=self.region_raw_pose.tolist(),
                                  anchor_stamp_ns=int(self.pose_stamp))
        elif self.drift is not None and self.drift.state=='observing':
            self.drift=None
        return result

    def on_odom(self,msg):
        prior_stamp=self.pose_stamp
        super().on_odom(msg)  # Original filtering, cloud frames and SCAN odom.
        if self.pose_stamp<=prior_stamp or self.drift_pending or self.drift is None:return
        context=self.drift_context()
        if context is None or context!=self.drift.context:
            self.drift=None  # Never accumulate drift across a new goal/identity.
            return
        bridge=self.bridge_safety
        safe=bridge.get('safe')
        adapter=bridge.get('joint_adapter',{})
        if not isinstance(safe,list) or len(safe)!=3 or not isinstance(adapter,dict):return
        now=time.monotonic()
        age=self.get_clock().now().nanoseconds-int(self.pose_stamp)
        # Planned pure rotation and actual guarded applied command are both
        # required; a stale/nonzero translation never qualifies as turn drift.
        pure_plan=self.command[0]==0. and self.command[1]==0. and self.command[2]!=0.
        _,_,_,steering=follow_trajectory(self.pose,self.rotation,self.samples,
            self.waypoints[self.waypoint_index],max_speed=self.max_speed,
            tracking_pose=self.tracking_pose,gate_translation=False,return_steering=True)
        result=self.drift.observe(int(self.pose_stamp),self.region_raw_pose,steering['heading'],context,
            phase=self.heading_gate.phase,actual_command=safe,adapter_state=adapter.get('state'),
            protected=bool(self.drift_protected(now) or not pure_plan),pose_age_ns=int(age))
        self.drift_record('observe',measurement=result,pose_stamp_ns=int(self.pose_stamp),
                          planned_command=list(self.command),bridge_safe=list(safe),
            adapter_state=adapter.get('state'),bridge_age_wall_s=now-self.drift_bridge_wall,
            command_evidence='bridge_safe_is_adapter_input; fresh_walk_protocol_forwards_it; native_emit_verified_offline')
        if result['event'] is not None:self.drift_stop(result['event'])

    def drift_stop(self,event):
        # The first side effect is the original exact-zero publication.
        super().publish_command()
        self.drift_stop_clock_ns=self.get_clock().now().nanoseconds
        self.drift_stop_wall=time.monotonic()
        self.drift_stop_count=self.bridge_safety.get('joint_adapter',{}).get('counters',{}).get('stops')
        self.drift_pending=True
        self.drift_events+=1
        self.reset_region_arrival('turn_drift_stop')
        self.arrival_since=None
        self.samples=None
        self.active_trajectory_id=None
        self.planning_start_state=None
        self.steering=None
        self.trajectory_association.reset()
        self.obstacle_resume_pending=False
        self.heading_gate.reset()
        self.alignment_hold=True
        self.last_reference=-math.inf
        self.drift_record('exact_zero_published',trigger=event,command=list(self.command),
                          actual_zero_command_clock_ns=self.drift_stop_clock_ns,
                          adapter_stop_count_before=self.drift_stop_count)
        # segment_start/deadline, waypoint, region definitions and safety
        # thresholds are deliberately untouched.
        self.message='纯转实际 SLAM 平移达到测试门限，停车等待真实归位与新 SCAN'
        self.publish_status()

    def on_bridge_safety(self,msg):
        super().on_bridge_safety(msg)  # Original failed/tilt propagation first.
        try:value=json.loads(msg.data)
        except (ValueError,TypeError):return
        if not isinstance(value,dict) or value.get('state') not in ('hold','ready','failed'):return
        self.drift_bridge_wall=time.monotonic()
        self.drift_bridge_sequence+=1
        self.drift_handoff(self.drift_bridge_wall)

    def measured_region_arrival(self):
        if self.drift_pending:
            self.reset_region_arrival('turn_drift_stop')
            return False,False
        return super().measured_region_arrival()

    def request_plan(self):
        if self.drift_pending:
            # Only the explicit handoff may request a reference while stopped.
            return
        return super().request_plan()

    def drift_handoff(self,now):
        if not self.drift_pending or self.drift is None or self.state!='running':return
        if self.drift_bridge_sequence==self.drift_consumed_bridge_sequence:return
        self.drift_consumed_bridge_sequence=self.drift_bridge_sequence
        if self.drift.state=='await_paired_source' and now-self.last_reference>3.0:
            # The original no-spline retry interval remains bounded by the
            # original goal deadline. Repeat the real idle handshake before
            # replacing a failed or absent SCAN response; never use its curve.
            self.drift.state='await_zero_idle'
            self.drift.zero_since=None
        bridge=self.bridge_safety
        adapter=bridge.get('joint_adapter',{})
        safe,requested=bridge.get('safe'),bridge.get('requested')
        if not isinstance(adapter,dict) or not isinstance(safe,list) or len(safe)!=3:return
        if not isinstance(requested,list) or len(requested)!=3:return
        if not np.isfinite(safe+requested).all():return
        t=int(self.get_clock().now().nanoseconds)
        adapter_time=adapter.get('sim')
        if not isinstance(adapter_time,(int,float)) or not math.isfinite(adapter_time):return
        # The bridge's actual protocol supplies floating sim time, not a ROS
        # Header. It is explicitly converted here and never used as raw odom.
        adapter_ns=round(adapter_time*1e9)
        age=bridge.get('joint_adapter_wall_age')
        stops=adapter.get('counters',{}).get('stops')
        ack=(type(stops) is int and type(self.drift_stop_count) is int
             and stops>self.drift_stop_count and adapter_ns>self.drift_stop_clock_ns
             and now>self.drift_stop_wall and type(age) in (int,float)
             and math.isfinite(age) and 0<=age<=.25)
        protected=(self.drift_protected(now) or not ack or any(requested))
        event=self.drift.execution(t,safe,adapter_state=adapter.get('state'),
            nominal_calibrated=adapter.get('nominal_calibrated'),adapter_stamp_ns=adapter_ns,
            adapter_age_ns=0 if age is None else round(age*1e9),protected=bool(protected))
        if event is None:return
        # These callbacks cannot preempt original request_plan: single threaded
        # executor commits an immutable new ref before metadata can be received.
        previous_requests=self.reference_requests
        super().request_plan()
        if self.reference_requests==previous_requests:
            # No planner subscriber: retry later, never authorize motion.
            self.drift.state='await_zero_idle'
            return
        self.drift.declare_reference(self.trajectory_association.reference_stamp,
                                     self.waypoints[self.waypoint_index])
        self.drift_record('fresh_reference_requested',handoff=event,
            reference_stamp=list(self.trajectory_association.reference_stamp),
            actual_adapter_ack=adapter,bridge_safe=safe,bridge_requested=requested,
            zero_output_evidence='fresh post-STOP adapter idle with incremented stop counter; original idle emits only exact zero',
            adapter_sim_timestamp_source='existing floating adapter status, not raw SLAM Header')

    def accept_spline(self,msg,metadata):
        if self.drift_pending:
            if self.drift is None or self.drift.state!='await_paired_source':return
            if (metadata.get('reference_stamp')!=list(self.drift.reference)
                    or int(msg.traj_id)<=int(self.drift.context[2])):
                self.trajectory_association.reject('drift handoff requires new checked source')
                return
        super().accept_spline(msg,metadata)  # Original pose/progress/SCAN checks.
        if self.drift_pending and self.samples is not None and self.active_trajectory_id==msg.traj_id:
            self.drift.metadata(metadata)
            event=self.drift.spline(msg)
            if event is None:
                self.samples=None
                self.active_trajectory_id=None
                super().publish_command()
                return
            self.drift_pending=False
            self.heading_gate.reset()  # Original 1 s preturn + align + 1 s settle.
            self.alignment_hold=True
            self.drift_record('fresh_checked_path_accepted',identity=event)
            self.drift=None

    def control(self,now):
        # Every stopped tick still executes the original 90 s deadline,
        # raw IMU/SLAM tilt, sensor watchdog and mission terminal handling.
        super().control(now)
        if self.state!='running' and self.drift_pending:
            self.drift_record('original_terminal_during_drift_hold',state=self.state,message=self.message)
            self.drift_pending=False
            self.drift=None

    def on_request(self,msg):
        previous=self.request_id
        super().on_request(msg)
        if self.request_id!=previous:
            self.drift=None
            self.drift_pending=False
            self.drift_stop_clock_ns=self.drift_stop_wall=self.drift_stop_count=None

    def close_drift(self):
        out=os.environ.get('DEMO_RUN_DIR')
        if not out:return
        out=Path(out)
        value=dict(candidate='displacement_015_02',events=list(self.drift_log),
            dropped=self.drift_log_dropped,interruptions=self.drift_events,
            source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                (Path(__file__).resolve(),Path(__file__).with_name('turn_drift.py'),ROOT/'navigation/controller.py',
                 ROOT/'navigation/control_core.py',ROOT/'navigation/trajectory_contract.py')})
        tmp=out/'nav_drift_guard.json.tmp'
        tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
        tmp.replace(out/'nav_drift_guard.json')


def main():
    rclpy.init()
    node=NavigationDrift()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    except _ros.RCLError:
        if rclpy.ok():raise
    finally:
        try:
            if rclpy.ok():node.publish_command()
        finally:
            try:node.close_drift()
            finally:
                node.event_archive.close()
                node.destroy_node()
                rclpy.try_shutdown()

if __name__=='__main__':main()
