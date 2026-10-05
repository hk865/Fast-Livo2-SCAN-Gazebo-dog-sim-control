#!/usr/bin/env python3
"""Excluded ALIGN raw-SLAM longitudinal PD plus original drift STOP chain.

Only fresh raw SLAM drives the additional longitudinal PD and drift STOP. No
truth or active body feedback enters control. Original Navigation is frozen.
The explicit scenario_path ROS parameter is required by the archived launch.
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
from align_translation import AlignTranslation
from control_core import obstacle_ahead


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


class NavigationAlign(NavigationDrift):
    """Additional ALIGN translation; original schema2 deadlines/guards remain.

Disabled mode calls every original NavigationDrift execution method unchanged.
Enabled mode captures rotation from exactly the accepted fresh pose, since the
original compatibility callback can overwrite self.rotation on duplicate data.
"""
    def __init__(self):
        selected=os.environ.get('DEMO_TEST_ALIGN_TRANSLATION_ENABLED')
        if selected not in ('0','1'):raise ValueError('explicit test ALIGN translation enabled 0/1 required')
        self.align_enabled=selected=='1';self.align_pd=AlignTranslation()
        self.align_raw=None;self.align_motion_guard=None;self.align_emit=None
        super().__init__()

    def drift_context(self):
        context=super().drift_context()
        if self.align_enabled and context is not None and self.goals[self.waypoint_index].legacy:return None
        return context

    def drift_status(self):
        value=super().drift_status()
        value['align_translation']=dict(enabled=self.align_enabled,config=dict(AlignTranslation.CONFIG),
            latest=dict(self.align_pd.latest),motion_guard=self.align_motion_guard,emit=self.align_emit,
            scope='raw-SLAM ALIGN only; mixed yaw <=.08; existing .15/.2/3 drift protection remains',
            command_evidence='emitted NAV request; bridge safe is adapter input, not applied plant velocity')
        return value

    def _align_new_drift(self,context):
        self.drift=TurnDriftSupervisor(require_path_offset=False,profile='displacement_015_02',allow_align_translation=True)
        self.drift.begin(context,int(self.align_raw['stamp']),self.align_raw['p'],
                         self.heading_gate.heading,self.waypoints[self.waypoint_index])
        self.drift_record('arm',context=context,raw_anchor=self.align_raw['p'].tolist(),
                          anchor_stamp_ns=self.align_raw['stamp'],align_translation_scope=True)

    def _align_reset(self,reason):
        self.align_pd.reset(reason)

    def _align_checked_stop(self,reason,details):
        context=self.drift_context();raw=self.align_raw
        if self.drift is None or context is None or raw is None:
            self._align_reset(reason);return Navigation.publish_command(self)
        self.drift.state='await_zero_idle';self.drift.stop_stamp=int(self.get_clock().now().nanoseconds)
        self.drift.since=None;self.drift.zero_since=None;self.drift.latest_zero=False
        self.drift_stop(dict(event=reason,stamp_ns=raw['stamp'],context=context,
                             raw_position=raw['p'].tolist(),details=details))

    def _align_body_corridor(self,vx):
        p,R=self.align_raw['p'],self.align_raw['R']
        # Union with the original forward/path corridor remains in Navigation.
        # This additional corridor follows actual signed body-x translation.
        direction=R[:2,0]*math.copysign(1.,vx)
        norm=float(np.linalg.norm(direction))
        ctx=self.cloud_input_context
        clock=int(self.get_clock().now().nanoseconds)
        matched=(isinstance(ctx,dict) and ctx.get('frame_id')=='camera_init'
            and type(ctx.get('message_stamp_ns')) is int and ctx['message_stamp_ns']==self.cloud_stamp
            and 0<=clock-ctx['message_stamp_ns']<=round(self.cloud_timeout*1e9)
            and type(ctx.get('filtering_body_stamp_ns')) is int
            and abs(ctx['filtering_body_stamp_ns']-ctx['message_stamp_ns'])<=150000000
            and isinstance(self.cloud,np.ndarray) and self.cloud.ndim==2 and self.cloud.shape[1]==3
            and len(self.cloud)>0 and ctx.get('filtered_points')==len(self.cloud)
            and 0<=time.monotonic()-self.cloud_updated<=self.cloud_timeout)
        if (not matched or norm<1e-6 or not np.isfinite(self.cloud).all()):
            return True,dict(reason='unmatched_or_invalid_motion_cloud',blocked=True)
        target=p.copy();target[:2]+=.8*direction/norm
        result=obstacle_ahead(self.cloud,p,target,np.vstack([self.segment_start,self.waypoints[self.waypoint_index]]))
        return bool(result[0]),dict(blocked=bool(result[0]),clearance=result[1],points=int(result[2]),
            body_x_sign=1 if vx>0 else -1,raw_pose_stamp_ns=self.align_raw['stamp'],
            cloud_message_stamp_ns=self.cloud_input_context['message_stamp_ns'],
            context=self.drift_context(),corridor='original .95m/.42m/min3; actual signed body-x')

    def publish_command(self,velocity=None,yaw_rate=0.):
        if not getattr(self,'align_enabled',False):return super().publish_command(velocity,yaw_rate)
        context=self.drift_context();phase=self.heading_gate.phase
        eligible=(velocity is not None and phase=='align' and context is not None
                  and self.heading_gate.heading is not None and not self.drift_pending)
        if not eligible:
            self._align_reset('phase_identity_or_pending')
            return super().publish_command(velocity,yaw_rate)
        wall=time.monotonic_ns();clock=int(self.get_clock().now().nanoseconds)
        protected=self.drift_protected(wall/1e9)
        raw=self.align_raw
        fresh=(raw is not None and raw['stamp']==self.pose_stamp and 0<=clock-raw['stamp']<=250000000
               and 0<=wall-raw['wall']<=250000000)
        if protected or not fresh:
            self._align_reset('protected_or_stale_raw')
            return Navigation.publish_command(self)
        if self.align_pd.context!=context:
            self.align_pd.begin(context,raw['stamp'],raw['p'],raw['R'],raw['wall'])
        if self.drift is None or self.drift.state!='observing' or self.drift.context!=context:
            self._align_new_drift(context)
        vx=self.align_pd.output(context,clock,wall,phase=phase,protected=False)
        if not self.align_pd.latest['valid'] and self.align_pd.latest['reason'] not in ('waiting_fresh_fit','waiting_fit_span'):
            return self._align_checked_stop('stop_for_align_estimate_quality',dict(self.align_pd.latest))
        desired_yaw=float(yaw_rate)
        # Cover both entering mixed motion and leaving it. The original base
        # zeros translation immediately for a zero target; retaining this cap
        # also makes that safety boundary explicit at this candidate boundary.
        if vx!=0. or self.command[0]!=0.:desired_yaw=float(np.clip(desired_yaw,-.08,.08))
        self.align_motion_guard=None
        if vx!=0.:
            blocked,self.align_motion_guard=self._align_body_corridor(vx)
            if blocked:
                # Exact zero first and the existing ACK/idle/fresh-SCAN chain.
                # This is never merely vx=0 with a continued unsafe old turn.
                return self._align_checked_stop('stop_for_align_translation_corridor',self.align_motion_guard)
            # Original yaw acceleration limiting can temporarily retain .12.
            # Do not emit translation until the previously emitted yaw is .08.
            if abs(self.command[2])>.08+1e-12 or abs(self.command[0])>.10+1e-12:vx=0.
        result=Navigation.publish_command(self,np.array([vx,0.]),desired_yaw)
        self.align_emit=dict(stamp_ns=clock,helper_vx=self.align_pd.latest.get('vx_m_s',0.),
            desired=[vx,0.,desired_yaw],emitted=list(self.command),
            original_acceleration_limit_m_s2=.15,helper_slew_limit_m_s2=.2)
        return result

    def on_odom(self,msg):
        if not self.align_enabled:return super().on_odom(msg)
        prior=self.pose_stamp
        Navigation.on_odom(self,msg)  # Original frames/filter/SCAN input unchanged.
        if self.pose_stamp<=prior:return
        # This rotation belongs to the just-accepted fresh message, not a later
        # duplicate/foreign callback's self.rotation or interpolated estimate.
        self.align_raw=dict(stamp=int(self.pose_stamp),p=self.region_raw_pose.copy(),
                            R=self.rotation.copy(),wall=time.monotonic_ns())
        context=self.drift_context()
        if self.drift_pending or context is None or self.heading_gate.phase!='align':
            self._align_reset('pose_outside_align_or_pending')
            return
        if self.align_pd.context is not None:
            accepted=self.align_pd.observe(context,self.align_raw['stamp'],self.align_raw['p'],
                                           self.align_raw['R'],self.align_raw['wall'])
            if not accepted and self.align_pd.context is None:
                return self._align_checked_stop('stop_for_align_raw_gap_or_identity',dict(self.align_pd.latest))
        if self.drift is None:return
        if self.drift.context!=context:
            self.drift=None;return
        bridge=self.bridge_safety;safe=bridge.get('safe');adapter=bridge.get('joint_adapter',{})
        if not isinstance(safe,list) or len(safe)!=3 or not isinstance(adapter,dict):return
        _,_,_,steering=follow_trajectory(self.pose,self.rotation,self.samples,
            self.waypoints[self.waypoint_index],max_speed=self.max_speed,
            tracking_pose=self.tracking_pose,gate_translation=False,return_steering=True)
        planned=(self.command[1]==0. and abs(self.command[0])<=.10+1e-12
                 and abs(self.command[2])<=(.08 if self.command[0]!=0. else .12)+1e-12 and any(self.command))
        age=int(self.get_clock().now().nanoseconds)-self.align_raw['stamp']
        result=self.drift.observe(self.align_raw['stamp'],self.align_raw['p'],steering['heading'],context,
            phase='align',actual_command=safe,adapter_state=adapter.get('state'),
            protected=bool(self.drift_protected(time.monotonic()) or not planned),pose_age_ns=age)
        self.drift_record('observe',measurement=result,pose_stamp_ns=self.align_raw['stamp'],
            planned_command=list(self.command),bridge_safe=list(safe),adapter_state=adapter.get('state'),
            bridge_age_wall_s=time.monotonic()-self.drift_bridge_wall,
            command_evidence='ALIGN stage includes translation; bridge safe is adapter input, native emission verified offline')
        if result['event'] is not None:self.drift_stop(result['event'])

    def drift_stop(self,event):
        if self.align_enabled:self._align_reset('drift_or_corridor_stop')
        return super().drift_stop(event)

    def on_request(self,msg):
        previous=self.request_id
        result=super().on_request(msg)
        if self.align_enabled and self.request_id!=previous:self._align_reset('new_request')
        return result

    def close_drift(self):
        super().close_drift()
        directory=os.environ.get('DEMO_RUN_DIR')
        if not directory:return
        out=Path(directory);path=out/'nav_drift_guard.json'
        value=json.loads(path.read_text())
        helper=Path(__file__).with_name('align_translation.py')
        value.update(align_translation=dict(enabled=self.align_enabled,config=dict(AlignTranslation.CONFIG)))
        value['source_sha256'][str(helper)]=hashlib.sha256(helper.read_bytes()).hexdigest()
        tmp=out/'nav_drift_guard.json.tmp';tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');tmp.replace(path)


def main():
    rclpy.init()
    node=NavigationAlign()
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
