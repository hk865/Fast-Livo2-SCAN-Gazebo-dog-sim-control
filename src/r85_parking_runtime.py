"""One R85 task's original first-zero-ACK/fixed5s owner; no ROS or actuator.

Task owner supplies actual accepted SLAM callbacks/status receipts and uniquely
archives each exact statusrow to mission46_actual_navigation_statuses.jsonl.
Those historical filenames are required by the unchanged source verifier;
they do not represent a Mission46 run or a 46/46 completion claim.
"""
from collections import deque
import copy
import hashlib
import json
import math
import threading
from pathlib import Path

from parking_contract import MODE, enabled, qualification
from runtime_evidence import (FileTail, native_projection, native_ns,
    bind_zero_qualification_sources, verify_parking, seal)

RESULT_SCHEMA='r85_actual_fixed5_parking_result/v1'
STUDENT_EXPORT_SHA='d8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185'
PROVENANCE='r85_single_task_pose10hz_fixed5_v1'


class R85ParkingRuntime:
    """Persistent single-request owner; one T0 and one audit, failure is terminal."""
    def __init__(self,run,request_id,profile):
        self.run=Path(run).resolve();self.request_id=request_id;self.profile=copy.deepcopy(profile)
        if (not isinstance(request_id,str) or not request_id.startswith(self.run.name+':r85:')
            or profile.get('r85_session',{}).get('enabled')is not True
            or profile.get('actor_kind')!='student' or not enabled(profile)):
            raise ValueError('Explicit R85 Student/original parking contract required')
        self.kind=profile['r85_task']['kind']
        if self.kind not in ('business_chain_prefix','f1_to_f3'):raise ValueError('Unknown R85 task kind')
        if (self.run/'r85_task_parking_result.json').exists():
            raise ValueError('R85 parking result already exists; do not restart/requalify')
        self.native_tail=FileTail(self.run/'telemetry.jsonl',project=native_projection)
        self.native_records=deque(maxlen=1024);self.pose_rows=deque(maxlen=2048)
        self.status_rows=deque(maxlen=2048)
        self.anchor=None;self.anchor_source=None;self.result=None;self.audit_started=False
        self._lock=threading.RLock()
        self.last_pose_stamp=None;self.last_tick_ns=None;self.native_last_ns=None
        self.qualification_sources=None;self.actual_first_T0_ns=None

    def _finish(self,passed,reason=None,receipt=None):
        with self._lock:
            return self._finish_locked(passed,reason,receipt)

    def _finish_locked(self,passed,reason=None,receipt=None):
        if self.result is not None:return self.result
        row=dict(schema=RESULT_SCHEMA,run_id=self.run.name,request_id=self.request_id,kind=self.kind,
            state='passed'if passed else'failed',passed=bool(passed),parking_pass=bool(passed),
            actual_first_T0_ns=self.actual_first_T0_ns,
            fixed_window_end_ns=None if self.actual_first_T0_ns is None else self.actual_first_T0_ns+5_000_000_000,
            formal_window_never_reset=True,audit_attempted=self.audit_started,
            failure_reason=None if passed else str(reason),reason=reason,receipt=receipt,
            provenance=PROVENANCE,compatibility_filenames_are_not_mission46=True,
            original46_completion_claimed=False,full_F3_PASS=False,navigation_PASS=False,
            navigation_ground_truth_used=False,parking_contract=MODE,
            parking_motion_rule_changed=True,parking_position_yaw_limits_unchanged=True,
            native_actuator_limits_changed=False,high_frequency_motion_gate=False,
            actor_kind='student',student_export_sha256=STUDENT_EXPORT_SHA)
        seal(self.run,'r85_task_parking_result.json',row)
        self.result=copy.deepcopy(row)
        return self.result

    def fail(self,reason):
        return self._finish(False,reason)

    def on_pose(self,row):
        """Accept the original callback row, with original source/receipt stamps."""
        with self._lock:return self._on_pose_locked(row)

    def _on_pose_locked(self,row):
        if self.result is not None:return self.result
        try:
            stamp=row['stamp_ns'];sim=row['received_sim_ns'];wall=row['received_wall_ns']
            source_wall=row['source_received_wall_ns'];p=row['position'];q=row['quaternion_xyzw']
            values=list(p)+list(q)+list(row['body_velocity'])+list(row['body_angular_velocity'])
            if (any(type(v)is not int or v<0 for v in (stamp,sim,wall,source_wall))
                or row.get('frame_id')!='camera_init' or row.get('child_frame_id')!='demo_slam_body'
                or row.get('source_topic')!='/demo/slam/body_odom' or row.get('navigation_ground_truth_used')is not False
                or len(p)!=3 or len(q)!=4 or len(row['body_velocity'])!=3 or len(row['body_angular_velocity'])!=3
                or not all(type(v)in (int,float) and math.isfinite(v)for v in values)
                or not .98<=sum(v*v for v in q)<=1.02
                or not 0<=sim-stamp<=300_000_000 or not 0<=wall-source_wall<=300_000_000):
                raise ValueError('R85 original SLAM callback source predicates rejected')
            if self.last_pose_stamp is not None and stamp<=self.last_pose_stamp:
                # Same exact original callback must never add samples/dwell.
                return None
            self.last_pose_stamp=stamp;self.pose_rows.append(copy.deepcopy(row))
        except (ValueError,TypeError,KeyError)as error:
            return self.fail(type(error).__name__+': '+str(error))
        return None

    def poll_native(self,max_bytes=2*1024**2):
        """Read actual serialized native records incrementally; never run a model."""
        with self._lock:return self._poll_native_locked(max_bytes)

    def _poll_native_locked(self,max_bytes):
        if self.result is not None:return self.result
        try:
            for record in self.native_tail.poll(max_bytes=max_bytes):
                ns=native_ns(record['data'])
                if self.native_last_ns is not None and ns<=self.native_last_ns:
                    raise ValueError('Original native physical clock repeated/backwards')
                self.native_last_ns=ns;self.native_records.append(record)
        except (OSError,ValueError,TypeError,KeyError)as error:
            return self.fail(type(error).__name__+': '+str(error))
        return None

    def on_status(self,statusrow):
        """The caller archives this exact row once before/with invoking this hook."""
        with self._lock:return self._on_status_locked(statusrow)

    def _on_status_locked(self,statusrow):
        if self.result is not None:return self.result
        try:
            row=copy.deepcopy(statusrow);data=row['data'];now=row['received_sim_ns']
            if type(now)is not int or now<0 or type(row['received_wall_ns'])is not int:
                raise ValueError('Actual status callback requires original integer clocks')
            rid=data.get('request_id')
            if rid!=self.request_id:
                if self.anchor is not None:raise ValueError('Fixed5s parking request changed')
                return None
            if row.get('r85_provenance')!=PROVENANCE:
                raise ValueError('R85 callback provenance absent; historical filename is compatibility only')
            self.status_rows.append(row)
            if data.get('state')=='failed':return self.fail('actual_controller_failed')
            cascade=data.get('cascade_parking')or{};owner=cascade.get('ordinary_parking')or{}
            proof=owner.get('first_actor_zero_ack')
            if self.anchor is not None:
                if (cascade.get('ordinary_terminal_zero_event')!=self.anchor['event']
                    or proof!=self.anchor['proof']):
                    raise ValueError('Original first T0/zero event changed; no later window allowed')
                return None
            if proof is None:return None
            # Seeing the first proof is a single qualification attempt. If it
            # lacks actual sources, fail rather than select a later zero ACK.
            if data.get('state')!='succeeded':raise ValueError('First ordinary proof is not the original arrived task')
            anchor=qualification(data,self.run.name,self.request_id,now)
            if anchor is None:raise ValueError('First zero proof did not qualify')
            self.anchor=copy.deepcopy(anchor);self.actual_first_T0_ns=anchor['start_stamp_ns']
            self.anchor_source=row
            if (anchor['proof'].get('actual_actor_kind')!='student'
                or anchor['proof'].get('actual_actor_observation_dimension')!=45
                ):
                raise ValueError('Actual first zero ACK was not the selected Student45 inference')
            failure=self.poll_native()
            if failure is not None:return failure
            self.qualification_sources=bind_zero_qualification_sources(self.run,anchor,row,tuple(self.native_records))
            seal(self.run,'mission46_parking_T0_anchor.json',dict(schema='ordinary_fixed5_first_T0_anchor/v1',
                run_id=self.run.name,request_id=self.request_id,anchor=copy.deepcopy(anchor),
                coordinator_first_observed_clock_ns=now,navigation_ground_truth_used=False,
                r85_provenance=PROVENANCE,original46_completion_claimed=False,
                actual_actor_kind='student',actual_student_export_sha256=STUDENT_EXPORT_SHA))
        except (OSError,ValueError,TypeError,KeyError,OverflowError)as error:
            return self.fail(type(error).__name__+': '+str(error))
        return None

    def audit_due(self,now_sim_ns):
        if type(now_sim_ns)is not int or now_sim_ns<0:raise ValueError('Original integer ROS clock required')
        return bool(self.result is None and self.anchor is not None and not self.audit_started
            and now_sim_ns>=self.actual_first_T0_ns+5_200_000_000)

    def audit_once(self,now_sim_ns):
        """One synchronous unchanged audit; caller may dispatch on its executor.

        The extra .2s only allows original callbacks to arrive. Verification is
        strictly the same first T0..T0+5s, with original counts/coverage/limits.
        """
        if self.result is not None:return self.result
        with self._lock:
            if not self.audit_due(now_sim_ns):return None
            self.audit_started=True
        try:
            failure=self.poll_native()
            if failure is not None:return failure
            with self._lock:
                poses,natives,statuses=tuple(self.pose_rows),tuple(self.native_records),tuple(self.status_rows)
                anchor=copy.deepcopy(self.anchor)
            old=verify_parking(self.run,self.request_id,self.actual_first_T0_ns,
                poses,natives,statuses,anchor['origin'],parking_contract=MODE,anchor=anchor,
                evidence_basename='r85_fixed5_parking_evidence.json')
            if (old.get('passed')is not True or old.get('binding_verified')is not True
                or old.get('run_id')!=self.run.name or old.get('request_id')!=self.request_id
                or old.get('start_stamp_ns')!=self.actual_first_T0_ns
                or old.get('end_stamp_ns')!=self.actual_first_T0_ns+5_000_000_000
                or old.get('parking_contract')!=MODE):
                raise ValueError('Original fixed5 verifier returned a mismatched receipt')
            helper_schema=old['schema'];adapted={k:v for k,v in old.items()if k!='schema'}
            adapted.update(schema='r85_reused_original_fixed5_verifier_receipt/v1',
                original_helper_schema=helper_schema,r85_provenance=PROVENANCE,
                historical_helper_name_is_not_original46_completion=True)
            return self._finish(True,receipt=adapted)
        except (OSError,ValueError,TypeError,KeyError,OverflowError)as error:
            return self.fail(type(error).__name__+': '+str(error))

    def tick(self,now_sim_ns):
        if self.result is not None:return self.result
        if type(now_sim_ns)is not int or now_sim_ns<0:return self.fail('Original integer ROS clock required')
        if self.last_tick_ns is not None and now_sim_ns<self.last_tick_ns:
            return self.fail('Original ROS clock backwards')
        self.last_tick_ns=now_sim_ns
        return self.audit_once(now_sim_ns)
