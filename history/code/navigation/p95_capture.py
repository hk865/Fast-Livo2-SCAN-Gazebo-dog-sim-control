"""Final-request P95 observer; original matcher/control predicates are untouched.

Only worker-owned checked jobs can supply source pins. The legacy whole-message
writer is reused structurally; an event is never an NN failure or accepted pose.
"""
from pathlib import Path
import copy, json, math, threading, time
from event_aware_source_capture import (BoundedSourceFrames, CAPS, GATES, TOPICS,
    TRIGGER as NN_TRIGGER, encoded, file_sha, sha)

SCHEMA='go2_P95_quality_source_capture_contract/v1'
TRIGGER=dict(original_predicate='top_candidate_only_ray_p95_failed',first_real_failure=True,
    mission_stage='navigating',current_request_phase='navigation_f1_f3',
    complete_previous_jobs=1,post_original_checked_job=True)

class P95SourceFrames(BoundedSourceFrames):
    def __init__(self,run,*,contract=None,serialize=None,deserialize=None,start_writer=True,
                 wall_ns=time.monotonic_ns):
        run=Path(run)
        if contract is None:
            p=run/'known_scene_event_capture_contract.json'
            if p.stat().st_size>64*1024:raise ValueError('capture contract too large')
            contract=json.loads(p.read_text())
        if (contract.get('schema')!=SCHEMA or contract.get('run_id')!=run.name
            or contract.get('caps')!=CAPS or contract.get('unchanged_gates')!=GATES
            or contract.get('topics')!=TOPICS or contract.get('diagnostic_trigger')!=TRIGGER
            or contract.get('diagnostic_only') is not True):
            raise ValueError('exact frozen final-request P95 contract required')
        bindings=contract.get('source_bindings',[])
        if not bindings:raise ValueError('P95 source bindings required')
        for item in bindings:
            if file_sha(item['path'])!=item['sha256']:raise ValueError('frozen P95 source differs')
        # The alias exists only to reuse the validated storage constructor. No
        # writer, callback or file sees the temporary NN schema/hash.
        actual=copy.deepcopy(contract);alias=copy.deepcopy(contract)
        alias['schema']='go2_P1_association_source_capture_contract/v1'
        alias['diagnostic_trigger']=dict(NN_TRIGGER)
        super().__init__(run,contract=alias,serialize=serialize,deserialize=deserialize,
            start_writer=False,wall_ns=wall_ns)
        self.contract=actual;self.contract_sha256=sha(encoded(actual))
        self.bindings=copy.deepcopy(bindings);self.p95_scope=None
        if start_writer:
            self.writer=threading.Thread(target=self._writer,name='bounded_P95_source_writer',daemon=True)
            self.writer.start()

    def _scope(self):
        m=self.mission or {};request=m.get('request_id')
        if (m.get('stage')=='navigating' and isinstance(request,str)
            and request.startswith(self.run.name+':navigation_f1_f3:')):
            return (self.run.name,request,'navigating')
        return None

    def _clock(self,ns):
        self.last_clock=max(self.last_clock,int(ns))
        if (self.arm and self.capture_needed
            and ns>=self.arm['runtime_clock_ns']+CAPS['event_window_ns']):
            self.expired=True;self.capture_needed=False
            self._event('P95_sampling_window_expired_only',runtime_clock_ns=int(ns),task_stop_requested=False)

    def observe_mission(self,text,runtime_clock_ns):
        ok=super().observe_mission(text,runtime_clock_ns)
        if ok:
            with self.lock:
                scope=self._scope()
                if scope!=self.p95_scope:
                    self.previous=None;self.inflight_source=None;self.p95_scope=scope
        return ok

    def observe_job(self,job,current_c,parent_generation):
        with self.lock:
            scope=self._scope()
            if (scope is None or self.closed or self.expired or self.terminal
                or not self.capture_needed):
                self.inflight_source=None
                return False
            self.inflight_source=self._snapshot(job,None,current_c,parent_generation)
            self.inflight_source['actual_final_request_scope']=scope
        return True

    @staticmethod
    def _p95_failed(result,source_ns):
        candidates=result.get('candidates');policy=result.get('policy') or {}
        if (result.get('source_ns')!=source_ns or result.get('reason')!='no_quality_candidate'
            or result.get('accepted') is not False or result.get('control_allowed') is not False
            or not isinstance(candidates,list) or not candidates
            or candidates[0].get('rejections')!=['ray_p95']
            or policy.get('ray_abs_p95_max_m')!=GATES['allray_abs_p95_m']):return False
        value=(candidates[0].get('rays') or {}).get('abs_p95_m')
        return (type(value) in (int,float) and math.isfinite(value)
            and value>GATES['allray_abs_p95_m'])

    def consider(self,job,result,current_c,parent_generation):
        with self.lock:
            snap=self.inflight_source;self.inflight_source=None;scope=self._scope()
            if (self.closed or self.expired or self.terminal or not self.capture_needed
                or scope is None):return False
            # An asynchronous result cannot retrieve a newer source cache. The
            # five references are those pinned for this exact worker job.
            if (snap is None or snap['source_ns']!=job.get('source_ns')
                or snap.get('actual_final_request_scope')!=scope
                or any(snap['source_messages'].get(k) is not job.get('source_messages',{}).get(k)
                    for k in ('raw','adapted','registered','body','mapper'))):
                return self.reject(job.get('source_ns'),'P95_missing_exact_inflight_source_pin')
            snap['result']=dict(result)
            if self.arm:
                if (job['source_ns']>self.arm['source_ns'] and 'POST' not in self.selected
                    and tuple(self.arm['actual_final_request_scope'])==scope):
                    self._select('POST',snap,dict(kind='first_original_checked_job_after_P95_rejection',
                        original_match_accepted=result.get('accepted'),actual_final_request_scope=scope,
                        task_stop_requested=False))
                    self.capture_needed=False
                    self._event('P95_sampling_selection_complete_only',task_stop_requested=False)
            elif self._p95_failed(result,job['source_ns']):
                self.arm=dict(trigger='first_actual_final_request_top_only_ray_p95_rejection',
                    source_ns=job['source_ns'],runtime_clock_ns=self.last_clock,
                    actual_final_request_scope=scope,parent_generation=parent_generation,
                    original_reason=result['reason'],original_top_rejections=['ray_p95'],
                    original_ray_abs_p95_m=result['candidates'][0]['rays']['abs_p95_m'],
                    original_ray_abs_p95_limit_m=GATES['allray_abs_p95_m'])
                if (self.previous is not None and self.previous['source_ns']<job['source_ns']
                    and self.previous.get('actual_final_request_scope')==scope):
                    self._select('PRE',self.previous,dict(kind='last_original_checked_job_before_P95_rejection',
                        actual_final_request_scope=scope,task_stop_requested=False))
                else:self.missing['PRE']='no earlier checked job in this exact final request; no fabricated frame'
                self._select('FAIL',snap,dict(kind='actual_original_top_only_ray_p95_rejection',
                    actual_final_request_scope=scope,original_match_accepted=False,task_stop_requested=False))
                self.previous=None
                self._event('P95_original_quality_rejection_armed',source_ns=job['source_ns'],task_stop_requested=False)
            else:self.previous=snap
        return True

    def consider_rejection(self,job,result,current_c,parent_generation):
        # NN/stage veto remains an original rejection but never spends this
        # final-navigation P95 observer's three-group allowance.
        return False

    def _select(self,role,snap,event):
        event=dict(event,reused_structural_whole_message_writer=True,
            event_is_NN_collision=False,quality_rejection_is_control_admission=False,
            same_integer_stamp_source_stage_basis='frozen post-LIO single-owner witness and original checked body/mapper/registered job',
            explicit_producer_commit_token='UNVERIFIED',require_complete_imu=False,
            auxiliary_sampling='actual source bracketing messages only; full LIO IMU consumption is not claimed')
        gaps={}
        for k,rows in snap['aux_at_source_observation'].items():
            stamps=sorted(r['source_ns'] for r in rows)
            gaps[k]=dict(retained_message_count=len(rows),retained_header_interval_ns=None if not stamps else [stamps[0],stamps[-1]],
                consecutive_retained_header_gaps_ns=[b-a for a,b in zip(stamps,stamps[1:])],
                source_arrival_wall_ns=[r['received_wall_ns'] for r in rows],
                missing_role=not bool(rows),frontend_consumption_observed=False)
        event['actual_auxiliary_retained_gap_summary']=gaps
        return super()._select(role,snap,event)

    def receipt(self,final=False,writer_drained=False):
        d=super().receipt(final,writer_drained)
        d.update(schema='known_scene_P95_event_capture_receipt/v1',capture_kind='actual_final_request_P95_quality_rejection',
            reused_structural_group_schema='known_scene_P1_complete_group/v1',event_is_NN_collision=False,
            ready_for_diagnostic_stop=False,task_stop_requested=False,
            sampling_selection_complete=bool(self.arm and 'POST' in self.selected),
            actual_final_request_scope=self.p95_scope,explicit_producer_commit_token='UNVERIFIED',
            complete_original_LIO_IMU_update_claimed=False,raw_adapted_registered_full_payloads_truncated=False)
        return d
