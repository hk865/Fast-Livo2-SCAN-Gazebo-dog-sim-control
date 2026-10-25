"""First sustained active-request P95 observer; no control or admission changes.

Only original worker-owned checked jobs supply source pins. PRE is the last
checked job before the first rejection, and need not be accepted. FAIL is the
second consecutive distinct rejection. The legacy whole-message writer is
reused structurally; saving diagnostics never stops the original mission.
"""
from pathlib import Path
import copy, json, math, threading, time
from event_aware_source_capture import (BoundedSourceFrames, CAPS, GATES, TOPICS,
    PHASES, TRIGGER as NN_TRIGGER, encoded, file_sha, sha)

SCHEMA='go2_P95_quality_source_capture_contract/v2'
TRIGGER=dict(original_predicate='top_candidate_only_ray_p95_failed',
    first_sustained_failure=True,active_mission_phases=dict(PHASES),
    request_suffix='positive_canonical_decimal_route_counter',
    consecutive_distinct_checked_jobs=2,same_scope_and_parent_generation=True,
    complete_previous_jobs=1,pre_relation='last_checked_job_before_first_rejection',
    fail_relation='second_consecutive_distinct_rejection',post_original_checked_job=True)
CAPTURE_STATUSES=('unarmed','armed_waiting_for_post',
    'selection_complete_writes_pending','selection_complete_with_missing_pre',
    'selection_complete_with_write_or_admission_failures',
    'all_groups_saved_diagnostic_only','expired_with_missing_groups',
    'terminal_with_missing_groups')

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
            raise ValueError('exact frozen active-request sustained P95 contract required')
        bindings=contract.get('source_bindings',[])
        if not bindings:raise ValueError('P95 source bindings required')
        for item in bindings:
            if file_sha(item['path'])!=item['sha256']:raise ValueError('frozen P95 source differs')
        # Validate the real contract first. The temporary NN alias starts no
        # writer, and is replaced before any callback or file can observe it.
        actual=copy.deepcopy(contract);alias=copy.deepcopy(contract)
        alias['schema']='go2_P1_association_source_capture_contract/v1'
        alias['diagnostic_trigger']=dict(NN_TRIGGER)
        super().__init__(run,contract=alias,serialize=serialize,deserialize=deserialize,
            start_writer=False,wall_ns=wall_ns)
        self.contract=actual;self.contract_sha256=sha(encoded(actual))
        self.bindings=copy.deepcopy(bindings);self.p95_scope=None
        self.p95_parent_generation=None;self.p95_last_checked_ns=None
        self.p95_first=None;self.p95_pre=None;self.p95_selection_complete=False
        if start_writer:
            self.writer=threading.Thread(target=self._writer,name='bounded_P95_source_writer',daemon=True)
            self.writer.start()

    def _scope(self):
        m=self.mission or {};stage=m.get('stage');request=m.get('request_id')
        phase=PHASES.get(stage)
        if phase is not None and isinstance(request,str):
            prefix=self.run.name+':'+phase+':'
            suffix=request[len(prefix):] if request.startswith(prefix) else ''
            if (suffix and suffix.isascii() and suffix.isdigit()
                and suffix[0]!='0'):
                return (self.run.name,request,stage)
        return None

    def _reset_streak(self,reason,clear_previous=False):
        pending=self.p95_first
        self.p95_first=None;self.p95_pre=None
        if clear_previous:self.previous=None;self.p95_last_checked_ns=None
        if pending is not None:
            self._event('P95_sustained_sequence_reset',reason=reason,
                first_rejection_source_ns=pending['source_ns'],task_stop_requested=False)

    def _context(self,scope,parent_generation):
        if scope!=self.p95_scope or parent_generation!=self.p95_parent_generation:
            self._reset_streak('scope_or_parent_generation_changed',clear_previous=True)
            self.p95_scope=scope;self.p95_parent_generation=parent_generation

    def _clock(self,ns):
        self.last_clock=max(self.last_clock,int(ns))
        if (self.arm and self.capture_needed
            and ns>=self.arm['runtime_clock_ns']+CAPS['event_window_ns']):
            self.expired=True;self.capture_needed=False
            self._reset_streak('sampling_window_expired',clear_previous=True)
            self.inflight_source=None
            self._event('P95_sampling_window_expired_only',runtime_clock_ns=int(ns),task_stop_requested=False)

    def observe_mission(self,text,runtime_clock_ns):
        ok=super().observe_mission(text,runtime_clock_ns)
        if ok:
            with self.lock:
                scope=self._scope()
                if scope!=self.p95_scope:
                    self._context(scope,None);self.inflight_source=None
                if self.terminal:self._reset_streak('original_mission_terminal',clear_previous=True)
        return ok

    def observe_job(self,job,current_c,parent_generation):
        with self.lock:
            scope=self._scope()
            if (scope is None or self.closed or self.expired or self.terminal
                or not self.capture_needed):
                self.inflight_source=None
                return False
            if type(parent_generation) is not int or type(job.get('source_ns')) is not int:
                self._reset_streak('invalid_job_identity',clear_previous=True)
                self.inflight_source=None
                return self.reject(job.get('source_ns'),'P95_invalid_job_identity')
            self._context(scope,parent_generation)
            self.inflight_source=self._snapshot(job,None,current_c,parent_generation)
            self.inflight_source['actual_active_request_scope']=scope
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
            # Never replace an old asynchronous pin with a current source cache.
            if (snap is None or snap['source_ns']!=job.get('source_ns')
                or snap.get('actual_active_request_scope')!=scope
                or snap['parent_generation']!=parent_generation
                or self.p95_parent_generation!=parent_generation
                or any(snap['source_messages'].get(k) is not job.get('source_messages',{}).get(k)
                    for k in ('raw','adapted','registered','body','mapper'))
                or result.get('source_ns')!=job.get('source_ns')):
                self._reset_streak('missing_or_changed_exact_inflight_pin',clear_previous=True)
                return self.reject(job.get('source_ns'),'P95_missing_exact_inflight_source_pin')
            ns=job['source_ns'];failed=self._p95_failed(result,ns)
            if self.p95_last_checked_ns is not None and ns<=self.p95_last_checked_ns:
                if ns<self.p95_last_checked_ns:
                    self._reset_streak('out_of_order_checked_job',clear_previous=True)
                elif not failed:
                    self._reset_streak('duplicate_stamp_non_P95_result')
                self._event('P95_duplicate_or_out_of_order_checked_job_ignored',
                    source_ns=ns,task_stop_requested=False)
                return False
            snap['result']=dict(result);self.p95_last_checked_ns=ns
            if self.arm:
                if (ns>self.arm['source_ns'] and not self.p95_selection_complete
                    and tuple(self.arm['actual_active_request_scope'])==scope):
                    self._select('POST',snap,dict(kind='first_original_checked_job_after_sustained_P95_rejection',
                        original_match_accepted=result.get('accepted'),actual_active_request_scope=scope,
                        task_stop_requested=False))
                    # Selection attempts have finished even if a bounded queue
                    # rejected POST. Receipt distinguishes attempts from saving.
                    self.p95_selection_complete=True;self.capture_needed=False
                    self.previous=None
                    self._event('P95_sampling_selection_complete_only',task_stop_requested=False)
            elif failed:
                current=dict(source_ns=ns,runtime_clock_ns=self.last_clock,
                    parent_generation=parent_generation,original_reason=result['reason'],
                    original_top_rejections=['ray_p95'],
                    original_ray_abs_p95_m=result['candidates'][0]['rays']['abs_p95_m'],
                    original_ray_abs_p95_limit_m=GATES['allray_abs_p95_m'])
                if self.p95_first is None:
                    self.p95_first=current;self.p95_pre=self.previous
                    self._event('P95_first_rejection_waiting_for_distinct_confirmation',
                        source_ns=ns,task_stop_requested=False)
                else:
                    self.arm=dict(trigger='first_active_request_sustained_top_only_ray_p95_rejection',
                        source_ns=ns,runtime_clock_ns=self.last_clock,
                        actual_active_request_scope=scope,parent_generation=parent_generation,
                        consecutive_distinct_checked_jobs=2,
                        sustained_rejections=[dict(self.p95_first),current],
                        original_reason=result['reason'],original_top_rejections=['ray_p95'],
                        original_ray_abs_p95_m=current['original_ray_abs_p95_m'],
                        original_ray_abs_p95_limit_m=GATES['allray_abs_p95_m'])
                    pre=self.p95_pre
                    if (pre is not None and pre['source_ns']<self.p95_first['source_ns']
                        and pre.get('actual_active_request_scope')==scope
                        and pre['parent_generation']==parent_generation):
                        self._select('PRE',pre,dict(kind='last_original_checked_job_before_first_P95_rejection',
                            original_match_accepted=(pre.get('result') or {}).get('accepted'),
                            clean_or_accepted_PRE_claimed=False,actual_active_request_scope=scope,
                            task_stop_requested=False))
                    else:self.missing['PRE']='no earlier checked job in this exact active request and generation; no fabricated frame'
                    self._select('FAIL',snap,dict(kind='second_consecutive_distinct_original_top_only_ray_p95_rejection',
                        sustained_rejections=list(self.arm['sustained_rejections']),
                        actual_active_request_scope=scope,original_match_accepted=False,task_stop_requested=False))
                    self.p95_first=None;self.p95_pre=None;self.previous=None
                    self._event('P95_original_sustained_quality_rejection_armed',source_ns=ns,task_stop_requested=False)
            else:self._reset_streak('other_original_checked_result')
            if self.arm is None:self.previous=snap
        return True

    def consider_rejection(self,job,result,current_c,parent_generation):
        # Original exact-association vetoes interrupt continuity. They are not
        # checked matcher results and cannot supply PRE, FAIL or POST here.
        with self.lock:
            scope=self._scope()
            self.inflight_source=None
            if self.closed or self.expired or self.terminal or not self.capture_needed:return False
            if scope!=self.p95_scope or parent_generation!=self.p95_parent_generation:
                self._context(scope,parent_generation)
            self._reset_streak('original_exact_association_rejection')
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
        with self.lock:
            failures={k:v for k,v in d['missing'].items() if k not in d['saved']
                and not v.startswith('no earlier checked job in this exact active request')
                and v!='not saved in bounded P1 window'}
            if all(k in d['saved'] for k in ('PRE','FAIL','POST')):
                status='all_groups_saved_diagnostic_only'
            elif self.p95_selection_complete and (self.write_failed or failures):
                status='selection_complete_with_write_or_admission_failures'
            elif self.arm is None:status='unarmed'
            elif self.expired:status='expired_with_missing_groups'
            elif self.terminal or (final and self.arm):status='terminal_with_missing_groups'
            elif self.p95_selection_complete and 'PRE' not in d['selected']:
                status='selection_complete_with_missing_pre'
            elif self.p95_selection_complete:status='selection_complete_writes_pending'
            else:status='armed_waiting_for_post'
            d.update(schema='known_scene_P95_event_capture_receipt/v2',
                capture_kind='first_active_request_sustained_P95_quality_rejection',capture_status=status,
                reused_structural_group_schema='known_scene_P1_complete_group/v1',event_is_NN_collision=False,
                ready_for_diagnostic_stop=False,task_stop_requested=False,
                sampling_selection_complete=self.p95_selection_complete,
                POST_selection_enqueued='POST' in d['selected'],
                sampling_selection_complete_means='PRE/FAIL selection and first eligible POST selection attempt finished; not a saving or admission PASS',
                actual_active_request_scope=None if self.arm is None else self.arm['actual_active_request_scope'],
                capture_write_or_admission_failures=failures,
                current_active_request_scope=self.p95_scope,current_parent_generation=self.p95_parent_generation,
                pending_first_rejection=None if self.p95_first is None else dict(self.p95_first),
                explicit_producer_commit_token='UNVERIFIED',
                complete_original_LIO_IMU_update_claimed=False,raw_adapted_registered_full_payloads_truncated=False)
        return d
