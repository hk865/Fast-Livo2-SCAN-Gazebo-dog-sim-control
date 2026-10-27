#!/usr/bin/env python3
"""One actual-SLAM R85 task owner. Creates no route/arrival or parking PASS.

The bridge's sole anchor publisher name is deliberately preserved. Only root
may launch this file; importing its pure helpers starts no ROS/model/process.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import importlib
import json
import math
import sys
from pathlib import Path
import time

HERE=Path(__file__).resolve().parent
if str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'src') not in sys.path:sys.path.append(str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'src'))
FIXED_T=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-.3],[0.,0.,0.,1.]]
RESULT_SCHEMA='r85_pct_scan_student_task_result/v1'


def observe_only(profile):
    value=profile.get('diagnostic_observe_only')
    return isinstance(value,dict) and value.get('enabled') is True


def full_evidence_observation(profile):
    value=profile.get('diagnostic_observe_only')
    return observe_only(profile) and value.get('full_evidence_after_endpoint') is True


def diagnostic_evidence_terminal(run,request_id,task,endpoint,parking,map_result,dynamic,capture,ns,progress):
    """Summarize original successful predicates without granting formal acceptance."""
    run=Path(run).resolve()
    if (task.get('kind')!='f1_to_f3' or task.get('dynamic_required')is not True
            or type(ns)is not int or ns<0):raise ValueError('full_evidence_requires_original_full_task')
    docs=((endpoint,'r85_diagnostic_endpoint_result/v1'),
          (parking,'r85_actual_fixed5_parking_result/v1'),
          (map_result,'r85_actual_current_rgb_save/v1'),
          (dynamic,'r85_dynamic_task_result/v1'))
    for value,schema in docs:
        if (not isinstance(value,dict) or value.get('schema')!=schema
                or value.get('run_id')!=run.name or value.get('request_id')!=request_id):
            raise ValueError('full_evidence_missing_or_foreign_original_receipt:'+schema)
        canonical(value)
    if (endpoint.get('diagnostic_reached_endpoint')is not True
            or endpoint.get('original_control_band_used')is not True
            or any(value.get('passed')is not True for value in (parking,map_result,dynamic))
            or dynamic.get('state')!='passed'):
        raise ValueError('full_evidence_original_predicate_not_passed')
    if not isinstance(capture,dict) or capture.get('schema')!='r85_dynamic_owner_verified_sources_binding/v1':
        raise ValueError('full_evidence_original_same_read_capture_missing')
    path=Path(capture['path'])
    if (path!=run/'r85_dynamic_owner_verified_sources.json' or path.is_symlink() or not path.is_file()
            or type(capture.get('bytes'))is not int or not 0<capture['bytes']<=24*1024**2
            or path.stat().st_size!=capture['bytes']):
        raise ValueError('full_evidence_original_same_read_capture_path_or_size')
    raw=path.read_bytes();capsule=json.loads(raw)
    if (hashlib.sha256(raw).hexdigest()!=capture.get('sha256')
            or capsule.get('schema')!='r85_dynamic_owner_verified_sources/v1'
            or capsule.get('run_id')!=run.name or capsule.get('request_id')!=request_id
            or capsule.get('owner_result_canonical_sha256')!=hashlib.sha256(canonical(dynamic).encode()).hexdigest()):
        raise ValueError('full_evidence_original_same_read_capture_binding')
    return dict(schema=RESULT_SCHEMA,run_id=run.name,request_id=request_id,kind=task['kind'],
        state='diagnostic_evidence_complete',diagnostic_full_evidence_observed=True,
        diagnostic_reached_endpoint=True,terminal=True,passed=False,failure_reason=None,
        parking_pass=False,dynamic_pass=False,current_rgb_map_verified=False,
        formal_full_route_pass=False,formal_acceptance_pass=False,navigation_PASS=False,
        full_F3_PASS=False,original46_PASS=False,short_chain_validation_only=False,
        actual_original_predicates=dict(endpoint_control_band_dwell=True,fixed5_parking=True,
            current_run_RGB_map=True,dynamic_protocol=True,dynamic_same_read_capture=True),
        endpoint_milestone=copy.deepcopy(endpoint),parking_result=copy.deepcopy(parking),
        current_rgb_map_result=copy.deepcopy(map_result),dynamic_result=copy.deepcopy(dynamic),
        dynamic_evidence_capture=copy.deepcopy(capture),dynamic_required=True,current_rgb_map_required=True,
        original_predicates_and_thresholds_unchanged=True,diagnostic_result_is_not_formal_acceptance=True,
        known_map_assistance=True,navigation_ground_truth_used=False,simulation_only=True,
        completed_sim_ns=ns,progress=progress,timing_scope=task['timing'])


def diagnostic_business_failure(reason):
    """Exact known business reasons only; unknown faults remain terminating."""
    return type(reason)is str and reason in {
        'less_than_05m_forward_progress_in_90_sim_seconds',
        'actual_navigation_failed:当前航点超过仿真时间限制，停车',
        'actual_navigation_failed:定位等待期间原区域超过仿真时间限制，停车',
        'locked_dynamic_scope_does_not_match_current_owner_reference',
        'actual_reference_scope_changed_during_dynamic_protocol',
        'encounter_trigger_window_missed_no_chase_or_late_forced_crossing',
    }


def observe_business(run,profile,gate,reason,evidence):
    if not observe_only(profile):return False
    from diagnostic_observation import record
    record(run,profile,gate,reason,evidence)
    return True


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
def xyz(value):
    if (not isinstance(value,(list,tuple)) or len(value)!=3 or any(
            type(x)not in(int,float) or not math.isfinite(x) for x in value)):
        raise ValueError('finite_actual_xyz_required')
    return list(map(float,value))
def atomic(path,value):
    path=Path(path);raw=(json.dumps(value,indent=2,allow_nan=False)+'\n').encode()
    temporary=path.with_name(path.name+'.r85.tmp');temporary.write_bytes(raw);temporary.replace(path)
    return hashlib.sha256(raw).hexdigest()
def input_digest(scope,path):
    path=Path(path).resolve();actual=sha(path)
    return actual


def completed_history_module(profile):
    """The new identity contract is exact-bool opt-in and grants no PASS."""
    value=profile.get('r85_dynamic_event_contract')if isinstance(profile,dict)else None
    if not isinstance(value,dict)or value.get('completed_same_geometry_preservation')is not True:return None
    return importlib.import_module('r85_dynamic_completed_scope')


class CompletedHistoryPending(ValueError):
    """No history acceptance while an owned asynchronous source is incomplete."""


def completed_history_owner_binding(run,task,scope,request_id,actual,anchor,expected,helper):
    """O(1) early identity reads only; full physical/chain checks remain late."""
    import base64
    require=helper.require;run=Path(run).resolve()
    input_digest(scope,HERE/'r85_dynamic_completed_scope.py')
    keys={'run_id','request_id','reference_id','reference_sha256','epoch',
        'localization_generation','source_route_sha256'}
    def binding(value):
        require(isinstance(value,dict)and set(value)==keys,'malformed_completed_navigation_binding')
        require(value['run_id']==run.name and value['request_id']==request_id
            and value['source_route_sha256']==task['source_typed_route_sha256'],
            'foreign_completed_navigation_binding')
        require(all(type(value[k])is int and value[k]>=1 for k in ('epoch','localization_generation')),
            'invalid_completed_navigation_clock')
        return value
    def read(path):
        require(path.is_file()and not path.is_symlink()and path.stat().st_size<=16*1024**2,
            'missing_or_oversize_completed_source')
        raw=path.read_bytes();doc=json.loads(raw);canonical(doc)
        return doc,hashlib.sha256(raw).hexdigest()
    def journal_line(receipt):
        require(isinstance(receipt,dict)and Path(receipt['path'])==run/'r85_dynamic_journal.jsonl'
            and type(receipt['source_offset'])is int and receipt['source_offset']>=0
            and type(receipt['source_length'])is int and 0<receipt['source_length']<=2*1024**2,
            'completed_history_journal_receipt_path_or_budget')
        path=run/'r85_dynamic_journal.jsonl';require(not path.is_symlink(),'foreign_completed_journal_symlink')
        if not path.is_file():raise CompletedHistoryPending('completed_history_real_journal_not_written')
        with path.open('rb')as stream:
            stream.seek(receipt['source_offset']);raw=stream.read(receipt['source_length'])
        if len(raw)<receipt['source_length']:raise CompletedHistoryPending('completed_history_real_journal_line_incomplete')
        require(raw.endswith(b'\n')and hashlib.sha256(raw).hexdigest()==receipt['source_line_sha256'],
            'completed_history_real_journal_line_SHA_differs')
        row=json.loads(raw);canonical(row)
        require(row.get('schema')=='r85_dynamic_obstacle_record/v1'
            and row.get('kind')=='completed_history_scope_preserved'and row.get('binding')==anchor,
            'completed_history_journal_anchor_differs')
        return row
    require(actual.get('completed_history_scope_policy')==helper.POLICY,
        'completed_history_policy_missing_or_foreign')
    require(actual.get('diagnostic_protocol_cancelled')is False,
        'cancelled_dynamic_cannot_preserve_completed_history')
    current=binding(actual.get('current_navigation_binding'));binding(anchor)
    qualification=actual.get('completed_history_qualification')
    require(isinstance(qualification,dict)
        and qualification.get('schema')=='r85_completed_history_retention_qualification/v1'
        and qualification.get('qualified_for_history_retention_only')is True
        and qualification.get('passed')is False and qualification.get('dynamic_PASS')is False
        and qualification.get('navigation_PASS')is False
        and qualification.get('original_single_scope_dynamic_PASS')is False
        and qualification.get('identity_contract_changed')is True,
        'invalid_completed_history_qualification')
    contract,contract_sha=read(run/'r85_dynamic_task_contract.json')
    require(contract.get('completed_history_scope_policy')==helper.POLICY
        and contract.get('binding')==anchor and contract.get('scope_locked_at_trigger')is True
        and contract.get('trigger_sim_ns')==actual.get('started_sim_ns'),
        'completed_history_immutable_anchor_differs')
    require(Path(contract['reference_file'])==run/helper.REF
        and Path(contract['registration_file'])==run/helper.REG
        and contract['trigger']['reference_file']==contract['reference_file'],
        'completed_history_trigger_snapshot_path_differs')
    reference,reference_sha=read(run/helper.REF);registration,registration_sha=read(run/helper.REG)
    require(reference_sha==anchor['reference_sha256']==contract['reference_sha256']
        and registration_sha==contract['registration_sha256'],
        'completed_history_trigger_snapshot_binding_differs')
    helper._validate_current_pair(run,anchor,reference,registration,anchor,reference_sha,registration_sha)
    original=qualification.get('original_physical_verifier_result',{})
    require(original.get('run_id')==run.name and original.get('request_id')==request_id
        and original.get('passed')is True and original.get('state')=='passed'
        and original.get('verification',{}).get('allowed')is True
        and original['verification'].get('binding')==anchor,
        'completed_history_original_qualification_source_differs')
    sources=original['source_receipt']
    require(sources['contract']['path']==str(run/'r85_dynamic_task_contract.json')
        and sources['contract']['sha256']==contract_sha,'completed_history_contract_receipt_differs')
    state_raw=base64.b64decode(qualification['original_runtime_state_raw_base64'],validate=True)
    require(hashlib.sha256(state_raw).hexdigest()==sources['runtime_state']['sha256']
        and len(state_raw)==sources['runtime_state']['bytes']
        and sources['runtime_state']['path']==str(run/'r85_dynamic_obstacle_state.json')
        and json.loads(state_raw).get('binding')==anchor,
        'completed_history_qualification_same_read_state_differs')
    first=journal_line(actual.get('completed_history_first_journal_receipt'))
    latest=journal_line(actual.get('completed_history_latest_journal_receipt'))
    qsha=hashlib.sha256(canonical(qualification).encode()).hexdigest()
    require(first['data'].get('qualification')==qualification
        and first['data'].get('previous_navigation_binding')==anchor,
        'completed_history_qualification_not_original_journal_source')
    for row in (first,latest):
        data=row['data'];new=binding(data.get('current_navigation_binding'));previous=binding(data.get('previous_navigation_binding'))
        source=data.get('handoff_source_ns');pose=data.get('source_actual_pose_stamp_ns')
        received=data.get('mission_received_sim_ns');sim=actual.get('sim_ns');message=data.get('mission_message',{})
        require(data.get('qualification_sha256')==qsha and new['epoch']>previous['epoch']
            and new['localization_generation']>=previous['localization_generation']
            and all(type(n)is int for n in (source,pose,received,sim,row.get('sim_ns')))
            and 0<=pose<=source<=received<=row['sim_ns']<=sim
            and received-pose<=300000000 and received-source<=300000000
            and qualification['stable_witness']['maximum_witness_source_ns']<source,
            'completed_history_handoff_before_complete_or_clock_reversed')
        ref=message.get('r85_reference',{})
        require(message.get('schema')=='r85_actual_single_task_mission_state/v1'
            and message.get('run_id')==run.name and message.get('current_request')==request_id
            and message.get('stage')=='navigating'and message.get('navigation_ground_truth_used')is False
            and message.get('ros_sim_time_ns')==source and message.get('source_actual_pose_stamp_ns')==pose
            and all(type(ref.get(k))is type(new[k])and ref[k]==new[k]
                for k in ('reference_id','epoch','localization_generation','source_route_sha256'))
            and ref.get('global_reference_sha256')==new['reference_sha256']
            and Path(ref['global_reference_file'])==run/'r85_global_reference.json',
            'completed_history_original_mission_source_differs')
        helper._validate_current_pair(run,anchor,data['current_reference_document'],
            data['current_registration_document'],new,data['current_reference_sha256'],data['current_registration_sha256'])
        require(hashlib.sha256(data['current_reference_raw_utf8'].encode()).hexdigest()==data['current_reference_sha256']
            and hashlib.sha256(data['current_registration_raw_utf8'].encode()).hexdigest()==data['current_registration_sha256']
            and canonical(json.loads(data['current_reference_raw_utf8']))==canonical(data['current_reference_document'])
            and canonical(json.loads(data['current_registration_raw_utf8']))==canonical(data['current_registration_document']),
            'completed_history_handoff_original_raw_source_differs')
        helper.same_geometry(reference,registration,data['current_reference_document'],data['current_registration_document'])
    require(latest['data']['current_navigation_binding']==current
        and latest['data']['handoff_source_ns']>=first['data']['handoff_source_ns'],
        'completed_history_current_binding_not_journal_source')
    real_reference,real_reference_sha=read(run/'r85_global_reference.json')
    real_registration,real_registration_sha=read(run/'r85_reference_registration.json')
    helper.same_geometry(reference,registration,real_reference,real_registration)
    helper._validate_current_pair(run,anchor,real_reference,real_registration,expected,real_reference_sha,real_registration_sha)
    if current!=expected:
        history=run/'r85_reference_scope_history.jsonl'
        require(history.is_file()and history.stat().st_size<=2*1024**2,'completed_history_unknown_pending_scope')
        rows=[json.loads(line)for line in history.read_bytes().splitlines(keepends=True)if line.endswith(b'\n')]
        require(current['epoch']<expected['epoch']and current['localization_generation']<=expected['localization_generation']
            and any(row.get('reference_id')==current['reference_id']
                and row.get('reference_file_sha256')==current['reference_sha256']
                and type(row.get('planner_epoch'))is int and row['planner_epoch']==current['epoch']
                and type(row.get('localization_generation'))is int
                and row['localization_generation']==current['localization_generation']for row in rows),
            'completed_history_foreign_or_future_pending_scope')
        raise CompletedHistoryPending('completed_history_waiting_real_current_navigation_handoff')
    require(real_reference==latest['data']['current_reference_document']
        and real_registration==latest['data']['current_registration_document']
        and real_registration_sha==latest['data']['current_registration_sha256'],
        'completed_history_current_real_reference_changed')
    return current


def dynamic_fatal_decision(run,task,scope,request_id,current_reference,reference_sha,runtime_source=None,*,completed_wait=None):
    """Early full-task failure observation; never an Actor or parking PASS.

    The state schema has no producer signature. The receipt therefore binds the
    owned run file to its frozen task/runtime source, without inventing one.
    Known pretrigger old-scope waiting is allowed while park ACKs drain.
    """
    decision=dict(schema='r85_dynamic_early_failure_check/v1',fatal=False,state='not_required',
        failure_reason=None,navigation_PASS=False,dynamic_PASS=False)
    if task.get('kind')!='f1_to_f3' or task.get('dynamic_required')is not True:return decision
    run=Path(run).resolve();path=run/'r85_dynamic_obstacle_state.json'
    if not path.exists():decision['state']='waiting_dynamic_state';return decision
    actual=None
    try:
        if path.stat().st_size>2*1024**2:raise ValueError('dynamic_state_exceeds_original_line_budget')
        raw=path.read_bytes();actual=json.loads(raw);canonical(actual)
        decision.update(state_file=str(path),state_sha256=hashlib.sha256(raw).hexdigest(),state_document=actual,
            original_dynamic_failure=actual.get('failure')if isinstance(actual,dict)else None,
            producer_identity_basis='owned_run_state_and_frozen_task/runtime_sources; state_has_no_producer_signature')
        if (not isinstance(actual,dict) or actual.get('schema')!='r85_dynamic_obstacle_state/v1'
                or actual.get('run_id')!=run.name):raise ValueError('foreign_or_malformed_dynamic_state_run')
        if request_id!=run.name+':r85:f1_to_f3:1':raise ValueError('owner_full_request_identity_differs')
        if (task.get('schema')!='r85_pct_scan_student_task/v1' or task.get('run_id')!=run.name
                or scope.get('run_dir')!=str(run) or scope.get('profile',{}).get('r85_task',{}).get('kind')!='f1_to_f3'):
            raise ValueError('dynamic_fatal_requires_same_run_frozen_full_task')
        task_path=run/'r85_task_contract.json';task_sha=input_digest(scope,task_path)
        if canonical(json.loads(task_path.read_text()))!=canonical(task):raise ValueError('actual_frozen_task_differs')
        source=Path(runtime_source or HERE/'r85_dynamic_obstacle_runtime.py').resolve()
        source_sha=input_digest(scope,source)
        decision.update(task_file=str(task_path),task_sha256=task_sha,
            runtime_source_file=str(source),runtime_source_sha256=source_sha)
        if (actual.get('navigation_ground_truth_used')is not False or actual.get('privileged_actor_pose')is not True
                or actual.get('robot_pose_service_called')is not False):raise ValueError('dynamic_state_privilege_boundary_differs')
        failure=actual.get('failure')
        if failure is not None and (not isinstance(failure,str) or not failure):raise ValueError('malformed_dynamic_failure_reason')
        binding=actual.get('binding');started=actual.get('started_sim_ns')
        if started is not None and (type(started)is not int or started<0):raise ValueError('malformed_dynamic_trigger_clock')
        if binding is None:
            if started is not None:raise ValueError('triggered_dynamic_state_without_binding')
            decision.update(state='waiting_dynamic_binding',binding_identity='unbound_owned_full_task')
        else:
            if not isinstance(binding,dict):raise ValueError('malformed_dynamic_binding')
            if binding.get('run_id')!=run.name or binding.get('request_id')!=request_id:
                raise ValueError('foreign_dynamic_binding_run_or_request')
            if binding.get('source_route_sha256')!=task['source_typed_route_sha256']:
                raise ValueError('foreign_dynamic_full_typed_source')
            if any(type(binding.get(k))is not int or binding[k]<1 for k in ('epoch','localization_generation')):
                raise ValueError('malformed_dynamic_epoch_or_generation')
            if not isinstance(current_reference,dict):raise ValueError('bound_dynamic_without_owner_reference')
            expected=dict(run_id=run.name,request_id=request_id,reference_id=current_reference['reference_id'],
                reference_sha256=reference_sha,epoch=current_reference['epoch'],
                localization_generation=current_reference['localization_generation'],
                source_route_sha256=task['source_typed_route_sha256'])
            navigation_binding=binding
            from r85_dynamic_obstacle_runtime import navigation_segments_enabled,SEGMENT_WINDOW,coordinate_handoffs_enabled,navigation_segment_schema,coordinate_owner_binding
            segmented=navigation_segments_enabled(scope.get('profile',{}))
            if segmented:
                if actual.get('event_lifetime_contract')!=navigation_segment_schema(scope.get('profile',{})):raise ValueError('navigation_segment_state_contract_differs')
                if actual.get('diagnostic_protocol_cancelled')is not False:raise ValueError('cancelled_navigation_event_cannot_wait_or_resume')
                boundaries=actual.get('navigation_boundary_sequences')
                nav=actual.get('current_navigation_binding')
                if (type(boundaries)is not list or len(boundaries)>(2 if coordinate_handoffs_enabled(scope.get('profile',{})) else 1) or any(type(q)is not int or q<1 for q in boundaries)
                        or type(nav)is not dict or set(nav)!=set(binding)
                        or any(type(nav.get(k))is not type(binding[k]) or nav[k]!=binding[k] for k in ('run_id','request_id','source_route_sha256'))):
                    raise ValueError('malformed_navigation_segment_owner_binding')
                if boundaries:
                    if not (type(nav.get('epoch'))is int and nav['epoch']>binding['epoch']
                            and type(nav.get('localization_generation'))is int and nav['localization_generation']>binding['localization_generation']):
                        raise ValueError('navigation_segment_owner_identity_did_not_advance')
                    navigation_binding=nav
                elif nav!=binding:raise ValueError('navigation_segment_missing_real_boundary')
                if coordinate_handoffs_enabled(scope.get('profile',{})):
                    navigation_binding=coordinate_owner_binding(run,binding,actual)
            completed_helper=completed_history_module(scope.get('profile',{}))
            if completed_helper is not None and (actual.get('current_navigation_binding')is not None
                    or actual.get('completed_history_qualification')is not None):
                navigation_binding=completed_history_owner_binding(run,task,scope,request_id,actual,
                    binding,expected,completed_helper)
                if completed_wait is not None:completed_wait.clear()
                decision.update(binding_identity='qualified_completed_event_anchor_with_current_navigation_scope',
                    immutable_event_anchor=binding,current_navigation_binding=navigation_binding,
                    completed_history_scope_policy=completed_helper.POLICY,history_retention_only=True)
            current=all(type(navigation_binding.get(k))is type(v) and navigation_binding[k]==v for k,v in expected.items())
            decision.update(current_reference_binding=expected,current_scope_matches=current)
            if not current:
                history=run/'r85_reference_scope_history.jsonl'
                if not history.is_file() or history.stat().st_size>2*1024**2:raise ValueError('dynamic_scope_not_in_owned_reference_history')
                rows=[json.loads(line)for line in history.read_bytes().splitlines(keepends=True)if line.endswith(b'\n')]
                known=[r for r in rows if r.get('reference_id')==binding.get('reference_id')
                    and r.get('reference_file_sha256')==binding.get('reference_sha256')
                    and type(r.get('planner_epoch'))is int and r['planner_epoch']==binding['epoch']
                    and type(r.get('localization_generation'))is int and r['localization_generation']==binding['localization_generation']]
                if (not known or binding['epoch']>=expected['epoch']
                        or binding['localization_generation']>expected['localization_generation']):
                    raise ValueError('foreign_or_future_dynamic_reference_scope')
                decision.update(binding_identity='known_prior_owner_reference_scope',known_prior_scope_record=known[-1],
                    reference_scope_history_file=str(history),reference_scope_history_sha256=sha(history))
                if started is not None and failure is None:
                    from r85_dynamic_obstacle_runtime import event_window_enabled, completed_event_window
                    if (segmented and actual.get('event_state')=='active'
                            and ((coordinate_handoffs_enabled(scope.get('profile',{})) and len(boundaries)<2)
                                 or (not boundaries and actual.get('dynamic_proof_seen')is False))
                            and expected['epoch']>binding['epoch']
                            and expected['localization_generation']>binding['localization_generation']):
                        # This grants no new binding or motion. The actor runtime
                        # freezes until its original ready gates validate the edge.
                        decision.update(state='waiting_pre_stop_navigation_segment_boundary',
                            event_lifetime_contract=navigation_segment_schema(scope.get('profile',{})),physical_verification_pending=True)
                    elif event_window_enabled(scope.get('profile',{})) and completed_event_window(actual) is not None:
                        decision.update(state='completed_event_pending_verification',
                            binding_identity='completed_event_anchor_separate_from_later_navigation',
                            event_lifetime_contract=actual.get('event_lifetime_contract'),
                            whole_task_single_scope_claimed=False,physical_verification_pending=True)
                    else:failure='locked_dynamic_scope_does_not_match_current_owner_reference'
                else:decision['state']='waiting_pretrigger_scope_reselection'
            else:
                decision.update(state='dynamic_runtime_active',binding_identity='current_owner_reference_scope')
                if navigation_binding is not binding:
                    decision['binding_identity']=('world_event_anchor_with_current_navigation_segment' if segmented else 'qualified_completed_event_anchor_with_current_navigation_scope')
        if failure is not None:
            decision.update(fatal=True,state='failed',failure_reason=failure)
            # Only this owned, validated prior-reference mismatch is business
            # evidence. Identity/format/source rejections below remain hard.
            if (observe_only(scope.get('profile',{}))
                    and decision.get('binding_identity')=='known_prior_owner_reference_scope'
                    and failure=='locked_dynamic_scope_does_not_match_current_owner_reference'):
                decision.update(fatal=False,state='diagnostic_protocol_cancelled',
                    observed_business_rejection=True,
                    dynamic_cancellation_ack_observed=actual.get('diagnostic_protocol_cancelled') is True)
    except CompletedHistoryPending as error:
        now=time.monotonic();clock=actual.get('monotonic_wall')if isinstance(actual,dict)else None
        waits={}if completed_wait is None else completed_wait
        began=waits.setdefault(reference_sha,now)
        if (actual.get('failure')is None and type(clock)in (int,float)and math.isfinite(clock)
                and 0<=now-clock<=.3 and 0<=now-began<=.3):
            decision.update(fatal=False,state='waiting_completed_history_navigation_handoff',failure_reason=None,
                pending_reason=str(error),current_scope_matches=False,dynamic_ready=False,
                history_retention_only=True)
        else:
            decision.update(fatal=True,state='failed',identity_rejected=True,
                failure_reason=actual.get('failure')or'completed_history_handoff_exceeded_original_source_age')
    except (OSError,ValueError,TypeError,KeyError,AttributeError)as error:
        decision.update(fatal=True,state='failed',identity_rejected=True,
            failure_reason='dynamic_state_identity_rejected:'+type(error).__name__+':'+str(error))
    return decision


def task_points(source,task,prefix_selector=None):
    if source.get('T_navigation_world')!=FIXED_T:
        raise ValueError('typed_T_differs_from_fixed_actual_canonical_axes_no_mapping_allowed')
    if task['kind']=='business_chain_prefix':
        if task.get('prefix_length_m')!=2.0:raise ValueError('short_requires_explicit_2m_business_chain')
        if prefix_selector is None:
            prefix_selector=importlib.import_module('r85_geometry_admission').deterministic_business_chain_prefix
        selected=prefix_selector(source)
        if selected.get('reference_scope')!='business_chain_prefix' or selected.get('prefix_length_m')!=2.0:
            raise ValueError('short_prefix_scope_or_length_mismatch')
    elif task['kind']=='f1_to_f3':
        selected=dict(reference_scope='full_route',body_points=source['body_points'],ground_points=source['ground_points'])
    else:raise ValueError('unknown_R85_task_kind')
    selected=copy.deepcopy(selected)
    body=[xyz(p)for p in selected['body_points']];ground=[xyz(p)for p in selected['ground_points']]
    if not 2<=len(body)<=4096 or len(ground)!=len(body):raise ValueError('bounded_typed_route_count')
    if any(math.dist(a,b)<=1e-6 for a,b in zip(body[:-1],body[1:])):raise ValueError('duplicate_typed_route_edge')
    if any(max(abs(b[i]-g[i]-(.32 if i==2 else 0.))for i in range(3))>1e-8 for b,g in zip(body,ground)):
        raise ValueError('true_ground_plus032_required')
    selected['body_points'],selected['ground_points']=body,ground
    return selected


def goal_binding(profile,task,origin,selected):
    """New task centers, unchanged original flat-region geometry/tolerances."""
    goal=copy.deepcopy(profile['original_scenario']['route_goals']['navigation_f1_f3'][-1])
    offset=xyz(goal['center']);origin=xyz(origin)
    original_F3=[origin[i]+offset[i]for i in range(3)]
    if task['kind']=='business_chain_prefix':
        goal['center']=xyz(selected['body_points'][-1]);goal['goal_id']='r85_business_chain_prefix_2m'
        goal['route_surface_id']='floor_1'
    else:goal['center']=original_F3
    goal['timeout_sim_s']=float(task['timing']['whole_task_maximum_sim_s'])
    arrival=goal['arrival'];band=arrival['control_band']
    identity=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    if arrival.get('type')!='disc_prism' or band.get('type')!='disc_prism' or arrival.get('axes')!=identity or band.get('axes')!=identity:
        raise ValueError('original_flat_F3_region_geometry_required')
    residual=[selected['body_points'][-1][i]-goal['center'][i]for i in range(3)]
    inside=(math.hypot(*residual[:2])<=band['radius_m'] and abs(residual[2])<=band['height_half_span_m'])
    if not inside:raise ValueError('typed_endpoint_outside_unchanged_task_control_band')
    return goal,dict(original_F3_goal_body_xyz=original_F3,task_goal_body_xyz=goal['center'],
        planner_body_endpoint=selected['body_points'][-1],planner_endpoint_minus_task_goal=residual,
        endpoint_inside_original_control_band=True,arrival_tolerances_changed=False,
        old_per_region_timeout_sim_s=profile['original_scenario']['route_goals']['navigation_f1_f3'][-1]['timeout_sim_s'],
        explicit_single_goal_timeout_sim_s=goal['timeout_sim_s'])


def projected_progress(points,position):
    p=xyz(position);best_distance=math.inf;best_s=0.;s=0.
    for a,b in zip(points[:-1],points[1:]):
        delta=[b[i]-a[i]for i in range(3)];length=math.sqrt(sum(x*x for x in delta))
        if length<=1e-6:raise ValueError('degenerate_progress_route')
        fraction=max(0.,min(1.,sum((p[i]-a[i])*delta[i]for i in range(3))/(length*length)))
        distance=sum((p[i]-a[i]-fraction*delta[i])**2 for i in range(3))
        candidate=s+fraction*length
        if distance<best_distance-1e-12 or abs(distance-best_distance)<=1e-12 and candidate<best_s:
            best_distance,best_s=distance,candidate
        s+=length
    return best_s


class TaskProgressWatchdog:
    def __init__(self,points,start_ns,maximum_sim_s,window_sim_s=90.,minimum_forward_m=.5,
            *,diagnostic_observe_only=False,on_business_rejection=None):
        self.points=[xyz(x)for x in points];self.start_ns=int(start_ns);self.maximum_ns=round(maximum_sim_s*1e9)
        self.window_ns=round(window_sim_s*1e9);self.minimum=float(minimum_forward_m)
        if not 0<self.maximum_ns<=1500_000_000_000 or self.window_ns!=90_000_000_000 or self.minimum!=.5:
            raise ValueError('explicit_finite_original_cap_and_90s_05m_scope_required')
        # The control clock and the producer header are distinct clocks. A
        # fresh source header can legitimately precede the task start clock.
        self.last_control_clock_ns=self.start_ns;self.last_source_header_ns=None
        self.last_source_callback_clock_ns=None;self.last_source_header_age_ns=None
        self.window_start_ns=self.start_ns;self.highwater=0.;self.window_start_progress=0.
        self.arrived=False;self.failed=None
        self.observe_only=diagnostic_observe_only is True
        self.on_business_rejection=on_business_rejection
        self.observed_business_rejections=[]
    def latch(self,reason):
        if self.failed is None:self.failed=reason
        return self.failed
    def deadline(self,now_ns):
        if self.failed is not None:return self.failed
        if now_ns<self.last_control_clock_ns:return self.latch('actual_simulation_clock_moved_backward')
        self.last_control_clock_ns=now_ns
        if now_ns-self.start_ns>=self.maximum_ns:return self.latch('whole_task_simulation_deadline_exceeded')
        return None
    def observe(self,stamp_ns,position,arrived=False,*,callback_clock_ns):
        failure=self.deadline(callback_clock_ns)
        if failure:return failure
        age_ns=callback_clock_ns-stamp_ns
        if age_ns<0:return self.latch('actual_pose_source_header_in_future')
        if age_ns>=300_000_000:return self.latch('actual_pose_source_header_not_fresh')
        if self.last_source_header_ns is not None:
            if stamp_ns<self.last_source_header_ns:return self.latch('actual_pose_source_header_moved_backward')
            if stamp_ns==self.last_source_header_ns:return None
        self.last_source_header_ns=stamp_ns;self.last_source_callback_clock_ns=callback_clock_ns
        self.last_source_header_age_ns=age_ns
        self.highwater=max(self.highwater,projected_progress(self.points,position))
        self.arrived=self.arrived or bool(arrived)
        if not self.arrived and callback_clock_ns-self.window_start_ns>=self.window_ns:
            if self.highwater-self.window_start_progress<self.minimum:
                reason='less_than_05m_forward_progress_in_90_sim_seconds'
                if not self.observe_only:return self.latch(reason)
                evidence=dict(original_reason=reason,window_start_sim_ns=self.window_start_ns,
                    window_end_sim_ns=callback_clock_ns,window_start_progress_m=self.window_start_progress,
                    forward_highwater_m=self.highwater,minimum_forward_progress_m=self.minimum,
                    original_would_stop=True,formal_full_route_pass=False)
                self.observed_business_rejections.append(evidence)
                if self.on_business_rejection is not None:self.on_business_rejection(reason,evidence)
            self.window_start_ns=callback_clock_ns;self.window_start_progress=self.highwater
        return None
    def snapshot(self):
        return dict(start_sim_ns=self.start_ns,last_control_clock_ns=self.last_control_clock_ns,
            last_actual_pose_stamp_ns=self.last_source_header_ns,last_source_header_ns=self.last_source_header_ns,
            last_source_callback_clock_ns=self.last_source_callback_clock_ns,last_source_header_age_ns=self.last_source_header_age_ns,
            forward_highwater_m=self.highwater,failed=self.failed,
            window_start_sim_ns=self.window_start_ns,window_start_progress_m=self.window_start_progress,
            window_sim_s=90,minimum_forward_progress_m=.5,arrival_only_exempts_progress_not_overall_deadline=self.arrived)


def build_node(run,scope,profile):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String,Bool
    from std_srvs.srv import Trigger
    from known_scene_route_registration import KnownSceneAxisRegistration,validate_initial_frame
    from request import StartupDeadline
    from runtime_io import EvidenceWriter,latest_sensor_qos
    from sensor_gate import TOPIC
    from runtime_evidence import json_bytes,status_projection,verify_rgb
    from r85_parking_runtime import R85ParkingRuntime

    class Owner(Node):
        def __init__(self):
            super().__init__('teacher_navigation_request')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.writer=EvidenceWriter();self.registration=KnownSceneAxisRegistration(run,scope,self.writer)
            self.started_wall=time.monotonic();self.deadline=StartupDeadline(profile,self.started_wall)
            taskfile=run/'r85_task_contract.json';self.task_sha=input_digest(scope,taskfile);self.task=json.loads(taskfile.read_text())
            if self.task.get('run_id')!=run.name or self.task.get('schema')!='r85_pct_scan_student_task/v1':raise ValueError('foreign_R85_task')
            self.source_file=Path(self.task['source_typed_route_file']).resolve()
            self.source=json.loads(self.source_file.read_text());self.selected=task_points(self.source,self.task)
            self.request_id=run.name+':r85:'+self.task['kind']+':1'
            self.parking=R85ParkingRuntime(run,self.request_id,profile);self.parking_result=None
            self.pose=None;self.last_pose_ns=None;self.health=None;self.health_wall=-math.inf
            self.nav=None;self.nav_wall=-math.inf;self.localization=None;self.localization_wall=-math.inf
            self.epoch=None;self.epoch_wall=-math.inf;self.reference_scope=None
            self.anchor=None;self.request=None;self.goal_binding=None;self.watchdog=None;self.result=None
            self.observed_business_keys=set();self.endpoint_since_ns=None;self.endpoint_last_ns=None
            self.endpoint_source_count=0;self.endpoint_milestone=None
            self.last_emit=-math.inf;self.dynamic_verifier=None;self.dynamic_result=None;self.dynamic_blocker=None
            self.dynamic_verifier_module=None;self.dynamic_capture_type=None;self.dynamic_evidence_capture=None
            self.completed_scope=completed_history_module(profile);self.dynamic_mission_v2_gate=None
            self.completed_history_wait={}
            if self.completed_scope is not None and not full_evidence_observation(profile):
                raise ValueError('completed_history_requires_diagnostic_full_evidence_profile')
            self.reference_doc=None;self.reference_sha=None;self.registration_sha=None;self.last_dynamic_try=-math.inf
            self.map_required=profile.get('engineering_recording',{}).get('current_run_RGB_map_save_required')is True
            if full_evidence_observation(profile) and not (self.task['kind']=='f1_to_f3'
                    and self.task.get('dynamic_required')is True and self.map_required):
                raise ValueError('full_evidence_requires_full_task_original_map_and_dynamic')
            self.map_client=self.create_client(Trigger,'/demo/slam/save_map')
            self.map_future=None;self.map_request_epoch=None;self.map_result=None
            self.status_stream=(run/'mission46_actual_navigation_statuses.jsonl').open('xb',buffering=0)
            self.pose_stream=(run/'mission46_actual_slam_poses.jsonl').open('xb',buffering=0)
            if self.task.get('dynamic_required'):
                try:
                    import r85_dynamic_obstacle_verifier as dynamic_verifier_module
                    from r85_dynamic_evidence_io import CaptureIO
                    input_digest(scope,HERE/'r85_dynamic_obstacle_verifier.py')
                    input_digest(scope,HERE/'r85_dynamic_evidence_io.py')
                    if self.completed_scope is not None:input_digest(scope,HERE/'r85_dynamic_completed_scope.py')
                    self.dynamic_verifier=dynamic_verifier_module.verify_result
                    self.dynamic_verifier_module=dynamic_verifier_module;self.dynamic_capture_type=CaptureIO
                except (ImportError,OSError,ValueError,AttributeError)as error:
                    self.dynamic_blocker='R85_dynamic_actual_evidence_runtime_unavailable:'+type(error).__name__+':'+str(error)
            self.pub=self.create_publisher(String,'/demo/navigation/request',10)
            self.anchor_pub=self.create_publisher(String,'/demo/teacher/navigation/anchor',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.stop_pub=self.create_publisher(Bool,'/demo/navigation/stop',10)
            self.status_pub=self.create_publisher(String,'/demo/teacher/navigation/request_status',10)
            self.mission_pub=self.create_publisher(String,'/demo/mission/state',10)
            self.create_subscription(Odometry,'/demo/slam/body_odom',self.on_pose,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Imu,'/livox/imu',self.on_imu,latest_sensor_qos())
            self.create_subscription(String,TOPIC,self.on_health,1)
            self.create_subscription(String,'/demo/navigation/status',self.on_nav,10)
            self.create_subscription(String,'/demo/navigation/localization_status',self.on_localization,10)
            self.create_subscription(String,'/demo/navigation/scan_epoch_ready',self.on_epoch,10)
            self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def ns(self):return int(self.get_clock().now().nanoseconds)
        def business(self,gate,reason,evidence):
            if not observe_only(profile):return False
            key=(gate,reason)
            if key not in self.observed_business_keys:
                observe_business(run,profile,gate,reason,dict(evidence,run_id=run.name,
                    request_id=self.request_id,observed_sim_ns=self.ns(),formal_full_route_pass=False))
                self.observed_business_keys.add(key)
            return True
        def observe_endpoint(self,row):
            if not observe_only(profile) or self.request is None or self.result is not None:return
            if full_evidence_observation(profile) and self.endpoint_milestone is not None:return
            goal=self.request['goals'][0];band=goal['arrival']['control_band']
            if band.get('type')!='disc_prism' or band.get('axes')!=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]:
                raise ValueError('diagnostic_endpoint_requires_original_fixed_disc_prism')
            center=xyz(goal['center']);position=xyz(row['position']);stamp=row['stamp_ns']
            radius=float(band['radius_m']);height=float(band['height_half_span_m']);dwell=float(band['dwell_sim_s'])
            if not all(math.isfinite(x) and x>0 for x in (radius,height,dwell)):
                raise ValueError('diagnostic_endpoint_original_band_invalid')
            inside=math.hypot(position[0]-center[0],position[1]-center[1])<=radius and abs(position[2]-center[2])<=height
            if not inside:
                self.endpoint_since_ns=None;self.endpoint_last_ns=None;self.endpoint_source_count=0;return
            if self.endpoint_last_ns is None or not 0<stamp-self.endpoint_last_ns<300_000_000:
                self.endpoint_since_ns=stamp;self.endpoint_source_count=0
            self.endpoint_last_ns=stamp;self.endpoint_source_count+=1
            if stamp-self.endpoint_since_ns<round(dwell*1e9):return
            evidence=dict(schema='r85_diagnostic_endpoint_result/v1',run_id=run.name,request_id=self.request_id,
                state='diagnostic_reached_endpoint',diagnostic_reached_endpoint=True,
                formal_full_route_pass=False,navigation_PASS=False,full_F3_PASS=False,original46_PASS=False,
                known_map_assistance=True,navigation_ground_truth_used=False,simulation_only=True,
                goal=copy.deepcopy(goal),actual_slam_pose=copy.deepcopy(row),
                first_inside_source_ns=self.endpoint_since_ns,last_inside_source_ns=stamp,
                unique_actual_pose_count=self.endpoint_source_count,original_control_band_used=True,
                parking_verified=False,dynamic_verified=False,arrival_is_not_formal_acceptance=True,
                progress=None if self.watchdog is None else self.watchdog.snapshot(),completed_sim_ns=self.ns())
            if full_evidence_observation(profile):
                evidence.update(state='diagnostic_endpoint_milestone',terminal=False,
                    endpoint_milestone_only=True,remaining_evidence=['original_fixed5_parking','original_current_RGB_map','original_dynamic_verifier'])
                atomic(run/'r85_diagnostic_endpoint_milestone.json',evidence)
                self.endpoint_milestone=copy.deepcopy(evidence)
                return
            atomic(run/'r85_diagnostic_result.json',evidence)
            self.result=dict(evidence,schema=RESULT_SCHEMA,kind=self.task['kind'],passed=False,
                failure_reason=None,parking_pass=False,dynamic_pass=False)
            atomic(run/'r85_task_result.json',self.result)
            self.stop_pub.publish(Bool(data=True))
        def read(self,msg):
            d=json.loads(msg.data)
            if not isinstance(d,dict):raise ValueError('actual_status_object_required')
            return d
        def fresh_health(self,now):
            h=self.health or{}
            return h.get('ready')is True and h.get('ground_truth_navigation_used')is False and h.get('mode')=='student' and 0<=now-self.health_wall<.3 and 0<=now-float(h.get('monotonic_wall',-math.inf))<.3
        def fresh_pose(self,now,ns):return self.pose is not None and 0<=now-self.pose['received_wall_ns']/1e9<.3 and 0<=ns-self.pose['stamp_ns']<300_000_000
        def on_health(self,msg):
            try:
                d=self.read(msg);now=time.monotonic()
                if d.get('mode')!='student' or d.get('ground_truth_navigation_used')is not False or not 0<=now-float(d['monotonic_wall'])<.3:return
                self.health,self.health_wall=d,now
                if d.get('ready')is not True:self.registration.reset('sensor_gate_not_ready')
            except (ValueError,TypeError,KeyError):return
        def on_imu(self,msg):
            v=msg.angular_velocity;a=msg.linear_acceleration
            self.registration.observe_imu(dict(stamp_ns=int(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec),
                callback_ros_clock_ns=self.ns(),received_wall_ns=time.monotonic_ns(),frame_id=msg.header.frame_id,
                angular_velocity_sensor=[v.x,v.y,v.z],linear_acceleration_sensor=[a.x,a.y,a.z],source_topic='/livox/imu'))
        def on_pose(self,msg):
            ns=self.ns();wall=time.monotonic_ns();stamp=int(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
            p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist.linear;w=msg.twist.twist.angular
            values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z]
            if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body' or not 0<=ns-stamp<300_000_000
                    or self.last_pose_ns is not None and stamp<=self.last_pose_ns
                    or not all(math.isfinite(x)for x in values) or not .98<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.02):return
            row=dict(stamp_ns=stamp,callback_ros_clock_ns=ns,received_sim_ns=ns,received_wall_ns=wall,source_received_wall_ns=wall,
                frame_id='camera_init',child_frame_id='demo_slam_body',position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],
                body_velocity=[v.x,v.y,v.z],body_angular_velocity=[w.x,w.y,w.z],source_topic='/demo/slam/body_odom',
                navigation_ground_truth_used=False,r85_provenance='r85_single_task_actual_canonical_SLAM')
            self.pose,self.last_pose_ns=row,stamp;self.pose_stream.write(json_bytes(row))
            self.registration.observe_pose(row,self.fresh_health(wall/1e9),wall)
            self.parking.on_pose(row)
            if self.watchdog is not None and self.result is None:
                failure=self.watchdog.observe(stamp,row['position'],arrived=(self.nav or{}).get('state')=='succeeded',callback_clock_ns=ns)
                if failure:self.fail(failure)
                else:self.observe_endpoint(row)
        def on_nav(self,msg):
            try:
                d=self.read(msg)
                if d.get('controller_kind')!='student' or d.get('navigation_ground_truth_used')is not False:return
                if d.get('request_id')!=self.request_id:return
                ns=self.ns();wall=time.monotonic_ns();self.nav,self.nav_wall=d,wall/1e9
                row=dict(data=status_projection(d),actual_full_status_canonical_sha256=hashlib.sha256(canonical(d).encode()).hexdigest(),
                    received_wall_ns=wall,received_sim_ns=ns,actual_slam_pose=copy.deepcopy(self.pose),
                    r85_provenance='r85_single_task_pose10hz_fixed5_v1')
                self.status_stream.write(json_bytes(row));self.parking.on_status(row)
                if d.get('state')=='failed':
                    reason='actual_navigation_failed:'+str(d.get('message'))
                    # The controller must separately observe its business gate;
                    # never resurrect an unknown numerical/native failure here.
                    business_message=diagnostic_business_failure(reason)
                    if not (business_message and self.business('navigation_task',reason,dict(actual_status=d))):self.fail(reason)
            except (ValueError,TypeError,KeyError,OSError)as error:self.fail('navigation_evidence_callback:'+str(error))
        def on_localization(self,msg):
            try:
                d=self.read(msg)
                if d.get('run_id')==run.name and d.get('schema')=='known_scene_navigation_status/v1':self.localization,self.localization_wall=d,time.monotonic()
            except (ValueError,TypeError):return
        def on_epoch(self,msg):
            try:
                d=self.read(msg)
                if d.get('run_id')==run.name and d.get('schema')=='known_scene_SCAN_epoch_ready/v1':self.epoch,self.epoch_wall=d,time.monotonic()
            except (ValueError,TypeError):return
        def freeze(self,ns,now):
            reg=self.registration.try_freeze(ns,time.monotonic_ns())
            if reg is None:return
            frame=json.loads((run/'known_scene_initialization_frame.json').read_text())
            proof=validate_initial_frame(run,scope,frame)
            if proof['T_navigation_world']!=FIXED_T or frame['T_navigation_world']!=self.source['T_navigation_world']:
                raise ValueError('actual_known_scene_T_does_not_equal_typed_fixed_T_no_mapping')
            source=copy.deepcopy(reg['stationary_source_records'][-1]['slam_source']);origin=source['position']
            goal,binding=goal_binding(profile,self.task,origin,self.selected);self.goal_binding=binding
            regsha=atomic(run/'navigation_scene_axis_registration.json',reg)
            self.anchor=dict(schema=1,experiment=profile['experiment'],run_dir=str(run),origin=origin,yaw=0.,
                pose_stamp_ns=source['stamp_ns'],frame_id='camera_init',source='/demo/slam/body_odom',ground_truth_navigation_used=False,
                frozen_once=True,monotonic_wall=now,scene_axis_registration=reg,
                scene_axis_registration_file='navigation_scene_axis_registration.json',scene_axis_registration_file_sha256=regsha,
                paired_sources_file=reg['paired_sources_file'],paired_sources_expected_file_sha256=reg['paired_sources_expected_file_sha256'],
                initial_actual_slam_source=source,registered_route_camera_init_xyz=[origin]+self.selected['body_points'],
                registered_route_points_sha256=hashlib.sha256(canonical([origin]+self.selected['body_points']).encode()).hexdigest(),
                registered_route_reference='single_R85_typed_PCT_route_with_actual_canonical_start_connector',
                original_region_sequence_executed=False,original_region_count_claimed=False,known_map_assistance=True,
                r85_task_kind=self.task['kind'],T_navigation_world=frame['T_navigation_world'])
            atomic(run/'navigation_anchor.json',self.anchor)
            self.request=dict(schema_version=2,request_id=self.request_id,frame_id='camera_init',goals=[goal])
            atomic(run/'navigation_request.json',self.request);atomic(run/'r85_task_goal_binding.json',binding)
            self.watchdog=TaskProgressWatchdog(self.selected['body_points'],ns,self.task['timing']['whole_task_maximum_sim_s'],
                diagnostic_observe_only=observe_only(profile),
                on_business_rejection=lambda reason,evidence:self.business('task_progress',reason,evidence))
        def register_reference(self,now):
            e=self.epoch or{};l=self.localization or{}
            if not (0<=now-self.epoch_wall<.3 and 0<=now-self.localization_wall<.3 and e.get('ready')is True
                    and l.get('initialized')is True and l.get('correction_transition')is False and e.get('generation')==l.get('generation')
                    and e.get('localization_generation')==l.get('generation') and e.get('robot_truth_pose_used')is False):return False
            epoch=e.get('planner_epoch');generation=l.get('generation');pid=(e.get('planner_identity')or{}).get('pid')
            if any(type(x)is not int or x<1 for x in (epoch,generation,pid)):raise ValueError('actual_planner_scope_missing')
            current=(epoch,pid,generation)
            if current==self.reference_scope:
                return True # Program-owned version is already installed.
            if self.reference_scope is not None and epoch<=self.reference_scope[0]:
                raise ValueError('actual_planner_scope_changed_without_epoch_advance')
            reference_id=self.request_id+':epoch:'+str(epoch)+':generation:'+str(generation)
            doc=dict(schema='r85_pct_scan_global_reference/v1',run_id=run.name,frame_id='camera_init',epoch=epoch,
                localization_generation=generation,reference_id=reference_id,
                body_points=self.selected['body_points'],ground_points=self.selected['ground_points'],
                physical_body_ground_offset_m=.32,scan_transport_offset_m=.4,navigation_ground_truth_used=False,
                source_route_file=str(self.source_file),source_route_sha256=sha(self.source_file),
                physical_sources=self.source['physical_sources'],T_navigation_world=FIXED_T,
                task_goal_body_xyz=self.goal_binding['task_goal_body_xyz'],
                original_F3_goal_body_xyz=self.goal_binding['original_F3_goal_body_xyz'],
                reference_scope=self.selected['reference_scope'],task_goal_binding=self.goal_binding)
            for key in ('prefix_length_m','prefix_endpoint_body_xyz','prefix_evidence'):
                if key in self.selected:doc[key]=self.selected[key]
            from terminal_heading_contract import attach_reference_heading
            doc=attach_reference_heading(profile,doc)
            from automatic_reference_runtime import reference_version_paths
            reference_version,registration_version=reference_version_paths(run,epoch,generation)
            if reference_version.exists()or registration_version.exists():
                raise ValueError('program_reference_version_already_exists')
            refsha=atomic(reference_version,doc)
            atomic(run/'r85_global_reference.json',doc) # Current compatibility view only.
            registration=dict(schema='r85_reference_source_registration/v1',run_id=run.name,
                reference_file_sha256=refsha,reference_id=reference_id,source_route_sha256=doc['source_route_sha256'],
                localization_generation=generation,navigation_ground_truth_used=False,producer='root_authorized_PCT_route_owner',
                task_source_file=str((run/'r85_task_contract.json').resolve()))
            self.registration_sha=atomic(registration_version,registration)
            atomic(run/'r85_reference_registration.json',registration)
            self.reference_version_path,self.registration_version_path=reference_version,registration_version
            self.writer.append(run/'r85_reference_scope_history.jsonl',dict(planner_epoch=epoch,planner_process_id=pid,
                localization_generation=generation,reference_id=reference_id,reference_file_sha256=refsha,
                reference_version_file=str(reference_version),registration_version_file=str(registration_version),
                global_reference_rebuilt_only_on_scope_change=True,navigation_ground_truth_used=False))
            self.reference_scope=current;self.reference_doc=doc;self.reference_sha=refsha;return True
        def mission_state(self,ns):
            doc=self.reference_doc
            if doc is None:return
            state=dict(schema='r85_actual_single_task_mission_state/v1',run_id=run.name,current_request=self.request_id,
                stage='navigating'if self.task['kind']=='f1_to_f3'else'r85_short_validation',
                navigation_ground_truth_used=False,original46_PASS=False,task_navigation_state=(self.nav or{}).get('state'),
                source_actual_pose_stamp_ns=self.pose['stamp_ns'],ros_sim_time_ns=ns,
                r85_reference=dict(reference_scope=doc['reference_scope'],reference_id=doc['reference_id'],
                    source_route_sha256=doc['source_route_sha256'],epoch=doc['epoch'],localization_generation=doc['localization_generation'],
                    global_reference_file=str(self.reference_version_path),registration_file=str(self.registration_version_path),
                    global_reference_sha256=self.reference_sha))
            self.mission_pub.publish(String(data=json.dumps(state,allow_nan=False)))
        def map_gate(self):
            if not self.map_required:return True
            if self.map_result is not None:return self.map_result.get('passed')is True
            if self.map_future is None:
                if not self.map_client.service_is_ready():return False
                self.map_request_epoch=time.time();self.map_future=self.map_client.call_async(Trigger.Request())
                return False
            if not self.map_future.done():return False
            response=self.map_future.result();service=dict(success=bool(response.success),message=response.message)
            atomic(run/'r85_map_save_service_response.json',dict(run_id=run.name,request_id=self.request_id,
                actual_service='/demo/slam/save_map',requested_epoch_s=self.map_request_epoch,response=service))
            receipt=verify_rgb(run,self.request_id,service,self.map_request_epoch)
            self.map_result=dict(schema='r85_actual_current_rgb_save/v1',run_id=run.name,request_id=self.request_id,
                passed=True,original_current_run_map_gate_unchanged=True,original46_PASS=False,
                known_map_assistance=True,navigation_ground_truth_used=False,receipt=receipt)
            atomic(run/'r85_current_rgb_save_receipt.json',self.map_result);return True
        def fail(self,reason):
            if self.result is not None:return
            self.finish(False,reason,hard_failure=True)
        def finish(self,passed,reason,*,hard_failure=False):
            if full_evidence_observation(profile) and not hard_failure:
                if passed is not True:raise ValueError('full_evidence_original_completion_not_passed')
                result=diagnostic_evidence_terminal(run,self.request_id,self.task,self.endpoint_milestone,
                    self.parking_result,self.map_result,self.dynamic_result,self.dynamic_evidence_capture,
                    self.ns(),None if self.watchdog is None else self.watchdog.snapshot())
                if self.completed_scope is not None:
                    result=self.completed_scope.honest_terminal_fields(result,self.dynamic_mission_v2_gate)
                atomic(run/'r85_task_result.json',result)
                atomic(run/'r85_task_status.json',result)
                # Last atomic file is the existing preflight diagnostic stop marker.
                atomic(run/'r85_diagnostic_result.json',result)
                self.result=result;self.stop_pub.publish(Bool(data=True))
                return
            if observe_only(profile) and not hard_failure:
                self.business('formal_task_completion',reason,dict(original_pass_candidate=bool(passed),
                    actual_parking_result=self.parking_result,actual_dynamic_result=self.dynamic_result))
                return
            if (passed and self.task['kind']=='f1_to_f3' and self.task.get('dynamic_required')
                    and self.dynamic_evidence_capture is None):
                passed=False;reason='actual_dynamic_same_read_capture_missing'
            self.result=dict(schema=RESULT_SCHEMA,run_id=run.name,request_id=self.request_id,kind=self.task['kind'],
                state='succeeded'if passed else'failed',passed=passed,failure_reason=None if passed else reason,
                parking_pass=bool(self.parking_result and self.parking_result.get('passed')),
                dynamic_required=bool(self.task.get('dynamic_required')),dynamic_pass=bool(self.dynamic_result and self.dynamic_result.get('passed')),
                short_chain_validation_only=self.task['kind']=='business_chain_prefix',full_F3_PASS=bool(passed and self.task['kind']=='f1_to_f3'),
                original46_PASS=False,navigation_ground_truth_used=False,simulation_only=True,
                completed_sim_ns=self.ns(),parking_result=self.parking_result,dynamic_result=self.dynamic_result,
                dynamic_evidence_capture=self.dynamic_evidence_capture,
                current_rgb_map_required=self.map_required,current_rgb_map_verified=bool(self.map_result and self.map_result.get('passed')),
                current_rgb_map_result=self.map_result,known_map_assistance=True,
                progress=None if self.watchdog is None else self.watchdog.snapshot(),timing_scope=self.task['timing'])
            atomic(run/'r85_task_result.json',self.result)
            if not passed:self.stop_pub.publish(Bool(data=True))
        def tick(self):
            now=time.monotonic();ns=self.ns()
            try:
                if self.result is not None:
                    if self.result['state'] in ('failed','diagnostic_reached_endpoint','diagnostic_evidence_complete'):self.stop_pub.publish(Bool(data=True))
                    return
                if self.writer.error:self.fail('evidence_writer:'+self.writer.error);return
                if self.dynamic_blocker:self.fail(self.dynamic_blocker);return
                early_dynamic=dynamic_fatal_decision(run,self.task,scope,self.request_id,self.reference_doc,self.reference_sha,
                    completed_wait=self.completed_history_wait)
                if early_dynamic.get('observed_business_rejection'):
                    self.business('dynamic_protocol',early_dynamic['failure_reason'],early_dynamic)
                if early_dynamic['fatal']:
                    early_dynamic.update(run_id=run.name,request_id=self.request_id,checked_sim_ns=ns,
                        checked_monotonic_wall=now,actual_navigation_or_parking_result_inferred=False)
                    atomic(run/'r85_dynamic_early_failure.json',early_dynamic)
                    self.dynamic_result=dict(schema='r85_dynamic_task_result/v1',run_id=run.name,request_id=self.request_id,
                        passed=False,state='failed',failure_reason=early_dynamic['failure_reason'],
                        early_failure_evidence_file=str(run/'r85_dynamic_early_failure.json'),
                        original_dynamic_failure=early_dynamic.get('original_dynamic_failure'),navigation_PASS=False,full_F3_PASS=False)
                    self.fail('actual_dynamic_runtime_fatal:'+early_dynamic['failure_reason']);return
                if self.request is None:
                    if self.deadline.observe(ns,now):self.fail(self.deadline.failed);return
                    if self.fresh_health(now)and self.fresh_pose(now,ns):self.freeze(ns,now)
                if self.request is not None:
                    failure=self.watchdog.deadline(ns)
                    if failure:self.fail(failure);return
                    ready=self.fresh_health(now)and self.fresh_pose(now,ns)and self.register_reference(now)
                    if ready and now-self.last_emit>=1.:
                        self.anchor_pub.publish(String(data=json.dumps(self.anchor,allow_nan=False)))
                        self.pub.publish(String(data=json.dumps(self.request,allow_nan=False)));self.last_emit=now
                    if ready:self.mission_state(ns)
                    self.parking.poll_native();value=self.parking.tick(ns)
                    if value is not None:self.parking_result=value
                    if self.parking_result is not None:
                        if self.parking_result.get('passed')is not True:
                            reason='actual_fixed5_parking_failed:'+str(self.parking_result.get('failure_reason'))
                            if (self.parking_result.get('failure_reason')=='actual_controller_failed'
                                    and diagnostic_business_failure('actual_navigation_failed:'+str((self.nav or{}).get('message')))
                                    and self.business('parking_acceptance',reason,dict(actual_parking_result=self.parking_result))):return
                            self.fail(reason);return
                        if observe_only(profile) and not full_evidence_observation(profile):
                            self.business('formal_task_completion','actual_parking_observed_not_formal_PASS',dict(actual_parking_result=self.parking_result))
                            return
                        if not self.map_gate():return
                        if early_dynamic.get('state')=='waiting_completed_history_navigation_handoff':return
                        if self.dynamic_verifier is None:self.finish(True,'short_chain_actual_parking_and_required_map_passed');return
                        if ns/1e9-self.last_dynamic_try>=1.:
                            self.last_dynamic_try=ns/1e9
                            capture=self.dynamic_capture_type(run,self.request_id,self.dynamic_verifier_module,scope,
                                owner_source=Path(__file__))
                            self.dynamic_result=self.dynamic_verifier(run,self.request_id,profile,scope,evidence_io=capture)
                            if self.dynamic_result.get('passed')is True:
                                if self.completed_scope is not None:
                                    self.dynamic_mission_v2_gate=self.completed_scope.verify_late_mission_gate(
                                        self.dynamic_verifier_module,run,self.dynamic_result)
                                    self.dynamic_result=self.completed_scope.enrich_dynamic_result(self.dynamic_result,
                                        self.dynamic_mission_v2_gate)
                                self.dynamic_evidence_capture=capture.seal(self.dynamic_result)
                        dynamic=self.dynamic_result
                        if dynamic is not None and dynamic.get('passed')is True:
                            if dynamic.get('run_id')!=run.name or dynamic.get('request_id')!=self.request_id:raise ValueError('foreign_actual_dynamic_receipt')
                            self.finish(True,'full_actual_parking_map_and_dynamic_passed')
                        elif dynamic is not None and dynamic.get('state')=='failed':self.fail('actual_dynamic_failed:'+str(dynamic.get('failure_reason')))
                if full_evidence_observation(profile) and self.result is not None:return
                status=dict(schema='r85_actual_task_status/v1',run_id=run.name,request_id=self.request_id,
                    state='waiting_actual_sensor_slam'if self.request is None else'active',kind=self.task['kind'],
                    reference_scope=self.reference_scope,actual_slam_pose_stamp_ns=None if self.pose is None else self.pose['stamp_ns'],
                    progress=None if self.watchdog is None else self.watchdog.snapshot(),timing_scope=self.task['timing'],
                    ros_sim_time_ns=ns,monotonic_wall=now,navigation_ground_truth_used=False,original46_PASS=False)
                atomic(run/'r85_task_status.json',status);self.status_pub.publish(String(data=json.dumps(status,allow_nan=False)))
            except (OSError,ValueError,TypeError,KeyError,AttributeError,RuntimeError)as error:self.fail(type(error).__name__+':'+str(error))
        def close(self):
            if rclpy.ok():self.stop_pub.publish(Bool(data=True))
            self.status_stream.close();self.pose_stream.close();self.writer.close()
    return Owner()


def main():
    import rclpy
    from pid_scope import verify_scope
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();receipt=verify_scope(run/'navigation_scope.json')
    scope=json.loads((run/'navigation_scope.json').read_text());profile=receipt['profile']
    if any((run/name).exists()for name in ('navigation_anchor.json','navigation_request.json','r85_task_result.json')):
        raise RuntimeError('new_R85_run_required_existing_task_not_reused')
    rclpy.init(args=ros);node=build_node(run,scope,profile)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
