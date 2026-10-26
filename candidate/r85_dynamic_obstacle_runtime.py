#!/usr/bin/env python3
"""Owned full-R85 obstacle choreography and raw evidence; never robot control.

Only the moving_obstacle model SetEntityPose service is called. A separate
read-only native Gazebo pose/info subscription observes that model, not robot
GT. A service ACK is always labelled as an ACK, never as observed world pose.
No ROS/Gazebo process is launched when this module is imported.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import numpy as np
import r85_dynamic_completed_scope as completed_scope
import r117_dynamic_speed_lead as speed_lead
from r74_dynamic_causal_intake import (enabled as causal_intake_enabled, BoundedMissionIntake,
    clock_targets as causal_clock_targets, envelope as causal_envelope)

HERE=Path(__file__).resolve().parent
if '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/workspace/multifloor_demo/teacher_mode/navigation' not in sys.path:sys.path.append('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/workspace/multifloor_demo/teacher_mode/navigation')

FIXED_T = [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-.3],[0.,0.,0.,1.]]
PARK = [17.,5.7,1.8]
BLOCK = [14.9,5.7,1.8]
SIZE = [.5,.5,1.2]
POSE_TOPIC = '/world/teacher_demo/pose/info'
SERVICE = '/world/teacher_demo/set_pose'
ENTITY = 'moving_obstacle'


ACTOR_KEY='r85_actor_choreography'
ACTOR_SELECTION='independent_posttrigger_world_motion/v2'
ACTOR_PROTOCOL='r85_f1_f3_fixed_crossing_world_motion/v2'
ACTOR_MOTION_CONTRACT=dict(schema=ACTOR_SELECTION,
    trigger_fresh_identity_and_lead_window_unchanged=True,
    posttrigger_navigation_ready_and_mission_freshness_required=False,
    same_trigger_actor_fresh_observation_required=True,
    original_service_ACK_failure_cancel_and_owner_stop_required=True,
    fixed_geometry_speed_and_real_stop_resume_recovery_required=True)

def actor_choreography(profile):
    if ACTOR_KEY not in profile:return None
    value=profile[ACTOR_KEY]
    if type(value)is not str or value!=ACTOR_SELECTION:
        raise ValueError('explicit_actor_choreography_selection_differs')
    return dict(ACTOR_MOTION_CONTRACT)


EVENT_WINDOW='single_completed_event_window/v1'
EVENT_SELECTION=dict(schema=EVENT_WINDOW,completed_event_window=True,
    whole_task_single_scope=False,completed_same_geometry_preservation=False)

SEGMENT_WINDOW='single_world_event_pre_stop_nav_segment/v2'
SEGMENT_SELECTION=dict(schema=SEGMENT_WINDOW,completed_event_window=True,
    whole_task_single_scope=False,completed_same_geometry_preservation=False,
    maximum_pre_stop_navigation_switches=1)
COORDINATE_WINDOW='single_world_event_coordinate_handoffs/v3'
COORDINATE_SELECTION=dict(schema=COORDINATE_WINDOW,completed_event_window=True,
    whole_task_single_scope=False,completed_same_geometry_preservation=False,maximum_coordinate_handoffs=2)

def coordinate_handoffs_enabled(profile):
    value=profile.get('r85_dynamic_event_contract',{})
    if not isinstance(value,dict) or value.get('schema')!=COORDINATE_WINDOW:return False
    if set(value)!=set(COORDINATE_SELECTION) or any(type(value[k])is not type(v) or value[k]!=v for k,v in COORDINATE_SELECTION.items()):
        raise ValueError('dynamic_coordinate_handoff_selection_differs')
    return True

def navigation_segment_schema(profile):
    return COORDINATE_WINDOW if coordinate_handoffs_enabled(profile) else SEGMENT_WINDOW

def navigation_segment_policy(profile):
    return dict(COORDINATE_SELECTION if coordinate_handoffs_enabled(profile) else SEGMENT_SELECTION)

NAVIGATION_KINDS=frozenset(('status','actual_pose','mission_state'))

def coordinate_navigation_source(kind,data):
    """Original payload identity only; never infer an epoch from callback order."""
    if kind=='status':
        epoch=data.get('known_scene_scan_epoch')or{};localization=data.get('known_scene_localization')or{}
        status=data.get('known_scene_runtime_status')or{}
        return dict(kind=kind,planner_epoch=epoch.get('planner_epoch'),
            localization_generation=epoch.get('localization_generation'),
            localization_payload_generation=localization.get('generation'),runtime_payload_generation=status.get('generation'))
    if kind=='mission_state':
        ref=data.get('r85_reference')or{}
        return dict(kind=kind,planner_epoch=ref.get('epoch'),localization_generation=ref.get('localization_generation'))
    return dict(kind=kind,planner_epoch=None,localization_generation=None,source_stamp_ns=data.get('stamp_ns'),
        corrected_body_message_sha256=data.get('corrected_body_message_sha256'),
        identity_scope='exact_body_bundle_join_required_offline')


def coordinate_status_matches(data,binding):
    source=coordinate_navigation_source('status',data)
    return (type(source['planner_epoch'])is int and source['planner_epoch']==binding['epoch']
        and all(type(source[k])is int and source[k]==binding['localization_generation']
            for k in ('localization_generation','localization_payload_generation','runtime_payload_generation')))


def navigation_segments_enabled(profile):
    if coordinate_handoffs_enabled(profile):return True
    value=profile.get('r85_dynamic_event_contract',{})
    if not isinstance(value,dict) or value.get('schema')!=SEGMENT_WINDOW:return False
    if set(value)!=set(SEGMENT_SELECTION) or any(type(value[k])is not type(v) or value[k]!=v for k,v in SEGMENT_SELECTION.items()):
        raise ValueError('dynamic_navigation_segment_selection_differs')
    return True

def navigation_segment_ready(status,binding,clock,wall,received):
    if not isinstance(status,dict) or not 0<=wall-received<.3:return False
    d=status.get('known_scene_localization')or{};st=status.get('known_scene_runtime_status')or{};e=status.get('known_scene_scan_epoch')or{}
    return bool(status.get('state')=='running' and status.get('request_id')==binding['request_id']
        and d.get('generation')==st.get('generation')==e.get('localization_generation')==binding['localization_generation']
        and e.get('planner_epoch')==binding['epoch'] and e.get('ready')is True
        and (e.get('child_odometry_consumption')or{}).get('ready')is True
        and st.get('initialized')is True and st.get('error')is None
        and d.get('control_allowed')is True and d.get('correction_stale')is False
        and d.get('correction_transition')is False and st.get('correction_transition')is False
        and type(d.get('source_ns'))is int and 0<=clock-d['source_ns']<=300_000_000
        and all(type(status.get(k))in(int,float) and 0<=status[k]<.3 for k in ('pose_age','cloud_age','raw_imu_age'))
        and (status.get('execution_bridge_safety')or{}).get('state')=='ready')

def validate_navigation_segment_edge(edge):
    """Once at a real version boundary; never manufactures a new trigger."""
    old=edge['old_binding'];new=edge['new_binding'];a=edge['old_reference'];b=edge['new_reference']['document']
    if edge.get('schema')!='world_event_pre_stop_navigation_edge/v1' or edge.get('switch_index')!=1 or edge.get('dynamic_proof_seen')is not False:
        raise ValueError('navigation_segment_not_first_pre_stop_edge')
    if any(type(old.get(k))is not type(new.get(k)) or old[k]!=new[k] for k in ('run_id','request_id','source_route_sha256')):
        raise ValueError('navigation_segment_foreign_world_event')
    if not (type(new.get('epoch'))is int and type(new.get('localization_generation'))is int and new['epoch']>old['epoch'] and new['localization_generation']>old['localization_generation']):
        raise ValueError('navigation_segment_identity_did_not_advance')
    if set(a)!=set(b) or any(a[k]!=b[k] for k in a if k not in ('epoch','localization_generation','reference_id')):
        raise ValueError('navigation_segment_physical_route_or_goal_changed')
    if not navigation_segment_ready(edge['navigation_status'],new,edge['cut_sim_ns'],edge['boundary_wall'],edge['status_received_wall']):
        raise ValueError('navigation_segment_original_sources_not_ready')
    release=edge['transaction_release'];c=release.get('convergence')or{};d=edge['navigation_status']['known_scene_localization']
    if (release.get('generation')!=new['localization_generation'] or release.get('measured_zero_stop_verified')is not True
            or c.get('converged')is not True or not c.get('body_residual_m',float('inf'))<.005
            or not c.get('origin_residual_m',float('inf'))<.005 or not c.get('rotation_residual_rad',float('inf'))<.003
            or release.get('C_world_odom')!=d.get('C_world_odom')
            or not 0<release.get('released_source_ns',0)<=d['source_ns']<=edge['cut_sim_ns']):
        raise ValueError('navigation_segment_coordinate_transaction_not_complete')
    actor=edge['actor_observation'];identity=edge['trigger_actor_identity']
    position=np.asarray(actor['position_world_xyz'],float);ack=np.asarray(edge['last_actor_ACK_position'],float)
    if position.shape!=(3,) or ack.shape!=(3,) or not np.isfinite(np.r_[position,ack]).all():raise ValueError('navigation_segment_actor_nonfinite')
    if (actor.get('native_gz_pose_info')is not True or actor.get('entity')!=ENTITY
            or actor.get('entity_id')!=identity.get('entity_id') or actor.get('entity')!=identity.get('entity')
            or actor.get('raw_model_pose',{}).get('name')!=ENTITY or actor.get('raw_model_pose',{}).get('id')!=actor.get('entity_id')
            or not 0<=edge['cut_sim_ns']-actor.get('stamp_ns',-1)<=300_000_000
            or not 0<=edge['boundary_wall']-edge['actor_received_wall']<.3
            or _stamp(actor['header'])!=actor['stamp_ns']
            or np.linalg.norm(np.asarray(actor['position_world_xyz'])-np.asarray(edge['last_actor_ACK_position']))>.05):
        raise ValueError('navigation_segment_actor_continuity_not_observed')
    return True

def validate_coordinate_handoff_edge(edge):
    if (edge.get('schema')!='world_event_coordinate_handoff_edge/v1'
            or type(edge.get('switch_index'))is not int or not 1<=edge['switch_index']<=2
            or edge.get('event_state')!='active'
            or type(edge.get('dynamic_proof_seen'))is not bool):
        raise ValueError('coordinate_handoff_state_or_bound_differs')
    # Reuse every original geometry, owner, transaction, source-freshness and
    # actor-continuity gate; the new policy changes only the event phase limit.
    checked=dict(edge,schema='world_event_pre_stop_navigation_edge/v1',switch_index=1,dynamic_proof_seen=False)
    validate_navigation_segment_edge(checked)
    return True


def coordinate_owner_binding(run,anchor,actual):
    sequences=actual.get('navigation_boundary_sequences')
    if (type(sequences)is not list or len(sequences)>2
            or any(type(q)is not int or q<1 for q in sequences)
            or sequences!=sorted(set(sequences))):
        raise ValueError('coordinate_handoff_sequence_chain_invalid')
    key=(str(run),json.dumps(anchor,sort_keys=True),tuple(sequences),actual.get('started_sim_ns'),json.dumps(actual.get('coordinate_boundary_receipts'),sort_keys=True))
    cached=getattr(coordinate_owner_binding,'_validated_tip',None)
    if cached is not None and cached[0]==key:
        if actual.get('current_navigation_binding')!=cached[1]:raise ValueError('coordinate_handoff_current_binding_not_chain_tip')
        return dict(cached[1])
    previous=anchor;previous_reference=None;last_cut=-1
    if sequences:
        path=run/'r85_dynamic_journal.jsonl';receipts=actual.get('coordinate_boundary_receipts')
        if type(receipts)is not list or len(receipts)!=len(sequences):
            raise ValueError('coordinate_handoff_receipts_missing')
        edges=[]
        with path.open('rb') as stream:
            for sequence,receipt in zip(sequences,receipts):
                offset=receipt.get('source_offset');length=receipt.get('source_length')
                if (receipt.get('path')!=str(path) or type(offset)is not int or offset<0
                        or type(length)is not int or not 0<length<=2*1024**2):
                    raise ValueError('coordinate_handoff_receipt_bound_or_path')
                stream.seek(offset);raw=stream.read(length)
                if (len(raw)!=length or not raw.endswith(b'\n')
                        or hashlib.sha256(raw).hexdigest()!=receipt.get('source_line_sha256')):
                    raise ValueError('coordinate_handoff_real_journal_bytes_differ')
                row=json.loads(raw)
                if row.get('kind')!='navigation_segment_boundary' or row.get('sequence')!=sequence:
                    raise ValueError('coordinate_handoff_journal_sequence_differs')
                edges.append(row)
        for index,row in enumerate(edges,1):
            edge=row['data'];validate_coordinate_handoff_edge(edge)
            if (row.get('binding')!=anchor or edge['switch_index']!=index
                    or edge['old_binding']!=previous or edge['cut_sim_ns']<=last_cut
                    or (previous_reference is not None and edge['old_reference']!=previous_reference)
                    or edge['world_event_trigger_sim_ns']!=actual.get('started_sim_ns')):
                raise ValueError('coordinate_handoff_anchor_or_order_differs')
            previous=edge['new_binding'];previous_reference=edge['new_reference']['document'];last_cut=edge['cut_sim_ns']
    if actual.get('current_navigation_binding')!=previous:
        raise ValueError('coordinate_handoff_current_binding_not_chain_tip')
    coordinate_owner_binding._validated_tip=(key,dict(previous))
    return previous


def event_window_enabled(profile):
    if navigation_segments_enabled(profile):return True
    value=profile.get('r85_dynamic_event_contract',{})
    if not isinstance(value,dict)or value.get('schema')!=EVENT_WINDOW:return False
    if value!=EVENT_SELECTION or any(type(value.get(k))is not bool for k in
            ('completed_event_window','whole_task_single_scope','completed_same_geometry_preservation')):
        raise ValueError('dynamic_event_window_selection_differs')
    return True

def completed_event_window(state):
    """Small program terminal record only; never a physical PASS assertion."""
    schema=state.get('event_lifetime_contract')
    if (schema not in (EVENT_WINDOW,SEGMENT_WINDOW,COORDINATE_WINDOW)
            or state.get('event_state')!='completed_pending_verification'
            or state.get('diagnostic_protocol_cancelled')is not False
            or state.get('failure')is not None):return None
    window=state.get('completed_event_window')
    if (not isinstance(window,dict)or window.get('schema')!=schema
            or window.get('binding')!=state.get('binding')
            or window.get('started_sim_ns')!=state.get('started_sim_ns')
            or any(type(window.get(k))is not int or window[k]<1 for k in
                ('started_sim_ns','completed_sim_ns','journal_last_sequence','recovery_origin_source_ns','recovery_end_source_ns'))
            or not window['started_sim_ns']<=window['recovery_origin_source_ns']<=window['recovery_end_source_ns']<=window['completed_sim_ns']<=state.get('sim_ns',-1)
            or window.get('business_completion_only')is not True
            or window.get('physical_verified')is not False):return None
    if schema in (SEGMENT_WINDOW,COORDINATE_WINDOW) and (window.get('navigation_binding')!=state.get('current_navigation_binding')
            or window.get('navigation_boundary_sequences')!=state.get('navigation_boundary_sequences')
            or type(window.get('navigation_boundary_sequences'))is not list
            or len(window['navigation_boundary_sequences'])>(2 if schema==COORDINATE_WINDOW else 1)):return None
    return window


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# Diagnostic-only: no source clocks, failure predicates or control state are changed.
MISSION_WITNESS_PAYLOAD_LIMIT = 16 * 1024
MISSION_WITNESS_DOCUMENT_LIMIT = 64 * 1024


def _mission_witness_wall():
    try:return time.monotonic()
    except Exception:return None


def _mission_witness_error(error,limit):
    prefix=type(error).__name__+':'
    text=str(error)
    return prefix+text[:max(0,limit-len(prefix))],len(prefix)+len(text)>limit


def _mission_witness_value(value):
    kind=type(value).__name__
    if value is None or type(value)is bool:return dict(type=kind,value=value)
    if type(value)is int:
        return dict(type=kind,value=value) if value.bit_length()<=128 else dict(type=kind,decimal_prefix=str(value)[:256],truncated=True)
    if type(value)is float:
        return dict(type=kind,value=value) if math.isfinite(value) else dict(type=kind,nonfinite=repr(value))
    if type(value)is str:return dict(type=kind,value=value[:256],truncated=len(value)>256)
    return dict(type=kind)


def _mission_witness_binding(value):
    return {key:_mission_witness_value(value.get(key)) for key in (
        'run_id','request_id','reference_id','reference_sha256','epoch','localization_generation','source_route_sha256')} if type(value)is dict else _mission_witness_value(value)


def _mission_clock_failure_witness(payload,run_id,entry_clock,received_wall,clock,state,binding,current_binding,phase,error,validation_wall=None,exception_wall=None):
    text=payload if type(payload)is str else ''
    # Bound extra allocation/work even for an already-received oversized String.
    bounded=text[:MISSION_WITNESS_PAYLOAD_LIMIT+1].encode('utf-8',errors='surrogatepass')
    complete=len(text)<=MISSION_WITNESS_PAYLOAD_LIMIT and len(bounded)<=MISSION_WITNESS_PAYLOAD_LIMIT
    prefix=bounded[:MISSION_WITNESS_PAYLOAD_LIMIT]
    obj=state if type(state)is dict else {}
    values={key:_mission_witness_value(obj.get(key)) for key in ('schema','ros_sim_time_ns','source_actual_pose_stamp_ns')}
    checks={};ages={}
    if clock is not None:
        checks['schema_exact']=obj.get('schema')=='r85_actual_single_task_mission_state/v1'
        for key in ('ros_sim_time_ns','source_actual_pose_stamp_ns'):
            exact=type(obj.get(key))is int;checks[key+'_exact_int']=exact
            if exact:
                checks[key+'_nonnegative']=obj[key]>=0
                checks[key+'_not_future']=obj[key]<=clock
                ages[key]=_mission_witness_value(clock-obj[key])
                checks[key+'_age_within_300ms']=0<=clock-obj[key]<=300_000_000
    failed=[key for key,value in checks.items() if value is False]
    result=dict(schema='r85_dynamic_mission_first_failure_witness/v1',run_id=run_id,
        original_failure=_mission_witness_error(error,2048)[0],
        original_failure_text_truncated=_mission_witness_error(error,2048)[1],failure_phase=phase,
        callback_entry_monotonic_wall=received_wall,callback_entry_local_clock_ns=entry_clock,
        validation_monotonic_wall=validation_wall,exception_monotonic_wall=exception_wall,
        callback_wall_until_validation_s=None if validation_wall is None else validation_wall-received_wall,
        original_validation_clock_ns=clock,validation_clock_samples=[] if clock is None else [clock],
        observed_fields=values,validation_predicates=checks,false_predicates=failed,signed_ages_ns=ages,
        current_binding=_mission_witness_binding(current_binding),rejected_binding=_mission_witness_binding(binding),
        rejected_reference=_mission_witness_binding(obj.get('r85_reference')),
        rejected_payload_binding=_mission_witness_binding(dict(run_id=obj.get('run_id'),request_id=obj.get('current_request'),
            reference_id=(obj.get('r85_reference') if type(obj.get('r85_reference'))is dict else {}).get('reference_id'),
            reference_sha256=(obj.get('r85_reference') if type(obj.get('r85_reference'))is dict else {}).get('global_reference_sha256'),
            epoch=(obj.get('r85_reference') if type(obj.get('r85_reference'))is dict else {}).get('epoch'),
            localization_generation=(obj.get('r85_reference') if type(obj.get('r85_reference'))is dict else {}).get('localization_generation'),
            source_route_sha256=(obj.get('r85_reference') if type(obj.get('r85_reference'))is dict else {}).get('source_route_sha256'))),
        payload=dict(encoding='ROS_String_data_UTF8_surrogatepass_not_CDR',payload_python_type=type(payload).__name__,
            character_count=len(text),maximum_retained_bytes=MISSION_WITNESS_PAYLOAD_LIMIT,
            retained_utf8_bytes=len(prefix),retained_utf8_hex=prefix.hex(),
            retained_prefix_sha256=hashlib.sha256(prefix).hexdigest(),overflow=not complete,
            original_utf8_sha256=hashlib.sha256(bounded).hexdigest() if complete else None,
            original_utf8_bytes=len(bounded) if complete else None),
        source_timestamps_rewritten=False,diagnostic_only=True,control_or_stop_credit=False,
        publication_semantics='first frozen witness; explicit durable publication status, independent original-state copy')
    assert len(json.dumps(result,allow_nan=False).encode('utf-8'))<MISSION_WITNESS_DOCUMENT_LIMIT
    return result


def _publish_mission_clock_witness_once(path,witness,status):
    """One bounded diagnostic publication; never replace an existing first witness."""
    path=Path(path);temporary=path.with_name(path.name+'.first.tmp');created=False
    try:
        raw=(json.dumps(witness,ensure_ascii=False,allow_nan=False)+'\n').encode('utf-8')
        if len(raw)>=MISSION_WITNESS_DOCUMENT_LIMIT:raise ValueError('mission_witness_document_overflow')
        fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600);created=True
        with os.fdopen(fd,'wb')as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
        try:os.link(temporary,path,follow_symlinks=False)
        except FileExistsError:status['publication']='existing_first_not_replaced';return
        directory=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(directory)
        finally:os.close(directory)
        status['publication']='durably_published'
    except Exception as error:status['publication']='failed';status['error']=_mission_witness_error(error,512)[0]
    finally:
        if created:
            try:temporary.unlink()
            except OSError:pass


def async_qualification_enabled(profile):
    value=profile.get('r85_completed_history_async_qualification',{'enabled':False})
    if not isinstance(value,dict) or set(value)!={'enabled'} or type(value.get('enabled'))is not bool:
        raise ValueError('exact_boolean_async_qualification_configuration_required')
    return value['enabled']


class NativeQualificationJob:
    """One daemon worker; only frozen data and verifier modules cross threads."""
    def __init__(self,verifier,inputs,request_id,profile,scope,handoff_source_ns,context_json):
        self.context_json=context_json;self.done=threading.Event();self.value=None;self.error=None
        self.params=(verifier,completed_scope,inputs,request_id,
            json.loads(json.dumps(profile,allow_nan=False)),json.loads(json.dumps(scope,allow_nan=False)),handoff_source_ns)
        self.thread=threading.Thread(target=self._run,daemon=True,name='r85-frozen-history-qualification')
        try:self.thread.start()
        except Exception:
            inputs.close()
            raise
    def _run(self):
        try:
            from frozen_qualification import qualify_frozen
            self.value=qualify_frozen(*self.params)
        except Exception as error:self.error=type(error).__name__+':'+str(error)
        finally:self.done.set()
    def result(self):
        if not self.done.is_set():raise RuntimeError('qualification_pending')
        if self.error is not None:raise ValueError('frozen_qualification_failed:'+self.error)
        return self.value
    def close(self):
        self.thread.join(timeout=2.)
        return dict(done=self.done.is_set(),thread_alive=self.thread.is_alive(),error=self.error,
            result_available=self.done.is_set() and self.error is None,
            context_sha256=hashlib.sha256(self.context_json.encode()).hexdigest(),
            qualification_committed_by_shutdown=False)


def native_observation_spool_enabled(profile):
    value=profile.get('r85_native_observation_spool',{'enabled':False})
    if not isinstance(value,dict) or set(value)!={'enabled'} or type(value.get('enabled'))is not bool:
        raise ValueError('exact_boolean_native_observation_spool_configuration_required')
    return value['enabled']


def observe_only(profile):
    value=profile.get('diagnostic_observe_only')
    return isinstance(value,dict) and value.get('enabled') is True



def frozen(scope, path):
    path=Path(path).resolve(); actual=sha(path)
    if scope['references'].get(str(path))!=actual:
        raise ValueError('unfrozen_R85_dynamic_input:'+str(path))
    return actual


def target_at(elapsed_s):
    if not math.isfinite(elapsed_s) or elapsed_s < 0: raise ValueError('obstacle_clock_reversed_or_nonfinite')
    if elapsed_s < 8.4: return [17.-.25*elapsed_s,5.7,1.8], 'entering'
    if elapsed_s < 28.4: return list(BLOCK), 'blocking'
    if elapsed_s < 36.8: return [14.9+.25*(elapsed_s-28.4),5.7,1.8], 'leaving'
    return list(PARK), 'clear'


def progress(points, position):
    points=np.asarray(points,float); p=np.asarray(position,float)
    if points.ndim!=2 or points.shape[1]!=3 or p.shape!=(3,) or not np.isfinite(np.r_[points.ravel(),p]).all():
        raise ValueError('nonfinite_route_progress_input')
    delta=np.diff(points,axis=0); lengths=np.linalg.norm(delta,axis=1)
    denominator=np.sum(delta*delta,axis=1)
    alpha=np.divide(np.sum((p-points[:-1])*delta,axis=1),denominator,out=np.zeros(len(delta)),where=denominator>0)
    alpha=np.clip(alpha,0,1); projected=points[:-1]+alpha[:,None]*delta
    distances=np.linalg.norm(projected-p,axis=1); edge=int(np.argmin(distances))
    return {'arc_m':float(np.sum(lengths[:edge])+alpha[edge]*lengths[edge]),
        'distance_m':float(distances[edge]),'source_edge':edge,'source_alpha':float(alpha[edge]),
        'projected_body_nav_xyz':projected[edge].tolist()}


def _stamp(header):
    stamp=header.get('stamp',{})
    sec=stamp.get('sec',0); nsec=stamp.get('nsec',stamp.get('nanosec',0))
    def integer(value):
        if type(value)is int:return value
        if isinstance(value,str) and value.isascii() and value.isdigit():return int(value)
        raise ValueError('invalid_native_stamp')
    sec,nsec=integer(sec),integer(nsec)
    if sec<0 or not 0<=nsec<1_000_000_000:raise ValueError('invalid_native_stamp')
    return sec*1_000_000_000+nsec


def obstacle_observations(message):
    """Parse only exact model names in native gz.msgs.Pose_V JSON."""
    stamp=_stamp(message.get('header',{})); out=[]
    for pose in message.get('pose',[]):
        if pose.get('name')!=ENTITY: continue
        p=pose.get('position',{}); xyz=[float(p.get(k,0.)) for k in ('x','y','z')]
        if stamp<=0 or not all(math.isfinite(v) for v in xyz): raise ValueError('invalid_native_obstacle_pose')
        orientation=pose.get('orientation',{})
        q=[float(orientation.get(k,0.)) for k in ('x','y','z','w')]
        if (not all(math.isfinite(v) for v in q) or np.linalg.norm(q[:3])>1e-6
                or abs(abs(q[3])-1.)>1e-6):raise ValueError('native_obstacle_axis_alignment_changed')
        out.append(dict(entity=ENTITY,entity_id=pose.get('id'),position_world_xyz=xyz,stamp_ns=stamp,
            topic=POSE_TOPIC,native_gz_pose_info=True,source_message_type='gz.msgs.Pose_V',
            raw_model_pose=pose,header=message.get('header',{})))
    return out


class NativeObstacleObserver:
    """Owned read-only CLI child. It never publishes or reads a robot GT pose."""
    def __init__(self,*,spool_path=None):
        self.error=None; self.stopping=False;self.spool_enabled=spool_path is not None
        if self.spool_enabled:
            from native_observation_spool import NativeObservationSpool
            self.queue=NativeObservationSpool(spool_path)
        else:self.queue=queue.Queue(512)
        try:
            self.process=subprocess.Popen(['gz','topic','-e','--json-output','-t',POSE_TOPIC],
                stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,bufsize=1)
        except Exception:
            if self.spool_enabled:self.queue.close()
            raise
        self.thread=threading.Thread(target=self._read,daemon=True,name='r85-obstacle-pose-info');self.thread.start()
    def _read(self):
        decoder=json.JSONDecoder(); buffer=''
        try:
            for line in self.process.stdout:
                if self.stopping:return
                buffer+=line
                if len(buffer)>4_194_304:raise ValueError('native_pose_info_message_exceeds_4MiB')
                while buffer.strip():
                    buffer=buffer.lstrip()
                    try: message,end=decoder.raw_decode(buffer)
                    except json.JSONDecodeError:break
                    buffer=buffer[end:]
                    for value in obstacle_observations(message):
                        self.queue.put_nowait((value,time.monotonic()))
            if not self.stopping:self.error='native_pose_info_observer_exited:'+str(self.process.poll())
        except Exception as error:self.error=type(error).__name__+':'+str(error)
    def close(self):
        self.stopping=True
        if self.process.poll()is None:
            self.process.terminate()
            try:self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2.)
        self.thread.join(timeout=2.)
        if self.spool_enabled:
            if self.thread.is_alive():
                if self.error is None:self.error='native_observation_spool_reader_shutdown_incomplete'
                raise RuntimeError(self.error)
            else:self.queue.close()


def build_node(run, scope, task_path, expected_partition):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity
    from runtime_io import EvidenceWriter,latest_sensor_qos

    class Dynamic(Node):
        def __init__(self):
            super().__init__('teacher_r85_dynamic_obstacle')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            if not expected_partition or os.environ.get('GZ_PARTITION')!=expected_partition:
                raise ValueError('explicit_owned_GZ_PARTITION_required')
            self.speed_lead=speed_lead.configuration(scope['profile'])
            self.lead_window=speed_lead.lead_window(self.speed_lead)
            if self.speed_lead is not None:frozen(scope,Path(speed_lead.__file__))
            self.task_path=task_path;self.task_sha=frozen(scope,task_path);self.source_sha=frozen(scope,Path(__file__))
            self.task=json.loads(task_path.read_text())
            if (self.task.get('schema')!='r85_pct_scan_student_task/v1' or self.task.get('run_id')!=run.name
                    or self.task.get('kind')!='f1_to_f3' or self.task.get('dynamic_required')is not True):
                raise ValueError('dynamic_runtime_requires_owned_full_F3_task')
            self.typed_path=Path(self.task['source_typed_route_file']).resolve()
            if frozen(scope,self.typed_path)!=self.task['source_typed_route_sha256']:raise ValueError('typed_source_changed')
            self.typed=json.loads(self.typed_path.read_text());self.points=np.asarray(self.typed['body_points'],float)
            if self.typed.get('T_navigation_world')!=FIXED_T or self.points.shape!=(337,3):raise ValueError('unexpected_frozen_R85_route_geometry')
            self.crossing=self.points[180]+[0,0,.3]
            if not np.allclose(self.crossing,[14.9,5.7,1.52],atol=1e-6,rtol=0):raise ValueError('frozen_floor2_encounter_point_changed')
            self.encounter_arc=float(np.linalg.norm(np.diff(self.points[:181],axis=0),axis=1).sum())
            if any((run/name).exists() for name in ('r85_dynamic_journal.jsonl','r85_dynamic_pretrigger_history.jsonl')):
                raise ValueError('R85_dynamic_evidence_already_exists_no_duplicate_actor')
            self.writer=EvidenceWriter();self.sequence=0;self.presequence=0;self.binding=None;self.reference=None;self.reference_sha=None
            self.scope_selected_sim_ns=None;self.scope_start_sequence=None;self.pre_last={};self.reference_record=None
            self.state={};self.state_wall=-math.inf;self.pose=None;self.pose_wall=-math.inf;self.imu=None
            self.pending=None;self.sent=None;self.last_sim=None;self.last_send_sim=None;self.started=None;self.failure=None
            self.park_ack=False;self.clear_ack=False;self.latest_observation=None;self.observation_wall=-math.inf
            self.visible_phase='original_world_spawn_not_observed';self.ack_position=None;self.motion_elapsed=0.;self.last_motion_sim=None
            self.profile=scope['profile'];self.protocol_cancelled=False;self.diagnostic_reason=None
            self.actor_motion_contract=actor_choreography(self.profile)
            self.world_owner_stopped=False;self.world_motion_decoupled_ticks=0;self.world_motion_decoupled_s=0.
            self.event_window_enabled=event_window_enabled(self.profile)
            self.navigation_segments_enabled=navigation_segments_enabled(self.profile)
            self.coordinate_handoffs_enabled=coordinate_handoffs_enabled(self.profile)
            self.coordinate_boundary_receipts=[]
            self.event_schema=navigation_segment_schema(self.profile) if self.navigation_segments_enabled else EVENT_WINDOW
            self.navigation_boundary_sequences=[];self.navigation_status=None;self.navigation_status_wall=-math.inf
            self.dynamic_proof_seen=False;self.pending_navigation_binding=None;self.trigger_actor_identity=None

            self.event_state='waiting';self.completed_window=None;self.event_scope_ready=False;self.event_scope_changed=False
            self.event_stop_token=None;self.event_resume_ns=None;self.event_recovery_origin=None
            self.event_recovery_last_ns=None;self.event_recovery_count=0
            self.event_recovery_progress=0.;self.event_recovery_moving=False
            self._causal_intake=BoundedMissionIntake(run.name) if causal_intake_enabled(self.profile) else None
            self.diagnostic_recovery_park_ack=False;self.last_recovery_sim=None;self.observed_business_keys=set()
            self.reference_pair_update_pending=False;self.mission_age_hold=False
            self.completed_history_enabled=completed_scope.enabled(self.profile)
            self.native_spool_enabled=native_observation_spool_enabled(self.profile)
            self.async_qualification_enabled=async_qualification_enabled(self.profile)
            if self.async_qualification_enabled and not (self.native_spool_enabled and self.completed_history_enabled):
                raise ValueError('async_qualification_requires_spool_and_completed_history')
            self.history_job=None;self.history_job_handled=False
            self.current_navigation_binding=None;self.current_navigation_registration_sha=None
            self.completed_history_qualification=None;self.completed_history_qualification_sha=None
            self.completed_history_last_source_ns=None;self.completed_history_last_pose_ns=None
            self.clear_response_sim_ns=None;self.journal_logical_bytes=0;self.last_journal_record_receipt=None
            self.completed_history_first_journal_receipt=None;self.completed_history_latest_journal_receipt=None
            if self.completed_history_enabled:frozen(scope,Path(completed_scope.__file__))
            if self.speed_lead is not None:
                self.writer.atomic(run/'r117_dynamic_speed_lead_loaded.json',dict(
                    schema='r117_actual_dynamic_speed_lead_loaded/v1',run_id=run.name,
                    pid=os.getpid(),monotonic_wall_ns=time.monotonic_ns(),
                    runtime=dict(path=str(Path(__file__).resolve()),sha256=self.source_sha),
                    helper=dict(path=str(Path(speed_lead.__file__).resolve()),
                        sha256=frozen(scope,Path(speed_lead.__file__))),
                    selection=self.speed_lead,lead_window_m=list(self.lead_window),
                    before_dynamic_subscriptions=True,actual_trigger_observed=False,
                    physical_stopping_bound_claimed=False))
            self.client=self.create_client(SetEntityPose,SERVICE);self.observer=None;self.observer_attempted=False
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub=self.create_publisher(String,'/demo/obstacle/state',qos)
            self.create_subscription(String,'/demo/mission/state',self.mission,10)
            self.create_subscription(String,'/demo/navigation/status',self.status,10)
            self.create_subscription(Odometry,'/demo/slam/body_odom',self.odom,latest_sensor_qos())
            self.create_subscription(Imu,'/livox/imu',self.on_imu,latest_sensor_qos())
            self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def ns(self):return int(self.get_clock().now().nanoseconds)
        def record(self,kind,data):
            if self.binding is None:return
            if self.event_window_enabled and self.event_state=='completed_pending_verification':return
            pretrigger=self.started is None
            record_binding=self.binding
            if self.navigation_segments_enabled and not self.coordinate_handoffs_enabled and not pretrigger and kind in NAVIGATION_KINDS:
                record_binding=self.current_navigation_binding or self.binding
            if pretrigger:self.presequence+=1;sequence=self.presequence
            else:self.sequence+=1;sequence=self.sequence
            try:
                if self.completed_history_enabled or self.coordinate_handoffs_enabled:
                    path=run/('r85_dynamic_pretrigger_history.jsonl' if pretrigger else 'r85_dynamic_journal.jsonl')
                    event=dict(schema='r85_dynamic_pretrigger_record/v1' if pretrigger else 'r85_dynamic_obstacle_record/v1',kind=kind,
                        sequence=sequence,binding=dict(record_binding),sim_ns=self.ns(),monotonic_wall=time.monotonic(),
                        source='actual_R85_runtime_callback_or_owned_service',data=data)
                    if self.coordinate_handoffs_enabled and kind in NAVIGATION_KINDS:
                        event['navigation_source_identity']=coordinate_navigation_source(kind,data)
                    self.writer.append(path,event)
                    if (self.completed_history_enabled or self.coordinate_handoffs_enabled) and not pretrigger:
                        # Exact EvidenceWriter serialization; its FIFO atomic state
                        # follows this append. Owner must verify the real line SHA.
                        raw=(json.dumps(event,allow_nan=False,ensure_ascii=False)+'\n').encode('utf-8')
                        if len(raw)>2*1024**2:raise ValueError('dynamic_original_source_line_budget')
                        self.last_journal_record_receipt=dict(path=str(path),source_offset=self.journal_logical_bytes,
                            source_length=len(raw),source_line_sha256=hashlib.sha256(raw).hexdigest())
                        self.journal_logical_bytes+=len(raw)
                else:
                    self.writer.append(run/('r85_dynamic_pretrigger_history.jsonl' if pretrigger else 'r85_dynamic_journal.jsonl'),
                        dict(schema='r85_dynamic_pretrigger_record/v1' if pretrigger else 'r85_dynamic_obstacle_record/v1',kind=kind,
                        sequence=sequence,binding=dict(record_binding),sim_ns=self.ns(),monotonic_wall=time.monotonic(),
                        source='actual_R85_runtime_callback_or_owned_service',data=data))
                if pretrigger:self.pre_last[kind]=sequence
                return sequence
            except Exception as error:
                self.failure='dynamic_evidence_error:'+type(error).__name__+':'+str(error)
        def freeze_contract(self):
            p=Path(self.reference_record['reference_file']);registration=Path(self.reference_record['registration_file'])
            if self.completed_history_enabled:
                p,registration=completed_scope.snapshot_trigger_inputs(run,self.reference_record)
            arming=dict(reference_binding=self.scope_start_sequence,actual_pose=self.pre_last['actual_pose'],
                mission_state=self.pre_last['mission_state'],entity_observation=self.pre_last['entity_observation'],
                park_service_ack=self.pre_last['service_ack'])
            contract=dict(schema='r85_dynamic_task_contract/v1',binding=dict(self.binding),entity=ENTITY,service=SERVICE,
                entity_pose_topic=POSE_TOPIC,obstacle_size_m=SIZE,T_navigation_world=FIXED_T,
                phase_positions_world_xyz=dict(entering=PARK,blocking=BLOCK,leaving=PARK,clear=PARK),
                protocol='r85_f1_f3_fixed_crossing/v1',
                protocol_parameters=dict(speed_mps=.25,enter_s=8.4,blocking_s=20.,leave_s=8.4,retreat_same_side=True),
                trigger=dict(reference_file=str(p),reference_sha256=self.reference_sha,encounter_source_point_index=180,
                    encounter_body_world_xyz=self.crossing.tolist(),encounter_arc_m=self.encounter_arc,
                    lead_window_m=list(self.lead_window),maximum_path_distance_m=.3,maximum_source_age_s=.3),
                task_contract_file=str(task_path),task_contract_sha256=self.task_sha,typed_source_file=str(self.typed_path),
                typed_source_sha256=self.task['source_typed_route_sha256'],runtime_source_file=str(Path(__file__).resolve()),
                runtime_source_sha256=self.source_sha,reference_file=str(p),reference_sha256=self.reference_sha,
                registration_file=str(registration),registration_sha256=self.registration_sha,
                scope_locked_at_trigger=True,trigger_sim_ns=self.started,
                pretrigger_history_file=str(run/'r85_dynamic_pretrigger_history.jsonl'),
                arming_pretrigger_sequences=arming,generated_runtime_evidence_not_preexisting_approval=True,
                privileged_actor_pose=True,navigation_ground_truth_used=False,robot_pose_service_called=False)
            if self.actor_motion_contract is not None:
                contract.update(protocol=ACTOR_PROTOCOL,actor_motion_contract=dict(self.actor_motion_contract))
            if self.event_window_enabled:
                contract.update(event_lifetime_contract=self.event_schema,whole_task_single_scope_claimed=False,
                    completion_is_business_pending_offline_original_nine_gate=True)
            if self.navigation_segments_enabled:contract.update(navigation_segment_policy=navigation_segment_policy(self.profile),
                completion_is_business_pending_offline_original_nine_gate=False,
                completion_is_business_pending_offline_segment_identity_and_original_numeric_physical_gates=True)
            if self.speed_lead is not None:contract[speed_lead.KEY]=self.speed_lead
            if self.completed_history_enabled:
                contract['completed_history_scope_policy']=completed_scope.POLICY
            self.writer.atomic(run/'r85_dynamic_task_contract.json',contract)
            self.record('reference_lock',dict(self.reference_record,scope_locked_at_trigger=True,
                arming_pretrigger_sequences=arming,pretrigger_records_migrated=False))
        def validated_reference(self,binding,reference_file):
            if (binding.get('run_id')!=run.name or binding.get('request_id')!=run.name+':r85:f1_to_f3:1'
                    or any(type(binding.get(key))is not int or binding[key]<1 for key in ('epoch','localization_generation'))):
                raise ValueError('foreign_request_or_invalid_actual_reference_identity_types')
            p=Path(reference_file).resolve();raw=p.read_bytes();doc=json.loads(raw)
            from automatic_reference_runtime import reference_version_paths
            version,registration_version=reference_version_paths(run,binding['epoch'],binding['localization_generation'])
            if p not in (run/'r85_global_reference.json',version) or hashlib.sha256(raw).hexdigest()!=binding['reference_sha256']:
                raise ValueError('actual_global_reference_file_binding_mismatch')
            if (doc.get('run_id')!=run.name or doc.get('reference_id')!=binding['reference_id']
                    or doc.get('reference_scope')!='full_route' or doc.get('epoch')!=binding['epoch']
                    or doc.get('localization_generation')!=binding['localization_generation']
                    or doc.get('source_route_sha256')!=self.task['source_typed_route_sha256']
                    or binding['source_route_sha256']!=self.task['source_typed_route_sha256']
                    or doc.get('T_navigation_world')!=FIXED_T
                    or not np.array_equal(np.asarray(doc['body_points'],float),self.points)):
                raise ValueError('actual_full_reference_does_not_match_bound_typed_source')
            registration=registration_version if p==version else run/'r85_reference_registration.json'
            registered=json.loads(registration.read_text())
            if (registered.get('schema')!='r85_reference_source_registration/v1'
                    or registered.get('run_id')!=run.name or registered.get('reference_file_sha256')!=binding['reference_sha256']
                    or registered.get('reference_id')!=binding['reference_id']
                    or registered.get('source_route_sha256')!=binding['source_route_sha256']
                    or registered.get('localization_generation')!=binding['localization_generation']
                    or registered.get('navigation_ground_truth_used')is not False
                    or registered.get('producer')!='root_authorized_PCT_route_owner'
                    or Path(registered.get('task_source_file','')).resolve()!=task_path
                    or registered.get('task_source_sha256')!=self.task_sha
                    or frozen(scope,registered['owner_source_file'])!=registered.get('owner_source_sha256')):
                raise ValueError('actual_reference_source_registration_mismatch')
            return p,raw,doc,registration,registered
        def switch_navigation_segment(self,binding,p,raw,doc,registration,registered,clock):
            coordinate=coordinate_handoffs_enabled(self.profile)
            if ((coordinate and (self.event_state!='active' or len(self.navigation_boundary_sequences)>=2))
                    or (not coordinate and (self.dynamic_proof_seen or self.event_stop_token is not None or self.event_resume_ns is not None or self.navigation_boundary_sequences))):
                raise ValueError('actual_reference_scope_changed_during_dynamic_protocol')
            releases=[]
            with (run/'known_scene_transaction_releases.jsonl').open() as stream:
                for line in stream:
                    if len(line)>65536:raise ValueError('navigation_release_line_bound')
                    value=json.loads(line)
                    if value.get('generation')==binding['localization_generation']:releases.append(value)
            if len(releases)!=1:raise ValueError('navigation_segment_release_missing_or_ambiguous')
            reference=dict(reference_file=str(p),reference_sha256=binding['reference_sha256'],document=doc,
                registration_file=str(registration),registration_sha256=sha(registration),registration_document=registered,
                reference_raw_utf8=raw.decode('utf-8'),registration_raw_utf8=registration.read_text())
            old_binding=(self.current_navigation_binding or self.binding) if coordinate else self.binding
            old_reference=self.current_navigation_reference if coordinate else self.reference
            edge=dict(schema='world_event_coordinate_handoff_edge/v1' if coordinate else 'world_event_pre_stop_navigation_edge/v1',
                switch_index=len(self.navigation_boundary_sequences)+1 if coordinate else 1,
                old_binding=dict(old_binding),new_binding=dict(binding),old_reference=old_reference,new_reference=reference,
                cut_sim_ns=clock,boundary_wall=time.monotonic(),dynamic_proof_seen=self.dynamic_proof_seen,
                navigation_status=self.navigation_status,status_received_wall=self.navigation_status_wall,
                transaction_release=releases[0],actor_observation=self.latest_observation,
                actor_received_wall=self.observation_wall,last_actor_ACK_position=self.ack_position,
                trigger_actor_identity=self.trigger_actor_identity,world_event_trigger_sim_ns=self.started)
            if coordinate:
                edge.update(event_state=self.event_state,event_stop_token=self.event_stop_token,event_resume_ns=self.event_resume_ns)
                validate_coordinate_handoff_edge(edge)
            else:validate_navigation_segment_edge(edge)
            sequence=self.record('navigation_segment_boundary',edge)
            if sequence is None or self.failure is not None:raise ValueError('navigation_segment_boundary_not_recorded')
            self.navigation_boundary_sequences.append(sequence);self.current_navigation_binding=dict(binding)
            if coordinate:
                self.current_navigation_reference=doc
                self.coordinate_boundary_receipts.append(dict(self.last_journal_record_receipt))
                # The world stop/resume facts stay immutable, but coordinate
                # correction displacement is never physical recovery progress.
                self.event_recovery_origin=None;self.event_recovery_last_ns=None
                self.event_recovery_count=0;self.event_recovery_progress=0.;self.event_recovery_moving=False
            self.scope_selected_sim_ns=clock;self.event_scope_ready=False;self.event_scope_changed=False
            self.pending_navigation_binding=None

        def cancel_business_protocol(self,reason,evidence):
            if not observe_only(self.profile):return False
            key=('dynamic_protocol',reason)
            if key not in self.observed_business_keys:
                from diagnostic_observation import record
                record(run,self.profile,'dynamic_protocol',reason,dict(evidence,run_id=run.name,
                    observed_sim_ns=self.ns(),original_would_stop=True,dynamic_PASS=False,
                    navigation_PASS=False,formal_full_route_pass=False))
                self.observed_business_keys.add(key)
                self.record('diagnostic_protocol_cancelled',dict(reason=reason,evidence=evidence,
                    dynamic_PASS=False,actor_chases_robot=False,rearm_or_replay=False))
            self.protocol_cancelled=True;self.diagnostic_reason=reason
            if self.event_window_enabled:self.event_state='cancelled'
            self.reference_pair_update_pending=False
            return True
        def preserve_completed_history(self,binding,p,raw,doc,registration,registered,state,clock,received_wall,*,qualification=None,from_worker=False):
            # Receive time belongs to callback entry, including a slow original
            # verifier call. Never refresh source clocks or wall freshness.
            now=time.monotonic();observation=self.latest_observation
            observation_clock=self.ns() if from_worker else clock
            if (self.failure is not None or self.protocol_cancelled or not self.clear_ack
                    or self.pending is not None or self.sent is not None or self.visible_phase!='clear'
                    or self.clear_response_sim_ns is None or observation is None
                    or not 0<=now-self.observation_wall<.3
                    or not 0<=observation_clock-observation['stamp_ns']<=300_000_000
                    or observation['stamp_ns']<self.clear_response_sim_ns
                    or np.linalg.norm(np.asarray(observation['position_world_xyz'])-PARK)>.03):
                raise ValueError('actual_reference_scope_changed_before_verified_completed_clear')
            completed_scope.same_geometry(self.reference_record['document'],
                self.reference_record['registration_document'],doc,registered)
            previous=self.current_navigation_binding or self.binding
            source=state['ros_sim_time_ns'];pose=state['source_actual_pose_stamp_ns']
            if (self.completed_history_last_source_ns is not None
                    and (source<=self.completed_history_last_source_ns or pose<self.completed_history_last_pose_ns)):
                raise ValueError('completed_handoff_actual_source_clock_not_monotonic')
            regraw=registration.read_bytes()
            if (sha(p)!=binding['reference_sha256'] or hashlib.sha256(regraw).hexdigest()!=sha(registration)
                    or completed_scope.canonical(json.loads(regraw))!=completed_scope.canonical(registered)):
                raise ValueError('completed_handoff_reference_pair_changed_during_read')
            first=self.completed_history_qualification is None
            if first:
                if self.async_qualification_enabled and qualification is None:
                    if self.history_job is not None:
                        expected=json.loads(self.history_job.context_json)
                        if binding!=expected['binding'] or previous!=expected['previous']:
                            raise ValueError('pending_qualification_target_or_anchor_changed')
                        return
                    import r85_dynamic_obstacle_verifier as verifier
                    from frozen_qualification import freeze_inputs
                    inputs=freeze_inputs(verifier,run)
                    try:
                        context=dict(binding=binding,previous=previous,anchor=self.binding,p=str(p),raw_utf8=raw.decode('utf-8'),doc=doc,
                            registration=str(registration),registration_raw_utf8=regraw.decode('utf-8'),registered=registered,state=state,clock=clock,received_wall=received_wall)
                        self.history_job=NativeQualificationJob(verifier,inputs,self.binding['request_id'],self.profile,scope,
                            source,json.dumps(context,allow_nan=False,ensure_ascii=False))
                    except BaseException:
                        inputs.close()
                        raise
                    self.reference_pair_update_pending=True
                    return
                if qualification is None:
                    import r85_dynamic_obstacle_verifier as verifier
                    qualification=completed_scope.qualify_history_retention(verifier,run,
                        self.binding['request_id'],self.profile,scope,source)
            else:
                qualification=self.completed_history_qualification
                if completed_scope.sha(completed_scope.canonical(qualification).encode())!=self.completed_history_qualification_sha:
                    raise ValueError('completed_history_qualification_changed')
            if self.failure is not None or self.protocol_cancelled or self.pending is not None:
                raise ValueError('completed_history_cancelled_or_uncertain_cannot_revive')
            if (p.read_bytes()!=raw or registration.read_bytes()!=regraw):
                raise ValueError('completed_handoff_reference_pair_changed_during_qualification')
            qsha=completed_scope.sha(completed_scope.canonical(qualification).encode())
            data=dict(previous_navigation_binding=dict(previous),current_navigation_binding=dict(binding),
                handoff_source_ns=source,source_actual_pose_stamp_ns=pose,mission_received_sim_ns=clock,
                mission_received_monotonic_wall=received_wall,mission_message=state,
                current_reference_sha256=binding['reference_sha256'],current_registration_sha256=hashlib.sha256(regraw).hexdigest(),
                current_reference_document=doc,current_registration_document=registered,
                current_reference_raw_utf8=raw.decode('utf-8'),current_registration_raw_utf8=regraw.decode('utf-8'),
                qualification_sha256=qsha,history_retention_only=True,dynamic_PASS=False,navigation_PASS=False)
            if first:data['qualification']=qualification
            sequence=self.record('completed_history_scope_preserved',data)
            if sequence is None or self.failure is not None:raise ValueError('completed_history_record_failed')
            if first:self.completed_history_first_journal_receipt=dict(self.last_journal_record_receipt)
            self.completed_history_latest_journal_receipt=dict(self.last_journal_record_receipt)
            self.completed_history_qualification=qualification;self.completed_history_qualification_sha=qsha
            self.completed_history_last_source_ns=source;self.completed_history_last_pose_ns=pose
            self.current_navigation_binding=dict(binding);self.current_navigation_registration_sha=data['current_registration_sha256']
            self.reference_pair_update_pending=False;self.mission_age_hold=False
            self.state=state;self.state_wall=(received_wall if self.completed_history_enabled else time.monotonic());self.record('mission_state',state)
        def complete_history_qualification(self):
            job=self.history_job
            if job is None or self.history_job_handled or not job.done.is_set():return
            self.history_job_handled=True
            try:
                context=json.loads(job.context_json)
                if (self.failure is not None or self.protocol_cancelled
                        or self.binding!=context['anchor']
                        or (self.current_navigation_binding or self.binding)!=context['previous']):
                    raise ValueError('completed_qualification_cancelled_failed_or_anchor_changed')
                p=Path(context['p']);registration=Path(context['registration'])
                raw=context['raw_utf8'].encode('utf-8')
                if p.read_bytes()!=raw or registration.read_bytes()!=context['registration_raw_utf8'].encode('utf-8'):
                    raise ValueError('completed_qualification_target_pair_changed')
                self.preserve_completed_history(context['binding'],p,raw,context['doc'],registration,
                    context['registered'],context['state'],context['clock'],context['received_wall'],
                    qualification=job.result(),from_worker=True)
            except Exception as error:
                if self.failure is not None or self.protocol_cancelled:return
                if not self.cancel_business_protocol('actual_reference_scope_changed_during_dynamic_protocol',
                        dict(qualification_rejected=type(error).__name__+':'+str(error),
                            asynchronous_qualification=True,context_sha256=hashlib.sha256(job.context_json.encode()).hexdigest())):
                    self.failure=type(error).__name__+':'+str(error)
        def _causal_intake_fail(self,error):
            if self.failure is not None:return
            reason=str(error)
            if not reason.startswith('r74_dynamic_causal_intake_'):
                reason='r74_dynamic_causal_intake_internal_error:'+type(error).__name__+':'+reason
            self.failure=type(error).__name__+':'+reason
            if self._causal_intake.failure is None:
                self._causal_intake.failure=reason
                self._causal_intake.counts['failures']+=1
        def _drain_causal_intake(self,clock,wall):
            intake=self._causal_intake;had_pending=intake.pending;motion_sample=None
            try:
                if self.failure is None:
                    intake.check(clock,wall)
                    while intake.pending and self.failure is None:
                        item=intake.pop_ready(self.ns(),time.monotonic())
                        if item is None:break
                        self._mission_body(item,_causal_envelope=item)
                if had_pending:
                    motion_clock=self.ns();motion_wall=time.monotonic()
                    if self.failure is None:intake.check(motion_clock,motion_wall)
                    if self.last_motion_sim is not None:self.last_motion_sim=motion_clock
                    motion_sample=(motion_clock,motion_wall)
            except Exception as error:self._causal_intake_fail(error)
            return motion_sample
        def mission(self,msg):
            if self.event_window_enabled and self.event_state=='completed_pending_verification':return
            if self._causal_intake is None:return self._mission_body(msg)
            if self.failure is not None:return
            received_wall=time.monotonic()
            try:
                arrival_clock=self.ns();self._causal_intake.check(arrival_clock,received_wall)
                targets=causal_clock_targets(msg.data,run.name)
                if targets is None:return self._mission_body(msg)
                # A legal later fresh message joins an existing FIFO before any
                # drain, retaining this callback's actual entry receipt/clock.
                if self._causal_intake.pending or any(stamp>arrival_clock for stamp in targets):
                    item=causal_envelope(msg.data,received_wall,arrival_clock,targets)
                    self._causal_intake.enqueue(item,arrival_clock,received_wall)
                    return
                return self._mission_body(msg)
            except Exception as error:self._causal_intake_fail(error)
        def _mission_body(self,msg,*,_causal_envelope=None):
            if self.event_window_enabled and self.event_state=='completed_pending_verification':return
            received_wall=time.monotonic() if _causal_envelope is None else _causal_envelope.received_wall
            witness_state=None;witness_binding=None;witness_clock=None;witness_validation_wall=None;witness_phase='parse_payload'
            try:witness_entry_clock=self.ns() if _causal_envelope is None else _causal_envelope.arrival_clock_ns
            except Exception:witness_entry_clock=None
            try:
                state=json.loads(msg.data);witness_state=state;witness_phase='eligibility';ref=state.get('r85_reference',{})
                if state.get('run_id')!=run.name:return
                if state.get('stage')!='navigating' or state.get('navigation_ground_truth_used')is not False:return
                if ref.get('reference_scope')!='full_route':return
                clock=self.ns();witness_clock=clock;witness_validation_wall=_mission_witness_wall();witness_phase='schema_types_and_clock_causality'
                if _causal_envelope is not None:self._causal_intake.note_validation(_causal_envelope,clock)
                if (state.get('schema')!='r85_actual_single_task_mission_state/v1'
                        or type(state.get('ros_sim_time_ns'))is not int
                        or type(state.get('source_actual_pose_stamp_ns'))is not int
                        or not 0<=state['ros_sim_time_ns']<=clock
                        or not 0<=state['source_actual_pose_stamp_ns']<=clock):
                    raise ValueError('actual_mission_source_clock_stale_or_malformed')
                witness_phase='binding_identity'
                binding=dict(run_id=run.name,request_id=state['current_request'],reference_id=ref['reference_id'],
                    reference_sha256=ref['global_reference_sha256'],epoch=ref['epoch'],localization_generation=ref['localization_generation'],
                    source_route_sha256=ref['source_route_sha256'])
                witness_binding=binding
                if (binding['request_id']!=run.name+':r85:f1_to_f3:1'
                        or any(type(binding[key])is not int or binding[key]<1 for key in ('epoch','localization_generation'))):
                    raise ValueError('foreign_request_or_invalid_actual_reference_identity_types')
                witness_phase='source_age'
                stale=(clock-state['ros_sim_time_ns']>300_000_000
                    or clock-state['source_actual_pose_stamp_ns']>300_000_000)
                if stale:
                    # Only a same-scope owned message with legal nonfuture
                    # integer clocks is an optional choreography-age veto.
                    # Keep the actual old stamps and freeze all new mutations
                    # until a new fresh, validated mission callback arrives.
                    if (observe_only(self.profile) and self.binding is not None
                            and all(type(binding.get(k))is type(v) and binding[k]==v
                                for k,v in self.binding.items())):
                        key=('dynamic_mission_age','original_mission_age_would_fail_dynamic_protocol')
                        if key not in self.observed_business_keys:
                            from diagnostic_observation import record
                            record(run,self.profile,key[0],key[1],dict(actual_mission_state=state,
                                binding=dict(self.binding),observed_sim_ns=clock,
                                mission_age_ns=clock-state['ros_sim_time_ns'],
                                pose_source_age_ns=clock-state['source_actual_pose_stamp_ns'],
                                original_age_limit_ns=300_000_000,original_would_stop=True,
                                choreography_frozen=True,dynamic_PASS=False,navigation_PASS=False))
                            self.observed_business_keys.add(key)
                        self.mission_age_hold=True
                        return
                    raise ValueError('actual_mission_source_clock_stale_or_malformed')
                witness_phase='reference_scope_and_acceptance'
                previous=(self.current_navigation_binding or self.binding) if (self.completed_history_enabled or self.navigation_segments_enabled) else self.binding
                changed=previous is not None and binding!=previous
                if changed and (binding['epoch']<=previous['epoch']
                        or binding['localization_generation']<previous['localization_generation']):
                    raise ValueError('pretrigger_reference_scope_did_not_advance_actual_planner_epoch')
                if changed and self.started is not None and self.navigation_segments_enabled:
                    if (((not coordinate_handoffs_enabled(self.profile)) and (self.dynamic_proof_seen or self.navigation_boundary_sequences))
                            or (coordinate_handoffs_enabled(self.profile) and len(self.navigation_boundary_sequences)>=2)
                            or any(binding.get(k)!=self.binding.get(k) for k in ('run_id','request_id','source_route_sha256'))):
                        raise ValueError('actual_reference_scope_changed_during_dynamic_protocol')
                    if self.pending_navigation_binding is not None and self.pending_navigation_binding!=binding:
                        raise ValueError('multiple_pending_navigation_scopes')
                    self.pending_navigation_binding=dict(binding)
                    if not navigation_segment_ready(self.navigation_status,binding,clock,time.monotonic(),self.navigation_status_wall):return
                    p,raw,doc,registration,registered=self.validated_reference(binding,ref['global_reference_file'])
                    self.switch_navigation_segment(binding,p,raw,doc,registration,registered,clock)
                    self.mission_age_hold=False;self.state=state;self.state_wall=received_wall
                    self.record('mission_state',state);return
                if self.binding is None or changed:
                    p,raw,doc,registration,registered=self.validated_reference(binding,ref['global_reference_file'])
                    if changed and self.started is not None:
                        if self.completed_history_enabled:
                            try:
                                self.preserve_completed_history(binding,p,raw,doc,registration,registered,state,clock,received_wall)
                                return
                            except Exception as qualification_error:
                                if self.cancel_business_protocol('actual_reference_scope_changed_during_dynamic_protocol',
                                        dict(previous_binding=dict(self.binding),new_binding=binding,
                                            qualification_rejected=type(qualification_error).__name__+':'+str(qualification_error),
                                            new_reference_sha256=sha(p),new_registration_sha256=sha(registration))):
                                    self.mission_age_hold=False;self.state=state;self.state_wall=received_wall;return
                                raise
                        if self.cancel_business_protocol('actual_reference_scope_changed_during_dynamic_protocol',
                                dict(previous_binding=dict(self.binding),new_binding=binding,
                                    new_reference_sha256=sha(p),new_registration_sha256=sha(registration))):
                            self.mission_age_hold=False;self.state=state;self.state_wall=(received_wall if _causal_envelope is not None or self.completed_history_enabled else time.monotonic());return
                        raise ValueError('actual_reference_scope_changed_during_dynamic_protocol')
                    if self.protocol_cancelled:
                        self.mission_age_hold=False;self.state=state;self.state_wall=(received_wall if _causal_envelope is not None or self.completed_history_enabled else time.monotonic());return
                    # Drain an already sent old-scope park request first. Its
                    # outcome belongs to the old monitoring history, never to
                    # the new scope's arming evidence.
                    if changed and self.pending is not None:return
                    self.binding=binding;self.reference=doc;self.reference_sha=binding['reference_sha256']
                    self.current_navigation_reference=doc
                    self.registration_sha=sha(registration)
                    self.scope_selected_sim_ns=self.ns();self.park_ack=False;self.pre_last={}
                    self.latest_observation=None;self.observation_wall=-math.inf;self.pose=None;self.pose_wall=-math.inf
                    self.reference_record=dict(reference_file=str(p),reference_sha256=self.reference_sha,
                        document=doc,registration_file=str(registration),registration_sha256=self.registration_sha,
                        registration_document=registered,task_file=str(task_path),task_sha256=self.task_sha,
                        typed_source_file=str(self.typed_path),typed_source_sha256=self.task['source_typed_route_sha256'],
                        reference_raw_utf8=raw.decode('utf-8'),registration_raw_utf8=registration.read_text())
                    self.scope_start_sequence=self.record('reference_binding',self.reference_record)
                self.mission_age_hold=False;self.state=state;self.state_wall=(received_wall if _causal_envelope is not None or self.completed_history_enabled else time.monotonic());self.record('mission_state',state)
                if (self.actor_motion_contract is not None and self.started is not None
                        and type(state.get('task_navigation_state'))is str
                        and state['task_navigation_state'] in ('failed','stopped','succeeded')):
                    self.world_owner_stopped=True
            except Exception as error:
                self.failure=type(error).__name__+':'+str(error)
                witness_exception_wall=_mission_witness_wall()
                if not getattr(self,'mission_failure_witness_attempted',False):
                    self.mission_failure_witness_attempted=True
                    try:
                        witness=_mission_clock_failure_witness(msg.data,run.name,witness_entry_clock,received_wall,
                            witness_clock,witness_state,witness_binding,self.binding,witness_phase,error,
                            validation_wall=witness_validation_wall,exception_wall=witness_exception_wall)
                        self.mission_failure_witness=witness
                        self.mission_failure_witness_status=dict(publication='pending',error=None)
                        _publish_mission_clock_witness_once(run/'r85_dynamic_mission_first_failure.json',
                            witness,self.mission_failure_witness_status)
                    except Exception as witness_error:
                        self.mission_failure_witness_status=dict(publication='failed',error=_mission_witness_error(witness_error,512)[0])
        def status(self,msg):
            try:
                data=json.loads(msg.data)
                if self.binding and data.get('request_id')==self.binding['request_id']:
                    self.navigation_status=data;self.navigation_status_wall=time.monotonic()
                    # Actual status has a run-bound request_id, not a run_id field.
                    if (self.actor_motion_contract is not None and self.started is not None
                            and self.binding.get('run_id')==run.name
                            and self.binding['request_id']==run.name+':r85:f1_to_f3:1'
                            and data.get('run_id',run.name)==run.name
                            and type(data.get('state'))is str and data['state'] in ('failed','stopped','succeeded')):
                        self.world_owner_stopped=True
                    if (not self.coordinate_handoffs_enabled or coordinate_status_matches(data,self.current_navigation_binding or self.binding)) and self.started is not None and (data.get('obstacle_hold')is True or data.get('dynamic_stop_event')is not None
                            or any(type(data.get(k))is int and data[k]>0 for k in ('obstacle_stops','obstacle_resumes','aligned_obstacle_resumes'))):self.dynamic_proof_seen=True
                    self.record('status',data)
                    if self.event_window_enabled and self.event_state=='active':
                        epoch=data.get('known_scene_scan_epoch')or{}
                        active_binding=(self.current_navigation_binding or self.binding) if self.navigation_segments_enabled else self.binding
                        same_scope=(epoch.get('planner_epoch')==active_binding['epoch']
                            and epoch.get('localization_generation')==active_binding['localization_generation']
                            and epoch.get('ready')is True)
                        if ((type(epoch.get('planner_epoch'))is int and epoch['planner_epoch']>active_binding['epoch'])
                                or (type(epoch.get('localization_generation'))is int and epoch['localization_generation']>active_binding['localization_generation'])):
                            if not self.navigation_segments_enabled:self.event_scope_changed=True
                            # A child epoch, even ready, is preparation rather than
                            # publication of a new route definition. Keep this event
                            # unready without moving its binding or granting evidence.
                            # v2 rejects post-stop changes; v3 admits only an explicit
                            # certified edge, retaining old token/ACK history identities.
                        if self.coordinate_handoffs_enabled:same_scope=same_scope and coordinate_status_matches(data,active_binding)
                        self.event_scope_ready=same_scope and not self.event_scope_changed
                        if not same_scope:return
                        token=data.get('dynamic_stop_event')
                        if (data.get('obstacle_hold')is True and isinstance(token,dict)
                                and data.get('command')==[0.,0.,0.]):
                            self.event_stop_token=dict(token);self.event_resume_ns=None
                            self.event_recovery_origin=None;self.event_recovery_last_ns=None
                            self.event_recovery_count=0;self.event_recovery_progress=0.;self.event_recovery_moving=False
                        elif (self.event_stop_token is not None and token==self.event_stop_token
                                and data.get('obstacle_hold')is False
                                and type(data.get('obstacle_resumes'))is int and data['obstacle_resumes']>0
                                and type(data.get('aligned_obstacle_resumes'))is int and data['aligned_obstacle_resumes']>0
                                and self.event_resume_ns is None):self.event_resume_ns=self.ns()
            except Exception as error:self.failure=type(error).__name__+':'+str(error)
        def on_imu(self,msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            self.imu=dict(stamp_ns=int(stamp),gyro=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        def odom(self,msg):
            ns=self.ns();stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec;p=msg.pose.pose.position
            v=msg.twist.twist.linear;w=msg.twist.twist.angular
            if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body' or not 0<=ns-stamp<=300_000_000
                    or not all(math.isfinite(x) for x in (p.x,p.y,p.z,v.x,v.y,v.z,w.x,w.y,w.z))):return
            self.pose=dict(position=[p.x,p.y,p.z],stamp_ns=int(stamp),frame_id=msg.header.frame_id,child_frame_id=msg.child_frame_id,
                source_topic='/demo/slam/body_odom',body_velocity=[v.x,v.y,v.z],body_angular_velocity=[w.x,w.y,w.z],
                body_gyro=None if self.imu is None else self.imu['gyro'],imu_stamp_ns=None if self.imu is None else self.imu['stamp_ns'],
                navigation_ground_truth_used=False)
            if self.completed_history_enabled or self.coordinate_handoffs_enabled:
                from point_provenance import message_sha
                self.pose['corrected_body_message_sha256']=message_sha(msg)
                self.pose['corrected_body_message_fingerprint']='go2_ros_semantic/v1:Odometry'
            self.pose_wall=time.monotonic();self.record('actual_pose',self.pose)
            if (self.event_window_enabled and self.event_state=='active' and self.event_resume_ns is not None
                    and stamp>=self.event_resume_ns and (not self.coordinate_handoffs_enabled or stamp>=self.scope_selected_sim_ns)
                    and (self.event_recovery_last_ns is None or stamp>self.event_recovery_last_ns)):
                if self.event_recovery_origin is None:self.event_recovery_origin=dict(self.pose)
                self.event_recovery_last_ns=stamp;self.event_recovery_count+=1
                self.event_recovery_progress=max(self.event_recovery_progress,
                    math.hypot(p.x-self.event_recovery_origin['position'][0],p.y-self.event_recovery_origin['position'][1]))
                self.event_recovery_moving=self.event_recovery_moving or math.hypot(v.x,v.y)>.025

        def complete_event_window(self,ns,*,fresh,observed,parked):
            if not (self.event_window_enabled and self.event_state=='active' and self.failure is None
                    and not self.protocol_cancelled and self.event_scope_ready and fresh and observed and parked and self.clear_ack
                    and self.pending is None and self.sent is None and self.clear_response_sim_ns is not None
                    and self.latest_observation['stamp_ns']>=self.clear_response_sim_ns
                    and self.event_stop_token is not None and self.event_resume_ns is not None
                    and self.event_recovery_count>=3 and self.event_recovery_moving and self.event_recovery_progress>=.08):return
            window=dict(schema=self.event_schema,binding=dict(self.binding),started_sim_ns=self.started,
                completed_sim_ns=ns,journal_last_sequence=self.sequence+1,stop_event=dict(self.event_stop_token),
                resume_sim_ns=self.event_resume_ns,recovery_origin_source_ns=self.event_recovery_origin['stamp_ns'],
                recovery_end_source_ns=self.event_recovery_last_ns,recovery_progress_m=self.event_recovery_progress,
                independent_park_source_ns=self.latest_observation['stamp_ns'],clear_response_sim_ns=self.clear_response_sim_ns,
                business_completion_only=True,physical_verified=False,dynamic_PASS=False)
            if self.navigation_segments_enabled:window.update(navigation_binding=dict(self.current_navigation_binding or self.binding),navigation_boundary_sequences=list(self.navigation_boundary_sequences))
            sequence=self.record('event_completed_pending_verification',window)
            if sequence==window['journal_last_sequence'] and self.failure is None:
                self.completed_window=window;self.event_state='completed_pending_verification'
        def world_motion_permitted(self,*,fresh,observed,reference_current,causal_hold):
            # Legacy and arming use exactly the original navigation freshness.
            if self.actor_motion_contract is None or self.started is None:return fresh
            identity=self.trigger_actor_identity;actor=self.latest_observation
            return bool(observed and reference_current and not causal_hold
                and self.failure is None and not self.protocol_cancelled and not self.world_owner_stopped
                and self.event_state=='active' and not self.event_scope_changed
                and self.pending_navigation_binding is None and not self.reference_pair_update_pending
                and self.state.get('task_navigation_state')=='running'
                and self.park_ack and self.ack_position is not None
                and isinstance(identity,dict) and isinstance(actor,dict)
                and actor.get('native_gz_pose_info')is True
                and actor.get('entity')==identity.get('entity')==ENTITY
                and type(actor.get('entity_id'))is int and actor['entity_id']==identity.get('entity_id'))

        def tick(self):
            ns=self.ns();now=time.monotonic()
            if self.writer.error:self.failure=self.writer.error
            # The navigation stack is launched before Gazebo and its clock.
            # Do not run gz topic against a world that has not started yet.
            if (not self.observer_attempted and self.failure is None and ns>0
                    and self.client.service_is_ready()):
                self.observer_attempted=True
                try:
                    if self.native_spool_enabled:
                        self.observer=NativeObstacleObserver(spool_path=run/'r85_native_observation_spool.jsonl')
                    else:self.observer=NativeObstacleObserver()
                except Exception as error:self.failure='native_observer_start_failed:'+type(error).__name__+':'+str(error)
            if self.observer is not None and self.observer.error:self.failure=self.observer.error
            if self.last_sim is not None and ns<self.last_sim:self.failure='actual_dynamic_sim_clock_moved_backward'
            self.last_sim=ns
            while self.observer is not None:
                try: observation,received_wall=self.observer.queue.get_nowait()
                except queue.Empty:break
                except Exception as error:
                    self.failure='native_observation_spool_error:'+type(error).__name__+':'+str(error);break
                self.latest_observation=observation;self.observation_wall=received_wall;self.record('entity_observation',observation)
            if self.async_qualification_enabled:self.complete_history_qualification()
            if self.pending is not None and self.pending.done():
                try:
                    success=bool(self.pending.result().success)
                    if success:
                        self.ack_position=self.sent['position_world_xyz'];self.visible_phase=self.sent['phase']
                        if self.visible_phase=='parked':self.park_ack=True
                        if self.visible_phase=='clear':
                            self.clear_ack=True
                            self.clear_response_sim_ns=ns
                        if self.visible_phase=='diagnostic_recovery_parked':self.diagnostic_recovery_park_ack=True
                    else:self.failure='actual_SetEntityPose_service_rejected_moving_obstacle'
                    self.record('service_ack',dict(**self.sent,success=success,response_sim_ns=ns))
                except Exception as error:
                    self.failure=type(error).__name__+':'+str(error);self.record('service_ack',dict(**self.sent,success=False,error=self.failure,response_sim_ns=ns))
                self.pending=None;self.sent=None
            elif self.pending is not None and (now-self.sent['request_monotonic_wall']>2.
                    or ns-self.sent['request_sim_ns']>2_000_000_000):
                # A ROS timeout cannot prove that the Gazebo mutation did not
                # execute. Freeze further calls and preserve the uncertain target.
                if self.failure is None:
                    self.failure='SetEntityPose_response_timeout_outcome_uncertain_no_retry'
                    self.record('service_outcome_uncertain',dict(**self.sent,reason=self.failure))
            _causal_intake_hold=False;_causal_motion_clock=None
            if self._causal_intake is not None:
                _causal_sample=self._drain_causal_intake(ns,now)
                if _causal_sample is not None:
                    ns,now=_causal_sample;_causal_motion_clock=ns
                _causal_intake_hold=self._causal_intake.pending
            fresh=(self.binding is not None and self.pose is not None and 0<=now-self.pose_wall<.3
                and 0<=ns-self.pose['stamp_ns']<=300_000_000 and 0<=now-self.state_wall<.3
                and self.pose['stamp_ns']>=self.scope_selected_sim_ns
                and self.state.get('source_actual_pose_stamp_ns',-1)>=self.scope_selected_sim_ns
                and self.state.get('task_navigation_state')=='running')
            observed=(self.binding is not None and self.latest_observation is not None and 0<=now-self.observation_wall<.3
                and 0<=ns-self.latest_observation['stamp_ns']<=300_000_000
                and self.latest_observation['stamp_ns']>=self.scope_selected_sim_ns)
            parked=(observed and np.linalg.norm(np.asarray(self.latest_observation['position_world_xyz'])-PARK)<=.03)
            if self.event_window_enabled:
                # The owner installs validated program versions at the boundary;
                # fresh mission identities handle active scope changes. No
                # full static-file or route rereads in this timer.
                reference_current=self.binding is not None and self.reference_record is not None
            else:
                if self.binding and self.failure is None and (sha(task_path)!=self.task_sha or sha(self.typed_path)!=self.task['source_typed_route_sha256']):
                    self.failure='frozen_task_or_typed_route_changed'
                if self.completed_history_enabled:
                    current_binding=(self.current_navigation_binding or self.binding) if self.completed_history_enabled else self.binding
                    current_reference_sha=(current_binding['reference_sha256'] if current_binding else self.reference_sha)
                    current_registration_sha=((self.current_navigation_registration_sha or self.registration_sha) if self.completed_history_enabled else self.registration_sha) if self.binding else None
                    reference_current=(self.binding is not None and sha(run/'r85_global_reference.json')==current_reference_sha
                        and sha(run/'r85_reference_registration.json')==current_registration_sha)
                else:
                    reference_current=(self.binding is not None and sha(run/'r85_global_reference.json')==self.reference_sha
                        and sha(run/'r85_reference_registration.json')==self.registration_sha)
            if self.started is not None and not reference_current and self.failure is None and not self.protocol_cancelled:
                completed_clear=(self.completed_history_enabled and self.clear_ack and self.pending is None
                    and self.sent is None and self.visible_phase=='clear' and parked
                    and self.clear_response_sim_ns is not None
                    and self.latest_observation['stamp_ns']>=self.clear_response_sim_ns)
                if completed_clear or observe_only(self.profile):
                    # The owner replaces global and registration separately.
                    # Freeze choreography immediately on a hash change, then
                    # let the subsequent completed owner mission callback
                    # validate both documents before cancellation/retreat.
                    # Never read a half-published pair as a new valid scope.
                    self.reference_pair_update_pending=True
                else:self.failure='locked_actual_reference_or_registration_changed'
            fresh=fresh and reference_current and not self.mission_age_hold
            if self.navigation_segments_enabled and self.started is not None:fresh=fresh and self.event_scope_ready and self.pending_navigation_binding is None
            fresh=fresh and not _causal_intake_hold
            if fresh and self.park_ack and parked and self.pending is None and self.started is None and self.failure is None and not self.protocol_cancelled:
                projected=progress(self.points,self.pose['position']);remaining=self.encounter_arc-projected['arc_m']
                if self.lead_window[0]<=remaining<=self.lead_window[1] and projected['distance_m']<=.3:
                    self.started=ns;self.last_motion_sim=ns
                    if self.navigation_segments_enabled or self.actor_motion_contract is not None:self.trigger_actor_identity={k:self.latest_observation[k]for k in ('entity','entity_id')}
                    if self.event_window_enabled:self.event_state='active'
                    self.freeze_contract()
                    self.record('trigger',dict(start_sim_ns=ns,actual_pose=dict(self.pose),actual_mission_state=self.state,
                        actual_park_observation=self.latest_observation,progress=projected,remaining_arclength_to_obstacle_m=remaining,path_distance_m=projected['distance_m'],
                        encounter_arc_m=self.encounter_arc,actor_chases_robot=False,one_shot=True))
                elif remaining<self.lead_window[0]:
                    reason='encounter_trigger_window_missed_no_chase_or_late_forced_crossing'
                    if not self.cancel_business_protocol(reason,dict(actual_progress=projected,remaining_m=remaining)):
                        self.failure=reason
            world_motion_allowed=self.world_motion_permitted(fresh=fresh,observed=observed,
                reference_current=reference_current,causal_hold=_causal_intake_hold)
            if self.started is not None and not self.protocol_cancelled and self.event_state!='completed_pending_verification':
                if world_motion_allowed and observed and self.failure is None and self.last_motion_sim is not None:
                    delta=max(0.,(ns-self.last_motion_sim)/1e9)
                    self.motion_elapsed+=delta
                    if self.actor_motion_contract is not None and not fresh and delta>0:
                        self.world_motion_decoupled_ticks+=1;self.world_motion_decoupled_s+=delta
                self.last_motion_sim=ns if _causal_motion_clock is None else _causal_motion_clock
            if world_motion_allowed and (self.started is None or observed) and self.failure is None and not self.protocol_cancelled and self.pending is None and self.client.service_is_ready() and self.last_send_sim!=ns:
                # Repeat the fixed park setpoint while waiting; require an actual
                # fresh pose/info observation to arm, never substitute these ACKs.
                if self.started is None:target,phase=list(PARK),'parked'
                elif self.started is not None and not self.clear_ack:target,phase=target_at(self.motion_elapsed)
                else:target=phase=None
                if target is not None:
                    req=SetEntityPose.Request();req.entity.name=ENTITY;req.entity.type=Entity.MODEL
                    req.pose.position.x,req.pose.position.y,req.pose.position.z=target;req.pose.orientation.w=1.
                    self.sent=dict(entity=ENTITY,service=SERVICE,position_world_xyz=target,phase=phase,request_sim_ns=ns,
                        request_monotonic_wall=now,protocol_elapsed_s=self.motion_elapsed)
                    if self.actor_motion_contract is not None and self.started is not None:
                        self.sent.update(actor_choreography=ACTOR_SELECTION,world_motion_allowed=world_motion_allowed,
                            source_fresh=fresh,actor_entity_id=self.trigger_actor_identity['entity_id'])
                    self.record('service_request',self.sent);self.pending=self.client.call_async(req);self.last_send_sim=ns
            if (self.actor_motion_contract is None and self.protocol_cancelled and not self.mission_age_hold and not _causal_intake_hold and self.failure is None and observed and self.pending is None
                    and self.client.service_is_ready() and self.last_send_sim!=ns
                    and not self.diagnostic_recovery_park_ack):
                position=np.asarray(self.latest_observation['position_world_xyz'],float)
                if (not np.isfinite(position).all() or position.shape!=(3,)
                        or abs(position[1]-PARK[1])>1e-6 or abs(position[2]-PARK[2])>1e-6
                        or not BLOCK[0]-.03<=position[0]<=PARK[0]+.03):
                    raise ValueError('diagnostic_actor_retreat_outside_original_fixed_corridor')
                previous=ns if self.last_recovery_sim is None else self.last_recovery_sim
                dt=max(0.,min(.1,(ns-previous)/1e9));self.last_recovery_sim=ns
                # Return on the same original side at the original .25m/s;
                # never chase the robot, restart the event, or retry uncertain
                # service calls. Original pending/timeout handling above owns
                # every request outcome before any next mutation.
                target=[float(position[0]+np.clip(PARK[0]-position[0],-.25*dt,.25*dt)),PARK[1],PARK[2]]
                phase='diagnostic_recovery_parked' if target==PARK else 'diagnostic_recovery_retreat'
                req=SetEntityPose.Request();req.entity.name=ENTITY;req.entity.type=Entity.MODEL
                req.pose.position.x,req.pose.position.y,req.pose.position.z=target;req.pose.orientation.w=1.
                self.sent=dict(entity=ENTITY,service=SERVICE,position_world_xyz=target,phase=phase,
                    request_sim_ns=ns,request_monotonic_wall=now,protocol_elapsed_s=self.motion_elapsed,
                    diagnostic_cancelled_protocol=True,actual_pose_observation=self.latest_observation)
                self.record('service_request',self.sent);self.pending=self.client.call_async(req);self.last_send_sim=ns
            self.complete_event_window(ns,fresh=fresh,observed=observed,parked=parked)
            state=dict(schema='r85_dynamic_obstacle_state/v1',run_id=run.name,binding=self.binding,sim_ns=ns,monotonic_wall=now,
                started_sim_ns=self.started,visible_phase=self.visible_phase,position=self.ack_position,size=SIZE,
                reported_position_semantics='last_successful_SetEntityPose_ACK_target_not_observed_pose',
                actual_pose_info=self.latest_observation,actual_pose_info_fresh=observed,source_fresh=fresh,failure=self.failure,
                motion_elapsed_s=self.motion_elapsed,privileged_actor_pose=True,known_map_assistance=True,
                diagnostic_observe_only=observe_only(self.profile),diagnostic_protocol_cancelled=self.protocol_cancelled,
                diagnostic_business_reason=self.diagnostic_reason,
                reference_pair_update_pending=self.reference_pair_update_pending,mission_age_hold=self.mission_age_hold,
                diagnostic_recovery_park_ack=self.diagnostic_recovery_park_ack,dynamic_PASS=False,
                native_observer_start_attempted=self.observer_attempted,native_observer_started=self.observer is not None,
                navigation_ground_truth_used=False,robot_pose_service_called=False,short_prefix_dynamic_PASS=False)
            if self.actor_motion_contract is not None:
                state.update(actor_choreography=ACTOR_SELECTION,world_motion_allowed=world_motion_allowed,
                    world_owner_stopped=self.world_owner_stopped,world_motion_decoupled_ticks=self.world_motion_decoupled_ticks,
                    world_motion_decoupled_s=self.world_motion_decoupled_s)
            if self.event_window_enabled:
                state.update(event_lifetime_contract=self.event_schema,event_state=self.event_state,
                    completed_event_window=self.completed_window,whole_task_single_scope_claimed=False)
            if self.navigation_segments_enabled:state.update(current_navigation_binding=self.current_navigation_binding or self.binding,navigation_boundary_sequences=list(self.navigation_boundary_sequences),dynamic_proof_seen=self.dynamic_proof_seen)
            if self.coordinate_handoffs_enabled:state['coordinate_boundary_receipts']=list(self.coordinate_boundary_receipts)
            if self.completed_history_enabled and self.completed_history_qualification is not None:
                state.update(current_navigation_binding=self.current_navigation_binding,
                    completed_history_qualification=self.completed_history_qualification,
                    completed_history_scope_policy=completed_scope.POLICY,
                    completed_history_first_journal_receipt=self.completed_history_first_journal_receipt,
                    completed_history_latest_journal_receipt=self.completed_history_latest_journal_receipt)
            if self.native_spool_enabled and self.observer is not None:
                state['native_observation_spool']=self.observer.queue.snapshot_state()
            if getattr(self,'mission_failure_witness',None) is not None:
                state['mission_first_failure_witness']=self.mission_failure_witness
                state['mission_first_failure_witness_publication']=dict(self.mission_failure_witness_status)
            self.writer.atomic(run/'r85_dynamic_obstacle_state.json',state)
            self.pub.publish(String(data=json.dumps(state,allow_nan=False)))
        def close(self):
            if self._causal_intake is None:return self._close_without_causal_receipt()
            try:
                self.writer.atomic(run/'r74_dynamic_causal_intake_receipt.json',
                    self._causal_intake.snapshot(node_failure=self.failure))
            finally:self._close_without_causal_receipt()
        def _close_without_causal_receipt(self):
            if not self.native_spool_enabled and not self.async_qualification_enabled:
                if self.observer is not None:self.observer.close()
                self.writer.close()
                return
            shutdown_error=None;job_status=None
            try:
                if self.history_job is not None:
                    try:
                        job_status=self.history_job.close()
                        if job_status['thread_alive'] or job_status['error'] is not None:
                            raise RuntimeError('frozen_qualification_shutdown_incomplete_or_failed:'+str(job_status))
                    except BaseException as error:
                        shutdown_error=error
                        if job_status is None:job_status=dict(shutdown_call_failed=True,error=type(error).__name__+':'+str(error))
                if self.observer is not None:
                    try:self.observer.close()
                    except BaseException as error:
                        if self.observer.error is None:self.observer.error=type(error).__name__+':'+str(error)
                        if shutdown_error is None:shutdown_error=error
                # Both owned resources were attempted before any fallible writer call.
                if job_status is not None:
                    try:
                        self.writer.atomic(run/'r85_completed_history_async_shutdown.json',dict(
                            schema='r85_completed_history_async_shutdown/v1',**job_status,main_result_handled=self.history_job_handled,
                            history_qualification_committed=self.completed_history_qualification is not None))
                    except BaseException as error:
                        if shutdown_error is None:shutdown_error=error
                if self.observer is not None:
                    try:
                        self.writer.atomic(run/'r85_native_observation_spool_receipt.json',dict(
                            schema='r85_native_observation_spool_shutdown/v1',
                            **self.observer.queue.snapshot_state(),observer_error=self.observer.error,
                            original_timestamps_retained=True,unread_records_not_claimed_journalled=True,
                            journal_completeness_not_established_by_spool=True))
                    except BaseException as error:
                        if shutdown_error is None:shutdown_error=error
            finally:
                try:self.writer.close()
                except BaseException as error:
                    if shutdown_error is None:shutdown_error=error
            if shutdown_error is not None:raise shutdown_error
    return Dynamic()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--task-contract',type=Path);parser.add_argument('--expected-partition',required=True)
    args,ros=parser.parse_known_args();run=args.run.resolve();task=(args.task_contract or run/'r85_task_contract.json').resolve()
    from pid_scope import verify_scope
    receipt=verify_scope(run/'navigation_scope.json')
    scope=json.loads((run/'navigation_scope.json').read_text())
    if scope.get('profile')!=receipt['profile']:raise ValueError('verified_scope_profile_changed_before_dynamic_start')
    import rclpy
    rclpy.init(args=ros);node=build_node(run,scope,task,args.expected_partition)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        try:node.close()
        finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
