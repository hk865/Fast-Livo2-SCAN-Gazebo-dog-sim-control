"""Completed-event history retention, never navigation or Actor authority.

The original physical verifier and evidence IO remain unchanged. Qualification
is read-only and is never a dynamic PASS. Late verification remains mandatory.
"""
from __future__ import annotations
import base64
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import time

POLICY='completed_same_geometry_v1'
REF='r85_dynamic_event_reference.json'
REG='r85_dynamic_event_registration.json'
BINDING_KEYS=('run_id','request_id','reference_id','reference_sha256','epoch',
              'localization_generation','source_route_sha256')
FIXED_T=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-.3],[0.,0.,0.,1.]]


def enabled(profile):
    value=profile.get('r85_dynamic_event_contract') if isinstance(profile,dict) else None
    return isinstance(value,dict) and value.get('completed_same_geometry_preservation') is True


def require(ok,reason):
    if not ok:raise ValueError(reason)


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
def sha(raw):return hashlib.sha256(raw).hexdigest()
def norm(value):return math.sqrt(sum(float(x)**2 for x in value))
def _hex(value):return isinstance(value,str) and len(value)==64 and all(x in '0123456789abcdef'for x in value)
def _equal(a,b):return canonical(a)==canonical(b)


def _fixed_navigation_world_equal(value):
    # This fixed transform is numeric: signed zero is equal, without tolerance.
    # Raw evidence, canonical documents and all other geometry remain exact.
    if type(value) is not list or len(value)!=4:return False
    for row,expected in zip(value,FIXED_T):
        if type(row) is not list or len(row)!=4:return False
        for item,target in zip(row,expected):
            if type(item) not in (int,float):return False
            try:
                if not math.isfinite(item) or item!=target:return False
            except (OverflowError,TypeError,ValueError):return False
    return True


def write_snapshot(path,raw):
    require(0<len(raw)<=16*1024**2,'dynamic_snapshot_budget')
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o444)
    with os.fdopen(fd,'wb')as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    with os.fdopen(fd,'rb')as stream:require(stream.read()==raw,'dynamic_snapshot_readback')
    fd=os.open(Path(path).parent,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)


def snapshot_trigger_inputs(run,record):
    run=Path(run).resolve()
    require(Path(record['reference_file']).resolve()==run/'r85_global_reference.json'
        and Path(record['registration_file']).resolve()==run/'r85_reference_registration.json',
        'foreign_dynamic_trigger_source_path')
    ref=record['reference_raw_utf8'].encode();reg=record['registration_raw_utf8'].encode()
    require(sha(ref)==record['reference_sha256'] and sha(reg)==record['registration_sha256'],
        'dynamic_trigger_same_raw_SHA_mismatch')
    require(_equal(json.loads(ref),record['document']) and _equal(json.loads(reg),record['registration_document']),
        'dynamic_trigger_raw_document_mismatch')
    require((run/'r85_global_reference.json').read_bytes()==ref
        and (run/'r85_reference_registration.json').read_bytes()==reg,'dynamic_trigger_source_changed')
    write_snapshot(run/REF,ref);write_snapshot(run/REG,reg)
    return run/REF,run/REG


def same_geometry(anchor_reference,anchor_registration,new_reference,new_registration):
    # Only the observed reference identity fields may change. Every geometry,
    # task/Actor/source/route field, including unknown future fields, is exact.
    ref_keys={'epoch','localization_generation','reference_id'}
    reg_keys={'localization_generation','reference_id','reference_file_sha256'}
    require(_equal({k:v for k,v in anchor_reference.items()if k not in ref_keys},
        {k:v for k,v in new_reference.items()if k not in ref_keys}),'completed_event_geometry_or_source_changed')
    require(_equal({k:v for k,v in anchor_registration.items()if k not in reg_keys},
        {k:v for k,v in new_registration.items()if k not in reg_keys}),'completed_event_registration_source_changed')
    return True


def _prefix_rows(verifier,run,receipt):
    """Read only the sealed original prefix; never rescan a growing full log."""
    run=Path(run).resolve();path=Path(receipt['path']).resolve()
    require(path.is_relative_to(run),'foreign_completed_evidence_prefix')
    length=receipt['complete_prefix_bytes']
    require(type(length)is int and 0<=length<=verifier.MAX_LOG_BYTES
        and _hex(receipt['complete_prefix_sha256']),'invalid_completed_prefix_receipt')
    require(length<=path.stat().st_size<=verifier.MAX_LOG_BYTES,'completed_prefix_shortened_or_log_budget')
    rows=[];offset=0;digest=hashlib.sha256()
    with path.open('rb')as stream:
        while offset<length:
            raw=stream.readline(min(verifier.MAX_LINE_BYTES+1,length-offset))
            require(raw and len(raw)<=verifier.MAX_LINE_BYTES and raw.endswith(b'\n'),'completed_prefix_truncated_or_partial')
            data=json.loads(raw);canonical(data)
            rows.append(dict(data=data,source_file=str(path),source_offset=offset,
                source_length=len(raw),source_line_sha256=sha(raw)))
            offset+=len(raw);digest.update(raw)
            require(len(rows)<=verifier.MAX_RECORDS,'completed_prefix_record_budget')
    require(offset==length and digest.hexdigest()==receipt['complete_prefix_sha256'],'completed_prefix_rewritten')
    return rows


def stable_selected_witness(verifier,run,result,handoff_source_ns,*,identity_prefixes=None):
    """Use the original stop/clear selections and exact corrected-message joins.

    Only original guard replay reads LiDAR payloads. This gate reads JSONL
    source identity records; no telemetry, native robot pose, or training NPZ.
    """
    run=Path(run).resolve();receipt=result['source_receipt'];binding=result['verification']['binding']
    rows=_prefix_rows(verifier,run,receipt['pretrigger_monitoring'])+_prefix_rows(verifier,run,receipt['journal'])
    rows=[dict(r,data=r['data']['data'],_journal_raw=r['data'],received_sim_ns=r['data']['sim_ns'])
        for r in rows if _equal(r['data'].get('binding'),binding)]
    def kind(name):return [r for r in rows if r['_journal_raw']['kind']==name]
    statuses=[r for r in kind('status')if r['data'].get('request_id')==binding['request_id']]
    held=[r for r in statuses if r['data'].get('obstacle_hold')is True]
    require(held,'stable_witness_held_missing')
    guards=[r for r in _prefix_rows(verifier,run,receipt['native_guard'])
        if r['data'].get('request_id')==binding['request_id']]
    modules,_=verifier._modules()
    preroll,stop_guard,event=modules['stop'].select_event_preroll(binding['request_id'],held,guards,verifier._verify_line)
    verifier._strict_clocks(preroll,'compute_ros_clock_ns',300000000,3,300000000,'Native pre-roll')
    resumed=[r for r in statuses if r['received_sim_ns']>held[-1]['received_sim_ns']
        and r['data'].get('obstacle_hold')is False and type(r['data'].get('obstacle_resumes'))is int
        and r['data']['obstacle_resumes']>0 and type(r['data'].get('aligned_obstacle_resumes'))is int
        and r['data']['aligned_obstacle_resumes']>0]
    require(resumed,'stable_witness_original_resume_missing');resume=resumed[0]['received_sim_ns']
    release,cleared=verifier.select_clear_window(guards,event,resume)
    clear_end=release['data']['compute_ros_clock_ns']
    verifier._strict_clocks(cleared,'compute_ros_clock_ns',300000000,10,1000000000,'Continuous clear guard')
    require(all(b['data'].get('sequence')==a['data'].get('sequence',-2)+1
        and b['data'].get('previous_actual_guard_clock_ns')==a['data']['compute_ros_clock_ns']
        for a,b in zip(cleared,cleared[1:])) and all(r['data']['union_result']['blocked']is False for r in cleared),
        'stable_witness_original_clear_guard_changed')
    expected=result['verification']['checks']['native_cloud_guard_stop_token_and_recovery']
    require(_equal(event,expected['event']) and len(preroll)==expected['pre_roll_count']
        and len(cleared)==expected['clear_count']
        and release['data']['sequence']==expected['clear_release_sequence']
        and clear_end==expected['clear_release_clock_ns']
        and resume==expected['aligned_resume_sim_ns'],'stable_witness_original_guard_selection_changed')
    selected=preroll+cleared
    for row in selected:verifier._verify_line(row)
    poses=[]
    for row in kind('actual_pose'):
        p=row['data']
        if poses and p['stamp_ns']==poses[-1]['data']['stamp_ns']:
            require(_equal(p,poses[-1]['data']),'conflicting_repeated_actual_message');continue
        poses.append(row)
    good=[];stop=None
    for row in poses:
        p=row['data'];stamp=p['stamp_ns']
        if not held[0]['received_sim_ns']<=stamp<=held[-1]['received_sim_ns']:continue
        imu=p.get('imu_stamp_ns')
        stopped=(type(imu)is int and abs(stamp-imu)<=300000000 and p.get('body_gyro')is not None
            and norm(p['body_velocity'][:2])<=.03 and abs(p['body_angular_velocity'][2])<=.05
            and norm(p['body_gyro'])<=.05)
        if not stopped:good=[];continue
        if good and stamp-good[-1]['data']['stamp_ns']>200000000:good=[]
        good.append(row)
        if len(good)>=3 and stamp-good[0]['data']['stamp_ns']>=300000000:stop=good.copy();break
    require(stop is not None,'stable_witness_original_measured_stop_missing')
    moving=[r for r in poses if r['data']['stamp_ns']>=resume];recovery=[]
    # First prefix satisfying the unchanged physical predicates. A later
    # correction cannot be substituted after a failed exact-source join.
    for row in moving:
        recovery.append(row)
        progress=max(norm([r['data']['position'][i]-moving[0]['data']['position'][i]for i in (0,1)])for r in recovery)
        if len(recovery)>=3 and progress>=.08 and any(norm(r['data']['body_velocity'][:2])>.025 for r in recovery):break
    require(len(recovery)>=3 and progress>=.08 and any(norm(r['data']['body_velocity'][:2])>.025 for r in recovery),
        'stable_witness_recovery_8cm_missing')
    wanted={r['data']['stamp_ns']for r in stop+recovery}
    for row in selected:wanted.update([row['data']['control_pose_stamp_ns'],row['data']['cloud_header_stamp_ns']])
    lower,upper=min(wanted),max(wanted)
    if identity_prefixes is None:bundle_rows,bundle_source=verifier._lines(run,'known_scene_bundles.jsonl')
    else:
        bundle_source=identity_prefixes['bundles'];bundle_rows=_prefix_rows(verifier,run,bundle_source)
    bundles={};stable_interval=[]
    for row in bundle_rows:
        d=row['data'];ns=d['source_ns']
        if lower<=ns<=upper:
            require(ns not in bundles,'duplicate_stable_source_bundle');bundles[ns]=row;stable_interval.append(row)
    require(wanted<=set(bundles),'stable_source_bundle_missing_no_nearest_join')
    generations={r['data']['generation']for r in stable_interval}
    matrices={sha(canonical(r['data']['C_world_odom']).encode())for r in stable_interval}
    require(len(generations)==1 and len(matrices)==1 and type(next(iter(generations)))is int,
        'cross_generation_or_correction_cannot_manufacture_stop_recovery')
    require(all(r['data'].get('schema')=='known_scene_navigation_bundle/v1' and r['data'].get('run_id')==run.name
        and r['data'].get('correction_transition')is False and r['data'].get('robot_truth_pose_used')is False
        and _fixed_navigation_world_equal(r['data'].get('T_navigation_world'))for r in stable_interval),'unstable_or_foreign_source_bundle')
    for row in stop+recovery:
        p=row['data'];require(_hex(p.get('corrected_body_message_sha256')) and
            p['corrected_body_message_sha256']==bundles[p['stamp_ns']]['data'].get('corrected_body_message_sha256'),
            'actual_body_message_SHA_missing_or_foreign')
    if identity_prefixes is None:inputs,input_source=verifier._lines(run,'known_scene_controller_inputs.jsonl')
    else:
        input_source=identity_prefixes['controller_inputs'];inputs=_prefix_rows(verifier,run,input_source)
    accepted={}
    for row in inputs:
        d=row['data'];ns=d['source_ns']
        if ns in wanted:
            require(ns not in accepted,'duplicate_stable_controller_source');accepted[ns]=row
    require(set(accepted)==wanted,'stable_actual_controller_source_missing')
    generation=next(iter(generations))
    require(all(type(row['data'].get('generation'))is int and row['data']['generation']==generation
        and type(row['data'].get('accepted_pose_stamp_ns'))is int and row['data']['accepted_pose_stamp_ns']==ns
        and type(row['data'].get('accepted_cloud_stamp_ns'))is int and row['data']['accepted_cloud_stamp_ns']==ns
        and row['data'].get('navigation_ground_truth_used')is False for ns,row in accepted.items()),
        'stable_source_not_original_controller_consumed')
    acks=kind('service_ack');clears=[r for r in acks if r['data']['phase']=='clear']
    require(clears,'stable_witness_actual_clear_missing');observations=kind('entity_observation')
    clear_observed=[r for r in observations if clears[0]['data']['request_sim_ns']<=r['data']['stamp_ns']<=clears[-1]['received_sim_ns']+500000000
        and norm([r['data']['position_world_xyz'][i]-[17.,5.7,1.8][i]for i in range(3)])<=.05]
    require(clear_observed,'stable_witness_independent_clear_missing')
    # Include every original phase witness, rather than only selected guard
    # clocks. This is conservative and cannot move the handoff before clear.
    physical=[*held,resumed[0],*stop,*recovery,*acks,*clear_observed,*kind('trigger')]
    for phase in ('entering','leaving','blocking'):
        phase_acks=[r for r in acks if r['data']['phase']==phase]
        require(phase_acks,'stable_witness_phase_missing')
        lo=phase_acks[0]['data']['request_sim_ns']-(500000000 if phase=='blocking' else 0)
        hi=phase_acks[-1]['data']['response_sim_ns']+500000000
        physical.extend(r for r in observations if lo<=r['data']['stamp_ns']<=hi)
    maximum=max([*wanted,*[r['received_sim_ns']for r in physical],
        *[r['data']['compute_ros_clock_ns']for r in selected]])
    require(type(handoff_source_ns)is int and maximum<handoff_source_ns,'physical_witness_not_complete_before_handoff')
    def row_receipt(row):return {k:row[k]for k in ('source_file','source_offset','source_length','source_line_sha256')}
    return dict(schema='r85_completed_stable_source_witness/v1',generation=generation,
        applied_C_sha256=next(iter(matrices)),maximum_witness_source_ns=maximum,
        original_stop_token=event,original_resume_sim_ns=resume,original_stop_pose_count=len(stop),
        recovery_source_start_ns=recovery[0]['data']['stamp_ns'],recovery_source_end_ns=recovery[-1]['data']['stamp_ns'],
        original_recovery_progress_m=progress,original_guard_receipts=[row_receipt(r)for r in selected],
        source_bundle_receipts=[row_receipt(r)for r in stable_interval],
        controller_input_receipts=[row_receipt(r)for _,r in sorted(accepted.items())],
        source_identity_prefixes=dict(bundles=bundle_source,controller_inputs=input_source),
        original_source_prefixes=copy.deepcopy(receipt))


class _QualificationCapture:
    """Observe the original verifier's same read, without writing a capsule."""
    def __init__(self,verifier,run):self.verifier=verifier;self.run=Path(run).resolve();self.state_raw=None
    def read_json(self,path,expected=None):
        path=Path(path).resolve()
        require(path in (self.run/'r85_dynamic_task_contract.json',self.run/'r85_dynamic_obstacle_state.json'),
            'foreign_qualification_JSON_read')
        def observe(actual,raw,doc,binding):
            if path.name=='r85_dynamic_obstacle_state.json':
                require(self.state_raw is None,'duplicate_qualification_state_read');self.state_raw=raw
        return self.verifier._read_json(path,expected,observe_raw=observe)
    def lines(self,run,filename):
        require(Path(run).resolve()==self.run,'foreign_qualification_log_read')
        return self.verifier._lines(run,filename)


class _QualificationReplay:
    """Feed sealed state bytes and real original prefixes to verify_result."""
    def __init__(self,verifier,run,qualification):
        from r85_dynamic_evidence_io import _CapturedJSONPath
        self.verifier=verifier;self.run=Path(run).resolve();self.receipt=qualification['original_physical_verifier_result']['source_receipt']
        self.state_raw=base64.b64decode(qualification['original_runtime_state_raw_base64'],validate=True)
        source=self.receipt['runtime_state']
        require(len(self.state_raw)<=16*1024**2 and len(self.state_raw)==source['bytes']
            and sha(self.state_raw)==source['sha256'],'qualification_runtime_same_read_SHA_changed')
        self.state_view=_CapturedJSONPath(self.run/'r85_dynamic_obstacle_state.json',self.state_raw)
        self.json_used=set();self.logs_used=set()
    def read_json(self,path,expected=None):
        path=Path(path).resolve();roles=[r for r in ('contract','runtime_state')if self.receipt[r]['path']==str(path)]
        require(len(roles)==1 and roles[0]not in self.json_used,'foreign_or_duplicate_qualification_JSON_replay')
        role=roles[0];self.json_used.add(role);source=self.receipt[role]
        require(expected is None or expected==source['sha256'],'qualification_JSON_expected_SHA_changed')
        doc,binding=self.verifier._read_json(self.state_view if role=='runtime_state'else path,source['sha256'])
        require(binding==source,'qualification_JSON_receipt_changed');return doc,binding
    def lines(self,run,filename):
        require(Path(run).resolve()==self.run,'foreign_qualification_prefix_replay')
        path=self.verifier._inside(self.run,filename)
        roles=[r for r in ('journal','pretrigger_monitoring','native_guard')if self.receipt[r]['path']==str(path)]
        require(len(roles)==1 and roles[0]not in self.logs_used,'foreign_or_duplicate_qualification_prefix_replay')
        role=roles[0];self.logs_used.add(role)
        return _prefix_rows(self.verifier,self.run,self.receipt[role]),copy.deepcopy(self.receipt[role])
    def assert_complete(self,replayed,original):
        require(self.json_used=={'contract','runtime_state'} and self.logs_used=={'journal','pretrigger_monitoring','native_guard'},
            'qualification_original_source_consumption_incomplete')
        require(_equal(replayed,original),'qualification_entire_original_verifier_result_changed')


def replay_history_qualification(verifier,run,qualification):
    scope,_=verifier._read_json(Path(run)/'navigation_scope.json')
    require(enabled(scope.get('profile')),'qualification_replay_optin_missing')
    original=qualification['original_physical_verifier_result']
    replay=_QualificationReplay(verifier,run,qualification)
    result=verifier.verify_result(run,original['request_id'],scope['profile'],scope,evidence_io=replay)
    replay.assert_complete(result,original)
    require(result.get('passed')is True and result.get('state')=='passed','qualification_original_replay_not_passed')
    return result


def qualify_history_retention(verifier,run,request_id,profile,scope,handoff_source_ns):
    started=time.monotonic()
    capture=_QualificationCapture(verifier,run)
    result=verifier.verify_result(run,request_id,profile,scope,evidence_io=capture)
    require(result.get('passed')is True and result.get('state')=='passed'
        and result['verification'].get('allowed')is True,'original_physical_history_not_complete:'+str(result.get('failure_reason')))
    require(capture.state_raw is not None,'qualification_runtime_same_read_missing')
    require(sha(capture.state_raw)==result['source_receipt']['runtime_state']['sha256'],
        'qualification_runtime_same_read_receipt_changed')
    stable=stable_selected_witness(verifier,run,result,handoff_source_ns)
    return dict(schema='r85_completed_history_retention_qualification/v1',
        qualified_for_history_retention_only=True,passed=False,dynamic_PASS=False,navigation_PASS=False,
        original_single_scope_dynamic_PASS=False,identity_contract_changed=True,
        original_physical_verifier_result=result,stable_witness=stable,
        original_runtime_state_raw_base64=base64.b64encode(capture.state_raw).decode('ascii'),
        actual_monotonic_duration_s=time.monotonic()-started,acceptance_deferred_until_endpoint_parking_map=True)


def _validate_current_pair(run,anchor,reference,registration,binding,reference_sha256,registration_sha256):
    require(isinstance(binding,dict) and set(binding)==set(BINDING_KEYS),'malformed_current_navigation_binding')
    for key in ('run_id','request_id','source_route_sha256'):
        require(type(binding[key])is type(anchor[key]) and binding[key]==anchor[key],'foreign_current_navigation_binding:'+key)
    require(all(type(binding[k])is int and binding[k]>=1 for k in ('epoch','localization_generation'))
        and _hex(binding['reference_sha256']) and isinstance(binding['reference_id'],str)and binding['reference_id'],
        'invalid_current_navigation_identity')
    require(binding['run_id']==Path(run).name and reference_sha256==binding['reference_sha256']
        and all(type(reference.get(k))is type(binding[k])and reference[k]==binding[k]
            for k in ('run_id','reference_id','epoch','localization_generation','source_route_sha256')),
        'current_reference_document_binding_mismatch')
    require(registration.get('run_id')==binding['run_id'] and registration.get('reference_id')==binding['reference_id']
        and registration.get('reference_file_sha256')==reference_sha256 and _hex(registration_sha256)
        and type(registration.get('localization_generation'))is int
        and registration['localization_generation']==binding['localization_generation']
        and registration.get('source_route_sha256')==binding['source_route_sha256'],'current_registration_binding_mismatch')


def verify_late_mission_gate(verifier,run,original_result,*,identity_prefixes=None):
    run=Path(run).resolve();contract,_=verifier._read_json(run/'r85_dynamic_task_contract.json')
    require(contract.get('completed_history_scope_policy')==POLICY,'missing_completed_history_policy')
    require(original_result.get('passed')is True and original_result.get('state')=='passed',
        'late_original_physical_verifier_not_passed')
    anchor=contract['binding'];verifier._same_binding(original_result['verification']['binding'],anchor)
    require(Path(contract['reference_file']).resolve()==run/REF and Path(contract['registration_file']).resolve()==run/REG
        and Path(contract['trigger']['reference_file']).resolve()==run/REF,'foreign_dynamic_snapshot_path')
    reference,_=verifier._read_json(run/REF,anchor['reference_sha256'])
    registration,_=verifier._read_json(run/REG,contract['registration_sha256'])
    current,current_source=verifier._read_json(run/'r85_global_reference.json')
    registered,registered_source=verifier._read_json(run/'r85_reference_registration.json')
    state,_=verifier._read_json(run/'r85_dynamic_obstacle_state.json')
    require(state.get('failure')is None and state.get('diagnostic_protocol_cancelled')is False,
        'cancelled_or_failed_history_cannot_revive')
    verifier._same_binding(state['binding'],anchor)
    same_geometry(reference,registration,current,registered)
    rows,_=verifier._lines(run,'r85_dynamic_journal.jsonl')
    preserved=[r for r in rows if r['data']['kind']=='completed_history_scope_preserved']
    if preserved:
        first=preserved[0]['data']['data'];qualification=first['qualification'];qsha=sha(canonical(qualification).encode())
        require(qualification.get('qualified_for_history_retention_only')is True and qualification.get('passed')is False
            and qualification.get('dynamic_PASS')is False and qualification.get('navigation_PASS')is False,
            'invalid_history_retention_qualification')
        replay_history_qualification(verifier,run,qualification)
        stable=stable_selected_witness(verifier,run,qualification['original_physical_verifier_result'],first['handoff_source_ns'],
            identity_prefixes=qualification['stable_witness']['source_identity_prefixes'])
        require(_equal(stable,qualification['stable_witness']),'completed_stable_witness_changed')
        previous=anchor;previous_source=stable['maximum_witness_source_ns'];previous_pose=-1
        for row in preserved:
            raw=row['data'];data=raw['data'];verifier._same_binding(raw['binding'],anchor)
            require(_equal(data['previous_navigation_binding'],previous) and data['qualification_sha256']==qsha,
                'completed_history_chain_or_receipt_changed')
            new=data['current_navigation_binding'];source=data['handoff_source_ns'];pose=data['source_actual_pose_stamp_ns']
            require(type(source)is int and type(pose)is int and source>previous_source and pose>=previous_pose
                and new['epoch']>previous['epoch'] and new['localization_generation']>=previous['localization_generation'],
                'completed_history_source_or_scope_not_monotonic')
            message=data['mission_message'];ref=message['r85_reference']
            require(message.get('schema')=='r85_actual_single_task_mission_state/v1'
                and message.get('run_id')==run.name and message.get('current_request')==anchor['request_id']
                and message.get('navigation_ground_truth_used')is False and message.get('stage')=='navigating'
                and message.get('ros_sim_time_ns')==source and message.get('source_actual_pose_stamp_ns')==pose
                and type(data['mission_received_sim_ns'])is int
                and 0<=data['mission_received_sim_ns']-source<=300000000
                and 0<=data['mission_received_sim_ns']-pose<=300000000
                and raw['sim_ns']>=data['mission_received_sim_ns']
                and type(data['mission_received_monotonic_wall'])in (int,float)
                and math.isfinite(data['mission_received_monotonic_wall'])
                and raw['monotonic_wall']>=data['mission_received_monotonic_wall'],
                'completed_history_actual_mission_source_chain_changed')
            require(all(type(ref.get(k))is type(new[k])and ref[k]==new[k]
                    for k in ('reference_id','epoch','localization_generation','source_route_sha256'))
                and ref.get('global_reference_sha256')==new['reference_sha256']
                and Path(ref['global_reference_file']).resolve()==run/'r85_global_reference.json',
                'completed_history_mission_reference_binding_changed')
            rdoc=data['current_reference_document'];gdoc=data['current_registration_document']
            _validate_current_pair(run,anchor,rdoc,gdoc,new,data['current_reference_sha256'],data['current_registration_sha256'])
            require(sha(data['current_reference_raw_utf8'].encode())==data['current_reference_sha256']
                and sha(data['current_registration_raw_utf8'].encode())==data['current_registration_sha256']
                and _equal(json.loads(data['current_reference_raw_utf8']),rdoc)
                and _equal(json.loads(data['current_registration_raw_utf8']),gdoc),'completed_history_source_raw_SHA_changed')
            same_geometry(reference,registration,rdoc,gdoc)
            if row is not preserved[0]:require('qualification'not in data,'duplicate_completed_qualification')
            previous=new;previous_source=source;previous_pose=pose
        require(state.get('completed_history_scope_policy')==POLICY
            and _equal(state.get('completed_history_qualification'),qualification)
            and _equal(state.get('current_navigation_binding'),previous),'actual_current_history_binding_changed')
        _validate_current_pair(run,anchor,current,registered,previous,current_source['sha256'],registered_source['sha256'])
        require(_equal(current,preserved[-1]['data']['data']['current_reference_document'])
            and _equal(registered,preserved[-1]['data']['data']['current_registration_document'])
            and registered_source['sha256']==preserved[-1]['data']['data']['current_registration_sha256'],
            'late_current_reference_or_registration_changed')
    else:
        require(current_source['sha256']==anchor['reference_sha256']
            and registered_source['sha256']==contract['registration_sha256'],'unrecorded_current_navigation_handoff')
        stable=stable_selected_witness(verifier,run,original_result,state['sim_ns'],identity_prefixes=identity_prefixes)
    return dict(schema='r85_completed_history_mission_gate/v1',run_id=run.name,
        request_id=original_result['request_id'],passed=True,mission_v2_gate_pass=True,
        original_physical_verifier_pass=True,original_single_scope_dynamic_PASS=False,
        identity_contract_changed=True,formal_acceptance_pass=False,navigation_PASS=False,
        full_F3_PASS=False,original46_PASS=False,stable_witness=stable,preserved_scope_count=len(preserved))


def honest_terminal_fields(result,gate):
    require(gate.get('mission_v2_gate_pass')is True and gate.get('original_physical_verifier_pass')is True,
        'mission_v2_gate_missing_at_terminal')
    result=copy.deepcopy(result)
    result.update(original_predicates_and_thresholds_unchanged=False,
        original_physical_thresholds_unchanged=True,identity_contract_changed=True,
        original_physical_verifier_pass=True,mission_v2_gate_pass=True,
        formal_acceptance_pass=False,formal_full_route_pass=False,navigation_PASS=False,full_F3_PASS=False,original46_PASS=False,
        original_single_scope_dynamic_PASS=False,dynamic_identity_contract=POLICY,
        dynamic_mission_v2_result=copy.deepcopy(gate))
    result['actual_original_predicates']['dynamic_protocol']=False
    result['actual_original_predicates']['dynamic_physical_verifier']=True
    result['actual_original_predicates']['mission_v2_identity_gate']=True
    return result


def enrich_dynamic_result(result,gate):
    require(result.get('passed')is True and result.get('state')=='passed'
        and gate.get('mission_v2_gate_pass')is True and gate.get('original_physical_verifier_pass')is True,
        'completed_history_full_verifier_or_late_gate_missing')
    result=copy.deepcopy(result)
    result.update(dynamic_identity_contract=POLICY,identity_contract_changed=True,
        original_physical_verifier_pass=True,mission_v2_gate_pass=True,
        original_single_scope_dynamic_PASS=False,dynamic_PASS=False,formal_acceptance_pass=False,
        formal_full_route_pass=False,navigation_PASS=False,full_F3_PASS=False,original46_PASS=False,
        dynamic_mission_v2_result=copy.deepcopy(gate))
    return result


def replay_verified_mission_result(run,capsule_binding,verifier,scope,owner_result):
    # The original replay observes exactly its sealed original JSON/log bytes.
    # Only after that full verifier passes is the separate v2 identity gate run.
    from r85_dynamic_evidence_io import ReplayIO
    run=Path(run).resolve();request_id=run.name+':r85:f1_to_f3:1'
    replay=ReplayIO(run,capsule_binding,verifier,scope,owner_result)
    physical=verifier.verify_result(run,request_id,scope['profile'],scope,evidence_io=replay)
    gate=verify_late_mission_gate(verifier,run,physical,
        identity_prefixes=owner_result['dynamic_mission_v2_result']['stable_witness']['source_identity_prefixes'])
    enriched=enrich_dynamic_result(physical,gate)
    report=replay.assert_complete(enriched,owner_result)
    return enriched,report
