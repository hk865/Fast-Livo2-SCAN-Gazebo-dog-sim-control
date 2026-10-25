"""Exact schedule-only identity renewal; no control or SCAN acceptance changes."""
from pathlib import Path
import copy
import hashlib
import json
import re

ROOT=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/workspace/multifloor_demo/teacher_mode/navigation/engineering_v66_r85_turn_handoff_candidate')
WORK=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1000')
STAGE=WORK/'schedule_adapter_stage'
PROOF=ROOT/'R85_SCHEDULE_RENEWAL_20261016_UNTIL1000.json'
PROFILE='profiles/r85_f1_f3_native_observation_spool_r13_20261016.json'
BASELINE_COMMIT='836606ee8f887dd112020f2c1763fc1e0f738a56'
OLD_SCANNER_SHA='fd89311e46399f63fef537c73e7a2d13d655e02f2b69591668ccd604e8398b9b'
OLD_GUARD_SHA='3a08ca4f1cdc38f6abf59efb31b5d7ff7a15194ff4e7137d8ed14e4fdc44e942'
V3_SHA='3fb1795ada9a6296c0cb84e352f6c27a5996b7458f74ffc47274fc082dcc5058'
BASE_SHA='6a028cb1dc37dfab63840172b9e0ba35fec3e0b3d09aa0926e91747f229e5a67'
REFERENCE_SHA={
 'authorization':(WORK/'RENEWED_USER_AUTHORIZATION.json','1b07c4b39727ebdcb11b8d35537519bb4e64a9be566c4382fa58a1751d1bbec7'),
 'budget_review':(WORK/'FINISH_BUDGET_RENEWAL_REVIEW.json','b9a700b8b0334d72f0c8e1e70c51ad460cc0eb4dfb1c19bce2fdd110f82c695b'),
 'root_scheduling_selection':(WORK/'ROOT_SCHEDULING_SELECTION.json','fc0b50276803cb68a01b45e778453576248663292fe56d52ef17caa5f806b26b'),
 'baseline_async_install':(WORK.parent/'native_observer_queue_v1/INSTALL_RECEIPT.json','a9ec63333de181e51d31c315221b5bd75dd9142ae3ff5b938fc8dea83a18b2a8'),
 'historical_v3_adapter':(ROOT/'R85_HEADING_BRAKING_CONTROL_ADAPTER_CONTRACT.json',V3_SHA),
 'base_contract':(ROOT/'R85_SCAN_REFERENCE_PROTOCOL_WORKSPACE_CONTRACT.json',BASE_SHA),
}
PATCH_OPERATIONS=[('def _heading_control_adapter(session,base):', 'def _heading_control_adapter(session,base,renewal=None):'), ('        path=R85_CANDIDATE_ROOT/filename;row=controls[str(path)];change=changes[filename]\n        if (set(row)', "        path=R85_CANDIDATE_ROOT/filename;row=controls[str(path)];change=changes[filename]\n        live_control_sha=row['after_sha256']\n        if renewal is not None and filename=='r85_session_guard.py':live_control_sha=renewal['guard']['after_sha256']\n        if (set(row)"), ("or reviewed[filename]!=row['after_sha256'] or sha(path)!=row['after_sha256']):\n            raise RuntimeError('IMU cumulative control source differs: '+filename)", "or reviewed[filename]!=row['after_sha256'] or sha(path)!=live_control_sha):\n            raise RuntimeError('IMU cumulative control source differs: '+filename)"), ("        actual[str(path)]=row['after_sha256']\n        if str(path)in base['bindings_sha256']:overrides[str(path)]=copy.deepcopy(row)\n    helper=data.get('extra_helper');path=R85_CANDIDATE_ROOT/'imu_causal_buffer.py'", "        actual[str(path)]=live_control_sha\n        if str(path)in base['bindings_sha256']:overrides[str(path)]=copy.deepcopy(row)\n    helper=data.get('extra_helper');path=R85_CANDIDATE_ROOT/'imu_causal_buffer.py'"), ("    original=R85_A1_INTEGRATION_STAGE/'beforeimage/scan_workspace.py'\n    before=R85_HEADING_INTEGRATION_STAGE/'beforeimage/scan_workspace.py'\n    if (not isinstance(validator,dict)", "    original=R85_A1_INTEGRATION_STAGE/'beforeimage/scan_workspace.py'\n    before=R85_HEADING_INTEGRATION_STAGE/'beforeimage/scan_workspace.py'\n    live_validator_sha=validator.get('after_sha256') if isinstance(validator,dict) else None\n    if renewal is not None:live_validator_sha=renewal['validator']['after_sha256']\n    if (not isinstance(validator,dict)"), ("or sha(path)!=validator['after_sha256']):\n        raise RuntimeError('IMU control adapter validator/deployment lineage differs')\n    evidence.extend([path,_adapter_file_ref(validator['beforeimage'],original),\n        _adapter_file_ref(validator['deployment_beforeimage'],before)])\n    actual[str(path)]=validator['after_sha256'];overrides[str(path)]=copy.deepcopy(validator)\n    if set(overrides)!={str(R85_CANDIDATE_ROOT/name)for name in ('controller.py','shared_controller.py','scan_workspace.py')}:\n        raise RuntimeError('IMU adapter attempted an undeclared SCAN override')\n    return dict(path=contract,sha256=digest,data=data,actual=actual,overrides=overrides,evidence=sorted(set(evidence)))\n\n\ndef _adapter_file_ref", "or sha(path)!=live_validator_sha):\n        raise RuntimeError('IMU control adapter validator/deployment lineage differs')\n    evidence.extend([path,_adapter_file_ref(validator['beforeimage'],original),\n        _adapter_file_ref(validator['deployment_beforeimage'],before)])\n    actual[str(path)]=live_validator_sha;overrides[str(path)]=copy.deepcopy(validator)\n    overrides[str(path)]['after_sha256']=live_validator_sha\n    if set(overrides)!={str(R85_CANDIDATE_ROOT/name)for name in ('controller.py','shared_controller.py','scan_workspace.py')}:\n        raise RuntimeError('IMU adapter attempted an undeclared SCAN override')\n    return dict(path=contract,sha256=digest,data=data,actual=actual,overrides=overrides,evidence=sorted(set(evidence)))\n\n\ndef _adapter_file_ref"), ("def _control_adapter(session,base):\n    if session.get('scan_control_adapter_contract')==str(R85_HEADING_ADAPTER):", "def _control_adapter(session,base):\n    if 'schedule_renewal_proof' in session or 'schedule_renewal_proof_sha256' in session:\n        from schedule_renewal_adapter import load_renewal,attach_renewal\n        renewal=load_renewal(session,base)\n        return attach_renewal(_heading_control_adapter(session,base,renewal),renewal)\n    if session.get('scan_control_adapter_contract')==str(R85_HEADING_ADAPTER):"), ("        resolved['resolved_adapter']['extra_preflight']=copy.deepcopy(adapter['data']['extra_preflight'])\n    return resolved", "        resolved['resolved_adapter']['extra_preflight']=copy.deepcopy(adapter['data']['extra_preflight'])\n    if 'schedule_renewal' in adapter:\n        from schedule_renewal_adapter import resolved_renewal\n        resolved_renewal(resolved,adapter['schedule_renewal'])\n    return resolved")]
GUARD_SUBSTITUTIONS=[['FINISH_RESERVE_S = 1200','FINISH_RESERVE_S = 300'],["AUTHORIZED_USER_DEADLINE_UTC = '2026-10-16T06:00:00+00:00'","AUTHORIZED_USER_DEADLINE_UTC = '2026-10-16T10:00:00+00:00'"],['explicit_user_continue_until_20261016_1400_UTCplus08','explicit_user_continue_until_20261016_1800_UTCplus08']]
SCHEDULE=dict(deadline_utc='2026-10-16T10:00:00+00:00',basis='explicit_user_continue_until_20261016_1800_UTCplus08',finish_reserve_s=300,outer_supervisor_wall_cap_s=2700,original_runner_wall_cap_s=2400,original_native_cap_s=770)


def require(ok,reason):
    if not ok:raise RuntimeError('Schedule renewal: '+reason)


def digest(raw):return hashlib.sha256(raw).hexdigest()


def file_ref(row,expected=None,expected_sha=None):
    require(isinstance(row,dict)and set(row)=={'path','sha256'},'file reference shape')
    path=Path(row['path'])
    require(type(row['path'])is str and path.is_absolute()and str(path.resolve())==str(path)
        and (expected is None or path==expected) and type(row['sha256'])is str
        and re.fullmatch('[0-9a-f]{64}',row['sha256'])is not None
        and (expected_sha is None or row['sha256']==expected_sha),'file reference identity')
    require(path.is_file()and digest(path.read_bytes())==row['sha256'],'file reference actual SHA: '+str(path))
    return path


def scanner_after_bytes(raw):
    require(digest(raw)==OLD_SCANNER_SHA,'scanner historical beforeimage')
    text=raw.decode()
    for index,(old,new)in enumerate(PATCH_OPERATIONS):
        if index in (1,2,3):
            lower=text.index('def _heading_control_adapter(');upper=text.index('\n\ndef _adapter_file_ref',lower)
            part=text[lower:upper];require(part.count(old)==1,'scoped scanner bridge occurrence')
            text=text[:lower]+part.replace(old,new)+text[upper:]
        else:
            require(text.count(old)==1,'scanner bridge occurrence');text=text.replace(old,new)
    return text.encode()


def guard_after_bytes(raw):
    require(digest(raw)==OLD_GUARD_SHA,'guard historical beforeimage')
    for old,new in GUARD_SUBSTITUTIONS:
        require(raw.count(old.encode())==1,'guard exact literal occurrence')
        raw=raw.replace(old.encode(),new.encode())
    return raw


def load_renewal(session,base):
    require(Path(__file__).resolve()==ROOT/'schedule_renewal_adapter.py','active helper location')
    require(session.get('scan_control_adapter_contract')==str(ROOT/'R85_HEADING_BRAKING_CONTROL_ADAPTER_CONTRACT.json')
        and session.get('scan_control_adapter_contract_sha256')==V3_SHA,'retain exact historical v3 adapter')
    proof_ref=dict(path=session.get('schedule_renewal_proof'),sha256=session.get('schedule_renewal_proof_sha256'))
    proof_path=file_ref(proof_ref,PROOF);data=json.loads(proof_path.read_text())
    keys={'schema','kind','baseline_commit','expected_bound_source_count_before','expected_bound_source_count_after','exact_targets','simulation_only','runtime_acceptance_inherited','navigation_acceptance_established','physical_thresholds_changed','scan_source_changed','scan_binary_changed','scan_contract_bytes_changed','authorization','budget_review','root_scheduling_selection','baseline_async_install','historical_v3_adapter','base_contract','guard','validator','bridge_helper','unchanged_sources','profile','schedule'}
    require(set(data)==keys and data['schema']=='r85_explicit_schedule_renewal/v1'
        and data['kind']=='until_20261016_1000UTC' and data['baseline_commit']==BASELINE_COMMIT
        and data['expected_bound_source_count_before']==250 and data['expected_bound_source_count_after']==252
        and data['exact_targets']==['r85_session_guard.py','scan_workspace.py','schedule_renewal_adapter.py','R85_SCHEDULE_RENEWAL_20261016_UNTIL1000.json',PROFILE]
        and data['simulation_only']is True and all(data[k]is False for k in ('runtime_acceptance_inherited','navigation_acceptance_established','physical_thresholds_changed','scan_source_changed','scan_binary_changed','scan_contract_bytes_changed'))
        and data['schedule']==SCHEDULE,'proof scope/authority')
    evidence=[proof_path];documents={}
    for key,(path,sha)in REFERENCE_SHA.items():
        p=file_ref(data[key],path,sha);evidence.append(p);documents[key]=json.loads(p.read_text())
    require(documents['authorization'].get('authorized_deadline_UTC')==SCHEDULE['deadline_utc']
        and documents['authorization'].get('automatic_extension')is False,'explicit authorized deadline')
    selection=documents['root_scheduling_selection'];budget=documents['budget_review']['recommendation']
    for document in (selection,budget):
        require(document['finish_reserve_s']==300 and document['original_native_cap_s']==770
            and document['original_runner_wall_cap_s']==2400 and document['outer_supervisor_wall_cap_s']==2700,'measured scheduling selection')
    require(selection['deadline_UTC']==SCHEDULE['deadline_utc']and selection['basis']==SCHEDULE['basis']
        and selection['acceptance_thresholds_changed']is False,'scheduling-only selection')
    require(data['base_contract']==dict(path=str(ROOT/'R85_SCAN_REFERENCE_PROTOCOL_WORKSPACE_CONTRACT.json'),sha256=BASE_SHA)
        and digest(json.dumps(base,sort_keys=True,separators=(',',':')).encode())==digest(json.dumps(documents['base_contract'],sort_keys=True,separators=(',',':')).encode()),'exact original SCAN contract data')
    v3=documents['historical_v3_adapter'];guard=data['guard'];validator=data['validator']
    require(set(guard)=={'path','before_sha256','after_sha256','beforeimage','substitutions'}
        and guard['path']==str(ROOT/'r85_session_guard.py')and guard['before_sha256']==OLD_GUARD_SHA
        and guard['substitutions']==GUARD_SUBSTITUTIONS,'guard exact declaration')
    before=file_ref(guard['beforeimage'],STAGE/'beforeimage/r85_session_guard.py',OLD_GUARD_SHA)
    expected=guard_after_bytes(before.read_bytes());require(digest(expected)==guard['after_sha256']
        and (ROOT/'r85_session_guard.py').read_bytes()==expected,'guard actual exact three substitutions')
    require(v3['controls'][guard['path']]['after_sha256']==OLD_GUARD_SHA,'v3 guard historical identity')
    evidence.extend([before,ROOT/'r85_session_guard.py'])
    require(set(validator)=={'path','before_sha256','after_sha256','beforeimage','change'}
        and validator['path']==str(ROOT/'scan_workspace.py')and validator['before_sha256']==OLD_SCANNER_SHA
        and validator['change']=='Exact schedule-only bridge replacement table in pinned new helper; inverse recovers original fd893 bytes.'
        and v3['validator']['after_sha256']==OLD_SCANNER_SHA,'validator exact declaration')
    before=file_ref(validator['beforeimage'],STAGE/'beforeimage/scan_workspace.py',OLD_SCANNER_SHA)
    expected=scanner_after_bytes(before.read_bytes());require(digest(expected)==validator['after_sha256']
        and (ROOT/'scan_workspace.py').read_bytes()==expected,'validator actual exact schedule bridge')
    evidence.extend([before,ROOT/'scan_workspace.py'])
    helper=file_ref(data['bridge_helper'],ROOT/'schedule_renewal_adapter.py');evidence.append(helper)
    receipt=documents['baseline_async_install'];require(receipt.get('installed')is True
        and receipt.get('candidate_commit')==BASELINE_COMMIT and receipt.get('all_bound_paths_verified')is True,'real async baseline deployment')
    controls=['transition_gate.py','cascade_core.py','controller.py','shared_controller.py','teacher_wrapper.py']
    async_names=['r85_dynamic_obstacle_runtime.py','r85_dynamic_completed_scope.py','native_observation_spool.py','frozen_qualification.py']
    changes={row['relative']:row for row in receipt['changes']}
    expected_unchanged={str(ROOT/n):v3['controls'][str(ROOT/n)]['after_sha256']for n in controls}
    expected_unchanged.update({str(ROOT/n):changes[n]['after_sha256']for n in async_names})
    expected_unchanged.update({str(ROOT/'imu_causal_buffer.py'):v3['extra_helper']['after_sha256'],str(ROOT/'r85_preflight.py'):v3['extra_preflight']['after_sha256'],str(ROOT/'compact_archives.py'):'3623c25f5343542811725556ef4b75883c298e4c27d1011e92e1589fb89d69fc'})
    require(data['unchanged_sources']==expected_unchanged,'exact unchanged five controls/four async/helpers/preflight closure')
    for path,sha in expected_unchanged.items():evidence.append(file_ref(dict(path=path,sha256=sha)))
    profile=data['profile'];require(set(profile)=={'path','beforeimage','added_session_keys','changed_value_paths'}
        and profile['path']==str(ROOT/PROFILE)and profile['added_session_keys']==['schedule_renewal_proof','schedule_renewal_proof_sha256']
        and profile['changed_value_paths']==['r85_session.deadline_utc','diagnostic_observe_only.user_authorized_work_deadline_utc','diagnostic_observe_only.work_deadline_basis'],'profile exact change declaration')
    old=file_ref(profile['beforeimage'],STAGE/'beforeimage'/PROFILE)
    require(digest(old.read_bytes())==changes[PROFILE]['after_sha256'],'profile real async baseline identity')
    expected_profile=json.loads(old.read_text())
    expected_profile['r85_session']['deadline_utc']=SCHEDULE['deadline_utc']
    expected_profile['diagnostic_observe_only']['user_authorized_work_deadline_utc']=SCHEDULE['deadline_utc']
    expected_profile['diagnostic_observe_only']['work_deadline_basis']=SCHEDULE['basis']
    expected_profile['r85_session']['schedule_renewal_proof']=str(PROOF)
    expected_profile['r85_session']['schedule_renewal_proof_sha256']=proof_ref['sha256']
    actual_profile=json.loads((ROOT/PROFILE).read_text())
    require(actual_profile==expected_profile and session==actual_profile['r85_session'],'profile actual exact three values plus two proof keys')
    evidence.extend([old,ROOT/PROFILE])
    return dict(proof=proof_ref,guard=guard,validator=validator,bridge_helper=data['bridge_helper'],
        baseline_commit=BASELINE_COMMIT,authorization=data['authorization'],budget_review=data['budget_review'],
        root_scheduling_selection=data['root_scheduling_selection'],evidence=sorted(set(evidence)))


def attach_renewal(adapter,renewal):
    adapter['schedule_renewal']=renewal
    adapter['actual'][renewal['bridge_helper']['path']]=renewal['bridge_helper']['sha256']
    adapter['evidence']=sorted(set(adapter['evidence']+renewal['evidence']))
    return adapter


def resolved_renewal(resolved,renewal):
    target=resolved['resolved_adapter'];target['schema']='r85_heading_braking_schedule_renewal_resolved_bindings/v1'
    overrides={renewal[k]['path']:dict(before_sha256=renewal[k]['before_sha256'],after_sha256=renewal[k]['after_sha256'])for k in ('guard','validator')}
    # Guard has no original 817 binding; this is explicit metadata, not an extra _bindings override.
    target['overrides'][renewal['guard']['path']]=copy.deepcopy(overrides[renewal['guard']['path']])
    target['schedule_renewal']=dict(proof=copy.deepcopy(renewal['proof']),baseline_commit=BASELINE_COMMIT,
        overrides=overrides,authorization=copy.deepcopy(renewal['authorization']),budget_review=copy.deepcopy(renewal['budget_review']),
        root_scheduling_selection=copy.deepcopy(renewal['root_scheduling_selection']),runtime_acceptance_inherited=False,navigation_acceptance_established=False)
