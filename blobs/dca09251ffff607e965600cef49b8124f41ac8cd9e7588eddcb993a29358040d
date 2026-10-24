"""Exact controller seed-classifier identity layer over retained v3/schedule proofs.

Historical b1/c1 identities are checked against real retained beforeimages;
current source checks use actual new controller/scanner bytes, never aliases.
"""
from pathlib import Path
import copy
import json
import re
import schedule_renewal_adapter as historical_schedule
from schedule_renewal_adapter import (ROOT,WORK,STAGE,PROOF,PROFILE,BASELINE_COMMIT,
    OLD_SCANNER_SHA,OLD_GUARD_SHA,V3_SHA,BASE_SHA,REFERENCE_SHA,SCHEDULE,GUARD_SUBSTITUTIONS,
    require,digest,file_ref,scanner_after_bytes,guard_after_bytes)

REPAIR_STAGE=WORK/'handoff_seed_recovery_stage/source_integration'
REPAIR_HELPER=ROOT/'handoff_seed_recovery_adapter.py'
REPAIR_PROOF=ROOT/'R85_HANDOFF_SEED_RECOVERY_CONTROL_ADAPTER_CONTRACT.json'
REPAIR_PROFILE='profiles/r85_f1_f3_handoff_seed_recovery_r14_20261016.json'
REPAIR_BASELINE='e4005d67a767caefc818322efa309801f301db88'
SOURCE_REPO=WORK.parent/'candidate_source_history'
BINDINGS_SHA='5d1d15d454a954f58ed5171f481e24991bee1eb3a53244154eea366907ab8dfa'
CONTROLLER_BEFORE='b1ed12d5ccd66edb9b8b72b643d8c41f8a74f3010fa63f01205047eaca706805'
SCANNER_BEFORE='c1af8899bea6122b55beccc8a955dff2f20b806d71288a5269872bc1a9a00315'
HISTORICAL_HELPER_SHA='7a1c879085c55f42dfb7e7f863f64e4a77a0ea39e6343cad8fba18eecbefbf00'
HISTORICAL_PROOF_SHA='bf7960f228e137c2d3d2d2bbb44fa60378104ac8c5f7cf8e37c44bbc4621def2'
HISTORICAL_PROFILE_SHA='2edd6daeceacc8cfc798e674e0f1cbf001733085a178c3e9aa05cf103dae3f24'
BASELINE_INSTALL_SHA='f0af93ea5f57b926e544c88fc9e15e09f3beb0df977e2ae7c20dc7f62f488d58'
SCANNER_PATCHES=[('def _heading_control_adapter(session,base,renewal=None):', 'def _heading_control_adapter(session,base,renewal=None,repair=None):'), ("        if renewal is not None and filename=='r85_session_guard.py':live_control_sha=renewal['guard']['after_sha256']", "        if renewal is not None and filename=='r85_session_guard.py':live_control_sha=renewal['guard']['after_sha256']\n        if repair is not None and filename=='controller.py':live_control_sha=repair['controller']['after_sha256']"), ("        if str(path)in base['bindings_sha256']:overrides[str(path)]=copy.deepcopy(row)\n    helper=data.get('extra_helper');path=R85_CANDIDATE_ROOT/'imu_causal_buffer.py'", "        if str(path)in base['bindings_sha256']:\n            overrides[str(path)]=copy.deepcopy(row)\n            overrides[str(path)]['after_sha256']=live_control_sha\n    helper=data.get('extra_helper');path=R85_CANDIDATE_ROOT/'imu_causal_buffer.py'"), ("def _control_adapter(session,base):\n    if 'schedule_renewal_proof' in session or 'schedule_renewal_proof_sha256' in session:", "def _control_adapter(session,base):\n    if 'handoff_seed_recovery_proof' in session or 'handoff_seed_recovery_proof_sha256' in session:\n        from handoff_seed_recovery_adapter import prepare_repair,attach_repair\n        from schedule_renewal_adapter import attach_renewal\n        repair,historical,live=prepare_repair(session,base)\n        return attach_repair(attach_renewal(_heading_control_adapter(session,base,live,repair),historical),repair)\n    if 'schedule_renewal_proof' in session or 'schedule_renewal_proof_sha256' in session:"), ("        resolved_renewal(resolved,adapter['schedule_renewal'])\n    return resolved", "        resolved_renewal(resolved,adapter['schedule_renewal'])\n    if 'handoff_seed_recovery' in adapter:\n        from handoff_seed_recovery_adapter import resolved_repair\n        resolved_repair(resolved,adapter['handoff_seed_recovery'])\n    return resolved")]
CONTROLLER_DELTA=[{'old_start': 25, 'old_end': 25, 'new_start': 25, 'new_end': 59, 'old_lines': [], 'new_lines': ['\n', '\n', 'def _pre_turn_external_hold_seed_wait(previous,cascade,gate,context,mode,guard,diagnostic,command,clock_ns,wall_ns):\n', '    """Only the first validated source seed after our ordinary pre-turn hold.\n', '\n', '    This recognizes a zero recovery output, never a control/stop qualification.\n', '    All source, attitude and causal ACK checks have already run in core.update.\n', '    """\n', "    if not (isinstance(previous,dict) and previous.get('cascade')is cascade\n", "            and previous.get('context')==context and previous.get('uninterrupted')is True\n", "            and mode=='brake' and guard is None and gate.phase=='settle'\n", "            and getattr(gate,'controlled_braking',False) and getattr(gate,'handoff_active',False)\n", "            and getattr(gate,'handoff_failure',None)is None):return False\n", "    event=getattr(gate,'handoff_event',None)\n", "    if not (isinstance(event,dict) and event.get('status')=='pending'\n", "            and event.get('brake_entry_control_s')==clock_ns/1e9\n", "            and event.get('context',{}).get('request_id')==context[0]\n", "            and event.get('context',{}).get('waypoint_index')==context[1]\n", "            and event.get('context')==previous.get('gate_context')==gate.transition_context\n", "            and event.get('purpose')==previous.get('purpose')==gate.transition_purpose\n", '            and gate.handoff_deadline_s is not None and clock_ns/1e9<=gate.handoff_deadline_s\n', "            and 0<=clock_ns-previous['clock_ns']<=300_000_000\n", "            and 0<=wall_ns-previous['wall_ns']<=300_000_000):return False\n", "    stamp=diagnostic.get('feedback_pose_stamp_ns')\n", "    return bool(diagnostic.get('mode')=='recovering' and diagnostic.get('reason')=='external_hold'\n", "        and diagnostic.get('kind')=='recovery_source_seed' and diagnostic.get('history_kind')=='seed'\n", "        and diagnostic.get('math_step_executed')is False and diagnostic.get('controller_updated')is False\n", "        and diagnostic.get('failure_latched')is None and diagnostic.get('protection_active')is True\n", "        and diagnostic.get('header_dt_s')==0. and diagnostic.get('integral_dt_s')==0.\n", "        and cascade.protected is True and cascade.protection_reason=='external_hold'\n", '        and cascade.recovery==[stamp] and cascade.last_seen_pose_stamp==stamp\n', '        and cascade.last_math_stamp==stamp and cascade.last_math_wall==wall_ns\n', '        and np.asarray(command).shape==(3,) and np.array_equal(command,np.zeros(3))\n', "        and diagnostic.get('command_body')==[0.,0.,0.])\n"]}, {'old_start': 1293, 'old_end': 1293, 'new_start': 1327, 'new_end': 1334, 'old_lines': [], 'new_lines': ['        context=(self.request_id,self.waypoint_index)\n', "        previous=getattr(self,'_pre_turn_hold_seed_context',None)\n", '        if previous is not None:\n', '            previous=dict(previous,uninterrupted=bool(self.cascade.protected\n', "                and self.cascade.protection_reason=='external_hold' and not self.cascade.recovery\n", '                and self.cascade.last_math_stamp is None\n', "                and self.cascade.last_clock==previous['clock_ns'] and self.cascade.last_wall==previous['wall_ns']))\n"]}, {'old_start': 1295, 'old_end': 1295, 'new_start': 1336, 'new_end': 1347, 'old_lines': [], 'new_lines': ['        seed_wait=_pre_turn_external_hold_seed_wait(previous,self.cascade,self.heading_gate,\n', '            context,mode,guard,diagnostic,cmd,ns,wall)\n', '        # Consume the token on any non-pre-turn call; a duplicate or another\n', '        # protection cannot reuse this single valid-source seed exception.\n', '        self._pre_turn_hold_seed_context=(dict(cascade=self.cascade,context=context,clock_ns=ns,wall_ns=wall,\n', '            gate_context=copy.deepcopy(self.heading_gate.transition_context),purpose=self.heading_gate.transition_purpose)\n', "            if mode=='hold' and self.heading_gate.phase=='pre_turn' and guard is None\n", "            and not getattr(self.heading_gate,'handoff_active',False)\n", "            and diagnostic.get('mode')=='protect' and diagnostic.get('reason')=='external_hold'\n", "            and diagnostic.get('failure_latched')is None else None)\n", "        if seed_wait:diagnostic['handoff_seed_recovery_wait']=True\n"]}, {'old_start': 1297, 'old_end': 1298, 'new_start': 1349, 'new_end': 1351, 'old_lines': ["                     and not (mode=='hold' and diagnostic.get('reason')=='external_hold'))):\n"], 'new_lines': ["                     and not (mode=='hold' and diagnostic.get('reason')=='external_hold'))\n", '                and not seed_wait):\n']}]
HISTORICAL_PATCHES=[('def load_renewal(session,base):', 'def _load_historical_schedule(session,base,repair):'), ("require(Path(__file__).resolve()==ROOT/'schedule_renewal_adapter.py','active helper location')", "require(Path(__file__).resolve()==ROOT/'handoff_seed_recovery_adapter.py','active repair helper location')"), ("and (ROOT/'scan_workspace.py').read_bytes()==expected,'validator actual exact schedule bridge')", "and _verified_repaired_scanner(expected,(ROOT/'scan_workspace.py').read_bytes(),repair),'validator historical schedule and actual repair bridge')"), ('    for path,sha in expected_unchanged.items():evidence.append(file_ref(dict(path=path,sha256=sha)))', "    for path,sha in expected_unchanged.items():\n        if path==str(ROOT/'controller.py'):\n            _verified_repaired_controller(path,sha,repair)\n            evidence.extend([Path(path),Path(repair['controller']['beforeimage']['path'])])\n        else:evidence.append(file_ref(dict(path=path,sha256=sha)))")]


def repair_scanner_after_bytes(raw):
    require(digest(raw)==SCANNER_BEFORE,'repair scanner historical c1 beforeimage')
    text=raw.decode()
    for index,(old,new)in enumerate(SCANNER_PATCHES):
        if index==2:
            lo=text.index('def _heading_control_adapter(');hi=text.index('\n\ndef _adapter_file_ref',lo)
            part=text[lo:hi];require(part.count(old)==1,'repair scoped scanner occurrence')
            text=text[:lo]+part.replace(old,new)+text[hi:]
        else:
            require(text.count(old)==1,'repair scanner occurrence');text=text.replace(old,new)
    return text.encode()


def repair_controller_after_bytes(raw):
    require(digest(raw)==CONTROLLER_BEFORE,'repair controller historical b1 beforeimage')
    lines=raw.decode().splitlines(keepends=True)
    for change in reversed(CONTROLLER_DELTA):
        lo,hi=change['old_start'],change['old_end']
        require(lines[lo:hi]==change['old_lines'],'exact reviewed controller old delta')
        lines[lo:hi]=change['new_lines']
    return ''.join(lines).encode()


def _verified_repaired_scanner(historical_bytes,actual_bytes,repair):
    row=repair['validator']
    before=file_ref(row['beforeimage'],REPAIR_STAGE/'beforeimage/scan_workspace.py',SCANNER_BEFORE)
    require(historical_bytes==before.read_bytes(),'schedule c1 identity equals actual retained historical beforeimage')
    expected=repair_scanner_after_bytes(historical_bytes)
    require(actual_bytes==expected and digest(actual_bytes)==row['after_sha256'],'actual new scanner exact repair bridge')
    return True


def _verified_repaired_controller(path,historical_sha,repair):
    row=repair['controller'];require(path==str(ROOT/'controller.py')and historical_sha==CONTROLLER_BEFORE,'controller only historical b1 identity')
    before=file_ref(row['beforeimage'],REPAIR_STAGE/'beforeimage/controller.py',CONTROLLER_BEFORE)
    expected=repair_controller_after_bytes(before.read_bytes())
    require((ROOT/'controller.py').read_bytes()==expected and digest(expected)==row['after_sha256'],'actual new controller exact reviewed classifier delta')
    return True


def _load_historical_schedule(session,base,repair):
    require(Path(__file__).resolve()==ROOT/'handoff_seed_recovery_adapter.py','active repair helper location')
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
        and _verified_repaired_scanner(expected,(ROOT/'scan_workspace.py').read_bytes(),repair),'validator historical schedule and actual repair bridge')
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
    for path,sha in expected_unchanged.items():
        if path==str(ROOT/'controller.py'):
            _verified_repaired_controller(path,sha,repair)
            evidence.extend([Path(path),Path(repair['controller']['beforeimage']['path'])])
        else:evidence.append(file_ref(dict(path=path,sha256=sha)))
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


def _binding_map(document):
    require(document.get('schema')=='r85_candidate_source_commit_binding/v1'
        and document.get('actual_candidate_root')==str(ROOT)
        and document.get('repository_is_evidence_copy_not_runtime')is True,'source binding document scope')
    rows=document.get('entries');require(isinstance(rows,list),'source binding entries')
    result={}
    for row in rows:
        path=Path(row['active']);relative=str(path.relative_to(ROOT))
        require(path.is_absolute()and str(path.resolve())==str(path)and row['tracked']=='candidate/'+relative
            and row['active']not in result and re.fullmatch('[0-9a-f]{64}',row['sha256'])is not None,'source binding exact path/hash')
        result[row['active']]=row
    return result


def _verify_binding_closure(proof,profile_sha,proof_sha):
    baseline=file_ref(proof['baseline_source_bindings'],REPAIR_STAGE/'beforeimage/SOURCE_BINDINGS.json',BINDINGS_SHA)
    old=_binding_map(json.loads(baseline.read_text()));require(len(old)==252,'actual old e400 252 binding snapshot')
    require(old[str(ROOT/'controller.py')]['sha256']==CONTROLLER_BEFORE
        and old[str(ROOT/'scan_workspace.py')]['sha256']==SCANNER_BEFORE,'e400 controller/scanner snapshot identities')
    expected={path:row['sha256']for path,row in old.items()}
    expected[proof['controller']['path']]=proof['controller']['after_sha256']
    expected[proof['validator']['path']]=proof['validator']['after_sha256']
    added={str(REPAIR_HELPER):proof['bridge_helper']['sha256'],str(REPAIR_PROOF):proof_sha,str(ROOT/REPAIR_PROFILE):profile_sha}
    require(not(set(added)&set(expected)),'three genuinely new repair bound paths')
    expected.update(added)
    current_path=SOURCE_REPO/'SOURCE_BINDINGS.json';current=_binding_map(json.loads(current_path.read_text()))
    require(len(current)==255 and set(current)==set(expected),'actual current exact 252 to255 source closure')
    evidence=[baseline,current_path]
    for path,sha in expected.items():
        row=current[path];active=file_ref(dict(path=path,sha256=sha))
        require(row['sha256']==sha and row.get('bytes')==active.stat().st_size,'current truthful source binding SHA/bytes')
        tracked=SOURCE_REPO/row['tracked']
        require(tracked.is_file()and not tracked.is_symlink()and digest(tracked.read_bytes())==sha,'tracked evidence source matches actual runtime')
        evidence.append(active)
    return evidence


def load_repair(session,base):
    require(Path(__file__).resolve()==REPAIR_HELPER,'actual new repair helper location')
    proof_ref=dict(path=session.get('handoff_seed_recovery_proof'),sha256=session.get('handoff_seed_recovery_proof_sha256'))
    path=file_ref(proof_ref,REPAIR_PROOF);data=json.loads(path.read_text())
    keys={'schema','kind','baseline_commit','expected_bound_source_count_before','expected_bound_source_count_after','exact_targets','simulation_only','runtime_acceptance_inherited','navigation_acceptance_established','source_or_feedback_guards_changed','physical_thresholds_changed','scan_source_changed','scan_binary_changed','scan_contract_bytes_changed','baseline_install','baseline_source_bindings','historical_schedule_helper','historical_schedule_proof','historical_R13_profile','controller','validator','bridge_helper','profile','meaning'}
    require(set(data)==keys and data['schema']=='r85_handoff_seed_recovery_source_adapter/v1'
        and data['kind']=='external_hold_recovery_seed_classifier_v1'and data['baseline_commit']==REPAIR_BASELINE
        and data['expected_bound_source_count_before']==252 and data['expected_bound_source_count_after']==255
        and data['exact_targets']==['controller.py','scan_workspace.py','handoff_seed_recovery_adapter.py','R85_HANDOFF_SEED_RECOVERY_CONTROL_ADAPTER_CONTRACT.json',REPAIR_PROFILE]
        and data['simulation_only']is True and all(data[k]is False for k in ('runtime_acceptance_inherited','navigation_acceptance_established','source_or_feedback_guards_changed','physical_thresholds_changed','scan_source_changed','scan_binary_changed','scan_contract_bytes_changed')),'exact repair proof authority and scope')
    evidence=[path]
    baseline=file_ref(data['baseline_install'],WORK/'install_tools/INSTALL_RECEIPT.json',BASELINE_INSTALL_SHA)
    receipt=json.loads(baseline.read_text());require(receipt.get('installed')is True and receipt.get('candidate_commit')==REPAIR_BASELINE,'real e400 installation')
    oldhelper=file_ref(data['historical_schedule_helper'],ROOT/'schedule_renewal_adapter.py',HISTORICAL_HELPER_SHA)
    require(Path(historical_schedule.__file__).resolve()==oldhelper,'real unmodified historical schedule module')
    oldproof=file_ref(data['historical_schedule_proof'],PROOF,HISTORICAL_PROOF_SHA)
    oldprofile=file_ref(data['historical_R13_profile'],REPAIR_STAGE/'beforeimage'/PROFILE,HISTORICAL_PROFILE_SHA)
    activeold=file_ref(dict(path=str(ROOT/PROFILE),sha256=HISTORICAL_PROFILE_SHA),ROOT/PROFILE)
    evidence.extend([baseline,oldhelper,oldproof,oldprofile,activeold])
    controller=data['controller'];validator=data['validator']
    require(set(controller)=={'path','before_sha256','after_sha256','beforeimage','delta_sha256'}
        and controller['path']==str(ROOT/'controller.py')and controller['before_sha256']==CONTROLLER_BEFORE
        and controller['delta_sha256']==digest(json.dumps(CONTROLLER_DELTA,sort_keys=True,separators=(',',':')).encode()),'reviewed exact controller declaration')
    _verified_repaired_controller(controller['path'],CONTROLLER_BEFORE,data)
    require(set(validator)=={'path','before_sha256','after_sha256','beforeimage','change'}
        and validator['path']==str(ROOT/'scan_workspace.py')and validator['before_sha256']==SCANNER_BEFORE
        and validator['change']=='Exact new repair dispatch, controller current identity/base-three override and resolved metadata only.','exact validator declaration')
    before=file_ref(validator['beforeimage'],REPAIR_STAGE/'beforeimage/scan_workspace.py',SCANNER_BEFORE)
    _verified_repaired_scanner(before.read_bytes(),(ROOT/'scan_workspace.py').read_bytes(),data)
    helper=file_ref(data['bridge_helper'],REPAIR_HELPER)
    profile=data['profile'];require(set(profile)=={'path','baseline_profile','added_session_keys'}
        and profile['path']==str(ROOT/REPAIR_PROFILE)and profile['baseline_profile']==dict(path=str(ROOT/PROFILE),sha256=HISTORICAL_PROFILE_SHA)
        and profile['added_session_keys']==['handoff_seed_recovery_proof','handoff_seed_recovery_proof_sha256'],'exact new R14 profile scope')
    expected=json.loads(oldprofile.read_text())
    expected['r85_session']['handoff_seed_recovery_proof']=str(REPAIR_PROOF)
    expected['r85_session']['handoff_seed_recovery_proof_sha256']=proof_ref['sha256']
    profile_path=ROOT/REPAIR_PROFILE;profile_raw=profile_path.read_bytes();actual=json.loads(profile_raw)
    require(actual==expected and session==actual['r85_session'],'new R14 profile only exact two repair identity keys')
    evidence.extend([helper,before,Path(controller['beforeimage']['path']),profile_path])
    evidence.extend(_verify_binding_closure(data,digest(profile_raw),proof_ref['sha256']))
    data['proof']=proof_ref;data['evidence']=sorted(set(evidence));return data


def prepare_repair(session,base):
    repair=load_repair(session,base)
    historical_session=copy.deepcopy(session)
    for key in ('handoff_seed_recovery_proof','handoff_seed_recovery_proof_sha256'):del historical_session[key]
    historical=_load_historical_schedule(historical_session,base,repair)
    live=copy.deepcopy(historical)
    live['validator']['after_sha256']=repair['validator']['after_sha256']
    return repair,historical,live


def attach_repair(adapter,repair):
    adapter['handoff_seed_recovery']=repair
    adapter['actual'][repair['bridge_helper']['path']]=repair['bridge_helper']['sha256']
    adapter['evidence']=sorted(set(adapter['evidence']+repair['evidence']))
    return adapter


def resolved_repair(resolved,repair):
    target=resolved['resolved_adapter'];target['schema']='r85_handoff_seed_recovery_resolved_bindings/v1'
    target['schedule_renewal']['source_identity_status']='historical_pre_repair'
    target['schedule_renewal']['actual_identity_superseded_by']=copy.deepcopy(repair['proof'])
    overrides={repair[k]['path']:dict(before_sha256=repair[k]['before_sha256'],after_sha256=repair[k]['after_sha256'])for k in ('controller','validator')}
    require(all(target['actual_control_sources_sha256'][p]==row['after_sha256']
        and target['overrides'][p]['after_sha256']==row['after_sha256']for p,row in overrides.items()),'resolved actual/three-base-overrides contain truthful new source SHA')
    target['handoff_seed_recovery']=dict(proof=copy.deepcopy(repair['proof']),baseline_commit=REPAIR_BASELINE,
        overrides=overrides,historical_v3_and_schedule_proof_bytes_preserved=True,
        old_R13_profile_unchanged=True,guard_and_async_sources_unchanged=True,
        runtime_acceptance_inherited=False,navigation_acceptance_established=False)
