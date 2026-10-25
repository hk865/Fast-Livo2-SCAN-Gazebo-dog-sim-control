"""R43 explicit Teacher positive heading-P taper over the real R42 checkpoint.

The checkpoint supplies historical source identity only. Current active bytes,
the exact current binding map, original SCAN live checks, source authorization,
causal ACK checks and physical acceptance retain their separate owners.
"""
from pathlib import Path
import copy,hashlib,json,re

ROOT=Path('/opt/experiment/workspace/multifloor_demo/teacher_mode/navigation/engineering_v66_r85_turn_handoff_candidate')
WORK=Path('/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400')
STAGE=WORK/'round_43_teacher_heading_p_taper_native_validation'
REPO=WORK.parent/'candidate_source_history'
BASELINE='24ad11066a4640d8fc83531ac63f11577c6d1912'
PROFILE='profiles/r85_f1_f3_teacher_heading_p_taper_native_r43_20261016.json'
OLD_PROFILE='profiles/r85_f1_f3_yaw_pd_fade_native_r42_20261016.json'
HELPER=ROOT/'teacher_heading_p_taper_native_adapter.py'
PROOF=ROOT/'R85_TEACHER_HEADING_P_TAPER_NATIVE_CONTROL_ADAPTER_CONTRACT.json'
BASE=ROOT/'R85_SCAN_REFERENCE_PROTOCOL_WORKSPACE_CONTRACT.json'
BASE_SHA='6a028cb1dc37dfab63840172b9e0ba35fec3e0b3d09aa0926e91747f229e5a67'
HISTORICAL=ROOT/'R85_HEADING_BRAKING_CONTROL_ADAPTER_CONTRACT.json'
HISTORICAL_SHA='3fb1795ada9a6296c0cb84e352f6c27a5996b7458f74ffc47274fc082dcc5058'
CHECKPOINT={'path': '/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/round_42_yaw_pd_fade_native_validation/install_tools/ACTUAL_POSTINSTALL_PROFILE_VALIDATION.json', 'sha256': '1add62a6e7aa9f88f572d73467975bb290bd0266c7908bcbf2467c688cb283ac'}
INSTALL={'path': '/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/round_42_yaw_pd_fade_native_validation/install_tools/INSTALL_RECEIPT.json', 'sha256': '1694bd2726f97997152a7f7b9d8ee65515f6ae3d9f4de20a61e73e0350b260df'}
SELECTION={'path': '/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/ROOT_TEACHER_POSITIVE_HEADING_P_TAPER_CANDIDATE_SELECTION.json', 'sha256': 'ec8b4b202284d360a523d4a6f3fad92f2ecc35c5ec89f6a1562508bdbb945f7b'}
BINDINGS_SHA='63e7d9dc77fa97bd6289d1f96f93832594bd03e0961f75730b1a1f5bf380c5ea'
OLD_PROFILE_SHA='a80579aeb56b64db4c3bcaf22c3c6165f10b05a61a3a0626679e86bdebcab5f8'
BEFORE={'cascade_core.py': '71d391b65e8e710a36eafa1456f367b21e405fa3bb42d6673dde5a4f79f73ad2', 'controller.py': 'bd12cffb11285aaa0b99c17b061bdf714d9b912c1707c512fbbd071c365cb275', 'r85_preflight.py': '2dfa70601870c6586f33050ffe9c00634e6b838e6c0da3ec5b78d0eb573d2029', 'scan_workspace.py': '33271619d58ec5d44d41678bc3f154cb53640c22bccb6f20395c02c48d84c29e'}
AFTER={'cascade_core.py': '282eb8ca637977149c9436e566675c2c3883ee34d044aaa454f59bae282ec83a', 'controller.py': '9c9548a38cf566e19cf5854189819386240feecffffe4534735258ba4f9da3c8', 'r85_preflight.py': '9f9ffe174006aa71af491e9770bd145fce21c7d2b4f43ca09521985fb197a7c5', 'scan_workspace.py': '18e3c31259b5b0001d951f9bf5e2c5257e817bb69a21d2ffc2dc2f134783cc05'}
DELTAS={'cascade_core.py': [{'old_start': 47, 'old_end': 47, 'new_start': 47, 'new_end': 75, 'old_lines': [], 'new_lines': ['    return copy.deepcopy(value)\n', '\n', '\n', 'def teacher_pre_zero_heading_p_taper_configuration(profile):\n', '    """Explicit R43 heading-P law; original gains remain the outside-event base."""\n', "    if not isinstance(profile,dict):raise ValueError('Actual full profile required for Teacher heading-P taper')\n", "    key='r85_teacher_pre_zero_heading_p_taper'\n", '    if key not in profile:return None\n', '    value=profile[key]\n', "    if isinstance(value,dict) and set(value)=={'enabled'} and value['enabled']is False:return None\n", "    expected=dict(enabled=True,kind='accepted_pre_zero_event_positive_heading_P_taper_v1',\n", '        base_kp=1.3,target_kp=.9,duration_s=1.)\n', '    if (not isinstance(value,dict) or set(value)!=set(expected) or value!=expected\n', "            or value['enabled']is not True or type(value['kind'])is not str\n", '            or any(type(value[k])not in (int,float) or not math.isfinite(value[k])\n', "                for k in ('base_kp','target_kp','duration_s'))):\n", "        raise ValueError('Exact selected Teacher positive heading-P taper configuration required')\n", "    for key,required in (('r85_active_turn_braking',True),('r85_quiet_handoff_target',True),\n", "                         ('r85_neutral_rate_brake_capture',False)):\n", '        setting=profile.get(key)\n', "        if not isinstance(setting,dict) or set(setting)!={'enabled'} or setting['enabled']is not required:\n", "            raise ValueError('Teacher taper requires original active quiet locked-heading brake and neutral off')\n", "    cascade=profile.get('cascade')\n", "    if (not isinstance(cascade,dict) or cascade.get('external_heading_gate')is not True\n", "            or cascade.get('r85_turn_brake_reference')!={'kind':'locked_heading_pd_v1'}):\n", "        raise ValueError('Teacher taper requires the original external locked-heading cascade')\n", '    from pre_zero_integral_unload import configuration as unload_configuration\n', "    if unload_configuration(profile)is None:raise ValueError('Teacher taper requires the existing owned unload event')\n"]}, {'old_start': 129, 'old_end': 129, 'new_start': 157, 'new_end': 158, 'old_lines': [], 'new_lines': ['        self.teacher_pre_zero_heading_p_taper_config = None\n']}, {'old_start': 391, 'old_end': 391, 'new_start': 420, 'new_end': 436, 'old_lines': [], 'new_lines': ['    def configure_teacher_pre_zero_heading_p_taper(self,profile,run_id=None,scope=None):\n', '        """Bind exact new math before the first control clock; no live retuning."""\n', '        setting=teacher_pre_zero_heading_p_taper_configuration(profile)\n', '        if self.last_clock is not None:\n', "            raise ValueError('Teacher heading-P taper cannot change after a control event')\n", "        if setting is not None and (profile.get('cascade')!=self.config\n", "                or not self.external_heading_gate or self.turn_brake_reference_kind!='locked_heading_pd_v1'):\n", "            raise ValueError('Teacher heading-P taper must bind the actual locked-heading cascade')\n", '        if setting is not None:\n', '            from pre_zero_integral_unload import valid_sha\n', '            if not isinstance(run_id,str) or not run_id or not valid_sha(scope):\n', "                raise ValueError('Teacher heading-P taper requires the actual run and source scope')\n", '        self.teacher_pre_zero_heading_p_taper_config=setting\n', '        self.teacher_pre_zero_heading_p_taper_run_id=run_id\n', '        self.teacher_pre_zero_heading_p_taper_scope=scope\n', '\n']}, {'old_start': 459, 'old_end': 459, 'new_start': 504, 'new_end': 505, 'old_lines': [], 'new_lines': ['        teacher_heading_p_taper = None\n']}, {'old_start': 466, 'old_end': 466, 'new_start': 512, 'new_end': 515, 'old_lines': [], 'new_lines': ['                if self.teacher_pre_zero_heading_p_taper_config is not None:\n', '                    validate_event(pre_zero_unload_event,self.teacher_pre_zero_heading_p_taper_run_id,\n', '                        self.teacher_pre_zero_heading_p_taper_scope,clock)\n']}, {'old_start': 774, 'old_end': 774, 'new_start': 823, 'new_end': 839, 'old_lines': [], 'new_lines': ['            if (self.teacher_pre_zero_heading_p_taper_config is not None\n', "                    and mode=='brake' and pre_zero_unload_event is not None):\n", '                # validate_event above owns the fixed entry/expiry. This control\n', '                # clock is never a renewed event or a wall-time estimate.\n', '                setting=self.teacher_pre_zero_heading_p_taper_config\n', "                elapsed_s=(clock-pre_zero_unload_event['brake_entry_clock_ns'])/1e9\n", "                progress=min(1.,max(0.,elapsed_s/setting['duration_s']))\n", "                effective_kp=setting['base_kp']+(setting['target_kp']-setting['base_kp'])*progress\n", '                yaw_P=effective_kp*error_yaw\n', "                teacher_heading_p_taper=dict(schema='r85_teacher_pre_zero_heading_p_taper/v1',\n", "                    kind=setting['kind'],event=copy.deepcopy(pre_zero_unload_event),control_clock_ns=clock,\n", "                    elapsed_s=elapsed_s,progress=progress,base_kp=setting['base_kp'],target_kp=setting['target_kp'],\n", "                    effective_kp=effective_kp,duration_s=setting['duration_s'],kd=.15,\n", '                    heading_error_rad=float(error_yaw),yaw_P=float(yaw_P),yaw_D=float(yaw_D),\n', '                    inside_authorized_brake=True,event_outside_restores_base_immediately=True,\n', '                    new_heading_P_math=True,stop_credit=False)\n']}, {'old_start': 931, 'old_end': 931, 'new_start': 996, 'new_end': 1002, 'old_lines': [], 'new_lines': ['        if teacher_heading_p_taper is not None:\n', "            self.row['teacher_pre_zero_heading_p_taper']=teacher_heading_p_taper\n", "            self.row['active_turn_braking'].update(original_gains_and_limits=False,\n", "                original_limits_unchanged=True,original_heading_PD_gains_role='outside_event_base_only',\n", "                effective_heading_PD_gains=dict(kp=teacher_heading_p_taper['effective_kp'],kd=.15),\n", '                new_positive_heading_P_taper=True)\n']}], 'controller.py': [{'old_start': 1260, 'old_end': 1260, 'new_start': 1260, 'new_end': 1263, 'old_lines': [], 'new_lines': ["            if 'r85_teacher_pre_zero_heading_p_taper'in self.profile:\n", '                self.cascade.configure_teacher_pre_zero_heading_p_taper(\n', '                    self.profile,self.run.name,self.handoff_scope_sha256)\n']}], 'r85_preflight.py': [{'old_start': 43, 'old_end': 43, 'new_start': 43, 'new_end': 46, 'old_lines': [], 'new_lines': ["    if 'r85_teacher_pre_zero_heading_p_taper' in p:\n", '        from cascade_core import teacher_pre_zero_heading_p_taper_configuration\n', '        teacher_pre_zero_heading_p_taper_configuration(p)\n']}], 'scan_workspace.py': [{'old_start': 778, 'old_end': 778, 'new_start': 778, 'new_end': 781, 'old_lines': [], 'new_lines': ["    if 'teacher_heading_p_taper_native_proof' in session or 'teacher_heading_p_taper_native_proof_sha256' in session:\n", '        from teacher_heading_p_taper_native_adapter import load_teacher_heading_p_taper_native\n', '        return load_teacher_heading_p_taper_native(session,base)\n']}, {'old_start': 934, 'old_end': 935, 'new_start': 937, 'new_end': 941, 'old_lines': ["    if 'yaw_pd_fade_native' in adapter:\n"], 'new_lines': ["    if 'teacher_heading_p_taper_native' in adapter:\n", '        from teacher_heading_p_taper_native_adapter import resolved_teacher_heading_p_taper_native\n', '        resolved_teacher_heading_p_taper_native(resolved,adapter)\n', "    elif 'yaw_pd_fade_native' in adapter:\n"]}]}
NEW_SOURCE_SHA={}
PROFILE_DELTA=[{'path': ['r85_teacher_pre_zero_heading_p_taper'], 'old_present': False, 'new_value': {'enabled': True, 'kind': 'accepted_pre_zero_event_positive_heading_P_taper_v1', 'base_kp': 1.3, 'target_kp': 0.9, 'duration_s': 1.0}}]
CONFIG={'enabled':True,'kind':'accepted_pre_zero_event_positive_heading_P_taper_v1','base_kp':1.3,'target_kp':.9,'duration_s':1.0}
METHOD='verified_R42_source_checkpoint_plus_exact_current_delta_v1'
IDENTITY_CHECKS=dict(runtime_all_active_SHA=True,runtime_current_binding_map_exact=True,
    tracked_files_are_evidence_copies_not_runtime_inputs=True,installer_active_tracked_copy_verification_required=True)
TARGETS=list(BEFORE)+list(NEW_SOURCE_SHA)+[HELPER.name,PROOF.name,PROFILE]
CHANGE=dict(Teacher_heading_P_math_changed=True,Student_yaw_PD_fade_unchanged=True,
    accepted_owned_prezero_event_only=True,active_brake_only=True,stateless_event_clock_progress=True,
    base_heading_Kp=1.3,target_heading_Kp=.9,taper_duration_s=1.,
    original_heading_Kd_unchanged=.15,outside_event_heading_Kp=1.3,
    outside_event_gain_restore_is_immediate=True,global_gain_rate_bound_claim=False,
    locked_heading_target_unchanged=True,velocity_PI_and_unload_and_antiwindup_unchanged=True,
    original_limits_and_single_command_slew_unchanged=True,quiet_and_brake_thresholds_unchanged=True,
    source_authorization_and_freshness_unchanged=True,original_business_acceptance_unchanged=True,
    formal_T0_moved=False,performance_or_physical_safety_proven=False)

def require(ok,reason):
    if not ok:raise RuntimeError('R43 source identity: '+reason)

def digest(raw):return hashlib.sha256(raw).hexdigest()

def same(a,b):
    if type(a)is not type(b):return False
    if isinstance(a,dict):return set(a)==set(b)and all(same(a[k],b[k])for k in a)
    if isinstance(a,list):return len(a)==len(b)and all(same(x,y)for x,y in zip(a,b))
    return a==b

def read_ref(row,path=None,sha=None):
    require(isinstance(row,dict)and set(row)=={'path','sha256'},'exact file reference')
    require(isinstance(row['path'],str)and isinstance(row['sha256'],str),'typed file reference')
    p=Path(row['path']);h=row['sha256']
    require(p.is_absolute()and not p.is_symlink()and p.is_file()and p.resolve()==p
        and re.fullmatch('[0-9a-f]{64}',h)is not None,'regular pinned file')
    require((path is None or p==path)and(sha is None or h==sha),'declared exact path/hash')
    raw=p.read_bytes();require(digest(raw)==h,'actual bytes: '+str(p));return p,raw

def binding_map(raw,count,root=ROOT):
    data=json.loads(raw);rows=data.get('entries');require(isinstance(rows,list)and len(rows)==count,'complete source count')
    result={}
    for row in rows:
        require(isinstance(row,dict)and set(row)=={'active','tracked','sha256','bytes'},'exact source binding row')
        p=Path(row['active']);t=Path(row['tracked'])
        require(p.is_absolute()and p.is_relative_to(root)and '..'not in p.parts
            and not t.is_absolute()and '..'not in t.parts and type(row['bytes'])is int and row['bytes']>0
            and isinstance(row['sha256'],str)and re.fullmatch('[0-9a-f]{64}',row['sha256'])is not None,'source binding scope/types')
        require(str(p)not in result,'no repeated source');result[str(p)]=row
    return result

def verify_current_tree(expected_hashes,binding_path,root=ROOT):
    """Actual file/hash verifier. Runtime uses fixed ROOT; CPU may pass a real
    staged mirror and a map whose active paths name that mirror explicitly.
    No installed identity or source authorization is granted to a staged tree.
    """
    root=Path(root);binding_path=Path(binding_path)
    require(root.is_absolute()and root.resolve()==root and not root.is_symlink(),'exact tree root')
    require(type(expected_hashes)is dict and len(expected_hashes)==277,'complete277 expected tree')
    require(all(Path(p).is_relative_to(root)and '..'not in Path(p).parts for p in expected_hashes),'expected tree paths')
    require(binding_path.is_file()and not binding_path.is_symlink(),'regular current binding map')
    current=binding_map(binding_path.read_bytes(),277,root)
    require(set(current)==set(expected_hashes),'exact274 to277 current source closure')
    for p,h in expected_hashes.items():
        row=current[p];_,raw=read_ref(dict(path=p,sha256=h))
        require(row['sha256']==h and len(raw)==row['bytes'],'complete actual current bytes: '+p)
    return dict(actual=dict(expected_hashes),actual_read_root=str(root),actual_read_count=len(current),
        runtime_C_identity_verified=(root==ROOT),staged_tree_only=(root!=ROOT))


def after_bytes(name,raw):
    require(name in BEFORE and digest(raw)==BEFORE[name],'exact predecessor source: '+name)
    lines=raw.decode().splitlines(keepends=True)
    for row in reversed(DELTAS[name]):
        i,j=row['old_start'],row['old_end']
        require(lines[i:j]==row['old_lines'],'exact delta context: '+name)
        lines[i:j]=row['new_lines']
    out=''.join(lines).encode()
    require(digest(out)==AFTER[name],'actual after source: '+name);return out

def expected_profile(old,proof_sha):
    profile=copy.deepcopy(old)
    for change in PROFILE_DELTA:
        target=profile
        for key in change['path'][:-1]:target=target[key]
        key=change['path'][-1]
        require((key in target)is change['old_present'],'profile predecessor key')
        if change['old_present']:require(same(target[key],change['old_value']),'profile exact predecessor value')
        target[key]=copy.deepcopy(change['new_value'])
    session=profile['r85_session']
    require('teacher_heading_p_taper_native_proof'not in session and 'teacher_heading_p_taper_native_proof_sha256'not in session,'new proof keys only')
    session['teacher_heading_p_taper_native_proof']=str(PROOF)
    session['teacher_heading_p_taper_native_proof_sha256']=proof_sha
    return profile

def validate_metadata(data):
    keys={'schema','baseline_commit','identity_verification_method','expected_bound_source_count_before',
        'expected_bound_source_count_after','exact_targets','baseline_source_bindings','predecessor_installation',
        'actual_predecessor_validation','source_identity_checks','historical_adapter','base_contract','baseline_profile','sources',
        'bridge_helper','profile','configuration','profile_delta','new_sources','change_declaration','selection',
        'runtime_acceptance_inherited','current_source_authorization_inherited','navigation_acceptance_established','physical_thresholds_changed'}
    require(isinstance(data,dict)and set(data)==keys,'exact proof keys')
    require(data['schema']=='r85_teacher_heading_P_taper_native_current_source_delta/v1'and data['baseline_commit']==BASELINE
        and data['identity_verification_method']==METHOD and data['exact_targets']==TARGETS
        and type(data['expected_bound_source_count_before'])is int and data['expected_bound_source_count_before']==274
        and type(data['expected_bound_source_count_after'])is int and data['expected_bound_source_count_after']==277,'proof identity/scope')
    require(all(data[k]is False for k in ['runtime_acceptance_inherited','current_source_authorization_inherited',
        'navigation_acceptance_established','physical_thresholds_changed']),'source identity only, no borrowed acceptance/authorization')
    require(same(data['configuration'],CONFIG)and same(data['change_declaration'],CHANGE)
        and same(data['profile_delta'],PROFILE_DELTA),'explicit new math/resources and exact profile delta')
    require(same(data['source_identity_checks'],IDENTITY_CHECKS),'active identity and installer-owned tracked evidence')
    require(data['predecessor_installation']==INSTALL and data['actual_predecessor_validation']==CHECKPOINT
        and data['selection']==SELECTION,'actual pinned predecessor and selection')
    require(data['base_contract']==dict(path=str(BASE),sha256=BASE_SHA)
        and data['historical_adapter']==dict(path=str(HISTORICAL),sha256=HISTORICAL_SHA),'unchanged original contract identities')
    require(set(data['sources'])==set(BEFORE),'exact existing-source delta set')
    for name,row in data['sources'].items():
        require(set(row)=={'path','before_sha256','after_sha256','beforeimage'}and row['path']==str(ROOT/name)
            and row['before_sha256']==BEFORE[name]and row['after_sha256']==AFTER[name],'true current delta: '+name)
    require(set(data['new_sources'])==set(NEW_SOURCE_SHA),'exact additional new source set')
    for name,row in data['new_sources'].items():
        require(row==dict(path=str(ROOT/name),sha256=NEW_SOURCE_SHA[name]),'new source identity: '+name)
    require(data['profile']==str(ROOT/PROFILE),'new unique selected profile')

def checkpoint(data,base):
    _,raw=read_ref(data['baseline_source_bindings'],STAGE/'source_integration/beforeimage/SOURCE_BINDINGS.json',BINDINGS_SHA)
    before=binding_map(raw,274)
    _,raw=read_ref(INSTALL);installation=json.loads(raw)
    _,raw=read_ref(CHECKPOINT);checked=json.loads(raw)
    require(installation.get('installed')is True and installation.get('candidate_commit')==BASELINE
        and installation['actual_bindings']['bound_source_count']==274
        and installation['actual_bindings']['all_active_tracked_SHA_equal']is True
        and installation['actual_bindings']['full_source_bindings']['sha256']==BINDINGS_SHA,'real274 source deployment')
    require(checked.get('schema')=='r85_R42_actual_postinstall_profile_validation/v1'
        and checked.get('candidate_commit')==BASELINE and checked.get('passed')is True
        and checked.get('actual_validate_profile_pass')is True and checked.get('actual_SCAN_resolver_pass')is True
        and checked.get('no_source_aliases_or_monkeypatches')is True and checked.get('bound_source_count')==274
        and checked['install_receipt']==INSTALL,'real preflight/SCAN checkpoint, identity only')
    require(checked['profile']==dict(path=str(ROOT/OLD_PROFILE),sha256=OLD_PROFILE_SHA)
        and checked['adapter']==dict(path=str(HISTORICAL),sha256=HISTORICAL_SHA),'checkpoint selected identities')
    runtime=checked.get('resolved_required_runtime_sources')
    require(isinstance(runtime,dict)and len(runtime)==10 and set(runtime)==set(base['required_runtime_sources'])
        and all(before[p]['sha256']==h for p,h in runtime.items()),'complete actual checkpoint runtime10')
    return before,runtime

def load_teacher_heading_p_taper_native(session,base):
    require(Path(__file__).resolve()==HELPER,'actual helper location')
    proof_ref=dict(path=session.get('teacher_heading_p_taper_native_proof'),sha256=session.get('teacher_heading_p_taper_native_proof_sha256'))
    _,raw=read_ref(proof_ref,PROOF);data=json.loads(raw);validate_metadata(data)
    require(session.get('scan_workspace_contract')==str(BASE)and session.get('scan_workspace_contract_sha256')==BASE_SHA
        and session.get('scan_control_adapter_contract')==str(HISTORICAL)
        and session.get('scan_control_adapter_contract_sha256')==HISTORICAL_SHA,'unchanged base/historical session declaration')
    _,base_raw=read_ref(data['base_contract'],BASE,BASE_SHA)
    require(same(json.loads(base_raw),base),'actual original SCAN data; full live verifier remains')
    before,runtime=checkpoint(data,base)
    old_path,old_raw=read_ref(data['baseline_profile'],ROOT/OLD_PROFILE,OLD_PROFILE_SHA)
    helper,_=read_ref(data['bridge_helper'],HELPER)
    selection,_=read_ref(SELECTION)
    evidence=[PROOF,BASE,HISTORICAL,old_path,helper,selection,Path(INSTALL['path']),Path(CHECKPOINT['path']),
        STAGE/'source_integration/beforeimage/SOURCE_BINDINGS.json']
    for name,row in data['sources'].items():
        p,raw=read_ref(row['beforeimage'],STAGE/'beforeimage'/name,BEFORE[name])
        require((ROOT/name).read_bytes()==after_bytes(name,raw),'actual existing delta bytes: '+name);evidence.extend([p,ROOT/name])
    for name,row in data['new_sources'].items():
        p,raw=read_ref(row,ROOT/name,NEW_SOURCE_SHA[name]);evidence.append(p)
        if name=='R85_RESOURCE_RENEWAL_UNTIL20261017T0000.json':
            for declaration in json.loads(raw)['source_authority']:
                p,b=read_ref(dict(path=declaration['path'],sha256=declaration['sha256']))
                require(len(b)==declaration['bytes'],'resource authority byte count');evidence.append(p)
    profile_path=ROOT/PROFILE;profile_raw=profile_path.read_bytes();profile=json.loads(profile_raw)
    require(same(profile,expected_profile(json.loads(old_raw),proof_ref['sha256']))
        and same(session,profile['r85_session']),'exact selected profile delta plus two identity keys')
    expected_hashes={p:r['sha256']for p,r in before.items()}
    for name in BEFORE:
        require(expected_hashes[str(ROOT/name)]==BEFORE[name],'baseline existing-source binding')
        expected_hashes[str(ROOT/name)]=AFTER[name]
    added={str(ROOT/name):h for name,h in NEW_SOURCE_SHA.items()}
    added.update({str(HELPER):data['bridge_helper']['sha256'],str(PROOF):proof_ref['sha256'],str(profile_path):digest(profile_raw)})
    require(not(set(added)&set(expected_hashes))and len(added)==3,'three genuinely new sources')
    expected_hashes.update(added)
    current_path=REPO/'SOURCE_BINDINGS.json'
    actual=verify_current_tree(expected_hashes,current_path,ROOT)['actual']
    # These original resource authority files are still actually read by the
    # unchanged resource configuration/guards; they are archive dependencies,
    # not inherited authorization for a new runtime sample.
    resource_path=ROOT/'R85_RESOURCE_RENEWAL_UNTIL20261017T0000.json'
    resource_expected=before[str(resource_path)]['sha256']
    _,resource_raw=read_ref(dict(path=str(resource_path),sha256=resource_expected))
    evidence.append(resource_path)
    for row in json.loads(resource_raw)['source_authority']:
        p,b=read_ref(dict(path=row['path'],sha256=row['sha256']))
        require(type(row['bytes'])is int and len(b)==row['bytes'],'unchanged resource authority size')
        evidence.append(p)
    overrides={str(ROOT/name):dict(before_sha256=base['bindings_sha256'][str(ROOT/name)],after_sha256=actual[str(ROOT/name)])
        for name in ['controller.py','shared_controller.py','scan_workspace.py']}
    _,v3_raw=read_ref(data['historical_adapter'],HISTORICAL,HISTORICAL_SHA)
    evidence.extend([profile_path,current_path]);data['proof']=proof_ref
    data['actual_runtime10']={p:actual[p]for p in runtime};data['evidence']=sorted(set(evidence))
    return dict(path=HISTORICAL,sha256=HISTORICAL_SHA,data=json.loads(v3_raw),actual=actual,
        overrides=overrides,evidence=data['evidence'],teacher_heading_p_taper_native=data)

def resolved_teacher_heading_p_taper_native(resolved,adapter):
    data=adapter['teacher_heading_p_taper_native'];target=resolved['resolved_adapter']
    require(resolved['required_runtime_sources']==data['actual_runtime10'],'truthful current runtime10')
    # The old v3 bytes still describe their historical preflight. The original
    # scanner copies that row before this selected dispatch; do not mislabel its
    # historical SHA as the current renewed preflight source.
    if 'extra_preflight'in target:
        target['historical_extra_preflight']=copy.deepcopy(target['extra_preflight'])
    current_preflight=copy.deepcopy(data['sources']['r85_preflight.py'])
    require(current_preflight['after_sha256']==AFTER['r85_preflight.py'],'truthful current preflight metadata')
    target['extra_preflight']=current_preflight
    target['schema']='r85_teacher_heading_P_taper_native_current_checkpoint_resolved/v1'
    target['teacher_heading_p_taper_native']=dict(proof=copy.deepcopy(data['proof']),baseline_commit=BASELINE,
        identity_verification_method=METHOD,historical_adapter_is_identity_only=True,
        source_identity_checks=copy.deepcopy(IDENTITY_CHECKS),actual_predecessor_validation=copy.deepcopy(CHECKPOINT),
        configuration=copy.deepcopy(CONFIG),change_declaration=copy.deepcopy(CHANGE),
        current_runtime10=copy.deepcopy(data['actual_runtime10']),original_SCAN_live_verifier_unchanged=True,
        original_three_override_set_unchanged=True,Teacher_heading_P_math_changed=True,original058_is_base_not_complete_new_Student_math=True,
        runtime_acceptance_inherited=False,current_source_authorization_inherited=False,navigation_acceptance_established=False)
