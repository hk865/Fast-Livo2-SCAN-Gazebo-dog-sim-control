"""R41 current source identity delta over the real validated R34 checkpoint.

Identity verification deliberately changes from recursive historical reader
replay to the pinned actual R34 deployment/checkpoint plus complete current
source hashes. No physical result or current producer authorization is inherited.
Original SCAN delivery/runtime and current source/ACK checks remain their owners.
"""
from pathlib import Path
import copy,hashlib,json,re
ROOT=Path('/opt/experiment/workspace/multifloor_demo/teacher_mode/navigation/engineering_v66_r85_turn_handoff_candidate')
WORK=Path('/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400')
STAGE=WORK/'stance_support_capture_candidate'
REPO=WORK.parent/'candidate_source_history'
BASELINE='16086ee080bc6284e347fc2de32a63b0a3217767'
PROFILE='profiles/r85_f1_f3_stance_support_capture_r41_20261016.json'
OLD_PROFILE='profiles/r85_f1_f3_neutral_rate_brake_capture_r34_20261016.json'
HELPER=ROOT/'stance_support_capture_adapter.py'
PROOF=ROOT/'R85_STANCE_SUPPORT_CAPTURE_CONTROL_ADAPTER_CONTRACT.json'
BASE=ROOT/'R85_SCAN_REFERENCE_PROTOCOL_WORKSPACE_CONTRACT.json'
BASE_SHA='6a028cb1dc37dfab63840172b9e0ba35fec3e0b3d09aa0926e91747f229e5a67'
HISTORICAL=ROOT/'R85_HEADING_BRAKING_CONTROL_ADAPTER_CONTRACT.json'
HISTORICAL_SHA='3fb1795ada9a6296c0cb84e352f6c27a5996b7458f74ffc47274fc082dcc5058'
CHECKPOINT={'path': '/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/round_34_neutral_rate_brake/install_tools/ACTUAL_POSTINSTALL_PROFILE_VALIDATION.json', 'sha256': 'f8a942b70886774de5efcb4ae594cb53b24d14f45a4c13a46b4a922f8a36479a'}
INSTALL={'path': '/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/round_34_neutral_rate_brake/install_tools/INSTALL_RECEIPT.json', 'sha256': '5d513279da1814d9b2fc33cf89c4e71828591f8cc9a7dedab1ef7dce800d82f0'}
BINDINGS_SHA='7847158bf6297ebd39a1b4b413245774c78f195d371e1c9b1b9886574884c0f8'
OLD_PROFILE_SHA='eee6b3677d06e5dd9994e33a9bc1a3b130b7def4679b231890e9ec261f48740a'
BEFORE={'transition_gate.py': '94a611780841dfe302d2f266286d11619084286af3aaa14ad97c1c3242b01d14', 'scan_workspace.py': '3e2c703539ea9f932becbd1ae83f02a22ce8bb7f279a5d84813921607de1c65c'}
AFTER={'transition_gate.py': '7a9b9f65c7297a90efdaa76ed70563672074f7a3c7f09e2c7f202f886154c7bb', 'scan_workspace.py': '4cfb6c4855e998a2d5f8dc1db0fba906a9e484a9a6f11065eacadbb8fd6f3bdb'}
GATE_DELTA=[{'old_start': 15, 'old_end': 15, 'new_start': 15, 'new_end': 38, 'old_lines': [], 'new_lines': ['def stance_support_handoff_configuration(profile):\n', '    """Explicit experimental internal target; original business stop owner remains."""\n', "    if 'r85_stance_support_handoff' not in profile:return None\n", "    value=profile['r85_stance_support_handoff']\n", "    if not isinstance(value,dict)or type(value.get('enabled'))is not bool:\n", "        raise ValueError('r85_stance_support_handoff requires an explicit boolean enabled')\n", "    if value['enabled']is False:\n", "        if set(value)!={'enabled'}:raise ValueError('Disabled stance support admits only enabled false')\n", '        return None\n', "    if set(value)!={'enabled','kind'}or value['kind']!='locked_heading_PD_stance_support_v1':\n", "        raise ValueError('Unknown stance-support handoff configuration')\n", "    cascade=profile.get('cascade',{})\n", "    if (any(not isinstance(profile.get(key),dict)or set(profile[key])!={'enabled'}\n", "                or profile[key]['enabled']is not required for key,required in\n", "                (('r85_active_turn_braking',True),('r85_quiet_handoff_target',True),('r85_neutral_rate_brake_capture',False)))\n", "            or not isinstance(cascade,dict)or cascade.get('external_heading_gate')is not True\n", "            or cascade.get('r85_turn_brake_reference')!={'kind':'locked_heading_pd_v1'}):\n", "        raise ValueError('Stance support requires active quiet locked-heading PD and explicit neutral off')\n", '    from pre_zero_integral_unload import configuration\n', "    if configuration(profile)is None:raise ValueError('Stance support requires original coordinated integral unload')\n", '    return copy.deepcopy(value)\n', '\n', '\n']}, {'old_start': 24, 'old_end': 24, 'new_start': 47, 'new_end': 50, 'old_lines': [], 'new_lines': ['        self.stance_support_handoff_config=stance_support_handoff_configuration(profile)\n', '        self.stance_old_brake_history=deque(maxlen=64);self.stance_old_quiet_history=deque(maxlen=64)\n', '        self.stance_old_capture_last_decision=None;self.stance_old_capture_last_sample=None\n']}, {'old_start': 61, 'old_end': 61, 'new_start': 87, 'new_end': 91, 'old_lines': [], 'new_lines': ["        self.quiet_command_yaw_cap=self.quiet_limits['yaw']\n", '        if self.stance_support_handoff_config is not None:\n', "            self.quiet_limits['heading']=.25*self.config['turn_complete_rad']\n", '            self.quiet_command_yaw_cap=.035\n']}, {'old_start': 180, 'old_end': 180, 'new_start': 210, 'new_end': 212, 'old_lines': [], 'new_lines': ['        if self.stance_support_handoff_config is not None:\n', '            self.stance_old_quiet_history.clear();self.stance_old_brake_history.clear()\n']}, {'old_start': 184, 'old_end': 185, 'new_start': 216, 'new_end': 217, 'old_lines': ['    def _low_total_command(self,value):\n'], 'new_lines': ['    def _low_total_command(self,value,yaw_cap=None):\n']}, {'old_start': 188, 'old_end': 189, 'new_start': 220, 'new_end': 221, 'old_lines': ["            and abs(value[2])<=self.quiet_limits['yaw'])\n"], 'new_lines': ['            and abs(value[2])<=(self.quiet_command_yaw_cap if yaw_cap is None else yaw_cap))\n']}, {'old_start': 227, 'old_end': 227, 'new_start': 259, 'new_end': 263, 'old_lines': [], 'new_lines': ['            # A high support publication between poses must interrupt the old\n', '            # strict shadow even if a later low publication precedes the next pose.\n', "            if self.stance_support_handoff_config is not None and not self._low_total_command(record['command_body'],.01):\n", '                self.stance_old_quiet_history.clear()\n']}, {'old_start': 280, 'old_end': 280, 'new_start': 316, 'new_end': 332, 'old_lines': [], 'new_lines': ['            if self.stance_support_handoff_config is not None:\n', "                sample=copy.deepcopy(self.quiet_history[-1]);error=sample['heading_error_rad']\n", '                old_commands=all(self._low_total_command(command,.01)for command in\n', "                    (p['command_body'],ack['requested_command_body'],ack['applied_command_body']))\n", "                old_brake=abs(error)<=.25*self.config['turn_complete_rad']\n", '                old_quiet=abs(error)<=.005 and old_commands\n', '                if old_brake:self.stance_old_brake_history.append((stamp_s,wall_s))\n', '                else:self.stance_old_brake_history.clear()\n', '                if old_quiet:self.stance_old_quiet_history.append(sample)\n', '                else:self.stance_old_quiet_history.clear()\n', '                self.stance_old_capture_last_sample.update(source_and_ACK_qualified=True,\n', '                    ACK_watermark=list(watermark),old_brake_instantaneous=old_brake,\n', '                    old_quiet_instantaneous=old_quiet,old_command_caps_passed=old_commands,\n', "                    published_command_body=list(p['command_body']),\n", "                    ACK_requested_command_body=list(ack['requested_command_body']),\n", "                    ACK_applied_command_body=list(ack['applied_command_body']))\n"]}, {'old_start': 319, 'old_end': 319, 'new_start': 371, 'new_end': 385, 'old_lines': [], 'new_lines': ['            locked_heading_error_rad=error,new_brake_ready=bool(new_brake_ready),new_quiet_ready=bool(new_quiet_ready),\n', '            old_brake_ready=old_brake,old_quiet_ready=old_quiet,old_internal_capture_ready=old_brake and old_quiet,\n', '            diagnostic_only=True,ACK_watermark_consumed_again=False,affects_new_decision=False)\n', '    def _record_stance_old_capture_decision(self,sim_time,wall_time,error,new_brake_ready,new_quiet_ready):\n', '        if self.stance_support_handoff_config is None:return\n', '        b=self.stance_old_brake_history;q=self.stance_old_quiet_history;c=self.config\n', "        latest=self.latest_motion['stamp_s']if self.latest_motion is not None else None\n", "        old_brake=bool(new_brake_ready and abs(error)<=.25*c['turn_complete_rad']\n", "            and len(b)>=c['stop_min_pose_samples']and b[-1][0]==latest\n", "            and b[-1][0]-b[0][0]>=c['stop_window_sim_s']-1e-9)\n", '        old_quiet=bool(new_quiet_ready and abs(error)<=.005\n', "            and len(q)>=c['stop_min_pose_samples']and q[-1]['stamp_s']==latest\n", "            and q[-1]['stamp_s']-q[0]['stamp_s']>=c['stop_window_sim_s']-1e-9)\n", '        self.stance_old_capture_last_decision=dict(control_s=sim_time,wall_s=wall_time,\n']}, {'old_start': 434, 'old_end': 434, 'new_start': 500, 'new_end': 503, 'old_lines': [], 'new_lines': ['        if self.stance_support_handoff_config is not None:\n', '            self.stance_old_brake_history.clear();self.stance_old_quiet_history.clear()\n', '            self.stance_old_capture_last_decision=None;self.stance_old_capture_last_sample=None\n']}, {'old_start': 479, 'old_end': 479, 'new_start': 548, 'new_end': 554, 'old_lines': [], 'new_lines': ['                if self.stance_support_handoff_config is not None:\n', '                    error=None if observed_yaw is None else self.error(self.heading,observed_yaw)\n', '                    self.stance_old_capture_last_sample=dict(pose_stamp_s=stamp_s,wall_s=wall_s,\n', '                        locked_heading_error_rad=error,new_brake_motion_valid=bool(brake_valid),\n', '                        source_and_ACK_qualified=False,old_brake_instantaneous=False,old_quiet_instantaneous=False,\n', '                        old_command_caps_passed=False)\n']}, {'old_start': 593, 'old_end': 594, 'new_start': 668, 'new_end': 670, 'old_lines': ['                    self._record_neutral_old_capture_decision(sim_time,wall_time,error,False,False);return False,0.\n'], 'new_lines': ['                    self._record_neutral_old_capture_decision(sim_time,wall_time,error,False,False)\n', '                    self._record_stance_old_capture_decision(sim_time,wall_time,error,False,False);return False,0.\n']}, {'old_start': 596, 'old_end': 596, 'new_start': 672, 'new_end': 673, 'old_lines': [], 'new_lines': ['                self._record_stance_old_capture_decision(sim_time,wall_time,error,brake_ready,quiet_ready)\n']}, {'old_start': 679, 'old_end': 679, 'new_start': 756, 'new_end': 783, 'old_lines': [], 'new_lines': ['        if self.stance_support_handoff_config is not None:\n', '            b=self.stance_old_brake_history;q=self.stance_old_quiet_history\n', "            result['stance_support_handoff']=dict(configuration=copy.deepcopy(self.stance_support_handoff_config),\n", '                new_internal_capture_changed=True,original_business_acceptance_unchanged=True,\n', "                reference_selection='restored locked_heading_pd_v1; neutral rate explicitly off',\n", '                reference_restored_from_R34_neutral=True,core_controller_Student_sources_unchanged=True,existing_PD_PI_formulas_unchanged=True,\n', '                locked_heading_rad=self.heading,goal_or_locked_target_moved=False,formal_T0_moved=False,\n', '                original_R16_internal_capture_claimed=False,\n', "                internal_limits=dict(brake_entry_heading_rad=.25*self.config['turn_complete_rad'],\n", '                    post_entry_brake_heading_rad=self._post_entry_brake_heading_band(),\n', "                    quiet_heading_rad=self.quiet_limits['heading'],command_yaw_cap_radps=self.quiet_command_yaw_cap,\n", "                    motion_SLAM_and_body_yaw_cap_radps=self.quiet_limits['yaw'],command_XY_norm_cap_mps=self.quiet_limits['xy']),\n", "                old_internal_limits=dict(brake_heading_rad=.25*self.config['turn_complete_rad'],quiet_heading_rad=.005,\n", '                    command_yaw_cap_radps=.01,motion_SLAM_and_body_yaw_cap_radps=.01),\n', '                old_strict_shadow=dict(diagnostic_only=True,brake_samples=len(b),quiet_samples=len(q),\n', '                    current_publication_old_command_quiet=bool(self.quiet_publication is not None\n', "                        and self._low_total_command(self.quiet_publication['command_body'],.01)),\n", '                    brake_first_stamp_s=b[0][0]if b else None,brake_last_stamp_s=b[-1][0]if b else None,\n', "                    quiet_first_stamp_s=q[0]['stamp_s']if q else None,quiet_last_stamp_s=q[-1]['stamp_s']if q else None,\n", '                    last_control_evaluation=copy.deepcopy(self.stance_old_capture_last_decision),\n', '                    last_pose_observation=copy.deepcopy(self.stance_old_capture_last_sample),\n', "                    history_scope='Subset of the same primary-qualified source/ACK samples, not an independent full old brake history.',\n", '                    old_numeric_conditions_on_candidate_trajectory=True,old_control_counterfactual_reproduced=False,\n', '                    same_validated_source_and_ACK_samples=True,independent_ACK_consumption=False,new_decision_affected=False),\n', '                physical_stance_support_proven=False,post_turn_fixed5_success_claimed=False,\n', '                absolute_heading_and_relative_T0_drift_are_distinct=True,\n', '                natural_wire_zero_diagnosed_from_original_stream=True,terminal_fixed5_owner_unchanged=True)\n']}]
SCANNER_PATCHES=[('def _control_adapter(session,base):\n', "def _control_adapter(session,base):\n    if 'stance_support_capture_proof' in session or 'stance_support_capture_proof_sha256' in session:\n        from stance_support_capture_adapter import load_stance_support_capture\n        return load_stance_support_capture(session,base)\n"), ("    if 'neutral_rate_brake_capture' in adapter:\n        from neutral_rate_brake_adapter import resolved_neutral_rate_brake_capture\n        resolved_neutral_rate_brake_capture(resolved,adapter)", "    if 'stance_support_capture' in adapter:\n        from stance_support_capture_adapter import resolved_stance_support_capture\n        resolved_stance_support_capture(resolved,adapter)\n    elif 'neutral_rate_brake_capture' in adapter:\n        from neutral_rate_brake_adapter import resolved_neutral_rate_brake_capture\n        resolved_neutral_rate_brake_capture(resolved,adapter)")]
CONFIG=dict(enabled=True,kind='locked_heading_PD_stance_support_v1')
METHOD='verified_R34_source_checkpoint_plus_exact_current_delta_v1'
IDENTITY_CHECKS=dict(runtime_all_active_SHA=True,runtime_current_binding_map_exact=True,
    tracked_files_are_evidence_copies_not_runtime_inputs=True,installer_active_tracked_copy_verification_required=True)
TARGETS=['transition_gate.py','scan_workspace.py','stance_support_capture_adapter.py',PROOF.name,PROFILE]
CHANGE=dict(control_reference_restored_from_neutral=True,locked_heading_PD_retained_relative_to_R16=True,
    internal_thresholds_changed=True,old_quiet_heading_rad=.005,new_quiet_heading_rad=.025,
    old_command_yaw_cap_radps=.01,new_command_yaw_cap_radps=.035,motion_yaw_cap_radps=.01,
    entry_and_postentry_brake_heading_rad=.025,XY_command_norm_cap_mps=.005,
    old_strict_shadow_includes_command_and_heading=True,shadow_second_ACK_consumption=False,
    original_quiet_window_s=.3,owned_prezero_timeout_s=10.,formal_zero_ACK_window_s=5.,
    locked_goal_target_moved=False,formal_T0_moved=False,original_business_acceptance_unchanged=True,
    performance_or_path_safety_proven=False)

def require(ok,reason):
    if not ok:raise RuntimeError('R41 source identity: '+reason)
def digest(raw):return hashlib.sha256(raw).hexdigest()
def read_ref(row,path=None,sha=None):
    require(isinstance(row,dict)and set(row)=={'path','sha256'},'exact file reference')
    p=Path(row['path']);h=row['sha256']
    require(p.is_absolute()and not p.is_symlink()and p.is_file()and p.resolve()==p
        and isinstance(h,str)and re.fullmatch('[0-9a-f]{64}',h)is not None,'regular pinned file')
    require((path is None or p==path)and(sha is None or h==sha),'declared exact path/hash')
    raw=p.read_bytes();require(digest(raw)==h,'actual bytes: '+str(p));return p,raw

def binding_map(raw,count):
    data=json.loads(raw);rows=data.get('entries');require(isinstance(rows,list)and len(rows)==count,'complete source count')
    result={}
    for row in rows:
        require(isinstance(row,dict)and set(row)=={'active','tracked','sha256','bytes'},'exact source binding row')
        p=Path(row['active']);t=Path(row['tracked'])
        require(p.is_absolute()and p.is_relative_to(ROOT)and '..'not in p.parts
            and not t.is_absolute()and '..'not in t.parts and type(row['bytes'])is int and row['bytes']>0
            and isinstance(row['sha256'],str)and re.fullmatch('[0-9a-f]{64}',row['sha256'])is not None,'source binding scope/types')
        require(str(p)not in result,'no repeated source');result[str(p)]=row
    return result

def after_bytes(name,raw):
    require(name in BEFORE and digest(raw)==BEFORE[name],'exact predecessor source: '+name)
    if name=='transition_gate.py':
        lines=raw.decode().splitlines(keepends=True)
        for row in reversed(GATE_DELTA):
            i,j=row['old_start'],row['old_end'];require(lines[i:j]==row['old_lines'],'gate exact delta context');lines[i:j]=row['new_lines']
        out=''.join(lines).encode()
    else:
        text=raw.decode()
        for a,b in SCANNER_PATCHES:require(text.count(a)==1,'scanner exact dispatch context');text=text.replace(a,b)
        out=text.encode()
    require(digest(out)==AFTER[name],'actual after source: '+name);return out

def validate_metadata(data):
    keys={'schema','baseline_commit','identity_verification_method','expected_bound_source_count_before',
        'expected_bound_source_count_after','exact_targets','baseline_source_bindings','predecessor_installation',
        'actual_predecessor_validation','source_identity_checks','historical_adapter','base_contract','baseline_profile','sources',
        'bridge_helper','profile','configuration','change_declaration','runtime_acceptance_inherited',
        'current_source_authorization_inherited','navigation_acceptance_established','physical_thresholds_changed'}
    require(isinstance(data,dict)and set(data)==keys,'exact proof keys')
    require(data['schema']=='r85_stance_support_current_source_delta/v1'and data['baseline_commit']==BASELINE
        and data['identity_verification_method']==METHOD and data['exact_targets']==TARGETS
        and data['expected_bound_source_count_before']==265 and data['expected_bound_source_count_after']==268,'proof identity/scope')
    require(all(data[k]is False for k in ['runtime_acceptance_inherited','current_source_authorization_inherited',
        'navigation_acceptance_established','physical_thresholds_changed']),'source identity only, no borrowed runtime/authorization/PASS')
    require(data['configuration']==CONFIG and type(data['configuration']['enabled'])is bool
        and data['change_declaration']==CHANGE
        and all(type(data['change_declaration'][k])is type(v)for k,v in CHANGE.items()),'explicit new internal conditions/reference')
    require(data['source_identity_checks']==IDENTITY_CHECKS
        and all(data['source_identity_checks'][k]is True for k in IDENTITY_CHECKS),'runtime active identity; tracked evidence copy belongs to installer')
    require(data['predecessor_installation']==INSTALL and data['actual_predecessor_validation']==CHECKPOINT,
        'actual pinned predecessor evidence')
    require(data['base_contract']==dict(path=str(BASE),sha256=BASE_SHA)
        and data['historical_adapter']==dict(path=str(HISTORICAL),sha256=HISTORICAL_SHA),'unchanged original contract identities')
    require(set(data['sources'])==set(BEFORE),'only two changed existing sources')
    for name,row in data['sources'].items():
        require(set(row)=={'path','before_sha256','after_sha256','beforeimage'}and row['path']==str(ROOT/name)
            and row['before_sha256']==BEFORE[name]and row['after_sha256']==AFTER[name],'true current delta: '+name)
    require(data['profile']==str(ROOT/PROFILE),'new unique selected profile')

def checkpoint(data,base):
    _,raw=read_ref(data['baseline_source_bindings'],STAGE/'beforeimage/SOURCE_BINDINGS.json',BINDINGS_SHA)
    before=binding_map(raw,265)
    _,raw=read_ref(INSTALL);installation=json.loads(raw)
    _,raw=read_ref(CHECKPOINT);checked=json.loads(raw)
    require(installation.get('installed')is True and installation.get('candidate_commit')==BASELINE
        and installation['actual_bindings']['bound_source_count']==265
        and installation['actual_bindings']['all_active_tracked_SHA_equal']is True
        and installation['actual_bindings']['full_source_bindings']['sha256']==BINDINGS_SHA,'real265 source deployment')
    require(checked.get('schema')=='r85_R34_actual_postinstall_profile_validation/v1'
        and checked.get('candidate_commit')==BASELINE and checked.get('passed')is True
        and checked.get('actual_validate_profile_pass')is True and checked.get('actual_SCAN_resolver_pass')is True
        and checked.get('no_source_aliases_or_monkeypatches')is True and checked.get('bound_source_count')==265
        and checked['install_receipt']==INSTALL,'real original preflight/SCAN positive, source identity only')
    require(checked['profile']==dict(path=str(ROOT/OLD_PROFILE),sha256=OLD_PROFILE_SHA)
        and checked['adapter']==dict(path=str(HISTORICAL),sha256=HISTORICAL_SHA),'checkpoint actual selected identities')
    runtime=checked.get('resolved_required_runtime_sources')
    require(isinstance(runtime,dict)and len(runtime)==10 and set(runtime)==set(base['required_runtime_sources'])
        and all(before[p]['sha256']==h for p,h in runtime.items()),'complete actual checkpoint runtime10')
    return before,runtime

def load_stance_support_capture(session,base):
    require(Path(__file__).resolve()==HELPER,'actual helper location')
    proof_ref=dict(path=session.get('stance_support_capture_proof'),sha256=session.get('stance_support_capture_proof_sha256'))
    _,raw=read_ref(proof_ref,PROOF);data=json.loads(raw);validate_metadata(data)
    require(session.get('scan_workspace_contract')==str(BASE)and session.get('scan_workspace_contract_sha256')==BASE_SHA
        and session.get('scan_control_adapter_contract')==str(HISTORICAL)
        and session.get('scan_control_adapter_contract_sha256')==HISTORICAL_SHA,'unchanged base/historical session declaration')
    _,base_raw=read_ref(data['base_contract'],BASE,BASE_SHA)
    require(json.loads(base_raw)==base,'actual original SCAN data; its full live verifier still runs')
    before,runtime=checkpoint(data,base)
    old_path,old_raw=read_ref(data['baseline_profile'],ROOT/OLD_PROFILE,OLD_PROFILE_SHA)
    helper,_=read_ref(data['bridge_helper'],HELPER)
    evidence=[PROOF,BASE,HISTORICAL,old_path,helper,Path(INSTALL['path']),Path(CHECKPOINT['path']),STAGE/'beforeimage/SOURCE_BINDINGS.json']
    for name,row in data['sources'].items():
        p,raw=read_ref(row['beforeimage'],STAGE/'beforeimage'/name,BEFORE[name])
        require((ROOT/name).read_bytes()==after_bytes(name,raw),'actual existing delta bytes: '+name);evidence.extend([p,ROOT/name])
    expected=json.loads(old_raw);expected['r85_neutral_rate_brake_capture']={'enabled':False}
    expected['r85_stance_support_handoff']=copy.deepcopy(CONFIG)
    expected['r85_session']['stance_support_capture_proof']=str(PROOF)
    expected['r85_session']['stance_support_capture_proof_sha256']=proof_ref['sha256']
    profile_path=ROOT/PROFILE;profile_raw=profile_path.read_bytes();profile=json.loads(profile_raw)
    require(profile==expected and session==profile['r85_session'],'exact profile neutraloff+stanceflag+two proof keys only')
    expected_hashes={p:r['sha256']for p,r in before.items()}
    for name in BEFORE:require(expected_hashes[str(ROOT/name)]==BEFORE[name],'baseline source delta binding');expected_hashes[str(ROOT/name)]=AFTER[name]
    added={str(HELPER):data['bridge_helper']['sha256'],str(PROOF):proof_ref['sha256'],str(profile_path):digest(profile_raw)}
    require(not(set(added)&set(expected_hashes)),'three genuinely new sources');expected_hashes.update(added)
    current_path=REPO/'SOURCE_BINDINGS.json';current=binding_map(current_path.read_bytes(),268)
    require(set(current)==set(expected_hashes),'exact265 to268 current source closure')
    for p,h in expected_hashes.items():
        row=current[p];active,raw=read_ref(dict(path=p,sha256=h))
        # Tracked files are verified evidence copies owned by the exact installer;
        # current runtime identity is the actual active bytes, not those copies.
        require(row['sha256']==h and len(raw)==row['bytes'],'complete actual current bytes: '+p)
    # Every current file whose bytes were actually checked above is returned
    # for the original archiver, including current contracts. Do not recurse
    # into references inside retained historical JSONs or old root reviews.
    actual=dict(expected_hashes)
    overrides={str(ROOT/name):dict(before_sha256=base['bindings_sha256'][str(ROOT/name)],after_sha256=actual[str(ROOT/name)])
        for name in ['controller.py','shared_controller.py','scan_workspace.py']}
    _,v3_raw=read_ref(data['historical_adapter'],HISTORICAL,HISTORICAL_SHA)
    evidence.extend([profile_path,current_path]);data['proof']=proof_ref
    data['actual_runtime10']={p:actual[p]for p in runtime};data['evidence']=sorted(set(evidence))
    return dict(path=HISTORICAL,sha256=HISTORICAL_SHA,data=json.loads(v3_raw),actual=actual,
        overrides=overrides,evidence=data['evidence'],stance_support_capture=data)

def resolved_stance_support_capture(resolved,adapter):
    data=adapter['stance_support_capture'];target=resolved['resolved_adapter']
    require(set(resolved['required_runtime_sources'])==set(data['actual_runtime10'])
        and resolved['required_runtime_sources']==data['actual_runtime10'],'truthful current runtime10')
    target['schema']='r85_stance_support_current_checkpoint_resolved/v1'
    target['stance_support_capture']=dict(proof=copy.deepcopy(data['proof']),baseline_commit=BASELINE,
        identity_verification_method=METHOD,historical_adapter_is_identity_only=True,
        source_identity_checks=copy.deepcopy(IDENTITY_CHECKS),
        actual_predecessor_validation=copy.deepcopy(CHECKPOINT),configuration=copy.deepcopy(CONFIG),
        change_declaration=copy.deepcopy(CHANGE),current_runtime10=copy.deepcopy(data['actual_runtime10']),
        original_SCAN_live_verifier_unchanged=True,original_three_override_set_unchanged=True,
        historical_recursive_identity_replay_replaced_by_verified_checkpoint=True,
        runtime_acceptance_inherited=False,current_source_authorization_inherited=False,navigation_acceptance_established=False)
