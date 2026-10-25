"""R42 explicit Student yaw P/D extension over the real R41 source checkpoint.

The checkpoint supplies historical source identity only. Current active bytes,
the exact current binding map, original SCAN live checks, source authorization,
causal ACK checks and physical acceptance retain their separate owners.
"""
from pathlib import Path
import copy,hashlib,json,re

ROOT=Path('/opt/experiment/workspace/multifloor_demo/teacher_mode/navigation/engineering_v66_r85_turn_handoff_candidate')
WORK=Path('/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400')
STAGE=WORK/'round_42_yaw_pd_fade_native_validation'
REPO=WORK.parent/'candidate_source_history'
BASELINE='2f17e25e1bb55e48d0548d551e5a2685e18bd5dc'
PROFILE='profiles/r85_f1_f3_yaw_pd_fade_native_r42_20261016.json'
OLD_PROFILE='profiles/r85_f1_f3_stance_support_capture_r41_20261016.json'
HELPER=ROOT/'yaw_pd_fade_native_adapter.py'
PROOF=ROOT/'R85_YAW_PD_FADE_NATIVE_CONTROL_ADAPTER_CONTRACT.json'
BASE=ROOT/'R85_SCAN_REFERENCE_PROTOCOL_WORKSPACE_CONTRACT.json'
BASE_SHA='6a028cb1dc37dfab63840172b9e0ba35fec3e0b3d09aa0926e91747f229e5a67'
HISTORICAL=ROOT/'R85_HEADING_BRAKING_CONTROL_ADAPTER_CONTRACT.json'
HISTORICAL_SHA='3fb1795ada9a6296c0cb84e352f6c27a5996b7458f74ffc47274fc082dcc5058'
CHECKPOINT={'path': '/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/round_41_stance_support_capture/install_tools/ACTUAL_POSTINSTALL_PROFILE_VALIDATION.json', 'sha256': '9d46d5b3899e5ff50af2b1fc8210dc43eba23bdf51501808ede838aed3331644'}
INSTALL={'path': '/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/round_41_stance_support_capture/install_tools/INSTALL_RECEIPT.json', 'sha256': '0f9bfefe289e7d55f1ca61baa01ebb10e45bfb1a6a22581623ff0aae534a3f16'}
SELECTION={'path': '/opt/experiment/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/ROOT_YAW_PD_FADE_NATIVE_CANDIDATE_SELECTION.json', 'sha256': 'bfc0c09d4d4923628180af0b5d2be674ca85d71786ba7d1f8b36132ce13c3739'}
BINDINGS_SHA='cbd3e5b61e75385ada627ba013f8fabdebc6792fad368fca94c1478de98ab64c'
OLD_PROFILE_SHA='6163b31af953ead830d4699fb757ec10281c655de22ba25bc9186e2f3dd30829'
BEFORE={'student_actor_adapter.py': '6f14828e53831c997ae3cd845b243b75dbcecd91b5b24caaaa2be37357923275', 'scan_workspace.py': '4cfb6c4855e998a2d5f8dc1db0fba906a9e484a9a6f11065eacadbb8fd6f3bdb', 'r85_preflight.py': 'bea31da0491eab2ae7e3a060e16b4cc3729a758382f2d8987f73594ba31fade8', 'r85_session_guard.py': '412c2362b8b2b987236196701db1377e1d7054d1a74567126f3f0ec4dbe84e91', 'r85_storage_guard.py': '6ec024319e24fac138e7483cc19cb3a166f79fc1856acf88b043a3f0ebfb4d7e', 'pid_scope.py': 'ef87a923caced8c58e7425ddd0293ae6abf9295041c1970f56bdfef3dc3d6d0a', 'guardian_archive_shutdown.py': 'e16a84682069a4277ab3a8f91f92e2535d3b6b5430c9c1ef182446bd50d8e054', 'student_lower_loop_ack.py': '92756657427bd447742411e644a3a68540875183fa51af7d595e3c30fcef119d'}
AFTER={'student_actor_adapter.py': '1a5baa2f9bf1eca3dee71427dfafc64f5e1145292900c5e620ff9faccc43201a', 'scan_workspace.py': '33271619d58ec5d44d41678bc3f154cb53640c22bccb6f20395c02c48d84c29e', 'r85_preflight.py': '2dfa70601870c6586f33050ffe9c00634e6b838e6c0da3ec5b78d0eb573d2029', 'r85_session_guard.py': '2b9954022713bae94512106da7e5bde80bd9eea6c06834a3380a6fbe908e1ac0', 'r85_storage_guard.py': 'e9448e11c3a2d28080214cbd2286795c3f1d222f58e27ae58751a150e8c81fbd', 'pid_scope.py': '1b667160255d4f1df0599fe50707c72022d649640c54a52876eaca912dea6d69', 'guardian_archive_shutdown.py': '312c63d813a5af9d7507fac280fa76dfe08fe5f795e7b17eed023c1ca28af8be', 'student_lower_loop_ack.py': '17a3718866e9d62dfc083cfe0abe7d8349413fb1500af4190fc074aaf50b65be'}
DELTAS={'student_actor_adapter.py': [{'old_start': 59, 'old_end': 59, 'new_start': 59, 'new_end': 60, 'old_lines': [], 'new_lines': ['        self.yaw_pd_fade=None\n']}, {'old_start': 60, 'old_end': 61, 'new_start': 61, 'new_end': 63, 'old_lines': ['    def update(self, stamp, requested, measured, motion_epoch, authoritative_zero, pre_zero_unload=None):\n'], 'new_lines': ['    def update(self, stamp, requested, measured, motion_epoch, authoritative_zero, pre_zero_unload=None,\n', '               pre_zero_pd_fade_frame=None):\n']}, {'old_start': 88, 'old_end': 88, 'new_start': 90, 'new_end': 103, 'old_lines': [], 'new_lines': ['        fade_diagnostic=None\n', '        if self.yaw_pd_fade is not None:\n', '            frame=pre_zero_pd_fade_frame\n', "            if (not isinstance(frame,dict)or set(frame)!={'input_clock_ns','input_frame_serial'}\n", "                    or type(frame['input_clock_ns'])is not int\n", "                    or frame['input_clock_ns']!=round((float(stamp)+.005)*1e9)):\n", "                raise ValueError('Fade requires the actual native input clock/frame')\n", "            fade_diagnostic=self.yaw_pd_fade.step(dt,frame['input_clock_ns'],\n", "                frame['input_frame_serial'],motion_epoch,authoritative_zero,pre_zero_unload)\n", '            kp,kd=(np.asarray(v,dtype=float)for v in self.yaw_pd_fade.gain_vectors(kp,kd))\n', '            fade_diagnostic.update(base_yaw_Kp=.5,base_yaw_Kd=.015,\n', '                effective_yaw_Kp=float(kp[2]),effective_yaw_Kd=float(kd[2]),\n', '                Ki_unchanged=float(ki[2]),base_PID_math_claim=False)\n']}, {'old_start': 129, 'old_end': 129, 'new_start': 144, 'new_end': 145, 'old_lines': [], 'new_lines': ["        if fade_diagnostic is not None:diagnostic['student_yaw_pd_fade']=fade_diagnostic\n"]}, {'old_start': 183, 'old_end': 183, 'new_start': 199, 'new_end': 230, 'old_lines': [], 'new_lines': ['        self.configure_yaw_pd_fade_candidate()\n', '\n', '    def configure_yaw_pd_fade_candidate(self):\n', '        """Source-only draft: require real snapshot binding before helper import."""\n', "        option=self.scope['profile'].get('r85_student_yaw_pd_fade')\n", "        if option is None or (isinstance(option,dict)and set(option)=={'enabled'}and option['enabled']is False):\n", '            return False\n', '        if self.frame_serial or self.pid.frame_count or self.pid.previous_stamp is not None:\n', "            raise ValueError('Fade configuration only before original first input')\n", '        if self.unload_helper is None or self.integral_unload is None:\n', "            raise ValueError('Original NativeUnload authority required')\n", '        import importlib.util\n', "        helper_path=Path(__file__).with_name('student_yaw_pd_fade.py')\n", "        expected_snapshot=Path(self.binding['student_adapter_snapshot']).with_name(helper_path.name)\n", "        helper_origin=Path(self.binding['student_adapter_origin']).with_name(helper_path.name)\n", "        scope_bytes=(self.run/'navigation_scope.json').read_bytes()\n", "        expected_sha=self.scope['references'].get(str(helper_origin))\n", '        if (json.loads(scope_bytes)!=self.scope or helper_path.resolve()!=expected_snapshot.resolve()\n', '                or not isinstance(expected_sha,str)or sha(helper_path)!=expected_sha):\n', "            raise ValueError('Actual loaded fade helper snapshot/source binding differs')\n", "        spec=importlib.util.spec_from_file_location('actual_snapshot_student_yaw_pd_fade',helper_path)\n", '        helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)\n', "        config=helper.configuration(self.scope['profile'],self.unload_helper)\n", '        self.pid.yaw_pd_fade=helper.StudentYawPDFade(config,self.unload_helper,\n', '            self.integral_unload.run_id,self.integral_unload.scope_sha)\n', "        self.yaw_pd_fade_identity=dict(schema='r85_student_yaw_PD_fade_deployment_extension/v1',\n", '            helper_path=str(helper_path),helper_sha256=sha(helper_path),\n', '            scope_sha256=hashlib.sha256(scope_bytes).hexdigest(),configuration=config,\n', '            base_student_contract_sha256=sha(self.contract_path),\n', '            base_contract_does_not_prove_new_PD_math=True)\n', '        return True\n']}, {'old_start': 209, 'old_end': 210, 'new_start': 256, 'new_end': 261, 'old_lines': ['        if unload is None:\n'], 'new_lines': ['        if self.pid.yaw_pd_fade is not None:\n', '            corrected,_=self.pid.update(float(self.native_state[0]-.005),request,\n', '                self.native_state[[8,9,13]],epoch,not bool(np.any(request)),pre_zero_unload=unload,\n', '                pre_zero_pd_fade_frame=dict(input_clock_ns=input_clock_ns,input_frame_serial=self.frame_serial))\n', '        elif unload is None:\n']}, {'old_start': 251, 'old_end': 251, 'new_start': 302, 'new_end': 305, 'old_lines': [], 'new_lines': ["        if 'student_yaw_pd_fade' in self.pid.last_diagnostic:\n", '            import copy\n', "            self.input_receipt['student_yaw_pd_fade']=copy.deepcopy(self.pid.last_diagnostic['student_yaw_pd_fade'])\n"]}, {'old_start': 327, 'old_end': 327, 'new_start': 381, 'new_end': 382, 'old_lines': [], 'new_lines': ["        if self.pid.yaw_pd_fade is not None:out['student_yaw_PD_fade_extension']=dict(self.yaw_pd_fade_identity)\n"]}], 'scan_workspace.py': [{'old_start': 778, 'old_end': 778, 'new_start': 778, 'new_end': 781, 'old_lines': [], 'new_lines': ["    if 'yaw_pd_fade_native_proof' in session or 'yaw_pd_fade_native_proof_sha256' in session:\n", '        from yaw_pd_fade_native_adapter import load_yaw_pd_fade_native\n', '        return load_yaw_pd_fade_native(session,base)\n']}, {'old_start': 931, 'old_end': 932, 'new_start': 934, 'new_end': 938, 'old_lines': ["    if 'stance_support_capture' in adapter:\n"], 'new_lines': ["    if 'yaw_pd_fade_native' in adapter:\n", '        from yaw_pd_fade_native_adapter import resolved_yaw_pd_fade_native\n', '        resolved_yaw_pd_fade_native(resolved,adapter)\n', "    elif 'stance_support_capture' in adapter:\n"]}], 'r85_preflight.py': [{'old_start': 23, 'old_end': 23, 'new_start': 23, 'new_end': 38, 'old_lines': [], 'new_lines': ['def validate_student_yaw_pd_fade(profile):\n', "    value=profile.get('r85_student_yaw_pd_fade')\n", '    if value is None:return None\n', "    if type(value)is not dict or type(value.get('enabled'))is not bool:raise ValueError('Strict yaw PD fade option required')\n", "    if value=={'enabled':False}:return None\n", "    expected={'enabled':True,'kind':'accepted_pre_zero_event_yaw_PD_fade_v1','maximum_alpha_rate_per_s':1.0}\n", "    if value!=expected or any(type(value.get(k))is not type(v)for k,v in expected.items()):raise ValueError('Unselected yaw PD fade law')\n", '    from pre_zero_integral_unload import configuration\n', "    if configuration(profile)is None:raise ValueError('Yaw PD fade requires original real prezero event authority')\n", "    if profile.get('r85_stance_support_handoff')!={'enabled':True,'kind':'locked_heading_PD_stance_support_v1'}:\n", "        raise ValueError('R42 preserves the selected R41 stance handoff')\n", '    from r85_resource_renewal import configuration as resource_configuration\n', "    if resource_configuration(profile)is None:raise ValueError('R42 requires explicit renewed resource authority')\n", '    return dict(value)\n', '\n']}, {'old_start': 25, 'old_end': 25, 'new_start': 40, 'new_end': 43, 'old_lines': [], 'new_lines': ['    from r85_resource_renewal import retained_hard_bytes\n', '    retained_cap=retained_hard_bytes(p)\n', '    validate_student_yaw_pd_fade(p)\n']}, {'old_start': 29, 'old_end': 30, 'new_start': 47, 'new_end': 48, 'old_lines': ['        simulation_only=True,maximum_retained_bytes=130000000000,maximum_round_bytes=10000000000)\n'], 'new_lines': ['        simulation_only=True,maximum_retained_bytes=retained_cap,maximum_round_bytes=10000000000)\n']}, {'old_start': 45, 'old_end': 46, 'new_start': 63, 'new_end': 64, 'old_lines': ["    if p['raw_storage_budget']!={'max_run_bytes':130000000000,'minimum_start_available_bytes':107374182400,'minimum_runtime_available_bytes':107374182400}:\n"], 'new_lines': ["    if p['raw_storage_budget']!={'max_run_bytes':retained_cap,'minimum_start_available_bytes':107374182400,'minimum_runtime_available_bytes':107374182400}:\n"]}], 'r85_session_guard.py': [{'old_start': 21, 'old_end': 21, 'new_start': 21, 'new_end': 24, 'old_lines': [], 'new_lines': ['    from r85_resource_renewal import configuration as resource_configuration\n', '    renewed=resource_configuration(profile)\n', "    if renewed is not None:return dt.datetime.fromisoformat(renewed['deadline_utc'])\n"]}, {'old_start': 121, 'old_end': 122, 'new_start': 124, 'new_end': 126, 'old_lines': ["        if u['current_retained_budget_bytes']+reserve>=130000000000:\n"], 'new_lines': ['        from r85_resource_renewal import retained_hard_bytes\n', "        if u['current_retained_budget_bytes']+reserve>=retained_hard_bytes(profile):\n"]}], 'r85_storage_guard.py': [{'old_start': 14, 'old_end': 14, 'new_start': 14, 'new_end': 18, 'old_lines': [], 'new_lines': ['    from r85_resource_renewal import configuration as resource_configuration\n', '    renewed=resource_configuration(profile)\n', "    hard=renewed['maximum_retained_bytes'] if renewed is not None else 130000000000\n", "    soft=renewed['soft_stop_bytes'] if renewed is not None else base.SOFT_BYTES\n"]}, {'old_start': 23, 'old_end': 24, 'new_start': 27, 'new_end': 28, 'old_lines': ['    expected=dict(max_run_bytes=130000000000,minimum_start_available_bytes=107374182400,minimum_runtime_available_bytes=107374182400)\n'], 'new_lines': ['    expected=dict(max_run_bytes=hard,minimum_start_available_bytes=107374182400,minimum_runtime_available_bytes=107374182400)\n']}, {'old_start': 44, 'old_end': 47, 'new_start': 48, 'new_end': 51, 'old_lines': ['        known_outputs=known,current_retained_budget_bytes=retained,remaining_retained_budget_bytes=130000000000-retained,\n', '        conservative_session_growth_bytes=retained,filesystems=fs,hard_bytes=130000000000,\n', '        soft_stop_bytes=base.SOFT_BYTES,predictive_growth_allowance_bytes_per_s=rate,\n'], 'new_lines': ['        known_outputs=known,current_retained_budget_bytes=retained,remaining_retained_budget_bytes=hard-retained,\n', '        conservative_session_growth_bytes=retained,filesystems=fs,hard_bytes=hard,\n', '        soft_stop_bytes=soft,predictive_growth_allowance_bytes_per_s=rate,\n']}, {'old_start': 49, 'old_end': 49, 'new_start': 53, 'new_end': 56, 'old_lines': [], 'new_lines': ['    if renewed is not None:\n', "        result['resource_renewal']=renewed\n", "        result['base_shared_policy_is_historical_capacity_only']=True\n"]}, {'old_start': 55, 'old_end': 56, 'new_start': 62, 'new_end': 63, 'old_lines': ["    if retained+rate*base.GRACE_S+base.ATOMIC_BYTES>=base.SOFT_BYTES:reasons.append('projected_retained_capacity_stop')\n"], 'new_lines': ["    if retained+rate*base.GRACE_S+base.ATOMIC_BYTES>=soft:reasons.append('projected_retained_capacity_stop')\n"]}], 'pid_scope.py': [{'old_start': 36, 'old_end': 36, 'new_start': 36, 'new_end': 37, 'old_lines': [], 'new_lines': ['    from r85_resource_renewal import retained_hard_bytes\n']}, {'old_start': 37, 'old_end': 38, 'new_start': 38, 'new_end': 39, 'old_lines': ['        simulation_only=True,maximum_retained_bytes=130000000000,maximum_round_bytes=10000000000)\n'], 'new_lines': ['        simulation_only=True,maximum_retained_bytes=retained_hard_bytes(profile),maximum_round_bytes=10000000000)\n']}], 'guardian_archive_shutdown.py': [{'old_start': 56, 'old_end': 56, 'new_start': 56, 'new_end': 57, 'old_lines': [], 'new_lines': ['    from r85_resource_renewal import retained_hard_bytes\n']}, {'old_start': 58, 'old_end': 59, 'new_start': 59, 'new_end': 60, 'old_lines': ['        maximum_retained_bytes=R85_SHARED_HARD_BYTES,maximum_round_bytes=RUN_CAP_BYTES)\n'], 'new_lines': ['        maximum_retained_bytes=retained_hard_bytes(profile,R85_SHARED_HARD_BYTES),maximum_round_bytes=RUN_CAP_BYTES)\n']}, {'old_start': 184, 'old_end': 186, 'new_start': 185, 'new_end': 188, 'old_lines': ['        shared_hard=R85_SHARED_HARD_BYTES if r85 else SHARED_HARD_BYTES\n', '        shared_soft=R85_SHARED_SOFT_BYTES if r85 else SHARED_SOFT_BYTES\n'], 'new_lines': ['        from r85_resource_renewal import retained_hard_bytes\n', "        shared_hard=retained_hard_bytes(plan['profile'],R85_SHARED_HARD_BYTES) if r85 else SHARED_HARD_BYTES\n", '        shared_soft=shared_hard-536_870_912 if r85 else SHARED_SOFT_BYTES\n']}, {'old_start': 249, 'old_end': 250, 'new_start': 251, 'new_end': 253, 'old_lines': ["                return finish('cancel_R85_original130GB_soft_gate_with_archive_reserve' if r85\n"], 'new_lines': ["                return finish(('cancel_R85_renewed160GB_soft_gate_with_archive_reserve' if shared_hard==160000000000\n", "                               else 'cancel_R85_original130GB_soft_gate_with_archive_reserve') if r85\n"]}], 'student_lower_loop_ack.py': [{'old_start': 91, 'old_end': 91, 'new_start': 91, 'new_end': 151, 'old_lines': [], 'new_lines': ['def validate_yaw_pd_fade(extension, scope, stamp, frame, unload, zero, pid=None):\n', '    """Bind the optional new-law diagnostic; alpha grants no parking authority."""\n', "    keys={'schema','enabled','new_control_math','alpha_before','alpha','target_alpha',\n", "        'maximum_alpha_rate_per_s','dt_s','active_current_owned_event','reason','event_id',\n", "        'event_expires_clock_ns','event_validated_at_source_clock','source_control_clock_ns',\n", "        'run_id','scope_sha256','motion_epoch','input_clock_ns','input_frame_serial',\n", "        'original_exact_zero_priority','PID_I_or_D_state_modified_by_helper','stop_or_ACK_credit',\n", "        'base_yaw_Kp','base_yaw_Kd','effective_yaw_Kp','effective_yaw_Kd','Ki_unchanged','base_PID_math_claim'}\n", '    q=extension\n', '    if not isinstance(q,dict) or set(q)!=keys:\n', "        raise ValueError('Typed same-frame yaw PD fade diagnostic required')\n", "    numeric=['alpha_before','alpha','target_alpha','maximum_alpha_rate_per_s','dt_s',\n", "        'base_yaw_Kp','base_yaw_Kd','effective_yaw_Kp','effective_yaw_Kd','Ki_unchanged']\n", '    if (any(type(q[k])not in(int,float)or not np.isfinite(q[k])for k in numeric)\n', "        or q['schema']!='student_yaw_PD_fade_candidate/v1' or q['enabled']is not True\n", "        or q['new_control_math']is not True or q['base_PID_math_claim']is not False\n", "        or q['PID_I_or_D_state_modified_by_helper']is not False or q['stop_or_ACK_credit']is not False\n", "        or type(q['original_exact_zero_priority'])is not bool or q['original_exact_zero_priority']!=zero\n", "        or q['scope_sha256']!=scope or type(q['input_clock_ns'])is not int or q['input_clock_ns']!=stamp\n", "        or type(q['input_frame_serial'])is not int or q['input_frame_serial']!=frame\n", "        or not isinstance(q['run_id'],str)or not q['run_id']\n", "        or not .019999<=q['dt_s']<=.020001 or q['maximum_alpha_rate_per_s']!=1.\n", "        or not 0.<=q['alpha_before']<=1. or not 0.<=q['alpha']<=1.\n", "        or type(q['active_current_owned_event'])is not bool\n", "        or type(q['event_validated_at_source_clock'])is not bool\n", "        or q['base_yaw_Kp']!=.5 or q['base_yaw_Kd']!=.015 or q['Ki_unchanged']!=.15\n", "        or q['effective_yaw_Kp']!=.5*q['alpha'] or q['effective_yaw_Kd']!=.015*q['alpha']):\n", "        raise ValueError('Actual yaw PD fade frame/scope/gains/no-credit differs')\n", "    active=q['active_current_owned_event'];target=0. if active else 1.\n", "    expected=max(target,q['alpha_before']-q['dt_s'])if q['alpha_before']>target else min(target,q['alpha_before']+q['dt_s'])\n", "    if q['target_alpha']!=target or q['alpha']!=expected:\n", "        raise ValueError('Yaw PD fade bounded recurrence differs')\n", "    if q['event_id'] is None:\n", "        if (active or q['event_expires_clock_ns']is not None or q['source_control_clock_ns']is not None\n", "            or q['event_validated_at_source_clock']is not False\n", "            or q['reason']!=('original_exact_zero_priority_restore'if zero else'no_owned_event_restore')):\n", "            raise ValueError('Yaw PD restore must not invent an event')\n", '    else:\n', '        from pre_zero_integral_unload import validate_native_extension\n', '        original=validate_native_extension(unload,scope,stamp,frame)\n', "        event=original.get('event')\n", "        if (zero or not isinstance(event,dict)or q['event_validated_at_source_clock']is not True\n", "            or q['event_id']!=event['event_id']or q['run_id']!=event['run_id']\n", "            or q['event_expires_clock_ns']!=event['expires_clock_ns']\n", "            or type(q['source_control_clock_ns'])is not int\n", "            or q['source_control_clock_ns']!=original['source_control_clock_ns']\n", "            or q['motion_epoch']!=event['context']['request_id']+':waypoint:'+str(event['context']['waypoint_index'])\n", "            or q['dt_s']!=original['dt_s']\n", "            or active!=(event['brake_entry_clock_ns']<=stamp<event['expires_clock_ns'])\n", "            or q['reason']!=('active_owned_event_fade'if active else'accepted_late_source_restore_only')):\n", "            raise ValueError('Yaw PD fade and original accepted owned event differ')\n", '    if pid is not None:\n', "        if q['dt_s']!=pid['dt_s']or q['motion_epoch']!=pid['motion_epoch']:\n", "            raise ValueError('Yaw PD fade and actual PID clock/epoch differ')\n", "        expected_P=0. if zero else q['effective_yaw_Kp']*vector(pid['pid_error'])[2]\n", "        if abs(vector(pid['pid_P'])[2]-expected_P)>1e-12:\n", "            raise ValueError('Yaw PD fade does not match actual PID P contribution')\n", '    return q\n', '\n', '\n']}, {'old_start': 141, 'old_end': 141, 'new_start': 201, 'new_end': 207, 'old_lines': [], 'new_lines': ["    if 'student_yaw_pd_fade'in pid or 'student_yaw_pd_fade'in receipt:\n", "        extension=pid.get('student_yaw_pd_fade')\n", "        validate_yaw_pd_fade(extension,scope,stamp,frame,receipt.get('pre_zero_integral_unload'),zero,pid)\n", "        if extension!=receipt.get('student_yaw_pd_fade'):\n", "            raise ValueError('Yaw PD fade PID and actual input receipt differ')\n", "        annotation['student_yaw_pd_fade']=copy.deepcopy(extension)\n"]}, {'old_start': 211, 'old_end': 211, 'new_start': 277, 'new_end': 282, 'old_lines': [], 'new_lines': ["    if 'student_yaw_pd_fade'in note or 'student_yaw_pd_fade'in receipt:\n", "        extension=note.get('student_yaw_pd_fade')\n", "        validate_yaw_pd_fade(extension,scope,stamp,frame,note.get('pre_zero_integral_unload'),zero)\n", "        if extension!=receipt.get('student_yaw_pd_fade'):\n", "            raise ValueError('Yaw PD fade ACK ledger and actual input receipt differ')\n"]}, {'old_start': 220, 'old_end': 220, 'new_start': 291, 'new_end': 292, 'old_lines': [], 'new_lines': ["    if 'student_yaw_pd_fade'in note:diagnostic['student_yaw_pd_fade']=copy.deepcopy(note['student_yaw_pd_fade'])\n"]}]}
NEW_SOURCE_SHA={'student_yaw_pd_fade.py': '84cec93abb32f58da42a6ede3f507c4339e198838d2ea10f1ec212c9c7428d7d', 'r85_resource_renewal.py': 'e154bc4b038eaba9ac8d30f0b3e190c750f89350061d9fd18ed76496279df4b0', 'R85_RESOURCE_RENEWAL_UNTIL20261017T0000.json': '98bfd59399845f53f27c94f36ef65add41b0eff3be9f65c8e5fc08a4f73d49a8'}
PROFILE_DELTA=[{'path': ['diagnostic_observe_only', 'user_authorized_work_deadline_utc'], 'old_present': True, 'old_value': '2026-10-16T14:00:00+00:00', 'new_value': '2026-10-17T00:00:00+00:00'}, {'path': ['diagnostic_observe_only', 'work_deadline_basis'], 'old_present': True, 'old_value': 'explicit_user_continue_until_20261016_2200_UTCplus08', 'new_value': 'explicit_user_continue_until_20261017_0800_UTCplus08'}, {'path': ['r85_resource_renewal'], 'old_present': False, 'new_value': {'enabled': True, 'kind': 'explicit_until20261017T0000_160GB_v1'}}, {'path': ['r85_session', 'deadline_utc'], 'old_present': True, 'old_value': '2026-10-16T14:00:00+00:00', 'new_value': '2026-10-17T00:00:00+00:00'}, {'path': ['r85_session', 'maximum_retained_bytes'], 'old_present': True, 'old_value': 130000000000, 'new_value': 160000000000}, {'path': ['r85_session', 'resource_renewal_proof'], 'old_present': False, 'new_value': '/opt/experiment/workspace/multifloor_demo/teacher_mode/navigation/engineering_v66_r85_turn_handoff_candidate/R85_RESOURCE_RENEWAL_UNTIL20261017T0000.json'}, {'path': ['r85_session', 'resource_renewal_proof_sha256'], 'old_present': False, 'new_value': '98bfd59399845f53f27c94f36ef65add41b0eff3be9f65c8e5fc08a4f73d49a8'}, {'path': ['r85_student_yaw_pd_fade'], 'old_present': False, 'new_value': {'enabled': True, 'kind': 'accepted_pre_zero_event_yaw_PD_fade_v1', 'maximum_alpha_rate_per_s': 1.0}}, {'path': ['raw_storage_budget', 'max_run_bytes'], 'old_present': True, 'old_value': 130000000000, 'new_value': 160000000000}]
CONFIG={'enabled':True,'kind':'accepted_pre_zero_event_yaw_PD_fade_v1','maximum_alpha_rate_per_s':1.0}
METHOD='verified_R41_source_checkpoint_plus_exact_current_delta_v1'
IDENTITY_CHECKS=dict(runtime_all_active_SHA=True,runtime_current_binding_map_exact=True,
    tracked_files_are_evidence_copies_not_runtime_inputs=True,installer_active_tracked_copy_verification_required=True)
TARGETS=list(BEFORE)+list(NEW_SOURCE_SHA)+[HELPER.name,PROOF.name,PROFILE]
CHANGE=dict(Student_yaw_PD_math_changed=True,accepted_owned_prezero_event_only=True,
    yaw_Kp_base=.5,yaw_Kd_base=.015,alpha_rate_abs_max_per_s=1.,
    I_state_and_original_unload_formulas_unchanged=True,exact_zero_priority_unchanged=True,
    original058_is_base_not_complete_new_math=True,new_math_identity_in_actual_input_and_canonical_ACK=True,
    Teacher_core_controller_gate_unchanged=True,source_authorization_and_freshness_unchanged=True,
    original_business_acceptance_unchanged=True,formal_T0_moved=False,
    deadline_from='2026-10-16T14:00:00+00:00',deadline_to='2026-10-17T00:00:00+00:00',
    retained_bytes_from=130000000000,retained_bytes_to=160000000000,round_bytes_unchanged=10000000000,
    performance_or_physical_safety_proven=False)

def require(ok,reason):
    if not ok:raise RuntimeError('R42 source identity: '+reason)

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
    require('yaw_pd_fade_native_proof'not in session and 'yaw_pd_fade_native_proof_sha256'not in session,'new proof keys only')
    session['yaw_pd_fade_native_proof']=str(PROOF)
    session['yaw_pd_fade_native_proof_sha256']=proof_sha
    return profile

def validate_metadata(data):
    keys={'schema','baseline_commit','identity_verification_method','expected_bound_source_count_before',
        'expected_bound_source_count_after','exact_targets','baseline_source_bindings','predecessor_installation',
        'actual_predecessor_validation','source_identity_checks','historical_adapter','base_contract','baseline_profile','sources',
        'bridge_helper','profile','configuration','profile_delta','new_sources','change_declaration','selection',
        'runtime_acceptance_inherited','current_source_authorization_inherited','navigation_acceptance_established','physical_thresholds_changed'}
    require(isinstance(data,dict)and set(data)==keys,'exact proof keys')
    require(data['schema']=='r85_yaw_PD_fade_native_current_source_delta/v1'and data['baseline_commit']==BASELINE
        and data['identity_verification_method']==METHOD and data['exact_targets']==TARGETS
        and type(data['expected_bound_source_count_before'])is int and data['expected_bound_source_count_before']==268
        and type(data['expected_bound_source_count_after'])is int and data['expected_bound_source_count_after']==274,'proof identity/scope')
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
    before=binding_map(raw,268)
    _,raw=read_ref(INSTALL);installation=json.loads(raw)
    _,raw=read_ref(CHECKPOINT);checked=json.loads(raw)
    require(installation.get('installed')is True and installation.get('candidate_commit')==BASELINE
        and installation['actual_bindings']['bound_source_count']==268
        and installation['actual_bindings']['all_active_tracked_SHA_equal']is True
        and installation['actual_bindings']['full_source_bindings']['sha256']==BINDINGS_SHA,'real268 source deployment')
    require(checked.get('schema')=='r85_R41_actual_postinstall_profile_validation/v1'
        and checked.get('candidate_commit')==BASELINE and checked.get('passed')is True
        and checked.get('actual_validate_profile_pass')is True and checked.get('actual_SCAN_resolver_pass')is True
        and checked.get('no_source_aliases_or_monkeypatches')is True and checked.get('bound_source_count')==268
        and checked['install_receipt']==INSTALL,'real preflight/SCAN checkpoint, identity only')
    require(checked['profile']==dict(path=str(ROOT/OLD_PROFILE),sha256=OLD_PROFILE_SHA)
        and checked['adapter']==dict(path=str(HISTORICAL),sha256=HISTORICAL_SHA),'checkpoint selected identities')
    runtime=checked.get('resolved_required_runtime_sources')
    require(isinstance(runtime,dict)and len(runtime)==10 and set(runtime)==set(base['required_runtime_sources'])
        and all(before[p]['sha256']==h for p,h in runtime.items()),'complete actual checkpoint runtime10')
    return before,runtime

def load_yaw_pd_fade_native(session,base):
    require(Path(__file__).resolve()==HELPER,'actual helper location')
    proof_ref=dict(path=session.get('yaw_pd_fade_native_proof'),sha256=session.get('yaw_pd_fade_native_proof_sha256'))
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
    require(not(set(added)&set(expected_hashes))and len(added)==6,'six genuinely new sources')
    expected_hashes.update(added)
    current_path=REPO/'SOURCE_BINDINGS.json';current=binding_map(current_path.read_bytes(),274)
    require(set(current)==set(expected_hashes),'exact268 to274 current source closure')
    for p,h in expected_hashes.items():
        row=current[p];active,raw=read_ref(dict(path=p,sha256=h))
        require(row['sha256']==h and len(raw)==row['bytes'],'complete actual current bytes: '+p)
    actual=dict(expected_hashes)
    overrides={str(ROOT/name):dict(before_sha256=base['bindings_sha256'][str(ROOT/name)],after_sha256=actual[str(ROOT/name)])
        for name in ['controller.py','shared_controller.py','scan_workspace.py']}
    _,v3_raw=read_ref(data['historical_adapter'],HISTORICAL,HISTORICAL_SHA)
    evidence.extend([profile_path,current_path]);data['proof']=proof_ref
    data['actual_runtime10']={p:actual[p]for p in runtime};data['evidence']=sorted(set(evidence))
    return dict(path=HISTORICAL,sha256=HISTORICAL_SHA,data=json.loads(v3_raw),actual=actual,
        overrides=overrides,evidence=data['evidence'],yaw_pd_fade_native=data)

def resolved_yaw_pd_fade_native(resolved,adapter):
    data=adapter['yaw_pd_fade_native'];target=resolved['resolved_adapter']
    require(resolved['required_runtime_sources']==data['actual_runtime10'],'truthful current runtime10')
    # The old v3 bytes still describe their historical preflight. The original
    # scanner copies that row before this selected dispatch; do not mislabel its
    # historical SHA as the current renewed preflight source.
    if 'extra_preflight'in target:
        target['historical_extra_preflight']=copy.deepcopy(target['extra_preflight'])
    current_preflight=copy.deepcopy(data['sources']['r85_preflight.py'])
    require(current_preflight['after_sha256']==AFTER['r85_preflight.py'],'truthful current preflight metadata')
    target['extra_preflight']=current_preflight
    target['schema']='r85_yaw_PD_fade_native_current_checkpoint_resolved/v1'
    target['yaw_pd_fade_native']=dict(proof=copy.deepcopy(data['proof']),baseline_commit=BASELINE,
        identity_verification_method=METHOD,historical_adapter_is_identity_only=True,
        source_identity_checks=copy.deepcopy(IDENTITY_CHECKS),actual_predecessor_validation=copy.deepcopy(CHECKPOINT),
        configuration=copy.deepcopy(CONFIG),change_declaration=copy.deepcopy(CHANGE),
        current_runtime10=copy.deepcopy(data['actual_runtime10']),original_SCAN_live_verifier_unchanged=True,
        original_three_override_set_unchanged=True,original058_is_base_not_complete_new_math=True,
        runtime_acceptance_inherited=False,current_source_authorization_inherited=False,navigation_acceptance_established=False)
