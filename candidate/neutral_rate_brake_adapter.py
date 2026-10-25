"""Reviewed quiet-target identity layer; original proofs remain historical bytes.

Current identities are verified with exact beforeimage-to-candidate deltas.
This source adapter establishes no physical or navigation acceptance.
"""
from pathlib import Path
import copy
import json
import re
import handoff_seed_recovery_adapter as historical_repair
import schedule_renewal_adapter as historical_schedule
from schedule_renewal_adapter import (ROOT,WORK,STAGE,PROOF,PROFILE,BASELINE_COMMIT,
    OLD_SCANNER_SHA,OLD_GUARD_SHA,V3_SHA,BASE_SHA,REFERENCE_SHA,SCHEDULE,GUARD_SUBSTITUTIONS,
    require,digest,file_ref,scanner_after_bytes,guard_after_bytes)

QUIET_STAGE=WORK/'round_15_pre_zero_quiet_source_integration'
QUIET_HELPER=ROOT/'pre_zero_quiet_adapter.py'
QUIET_PROOF=ROOT/'R85_PRE_ZERO_QUIET_CONTROL_ADAPTER_CONTRACT.json'
QUIET_PROFILE='profiles/r85_f1_f3_pre_zero_quiet_r15_20261016.json'
OLD_PROFILE='profiles/r85_f1_f3_handoff_seed_recovery_r14_20261016.json'
QUIET_BASELINE='0934e3aba0fb34a910756f129bad889d1d7dc1f9'
BINDINGS_SHA='ffc832463c766ab8ba5367c8cc1d0fe1af40713d2ddf31558dbabd6fc840c470'
SOURCE_REPO=WORK.parent/'candidate_source_history'
MODE_PATH=WORK/'R15_CONTROL_ONLY_MODE_SELECTION.json'
MODE_SHA='335f5634dd9ef071d6cd5a3556e5ed3fc40f3cf443eb6f2b32d9fc4d9edc1089'
HISTORICAL_HELPER_SHA='dca09251ffff607e965600cef49b8124f41ac8cd9e7588eddcb993a29358040d'
HISTORICAL_PROOF_SHA='6b6bf2ae0a09014d4dfdb0a3f4cd77b0c5c015314460b57f6be5b3cbe77f5089'
HISTORICAL_PROFILE_SHA='3a16cfe8f9953f11bb6019c9675b3cac241a9a4f5d2d9db6c0c70240ca5e4253'
BASELINE_INSTALL_SHA='e5bc17da0a976ab445f3ca9d93e9888203a80e2f142a994b88f474791353a88f'
SOURCE_DELTAS={'controller.py': [{'old_start': 450, 'old_end': 451, 'new_start': 450, 'new_end': 456, 'old_lines': ['        before=self.pose_stamp\n'], 'new_lines': ["        before=self.pose_stamp;quiet=getattr(self.heading_gate,'quiet_target_enabled',False)\n", '        if quiet:\n', '            self.heading_gate.begin_quiet_pose(stamp,clock,wall,self.heading_gate.transition_context,\n', '                self.heading_gate.transition_purpose,self.handoff_scope_sha256,self.cascade_path_id,\n', '                None if self.cascade is None else self.cascade.path_sha,\n', '                None if self.cascade is None else self.cascade.goal_sha)\n']}, {'old_start': 452, 'old_end': 453, 'new_start': 457, 'new_end': 460, 'old_lines': ['        finally:self.gyro_body,self.gyro_stamp,self.gyro_wall=latest\n'], 'new_lines': ['        finally:\n', '            self.gyro_body,self.gyro_stamp,self.gyro_wall=latest\n', '            if quiet:self.heading_gate.finish_quiet_pose(stamp,self.pose_stamp>before)\n']}, {'old_start': 1385, 'old_end': 1385, 'new_start': 1392, 'new_end': 1396, 'old_lines': [], 'new_lines': ["        if getattr(self.heading_gate,'quiet_target_enabled',False):\n", "            self.pid_row['quiet_handoff_binding']=self.heading_gate._quiet_binding(self.heading_gate.transition_context,\n", "                self.heading_gate.transition_purpose,self.handoff_scope_sha256,diagnostic.get('path_id'),\n", "                diagnostic.get('path_sha256'),diagnostic.get('fixed_goal_sha256'))\n"]}, {'old_start': 1716, 'old_end': 1716, 'new_start': 1727, 'new_end': 1753, 'old_lines': [], 'new_lines': ["        if getattr(gate,'quiet_target_enabled',False):\n", "            record=None;core={}if row is None else row.get('cascade',{})\n", "            ref={}if row is None else row.get('heading_gate_reference',{})\n", "            f=None if row is None else row.get('feedback');i=None if row is None else row.get('imu')\n", '            binding=gate._quiet_binding(gate.transition_context,gate.transition_purpose,self.handoff_scope_sha256,\n', '                self.cascade_path_id,None if self.cascade is None else self.cascade.path_sha,\n', '                None if self.cascade is None else self.cascade.goal_sha)\n', '            normal=bool(row is not None and isinstance(f,dict) and isinstance(i,dict)\n', "                and 'received_wall_ns'in f and 'received_wall_ns'in i\n", "                and velocity is not None and self.state=='running' and gate.controlled_braking and gate.handoff_active\n", "                and core.get('mode')=='brake' and core.get('controller_updated')is True and core.get('protection_active')is False\n", "                and ref.get('phase')=='settle' and ref.get('controlled_braking')is True\n", "                and ref.get('settle_control_stage')=='controlled_braking' and row.get('quiet_handoff_binding')==binding\n", "                and row.get('request_id')==self.request_id and row.get('waypoint_index')==self.waypoint_index\n", "                and not self.obstacle_hold and not self.tilt_hold and not getattr(self,'reference_consistency_hold',False)\n", "                and not getattr(self,'path_admission_hold',False) and self.samples is not None\n", '                and not self.control_clock_hold.holding and not self.control_clock_hold.guard_required\n', '                and not self.pid_guard_needs_evaluation and self.evidence.error is None\n', "                and not row.get('launch_output_constraint',{}).get('applied',False))\n", '            if normal:\n', "                record=dict(sequence=self.counts['commands'],publish_clock_ns=self.get_clock().now().nanoseconds,\n", "                    publish_wall_ns=time.monotonic_ns(),control_clock_ns=row['compute_ros_clock_ns'],\n", "                    control_wall_ns=core['compute_wall_ns'],source_pose_stamp_ns=row['source_pose_stamp_ns'],\n", "                    paired_imu_stamp_ns=row['paired_imu_stamp_ns'],source_pose_received_wall_ns=f['received_wall_ns'],\n", "                    source_imu_received_wall_ns=i['received_wall_ns'],command_body=list(self.command),normal_braking=True,**binding)\n", '            gate.note_quiet_publication(record)\n']}], 'transition_gate.py': [{'old_start': 11, 'old_end': 11, 'new_start': 11, 'new_end': 12, 'old_lines': [], 'new_lines': ['from numbers import Real\n']}, {'old_start': 36, 'old_end': 36, 'new_start': 37, 'new_end': 51, 'old_lines': [], 'new_lines': ["        quiet=profile.get('r85_quiet_handoff_target',{'enabled':False})\n", "        if (not isinstance(quiet,dict) or set(quiet)!={'enabled'} or type(quiet.get('enabled'))is not bool):\n", "            raise ValueError('r85_quiet_handoff_target requires only explicit boolean enabled')\n", "        self.quiet_target_enabled=quiet['enabled']\n", '        if self.quiet_target_enabled and (not self.active_braking_enabled\n', "                or profile.get('cascade',{}).get('external_heading_gate')is not True\n", "                or profile.get('cascade',{}).get('r85_turn_brake_reference')!={'kind':'locked_heading_pd_v1'}):\n", "            raise ValueError('Quiet handoff target requires active locked-heading braking')\n", '        # Half the original drift budgets over5s are conservative command\n', '        # targets, not a claim that velocity commands guarantee physical drift.\n', '        self.quiet_limits=dict(xy=.05/2/5.,yaw=.1/2/5.,heading=.005)\n', '        self.quiet_history=deque(maxlen=64);self.quiet_publication=None;self.quiet_binding=None\n', '        self.quiet_pose_context=None;self.quiet_observed_stamp_ns=None;self.quiet_ack_watermark=None\n', "        self.quiet_reason='not_observed';self.last_quiet_qualification=None\n"]}, {'old_start': 56, 'old_end': 56, 'new_start': 71, 'new_end': 74, 'old_lines': [], 'new_lines': ['        if self.quiet_target_enabled:\n', "            self._clear_quiet('reset');self.quiet_publication=None;self.quiet_binding=None\n", '            self.quiet_ack_watermark=None;self.quiet_pose_context=None\n']}, {'old_start': 89, 'old_end': 89, 'new_start': 107, 'new_end': 108, 'old_lines': [], 'new_lines': ["        if self.quiet_target_enabled:self._clear_quiet('handoff_failed')\n"]}, {'old_start': 90, 'old_end': 90, 'new_start': 109, 'new_end': 216, 'old_lines': [], 'new_lines': ['    def _clear_quiet(self,reason):\n', '        self.quiet_history.clear();self.quiet_reason=str(reason)\n', '    @staticmethod\n', '    def _quiet_binding(context,purpose,scope,path_id,path_sha256,goal_sha256):\n', '        return dict(context=copy.deepcopy(context),purpose=purpose,scope=scope,\n', '            path_id=path_id,path_sha256=path_sha256,goal_sha256=goal_sha256)\n', '    def _low_total_command(self,value):\n', '        return bool(isinstance(value,(list,tuple)) and len(value)==3\n', '            and all(isinstance(v,Real) and not isinstance(v,bool) and math.isfinite(v)for v in value)\n', "            and math.hypot(value[0],value[1])<=self.quiet_limits['xy']\n", "            and abs(value[2])<=self.quiet_limits['yaw'])\n", '    @staticmethod\n', '    def _quiet_publication_fresh(record,clock_ns,wall_ns):\n', '        return bool(all(0<=clock_ns-record[k]<300_000_000 for k in\n', "            ('publish_clock_ns','control_clock_ns','source_pose_stamp_ns','paired_imu_stamp_ns'))\n", '            and all(0<=wall_ns-record[k]<300_000_000 for k in\n', "                ('publish_wall_ns','control_wall_ns','source_pose_received_wall_ns','source_imu_received_wall_ns')))\n", '    def note_quiet_publication(self,record):\n', '        if not self.quiet_target_enabled:return False\n', '        self.quiet_publication=None\n', '        try:\n', '            if not (self.controlled_braking and self.handoff_active and isinstance(record,dict)\n', "                    and record['normal_braking']is True and self._low_total_command(record['command_body'])\n", "                    and record['context']==self.transition_context and record['purpose']==self.transition_purpose\n", "                    and record['context']==self.handoff_event['context']):raise ValueError('publication_not_quiet_brake_math')\n", "            clocks=('sequence','publish_clock_ns','publish_wall_ns','control_clock_ns','control_wall_ns',\n", "                'source_pose_stamp_ns','paired_imu_stamp_ns','source_pose_received_wall_ns','source_imu_received_wall_ns')\n", "            if any(type(record[k])is not int or record[k]<0 for k in clocks):raise ValueError('publication_integer_clocks_required')\n", "            if not (self._quiet_publication_fresh(record,record['publish_clock_ns'],record['publish_wall_ns'])\n", "                    and 0<=record['source_pose_stamp_ns']-record['paired_imu_stamp_ns']<=20_000_000\n", "                    and record['control_clock_ns']/1e9>=self.handoff_event['brake_entry_control_s']):raise ValueError('publication_source_not_fresh')\n", "            binding=self._quiet_binding(*(record[k]for k in ('context','purpose','scope','path_id','path_sha256','goal_sha256')))\n", "            if not (isinstance(record['path_id'],str) and record['path_id']\n", "                    and all(isinstance(record[k],str) and len(record[k])==64 for k in ('scope','path_sha256','goal_sha256'))):\n", "                raise ValueError('publication_source_binding_missing')\n", '            if self.quiet_binding is not None and binding!=self.quiet_binding:\n', "                if any(binding[k]!=self.quiet_binding[k]for k in ('context','purpose','scope','goal_sha256')):\n", "                    raise ValueError('publication_source_binding_changed')\n", '                # A legitimate same-goal SCAN path refresh restarts only this\n', '                # internal target window, never the handoff event or budget.\n', "                self._clear_quiet('publication_path_refreshed')\n", '            self.quiet_binding=copy.deepcopy(binding)\n', '            self.quiet_publication=copy.deepcopy(record);return True\n', '        except (ValueError,TypeError,KeyError)as error:\n', '            self._clear_quiet(error);return False\n', '    def begin_quiet_pose(self,stamp_ns,control_clock_ns,wall_ns,context,purpose,scope,path_id,path_sha256,goal_sha256):\n', '        if not self.quiet_target_enabled:return\n', '        self.quiet_observed_stamp_ns=None\n', '        self.quiet_pose_context=dict(stamp_ns=stamp_ns,clock_ns=control_clock_ns,wall_ns=wall_ns,\n', '            binding=self._quiet_binding(context,purpose,scope,path_id,path_sha256,goal_sha256))\n', '    def finish_quiet_pose(self,stamp_ns,accepted):\n', '        if self.quiet_target_enabled and accepted and self.quiet_observed_stamp_ns!=stamp_ns:\n', "            self._clear_quiet('accepted_pose_without_qualified_motion_observation')\n", '    def _observe_quiet(self,stamp_s,wall_s,brake_valid,observed_yaw):\n', '        stamp_ns=round(stamp_s*1e9);self.quiet_observed_stamp_ns=stamp_ns\n', '        p=self.quiet_publication;obs=self.quiet_pose_context;ack=self.zero_ack_candidate;m=self.latest_motion\n', '        try:\n', '            if not (brake_valid and p is not None and obs is not None and ack is not None\n', "                    and obs['stamp_ns']==stamp_ns and obs['binding']==self.quiet_binding\n", "                    and p['context']==self.transition_context and p['purpose']==self.transition_purpose\n", "                    and abs(self.error(self.heading,observed_yaw))<=self.quiet_limits['heading']\n", "                    and abs(m['slam_wz'])<=self.quiet_limits['yaw'] and abs(m['body_gyro'][2])<=self.quiet_limits['yaw']):\n", "                raise ValueError('motion_heading_or_publication_not_quiet')\n", "            if not (self._quiet_publication_fresh(p,obs['clock_ns'],round(wall_s*1e9))\n", "                    and 0<=stamp_ns-ack['input_clock_ns']<300_000_000\n", "                    and 0<=round(wall_s*1e9)-ack['received_wall_ns']<300_000_000\n", "                    and ack['source_scope_sha256']==p['scope']\n", "                    and (ack['request_id'],ack['waypoint_index'])==(self.transition_context.get('request_id'),self.transition_context.get('waypoint_index'))\n", "                    and ack['input_clock_ns']>round(self.handoff_event['brake_entry_control_s']*1e9)\n", "                    and self._low_total_command(ack['requested_command_body']) and self._low_total_command(ack['applied_command_body'])):\n", "                raise ValueError('actual_commands_or_causal_ACK_not_quiet')\n", "            watermark=tuple(ack[k]for k in ('sequence','input_clock_ns','input_frame_serial','inference_sequence'))\n", '            if self.quiet_ack_watermark is not None and any(a<=b for a,b in zip(watermark,self.quiet_ack_watermark)):\n', "                raise ValueError('quiet_ACK_frame_not_independent')\n", '            self.quiet_ack_watermark=watermark\n', "            if self.quiet_history and stamp_s-self.quiet_history[-1]['stamp_s']>self.config['stop_pose_gap_max_s']:\n", "                self._clear_quiet('quiet_pose_gap')\n", "            self.quiet_history.append(dict(stamp_s=stamp_s,wall_s=wall_s,publication_sequence=p['sequence'],\n", '                ack_watermark=list(watermark),heading_error_rad=self.error(self.heading,observed_yaw),\n', "                slam_wz=m['slam_wz'],body_wz=m['body_gyro'][2]))\n", '            self.quiet_reason=None\n', '        except (ValueError,TypeError,KeyError)as error:self._clear_quiet(error)\n', '    def _quiet_qualified(self,sim_time,wall_time,error):\n', '        if not self.quiet_target_enabled:return True\n', '        h=self.quiet_history;p=self.quiet_publication;ack=self.zero_ack_candidate\n', '        clock_ns=round(sim_time*1e9);wall_ns=round(wall_time*1e9)\n', "        if not (h and p is not None and ack is not None and abs(error)<=self.quiet_limits['heading']\n", "                and h[-1]['stamp_s']==self.latest_motion['stamp_s']\n", "                and 0<=wall_time-h[-1]['wall_s']<.3 and -.05<=sim_time-h[-1]['stamp_s']<.3\n", '                and self._quiet_publication_fresh(p,clock_ns,wall_ns)\n', "                and 0<=clock_ns-ack['input_clock_ns']<300_000_000\n", "                and 0<=wall_ns-ack['received_wall_ns']<300_000_000\n", "                and ack['ack_native_clock_ns']<=clock_ns\n", "                and ack['input_clock_ns']<=round(self.latest_motion['stamp_s']*1e9)\n", "                and [ack[k]for k in ('sequence','input_clock_ns','input_frame_serial','inference_sequence')]==h[-1]['ack_watermark']\n", "                and ack['source_scope_sha256']==p['scope']\n", "                and (ack['request_id'],ack['waypoint_index'])==(self.transition_context.get('request_id'),self.transition_context.get('waypoint_index'))\n", "                and p['context']==self.transition_context and p['purpose']==self.transition_purpose\n", "                and self._low_total_command(p['command_body'])\n", "                and self._low_total_command(ack['requested_command_body']) and self._low_total_command(ack['applied_command_body'])):\n", "            self._clear_quiet('quiet_target_not_current');return False\n", "        ready=(len(h)>=self.config['stop_min_pose_samples']\n", "            and h[-1]['stamp_s']-h[0]['stamp_s']>=self.config['stop_window_sim_s']-1e-9)\n", "        if ready:self.last_quiet_qualification=dict(first_stamp_s=h[0]['stamp_s'],last_stamp_s=h[-1]['stamp_s'],\n", '            samples=len(h),actual_control_s=sim_time,last_publication=copy.deepcopy(p),last_pose=copy.deepcopy(h[-1]),\n', '            low_total_command_target_only=True,original_stop_PASS=False)\n', '        return ready\n']}, {'old_start': 190, 'old_end': 190, 'new_start': 316, 'new_end': 320, 'old_lines': [], 'new_lines': ['        if self.quiet_target_enabled:\n', "            self._clear_quiet('brake_entry');self.quiet_publication=None;self.quiet_binding=None;self.last_quiet_qualification=None\n", "            self.quiet_ack_watermark=(tuple(self.zero_ack_highwater[k]for k in ('sequence','input_clock_ns','input_frame_serial','inference_sequence'))\n", '                if self.zero_ack_highwater is not None else None)\n']}, {'old_start': 212, 'old_end': 213, 'new_start': 342, 'new_end': 343, 'old_lines': ['            heading_valid=False\n'], 'new_lines': ['            heading_valid=False;observed_yaw=None\n']}, {'old_start': 224, 'old_end': 224, 'new_start': 354, 'new_end': 355, 'old_lines': [], 'new_lines': ['            if self.quiet_target_enabled:self._observe_quiet(stamp_s,wall_s,brake_valid,observed_yaw)\n']}, {'old_start': 336, 'old_end': 336, 'new_start': 467, 'new_end': 468, 'old_lines': [], 'new_lines': ['                if not self._quiet_qualified(sim_time,wall_time,error):return False,0.\n']}, {'old_start': 372, 'old_end': 373, 'new_start': 504, 'new_end': 505, 'old_lines': ["        return {'transition':'teacher_measured_stop_heading_v1','phase':self.phase,\n"], 'new_lines': ["        result={'transition':'teacher_measured_stop_heading_v1','phase':self.phase,\n"]}, {'old_start': 390, 'old_end': 390, 'new_start': 522, 'new_end': 529, 'old_lines': [], 'new_lines': ['        if self.quiet_target_enabled:\n', '            h=self.quiet_history\n', "            result['quiet_handoff_target']=dict(enabled=True,target='low_total_command_only',limits=copy.deepcopy(self.quiet_limits),\n", "                samples=len(h),first_stamp_s=h[0]['stamp_s']if h else None,last_stamp_s=h[-1]['stamp_s']if h else None,\n", '                reason=self.quiet_reason,last_qualification=copy.deepcopy(self.last_quiet_qualification),\n', '                original_stop_PASS=False,physical_drift_guaranteed=False)\n', '        return result\n']}]}
SOURCE_BEFORE={'controller.py': '54a3b788f2e796c43bd749932916ccea76278bb552e671c711373659bd58d08f', 'transition_gate.py': 'e33190adfa679a3a9bb955930e6f8425ef20f3895fb3abb342a11c1d835291c8', 'scan_workspace.py': '8d5a55c606db6f1c97740d51424ff949f178fa1d8aea0567e1b3812f87c0e540'}
SCANNER_PATCHES=[('def _heading_control_adapter(session,base,renewal=None,repair=None):', 'def _heading_control_adapter(session,base,renewal=None,repair=None,quiet=None):'), ("        if repair is not None and filename=='controller.py':live_control_sha=repair['controller']['after_sha256']", "        if repair is not None and filename=='controller.py':live_control_sha=repair['controller']['after_sha256']\n        if quiet is not None and filename in ('controller.py','transition_gate.py'):live_control_sha=quiet['sources'][filename]['after_sha256']"), ("def _control_adapter(session,base):\n    if 'handoff_seed_recovery_proof' in session or 'handoff_seed_recovery_proof_sha256' in session:", "def _control_adapter(session,base):\n    if 'pre_zero_quiet_proof' in session or 'pre_zero_quiet_proof_sha256' in session:\n        from pre_zero_quiet_adapter import prepare_quiet,attach_quiet\n        from schedule_renewal_adapter import attach_renewal\n        quiet,historical,live_schedule,live_repair=prepare_quiet(session,base)\n        return attach_quiet(attach_renewal(_heading_control_adapter(session,base,live_schedule,live_repair,quiet),historical),quiet)\n    if 'handoff_seed_recovery_proof' in session or 'handoff_seed_recovery_proof_sha256' in session:"), ("        resolved_repair(resolved,adapter['handoff_seed_recovery'])\n    return resolved", "        resolved_repair(resolved,adapter['handoff_seed_recovery'])\n    if 'pre_zero_quiet' in adapter:\n        from pre_zero_quiet_adapter import resolved_quiet\n        resolved_quiet(resolved,adapter['pre_zero_quiet'])\n    return resolved")]
HISTORICAL_PATCHES=[('def _load_historical_schedule(session,base,repair):', 'def _load_historical_schedule(session,base,repair,quiet):'), ("require(Path(__file__).resolve()==ROOT/'handoff_seed_recovery_adapter.py','active repair helper location')", "require(Path(__file__).resolve()==ROOT/'pre_zero_quiet_adapter.py','active quiet helper location')"), ("_verified_repaired_scanner(expected,(ROOT/'scan_workspace.py').read_bytes(),repair)", "_verified_historical_scanner(expected,(ROOT/'scan_workspace.py').read_bytes(),repair,quiet)"), ('_verified_repaired_controller(path,sha,repair)', '_verified_historical_controller(path,sha,repair,quiet)'), ('        else:evidence.append(file_ref(dict(path=path,sha256=sha)))', "        elif path==str(ROOT/'transition_gate.py'):\n            _verified_quiet_source('transition_gate.py',sha,quiet)\n            evidence.extend([Path(path),Path(quiet['sources']['transition_gate.py']['beforeimage']['path'])])\n        else:evidence.append(file_ref(dict(path=path,sha256=sha)))")]
EXACT_TARGETS=['controller.py','transition_gate.py','scan_workspace.py','pre_zero_quiet_adapter.py','R85_PRE_ZERO_QUIET_CONTROL_ADAPTER_CONTRACT.json',QUIET_PROFILE]


def quiet_source_after_bytes(name,raw):
    require(name in SOURCE_DELTAS and digest(raw)==SOURCE_BEFORE[name],'quiet exact source beforeimage: '+name)
    lines=raw.decode().splitlines(keepends=True)
    for row in reversed(SOURCE_DELTAS[name]):
        i,j=row['old_start'],row['old_end']
        require(lines[i:j]==row['old_lines'],'quiet exact source delta context: '+name)
        lines[i:j]=row['new_lines']
    return ''.join(lines).encode()


def quiet_scanner_after_bytes(raw):
    require(digest(raw)==SOURCE_BEFORE['scan_workspace.py'],'quiet scanner actual R14 beforeimage')
    text=raw.decode()
    for old,new in SCANNER_PATCHES:
        require(text.count(old)==1,'quiet scanner exact replacement occurrence')
        text=text.replace(old,new)
    return text.encode()


def _verified_quiet_source(name,historical_sha,quiet):
    row=quiet['sources'][name]
    require(row['before_sha256']==historical_sha==SOURCE_BEFORE[name],'quiet source historical identity: '+name)
    before=file_ref(row['beforeimage'],QUIET_STAGE/'beforeimage'/name,historical_sha)
    expected=(quiet_scanner_after_bytes(before.read_bytes())if name=='scan_workspace.py'else quiet_source_after_bytes(name,before.read_bytes()))
    require(digest(expected)==row['after_sha256'] and verify_historical_actual(name,expected),'quiet historical identity and explicit actual unload identity: '+name)
    return True


def _verified_historical_scanner(historical_bytes,actual_bytes,repair,quiet):
    row=repair['validator'];before=file_ref(row['beforeimage'],historical_repair.REPAIR_STAGE/'beforeimage/scan_workspace.py',historical_repair.SCANNER_BEFORE)
    r14=historical_repair.repair_scanner_after_bytes(before.read_bytes())
    require(before.read_bytes()==historical_bytes and digest(r14)==row['after_sha256']==SOURCE_BEFORE['scan_workspace.py'],'retained R13 to R14 scanner identity')
    snapshot=file_ref(quiet['sources']['scan_workspace.py']['beforeimage'],QUIET_STAGE/'beforeimage/scan_workspace.py',row['after_sha256'])
    require(snapshot.read_bytes()==r14 and verify_historical_actual('scan_workspace.py',quiet_scanner_after_bytes(r14)) and actual_bytes==(ROOT/'scan_workspace.py').read_bytes(),'explicit R13-R14-R15-R16 scanner chain')
    return _verified_quiet_source('scan_workspace.py',row['after_sha256'],quiet)


def _verified_historical_controller(path,historical_sha,repair,quiet):
    row=repair['controller'];require(path==str(ROOT/'controller.py')and historical_sha==historical_repair.CONTROLLER_BEFORE,'historical controller path and R13 identity')
    before=file_ref(row['beforeimage'],historical_repair.REPAIR_STAGE/'beforeimage/controller.py',historical_sha)
    r14=historical_repair.repair_controller_after_bytes(before.read_bytes())
    snapshot=file_ref(quiet['sources']['controller.py']['beforeimage'],QUIET_STAGE/'beforeimage/controller.py',row['after_sha256'])
    require(digest(r14)==row['after_sha256']==SOURCE_BEFORE['controller.py']and snapshot.read_bytes()==r14,'retained R13 to R14 controller identity')
    return _verified_quiet_source('controller.py',row['after_sha256'],quiet)


def _load_historical_schedule(session,base,repair,quiet):
    require(Path(__file__).resolve()==RENEW_HELPER,'active14UTC historical identity bridge location')
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
        and _verify_1400_guard(expected),'historical guard exact three substitutions plus declared14UTC delta')
    require(v3['controls'][guard['path']]['after_sha256']==OLD_GUARD_SHA,'v3 guard historical identity')
    evidence.extend([before,ROOT/'r85_session_guard.py'])
    require(set(validator)=={'path','before_sha256','after_sha256','beforeimage','change'}
        and validator['path']==str(ROOT/'scan_workspace.py')and validator['before_sha256']==OLD_SCANNER_SHA
        and validator['change']=='Exact schedule-only bridge replacement table in pinned new helper; inverse recovers original fd893 bytes.'
        and v3['validator']['after_sha256']==OLD_SCANNER_SHA,'validator exact declaration')
    before=file_ref(validator['beforeimage'],STAGE/'beforeimage/scan_workspace.py',OLD_SCANNER_SHA)
    expected=scanner_after_bytes(before.read_bytes());require(digest(expected)==validator['after_sha256']
        and _verified_historical_scanner(expected,(ROOT/'scan_workspace.py').read_bytes(),repair,quiet),'validator historical schedule and actual repair bridge')
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
            _verified_historical_controller(path,sha,repair,quiet)
            evidence.extend([Path(path),Path(repair['controller']['beforeimage']['path'])])
        elif path==str(ROOT/'transition_gate.py'):
            _verified_quiet_source('transition_gate.py',sha,quiet)
            evidence.extend([Path(path),Path(quiet['sources']['transition_gate.py']['beforeimage']['path'])])
        elif path==str(ROOT/'cascade_core.py'):
            verify_historical_actual('cascade_core.py',(UNLOAD_STAGE/'beforeimage/cascade_core.py').read_bytes())
            require(SOURCE_BEFORE_UNLOAD['cascade_core.py']==sha,'historical core exact original identity')
            evidence.extend([Path(path),UNLOAD_STAGE/'beforeimage/cascade_core.py'])
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


def _verify_reference_tree(value,evidence):
    if isinstance(value,dict):
        if set(value)=={'path','sha256'}:
            evidence.append(file_ref(value))
        else:
            for item in value.values():_verify_reference_tree(item,evidence)
    elif isinstance(value,list):
        for item in value:_verify_reference_tree(item,evidence)


def _verify_binding_closure(quiet,profile_sha,proof_sha):
    baseline=file_ref(quiet['baseline_source_bindings'],QUIET_STAGE/'beforeimage/SOURCE_BINDINGS.json',BINDINGS_SHA)
    old=historical_repair._binding_map(json.loads(baseline.read_text()))
    require(len(old)==255,'actual retained R14 255 source snapshot')
    expected={path:row['sha256']for path,row in old.items()}
    for name,row in quiet['sources'].items():
        require(old[str(ROOT/name)]['sha256']==row['before_sha256'],'source before identity matches R14 source binding')
        expected[str(ROOT/name)]=row['after_sha256']
    added={str(QUIET_HELPER):quiet['bridge_helper']['sha256'],str(QUIET_PROOF):proof_sha,str(ROOT/QUIET_PROFILE):profile_sha}
    require(not(set(added)&set(expected)),'three genuinely new quiet bound sources')
    expected.update(added)
    current_path=UNLOAD_STAGE/'beforeimage/SOURCE_BINDINGS.json';current=historical_repair._binding_map(json.loads(current_path.read_text()))
    require(digest(current_path.read_bytes())==UNLOAD_BINDINGS_SHA,'retained exact actual R15 258 binding snapshot')
    require(len(current)==258 and set(current)==set(expected),'actual exact 255 to258 closure')
    evidence=[baseline,current_path]
    for path,sha in expected.items():
        row=current[path]
        name=Path(path).name
        if path==str(ROOT/name) and name in SOURCE_BEFORE_UNLOAD:
            active=UNLOAD_STAGE/'beforeimage'/name
            require(active.is_file()and not active.is_symlink()and digest(active.read_bytes())==sha,'historical R15 source beforeimage')
            verify_historical_actual(name,active.read_bytes())
        elif path==str(ROOT/'r85_session_guard.py'):
            active=RENEW_STAGE/'beforeimage/r85_session_guard.py'
            require(digest(active.read_bytes())==sha and _verify_1400_guard(active.read_bytes()),'historical10UTC guard and truthful14UTC current bytes')
        else:
            active=file_ref(dict(path=path,sha256=sha))
            tracked=SOURCE_REPO/row['tracked'];require(tracked.is_file()and not tracked.is_symlink()and digest(tracked.read_bytes())==sha,'unchanged tracked evidence matches actual source')
        require(row['sha256']==sha and row.get('bytes')==active.stat().st_size,'historical binding real bytes/hash')
        evidence.append(active)
    return evidence


def load_quiet(session,base):
    require(Path(__file__).resolve()==RENEW_HELPER,'actual14UTC historical quiet reader location')
    require(Path(historical_quiet.__file__).resolve()==QUIET_HELPER and digest(QUIET_HELPER.read_bytes())==HISTORICAL_QUIET_HELPER_SHA,'real unchanged historical R15 reader')
    proof_ref=dict(path=session.get('pre_zero_quiet_proof'),sha256=session.get('pre_zero_quiet_proof_sha256'))
    path=file_ref(proof_ref,QUIET_PROOF);quiet=json.loads(path.read_text())
    keys={'schema','kind','baseline_commit','expected_bound_source_count_before','expected_bound_source_count_after','exact_targets','simulation_only','runtime_acceptance_inherited','navigation_acceptance_established','source_or_feedback_guards_changed','physical_thresholds_changed','scan_source_changed','scan_binary_changed','scan_contract_bytes_changed','controller_student_PID_math_changed','guard_async_epoch_known_scene_changed','mode_selection','baseline_install','baseline_source_bindings','historical_R14_helper','historical_R14_proof','historical_R14_profile','sources','bridge_helper','profile','meaning'}
    require(set(quiet)==keys and quiet['schema']=='r85_pre_zero_quiet_source_adapter/v1'and quiet['kind']=='pre_final_zero_low_total_command_target_v1'
        and quiet['baseline_commit']==QUIET_BASELINE and quiet['expected_bound_source_count_before']==255 and quiet['expected_bound_source_count_after']==258
        and quiet['exact_targets']==EXACT_TARGETS and quiet['simulation_only']is True
        and all(quiet[k]is False for k in ('runtime_acceptance_inherited','navigation_acceptance_established','source_or_feedback_guards_changed','physical_thresholds_changed','scan_source_changed','scan_binary_changed','scan_contract_bytes_changed','controller_student_PID_math_changed','guard_async_epoch_known_scene_changed')),'quiet exact proof scope')
    mode=file_ref(quiet['mode_selection'],MODE_PATH,MODE_SHA);selection=json.loads(mode.read_text())
    require(selection['mode']=='simulation_observer_diagnostic_controlled_comparison'and selection['strict_source_authority_candidate_selected']is False
        and selection['strict_candidate_installed']is False and selection['original_acceptance_thresholds_changed']is False,'root selected diagnostic control-only mode')
    evidence=[path,mode]
    receipt=file_ref(quiet['baseline_install'],WORK/'round_14_handoff_seed_recovery/install_tools/INSTALL_RECEIPT.json',BASELINE_INSTALL_SHA)
    receipt_data=json.loads(receipt.read_text());require(receipt_data.get('installed')is True and receipt_data.get('candidate_commit')==QUIET_BASELINE,'actual R14 installation')
    oldhelper=file_ref(quiet['historical_R14_helper'],ROOT/'handoff_seed_recovery_adapter.py',HISTORICAL_HELPER_SHA)
    require(Path(historical_repair.__file__).resolve()==oldhelper,'real retained R14 helper module')
    oldproof=file_ref(quiet['historical_R14_proof'],ROOT/'R85_HANDOFF_SEED_RECOVERY_CONTROL_ADAPTER_CONTRACT.json',HISTORICAL_PROOF_SHA)
    oldprofile=file_ref(quiet['historical_R14_profile'],QUIET_STAGE/'beforeimage'/OLD_PROFILE,HISTORICAL_PROFILE_SHA)
    evidence.extend([receipt,oldhelper,oldproof,oldprofile,file_ref(dict(path=str(ROOT/OLD_PROFILE),sha256=HISTORICAL_PROFILE_SHA))])
    repair=json.loads(oldproof.read_text());_verify_reference_tree(repair,evidence)
    require(Path(historical_schedule.__file__).resolve()==ROOT/'schedule_renewal_adapter.py','real retained historical schedule module')
    require(set(quiet['sources'])=={'controller.py','transition_gate.py','scan_workspace.py'},'only three existing quiet source targets')
    for name,row in quiet['sources'].items():
        require(set(row)=={'path','before_sha256','after_sha256','beforeimage','delta_sha256'}and row['path']==str(ROOT/name),'quiet exact source row')
        table=SCANNER_PATCHES if name=='scan_workspace.py'else SOURCE_DELTAS[name]
        require(row['delta_sha256']==digest(json.dumps(table,sort_keys=True,separators=(',',':')).encode()),'quiet reviewed exact source delta declaration')
        _verified_quiet_source(name,SOURCE_BEFORE[name],quiet);evidence.extend([ROOT/name,Path(row['beforeimage']['path'])])
    helper=file_ref(quiet['bridge_helper'],QUIET_HELPER);evidence.append(helper)
    p=quiet['profile'];require(set(p)=={'path','baseline_profile','added_session_keys','added_top_level_flags'}and p['path']==str(ROOT/QUIET_PROFILE)
        and p['baseline_profile']==dict(path=str(ROOT/OLD_PROFILE),sha256=HISTORICAL_PROFILE_SHA)
        and p['added_session_keys']==['pre_zero_quiet_proof','pre_zero_quiet_proof_sha256']
        and p['added_top_level_flags']=={'r85_quiet_handoff_target':{'enabled':True}},'exact profile declaration')
    expected=json.loads(oldprofile.read_text());expected['r85_quiet_handoff_target']={'enabled':True}
    expected['r85_session']['pre_zero_quiet_proof']=str(QUIET_PROOF);expected['r85_session']['pre_zero_quiet_proof_sha256']=proof_ref['sha256']
    profile_path=ROOT/QUIET_PROFILE;raw=profile_path.read_bytes();actual=json.loads(raw)
    require(actual==expected and session==actual['r85_session'],'R15 profile exact two identity keys and quiet target flag only')
    evidence.append(profile_path);evidence.extend(_verify_binding_closure(quiet,digest(raw),proof_ref['sha256']))
    quiet['proof']=proof_ref;quiet['evidence']=sorted(set(evidence));quiet['historical_repair']=repair
    return quiet


def prepare_quiet(session,base):
    quiet=load_quiet(session,base);repair=quiet['historical_repair'];historical_session=copy.deepcopy(session)
    for key in ('pre_zero_quiet_proof','pre_zero_quiet_proof_sha256','handoff_seed_recovery_proof','handoff_seed_recovery_proof_sha256'):del historical_session[key]
    historical=_load_historical_schedule(historical_session,base,repair,quiet)
    live_schedule=copy.deepcopy(historical);live_schedule['validator']['after_sha256']=quiet['sources']['scan_workspace.py']['after_sha256']
    live_repair=copy.deepcopy(repair);live_repair['controller']['after_sha256']=quiet['sources']['controller.py']['after_sha256']
    live_repair['validator']['after_sha256']=quiet['sources']['scan_workspace.py']['after_sha256']
    return quiet,historical,live_schedule,live_repair


def attach_quiet(adapter,quiet):
    adapter['pre_zero_quiet']=quiet
    for key in ('bridge_helper','historical_R14_helper'):
        row=quiet[key];adapter['actual'][row['path']]=row['sha256']
    adapter['evidence']=sorted(set(adapter['evidence']+quiet['evidence']))
    return adapter


def resolved_quiet(resolved,quiet):
    target=resolved['resolved_adapter'];target['schema']='r85_pre_zero_quiet_resolved_bindings/v1'
    for name,row in quiet['sources'].items():
        path=row['path'];require(target['actual_control_sources_sha256'][path]==row['after_sha256'],'resolved truthful quiet current source')
        if name!='transition_gate.py':require(target['overrides'][path]['after_sha256']==row['after_sha256'],'original three-base overrides retain truthful actual current source')
    target['schedule_renewal']['source_identity_status']='historical_pre_R14_R15'
    target['schedule_renewal']['actual_identity_superseded_by']=copy.deepcopy(quiet['proof'])
    target['historical_handoff_seed_recovery']=dict(proof=copy.deepcopy(quiet['historical_R14_proof']),source_identity_status='historical_pre_R15',actual_identity_superseded_by=copy.deepcopy(quiet['proof']))
    target['pre_zero_quiet']=dict(proof=copy.deepcopy(quiet['proof']),baseline_commit=QUIET_BASELINE,mode_selection=copy.deepcopy(quiet['mode_selection']),
        current_sources={row['path']:row['after_sha256']for row in quiet['sources'].values()},
        old_R13_R14_proof_helper_profile_bytes_preserved=True,original_base_three_override_set_unchanged=True,
        guard_async_epoch_known_scene_unchanged=True,runtime_acceptance_inherited=False,navigation_acceptance_established=False)

# R16 explicit current identity extension. Earlier reader predicates above remain
# historical and are connected to actual current bytes only by exact reviewed deltas.
import pre_zero_quiet_adapter as historical_quiet
UNLOAD_STAGE=WORK/'pre_zero_integral_unload_source_integration'
UNLOAD_HELPER=ROOT/'pre_zero_integral_unload_adapter.py'
UNLOAD_PROOF=ROOT/'R85_PRE_ZERO_INTEGRAL_UNLOAD_CONTROL_ADAPTER_CONTRACT.json'
UNLOAD_PROFILE='profiles/r85_f1_f3_pre_zero_integral_unload_r16_20261016.json'
UNLOAD_OLD_PROFILE='profiles/r85_f1_f3_pre_zero_quiet_r15_20261016.json'
UNLOAD_BASELINE='b5f7b6b65fbd1fb53b026504dcfde42977ba3475'
UNLOAD_BINDINGS_SHA='9d2d71028f30fbf9bc7f1a0ebb6a8b3b6dea98d33443742cfb161cc585535d0f'
HISTORICAL_QUIET_HELPER_SHA='71c7c05f3071a03d32dc930f5423331534817aceda264af0f22a2a81b846410e'
SOURCE_BEFORE_UNLOAD={'cascade_core.py': 'f05efd2fb3cddf3f4652c949ccc436ce061d0cbc6ca482389360d46fb997eae7', 'controller.py': '85c418262ddae38a9c8c46bcc2dbe725dbad0a3e13389e3c17dd71817a813b98', 'student_actor_adapter.py': '2de4338a884499b5ac20cadaca1d08e588d30b55329913ff1acce0dd161176b7', 'student_lower_loop_ack.py': 'f13854eb0b75bec7554019d17c6bdd9afff3452b4ffac17e9db4fe23cf0e6839', 'transition_gate.py': '13d626f3e17d54d0f811cc63549f0d8167294c82231cc26099cd4d27f2b9d4c1', 'scan_workspace.py': '04a85638a2e99d94457789909b550edacbcc7c0469ad67eb7a986c5a78fb6aba'}
SOURCE_AFTER_UNLOAD={'cascade_core.py': '8f9a7276c15c86d7c1de13e619d59f1f27861fd12518dc5b454805148fc96bd8', 'controller.py': 'eb54d8b7c15df077de83724e527f51578aa40c361abbbd2ef0fc334fbabcce54', 'pre_zero_integral_unload.py': '84ea9e6fe1a9f0cea81c46aef75a23c8f6d50496c098fc79fcb333553ebdb5c8', 'student_actor_adapter.py': '6f14828e53831c997ae3cd845b243b75dbcecd91b5b24caaaa2be37357923275', 'student_lower_loop_ack.py': '92756657427bd447742411e644a3a68540875183fa51af7d595e3c30fcef119d', 'transition_gate.py': '011041952eea773db281be8716f519f384bcfca9a8d6998073e669fe009a3a71', 'scan_workspace.py': 'bf86b3ca9bfbd24cb7f66adbfb0d08abdd27eba4325efa23dabcbc1607a42551'}
UNLOAD_DELTAS={'cascade_core.py': [{'old_start': 360, 'old_end': 361, 'new_start': 360, 'new_end': 361, 'old_lines': ['    def _pi(self, target, actual, dt, integral, limits, ack, forward_only, turn_only=False):\n'], 'new_lines': ['    def _pi(self, target, actual, dt, integral, limits, ack, forward_only, turn_only=False, pre_zero_unload=None):\n']}, {'old_start': 366, 'old_end': 366, 'new_start': 366, 'new_end': 370, 'old_lines': [], 'new_lines': ['        unload_before=None\n', '        if pre_zero_unload is not None:\n', '            from pre_zero_integral_unload import toward_zero\n', '            unload_before=float(integral[2]);candidate[2]=toward_zero(unload_before,.1,dt)\n']}, {'old_start': 378, 'old_end': 378, 'new_start': 382, 'new_end': 383, 'old_lines': [], 'new_lines': ['        if unload_before is not None:candidate[2]=toward_zero(unload_before,.1,dt)\n']}, {'old_start': 387, 'old_end': 387, 'new_start': 392, 'new_end': 396, 'old_lines': [], 'new_lines': ['        if unload_before is not None:\n', '            from pre_zero_integral_unload import receipt\n', "            terms['pre_zero_integral_unload']=receipt(pre_zero_unload,'teacher',unload_before,candidate[2],dt,\n", '                self.last_clock,self.path_sha,self.goal_sha)\n']}, {'old_start': 390, 'old_end': 391, 'new_start': 399, 'new_end': 400, 'old_lines': ['               external_turn_heading=None):\n'], 'new_lines': ['               external_turn_heading=None,pre_zero_unload_event=None):\n']}, {'old_start': 413, 'old_end': 413, 'new_start': 422, 'new_end': 433, 'old_lines': [], 'new_lines': ['            if pre_zero_unload_event is not None:\n', '                from pre_zero_integral_unload import validate_event\n', '                validate_event(pre_zero_unload_event,clock_ns=clock)\n', "                if (mode_override!='brake' or not self.external_heading_gate or self.capture_stamp is not None\n", "                        or pre_zero_unload_event['goal_sha256']!=self.goal_sha):\n", "                    raise ValueError('Coordinated unload outside original owned brake')\n", "                old=getattr(self,'pre_zero_unload_event',None)\n", '                if old is not None and old!=pre_zero_unload_event:\n', '                    from pre_zero_integral_unload import successor\n', "                    if not successor(old,pre_zero_unload_event):raise ValueError('Outer unload pending event cannot renew')\n", '                self.pre_zero_unload_event=copy.deepcopy(pre_zero_unload_event)\n']}, {'old_start': 804, 'old_end': 805, 'new_start': 824, 'new_end': 825, 'old_lines': ["            mode=='drive', turn_only=mode in ('turn','brake'))\n"], 'new_lines': ["            mode=='drive', turn_only=mode in ('turn','brake'),pre_zero_unload=pre_zero_unload_event)\n"]}, {'old_start': 850, 'old_end': 850, 'new_start': 870, 'new_end': 873, 'old_lines': [], 'new_lines': ["        if pre_zero_unload_event is not None and mode=='brake':\n", "            self.row['active_turn_braking'].update(original_PI_antiwindup_and_limits=False,\n", '                original_gains_and_limits=True,new_yaw_integral_recurrence=True)\n']}], 'controller.py': [{'old_start': 1341, 'old_end': 1342, 'new_start': 1341, 'new_end': 1347, 'old_lines': ['        cmd,diagnostic=self.cascade.update(self.feedback,self.paired_imu,ack,ns,wall,guard_reason=guard,mode_override=mode,\n'], 'new_lines': ['        unload_event=None\n', "        if getattr(self.heading_gate,'integral_unload_config',None)is not None and mode=='brake' and guard is None:\n", '            unload_event=self.heading_gate.bind_integral_unload(self.run.name,self.handoff_scope_sha256,\n', '                self.cascade.path_sha,self.cascade.goal_sha,ns)\n', "            if self.heading_gate.handoff_failure:guard='active_turn_handoff_failure:'+self.heading_gate.handoff_failure\n", '        update_args=dict(guard_reason=guard,mode_override=mode,\n']}, {'old_start': 1343, 'old_end': 1343, 'new_start': 1348, 'new_end': 1350, 'old_lines': [], 'new_lines': ["        if unload_event is not None:update_args['pre_zero_unload_event']=unload_event\n", '        cmd,diagnostic=self.cascade.update(self.feedback,self.paired_imu,ack,ns,wall,**update_args)\n']}, {'old_start': 1469, 'old_end': 1469, 'new_start': 1476, 'new_end': 1477, 'old_lines': [], 'new_lines': ["        if getattr(gate,'integral_unload_config',None)is not None:gate.check_integral_unload_deadline(clock_ns)\n"]}, {'old_start': 1692, 'old_end': 1692, 'new_start': 1700, 'new_end': 1702, 'old_lines': [], 'new_lines': ["            if getattr(gate,'integral_unload_config',None)is not None and row['cascade']['mode']=='brake':\n", "                source['pre_zero_integral_unload']=copy.deepcopy(gate.integral_unload_event)\n"]}, {'old_start': 1752, 'old_end': 1752, 'new_start': 1762, 'new_end': 1767, 'old_lines': [], 'new_lines': ["            if record is not None and getattr(gate,'integral_unload_config',None)is not None:\n", "                pi=core.get('velocity_PI',{})\n", "                record['pre_zero_integral_unload']=copy.deepcopy(pi.get('pre_zero_integral_unload'))\n", "                record['yaw_integral_state']=pi.get('integral_state',[None]*3)[2]\n", "                record['integral_dt_s']=pi.get('integral_dt_s')\n"]}], 'student_actor_adapter.py': [{'old_start': 60, 'old_end': 61, 'new_start': 60, 'new_end': 61, 'old_lines': ['    def update(self, stamp, requested, measured, motion_epoch, authoritative_zero):\n'], 'new_lines': ['    def update(self, stamp, requested, measured, motion_epoch, authoritative_zero, pre_zero_unload=None):\n']}, {'old_start': 90, 'old_end': 90, 'new_start': 90, 'new_end': 96, 'old_lines': [], 'new_lines': ['        unload_before=None\n', '        if pre_zero_unload is not None and not holding:\n', "            helper=pre_zero_unload['helper'];authorization=pre_zero_unload['authorization']\n", "            helper.validate_event(authorization['event'],clock_ns=authorization['source_control_clock_ns'])\n", '            unload_before=float(self.integral[2])\n', '            candidate[2]=helper.toward_zero(unload_before,.04,dt)\n']}, {'old_start': 97, 'old_end': 97, 'new_start': 103, 'new_end': 107, 'old_lines': [], 'new_lines': ['        if unload_before is not None:\n', '            # Release an existing state even if accumulation would be blocked.\n', '            # This is the explicit new recurrence, not an unchanged PID claim.\n', '            candidate[2]=helper.toward_zero(unload_before,.04,dt)\n']}, {'old_start': 115, 'old_end': 115, 'new_start': 125, 'new_end': 129, 'old_lines': [], 'new_lines': ['        if unload_before is not None:\n', "            diagnostic[helper.KEY]=helper.receipt(authorization['event'],'student',unload_before,\n", "                self.integral[2],dt,pre_zero_unload['input_clock_ns'],authorization['path_sha256'],\n", "                authorization['goal_sha256'],pre_zero_unload['input_frame_serial'],authorization['source_control_clock_ns'])\n"]}, {'old_start': 143, 'old_end': 143, 'new_start': 157, 'new_end': 179, 'old_lines': [], 'new_lines': ['        self.unload_helper=None;self.integral_unload=None\n', "        option=scope['profile'].get('r85_pre_zero_integral_unload')\n", "        if option is not None and not (option=={'enabled':False} and type(option.get('enabled'))is bool):\n", '            # Verify the real archive mapping BEFORE executing adjacent helper.\n', '            import importlib.util\n', "            helper_path=Path(__file__).with_name('pre_zero_integral_unload.py')\n", "            expected_snapshot=Path(binding['student_adapter_snapshot']).with_name(helper_path.name)\n", "            helper_origin=Path(binding['student_adapter_origin']).with_name(helper_path.name)\n", "            expected_sha=scope['references'].get(str(helper_origin))\n", "            scope_bytes=(self.run/'navigation_scope.json').read_bytes()\n", '            if (json.loads(scope_bytes)!=scope or helper_path.resolve()!=expected_snapshot.resolve()\n', '                    or not isinstance(expected_sha,str)or sha(helper_path)!=expected_sha):\n', "                raise ValueError('Actual loaded unload helper snapshot/source binding differs')\n", "            spec=importlib.util.spec_from_file_location('actual_snapshot_pre_zero_integral_unload',helper_path)\n", '            helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)\n', "            if helper.configuration(scope['profile']) is not None:\n", '                self.unload_helper=helper\n', '                self.integral_unload=helper.NativeUnload(scope,hashlib.sha256(scope_bytes).hexdigest(),self.run.name)\n', "                self.integral_unload_identity=dict(schema='r85_coordinated_unload_deployment_extension/v1',\n", '                    helper_path=str(helper_path),helper_sha256=sha(helper_path),\n', "                    scope_sha256=hashlib.sha256(scope_bytes).hexdigest(),configuration=helper.configuration(scope['profile']),\n", '                    base_contract_only_sha256=sha(self.contract_path),base_contract_does_not_prove_new_recurrence=True)\n']}, {'old_start': 167, 'old_end': 169, 'new_start': 203, 'new_end': 215, 'old_lines': ['        corrected,_=self.pid.update(float(self.native_state[0]-.005),request,\n', '                                    self.native_state[[8,9,13]],epoch,not bool(np.any(request)))\n'], 'new_lines': ['        unload=None\n', '        if self.integral_unload is not None:\n', '            authorization=self.integral_unload.authorize(evidence,request,input_clock_ns)\n', '            if authorization is not None:\n', '                unload=dict(helper=self.unload_helper,authorization=authorization,input_clock_ns=input_clock_ns,\n', '                    input_frame_serial=self.frame_serial)\n', '        if unload is None:\n', '            corrected,_=self.pid.update(float(self.native_state[0]-.005),request,\n', '                                        self.native_state[[8,9,13]],epoch,not bool(np.any(request)))\n', '        else:\n', '            corrected,_=self.pid.update(float(self.native_state[0]-.005),request,\n', '                self.native_state[[8,9,13]],epoch,not bool(np.any(request)),pre_zero_unload=unload)\n']}, {'old_start': 181, 'old_end': 181, 'new_start': 227, 'new_end': 231, 'old_lines': [], 'new_lines': ['        if self.unload_helper is not None and not np.any(request):\n', '            self.pid.last_diagnostic[self.unload_helper.KEY]=self.unload_helper.zero_observation(\n', '                self.integral_unload.scope_sha,input_clock_ns,self.frame_serial,\n', "                self.pid.last_diagnostic['dt_s'],self.pid.integral[2])\n"]}, {'old_start': 198, 'old_end': 198, 'new_start': 248, 'new_end': 251, 'old_lines': [], 'new_lines': ['        if self.unload_helper is not None and self.unload_helper.KEY in self.pid.last_diagnostic:\n', '            import copy\n', '            self.input_receipt[self.unload_helper.KEY]=copy.deepcopy(self.pid.last_diagnostic[self.unload_helper.KEY])\n']}, {'old_start': 262, 'old_end': 263, 'new_start': 315, 'new_end': 316, 'old_lines': ["        return dict(actor_kind='student',controller_kind='student',checkpoint_sha256=CHECKPOINT_SHA,\n"], 'new_lines': ["        out=dict(actor_kind='student',controller_kind='student',checkpoint_sha256=CHECKPOINT_SHA,\n"]}, {'old_start': 273, 'old_end': 273, 'new_start': 326, 'new_end': 328, 'old_lines': [], 'new_lines': ["        if self.integral_unload is not None:out['pre_zero_integral_unload_extension']=dict(self.integral_unload_identity)\n", '        return out\n']}], 'student_lower_loop_ack.py': [{'old_start': 130, 'old_end': 130, 'new_start': 130, 'new_end': 141, 'old_lines': [], 'new_lines': ["    if 'pre_zero_integral_unload'in pid or 'pre_zero_integral_unload'in receipt:\n", '        from pre_zero_integral_unload import KEY,validate_native_extension\n', '        extension=pid.get(KEY)\n', '        validate_native_extension(extension,scope,stamp,frame)\n', "        observed=extension.get('kind')=='original_exact_zero_I_observation'\n", "        actual_I=extension['actual_yaw_integral']if observed else extension['yaw_integral_after']\n", "        if (observed and (not zero or vector(pid['pid_integral'])[2]!=0.)\n", "                or receipt.get(KEY)!=extension or vector(pid['pid_integral'])[2]!=actual_I\n", "                or extension['dt_s']!=pid['dt_s']):\n", "            raise ValueError('Unload receipt does not match actual PID and Student frame')\n", '        annotation[KEY]=copy.deepcopy(extension)\n']}, {'old_start': 195, 'old_end': 195, 'new_start': 206, 'new_end': 211, 'old_lines': [], 'new_lines': ["    if 'pre_zero_integral_unload'in note or 'pre_zero_integral_unload'in receipt:\n", '        from pre_zero_integral_unload import KEY,validate_native_extension\n', '        validate_native_extension(note.get(KEY),scope,stamp,frame)\n', "        if (note.get(KEY)!=receipt.get(KEY)or note[KEY].get('kind')=='original_exact_zero_I_observation'and not zero):\n", "            raise ValueError('Unload ACK ledger and actual input receipt differ')\n"]}, {'old_start': 203, 'old_end': 203, 'new_start': 219, 'new_end': 220, 'old_lines': [], 'new_lines': ["    if 'pre_zero_integral_unload'in note:diagnostic['pre_zero_integral_unload']=copy.deepcopy(note['pre_zero_integral_unload'])\n"]}], 'transition_gate.py': [{'old_start': 41, 'old_end': 41, 'new_start': 41, 'new_end': 45, 'old_lines': [], 'new_lines': ['        self.integral_unload_config=None;self.integral_unload_event=None;self.integral_unload_previous_zero=None;self.integral_unload_failed_event_id=None\n', "        if profile.get('r85_pre_zero_integral_unload')is not None:\n", '            from pre_zero_integral_unload import configuration\n', '            self.integral_unload_config=configuration(profile)\n']}, {'old_start': 104, 'old_end': 104, 'new_start': 108, 'new_end': 110, 'old_lines': [], 'new_lines': ['        if self.integral_unload_config is not None and self.integral_unload_event is not None:\n', "            self.integral_unload_failed_event_id=self.integral_unload_failed_event_id or self.integral_unload_event['event_id']\n"]}, {'old_start': 109, 'old_end': 109, 'new_start': 115, 'new_end': 164, 'old_lines': [], 'new_lines': ['    def bind_integral_unload(self,run_id,scope,path_sha256,goal_sha256,clock_ns):\n', '        if self.integral_unload_config is None:return None\n', '        if not self.handoff_active or not self.controlled_braking:return None\n', '        if self.integral_unload_failed_event_id is not None:\n', "            self.fail_handoff('pre_zero_integral_unload_previous_failure',clock_ns/1e9);return None\n", '        from pre_zero_integral_unload import make_event\n', '        old=self.integral_unload_event\n', '        # Same owned pending entry always keeps its original path identity,\n', '        # expiry, and hash. A legal path refresh only restarts quiet samples.\n', "        initial_path=(old['initial_path_sha256']if old is not None\n", "            and old['brake_entry_clock_ns']==round(self.handoff_event['brake_entry_control_s']*1e9)else path_sha256)\n", '        event=make_event(run_id,scope,self.handoff_event,initial_path,goal_sha256,\n', '            self.zero_handoff_count,self.integral_unload_previous_zero)\n', '        if old is None:self.integral_unload_event=event\n', '        elif event!=old:\n', '            from pre_zero_integral_unload import successor\n', '            if successor(old,event):self.integral_unload_event=event\n', '            else:\n', "                self.fail_handoff('pre_zero_integral_unload_binding_changed',clock_ns/1e9);return None\n", '        self.check_integral_unload_deadline(clock_ns)\n', '        return copy.deepcopy(self.integral_unload_event) if self.handoff_active else None\n', '    def check_integral_unload_deadline(self,clock_ns):\n', '        event=self.integral_unload_event\n', '        if (event is not None and self.handoff_active and self.controlled_braking\n', "                and clock_ns>=event['expires_clock_ns']):\n", "            self.fail_handoff('pre_zero_integral_unload_timeout',clock_ns/1e9)\n", '    def _unloaded_actual_pair(self,publication,ack):\n', '        if self.integral_unload_config is None:return True\n', '        try:\n', '            from pre_zero_integral_unload import validate_receipt\n', '            e=self.integral_unload_event\n', '            if e is None:return False\n', "            teacher=validate_receipt(publication.get('pre_zero_integral_unload'),event=e,side='teacher',scope=publication['scope'],clock_ns=publication['control_clock_ns'])\n", "            native=ack.get('pre_zero_integral_unload')\n", "            if isinstance(native,dict)and native.get('kind')=='original_exact_zero_I_observation':\n", '                from pre_zero_integral_unload import validate_native_extension\n', "                validate_native_extension(native,ack['source_scope_sha256'],ack['input_clock_ns'],ack['input_frame_serial'])\n", '                # This observes actual original reset state; it does not itself\n', '                # authorize an event. Current published Teacher event and all\n', '                # original causal/quiet source predicates still bind this pair.\n', "                return bool(teacher['yaw_integral_after']==0. and ack['exact_zero']\n", "                    and ack['input_clock_ns']>e['brake_entry_clock_ns']\n", "                    and teacher['path_sha256']==publication['path_sha256']\n", "                    and teacher['goal_sha256']==publication['goal_sha256'])\n", "            native=validate_receipt(native,event=e,side='student',scope=ack['source_scope_sha256'],clock_ns=ack['input_clock_ns'],frame=ack['input_frame_serial'])\n", "            return bool(teacher['yaw_integral_after']==0. and native['yaw_integral_after']==0.\n", "                and teacher['path_sha256']==publication['path_sha256']==native['path_sha256']\n", "                and teacher['goal_sha256']==publication['goal_sha256']==native['goal_sha256'])\n", '        except (ValueError,KeyError,TypeError):return False\n']}, {'old_start': 150, 'old_end': 150, 'new_start': 205, 'new_end': 211, 'old_lines': [], 'new_lines': ['            if self.integral_unload_config is not None:\n', '                from pre_zero_integral_unload import validate_receipt\n', "                proof=validate_receipt(record.get('pre_zero_integral_unload'),event=self.integral_unload_event,\n", "                    side='teacher',scope=record['scope'],clock_ns=record['control_clock_ns'])\n", "                if (proof['yaw_integral_after']!=0. or record.get('yaw_integral_state')!=proof['yaw_integral_after']\n", "                        or record.get('integral_dt_s')!=proof['dt_s']):raise ValueError('actual_published_Teacher_I_not_zero')\n"]}, {'old_start': 180, 'old_end': 180, 'new_start': 241, 'new_end': 242, 'old_lines': [], 'new_lines': ["            if not self._unloaded_actual_pair(p,ack):raise ValueError('actual_same_event_integrals_not_zero')\n"]}, {'old_start': 210, 'old_end': 210, 'new_start': 272, 'new_end': 274, 'old_lines': [], 'new_lines': ['        if not self._unloaded_actual_pair(p,ack):\n', "            self._clear_quiet('actual_same_event_integrals_not_zero');return False\n"]}, {'old_start': 276, 'old_end': 276, 'new_start': 340, 'new_end': 343, 'old_lines': [], 'new_lines': ['            if self.integral_unload_config is not None:\n', "                extension=receipt.get('pre_zero_integral_unload')\n", "                if extension is not None:self.zero_ack_candidate['pre_zero_integral_unload']=copy.deepcopy(extension)\n"]}, {'old_start': 475, 'old_end': 475, 'new_start': 542, 'new_end': 545, 'old_lines': [], 'new_lines': ['                if self.integral_unload_config is not None:\n', "                    self.integral_unload_previous_zero=dict(event_id=self.integral_unload_event['event_id'],\n", '                        zero_control_clock_ns=round(sim_time*1e9),zero_handoff_count=self.zero_handoff_count)\n']}, {'old_start': 528, 'old_end': 528, 'new_start': 598, 'new_end': 604, 'old_lines': [], 'new_lines': ['        if self.integral_unload_config is not None:\n', "            result['pre_zero_integral_unload']=dict(enabled=True,\n", '                configuration=copy.deepcopy(self.integral_unload_config),event=copy.deepcopy(self.integral_unload_event),\n', '                previous_owned_zero_identity=copy.deepcopy(self.integral_unload_previous_zero),\n', '                failed_event_id=self.integral_unload_failed_event_id,\n', '                new_deployment_math=True,stop_PASS=False,event_hash_is_publication_proof=False)\n']}]}
UNLOAD_SCANNER_PATCHES=[('def _heading_control_adapter(session,base,renewal=None,repair=None,quiet=None):', 'def _heading_control_adapter(session,base,renewal=None,repair=None,quiet=None,unload=None):'), ("        if quiet is not None and filename in ('controller.py','transition_gate.py'):live_control_sha=quiet['sources'][filename]['after_sha256']", "        if quiet is not None and filename in ('controller.py','transition_gate.py'):live_control_sha=quiet['sources'][filename]['after_sha256']\n        if unload is not None and filename in ('controller.py','transition_gate.py','cascade_core.py'):live_control_sha=unload['sources'][filename]['after_sha256']"), ("def _control_adapter(session,base):\n    if 'pre_zero_quiet_proof' in session or 'pre_zero_quiet_proof_sha256' in session:", "def _control_adapter(session,base):\n    if 'pre_zero_integral_unload_proof' in session or 'pre_zero_integral_unload_proof_sha256' in session:\n        from pre_zero_integral_unload_adapter import prepare_unload,attach_unload\n        from schedule_renewal_adapter import attach_renewal\n        unload,historical,live_schedule,live_repair=prepare_unload(session,base)\n        return attach_unload(attach_renewal(_heading_control_adapter(session,base,live_schedule,live_repair,None,unload),historical),unload)\n    if 'pre_zero_quiet_proof' in session or 'pre_zero_quiet_proof_sha256' in session:"), ("        resolved_quiet(resolved,adapter['pre_zero_quiet'])\n    return resolved", "        resolved_quiet(resolved,adapter['pre_zero_quiet'])\n    if 'pre_zero_integral_unload' in adapter:\n        from pre_zero_integral_unload_adapter import resolved_unload\n        resolved_unload(resolved,adapter['pre_zero_integral_unload'])\n    return resolved")]
HISTORICAL_READER_PATCHES=[("require(Path(__file__).resolve()==ROOT/'pre_zero_quiet_adapter.py','active quiet helper location')", "require(Path(__file__).resolve()==UNLOAD_HELPER,'active unload identity helper location')"), ("    require(Path(__file__).resolve()==QUIET_HELPER,'actual quiet helper location')", "    require(Path(__file__).resolve()==UNLOAD_HELPER,'actual unload helper location')\n    require(Path(historical_quiet.__file__).resolve()==QUIET_HELPER and digest(QUIET_HELPER.read_bytes())==HISTORICAL_QUIET_HELPER_SHA,'real unchanged historical R15 reader')"), ("    require(digest(expected)==row['after_sha256'] and (ROOT/name).read_bytes()==expected,'quiet truthful actual current identity: '+name)", "    require(digest(expected)==row['after_sha256'] and verify_historical_actual(name,expected),'quiet historical identity and explicit actual unload identity: '+name)"), ("require(snapshot.read_bytes()==r14 and quiet_scanner_after_bytes(r14)==actual_bytes,'explicit R13-R14-R15 scanner chain')", "require(snapshot.read_bytes()==r14 and verify_historical_actual('scan_workspace.py',quiet_scanner_after_bytes(r14)) and actual_bytes==(ROOT/'scan_workspace.py').read_bytes(),'explicit R13-R14-R15-R16 scanner chain')"), ("        else:evidence.append(file_ref(dict(path=path,sha256=sha)))\n    profile=data['profile'];", "        elif path==str(ROOT/'cascade_core.py'):\n            verify_historical_actual('cascade_core.py',(UNLOAD_STAGE/'beforeimage/cascade_core.py').read_bytes())\n            require(SOURCE_BEFORE_UNLOAD['cascade_core.py']==sha,'historical core exact original identity')\n            evidence.extend([Path(path),UNLOAD_STAGE/'beforeimage/cascade_core.py'])\n        else:evidence.append(file_ref(dict(path=path,sha256=sha)))\n    profile=data['profile'];"), ("    current_path=SOURCE_REPO/'SOURCE_BINDINGS.json';current=historical_repair._binding_map(json.loads(current_path.read_text()))", "    current_path=UNLOAD_STAGE/'beforeimage/SOURCE_BINDINGS.json';current=historical_repair._binding_map(json.loads(current_path.read_text()))\n    require(digest(current_path.read_bytes())==UNLOAD_BINDINGS_SHA,'retained exact actual R15 258 binding snapshot')"), ("        row=current[path];active=file_ref(dict(path=path,sha256=sha))\n        require(row['sha256']==sha and row.get('bytes')==active.stat().st_size,'truthful full source binding bytes/hash')\n        tracked=SOURCE_REPO/row['tracked'];require(tracked.is_file()and not tracked.is_symlink()and digest(tracked.read_bytes())==sha,'tracked evidence matches actual source')\n        evidence.append(active)", "        row=current[path]\n        name=Path(path).name\n        if path==str(ROOT/name) and name in SOURCE_BEFORE_UNLOAD:\n            active=UNLOAD_STAGE/'beforeimage'/name\n            require(active.is_file()and not active.is_symlink()and digest(active.read_bytes())==sha,'historical R15 source beforeimage')\n            verify_historical_actual(name,active.read_bytes())\n        else:\n            active=file_ref(dict(path=path,sha256=sha))\n            tracked=SOURCE_REPO/row['tracked'];require(tracked.is_file()and not tracked.is_symlink()and digest(tracked.read_bytes())==sha,'unchanged tracked evidence matches actual source')\n        require(row['sha256']==sha and row.get('bytes')==active.stat().st_size,'historical binding real bytes/hash')\n        evidence.append(active)")]
UNLOAD_CONFIG={'enabled': True, 'kind': 'coordinated_yaw_toward_zero_v1', 'teacher_rate_integral_per_s': 0.1, 'student_rate_integral_per_s': 0.04, 'maneuver_timeout_s': 10.0}
UNLOAD_OLD_PROFILE_SHA='8ecd452a40f651b0b364fcc71f8cf73cdb49c45cda54b3ae55f92bb875fdd7f8'
UNLOAD_REFS={'mode_selection': {'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1000/R16_CONTROL_ONLY_COORDINATED_UNLOAD_MODE_SELECTION.json', 'sha256': '3f6ff7c37dcea0b938e4d082b2e3ef445446ecb8eeaad0f16dc60c0418e7b091'}, 'event_scope_clarification': {'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1000/R16_PER_OWNED_EVENT_SCOPE_CLARIFICATION.json', 'sha256': 'dbc7e5ea8d7a15fa262089f5c90bf537e7f4e2cf1eb57dffecdbaf5b6da18ace'}, 'baseline_install': {'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1000/round_15_pre_zero_quiet/install_tools/INSTALL_RECEIPT.json', 'sha256': '0f508d54a51bd6c17bd3efc6ac23a63aec18dccd3ab98630f8c24aa685b36d8c'}, 'behavior_manifest': {'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1000/pre_zero_integral_unload_stage/FINAL_BEHAVIOR_CANDIDATE_MANIFEST_v2.json', 'sha256': '3a7ae336499fdf55b36bd0d0806502f2fc308df1fd2c012c9113acd318dc727b'}}
UNLOAD_MATH_DECLARATION={'kind': 'coordinated_yaw_toward_zero_v1', 'teacher_yaw_integral_recurrence': 'sign(I)*max(0,abs(I)-0.1*dt)', 'student_yaw_integral_recurrence': 'sign(I)*max(0,abs(I)-0.04*dt)', 'new_integral_math': True, 'base_student_contract_sha256': '058db968f5ea471ffcc7b8bd132ab453f36c2a4911a1036d85e0db628a1f3b78', 'base_contract_is_complete_active_math': False, 'original_gains_limits_slew_reset_45_weights_preserved': True, 'extension_receipt_schema': 'r85_pre_zero_yaw_integral_update/v1', 'extension_receipt_in_canonical_causal_ACK_ledger': True, 'per_owned_event': True}
UNLOAD_TARGETS=['cascade_core.py', 'controller.py', 'pre_zero_integral_unload.py', 'student_actor_adapter.py', 'student_lower_loop_ack.py', 'transition_gate.py', 'scan_workspace.py', 'pre_zero_integral_unload_adapter.py', 'R85_PRE_ZERO_INTEGRAL_UNLOAD_CONTROL_ADAPTER_CONTRACT.json', 'profiles/r85_f1_f3_pre_zero_integral_unload_r16_20261016.json']
UNLOAD_PROOF_KEYS={'behavior_helper', 'physical_thresholds_changed', 'sources', 'event_scope_clarification', 'baseline_source_bindings', 'baseline_commit', 'bridge_helper', 'behavior_manifest', 'baseline_profile', 'configuration', 'expected_bound_source_count_after', 'simulation_only', 'navigation_acceptance_established', 'SCAN_contract_bytes_changed', 'baseline_install', 'source_or_feedback_guards_changed', 'runtime_acceptance_inherited', 'expected_bound_source_count_before', 'exact_targets', 'mode_selection', 'math_extension', 'schema', 'historical_R15_helper'}

def unload_after_bytes(name,raw):
    require(name in SOURCE_BEFORE_UNLOAD and digest(raw)==SOURCE_BEFORE_UNLOAD[name],'unload actual R15 beforeimage: '+name)
    if name=='scan_workspace.py':
        text=raw.decode()
        for old,new in UNLOAD_SCANNER_PATCHES:
            require(text.count(old)==1,'unload scanner exact patch occurrence');text=text.replace(old,new)
        return text.encode()
    lines=raw.decode().splitlines(keepends=True)
    for row in reversed(UNLOAD_DELTAS[name]):
        i,j=row['old_start'],row['old_end'];require(lines[i:j]==row['old_lines'],'unload reviewed exact patch context');lines[i:j]=row['new_lines']
    return ''.join(lines).encode()

def verify_historical_actual(name,historical_bytes):
    require(digest(historical_bytes)==SOURCE_BEFORE_UNLOAD[name],'explicit historical source identity: '+name)
    raw=unload_after_bytes(name,historical_bytes)
    require(digest(raw)==SOURCE_AFTER_UNLOAD[name] and _verify_1400_actual(name,raw),'truthful historical R16 then current14UTC source identity: '+name)
    return True

def _verify_unload_closure(data,proof_sha,profile_sha):
    snapshot=file_ref(data['baseline_source_bindings'],UNLOAD_STAGE/'beforeimage/SOURCE_BINDINGS.json',UNLOAD_BINDINGS_SHA)
    old=historical_repair._binding_map(json.loads(snapshot.read_text()));require(len(old)==258,'retained actual R15 full258')
    expected={p:r['sha256']for p,r in old.items()}
    for name,row in data['sources'].items():
        require(expected[str(ROOT/name)]==row['before_sha256'],'new source baseline match');expected[str(ROOT/name)]=row['after_sha256']
    added={str(ROOT/'pre_zero_integral_unload.py'):data['behavior_helper']['sha256'],str(UNLOAD_HELPER):data['bridge_helper']['sha256'],str(UNLOAD_PROOF):proof_sha,str(ROOT/UNLOAD_PROFILE):profile_sha}
    require(not(set(added)&set(expected)),'four genuinely new bound paths');expected.update(added)
    expected,renewed_evidence=_extend_1400_closure(expected)
    p=SOURCE_REPO/'SOURCE_BINDINGS.json';current=historical_repair._binding_map(json.loads(p.read_text()))
    require(len(current)==265 and set(current)==set(expected),'exact actual265 scheduling source closure')
    evidence=[snapshot,p]+renewed_evidence
    for path,sha in expected.items():
        row=current[path];active=file_ref(dict(path=path,sha256=sha));require(row['sha256']==sha and row['bytes']==active.stat().st_size,'actual full262 byte binding')
        tracked=SOURCE_REPO/row['tracked'];require(tracked.is_file()and not tracked.is_symlink()and digest(tracked.read_bytes())==sha,'actual262 tracked bytes')
        evidence.append(active)
    return evidence

def load_unload(session,base):
    require(Path(__file__).resolve()==RENEW_HELPER,'actual14UTC historical R16 identity bridge')
    r=dict(path=session.get('pre_zero_integral_unload_proof'),sha256=session.get('pre_zero_integral_unload_proof_sha256'))
    p=file_ref(r,UNLOAD_PROOF);data=json.loads(p.read_text())
    require(set(data)==UNLOAD_PROOF_KEYS and data['schema']=='r85_pre_zero_integral_unload_source_adapter/v1'and data['baseline_commit']==UNLOAD_BASELINE
        and data['expected_bound_source_count_before']==258 and data['expected_bound_source_count_after']==262 and data['exact_targets']==UNLOAD_TARGETS,'exact unload proof schema and scope')
    require(data['configuration']==UNLOAD_CONFIG and data['math_extension']==UNLOAD_MATH_DECLARATION,'explicit new integral math identity')
    require(data['simulation_only']is True and all(data[k]is False for k in ('runtime_acceptance_inherited','navigation_acceptance_established','source_or_feedback_guards_changed','physical_thresholds_changed','SCAN_contract_bytes_changed')),'no inherited runtime or physical acceptance')
    evidence=[p]
    for key,expected in UNLOAD_REFS.items():
        path=file_ref(data[key],Path(expected['path']),expected['sha256']);evidence.append(path)
    require(data['historical_R15_helper']==dict(path=str(QUIET_HELPER),sha256=HISTORICAL_QUIET_HELPER_SHA),'old R15 reader retained unchanged')
    evidence.append(file_ref(data['historical_R15_helper']))
    require(set(data['sources'])==set(SOURCE_BEFORE_UNLOAD),'exact six existing source extension set')
    for name,row in data['sources'].items():
        require(set(row)=={'path','before_sha256','after_sha256','beforeimage','delta_sha256'}and row['path']==str(ROOT/name)
            and row['before_sha256']==SOURCE_BEFORE_UNLOAD[name]and row['after_sha256']==SOURCE_AFTER_UNLOAD[name],'exact actual source extension declaration')
        before=file_ref(row['beforeimage'],UNLOAD_STAGE/'beforeimage'/name,row['before_sha256'])
        table=UNLOAD_SCANNER_PATCHES if name=='scan_workspace.py'else UNLOAD_DELTAS[name]
        require(row['delta_sha256']==digest(json.dumps(table,sort_keys=True,separators=(',',':')).encode()),'reviewed extension exact delta identity')
        verify_historical_actual(name,before.read_bytes());evidence.extend([before,ROOT/name])
    require(data['behavior_helper']==dict(path=str(ROOT/'pre_zero_integral_unload.py'),sha256=SOURCE_AFTER_UNLOAD['pre_zero_integral_unload.py']),'actual adjacent snapshot behavior helper identity')
    evidence.extend([file_ref(data['behavior_helper']),file_ref(data['bridge_helper'],UNLOAD_HELPER)])
    before=file_ref(data['baseline_profile'],ROOT/UNLOAD_OLD_PROFILE,UNLOAD_OLD_PROFILE_SHA)
    expected=json.loads(before.read_text());expected['r85_pre_zero_integral_unload']=copy.deepcopy(UNLOAD_CONFIG)
    expected['r85_session']['pre_zero_integral_unload_proof']=str(UNLOAD_PROOF);expected['r85_session']['pre_zero_integral_unload_proof_sha256']=r['sha256']
    actual_path=ROOT/UNLOAD_PROFILE;raw=actual_path.read_bytes();actual=json.loads(raw)
    require(actual==expected and session==actual['r85_session'],'exact profile oldR15 plus config and two identity keys only')
    evidence.extend([before,actual_path]);evidence.extend(_verify_unload_closure(data,r['sha256'],digest(raw)))
    data['proof']=r;data['evidence']=sorted(set(evidence));return data

def prepare_unload(session,base):
    data=load_unload(session,base);oldsession=copy.deepcopy(session)
    for key in ('pre_zero_integral_unload_proof','pre_zero_integral_unload_proof_sha256'):del oldsession[key]
    quiet,historical,live_schedule,live_repair=prepare_quiet(oldsession,base)
    live_schedule['validator']['after_sha256']=data['sources']['scan_workspace.py']['after_sha256']
    live_repair['controller']['after_sha256']=data['sources']['controller.py']['after_sha256']
    live_repair['validator']['after_sha256']=data['sources']['scan_workspace.py']['after_sha256']
    data['historical_quiet_proof']=copy.deepcopy(quiet['proof']);data['evidence']=sorted(set(data['evidence']+quiet['evidence']))
    return data,historical,live_schedule,live_repair

def attach_unload(adapter,data):
    adapter['pre_zero_integral_unload']=data
    for row in list(data['sources'].values())+[data['behavior_helper'],data['bridge_helper'],data['historical_R15_helper']]:
        adapter['actual'][row['path']]=row.get('after_sha256',row.get('sha256'))
    adapter['evidence']=sorted(set(adapter['evidence']+data['evidence']));return adapter

def resolved_unload(resolved,data):
    target=resolved['resolved_adapter'];target['schema']='r85_pre_zero_integral_unload_resolved_bindings/v1'
    for name,row in data['sources'].items():
        require(target['actual_control_sources_sha256'][row['path']]==row['after_sha256'],'resolved actual R16 source identity')
        if name in ('controller.py','scan_workspace.py'):require(target['overrides'][row['path']]['after_sha256']==row['after_sha256'],'base override truthful actualSHA')
    target['schedule_renewal']['source_identity_status']='historical_pre_R14_R15_R16'
    target['schedule_renewal']['actual_identity_superseded_by']=copy.deepcopy(data['proof'])
    target['historical_pre_zero_quiet']=dict(proof=copy.deepcopy(data['historical_quiet_proof']),source_identity_status='historical_pre_integral_unload',actual_identity_superseded_by=copy.deepcopy(data['proof']))
    target['pre_zero_integral_unload']=dict(proof=copy.deepcopy(data['proof']),baseline_commit=UNLOAD_BASELINE,mode_selection=copy.deepcopy(data['mode_selection']),event_scope_clarification=copy.deepcopy(data['event_scope_clarification']),
        math_extension=copy.deepcopy(data['math_extension']),current_sources={r['path']:r['after_sha256']for r in data['sources'].values()},behavior_helper=copy.deepcopy(data['behavior_helper']),
        original_base_three_override_set_unchanged=True,original_required_runtime_source_paths_unchanged=True,old_base058_bytes_preserved=True,old_complete_fixed_PID_math_claimed=False,runtime_acceptance_inherited=False,navigation_acceptance_established=False)

# Explicit combined14UTC + neutral-rate capture layer. Earlier R16 proofs and
# source identities above remain historical, verified through exact byte deltas.
RENEW_WORK=WORK.parent/'window_20261016_until1400'
RENEW_STAGE=RENEW_WORK/'neutral_rate_brake_candidate/combined_identity'
RENEW_HELPER=ROOT/'neutral_rate_brake_adapter.py'
RENEW_PROOF=ROOT/'R85_NEUTRAL_RATE_BRAKE_CAPTURE_CONTROL_ADAPTER_CONTRACT.json'
RENEW_PROFILE='profiles/r85_f1_f3_neutral_rate_brake_capture_r34_20261016.json'
RENEW_BASELINE='fd176b813f0136087d407719b127d509a9194bea'
RENEW_AUTH={'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/RENEWED_USER_AUTHORIZATION.json', 'sha256': 'ef3e13d4bc7e3d9692407e1c04eb5f36e2724a99aad50fb720c9e15bff1ea6e7'}
RENEW_OLD_HELPER_SHA='40376b484097910734da40916409bee661471e65d99546a30debd50bb5fd84c9'
RENEW_OLD_PROFILE_SHA='6c78b6405007e68d3b4c0b27b69f196ee57af81d3384601dbc27d1da71f4facc'
RENEW_BINDINGS_SHA='7db5b1934fd02c29c867cfe7e4ba440246581815d1f77daebd1d4ca37b6f75b6'
RENEW_GUARD_BEFORE='e3d65e45a704b09a033809dd3d8bcb891129f825f2a8c13239db6d1c00679210'
RENEW_SCANNER_BEFORE='bf86b3ca9bfbd24cb7f66adbfb0d08abdd27eba4325efa23dabcbc1607a42551'
RENEW_SCANNER_AFTER='3e2c703539ea9f932becbd1ae83f02a22ce8bb7f279a5d84813921607de1c65c'
RENEW_HELPER_DELTA_SHA='1110095d1390367b07f6d5e43a0bf358a228072700c59ba51bffdd03ec0af42f'
RENEW_SCHEDULE=dict(deadline_utc='2026-10-16T14:00:00+00:00',
    basis='explicit_user_continue_until_20261016_2200_UTCplus08',
    finish_reserve_s=300,outer_supervisor_wall_cap_s=2700,
    original_runner_wall_cap_s=2400,original_native_cap_s=770,
    original_preparation_reserve_s=60,admission_remaining_seconds_exclusive=3060)
RENEW_GUARD_SUBSTITUTIONS=[("AUTHORIZED_USER_DEADLINE_UTC = '2026-10-16T10:00:00+00:00'", "AUTHORIZED_USER_DEADLINE_UTC = '2026-10-16T14:00:00+00:00'"), ('explicit_user_continue_until_20261016_1800_UTCplus08', 'explicit_user_continue_until_20261016_2200_UTCplus08')]
RENEW_SCANNER_PATCHES=[('def _control_adapter(session,base):\n', "def _control_adapter(session,base):\n    if 'neutral_rate_brake_capture_proof' in session or 'neutral_rate_brake_capture_proof_sha256' in session:\n        from neutral_rate_brake_adapter import prepare_neutral_rate_brake_capture,attach_unload,attach_neutral_rate_brake_capture\n        from schedule_renewal_adapter import attach_renewal\n        renewed,unload,historical,live_schedule,live_repair=prepare_neutral_rate_brake_capture(session,base)\n        adapter=attach_unload(attach_renewal(_heading_control_adapter(session,base,live_schedule,live_repair,None,unload),historical),unload)\n        return attach_neutral_rate_brake_capture(adapter,renewed)\n"), ("    if 'pre_zero_integral_unload' in adapter:\n        from pre_zero_integral_unload_adapter import resolved_unload\n        resolved_unload(resolved,adapter['pre_zero_integral_unload'])", "    if 'neutral_rate_brake_capture' in adapter:\n        from neutral_rate_brake_adapter import resolved_neutral_rate_brake_capture\n        resolved_neutral_rate_brake_capture(resolved,adapter)\n    elif 'pre_zero_integral_unload' in adapter:\n        from pre_zero_integral_unload_adapter import resolved_unload\n        resolved_unload(resolved,adapter['pre_zero_integral_unload'])")]
NEUTRAL_BEFORE={'cascade_core.py': '8f9a7276c15c86d7c1de13e619d59f1f27861fd12518dc5b454805148fc96bd8', 'transition_gate.py': '011041952eea773db281be8716f519f384bcfca9a8d6998073e669fe009a3a71', 'controller.py': 'eb54d8b7c15df077de83724e527f51578aa40c361abbbd2ef0fc334fbabcce54'}
NEUTRAL_AFTER={'cascade_core.py': '71d391b65e8e710a36eafa1456f367b21e405fa3bb42d6673dde5a4f79f73ad2', 'transition_gate.py': '94a611780841dfe302d2f266286d11619084286af3aaa14ad97c1c3242b01d14', 'controller.py': 'bd12cffb11285aaa0b99c17b061bdf714d9b912c1707c512fbbd071c365cb275'}
NEUTRAL_DELTAS={'cascade_core.py': [{'old_start': 22, 'old_end': 22, 'new_start': 22, 'new_end': 48, 'old_lines': [], 'new_lines': ['\n', '\n', 'def neutral_rate_brake_capture_configuration(profile):\n', '    """Explicit profile extension; disabled/absent preserves the original cascade."""\n', '    if not isinstance(profile,dict):\n', "        raise ValueError('Neutral-rate brake capture requires the actual profile')\n", "    if 'r85_neutral_rate_brake_capture'not in profile:return None\n", "    value=profile['r85_neutral_rate_brake_capture']\n", "    if value=={'enabled':False} and type(value['enabled'])is bool:return None\n", "    expected=dict(enabled=True,kind='neutral_rate_with_original_heading_band_v1')\n", '    if (not isinstance(value,dict) or set(value)!=set(expected) or value!=expected\n', "            or type(value['enabled'])is not bool or type(value['kind'])is not str):\n", "        raise ValueError('Exact explicit neutral-rate brake capture configuration required')\n", "    for key in ('r85_active_turn_braking','r85_quiet_handoff_target'):\n", '        setting=profile.get(key)\n', "        if (not isinstance(setting,dict) or set(setting)!={'enabled'}\n", "                or type(setting['enabled'])is not bool or setting['enabled']is not True):\n", "            raise ValueError('Neutral-rate capture requires original active quiet braking')\n", "    cascade=profile.get('cascade')\n", "    if (not isinstance(cascade,dict) or cascade.get('external_heading_gate')is not True\n", "            or cascade.get('r85_turn_brake_reference')!={'kind':'locked_heading_pd_v1'}):\n", "        raise ValueError('Neutral-rate capture requires the original locked-heading cascade')\n", '    from pre_zero_integral_unload import configuration as unload_configuration\n', '    if unload_configuration(profile)is None:\n', "        raise ValueError('Neutral-rate capture requires the original coordinated unload event')\n", '    return copy.deepcopy(value)\n']}, {'old_start': 102, 'old_end': 102, 'new_start': 128, 'new_end': 129, 'old_lines': [], 'new_lines': ['        self.neutral_rate_brake_capture_config = None\n']}, {'old_start': 354, 'old_end': 354, 'new_start': 381, 'new_end': 391, 'old_lines': [], 'new_lines': ['    def configure_neutral_rate_brake_capture(self,profile):\n', '        """Configure once at construction, before any source-clock/control event."""\n', '        setting=neutral_rate_brake_capture_configuration(profile)\n', '        if self.last_clock is not None:\n', "            raise ValueError('Neutral-rate capture configuration cannot change after a control event')\n", "        if setting is not None and (profile.get('cascade')!=self.config\n", "                or not self.external_heading_gate or self.turn_brake_reference_kind!='locked_heading_pd_v1'):\n", "            raise ValueError('Neutral-rate capture must bind the actual original locked-heading cascade')\n", '        self.neutral_rate_brake_capture_config=setting\n', '\n']}, {'old_start': 421, 'old_end': 421, 'new_start': 458, 'new_end': 459, 'old_lines': [], 'new_lines': ['        neutral_rate_capture = None\n']}, {'old_start': 422, 'old_end': 422, 'new_start': 460, 'new_end': 463, 'old_lines': [], 'new_lines': ["            if (self.neutral_rate_brake_capture_config is not None and mode_override=='brake'\n", '                    and pre_zero_unload_event is None):\n', "                raise ValueError('Neutral-rate capture requires the actual authorized unload event')\n"]}, {'old_start': 797, 'old_end': 798, 'new_start': 838, 'new_end': 850, 'old_lines': ["                if self.turn_brake_reference_kind=='pure_zero_euler_rate_v1':\n"], 'new_lines': ['                if self.neutral_rate_brake_capture_config is not None:\n', "                    neutral_rate_capture=dict(schema='r85_neutral_rate_brake_capture/v1',\n", "                        kind=self.neutral_rate_brake_capture_config['kind'],\n", '                        locked_heading_target_rad=float(heading),locked_heading_error_rad=float(error_yaw),\n', '                        unshaped_locked_heading_PD=dict(P=float(yaw_P),D=float(yaw_D),\n', '                            curvature_FF=float(yaw_ff),reference_Euler_yawrate_radps=float(wref)),\n', '                        applied_heading_PD=dict(P=0.,D=0.,curvature_FF=0.),\n', '                        reference_Euler_yawrate_radps=0.,heading_target_moved=False,\n', "                        actual_heading_error_zeroed=False,pre_zero_event_id=pre_zero_unload_event['event_id'],\n", '                        final_zero_stop_evidence=False)\n', '                    wref=0.;yaw_P=yaw_D=yaw_ff=0.\n', "                elif self.turn_brake_reference_kind=='pure_zero_euler_rate_v1':\n"]}, {'old_start': 870, 'old_end': 870, 'new_start': 922, 'new_end': 928, 'old_lines': [], 'new_lines': ["        if mode=='brake' and neutral_rate_capture is not None:\n", "            self.row['neutral_rate_brake_capture']=neutral_rate_capture\n", "            self.row['active_turn_braking'].update(\n", "                reference_kind=self.neutral_rate_brake_capture_config['kind'],\n", '                base_reference_kind=self.turn_brake_reference_kind,locked_heading_restoration=False,\n', '                neutral_rate_capture=True)\n']}], 'transition_gate.py': [{'old_start': 20, 'old_end': 20, 'new_start': 20, 'new_end': 27, 'old_lines': [], 'new_lines': ['        self.neutral_rate_brake_capture_config=None\n', "        if 'r85_neutral_rate_brake_capture'in profile:\n", '            from cascade_core import neutral_rate_brake_capture_configuration\n', '            self.neutral_rate_brake_capture_config=neutral_rate_brake_capture_configuration(profile)\n', '        self.neutral_first_natural_zero_publication=None;self.neutral_first_natural_zero_ack=None\n', '        self.neutral_old_brake_history=deque(maxlen=64);self.neutral_old_quiet_history=deque(maxlen=64)\n', '        self.neutral_old_capture_last_decision=None;self.neutral_old_capture_last_sample=None\n']}, {'old_start': 52, 'old_end': 52, 'new_start': 59, 'new_end': 61, 'old_lines': [], 'new_lines': ['        if self.neutral_rate_brake_capture_config is not None:\n', "            self.quiet_limits['heading']=self.config['turn_complete_rad']\n"]}, {'old_start': 164, 'old_end': 164, 'new_start': 173, 'new_end': 176, 'old_lines': [], 'new_lines': ['    def _post_entry_brake_heading_band(self):\n', "        return (self.config['turn_complete_rad']if self.neutral_rate_brake_capture_config is not None\n", "            else .25*self.config['turn_complete_rad'])\n"]}, {'old_start': 166, 'old_end': 166, 'new_start': 178, 'new_end': 180, 'old_lines': [], 'new_lines': ['        if self.neutral_rate_brake_capture_config is not None:\n', '            self.neutral_old_quiet_history.clear();self.neutral_old_brake_history.clear()\n']}, {'old_start': 212, 'old_end': 213, 'new_start': 226, 'new_end': 231, 'old_lines': ['            self.quiet_publication=copy.deepcopy(record);return True\n'], 'new_lines': ['            self.quiet_publication=copy.deepcopy(record)\n', "            if (self.neutral_rate_brake_capture_config is not None and not any(record['command_body'])\n", '                    and self.neutral_first_natural_zero_publication is None):\n', '                self.neutral_first_natural_zero_publication=copy.deepcopy(record)\n', '            return True\n']}, {'old_start': 252, 'old_end': 252, 'new_start': 270, 'new_end': 280, 'old_lines': [], 'new_lines': ['            if self.neutral_rate_brake_capture_config is not None:\n', "                error=self.quiet_history[-1]['heading_error_rad']\n", "                if abs(error)<=.25*self.config['turn_complete_rad']:\n", '                    self.neutral_old_brake_history.append((stamp_s,wall_s))\n', '                else:self.neutral_old_brake_history.clear()\n', '                if abs(error)<=.005:self.neutral_old_quiet_history.append(copy.deepcopy(self.quiet_history[-1]))\n', '                else:self.neutral_old_quiet_history.clear()\n', '                self.neutral_old_capture_last_sample.update(source_and_ACK_qualified=True,\n', "                    ACK_watermark=list(watermark),old_brake_instantaneous=abs(error)<=.25*self.config['turn_complete_rad'],\n", '                    old_quiet_instantaneous=abs(error)<=.005)\n']}, {'old_start': 280, 'old_end': 280, 'new_start': 308, 'new_end': 322, 'old_lines': [], 'new_lines': ['    def _record_neutral_old_capture_decision(self,sim_time,wall_time,error,new_brake_ready,new_quiet_ready):\n', '        if self.neutral_rate_brake_capture_config is None:return\n', '        b=self.neutral_old_brake_history;q=self.neutral_old_quiet_history;c=self.config\n', "        latest=self.latest_motion['stamp_s']if self.latest_motion is not None else None\n", "        old_brake=bool(new_brake_ready and abs(error)<=.25*c['turn_complete_rad']\n", "            and len(b)>=c['stop_min_pose_samples']and b[-1][0]==latest\n", "            and b[-1][0]-b[0][0]>=c['stop_window_sim_s']-1e-9)\n", '        old_quiet=bool(new_quiet_ready and abs(error)<=.005\n', "            and len(q)>=c['stop_min_pose_samples']and q[-1]['stamp_s']==latest\n", "            and q[-1]['stamp_s']-q[0]['stamp_s']>=c['stop_window_sim_s']-1e-9)\n", '        self.neutral_old_capture_last_decision=dict(control_s=sim_time,wall_s=wall_time,\n', '            locked_heading_error_rad=error,new_brake_ready=bool(new_brake_ready),new_quiet_ready=bool(new_quiet_ready),\n', '            old_brake_ready=old_brake,old_quiet_ready=old_quiet,old_internal_capture_ready=old_brake and old_quiet,\n', '            diagnostic_only=True,ACK_watermark_consumed_again=False,affects_new_decision=False)\n']}, {'old_start': 343, 'old_end': 343, 'new_start': 385, 'new_end': 390, 'old_lines': [], 'new_lines': ['            if (self.neutral_rate_brake_capture_config is not None and self.controlled_braking\n', "                    and self.handoff_active and self.zero_ack_candidate['exact_zero']\n", "                    and stamp>round(self.handoff_event['brake_entry_control_s']*1e9)\n", '                    and self.neutral_first_natural_zero_ack is None):\n', '                self.neutral_first_natural_zero_ack=copy.deepcopy(self.zero_ack_candidate)\n']}, {'old_start': 383, 'old_end': 383, 'new_start': 430, 'new_end': 434, 'old_lines': [], 'new_lines': ['        if self.neutral_rate_brake_capture_config is not None:\n', '            self.neutral_first_natural_zero_publication=None;self.neutral_first_natural_zero_ack=None\n', '            self.neutral_old_brake_history.clear();self.neutral_old_quiet_history.clear()\n', '            self.neutral_old_capture_last_decision=None;self.neutral_old_capture_last_sample=None\n']}, {'old_start': 392, 'old_end': 392, 'new_start': 443, 'new_end': 444, 'old_lines': [], 'new_lines': ['        if self.neutral_rate_brake_capture_config is not None:center_band=self._post_entry_brake_heading_band()\n']}, {'old_start': 413, 'old_end': 414, 'new_start': 465, 'new_end': 466, 'old_lines': ["                heading_valid=abs(self.error(self.heading,observed_yaw))<=.25*c['turn_complete_rad']\n"], 'new_lines': ['                heading_valid=abs(self.error(self.heading,observed_yaw))<=self._post_entry_brake_heading_band()\n']}, {'old_start': 421, 'old_end': 422, 'new_start': 473, 'new_end': 480, 'old_lines': ['            if self.quiet_target_enabled:self._observe_quiet(stamp_s,wall_s,brake_valid,observed_yaw)\n'], 'new_lines': ['            if self.quiet_target_enabled:\n', '                if self.neutral_rate_brake_capture_config is not None:\n', '                    error=None if observed_yaw is None else self.error(self.heading,observed_yaw)\n', '                    self.neutral_old_capture_last_sample=dict(pose_stamp_s=stamp_s,wall_s=wall_s,\n', '                        locked_heading_error_rad=error,new_brake_motion_valid=bool(brake_valid),\n', '                        source_and_ACK_qualified=False,old_brake_instantaneous=False,old_quiet_instantaneous=False)\n', '                self._observe_quiet(stamp_s,wall_s,brake_valid,observed_yaw)\n']}, {'old_start': 532, 'old_end': 535, 'new_start': 590, 'new_end': 597, 'old_lines': ["                center_band=.25*c['turn_complete_rad']\n", '                if not self._braking_qualified(sim_time,wall_time,error,center_band):return False,0.\n', '                if not self._quiet_qualified(sim_time,wall_time,error):return False,0.\n'], 'new_lines': ['                center_band=self._post_entry_brake_heading_band()\n', '                brake_ready=self._braking_qualified(sim_time,wall_time,error,center_band)\n', '                if not brake_ready:\n', '                    self._record_neutral_old_capture_decision(sim_time,wall_time,error,False,False);return False,0.\n', '                quiet_ready=self._quiet_qualified(sim_time,wall_time,error)\n', '                self._record_neutral_old_capture_decision(sim_time,wall_time,error,brake_ready,quiet_ready)\n', '                if not quiet_ready:return False,0.\n']}, {'old_start': 592, 'old_end': 592, 'new_start': 654, 'new_end': 679, 'old_lines': [], 'new_lines': ['        if self.neutral_rate_brake_capture_config is not None:\n', "            result['neutral_rate_brake_capture']=dict(configuration=copy.deepcopy(self.neutral_rate_brake_capture_config),\n", '                new_internal_capture_changed=True,original_business_acceptance_unchanged=True,\n', "                acceptance_scope='Original teacher_transition fields, source/motion/ACK and terminal fixed5 owner; R16 internal capture is changed.',\n", "                brake_entry_heading_band_rad=.25*self.config['turn_complete_rad'],\n", '                post_entry_capture_heading_band_rad=self._post_entry_brake_heading_band(),\n', '                locked_heading_rad=self.heading,locked_heading_target_moved=False,\n', '                original_R16_internal_capture_claimed=False,\n', "                old_internal_capture_shadow=dict(diagnostic_only=True,brake_heading_band_rad=.25*self.config['turn_complete_rad'],\n", '                    quiet_heading_band_rad=.005,brake_samples=len(self.neutral_old_brake_history),\n', '                    brake_first_stamp_s=self.neutral_old_brake_history[0][0]if self.neutral_old_brake_history else None,\n', '                    brake_last_stamp_s=self.neutral_old_brake_history[-1][0]if self.neutral_old_brake_history else None,\n', '                    quiet_samples=len(self.neutral_old_quiet_history),\n', "                    quiet_first_stamp_s=self.neutral_old_quiet_history[0]['stamp_s']if self.neutral_old_quiet_history else None,\n", "                    quiet_last_stamp_s=self.neutral_old_quiet_history[-1]['stamp_s']if self.neutral_old_quiet_history else None,\n", '                    last_control_evaluation=copy.deepcopy(self.neutral_old_capture_last_decision),\n', '                    last_pose_observation=copy.deepcopy(self.neutral_old_capture_last_sample),\n', '                    old_numeric_conditions_on_new_control_trajectory=True,old_R16_control_counterfactual_reproduced=False,\n', '                    same_validated_source_and_ACK_samples=True,independent_ACK_consumption=False,new_decision_affected=False),\n', '                heading_error_band_is_not_relative_T0_drift=True,\n', '                first_qualified_brake_zero_publication=copy.deepcopy(self.neutral_first_natural_zero_publication),\n', '                first_observed_validated_brake_zero_ack=copy.deepcopy(self.neutral_first_natural_zero_ack),\n', "                natural_zero_witness_scope='First validator-qualified publication or selected ACK observed here; not earliest actual wire zero in the full stream.',\n", '                natural_zero_is_not_formal_T0=True,natural_zero_adds_stop_evidence=False,\n', '                terminal_fixed5_owner_unchanged=True)\n']}], 'controller.py': [{'old_start': 1258, 'old_end': 1258, 'new_start': 1258, 'new_end': 1260, 'old_lines': [], 'new_lines': ["            if 'r85_neutral_rate_brake_capture'in self.profile:\n", '                self.cascade.configure_neutral_rate_brake_capture(self.profile)\n']}]}
NEUTRAL_EVIDENCE=[{'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/R34_CANDIDATE_SELECTION.json', 'sha256': 'bfcb528cd29f6b222b92083c7843bd45e3ab69f82e80ff77abde013f9778f3fb'}, {'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/neutral_rate_brake_candidate/AUTHOR_BEHAVIOR_DELIVERY.json', 'sha256': '4386ee7b18f4183f7ff50e4a096d6702b6c03853a7434b75ed47e4ca5c0760b5'}, {'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/neutral_rate_brake_candidate/AUTHOR_CORE_CPU_REPORT.json', 'sha256': '24c4dba3d7b6e2ba3d37984c9c21a74a6bd11aee0c679d19dd8bd956951043a3'}, {'path': '/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-14/task/r83_demo/implementation_attempt_20261014/closed_loop_integration_20261014/r85_support_contract_repair_20261015/window_20261016_until1400/neutral_rate_brake_candidate/AUTHOR_GATE_CONTROLLER_CPU_REPORT.json', 'sha256': '3fdaa568fa0cd9992da41b790afe97db31b669d3fe07751867111bd5cd8f2025'}]
NEUTRAL_CONFIG=dict(enabled=True,kind='neutral_rate_with_original_heading_band_v1')
NEUTRAL_CHANGE=dict(reference_control_changed=True,new_internal_capture_changed=True,
    original_business_acceptance_unchanged=True,locked_heading_target_changed=False,
    original_brake_heading_band_rad=.025,original_quiet_heading_band_rad=.005,
    new_brake_heading_band_rad=.1,new_quiet_heading_band_rad=.1,
    original_entry_center_band_rad=.025,new_entry_center_band_rad=.025,
    original_drive_heading_rearm_rad=.2,new_drive_heading_rearm_rad=.2,
    heading_band_source='original teacher_transition.turn_complete_rad',
    new_capture_may_exit_with_larger_heading_error=True,path_safety_certified=False,
    shadow_semantics='old thresholds under new trajectory; no old-controller counterfactual',
    original_strict_shadow_same_samples_nonconsuming=True,
    formal_zero_T0_shifted=False,original_post_zero_window_s=5.,
    teacher_PI_and_student_PID_still_active=True,physical_convergence_or_rebound_proven=False)
NEUTRAL_TARGETS=['cascade_core.py','transition_gate.py','controller.py','r85_session_guard.py',
    'scan_workspace.py',RENEW_HELPER.name,RENEW_PROOF.name,RENEW_PROFILE]

def _renew_guard_bytes(raw):
    require(digest(raw)==RENEW_GUARD_BEFORE,'actual R16 guard beforeimage')
    for old,new in RENEW_GUARD_SUBSTITUTIONS:
        require(raw.count(old.encode())==1,'two exact14UTC guard substitutions')
        raw=raw.replace(old.encode(),new.encode())
    return raw

def _renew_scanner_bytes(raw):
    require(digest(raw)==RENEW_SCANNER_BEFORE,'actual R16 scanner beforeimage')
    text=raw.decode()
    for old,new in RENEW_SCANNER_PATCHES:
        require(text.count(old)==1,'exact combined neutral capture scanner dispatch')
        text=text.replace(old,new)
    return text.encode()

def _neutral_source_bytes(name,raw):
    require(name in NEUTRAL_BEFORE and digest(raw)==NEUTRAL_BEFORE[name],
        'neutral capture original R16 beforeimage: '+name)
    lines=raw.decode().splitlines(keepends=True)
    for row in reversed(NEUTRAL_DELTAS[name]):
        i,j=row['old_start'],row['old_end']
        require(lines[i:j]==row['old_lines'],'neutral capture exact reviewed patch context: '+name)
        lines[i:j]=row['new_lines']
    out=''.join(lines).encode()
    require(digest(out)==NEUTRAL_AFTER[name],'neutral capture exact actual source SHA: '+name)
    return out

def _verify_1400_guard(historical_bytes):
    return (ROOT/'r85_session_guard.py').read_bytes()==_renew_guard_bytes(historical_bytes)

def _verify_1400_actual(name,historical_bytes):
    if name=='scan_workspace.py':historical_bytes=_renew_scanner_bytes(historical_bytes)
    elif name in NEUTRAL_BEFORE:historical_bytes=_neutral_source_bytes(name,historical_bytes)
    return (ROOT/name).read_bytes()==historical_bytes

def _validate_1400_metadata(data):
    keys={'schema','kind','baseline_commit','exact_targets','expected_bound_source_count_before',
        'expected_bound_source_count_after','authorization','baseline_source_bindings','historical_helper',
        'historical_profile','guard','validator','controls','bridge_helper','profile','schedule',
        'helper_delta_sha256','behavior_evidence','configuration','change_declaration',
        'simulation_only','runtime_acceptance_inherited','navigation_acceptance_established',
        'source_or_feedback_guards_changed','physical_thresholds_changed','SCAN_contract_bytes_changed',
        'SCAN_implementation_changed'}
    require(isinstance(data,dict)and set(data)==keys,'combined neutral capture exact proof keys')
    require(isinstance(data['schedule'],dict)and set(data['schedule'])==set(RENEW_SCHEDULE)
        and all(type(data['schedule'][k])is type(v)for k,v in RENEW_SCHEDULE.items()),'exact schedule value types')
    require(data['schema']=='r85_neutral_rate_brake_capture_source_adapter/v1'
        and data['kind']==NEUTRAL_CONFIG['kind']and data['baseline_commit']==RENEW_BASELINE
        and data['schedule']==RENEW_SCHEDULE,'explicit neutral reference and14UTC finite budgets')
    require(data['exact_targets']==NEUTRAL_TARGETS and data['expected_bound_source_count_before']==262
        and data['expected_bound_source_count_after']==265,'exact eight combined targets')
    require(data['simulation_only']is True and all(data[k]is False for k in
        ('runtime_acceptance_inherited','navigation_acceptance_established','source_or_feedback_guards_changed',
         'physical_thresholds_changed','SCAN_contract_bytes_changed','SCAN_implementation_changed')),
        'control candidate does not establish physical/source/navigation acceptance')
    require(data['configuration']==NEUTRAL_CONFIG and type(data['configuration']['enabled'])is bool
        and data['change_declaration']==NEUTRAL_CHANGE
        and all(type(data['change_declaration'][k])is type(v)for k,v in NEUTRAL_CHANGE.items()),
        'new reference and internal capture bands are explicit; original business window retained')
    require(data['authorization']==RENEW_AUTH and data['helper_delta_sha256']==RENEW_HELPER_DELTA_SHA
        and data['behavior_evidence']==NEUTRAL_EVIDENCE,'trusted authorization and pinned reviewed source evidence')
    for key,name,before in [('guard','r85_session_guard.py',RENEW_GUARD_BEFORE),
                            ('validator','scan_workspace.py',RENEW_SCANNER_BEFORE)]:
        row=data[key]
        require(set(row)=={'path','before_sha256','after_sha256','beforeimage'}and row['path']==str(ROOT/name)
            and row['before_sha256']==before,'exact current-source delta: '+name)
    require(data['validator']['after_sha256']==RENEW_SCANNER_AFTER,'exact new scanner')
    require(isinstance(data['controls'],dict)and set(data['controls'])==set(NEUTRAL_BEFORE),'exact three control source deltas')
    for name,row in data['controls'].items():
        require(set(row)=={'path','before_sha256','after_sha256','beforeimage','delta_sha256'}
            and row['path']==str(ROOT/name)and row['before_sha256']==NEUTRAL_BEFORE[name]
            and row['after_sha256']==NEUTRAL_AFTER[name]
            and row['delta_sha256']==digest(json.dumps(NEUTRAL_DELTAS[name],sort_keys=True,separators=(',',':')).encode()),
            'exact neutral control declaration: '+name)
    require(data['profile']==str(ROOT/RENEW_PROFILE),'new unique neutral capture profile')
    return True

def _schedule_1400(session=None):
    require(Path(__file__).resolve()==RENEW_HELPER,'actual combined neutral capture helper location')
    require(RENEW_PROOF.is_file()and not RENEW_PROOF.is_symlink(),'regular neutral capture proof file')
    proof_sha=digest(RENEW_PROOF.read_bytes())
    if session is not None:
        require(session.get('neutral_rate_brake_capture_proof')==str(RENEW_PROOF)
            and session.get('neutral_rate_brake_capture_proof_sha256')==proof_sha,'explicit neutral capture session proof')
    data=json.loads(RENEW_PROOF.read_text());_validate_1400_metadata(data)
    evidence=[RENEW_PROOF]
    authority=file_ref(data['authorization'],Path(RENEW_AUTH['path']),RENEW_AUTH['sha256'])
    auth=json.loads(authority.read_text())
    require(auth.get('authorized_deadline_UTC')==RENEW_SCHEDULE['deadline_utc']
        and auth.get('automatic_extension')is False and auth.get('retained_cap_B')==130000000000
        and auth.get('round_cap_B')==10000000000,'trusted14UTC user authority and original storage caps')
    old_helper=file_ref(data['historical_helper'],UNLOAD_HELPER,RENEW_OLD_HELPER_SHA)
    old_profile=file_ref(data['historical_profile'],ROOT/UNLOAD_PROFILE,RENEW_OLD_PROFILE_SHA)
    bindings=file_ref(data['baseline_source_bindings'],RENEW_STAGE/'beforeimage/SOURCE_BINDINGS.json',RENEW_BINDINGS_SHA)
    for key,fn in [('guard',_renew_guard_bytes),('validator',_renew_scanner_bytes)]:
        row=data[key];before=file_ref(row['beforeimage'],RENEW_STAGE/'beforeimage'/Path(row['path']).name,row['before_sha256'])
        raw=fn(before.read_bytes());require(digest(raw)==row['after_sha256']and Path(row['path']).read_bytes()==raw,'actual combined source bytes')
        evidence.extend([before,Path(row['path'])])
    for name,row in data['controls'].items():
        before=file_ref(row['beforeimage'],RENEW_STAGE/'beforeimage'/name,row['before_sha256'])
        require(Path(row['path']).read_bytes()==_neutral_source_bytes(name,before.read_bytes()),'actual neutral control source bytes')
        evidence.extend([before,Path(row['path'])])
    for row in data['behavior_evidence']:evidence.append(file_ref(row))
    helper=file_ref(data['bridge_helper'],RENEW_HELPER)
    expected=json.loads(old_profile.read_text());expected['r85_session']['deadline_utc']=RENEW_SCHEDULE['deadline_utc']
    expected['diagnostic_observe_only']['user_authorized_work_deadline_utc']=RENEW_SCHEDULE['deadline_utc']
    expected['diagnostic_observe_only']['work_deadline_basis']=RENEW_SCHEDULE['basis']
    expected['r85_neutral_rate_brake_capture']=copy.deepcopy(NEUTRAL_CONFIG)
    expected['r85_session']['neutral_rate_brake_capture_proof']=str(RENEW_PROOF)
    expected['r85_session']['neutral_rate_brake_capture_proof_sha256']=proof_sha
    actual_path=ROOT/RENEW_PROFILE;actual=json.loads(actual_path.read_text())
    require(actual==expected and actual['teacher_transition']['turn_complete_rad']==.1
        and(session is None or session==actual['r85_session']),'profile exact3 renewed values +neutral config +two proof keys')
    data['proof']=dict(path=str(RENEW_PROOF),sha256=proof_sha)
    data['evidence']=sorted(set(evidence+[authority,old_helper,old_profile,bindings,helper,actual_path]))
    return data

def _extend_1400_closure(expected):
    data=_schedule_1400();before_path=Path(data['baseline_source_bindings']['path'])
    before=historical_repair._binding_map(json.loads(before_path.read_text()))
    require(len(before)==262 and {p:r['sha256']for p,r in before.items()}==expected,'exact historical262 mapping retained')
    expected=dict(expected)
    for row in [data['guard'],data['validator']]+list(data['controls'].values()):
        require(expected[row['path']]==row['before_sha256'],'combined source inherited beforeSHA')
        expected[row['path']]=row['after_sha256']
    added={str(RENEW_HELPER):data['bridge_helper']['sha256'],str(RENEW_PROOF):data['proof']['sha256'],
        str(ROOT/RENEW_PROFILE):digest((ROOT/RENEW_PROFILE).read_bytes())}
    require(not(set(added)&set(expected)),'three genuinely new combined identities');expected.update(added)
    return expected,data['evidence']

def prepare_neutral_rate_brake_capture(session,base):
    renewed=_schedule_1400(session)
    old=json.loads((ROOT/UNLOAD_PROFILE).read_text())['r85_session']
    unload,historical,live_schedule,live_repair=prepare_unload(old,base)
    unload['historical_sources']=copy.deepcopy(unload['sources'])
    for name,row in renewed['controls'].items():unload['sources'][name]['after_sha256']=row['after_sha256']
    unload['sources']['scan_workspace.py']['after_sha256']=renewed['validator']['after_sha256']
    live_schedule['guard']['after_sha256']=renewed['guard']['after_sha256']
    live_schedule['validator']['after_sha256']=renewed['validator']['after_sha256']
    live_repair['controller']['after_sha256']=renewed['controls']['controller.py']['after_sha256']
    live_repair['validator']['after_sha256']=renewed['validator']['after_sha256']
    return renewed,unload,historical,live_schedule,live_repair

def attach_neutral_rate_brake_capture(adapter,data):
    adapter['neutral_rate_brake_capture']=data
    adapter['actual'][str(RENEW_HELPER)]=data['bridge_helper']['sha256']
    adapter['evidence']=sorted(set(adapter['evidence']+data['evidence']))
    return adapter

def resolved_neutral_rate_brake_capture(resolved,adapter):
    data=adapter['neutral_rate_brake_capture'];unload=adapter['pre_zero_integral_unload']
    resolved_unload(resolved,unload)
    target=resolved['resolved_adapter'];target['schema']='r85_neutral_rate_brake_capture_resolved_bindings/v1'
    target['overrides'][data['guard']['path']]['after_sha256']=data['guard']['after_sha256']
    target['schedule_renewal']['source_identity_status']='historical_until1000UTC'
    target['schedule_renewal']['actual_identity_superseded_by']=copy.deepcopy(data['proof'])
    target['pre_zero_integral_unload']['source_identity_status']='historical_R16_source_ids; integral_unload_recurrence_retained'
    target['pre_zero_integral_unload']['historical_sources']=copy.deepcopy(unload['historical_sources'])
    target['pre_zero_integral_unload']['actual_identity_superseded_by']=copy.deepcopy(data['proof'])
    target['neutral_rate_brake_capture']=dict(proof=copy.deepcopy(data['proof']),schedule=copy.deepcopy(data['schedule']),
        authorization=copy.deepcopy(data['authorization']),configuration=copy.deepcopy(data['configuration']),
        change_declaration=copy.deepcopy(data['change_declaration']),current_control_sources=copy.deepcopy(data['controls']),
        actual_guard=copy.deepcopy(data['guard']),actual_validator=copy.deepcopy(data['validator']),
        historical_R16_helper=copy.deepcopy(data['historical_helper']),
        reference_control_changed=True,new_internal_capture_changed=True,original_business_acceptance_unchanged=True,
        runtime_acceptance_inherited=False,navigation_acceptance_established=False)
