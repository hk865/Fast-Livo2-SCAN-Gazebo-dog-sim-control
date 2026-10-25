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
    require(digest(expected)==row['after_sha256'] and (ROOT/name).read_bytes()==expected,'quiet truthful actual current identity: '+name)
    return True


def _verified_historical_scanner(historical_bytes,actual_bytes,repair,quiet):
    row=repair['validator'];before=file_ref(row['beforeimage'],historical_repair.REPAIR_STAGE/'beforeimage/scan_workspace.py',historical_repair.SCANNER_BEFORE)
    r14=historical_repair.repair_scanner_after_bytes(before.read_bytes())
    require(before.read_bytes()==historical_bytes and digest(r14)==row['after_sha256']==SOURCE_BEFORE['scan_workspace.py'],'retained R13 to R14 scanner identity')
    snapshot=file_ref(quiet['sources']['scan_workspace.py']['beforeimage'],QUIET_STAGE/'beforeimage/scan_workspace.py',row['after_sha256'])
    require(snapshot.read_bytes()==r14 and quiet_scanner_after_bytes(r14)==actual_bytes,'explicit R13-R14-R15 scanner chain')
    return _verified_quiet_source('scan_workspace.py',row['after_sha256'],quiet)


def _verified_historical_controller(path,historical_sha,repair,quiet):
    row=repair['controller'];require(path==str(ROOT/'controller.py')and historical_sha==historical_repair.CONTROLLER_BEFORE,'historical controller path and R13 identity')
    before=file_ref(row['beforeimage'],historical_repair.REPAIR_STAGE/'beforeimage/controller.py',historical_sha)
    r14=historical_repair.repair_controller_after_bytes(before.read_bytes())
    snapshot=file_ref(quiet['sources']['controller.py']['beforeimage'],QUIET_STAGE/'beforeimage/controller.py',row['after_sha256'])
    require(digest(r14)==row['after_sha256']==SOURCE_BEFORE['controller.py']and snapshot.read_bytes()==r14,'retained R13 to R14 controller identity')
    return _verified_quiet_source('controller.py',row['after_sha256'],quiet)


def _load_historical_schedule(session,base,repair,quiet):
    require(Path(__file__).resolve()==ROOT/'pre_zero_quiet_adapter.py','active quiet helper location')
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
    current_path=SOURCE_REPO/'SOURCE_BINDINGS.json';current=historical_repair._binding_map(json.loads(current_path.read_text()))
    require(len(current)==258 and set(current)==set(expected),'actual exact 255 to258 closure')
    evidence=[baseline,current_path]
    for path,sha in expected.items():
        row=current[path];active=file_ref(dict(path=path,sha256=sha))
        require(row['sha256']==sha and row.get('bytes')==active.stat().st_size,'truthful full source binding bytes/hash')
        tracked=SOURCE_REPO/row['tracked'];require(tracked.is_file()and not tracked.is_symlink()and digest(tracked.read_bytes())==sha,'tracked evidence matches actual source')
        evidence.append(active)
    return evidence


def load_quiet(session,base):
    require(Path(__file__).resolve()==QUIET_HELPER,'actual quiet helper location')
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
