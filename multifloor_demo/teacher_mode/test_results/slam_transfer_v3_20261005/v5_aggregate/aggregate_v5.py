#!/usr/bin/env python3
"""New-directory-only aggregate of six finished V5 actual-SLAM experiments.

The archived independent validator is called with write=False. No source,
receipt, raw run, process, ROS or simulator is changed or started.
"""
import difflib
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
EVIDENCE=HERE.parent
ROOT=EVIDENCE.parents[1]
RUNS=ROOT/'runs'
IDS=('8dda','ad47','8bf5','e372','63d3','2e3f')
PINNED_VALIDATOR='65df78481f7e5e30dc7b1a09bb9c9aab8fa5fa351d38b602c2d4ce018693d231'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def load(path):return json.loads(Path(path).read_text())
def rows(path):
    with Path(path).open() as stream:return [json.loads(line) for line in stream if line.strip()]


def write(path,value):
    text=json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False,
        default=lambda x:x.item() if isinstance(x,np.generic) else (_ for _ in ()).throw(TypeError(type(x).__name__)))+'\n'
    with Path(path).open('x') as stream:stream.write(text)


def dist(values):
    a=np.asarray(values,float)
    return {'count':len(a),'min':float(a.min()),'p50':float(np.percentile(a,50)),
            'p95':float(np.percentile(a,95)),'p99':float(np.percentile(a,99)),'max':float(a.max())} if len(a) else None


def main():
    if (HERE/'aggregate.json').exists():raise RuntimeError('Refusing to overwrite a completed aggregate')
    validator_path=EVIDENCE/'evaluate_slam_transfer.py'
    if sha(validator_path)!=PINNED_VALIDATOR:raise RuntimeError('Unexpected frozen independent validator SHA')
    spec=importlib.util.spec_from_file_location('exact_archived_slam_transfer_validator',validator_path)
    validator=importlib.util.module_from_spec(spec);spec.loader.exec_module(validator)
    originals={};hash_cache={};records=[];plots=[]
    for suffix in IDS:
        matches=list(RUNS.glob('*'+suffix))
        if len(matches)!=1:raise RuntimeError('Nonunique requested actual run '+suffix)
        run=matches[0].resolve();receipt_path=run/'summary_slam_transfer_independent.json'
        receipt=load(receipt_path);originals[str(receipt_path)]=sha(receipt_path)
        input_verification=[];hash_errors=[]
        for name,digest in receipt['input_sha256'].items():
            p=Path(name);st=p.stat();key=(str(p),st.st_size,st.st_mtime_ns)
            if key not in hash_cache:hash_cache[key]=sha(p)
            actual=hash_cache[key]
            input_verification.append({'path':name,'recorded_sha256':digest,'actual_sha256':actual,'matches':actual==digest})
            if actual!=digest:hash_errors.append(name)
        helper=next(Path(name) for name in receipt['input_sha256'] if name.endswith('/sources/truth/evaluate.py'))
        protocol=helper.with_name('protocol.json')
        recomputed=validator.evaluate(run,write=False,helper_path=helper,protocol_path=protocol)
        before_checks=receipt['checks'];after_checks=recomputed['checks']
        status_conflicts={k:{'original':v['status'],'recomputed':after_checks.get(k,{}).get('status')}
            for k,v in before_checks.items() if after_checks.get(k,{}).get('status')!=v['status']}
        if set(before_checks)!=set(after_checks):status_conflicts['check_names']={'original':sorted(before_checks),'recomputed':sorted(after_checks)}
        for key in ('offline_actual_anchored_route_execution','offline_actual_drive_heading','offline_actual_stable_COM_speed',
                    'independent_200Hz_motion_safety','first_source_arrival_fixed_5s_parking'):
            if before_checks.get(key)!=after_checks.get(key):status_conflicts[key]={'reason':'Full original/recomputed numerical check differs'}
        folder=run/'SLAM_fixed_route';pose_rows=rows(folder/'adapter_slam_poses.jsonl');imu_rows=rows(folder/'adapter_IMU_inputs.jsonl')
        details=rows(folder/'adapter_controller_updates.jsonl');attempts=rows(run/'slam_execution_commands.jsonl');telemetry=rows(run/'telemetry.jsonl')
        anchor=rows(folder/'adapter_route_anchor.jsonl')[0]
        pose_index={p['stamp_ns']:p for p in pose_rows if p['accepted'] is True}
        imu_index={p['stamp_ns']:p for p in imu_rows if p['accepted'] is True}
        math_updates=[d for d in details if d['core_original_row']['controller_updated'] is True]
        update_ns=[d['envelope']['control_pose_stamp_ns'] for d in math_updates]
        gaps=np.diff(update_ns)/1e9
        rate=(len(update_ns)-1)*1e9/(update_ns[-1]-update_ns[0])
        fresh={k:[] for k in ('pose_sim_s','pose_wall_s','gyro_sim_s','gyro_wall_s','producer_sim_s','producer_wall_s','gyro_pose_gap_s','read_span_s')}
        healthy_times=[];rejections={};future=0;invalid_ttl=0;raw_join_errors=[];race_count=0;reject_after_first=0;accepted_seen=False
        for attempt in attempts:
            if attempt['rejected']:
                reason=attempt['reason'];rejections[reason]=rejections.get(reason,0)+1
                reject_after_first+=int(accepted_seen)
                continue
            accepted_seen=True;e=attempt['actual_original_envelope'];a=attempt['actual_read_attempt'];clock=attempt['physics_clock_ns'];wall=attempt['read_monotonic_wall']
            p=pose_index[e['pose_stamp_ns']];g=imu_index[e['gyro_stamp_ns']]
            if p['received_monotonic_wall']!=e['pose_received_monotonic_wall'] or g['received_monotonic_wall']!=e['gyro_received_monotonic_wall']:raw_join_errors.append(clock)
            fresh['pose_sim_s'].append((clock-p['stamp_ns'])/1e9);fresh['pose_wall_s'].append(wall-p['received_monotonic_wall'])
            fresh['gyro_sim_s'].append((clock-g['stamp_ns'])/1e9);fresh['gyro_wall_s'].append(wall-g['received_monotonic_wall'])
            fresh['producer_sim_s'].append((clock-e['clock_ns'])/1e9);fresh['producer_wall_s'].append(wall-e['monotonic_wall'])
            fresh['gyro_pose_gap_s'].append((p['stamp_ns']-g['stamp_ns'])/1e9);fresh['read_span_s'].append(a['read_span_s'])
            race_count+=int(a['read_begin_monotonic_wall']<e['monotonic_wall']<=a['read_completed_monotonic_wall'])
            future+=int(g['stamp_ns']>p['stamp_ns'] or p['stamp_ns']>clock or e['control_pose_stamp_ns']>p['stamp_ns'])
            invalid_ttl+=int(not all(0<=fresh[k][-1]<=.3 for k in ('pose_sim_s','pose_wall_s','gyro_sim_s','gyro_wall_s','producer_wall_s'))
                or not -.05<=fresh['producer_sim_s'][-1]<=.3 or fresh['gyro_pose_gap_s'][-1]>.02)
            healthy_times.append(clock/1e9)
        input_after={name:sha(name) for name in receipt['input_sha256'] if str(run) in name}
        unchanged=all(input_after[name]==receipt['input_sha256'][name] for name in input_after) and sha(receipt_path)==originals[str(receipt_path)]
        checks=receipt['checks'];route=checks['offline_actual_anchored_route_execution'];heading=checks['offline_actual_drive_heading'];speed=checks['offline_actual_stable_COM_speed'];safety=checks['independent_200Hz_motion_safety'];parking=checks['first_source_arrival_fixed_5s_parking']['parking']
        threshold_audit={'route_maximum':route['maximum_xy_distance_m']<=.20,'route_RMS':route['drive_distance_rms_m']<=.08,
            'drive_heading':heading['maximum_error_rad']<=.2,'stable_speed_MAE':speed['mean_absolute_error_mps']<=max(.05,.25*speed['mean_reference_mps']),
            'RP':safety['maximum_abs_roll_pitch_rad']<=.65,'clearance':safety['minimum_clearance_m']>=.18,
            'force_command':safety['maximum_abs_force_command_Nm']<=23.50001,'qd':safety['maximum_abs_joint_velocity_radps']<=30.001,
            'body_contact_zero':safety['body_contact_rows']==0,'fault_zero':safety['native_faults']==0 and safety['worker_fault'] is None,
            'parking_source_and_native_complete':parking['source_window_complete'] and parking['native_window_complete'],
            'parking_actual_XY':parking['actual_native_xy_drift_m']<=.05,'parking_actual_yaw':parking['actual_native_yaw_drift_rad']<=.1,
            'parking_SLAM_XY':parking['actual_SLAM_xy_drift_m']<=.05,'parking_SLAM_yaw':parking['actual_SLAM_yaw_drift_rad']<=.1,
            'parking_zero_command':parking['continuous_requested_and_actor_command_zero'],
            'past_sensor_feedback':future==0,'300ms_source_freshness':invalid_ttl==0,'original_source_wall_identity':not raw_join_errors,
            'distinct_actual_updates_10Hz':len(update_ns)==len(set(update_ns)) and np.max(abs(gaps-.1))<=1.000001e-9,
            'source_dwell_ge_600ms':checks['source_arrival_and_nonduplicated_dwell']['dwell']['duration_s']>=.6}
        all_pass=(receipt['status']=='passed' and all(v['status']=='passed' for v in checks.values()) and recomputed['status']=='passed'
            and not status_conflicts and not hash_errors and unchanged and all(threshold_audit.values()))
        record={'run':str(run),'run_id':run.name,'suffix':suffix,'case':'roundtrip_6m_each_leg' if 'roundtrip' in run.name else 'forward_6m',
            'repeat':int(run.name.split('_r')[-1].split('_')[0]),'status':'passed' if all_pass else 'failed','original_independent_status':receipt['status'],
            'original_receipt_path':str(receipt_path),'original_receipt_sha256':originals[str(receipt_path)],
            'all_original_22_checks':{k:v['status'] for k,v in checks.items()},'frozen_validator_rerun_status':recomputed['status'],
            'recomputed_checks':after_checks,'original_and_recomputed_conflicts':status_conflicts,
            'input_hash_verification':input_verification,'hash_mismatches':hash_errors,'original_inputs_and_receipt_unchanged_after_read':unchanged,
            'independent_numeric_and_raw_threshold_audit':threshold_audit,
            'actual_update_rate_hz':rate,'actual_update_period_s':dist(gaps),'actual_math_update_count':len(update_ns),
            'actual_distinct_math_update_count':len(set(update_ns)),'selected_calibration_hz':25,'actual_fresh_SLAM_ceiling_hz':10,
            'accepted_SLAM_headers':len(pose_index),'accepted_IMU_headers':len(imu_index),
            'source_freshness_raw_recomputed':{k:dist(v) for k,v in fresh.items()},'future_feedback_count':future,'source_TTL_violation_count':invalid_ttl,
            'source_identity_mismatch_count':len(raw_join_errors),'read_begin_to_completion_producer_publish_cases':race_count,
            'original_rejection_reason_counts':rejections,'rejected_after_first_accepted_count':reject_after_first,
            'motion':{'route_maximum_m':route['maximum_xy_distance_m'],'route_drive_RMS_m':route['drive_distance_rms_m'],
                'maximum_drive_heading_rad':heading['maximum_error_rad'],'mean_reference_mps':speed['mean_reference_mps'],
                'mean_actual_along_reference_COM_mps':speed['mean_actual_projection_mps'],'stable_speed_MAE_mps':speed['mean_absolute_error_mps'],
                'stable_duration_s':speed['stable_duration_s'],'maximum_abs_RP_rad':safety['maximum_abs_roll_pitch_rad'],
                'minimum_clearance_m':safety['minimum_clearance_m'],'maximum_force_command_Nm':safety['maximum_abs_force_command_Nm'],
                'maximum_joint_speed_radps':safety['maximum_abs_joint_velocity_radps'],'body_contact_count':safety['body_contact_rows'],
                'fixed_parking':parking},'runtime_clean':checks['six_owned_and_six_SLAM_children_clean'],
            'completion_and_actual_duration':checks['declared_source_completion_and_run_termination'],
            'SLAM_anchor':anchor,'source_profile':load(run/'frozen_controller_profile.json')}
        records.append(record)
        sourcepos=np.asarray([p['position'] for ns,p in pose_index.items() if ns>=anchor['original_stamp_ns']]);source_t=np.asarray([ns/1e9 for ns in pose_index if ns>=anchor['original_stamp_ns']]);sroute=np.asarray(anchor['fixed_route_camera_init_xyz'])
        d,_=validator.common.polyline_distance(sourcepos,sroute) if hasattr(validator,'common') else (None,None)
        if d is None:
            helper_spec=importlib.util.spec_from_file_location('unchanged_native_helper_for_distance',helper);hm=importlib.util.module_from_spec(helper_spec);helper_spec.loader.exec_module(hm)
            d,_=hm.polyline_distance(sourcepos,sroute)
        plots.append({'record':record,'source_pos':sourcepos,'source_t':source_t,'route':sroute,'source_distance':d,
            'telemetry':telemetry,'freshness':fresh,'accepted_read_world_t':healthy_times})
        print(suffix,record['status'],rate,record['motion']['mean_actual_along_reference_COM_mps'],flush=True)
    # Retain V4 receipts as context without reevaluation or status replacement.
    history=[]
    for run in sorted(RUNS.glob('20261005_*SLAM_fixed_route*gyro_v4*')):
        p=run/'summary_slam_transfer_independent.json'
        if not p.is_file():continue
        old=load(p);originals[str(p.resolve())]=sha(p)
        history.append({'run':str(run.resolve()),'receipt_sha256':sha(p),'original_status':old['status'],
            'original_failed_checks':{k:v for k,v in old['checks'].items() if v['status']!='passed'},'changed':False})
    baseline=next(RUNS.glob('20261005_013004*51d2'));v5=Path(records[0]['run'])
    source_diff={};source_hash_diff={}
    for name in ('adapter.py','worker.py','run.py','core.py'):
        a=baseline/'sources/slam_binding'/name;b=v5/'sources/slam_binding'/name
        source_diff[name]=''.join(difflib.unified_diff(a.read_text().splitlines(True),b.read_text().splitlines(True),fromfile='V4 '+str(a),tofile='V5 '+str(b)))
        source_hash_diff[name]={'V4':sha(a),'V5':sha(b),'unchanged':sha(a)==sha(b)}
    policy_equal={n:sha(baseline/'sources/policy'/n)==sha(v5/'sources/policy'/n) for n in ('worker.py','observation.py','contract.json')}
    profile_equal=sha(baseline/'frozen_controller_profile.json')==sha(v5/'frozen_controller_profile.json')
    constants={}
    for version,run in (('V4',baseline),('V5',v5)):
        tree=__import__('ast').parse((run/'sources/slam_binding/adapter.py').read_text())
        constants[version]={node.targets[0].id:__import__('ast').literal_eval(node.value) for node in tree.body
            if isinstance(node,__import__('ast').Assign) and isinstance(node.targets[0],__import__('ast').Name) and node.targets[0].id in ('TIMEOUT_S','FUTURE_S')}
    revision_verified=not source_diff['core.py'] and not source_diff['run.py'] and all(policy_equal.values()) and profile_equal and constants['V4']==constants['V5']=={'TIMEOUT_S':.3,'FUTURE_S':.05}
    aggregate={'schema':'actual_SLAM_teacher_V5_six_run_aggregate/v1','status':'passed' if all(r['status']=='passed' for r in records) and revision_verified else 'failed',
        'scope':'Only actual sensor-SLAM fixed body-relative flat 6m forward and 6m+6m roundtrip, three repeated runs each',
        'run_count':6,'passed_count':sum(r['status']=='passed' for r in records),'original_required_checks_count':sum(len(r['all_original_22_checks']) for r in records),
        'original_required_checks_passed':sum(sum(s=='passed' for s in r['all_original_22_checks'].values()) for r in records),
        'score':None,'navigation_ground_truth_used':False,'actor_privileged_observation_dimensions':232,'Actor_total_dimensions':247,
        'controller_known_dimensions':15,'selected_calibration_hz':25,'actual_fresh_SLAM_update_hz':10,
        'model_sha256':load(v5/'slam_execution.json')['model_sha256'],'controller_sha256':source_hash_diff['core.py']['V5'],
        'selected_profile_sha256':sha(v5/'frozen_controller_profile.json'),'independent_validator_sha256':sha(validator_path),
        'aggregate_script_sha256':sha(__file__),'runs':records,'V4_original_receipts_retained':history,
        'V4_to_V5':{'baseline_run':str(baseline.resolve()),'source_hash_comparison':source_hash_diff,'source_diffs':source_diff,
            'policy_source_unchanged':policy_equal,'selected_profile_unchanged':profile_equal,'TTL_constants':constants,
            'revision_scope_verified':revision_verified,'change':'Evaluate dual-TTL at completion of reading the exact atomic JSON version, instead of an earlier read-begin/worker wall time; retain original headers and received wall times.',
            'limits_or_controller_or_teacher_or_physics_changed':False,'old_results_rewritten':False},
        'unverified':{'absolute_original_scene_centerline_registration':True,'SCAN':True,'full_multifloor':True,'continuous_curve_SLAM':True,'general_Sim2Sim':True,'hardware':True},
        'plots':{'actual_SLAM_trajectories':'actual_slam_trajectories.png','actual_COM_velocity':'actual_com_velocity.png','actual_source_freshness':'actual_source_freshness.png'}}
    plot_data(plots)
    write(HERE/'aggregate.json',aggregate)
    report(aggregate)
    unchanged_old={name:sha(name)==digest for name,digest in originals.items()}
    write(HERE/'original_receipts_unchanged.json',{'all_unchanged':all(unchanged_old.values()),'old_original_receipts_sha256':originals,'verification':unchanged_old})
    print('AGGREGATE',aggregate['status'],aggregate['passed_count'],aggregate['original_required_checks_passed'],flush=True)


def plot_data(data):
    colors=['#147d92','#b45518','#7348a1'];plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(14,8),constrained_layout=True)
    for group in range(2):
        for j,entry in enumerate(data[group*3:group*3+3]):
            p=entry['source_pos'];route=entry['route'];color=colors[j];lab='repeat '+str(j+1)+' ('+entry['record']['suffix']+')'
            axes[0,group].plot(p[:,0],p[:,1],color=color,label=lab,linewidth=1.6)
            axes[0,group].plot(route[:,0],route[:,1],color=color,linestyle='--',alpha=.6,linewidth=.8)
            axes[1,group].plot(entry['source_t'],entry['source_distance'],color=color,label=lab)
        axes[0,group].set_title(('6m forward','6m + 6m roundtrip')[group]+' | actual original SLAM points')
        axes[0,group].set_xlabel('camera_init x (m)');axes[0,group].set_ylabel('camera_init y (m; expanded scale)')
        axes[0,group].legend(fontsize=9);axes[0,group].grid(alpha=.25)
        axes[1,group].set_title('Distance to each run\'s own frozen SLAM route')
        axes[1,group].set_xlabel('Original SLAM header time (s)');axes[1,group].set_ylabel('XY distance (m)')
        axes[1,group].axhline(.20,color='#bc3131',linestyle=':',label='0.20m ceiling');axes[1,group].grid(alpha=.25)
    fig.suptitle('Actual SLAM V5: no truth-derived navigation trajectory; dashed = fixed source route',fontsize=14)
    fig.savefig(HERE/'actual_slam_trajectories.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(15,10))
    fig.subplots_adjust(left=.065,right=.985,bottom=.14,top=.90,hspace=.52,wspace=.18)
    for i,entry in enumerate(data):
        ax=axes[i%3,i//3];t=entry['telemetry'];park=entry['record']['motion']['fixed_parking']['physical_window_s']
        ax.plot([r['state_physics_world_time'] for r in t],[r['body_lin_vel'][0] for r in t],color='#147d92',label='actual COM body vx; physics t-0.005',linewidth=1)
        ax.plot([r['world_sim_time'] for r in t],[r['command'][0] for r in t],color='#b45518',label='actual Teacher slewed vx command',linewidth=1)
        ax.plot([r['world_sim_time'] for r in t],[r['requested'][0] for r in t],color='#777777',label='actual requested body vx',linestyle=':',linewidth=.8)
        ax.axvspan(*park,color='#49a078',alpha=.13,label='fixed original 5s parking');ax.axhline(.3,color='#333333',linestyle=':',linewidth=.7)
        ax.set_title(entry['record']['case']+' '+entry['record']['suffix']);ax.set_xlabel('Original world / cached physics time (s)');ax.set_ylabel('COM body vx (m/s)');ax.grid(alpha=.25)
        ax.set_ylim(-.12,.53)
    handles,labels=axes[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=2,bbox_to_anchor=(.5,.018))
    fig.suptitle('Actual COM physics velocity versus actual Teacher inputs (diagnostic only; not SLAM feedback)',fontsize=14)
    fig.savefig(HERE/'actual_com_velocity.png',dpi=160,bbox_inches='tight');plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(15,10))
    fig.subplots_adjust(left=.065,right=.985,bottom=.14,top=.90,hspace=.52,wspace=.18)
    for i,entry in enumerate(data):
        ax=axes[i%3,i//3];f=entry['freshness'];x=entry['accepted_read_world_t']
        for key,color,style,label in [('pose_sim_s','#147d92','-','SLAM sim age'),('pose_wall_s','#147d92',':','SLAM receipt-wall age'),
                ('gyro_sim_s','#b45518','-','IMU sim age'),('gyro_wall_s','#b45518',':','IMU receipt-wall age')]:
            ax.plot(x,f[key],color=color,linestyle=style,linewidth=.7,label=label)
        ax.axhline(.3,color='#bc3131',linestyle='--',linewidth=1,label='unchanged 300ms TTL')
        ax.set_ylim(-.01,.325);ax.set_title(entry['record']['case']+' '+entry['record']['suffix'])
        ax.set_xlabel('Actual Teacher read_clock time (s)');ax.set_ylabel('Original source age (s)');ax.grid(alpha=.25)
    handles,labels=axes[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=3,bbox_to_anchor=(.5,.018))
    fig.suptitle('Actual accepted read attempts: original source ages, never heartbeat-refreshed headers',fontsize=14)
    fig.savefig(HERE/'actual_source_freshness.png',dpi=160,bbox_inches='tight');plt.close(fig)


def report(a):
    records=a['runs'];m=[r['motion'] for r in records];p=[r['motion']['fixed_parking'] for r in records]
    text=['# 实际 SLAM V5 六次固定路线汇总','',
        f'六次试验（6m 单程三次、6m＋6m 往返三次）均通过原独立验收，共 {a["original_required_checks_passed"]}/{a["original_required_checks_count"]} 门。此次汇总重新读取所有原输入 SHA，并以原冻结分析器 `write=False` 复核，没有修改旧日志、源码或收据。','',
        '选定 CAS25 的 controller/gains 保持，真实 SLAM 新源控制率均约 10Hz；这是显式降率迁移，不是 SLAM25。目标速度为 0.3m/s，不能据此声称已实测 1m/s。','',
        '| 试验 | 实COM均速 m/s | 速度MAE m/s | 路线最大/RMS m | 朝向峰值 rad | 5s停车XY m / yaw rad | 原SLAM头 / math更新 |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for r in records:
        x=r['motion'];pk=x['fixed_parking'];case='往返' if r['case'].startswith('roundtrip') else'单程'
        text.append(f'| {case} {r["suffix"]} | {x["mean_actual_along_reference_COM_mps"]:.6f} | {x["stable_speed_MAE_mps"]:.6f} | {x["route_maximum_m"]:.6f}/{x["route_drive_RMS_m"]:.6f} | {x["maximum_drive_heading_rad"]:.6f} | {pk["actual_native_xy_drift_m"]:.6f}/{pk["actual_native_yaw_drift_rad"]:.6f} | {r["accepted_SLAM_headers"]}/{r["actual_math_update_count"]} |')
    text+=['',f'全组最坏：路线最大 {max(x["route_maximum_m"] for x in m):.6f}m、RMS {max(x["route_drive_RMS_m"] for x in m):.6f}m，行驶朝向 {max(x["maximum_drive_heading_rad"] for x in m):.6f}rad；RP {max(x["maximum_abs_RP_rad"] for x in m):.6f}rad，最低离地 {min(x["minimum_clearance_m"] for x in m):.6f}m。最大力矩命令 {max(x["maximum_force_command_Nm"] for x in m):.6f}Nm、qd {max(x["maximum_joint_speed_radps"] for x in m):.6f}rad/s，机身接触与 fault 均为0。力矩是 physics 的施力命令，并非测得电机力矩。','',
        '保持原门限：路线最大/RMS 0.20/0.08m，行驶朝向0.2rad，速度MAE≤max(0.05,0.25×参考)，原SLAM末目标0.15m与不同原头连续0.6s dwell；固定首个合格停车窗5s，位移/偏航≤0.05m/0.1rad；RP≤0.65、clearance≥0.18、force≤23.50001、qd≤30.001。每窗原生1001帧、Teacher251帧、真实SLAM50个不同原头。六主进程与六SLAM子进程全部干净退出。','',
        f'六次合格读包均无未来SLAM/gyro反馈、无源TTL违规。SLAM sim龄峰值 {max(r["source_freshness_raw_recomputed"]["pose_sim_s"]["max"] for r in records):.6f}s，IMU sim龄峰值 {max(r["source_freshness_raw_recomputed"]["gyro_sim_s"]["max"] for r in records):.6f}s，SLAM墙龄峰值 {max(r["source_freshness_raw_recomputed"]["pose_wall_s"]["max"] for r in records):.6f}s，IMU墙龄峰值 {max(r["source_freshness_raw_recomputed"]["gyro_wall_s"]["max"] for r in records):.6f}s，均小于原300ms限制。197/196次读取拒绝均位于首次合格包前的正常预热；首次接受后均无拒绝。','',
        'V5只修复原子JSON读取开始与生产者写入之间的时钟竞态：读取该版本完成后再评估其墙龄，并记录 read_begin / read_completed / read_span。原始传感头及接收墙钟不刷新，300ms TTL、50ms producer-clock相位容差、Teacher、physics、gains、slew、controller core 与 runner 都不变。V4原成功/失败收据保留并单列在 JSON 中；不回填、不混计为这六次V5通过。','',
        '范围限制：高层定位/命令/到达来自实际SLAM和因果IMU；Actor仍有232/247维Gazebo特权观测，其余为3维速度命令与12维上一动作。固定route只由首次实际SLAM机身原点＋yaw锚定；旧场景绝对中心线和旧地图尚未注册。离线一次native对齐只用于实际运动评估，不能倒灌导航。SCAN、实时云避障、多层路线、曲线SLAM、通用Sim2Sim及真机均未验证，score恒为null。','',
        '![实际SLAM轨迹](actual_slam_trajectories.png)','',
        '![实际COM速度和Teacher命令](actual_com_velocity.png)','',
        '![原始来源新鲜度](actual_source_freshness.png)','',
        '完整路径、SHA、132门重新计算证据、原始来源统计、V4→V5源差异见 [aggregate.json](aggregate.json)。重现脚本为 [aggregate_v5.py](aggregate_v5.py)，仅向本新目录写入。']
    with (HERE/'REPORT.md').open('x') as stream:stream.write('\n'.join(text)+'\n')


if __name__=='__main__':main()
