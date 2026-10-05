#!/usr/bin/env python3
"""Immutable actual V4 comparison: receipt outcomes never inferred from plots.

No controller/ROS/policy imports. No launch. Only new analysis artifacts.
"""
import argparse
import importlib.util
import json
from pathlib import Path
from datetime import datetime, timezone
import numpy as np

HERE = Path(__file__).resolve().parent
ANALYSIS = HERE.parent
CAMPAIGN = ANALYSIS.parent
HELPER = ANALYSIS/'analyze_active_hold.py'
HELPER_SHA = '07cd7f8756e90dc10db5ecdced5f866eada1834bde0b2251639bbcb2e8a01215'
OLD_V3_RUN = '20261005_082834_truth_pid_S_tight_active_hold_weak_p1_curvature_cascade_pi_active_hold_25hz_7f55'
OLD_V3_RECEIPT_SHA = '24c68c25cd47dfb48c9c41ffe10c3a896237f577ffbcf155f8a5307f32bf2215'
CORE_V4_SHA = '9eca389f54277986656c0ccda3da42f01d9cbc25ed15d99b57cc9a1e51087384'
spec = importlib.util.spec_from_file_location('immutable_raw_diagnostic_helper', HELPER)
h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(h)
if h.sha(HELPER) != HELPER_SHA:
    raise RuntimeError('Read-only helper has changed')


def write_json(path, value):
    with Path(path).open('x') as stream:
        json.dump(h.serial(value), stream, indent=2, ensure_ascii=False, allow_nan=False)


def describe(values):
    x = np.asarray(values, float)
    return None if not len(x) else dict(min=float(x.min()), median=float(np.median(x)),
        mean=float(x.mean()), p95=float(np.quantile(x, .95)), maximum=float(x.max()))


def split_mode_diagnostics(item):
    a = item['a']; control = item['control']; t = a['t']
    modes = np.asarray([r['mode'] for r in control], object)
    ni = a['causal_control_index']; vi = ni >= 0
    # Preserve unknown first state. Never clamp a -1 index to a future row.
    nm = np.full(len(t), 'no_causal_row', object); nm[vi] = modes[ni[vi]]
    ti = h.causal_indices([r['control_t_s'] for r in control], a['actor_world_time'])
    tm = np.full(len(ti), 'no_causal_row', object); good = ti >= 0; tm[good] = modes[ti[good]]
    result = {}
    for mode in ('goal_dwell', 'capture', 'active_hold'):
        fresh = [r for r in control if r.get('controller_updated') is True and r['mode']==mode]
        if not fresh:
            continue
        req = np.asarray([r['command_body'] for r in fresh], float)
        limits = fresh[0].get('parking_command_limits_body')
        result[mode] = dict(fresh_source_updates=len(fresh),
            source_state_interval_s=[fresh[0]['feedback_time_s'], fresh[-1]['feedback_time_s']],
            source_compute_interval_s=[fresh[0]['control_t_s'],fresh[-1]['control_t_s']],
            declared_command_limits_body=limits,
            requested_command_by_axis=[describe(req[:,k]) for k in range(3)],
            causal_native_state_samples=int(np.sum(nm==mode)),
            native_origin_body_velocity_by_axis=[describe(a['origin_body_velocity'][nm==mode,k]) for k in range(3)],
            native_Euler_yawrate=describe(a['yawdot'][nm==mode]),
            actor_after_slew_command_by_axis=[describe(a['actor_command'][tm==mode,k]) for k in range(3)])
        if 'velocity_PI' in fresh[0]:
            result[mode].update(vx_antiwindup_fraction=float(np.mean([r['velocity_PI']['blocked_axis'][0] for r in fresh])),
                raw_body_vx=describe([r['raw_command_body'][0] for r in fresh]),
                PI_P_x=describe([r['velocity_PI']['P'][0] for r in fresh]),
                PI_I_x=describe([r['velocity_PI']['I'][0] for r in fresh]))
        if limits:
            result[mode]['vx_at_own_axis_limit_fraction'] = float(np.mean(abs(abs(req[:,0])-limits[0])<=1e-8))
    return result


def source_binding(item):
    """Profile-frozen but not executed optional observers stay missing, not filled."""
    manifest = h.read_json(item['run']/'source_manifest.json')
    archived = set(v if isinstance(v,str) else v['sha256'] for v in manifest.values())
    expected = item['profile'].get('frozen_source_hashes', {})
    missing = {k:v for k,v in expected.items() if v not in archived}
    current_hashes = {}
    for original in expected:
        p = Path(original)
        current_hashes[original] = h.sha(p) if p.exists() else None
    return dict(expected_profile_frozen_source_hashes=expected,
        actual_executed_archive_manifest_sha256=h.sha(item['run']/'source_manifest.json'),
        source_snapshot_hashes=manifest,
        profile_frozen_sources_without_matching_archive=missing,
        current_working_tree_hashes_diagnostic_only=current_hashes,
        current_working_tree_never_substitutes_original_missing_archive=True)


def prefix_comparison(original, item):
    a=original['a']; b=item['a']; end=min(original['onset'],item['onset'])
    ma=a['t']<end-1e-8; mb=b['t']<end-1e-8
    if not np.array_equal(np.rint(a['t'][ma]*1e9),np.rint(b['t'][mb]*1e9)):
        raise ValueError('Original pre-capture native sampling is not matched')
    keys=('position','yaw','origin_body_velocity','omega')
    return dict(native_prefix_rows=int(np.sum(ma)), end_exclusive_state_time_s=end,
        maximum_absolute_errors={k:float(np.max(abs(a[k][ma]-b[k][mb]))) for k in keys},
        exact_states=all(np.array_equal(a[k][ma],b[k][mb]) for k in keys),
        comparison_is_diagnostic_not_acceptance=True)


def fixed_window_checks(window):
    if 'window_s' not in window:
        return dict(status='unverified',reason='No declared formal hold; never select another quiet window')
    names=['xy_drift_m','yaw_drift_rad','peak_origin_planar_speed_mps',
           'peak_Euler_yaw_rate_radps','peak_body_wz_radps']
    limits=[.05,.1,.08,.1,.1]
    return {n:dict(value=window[n],prospective_limit=l,numerical_within_limit=window[n]<=l)
            for n,l in zip(names,limits)}


def figures(items, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,
        'axes.spines.top':False,'axes.spines.right':False})
    colors=['#222222','#c44c3d','#167ea6','#6b9444','#8868ae','#b18120']
    ymax=max(x['a']['t'][-1]-x['onset'] for x in items)
    fig,axes=plt.subplots(3,2,figsize=(12.5,9),constrained_layout=True)
    for j,item in enumerate(items):
        a=item['a']; x=a['t']-item['onset']; mask=x>=-1
        label=item['label']+' ['+item['receipt']['status'].upper()+']'
        values=[a['target_distance'], np.rad2deg(a['yaw']-item['profile']['path']['heading0_rad']),
                np.linalg.norm(a['origin_body_velocity'][:,:2],axis=1)]
        for k,y in enumerate(values):
            for c in range(2):
                axes[k,c].plot(x[mask],y[mask],color=colors[j],lw=.8,alpha=.86,label=label)
        if item['role'] not in ('old_zero','weak_capture_v3') and 'window_s' in item['formal']:
            delay=item['formal']['window_s'][0]-item['onset']
            axes[0,0].axvline(delay,color=colors[j],lw=.7,ls='--')
    for k,name in enumerate(('Distance to fixed endpoint (m)','Yaw - fixed heading (deg)','Origin planar speed (m/s)')):
        for c in range(2):
            ax=axes[k,c]; ax.set_ylabel(name);ax.grid(alpha=.2);ax.axvline(0,color='gray',ls=':')
            ax.axvspan(.6,5.6,color='#dedbd2',alpha=.35);ax.set_xlim(-1,25 if c==0 else ymax)
    for c in range(2):
        axes[0,c].axhline(.025,color='gray',ls='--',lw=.7)
        axes[2,c].set_xlabel('Seconds from first fresh arrival / capture native state')
    axes[0,0].legend(fontsize=7,loc='upper right');axes[0,0].set_title('Capture and declaration (dashed); grey = diagnostic only')
    axes[0,1].set_title('Full original recording; old line ends when its recording ends')
    fig.suptitle('Tight S25 endpoint response: original zero stop, weak capture, stronger capture')
    fig.savefig(out/'01_endpoint_capture_lag_actual.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(12.5,9),constrained_layout=True)
    for j,item in enumerate(items):
        a=item['a'];tx=a['actor_world_time']-item['onset'];nx=a['t']-item['onset']
        mt=tx>=-1;mn=nx>=-1
        actual=np.column_stack((a['origin_body_velocity'][:,:2],a['omega'][:,2]))
        for k in range(3):
            axes[k,0].plot(tx[mt],a['actor_requested'][mt,k],color=colors[j],ls='--',lw=.8,alpha=.6)
            axes[k,0].plot(tx[mt],a['actor_command'][mt,k],color=colors[j],lw=.9,label=item['label'])
            axes[k,1].plot(nx[mn],actual[mn,k],color=colors[j],lw=.7,label=item['label'])
    for k,name in enumerate(('Body vx (m/s)','Body vy (m/s)','Body omega_z (rad/s)')):
        for c in range(2):
            ax=axes[k,c];ax.set_ylabel(name);ax.set_xlim(-1,25);ax.grid(alpha=.2);ax.axvspan(.6,5.6,color='#dedbd2',alpha=.3)
            ax.axvline(0,color='gray',ls=':',lw=.7)
    axes[0,0].legend(fontsize=7);axes[0,0].set_title('Dashed: requested; solid: actual after-slew Teacher input')
    axes[0,1].set_title('Actual native response; body origin / angular rates')
    for c in range(2):axes[2,c].set_xlabel('Seconds from first fresh arrival / capture state')
    fig.suptitle('Actual 50Hz commands versus original 200Hz measurements; no interpolation')
    fig.savefig(out/'02_requested_teacher_actual_response.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(12.5,8),constrained_layout=True)
    keys=['xy_drift_m','yaw_drift_rad','peak_origin_planar_speed_mps','peak_Euler_yaw_rate_radps','peak_body_wz_radps']
    bounds=[.05,.1,.08,.1,.1];names=['XY drift (m)','Yaw drift (rad)','Planar peak (m/s)','Euler yawrate peak (rad/s)','Body wz peak (rad/s)']
    for k,key in enumerate(keys):
        for j,item in enumerate(items):
            if key in item['formal']:
                axes.flat[k].bar(j,item['formal'][key],color=colors[j],alpha=.8)
            else:axes.flat[k].text(j,0,'NO HOLD',rotation=90,ha='center',va='bottom',fontsize=7)
        ax=axes.flat[k];ax.axhline(bounds[k],color='#333333',ls='--',lw=.8)
        ax.set_xticks(range(len(items)),[i['label']+'\n'+i['receipt']['status'].upper() for i in items],rotation=35,ha='right',fontsize=7)
        ax.set_ylabel(names[k]);ax.grid(axis='y',alpha=.2)
    ax=axes.flat[5]
    for j,item in enumerate(items):
        delay=item['diagnostics']['capture_to_hold_delay_s']
        if item['role']=='old_zero':ax.text(j,0,'ZERO STOP',rotation=90,ha='center',va='bottom',fontsize=7)
        elif delay is None:ax.text(j,0,'NO HOLD',rotation=90,ha='center',va='bottom',fontsize=7)
        else:ax.bar(j,delay,color=colors[j],alpha=.8)
    ax.set_ylabel('Capture -> first declared hold (s)');ax.set_xticks(range(len(items)),[i['label'] for i in items],rotation=35,ha='right',fontsize=7)
    ax.grid(axis='y',alpha=.2)
    fig.suptitle('First formal 5s windows: numerical diagnostic bars cannot upgrade missing source evidence')
    fig.savefig(out/'03_formal_window_limits_and_delay.png',dpi=170);plt.close(fig)


def analyze(plan_path, registry_path, output):
    out=Path(output).resolve()
    if not out.is_relative_to(ANALYSIS) or any((out/x).exists() for x in ('aggregate.json','REPORT.md','original_actual_series.npz','figure_provenance.json')):
        raise ValueError('New analysis outputs only; existing artifacts refused')
    plan=h.read_json(plan_path);registry=h.rows(registry_path)
    expected_schema='prospective_tight_S_active_hold_campaign/v4r2' if 'v4r2' in Path(plan_path).name else 'prospective_tight_S_active_hold_campaign/v4'
    if plan['schema']!=expected_schema:raise ValueError('Not the explicit prospective campaign')
    if len(registry)!=len(plan['cases']) or len({r['case_index'] for r in registry})!=len(registry):
        raise ValueError('Final requires all prospectively planned actual registry rows')
    inv={};h.verify(__file__,None,inv);h.verify(HELPER,HELPER_SHA,inv)
    h.verify(plan_path,None,inv);h.verify(registry_path,None,inv)
    for case,row in zip(plan['cases'],registry):
        for key in ('case_index','role','repetition','profile_path','profile_sha256','camera'):
            if case[key]!=row[key]:raise ValueError('Registry case mismatch '+key)
        h.verify(case['profile_path'],case['profile_sha256'],inv)
    old=h.load_run(plan['baseline']['run'],'summary_curve_independent.json',plan['baseline']['receipt_sha256'],inv,'old_zero')
    old['label']='V1 zero 0b78'
    root=old['run'].parent
    weak=h.load_run(root/OLD_V3_RUN,'summary_active_hold_independent.json',OLD_V3_RECEIPT_SHA,inv,'weak_capture_v3')
    weak['label']='V3 weak 7f55';items=[old,weak];actual=[]
    prior=plan.get('prior_accepted_same_controller_pilot')
    if prior:
        pilot=h.load_run(prior['run'],'summary_active_hold_independent.json',prior['receipt_sha256'],inv,'accepted_prior_pilot')
        if pilot['receipt']['status']!='passed' or prior['controller_sha256']!=CORE_V4_SHA:raise ValueError('Invalid prior-pilot binding')
        pilot['label']='V4 pilot '+pilot['run'].name[-4:];items.append(pilot);actual.append(pilot)
    for row in registry:
        item=h.load_run(row['run'],'summary_active_hold_independent.json',row['receipt_sha256'],inv,row['role'])
        if item['receipt']['status']!=row['status']:raise ValueError('Original status mismatch')
        if item['receipt']['schema']!='independent_truth_teacher_active_endpoint_hold/v4':raise ValueError('Wrong executed receipt version')
        if np.max(abs(item['target']-old['target']))>1e-12:raise ValueError('Not the same fixed endpoint')
        if CORE_V4_SHA not in item['profile']['frozen_source_hashes'].values():raise ValueError('Unexpected controller source')
        item['label']=('V4r2' if prior else 'V4')+' '+row['role']+' '+str(row['repetition'])+' '+item['run'].name[-4:]
        actual.append(item);items.append(item)
    result=[]
    for item in items:
        sources=source_binding(item)
        if item in actual and item['receipt']['status']=='passed' and sources['profile_frozen_sources_without_matching_archive']:
            raise ValueError('Passed source receipt has a missing expected source')
        formal=item['formal']
        m=item['receipt'].get('metrics',{}).get('parking',{})
        errors={}
        for key in ('xy_drift_m','yaw_drift_rad','peak_origin_planar_speed_mps','peak_Euler_yaw_rate_radps','peak_body_wz_radps'):
            if key in m and key in formal:
                errors[key]=abs(float(m[key])-float(formal[key]))
                if errors[key]>1e-9:raise ValueError('Receipt parking metric disagrees with native '+key)
        result.append(dict(label=item['label'],run=str(item['run']),role=item['role'],original_receipt_status=item['receipt']['status'],
            original_receipt_sha256=h.sha(item['run']/('summary_curve_independent.json' if item['role']=='old_zero' else 'summary_active_hold_independent.json')),
            original_checks=item['receipt']['checks'],source_binding=sources,
            original_common_status=item['common']['status'],original_common_checks={k:v['status'] for k,v in item['common']['checks'].items()},
            original_common_UNVERIFIED=[k for k,v in item['common']['checks'].items() if v['status']=='unverified'],
            formal_native_window_diagnostic=formal,formal_numerical_limits_only=fixed_window_checks(formal),
            independent_receipt_metric_errors=errors,predeclared_capture_0p6_to_5p6_diagnostic=item['diagnostic'],
            mode_diagnostics=split_mode_diagnostics(item),capture_to_hold_delay_s=item['diagnostics']['capture_to_hold_delay_s'],
            onset_state_s=item['onset'],onset_compute_s=item['diagnostics']['capture_onset_world_s'],
            final_target_distance_m=item['diagnostics']['final_target_distance_m'],first_cached_native_phase=item['first_native_phase'],
            independent_arrays_crosscheck=item['derived_array_crosschecks'],
            matched_original_drive_prefix=prefix_comparison(old,item),runtime=item['runtime']))
    statuses=[x['receipt']['status'] for x in actual]
    status='failed' if 'failed' in statuses else 'passed' if all(s=='passed' for s in statuses) else 'unverified'
    aggregate=dict(schema='actual_active_endpoint_hold_V4_campaign_diagnostic/v1',created_at_utc=datetime.now(timezone.utc).isoformat(),
        status=status,registry_stopped_and_complete=True,actual_registry_rows=len(registry),accepted_prior_pilot=prior,
        outcomes={s:statuses.count(s) for s in ('passed','failed','unverified')},actual_runs=result,
        source_input_sha256=inv,script_sha256=h.sha(__file__),original_receipts_and_inputs_modified=False,
        all_referenced_existing_hashes_verified=True,missing_original_archives_not_replaced=True,
        same_mathematical_controller_for_new_cases=CORE_V4_SHA,
        scope='Truth feedback S25 calibration: new active capture and hold parking semantics, not old zero-command gate or SLAM/SCAN/Sim2Sim/global optimality.',
        policy_actor_still_CPU_continuous_with_native_PD=True,diagnostic_windows_not_acceptance=True,
        source_phase='native state t-dt; origin=COM-omega cross actual COM offset; prior compute-time references only; no future cached-state binding',
        limitations=['Original common zero-command parking remains UNVERIFIED for active nonzero hold; original receipt preserved.',
            'A missing frozen optional observer archive leaves the complete-evidence gate UNVERIFIED, even when native diagnostic values are within limits.',
            'Repeated deterministic fixture data are limited repeatability evidence, not domain randomization or robust global optimality.',
            'Capture gains and velocity envelope changed prospectively; their individual causal contributions are not separated.',
            'Predeclared capture+[.6,5.6] is only a diagnostic, never a replacement formal parking window.'])
    out.mkdir(parents=True,exist_ok=True)
    figures(items,out)
    arr={f'run{j}_{key}':v for j,item in enumerate(items) for key,v in item['a'].items()}
    with (out/'original_actual_series.npz').open('xb') as stream:np.savez_compressed(stream,**arr)
    write_json(out/'aggregate.json',aggregate)
    write_json(out/'figure_provenance.json',dict(schema='actual_active_endpoint_hold_figure_provenance/v2',input_sha256=inv,
        script_sha256=h.sha(__file__),figures={p.name:h.sha(p) for p in out.glob('*.png')},
        series_sha256=h.sha(out/'original_actual_series.npz'),native200Hz_all_samples=True,command50Hz_original_samples=True,
        no_interpolation=True,no_model_inference=True,no_simulation=True,no_filtering_or_synthetic_samples=True))
    report=['# Tight S25 新主动捕获与保持：原件诊断对照','',
        f"登记完整：本次{len(registry)}轮；原验收{' + '.join(str(aggregate['outcomes'][s])+' '+s.upper() for s in ('passed','failed','unverified'))}。"+('包括事前绑定的旧V4已通过先导。' if prior else ''),
        '原V1零命令停车FAILED、V3弱捕获FAILED全部保留。新合同允许有界非零Teacher命令：不能追认旧零命令停车，也不证明SLAM、Sim2Sim或全局最优。','',
        '|原件|原收据|捕获→首次hold s|正式5s XY/yaw|正式峰值 planar/Euler|缺源归档|',
        '|---|---|---:|---|---|---|']
    for r in result:
        f=r['formal_native_window_diagnostic'];fmt=f"{f['xy_drift_m']:.6f}m / {f['yaw_drift_rad']:.6f}rad" if 'window_s' in f else '未声明hold，未验证'
        peak=f"{f['peak_origin_planar_speed_mps']:.6f}m/s / {f['peak_Euler_yaw_rate_radps']:.6f}rad/s" if 'window_s' in f else '未验证'
        missing=', '.join(Path(k).name for k in r['source_binding']['profile_frozen_sources_without_matching_archive']) or '无'
        report.append(f"|{r['label']}|{r['original_receipt_status']}|{r['capture_to_hold_delay_s']}|{fmt}|{peak}|{missing}|")
    report+=['','## 停车语义与相位','',
        'V4捕获改为位置P=.8、D=.2、世界XY参考模长上限.05m/s、body命令上限[.15,.07,.1]。保持仍用弱P=.18、D=.2与原[.06,.04,.1]上限；25mm入、30mm保持、fresh .6s驻留、300ms保护和正式5s漂移/峰值标准保持。模型、PD、动作语义未改变。',
        '目标是事前固定曲线终点，不是到达时的测量姿态。capture是运动阶段，只有首次真实active_hold声明才开启正式窗。capture+[.6,5.6]仅诊断，完整数值见aggregate；不得选更晚安静窗。',
        '首个正式native state关联时间不晚于state的缓存capture行，保留5ms相位；新active_hold命令计算在约5ms后，不回绑定到更早状态。',
        '新V4缺可选capture.py归档的三条原件保持UNVERIFIED。当前源码或其他run的副本不能替代运行时缺失归档；图中即使物理曲线重合也不能升级验收。','',
        '## 限定结论','',
        '旧弱捕获末段vx请求约.06而原点实际vx近零、目标残差约66mm；新捕获包络使实际接近固定目标并声明hold。该配置对照支持当前捕获阶段的改进，未区分位置增益和速度包络各自的因果贡献，也不证明通用Teacher死区或GPU原因。',
        '既有common中仅适用原零速度停车的UNVERIFIED原样保留；独立主动保持收据是不同明示合同。重复是相同冻结场景的有限证据，不能扩展到任意S曲率、SLAM闭环或实体机器人。','',
        '## 实际科研图','',
        '![固定目标与捕获延迟](01_endpoint_capture_lag_actual.png)','',
        '![请求、Teacher输入和实际响应](02_requested_teacher_actual_response.png)','',
        '![正式窗口和峰值](03_formal_window_limits_and_delay.png)','',
        '使用全部原生200Hz与Teacher 50Hz样本，无插值、滤波、合成或模型运行。旧线条到记录终点即结束。原始输入和归档源SHA、现有独立NPZ交叉核对、逐门原状态均在aggregate.json。']
    with (out/'REPORT.md').open('x') as stream:stream.write('\n'.join(report)+'\n')
    return dict(status=status,out=str(out),outcomes=aggregate['outcomes'],aggregate_sha256=h.sha(out/'aggregate.json'),script_sha256=h.sha(__file__))


def pure_checks():
    assert h.pure_checks()['status']=='passed'
    assert fixed_window_checks({'status':'unverified'})['status']=='unverified'
    w=dict(xy_drift_m=.05,yaw_drift_rad=.1,peak_origin_planar_speed_mps=.08,peak_Euler_yaw_rate_radps=.1,peak_body_wz_radps=.1)
    assert all(x['numerical_within_limit'] for x in fixed_window_checks(w).values())
    w['xy_drift_m']=.050001;assert not fixed_window_checks(w)['xy_drift_m']['numerical_within_limit']
    assert h.causal_indices([.01,.03],[.005,.025,.035]).tolist()==[-1,0,1]
    return dict(status='passed',pure_checks=12,no_actual_model_or_simulation=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--self-test',action='store_true');p.add_argument('--final',action='store_true')
    p.add_argument('--plan',type=Path,default=CAMPAIGN/'active_hold_plan_v4.json')
    p.add_argument('--registry',type=Path,default=CAMPAIGN/'active_hold_results_v4.jsonl')
    p.add_argument('--output',type=Path,default=HERE)
    args=p.parse_args()
    if args.self_test:print(json.dumps(pure_checks()));raise SystemExit(0)
    if not args.final:raise SystemExit('Wait for stopped complete registry, then --final')
    print(json.dumps(analyze(args.plan,args.registry,args.output),ensure_ascii=False))
