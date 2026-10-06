#!/usr/bin/env python3
"""Read-only actual active-hold comparison; no controller imports or launches.

All new artifacts live below this analysis directory. Existing results are
refused, partial registries are refused, and diagnostics cannot change gates.
"""
import argparse
import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CAMPAIGN = HERE.parent
MODEL = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
HASH_CACHE = {}


def sha(path):
    path = Path(path).resolve()
    stat = path.stat()
    key = (str(path), stat.st_size, stat.st_mtime_ns)
    if key not in HASH_CACHE:
        h = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                h.update(chunk)
        HASH_CACHE[key] = h.hexdigest()
    return HASH_CACHE[key]


def read_json(path):
    return json.loads(Path(path).read_text())


def rows(path):
    with Path(path).open() as stream:
        return [json.loads(x) for x in stream if x.strip()]


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [serial(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError('Nonfinite output is not silently replaced')
    return value


def verify(path, expected, inventory):
    path = Path(path).resolve()
    actual = sha(path)
    if expected is not None and actual != expected:
        raise ValueError('Hash mismatch: ' + str(path))
    inventory[str(path)] = actual
    return actual


def rotations(q):
    q = np.asarray(q, float)
    if q.shape[1] != 4 or not np.isfinite(q).all():
        raise ValueError('Missing native quaternion')
    if np.max(abs(np.sum(q*q, axis=1)-1)) > .02:
        raise ValueError('Invalid native quaternion norm')
    w, x, y, z = (q / np.linalg.norm(q, axis=1)[:, None]).T
    return np.stack((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
        2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
        2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=1).reshape(-1, 3, 3)


def causal_indices(source_time, reference_time):
    # Integer nanoseconds preserve the .005s native phase. Invalid initial
    # states stay -1; never attach a future controller command to them.
    s = np.rint(np.asarray(source_time)*1e9).astype(np.int64)
    t = np.rint(np.asarray(reference_time)*1e9).astype(np.int64)
    return np.searchsorted(s, t, side='right')-1


def window(a, start, width=5., *, diagnostic=False):
    if start is None:
        return dict(status='unverified', reason='No first active_hold declaration; window not tested')
    t = a['t']; idx = np.flatnonzero((t >= start-1e-8) & (t <= start+width+1e-8))
    complete = (len(idx) == round(width/.005)+1 and
                abs(t[idx[0]]-start) <= 1e-8 and abs(t[idx[-1]]-start-width) <= 1e-8 and
                np.all(abs(np.diff(t[idx])-.005) <= 1e-8)) if len(idx) else False
    if not complete:
        return dict(status='unverified', reason='Original fixed window incomplete', start_s=start,
                    end_s=start+width, native_rows=len(idx), diagnostic_only=diagnostic)
    xy = np.linalg.norm(a['position'][idx, :2]-a['position'][idx[0], :2], axis=1)
    dyaw = abs(a['yaw'][idx]-a['yaw'][idx[0]])
    speed = np.linalg.norm(a['origin_body_velocity'][idx, :2], axis=1)
    return dict(status='diagnostic_only' if diagnostic else 'measured_not_a_new_gate',
                window_s=[float(t[idx[0]]), float(t[idx[-1]])], native_rows=len(idx),
                xy_drift_m=float(xy.max()), yaw_drift_rad=float(dyaw.max()),
                peak_origin_planar_speed_mps=float(speed.max()),
                peak_Euler_yaw_rate_radps=float(abs(a['yawdot'][idx]).max()),
                peak_body_wz_radps=float(abs(a['omega'][idx, 2]).max()),
                mean_endpoint_distance_m=float(a['target_distance'][idx].mean()),
                final_endpoint_distance_m=float(a['target_distance'][idx[-1]]),
                diagnostic_only=diagnostic, original_window_never_reselected=True)


def verify_receipt(run, name, expected, inventory):
    path = run/name
    verify(path, expected, inventory)
    receipt = read_json(path)
    if Path(receipt['run']).resolve() != run:
        raise ValueError('Receipt identity is not its actual run')
    for filename, digest in receipt.get('verified_input_source_sha256', {}).items():
        verify(filename, digest, inventory)
    for filename, digest in receipt.get('input_source_sha256', {}).items():
        verify(filename, digest, inventory)
    if receipt.get('independent_arrays'):
        verify(receipt['independent_arrays']['path'], receipt['independent_arrays']['sha256'], inventory)
    return receipt


def load_run(run, name, expected, inventory, role):
    run = Path(run).resolve()
    receipt = verify_receipt(run, name, expected, inventory)
    common = verify_receipt(run, 'summary_truth_pid.json', None, inventory)
    runtime = read_json(run/'runtime_manifest.json')
    profile = read_json(run/'truth_profile.json')
    policy = read_json(run/'policy_manifest.json')
    manifest = read_json(run/'source_manifest.json')
    for filename in ('runtime_manifest.json', 'truth_profile.json', 'policy_manifest.json',
                     'actuator.jsonl', 'telemetry.jsonl', 'control.jsonl', 'worker_result.json',
                     'observations_actions.npz', 'world.sdf'):
        verify(run/filename, None, inventory)
    verify(run/'source_manifest.json', runtime['source_manifest_sha256'], inventory)
    verify(run/'truth_profile.json', runtime['truth_profile_sha256'], inventory)
    if Path(runtime['run']).resolve() != run:
        raise ValueError('Runtime is copied from another run')
    for key, digest in manifest.items():
        source = (run/'sources'/key).resolve()
        if not source.is_relative_to(run/'sources'):
            raise ValueError('Escaping source archive')
        verify(source, digest if isinstance(digest, str) else digest['sha256'], inventory)
    verify(run/'sources/native/libteacher_actuator.so', runtime['native_plugin_sha256'], inventory)
    verify(policy['checkpoint'], MODEL, inventory)
    if policy['checkpoint_sha256'] != MODEL or runtime['frozen_model_sha256'] != MODEL:
        raise ValueError('Frozen model mismatch')
    if policy['inference_device'] != 'cpu' or policy['torch_threads'] != 1:
        raise ValueError('Not the CPU-one-thread plant')
    native = [r for r in rows(run/'actuator.jsonl') if r.get('kind') == 'physics_step']
    control = rows(run/'control.jsonl'); telemetry = rows(run/'telemetry.jsonl')
    get = lambda key: np.asarray([r[key] for r in native], float)
    t = get('t')-get('dt'); R = rotations(get('quaternion_wxyz'))
    if not (np.all(abs(get('dt')-.005) <= 1e-8) and np.all(np.diff(get('iteration')) == 1)
            and np.all(abs(np.diff(t)-.005) <= 1e-8)):
        raise ValueError('Incomplete native200Hz phase')
    com = get('body_lin_vel_com'); omega = get('body_ang_vel')
    origin = com-np.cross(omega, np.asarray(profile['base_com_offset'], float))
    origin_error = float(np.max(abs(origin-get('body_lin_vel_origin'))))
    if origin_error > 1e-9:
        raise ValueError('COM/origin velocity discrepancy')
    yaw = np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0]))
    roll = np.arctan2(R[:, 2, 1], R[:, 2, 2]); pitch = np.arcsin(np.clip(-R[:, 2, 0], -1, 1))
    yawdot = (np.sin(roll)*omega[:, 1]+np.cos(roll)*omega[:, 2])/np.cos(pitch)
    target = np.asarray(profile['route_world_xyz'][-1], float)
    if role != 'old_zero':
        target = np.asarray(profile['active_parking']['target']['position_world_xyz'], float)
    onset = next((r for r in control if r.get('controller_updated') is True and
                  r['mode'] == ('goal_dwell' if role == 'old_zero' else 'capture')), None)
    if onset is None:
        raise ValueError('No original fresh arrival/capture onset')
    hold = next((r for r in control if r.get('controller_updated') is True and
                 r['mode'] == 'active_hold'), None)
    capture = float(onset['feedback_time_s'])
    ct = np.asarray([r['control_t_s'] for r in control], float)
    bound = causal_indices(ct, t)
    a = dict(t=t, position=get('position'), yaw=yaw, yawdot=yawdot, omega=omega,
             origin_body_velocity=origin, origin_world_velocity=np.einsum('nij,nj->ni', R, origin),
             target_distance=np.linalg.norm(get('position')[:, :2]-target[:2], axis=1),
             causal_control_index=bound, actor_world_time=np.asarray([r['world_sim_time'] for r in telemetry]),
             actor_command=np.asarray([r['command'] for r in telemetry], float),
             actor_requested=np.asarray([r['requested'] for r in telemetry], float))
    independent = receipt.get('independent_arrays')
    array_crosschecks = dict(present=bool(independent), missing_not_filled=True)
    if independent:
        with np.load(independent['path'], allow_pickle=False) as arr:
            for key, actual in [('t', t), ('position', a['position']), ('body_omega', omega),
                                ('origin_body_velocity', origin), ('unwrapped_yaw', yaw)]:
                if key in arr:
                    error = float(np.max(abs(arr[key]-actual)))
                    if error > 1e-9:
                        raise ValueError('Derived array is not original native: '+key)
                    array_crosschecks[key+'_maximum_error'] = error
    formal_start = (receipt['checks']['original_first_five_second_parking_unchanged']['original_window_s'][0]
                    if role == 'old_zero' else float(hold['parking_hold_declared_state_time_s']) if hold else None)
    formal = window(a, formal_start)
    diagnostic = window(a, capture+.6, diagnostic=True)
    valid_phase = np.flatnonzero(t >= (formal_start if formal_start is not None else capture)-1e-8)[0]
    i = int(bound[valid_phase])
    phase = dict(native_state_time_s=float(t[valid_phase]), causal_control_index=i,
                 causal_control_mode=control[i]['mode'] if i >= 0 else None,
                 causal_control_compute_time_s=control[i]['control_t_s'] if i >= 0 else None,
                 source_is_before_or_equal_native_state=bool(i >= 0 and ct[i] <= t[valid_phase]+1e-8),
                 controller_at_capture_world_s=float(onset['control_t_s']),
                 declaration_cached_state_offset_s=.005, future_control_binding=False)
    fresh_capture = [r for r in control if r.get('controller_updated') is True and r['mode'] in ('capture', 'active_hold')]
    cmask = a['actor_world_time'] >= float(onset['control_t_s'])-1e-8
    nmask = t >= capture-1e-8
    late = nmask & (t >= t[-1]-20.)
    def describe(x):
        x = np.asarray(x, float)
        return dict(min=float(x.min()), median=float(np.median(x)), mean=float(x.mean()),
                    p95=float(np.quantile(x, .95)), maximum=float(x.max())) if len(x) else None
    diagnostics = dict(capture_onset_state_s=capture, capture_onset_world_s=float(onset['control_t_s']),
        first_hold_state_s=formal_start if role != 'old_zero' else None,
        capture_to_hold_delay_s=float(formal_start-capture) if formal_start is not None else None,
        active_hold_declared=hold is not None, capture_recorded_duration_s=float(t[-1]-capture),
        onset_target_distance_m=float(a['target_distance'][np.flatnonzero(nmask)[0]]),
        final_target_distance_m=float(a['target_distance'][-1]),
        final_yaw_error_rad=float(math.atan2(math.sin(yaw[-1]-profile['path']['heading0_rad']),
                                            math.cos(yaw[-1]-profile['path']['heading0_rad']))),
        full_capture_origin_body_vx=describe(origin[nmask, 0]),
        capture_actor_body_vx=describe(a['actor_command'][cmask, 0]),
        late_twenty_seconds_diagnostic_only=dict(window_s=[float(t[late][0]), float(t[late][-1])],
            origin_body_vx=describe(origin[late, 0]), endpoint_distance=describe(a['target_distance'][late]),
            Euler_yaw_rate=describe(yawdot[late])),
        actor_inference_fraction_after_onset=float(np.mean([r.get('actor_inferred_this_frame') is True
            for r in telemetry if r['world_sim_time'] >= onset['control_t_s']])) )
    if fresh_capture:
        commands = np.asarray([r['command_body'] for r in fresh_capture], float)
        diagnostics['fresh_capture_requested_vx_at_0p06_fraction'] = float(np.mean(abs(commands[:, 0]-.06) <= 1e-8))
        diagnostics['fresh_capture_vx_antiwindup_fraction'] = float(np.mean([r['velocity_PI']['blocked_axis'][0] for r in fresh_capture]))
        diagnostics['fresh_capture_raw_vx'] = describe([r['raw_command_body'][0] for r in fresh_capture])
        diagnostics['fresh_capture_PI_P_x'] = describe([r['velocity_PI']['P'][0] for r in fresh_capture])
        diagnostics['fresh_capture_PI_I_x'] = describe([r['velocity_PI']['I'][0] for r in fresh_capture])
    return dict(run=run, receipt=receipt, common=common, profile=profile, runtime=runtime,
                control=control, telemetry=telemetry, a=a, onset=capture, target=target,
                role=role, formal=formal, diagnostic=diagnostic, diagnostics=diagnostics,
                first_native_phase=phase, derived_array_crosschecks=array_crosschecks)


def validate_registry(plan, registry):
    if not registry or len({r['case_index'] for r in registry}) != len(registry):
        raise ValueError('Empty or duplicated registry')
    for index, row in enumerate(registry):
        case = plan['cases'][index]
        if any(row[k] != case[k] for k in ('case_index', 'role', 'repetition', 'profile_path', 'profile_sha256', 'camera')):
            raise ValueError('Actual registry does not bind prospective case')
    early_stop = len(registry) == 1 and registry[0]['role'] == 'pilot' and registry[0]['status'] != 'passed'
    if not early_stop and len(registry) != len(plan['cases']):
        raise ValueError('Partial actual registry; wait for completed campaign')
    return early_stop


def plot_figures(items, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':9,
                         'axes.spines.top':False, 'axes.spines.right':False})
    colors = ['#222222', '#c7493b', '#227fa4', '#67903f', '#946fb5']
    fig, ax = plt.subplots(3, 2, figsize=(12, 9), constrained_layout=True)
    ymax = max(x['a']['t'][-1]-x['onset'] for x in items)
    for j, item in enumerate(items):
        a=item['a']; x=a['t']-item['onset']; m=x>=-1.
        label='Old zero-command 0b78' if j==0 else item['role']+' '+item['run'].name[-4:]
        metrics=[np.rad2deg(a['yaw']-item['profile']['path']['heading0_rad']),a['target_distance'],
                 np.linalg.norm(a['origin_body_velocity'][:, :2],axis=1)]
        for k,y in enumerate(metrics):
            for c in range(2):ax[k,c].plot(x[m],y[m],color=colors[j],label=label,lw=.8,alpha=.9)
    for k,label in enumerate(('Actual yaw - fixed target (deg)','Distance to fixed endpoint (m)','Actual origin planar speed (m/s)')):
        for c in range(2):
            ax[k,c].set_ylabel(label);ax[k,c].grid(alpha=.2);ax[k,c].axvspan(.6,5.6,color='#dedbd2',alpha=.35)
            ax[k,c].axvline(0,color='#555555',ls=':',lw=.8)
            ax[k,c].set_xlim(-1,8 if c==0 else ymax)
    ax[1,0].axhline(.025,color='gray',ls='--',lw=.8,label='Strict hold entry 25 mm')
    ax[1,1].axhline(.025,color='gray',ls='--',lw=.8)
    for c in range(2):ax[2,c].set_xlabel('Seconds from first fresh goal_dwell / capture native state')
    ax[0,0].legend(fontsize=8);ax[1,0].legend(fontsize=8)
    ax[0,0].set_title('Early matched onset; grey = predeclared diagnostic window')
    ax[0,1].set_title('Full actual recording; no extrapolation of old run')
    fig.suptitle('Fixed-endpoint active capture versus original zero-command stop (truth-feedback simulation)')
    fig.savefig(out/'01_capture_actual_pose_speed.png',dpi=170);plt.close(fig)
    fig, ax = plt.subplots(3,2,figsize=(12,9),constrained_layout=True)
    for j,item in enumerate(items):
        a=item['a']; nx=a['t']-item['onset'];tx=a['actor_world_time']-item['onset']
        label='Old 0b78' if j==0 else item['role']+' '+item['run'].name[-4:]
        actual=np.column_stack((a['origin_body_velocity'][:,:2],a['omega'][:,2]))
        for k in range(3):
            ax[k,0].plot(tx,a['actor_command'][:,k],color=colors[j],label=label,lw=1.)
            ax[k,1].plot(nx,actual[:,k],color=colors[j],label=label,lw=.7)
    for k,name in enumerate(('Body vx (m/s)','Body vy (m/s)','Body omega_z (rad/s)')):
        for c in range(2):
            ax[k,c].set_ylabel(name);ax[k,c].grid(alpha=.2);ax[k,c].set_xlim(-1,ymax)
            ax[k,c].axvspan(.6,5.6,color='#dedbd2',alpha=.35)
    ax[0,0].set_title('Actual Teacher after-slew velocity input, original world stamp')
    ax[0,1].set_title('Original native body-origin / angular measurements, state=t-dt')
    ax[0,0].legend(fontsize=8)
    for c in range(2):ax[2,c].set_xlabel('Seconds from first fresh arrival / capture native state')
    fig.suptitle('Command versus physical response; no filtering or synthetic samples')
    fig.savefig(out/'02_actor_commands_native_response.png',dpi=170);plt.close(fig)
    fig, ax=plt.subplots(2,3,figsize=(12,7),constrained_layout=True)
    names=['xy_drift_m','yaw_drift_rad','peak_origin_planar_speed_mps','peak_Euler_yaw_rate_radps','peak_body_wz_radps']
    limits=[.05,.1,.08,.1,.1]; labels=['XY drift (m)','Yaw drift (rad)','Planar peak (m/s)','Euler yawrate peak (rad/s)','Body wz peak (rad/s)']
    measurements=[]
    for item in items:
        short='Old' if item['role']=='old_zero' else item['role']+' '+item['run'].name[-4:]
        if 'window_s' in item['formal']:measurements.append((short+' formal',item['formal']))
        measurements.append((short+' capture+0.6..5.6 diag',item['diagnostic']))
    for k,key in enumerate(names):
        axes=ax.flat[k]; ys=[d.get(key,np.nan) for _,d in measurements]
        axes.bar(np.arange(len(ys)),ys,color=['#888888' if 'diag' in n else '#c7493b' for n,_ in measurements])
        axes.axhline(limits[k],ls='--',color='#333333',lw=.9,label='Original / new stricter bound')
        axes.set_ylabel(labels[k]);axes.set_xticks(np.arange(len(ys)),[n for n,_ in measurements],rotation=35,ha='right',fontsize=7);axes.grid(axis='y',alpha=.2)
    ax.flat[5].axis('off')
    text=['Formal windows are different declared stages.','Grey bars are diagnostics, never replacement gates.',
          'Original peak limits were not old parking gates.']
    for item in items[1:]:
        text+=['',item['run'].name[-4:]+': '+item['receipt']['status'].upper(),
               'Active hold declared: '+str(item['diagnostics']['active_hold_declared']),
               'Confirmation not run if pilot failed.']
    ax.flat[5].text(0,1,'\n'.join(text),va='top',fontsize=10)
    fig.suptitle('Fixed stage windows; a capture diagnostic cannot declare parking passed')
    fig.savefig(out/'03_fixed_window_stage_metrics.png',dpi=170);plt.close(fig)


def analyze(plan_path, registry_path, output):
    output=Path(output).resolve()
    if not output.is_relative_to(HERE) or output.exists():
        raise ValueError('Only a fresh output directory below this analysis directory is allowed')
    plan=read_json(plan_path);registry=rows(registry_path)
    if plan['schema']!='prospective_tight_S_active_hold_campaign/v3':raise ValueError('Wrong frozen campaign')
    early_stop=validate_registry(plan,registry);inventory={}
    verify(plan_path,None,inventory);verify(registry_path,None,inventory)
    for case in plan['cases']:verify(case['profile_path'],case['profile_sha256'],inventory)
    old=load_run(plan['baseline']['run'],'summary_curve_independent.json',plan['baseline']['receipt_sha256'],inventory,'old_zero')
    items=[old]
    for row in registry:
        item=load_run(row['run'],'summary_active_hold_independent.json',row['receipt_sha256'],inventory,row['role'])
        if item['receipt']['status']!=row['status']:raise ValueError('Registry outcome mismatch')
        if np.max(abs(item['target']-old['target']))>1e-12:raise ValueError('Fixed endpoints differ')
        for original,digest in plan['frozen_sources'].items():
            p=Path(original)
            # Campaign launcher is not a per-run executing dependency. All
            # profile-frozen execution sources are independently checked above.
            if original in item['profile'].get('frozen_source_hashes',{}):
                if item['profile']['frozen_source_hashes'][original]!=digest:raise ValueError('Plan execution source mismatch')
        items.append(item)
    summaries=[]
    for item in items:
        summaries.append(dict(run=str(item['run']),role=item['role'],receipt_status=item['receipt']['status'],
            receipt_sha256=sha(item['run']/('summary_curve_independent.json' if item['role']=='old_zero' else 'summary_active_hold_independent.json')),
            receipt_checks=item['receipt']['checks'],original_common_status=item['common']['status'],
            original_common_checks={k:v['status'] for k,v in item['common']['checks'].items()},
            original_common_UNVERIFIED_preserved=[k for k,v in item['common']['checks'].items() if v['status']=='unverified'],
            formal_window=item['formal'],predeclared_capture_0p6_to_5p6_diagnostic=item['diagnostic'],
            capture_diagnostics=item['diagnostics'],first_cached_native_phase=item['first_native_phase'],
            independent_array_crosschecks=item['derived_array_crosschecks'],runtime=item['runtime']))
    aggregate=dict(schema='actual_fixed_endpoint_active_hold_campaign_comparison/v1',
        created_at_utc=datetime.now(timezone.utc).isoformat(),status='failed' if any(x['receipt']['status']=='failed' for x in items[1:]) else 'passed' if all(x['receipt']['status']=='passed' for x in items[1:]) else 'unverified',
        scope='Read-only comparison, not a new or corrected physical acceptance. Truth feedback; no SLAM/SCAN/multifloor/Isaac/real robot promotion.',
        registry_complete_for_conditional_campaign=True,failed_pilot_stopped_campaign=early_stop,
        planned_new_runs=len(plan['cases']),actual_new_runs=len(registry),unlaunched_confirmations=plan['cases'][len(registry):],
        source_and_input_hashes_all_verified=True,verified_input_sha256=inventory,
        script_sha256=sha(__file__),old_receipts_or_run_inputs_modified=False,
        diagnostic_windows_not_acceptance=True,receipt_missing_arrays_never_filled_into_original_run=True,
        source_phase='Native state t-dt, body COM minus omega cross actual archived COM offset; commands keep original world timestamp; prior compute-time references only.',
        actual_runs=summaries,limitations=['No feedback frequency/gain/global optimality claim.',
            'Absence of declared hold leaves formal fixed parking unverified even when an early capture diagnostic is numerically small.',
            'A persistent low-speed command with little displacement supports a low-command-response limitation; causality and a remedy need a separate prospective test.',
            'Capture is an active motion stage; arbitrary late quiet windows are diagnostics only.'])
    output.mkdir()
    plot_figures(items,output)
    arrays={}
    for j,item in enumerate(items):
        for key,value in item['a'].items():arrays[f'run{j}_{key}']=value
    with (output/'original_actual_series.npz').open('xb') as stream:np.savez_compressed(stream,**arrays)
    with (output/'aggregate.json').open('x') as stream:json.dump(serial(aggregate),stream,ensure_ascii=False,indent=2,allow_nan=False)
    provenance=dict(schema='actual_active_hold_figure_provenance/v1',input_sha256=inventory,
        script_sha256=sha(__file__),figures={p.name:sha(p) for p in sorted(output.glob('*.png'))},
        series_sha256=sha(output/'original_actual_series.npz'),all_native_measurements_plotted=True,
        no_model_inference=True,no_simulation=True,no_synthetic_samples=True,no_downsampling=True)
    with (output/'figure_provenance.json').open('x') as stream:json.dump(provenance,stream,ensure_ascii=False,indent=2)
    report=['# Tight S25：固定终点主动捕获与原零命令停车实际对照','','本报告只读已完成原件。'+str(len(registry))+'个新实际run；'+('先导失败，三次确认未启动。' if early_stop else '登记表已完整。'),
        '旧0b78停车FAILED原件保留；新合同允许有界非零Teacher速度命令，不追认旧零命令门。','',
        '|run / stage|原收据|capture→hold delay s|正式5s XY / yaw|capture+[.6,5.6] XY / yaw（诊断）|',
        '|---|---|---:|---|---|']
    for item in items:
        f=item['formal'];d=item['diagnostic'];fmt=lambda x:(f"{x['xy_drift_m']:.6f}m / {x['yaw_drift_rad']:.6f}rad" if 'window_s'in x else '未声明hold，未验证')
        report.append(f"|{item['run'].name[-4:]} / {item['role']}|{item['receipt']['status']}|{item['diagnostics']['capture_to_hold_delay_s']}|{fmt(f)}|{fmt(d)}|")
    report+=['','## 原始物理与指令事实','']
    for item in items[1:]:
        z=item['diagnostics'];late=z['late_twenty_seconds_diagnostic_only'];report+=[
            f"- {item['run'].name[-4:]} capture有效物理源戳 {z['capture_onset_state_s']:.6f}s；持续记录{z['capture_recorded_duration_s']:.3f}s，首次hold声明：{z['active_hold_declared']}。",
            f"- capture始/末终点误差 {z['onset_target_distance_m']:.6f}/{z['final_target_distance_m']:.6f}m；末yaw误差 {z['final_yaw_error_rad']:.6f}rad。",
            f"- 新鲜capture vx请求在+.06上限的比例 {z.get('fresh_capture_requested_vx_at_0p06_fraction',0):.3%}；实际Teacher after-slew capture vx均值 {z['capture_actor_body_vx']['mean']:.6f}m/s。",
            f"- 最后20s仅作诊断：实际原点body vx均值 {late['origin_body_vx']['mean']:.9f}m/s、median {late['origin_body_vx']['median']:.9f}m/s；目标距离均值 {late['endpoint_distance']['mean']:.6f}m。",
            '- 原common UNVERIFIED保持：'+', '.join(k for k,v in item['common']['checks'].items() if v['status']=='unverified')+'。缺失receipt NPZ未补写回run，全部曲线由原native/telemetry重建。']
    report+=['','这支持当前弱捕获速度包络存在请求非零但实际几乎不前进的响应问题，尚不能单独证明通用Teacher死区、GPU或积分唯一因果。姿态收敛不等于严格25mm目标捕获成功；capture诊断小漂移不等于停车通过。后续候选必须独立登记并实际检验，不能调低入/保持误差或挑安静窗口。',
        '','## 源链与相位','',
        '所有原收据、其input hashes、已有arrays、runtime、逐run source archives、模型和profile都验证。正式停车首个native state仍关联计算时间不晚于该state的旧缓存reference，保留约5ms相位；未向前绑定新的active_hold命令。',
        '原目标/新目标允许已预登记的1e-12m绑定容差；不是事后重设。完整source/input SHA与每项原判据保存在aggregate.json。',
        '','## 实际科研图','',
        '![capture真实姿态与速度](01_capture_actual_pose_speed.png)','',
        '![Teacher命令与实际响应](02_actor_commands_native_response.png)','',
        '![固定阶段窗口](03_fixed_window_stage_metrics.png)','',
        '灰色窗口及灰柱是事前登记capture+[.6,5.6]诊断；从未替代新hold首次正式5s窗。保留全部200Hz运动脉动与50Hz命令原始时标，不插值、合成或执行模型。']
    with (output/'REPORT.md').open('x') as stream:stream.write('\n'.join(report)+'\n')
    return dict(status=aggregate['status'],output=str(output),actual_new_runs=len(registry),
                aggregate_sha256=sha(output/'aggregate.json'),script_sha256=sha(__file__))


def pure_checks():
    t=np.arange(1001)*.005
    a=dict(t=t,position=np.column_stack((.001*t,np.zeros((1001,2)))),yaw=.01*t,
           origin_body_velocity=np.column_stack((np.full(1001,.001),np.zeros((1001,2)))),
           omega=np.zeros((1001,3)),yawdot=np.full(1001,.01),target_distance=np.zeros(1001))
    assert window(a,0)['native_rows']==1001
    assert abs(window(a,0)['xy_drift_m']-.005)<1e-12
    assert window(a,None)['status']=='unverified'
    assert window(a,.005)['status']=='unverified'
    assert causal_indices([.01,.03],[.005,.025,.035]).tolist()==[-1,0,1]
    cases=[dict(case_index=i,role='pilot' if i==0 else 'confirmation',repetition=1 if i==0 else i,
                profile_path=str(i),profile_sha256=str(i),camera=i==0) for i in range(4)]
    assert validate_registry(dict(cases=cases),[{**cases[0],'status':'failed'}])
    try:validate_registry(dict(cases=cases),[{**cases[0],'status':'passed'}])
    except ValueError:pass
    else:raise AssertionError('Partial successful pilot must not become final')
    assert np.allclose(rotations([[1,0,0,0]])[0],np.eye(3))
    return dict(pure_checks=8,status='passed',simulation_or_model_executed=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--final',action='store_true');parser.add_argument('--output',type=Path,default=HERE/'analysis_v3_failure')
    args=parser.parse_args()
    if args.self_test:print(json.dumps(pure_checks()));sys.exit(0)
    if not args.final:raise SystemExit('Explicit --final after parent confirms the registry is stopped is required')
    print(json.dumps(analyze(CAMPAIGN/'active_hold_plan_v3.json',CAMPAIGN/'active_hold_results_v3.jsonl',args.output),ensure_ascii=False))
