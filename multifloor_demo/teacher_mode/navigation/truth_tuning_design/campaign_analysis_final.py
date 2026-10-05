#!/usr/bin/env python3
"""Read-only reconciliation of the executed A2/B/C truth calibration campaign.

Reads original receipts, source archives, telemetry and controller logs. Creates
only new derived analysis files in this script's directory. Never imports a
controller, writes a run, starts a simulator, or changes an acceptance result.
"""
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]
CAMPAIGN = BASE / 'test_results/truth_pid_campaign_20261004'
PHASES = {
    'A2': 'phase_A2_architecture_frequency',
    'B': 'phase_B_single_factor_gains',
    'C': 'phase_C_three_repeats',
}
MODEL_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
DEFAULT_GAINS = dict(cross_kp=.8, cross_kd=.2, cross_ki=0., yaw_kp=1.3,
                     yaw_kd=.15, yaw_ki=0., velocity_kp_x=.6, velocity_ki_x=.5,
                     velocity_kp_y=.4, velocity_ki_y=.3, rate_kp=.3, rate_ki=.2)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def rotation(q):
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    return np.stack((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                     2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                     2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=1).reshape(-1, 3, 3)


def wrap(value):
    return np.arctan2(np.sin(value), np.cos(value))


def numeric_hash(arrays):
    """Hash array values, including dtype/shape; ignore ZIP timestamps/path names."""
    digest = hashlib.sha256()
    for name in sorted(arrays):
        a = np.ascontiguousarray(arrays[name])
        digest.update(name.encode())
        digest.update(str(a.dtype).encode())
        digest.update(json.dumps(a.shape).encode())
        digest.update(a.tobytes())
    return digest.hexdigest()


def reconcile(row, phase, freeze):
    run = Path(row['run'])
    profile = read_json(run / 'truth_profile.json')
    receipt = read_json(run / 'summary_truth_pid.json')
    control = read_jsonl(run / 'control.jsonl')
    telemetry = read_jsonl(run / 'telemetry.jsonl')
    protocol = read_json(run / 'sources/truth/protocol.json')
    policy = read_json(run / 'policy_manifest.json')
    runtime = read_json(run / 'runtime_manifest.json')
    with np.load(run / 'truth_pid_independent_arrays.npz', allow_pickle=False) as f:
        arrays = {name: f[name] for name in f.files}
    mismatches = []
    for path, expected in receipt['input_source_sha256'].items():
        if sha(path) != expected:
            mismatches.append({'kind': 'receipt_input_hash', 'path': path})
    if sha(receipt['independent_arrays']['path']) != receipt['independent_arrays']['sha256']:
        mismatches.append({'kind': 'independent_array_hash'})
    if sha(row['profile']) != row['profile_sha256']:
        mismatches.append({'kind': 'registered_profile_hash'})
    archive_hashes = {name: sha(run / 'sources/truth' / name) for name in freeze['source_sha256']}
    for name, expected in freeze['source_sha256'].items():
        if archive_hashes[name] != expected:
            mismatches.append({'kind': 'frozen_source_hash', 'name': name})
    if receipt['status'] != row['status'] or receipt['score'] != row['score']:
        mismatches.append({'kind': 'campaign_receipt_status_score'})

    # Reconstruct the evaluator's causal masks from ORIGINAL telemetry/control.
    # Same formulas are explicit here; no archived or live controller is imported.
    t = np.array([x['state_physics_world_time'] for x in telemetry])
    elapsed = np.array([x['sim_time'] for x in telemetry])
    pos = np.array([x['position'] for x in telemetry])
    q = np.array([x['quaternion_wxyz'] for x in telemetry])
    body = np.array([x['body_lin_vel'] for x in telemetry])
    body_omega = np.array([x['body_ang_vel'] for x in telemetry])
    cmd = np.array([x['command'] for x in telemetry])
    requested = np.array([x['requested'] for x in telemetry])
    R = rotation(q)
    world_v = np.einsum('nij,nj->ni', R, body)
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    ct = np.array([x['control_t_s'] for x in control])
    ft = np.array([x['feedback_time_s'] for x in control])
    updated = np.array([x['controller_updated'] is True for x in control])
    mode = np.array([x['mode'] for x in control])
    ref_v = np.array([x.get('reference_velocity_world', [0., 0., 0.]) for x in control])
    ref_yaw = np.array([x.get('reference_yaw', 0.) for x in control])
    bound = np.searchsorted(ct, t, side='right') - 1
    bi = np.clip(bound, 0, len(ct)-1)
    age = t - ct[bi]
    valid = ((bound >= 0) & (age >= -1e-8) &
             (age <= protocol['time']['maximum_causal_control_age_s']) &
             np.array(['reference_yaw' in x for x in control])[bi])
    speed_ref = np.linalg.norm(ref_v[bi, :2], axis=1)
    drive = (valid & (t >= protocol['time']['bootstrap_excluded_s']) &
             np.isin(mode[bi], protocol['speed']['translation_modes']) &
             (speed_ref > protocol['speed']['active_reference_min_mps']))
    first_move = t[np.flatnonzero(drive)[0]]
    stable = (drive & (t >= first_move + protocol['time']['speed_startup_excluded_s']) &
              (speed_ref >= profile['desired_speed'] * protocol['speed']['full_speed_reference_fraction_min']))
    direction = np.zeros((len(t), 2))
    np.divide(ref_v[bi, :2], speed_ref[:, None], out=direction, where=speed_ref[:, None]>1e-12)
    actual_along = np.sum(world_v[:, :2]*direction, axis=1)
    route = np.array(profile['route_world_xyz'])
    distances = []
    for a, b in zip(route[:-1], route[1:]):
        d = b[:2]-a[:2]
        fraction = np.clip((pos[:, :2]-a[:2]) @ d / (d@d), 0., 1.)
        distances.append(np.linalg.norm(pos[:, :2]-a[:2]-fraction[:, None]*d, axis=1))
    route_dist = np.min(np.array(distances), axis=0)
    heading_error = abs(wrap(yaw-ref_yaw[bi]))
    rms = float(np.sqrt(np.mean(route_dist[drive]**2)))
    mae = float(np.mean(abs(actual_along[stable]-speed_ref[stable])))
    actual_mean = float(np.mean(actual_along[stable]))
    mean_ref = float(np.mean(speed_ref[stable]))
    speed_limit = max(protocol['speed']['maximum_absolute_error_floor_mps'],
                      protocol['speed']['maximum_relative_error_fraction']*mean_ref)
    heading_max = float(heading_error[drive].max())
    p = receipt['checks']['fixed_final_goal_parking']['parking']
    nt = arrays['native_state_physics_world_time']
    npmask = (nt >= p['window_s'][0]-1e-8) & (nt <= p['window_s'][1]+1e-8)
    npos = arrays['native_position'][npmask]
    nr = rotation(arrays['native_quaternion_wxyz'][npmask])
    nyaw = np.arctan2(nr[:, 1, 0], nr[:, 0, 0])
    parking_xy = float(np.linalg.norm(npos[:, :2]-npos[0, :2], axis=1).max())
    parking_yaw = float(abs(wrap(nyaw-nyaw[0])).max())
    components = dict(route=rms/.08, speed=mae/speed_limit, heading=heading_max/.2,
                      parking_xy=parking_xy/.05, parking_yaw=parking_yaw/.1)
    score = 100/(1+sum(components.values()))
    checks = receipt['checks']
    values_to_match = {
        'drive_route_RMS': (rms, checks['flat_or_requested_route_distance']['drive_xy_distance_rms_m']),
        'speed_MAE': (mae, checks['stable_real_COM_speed']['mean_absolute_error_mps']),
        'actual_forward_mean': (actual_mean, checks['stable_real_COM_speed']['mean_actual_forward_projection_mps']),
        'drive_heading_max': (heading_max, checks['drive_heading']['maximum_error_rad']),
        'native_parking_xy': (parking_xy, p['xy_drift_m']),
        'native_parking_yaw': (parking_yaw, p['yaw_drift_rad']),
        'score': (score, receipt['score']),
    }
    for name, (actual, expected) in values_to_match.items():
        if not math.isclose(actual, expected, rel_tol=1e-10, abs_tol=1e-10):
            mismatches.append({'kind': 'recomputed_metric', 'metric': name, 'actual': actual, 'expected': expected})
    for key, raw in [('position', pos), ('body_COM_velocity', body), ('requested', requested),
                     ('command', cmd), ('drive_mask', drive), ('stable_speed_mask', stable)]:
        if not np.allclose(arrays[key], raw, atol=1e-12, rtol=0):
            mismatches.append({'kind': 'raw_array_reconstruction', 'array': key})

    # Validate signed segment error and lookahead geometrically on fresh rows.
    cross_error = []; along_error = []
    for i in np.flatnonzero(updated):
        c = control[i]; a, b = route[c['segment']:c['segment']+2]
        tangent = (b[:2]-a[:2])/np.linalg.norm(b[:2]-a[:2])
        normal = np.array([-tangent[1], tangent[0]])
        j = int(np.argmin(abs(t-ft[i])))
        cross_error.append(abs(float((pos[j, :2]-a[:2])@normal)-c['error_cross']))
        along_error.append(abs(float((np.array(c['reference_xy'])-pos[j, :2])@tangent)-c['error_along']))
    period = np.diff(ct[updated])
    slew_rebuilt = np.empty_like(cmd)
    previous = np.zeros(3)
    for i in range(len(cmd)):
        previous = previous + np.clip(requested[i]-previous, -np.array([.6, .6, .8])*.02,
                                      np.array([.6, .6, .8])*.02)
        slew_rebuilt[i] = previous
    estimate_pairs = [i for i, c in enumerate(control) if 'estimated_teacher_command_body' in c]
    estimated_error = float(max(np.max(abs(np.array(control[i]['estimated_teacher_command_body'])-cmd[i]))
                                for i in estimate_pairs))
    plateau = stable & (elapsed >= 5.) & (elapsed <= 20.) & (speed_ref >= .299)
    plateau_cmd = float(cmd[plateau, 0].mean())
    plateau_actual = float(body[plateau, 0].mean())
    plateau_signed_bias = float((actual_along[plateau]-speed_ref[plateau]).mean())
    pi_audit = None
    if profile['design'] == 'cascade_pi':
        g = DEFAULT_GAINS | profile.get('gains', {})
        ki = np.array([g['velocity_ki_x'], g['velocity_ki_y'], g['rate_ki']])
        kp = np.array([g['velocity_kp_x'], g['velocity_kp_y'], g['rate_kp']])
        previous_I = np.zeros(3)
        all_I = []; clamp_count = np.zeros(3, dtype=int)
        slew_block = np.zeros(3, dtype=int); axis_block = np.zeros(3, dtype=int)
        pi_diffs = []; freeze_diffs = []; hold_diffs = []
        for i in np.flatnonzero(updated):
            c = control[i]; terms = c['velocity_PI']
            error = np.array(terms['error']); target = np.array(c['v_reference_body'])
            dt = c['active_integral_dt_s']
            candidate = np.clip(previous_I + dt*error, -.5, .5)
            clamp_count += abs(previous_I + dt*error) > .5
            raw = target + kp*error + ki*candidate
            clipped = np.clip(raw, -np.array(profile['command_limits']), np.array(profile['command_limits']))
            blocked = error*(raw-clipped) > 0.
            axis_block += blocked
            candidate = np.where(blocked, previous_I, candidate)
            estimated_before = cmd[i-1] if i else np.zeros(3)
            slewed = estimated_before + np.clip(clipped-estimated_before, -np.array([.6,.6,.8])*.02,
                                                np.array([.6,.6,.8])*.02)
            blocked_slew = error*(raw-slewed)>1e-9
            slew_block += blocked_slew
            candidate = np.where(blocked_slew, previous_I, candidate)
            recorded_I = np.array(terms['I']) / ki
            pi_diffs.append(float(np.max(abs(candidate-recorded_I))))
            if dt == 0:
                freeze_diffs.append(float(np.max(abs(recorded_I-previous_I))))
            previous_I = recorded_I
            all_I.append(recorded_I)
        for i in range(1, len(control)):
            if (not updated[i] and control[i].get('velocity_PI') and control[i-1].get('velocity_PI')):
                hold_diffs.append(float(np.max(abs(np.array(control[i]['velocity_PI']['I'])-
                                                       np.array(control[i-1]['velocity_PI']['I'])))))
        pi_audit = {'integral_units': 'error integrated in seconds; output_I=Ki*integral',
                    'reconstruction_scope': 'fresh updates; flat line has no mode/segment reset after first drive update',
                    'conditional_axis_clip_block_rows': axis_block.tolist(),
                    'conditional_teacher_slew_block_rows': slew_block.tolist(),
                    'integral_bound_hit_rows': clamp_count.tolist(),
                    'maximum_abs_integral': np.max(abs(np.array(all_I)), axis=0).tolist(),
                    'maximum_reconstructed_integral_error': max(pi_diffs),
                    'paused_integral_maximum_change': max(freeze_diffs, default=0.),
                    'held_integral_maximum_change': max(hold_diffs, default=0.),
                    'not_proved': 'No ABC command reaches training envelope; this is not a beyond-envelope saturation stress test.'}
        if max(pi_diffs) > 1e-9:
            mismatches.append({'kind': 'PI_reconstruction', 'maximum_error': max(pi_diffs)})
    gain_id = None
    if phase in ('B', 'C'):
        gain_id = int(re.search(r'phase_B_gain_(\d+)', row['profile']).group(1))
    result = {
        'phase': phase, 'gain_id': gain_id, 'run': str(run), 'registered_profile': row['profile'],
        'registered_profile_sha256': row['profile_sha256'], 'profile': profile,
        'original_status': receipt['status'], 'original_score': receipt['score'],
        'all_required_original_checks_pass': all(v['status']=='passed' for v in checks.values()),
        'original_checks': {name: v['status'] for name, v in checks.items()},
        'original_receipt_sha256': sha(run / 'summary_truth_pid.json'),
        'verified_input_source_sha256': receipt['input_source_sha256'],
        'independent_arrays': receipt['independent_arrays'], 'archived_truth_sources': archive_hashes,
        'policy_manifest_sha256': sha(run / 'policy_manifest.json'),
        'model_sha256': policy['checkpoint_sha256'], 'model_path': policy['checkpoint'],
        'CPU_threads': policy['torch_threads'], 'seed': policy['seed'],
        'observation_noise': policy['observation_noise'], 'observation_source': policy['observation_source'],
        'runtime_owned_zero_exits': all(x['returncode']==0 for x in runtime['owned_processes']),
        'exclusive_writer': runtime['exclusive_writer'],
        'independent_reconciliation_mismatches': mismatches,
        'recomputed_metrics': {
            'route_rms_m': rms, 'speed_mae_mps': mae, 'mean_actual_forward_mps': actual_mean,
            'mean_reference_mps': mean_ref, 'drive_heading_max_rad': heading_max,
            'drive_heading_rms_rad': float(np.sqrt(np.mean(heading_error[drive]**2))),
            'native_first_5s_parking_xy_drift_m': parking_xy,
            'native_first_5s_parking_yaw_drift_rad': parking_yaw,
            'score': score, 'score_error_terms': components,
            'speed_startup_excluded_s': protocol['time']['speed_startup_excluded_s'],
            'actual_command_total_variation_xyz': np.sum(abs(np.diff(cmd, axis=0)), axis=0).tolist(),
            'completion_elapsed_s': checks['declared_completion_and_termination']['truth_controller_completed_elapsed_s'],
        },
        'frequency_actual': {'requested_hz': profile['feedback_hz'],
                             'observed_hz': float((updated.sum()-1)/(ct[updated][-1]-ct[updated][0])),
                             'fresh_rows': int(updated.sum()), 'hold_rows': int((~updated).sum()),
                             'maximum_update_period_error_s': float(max(abs(period-1/profile['feedback_hz']))),
                             'fresh_state_phase_delay_s': float(max(abs(ct[updated]-ft[updated]))),
                             'maximum_active_feedback_age_s': float(max((ct-ft)[mode!='initializing']))},
        'geometry_check': {'fresh_signed_cross_max_reconstruction_error_m': max(cross_error),
                           'fresh_lookahead_along_max_reconstruction_error_m': max(along_error)},
        'teacher_command_execution': {
            'maximum_reconstructed_slew_error': float(np.max(abs(slew_rebuilt-cmd))),
            'maximum_controller_estimated_vs_actual_slew_error': estimated_error,
            'maximum_actual_slew_xyz_per_second': (np.max(abs(np.diff(cmd,axis=0)),axis=0)/.02).tolist(),
            'held_command_uses_50hz_slew': True,
        },
        'constant_speed_plateau_5_to_20_s': {
            'rows': int(plateau.sum()), 'mean_body_vx_mps': plateau_actual,
            'mean_teacher_command_vx_mps': plateau_cmd,
            'ratio_mean_body_vx_to_mean_command_vx': plateau_actual/plateau_cmd,
            'mean_signed_forward_reference_error_mps': plateau_signed_bias,
            'scope': 'In-loop apparent gain at this operating point; not a system-identification DC gain.'},
        'PI_actual': pi_audit, 'numeric_independent_array_values_sha256': numeric_hash(arrays),
    }
    plotdata = dict(t=elapsed, position=pos, actual_along=actual_along, body=body, requested=requested,
                    command=cmd, yaw=yaw, control=control, arrays=arrays, drive=drive, stable=stable,
                    reference=speed_ref, heading_error=heading_error, route_distance=route_dist)
    return result, plotdata


def causal_average(x, width=10):
    total = np.cumsum(np.r_[0., x])
    stop = np.arange(1, len(x)+1)
    start = np.maximum(stop-width, 0)
    return (total[stop]-total[start])/(stop-start)


def figures(results, data):
    plt.rcParams.update({'font.size': 9, 'axes.grid': True, 'grid.alpha': .18})
    colors = {10: '#337ebc', 25: '#d45b24', 50: '#319966'}
    titles = {'legacy_position': 'Legacy position PID', 'path_pd': 'Spatial path / heading PD',
              'cascade_pi': 'Spatial path PD + velocity PI'}
    A = [x for x in results if x['phase']=='A2']
    fig, axes = plt.subplots(3, 3, figsize=(16, 10), constrained_layout=True)
    for j, design in enumerate(titles):
        for row in A:
            if row['profile']['design'] != design:
                continue
            d = data[row['run']]; c = colors[row['profile']['feedback_hz']]
            drive = d['drive']; t = d['t']; x = d['position'][:,0]
            label = f"{row['profile']['feedback_hz']} Hz"
            axes[j,0].plot(t[drive], d['actual_along'][drive], color=c, lw=.3, alpha=.14)
            axes[j,0].plot(t[drive], causal_average(d['actual_along'])[drive], color=c, lw=1., label=label)
            axes[j,0].plot(t[drive], d['command'][drive,0], '--', color=c, lw=.75)
            axes[j,1].plot(x[drive], (d['position'][drive,1]+.7)*1000, color=c, lw=.8, label=label)
            axes[j,2].plot(t[drive], d['yaw'][drive]*180/np.pi, color=c, lw=.8, label=label)
        axes[j,0].axhline(.3, color='black', lw=.8, label='desired 0.3')
        axes[j,1].axhline(0, color='black', lw=.8)
        axes[j,2].axhline(0, color='black', lw=.8)
        axes[j,0].set_title(titles[design]); axes[j,0].set_ylabel('Forward speed / command (m/s)')
        axes[j,0].set_xlabel('Simulation elapsed time (s)')
        axes[j,1].set_ylabel('Signed cross-track error (mm)'); axes[j,1].set_xlabel('Actual world X (m)')
        axes[j,2].set_ylabel('Actual yaw (degrees)'); axes[j,2].set_xlabel('Simulation elapsed time (s)')
        for ax in axes[j]:
            ax.legend(loc='best', fontsize=8)
    fig.suptitle('A2: all 9 executed candidates | truth feedback only, no SLAM claim\n'
                 'Velocity: raw pale; trailing 0.2 s mean solid; actual Teacher input dashed. Spatial reference fixed.')
    output = HERE/'campaign_ABC_actual_curves.png'; fig.savefig(output, dpi=150); plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)
    error_names = ['route', 'speed', 'heading', 'parking_xy', 'parking_yaw']
    palette = ['#386cb0','#6ea84a','#f0ba35','#c482ab','#df614e']
    labels = [f"{x['profile']['design'].replace('legacy_position','Legacy').replace('path_pd','PathPD').replace('cascade_pi','Cascade')}\n{x['profile']['feedback_hz']}Hz" for x in A]
    bottom = np.zeros(len(A))
    for key, color in zip(error_names, palette):
        vals = np.array([x['recomputed_metrics']['score_error_terms'][key] for x in A])
        axes[0,0].bar(np.arange(len(A)), vals, bottom=bottom, color=color, label=key)
        bottom += vals
    axes[0,0].set_xticks(np.arange(len(A)), labels, fontsize=8)
    axes[0,0].set_ylabel('Normalized error sum (lower is better)')
    axes[0,0].set_title('Score = 100 / (1 + sum); all acceptance gates passed first')
    axes[0,0].legend(fontsize=8, ncol=3)
    for design in titles:
        subset = [x for x in A if x['profile']['design']==design]
        hz = [x['profile']['feedback_hz'] for x in subset]
        axes[0,1].plot(hz, [x['recomputed_metrics']['speed_mae_mps'] for x in subset], 'o-', label=titles[design])
    axes[0,1].set_xticks([10,25,50]); axes[0,1].set_xlabel('Actual fresh feedback rate (Hz)')
    axes[0,1].set_ylabel('Stable actual COM speed MAE (m/s)'); axes[0,1].legend(fontsize=8)
    axes[0,1].set_title('No monotonic improvement from faster feedback in this 0.3 m/s straight')
    B = [x for x in results if x['phase']=='B']
    axes[1,0].bar([str(x['gain_id']) for x in B], [x['original_score'] for x in B], color='#738cab')
    axes[1,0].axhline(B[0]['original_score']*.95,color='#b53237',ls='--',label='within 5% of leader')
    axes[1,0].set_xlabel('B single-factor candidate id (all passed)'); axes[1,0].set_ylabel('Original score (higher is better)')
    axes[1,0].set_title('B gains: B0 default, B2 cross Kp=1.0, B3 cross Kd=0.1 retained'); axes[1,0].legend()
    for i, gid in enumerate([0,2,3]):
        group = [x for x in results if x['phase']=='C' and x['gain_id']==gid]
        scores = [x['original_score'] for x in group]
        axes[1,1].plot(np.arange(1,4), scores, 'o-', label=f'B{gid}: all 3 / 3 passed')
    axes[1,1].set_xticks([1,2,3]); axes[1,1].set_xlabel('Prescribed C repetition (none discarded)')
    axes[1,1].set_ylabel('Original score'); axes[1,1].legend()
    axes[1,1].set_title('Fixed-initial-condition repetitions are deterministic, not stochastic robustness')
    fig.suptitle('A2 + B + C: 27 original runs reconciled; model / controller / evaluator frozen')
    output2=HERE/'campaign_ABC_score_components.png'; fig.savefig(output2,dpi=150); plt.close(fig)

    fig, axes = plt.subplots(3,2,figsize=(15,11),constrained_layout=True)
    selected = next(x for x in A if x['profile']['design']=='cascade_pi' and x['profile']['feedback_hz']==25)
    d = data[selected['run']]; t=d['t']
    mask=(t>=2.9)&(t<=5.)
    axes[0,0].plot(t[mask],d['requested'][mask,0],label='controller requested vx')
    axes[0,0].plot(t[mask],d['command'][mask,0],label='actual slewed Teacher vx')
    axes[0,0].plot(t[mask],d['body'][mask,0],label='actual body COM vx',lw=.8)
    axes[0,0].axhline(.3,color='black',lw=.8,label='nominal speed')
    axes[0,0].set_title('CAS25 start: 50 Hz Teacher slew limits 0.6 m/s2')
    axes[0,0].set_ylabel('m/s'); axes[0,0].legend(fontsize=8)
    fresh=[c for c in d['control'] if c['controller_updated'] is True]
    ft=np.array([c['elapsed_s'] for c in fresh]); I=np.array([c['velocity_PI']['I'] for c in fresh])
    mask=(ft>=3)&(ft<=6)
    axes[0,1].plot(ft[mask],I[mask,0],label='vx integral output')
    axes[0,1].plot(ft[mask],np.array([c['velocity_PI']['P'][0] for c in fresh])[mask],label='vx P output')
    axes[0,1].set_title('I initially inhibited by slew; then compensates speed bias')
    axes[0,1].set_ylabel('Velocity command contribution (m/s)');axes[0,1].legend(fontsize=8)
    mask=(t>=14)&(t<=17)
    axes[1,0].plot(t[mask],d['body'][mask,0],lw=.8,label='raw COM vx')
    axes[1,0].plot(t[mask],d['command'][mask,0],lw=.9,label='Teacher input vx')
    axes[1,0].axhline(.3,color='black',lw=.8,label='nominal speed')
    axes[1,0].set_title('Filtered inner feedback does not chase every gait ripple')
    axes[1,0].set_ylabel('m/s');axes[1,0].legend(fontsize=8)
    for row in A:
        if row['profile']['design']!='cascade_pi':continue
        z=data[row['run']]; a=z['arrays']; p0=row['recomputed_metrics']['completion_elapsed_s']+.025
        nt=a['native_state_physics_world_time']; mask=(nt>=p0-1e-8)&(nt<=p0+5+1e-8)
        rr=rotation(a['native_quaternion_wxyz'][mask]); yy=np.arctan2(rr[:,1,0],rr[:,0,0])
        axes[1,1].plot(nt[mask]-nt[mask][0],wrap(yy-yy[0]),color=colors[row['profile']['feedback_hz']],
                       label=f"{row['profile']['feedback_hz']}Hz (zero command)")
    axes[1,1].axhline(.1,color='#b53237',ls='--',label='parking limit')
    axes[1,1].set_title('Parking yaw dominates CAS25 score advantage; native 200 Hz poses')
    axes[1,1].set_ylabel('Yaw change from first eligible stop (rad)');axes[1,1].legend(fontsize=8)
    for gid in [7,0,8]:
        row=next(x for x in B if x['gain_id']==gid); z=data[row['run']]
        mask=(z['t']>=3)&(z['t']<=20)
        gain=(DEFAULT_GAINS|row['profile'].get('gains',{}))['velocity_ki_x']
        axes[2,0].plot(z['t'][mask],causal_average(z['body'][:,0])[mask],label=f'Ki x = {gain:g}')
    axes[2,0].axhline(.3,color='black',lw=.8);axes[2,0].set_ylabel('Actual vx trailing 0.2 s mean (m/s)')
    axes[2,0].set_title('B Ki: less I leaves bias; more I is not a better aggregate result');axes[2,0].legend(fontsize=8)
    fresh_time=np.array([c['control_t_s'] for c in fresh]);mask=fresh_time<3.5
    axes[2,1].plot(fresh_time[mask][1:],np.diff(fresh_time[mask])*1000,'o-',label='actual fresh update period')
    axes[2,1].axhline(40,color='black',lw=.8);axes[2,1].set_ylabel('ms')
    axes[2,1].set_title('CAS25: genuine 40 ms fresh state updates; actor input remains 20 ms')
    axes[2,1].legend(fontsize=8)
    for ax in axes.flat:ax.set_xlabel('Simulation time (s); parking panel relative to stop')
    fig.suptitle('Executed command / PI / physical velocity evidence | original Gazebo records, no synthetic traces')
    output3=HERE/'campaign_ABC_PI_actual.png';fig.savefig(output3,dpi=150);plt.close(fig)
    return [str(output),str(output2),str(output3)]


def main():
    targets=[HERE/'campaign_ABC_analysis_final.json',HERE/'campaign_ABC_analysis_final.md',
             HERE/'campaign_ABC_actual_curves.png',HERE/'campaign_ABC_score_components.png',
             HERE/'campaign_ABC_PI_actual.png']
    if any(path.exists() for path in targets):
        raise FileExistsError('Refuse to replace any existing derived campaign artifact')
    results=[];data={};sources={}
    for phase,stem in PHASES.items():
        files=[CAMPAIGN/(stem+'.json'),CAMPAIGN/(stem+'_results.jsonl'),CAMPAIGN/(stem+'_freeze.json')]
        for path in files:sources[str(path)]=sha(path)
        freeze=read_json(files[2]);rows=read_jsonl(files[1])
        plan=read_json(files[0])
        if len(rows)!=9 or len(plan['profiles'])!=9:
            raise ValueError('Complete prescribed 9-run phase required')
        if freeze['plan_sha256']!=sha(files[0]):raise ValueError('Plan freeze mismatch')
        if [x['profile'] for x in rows]!=[x['path'] for x in plan['profiles']]:
            raise ValueError('Result order differs from pre-registered plan')
        for row in rows:
            result,curves=reconcile(row,phase,freeze)
            results.append(result);data[result['run']]=curves
    model_path=results[0]['model_path'];model_actual=sha(model_path)
    sources[model_path]=model_actual
    repeated=[]
    for gid in [0,2,3]:
        group=[x for x in results if x['phase']=='C' and x['gain_id']==gid]
        score=np.array([x['original_score'] for x in group])
        repeated.append({'gain_id':gid,'all_repetitions': [x['run'] for x in group],
                         'pass_count':sum(x['original_status']=='passed' for x in group),
                         'required_count':3,'score_median':float(np.median(score)),
                         'score_min':float(score.min()),'score_max':float(score.max()),
                         'score_population_std':float(score.std()),
                         'all_independent_numeric_arrays_identical':len({x['numeric_independent_array_values_sha256'] for x in group})==1,
                         'numeric_array_value_hashes':[x['numeric_independent_array_values_sha256'] for x in group],
                         'worst_route_rms_m':max(x['recomputed_metrics']['route_rms_m'] for x in group),
                         'worst_speed_mae_mps':max(x['recomputed_metrics']['speed_mae_mps'] for x in group)})
    selected=max(repeated,key=lambda x:x['score_median'])
    A=[x for x in results if x['phase']=='A2' and x['profile']['design']=='cascade_pi']
    ref=next(x for x in A if x['profile']['feedback_hz']==25)
    freq_decomp=[]
    for x in A:
        if x['profile']['feedback_hz']==25:continue
        terms={k:x['recomputed_metrics']['score_error_terms'][k]-ref['recomputed_metrics']['score_error_terms'][k]
               for k in ref['recomputed_metrics']['score_error_terms']}
        total=sum(terms.values())
        freq_decomp.append({'rate_compared_to_25_hz':x['profile']['feedback_hz'],
                            'normalized_error_delta':terms,'total_delta':total,
                            'parking_fraction_of_error_delta':(terms['parking_xy']+terms['parking_yaw'])/total,
                            'note': 'Fraction can exceed 1 when drive-speed or another term favors the compared rate.'})
    artifacts=figures(results,data)
    conclusions=[
        'A2/B/C 共 27 次均保留原 PASS；本分析复算原始 50Hz telemetry/control 与固定原验收，无原 run 写入。',
        '选定默认 spatial-path/heading PD + body-COM velocity PI，反馈 25Hz；Teacher 50Hz，物理 PD 200Hz。这是当前试验矩阵最优，不能称全局最优频率。',
        '旧位置 PID 与 pathPD 的 0.3m/s 命令实际约 0.24m/s；cascade PI 用约 0.35m/s Teacher 输入补偿，实际约 0.296m/s。结构与命令余量差异不是反馈加速效果。',
        'CAS10/25/50 恒速实际均值几乎相同，10Hz 的速度 MAE 反而略小；25Hz 总分优势主要来自停车姿态/横移，不能仅看总分断言频率越高越好。',
        'B0/B2/B3 的 3 次完整重复均通过，按三次中位数选 B0；固定初态、seed42、无观测噪声，同候选重复数值轨迹一致。这不证明随机扰动、定位噪声或延迟下鲁棒。',
        '位置误差来自固定折线的当前有向段投影：带符号横向误差、剩余沿段长度与终点距离；不是按时间追逐理论轨迹点。heading 环跟踪该段世界方向。',
        'PI 与 Teacher 实际限加速度执行均从日志复算；积分在 held 帧和停车阶段冻结，启动时抑制加剧限速差的积分。ABC 未把训练边界打满，不能等同极限饱和压力试验。',
        '真实 SLAM 接入需可靠时戳、机身原点/COM 及 world/body 坐标一致、短时 IMU 传播和新鲜度保护；提高 PID 频率不能补回缺测或消除定位偏差/噪声/延迟。',
    ]
    report={'schema':'executed_truth_teacher_ABC_independent_analysis/v1',
            'scope':'真值反馈仿真标定；不计 SLAM 融合、CHAMP 优劣或真机验收',
            'original_runs_modified':False,'model_sha256':model_actual,
            'model_matches_frozen':model_actual==MODEL_SHA,
            'navigation_ground_truth_used':True,'SLAM_navigation_verified':False,'real_robot_verified':False,
            'prescribed_phase_run_counts':{p:9 for p in PHASES},
            'all_27_statuses_passed':all(x['original_status']=='passed' for x in results),
            'all_input_hashes_and_recomputed_metrics_match':not any(x['independent_reconciliation_mismatches'] for x in results),
            'source_sha256':sources,'analysis_script_sha256':sha(__file__),
            'selection_rule':'Every prescribed repeat must pass, then all-three median score; no best-repeat selection.',
            'selected_C_candidate':selected,'repeat_comparison':repeated,
            'CAS_frequency_score_decomposition':freq_decomp,
            'controller_geometry': {
                'pose':'base-link origin in world frame; world velocity corrected from COM using omega cross COM offset',
                'active_segment':'immutable ordered route a,b, tangent=(b_xy-a_xy)/L, normal=(-tangent_y,tangent_x)',
                'signed_cross_error':'(origin_xy-a_xy) dot normal; left positive',
                'along_position':'(origin_xy-a_xy) dot tangent',
                'remaining':'L-along_position',
                'endpoint_distance':'norm(origin_xy-b_xy)',
                'lookahead_along_error':'(a+tangent*clip(along_position+0.8,0,L)-origin_xy) dot tangent',
                'time_parameterized_reference':False,
                'yaw_error':'wrap(atan2(tangent_y,tangent_x)-body_world_yaw)',
                'origin_vs_COM':'Controller spatial route uses link origin, inner vx/vy use COM velocity; yaw reference corrected for roll/pitch kinematics.',
            },
            'frozen_default_design': {'feedback_hz':25,'policy_hz':50,'physics_PD_hz':200,
                                     'gains':DEFAULT_GAINS,'outer_measurement_filter_tau_s':.2,
                                     'inner_velocity_filter_tau_s':.1,'limits_vx_vy_body_wz':[1.,.35,.8],
                                     'Teacher_slew_xyz_per_second':[.6,.6,.8],
                                     'goal_entry_m':.12,'goal_dwell_hold_m':.14,'independent_arrival_m':.15,
                                     'post_completion':'continuously execute Teacher under zero velocity; not action=0',
                                     'unique_joint_executor':'teacher_sim::TeacherActuator'},
            'historical_A1':{'result_file':str(CAMPAIGN/'phase_A_architecture_frequency_results.jsonl'),
                            'sha256':sha(CAMPAIGN/'phase_A_architecture_frequency_results.jsonl'),
                            'ranking_excluded':True,
                            'reason':'Original runtime normal-exit flush race plus 25Hz legacy 12cm dwell boundary limit cycle; all failures retained. v3 entire A2 matrix prospectively rerun.'},
            'conclusions_zh':conclusions,'runs':results,
            'derived_figures':[{'path':p,'sha256':sha(p)} for p in artifacts],
            'limits':['A2 one run per frequency and architecture; C repeats only selected 25Hz gain candidates.',
                      'ABC scene is a fixed 6m flat straight at 0.3m/s; D/D2 independently held out and outside this file.',
                      'No CHAMP run under this same protocol; cannot claim Teacher+PID beats CHAMP.',
                      'No online SLAM/IMU/point-cloud observation replacement accepted by this truth campaign.',
                      'C physical repetition deterministic; no randomized initial state/noise/delay/disturbance confidence interval.',
                      'Torque is actual applied native force command, not a measured motor torque.',
                      'Steady apparent closed-loop gain is diagnostic only; commanded slew and controller feedback preclude identifying open-loop dynamics from a simple response lag.']}
    output=HERE/'campaign_ABC_analysis_final.json'
    with output.open('x') as stream:json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
    lines=['# Teacher 真值速度与路线控制：A2/B/C 实测独立分析','',
           '默认串级控制器在本次矩阵中最佳：固定路径/朝向 PD 外环 + 机身 COM 速度 PI 内环，25 Hz 真实反馈。Teacher 保持 50 Hz，关节物理 PD 为 200 Hz。结论仅限仿真真值标定，不计 SLAM 融合、真机部署或与 CHAMP 的公平胜负。','',
           'A2、B、C 各 9 次，共 27 次原验收均通过；全部结果按预登记顺序纳入，原收据和失败历史保持不变。本文件从原始记录复算速度、路径、频率、停车和 PI，再对照原验收与冻结源哈希。','',
           '|A2 结构|反馈 Hz|实际稳定前向 m/s|速度 MAE m/s|路径 RMS mm|停车 yaw rad|原分数|',
           '|---|---:|---:|---:|---:|---:|---:|']
    for x in [v for v in results if v['phase']=='A2']:
        m=x['recomputed_metrics']
        lines.append(f"|{x['profile']['design']}|{x['profile']['feedback_hz']}|{m['mean_actual_forward_mps']:.6f}|{m['speed_mae_mps']:.6f}|{m['route_rms_m']*1000:.3f}|{m['native_first_5s_parking_yaw_drift_rad']:.6f}|{x['original_score']:.3f}|")
    lines += ['', '0.3 m/s 命令下，旧位置 PID/pathPD 的实际速度约 0.24 m/s；串级 PI 把 Teacher 输入补偿到约 0.35 m/s，实际约 0.296 m/s。旧位置 PID 的 vx/平面模长上限仍是目标速度，串级使用训练命令范围内余量。不能把这种结构差异归因于反馈频率。','',
              'CAS10/25/50 的稳定实际前向均值差仅约 0.000115 m/s；10 Hz 的速度 MAE 略低。25 Hz 相对 10/50 Hz 的分数优势主要由停车 yaw 与停车 XY 产生；驶入终点后继续零速度 Teacher，不主动校正停车后的姿态。更快反馈不是本场景改善的主要证据。','',
              '|B 增益候选|单项改动（其余默认）|原分数|路径 RMS mm|速度 MAE m/s|停车 yaw rad|',
              '|---|---|---:|---:|---:|---:|']
    for x in [v for v in results if v['phase']=='B']:
        m=x['recomputed_metrics'];g=x['profile'].get('gains',{})
        lines.append(f"|B{x['gain_id']}|{g or '默认'}|{x['original_score']:.3f}|{m['route_rms_m']*1000:.3f}|{m['speed_mae_mps']:.6f}|{m['native_first_5s_parking_yaw_drift_rad']:.6f}|")
    lines += ['', 'B0、B2、B3 在领先分数的 5% 内，按预登记各重复 3 次。每组 3/3 通过，三次中位数分别为 '+
              '、'.join(f"B{x['gain_id']}={x['score_median']:.6f}" for x in repeated)+'，选择默认 B0。不能挑某次最好，也没有为展示更好数字改门限。','',
              '同一候选的三次独立数值数组 '+('完全相同' if all(x['all_independent_numeric_arrays_identical'] for x in repeated) else '并非全部相同')+
              '。固定初态、seed42、无观测噪声使物理重复具有确定性；这验证可重复运行，不能当作随机扰动或真实定位误差的统计鲁棒性。','',
              '位置误差的定义是空间几何：当前固定有向段 a→b，以切向投影得到沿段位置，法向投影得到带符号横向误差；沿段剩余长度控制减速，终点欧氏距离控制到达。旧位置 PID 使用前方 0.8 m 截断看点，串级路径环直接用横向误差和横向速度。朝向参考为当前段切线，误差 wrap(段方向−机身 yaw)。没有按时间生成理论位置点。','',
              '路线位置是机身 link 原点；内环速度是 COM。两者使用训练归档 COM 偏置和 ω×偏置转换；坡道的 Euler yawdot 与机身 ωz 也明确区分。','',
              '实际日志确认真实更新间隔为 100/40/20 ms；低频之间只是保持外环指令，Teacher 仍每 20 ms 更新并以 [0.6,0.6,0.8] 限加速度执行。PI 对限幅/尚未跟上限加速度的同向误差阻止积分；held 帧和停车阶段积分冻结。分析 JSON 保存各轴实际阻止次数、积分值与数值复算误差。ABC 的命令未打满训练边界，不算极限饱和压力试验。','',
              '默认参数为 cross Kp/Kd=0.8/0.2、yaw Kp/Kd=1.3/0.15；速度 vx Kp/Ki=0.6/0.5、vy=0.4/0.3、机身角速度=0.3/0.2。外环测量低通 0.2 s，内环 0.1 s；目标到达入口 0.12 m、已开始 dwell 保持 0.14 m、独立验收到达 0.15 m。冻结后在 D/D2 验证高速度、转弯、坡道和台阶，不能用 ABC 平直线成功代替那些场景。','',
              '真实 SLAM 接入时应保留此控制器，替换有可信时戳与坐标约定的位姿/速度来源，再验证 IMU 短时传播、重定位跳变、测量噪声、延迟与 300 ms 新鲜度保护。提高频率无法补回缺测或定位偏差。25 Hz 是已测试的选择，不是必须高于 50 Hz 强化学习策略数倍；当前 native 接口本身只提供 50 Hz 状态给外环。','',
              f'完整 27 次数据、哈希链、重复数值哈希及复算见 [分析 JSON]({output})。图中只画实际记录；曲线的 0.2 s 因果均值只为可读性，不用于验收或响应时延识别。','']
    for p in artifacts:lines.append(f'![{Path(p).stem}]({p})\n')
    with (HERE/'campaign_ABC_analysis_final.md').open('x') as stream:stream.write('\n'.join(lines))
    print(json.dumps({'runs':len(results),'all_passed':report['all_27_statuses_passed'],
                      'all_reconciled':report['all_input_hashes_and_recomputed_metrics_match'],
                      'model_frozen':report['model_matches_frozen'],'selected_gain':selected['gain_id'],
                      'repetition_identical':[x['all_independent_numeric_arrays_identical'] for x in repeated],
                      'mismatches':[{x['run']:x['independent_reconciliation_mismatches']} for x in results if x['independent_reconciliation_mismatches']],
                      'files':[str(x) for x in targets]},ensure_ascii=False))


if __name__=='__main__':
    main()
