#!/usr/bin/env python3
"""Read completed simulation evidence. No ROS, runtime editing, or acceptance writes."""
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
RUNS = (
    '20261004_150033_navigation_teacher_pid_v5_routeleg_6m_r1_r1_8ef5',
    '20261004_142327_navigation_teacher_pid_v4_flat_long_r1_r1_1939',
)


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def rotations(q):
    w, x, y, z = q.T
    return np.stack([
        1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
        2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
        2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)
    ], axis=1).reshape(-1, 3, 3)


def average(t, v, window=.5):
    """Causal mean over observed samples, not a fitted response or generated data."""
    first = np.searchsorted(t, t-window, side='left')
    cumulative = np.concatenate([np.zeros((1, v.shape[1])), np.cumsum(v, axis=0)])
    count = np.arange(len(t))+1-first
    return (cumulative[1:]-cumulative[first])/count[:, None]


def analyze(name):
    run = ROOT/'runs'/name
    inputs = [run/f for f in ('navigation_pid_history.jsonl',
        'pid_navigation_independent_arrays.npz', 'summary_pid_navigation_independent.json',
        'pid_profile_input.json')]
    rows = [json.loads(line) for line in inputs[0].open() if line.strip()]
    fresh = [r for r in rows if r['pid'].get('updated') and r['mode']=='drive']
    profile = json.loads(inputs[3].read_text())
    summary = json.loads(inputs[2].read_text())
    with np.load(inputs[1]) as z:
        a = {k: z[k] for k in z.files}
    t = np.asarray([r['compute_ros_clock_ns']/1e9 for r in fresh])
    stamp = np.asarray([r['control_pose_stamp_ns']/1e9 for r in fresh])
    position = np.asarray([r['control_pose'][:2] for r in fresh])
    targets = np.asarray([r['checked_target'][:2] for r in fresh])
    yaw = np.asarray([r['yaw'] for r in fresh])
    pose_errors = np.asarray([r['pid']['error_world_xy_yaw'][:2] for r in fresh])
    body_errors = np.asarray([r['pid']['error_body_xy'] for r in fresh])
    # Same frame conversion as logged control, independently evaluated from data.
    rt = np.stack([np.cos(yaw), np.sin(yaw), -np.sin(yaw), np.cos(yaw)],
                  axis=1).reshape(-1, 2, 2)
    terms = {label: np.einsum('nij,nj->ni', rt,
        np.asarray([r['pid'][label+'_world_xy_yaw'][:2] for r in fresh]))
        for label in ('P', 'I', 'D')}
    leg = fresh[0].get('heading_reference')
    if leg is not None:
        leg_start = np.asarray(leg['leg_start'][:2])
        leg_goal = np.asarray(leg['leg_goal'][:2])
    else:
        anchor = json.loads((run/'navigation_anchor.json').read_text())
        leg_start = np.asarray(anchor['origin'][:2])
        leg_goal = np.asarray(fresh[0]['goal'][:2])
        inputs.append(run/'navigation_anchor.json')
    tangent = (leg_goal-leg_start)/np.linalg.norm(leg_goal-leg_start)
    normal = np.array([-tangent[1], tangent[0]])
    leg_cross = (position-leg_start)@normal
    nearest_distance, selected_difference, paths = [], [], {}
    for r, p in zip(fresh, position):
        name = r['trajectory_archive_file']
        if name not in paths:
            path = run/name
            with np.load(path) as z:
                paths[name] = z['samples'][:, :2]
            inputs.append(path)
        samples = paths[name]
        nearest = int(np.argmin(np.linalg.norm(samples-p, axis=1)))
        chosen, length = nearest, 0.
        while chosen+1 < len(samples) and length < .8:
            length += np.linalg.norm(samples[chosen+1]-samples[chosen])
            chosen += 1
        nearest_distance.append(np.linalg.norm(samples[nearest]-p))
        selected_difference.append(np.linalg.norm(samples[chosen]-r['checked_target'][:2]))
    nearest_distance = np.asarray(nearest_distance)
    nt = a['native_world_time_s']
    body_velocity = np.einsum('nji,nj->ni', rotations(a['native_quaternion_wxyz']),
                             a['native_world_COM_velocity'])
    et = a['execution_world_time_s']
    execution_command = a['execution_command']
    index = np.searchsorted(et, nt, side='right')-1
    valid = (index >= 0) & (nt-et[np.maximum(index, 0)] <= .020001)
    native_command = execution_command[np.maximum(index, 0)]
    steady = valid & (nt >= 8.) & (nt <= 43.)
    diagnostics = [r['pid'] for r in fresh]
    raw = np.asarray([d['raw_body_command'] for d in diagnostics])
    clipped = np.asarray([d['clipped_body_command'] for d in diagnostics])
    events = [r for i, r in enumerate(rows) if i==0 or
              rows[i-1]['trajectory_id'] != r['trajectory_id']]
    samples = []
    for wanted in (8., 18., 28., 32., 36., 44., 48.):
        i = int(np.argmin(abs(t-wanted)))
        samples.append({'compute_sim_time_s':float(t[i]), 'source_stamp_s':float(stamp[i]),
            'body_position_error_m':body_errors[i].tolist(),
            'body_P_command_mps':terms['P'][i].tolist(),
            'body_I_command_mps':terms['I'][i].tolist(),
            'body_D_command_mps':terms['D'][i].tolist(),
            'clipped_body_command_mps_radps':clipped[i].tolist(),
            'nearest_active_SCAN_sample_distance_m':float(nearest_distance[i]),
            'signed_requested_leg_cross_track_m':float(leg_cross[i])})
    metrics = {
        'run':str(run), 'original_formal_status':summary['status'],
        'original_route_metrics':summary['metrics']['route'],
        'fresh_drive_PID_samples':len(fresh), 'total_PID_calls':len(rows),
        'spatial_target_selection': {
            'nearest':'argmin Euclidean XY distance to active discrete SCAN samples',
            'lookahead_arc_length_m':.8,
            'target':'first subsequent sample whose accumulated XY arc length reaches .8m, or endpoint',
            'maximum_reconstructed_target_difference_m':float(max(selected_difference)),
            'maximum_logged_error_minus_target_minus_pose_m':float(np.max(abs(pose_errors-(targets-position)))),
            'time_parameterized_target_used':False,
            'PID_position_input':'two component vector: checked lookahead target XY minus actual raw SLAM XY',
            'nearest_active_SCAN_sample_distance_max_m':float(nearest_distance.max()),
            'nearest_active_SCAN_sample_distance_RMS_m':float(np.sqrt(np.mean(nearest_distance**2))),
            'signed_fixed_requested_leg_cross_track_min_m':float(leg_cross.min()),
            'signed_fixed_requested_leg_cross_track_max_m':float(leg_cross.max()),
            'fixed_requested_leg_cross_track_RMS_m':float(np.sqrt(np.mean(leg_cross**2))),
            'limitation':'Active SCAN references are replanned from current position; their small local distance does not establish tracking of the immutable mission centerline.'},
        'steady_response': {
            'world_interval_s':[8.,43.], 'native_observed_samples':int(steady.sum()),
            'average_executed_body_command_mps_radps':native_command[steady].mean(0).tolist(),
            'average_actual_body_COM_velocity_mps':body_velocity[steady].mean(0).tolist(),
            'average_forward_command_minus_speed_mps':float(np.mean(native_command[steady,0]-body_velocity[steady,0])),
            'actual_forward_mean_div_command_mean':float(body_velocity[steady,0].mean()/native_command[steady,0].mean()),
            'causal_command_source':'most recent recorded actual Actor command, maximum .020001s source gap',
            'velocity_scope':'Recorded native physics COM body twist, offline response diagnosis only. PID feedback itself is raw SLAM body-origin twist.'},
        'limits_and_terms': {
            'forward_antiwindup_blocked_fraction':float(np.mean([d['antiwindup_blocked_axes'][0] for d in diagnostics])),
            'raw_forward_command_median_mps':float(np.median(raw[:,0])),
            'maximum_planar_command_mps':float(np.linalg.norm(clipped[:,:2],axis=1).max()),
            'body_lateral_I_contribution_min_max_mps':[float(terms['I'][:,1].min()),float(terms['I'][:,1].max())],
            'distinct_active_drive_reset_counts':sorted(set(d['reset_count'] for d in diagnostics)),
            'configured_shared_XY_kp_ki_kd':[profile['pid']['position_'+k] for k in ('kp','ki','kd')],
            'configured_yaw_kp_ki_kd':[profile['pid']['yaw_'+k] for k in ('kp','ki','kd')],
            'configured_limits_vx_vy_wz':[profile['max_speed_mps'],profile['max_lateral_speed_mps'],profile['max_yaw_rate_radps']],
            'separate_velocity_error_PI_or_PID_present':False},
        'selected_original_logged_samples':samples,
        'input_hashes':{str(p.relative_to(ROOT)):sha(p) for p in inputs},
    }
    return metrics, dict(t=t, stamp=stamp, position=position, targets=targets,
        terms=terms, raw=raw, clipped=clipped, body_errors=body_errors,
        leg_cross=leg_cross, nearest_distance=nearest_distance, paths=paths,
        leg_start=leg_start, leg_goal=leg_goal, nt=nt, body_velocity=body_velocity,
        et=et, execution_command=execution_command, replans=events,
        heading_error=np.asarray([d['error_world_xy_yaw'][2] for d in diagnostics]))


def main():
    results = [analyze(name) for name in RUNS]
    m, d = results[0]
    fig, ax = plt.subplots(3, 2, figsize=(15, 11), constrained_layout=True)
    fig.suptitle('Actual V5 6m run: geometry and PID terms before tuning\n'
                 'Recorded sensor SLAM + CPU Teacher; offline diagnosis, original formal FAIL retained', fontsize=14)
    a = ax[0,0]
    a.plot(*np.vstack([d['leg_start'],d['leg_goal']]).T,'k--',label='Immutable requested SLAM leg')
    a.plot(*d['position'].T,color='#146e82',lw=2,label='Actual SLAM trajectory')
    for number, color in zip((1,10,14),('#999999','#cb6b21','#ad5ab2')):
        event=d['replans'][number-1];path=d['paths'][event['trajectory_archive_file']]
        a.plot(*path.T,color=color,lw=1,label=f"Actual SCAN #{number} at {event['compute_ros_clock_ns']/1e9:.1f}s")
    a.set(xlabel='SLAM camera_init X (m)',ylabel='SLAM camera_init Y (m)',
          title='Local SCAN path also moves away from the requested line')
    a.set_aspect('equal',adjustable='datalim');a.legend(fontsize=8)
    a = ax[0,1]
    a.plot(d['t'],d['leg_cross'],color='#146e82',label='Signed cross-track to immutable leg')
    a.plot(d['t'],d['nearest_distance'],color='#cb6b21',label='Distance to active SCAN nearest sample')
    a.axhline(0,color='grey',lw=.6);a.set(title='These errors describe different references',ylabel='m');a.legend(fontsize=8)
    a = ax[1,0]
    a.step(d['et'],d['execution_command'][:,0],where='post',color='#cb6b21',label='Executed forward command')
    a.plot(d['nt'],d['body_velocity'][:,0],color='#65a6b4',alpha=.35,lw=.4,label='Actual native COM speed, raw')
    averaged = average(d['nt'], d['body_velocity'])
    a.plot(d['nt'],averaged[:,0],color='#146e82',lw=1.5,label='Actual speed, causal 0.5s mean')
    a.set(title='8–43s mean: command 0.193 vs actual 0.140 m/s',ylabel='m/s');a.legend(fontsize=8)
    a = ax[1,1]
    a.step(d['et'],d['execution_command'][:,1],where='post',color='#cb6b21',label='Executed lateral command')
    a.plot(d['nt'],d['body_velocity'][:,1],color='#65a6b4',alpha=.3,lw=.4,label='Actual COM lateral speed, raw')
    a.plot(d['nt'],averaged[:,1],color='#146e82',lw=1.5,label='Actual speed, causal 0.5s mean')
    a.axhline(0,color='grey',lw=.6);a.set(title='Lateral motion: observed reversal and residual command',ylabel='m/s');a.legend(fontsize=8)
    a = ax[2,0]
    for label,color in [('P','#146e82'),('I','#ad5ab2'),('D','#777777')]:
        a.plot(d['t'],d['terms'][label][:,1],color=color,label=f'{label}: body lateral contribution')
    a.plot(d['t'],d['clipped'][:,1],color='#cb6b21',label='Final limited lateral output')
    a.axhline(0,color='grey',lw=.6);a.set(title='Integral contribution reaches about +/-0.032 m/s',ylabel='m/s');a.legend(fontsize=8)
    a = ax[2,1]
    a.plot(d['t'],d['raw'][:,0],color='#146e82',label='Forward PID before clipping')
    a.plot(d['t'],d['clipped'][:,0],color='#cb6b21',label='Forward command after limits')
    a.axhline(.2,color='#aa3333',ls='--',label='Current command limit 0.20 m/s')
    a.set(title='Forward saturation: increasing position P has little room',ylabel='m/s');a.legend(fontsize=8)
    for i,a in enumerate(ax.flat):
        a.grid(alpha=.2)
        if i:
            a.set_xlim(5,51);a.set_xlabel('World simulation time (s)')
            for r in d['replans'][1:]:
                a.axvline(r['compute_ros_clock_ns']/1e9,color='#bbbbbb',alpha=.2,lw=.5)
    figure=OUT/'actual_pid_curves.png'
    fig.savefig(figure,dpi=140);plt.close(fig)
    result={'schema':'pid_curve_analysis/v1',
        'scope':'Offline observed diagnosis, no controller gain change, no new simulated test or revised acceptance',
        'recorded_runs':[x[0] for x in results],
        'tuning_interpretation':[
            'Confirm one immutable SLAM/map route reference before optimizing global route error; actual locally replanned SCAN geometry bends.',
            'Forward command already saturates; position Kp cannot correct the Teacher velocity deficit while this limit remains.',
            'Decompose tangent/normal control or separate forward/lateral gains before targeted lateral tuning; current X/Y gains are shared.',
            'First prospective lateral comparison: disable only lateral I while holding P 0.6, D 0.12, all command limits, freshness and protections. This has not been simulated.',
            'After reference correction and I isolation, a separate P 0.4 comparison is reasonable only if slow lateral overshoot persists; do not change P and I together or infer optimum D from gait-frequency ripples.',
            'Only reintroduce small lateral I if a persistent bias remains after reference/response correction; proposed Ki 0.02–0.04 with 0.01–0.02m/s contribution bound is an unverified trial range.',
            'A new velocity PI would use v_reference minus measured velocity; it is absent currently, needs separate identification and prospective validation within original bounds.'
        ],
        'limitations':[
            'One V5 and one V4 long run do not identify closed-loop plant transfer functions or uniquely determine optimum gains.',
            'Causal 0.5s plot smoothing adds delay; apparent shift is not an identified actuator delay.',
            'Native body COM velocity is offline diagnostic, never substituted into navigation PID feedback.',
            'No change to original summary, goal definition, frozen acceptance, CPU policy, camera demo or other training.'
        ],
        'analysis_script_sha256':sha(Path(__file__)),
        'figure_sha256':sha(figure), 'figure_visual_QA':'pending pixel inspection'}
    (OUT/'analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'analysis':str(OUT/'analysis.json'),'figure':str(figure),
        'V5_nearest_SCAN_max_m':m['spatial_target_selection']['nearest_active_SCAN_sample_distance_max_m'],
        'V5_requested_line_cross_track_min_m':m['spatial_target_selection']['signed_fixed_requested_leg_cross_track_min_m'],
        'V5_command_speed_means':m['steady_response']},ensure_ascii=False))


if __name__=='__main__':
    main()
