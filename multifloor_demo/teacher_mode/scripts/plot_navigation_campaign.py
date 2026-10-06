#!/usr/bin/env python3
"""Plot archived finite navigation evidence without launching any runtime.

Reads actual JSONL / independently aligned NPZ data. No interpolation of SLAM
poses, no invented arrival samples, and no new navigation acceptance decision.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_GENERIC_SHA = {
    '9c33': '9151c97b6a5270af0a07a118c273d60e0f6613c09381e09bd5d5dde1ea33a425',
    '1c57': '809984f8179efc6c5fb825a6611c0c7e3ab867a9c289d99046dab345adc762dd',
    'bd19': '26abd5902204496d65236e889c66717b8cd6a76e22ecd6f646012d5032239524',
}

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()

def read(path):
    return json.loads(Path(path).read_text())

def lines(path):
    with Path(path).open() as f:
        for line in f:
            if line.strip():
                yield json.loads(line)

def trailing_mean(x, samples=20):
    """Display-only trailing100ms mean of actual200Hz samples; no padding."""
    result = np.full(len(x), np.nan)
    sums = np.r_[0., np.cumsum(x)]
    result[samples-1:] = (sums[samples:] - sums[:-samples]) / samples
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'test_results/navigation_v4_independent_campaign')
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    n = len(args.runs)
    fig, axs = plt.subplots(n, 2, figsize=(16, 3.5*n), sharex=True, squeeze=False)
    xyfig, xyaxs = plt.subplots(1, n, figsize=(6*n, 5.2), squeeze=False)
    evidence = []
    allarrays = {}
    navfreeze = read(ROOT/'test_results/navigation_v4_freeze.json')
    freeze_manifest = read(ROOT/'test_results/navigation_independent_audit_20261004/candidate_evaluator_20261004/freeze_manifest.json')
    shared = None
    core_differences = []
    for index, raw in enumerate(args.runs):
        run = raw.resolve()
        summary = read(run/'summary_navigation_independent.json')
        short = run.name.rsplit('_', 1)[-1]
        generic = sha(run/'summary.json')
        if short not in ORIGINAL_GENERIC_SHA or generic != ORIGINAL_GENERIC_SHA[short]:
            raise ValueError(f'{run.name}: original generic summary hash changed or run not in campaign')
        hashes = {name: sha(run/name) for name in [
            'telemetry.jsonl', 'actuator.jsonl', 'navigation_slam_poses.jsonl',
            'navigation_independent_truth_error.npz', 'navigation_request.json',
            'navigation_anchor.json', 'summary_navigation_independent.json',
            'navigation_timing_audit.json', 'source_manifest.json', 'summary.json',
            'runtime_manifest.json', 'navigation_profile.json', 'navigation_scope.json',
        ]}
        frozen = read(run/'source_manifest.json')
        freeze_mismatch = []
        for path, expected in navfreeze['source_sha256'].items():
            name = str(Path(path).relative_to(ROOT))
            archive = run/'sources'/name
            if frozen.get(name) != expected or not archive.is_file() or sha(archive) != expected:
                freeze_mismatch.append(name)
        core = {k: v for k, v in frozen.items() if k.startswith(('navigation/', 'policy/')) and Path(k).suffix in {'.py', '.json'}}
        if shared is None:
            shared = core
        else:
            core_differences.extend({'run': run.name, 'source': k} for k in set(shared)|set(core) if shared.get(k) != core.get(k))
        ts, commands, requests, expired = [], [], [], []
        world0 = None
        for row in lines(run/'telemetry.jsonl'):
            if world0 is None:
                world0 = row['world_sim_time']
            ts.append(row['world_sim_time']-world0)
            commands.append(row['command'])
            requests.append(row['requested'])
            expired.append(row['command_expired'])
        ts, commands, requests = map(np.asarray, [ts, commands, requests])
        nts, bv, omega = [], [], []
        for row in lines(run/'actuator.jsonl'):
            if row.get('kind') == 'physics_step':
                # Actual physical state precedes PreUpdate log time by one5ms step.
                nts.append(row['t']-.005-world0)
                bv.append(row['body_lin_vel_com'])
                omega.append(row['body_ang_vel'])
        nts, bv, omega = map(np.asarray, [nts, bv, omega])
        poses = list(lines(run/'navigation_slam_poses.jsonl'))
        stamp = np.asarray([row['stamp_ns'] for row in poses], dtype=np.int64)
        slam = np.asarray([row['position'] for row in poses])
        with np.load(run/'navigation_independent_truth_error.npz') as saved:
            # Existing causal matched audit array must equal actual source poses.
            if not np.array_equal(stamp, saved['slam_stamp_ns']) or not np.array_equal(slam, saved['slam_position']):
                raise ValueError(f'{run.name}: independently aligned NPZ does not retain exact raw SLAM poses')
            aligned = saved['truth_aligned_position'].copy()
        request = read(run/'navigation_request.json')
        anchor = read(run/'navigation_anchor.json')
        arrival_times = [entry['claimed_receipt']['stamp_ns']*1e-9-world0 for entry in summary['metrics']['arrivals']]
        stop = summary['metrics']['stop']
        parking = np.asarray(stop['world_time_window_s'])-world0
        for col, (field, measured, unit) in enumerate([(0, bv[:, 0], 'body COM vx (m/s)'), (2, omega[:, 2], 'body wz (rad/s)')]):
            ax = axs[index, col]
            ax.plot(nts, measured, color='#7794c0', alpha=.22, lw=.4, label='physical200Hz raw')
            ax.plot(nts, trailing_mean(measured), color='#1b4f91', lw=1., label='physical100ms trailing mean')
            ax.step(ts, requests[:, field], where='post', color='#808080', lw=.8, alpha=.6, label='accepted/stale request50Hz')
            ax.step(ts, commands[:, field], where='post', color='#e07918', lw=1., label='actual actor input50Hz')
            for at in arrival_times:
                ax.axvline(at, color='#267d42', linestyle=':', lw=1)
            ax.axvspan(*parking, color='#51b970', alpha=.18, label='measured settled stop3s')
            ax.axhline(0, color='#999999', lw=.5)
            ax.set_xlim(0, 180)
            ax.set_ylabel(unit)
            ax.set_title(f'{index+1}: {short} — finite flat passed / expires {sum(expired)}/{len(expired)}')
            ax.grid(alpha=.18)
            if index == 0:
                ax.legend(loc='upper right', fontsize=7, ncol=2)
            if index == n-1:
                ax.set_xlabel('actual simulation time (s); native physical state uses PreUpdate t−0.005')
        ax = xyaxs[0, index]
        ax.plot(aligned[:, 0], aligned[:, 1], '--', color='#7e7e7e', lw=.9, label='Gazebo aligned OFFLINE only')
        ax.plot(slam[:, 0], slam[:, 1], color='#2261a2', lw=1.3, label='actual SLAM camera_init')
        ax.scatter([anchor['origin'][0]], [anchor['origin'][1]], marker='x', s=40, color='#343434', label='once-frozen SLAM anchor')
        for j, goal in enumerate(request['goals']):
            center = goal['center']
            outer = goal['arrival']['radius_m']
            control = goal['arrival']['control_band']['radius_m']
            color = ['#e58b28', '#3b9151'][j]
            ax.add_patch(Circle(center[:2], outer, fill=False, edgecolor=color, linestyle=':', lw=1.1, label='outer radius.22m' if j == 0 else None))
            ax.add_patch(Circle(center[:2], control, facecolor=color, edgecolor=color, alpha=.12, label='control radius.17m' if j == 0 else None))
            ax.scatter(center[0], center[1], marker='+', color=color, s=55)
            receipt = summary['metrics']['arrivals'][j]['claimed_receipt']
            dwell = (stamp >= receipt['start_stamp_ns']) & (stamp <= receipt['stamp_ns'])
            ax.scatter(slam[dwell, 0], slam[dwell, 1], color=color, s=16, zorder=5, label='actual raw SLAM dwell7pts' if j == 0 else None)
            ax.annotate(goal['goal_id'], center[:2], xytext=(4, 12), textcoords='offset points', fontsize=8)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(-.30, 1.28)
        ax.set_ylim(-.35, .35)
        ax.set_xlabel('SLAM x (m)')
        ax.set_ylabel('SLAM y (m)')
        ax.set_title(f'{short}: actual raw SLAM and registered goals\nGoal centers1m apart; physical excursion {summary["checks"]["teacher_received_and_executed_motion"]["maximum_actual_xy_excursion_m"]:.3f}m')
        ax.legend(loc='lower center', fontsize=7)
        ax.grid(alpha=.18)
        for key, value in {'policy_time_s': ts, 'requested_body_vx_vy_wz': requests, 'actor_body_vx_vy_wz': commands,
                           'native_physical_time_s': nts, 'native_body_com_velocity': bv, 'native_body_angular_velocity': omega,
                           'slam_stamp_ns': stamp, 'raw_slam_position': slam, 'offline_aligned_gazebo_position': aligned}.items():
            allarrays[short+'_'+key] = value
        checks = summary['checks']
        timing = summary['metrics']['execution_timing']
        evidence.append({
            'run_dir': str(run), 'run_id': run.name, 'status': summary['status'], 'levels': summary['levels'],
            'analysis_errors': summary['errors'], 'check_statuses': {k: v['status'] for k, v in checks.items()},
            'analyzer_sha256': summary['analyzer_sha256'], 'input_sha256': hashes,
            'source_archive_matches_runtime_freeze': not freeze_mismatch, 'freeze_mismatches': freeze_mismatch,
            'original_generic_summary_sha256': generic, 'original_generic_summary_bytes_unchanged': True,
            'policy_samples': len(ts), 'native_samples': len(nts), 'slam_pose_samples': len(slam),
            'scan_payloads': summary['metrics']['recorded_inputs']['counts']['trajectories'],
            'motion': checks['teacher_received_and_executed_motion'], 'stop': stop,
            'arrivals': summary['metrics']['arrivals'], 'safety': checks['native_physical_safety'],
            'command_expired_samples': int(sum(expired)), 'command_expired_fraction': sum(expired)/len(expired),
            'observed_real_time_factor': timing['real_time_factor_observed']['simulation_seconds_per_wall_second'],
            'offline_truth_error_p95_m': summary['metrics']['independent_truth_error']['p95_error_m'],
        })
    fig.suptitle('Three actual180s finite SLAM/SCAN → CPU Teacher runs: requests, actor input and physical motion', fontsize=14)
    fig.text(.5, .01, 'All data recorded in actual runs. Green dotted lines: raw SLAM region arrival. Green band: settled stop evaluation. No truth feedback to navigation.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .025, 1, .96))
    commands_path = out/'navigation_three_runs_commands_physical_180s.png'
    fig.savefig(commands_path, dpi=150)
    plt.close(fig)
    xyfig.suptitle('Actual SLAM XY and fixed region goals — Gazebo comparison is offline diagnostic only', fontsize=13)
    xyfig.text(.5, .015, 'Disc plot is XY only; independent arrival additionally checks original SLAM height ±.10m, dwell≥.6s and gaps≤.2s.', ha='center', fontsize=9)
    xyfig.tight_layout(rect=(0, .04, 1, .95))
    xy_path = out/'navigation_three_runs_actual_slam_xy.png'
    xyfig.savefig(xy_path, dpi=150)
    plt.close(xyfig)
    arrays_path = out/'navigation_three_runs_plot_arrays.npz'
    np.savez_compressed(arrays_path, **allarrays)
    aggregate = {
        'schema': 1, 'scope': 'finite_flat_relative_roundtrip_v4_three_actual_runs',
        'status': 'passed' if all(v['status'] == 'passed' and not v['analysis_errors'] and v['source_archive_matches_runtime_freeze'] for v in evidence) and not core_differences else 'failed',
        'runs_evaluated': len(evidence), 'passed_runs': sum(v['status'] == 'passed' for v in evidence),
        'all_runtime_navigation_and_policy_sources_same': not core_differences, 'runtime_source_differences': core_differences,
        'actual_motion_scope': 'Goal centers1m apart; actual body excursions about.83–.86m. Regions, not exact goal centers, were reached.',
        'truth_scope': 'Immutable anchor alignment AFTER recording, diagnostic only, never target/command/arrival feedback',
        'command_TTL_physical_fault_injection': 'unverified; transient expires and post-arrival stopping do not establish a controlled producer outage',
        'global_levels_preserved': {'sim2sim': 'failed', 'navigation_all_tasks': 'unverified', 'multifloor': 'unverified', 'dynamic_obstacle': 'unverified', 'real_robot': 'unverified'},
        'runtime_freeze_sha256': sha(ROOT/'test_results/navigation_v4_freeze.json'),
        'independent_candidate_freeze_sha256': sha(ROOT/'test_results/navigation_independent_audit_20261004/candidate_evaluator_20261004/freeze_manifest.json'),
        'independent_candidate_manifest': freeze_manifest,
        'plot_script_sha256': sha(__file__), 'artifacts': {str(p): sha(p) for p in [commands_path, xy_path, arrays_path]},
        'display_processing': 'Native raw200Hz and actual50Hz request/input shown. Additional physical line is trailing20samples/100ms mean, only a visual aid. SLAM XY uses exact original raw poses without interpolation.',
        'runs': evidence,
    }
    receipt = out/'navigation_v4_independent_campaign.json'
    receipt.write_text(json.dumps(aggregate, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps({'status': aggregate['status'], 'runs': len(evidence), 'core_differences': core_differences, 'receipt': str(receipt), 'commands_png': str(commands_path), 'slam_xy_png': str(xy_path)}))

if __name__ == '__main__':
    main()
