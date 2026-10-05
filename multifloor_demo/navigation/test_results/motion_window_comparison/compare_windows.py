#!/usr/bin/env python3
"""Excluded passive window comparison; no ROS, feedback, GT tuning, or commands.

The exact existing observer is instantiated with a different declared window.
All fit, gap and transition gates remain unchanged. GT is a distinct observer
for error assessment in one pre-existing frozen initial SE(3), not an input to
the SLAM observer or a controller. Commands lack Headers: historical receipt
sim-clock floats are explicitly reconstructed, never called native stamps.
"""
import argparse
import bisect
from collections import Counter, defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path
import tarfile

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[3]
OBSERVER = ROOT / 'navigation/test_results/measured_motion_design/motion_observer.py'
spec = importlib.util.spec_from_file_location('unchanged_passive_observer', OBSERVER)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Observer = module.MeasuredMotionObserver
NS = 1_000_000_000
WINDOWS = (400_000_000, 600_000_000, 800_000_000, 1_200_000_000)
GAIT_NS = 600_000_000


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open() as handle:
        for line in handle:
            yield json.loads(line)


def stats(values):
    if not values:
        return {'count': 0}
    a = np.asarray(values)
    return dict(count=len(a), mean=np.mean(a, axis=0).tolist(),
                median=np.median(a, axis=0).tolist(),
                p05=np.quantile(a, .05, axis=0).tolist(),
                p95=np.quantile(a, .95, axis=0).tolist(),
                abs_p95=np.quantile(np.abs(a), .95, axis=0).tolist())


def opposite(command, measured, minimum=.02):
    return bool(abs(command) > minimum and abs(measured) > minimum and command * measured < 0)


def episodes(samples, field, gap_ns=150_000_000):
    """Contiguous classifications, no bridging invalid or new-context samples."""
    result = []
    current = None
    for sample in samples:
        key = (sample['stage'], tuple(sample.get('context') or ()), sample.get('command_sign'))
        match = bool(sample.get(field))
        if current and (not match or key != current['key'] or sample['stamp_ns']-current['last_stamp_ns'] > gap_ns):
            result.append(current)
            current = None
        if match:
            if current is None:
                current = dict(key=key, start_stamp_ns=sample['stamp_ns'], last_stamp_ns=sample['stamp_ns'], count=0)
            current['last_stamp_ns'] = sample['stamp_ns']
            current['count'] += 1
    if current:
        result.append(current)
    for item in result:
        item['duration_ns'] = item['last_stamp_ns']-item['start_stamp_ns']
        item['at_least_one_nominal_gait_cycle'] = item['duration_ns'] >= GAIT_NS
        item['key'] = list(item['key'])
    return result


def transition_causes(observer, start, end):
    cs = [c for c in observer.commands if c['t'] <= end]
    before = [i for i, c in enumerate(cs) if c['t'] <= start]
    if not before:
        return ['missing_preceding_command']
    cs = cs[before[-1]:]
    causes = []
    if len({c['mode'] for c in cs}) > 1:
        causes.append('actual_motion_mode_change')
    if len({c['state'] for c in cs}) > 1:
        causes.append('adapter_state_change')
    contexts = {c['context'] for c in cs}
    if len(contexts) > 1:
        goals = {c[:2] if c is not None else None for c in contexts}
        causes.append('goal_or_request_change' if len(goals) > 1 else 'trajectory_identity_only_change')
    return causes or ['unclassified']


def command_window_quality(observer, start, end):
    cs = [c for c in observer.commands if c['t'] <= end]
    before = [i for i, c in enumerate(cs) if c['t'] <= start]
    if not before:
        return None
    cs = cs[before[-1]:]
    signs = set()
    saturated_yaw_ns = 0
    for i,c in enumerate(cs):
        left = max(start,c['t'])
        right = min(end,cs[i+1]['t'] if i+1<len(cs) else end)
        if right <= left:
            continue
        if abs(c['c'][2])>.001:
            signs.add(int(np.sign(c['c'][2])))
        if abs(c['c'][2])>=.078:
            saturated_yaw_ns += right-left
    latest = cs[-1]['c']
    return dict(latest_applied_command=list(latest), meaningful_yaw_signs=sorted(signs),
                yaw_sign_consistent=len(signs)<=1,
                yaw_saturated_time_fraction=saturated_yaw_ns/max(1,end-start),
                latest_vx_positive_headroom=max(0.,.12-latest[0]),
                latest_yaw_same_direction_headroom=max(0.,.08-abs(latest[2])))


def truth_pose(t, times, values, Rws, tws):
    k = int(np.searchsorted(times, t))
    if k < len(times) and times[k] == t:
        p = np.asarray(values[k]['p'])
        R = Rotation.from_quat(values[k]['q']).as_matrix()
    elif 0 < k < len(times) and times[k]-times[k-1] <= 150_000_000:
        alpha = (t-int(times[k-1])) / int(times[k]-times[k-1])
        p = (1-alpha)*np.asarray(values[k-1]['p']) + alpha*np.asarray(values[k]['p'])
        R = Slerp([0., 1.], Rotation.from_quat([values[k-1]['q'], values[k]['q']]))(alpha).as_matrix()
    else:
        return None
    return Rws.T @ (p-tws), Rotation.from_matrix(Rws.T @ R).as_quat()


def compare(run, output, alignment_path):
    output.mkdir(parents=True, exist_ok=False)
    alignment = json.loads(alignment_path.read_text())['initial_SE3']
    Rws = np.asarray(alignment['rotation_world_from_slam'])
    tws = np.asarray(alignment['translation_world_from_slam'])
    ps, truth = [], {}
    for p in rows(run/'pose_audit.jsonl'):
        if p['source'] == 'slam':
            ps.append(p)
        elif p['source'] == 'truth':
            truth[p['stamp_ns']] = p
    times = np.asarray(sorted(truth), dtype='int64')
    gts = [truth[int(t)] for t in times]
    gt_poses = {p['stamp_ns']: truth_pose(p['stamp_ns'], times, gts, Rws, tws) for p in ps}
    commands = [dict(t=round(c['sim']*NS), c=[c['value'][0], c['value'][1], c['value'][5]], state=c['state'])
                for c in rows(run/'joint_stop_adapter.jsonl') if c['kind'] == 'actual_champ_command']
    commands.sort(key=lambda c: c['t'])
    contexts = {}
    for n in rows(run/'navigation_audit.jsonl'):
        s = n['status']
        st = s.get('steering')
        if st:
            contexts[round(st['stamp']*NS)] = (s['request_id'], s['waypoint_index'], s.get('accepted_trajectory_id'))
    ct = sorted(contexts)
    for c in commands:
        k = bisect.bisect_right(ct, c['t'])-1
        c['context'] = contexts[ct[k]] if k >= 0 else None
    # Archive the actual run's gait fields and compiled phase-generator source.
    with tarfile.open(run/'source_snapshot.tar.gz') as archive:
        names = archive.getnames()
        selected = [n for n in names if n.endswith('simulation/config/gait.yaml') or n.endswith('champ/leg_controller/phase_generator.h')]
        gait_sources = {}
        for name in selected:
            data = archive.extractfile(name).read()
            gait_sources[name] = dict(sha256=hashlib.sha256(data).hexdigest(), text=data.decode())
    report = dict(scope=__doc__, run=run.name, windows={}, gait_nominal_cycle_ns=GAIT_NS,
                  nominal_cycle_basis='Frozen CHAMP swing .25s plus configured stance .35s, not measured contact-cycle phase.',
                  frozen_gait_sources=gait_sources, initial_SE3=alignment,
                  timestamp_contract=dict(native_SLAM_integer_headers=True, GT_integer_headers=True,
                      GT_max_interpolation_gap_ns=150_000_000, callback_wall_age_evaluated=False,
                      replay_wall_age='ideal zero receipt age; unavailable in archived body-pose data',
                      actual_command_time='Float adapter receipt sim clock reconstructed to ns, no Twist Header',
                      context='Sparse status steering clock causally matched, not an exact command-native trajectory ID'),
                  control_output_generated=False, production_source_changed=False, ground_truth_used_for_control=False,
                  observer_source_sha256=sha(OBSERVER), script_sha256=sha(Path(__file__)),
                  input_sha256={str(p.relative_to(ROOT)): sha(p) for p in [run/'pose_audit.jsonl', run/'joint_stop_adapter.jsonl', run/'navigation_audit.jsonl', run/'source_snapshot.tar.gz', alignment_path]},
                  unchanged_gates=dict(position_rms_m=.02, angular_rms_rad=.04, max_pose_gap_ns=150_000_000,
                      max_command_gap_ns=100_000_000, max_pose_age_ns=250_000_000, min_span='window minus .05s'),
                  limitations=['Overlapping fits are not independent plant trials or instantaneous response.',
                      'No new feedback, velocity command, guard, cap or arrival change.',
                      'Fit residual gates are unvalidated quality tags, not estimator covariance.',
                      'Existing observed_motion_ready expects driving but actual adapter uses walk; feedback_eligible always false.',
                      'Frozen gait cycle is nominal; contact-recorded cycle phase is not inferred.',
                      'Long-window body velocity uses current-end body axes; it is a finite-window fit, not integrated gait odometry.'])
    for width in WINDOWS:
        label = f'{width/NS:.1f}s'
        o = Observer(window_ns=width, min_span_ns=width-50_000_000)
        go = Observer(window_ns=width, min_span_ns=width-50_000_000)
        j = 0
        reasons, transitions, counts = Counter(), Counter(), Counter()
        groups = defaultdict(lambda: defaultdict(list))
        labels = []
        with (output/f'snapshots_{label}.jsonl').open('x') as handle:
            for p in ps:
                t = p['stamp_ns']
                while j < len(commands) and commands[j]['t'] <= t:
                    c = commands[j]
                    for observer in (o, go):
                        observer.observe_applied_command(c['t'], c['c'], c['state'], c['context'])
                    j += 1
                o.observe_pose(t, p['p'], p['q'], t)
                gp = gt_poses[t]
                if gp is not None:
                    go.observe_pose(t, gp[0], gp[1], t)
                if p['stage'] not in ('exploring', 'returning', 'navigating'):
                    continue
                s = o.snapshot(t, t)
                gs = go.snapshot(t, t) if gp is not None else None
                reasons[s['invalid_reason'] or 'valid'] += 1
                counts['active_snapshots'] += 1
                counts['unpaired_GT'] += int(gp is None)
                causes = transition_causes(o, s['window_start_ns'], t) if s['invalid_reason'] == 'execution_transition' else []
                transitions.update(causes)
                cls = dict(stage=p['stage'], stamp_ns=t, context=s.get('context_id'),
                           command_sign=None, opposite_body_SLAM=False, opposite_body_GT_confirmed=False,
                           opposite_heading_SLAM=False, opposite_heading_GT_confirmed=False,
                           opposite_body_GT_confirmed_consistent_command_sign=False)
                cq = command_window_quality(o,s['window_start_ns'],t)
                if s['available']:
                    mode = s['execution_mode']
                    key = mode + ('_valid' if s['valid'] else '_fit_rejected')
                    cmd = np.asarray(s['actual_command_average'])
                    motion = np.asarray([s['window_body_twist']['linear'][0], s['window_body_twist']['angular'][2], s['heading_rate_world']])
                    groups[key]['motion'].append(motion)
                    groups[key]['command'].append(cmd)
                    groups[key]['effective_age_s'].append(s['effective_measurement_age_ns']/NS)
                    groups[key]['fit_residuals'].append(list(s['fit_residuals'].values()))
                    match = bool(gs and gs.get('available') and gs.get('window_start_ns') == s['window_start_ns'] and gs.get('stamp_ns') == t)
                    if match:
                        gm = np.asarray([gs['window_body_twist']['linear'][0], gs['window_body_twist']['angular'][2], gs['heading_rate_world']])
                        groups[key]['GT_motion'].append(gm)
                        groups[key]['SLAM_minus_GT'].append(motion-gm)
                    if s['valid'] and mode == 'walk':
                        counts['valid_walk'] += 1
                        cls['command_sign'] = int(np.sign(cmd[2]))
                        groups[key]['vx_positive_headroom'].append(max(0., .12-cmd[0]))
                        groups[key]['yaw_same_sign_headroom'].append(max(0., .08-abs(cmd[2])))
                        if cq:
                            groups[key]['latest_vx_positive_headroom'].append(cq['latest_vx_positive_headroom'])
                            groups[key]['latest_yaw_same_direction_headroom'].append(cq['latest_yaw_same_direction_headroom'])
                            groups[key]['yaw_saturated_time_fraction'].append(cq['yaw_saturated_time_fraction'])
                            counts['valid_walk_yaw_sign_changes_within_window'] += int(not cq['yaw_sign_consistent'])
                        counts['forward_saturated_ge_118'] += int(cmd[0] >= .118)
                        counts['yaw_saturated_ge_078'] += int(abs(cmd[2]) >= .078)
                        counts['body_sign_comparable_SLAM'] += int(abs(cmd[2]) > .02 and abs(motion[1]) > .02)
                        counts['heading_sign_comparable_SLAM'] += int(abs(cmd[2]) > .02 and abs(motion[2]) > .02)
                        cls['opposite_body_SLAM'] = opposite(cmd[2], motion[1])
                        cls['opposite_heading_SLAM'] = opposite(cmd[2], motion[2])
                        if match:
                            counts['paired_valid_walk'] += 1
                            counts['body_sign_comparable_GT'] += int(abs(cmd[2]) > .02 and abs(gm[1]) > .02)
                            counts['heading_sign_comparable_GT'] += int(abs(cmd[2]) > .02 and abs(gm[2]) > .02)
                            counts['opposite_body_GT'] += int(opposite(cmd[2], gm[1]))
                            counts['opposite_heading_GT'] += int(opposite(cmd[2], gm[2]))
                            cls['opposite_body_GT_confirmed'] = cls['opposite_body_SLAM'] and opposite(cmd[2], gm[1])
                            cls['opposite_heading_GT_confirmed'] = cls['opposite_heading_SLAM'] and opposite(cmd[2], gm[2])
                            cls['opposite_body_GT_confirmed_consistent_command_sign'] = bool(cls['opposite_body_GT_confirmed'] and cq and cq['yaw_sign_consistent'])
                for key in ('opposite_body_SLAM', 'opposite_heading_SLAM', 'opposite_body_GT_confirmed', 'opposite_heading_GT_confirmed', 'opposite_body_GT_confirmed_consistent_command_sign'):
                    counts[key] += int(cls[key])
                labels.append(cls)
                handle.write(json.dumps(dict(stage=p['stage'], slam=s, GT_evaluation_only=gs, classifications=cls, command_window_quality=cq, transition_causes=causes), allow_nan=False)+'\n')
        episode_result = {}
        for field in ('opposite_body_SLAM', 'opposite_body_GT_confirmed', 'opposite_heading_SLAM', 'opposite_heading_GT_confirmed', 'opposite_body_GT_confirmed_consistent_command_sign'):
            es = episodes(labels, field)
            episode_result[field] = dict(count=len(es), max_duration_s=max((x['duration_ns']/NS for x in es), default=0.),
                at_least_one_nominal_cycle_count=sum(x['at_least_one_nominal_gait_cycle'] for x in es),
                episodes=es)
        (output/f'episodes_{label}.json').write_text(json.dumps(episode_result, indent=2)+'\n')
        report['windows'][label] = dict(window_ns=width, min_span_ns=width-50_000_000,
            reasons=dict(reasons), transition_causes=dict(transitions), counts=dict(counts),
            statistics={key:{metric:stats(values) for metric,values in group.items()} for key,group in groups.items()},
            opposite_episodes_summary={key:{k:v for k,v in values.items() if k!='episodes'} for key,values in episode_result.items()},
            angular_rotation_gate_rad=.6, mode_and_trajectory_transition_gate_preserved=True)
        print(json.dumps(dict(window=label, reasons=dict(reasons), counts=dict(counts)), ensure_ascii=False), flush=True)
    (output/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    receipt = dict(result_sha256=sha(output/'result.json'), script_sha256=sha(Path(__file__)),
                   observer_sha256=sha(OBSERVER), scope='Offline excluded comparison only',
                   outputs={p.name:sha(p) for p in output.iterdir() if p.is_file()})
    (output/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=ROOT/'runs/20261001_203152_88725b')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--alignment', type=Path, default=ROOT/'slam/test_results/full15_final_independent_evidence.json')
    args = parser.parse_args()
    compare(args.run.resolve(), args.output.resolve(), args.alignment.resolve())
