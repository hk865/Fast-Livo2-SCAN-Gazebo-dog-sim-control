#!/usr/bin/env python3
"""Offline CDR/native review only. Does not create any ROS context or node."""
import argparse
import bisect
import collections
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation, Slerp


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def tilt(q):
    x, y, z, w = q
    return max(abs(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
               abs(math.asin(max(-1., min(1., 2*(w*y-z*x))))))


def pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]


def analyze(run):
    spec = importlib.util.spec_from_file_location('archived_readonly_observer', run/'staging/observer.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    types = {k: t for k, _, t in module.TOPICS}
    arrays = collections.defaultdict(list)
    counts, nonzero = collections.Counter(), collections.Counter()
    nominal = None
    target_max_change = 0.
    gaps, previous = {}, {}
    first_bad_raw = None
    for header, cdr in module.read_capture(run/'actuator_startup.cdrlog'):
        kind = header['kind']
        counts[kind] += 1
        if not cdr or kind not in ('imu', 'truth', 'slam', 'jtc', 'joint', 'target', 'actual_command', 'safe_command', 'clock'):
            continue
        msg = deserialize_message(cdr, types[kind])
        stamp, wall = header['header_stamp_ns'], header['wall_monotonic_ns']
        if kind in previous:
            old = previous[kind]
            entry = {'previous_stamp_ns': old['header_stamp_ns'], 'stamp_ns': stamp,
                     'wall_gap_ns': wall-old['wall_monotonic_ns'],
                     'header_gap_ns': None if stamp is None or old['header_stamp_ns'] is None else stamp-old['header_stamp_ns']}
            if kind not in gaps or entry['wall_gap_ns'] > gaps[kind]['wall_gap_ns']:
                gaps[kind] = entry
        previous[kind] = header
        if kind == 'imu':
            q = msg.orientation
            value = tilt([q.x, q.y, q.z, q.w])
            arrays[kind].append({'t': stamp, 'tilt': value})
            if stamp >= 10_000_000_000 and value >= .50 and first_bad_raw is None:
                first_bad_raw = {'stamp_ns': stamp, 'tilt': value}
        if kind in ('truth', 'slam'):
            p, q = pose(msg)
            arrays[kind].append({'t': stamp, 'p': p, 'q': q, 'tilt': tilt(q)})
        if kind == 'jtc':
            arrays[kind].append({'t': stamp, 'q': list(msg.feedback.positions), 'v': list(msg.feedback.velocities),
                                 'refv': list(msg.reference.velocities), 'eff': list(msg.output.effort)})
        if kind == 'target':
            qq = list(msg.points[0].positions)
            assert len(qq) == 12 and all(math.isfinite(x) for x in qq)
            if nominal is None:
                nominal = qq
            target_max_change = max(target_max_change, max(abs(a-b) for a,b in zip(qq, nominal)))
        if kind.endswith('command'):
            nonzero[kind] += int(any([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]))
    windows = {}
    for label, low, high in [('startup', 0, 10), ('gate', 10, 13), ('post_gate', 13, 17), ('late', 20, 40)]:
        selected = {k: [x for x in arrays[k] if low*1e9 <= x['t'] <= high*1e9] for k in ('imu','truth','jtc')}
        imu, gt, jtc = (selected[k] for k in ('imu','truth','jtc'))
        windows[label] = {'samples': {k: len(v) for k,v in selected.items()},
                         'raw_tilt_max': max((x['tilt'] for x in imu), default=None),
                         'truth_tilt_max': max((x['tilt'] for x in gt), default=None),
                         'truth_z_range': [min((x['p'][2] for x in gt), default=None), max((x['p'][2] for x in gt), default=None)],
                         'reference_v_max': max((abs(v) for x in jtc for v in x['refv']), default=None),
                         'feedback_v_max': max((abs(v) for x in jtc for v in x['v']), default=None),
                         'output_effort_command_max': max((abs(v) for x in jtc for v in x['eff']), default=None)}
    imu_window = [x['t'] for x in arrays['imu'] if 10_000_000_000 <= x['t'] <= 39_900_000_000]
    raw_coverage = {'expected': 29901, 'actual': len(imu_window), 'unique': len(set(imu_window)),
                    'first_ns': min(imu_window, default=None), 'last_ns': max(imu_window, default=None),
                    'all_adjacent_1ms': all(b-a == 1_000_000 for a,b in zip(imu_window, imu_window[1:]))}
    native = {}
    for path in run.glob('controller_update_native.*.jsonl'):
        data = [json.loads(line) for line in path.read_text().splitlines()]
        for name in sorted({r['controller'] for r in data if r.get('kind') == 'call'}):
            rows = [r for r in data if r.get('kind') == 'call' and r['controller'] == name]
            periods = collections.Counter(r['period_ns'] for r in rows)
            times = collections.Counter(r['time_ns'] for r in rows)
            native[name] = {'calls': len(rows), 'dt0': periods[0], 'negative_dt': sum(n for p,n in periods.items() if p < 0),
                            'dt_max_ns': max(periods), 'dt_min_ns': min(periods), 'periods': dict(periods),
                            'longest_same_time_count': max(times.values()),
                            'dt0_stamps': dict(collections.Counter(r['time_ns'] for r in rows if r['period_ns'] == 0)),
                            'all_return_period_equal_input': all(r['has_return_period'] and r['return_period_ns'] == r['period_ns'] for r in rows)}
    truth = sorted(arrays['truth'], key=lambda x: x['t'])
    stamps = [x['t'] for x in truth]
    transform = None
    errors, unpaired = [], []
    for raw in arrays['slam']:
        t = raw['t']
        if not 13_000_000_000 <= t <= 40_000_000_000:
            continue
        i = bisect.bisect_left(stamps, t)
        if i < len(stamps) and stamps[i] == t:
            gp, gq = np.array(truth[i]['p']), np.array(truth[i]['q'])
        elif not 0 < i < len(stamps) or stamps[i]-stamps[i-1] > 150_000_000:
            unpaired.append(t)
            continue
        else:
            weight = (t-stamps[i-1])/(stamps[i]-stamps[i-1])
            gp = (1-weight)*np.array(truth[i-1]['p'])+weight*np.array(truth[i]['p'])
            # Relative zero epoch avoids converting epoch-ns to imprecise float seconds.
            gq = Slerp([0.,1.], Rotation.from_quat([truth[i-1]['q'], truth[i]['q']]))([weight]).as_quat()[0]
        sp = np.array(raw['p'])
        if transform is None:
            rot = Rotation.from_quat(gq).as_matrix() @ Rotation.from_quat(raw['q']).as_matrix().T
            off = gp-rot@sp
            transform = {'stamp_ns': t, 'rotation': rot.tolist(), 'translation': off.tolist()}
        errors.append(float(np.linalg.norm(rot@sp+off-gp)))
    precision = {'time_domain_ns': [13_000_000_000,40_000_000_000], 'alignment': transform,
                 'transform_fits': int(transform is not None), 'samples': len(errors), 'unpaired_stamps_ns': unpaired,
                 'rmse_m': float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                 'max_m': max(errors, default=None)}
    report = {'run': str(run), 'windows': windows, 'native_actual_arguments': native, 'raw_1000hz_coverage': raw_coverage,
              'SLAM_single_initial_SE3_diagnostic': precision, 'command_nonzero': dict(nonzero), 'target_position_change': target_max_change,
              'first_bad_postwarm_raw': first_bad_raw, 'counts': dict(counts), 'max_callback_wall_gaps': gaps,
              'observer': json.loads((run/'observer_result.json').read_text()), 'cleanup': json.loads((run/'process_cleanup.json').read_text()),
              'input_sha256': {n: sha(run/n) for n in ('actuator_startup.cdrlog','observer_result.json','process_cleanup.json','diagnostic_manifest.json')},
              'limits': ['Standing-only component; no commanded-motion/region/map-save/full-demo acceptance.',
                         'Native ControllerInterface arguments are direct observations; CDR header gap is not publication-loss proof.',
                         'JTC output.effort is command, not measured applied torque.',
                         'GT only used in independent diagnostic single-first-SE3 comparison, never control.',
                         'Different fresh runs need not have identical microscopic physical initial states.',
                         'Actual child shutdown errors must be retained separately from collected standing samples.']}
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = analyze(args.run.resolve())
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: result[k] for k in ('windows','native_actual_arguments','raw_1000hz_coverage','SLAM_single_initial_SE3_diagnostic','command_nonzero','target_position_change')}, indent=2))
