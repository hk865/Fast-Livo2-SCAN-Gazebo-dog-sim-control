#!/usr/bin/env python3
"""Offline source separation using immutable real-run sensor and core logs.

Ground truth is an offline comparison only. This script never publishes or
changes recorded samples, and it does not turn a failed mission into a pass.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def records(path, source):
    return [row for line in path.open() if (row := json.loads(line))['source'] == source]


def grouped(path):
    groups = defaultdict(list)
    for row in np.loadtxt(path):
        groups[round(float(row[0]), 3)].append(row)
    return groups


def interpolation(t, stamps, values):
    return np.column_stack([np.interp(t, stamps, values[:, k]) for k in range(3)])


def integral(lo, hi, stamps, values):
    if lo < stamps[0] or hi > stamps[-1]:
        raise ValueError('Requested interval is outside the actual input coverage')
    ts = np.r_[lo, stamps[(stamps > lo) & (stamps < hi)], hi]
    samples = interpolation(ts, stamps, values)
    return np.trapz(samples, ts, axis=0), float(np.diff(ts).max())


def audit(run, lo, hi):
    imu = records(run / 'sensor_audit.jsonl', 'imu')
    truth = records(run / 'pose_audit.jsonl', 'truth')
    slam = records(run / 'pose_audit.jsonl', 'slam')
    stamps = np.array([row['stamp'] for row in imu])
    accelerations = np.array([row['acceleration'] for row in imu])
    orientations = Rotation.from_quat([row['quaternion'] for row in imu])
    world_acc = orientations.apply(accelerations) - [0, 0, 9.81]
    truth_stamps = np.array([row['stamp'] for row in truth])
    truth_pos = np.array([row['p'] for row in truth])
    # A central position difference is an offline estimate, not an IMU source.
    truth_vel = np.gradient(truth_pos, truth_stamps, axis=0)
    pre = grouped(run / 'fastlivo_mat_pre.txt')
    out = grouped(run / 'fastlivo_mat_out.txt')
    first_out = np.loadtxt(run / 'fastlivo_mat_out.txt')[0]
    if not np.allclose(first_out[4:7], slam[0]['p'], atol=1e-6):
        raise ValueError('The initial internal/official position pair does not match')
    offset = float(slam[0]['stamp'] - first_out[0])
    rows = []
    previous = None
    for relative_stamp, outputs in sorted(out.items()):
        inputs = pre[relative_stamp]
        if len(outputs) != 2 or len(inputs) != 2:
            previous = None
            continue
        current = float(relative_stamp + offset)
        if previous is not None and lo <= current <= hi:
            previous_stamp, previous_output = previous
            raw_dv, gap = integral(previous_stamp, current, stamps, world_acc)
            gt_v = interpolation(np.array([previous_stamp, current]), truth_stamps, truth_vel)
            gt_p = interpolation(np.array([previous_stamp, current]), truth_stamps, truth_pos)
            rows.append({
                'stamp': current, 'previous_stamp': previous_stamp,
                'pre_position': inputs[0][4:7].tolist(),
                'lio_position': outputs[0][4:7].tolist(),
                'vio_position': outputs[1][4:7].tolist(),
                'imu_propagation_position_delta': (inputs[0][4:7] - previous_output[4:7]).tolist(),
                'lio_position_correction': (outputs[0][4:7] - inputs[0][4:7]).tolist(),
                'vio_position_correction': (outputs[1][4:7] - inputs[1][4:7]).tolist(),
                'imu_propagation_velocity_delta': (inputs[0][7:10] - previous_output[7:10]).tolist(),
                'raw_world_acceleration_velocity_delta': raw_dv.tolist(),
                'ground_truth_central_velocity_delta': (gt_v[1] - gt_v[0]).tolist(),
                'ground_truth_position_delta': (gt_p[1] - gt_p[0]).tolist(),
                'largest_actual_imu_gap': gap,
                'lio_velocity': outputs[0][7:10].tolist(),
                'vio_velocity': outputs[1][7:10].tolist(),
                'gyro_bias': outputs[1][10:13].tolist(),
                'acceleration_bias': outputs[1][13:16].tolist(),
            })
        previous = current, outputs[1]
    windows = []
    for begin, end in [(498.1, 499.1), (525.9, 526.9), (526.9, 527.6), (lo, hi)]:
        dv, gap = integral(begin, end, stamps, world_acc)
        gv = interpolation(np.array([begin, end]), truth_stamps, truth_vel)
        gp = interpolation(np.array([begin, end]), truth_stamps, truth_pos)
        selected = [row for row in rows if begin < row['stamp'] <= end + 1e-7]
        mask = (stamps >= begin) & (stamps <= end)
        windows.append({
            'start': begin, 'end': end, 'actual_imu_samples': int(mask.sum()),
            'largest_actual_imu_gap': gap,
            'raw_world_acceleration_velocity_delta': dv.tolist(),
            'ground_truth_central_velocity_delta': (gv[1] - gv[0]).tolist(),
            'ground_truth_position_delta': (gp[1] - gp[0]).tolist(),
            'raw_world_acceleration_std': world_acc[mask].std(axis=0).tolist(),
            'core_interval_count': len(selected),
            **{key + '_sum': np.array([row[key] for row in selected]).sum(axis=0).tolist()
               for key in ['imu_propagation_position_delta', 'lio_position_correction', 'vio_position_correction']},
        })
    return {
        'run': str(run.resolve()), 'interval': [lo, hi],
        'scope': 'offline recorded input/source audit; no control, pose correction or acceptance override',
        'internal_time_offset': offset,
        'method': {
            'raw_acceleration': 'actual raw IMU quaternion rotates specific acceleration into world; subtract [0,0,9.81]; trapezoidal integration with interpolated interval endpoints',
            'ground_truth_velocity': 'central difference of actual 50Hz position; approximate comparison, not exact contact-scale acceleration',
            'core_separation': 'ordered paired LIO/VIO pre/out rows; previous VIO out to LIO pre is IMU propagation; each pre to its out is measurement correction',
            'point_time': 'configured lidar_type=0 generic handler assigns curvature=0 to every point; no supplied per-point time is consumed',
        },
        'findings': [
            'Large Z excursions already exist in LIO-pre and mainly accumulate through propagated velocity. Visual position corrections are much smaller in the examined pulse.',
            'Raw world-frame acceleration itself has substantial integrated velocity discrepancy from physical position-derived velocity during gait; the core propagation follows that input.',
            '100Hz sensor versus 1000Hz physics and instantaneous acceleration sampling is a plausible cause, awaiting controlled physical frequency comparison.',
            'A separate synchronous instantaneous-cloud boundary issue is visible in source: curvature=0 at equal LiDAR/camera stamp is postponed by strict <; the subsequent zero-offset point is skipped by the deskew strict > boundary. A linked-library fixture and actual slice diagnostics are required before claiming a fix.',
        ],
        'limitations': [
            'No original raw LiDAR point bytes or estimator plane residual/covariance traces were recorded in this run; registered cropped navigation clouds cannot uniquely distinguish every erroneous plane constraint or removed self return.',
            'Ground-truth central velocity is approximate at 20ms cadence; source-level acceleration alias diagnosis needs actual higher-rate A/B confirmation.',
            'This run failed waypoint 8, with only the first seven ordered joint checkpoints passed; no map-save or complete multilayer mission is claimed.',
        ],
        'windows': windows, 'core_rows': rows,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--start', type=float, default=437.5)
    parser.add_argument('--end', type=float, default=532.7)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = audit(args.run, args.start, args.end)
    output = args.output or args.run / 'slam_slope_input_audit.json'
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'output': str(output), 'core_rows': len(report['core_rows']), 'windows': report['windows']}))


if __name__ == '__main__':
    main()
