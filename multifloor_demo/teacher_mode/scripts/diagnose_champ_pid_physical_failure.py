#!/usr/bin/env python3
"""Offline CHAMP initialization/route diagnostic, never changes frozen gates."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    helper_path = Path(__file__).with_name('analyze_pid_navigation_v3.py')
    spec = importlib.util.spec_from_file_location('diagnostic_helpers', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    anchor = json.loads((run / 'navigation_anchor.json').read_text())
    poses = [json.loads(x) for x in (run / 'navigation_slam_poses.jsonl').open()]
    summary = json.loads((run / 'summary_pid_navigation_independent.json').read_text())
    anchor_pose = [p for p in poses if p['stamp_ns'] == anchor['pose_stamp_ns']]
    if len(anchor_pose) != 1:
        raise ValueError('Exactly one original raw anchor pose is required')
    with np.load(run / 'pid_navigation_independent_arrays.npz') as data:
        nt, pos = data['native_world_time_s'], data['native_position']
        quaternion = data['native_quaternion_wxyz']
        rot, rpy = helper.rotation(quaternion), helper.rpy(quaternion)
        def native_index(stamp):
            t = stamp / 1e9
            i = int(np.searchsorted(nt, t, side='right') - 1)
            if i < 0 or not 0 <= t - nt[i] <= .005001:
                raise ValueError('No causal native state within5.001ms')
            return i
        i = native_index(anchor['pose_stamp_ns'])
        q = anchor_pose[0]['quaternion']
        slamrot = helper.rotation(np.array([[q[3], *q[:3]]]))[0]
        alignment = rot[i] @ slamrot.T
        translation = pos[i] - alignment @ anchor_pose[0]['position']
        comparisons = []
        for arrival in summary['metrics']['arrivals']:
            receipt = arrival.get('claimed_receipt')
            if not receipt:
                continue
            matching = [p for p in poses if p['stamp_ns'] == receipt['stamp_ns']]
            if len(matching) != 1:
                raise ValueError('Original arrival pose is missing or duplicated')
            p = matching[0]
            j = native_index(p['stamp_ns'])
            mapped = alignment @ p['position'] + translation
            comparisons.append({'goal_id': receipt['goal_id'], 'original_stamp_ns': p['stamp_ns'],
                'causal_native_world_time_s': float(nt[j]),
                'raw_SLAM_position': p['position'], 'offline_registered_SLAM_world_position': mapped.tolist(),
                'actual_native_position': pos[j].tolist(),
                'position_error_m': float(np.linalg.norm(mapped - pos[j])),
                'registration_frozen_at_original_anchor': True})
        audit = nt >= .1
        bad = np.flatnonzero(audit & (np.max(np.abs(rpy[:, :2]), axis=1) > .65))
        peak = int(np.flatnonzero(audit)[np.argmax(np.max(np.abs(rpy[audit, :2]), axis=1))])
        distance = data['lateral_error_m']
        failed = np.flatnonzero(audit & (distance > .45))
        worst = int(np.argmax(np.where(audit, distance, -np.inf)))
        result = {'schema': 'champ_actual_pid_physical_diagnostic/v1', 'run': str(run),
            'acceptance_status_unchanged': summary['status'],
            'post_bootstrap_pose_violation': {'threshold_rad': .65, 'begin_s': .1,
                'actual_200hz_samples_above_limit': len(bad),
                'first_world_time_s': float(nt[bad[0]]) if len(bad) else None,
                'last_world_time_s': float(nt[bad[-1]]) if len(bad) else None,
                'peak_world_time_s': float(nt[peak]), 'peak_rpy_rad': rpy[peak].tolist(),
                'peak_body_position': pos[peak].tolist()},
            'original_fixture_route_violation': {'distance_is': 'bounded predefined world XY polyline, including endpoint overrun',
                'threshold_m': .45, 'first_world_time_s': float(nt[failed[0]]) if len(failed) else None,
                'first_body_position': pos[failed[0]].tolist() if len(failed) else None,
                'maximum_world_time_s': float(nt[worst]), 'maximum_distance_m': float(distance[worst]),
                'maximum_body_position': pos[worst].tolist(),
                'pure_world_y_deviation_at_maximum_m': float(abs(pos[worst, 1] + .7))},
            'immutable_anchor_native_state': {'original_anchor_stamp_ns': anchor['pose_stamp_ns'],
                'causal_native_world_time_s': float(nt[i]), 'body_position': pos[i].tolist(),
                'rpy_rad': rpy[i].tolist(), 'initial_position_from_original_case_spawn': (pos[i] - [6, -.7, .4]).tolist()},
            'offline_alignment': {'rotation': alignment.tolist(), 'translation': translation.tolist(),
                'uses_full_SE3_once_only': True, 'source': 'Original raw SLAM anchor and causal PostUpdate body state',
                'navigation_received_truth': False, 'test_route_or_goal_redefined': False},
            'original_arrival_truth_diagnostic': comparisons,
            'causal_limitations': ['An original world coordinate and a SLAM coordinate cannot be subtracted without registration.',
                'Small registered arrival error does not prove continuous map accuracy or repair physical route/initialization failures.',
                'No old criterion or original fixture reference is translated to make the result favorable.'],
            'input_hashes': {n: sha(run / n) for n in ['navigation_anchor.json', 'navigation_slam_poses.jsonl',
                'summary_pid_navigation_independent.json', 'pid_navigation_independent_arrays.npz']},
            'analyzer_sha256': sha(__file__), 'frozen_helper_sha256': sha(helper_path),
            'original_summary_or_arrays_overwritten': False}
    output = run / 'pid_physical_failure_diagnostic.json'
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(output)


if __name__ == '__main__':
    main()
