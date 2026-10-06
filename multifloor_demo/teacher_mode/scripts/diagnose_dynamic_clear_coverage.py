#!/usr/bin/env python3
"""Quantify frozen clear-evidence gaps without changing any acceptance receipt."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def evaluate(run):
    run = Path(run).resolve()
    summary_path = run / 'summary_dynamic_obstacle_independent.json'
    original = summary_path.read_bytes()
    summary = json.loads(original)
    status = rows(run / 'navigation_status.jsonl')
    poses = {r['stamp_ns']: r for r in rows(run / 'navigation_slam_poses.jsonl')}
    replay = summary['checks']['actual_registered_cloud_corridor_clear_after_withdrawal']['exact_consumer_buffer_replays']
    leaving = summary['checks']['actual_box_entry_block_and_withdrawal']['phase_times_s']['leaving']
    details = []
    for row in replay:
        cs = row['stamp_ns']
        same = [r for r in status if r.get('actual_cloud_message_stamp_ns') == cs]
        wp = [r for r in same if r.get('waypoint_index') == 0]
        steer = [r for r in wp if r.get('steering')]
        valid = [r for r in steer if r.get('ros_sim_time', 0) >= cs/1e9 and r.get('ros_sim_time', 0)-cs/1e9 < .3]
        integer, near = None, []
        if not same:
            reason = 'no_status_snapshot_with_exact_cloud_stamp'
        elif not wp:
            reason = 'same_cloud_status_not_outward_waypoint_zero'
        elif not steer:
            reason = 'same_cloud_waypoint_zero_status_without_steering'
        elif not valid:
            reason = 'same_cloud_steering_status_outside_original_causal_point3_window'
        else:
            used = min(valid, key=lambda r: r['ros_sim_time'])
            integer = int(round(used['steering']['odom_stamp']*1e9))
            if integer in poses:
                reason = 'exact_control_pose_and_steering_available'
            else:
                near = [key for key in poses if abs(key-integer) <= 1]
                reason = 'float_reconstructed_control_stamp_missing_exact_ns'
        details.append({'cloud_stamp_ns': cs, 'fixture_phase': row['phase'], 'reason': reason,
            'same_stamp_status_snapshots': len(same), 'eligible_original_status_snapshots': len(valid),
            'float_reconstructed_control_stamp_ns': integer, 'original_raw_pose_stamps_within_one_ns_diagnostic_only': near,
            'after_leaving': cs/1e9 >= leaving,
            'actual_consumer_filtered_buffer_exact': row['actual_consumer_filtered_buffer_exact'],
            'recorded_corridor_clear': row['replayed_recorded_steering_corridor'] is not None and not row['replayed_recorded_steering_corridor'][0],
            'goal_corridor_clear': not row['replayed_goal_corridor'][0]})
    start, previous, best, best_window = None, None, 0., None
    for detail in details:
        if not detail['after_leaving'] or detail['reason'] != 'exact_control_pose_and_steering_available':
            continue
        time = detail['cloud_stamp_ns']*1e-9
        good = detail['actual_consumer_filtered_buffer_exact'] and detail['recorded_corridor_clear'] and detail['goal_corridor_clear']
        if not good:
            start = None
        elif start is None or previous is not None and time-previous > .300000001:
            start = time
        if good and time-start > best:
            best, best_window = time-start, [start, time]
        previous = time
    result = {'schema': 1, 'status': 'recorded_coverage_diagnostic', 'run_dir': str(run),
        'frozen_clear_check_status': summary['checks']['actual_registered_cloud_corridor_clear_after_withdrawal']['status'],
        'all_replays': dict(Counter(r['reason'] for r in details)),
        'after_leaving_replays': dict(Counter(r['reason'] for r in details if r['after_leaving'])),
        'original_criteria_longest_exact_corroborated_clear_s': best, 'longest_world_window_s': best_window,
        'prospective_correction_only': 'Missing status/steering/control integer stamps cannot be manufactured from model truth. '
            'A one-ns diagnostic neighbor is reported but not used to repair old frozen acceptance. '
            'A future runtime can retain exact integer pose stamp and per-guard original arguments/result/hash for replay.',
        'details': details, 'input_sha256': {n: sha(run / n) for n in [
            'summary_dynamic_obstacle_independent.json', 'navigation_status.jsonl', 'navigation_slam_poses.jsonl']},
        'analyzer_sha256': sha(__file__), 'original_frozen_receipt_bytes_preserved': summary_path.read_bytes() == original}
    (run / 'dynamic_clear_evidence_coverage_diagnostic.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: result[k] for k in ['all_replays','after_leaving_replays','original_criteria_longest_exact_corroborated_clear_s','longest_world_window_s']}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    evaluate(parser.parse_args().run)
