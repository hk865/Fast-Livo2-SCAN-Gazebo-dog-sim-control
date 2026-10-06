#!/usr/bin/env python3
"""Bind NEW completed capture/index and prefix clocks; no ROS/physics."""
import argparse
import json
from pathlib import Path
from input_identity import TOPICS, load_index, sha, summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--index-dir', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args(); run = args.run.resolve(); index = args.index_dir.resolve()
    manifest = json.loads((index / 'CAPTURED_INPUT_MANIFEST.json').read_text())
    if manifest['schema'] != 'go2_pipeline_v19_captured_input_manifest/v1': raise ValueError('Foreign manifest')
    expected_bag = (run / 'sensor_input_bag').resolve()
    if Path(manifest['bag']).resolve() != expected_bag: raise ValueError('Foreign capture run/index')
    if sha(index / 'captured_inputs.jsonl') != manifest['index_sha256'] or sha(expected_bag / 'metadata.yaml') != manifest['metadata_sha256']:
        raise ValueError('Changed original index/metadata')
    rows = load_index(index / 'captured_inputs.jsonl'); actual = summary(rows)
    for key, value in actual.items():
        if manifest.get(key) != value: raise ValueError('Changed index summary ' + key)
    lifecycle = json.loads((run / 'sensor_capture_lifecycle.json').read_text())
    loaded = json.loads((run / 'slam_loaded_binary.json').read_text())
    runtime = json.loads((run / 'runtime_manifest.json').read_text())
    params = json.loads((run / 'navigation_fastlivo.yaml').read_text())['/**']['ros__parameters']
    bindings = {}
    required = ('sensor_capture_setup.json', 'sensor_capture_lifecycle.json', 'source_manifest.json',
                'navigation_slam_contract.json', 'navigation_fastlivo.yaml', 'navigation_camera.yaml',
                'navigation_profile.json', 'runtime_manifest.json', 'slam_loaded_binary.json')
    for name in required: bindings[str(run / name)] = sha(run / name)
    checks = {}
    def ck(name, value, **details): checks[name] = {'status': 'passed' if value else 'failed', 'passed': bool(value), **details}
    stop = lifecycle.get('recorder_stop', {})
    ck('capture_original_ready_before_physics_and_clean_exit',
       lifecycle.get('schema') == 'v19_sensor_capture_lifecycle/v1' and Path(lifecycle['run']).resolve() == run
       and lifecycle.get('ready_before_physics') is True and lifecycle.get('error') is None
       and lifecycle.get('baseline_runtime_error') is None and lifecycle.get('historical_raw_not_used') is True
       and stop.get('returncode') == 0 and stop.get('remaining_owned_group_members') == [])
    ck('actual_simulation_only_no_truth_navigation_bound', loaded.get('verified') is True
       and loaded.get('before_physics') is True and Path(loaded['run']).resolve() == run
       and runtime.get('error') is None and runtime.get('simulation_only') is True
       and runtime.get('real_robot') is False and runtime.get('navigation_ground_truth_used') is False)
    ck('all_complete_CDR_records_indexed_and_budget', manifest['all_metadata_messages_indexed'] is True
       and manifest['within_raw_budget'] is True and actual['complete_topic_set'] is True
       and manifest['bag_files_bytes'] == lifecycle.get('bag_bytes_final'))
    stamps = {topic: [r['source_ns'] for r in rows if r['topic'] == topic] for topic in TOPICS}
    per_topic = {}
    for topic, values in stamps.items():
        gaps = [b-a for a, b in zip(values, values[1:])]
        per_topic[topic] = {'first_ns': values[0], 'last_ns': values[-1], 'count': len(values),
                            'minimum_gap_ns': min(gaps), 'maximum_gap_ns': max(gaps),
                            'duplicate_or_backward': sum(g <= 0 for g in gaps)}
    ck('original_clock_5ms_grid_no_internal_gap', all(b-a == 5_000_000 for a, b in zip(stamps['/clock'], stamps['/clock'][1:])),
       first_clock_ns=stamps['/clock'][0], last_clock_ns=stamps['/clock'][-1],
       limit='Complete within observed clock interval; does not invent an earlier unpublished/uncaptured first clock')
    ck('original_IMU_5ms_grid_no_internal_gap', all(b-a == 5_000_000 for a, b in zip(stamps['/demo/teacher/slam/imu'], stamps['/demo/teacher/slam/imu'][1:])))
    ck('lidar_and_RGB_original_source_stamp_sequences_identical', stamps['/demo/slam/lidar_filtered'] == stamps['/demo/teacher/slam/image'])
    ck('sensor_headers_within_final_source_clock', max(max(values) for topic, values in stamps.items() if topic != '/clock') <= stamps['/clock'][-1])
    prefix = [value for value in stamps['/demo/teacher/slam/imu'] if value <= 3_000_000_000]
    ck('contains_600_IMU_initialization_prefix',
       params['imu']['stationary_initialization_en'] is True and params['imu']['imu_int_frame'] == 600
       and len(prefix) >= 600 and stamps['/demo/teacher/slam/imu'][0] == 5_000_000
       and stamps['/demo/slam/lidar_filtered'][0] == stamps['/demo/teacher/slam/image'][0] == 10_000_000,
       IMU_count_through_3s=len(prefix), qualification='Prefix coverage; stationary estimator acceptance and exact original callback order require production replay/trace')
    result = {'schema': 'go2_pipeline_v19_capture_prefix_audit/v1', 'run': str(run),
              'status': 'passed_captured_input_prefix_only' if all(x['passed'] for x in checks.values()) else 'failed',
              'checks': checks, 'source_bindings_sha256': bindings,
              'index_manifest_sha256': sha(index / 'CAPTURED_INPUT_MANIFEST.json'), 'topic_headers': per_topic,
              'bag_recorder_order_is_original_production_order': False,
              'original_estimator_initial_state_reconstructed': False, 'actual_V19_speedup_or_navigation_PASS': False,
              'capture_overhead': 'MCAP recording + compression coexisted with V18; not an uninstrumented performance baseline'}
    with args.out.open('x') as f: json.dump(result, f, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps({'status': result['status'], 'out': str(args.out)}))
    return 0 if result['status'] == 'passed_captured_input_prefix_only' else 1


if __name__ == '__main__': raise SystemExit(main())
