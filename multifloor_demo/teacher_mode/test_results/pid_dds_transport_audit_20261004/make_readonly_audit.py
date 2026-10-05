#!/usr/bin/env python3
"""Historical receipt/source audit only; no ROS, network, signals or control writes."""
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
MODE = HERE.parent.parent
ROOT = MODE.parent.parent


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def rows(path, predicate=None):
    """Hash every original byte; optionally parse only the relevant receipts."""
    digest = hashlib.sha256()
    selected = []
    with Path(path).open('rb') as stream:
        for line in stream:
            digest.update(line)
            if line.strip() and (predicate is None or predicate(line)):
                selected.append(json.loads(line))
    return selected, digest.hexdigest()


def audit_run(name):
    run = MODE / 'runs' / name
    refs = {}
    cloud, refs[str(run / 'navigation_cloud_history.jsonl')] = rows(run / 'navigation_cloud_history.jsonl')
    pose, refs[str(run / 'navigation_slam_poses.jsonl')] = rows(run / 'navigation_slam_poses.jsonl')
    gates, refs[str(run / 'navigation_sensor_gate_history.jsonl')] = rows(run / 'navigation_sensor_gate_history.jsonl')
    shadow, refs[str(run / 'sensor_shadow/sensor_samples.jsonl')] = rows(
        run / 'sensor_shadow/sensor_samples.jsonl', lambda line: b'"source": "cloud"' in line)
    sets = {
        'controller': {int(row['stamp_ns']) for row in cloud},
        'gate': {int(row['ages']['cloud']['stamp_ns']) for row in gates
                 if row.get('ages', {}).get('cloud', {}).get('stamp_ns') is not None},
        'shadow': {int(row['stamp_ns']) for row in shadow},
    }
    union = set.union(*sets.values())
    common = set.intersection(*sets.values())
    poses = {int(row['stamp_ns']) for row in pose}
    metadata_path = run / 'map_metadata.json'
    metadata = json.loads(metadata_path.read_text())
    refs[str(metadata_path)] = sha(metadata_path)
    shadow_path = run / 'sensor_shadow/summary.json'
    shadow_summary = json.loads(shadow_path.read_text())
    refs[str(shadow_path)] = sha(shadow_path)
    log_path = run / 'navigation_stack.log'
    with log_path.open('rb') as stream:
        slices = sum(b'[DEMO_LIDAR_SLICE]' in line for line in stream)
    refs[str(log_path)] = sha(log_path)
    payload_points = [row['input_points'] for row in cloud]
    info = {
        'run': str(run),
        'sync_slice_log_records_not_publish_receipts': slices,
        'actual_slam_pose_rows': len(pose),
        'actual_slam_pose_unique_stamps': len(poses),
        'controller_cloud_rows': len(cloud),
        'cloud_unique_stamps': {key: len(value) for key, value in sets.items()},
        'cloud_union_unique_stamps': len(union),
        'cloud_common_unique_stamps': len(common),
        'actual_pose_stamps_absent_from_all_three_cloud_receipts': len(poses - union),
        'map_archive_same_process_received_counts': metadata['counts'],
        'shadow_received_counts': {key: value['received'] for key, value in shadow_summary['sources'].items()},
        'shadow_cloud_received_period': shadow_summary['sources']['cloud']['received_period_s'],
        'controller_cloud_input_points_min_max': [min(payload_points), max(payload_points)],
        'inferred_cloud_data_bytes_min_max_from_48_byte_point_stride': [min(payload_points) * 48, max(payload_points) * 48],
        'cloud_payload_inference_scope': 'Existing receipts did not record point_step/data_bytes. Installed PCL conversion and exact point type imply 48 bytes per point, excluding DDS/CDR/header overhead; future probe records actual message fields.',
        'publisher_actual_publish_count_available': False,
        'historical_process_RMW_environment_and_maps_available': False,
        'original_input_sha256': refs,
    }
    callback_path = run / 'navigation_cloud_callbacks.jsonl'
    if callback_path.exists():
        callbacks, refs[str(callback_path)] = rows(callback_path)
        info['cloud_callback_rows'] = len(callbacks)
        info['cloud_callback_keys'] = sorted(callbacks[0]) if callbacks else []
        diagnostic = run / 'pid_cloud_callback_diagnostic.json'
        if diagnostic.exists():
            info['recorded_cloud_callback_diagnostic'] = json.loads(diagnostic.read_text())
            refs[str(diagnostic)] = sha(diagnostic)
    return info


def main():
    freeze_path = MODE / 'test_results/pid_navigation_runtime_v4_transport_freeze_20261004/v4.json'
    freeze = json.loads(freeze_path.read_text())
    mismatches, noncanonical = [], []
    for name, expected in freeze['source_hashes'].items():
        actual = sha(name)
        if actual != expected:
            mismatches.append({'path': name, 'expected': expected, 'actual': actual})
        if str(Path(name).resolve()) != name:
            noncanonical.append(name)
    source_paths = [
        ROOT / 'multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/LIVMapper.cpp',
        ROOT / 'multifloor_demo/slam/ros2_ws/src/fast_livo2_core/include/fast_livo2_core/utils/types.h',
        Path('/opt/ros/jazzy/include/pcl_conversions/pcl_conversions/pcl_conversions.h'),
        Path('/usr/include/pcl-1.14/pcl/conversions.h'),
        Path('/usr/include/pcl-1.14/pcl/pcl_config.h'),
    ]
    report = {
        'schema': 'readonly_DDS_historical_source_audit/v1',
        'as_of_UTC': datetime.now(timezone.utc).isoformat(),
        'audit_script': {'path': str(Path(__file__).resolve()), 'sha256': sha(__file__)},
        'scope': 'Read existing metadata/logs only. No NPZ evaluation, ROS, participant creation, signal, training, system or runtime mutation.',
        'runs': [audit_run(name) for name in [
            '20261004_124855_navigation_teacher_pid_v2_safety_r1_b579',
            '20261004_132148_navigation_teacher_pid_v3_slew_r1_cddc']],
        'frozen_runtime_recheck': {
            'path': str(freeze_path), 'sha256': sha(freeze_path),
            'source_count': len(freeze['source_hashes']),
            'source_mismatches': mismatches, 'noncanonical_source_keys': noncanonical,
            'actual_transport_or_navigation_pass_claimed': False,
        },
        'source_as_of_sha256': {str(path): sha(path) for path in source_paths},
        'official_tagged_evidence_sha256': {str(path): sha(path) for path in (HERE / 'sources').iterdir() if path.is_file()},
        'current_auditor_environment_only': {'path': str(HERE / 'asof_environment.json'), 'sha256': sha(HERE / 'asof_environment.json')},
        'direct_findings': [
            'Full-cloud receipts are sparse in controller/gate/shadow and separately in map archive; missing stamps extend upstream of the controller callback.',
            'LIVMapper publishes odometry and full cloud from handleLIO, and full cloud is outside colored-cloud decimation. Sync slice log is not a direct publish receipt.',
            'Tagged rmw_fastrtps 8.4.4 participant creation loads XML before LOCALHOST appends default SHM and UDP descriptors. SYSTEM_DEFAULT leaves XML transports unchanged.',
            'FastDDS 2.14.6 SHM default is 512KiB; send allocates shared buffers and overflow discard can return success with only INFO logging.',
            'Offline installed-parser receipts show the two new presets load via FASTRTPS_DEFAULT_PROFILES_FILE with SKIP_DEFAULT_XML=1; no participant was created.',
        ],
        'inferences_not_proven_causes': [
            'Large PointXYZINormal full-cloud fragmentation and per-participant SHM capacity/load are plausible contributors; global /dev/shm free space does not establish per-participant capacity.',
            'Publisher transport, serialization/blocking time, fragment drops, receive executor/queue load and topic-specific scheduling are not isolated by historical receipts.',
            'No old process environment/maps or direct publish receipt proves historical actual RMW transport or unique DDS causality.',
        ],
        'prospective_controlled_A_B': {
            'profiles': ['shm_512k', 'shm_64m'],
            'sole_preset_difference': 'SHM segment_size=524288 versus 67108864 bytes',
            'historical_localhost_baseline_is_not_single_factor_comparison': True,
            'ROS_endpoint_QoS_math_header_TTL_physics_unchanged': True,
            'future_probe_measures': ['actual cloud point_step/data_bytes/fields', 'original integer header stamp', 'MessageInfo fields when available', 'paired reliable/best_effort receipts', 'actual observer endpoint QoS/RMW/maps'],
            'probe_limitations': ['Reliable diagnostic reader can affect publisher/retransmission/backpressure.', 'Observer RMW/maps prove observer process only.', 'Its existing environment metadata omits SKIP_DEFAULT_XML/ROS_STATIC_PEERS; run manifest/effective-environment receipt supplements those keys.'],
        },
        'source_writers_stopped': True,
    }
    output = HERE / 'dds_historical_source_audit.json'
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps({'report': str(output), 'report_sha256': sha(output), 'runtime_hash_mismatches': len(mismatches),
                      'counts': [{key: run[key] for key in ['run', 'sync_slice_log_records_not_publish_receipts', 'actual_slam_pose_rows', 'controller_cloud_rows', 'cloud_unique_stamps', 'cloud_union_unique_stamps', 'actual_pose_stamps_absent_from_all_three_cloud_receipts']} for run in report['runs']]}))


if __name__ == '__main__':
    main()
