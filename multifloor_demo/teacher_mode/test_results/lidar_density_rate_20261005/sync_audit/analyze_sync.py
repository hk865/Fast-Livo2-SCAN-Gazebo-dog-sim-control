#!/usr/bin/env python3
"""Read archived V8 records; no ROS, simulations or estimator changes."""
from pathlib import Path
import collections
import csv
import hashlib
import json
import math
import re
import statistics
import struct

HERE = Path(__file__).resolve().parent
TEACHER = HERE.parents[2]
RUN_NAMES = {
    'rgb10': '20261005_192400_closed_loop_cascade_lidar64_30hz_rgb10_r1_f728',
    'rgb30': '20261005_192802_closed_loop_cascade_lidar64_30hz_rgb30_r1_cb5c',
}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def stats(a):
    return {'count': len(a), 'minimum': min(a), 'median': statistics.median(a),
            'mean': statistics.mean(a), 'maximum': max(a)} if a else {'count': 0}


def rows(path):
    return [json.loads(raw) for raw in Path(path).open()]


def diagnostic(run):
    data = collections.defaultdict(list)
    phases = collections.defaultdict(dict)
    byte_counts = collections.Counter()
    record_counts = collections.Counter()
    with (run / 'fastlivo_diagnostics/records.bin').open('rb') as binary:
        with (run / 'fastlivo_diagnostics/index.csv').open() as index:
            for row in csv.DictReader(index):
                kind = int(row['kind'])
                byte_counts[kind] += 56 + 8 * int(row['n_values'])
                record_counts[kind] += 1
                if kind not in (1, 2, 3, 10, 11, 12, 13):
                    continue
                sequence, stage = int(row['sequence']), int(row['stage'])
                stamp = int(row['stamp_ns']) / 1e9
                binary.seek(int(row['offset']) + 56)
                n = min(14, int(row['n_values']))
                values = struct.unpack('<' + 'd' * n, binary.read(8 * n))
                wall = values[1] if kind in (12, 13) else values[0]
                data[kind].append((stamp, wall, values, sequence, stage))
                if kind in (10, 11, 12, 13):
                    phases[(sequence, stamp, stage)][kind] = (wall, values)
    streams = {}
    for kind, a in data.items():
        streams[str(kind)] = {
            'records': len(a), 'first_last_source_s': [a[0][0], a[-1][0]],
            'first_last_receipt_steady_s': [a[0][1], a[-1][1]],
            'unique_source_headers': len(set(x[0] for x in a)),
            'source_interval_s': stats([b[0] - a[0] for a, b in zip(a, a[1:])]),
            'receipt_interval_s': stats([b[1] - a[1] for a, b in zip(a, a[1:])]),
        }
    windows = []
    image_receipt = {round(x[0] * 1e9): x[1] for x in data[3]}
    for begin, end in [(5, 20), (20, 40), (40, 53), (53, 100), (115, 165), (170, 190)]:
        result = {'source_window_s': [begin, end]}
        # Actual EKF enum: VIO=1; LIO=2. Kind10/11 surround Process2;
        # kinds13/12 finish the StateEstimation/processFrame, respectively.
        for label, stage, last_kind in [('lio', 2, 13), ('vio', 1, 12)]:
            a = [(x[10][0], x[last_kind][0], x[11][0])
                 for (seq, stamp, st), x in phases.items()
                 if st == stage and begin < stamp <= end
                 and 10 in x and 11 in x and last_kind in x]
            result[label + '_Process2_and_update_wall_ms'] = stats([(finish - start) * 1000 for start, finish, imu in a])
            result[label + '_Process2_wall_ms'] = stats([(imu - start) * 1000 for start, finish, imu in a])
        lio = [x for x in data[10] if x[4] == 2 and begin < x[0] <= end]
        cycle = [b[1] - a[1] for a, b in zip(lio, lio[1:])]
        result['complete_LIO_to_next_LIO_cycle_wall_ms'] = stats([x * 1000 for x in cycle])
        result['achieved_mapping_cycles_per_wall_second'] = 1 / statistics.mean(cycle) if cycle else None
        result['camera_callback_to_matching_LIO_start_wall_ms'] = stats([
            (x[1] - image_receipt[round(x[0] * 1e9)]) * 1000 for x in lio
            if round(x[0] * 1e9) in image_receipt])
        if len(lio) > 1:
            wall_begin, wall_end = lio[0][1], lio[-1][1]
            inputs = [x for x in data[3] if wall_begin <= x[1] <= wall_end]
            result['camera_input_during_same_processing_wall_window'] = {
                'wall_window_s': [wall_begin, wall_end], 'frames': len(inputs),
                'arrival_hz': (len(inputs) - 1) / (inputs[-1][1] - inputs[0][1]) if len(inputs) > 1 else None,
                'first_last_source_s': [inputs[0][0], inputs[-1][0]] if inputs else [],
                'source_seconds_per_wall_second': (inputs[-1][0] - inputs[0][0]) / (inputs[-1][1] - inputs[0][1]) if len(inputs) > 1 else None}
        windows.append(result)
    return data, {'streams': streams, 'core_elapsed_windows': windows,
                  'indexed_binary_bytes_by_kind': {str(k): {'bytes': n, 'records': record_counts[k]}
                                                   for k, n in byte_counts.items()}}


def source_audit():
    v7 = TEACHER / 'navigation/closed_loop_multifloor_v7'
    core = v7 / 'slam_ws/src/fast_livo2_core'
    names = [core / 'CMakeLists.txt', core / 'src/LIVMapper.cpp', core / 'src/voxel_map.cpp',
             core / 'src/vio.cpp', core / 'include/fast_livo2_core/core/diagnostics.h',
             v7 / 'slam_ws/build/fast_livo2_core/CMakeCache.txt',
             v7 / 'slam_ws/build/fast_livo2_core/compile_commands.json',
             v7 / 'shared_controller.py', v7 / 'teacher_wrapper.py',
             TEACHER / 'policy/worker.py', TEACHER / 'simulation/teacher_actuator.cpp',
             TEACHER / 'navigation/guard_audit.py']
    compile_commands = json.loads(names[6].read_text())
    selected_commands = {Path(x['file']).name: x['command'] for x in compile_commands
                         if Path(x['file']).name in ('LIVMapper.cpp', 'voxel_map.cpp', 'vio.cpp')}
    return {'frozen_source_files': {str(x.relative_to(TEACHER)): {'path': str(x),
              'sha256': sha(x), 'bytes': x.stat().st_size} for x in names},
        'actual_compiler_commands': selected_commands,
        'build_type': 'Release', 'MP_EN_defined_in_actual_commands': any('-DMP_EN' in x for x in selected_commands.values()),
        'MPI_or_OpenMP_link_alone_not_loop_enable': True,
        'lio_parallel_candidate': {
            'source': 'voxel_map.cpp:777-995',
            'scope': 'Only src/voxel_map.cpp COMPILE_DEFINITIONS=MP_EN;MP_PROC_NUM=2 in an isolated build',
            'independent_indices': ['pv_list[i]', 'diagnostic_query_rows_[i]', 'all_ptpl_list[i]'],
            'packed_bool_writes': 'useful_ptpl[i] is protected by one shared mylock, lines 874-886',
            'shared_map_and_plane': 'Read-only find and recursive plane reads; UpdateVoxelMap runs separately after StateEstimation on main thread',
            'collection': 'After OpenMP implicit barrier, residual push_back is serial in original input index order',
            'floating_point_reduction_in_LIO_loop': False,
            'numerical_equivalence_scope': 'Expected from independent operations/order, requires independent fixture/replay verification; not established by this read-only audit',
            'caveat': 'omp_set_num_threads(2) persists in current serial thread and can affect later OpenMP library calls; mutex contention and scheduling require measurement'},
        'vio_parallel_candidate': {
            'source': 'vio.cpp:1747-1890', 'actual_inverse_composition_en': False,
            'independent_indices': ['z point-patch rows', 'H_sub point-patch rows', 'visual_submap->errors[i]'],
            'shared_writes': 'OpenMP reduction of float error and int n_meas; Jacobian/projection outputs are loop-local',
            'risk': 'Float reduction order changes; error<=last_error acceptance/rollback can differ. Keep VIO serial in first candidate'},
        'diagnostic_cost': {
            'enqueue': 'std::move of vector, short bounded queue mutex, asynchronous writer; not whole-vector deepcopy at enqueue',
            'producer': 'Main estimator thread constructs and copies large diagnostic vectors before enqueue',
            'always_enabled_query_metadata': 'BuildResidualListOMP assign/fill 60 doubles per point even outside detail window',
            'hardcoded_raw_processed_cloud_window': {'source': 'LIVMapper.cpp:979', 'begin_s': 115, 'end_s': 165,
                'honors_detail_env_begin_end': False},
            'state_file_flush': 'mat_pre/mat_out use std::endl in LIO and VIO; synchronous flush remains'},
        'publication_cost': {
            'preserve': ['algorithm RGB/CameraInfo/IMU/LiDAR inputs', '/cloud_registered_full navigation cloud',
                         '/aft_mapped_to_init odometry', 'actual CameraInfo and required archived records'],
            'already_disabled': ['pub_effect_point_en', 'pub_plane_en', 'PCD save', 'Colmap output', 'image_save', 'evo pose output'],
            'candidates': ['optional /rgb_img output conversion/publish', 'optional /cloud_registered colored rendering projection/serialization',
                           'debug per-frame status text', '/path whole-history output publication', 'unused MAVROS pose output'],
            'warnings': ['pub_scan_num accumulates points before projection, so cannot assume it reduces total projected points',
                         'dense_map_en also changes navigation cloud points; do not use it as a display-only optimization',
                         'existing stage probes cannot isolate MapUpdate, colored cloud, image, ROS serialization or callback cost separately']}}


def sync_log(run):
    pattern = re.compile(r'\[(\d+\.\d+)\].*\[DEMO_SYNC\] camera=([\d.]+) lidar_newest=([\d.]+) imu_newest=([\d.]+) imu_last_used=([\d.]+) imu_count=(\d+) complete=(\d)')
    events = [tuple(float(x) for x in m.groups()) for m in pattern.finditer((run / 'navigation_stack.log').read_text())]
    result = {'records': len(events), 'complete_true': sum(int(x[6]) for x in events),
              'complete_false': sum(not int(x[6]) for x in events), 'windows': []}
    for begin, end in [(5, 20), (20, 40), (40, 53), (53, 100), (115, 165), (170, 190)]:
        a = [x for x in events if begin < x[1] <= end]
        if not a:
            continue
        result['windows'].append({'source_window_s': [begin, end], 'records': len(a),
            'complete_fraction': statistics.mean(x[6] for x in a),
            'newest_lidar_minus_LIO_target_s': stats([x[2] - x[1] for x in a]),
            'LIO_target_minus_newest_imu_s': stats([x[1] - x[3] for x in a]),
            'selected_imu_count': stats([x[5] for x in a])})
    result['selected_events'] = [{'source_requested_s': t, 'log_epoch_s': x[0],
        'camera_s': x[1], 'newest_lidar_s': x[2], 'newest_imu_s': x[3],
        'last_used_imu_s': x[4], 'imu_count': int(x[5]), 'complete': bool(x[6])}
        for t in (5, 20, 40, 50, 52.5, 55, 100, 150, 190)
        for x in [min(events, key=lambda x: abs(x[1] - t))]]
    return result


def main():
    evidence = {'schema': 'go2_lidar_rate_readonly_sync_audit/v1',
                'method': 'Archived data only; no ROS, process signal, simulation or source mutation',
                'analysis_script_sha256': sha(__file__), 'source_audit': source_audit(), 'runs': {}}
    for label, name in RUN_NAMES.items():
        run = TEACHER / 'runs' / name
        data, summary = diagnostic(run)
        summary.update(run_dir=str(run), sync=sync_log(run))
        names = ['navigation_stack.log', 'navigation_status.jsonl', 'navigation_pid_history.jsonl',
                 'navigation_guard_history.jsonl', 'navigation_slam_poses.jsonl',
                 'navigation_cloud_callbacks.jsonl', 'navigation_sensor_gate_history.jsonl',
                 'navigation_imu_history.jsonl', 'runtime_manifest.json', 'owned_resources.jsonl',
                 'actuator.jsonl', 'telemetry.jsonl', 'fastlivo_diagnostics/index.csv',
                 'fastlivo_diagnostics/writer_stats.json', 'sensor_sampling_contract.json',
                 'navigation_scene_axis_registration.json']
        summary['evidence_files'] = {n: {'path': str(run / n), 'bytes': (run / n).stat().st_size,
                                      'sha256': sha(run / n)} for n in names}
        summary['large_binary'] = {'path': str(run / 'fastlivo_diagnostics/records.bin'),
            'bytes': (run / 'fastlivo_diagnostics/records.bin').stat().st_size,
            'sha256_recomputed': False, 'reason': 'Avoid redundant parallel full-file read; bind the complete index and writer statistics'}
        status = rows(run / 'navigation_status.jsonl')
        failed = next((x for x in status if x['state'] == 'failed'), None)
        summary['first_failure'] = {k: failed.get(k) for k in ('ros_sim_time', 'monotonic_wall', 'message',
            'waypoint_index', 'pose_age', 'cloud_age', 'raw_imu_age', 'raw_imu_stamp_age')} if failed else None
        poses = rows(run / 'navigation_slam_poses.jsonl')
        clouds = rows(run / 'navigation_cloud_callbacks.jsonl')
        accepted = [x for x in clouds if x.get('accepted')]
        summary['last_accepted_controller_pose'] = poses[-1]
        summary['last_accepted_controller_cloud'] = accepted[-1] if accepted else None
        summary['last_controller_cloud_callback'] = clouds[-1]
        gates = rows(run / 'navigation_sensor_gate_history.jsonl')
        if failed:
            summary['raw_gate_near_first_failure'] = min(gates, key=lambda x: abs(x['ros_sim_time_ns'] / 1e9 - failed['ros_sim_time']))
        summary['raw_gate_near_200s_while_active'] = min(gates, key=lambda x: abs(x['ros_sim_time_ns'] / 1e9 - 200.0))
        raw_recent = [x for x in gates if 209 <= x.get('ros_sim_time_ns', 0) / 1e9 <= 210.02
                      and x['ages']['imu']['wall_s'] is not None and x['ages']['imu']['wall_s'] < .3]
        summary['raw_gate_near_simulation_end'] = raw_recent[-1]
        resources = rows(run / 'owned_resources.jsonl')
        summary['resource_peak'] = {
            'logical_cpus': resources[0]['logical_cpus'],
            'maximum_load_1': max(x['load_1_5_15'][0] for x in resources),
            'maximum_GPU_utilization_percent': max(x['gpu']['utilization_percent'] for x in resources if x.get('gpu')),
            'maximum_GPU_used_MiB': max(x['gpu']['used_MiB'] for x in resources if x.get('gpu')),
            'minimum_available_RAM_KiB': min(x['memory_KiB']['MemAvailable'] for x in resources)}
        manifest = json.loads((run / 'runtime_manifest.json').read_text())
        summary['all_processes_clean'] = manifest['all_owned_and_children_clean']
        summary['training_processes_signaled'] = manifest['training_processes_signaled']
        summary['frozen_actual_SLAM_binaries'] = manifest['slam_binary_contract']
        if label == 'rgb10':
            last_guard = rows(run / 'navigation_guard_history.jsonl')[-1]
            ticks = rows(run / 'navigation_pid_history.jsonl')[-3:]
            fields = ['feedback', 'imu', 'ack', 'path_receipt', 'control_pose', 'control_quaternion',
                      'checked_target', 'steering_direction', 'prepared_after_slew_command']
            summary['clock_repeat_evidence'] = {
                'last_guard': last_guard,
                'three_ticks': [{k: x.get(k) for k in ('sequence', 'source_pose_stamp_ns',
                    'compute_ros_clock_ns', 'compute_monotonic_wall', 'state', 'command_after_slew')}
                    | {'controller_updated': x['cascade'].get('controller_updated'),
                       'reason': x['cascade'].get('reason')} for x in ticks],
                'same_canonical_payload_fields': {field: [hashlib.sha256(json.dumps(x[field],
                    sort_keys=True, separators=(',', ':')).encode()).hexdigest() for x in ticks]
                    for field in fields},
                'new_cloud_at_unchanged_compute_clock': next(x for x in clouds
                    if x['producer_stamp_ns'] == 190199999999),
            }
            counts = collections.Counter()
            non_increasing = collections.Counter()
            previous = {}
            exchange = []
            for raw in (run / 'actuator.jsonl').open():
                x = json.loads(raw)
                kind = x['kind']; counts[kind] += 1
                if 't' in x:
                    if kind in previous and x['t'] <= previous[kind]:
                        non_increasing[kind] += 1
                    previous[kind] = x['t']
                if kind == 'policy_exchange' and 190.24 <= x['t'] <= 190.30:
                    exchange.append(x)
            summary['clock_repeat_evidence'].update(native_kind_counts=dict(counts),
                native_non_increasing_by_kind=dict(non_increasing), nearby_policy_exchanges=exchange)
            telemetry = next(json.loads(raw) for raw in (run / 'telemetry.jsonl').open()
                             if 190.259 <= json.loads(raw)['sim_time'] <= 190.261)
            summary['clock_repeat_evidence']['matching_actor_inference_ms'] = telemetry['inference_ms']
        evidence['runs'][label] = summary
    (HERE / 'evidence.json').write_text(json.dumps(evidence, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'output': str(HERE / 'evidence.json'),
                      'summary': {k: v['first_failure'] for k, v in evidence['runs'].items()}}, ensure_ascii=False))


if __name__ == '__main__':
    main()
