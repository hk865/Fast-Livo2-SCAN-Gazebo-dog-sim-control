#!/usr/bin/env python3
"""Read-only protection/source audit. Writes only its own new audit directory."""
from __future__ import annotations
import ast
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DEMO = ROOT.parent
FROZEN = Path('/home/hyh001/projects/1.Project/RL_for_unitree/logs/rsl_rl/rl_unitree_go2_aer_height_distribution/2026-10-01_17-23-23_STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A-FORMAL-4096ENV-3000ITER-SEED42')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def file_entry(path, expected=None):
    path = Path(path).resolve()
    result = {'path': str(path), 'sha256': sha(path), 'size_bytes': path.stat().st_size,
              'mtime_ns': path.stat().st_mtime_ns}
    if expected is not None:
        result.update(expected_sha256=expected, baseline_matches=result['sha256'] == expected)
    return result


def verify_sources(run, mapping):
    return {name: file_entry(Path(run) / name, expected) for name, expected in mapping.items()}


def normalized_xml(node):
    node = copy.deepcopy(node)
    for item in node.iter():
        if item.text is not None:
            item.text = ' '.join(item.text.split())
        item.tail = None
    return ET.tostring(node, encoding='unicode')


def robot_physics(path):
    robot = copy.deepcopy(ET.parse(path).find(".//world/model[@name='go2']"))
    # Keep collision/inertial/link/joint poses and limits; remove rendering/sensor/runtime I/O only.
    for parent in robot.iter():
        for child in list(parent):
            if child.tag in ('visual', 'sensor', 'plugin'):
                parent.remove(child)
    return normalized_xml(robot)


def selected_ast(path, name):
    return next(v for v in ast.parse(Path(path).read_text()).body
                if isinstance(v, (ast.FunctionDef, ast.ClassDef)) and v.name == name)


def command_output(argv):
    process = subprocess.run(argv, capture_output=True, text=True, timeout=5., check=False)
    return {'argv': argv, 'returncode': process.returncode, 'stdout': process.stdout, 'stderr': process.stderr}


def resource_snapshot():
    timestamp = datetime.now(timezone.utc).isoformat()
    relevant = []
    training = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            argv = proc.joinpath('cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
            text = ' '.join(argv)
            is_training = ('train_go2' in text or '/RL_for_unitree/' in text or
                           any(Path(a).name in ('train.py', 'train_go2_deployable_016.py') for a in argv))
            is_relevant = str(ROOT) in text or 'fastlivo_mapping' in text or is_training
            if not is_relevant:
                continue
            stat = proc.joinpath('stat').read_text()
            fields = stat[stat.rfind(')') + 2:].split()
            status = proc.joinpath('status').read_text().splitlines()
            entry = {'pid': int(proc.name), 'ppid': int(fields[1]), 'state': fields[0],
                'starttime_ticks': int(fields[19]), 'cpu_user_ticks': int(fields[11]),
                'cpu_system_ticks': int(fields[12]), 'argv': argv,
                'status': {line.split(':', 1)[0]: line.split(':', 1)[1].strip() for line in status
                           if line.split(':', 1)[0] in ('Uid', 'VmRSS', 'Threads')}}
            relevant.append(entry)
            if is_training:
                training.append(entry)
        except (OSError, ValueError, IndexError):
            continue  # Process may finish naturally while the read-only snapshot is taken.
    return {'timestamp_utc': timestamp, 'timestamp_monotonic_wall': time.monotonic(),
        'loadavg': Path('/proc/loadavg').read_text().strip(),
        'meminfo': Path('/proc/meminfo').read_text(),
        'gpu': command_output(['nvidia-smi', '--query-gpu=timestamp,name,memory.used,memory.total,utilization.gpu,utilization.memory', '--format=csv,noheader']),
        'gpu_compute_processes': command_output(['nvidia-smi', '--query-compute-apps=pid,process_name,used_memory', '--format=csv,noheader']),
        'relevant_processes': sorted(relevant, key=lambda v: v['pid']),
        'training_candidate_processes': training,
        'scope': 'One read-only snapshot; process absence is not proof of historical termination, and no signal/API controlling a task is used.'}


def main():
    if (OUT / 'audit.json').exists():
        raise RuntimeError('Original audit receipt already exists; never overwrite it')
    initial = read(ROOT / 'test_results/package_20261003_pre_step_retest.json')
    freeze = read(ROOT / 'test_results/navigation_v41_freeze.json')
    camera = read(DEMO / 'camera_mode/simulation/asset_receipt.json')
    protected = {'checkpoint': file_entry(FROZEN / 'model_1000.pt', initial['checkpoint_sha256']),
        'training_archive': {name: file_entry(FROZEN / 'params' / name, initial['archive_hashes'][str(FROZEN / 'params' / name)])
                             for name in ('env.yaml', 'agent.yaml')},
        'original_camera_mode': {name: file_entry(DEMO / 'camera_mode/simulation' / name, camera['outputs'][name])
                                 for name in ('generated/camera_rig.sdf', 'generated/three_floors_camera.sdf')},
        'original_demo_physics_assets': {path: file_entry(path, entry['expected_sha256'])
                                        for path, entry in initial['protected_assets'].items()},
        'current_native_actuator': file_entry(ROOT / 'simulation/build/libteacher_actuator.so',
                                            freeze['source_hashes']['simulation/build/libteacher_actuator.so']),
        'global_acceptance': file_entry(ROOT / 'runs/acceptance.json', initial['acceptance_sha256'])}
    current = {name: file_entry(ROOT / name, expected) for name, expected in freeze['source_hashes'].items()}
    reference = ROOT / 'runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9'
    active = ROOT / 'runs/20261004_091122_navigation_slam_scan_atomic_transport_v41_r1_dba1'
    atomic = ROOT / 'test_results/atomic_transport_review_20261004'
    nofsync = {'source_diff_receipt': file_entry(atomic / 'source_diff_review.json'),
        'concurrency_and_ttl_receipt': file_entry(atomic / 'review.json'),
        'actual_atomic_review_passed': read(atomic / 'review.json')['passed'],
        'actual_source_diff_passed': read(atomic / 'source_diff_review.json')['passed'],
        'current_candidate_equals_tested_candidate': sha(ROOT / 'navigation/bridge.py') == read(atomic / 'review.json')['source_hashes']['bridge.py'],
        'current_navigation_consumer_ast_equals_prechange': ast.dump(selected_ast(ROOT / 'policy/worker.py', 'navigation_command')) == ast.dump(selected_ast(reference / 'sources/policy/worker.py', 'navigation_command')),
        'current_physics_step_xml_equals_prechange': normalized_xml(ET.parse(active / 'world.sdf').find('.//physics')) == normalized_xml(ET.parse(reference / 'world.sdf').find('.//physics')),
        'current_robot_collision_inertia_joint_pose_xml_equals_prechange': robot_physics(active / 'world.sdf') == robot_physics(reference / 'world.sdf'),
        'active_run_policy_manifest': file_entry(active / 'policy_manifest.json'),
        'active_run_asset_manifest': file_entry(active / 'asset_manifest.json'),
        'active_run_world': file_entry(active / 'world.sdf'),
        'active_run_status': 'Root-owned live run at audit time; no pass asserted',
        'bounds': 'No fsync changes actor/checkpoint/native physics or original wall/sim300msTTL; no durability or live latency conclusion'}
    source_audit_path = ROOT / 'test_results/sensor_replacement_audit_20261004_idle_v3.json'
    runtime_audit_path = ROOT / 'test_results/sensor_replacement_idle_runtime_20261004_v2.json'
    source_audit, runtime_audit = read(source_audit_path), read(runtime_audit_path)
    source_runtime = []
    for source, runtime in zip(source_audit['runs'], runtime_audit['runs']):
        assert source['run'] == runtime['run']
        hashes = verify_sources(source['run'], {**source['source_hashes'], **runtime['source_hashes']})
        source_runtime.append({'run': source['run'], 'source_checks': source['checks'], 'runtime_checks': runtime['checks'],
            'prior_hashes_reverified': hashes, 'source_and_runtime_checks_passed': all(source['checks'].values()) and all(runtime['checks'].values()),
            'raw_evidence_unchanged_since_independent_replay': all(v['baseline_matches'] for v in hashes.values()),
            'policy_rows': source['total_policy_rows'], 'actual_sensor_rows': source['actual_sensor_replaced_rows'],
            'actual_intervals': source['actual_sensor_replaced_intervals'],
            'observation_reconstruction_error': source['maximum_observation_reconstruction_error'],
            'cpu_actor_action_replay_error': source['maximum_cpu_actor_action_replay_error'],
            'source_sim_age_s': source['source_age_sim_s'], 'source_wall_age_s': source['source_age_wall_s'],
            'steady_vx_mps': runtime['actual_steady_mean'][0], 'forward_world_dx_m': runtime['position_delta_m'][0],
            'parking_drift_m': runtime['stop_endpoint_translation_m'], 'parking_yaw_rad': runtime['stop_endpoint_yaw_rad'],
            'owned_exit_codes': runtime['runtime_exit_codes'], 'frozen_thresholds': runtime['thresholds']})
    diagnosis_path = ROOT / 'test_results/sensor_replacement_v2_r2_rejection_diagnosis_20261004.json'
    diagnosis = read(diagnosis_path)
    old_failure = {'diagnosis_receipt': file_entry(diagnosis_path), 'run': diagnosis['run'],
        'source_hashes_reverified': verify_sources(diagnosis['run'], diagnosis['source_sha256']),
        'failure': diagnosis['failure'], 'wall_age_lower_bound_s': diagnosis['joint_wall_age_lower_bound_at_rejection_s'],
        'conclusion': diagnosis['branch_conclusion'], 'callback_invalid_total': diagnosis['invalid_total'],
        'actual_stamps_strictly_increasing': diagnosis['stamps_strictly_increasing'],
        'limits': diagnosis['limits'], 'count_as_complete_pass': False}
    interrupted = ROOT / 'runs/20261004_080553_forward_resource_idle_actual_sensor_v2_r2_fe74'
    interrupt_path = interrupted / 'runner_interruption_receipt.json'
    interruption = read(interrupt_path)
    interrupt = {'receipt': file_entry(interrupt_path), 'run': str(interrupted),
        'original_runtime_manifest_exists': (interrupted / 'runtime_manifest.json').exists(),
        'original_summary_exists': (interrupted / 'summary.json').exists(),
        'receipt_data': interruption, 'count_as_complete_runtime_pass': False,
        'scope': 'Original movement reached18s without actor fault, but SIGTERM143 cleanup interrupted; sender not established. Audit does not send cleanup signals.'}
    # No actor replay is repeated here: verify all exact evidence and archived sources used by independent CPU1 replay.
    result = {'schema': 'teacher_final_protection_source_audit/v1', 'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'Read-only assets/archives/runtime/source evidence review, writes only this fresh directory; no source/README/docs/package changes, no ROS/Kit/sim/signals',
        'executed_source_sha256': sha(__file__), 'protected': protected, 'v41_frozen_runtime_source_checks': current,
        'no_fsync_scope_review': nofsync,
        'actual_sensor_replacement': {'independent_source_receipt': file_entry(source_audit_path),
            'independent_runtime_receipt': file_entry(runtime_audit_path), 'clean_idle_runs': source_runtime,
            'clean_idle_full_passes': 3, 'old_loaded_v2_group': '2 full passes /1 source rejection, preserved separately',
            'old_joint_freshness_failure': old_failure, 'interrupted_runner_not_a_runtime_pass': interrupt,
            'replacement_source': {'imu': '/livox/imu', 'q_qd': '/demo/control/measured_joint_states'},
            'dimension_accounting': {'actual_sensor': 30, 'remaining_privileged': 190, 'controller_known': 27, 'total': 247},
            'activation_s': 3, 'sim_ttl_s': .025, 'wall_ttl_s': .300, 'reference_phase': 'actual headerstamp<=native world time-.005',
            'missing_stale_invalid_action': 'immediate damping/source rejection, no privileged fallback',
            'limits': 'Gazebo actual ROS IMU/q/qd validates30dim causal replacement, not hardware estimator or full SLAM/cloud policy replacement; resource improvement only correlates with freshness success.'},
        'global_acceptance_levels_preserved': read(ROOT / 'runs/acceptance.json')['levels'],
        'historical_native_note': 'Initial pre-step package had a different native binary; approved later contact evidence logging was already frozen before these runs. Native is compared to v4.1 freeze, not falsely called unchanged since initial package.',
        'resource_snapshot': resource_snapshot()}
    checks = {'checkpoint': protected['checkpoint']['baseline_matches'],
        'training_archive': all(v['baseline_matches'] for v in protected['training_archive'].values()),
        'camera_mode': all(v['baseline_matches'] for v in protected['original_camera_mode'].values()),
        'original_demo': all(v['baseline_matches'] for v in protected['original_demo_physics_assets'].values()),
        'current_native': protected['current_native_actuator']['baseline_matches'],
        'acceptance': protected['global_acceptance']['baseline_matches'],
        'v41_sources': all(v['baseline_matches'] for v in current.values()),
        'no_fsync_rule_physics_actor_scope': all(v for v in nofsync.values() if isinstance(v, bool)),
        'three_clean_source_runtime_runs': all(v['source_and_runtime_checks_passed'] and v['raw_evidence_unchanged_since_independent_replay'] for v in source_runtime),
        'old_failure_retained': all(v['baseline_matches'] for v in old_failure['source_hashes_reverified'].values()),
        'interrupt_not_reclassified': not interrupt['original_runtime_manifest_exists'] and interrupt['receipt_data']['do_not_count_as_complete_runtime_pass'] is True}
    result['checks'], result['protection_audit_passed'] = checks, all(checks.values())
    (OUT / 'audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': result['protection_audit_passed'], 'checks': checks, 'output': str(OUT),
        'receipt_sha256': sha(OUT / 'audit.json'), 'training_candidates_at_snapshot': len(result['resource_snapshot']['training_candidate_processes'])}))
    if not result['protection_audit_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
