#!/usr/bin/env python3
"""Own CPU Teacher executor; high-level velocity comes only from SLAM binding.

The unmodified archived policy worker keeps its privileged actor observations,
native torque actuator, slew, bootstrap and damping. Its old scripted source
labels are replaced in this isolated module instance and actual evidence only.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.dont_write_bytecode = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=Path, required=True)
    ap.add_argument('--socket', required=True)
    args = ap.parse_args()
    run = args.run.resolve()
    sys.path.insert(0, str(run/'sources/policy'))
    sys.path.insert(0, str(run/'sources/slam_binding'))
    from adapter import read_command, canonical, sha
    execution = json.loads((run/'slam_execution.json').read_text())
    if (execution.get('schema') != 'teacher_slam_fixed_route_execution/v1' or execution.get('allowed') is not True
        or execution.get('real_robot') is not False or execution.get('navigation_ground_truth_used') is not False
        or execution.get('exclusive_writer') != 'teacher_sim::TeacherActuator'
        or execution.get('model_sha256') != 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'):
        raise RuntimeError('Unknown or invalid independent executor scope')
    binding = json.loads((run/'slam_binding.json').read_text())
    source_manifest = json.loads((run/'source_manifest.json').read_text())
    fingerprints = {}
    for name, digest in source_manifest.items():
        source = Path(name)
        if not source.is_absolute():
            source = run/'sources'/source
        if sha(source) != digest:
            raise RuntimeError('Frozen executor source mismatch: '+str(source))
        st = source.stat(); fingerprints[str(source)] = (st.st_size, st.st_mtime_ns)
    if sha(run/'slam_binding.json') != execution['binding_sha256']:
        raise RuntimeError('Binding differs from the execution scope')
    spec = importlib.util.spec_from_file_location('frozen_cpu_teacher_slam_executor', run/'sources/policy/worker.py')
    legacy = importlib.util.module_from_spec(spec); spec.loader.exec_module(legacy)
    old_recv, old_atomic = legacy.recv_exact, legacy.atomic_json
    latest = {'state_clock_ns': None, 'metadata': None, 'evidence': None, 'completion_worker_elapsed_s': None}
    sequence = {}
    limits = execution['command_limits']
    log = (run/'slam_execution_commands.jsonl').open('w', buffering=1)
    immutable = [(Path(name), digest) for name, digest in execution['immutable_generated_files'].items()]
    for p in (run/'slam_execution.json', run/'source_manifest.json'):
        st = p.stat(); fingerprints[str(p)] = (st.st_size, st.st_mtime_ns)
    for p, digest in immutable:
        if sha(p) != digest:
            raise RuntimeError('Generated execution input mismatch: '+str(p))
        st = p.stat(); fingerprints[str(p)] = (st.st_size, st.st_mtime_ns)

    def receive(conn, n):
        data = old_recv(conn, n)
        if n == 512:
            # Only simulator clock is used here. Position/yaw never enters the
            # outer controller, its route anchor, commands or arrival state.
            latest['state_clock_ns'] = round(float(np.frombuffer(data, dtype='<f8')[0])*1e9)
        return data

    def request(test, t):
        now = time.monotonic()
        changed = []
        for name, old in fingerprints.items():
            try:
                st = Path(name).stat()
                if (st.st_size, st.st_mtime_ns) != old:
                    changed.append(name)
            except OSError:
                changed.append(name)
        if changed:
            cmd, rejected, reason = np.zeros(3), True, 'frozen_source_changed:'+','.join(changed)
            sequence['read_attempt'] = {'status': 'not_read_due_to_changed_source',
                'read_monotonic_wall': now, 'read_clock_ns': latest['state_clock_ns'],
                'decoded_envelope': None, 'reason': reason}
        else:
            cmd, rejected, reason = read_command(run/'slam_velocity_command.json', latest['state_clock_ns'],
                execution['binding_sha256'], limits, sequence)
        last_accepted_envelope = sequence.get('envelope')
        attempt = sequence.get('read_attempt')
        envelope = None if attempt is None else attempt.get('decoded_envelope')
        if not rejected and envelope and envelope.get('controller_completed_pose_stamp_ns') is not None:
            if latest['completion_worker_elapsed_s'] is None:
                latest['completion_worker_elapsed_s'] = t
            end = latest['completion_worker_elapsed_s']+execution['post_completion_s']
            if latest['metadata'] is not None:
                latest['metadata']['test_duration_s'] = min(execution['duration_s'], end)
        evidence = {'sim_time': float(t), 'physics_clock_ns': latest['state_clock_ns'],
            'read_monotonic_wall': attempt.get('read_monotonic_wall',now) if attempt else now, 'source': 'slam_fixed_route',
            'command': cmd.tolist(), 'rejected': rejected, 'reason': reason,
            'actual_original_envelope': envelope, 'command_config_sha256': execution['binding_sha256'],
            'last_accepted_envelope': last_accepted_envelope,
            'actual_read_attempt': attempt,
            'navigation_ground_truth_used': False, 'changed_source_fingerprints': changed}
        latest['evidence'] = evidence
        log.write(canonical(evidence)+'\n')
        return cmd, 'initializing' if t < .1 else 'tracking' if np.any(cmd) else 'stop_transition'

    class IsolatedJSON:
        def __getattr__(self, name):
            return getattr(json, name)
        def dumps(self, obj, **kwargs):
            if isinstance(obj, dict) and 'world_sim_time' in obj and 'command_expired' in obj:
                obj = dict(obj)
                e = latest['evidence'] or {}
                obj.update(command_expired=e.get('rejected', True), command_reason=e.get('reason', 'not read'),
                    command_source='actual SLAM/IMU fixed-route command file',
                    slam_command_read_evidence=e, navigation_envelope=None,
                    outer_navigation_ground_truth_used=False,
                    privileged_actor_observations=True)
                envelope = e.get('actual_original_envelope')
                stamp = envelope.get('clock_ns') if isinstance(envelope, dict) else None
                obj['command_age_sim_s'] = (latest['state_clock_ns']-stamp)/1e9 if isinstance(stamp, int) and not isinstance(stamp, bool) else None
            return json.dumps(obj, **kwargs)

    def atomic(path, obj):
        if path.name == 'policy_manifest.json':
            obj.update(test='SLAM_fixed_route_frozen_controller',
                command_source='actual SLAM/IMU fixed-route command file',
                navigation_truth_used=False, counts_as_SLAM_navigation=False, high_level_source_is_actual_SLAM=True,
                test_duration_s=execution['duration_s'], command_end_s=None,
                command_start_seconds=None, command_start_semantics='First actual accepted nonzero SLAM/IMU command; see original envelope source time',
                legacy_motion_evaluation_applicable=False, acceptance_is_global=False,
                external_command_limits=limits, source_binding_sha256=execution['binding_sha256'],
                high_level_feedback_source='original /demo/slam/body_odom plus causal actual /livox/imu gyro',
                actor_observations_remain_privileged=True,
                source_timeout_s=.3, route_provider='fixed actual SLAM anchor; no SCAN',
                selected_profile_sha256=binding['controller_profile']['sha256'])
            latest['metadata'] = obj
        if path.name == 'worker_result.json':
            obj.update(controller_completed_observed_worker_elapsed_s=latest['completion_worker_elapsed_s'],
                navigation_ground_truth_used=False, actor_observations_remain_privileged=True,
                independent_route_validation='unverified', last_slam_command_read=latest['evidence'])
        old_atomic(path, obj)

    legacy.recv_exact = receive
    legacy.requested = request
    legacy.atomic_json = atomic
    legacy.json = IsolatedJSON()
    legacy.duration = lambda _: float(execution['duration_s'])
    sys.argv = [str(run/'sources/policy/worker.py'), '--run', str(run), '--test', 'forward', '--socket', args.socket]
    if execution.get('terrain_target_manifest'):
        sys.argv += ['--terrain-target-manifest', str(run/'terrain_target_manifest.json')]
    try:
        legacy.main()
    finally:
        log.close()


if __name__ == '__main__':
    main()
