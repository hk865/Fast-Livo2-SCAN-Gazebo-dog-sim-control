#!/usr/bin/env python3
"""Source-snapshot-only native guard observer audit; no ROS or live state."""
from __future__ import annotations
import ast
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time
import types
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_defs(path, names, namespace):
    tree = ast.parse(path.read_text())
    body = [v for v in tree.body if isinstance(v, (ast.FunctionDef, ast.ClassDef)) and v.name in names]
    if {v.name for v in body} != set(names):
        raise RuntimeError('Required source definitions missing')
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(path), 'exec'), namespace)


def stats(seconds):
    values = np.asarray(seconds) * 1e6
    return {'n': len(values), 'p50_us': float(np.quantile(values, .5)), 'p95_us': float(np.quantile(values, .95)),
        'p99_us': float(np.quantile(values, .99)), 'max_us': float(values.max()), 'mean_us': float(values.mean())}


def main():
    if (OUT / 'review.json').exists():
        raise RuntimeError('Never overwrite the original review receipt')
    sources = OUT / 'sources'
    sources.mkdir(exist_ok=False)
    originals = {
        'guard_audit.py': ROOT / 'navigation/guard_audit.py',
        'teacher_controller.py': ROOT / 'navigation/controller.py',
        'shared_control_core.py': ROOT.parent / 'navigation/control_core.py',
        'shared_controller.py': ROOT.parent / 'navigation/controller.py',
        'runtime_io.py': ROOT / 'navigation/runtime_io.py',
        'guard_trace_schema.json': ROOT / 'navigation/dynamic/guard_trace_schema.json',
        'flat_profile.json': ROOT / 'navigation/flat_relative_roundtrip.json',
        'dynamic12_profile.json': ROOT / 'navigation/dynamic/flat_dynamic_profile.json',
        'dynamic20_profile.json': ROOT / 'navigation/dynamic/flat_dynamic_profile_turn20.json',
        'teacher_transition.py': ROOT / 'navigation/teacher_transition.py',
        'scoped_profile.py': ROOT / 'navigation/scoped_profile.py',
        'dynamic_prepare.py': ROOT / 'navigation/dynamic/prepare.py',
        'worker.py': ROOT / 'policy/worker.py',
        'contract.json': ROOT / 'policy/contract.json',
    }
    for name, path in originals.items():
        shutil.copyfile(path, sources / name)
    namespace = {'np': np, 'math': math, 'FunctionType': types.FunctionType}
    load_defs(sources / 'shared_control_core.py', ['obstacle_ahead', 'steering_obstacle_ahead'], namespace)
    load_defs(sources / 'guard_audit.py', ['NativeGuardAudit', 'result_json'], namespace)
    native, obstacle, Audit = (namespace[n] for n in ('steering_obstacle_ahead', 'obstacle_ahead', 'NativeGuardAudit'))
    obstacle_count = [0]

    def counted_obstacle(*args, **kwargs):
        obstacle_count[0] += 1
        return obstacle(*args, **kwargs)

    namespace['obstacle_ahead'] = counted_obstacle
    observer = Audit(native)
    checks = {
        'private_guard_code_object_is_original': observer.instrumented.__code__ is native.__code__,
        'native_defaults_and_closure_retained': observer.instrumented.__defaults__ is native.__defaults__
            and observer.instrumented.__closure__ is native.__closure__
            and observer.instrumented.__kwdefaults__ is native.__kwdefaults__,
        'observer_globals_are_private_mapping': observer.instrumented.__globals__ is not native.__globals__,
        'original_obstacle_global_is_unchanged': native.__globals__['obstacle_ahead'] is counted_obstacle,
        'observer_only_replaces_obstacle_binding': set(observer.instrumented.__globals__) == set(native.__globals__)
            and all(observer.instrumented.__globals__[k] is v for k, v in native.__globals__.items() if k != 'obstacle_ahead'),
    }
    rng = np.random.default_rng(20261004)
    input_changes = []
    result_errors = []
    baseline_times, instrumented_times = [], []
    # 500 synthetic input cases, actual frozen code geometry. No simulator truth/input files used.
    for i in range(500):
        cloud = rng.uniform([-.4, -.8, -.3], [1.4, .8, .6], size=(128 + i % 64, 3))
        if i % 10 == 0:
            cloud = np.empty((0, 3))
        if i % 13 == 0:
            cloud[:, 2] = -.2  # Ground-only sample.
        pose = rng.uniform([-.1, -.1, -.03], [.1, .1, .03], size=3)
        target = pose + np.array([.65, .15 * math.sin(i), .03])
        steer = np.array([math.cos(.5 * i), math.sin(.5 * i)])
        route = np.array([pose, target])
        before = [a.copy() for a in (cloud, pose, target, steer, route)]
        args = cloud, pose, target, steer, route
        begin = time.perf_counter_ns()
        expected = native(*args)
        baseline_times.append((time.perf_counter_ns() - begin) * 1e-9)
        begin = time.perf_counter_ns()
        actual = observer(*args)
        instrumented_times.append((time.perf_counter_ns() - begin) * 1e-9)
        if expected != actual:
            result_errors.append({'case': i, 'expected': expected, 'actual': actual})
        if any(not np.array_equal(a, b) for a, b in zip(args, before)):
            input_changes.append(i)
        assert len(observer.calls) == 2
    checks['500_actual_native_geometry_results_exactly_equal'] = not result_errors
    checks['500_input_cloud_pose_target_route_arrays_unchanged'] = not input_changes
    checks['exactly_two_native_corridor_calls_per_original_and_instrumented'] = obstacle_count[0] == 2000
    checks['existing_original_globals_not_mutated_after_500'] = native.__globals__['obstacle_ahead'] is counted_obstacle
    # Existing per-thread profiler must be left untouched, including native exceptions.
    saved_profile = sys.getprofile()
    def sentinel(frame, event, arg):
        return None
    def raising_obstacle(*args, **kwargs):
        raise ValueError('offline original geometry error')
    exception_namespace = dict(namespace)
    exception_namespace['obstacle_ahead'] = raising_obstacle
    native_raising = types.FunctionType(native.__code__, exception_namespace, native.__name__, native.__defaults__, native.__closure__)
    exception_observer = Audit(native_raising)
    try:
        sys.setprofile(sentinel)
        try:
            exception_observer(np.empty((0, 3)), np.zeros(3), np.ones(3), np.array([1., 0.]), np.array([[0., 0., 0.], [1., 0., 0.]]))
        except ValueError as exc:
            original_exception = {'type': type(exc).__name__, 'args': list(exc.args)}
        else:
            original_exception = None
        checks['existing_sys_profile_unchanged_after_native_exception'] = sys.getprofile() is sentinel
        checks['original_geometry_exception_is_not_hidden_or_replaced'] = original_exception == {'type': 'ValueError', 'args': ['offline original geometry error']}
    finally:
        sys.setprofile(saved_profile)  # Offline test's sentinel only, not observer behavior.
    checks['offline_test_restored_its_own_prior_profiler'] = sys.getprofile() is saved_profile
    # Execute exact record_native_guard method using a fake accepted sensor context, no ROS node/import.
    tree = ast.parse((sources / 'teacher_controller.py').read_text())
    teacher = next(v for v in tree.body if isinstance(v, ast.ClassDef) and v.name == 'TeacherNavigation')
    method = copy.deepcopy(next(v for v in teacher.body if isinstance(v, ast.FunctionDef) and v.name == 'record_native_guard'))
    fakeclass = ast.ClassDef(name='FakeInputNode', bases=[], keywords=[], body=[method], decorator_list=[])
    method_namespace = {'np': np, 'hashlib': hashlib, 'time': time, 'result_json': namespace['result_json']}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fakeclass], type_ignores=[])), str(sources / 'teacher_controller.py'), 'exec'), method_namespace)
    node = method_namespace['FakeInputNode']()
    big_ns = 9007199254741234  # Beyond2**53: canonical integer stamps must roundtrip exactly.
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=big_ns))
    cloud = np.array([[.5, 0., .1], [.52, .02, .1], [.54, -.02, .1]])
    cloud.setflags(write=False)
    cloud_hash = hashlib.sha256(np.asarray(cloud, dtype='<f8').tobytes()).hexdigest()
    pose, target = np.zeros(3), np.array([1., 0., 0.])
    route = np.array([pose, target])
    node.guard_cloud = cloud
    node.guard_cloud_receipt = {'stamp_ns': big_ns - 5000000, 'frame_id': 'camera_init',
        'received_monotonic_wall': time.monotonic(), 'callback_ros_clock_ns': big_ns - 1000000,
        'nearest_slam_pose_stamp_ns': big_ns - 5000000, 'filtering_body_pose': [0., 0., 0.],
        'filtering_body_rotation': np.eye(3).tolist(), 'filtered_xyz_float64_sha256': cloud_hash,
        'filtered_points': 3, 'self_filtered_points': 0}
    node.cloud_stamp = node.guard_cloud_receipt['stamp_ns']
    node.pose_stamp = big_ns - 4000000
    node.pose_quaternion = [0., 0., 0., 1.]
    node.native_guard_audit = Audit(native)
    node.guard_sequence = 0
    node.request_id = 'offline_fake_actual_source_context'
    node.waypoint_index = 0
    node.active_trajectory_id = 7
    node.trajectory_association = types.SimpleNamespace(reference_stamp=[10, 0])
    node.trajectory_archive_reference = 'navigation_trajectories/fake.npz'
    node.rotation, node.tracking_pose = np.eye(3), pose.copy()
    node.pose_updated = time.monotonic()
    node.waypoints = [target]
    node.obstacle_hold = False
    node.obstacle_clear_since = None
    result = node.record_native_guard(cloud, pose, target, np.array([1., 0.]), route)
    trace = node.pending_guard_row
    reloaded = json.loads(json.dumps(trace, allow_nan=False))
    checks['controller_hook_returns_exact_original_guard_result'] = result == native(cloud, pose, target, np.array([1., 0.]), route)
    checks['trace_preserves_actual_integer_stamps_above2pow53'] = all(
        isinstance(reloaded[k], int) and reloaded[k] == trace[k]
        for k in ('compute_ros_clock_ns', 'control_pose_stamp_ns', 'cloud_header_stamp_ns', 'cloud_callback_ros_clock_ns', 'cloud_filtering_body_stamp_ns'))
    checks['trace_filtered_buffer_hash_is_exact_source_buffer'] = trace['filtered_xyz_float64_sha256'] == cloud_hash
    checks['trace_route_hash_exact_little_endian_f64'] = trace['route_float64_sha256'] == hashlib.sha256(np.asarray(route, dtype='<f8').tobytes()).hexdigest()
    checks['trace_records_both_native_results_and_union'] = trace['goal_corridor_result'] == namespace['result_json'](node.native_guard_audit.calls[0][1]) and trace['motion_corridor_result'] == namespace['result_json'](node.native_guard_audit.calls[1][1]) and trace['union_result'] == namespace['result_json'](result)
    checks['fake_schema_execution_keeps_cloud_readonly_and_unchanged'] = not cloud.flags.writeable and cloud_hash == hashlib.sha256(np.asarray(cloud, dtype='<f8').tobytes()).hexdigest()
    observer_ast = ast.parse((sources / 'guard_audit.py').read_text())
    calls = [v.func for v in ast.walk(observer_ast) if isinstance(v, ast.Call)]
    checks['observer_installs_no_sys_profile_or_fsync_or_file_io'] = not any(
        isinstance(v, ast.Attribute) and v.attr in ('setprofile', 'getprofile', 'fsync', 'open', 'write', 'write_text', 'replace')
        or isinstance(v, ast.Name) and v.id == 'open' for v in calls)
    profiles = {name: json.loads((sources / name).read_text()) for name in ('flat_profile.json', 'dynamic12_profile.json', 'dynamic20_profile.json')}
    before, after = profiles['dynamic12_profile.json'], profiles['dynamic20_profile.json']
    diff = {k: {'before': before.get(k), 'after': after.get(k)} for k in set(before) | set(after) if before.get(k) != after.get(k)}
    checks['turn20_scope_is_distinct_from_flat_and_dynamic12'] = after['experiment'] != before['experiment'] != profiles['flat_profile.json']['experiment'] and after['max_yaw_rate_radps'] == .2 and before['max_yaw_rate_radps'] == profiles['flat_profile.json']['max_yaw_rate_radps'] == .12
    checks['turn20_changes_only_experiment_turn_cap_and_variant_metadata'] = set(diff) == {'experiment', 'max_yaw_rate_radps', 'variant'}
    result = {'schema': 'teacher_native_guard_observer_offline_review/v1',
        'asof_utc': datetime.now(timezone.utc).isoformat(), 'prospective_runtime_unverified': True,
        'scope': 'Frozen read snapshots of still-in-development code; no ROS/Kit/sim/native process signals or actual navigation pass',
        'source_hashes': {name: {'actual_source_path': str(path), 'snapshot_sha256': sha(sources / name), 'current_still_equals_snapshot': sha(path) == sha(sources / name)} for name, path in originals.items()},
        'executed_audit_sha256': sha(__file__), 'checks': checks, 'offline_checks_passed': all(checks.values()),
        'geometry_cases': {'cases': 500, 'two_calls_each_baseline_and_observer': obstacle_count[0],
            'errors': result_errors, 'input_mutations': input_changes,
            'native_baseline': stats(baseline_times), 'observer_code_clone': stats(instrumented_times),
            'median_per_pair_observer_minus_native_us': float(np.median(np.asarray(instrumented_times) - np.asarray(baseline_times)) * 1e6),
            'limit': 'Sequential synthetic cases and timing are affected by cache/scheduler; not actual callback latency or a formal worst-case budget.'},
        'fake_context_trace': trace, 'native_exception': original_exception,
        'profile_implementation': 'Actual code uses FunctionType private globals, not sys.setprofile. No observer profile is installed; unrelated existing profile remains unchanged even on original geometry exceptions.',
        'turn20_profile_diff': diff,
        'review_notes': [
            'Original two native corridor calls and union return are unchanged; logging adds target copies/list appends and serialization work only.',
            'Teacher base module is loaded privately; only that module binding is replaced for dynamic scope. Shared camera-mode source files are not changed by observer.',
            'Guard input cloud and pose are actual accepted registered cloud/SLAM. Passive moving_obstacle simulator pose is not read in the hook.',
            'Canonical pose/cloud/clock stamps are integers. Derived sim seconds are explanatory float fields, not substitutes for integer source stamps.',
            'filtered_xyz_float64_sha256 hashes the actual self-filtered immutable guard XYZ, not original PointCloud2 serialized raw payload. Full per-guard cloud arrays are not saved by this hook.',
            'Clearance geometry replay requires matching original passive cloud and actual filter pose to reproduce the exact hash; hash alone is not clear-space proof.',
            'Errors while gathering evidence can still raise after native geometry; this is an added fail-closed audit boundary, not a universal result-preservation guarantee under logging failure.',
            'Single rclpy executor invocation path is assumed; observer.calls is per-instance mutable state, not designed for concurrent/reentrant guard invocations.',
            'Asynchronous EvidenceWriter append remains bounded and no fsync is introduced. Required record count/drained receipt and actual event hashes must be verified after live run.',
            'No actual live trace, stopping, recovery, region arrival or global Sim2Sim success is asserted by this offline review.'
        ]}
    (OUT / 'review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': result['offline_checks_passed'], 'checks': checks,
        'output': str(OUT), 'receipt_sha256': sha(OUT / 'review.json'),
        'geometry_times': result['geometry_cases']}))
    if not result['offline_checks_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
