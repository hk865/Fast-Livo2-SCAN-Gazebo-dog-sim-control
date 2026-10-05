#!/usr/bin/env python3
"""Append-only active endpoint hold evidence; never an old parking upgrade.

Only hash-verified archived references are imported. No Controller is created,
no policy is inferred and no simulator/process/control interface is used.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np

MODEL_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
V1_CORE_SHA = '82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d'
V1_CURVE_SHA = '14adc95fc60bd65f4b9a8c0dd456c4af680c8315a191016a7531e328582aeb28'
COMMON_SHA = '7e386d004e2d1187753609b73f6b9f419c417aa35c1a30147d28385ffcd9fb8a'
PROTOCOL_SHA = 'e2308b7b7585f933081cf741bd8670e950607cea495c3f4eca293e33ae7796a8'
OUTPUT = 'summary_active_hold_independent.json'
ARRAY_OUTPUT = 'active_hold_independent_arrays.npz'
OLD_PARKING = 'fixed_final_goal_parking'
COMMON_NONPARK = (
    'physical_time_and_telemetry_coverage', 'original_control_clock_and_feedback_causality',
    'causal_control_reference_coverage', 'native_physics_coverage_and_state_pairing',
    'flat_or_requested_route_distance', 'drive_heading', 'stable_real_COM_speed',
    'as_recorded_physical_safety', 'ordered_route_and_final_goal',
    'declared_completion_and_termination', 'actual_runtime_receipt',
    'strict_terrain_support_applicability')
DRIVE_FIELDS = ('path', 'path_parameters_sha256', 'path_length_m', 'maximum_abs_curvature_1pm',
                'desired_speed', 'feedback_hz', 'duration_s', 'post_completion_s',
                'command_limits', 'gains', 'base_com_offset', 'spawn')
SLEW = np.asarray([.6, .6, .8])


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): h.update(block)
    return h.hexdigest()


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def serial(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {k: serial(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [serial(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return None
    return value


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try: spec.loader.exec_module(module)
    finally: sys.dont_write_bytecode = previous
    return module


def wrapped(value):
    return np.arctan2(np.sin(value), np.cos(value))


def fixed_first_window(t, position, yaw, origin_velocity, body_omega, yawdot, declared,
                       width=5.):
    """No delayed/quiet-window selection; missing first declared sample rejects."""
    t = np.asarray(t, float)
    if not math.isfinite(float(declared)) or width != 5.:
        raise ValueError('A finite first declaration and original five seconds are required')
    start_index = int(np.searchsorted(t, declared-1e-8))
    if start_index >= len(t) or abs(t[start_index]-declared) > 1e-8:
        raise ValueError('First declared native state is absent; cannot move window')
    end = float(t[start_index]+width)
    indexes = np.flatnonzero((t >= t[start_index]-1e-8) & (t <= end+1e-8))
    complete = (len(indexes) == 1001 and abs(t[indexes[-1]]-end) <= 1e-8 and
                np.all(abs(np.diff(t[indexes])-.005) <= 1e-8))
    xy = float(np.linalg.norm(position[indexes, :2]-position[start_index, :2], axis=1).max())
    dyaw = float(abs(yaw[indexes]-yaw[start_index]).max())
    return dict(window_s=[float(t[start_index]), end], first_native_index=start_index,
                native_rows=len(indexes), complete=bool(complete), native_indexes=indexes,
                xy_drift_m=xy, yaw_drift_rad=dyaw,
                peak_origin_planar_speed_mps=float(np.linalg.norm(origin_velocity[indexes, :2], axis=1).max()),
                peak_body_wz_radps=float(abs(body_omega[indexes, 2]).max()),
                peak_Euler_yaw_rate_radps=float(abs(yawdot[indexes]).max()))


def status_from_checks(checks):
    states = [c['status'] for c in checks.values()]
    return 'failed' if 'failed' in states else 'passed' if states and all(s == 'passed' for s in states) else 'unverified'


def common_nonparking_gate(original, replay):
    """An old parking failure is retained, never used to hide other failures."""
    return (original.get('schema') == 'independent_truth_teacher_pid_benchmark/v2' and
            original.get('navigation_ground_truth_used') is True and
            original.get('SLAM_navigation_verified') is False and original.get('real_robot_verified') is False and
            all(original.get('checks', {}).get(k, {}).get('status') == 'passed' and
                replay.get('checks', {}).get(k, {}).get('status') == 'passed' for k in COMMON_NONPARK))


def native_index(t, feedback):
    i = int(np.searchsorted(t, feedback-1e-8))
    if i >= len(t) or abs(t[i]-feedback) > 1e-8:
        raise ValueError('Original feedback has no exact causal native state')
    return i


def archive_key(original):
    path = Path(original)
    if path.name == 'reference_v1_core.py': return 'references/v1_core.py'
    if path.name == 'reference_curve_receipt.py': return 'references/v1_curve_receipt.py'
    if 'policy' in path.parts: return 'policy/'+path.name
    if 'simulation' in path.parts: return 'native/'+path.name
    if path.name == 'capture.py': return 'observer/capture.py'
    return 'truth/'+path.name


def validate_contract(p, path):
    target = dict(position_world_xyz=path.final_xyz.tolist(), heading0_rad=path.heading0,
                  path_parameters_sha256=path.sha256)
    ap = p['active_parking']
    exact = dict(position_kp=.18, position_kd=.20, yaw_kp=.65, yaw_kd=.18,
        position_deadband_m=.003, yaw_deadband_rad=.005,
        reference_xy_norm_limit_mps=.025, reference_yaw_limit_radps=.07,
        command_limits_body=[.06, .04, .10], capture_xy_entry_m=.025, capture_xy_hold_m=.03,
        capture_yaw_entry_rad=.035, capture_yaw_hold_rad=.045,
        capture_actual_xy_speed_mps=.03, capture_actual_yawrate_radps=.06, capture_fresh_dwell_s=.6,
        first_fixed_parking_window_s=5., maximum_xy_drift_m=.05, maximum_yaw_drift_rad=.1,
        maximum_native_xy_speed_mps=.08, maximum_native_Euler_yawrate_radps=.1, maximum_native_body_wz_radps=.1,
        command_slew_per_s=[.6, .6, .8], feedback_TTL_sim_and_wall_s=.3)
    if (ap.get('schema') != 'fixed_endpoint_active_parking/v1' or ap.get('target') != target or
        ap.get('target_sha256') != canonical_sha(target) or
        ap.get('target_kind') != 'prospective_fixed_path_endpoint' or
        ap.get('actual_command_must_be_zero') is not False or
        any(ap.get(k) != v for k, v in exact.items())):
        raise ValueError('Prospective weak_hold_p1 contract differs; use a new receipt revision')
    return ap, target


def capture_evidence(control, a, path, ap):
    fresh = [c for c in control if c.get('controller_updated') is True and c.get('mode') in ('capture', 'active_hold')]
    if not fresh or fresh[0]['mode'] != 'capture': raise ValueError('No original capture transition')
    first_hold = next((c for c in fresh if c['mode'] == 'active_hold'), None)
    if first_hold is None: raise ValueError('No actual first active_hold declaration')
    first = fresh[0]; start = float(first['parking_capture_state_time_s'])
    declared = float(first_hold['parking_hold_declared_state_time_s'])
    if abs(start-first['feedback_time_s']) > 1e-8 or abs(declared-first_hold['feedback_time_s']) > 1e-8:
        raise ValueError('Capture/hold declared state must be the original feedback state')
    reset = [c for c in control if c.get('entry_integral_reset') is True]
    if len(reset) != 1 or reset[0] is not first: raise ValueError('Parking integral must reset once, at capture')
    holding_started = False; eligible_since = None; records = []; eligible_ok = True
    old_ft = None
    for c in fresh:
        ft = float(c['feedback_time_s'])
        if old_ft is not None and (ft <= old_ft or ft-old_ft > .30000001):
            raise ValueError('Repeated/backward/stale fresh capture/hold feedback')
        old_ft = ft; i = native_index(a['t'], ft)
        distance = float(np.linalg.norm(a['position'][i, :2]-path.final_xyz[:2]))
        eyaw = float(abs(wrapped(a['unwrapped_yaw'][i]-path.heading0)))
        speed = float(np.linalg.norm(a['origin_body_velocity'][i, :2]))
        rate = float(abs(a['Euler_yaw_rate'][i]))
        entry = eligible_since is None
        eligible = (distance <= ap['capture_xy_entry_m' if entry else 'capture_xy_hold_m'] and
                    eyaw <= ap['capture_yaw_entry_rad' if entry else 'capture_yaw_hold_rad'] and
                    speed < .03 and rate < .06)
        if not holding_started:
            if eligible and eligible_since is None: eligible_since = ft
            if not eligible: eligible_since = None
            eligible_ok &= c.get('capture_measurement_eligible') is bool(eligible)
            logged_start = c.get('capture_fresh_dwell_start_state_time_s')
            eligible_ok &= ((logged_start is None and eligible_since is None) or
                            (logged_start is not None and eligible_since is not None and abs(logged_start-eligible_since) <= 1e-8))
            if c['mode'] == 'active_hold':
                if not eligible or eligible_since is None or ft-eligible_since < .6-1e-9:
                    raise ValueError('Active hold declared before original fresh-stamp dwell')
                holding_started = True
            elif eligible_since is not None and ft-eligible_since >= .6-1e-9:
                raise ValueError('First eligible hold declaration was delayed')
        elif c['mode'] != 'active_hold': raise ValueError('Active hold changed back to capture')
        records.append(dict(state_time_s=ft, endpoint_distance_m=distance, heading_error_rad=eyaw,
                            actual_origin_planar_speed_mps=speed, actual_Euler_yawrate_radps=rate,
                            eligible=eligible, mode=c['mode']))
    if not eligible_ok: raise ValueError('Logged capture eligibility/dwell differs from original native measurements')
    return dict(first_capture_state_time_s=start, first_hold_state_time_s=declared,
                first_capture_elapsed_s=float(first['parking_capture_elapsed_s']),
                first_hold_elapsed_s=float(first_hold['parking_hold_declared_elapsed_s']),
                fresh_rows=len(fresh), capture_rows=sum(c['mode'] == 'capture' for c in fresh),
                first_hold_eligible_dwell_start_state_time_s=first_hold['capture_fresh_dwell_start_state_time_s'],
                capture_support_semantics='Fresh native feedback sample dwell; not an assertion of between-sample eligibility',
                sample_metrics=records), fresh


def hold_pi_replay(control, a, ap, helper, gains):
    """Rebuild weak outer hold and its own antiwindup PI, never infer policy."""
    integral = np.zeros(3); filt = None; outer = None; errors = []; previous_estimate = np.zeros(3)
    kp = np.asarray([gains['velocity_kp_x'], gains['velocity_kp_y'], gains['rate_kp']])
    ki = np.asarray([gains['velocity_ki_x'], gains['velocity_ki_y'], gains['rate_ki']])
    limits = np.asarray(ap['command_limits_body']); last_ft = None
    def limit(v):
        v = np.clip(v, -limits, limits)
        n = np.linalg.norm(v[:2])
        if n > limits[0]: v[:2] *= limits[0]/n
        return v
    def deadband(v, d): return np.sign(v)*np.maximum(abs(v)-d, 0.)
    for c in control:
        hold = c['mode'] in ('capture', 'active_hold')
        fresh = c.get('controller_updated') is True
        if fresh:
            i = native_index(a['t'], c['feedback_time_s']); dt = float(c['header_dt_s'])
            R = helper.rotations(a['quaternion_wxyz'][i:i+1])[0]
            actual = np.r_[a['body_COM_velocity'][i, :2], a['body_omega'][i, 2]]
            measurement = np.r_[a['origin_world_velocity'][i, :2], a['Euler_yaw_rate'][i]]
            # These filter states continue from original drive, including the
            # capture frame that was already processed once by its parent.
            beta = 1. if filt is None else dt/(.1+dt)
            alpha = 1. if outer is None else dt/(.2+dt)
            filt = actual.copy() if filt is None else filt+beta*(actual-filt)
            outer = measurement.copy() if outer is None else outer+alpha*(measurement-outer)
            if hold:
                if c.get('entry_integral_reset') is True: integral[:] = 0.
                delta = np.asarray(ap['target']['position_world_xyz'])[:2]-a['position'][i, :2]
                eyaw = float(wrapped(ap['target']['heading0_rad']-a['unwrapped_yaw'][i]))
                velocity = np.r_[ap['position_kp']*deadband(delta, ap['position_deadband_m'])-ap['position_kd']*outer[:2], 0.]
                norm = np.linalg.norm(velocity[:2])
                if norm > .025: velocity[:2] *= .025/norm
                yaw_ref = float(np.clip(ap['yaw_kp']*deadband(eyaw, ap['yaw_deadband_rad'])-ap['yaw_kd']*outer[2], -.07, .07))
                roll = math.atan2(R[2, 1], R[2, 2]); pitch = math.asin(np.clip(-R[2, 0], -1, 1))
                wref = (yaw_ref*math.cos(pitch)-math.sin(roll)*a['body_omega'][i, 1])/math.cos(roll)
                origin_target = R.T@velocity
                com_target = origin_target+np.cross([0., 0., wref], np.asarray(gains['COM_offset']))
                target = np.r_[com_target[:2], wref]; error = target-filt
                candidate = np.clip(integral+dt*error, -.5, .5)
                raw = target+kp*error+ki*candidate; clipped = limit(raw)
                blocked_axis = error*(raw-clipped) > 0
                candidate = np.where(blocked_axis, integral, candidate)
                next_slew = previous_estimate+np.clip(clipped-previous_estimate, -SLEW*.02, SLEW*.02)
                blocked_slew = error*(raw-next_slew) > 1e-9
                candidate = np.where(blocked_slew, integral, candidate); integral = candidate
                raw = target+kp*error+ki*candidate; clipped = limit(raw)
                pi = c['velocity_PI']
                values = ((filt, pi['filtered_actual_body']), (error, pi['error']),
                          (kp*error, pi['P']), (ki*candidate, pi['I']),
                          (target, c['v_reference_body']), (raw, c['raw_command_body']),
                          (clipped, c['command_body']), (velocity, c['reference_velocity_world']))
                err = max(float(np.max(abs(x-np.asarray(y)))) for x, y in values)
                err = max(err, abs(yaw_ref-c['parking_reference_yaw_rate_radps']), abs(dt-c['active_integral_dt_s']))
                if not (np.array_equal(blocked_axis, pi['blocked_axis']) and np.array_equal(blocked_slew, pi['blocked_slew'])):
                    err = math.inf
                errors.append(err)
            last_ft = float(c['feedback_time_s'])
        elif hold and last_ft is not None:
            # Held rows retain old PI; no additional integral or filter update.
            errors.append(float(np.max(abs(np.asarray(c['velocity_PI']['I'])-ki*integral))))
        previous_estimate += np.clip(np.asarray(c['command_body'])-previous_estimate, -SLEW*.02, SLEW*.02)
    return dict(rows=len(errors), maximum_absolute_replay_error=max(errors, default=math.inf),
                passed=bool(errors) and max(errors) <= 1e-8,
                method='Native COM/body/Euler measurements, original filters, immutable goal weak PD, separate bounded antiwindup PI; actual fresh selector only')


def evaluate(run, write=True):
    run = Path(run).resolve(); output = run/OUTPUT; array_output = run/ARRAY_OUTPUT
    if write and (output.exists() or array_output.exists()):
        raise FileExistsError('Append-only receipt refuses to replace prior evidence')
    checks = {}; verified = {}; arrays = {}; ancestors = {}; metrics = {}
    def check(name, ok, **details):
        checks[name] = dict(status='unverified' if ok is None else 'passed' if bool(ok) else 'failed', **serial(details))
    def verify(path, digest=None):
        path = Path(path).resolve(); actual = sha(path)
        if digest is not None and actual != digest: raise ValueError('SHA mismatch: '+str(path))
        verified[str(path)] = actual; return actual
    try:
        for name in ('summary_truth_pid.json', 'truth_profile.json', 'source_manifest.json', 'runtime_manifest.json',
                     'policy_manifest.json', 'worker_result.json', 'actuator.jsonl', 'telemetry.jsonl',
                     'control.jsonl', 'observations_actions.npz', 'world.sdf'): verify(run/name)
        common = json.loads((run/'summary_truth_pid.json').read_text())
        profile = json.loads((run/'truth_profile.json').read_text())
        manifest = json.loads((run/'source_manifest.json').read_text())
        runtime = json.loads((run/'runtime_manifest.json').read_text())
        policy = json.loads((run/'policy_manifest.json').read_text())
        required = ('truth/core.py', 'truth/worker.py', 'truth/run.py', 'truth/evaluate.py', 'truth/protocol.json',
                    'truth/profiles.py', 'truth/active_hold_receipt.py', 'truth/test_active_hold_receipt.py',
                    'truth/README_RECEIPT.md', 'references/v1_core.py', 'references/v1_curve_receipt.py',
                    'policy/worker.py', 'policy/observation.py', 'policy/contract.json',
                    'native/teacher_actuator.cpp', 'native/libteacher_actuator.so')
        if not set(required) <= set(manifest): raise ValueError('Missing frozen execution/reference archive')
        for key, value in manifest.items():
            source = (run/'sources'/key).resolve()
            if not source.is_relative_to(run/'sources'): raise ValueError('Archive path escapes run')
            verify(source, value if isinstance(value, str) else value['sha256'])
        verify(run/'source_manifest.json', runtime['source_manifest_sha256'])
        verify(run/'truth_profile.json', runtime['truth_profile_sha256'])
        verify(run/'sources/native/libteacher_actuator.so', runtime['native_plugin_sha256'])
        verify(run/'sources/references/v1_core.py', V1_CORE_SHA)
        verify(run/'sources/references/v1_curve_receipt.py', V1_CURVE_SHA)
        verify(run/'sources/truth/evaluate.py', COMMON_SHA)
        verify(run/'sources/truth/protocol.json', PROTOCOL_SHA)
        verify(run/'sources/truth/active_hold_receipt.py', sha(__file__))
        # Prospective freeze identities are matched against immutable archive,
        # never against a later live working tree. All executing sources needed.
        frozen = profile.get('frozen_source_hashes', {})
        if not frozen: raise ValueError('No prospective source freeze')
        for original, digest in frozen.items():
            key = archive_key(original)
            archived = manifest.get(key)
            archived = archived if isinstance(archived, str) else archived.get('sha256') if isinstance(archived, dict) else None
            if not Path(original).is_absolute() or digest != archived:
                raise ValueError('Frozen source lacks matching executed archive: '+original)
        for key in required:
            if manifest[key] not in set(frozen.values()): raise ValueError('Required source absent from prospective freeze: '+key)
        verify(policy['checkpoint'], MODEL_SHA)
        scope_ok = (profile.get('schema') == 'truth_teacher_static_curve_active_hold_profile/v2' and
            profile['terrain'] == 'flat' and policy['checkpoint_sha256'] == MODEL_SHA and
            runtime['frozen_model_sha256'] == MODEL_SHA and policy['inference_device'] == 'cpu' and
            policy['torch_threads'] == 1 and runtime['exclusive_writer'] == 'teacher_sim::TeacherActuator' and
            runtime['uses_truth_for_control'] is True and runtime['counts_as_SLAM_navigation'] is False and
            runtime['training_processes_signaled'] is False and runtime['real_robot'] is False)
        check('all_frozen_source_model_CPU_exclusive_scope', scope_ok,
              model_sha256=MODEL_SHA, actor_privileged_dimensions=232, actor_input_dimensions=247,
              measured_motor_torque=False, source_manifest_verified=True)
        core_bytes = (run/'sources/truth/core.py').read_bytes()
        reference_bytes = (run/'sources/references/v1_core.py').read_bytes()
        prefix_ok = core_bytes[:len(reference_bytes)] == reference_bytes
        parent = profile['v1_parent_profile']; verify(parent['path'], parent['sha256'])
        parent_profile = json.loads(Path(parent['path']).read_text())
        drive_fields_ok = (all(profile[k] == parent_profile[k] for k in DRIVE_FIELDS) and
                          canonical_sha({k: profile[k] for k in DRIVE_FIELDS}) == profile['v1_drive_fields_sha256'])
        check('original_v1_drive_source_prefix_and_profile_unchanged', prefix_ok and drive_fields_ok,
              prefix_sha256=V1_CORE_SHA, prefix_size_bytes=len(reference_bytes), immutable_drive_fields=list(DRIVE_FIELDS))
        helper = load_module(run/'sources/references/v1_curve_receipt.py', '_hold_reference_curve_'+sha(run/'truth_profile.json')[:12])
        reference = load_module(run/'sources/references/v1_core.py', '_hold_reference_path_'+sha(run/'truth_profile.json')[:12])
        common_helper = load_module(run/'sources/truth/evaluate.py', '_hold_common_'+sha(run/'truth_profile.json')[:12])
        path = reference.StaticPath(profile['path'])
        ap, target = validate_contract(profile, path)
        check('prospective_fixed_endpoint_unchanged', path.sha256 == profile['path_parameters_sha256'],
              target=target, target_sha256=canonical_sha(target), target_is_captured_pose=False)
        for filename, digest in common['input_source_sha256'].items(): verify(filename, digest)
        verify(common['independent_arrays']['path'], common['independent_arrays']['sha256'])
        ancestors['common'] = dict(path=str(run/'summary_truth_pid.json'), sha256=sha(run/'summary_truth_pid.json'),
            status=common['status'], original_checks=common['checks'], original_parking_check=common['checks'].get(OLD_PARKING),
            status_preserved=True, excluded_from_new_gate_only=[OLD_PARKING])
        if (run/'summary_curve_independent.json').exists():
            old_curve = json.loads((run/'summary_curve_independent.json').read_text()); verify(run/'summary_curve_independent.json')
            ancestors['v1_curve'] = dict(path=str(run/'summary_curve_independent.json'), sha256=sha(run/'summary_curve_independent.json'),
                status=old_curve['status'], original_checks=old_curve['checks'], used_as_v2_identity=False, status_preserved=True)
        replay = common_helper.evaluate_run(run, write=False)
        check('all_twelve_original_nonparking_common_checks', common_nonparking_gate(common, replay),
              required_checks=list(COMMON_NONPARK), original_common_status=common['status'],
              replay_checks={k: replay['checks'].get(k) for k in COMMON_NONPARK},
              old_zero_parking_status=common['checks'].get(OLD_PARKING, {}).get('status'), thresholds_loosened=False)
        protocol = json.loads((run/'sources/truth/protocol.json').read_text())
        native = [r for r in helper.rows(run/'actuator.jsonl') if r.get('kind') == 'physics_step']
        control = helper.rows(run/'control.jsonl'); telemetry = helper.rows(run/'telemetry.jsonl')
        a = helper.native_arrays(native, profile['base_com_offset']); t = a['t']
        ct = helper.finite_array(control, 'control_t_s'); ft = helper.finite_array(control, 'feedback_time_s')
        tt = helper.finite_array(telemetry, 'state_physics_world_time')
        with np.load(run/'observations_actions.npz', allow_pickle=False) as actor:
            actor_ok = (actor['observations'].shape == (len(telemetry), 247) and
                        actor['actions'].shape == (len(telemetry), 12) and
                        np.isfinite(actor['observations']).all() and np.isfinite(actor['actions']).all())
        origin_error = float(np.max(abs(a['origin_body_velocity']-a['recorded_native_origin_body_velocity'])))
        check('native_200Hz_actor_50Hz_and_COM_phase_coverage', actor_ok and helper.native_coverage(a) and
            len(control) == len(telemetry) and len(tt) > 1 and np.all(abs(np.diff(tt)-.02) < 1e-8) and
            np.all(np.diff(ct) > 0) and t[0] <= tt[0]+1e-8 and t[-1] >= tt[-1]-1e-8 and origin_error <= 1e-9,
            native_rows=len(native), actor_rows=len(telemetry), native_phase='t-dt; dt=.005',
            maximum_COM_lever_arm_replay_error_mps=origin_error, inference_reexecuted=False)
        g = helper.geometric_trace(path, a['position'])
        bound = np.searchsorted(ct, t, side='right')-1; bi = np.clip(bound, 0, len(ct)-1)
        age = t-ct[bi]; valid = (bound >= 0) & (age >= -1e-8) & (age <= protocol['time']['maximum_causal_control_age_s'])
        cmode = np.asarray([c['mode'] for c in control]); mode = cmode[bi]
        cprogress = helper.finite_array(control, 'arc_progress_m')
        curved = (g['progress'] >= path.entry) & (g['progress'] <= path.entry+path.curve_length)
        mode_ok, bad_modes = helper.continuous_modes(mode, curved, valid)
        check('continuous_curved_drive_never_stop_turn', mode_ok, forbidden_modes=bad_modes)
        ordered, visits = helper.ordered_traversal(path, a, g)
        check('complete_original_ordered_circle_or_S_traversal', ordered, **visits)
        drive = valid & (mode == 'drive') & (t >= .1)
        if not drive.any(): raise ValueError('No original translating drive')
        first_drive = t[np.flatnonzero(drive)[0]]
        eligible = drive & (t >= first_drive+1.) & (cprogress[bi] <= path.length-.8)
        forward = np.sum(a['origin_world_velocity']*g['tangent'], axis=1)
        rolling = helper.rolling_forward(t, forward, eligible)
        check('every_full_one_second_actual_forward_motion', rolling['complete_windows'] > 0 and
              rolling['minimum_mean_mps'] >= .7*profile['desired_speed'],
              complete_sliding_windows=rolling['complete_windows'], minimum_mean_mps=rolling['minimum_mean_mps'],
              required_minimum_mps=.7*profile['desired_speed'], fixed_startup_exclusion_s=1., last_arc_exclusion_m=.8)
        directions_ok, directions = helper.turning_segments(path, a, g)
        check('all_signed_curvature_segments_actual_turn_direction', directions_ok, segments=directions)
        rv = helper.finite_array(control, 'reference_velocity_world', 3); cyaw = helper.finite_array(control, 'reference_yaw')
        speeds = np.linalg.norm(rv[bi, :2], axis=1)
        moving = drive & (speeds > .03); stable = moving & (t >= first_drive+1.) & (speeds >= .8*profile['desired_speed'])
        direction = np.zeros((len(t), 2)); np.divide(rv[bi, :2], speeds[:, None], out=direction, where=speeds[:, None] > 1e-12)
        world_com = np.einsum('nij,nj->ni', helper.rotations(a['quaternion_wxyz']), a['body_COM_velocity'])
        com_forward = np.sum(world_com[:, :2]*direction, axis=1)
        error_yaw = abs(wrapped(a['unwrapped_yaw']-cyaw[bi]))
        route_rms = float(np.sqrt(np.mean(g['nearest_xy_distance'][moving]**2))) if moving.any() else None
        route_max = float(g['nearest_xy_distance'][t >= .1].max())
        heading_max = float(error_yaw[moving].max()) if moving.any() else None
        speed_mae = float(np.mean(abs(com_forward[stable]-speeds[stable]))) if stable.any() else None
        speed_limit = max(.05, .25*float(np.mean(speeds[stable]))) if stable.any() else None
        check('native_exact_path_and_causal_heading_original_thresholds', route_rms is not None and
              route_rms <= .08 and route_max <= .20 and heading_max <= .20,
              drive_exact_path_rms_m=route_rms, maximum_exact_path_distance_m=route_max, maximum_drive_heading_rad=heading_max)
        check('native_real_COM_speed_original_threshold', speed_mae is not None and speed_mae <= speed_limit and
              stable.sum()*.005 >= .5, mean_absolute_error_mps=speed_mae, limit_mps=speed_limit)
        ff_errors = []; geometry_errors = []; budget = []; progress_errors = []
        fresh_drive = [c for c in control if c.get('controller_updated') is True and c['mode'] == 'drive']
        for c in fresh_drive:
            arc = float(c['arc_progress_m']); point, tangent, normal, heading, kappa = path.geometry(arc)
            ff_errors.append(abs(c['yaw_feedforward_radps']-kappa*c['planned_tangential_speed_mps']))
            budget.append(abs(c['yaw_feedforward_radps']))
            geometry_errors.append(max(abs(c['path_curvature_1pm']-kappa), abs(float(wrapped(c['reference_yaw']-heading))),
                                       float(np.max(abs(np.asarray(c['reference_xy'])-point[:2])))))
            j = native_index(t, c['feedback_time_s']); progress_errors.append(abs(arc-g['progress'][j]))
            if c.get('path_parameters_sha256') != path.sha256: raise ValueError('Actual drive path hash differs')
        check('original_kappa_v_feedforward_geometry_and_budget', bool(fresh_drive) and max(ff_errors) <= 1e-9 and
              max(geometry_errors) <= 1e-9 and max(budget) <= .64+1e-9 and max(progress_errors) <= .15,
              actual_fresh_drive_rows=len(fresh_drive), maximum_kappa_v_error_radps=max(ff_errors, default=None),
              maximum_geometry_error=max(geometry_errors, default=None), maximum_controller_progress_error_m=max(progress_errors, default=None))
        capture, parking_fresh = capture_evidence(control, a, path, ap)
        check('actual_capture_fresh_dwell_and_single_integral_reset', True, **capture)
        declaration = capture['first_hold_state_time_s']
        first_capture = next(c for c in control if c['mode'] == 'capture')
        capture_index = native_index(t, first_capture['feedback_time_s'])
        capture_parent_ok = (first_capture['arc_progress_m'] >= path.length-.15 and
            g['progress'][capture_index] >= path.length-.15 and
            np.linalg.norm(a['position'][capture_index, :2]-path.final_xyz[:2]) <= .12 and
            np.linalg.norm(a['origin_body_velocity'][capture_index, :2]) < .08 and
            abs(a['Euler_yaw_rate'][capture_index]) < .1 and
            abs(float(wrapped(a['unwrapped_yaw'][capture_index]-path.heading0))) < .2)
        check('capture_follows_original_full_progress_arrival_candidate', capture_parent_ok,
              actual_native_progress_m=float(g['progress'][capture_index]), required_progress_m=path.length-.15,
              original_arrival_candidate_limits={'xy_m':.12, 'origin_planar_mps':.08, 'Euler_yawrate_radps':.1, 'heading_rad':.2})
        target_keys = ('parking_target_world_xyz', 'parking_target_yaw_rad', 'parking_target_sha256',
                       'parking_capture_elapsed_s', 'parking_capture_world_time_s', 'parking_capture_state_time_s',
                       'arrival_capture_position_world', 'arrival_capture_yaw_rad')
        first_hold = next(c for c in control if c['mode'] == 'active_hold')
        for c, prefix in ((first_capture, 'parking_capture'), (first_hold, 'parking_hold_declared')):
            if (abs(c[prefix+'_world_time_s']-c['control_t_s']) > 1e-8 or
                abs(c[prefix+'_state_time_s']-c['feedback_time_s']) > 1e-8 or
                abs(c[prefix+'_elapsed_s']-c['elapsed_s']) > 1e-8 or
                abs(c[prefix+'_world_time_s']-c[prefix+'_state_time_s']-.005) > 1e-8):
                raise ValueError('First capture/hold source clock declaration differs')
        if abs(first_hold['completed_t_s']-first_hold['parking_hold_declared_elapsed_s']) > 1e-8:
            raise ValueError('Completed clock differs from first active hold declaration')
        hold_keys = ('parking_hold_declared_elapsed_s', 'parking_hold_declared_world_time_s', 'parking_hold_declared_state_time_s')
        parking_rows = [c for c in control if c['mode'] in ('capture', 'active_hold')]
        immutable = all(all(c[k] == first_capture[k] for k in target_keys) and
                        c['parking_reference_kind'] == 'prospective_fixed_path_endpoint' and
                        c['parking_target_world_xyz'] == target['position_world_xyz'] and
                        c['parking_target_yaw_rad'] == target['heading0_rad'] and
                        c['parking_target_sha256'] == canonical_sha(target) for c in parking_rows)
        immutable &= all(all(c[k] == first_hold[k] for k in hold_keys) for c in parking_rows if c['mode'] == 'active_hold')
        check('immutable_target_and_first_declaration_no_measured_pose_anchor', immutable,
              fixed_target=target, hold_declaration=declaration, arrival_is_diagnostic_only=True)
        window = fixed_first_window(t, a['position'], a['unwrapped_yaw'], a['origin_body_velocity'],
                                    a['body_omega'], a['Euler_yaw_rate'], declaration)
        mask = np.zeros(len(t), bool); mask[window.pop('native_indexes')] = True
        check('first_declared_five_seconds_original_drift_limits', window['complete'] and
              window['xy_drift_m'] <= .05 and window['yaw_drift_rad'] <= .1, **window,
              maximum_xy_drift_m=.05, maximum_yaw_drift_rad=.1, old_zero_command_condition_replaced=False,
              stopping_semantics='New fixed-endpoint bounded active hold; original zero-command receipt unchanged')
        check('new_stricter_native_peak_parking_velocity', window['complete'] and
              window['peak_origin_planar_speed_mps'] <= .08 and window['peak_Euler_yaw_rate_radps'] <= .1 and
              window['peak_body_wz_radps'] <= ap['maximum_native_body_wz_radps'], **{k: v for k, v in window.items() if k.startswith('peak_')},
              original_common_had_peak_gate=False, new_limits={'origin_planar_mps':.08, 'Euler_yawrate_radps':.1, 'body_wz_radps':.1})
        requested = helper.finite_array(telemetry, 'requested', 3); command = helper.finite_array(telemetry, 'command', 3)
        control_command = helper.finite_array(control, 'command_body', 3)
        estimate = helper.finite_array(control, 'estimated_teacher_command_body', 3)
        rebuilt = []; current = np.zeros(3)
        for r in requested:
            current = current+np.clip(r-current, -SLEW*.02, SLEW*.02); rebuilt.append(current.copy())
        rebuilt = np.asarray(rebuilt); parking_actor = np.isin(cmode, ['capture', 'active_hold'])
        limits = np.asarray(ap['command_limits_body'])
        cap_ok = np.all(abs(requested[parking_actor]) <= limits+1e-9) and np.all(np.linalg.norm(requested[parking_actor, :2], axis=1) <= .06+1e-9)
        target_cap_ok = all(np.linalg.norm(c['reference_velocity_world'][:2]) <= .025+1e-9 and
                            abs(c['parking_reference_yaw_rate_radps']) <= .07+1e-9 for c in parking_fresh)
        slew_error = float(np.max(abs(command-rebuilt))); estimate_error = float(np.max(abs(command-estimate)))
        check('actual_bounded_command_and_unchanged_teacher_slew', cap_ok and target_cap_ok and
              slew_error <= 1e-8 and estimate_error <= 1e-8 and np.max(abs(control_command-requested)) <= 1e-9,
              maximum_slew_replay_error=slew_error, maximum_estimated_vs_actual_command_error=estimate_error,
              requested_caps=limits, actual_nonzero_parking_commands=int(np.count_nonzero(np.linalg.norm(command[parking_actor], axis=1) > 1e-9)),
              action_zero_never_required=True)
        fresh = [c for c in control if c.get('controller_updated') is True]
        ttl_ok = all(type(c.get('controller_updated')) is bool and
            0 <= c['feedback_age_sim_s'] <= .30000001 and 0 <= c['feedback_age_wall_s'] <= .30000001 and
            c['fault'] in (None, 0, '') for c in control if c['elapsed_s'] >= 3.)
        ttl_ok &= all(0 <= c['previous_fresh_wall_receipt_gap_s'] <= .30000001 for c in fresh)
        cadence = np.asarray([c['feedback_time_s'] for c in fresh]); periods = np.diff(cadence)
        ttl_ok &= len(periods) > 0 and np.all(abs(periods-1/profile['feedback_hz']) <= 1e-8)
        check('actual_fresh_causality_dual_300ms_and_nonduplicate_cadence', ttl_ok,
              actual_fresh_math_rows=len(fresh), actual_feedback_rate_hz=1/float(np.mean(periods)) if len(periods) else None,
              maximum_sim_age_s=max(c['feedback_age_sim_s'] for c in control),
              maximum_wall_age_s=max(c['feedback_age_wall_s'] for c in control),
              original_threshold_s=.3, held_rows_count_as_updates=False)
        gains = dict(reference.DEFAULT_GAINS); gains.update(profile['gains']); gains['COM_offset'] = profile['base_com_offset']
        replay_pi = hold_pi_replay(control, a, ap, helper, gains)
        check('native_causal_active_PD_PI_and_antiwindup_replayed', replay_pi['passed'], **replay_pi)
        # The cached first declaration state is .005s before the controller
        # computes its new command. Preserve that phase, not future-bind it.
        hold_world = float(first_hold['parking_hold_declared_world_time_s'])
        phase_modes = np.where(t < hold_world-1e-8, np.isin(mode, ['capture', 'active_hold']), mode == 'active_hold')
        check('all_fixed_window_physics_has_causal_capture_or_active_reference', np.all(valid[mask]) and np.all(phase_modes[mask]),
              native_window_rows=int(mask.sum()), native_reference_age_max_s=float(age[mask].max()),
              declaration_cached_state_to_control_offset_s=hold_world-declaration,
              first_cached_state_reference_is_prior_capture=True, future_binding_used=False)
        metrics = dict(drive_exact_path_rms_m=route_rms, speed_mae_mps=speed_mae,
                       drive_maximum_heading_error_rad=heading_max, parking=window,
                       signed_turn_segments=directions, actual_final_progress_m=float(g['progress'][-1]),
                       geometry_formula_fully_independent=False)
        arrays = {**a, **{f'geometry_{k}': v for k, v in g.items()}, 'drive_mask':drive,
                  'first_active_hold_window_mask':mask, 'causal_control_index':bound,
                  'causal_control_age_s':age, 'actual_COM_reference_forward_mps':com_forward,
                  'actual_origin_tangent_forward_mps':forward, 'reference_speed_mps':speeds,
                  'causal_heading_error_rad':error_yaw, 'rolling_forward_start_time_s':rolling['time'],
                  'rolling_forward_mean_mps':rolling['means']}
        check('complete_parseable_active_hold_evidence', True)
    except Exception as error:
        check('complete_parseable_active_hold_evidence', None, error=repr(error), incomplete_evidence_never_passes=True)
    status = status_from_checks(checks)
    result = dict(schema='independent_truth_teacher_active_endpoint_hold/v2', run=str(run), status=status,
        scope='Truth-feedback continuous curve with prospective bounded active endpoint hold; different stopping semantics from v1',
        navigation_ground_truth_used=True, SLAM_navigation_verified=False, SCAN_verified=False,
        full_multifloor_verified=False, Isaac_new_controller_verified=False, real_robot_verified=False,
        actor_input_dimensions=247, privileged_actor_observation_dimensions=232, score=None,
        old_zero_command_parking_verified=False, old_receipts_modified=False,
        ancestors=ancestors, checks=checks, metrics=metrics, verified_input_source_sha256=verified,
        analyzer_sha256=sha(__file__), failed_checks=[k for k, v in checks.items() if v['status'] == 'failed'],
        unverified_checks=[k for k, v in checks.items() if v['status'] == 'unverified'],
        limitations=['Original common/curve parking statuses are preserved; active hold is a new contract, not a correction of their failure.',
                     'Capture dwell is evaluated on original fresh measurements with entry/hold hysteresis; no assertion of between-sample capture eligibility.',
                     'StaticPath formulas are shared with the hash-pinned original controller reference; native measurement/time/velocity reconstruction is independent.',
                     'A single active hold result does not establish global frequency/gain optimality, real sensor navigation or hardware safety.'])
    if write:
        if arrays:
            with array_output.open('xb') as stream: np.savez_compressed(stream, **arrays)
            result['independent_arrays'] = dict(path=str(array_output), sha256=sha(array_output))
        with output.open('x') as stream: json.dump(serial(result), stream, indent=2, ensure_ascii=False, allow_nan=False)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args(); result = evaluate(args.run)
    print(json.dumps(dict(status=result['status'], failed_checks=result['failed_checks'],
                         unverified_checks=result['unverified_checks'], receipt=str(args.run/OUTPUT))))
