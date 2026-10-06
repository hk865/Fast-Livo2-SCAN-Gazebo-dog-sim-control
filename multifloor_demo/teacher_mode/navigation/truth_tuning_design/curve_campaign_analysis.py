#!/usr/bin/env python3
"""Read-only inputs, append-only 14-run curve campaign evidence.

Default: readiness only. --final requires every prospective index 0..13,
every original receipt, and a new output directory. No Controller, ROS,
Gazebo, torch, signals, or writes to an ancestor/run are used. Geometry
formulas and the original fifteen gates are hash-pinned archived helpers;
native measurements are reconstructed independently for diagnostic traces.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAMPAIGN = ROOT / 'test_results/curvature_campaign_20261005'
EXPECTED = tuple(range(14))
CURVE_CHECKS = (
    'untouched_original_flat_common_all_passed',
    'all_frozen_source_inputs_model_CPU_and_scope',
    'immutable_analytic_path_parameters',
    'original_finite_247_by_12_actor_evidence',
    'every_native_200Hz_step_and_original_actor_control_coverage',
    'body_origin_COM_and_frame_semantics_recomputed',
    'continuous_curved_drive_never_stop_turn',
    'complete_original_ordered_circle_or_S_traversal',
    'every_full_one_second_actual_forward_motion',
    'all_signed_curvature_segments_actual_turn_direction',
    'native_exact_path_and_causal_heading_original_thresholds',
    'native_real_COM_speed_original_threshold',
    'original_kappa_v_feedforward_geometry_and_budget',
    'original_first_five_second_parking_unchanged',
    'complete_parseable_curve_evidence',
)
DYNAMIC_CHECKS = CURVE_CHECKS[6:13]
SHA_CACHE = {}


def sha(path):
    path = Path(path).resolve()
    stat = path.stat()
    key = (str(path), stat.st_size, stat.st_mtime_ns)
    if key not in SHA_CACHE:
        d = hashlib.sha256()
        with path.open('rb') as f:
            for block in iter(lambda: f.read(1024 * 1024), b''):
                d.update(block)
        SHA_CACHE[key] = d.hexdigest()
    return SHA_CACHE[key]


def load(path):
    return json.loads(Path(path).read_text())


def rows(path):
    with Path(path).open() as f:
        return [json.loads(line) for line in f if line.strip()]


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serial(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_new(path, value):
    with Path(path).open('x') as f:
        json.dump(serial(value), f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')


def module(path, prefix):
    name = prefix + '_' + hashlib.sha256(str(path).encode()).hexdigest()[:16]
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(m)
    finally:
        sys.dont_write_bytecode = previous
    return m


def verify(path, expected, evidence):
    actual = sha(path)
    evidence[str(Path(path).resolve())] = actual
    if actual != expected:
        raise ValueError('Original SHA mismatch: ' + str(path))
    return actual


def index_campaign(plan, results):
    planned = {int(p['index']): p for p in plan['profiles']}
    if len(plan['profiles']) != 14 or tuple(sorted(planned)) != EXPECTED:
        raise ValueError('Prospective plan must contain exactly indices 0..13')
    indexed = {}; run_paths = set()
    for r in results:
        i = int(r['index'])
        if i in indexed or i not in planned:
            raise ValueError('Duplicate or unplanned result index: ' + str(i))
        if r['profile']['sha256'] != planned[i]['sha256']:
            raise ValueError('Result profile differs from prospective index: ' + str(i))
        run_path = str(Path(r['run']).resolve())
        if run_path in run_paths:
            raise ValueError('One actual run reused by different prospective cases')
        run_paths.add(run_path)
        indexed[i] = r
    return planned, indexed


def readiness(campaign):
    plan = load(campaign / 'phase_curve_v1_plan.json')
    planned, indexed = index_campaign(plan, rows(campaign / 'phase_curve_v1_results.jsonl'))
    missing = [i for i in EXPECTED if i not in indexed]
    receipt_missing = [i for i in indexed if not (Path(indexed[i]['run']) / 'summary_curve_independent.json').is_file()]
    return plan, planned, indexed, {
        'planned': 14, 'recorded_indices': sorted(indexed), 'missing_indices': missing,
        'missing_original_receipts': receipt_missing, 'ready': not missing and not receipt_missing,
        'final_written': False,
    }


def rotations(q):
    q = np.asarray(q, float)
    if q.shape[1:] != (4,) or not np.isfinite(q).all() or np.any(abs(np.sum(q*q, axis=1)-1) > .02):
        raise ValueError('Invalid original quaternion')
    w, x, y, z = (q / np.linalg.norm(q, axis=1)[:, None]).T
    return np.stack((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
        2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
        2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=1).reshape(-1, 3, 3)


def array(rs, key, width=None):
    a = np.asarray([r[key] for r in rs], float)
    if not np.isfinite(a).all() or (width is not None and a.shape != (len(rs), width)):
        raise ValueError('Missing/nonfinite shape: ' + key)
    return a


def gap_rate(control):
    fresh = [r for r in control if r.get('controller_updated') is True]
    t = array(fresh, 'feedback_time_s')
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError('No distinct original mathematical updates')
    gaps = np.diff(t)
    return {'fresh_math_updates': len(fresh), 'all_control_rows': len(control),
        'initialization_hold_parking_rows': len(control)-len(fresh),
        'actual_math_hz': (len(t)-1)/(t[-1]-t[0]),
        'minimum_update_gap_s': gaps.min(), 'maximum_update_gap_s': gaps.max(),
        'nonincreasing_original_feedback_stamps': int(np.sum(gaps <= 0)),
        'selector': 'controller_updated is exactly True; not kind or Teacher-frame count'}


def pi_and_slew(control, telemetry, profile, native_t, com, omega, core):
    """Replay inner filter/PI conditional integration using causal actual state.

    Logged geometric target is the reference. Reconstruct physical input,
    filter, conditional I, axis cap, downstream slew and actual command.
    Nontranslation modes never become performance-ranking samples.
    """
    if len(control) != len(telemetry):
        raise ValueError('Control/Teacher row count mismatch')
    gains = core.DEFAULT_GAINS | profile.get('gains', {})
    kp = np.array([gains['velocity_kp_x'], gains['velocity_kp_y'], gains['rate_kp']])
    ki = np.array([gains['velocity_ki_x'], gains['velocity_ki_y'], gains['rate_ki']])
    limits = np.asarray(profile['command_limits'], float)
    integral = np.zeros(3); estimated = np.zeros(3); filtered = None
    axis_count = np.zeros(3, int); slew_count = np.zeros(3, int); saturated = np.zeros(3, int)
    max_errors = dict(filtered_state=0., measured_PI_error=0., P=0., I=0., blocked_axis=0.,
        blocked_slew=0., raw=0., requested=0., estimated_teacher=0., actual_teacher=0.,
        telemetry_requested=0., actual_command_slew_excess=0., command_axis_limit_excess=0.,
        nondrive_preserved_I=0., nondrive_zero_reference=0.)
    previous_actual = np.zeros(3)
    drive_fresh = 0
    def err(name, expected, actual):
        max_errors[name] = max(max_errors[name], float(np.max(abs(np.asarray(expected)-np.asarray(actual)))))
    for r, a in zip(control, telemetry):
        requested = np.asarray(r['command_body'], float)
        if r.get('controller_updated') is True:
            j = int(np.argmin(abs(native_t-float(r['feedback_time_s']))))
            if abs(native_t[j]-r['feedback_time_s']) > 1e-8:
                raise ValueError('PI feedback has no exact native state')
            actual = np.r_[com[j, :2], omega[j, 2]]
            dt = float(r['header_dt_s'])
            beta = 1. if filtered is None else dt/(.1+dt)
            filtered = actual.copy() if filtered is None else filtered+beta*(actual-filtered)
            p = r.get('velocity_PI')
            if p is not None:
                err('filtered_state', filtered, p['filtered_actual_body'])
            if r['mode'] == 'drive':
                drive_fresh += 1
                target = np.asarray(r['v_reference_body'], float)
                ev = target-filtered
                err('measured_PI_error', ev, p['error']); err('P', kp*ev, p['P'])
                candidate = np.clip(integral+float(r['active_integral_dt_s'])*ev, -.5, .5)
                raw = target+kp*ev+ki*candidate
                clipped = np.clip(raw, -limits, limits)
                blocked_axis = ev*(raw-clipped) > 0
                candidate = np.where(blocked_axis, integral, candidate)
                downstream = estimated+np.clip(clipped-estimated, -np.array([.6,.6,.8])*.02, np.array([.6,.6,.8])*.02)
                blocked_slew = ev*(raw-downstream) > 1e-9
                candidate = np.where(blocked_slew, integral, candidate)
                integral = candidate
                raw = target+kp*ev+ki*integral
                clipped = np.clip(raw, -limits, limits); clipped[0] = max(0., clipped[0])
                planar = np.linalg.norm(clipped[:2])
                if planar > limits[0]:
                    clipped[:2] *= limits[0]/planar
                err('I', ki*integral, p['I']); err('raw', raw, r['raw_command_body'])
                err('requested', clipped, requested)
                err('blocked_axis', blocked_axis.astype(int), np.asarray(p['blocked_axis'], int))
                err('blocked_slew', blocked_slew.astype(int), np.asarray(p['blocked_slew'], int))
                axis_count += blocked_axis; slew_count += blocked_slew
                saturated += np.asarray(r['saturated'], int)
            elif abs(float(r.get('active_integral_dt_s', 0))) > 1e-10:
                raise ValueError('Nontranslation mode integrated with nonzero dt')
            else:
                if p is not None:
                    err('nondrive_preserved_I',ki*integral,p['I'])
                for key in ('raw_command_body','command_body','v_reference_body','reference_velocity_world'):
                    err('nondrive_zero_reference',np.zeros(3),r[key])
        estimated += np.clip(requested-estimated, -np.array([.6,.6,.8])*.02, np.array([.6,.6,.8])*.02)
        err('estimated_teacher', estimated, r['estimated_teacher_command_body'])
        err('actual_teacher', estimated, a['command']); err('telemetry_requested', requested, a['requested'])
        actual_command = np.asarray(a['command'], float)
        max_errors['actual_command_slew_excess'] = max(max_errors['actual_command_slew_excess'],
            float(np.max(abs(actual_command-previous_actual)-np.array([.6,.6,.8])*.02)))
        max_errors['command_axis_limit_excess'] = max(max_errors['command_axis_limit_excess'],
            float(np.max(abs(actual_command)-limits)))
        previous_actual = actual_command
    return {'reconstruction_max_errors': max_errors, 'parity_passed': max(max_errors.values()) <= 1e-8,
        'drive_fresh_updates': drive_fresh, 'axis_blocked_update_counts_xyz': axis_count,
        'downstream_slew_blocked_update_counts_xyz': slew_count,
        'saturated_drive_updates_xyz': saturated, 'command_limits_body': limits,
        'downstream_slew_per_s': [.6,.6,.8],
        'antiwindup_axis_pressure_observed': bool(axis_count.any()),
        'antiwindup_slew_pressure_observed': bool(slew_count.any()),
        'geometric_target_reconstructed':False,
        'geometric_target_scope':'Frozen original v_reference_body; only actual inner feedback/filter/PI/conditional integration and resulting command independently replayed',
        'not_a_motor_torque_limiter': True}


def heading_peak(mask, error, t, progress, control, bound):
    if not mask.any():
        return None
    selected = np.flatnonzero(mask)
    j = int(selected[np.argmax(abs(error[selected]))])
    ci = int(bound[j]); c = control[ci]
    context = [r for r in control if r.get('controller_updated') is True and
        abs(float(r['control_t_s'])-t[j]) <= .3]
    fields = ('control_t_s','feedback_time_s','elapsed_s','header_dt_s','mode','arc_progress_m',
        'path_curvature_1pm','planned_tangential_speed_mps','yaw_feedforward_radps',
        'reference_yaw','error_yaw','measured_yaw_rate','yaw_PD','v_reference_body',
        'command_body','raw_command_body','estimated_teacher_command_body','velocity_PI','saturated')
    return {'absolute_error_rad':float(abs(error[j])), 'signed_error_rad':float(error[j]),
        'native_physical_time_s':float(t[j]),'native_arc_progress_m':float(progress[j]),
        'causal_control_index':ci,'control_age_s':float(t[j]-c['control_t_s']),
        'causal_control_row':{k:c.get(k) for k in fields},
        'fixed_plus_minus_0p3s_fresh_control_context':[{k:r.get(k) for k in fields} for r in context],
        'source':'Independent original 200Hz native yaw with latest causal original controller reference; not a chosen quiet window'}


def planning_chain(profile, verified):
    boundary = load(profile['boundary_receipt_path'])
    groups = boundary if isinstance(boundary,list) else boundary.get('groups',[])
    details = []
    for group in groups:
        sources = []
        for original in group.get('source_receipts',[]):
            verify(original['source_path'],original['source_sha256'],verified)
            actual_paths={}
            for name,digest in original.get('original_input_hashes',{}).items():
                source_path=Path(original['run'])/name
                if name=='protocol.json':
                    # Radius receipt uses a logical protocol key; its
                    # prospective executed copy is sources/curvature, not
                    # a fabricated root-level protocol.json.
                    source_path=Path(original['run'])/'sources/curvature/protocol.json'
                verify(source_path,digest,verified)
                actual_paths[name]=str(source_path)
            sources.append({'path':original['source_path'],'sha256':original['source_sha256'],
                'original_status':original.get('plant_status'),'repetition':original['repetition'],
                'raw_input_count':len(original.get('original_input_hashes',{})),
                'logical_input_key_to_original_archive_path':actual_paths})
        details.append({'speed_mps':group.get('desired_speed_mps'),'sign':group.get('direction_sign'),
            'original_status':group.get('status'),'verified_repeat_sources':sources,
            'scope':'Planning margin evidence only; does not itself pass curve tracking'})
    return {'boundary_receipt_path':profile['boundary_receipt_path'],
        'boundary_receipt_sha256':profile['boundary_receipt_sha256'],'groups':details,
        'old_profile_feasibility_metadata_untouched':profile.get('feasibility')}


def inspect_run(run, prospective, freeze, recheck=True):
    run = Path(run).resolve(); verified = {}
    receipt_path = run/'summary_curve_independent.json'
    receipt_sha = sha(receipt_path); receipt = load(receipt_path)
    original = {'path': str(receipt_path), 'sha256': receipt_sha, 'status': receipt.get('status')}
    profile = load(run/'truth_profile.json')
    verify(run/'truth_profile.json', prospective['sha256'], verified)
    verify(prospective['path'], prospective['sha256'], verified)
    if receipt.get('schema') != 'independent_truth_teacher_continuous_curve_tracking/v1':
        raise ValueError('Original curve schema differs')
    for p, digest in receipt['verified_input_source_sha256'].items():
        verify(p, digest, verified)
    verify(receipt['independent_arrays']['path'], receipt['independent_arrays']['sha256'], verified)
    common_path = run/'summary_truth_pid.json'; common = load(common_path)
    verify(common_path, receipt['ancestor_common']['sha256'], verified)
    for p, digest in common['input_source_sha256'].items():
        verify(p, digest, verified)
    verify(common['independent_arrays']['path'], common['independent_arrays']['sha256'], verified)
    manifest = load(run/'source_manifest.json'); runtime = load(run/'runtime_manifest.json')
    verify(run/'source_manifest.json', runtime['source_manifest_sha256'], verified)
    verify(run/'truth_profile.json', runtime['truth_profile_sha256'], verified)
    for name, digest in manifest.items():
        verify(run/'sources'/name, digest if isinstance(digest, str) else digest['sha256'], verified)
    # Prospective original-address hashes are linked to immutable execution
    # copies. Later live-source changes do not invalidate legitimate old runs.
    archive_links = {}
    if set(profile['frozen_source_hashes']) != set(freeze['sources']):
        raise ValueError('Prospective frozen source-key set differs')
    for original_path, digest in profile['frozen_source_hashes'].items():
        if freeze['sources'].get(original_path) != digest:
            raise ValueError('Profile source differs from prospective source freeze')
        matches = [name for name, archived_digest in manifest.items()
            if archived_digest == digest and Path(name).name == Path(original_path).name]
        archive_links[original_path] = {'sha256': digest,
            'executed_archive_paths': [str(run/'sources'/name) for name in matches]}
        # README.md is frozen prospective documentation but was not copied
        # by the runner. It is identified as such, never called executed code.
        optional_capture=(Path(original_path).name=='capture.py' and
            not any(p.get('role')=='capture' for p in runtime.get('owned_processes',[])) and
            prospective.get('camera') is False)
        if not matches and Path(original_path).name != 'README.md' and not optional_capture:
            raise ValueError('Frozen execution source missing archive copy: '+original_path)
        if not matches:
            archive_links[original_path]['scope'] = ('Prospective optional camera recorder not launched in this case; no executed copy' if optional_capture else
                'prospective documentation only; no executed copy')
    verify(profile['boundary_receipt_path'], profile['boundary_receipt_sha256'], verified)
    planning = planning_chain(profile,verified)
    if receipt['analyzer_sha256'] != manifest['truth/curve_receipt.py']:
        raise ValueError('Original analyzer SHA differs from executed archive')
    core = module(run/'sources/truth/core.py', '_aggregate_static_path')
    path = core.StaticPath(profile['path'])
    if path.sha256 != profile['path_parameters_sha256']:
        raise ValueError('Immutable analytic path hash differs')
    check_names = tuple(receipt.get('checks', {}))
    gates_complete = set(check_names) == set(CURVE_CHECKS)
    rerun = module(run/'sources/truth/curve_receipt.py', '_aggregate_frozen_curve_gate').evaluate(run, write=False) if recheck else receipt
    gate_parity = (set(rerun['checks']) == set(receipt['checks']) and
        all(rerun['checks'][k]['status'] == v['status'] for k, v in receipt['checks'].items()) and
        rerun['status'] == receipt['status'])
    native = [r for r in rows(run/'actuator.jsonl') if r.get('kind') == 'physics_step']
    control = rows(run/'control.jsonl'); actor = rows(run/'telemetry.jsonl')
    t = array(native,'t')-array(native,'dt'); position = array(native,'position',3)
    q = array(native,'quaternion_wxyz',4); com = array(native,'body_lin_vel_com',3)
    omega = array(native,'body_ang_vel',3); R = rotations(q)
    origin_body = com-np.cross(omega, np.asarray(profile['base_com_offset']))
    origin_world = np.einsum('nij,nj->ni',R,origin_body)
    com_world = np.einsum('nij,nj->ni',R,com)
    roll = np.arctan2(R[:,2,1],R[:,2,2]); pitch = np.arcsin(np.clip(-R[:,2,0],-1,1))
    yawdot = (np.sin(roll)*omega[:,1]+np.cos(roll)*omega[:,2])/np.cos(pitch)
    yaw = np.unwrap(np.arctan2(R[:,1,0],R[:,0,0]))
    with np.load(receipt['independent_arrays']['path'],allow_pickle=False) as stored:
        a = {k: stored[k].copy() for k in stored.files}
    parity = {}
    for key, rebuilt in {'t':t,'position':position,'body_COM_velocity':com,'body_omega':omega,
            'origin_body_velocity':origin_body,'origin_world_velocity':origin_world,
            'Euler_yaw_rate':yawdot,'unwrapped_yaw':yaw}.items():
        if rebuilt.shape != a[key].shape:
            raise ValueError('Native independent-array shape differs: '+key)
        parity[key] = float(np.max(abs(rebuilt-a[key])))
    parity['native_recorded_origin_body'] = float(np.max(abs(origin_body-array(native,'body_lin_vel_origin',3))))
    if max(parity.values()) > 1e-9:
        raise ValueError('Original raw native measurement reconstruction differs')
    ct = array(control,'control_t_s'); bound = np.searchsorted(ct,t,side='right')-1
    bi = np.clip(bound,0,len(ct)-1); age = t-ct[bi]
    mode = np.array([r['mode'] for r in control])[bi]
    protocol = load(run/'sources/truth/protocol.json')
    valid = (bound>=0)&(age>=-1e-8)&(age<=protocol['time']['maximum_causal_control_age_s']+1e-8)
    drive = valid&(mode=='drive')&(t>=.1)
    progress = a['geometry_progress']; tangent = a['geometry_tangent']
    native_forward = np.sum(origin_world*tangent,axis=1)
    native_com_tangent = np.sum(com_world*tangent,axis=1)
    ff = np.array([r.get('yaw_feedforward_radps',0.) for r in control])[bi]
    planned_speed = np.array([r.get('planned_tangential_speed_mps',0.) for r in control])[bi]
    reference_heading = array(control,'reference_yaw')[bi]
    heading_error = np.arctan2(np.sin(yaw-reference_heading),np.cos(yaw-reference_heading))
    cross = a['geometry_signed_cross']
    curve = drive&(progress>=path.entry)&(progress<=path.entry+path.curve_length)
    first = t[np.flatnonzero(drive)[0]] if drive.any() else math.inf
    stable = drive&(t>=first+1)&(planned_speed>=.8*profile['desired_speed'])&(progress<=path.length-.8)
    rate = gap_rate(control)
    pi = pi_and_slew(control,actor,profile,t,com,omega,core)
    fresh_drive=[r for r in control if r.get('controller_updated') is True and r['mode']=='drive']
    fresh_t=array(fresh_drive,'feedback_time_s'); fresh_ff=array(fresh_drive,'yaw_feedforward_radps')
    ff_delta=np.diff(fresh_ff); ff_rate=ff_delta/np.diff(fresh_t)
    ff_peak=int(np.argmax(abs(ff_rate))) if len(ff_rate) else None
    dynamics={'curved_native_rows':int(curve.sum()),
        'arc_region_only_heading_RMS_rad':float(np.sqrt(np.mean(heading_error[curve]**2))) if curve.any() else None,
        'arc_region_only_heading_max_rad':float(np.max(abs(heading_error[curve]))) if curve.any() else None,
        'arc_region_only_signed_cross_RMS_m':float(np.sqrt(np.mean(cross[curve]**2))) if curve.any() else None,
        'arc_region_only_origin_tangent_speed_MAE_mps':float(np.mean(abs(native_forward[curve]-planned_speed[curve]))) if curve.any() else None,
        'arc_region_only_COM_tangent_speed_MAE_mps':float(np.mean(abs(native_com_tangent[curve]-planned_speed[curve]))) if curve.any() else None,
        'arc_region_native_progress_bounds_m':[path.entry,path.entry+path.curve_length],
        'curve_only_heading_peak':heading_peak(curve,heading_error,t,progress,control,bound),
        'all_drive_heading_peak':heading_peak(drive,heading_error,t,progress,control,bound),
        'maximum_recorded_kappa_v_input_rate_radps2':float(np.max(abs(ff_rate))) if len(ff_rate) else None,
        'maximum_feedforward_change_context':None if ff_peak is None else {
            'before':fresh_drive[ff_peak],'after':fresh_drive[ff_peak+1],
            'signed_FF_change_radps':float(ff_delta[ff_peak]),'feedback_interval_s':float(fresh_t[ff_peak+1]-fresh_t[ff_peak]),
            'signed_FF_change_rate_radps2':float(ff_rate[ff_peak]),
            'downstream_wz_slew_radps2':.8,
            'inference':'Reference join/slew temporal association is diagnostic; no causal A/B has changed curvature or speed'},
    }
    metric = {'feedback_rate': rate, 'native_rows':len(native), 'native_physical_phase':'t-dt, dt=.005',
        'drive_native_rows':int(drive.sum()),'curved_drive_native_rows':int(curve.sum()),
        'native_reconstruction_max_errors':parity,
        'mean_actual_origin_tangent_speed_mps':float(np.mean(native_forward[stable])) if stable.any() else None,
        'mean_actual_COM_tangent_speed_mps':float(np.mean(native_com_tangent[stable])) if stable.any() else None,
        'tangent_origin_speed_MAE_mps':float(np.mean(abs(native_forward[stable]-planned_speed[stable]))) if stable.any() else None,
        'drive_signed_cross_RMS_m':float(np.sqrt(np.mean(cross[drive]**2))) if drive.any() else None,
        'drive_maximum_abs_cross_m':float(np.max(abs(cross[drive]))) if drive.any() else None,
        'drive_maximum_abs_heading_rad':float(np.max(abs(heading_error[drive]))) if drive.any() else None,
        'drive_heading_diagnostic_mask':'All causal drive including low-speed endpoint; not substituted for original moving-reference threshold gate',
        'curved_Euler_yawrate_vs_kappa_v_MAE_radps':float(np.mean(abs(yawdot[curve]-ff[curve]))) if curve.any() else None,
        'curved_actual_accumulated_body_yaw_rad':float(yaw[np.flatnonzero(curve)[-1]]-yaw[np.flatnonzero(curve)[0]]) if curve.any() else None,
        'PI_and_actual_Teacher_slew': pi,
        'actual_arc_region_diagnostics':dynamics,
        'yawrate_vs_FF_is_diagnostic_not_a_gate':True,
        'original_curve_metrics':receipt['metrics'],
        'original_common_safety':common['checks'].get('as_recorded_physical_safety',{}),
        'original_common_parking':common['checks'].get('fixed_final_goal_parking',{}),
    }
    # Extra diagnostics cannot upgrade an original failed/unverified receipt.
    parity_ok = gates_complete and gate_parity and pi['parity_passed'] and abs(rate['actual_math_hz']-profile['feedback_hz'])<1e-6
    status = receipt['status'] if parity_ok else ('failed' if receipt['status']=='failed' else 'unverified')
    info = {'index':prospective['index'],'run':str(run),'status':status,'original_receipt':original,
        'profile_path':str(run/'truth_profile.json'),'profile_sha256':prospective['sha256'],
        'path_parameters':profile['path'],'path_sha256':path.sha256,'feedback_hz_selected':profile['feedback_hz'],
        'repetition':profile.get('repetition',1),'desired_speed_mps':profile['desired_speed'],
        'original_15_checks':{k:v['status'] for k,v in receipt['checks'].items()},
        'frozen_gate_readonly_recheck':{'performed':recheck,'status':rerun['status'],'all_statuses_match':gate_parity,
            'archive_analyzer_sha256':manifest['truth/curve_receipt.py']},
        'all_15_original_checks_present':gates_complete,'ancestor_common':{
            'path':str(common_path),'sha256':sha(common_path),'status':common['status'],
            'checks':{k:v['status'] for k,v in common['checks'].items()}},
        'verified_input_source_sha256':verified,'prospective_to_executed_archive_links':archive_links,
        'planning_evidence_chain':planning,
        'original_dynamic_curve_checks':{k:receipt['checks'].get(k,{}).get('status','unverified') for k in DYNAMIC_CHECKS},
        'runtime':runtime,'metrics':metric,'score':None,
        'old_original_receipt_unchanged':sha(receipt_path)==receipt_sha,
        'geometry_method':'Native measurements reconstructed from original 200Hz logs; geometric projection is original hash-verified independent receipt trace; StaticPath formulas shared with archived controller, not a completely independent geometric derivation'}
    traces = {'t':t,'position':position,'progress':progress,'path_length':path.length,
        'cross':cross,'heading_error':heading_error,'origin_tangent_speed':native_forward,
        'COM_tangent_speed':native_com_tangent,'planned_speed':planned_speed,'yawrate':yawdot,
        'yaw_ff':ff,'drive':drive,'curve':curve,'path_sample':path.position(np.linspace(0,path.length,800))}
    return info,traces


def smooth(t,y,width=1.):
    # Fixed centered one-second arithmetic mean for display only. Original
    # unfiltered samples and all gate metrics remain in the arrays/receipts.
    left = np.searchsorted(t,t-width/2); right = np.searchsorted(t,t+width/2,side='right')
    total = np.r_[0.,np.cumsum(y)]
    return (total[right]-total[left])/np.maximum(1,right-left)


def plots(output,records,traces):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.25})
    figure,axes = plt.subplots(4,4,figsize=(14,13),constrained_layout=True)
    for r,ax in zip(records,axes.flat):
        tr = traces[r['index']]; p = r['path_parameters']
        ax.plot(tr['path_sample'][:,0],tr['path_sample'][:,1],'k--',lw=1,label='Frozen analytic route')
        ax.plot(tr['position'][:,0],tr['position'][:,1],lw=1,label='Actual native body origin')
        case = 'R='+str(p['radius_m']) if p['kind']=='circle' else 'S L='+str(p['curve_length_m'])+' A='+str(p['heading_amplitude_rad'])
        ax.set_title(f"{r['index']:02d} {case}, sign {p['direction_sign']:+d}, {r['feedback_hz_selected']}Hz r{r['repetition']}\n{r['status'].upper()}")
        ax.set(xlabel='World x (m)',ylabel='World y (m)',aspect='equal')
    for ax in axes.flat[len(records):]:ax.axis('off')
    axes.flat[-2].legend(*axes.flat[0].get_legend_handles_labels(),loc='center',frameon=False)
    axes.flat[-1].text(.04,.65,'Actual simulation truth feedback\nNo SLAM / SCAN / hardware claim\n14 prospective cases; old failures retained\nReference geometry shares StaticPath formulas',transform=axes.flat[-1].transAxes)
    figure.suptitle('Continuous circles and S curves: original actual 200Hz trajectories',fontsize=15)
    figure.savefig(output/'actual_xy_all14.png',dpi=170);plt.close(figure)
    colors = {10:'#1565c0',25:'#ef6c00',50:'#2e7d32'}
    srecords = [r for r in records if r['path_parameters']['kind']=='s_curve']
    lengths = sorted({r['path_parameters']['curve_length_m'] for r in srecords},reverse=True)
    fig,axs = plt.subplots(2,3,figsize=(14,8),constrained_layout=True)
    for row,L in enumerate(lengths):
        selected = sorted([r for r in srecords if r['path_parameters']['curve_length_m']==L],key=lambda r:r['feedback_hz_selected'])
        for r in selected:
            tr=traces[r['index']];m=tr['curve'];x=tr['progress'][m];hz=r['feedback_hz_selected'];color=colors[hz]
            axs[row,0].plot(x,tr['cross'][m],color=color,label=f'{hz}Hz actual math')
            axs[row,1].plot(x,abs(tr['heading_error'][m]),color=color)
            axs[row,2].plot(x,smooth(tr['t'],tr['origin_tangent_speed'])[m],color=color,label=f'{hz}Hz actual origin tangent, 1s mean')
            axs[row,2].plot(x,tr['planned_speed'][m],color=color,ls='--',alpha=.65)
        for col,ax in enumerate(axs[row]):
            ax.set(xlabel='Actual admitted native arc progress (m)',ylabel=['Signed cross error (m)','Absolute heading error (rad)','Tangent speed (m/s)'][col])
            ax.set_title(f"S L={L:g}m, A={selected[0]['path_parameters']['heading_amplitude_rad']:g}rad")
        axs[row,1].axhline(.2,color='k',ls=':',lw=.8)
        axs[row,0].legend(loc='upper right');axs[row,2].legend(loc='lower left',fontsize=8)
    fig.suptitle('S frequency comparison: actual curved arc only; startup/straight maxima excluded',fontsize=14)
    fig.savefig(output/'S_frequency_errors_velocity.png',dpi=170);plt.close(fig)
    fig,axs=plt.subplots(2,3,figsize=(14,8),constrained_layout=True)
    categories=[('circle',.6,1),('circle',.6,-1),('circle',1.2,None),('s_curve',8.,None),('s_curve',6.,None)]
    for ax,(kind,size,sign) in zip(axs.flat,categories):
        for r in records:
            p=r['path_parameters'];measure=p.get('radius_m',p.get('curve_length_m'))
            if p['kind']!=kind or measure!=size or (sign is not None and p['direction_sign']!=sign):continue
            tr=traces[r['index']];m=tr['curve'];x=tr['progress'][m]
            label=f"{r['feedback_hz_selected']}Hz sign {p['direction_sign']:+d} r{r['repetition']}"
            line,=ax.plot(x,smooth(tr['t'],tr['yawrate'])[m],lw=1,label=label)
            ax.plot(x,tr['yaw_ff'][m],lw=.7,ls='--',color=line.get_color(),alpha=.8)
        ax.set(xlabel='Actual native arc progress (m)',ylabel='Euler yaw rate / FF (rad/s)')
        ax.set_title(('Circle R=' if kind=='circle' else 'S L=')+str(size)+'; actual 1s mean / dashed kappa*v')
        ax.legend(fontsize=7,loc='best')
    ax=axs.flat[-1];indices=np.arange(len(records))
    blocked=[sum(r['metrics']['PI_and_actual_Teacher_slew']['downstream_slew_blocked_update_counts_xyz']) for r in records]
    saturated=[sum(r['metrics']['PI_and_actual_Teacher_slew']['saturated_drive_updates_xyz']) for r in records]
    ax.bar(indices-.2,blocked,.4,label='PI downstream-slew blocked axes')
    ax.bar(indices+.2,saturated,.4,label='Requested command saturated axes')
    ax.set(xlabel='Prospective case index',ylabel='Axis-update count',title='Recorded limiting pressure, not parking score')
    ax.set_xticks(indices);ax.legend(fontsize=7)
    fig.suptitle('Actual Euler yaw response vs curvature feedforward; feedback corrections remain active',fontsize=14)
    fig.savefig(output/'actual_Euler_rate_and_PI.png',dpi=170);plt.close(fig)


def report(output,records,receipt):
    lines=['# 14轮实际连续曲率试验汇总','',
        f"事前索引0–13完整，原新增15门共{receipt['original_curve_checks_passed']}/210通过。实际曲线完整收据通过{receipt['original_curve_passed_runs']}/14；其中连续drive动态7门通过{receipt['original_dynamic_drive_checks_passed']}/98，动态全通过的run为{receipt['original_dynamic_drive_passed_runs']}/14。本汇总来源/重建检查通过{sum(r['status']=='passed' for r in records)}/14。所有旧失败和原共同验收保留，score恒为null。",'',
        '这些试验使用实际Gazebo原生反馈控制Teacher，是仿真真值控制基准，不是SLAM导航。Actor仍有232维特权观测，3维命令和12维上一动作；模型未重新训练。', '',
        '|索引|路径|Hz选定/实际数学更新|原状态|横误差RMS m|朝向峰值 rad|沿切线origin均速 m/s|停车XY/yaw|',
        '|---|---|---|---|---|---|---|---|']
    for r in records:
        p=r['path_parameters'];m=r['metrics'];old=m['original_curve_metrics']
        name=f"圆R{p['radius_m']} sign{p['direction_sign']:+d} r{r['repetition']}" if p['kind']=='circle' else f"S L{p['curve_length_m']} A{p['heading_amplitude_rad']}"
        def fmt(v):return '未验证' if v is None else f'{v:.6f}'
        lines.append(f"|{r['index']}|{name}|{r['feedback_hz_selected']}/{m['feedback_rate']['actual_math_hz']:.3f}|{r['original_receipt']['status']}|{fmt(m['drive_signed_cross_RMS_m'])}|{fmt(old.get('maximum_heading_error_rad'))}|{fmt(m['mean_actual_origin_tangent_speed_mps'])}|{fmt(old.get('parking_xy_m'))}/{fmt(old.get('parking_yaw_rad'))}|")
    lines += ['','仅实际弧区（原native progress位于entry至entry＋curve_length）诊断如下，原验收数字不变。初始化/直线段的drive峰值不用于推断曲线动态频率差异。','',
        '|索引|弧区heading RMS / max rad|弧区cross RMS m|弧区origin速度 MAE m/s|弧区heading峰值物理时刻 s|',
        '|---|---|---|---|---|']
    for r in records:
        d=r['metrics']['actual_arc_region_diagnostics'];peak=d['curve_only_heading_peak']
        lines.append(f"|{r['index']}|{fmt(d['arc_region_only_heading_RMS_rad'])}/{fmt(d['arc_region_only_heading_max_rad'])}|{fmt(d['arc_region_only_signed_cross_RMS_m'])}|{fmt(d['arc_region_only_origin_tangent_speed_MAE_mps'])}|{fmt(peak['native_physical_time_s'] if peak else None)}|")
    lines += ['',
        '原新增15门由各run归档的同SHA分析器以write=False重算并对照原状态；原共同验收、profile、raw、执行源码和独立数组的哈希均逐项核对。原生姿态/COM/角速度来自每个physics_step，时间为t−dt（dt=.005），不是force应用时刻t。实际origin速度=COM−ω×归档COM偏移，再按原姿态转入world；COM沿因果参考速度投影与origin沿几何切线速度分别报告。', '',
        '分析器重建实际native量并与原独立数组逐帧对照；局部单调弧长投影复用原哈希验证的独立曲线收据，StaticPath几何公式仍与归档控制器共享。因此测量与收据复核独立，不是完全独立推导的几何验收。未把bodyωz当Euler yawdot，也未把yawdot/v称为真实运动曲率。Euler yawdot与κv前馈不同还包括yaw反馈纠偏、内环响应和步态振荡；两者差值仅诊断，不新增或放宽验收门。', '',
        '真实反馈Hz只统计controller_updated=true的不同原feedback头，排除初始化/hold/parking。Teacher仍50Hz，native仍200Hz。图中的1s平滑仅用于显示，所有门限和原始数组保留未平滑数据。请求命令限幅、Teacher实际slew和内环PI条件积分根据原始记录重建；未触及某限幅的试验不能宣称已压力验证该限幅。', '',
        'S gentle(L8,A.6)、tight(L6,A1)各频率仅一轮；R.6每方向三轮，R1.2每方向一轮。不能仅凭停车分数、一次成功或这些有限场景宣称全局最优频率、最小物理半径或其他速度泛化。原规划profile中的旧plant重复验证字段保持原样，新增boundary收据链单独核验；未把旧unverified字段回填为passed。', '',
        '紧S25原固定停车yaw漂移超过0.1rad时，完整项必须保留FAILED，即使连续drive动态全部通过；不挑更安静停车窗。停车阶段持续Teacher零速度命令，外/内PI绕过输出零，频率改变到达及足步相位可能间接影响最后漂移，不能说高频停车闭环已修复漂移。active velocity/pose hold属于未实现新契约。', '',
        '紧圈/宽圈朝向峰值在JSON中保留真实物理时间、因果控制头、弧长、前馈变化和±0.3s固定上下文。圆与直线连接处曲率可发生跳变，κv变化率与实际slew/限幅相交仅为时间关联证据；渐变曲率或限速需事前冻结的同场景A/B才能确认改善，当前没有该因果实证。', '',
        'SCAN、实际SLAM曲线、多层、动态障碍、新曲线律Isaac对照及真机仍未验证。', '',
        '![实际轨迹](actual_xy_all14.png)','',
        '![S频率误差速度](S_frequency_errors_velocity.png)','',
        '![真实偏航角速度与PI](actual_Euler_rate_and_PI.png)','',
        '完整来源、逐项状态、PI重建与固定停车统计见[aggregate.json](aggregate.json)；独立诊断数组为[actual_curve_arrays.npz](actual_curve_arrays.npz)。PNG人工像素QA单独保存，不自动假定通过。','']
    with (output/'REPORT.md').open('x') as f:f.write('\n'.join(lines))


def finalize(campaign,output):
    plan,planned,indexed,ready=readiness(campaign)
    if not ready['ready']:
        raise ValueError('Final aggregate refused until all14 original receipts exist: '+json.dumps(ready))
    if output.exists():raise FileExistsError('Append-only output directory already exists')
    verified={};freeze_path=campaign/'curve_freeze_v1.json'
    verify(freeze_path,plan['source_freeze_sha256'],verified);freeze=load(freeze_path)
    planned_sha=sha(campaign/'phase_curve_v1_plan.json');results_sha=sha(campaign/'phase_curve_v1_results.jsonl')
    records=[];traces={}
    for i in EXPECTED:
        try:
            r,tr=inspect_run(indexed[i]['run'],planned[i],freeze)
            records.append(r);traces[i]=tr
        except Exception as exc:
            # Do not produce misleading plots/partial final. Diagnose to the
            # caller; originals remain untouched. A failed motion receipt with
            # complete evidence is preserved normally in the final aggregate.
            raise RuntimeError(f'Index {i}: evidence incomplete/mismatched: {exc}') from exc
    if sha(campaign/'phase_curve_v1_plan.json')!=planned_sha or sha(campaign/'phase_curve_v1_results.jsonl')!=results_sha:
        raise ValueError('Campaign inputs changed during readonly analysis')
    receipt={'schema':'actual_teacher_continuous_curve_campaign_analysis/v1',
        'status':'passed' if all(r['status']=='passed' for r in records) else 'failed' if any(r['status']=='failed' for r in records) else 'unverified',
        'prospective_indices':list(EXPECTED),'run_count':14,
        'original_curve_passed_runs':sum(r['original_receipt']['status']=='passed' for r in records),
        'original_curve_checks_passed':sum(v=='passed' for r in records for v in r['original_15_checks'].values()),
        'original_dynamic_drive_checks_passed':sum(v=='passed' for r in records for v in r['original_dynamic_curve_checks'].values()),
        'original_dynamic_drive_passed_runs':sum(all(v=='passed' for v in r['original_dynamic_curve_checks'].values()) for r in records),
        'original_curve_required_checks':210,'score':None,'navigation_ground_truth_used':True,
        'SLAM_navigation_verified':False,'SCAN_verified':False,'multifloor_verified':False,'hardware_verified':False,
        'actual_SLAM_curves_verified':False,'Isaac_new_curve_law_verified':False,
        'actor_privileged_dimensions':232,'known_controller_dimensions':15,
        'campaign_plan':{'path':str(campaign/'phase_curve_v1_plan.json'),'sha256':planned_sha},
        'campaign_results':{'path':str(campaign/'phase_curve_v1_results.jsonl'),'sha256':results_sha},
        'prospective_source_freeze':{'path':str(freeze_path),'sha256':sha(freeze_path)},
        'analysis_source':{'path':str(Path(__file__).resolve()),'sha256':sha(__file__)},
        'original_inputs_modified':False,'geometry_formulas_fully_independent':False,'records':records}
    output.mkdir(parents=True)
    write_new(output/'aggregate.json',receipt)
    arrays={f'case{i:02d}_{key}':value for i,tr in traces.items() for key,value in tr.items()}
    with (output/'actual_curve_arrays.npz').open('xb') as f:np.savez_compressed(f,**arrays)
    plots(output,records,traces);report(output,records,receipt)
    write_new(output/'manifest.json',{'schema':'curve_campaign_artifact_manifest/v1',
        'files':{p.name:sha(p) for p in sorted(output.iterdir()) if p.is_file()},
        'pixel_QA':'Requires independent image inspection; no automatic visual pass',
        'all_original_run_files_untouched':all(r['old_original_receipt_unchanged'] for r in records)})
    return {'status':receipt['status'],'output':str(output),'aggregate_sha256':sha(output/'aggregate.json')}


def self_test():
    plan={'profiles':[{'index':i,'sha256':str(i)} for i in EXPECTED]}
    result=[{'index':i,'profile':{'sha256':str(i)},'run':f'/readonly/example_run_{i}'} for i in EXPECTED]
    assert len(index_campaign(plan,result)[1])==14
    for bad in (result+[result[0]],result+[{'index':14,'profile':{'sha256':'14'},'run':'/readonly/extra'}],
            result[:-1]+[{'index':13,'profile':{'sha256':'changed'},'run':'/readonly/changed'}],
            result[:-1]+[{'index':13,'profile':{'sha256':'13'},'run':'/readonly/example_run_0'}]):
        try:index_campaign(plan,bad)
        except ValueError:pass
        else:raise AssertionError('Invalid campaign accepted')
    c=[{'controller_updated':False,'feedback_time_s':0},
       {'controller_updated':True,'feedback_time_s':1},
       {'controller_updated':False,'feedback_time_s':1},
       {'controller_updated':True,'feedback_time_s':1.04}]
    assert abs(gap_rate(c)['actual_math_hz']-25)<1e-10
    c[-1]['feedback_time_s']=1
    try:gap_rate(c)
    except ValueError:pass
    else:raise AssertionError('Duplicated math feedback accepted')
    # COM lever arm and Euler/body-wz distinction are not interchangeable.
    omega=np.array([.1,.2,.3]);offset=np.array([.05,-.01,.02]);com=np.array([.4,.1,0.])
    assert np.linalg.norm(com-(com-np.cross(omega,offset)))>0
    roll,pitch=.2,.1;rate=(math.sin(roll)*omega[1]+math.cos(roll)*omega[2])/math.cos(pitch)
    assert abs(rate-omega[2])>.01
    return {'pure_checks':'passed','negative_cases':5,'no_files_written':True,'no_simulation_started':True}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--campaign',type=Path,default=CAMPAIGN)
    ap.add_argument('--output',type=Path)
    ap.add_argument('--final',action='store_true')
    ap.add_argument('--self-test',action='store_true')
    ap.add_argument('--inspect-run',type=Path,help='Readonly single-run diagnostic JSON to stdout, no final output')
    args=ap.parse_args();campaign=args.campaign.resolve()
    if args.self_test:result=self_test()
    elif args.inspect_run:
        plan,planned,indexed,_=readiness(campaign)
        matching=[i for i,r in indexed.items() if Path(r['run']).resolve()==args.inspect_run.resolve()]
        if len(matching)!=1:raise ValueError('Inspection run is not unique prospective result')
        i=matching[0];r,_=inspect_run(args.inspect_run,planned[i],load(campaign/'curve_freeze_v1.json'))
        result={'index':i,'status':r['status'],'metrics':r['metrics'],'old_inputs_written':False}
    elif args.final:result=finalize(campaign,(args.output or campaign/'curve_analysis').resolve())
    else:result=readiness(campaign)[3]
    print(json.dumps(serial(result),ensure_ascii=False,allow_nan=False))


if __name__=='__main__':main()
