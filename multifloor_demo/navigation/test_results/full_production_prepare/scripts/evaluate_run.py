#!/usr/bin/env python3
"""Evaluate recorded evidence for one complete Go2 multifloor run.

No ROS node, robot command, or trajectory correction is created. Truth is used
only for evaluation. One initial time-matched SE(3) transform is held fixed for
all stages. Missing evidence is a failure, never an inferred success.
"""
import argparse
import bisect
import hashlib
import json
import math
import sys
import tempfile
import tarfile
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from slam.heading_alignment import calibrate_scene_heading
from navigation.goal_regions import contains, contains_control, definitions_sha256, parse_goal
from mission.route_regions import transformed_route_goals, validate_route_regions


STAGES = ['exploring', 'returning', 'saving_map', 'navigating']
MOVING = {'exploring', 'returning', 'navigating'}
ROUTE_KEYS = {'exploring': 'exploration', 'returning': 'return_origin', 'navigating': 'navigation_f1_f3'}
GOAL_TOLERANCE = .30
MAP_DTYPE = np.dtype([('xyz', '<f4', (3,)), ('rgb', 'u1', (3,)), ('pad', 'u1')])


def finite_vector(value, length=3):
    try:
        array = np.asarray(value, dtype=float)
        return array if array.shape == (length,) and np.isfinite(array).all() else None
    except (ValueError, TypeError):
        return None


def statistics(errors):
    errors = np.asarray(errors, dtype=float)
    return {'samples': len(errors), 'rmse_m': float(np.sqrt(np.mean(errors**2))),
            'max_m': float(errors.max()), 'final_m': float(errors[-1])} if len(errors) else None


def obstacle_motion_evidence(history):
    """Verify acknowledged Gazebo positions from navigation-only history.

    A stop counter or repeated publication of a stationary box is insufficient.
    Positions are compared only across increasing successful-update counters;
    requiring a one-metre excursion prevents accumulated stationary jitter from
    looking like the requested moving obstacle.
    """
    result={'required_displacement_m':1.0,'history_samples':len(history) if isinstance(history,list) else 0,
            'acknowledged_position_samples':0,'invalid_records':0,'update_counter_backwards':0,
            'timestamp_backwards':0,'failed_update_samples':0,'blocking_then_leaving_or_clear':False}
    if not isinstance(history,list) or not history:
        result['error']='missing navigation-stage active obstacle_history'
        return False,result
    records=[]
    for row in history:
        if not isinstance(row,dict):result['invalid_records']+=1;continue
        position=finite_vector(row.get('position'))
        try:
            updates=float(row['gazebo_updates']);failures=float(row['failed_updates']);stamp=float(row['stamp'])
            if row.get('active') is not True or position is None or not all(map(math.isfinite,[updates,failures,stamp])) \
                    or updates<0 or failures<0 or updates!=int(updates) or failures!=int(failures):
                raise ValueError('invalid active physical obstacle state')
            records.append({'position':position,'updates':updates,'failures':failures,'stamp':stamp,
                            'phase':row.get('visible_phase')})
        except (ValueError,TypeError,KeyError):result['invalid_records']+=1
    if not records:
        result['error']='no valid acknowledged active obstacle positions'
        return False,result
    result['failed_update_samples']=sum(r['failures']>0 for r in records)
    result['update_counter_backwards']=sum(b['updates']<a['updates'] for a,b in zip(records,records[1:]))
    result['timestamp_backwards']=sum(b['stamp']<a['stamp'] for a,b in zip(records,records[1:]))
    acknowledged=[];previous_updates=None;phases=[];blocking_seen=False
    for row in records:
        phases.append(row['phase'])
        if row['updates']>0:
            if row['phase']=='blocking':blocking_seen=True
            elif blocking_seen and row['phase'] in ('leaving','clear'):
                result['blocking_then_leaving_or_clear']=True
            if previous_updates is None or row['updates']>previous_updates:
                acknowledged.append(row)
            previous_updates=row['updates']
    result['acknowledged_position_samples']=len(acknowledged)
    result['phases']=sorted({str(phase) for phase in phases})
    result['successful_update_increase']=records[-1]['updates']-records[0]['updates']
    if acknowledged:
        positions=np.array([r['position'] for r in acknowledged])
        result['first_position']=positions[0].tolist();result['last_position']=positions[-1].tolist()
        result['max_displacement_from_first_m']=float(np.linalg.norm(positions-positions[0],axis=1).max())
        result['path_length_m']=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum())
    passed=not any(result[k] for k in ('invalid_records','update_counter_backwards','timestamp_backwards','failed_update_samples')) \
        and result['acknowledged_position_samples']>=2 and result['successful_update_increase']>0 \
        and result.get('max_displacement_from_first_m',0)>=1.0 and result['blocking_then_leaving_or_clear']
    return passed,result


def recorded_heading_alignment(scenario, mission, slam_samples):
    """Validate only the run's frozen IMU-derived calibration, never fit truth.

    Legacy relative_initial_body routes intentionally remain translation-only.
    The new relative_world_axes contract requires a recorded, bounded heading
    estimate before movement; the independent truth alignment is not an input.
    """
    reference=scenario.get('waypoint_reference','relative_initial_body')
    required=reference=='relative_world_axes' or scenario.get('require_heading_alignment') is True
    report={'required':required,'waypoint_reference':reference,'source':'recorded mission.heading_alignment; no evaluation truth used'}
    if not required:
        report['applied_to_waypoints']=False
        return True,report,None
    heading=mission.get('heading_alignment')
    if not isinstance(heading,dict):
        report['error']='relative_world_axes requires this run frozen IMU/SLAM heading calibration'
        return False,report,None
    reasons=[]
    def numeric(name):
        value=heading.get(name)
        if not isinstance(value,(int,float)) or isinstance(value,bool) or not math.isfinite(value):
            reasons.append(f'missing/non-finite {name}')
            return None
        return float(value)
    theta=numeric('yaw_camera_init_from_world')
    count=numeric('sample_count');first=numeric('first_stamp');last=numeric('last_stamp')
    pairing=numeric('max_stamp_difference_s');spread=numeric('max_heading_spread_rad');tilt=numeric('max_gravity_residual_rad')
    if count is not None and (count<10 or count!=int(count)):reasons.append('fewer than ten distinct paired attitudes')
    if first is not None and last is not None and last-first<.8-1e-9:reasons.append('calibration span below 0.8 seconds')
    if pairing is not None and not 0<=pairing<=.02+1e-9:reasons.append('IMU/SLAM timestamp difference exceeds 20 ms')
    if spread is not None and not 0<=spread<=math.radians(1)+1e-9:reasons.append('heading spread exceeds one degree')
    if tilt is not None and not 0<=tilt<=math.radians(3)+1e-9:reasons.append('gravity-axis mismatch exceeds three degrees')
    if heading.get('ground_truth_used') is not False:reasons.append('calibration must explicitly exclude evaluation truth')
    imu=scenario.get('sensors',{}).get('imu',{})
    declaration=imu.get('orientation_reference',{})
    if not heading.get('imu_reference_description') or heading.get('imu_reference_description')!=declaration.get('description'):
        reasons.append('missing or mismatched declared IMU orientation reference')
    for actual,expected,name in [(heading.get('imu_reference_world_quaternion'),declaration.get('world_quaternion'),'world reference'),
                                 (heading.get('body_imu_quaternion'),declaration.get('body_imu_quaternion'),'IMU mounting')]:
        qa,qb=finite_vector(actual,4),finite_vector(expected,4)
        if qa is None or qb is None or min(np.linalg.norm(qa),np.linalg.norm(qb))<1e-9:
            reasons.append(f'invalid {name} quaternion')
        elif not np.allclose(Rotation.from_quat(qa).as_matrix(),Rotation.from_quat(qb).as_matrix(),atol=1e-9):
            reasons.append(f'{name} quaternion differs from this run scenario')
    body_rpy=finite_vector(imu.get('body_rpy'))
    body_q=finite_vector(heading.get('body_imu_quaternion'),4)
    if body_rpy is None or body_q is None or np.linalg.norm(body_q)<1e-9:
        reasons.append('missing actual sensor mounting geometry')
    elif not np.allclose(Rotation.from_euler('xyz',body_rpy).as_matrix(),Rotation.from_quat(body_q).as_matrix(),atol=1e-9):
        reasons.append('declared IMU mounting disagrees with physical sensor extrinsic')
    matrix=None
    if theta is not None:
        c,s=math.cos(theta),math.sin(theta);matrix=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        try:
            stored=np.asarray(heading.get('rotation_camera_init_from_world'),dtype=float)
            if stored.shape!=(3,3) or not np.isfinite(stored).all() or not np.allclose(stored,matrix,atol=1e-9):
                reasons.append('recorded rotation matrix is inconsistent with frozen yaw')
        except (TypeError,ValueError):reasons.append('invalid recorded heading rotation matrix')
    waiting=[r['stamp'] for r in slam_samples if r['stage']=='waiting_sensors']
    moving=[r['stamp'] for r in slam_samples if r['stage'] in MOVING]
    if not waiting or first is None or last is None or first<min(waiting)-1e-6 or last>max(waiting)+1e-6:
        reasons.append('calibration timestamps are not supported by the recorded initialization window')
    if moving and last is not None and last>=min(moving):reasons.append('calibration was not frozen before movement')
    pairs=heading.get('attitude_pairs')
    recomputed=None
    if not isinstance(pairs,list) or not pairs:
        reasons.append('missing the actual IMU/SLAM calibration attitude pairs')
    else:
        try:
            recomputed=calibrate_scene_heading(pairs,
                imu_reference_world_quaternion=declaration['world_quaternion'],
                body_imu_quaternion=declaration['body_imu_quaternion'],
                reference_description=declaration['description'])
            for name in ('yaw_camera_init_from_world','sample_count','first_stamp','last_stamp',
                         'max_stamp_difference_s','max_heading_spread_rad','max_gravity_residual_rad'):
                if not np.isclose(heading.get(name,math.nan),recomputed[name],atol=1e-9,rtol=0):
                    reasons.append(f'recorded {name} disagrees with actual calibration pairs')
            waiting_rows=[r for r in slam_samples if r['stage']=='waiting_sensors']
            paired_positions=[]
            for pair in pairs:
                row=next((r for r in waiting_rows if abs(r['stamp']-pair['slam_stamp'])<=1e-8),None)
                paired_q=finite_vector(pair.get('slam_body_quaternion'),4)
                if row is None or paired_q is None or np.linalg.norm(paired_q)<1e-9:
                    reasons.append('calibration pair has no matching recorded initialization SLAM pose')
                    break
                if not np.allclose(Rotation.from_quat(row['q']).as_matrix(),Rotation.from_quat(paired_q).as_matrix(),atol=1e-9):
                    reasons.append('calibration pair attitude disagrees with recorded initialization SLAM pose')
                    break
                paired_positions.append(row['p'])
            if paired_positions and np.linalg.norm(np.asarray(paired_positions)-paired_positions[0],axis=1).max()>.08:
                reasons.append('calibration pose window is not stationary within 0.08 m')
        except (ValueError,TypeError,KeyError,IndexError) as error:
            reasons.append(f'invalid recorded calibration pairs: {error}')
    report.update({'applied_to_waypoints':reference=='relative_world_axes','yaw_camera_init_from_world':theta,
        'sample_count':count,'first_stamp':first,'last_stamp':last,'max_stamp_difference_s':pairing,
        'max_heading_spread_rad':spread,'max_gravity_residual_rad':tilt,
        'imu_reference_description':heading.get('imu_reference_description'),
        'recomputed_from_recorded_pairs':recomputed,'errors':reasons})
    return not reasons,report,matrix if not reasons else None


def inside_run(directory, filename):
    if not isinstance(filename, str) or not filename:
        raise ValueError('missing run-local filename')
    path = (directory / filename).resolve()
    if path.parent != directory.resolve():
        raise ValueError('artifact must be a direct child of this run directory')
    return path


def read_pcd(path):
    """Read this module's binary XYZ+packed-RGB PCD without an external loader."""
    raw = path.read_bytes()
    marker = b'DATA binary\n'
    if marker not in raw:
        raise ValueError('expected binary PCD, not reference/unsupported encoding')
    header, payload = raw.split(marker, 1)
    rows = {}
    for line in header.decode('ascii').splitlines():
        if line and not line.startswith('#'):
            words = line.split(); rows[words[0]] = words[1:]
    if rows.get('FIELDS') != ['x', 'y', 'z', 'rgb'] or rows.get('SIZE') != ['4']*4:
        raise ValueError('PCD must contain float32 XYZ and packed camera RGB')
    if rows.get('TYPE', [])[:3] != ['F']*3 or rows.get('TYPE', [''])[-1] not in ('F', 'U'):
        raise ValueError('unexpected PCD field type')
    count = int(rows['POINTS'][0])
    if len(payload) != count*16:
        raise ValueError('PCD payload size disagrees with POINTS')
    points = np.frombuffer(payload, dtype=[('xyz', '<f4', (3,)), ('rgb', '<u4')])
    if not np.isfinite(points['xyz']).all():
        raise ValueError('PCD contains invalid geometry')
    return {'point_count': count, 'unique_colors': int(len(np.unique(points['rgb'] & 0xffffff))),
            'size_bytes': len(raw)}


def read_poses(path):
    samples = {'slam': [], 'truth': []}
    errors = []
    if not path.is_file():
        return samples, ['pose_audit.jsonl is missing; no independently timed trajectory evidence']
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                name = row['source']
                p, q = finite_vector(row['p']), finite_vector(row['q'], 4)
                ts = float(row['stamp'])
                if name not in samples or p is None or q is None or np.linalg.norm(q)<1e-9 or not math.isfinite(ts):
                    raise ValueError('invalid source, pose, quaternion, or timestamp')
                samples[name].append({'stamp': ts, 'stamp_ns': row.get('stamp_ns'),
                                      'p': p, 'q': q/np.linalg.norm(q), 'stage': row.get('stage', '')})
            except (ValueError, KeyError, TypeError) as error:
                if len(errors)<10:
                    errors.append(f'line {line_number}: {error}')
    return samples, errors


def time_matched_positions(slam, ordered_truth):
    """Match real SLAM timestamps to bounded truth interpolation, without fitting."""
    if not ordered_truth:
        return [], np.empty((0, 3))
    tt = np.array([row['stamp'] for row in ordered_truth])
    gp = np.array([row['p'] for row in ordered_truth])
    candidates = sorted((row for row in slam if tt[0] <= row['stamp'] <= tt[-1]), key=lambda row: row['stamp'])
    matched = []
    for row in candidates:
        stamp = row['stamp']; index = np.searchsorted(tt, stamp, side='right')
        left, right = max(0, index-1), min(len(tt)-1, index)
        exact = min(abs(tt[left]-stamp), abs(tt[right]-stamp)) <= 1e-8
        if exact or tt[right]-tt[left] <= .15:
            matched.append(row)
    stamps = np.array([row['stamp'] for row in matched])
    positions = np.column_stack([np.interp(stamps, tt, gp[:, axis]) for axis in range(3)])
    return matched, positions


def scenario_route_targets(scenario, name, origin, scene_rotation):
    """Reconstruct the prescribed route, independently of mission progress."""
    raw = scenario.get(name)
    if not isinstance(raw, list) or not raw:
        return [], f'missing or empty prescribed scenario route: {name}'
    targets = [finite_vector(point) for point in raw]
    if any(point is None for point in targets):
        return [], f'non-finite or malformed prescribed waypoint in {name}'
    reference = scenario.get('waypoint_reference', 'relative_initial_body')
    if reference in ('relative_initial_body', 'relative_world_axes') and origin is None:
        return [], 'relative waypoints require this run mission.origin'
    if reference == 'relative_world_axes':
        if scene_rotation is None:
            return [], 'world-axis waypoints require the valid frozen IMU/SLAM heading'
        targets = [origin + scene_rotation @ point for point in targets]
    elif reference == 'relative_initial_body':
        targets = [origin + point for point in targets]
    return targets, None


def ordered_waypoint_evidence(scenario, origin, scene_rotation, samples, rotation, translation, truth):
    """Require the complete route at real, ordered, jointly valid pose samples.

    The sole trajectory alignment is supplied by compare_trajectory. A waypoint
    requires SLAM and interpolated truth to be within tolerance at the SAME
    SLAM timestamp. We never interpolate a robot path to invent an arrival.
    A missing earlier point blocks acceptance of the remaining route prefix.
    """
    report = {'tolerance_m': GOAL_TOLERANCE,
              'alignment': 'the same single initial SE(3) as trajectory; no per-stage or waypoint fit',
              'arrival_rule': 'earliest jointly valid recorded SLAM timestamp strictly after the preceding waypoint',
              'stages': {}}
    matched, positions = time_matched_positions(samples['slam'], truth)
    previous_stamp, blocked = None, False
    for stage, key in ROUTE_KEYS.items():
        targets, error = scenario_route_targets(scenario, key, origin, scene_rotation)
        indices = [index for index, row in enumerate(matched) if row['stage'] == stage]
        stage_rows = [matched[index] for index in indices]
        slam_positions = np.array([row['p'] for row in stage_rows]).reshape((-1, 3))
        truth_positions = positions[indices]
        stamps = np.array([row['stamp'] for row in stage_rows])
        stage_report = {'scenario_route': key, 'required_waypoints': len(targets),
                        'matched_pose_samples': len(stage_rows), 'waypoints': []}
        report['stages'][stage] = stage_report
        if error:
            stage_report.update({'passed': False, 'error': error, 'reached_waypoints': 0})
            blocked = True
            continue
        for index, target in enumerate(targets):
            expected = rotation @ target + translation if rotation is not None else None
            point = {'index': index+1, 'slam_target': target.tolist(),
                     'world_target': expected.tolist() if expected is not None else None,
                     'reached_in_order': False, 'arrival': None,
                     'nearest_observation': None, 'nearest_ordered_observation': None}
            stage_report['waypoints'].append(point)
            if expected is None or not stage_rows:
                point['error'] = 'missing fixed alignment or time-matched stage trajectory'
                blocked = True
                continue
            slam_errors = np.linalg.norm(slam_positions-target, axis=1)
            truth_errors = np.linalg.norm(truth_positions-expected, axis=1)
            joint_errors = np.maximum(slam_errors, truth_errors)
            def observation(which):
                return {'stamp': float(stamps[which]), 'slam_error_m': float(slam_errors[which]),
                        'truth_error_m': float(truth_errors[which]),
                        'slam_position': slam_positions[which].tolist(),
                        'truth_position': truth_positions[which].tolist()}
            point['nearest_observation'] = observation(int(np.argmin(joint_errors)))
            eligible = np.ones(len(stamps), dtype=bool) if previous_stamp is None else \
                stamps > previous_stamp
            remaining = np.flatnonzero(eligible)
            if len(remaining):
                point['nearest_ordered_observation'] = observation(int(remaining[np.argmin(joint_errors[remaining])]))
            arrivals = np.flatnonzero(eligible & (slam_errors <= GOAL_TOLERANCE) & (truth_errors <= GOAL_TOLERANCE))
            if blocked:
                point['error'] = 'an earlier prescribed waypoint is missing; ordered route prefix is incomplete'
            elif len(arrivals):
                arrival = int(arrivals[0])
                point['arrival'] = observation(arrival)
                point['reached_in_order'] = True
                previous_stamp = float(stamps[arrival])
            else:
                point['error'] = 'no later real sample reaches both SLAM and fixed-alignment truth targets within 0.30 m'
                blocked = True
        stage_report['reached_waypoints'] = sum(point['reached_in_order'] for point in stage_report['waypoints'])
        stage_report['passed'] = bool(targets) and stage_report['reached_waypoints'] == len(targets)
    report['passed'] = all(stage['passed'] for stage in report['stages'].values())
    return report


def region_truth_pairs(samples, truth):
    """Pair every active raw SLAM sample using integer sensor timestamps.

    Missing leading/trailing truth and interpolation holes remain explicit.
    The legacy matching function intentionally retains its old behavior.
    """
    active_stages = MOVING | {'saving_map'}
    active = [row for row in samples['slam'] if row['stage'] in active_stages]
    def valid_ns(row):
        ns = row.get('stamp_ns')
        return type(ns) is int and ns >= 0 and abs(row['stamp']-ns/1e9) <= 1e-6
    invalid_truth = [row for row in truth if not valid_ns(row)]
    ordered = sorted((row for row in truth if valid_ns(row)), key=lambda row: row['stamp_ns'])
    tt = [row['stamp_ns'] for row in ordered]
    positions = {}
    missing = []
    for row in active:
        ns = row.get('stamp_ns')
        reason = None
        if not valid_ns(row):
            reason = 'missing/invalid original integer SLAM stamp_ns'
        elif not tt or ns < tt[0] or ns > tt[-1]:
            reason = 'truth does not cover this active SLAM timestamp'
        else:
            i = bisect.bisect_left(tt, ns)
            if i < len(tt) and tt[i] == ns:
                positions[ns] = ordered[i]['p']
            elif tt[i]-tt[i-1] > 150_000_000:
                reason = 'truth interpolation bracket exceeds 150 ms'
            else:
                fraction = (ns-tt[i-1])/(tt[i]-tt[i-1])
                positions[ns] = ordered[i-1]['p'] + fraction*(ordered[i]['p']-ordered[i-1]['p'])
        if reason:
            missing.append({'stamp': row['stamp'], 'stamp_ns': ns, 'stage': row['stage'], 'reason': reason})
    coverage = {'active_slam_samples': len(active), 'matched_active_samples': len(active)-len(missing),
                'active_stages': sorted(active_stages),
                'scope': 'All motion/save-stage SLAM; completed shutdown tail is diagnostic only, endpoints use the actual NAV receipt window.',
                'unpaired_samples': len(missing), 'unpaired': missing,
                'invalid_truth_integer_stamps': len(invalid_truth),
                'max_truth_interpolation_gap_ns': 150_000_000,
                'passed': bool(active) and not missing and not invalid_truth}
    return active, positions, coverage


def preregistered_region_evidence(directory, scenario):
    """Verify the declaration against the source archive made before launch."""
    try:
        manifest = json.loads((directory/'source_manifest.json').read_text())
        with tarfile.open(directory/'source_snapshot.tar.gz', 'r:gz') as archive:
            stream = archive.extractfile('simulation/scenario.json')
            if stream is None:
                raise ValueError('source archive lacks the preregistered scenario')
            encoded = stream.read()
        actual = hashlib.sha256(encoded).hexdigest()
        if manifest.get('sha256', {}).get('simulation/scenario.json') != actual:
            raise ValueError('archived scenario SHA differs from prelaunch manifest')
        declared = json.loads(encoded)
        validate_route_regions(declared)
        if declared != scenario:
            raise ValueError('run scenario differs from its prelaunch source declaration')
        return {'passed': True, 'source': 'prelaunch source_snapshot simulation/scenario.json', 'sha256': actual}
    except (OSError, ValueError, KeyError, TypeError, tarfile.TarError) as error:
        return {'passed': False, 'error': str(error)}


def ordered_region_evidence(directory, scenario, mission, samples, rotation, translation, truth):
    """Validate exactly the real NAV arrival window, under one initial SE(3).

    No later window or next-stage observations can rescue a raw NAV arrival.
    Every raw SLAM observation in its declared window must have same-time
    bounded truth. With an explicitly declared control band, raw SLAM must
    inhabit that inner band while truth remains inside the original outer
    region. Historical requests without a band keep their original rule.
    """
    report = {'alignment': 'the same single initial SE(3) as trajectory; never fit per goal/stage',
              'arrival_rule': 'actual NAV receipt window: same-time raw SLAM and fixed-alignment truth both inside throughout measured dwell',
              'tolerance_m': None, 'max_observation_gap_ns': 200_000_000, 'stages': {}}
    try:
        validate_route_regions(scenario)
        if 'route_goals' not in scenario:
            raise ValueError('region evaluation requires an explicit route_goals declaration')
        goals = {stage: transformed_route_goals(scenario, key, mission.get('origin'), mission.get('heading_alignment'))
                 for stage, key in ROUTE_KEYS.items()}
    except (ValueError, KeyError, TypeError) as error:
        report.update(passed=False, error=str(error))
        return report
    report['preregistered_declaration'] = preregistered_region_evidence(directory, scenario)
    if any(g.control_extents is not None for values in goals.values() for g in values):
        report['control_arrival_rule'] = 'same actual NAV window: raw SLAM inside preregistered inner control band; fixed-alignment truth inside unchanged outer arrival region'
    active, paired, coverage = region_truth_pairs(samples, truth)
    report['truth_full_pose_coverage'] = coverage
    statuses = []
    errors = []
    try:
        with (directory/'navigation_audit.jsonl').open() as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                    if not isinstance(row, dict) or not isinstance(row.get('status'), dict):
                        raise ValueError('missing actual navigation callback status')
                    statuses.append(row)
                except (ValueError, TypeError) as error:
                    errors.append(f'line {line_number}: {error}')
    except OSError as error:
        errors.append(str(error))
    report['navigation_audit_errors'] = errors
    previous = -1
    blocked = rotation is None or translation is None
    for counter, (stage, key) in enumerate(ROUTE_KEYS.items(), 1):
        expected = goals[stage]
        goal_hash = definitions_sha256(expected)
        request_id = f"{mission.get('run_id')}:{key}:{counter}"
        matching = [row['status'] for row in statuses if row.get('stage') == stage
                    and row.get('current_request') == request_id and row['status'].get('request_id') == request_id]
        definition_errors = []
        for status in matching:
            if status.get('goals_definition_sha256') != goal_hash or status.get('goals_definitions') != [g.definition() for g in expected]:
                definition_errors.append('actual NAV definition/hash differs from this stage declaration')
        complete = [status for status in matching if status.get('state') == 'succeeded'
                    and status.get('waypoint_index') == status.get('total') == len(expected)
                    and isinstance(status.get('region_arrivals'), list) and len(status['region_arrivals']) == len(expected)]
        # Preserve a genuinely reached prefix when a later goal times out.
        observed = [status for status in matching if isinstance(status.get('region_arrivals'), list)]
        status = complete[0] if complete else max(observed, key=lambda s: len(s['region_arrivals']), default=None)
        receipts = status['region_arrivals'] if status else []
        if len(receipts) > len(expected) or any(s['region_arrivals'] != receipts[:len(s['region_arrivals'])] for s in observed):
            definition_errors.append('previously completed goal receipts changed or exceed the prescribed route')
        stage_rows = [row for row in active if row['stage'] == stage]
        entry = {'scenario_route': key, 'request_id': request_id, 'goals_definition_sha256': goal_hash,
                 'required_waypoints': len(expected), 'actual_status_samples': len(matching),
                 'definition_errors': definition_errors, 'waypoints': []}
        report['stages'][stage] = entry
        if not complete or definition_errors:
            entry['error'] = 'missing actual complete NAV callback or inconsistent goal declaration'
        if definition_errors:
            blocked = True
        for index, g in enumerate(expected):
            point = {'index': index+1, 'goal_id': g.goal_id, 'slam_target': list(g.center),
                     'region_definition': g.definition(), 'reached_in_order': False, 'arrival': None}
            has_control_band = g.control_extents is not None
            if has_control_band:
                point['control_region_definition'] = g.control_goal().definition()
            entry['waypoints'].append(point)
            receipt = receipts[index] if index < len(receipts) else None
            reasons = []
            if not isinstance(receipt, dict):
                point['error'] = 'missing actual NAV goal receipt'
                blocked = True
                continue
            point['navigation_receipt'] = receipt
            if receipt.get('request_id') != request_id or receipt.get('goal_id') != g.goal_id \
                    or receipt.get('goals_definition_sha256') != goal_hash or receipt.get('waypoint_index') != index:
                reasons.append('receipt request/goal/index/hash association is inconsistent')
            start, end, dwell = (receipt.get(k) for k in ('start_stamp_ns', 'stamp_ns', 'dwell_ns'))
            valid_time = all(type(v) is int and v >= 0 for v in (start, end, dwell))
            if not valid_time or end < start or start <= previous or dwell != end-start or dwell < round(g.dwell_sim_s*1e9):
                reasons.append('receipt has invalid or unordered measured dwell timestamps')
            if receipt.get('region_inside') is not True or receipt.get('protected') is not False \
                    or receipt.get('reason') != 'arrived' or receipt.get('max_observation_gap_ns') != 200_000_000 \
                    or receipt.get('arrival_definition') != g.definition()['arrival']:
                reasons.append('receipt region/protection/dwell contract differs from the declaration')
            if has_control_band and (receipt.get('control_region_inside') is not True \
                    or receipt.get('control_arrival_definition') != g.control_arrival_definition()):
                reasons.append('receipt control band differs from the preregistered inner geometry')
            raw = finite_vector(receipt.get('raw_position'))
            try:
                raw_inside = contains(g, receipt.get('raw_position'))
            except (ValueError, TypeError):
                raw_inside = False
            if raw is None or not raw_inside:
                reasons.append('receipt raw position is invalid or outside the region')
            center_error = None if raw is None else float(np.linalg.norm(raw-np.asarray(g.center)))
            recorded_error = receipt.get('center_error_m')
            if type(recorded_error) not in (int, float) or not math.isfinite(recorded_error) or center_error is None \
                    or abs(center_error-recorded_error) > 1e-8:
                reasons.append('receipt center error disagrees with its raw position')
            window = [row for row in stage_rows if type(row.get('stamp_ns')) is int and valid_time and start <= row['stamp_ns'] <= end]
            stamps = sorted({row['stamp_ns'] for row in window})
            if not valid_time or not stamps or stamps[0] != start or stamps[-1] != end \
                    or any(b-a > 200_000_000 for a, b in zip(stamps, stamps[1:])):
                reasons.append('actual raw SLAM does not cover the complete bounded receipt window')
            at_end = [row for row in window if row['stamp_ns'] == end]
            if raw is None or not at_end or any(np.linalg.norm(row['p']-raw) > 1e-8 for row in at_end):
                reasons.append('receipt raw position is not the actual SLAM pose at its stamp')
            observations = []
            for row in window:
                ns = row['stamp_ns']
                gt = paired.get(ns)
                if gt is None or rotation is None:
                    reasons.append('receipt window has no same-time bounded fixed-alignment truth')
                    continue
                # compare_trajectory returns world<-SLAM. Its inverse maps
                # independently observed GT into the same declared SLAM region.
                gt_slam = rotation.T @ (gt-translation)
                slam_inside, truth_inside = contains(g, row['p']), contains(g, gt_slam)
                observations.append({'stamp_ns': ns, 'slam_inside': slam_inside, 'truth_inside': truth_inside,
                    'slam_position': row['p'].tolist(), 'truth_position': gt.tolist(),
                    'truth_position_in_slam': gt_slam.tolist(),
                    'slam_error_m': float(np.linalg.norm(row['p']-g.center)),
                    'truth_error_m': float(np.linalg.norm(gt_slam-g.center))})
                if has_control_band:
                    control_inside = contains_control(g, row['p'])
                    observations[-1]['slam_control_inside'] = control_inside
                    if not control_inside:
                        reasons.append('raw SLAM is outside the preregistered control band during the receipt dwell')
                if not slam_inside or not truth_inside:
                    reasons.append('SLAM and truth do not both inhabit the same region throughout the receipt dwell')
            point['window_samples'] = len(window)
            point['window_observations'] = observations
            if blocked:
                reasons.append('an earlier required goal/stage is missing; ordered prefix is incomplete')
            if reasons:
                point['errors'] = list(dict.fromkeys(reasons))
                blocked = True
            else:
                point['reached_in_order'] = True
                point['arrival'] = {**observations[-1], 'stamp': end/1e9, 'start_stamp_ns': start,
                                    'dwell_ns': dwell, 'sample_stage': stage}
                previous = end
        entry['reached_waypoints'] = sum(p['reached_in_order'] for p in entry['waypoints'])
        entry['passed'] = bool(expected) and bool(complete) and not definition_errors and entry['reached_waypoints'] == len(expected)
        if not entry['passed']:
            blocked = True
    report['passed'] = report['preregistered_declaration']['passed'] and coverage['passed'] and not errors \
        and all(stage['passed'] for stage in report['stages'].values())
    return report


def compare_trajectory(samples):
    slam, truth = samples['slam'], samples['truth']
    result = {'alignment': 'one SE(3) transform from the earliest time-matched pair; never reset per stage',
              'source_samples': {'slam': len(slam), 'truth': len(truth)},
              'max_truth_interpolation_gap_s': .15, 'by_stage': {}}
    if len(slam)<2 or len(truth)<2:
        result['error'] = 'at least two real SLAM and truth samples are required'
        return result, None, None, []
    result['timestamp_backwards'] = {name: sum(b['stamp']<a['stamp'] for a,b in zip(rows,rows[1:])) for name,rows in samples.items()}
    result['timestamp_duplicates'] = {name: len(rows)-len({r['stamp'] for r in rows}) for name,rows in samples.items()}
    truth_unique = {r['stamp']: r for r in truth}
    ordered_truth = [truth_unique[t] for t in sorted(truth_unique)]
    tt = np.array([r['stamp'] for r in ordered_truth])
    if len(tt)<2:
        result['error'] = 'truth contains fewer than two distinct timestamps'
        return result, None, None, []
    gq = np.array([r['q'] for r in ordered_truth])
    matched, gt_positions = time_matched_positions(slam, ordered_truth)
    if not matched:
        result['error'] = 'no time-matched poses within the 150 ms truth gap bound'
        return result, None, None, []
    st = np.array([r['stamp'] for r in matched])
    gt_rotations = Slerp(tt,Rotation.from_quat(gq))(st).as_matrix()
    slam_positions = np.array([r['p'] for r in matched])
    slam_rotations = Rotation.from_quat([r['q'] for r in matched]).as_matrix()
    rotation = gt_rotations[0] @ slam_rotations[0].T
    translation = gt_positions[0] - rotation @ slam_positions[0]
    aligned = slam_positions @ rotation.T + translation
    errors = np.linalg.norm(aligned-gt_positions,axis=1)
    result.update({'initial_pair_stamp': float(st[0]), 'initial_pair_stage': matched[0]['stage'],
                   'last_matched_stamp': float(st[-1]),
                   'rotation_world_from_slam': rotation.tolist(), 'translation_world_from_slam': translation.tolist(),
                   'overall': statistics(errors), 'matched_fraction': len(matched)/len(slam)})
    for stage in sorted({r['stage'] for r in matched}):
        selection = np.array([r['stage']==stage for r in matched])
        result['by_stage'][stage] = statistics(errors[selection])
    active_truth = [r for r in ordered_truth if r['stage'] in MOVING or r['stage'] in {'saving_map','completed'}]
    if len(active_truth)>=2:
        points=np.array([r['p'] for r in active_truth])
        angles=Rotation.from_quat([r['q'] for r in active_truth]).as_euler('xyz')
        result['truth_motion']={'samples':len(active_truth),
            'path_length_m':float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
            'net_displacement_m':float(np.linalg.norm(points[-1]-points[0])),
            'z_min_m':float(points[:,2].min()), 'z_max_m':float(points[:,2].max()),
            'height_range_m':float(np.ptp(points[:,2])),
            'max_abs_roll_pitch_rad':float(np.max(np.abs(angles[:,:2])))}
    return result, rotation, translation, ordered_truth


def full_control_runtime_evidence_check(directory):
    """Optional new-run gate; old runs without literal True keep old checks."""
    directory = Path(directory)
    try:
        manifest = json.loads((directory/'runtime_manifest.json').read_text())
    except (OSError, ValueError):
        return None
    if not isinstance(manifest, dict) or manifest.get('full_control_runtime_required') is not True:
        return None
    artifact = directory/'full_control_runtime_evidence.json'
    try:
        evidence = json.loads(artifact.read_text())
        if not isinstance(evidence, dict):
            raise ValueError('expected full control runtime evidence object')
        named_checks = evidence.get('checks')
        valid = evidence.get('passed') is True and isinstance(named_checks, dict) \
            and bool(named_checks) and all(value is True for value in named_checks.values())
        return {'passed': valid, 'artifact': artifact.name,
                'artifact_sha256': hashlib.sha256(artifact.read_bytes()).hexdigest(),
                'observed_passed': evidence.get('passed'), 'observed_checks': named_checks,
                'requirement': 'Literal passed=true and a nonempty named checks object with every value true'}
    except (OSError, ValueError) as error:
        return {'passed': False, 'artifact': artifact.name, 'error': str(error)}


def evaluate(directory, contact_publisher_present=None):
    directory = Path(directory).resolve()
    checks=[]; diagnostics=[]
    def check(name, passed, detail):
        checks.append({'name':name,'passed':bool(passed),'detail':detail})
    def load(filename):
        try:
            value=json.loads((directory/filename).read_text())
            if not isinstance(value,dict): raise ValueError('expected JSON object')
            return value
        except (OSError,ValueError) as error:
            diagnostics.append(f'{filename}: {error}')
            return {}
    mission,scenario,metadata=[load(n) for n in ('mission.json','scenario.json','map_metadata.json')]
    check('required_json_artifacts', bool(mission and scenario and metadata), 'mission.json, scenario.json and map_metadata.json must all be readable')
    check('run_identity', mission.get('run_id')==metadata.get('run_id')==directory.name, 'mission and map must belong to this run directory')
    check('mission_completed',mission.get('stage')=='completed',{'stage':mission.get('stage'),'message':mission.get('message')})
    check('all_stages_completed',mission.get('completed_stages')==STAGES,{'required_order':STAGES,'observed':mission.get('completed_stages')})
    event_stages=[r.get('stage') for r in mission.get('events',[]) if isinstance(r,dict)]
    check('stage_transition_evidence',event_stages==['waiting_sensors']+STAGES+['completed'],{'observed':event_stages})
    check('physical_simulation_scope',scenario.get('simulation',{}).get('kind')=='gazebo_physics',scenario.get('simulation'))
    samples,pose_errors=read_poses(directory/'pose_audit.jsonl')
    diagnostics.extend(pose_errors)
    check('pose_audit_evidence',not pose_errors and len(samples['slam'])>=20 and len(samples['truth'])>=20,
          {'slam_samples':len(samples['slam']),'truth_samples':len(samples['truth']),'errors':pose_errors})
    heading_valid,heading_report,scene_rotation=recorded_heading_alignment(scenario,mission,samples['slam'])
    check('frozen_scene_heading_calibration',heading_valid,heading_report)
    trajectory,rotation,translation,truth=compare_trajectory(samples)
    check('trajectory_time_coverage',rotation is not None and trajectory.get('matched_fraction',0)>=.8
          and not any(trajectory.get('timestamp_backwards',{}).values()),
          {'matched_fraction':trajectory.get('matched_fraction'),'timestamp_backwards':trajectory.get('timestamp_backwards'),
           'error':trajectory.get('error')})
    check('single_alignment_from_initialization',trajectory.get('initial_pair_stage')=='waiting_sensors',{
        'initial_pair_stamp':trajectory.get('initial_pair_stamp'),'initial_pair_stage':trajectory.get('initial_pair_stage'),
        'alignment':trajectory['alignment']})
    check('moving_stage_pose_evidence',all(trajectory.get('by_stage',{}).get(s,{}).get('samples',0)>=2 for s in MOVING),
          {s:trajectory.get('by_stage',{}).get(s) for s in sorted(MOVING)})
    region_mode = 'route_goals' in scenario
    if region_mode:
        waypoints = ordered_region_evidence(directory,scenario,mission,samples,rotation,translation,truth)
        check('preregistered_region_declaration',waypoints.get('preregistered_declaration',{}).get('passed',False),
              waypoints.get('preregistered_declaration',waypoints.get('error')))
        check('all_active_slam_has_bounded_truth',waypoints.get('truth_full_pose_coverage',{}).get('passed',False),
              waypoints.get('truth_full_pose_coverage',waypoints.get('error')))
    else:
        waypoints=ordered_waypoint_evidence(scenario,finite_vector(mission.get('origin')),scene_rotation,
            samples,rotation,translation,truth)
    check('all_prescribed_waypoints_in_order',waypoints['passed'],{
        'arrival_rule':waypoints.get('arrival_rule'),'tolerance_m':waypoints.get('tolerance_m'),
        'stages':{s:{k:v for k,v in rows.items() if k!='waypoints'} for s,rows in waypoints['stages'].items()}})
    motion=trajectory.get('truth_motion',{})
    elevations=scenario.get('floor_elevations')
    valid_elevations=isinstance(elevations,list) and len(elevations)>=3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in elevations)
    required_height = float(elevations[-1]-elevations[0]) if valid_elevations else 2.4
    check('three_floor_scenario',valid_elevations and elevations[0]<elevations[1]<elevations[2],{'floor_elevations':elevations})
    check('physical_multifloor_motion',motion.get('path_length_m',0)>.5 and motion.get('height_range_m',0)>=required_height-GOAL_TOLERANCE,
          {'observed':motion,'required_floor_height_change_m':required_height,'height_tolerance_m':GOAL_TOLERANCE})
    endpoints={}
    origin,goal=finite_vector(mission.get('origin')),finite_vector(mission.get('goal'))
    for name,preferred,fallback,target in [('return_origin','saving_map','returning',origin),('floor3','completed','navigating',goal)]:
        if region_mode:
            stage_report = waypoints.get('stages',{}).get(fallback,{})
            points = stage_report.get('waypoints',[])
            final = points[-1] if points else {}
            arrival = final.get('arrival') or {}
            endpoints[name] = {**arrival, 'region_definition':final.get('region_definition'),
                               'sample_choice':'actual NAV receipt endpoint, same raw SLAM/truth stamp',
                               'joint_region_arrival':final.get('reached_in_order',False),
                               'center_errors_are_diagnostic':True}
            check(name+'_truth_region',final.get('reached_in_order') is True and arrival.get('truth_inside') is True,endpoints[name])
            check(name+'_slam_region',final.get('reached_in_order') is True and arrival.get('slam_inside') is True,endpoints[name])
            continue
        preferred_rows=[r for r in truth if r['stage']==preferred]
        fallback_rows=[r for r in truth if r['stage']==fallback]
        row=preferred_rows[0] if preferred_rows else fallback_rows[-1] if fallback_rows else None
        if row is not None and target is not None and rotation is not None:
            expected=rotation@target+translation
            endpoints[name]={'stamp':row['stamp'],'sample_stage':row['stage'], 'sample_choice':'first transition-stage sample' if preferred_rows else 'last preceding-stage sample',
                'slam_target':target.tolist(),'world_target':expected.tolist(),'truth_position':row['p'].tolist(),
                'truth_error_m':float(np.linalg.norm(row['p']-expected)),'tolerance_m':GOAL_TOLERANCE}
            nearest=min(samples['slam'],key=lambda sample:abs(sample['stamp']-row['stamp']))
            if abs(nearest['stamp']-row['stamp'])<=.15:
                endpoints[name]['slam_error_m']=float(np.linalg.norm(nearest['p']-target))
                endpoints[name]['slam_sample_stamp']=nearest['stamp']
        check(name+'_truth_error',endpoints.get(name,{}).get('truth_error_m',math.inf)<=GOAL_TOLERANCE,
              endpoints.get(name,'missing origin/goal or independently timed endpoint truth'))
        check(name+'_slam_error',endpoints.get(name,{}).get('slam_error_m',math.inf)<=GOAL_TOLERANCE,
              endpoints.get(name,'missing independently recorded SLAM endpoint near the truth sample'))
    expected_goal=None
    if scenario.get('navigation_f1_f3') and origin is not None:
        expected_goal=finite_vector(scenario['navigation_f1_f3'][-1])
        if expected_goal is not None and scenario.get('waypoint_reference','relative_initial_body')=='relative_initial_body':
            expected_goal=expected_goal+origin
        elif expected_goal is not None and scenario.get('waypoint_reference')=='relative_world_axes':
            expected_goal=origin+scene_rotation@expected_goal if scene_rotation is not None else None
    check('floor3_target_matches_scenario',goal is not None and expected_goal is not None and np.linalg.norm(goal-expected_goal)<1e-6,
          {'mission_goal':None if goal is None else goal.tolist(),'scenario_goal':None if expected_goal is None else expected_goal.tolist()})
    nav,validation=mission.get('navigation',{}),mission.get('validation',{})
    counts=nav.get('counts',{})
    check('navigation_uses_measured_slam',nav.get('feedback_source')=='/demo/slam/body_odom' and nav.get('frame_id')=='camera_init'
          and all(counts.get(k,0)>0 for k in ('odom','cloud','bspline','commands')),{'feedback_source':nav.get('feedback_source'),'counts':counts})
    dynamic={'navigation_stops':nav.get('obstacle_stops',0),'navigation_resumes':nav.get('obstacle_resumes',0),
             'observed_hold_events':validation.get('obstacle_hold_events',0),'observed_motion_after_hold':validation.get('resumed_after_hold',False)}
    check('dynamic_obstacle_stop_and_resume',dynamic['navigation_stops']>0 and dynamic['navigation_resumes']>0
          and dynamic['observed_hold_events']>0 and dynamic['observed_motion_after_hold'],dynamic)
    obstacle=mission.get('obstacle',{})
    check('physical_obstacle_updates',obstacle.get('gazebo_updates',0)>0 and obstacle.get('failed_updates',0)==0,
          {'gazebo_updates':obstacle.get('gazebo_updates'),'failed_updates':obstacle.get('failed_updates'),'source':obstacle.get('source')})
    obstacle_moved,obstacle_motion=obstacle_motion_evidence(validation.get('obstacle_history'))
    check('physical_dynamic_obstacle_motion',obstacle_moved,obstacle_motion)
    publisher_observation=contact_publisher_present
    if publisher_observation is None:
        publisher_observation=validation.get('body_contact_publisher_present',validation.get('body_contact_sensor_publisher_present',
            validation.get('body_contact_monitor',{}).get('publisher_present',False)))
    contact_count=mission.get('counts',{}).get('body_contact_sensor',0)
    check('body_contact_monitor_available',contact_count>0 or publisher_observation is True,
          {'received_messages':contact_count,'publisher_present':publisher_observation,'observation_source':'caller CLI graph observation' if contact_publisher_present is not None else 'recorded mission graph/message evidence'})
    contacts=validation.get('body_contact_events')
    check('no_body_contact_events',isinstance(contacts,list) and not contacts,{'body_contact_events':contacts})
    recorded_tilt=validation.get('max_abs_tilt_rad')
    actual_tilt=motion.get('max_abs_roll_pitch_rad')
    check('body_tilt_within_limit',isinstance(recorded_tilt,(int,float)) and math.isfinite(recorded_tilt) and recorded_tilt<.75
          and actual_tilt is not None and actual_tilt<.75,{'recorded_max_rad':recorded_tilt,'audit_max_rad':actual_tilt,'strict_limit_rad':.75})
    map_report={}
    try:
        binary=inside_run(directory,metadata.get('binary_filename')).read_bytes()
        if len(binary)%16: raise ValueError('browser map binary is not a whole 16-byte XYZRGB record sequence')
        records=np.frombuffer(binary,dtype=MAP_DTYPE)
        unique_colors=len(np.unique(records['rgb'],axis=0))
        map_report.update({'binary_points':len(records),'binary_unique_colors':unique_colors,'binary_bytes':len(binary)})
        check('observed_rgb_map_points',len(records)==metadata.get('point_count') and len(records)>=500
              and metadata.get('rgb_points')==len(records) and np.isfinite(records['xyz']).all(),map_report.copy())
        check('rgb_color_diversity',unique_colors>=8,{'unique_colors':unique_colors,'minimum':8,'interpretation':'camera RGB, not height/intensity colour'})
    except (OSError,ValueError) as error:
        check('observed_rgb_map_points',False,str(error));check('rgb_color_diversity',False,'actual map binary is missing or invalid')
    map_counts=metadata.get('counts',{})
    provenance=metadata.get('ground_truth_used') is False and metadata.get('reference_map_loaded') is False \
        and metadata.get('source_topic')=='/cloud_registered' and '/camera/image_color' in metadata.get('color_source','') \
        and metadata.get('frame_id')=='camera_init' and metadata.get('observed_rgb_samples',0)>=500 \
        and all(map_counts.get(k,0)>0 for k in ('camera','lidar','imu','odom','colored_cloud'))
    check('current_run_sensor_rgb_provenance',provenance,{'ground_truth_used':metadata.get('ground_truth_used'),
          'reference_map_loaded':metadata.get('reference_map_loaded'),'source_topic':metadata.get('source_topic'),
          'color_source':metadata.get('color_source'),'observed_rgb_samples':metadata.get('observed_rgb_samples'),'counts':map_counts})
    save=metadata.get('save',{})
    try:
        pcd=read_pcd(inside_run(directory,save.get('filename')))
        map_report['saved_pcd']=pcd
        check('map_saved_with_rgb',save.get('complete') is True and pcd['point_count']>=500
              and pcd['unique_colors']>=8 and pcd['point_count']==save.get('point_count'),{'save':save,'actual_pcd':pcd})
    except (OSError,ValueError,KeyError) as error:
        check('map_saved_with_rgb',False,{'save':save,'error':str(error)})
    saved_health=save.get('healthy_sensor_evidence',{})
    saved_ages=saved_health.get('ages',{})
    required_sources=('odom','lidar','imu','full_cloud','camera','colored_cloud')
    fresh_save=saved_health.get('slam_healthy') is True and saved_health.get('camera_healthy') is True \
        and saved_health.get('error') is None \
        and all(isinstance(saved_ages.get(source),(int,float)) and math.isfinite(saved_ages[source])
                and 0<=saved_ages[source]<2 for source in required_sources)
    check('map_saved_with_fresh_sensor_evidence',fresh_save,saved_health)
    check('map_not_capacity_truncated',metadata.get('capacity_rejections')==0,{'rejected_points':metadata.get('capacity_rejections')})
    runtime_evidence = full_control_runtime_evidence_check(directory)
    if runtime_evidence is not None:
        check('full_control_runtime_evidence', runtime_evidence['passed'], runtime_evidence)
    return {'schema_version':1,'run_id':mission.get('run_id',directory.name),'run_directory':str(directory),
        'evaluated_at':time.time(),'passed':all(c['passed'] for c in checks),'checks':checks,
        'failed_checks':[c['name'] for c in checks if not c['passed']], 'trajectory':trajectory,'endpoints':endpoints,
        'map':map_report,'obstacle_motion':obstacle_motion,'heading_alignment':heading_report,
        'ordered_waypoints':waypoints,'diagnostics':diagnostics,
        'scope':{'execution':'Gazebo physical Go2 / three platforms and ramps, not real hardware or stair certification',
                 'avoidance':'measured-cloud stop, wait for clearance, replan and resume; not a claim of lateral moving-obstacle bypass',
                 'trajectory_errors':('reported under one fixed initial alignment; same-time SLAM/truth preregistered region containment with measured dwell; center errors are diagnostic'
                                      if region_mode else 'reported for whole run and stages under one fixed alignment; endpoint acceptance threshold is 0.30 m'),
                 'color_evidence':'recorded live source metadata plus actual binary/PCD RGB payload; no preloaded reference map accepted'}}


def self_test():
    """Synthetic acceptance fixtures test the evaluator, never the robot demo."""
    results={}
    with tempfile.TemporaryDirectory(prefix='evaluate_go2_fixture_') as temporary:
        root=Path(temporary);case=root/'synthetic_fixture';case.mkdir()
        scenario={'floor_elevations':[0,1.2,2.4],
                  'exploration':[[0,0,0],[1/3,0,0],[1,0,0]],
                  'return_origin':[[1,0,0],[2/3,0,0],[0,0,0]],
                  'navigation_f1_f3':[[0,0,0],[0,7/3,.8],[0,14/3,1.6],[0,7,2.4]],
                  'waypoint_reference':'relative_initial_body',
                  'simulation':{'kind':'gazebo_physics','terrain':'ramps'}}
        mission={'run_id':case.name,'stage':'completed','message':'synthetic evaluator unit fixture',
            'origin':[0,0,0],'goal':[0,7,2.4],'completed_stages':STAGES,
            'events':[{'stage':s} for s in ['waiting_sensors']+STAGES+['completed']],
            'navigation':{'feedback_source':'/demo/slam/body_odom','frame_id':'camera_init',
                'counts':{'odom':100,'cloud':100,'bspline':10,'commands':100},'obstacle_stops':1,'obstacle_resumes':1},
            'validation':{'obstacle_hold_events':1,'resumed_after_hold':True,'body_contact_events':[],
                'max_abs_tilt_rad':0.,'body_contact_publisher_present':True,'obstacle_history':[
                    {'active':True,'position':[7.5,4,1.15],'visible_phase':'entering','gazebo_updates':1,'failed_updates':0,'stamp':10.},
                    {'active':True,'position':[7.5,2,1.15],'visible_phase':'blocking','gazebo_updates':11,'failed_updates':0,'stamp':12.},
                    {'active':True,'position':[7.5,1.8,1.15],'visible_phase':'leaving','gazebo_updates':12,'failed_updates':0,'stamp':13.},
                    {'active':True,'position':[7.5,0,1.15],'visible_phase':'clear','gazebo_updates':21,'failed_updates':0,'stamp':15.}]},
            'counts':{'body_contact_sensor':0},'obstacle':{'gazebo_updates':10,'failed_updates':0}}
        data=np.zeros(600,dtype=MAP_DTYPE)
        data['xyz']=np.column_stack([np.arange(600)*.1,np.arange(600)%7,np.arange(600)%3]);data['rgb'][:,0]=np.arange(600)%255;data['rgb'][:,1]=123
        (case/'colored_map.000001.bin').write_bytes(data.tobytes())
        packed=np.empty(600,dtype=[('xyz','<f4',(3,)),('rgb','<u4')]);packed['xyz']=data['xyz'];packed['rgb']=(data['rgb'][:,0].astype('u4')<<16)|(123<<8)
        (case/'colored_map.pcd').write_bytes(b'VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH 600\nHEIGHT 1\nPOINTS 600\nDATA binary\n'+packed.tobytes())
        meta={'run_id':case.name,'binary_filename':'colored_map.000001.bin','point_count':600,'rgb_points':600,
            'frame_id':'camera_init','ground_truth_used':False,'reference_map_loaded':False,'source_topic':'/cloud_registered',
            'color_source':'projection of /camera/image_color','observed_rgb_samples':600,'capacity_rejections':0,
            'counts':{k:100 for k in ['camera','lidar','imu','odom','colored_cloud']},
            'save':{'complete':True,'filename':'colored_map.pcd','point_count':600,
                'healthy_sensor_evidence':{'slam_healthy':True,'camera_healthy':True,'error':None,
                    'ages':{k:.1 for k in ('odom','lidar','imu','full_cloud','camera','colored_cloud')}}}}
        def write_mission(): (case/'mission.json').write_text(json.dumps(mission))
        write_mission();(case/'scenario.json').write_text(json.dumps(scenario));(case/'map_metadata.json').write_text(json.dumps(meta))
        r=Rotation.from_euler('z',.8);offset=np.array([10,-4,.315]);rows=[]
        stages=[('waiting_sensors',[0,0,0],[0,0,0]),('exploring',[0,0,0],[1,0,0]),('returning',[1,0,0],[0,0,0]),
                ('saving_map',[0,0,0],[0,0,0]),('navigating',[0,0,0],[0,7,2.4]),('completed',[0,7,2.4],[0,7,2.4])]
        for stage,a,b in stages:
            for p in np.linspace(a,b,10):
                t=10+len(rows)*.1
                rows.append({'source':'slam','stamp':t,'p':p.tolist(),'q':[0,0,0,1],'stage':stage})
                rows.append({'source':'truth','stamp':t,'p':(r.apply(p)+offset).tolist(),'q':r.as_quat().tolist(),'stage':stage})
        pose_path=case/'pose_audit.jsonl'
        pose_path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        baseline=evaluate(case);results['valid_synthetic_fixture_passes']=baseline['passed']
        results['known_rotation_and_translation_recovered']=np.allclose(baseline['trajectory']['rotation_world_from_slam'],r.as_matrix()) and np.allclose(baseline['trajectory']['translation_world_from_slam'],offset)
        results['every_prescribed_waypoint_verified_with_fixed_alignment']=baseline['ordered_waypoints']['passed'] \
            and all(point['arrival']['slam_error_m']<=.30 and point['arrival']['truth_error_m']<=.30
                    for stage in baseline['ordered_waypoints']['stages'].values() for point in stage['waypoints'])
        original_exploration=scenario['exploration']
        scenario['exploration']=[[0,0,0],[0,0,0]]
        (case/'scenario.json').write_text(json.dumps(scenario))
        repeated=evaluate(case)['ordered_waypoints']['stages']['exploring']['waypoints']
        results['even_repeated_waypoints_need_strictly_increasing_samples']=all(p['reached_in_order'] for p in repeated) \
            and repeated[1]['arrival']['stamp']>repeated[0]['arrival']['stamp']
        scenario['exploration']=[[0,0,0],[0,2,0],[1,0,0]]
        (case/'scenario.json').write_text(json.dumps(scenario))
        results['skipped_interior_waypoint_despite_correct_endpoint_rejected']='all_prescribed_waypoints_in_order' in evaluate(case)['failed_checks']
        scenario['exploration']=[[1,0,0],[0,0,0]]
        (case/'scenario.json').write_text(json.dumps(scenario))
        results['individually_reached_but_reverse_order_waypoints_rejected']='all_prescribed_waypoints_in_order' in evaluate(case)['failed_checks']
        scenario.pop('exploration');(case/'scenario.json').write_text(json.dumps(scenario))
        results['missing_required_stage_route_rejected']='all_prescribed_waypoints_in_order' in evaluate(case)['failed_checks']
        scenario['exploration']=[[.5,0,0]];(case/'scenario.json').write_text(json.dumps(scenario))
        nonjoint_rows=[]
        for row in rows:
            item=dict(row)
            if row['source']=='truth' and row['stage']=='exploring':
                item['p']=(np.asarray(row['p'])+r.apply([.7,0,0])).tolist()
            nonjoint_rows.append(item)
        pose_path.write_text(''.join(json.dumps(row)+'\n' for row in nonjoint_rows))
        results['slam_and_truth_arrival_at_different_times_rejected']='all_prescribed_waypoints_in_order' in evaluate(case)['failed_checks']
        scenario['exploration']=original_exploration;(case/'scenario.json').write_text(json.dumps(scenario))
        pose_path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        # New scene-axis reference: both SLAM trajectory and target are rotated
        # with the frozen IMU-derived heading. Truth fitting must not supply it.
        legacy_pose=pose_path.read_text()
        reference={'description':'synthetic CUSTOM/world heading fixture','world_quaternion':[0,0,0,1],
                   'body_imu_quaternion':[0,0,0,1]}
        scenario['waypoint_reference']='relative_world_axes'
        scenario['sensors']={'imu':{'body_rpy':[0,0,0],'orientation_reference':reference}}
        heading={'yaw_camera_init_from_world':-.8,'rotation_camera_init_from_world':r.inv().as_matrix().tolist(),
            'sample_count':10,'first_stamp':10.,'last_stamp':11.8,'max_stamp_difference_s':.005,
            'max_heading_spread_rad':0.,'max_gravity_residual_rad':0.,'ground_truth_used':False,
            'imu_reference_description':reference['description'],'imu_reference_world_quaternion':[0,0,0,1],
            'body_imu_quaternion':[0,0,0,1],
            'attitude_pairs':[{'slam_stamp':row['stamp'],'imu_stamp':row['stamp']+.005,
                'imu_quaternion':r.as_quat().tolist(),'slam_body_quaternion':row['q']}
                for row in rows if row['source']=='slam' and row['stage']=='waiting_sensors']}
        mission['heading_alignment']=dict(heading);mission['goal']=r.inv().apply([0,7,2.4]).tolist()
        rotated_rows=[]
        for row in rows:
            item=dict(row)
            item['p']=r.inv().apply(row['p']).tolist() if row['source']=='slam' else (r.inv().apply(np.array(row['p'])-offset)+offset).tolist()
            rotated_rows.append(item)
        pose_path.write_text(''.join(json.dumps(row)+'\n' for row in rotated_rows))
        (case/'scenario.json').write_text(json.dumps(scenario));write_mission()
        aligned_case=evaluate(case)
        results['frozen_world_axes_goal_and_independent_truth_pass']=aligned_case['passed']
        results['new_goal_is_not_unrotated_scenario_offset']=not np.allclose(mission['goal'],scenario['navigation_f1_f3'][-1]) \
            and aligned_case['endpoints']['floor3']['truth_error_m']<1e-9
        results['frozen_heading_used_for_every_stage_waypoint']=aligned_case['ordered_waypoints']['passed']
        mission['heading_alignment']={k:v for k,v in heading.items() if k!='attitude_pairs'};write_mission()
        results['missing_actual_heading_pairs_rejected']='frozen_scene_heading_calibration' in evaluate(case)['failed_checks']
        altered_pairs=[dict(pair) for pair in heading['attitude_pairs']]
        altered_pairs[0]['slam_body_quaternion']=Rotation.from_euler('z',.1).as_quat().tolist()
        mission['heading_alignment']={**heading,'attitude_pairs':altered_pairs};write_mission()
        results['calibration_pairs_inconsistent_with_recorded_poses_rejected']='frozen_scene_heading_calibration' in evaluate(case)['failed_checks']
        mission.pop('heading_alignment');write_mission()
        results['required_missing_calibration_rejected']='frozen_scene_heading_calibration' in evaluate(case)['failed_checks']
        mission['heading_alignment']={**heading,'ground_truth_used':True};write_mission()
        results['truth_derived_navigation_heading_rejected']='frozen_scene_heading_calibration' in evaluate(case)['failed_checks']
        mission['heading_alignment']={**heading,'last_stamp':20.};write_mission()
        results['calibration_after_motion_rejected']='frozen_scene_heading_calibration' in evaluate(case)['failed_checks']
        mission['heading_alignment']={**heading,'yaw_camera_init_from_world':0.};write_mission()
        results['inconsistent_frozen_matrix_and_yaw_rejected']='frozen_scene_heading_calibration' in evaluate(case)['failed_checks']
        mission['heading_alignment']=dict(heading);mission['goal']=[0,7,2.4];write_mission()
        results['ignoring_required_heading_rotation_rejected']='floor3_target_matches_scenario' in evaluate(case)['failed_checks']
        mission.pop('heading_alignment');scenario['waypoint_reference']='relative_initial_body';scenario.pop('sensors')
        (case/'scenario.json').write_text(json.dumps(scenario));pose_path.write_text(legacy_pose);write_mission()
        history=mission['validation'].pop('obstacle_history');write_mission()
        results['old_run_without_obstacle_history_rejected']='physical_dynamic_obstacle_motion' in evaluate(case)['failed_checks']
        mission['validation']['obstacle_history']=history
        stationary=[{**row,'position':[7.5,2,1.15]} for row in history]
        results['static_box_with_stop_resume_counters_rejected']=not obstacle_motion_evidence(stationary)[0]
        rejected=[{**row,'failed_updates':1 if i==1 else 0} for i,row in enumerate(history)]
        results['historical_failed_gazebo_update_rejected']=not obstacle_motion_evidence(rejected)[0]
        stale_ack=[{**row,'gazebo_updates':1} for row in history]
        results['position_changes_without_successful_updates_rejected']=not obstacle_motion_evidence(stale_ack)[0]
        no_clear=[{**row,'visible_phase':'blocking'} for row in history]
        results['blocking_without_clear_or_leaving_rejected']=not obstacle_motion_evidence(no_clear)[0]
        short=[{**row,'position':[7.5,2+.1*i,1.15]} for i,row in enumerate(history)]
        results['less_than_one_metre_motion_rejected']=not obstacle_motion_evidence(short)[0]
        write_mission()
        mission['stage']='failed';write_mission();results['failed_mission_rejected']=not evaluate(case)['passed'];mission['stage']='completed';write_mission()
        original=pose_path.read_text();pose_path.unlink();results['missing_pose_audit_rejected']='pose_audit_evidence' in evaluate(case)['failed_checks'];pose_path.write_text(original)
        missing_initial=[row for row in rows if row['stage']!='waiting_sensors']
        pose_path.write_text(''.join(json.dumps(row)+'\n' for row in missing_initial))
        results['alignment_without_initialization_trajectory_rejected']='single_alignment_from_initialization' in evaluate(case)['failed_checks']
        pose_path.write_text(original)
        drifted=[]
        for row in rows:
            row=dict(row)
            if row['source']=='slam' and row['stage'] in ('navigating','completed'): row['p']=[row['p'][0]+1,*row['p'][1:]]
            drifted.append(row)
        pose_path.write_text(''.join(json.dumps(row)+'\n' for row in drifted));drift=evaluate(case)
        results['stage_drift_not_realigned_away']=drift['trajectory']['by_stage']['navigating']['max_m']>.99
        results['false_completion_with_slam_off_goal_rejected']='floor3_slam_error' in drift['failed_checks']
        pose_path.write_text(original)
        mission['validation']['resumed_after_hold']=False;write_mission();results['missing_motion_resume_rejected']='dynamic_obstacle_stop_and_resume' in evaluate(case)['failed_checks']
        mission['validation']['resumed_after_hold']=True;mission['validation']['body_contact_publisher_present']=False;write_mission()
        results['silent_contact_topic_needs_publisher_evidence']='body_contact_monitor_available' in evaluate(case)['failed_checks']
        mission['validation']['body_contact_publisher_present']=True;mission['validation']['body_contact_events']=[{'pairs':['body','wall']}];write_mission()
        results['physical_body_contact_rejected']='no_body_contact_events' in evaluate(case)['failed_checks']
        data['rgb'][:]=128;(case/'colored_map.000001.bin').write_bytes(data.tobytes())
        results['single_color_map_rejected']='rgb_color_diversity' in evaluate(case)['failed_checks']
        meta['save']['healthy_sensor_evidence']['ages']['camera']=3.
        (case/'map_metadata.json').write_text(json.dumps(meta))
        results['saved_map_with_stale_camera_rejected']='map_saved_with_fresh_sensor_evidence' in evaluate(case)['failed_checks']
    print(json.dumps({'test_kind':'synthetic evaluator fixtures only','passed':all(results.values()),'checks':results},indent=2))
    return 0 if all(results.values()) else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',nargs='?')
    parser.add_argument('--contact-publisher-present',action='store_const',const=True,default=None,
                        help='Caller has independently checked the contact sensor publisher in this run; recorded in output')
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args()
    if args.self_test: return self_test()
    if not args.run_dir: parser.error('run_dir is required')
    directory=Path(args.run_dir).resolve()
    if not directory.is_dir(): parser.error('run_dir does not exist')
    result=evaluate(directory,args.contact_publisher_present)
    output=directory/'acceptance.json';temporary=directory/'acceptance.tmp'
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n');temporary.replace(output)
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {result['run_id']}")
    for row in result['checks']:
        print(f"  {'PASS' if row['passed'] else 'FAIL'} {row['name']}")
    if result['trajectory'].get('overall'):
        print('Trajectory under one fixed initial alignment:',json.dumps(result['trajectory']['overall']))
    for name,endpoint in result['endpoints'].items():
        if endpoint.get('center_errors_are_diagnostic'):
            error=endpoint.get('truth_error_m')
            print(f"{name} independent joint region arrival: {endpoint.get('joint_region_arrival',False)}; center error: {error if error is not None else 'missing'}")
        else:
            print(f"{name} independent truth error: {endpoint['truth_error_m']:.4f} m (limit {GOAL_TOLERANCE:.2f} m)")
    print(f'Evidence: {output}')
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
