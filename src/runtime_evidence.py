"""Current actual-source parking and RGB verification; no historical mission imports."""
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from parking_contract import MODE as ZERO_PARKING_MODE, qualification as zero_qualification
from parking_zero_contract import validate_zero_observation, validate_zero_ack
LEGACY_PARK_CHECKS = {'first_five_seconds', 'actual_slam_hold_drift', 'native_motion_limits', 'source_freshness', 'continuous_teacher_control', 'no_protection'}
PARK_CHECKS = {'first_five_seconds', 'actual_slam_pose_10hz_fixed_anchor', 'high_frequency_motion_diagnostic_only', 'source_freshness', 'continuous_teacher_control', 'no_protection'}
MAP_CHECKS = {'actual_saved_rgb_file', 'current_run_rgb_provenance', 'fresh_sensor_evidence', 'not_capacity_truncated'}

def raw_sha(raw):
    return hashlib.sha256(raw).hexdigest()

def json_bytes(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False) + '\n').encode()

def inside(run, value):
    p = Path(value)
    if not p.is_absolute():
        p = Path(run) / p
    p = p.resolve()
    if not p.is_relative_to(Path(run).resolve()) or not p.is_file():
        raise ValueError('Required actual evidence is missing or outside this run: ' + str(p))
    return p

def checked_json(run, value, expected_sha=None):
    p = inside(run, value)
    raw = p.read_bytes()
    if expected_sha is not None and raw_sha(raw) != expected_sha:
        raise ValueError('Actual evidence hash differs: ' + str(p))
    return (json.loads(raw), dict(file=str(p), sha256=raw_sha(raw), bytes=len(raw)))

def seal(run, basename, data):
    """Create an immutable small evidence document; refuse different rewrites."""
    p = Path(run) / basename
    raw = json_bytes(data)
    if p.exists():
        if p.read_bytes() != raw:
            raise ValueError('Refusing to replace sealed evidence: ' + str(p))
    else:
        with p.open('xb') as stream:
            stream.write(raw)
    return dict(source_evidence_file=str(p.resolve()), source_evidence_sha256=raw_sha(raw))

def receipt(run, schema, checks, evidence, **fields):
    return dict(schema=schema, run_id=Path(run).name, binding_verified=True, passed=True, checks={k: dict(status='passed', passed=True) for k in checks}, **evidence, **fields)

class FileTail:
    """Bounded incremental exact-line reader; partial lines stay unread."""

    def __init__(self, path, project=None):
        self.path = Path(path)
        self.offset = 0
        self.identity = None
        self.last_size = 0
        self.project = project

    def poll(self, max_bytes=2 * 1024 ** 2):
        if not self.path.exists():
            return []
        st = self.path.stat()
        identity = (st.st_dev, st.st_ino)
        if self.identity is not None and (identity != self.identity or st.st_size < self.offset):
            raise ValueError('Actual source was replaced/truncated: ' + str(self.path))
        self.identity = identity
        self.last_size = st.st_size
        with self.path.open('rb') as f:
            f.seek(self.offset)
            raw = f.read(max_bytes)
        end = raw.rfind(b'\n') + 1
        if not end:
            return []
        records = []
        cursor = self.offset
        for line in raw[:end].splitlines(keepends=True):
            full = json.loads(line)
            data = full if self.project is None else self.project(full)
            record = dict(data=data, source_file=str(self.path.resolve()), source_offset=cursor, source_length=len(line), source_line_sha256=raw_sha(line))
            if self.project is not None:
                record['source_projection_fields'] = list(data)
            records.append(record)
            cursor += len(line)
        self.offset += end
        return records

def verify_line(record):
    p = Path(record['source_file'])
    with p.open('rb') as f:
        f.seek(record['source_offset'])
        raw = f.read(record['source_length'])
    full = json.loads(raw)
    expected = full if 'source_projection_fields' not in record else {k: full.get(k) for k in record['source_projection_fields']}
    if raw_sha(raw) != record['source_line_sha256'] or expected != record['data']:
        raise ValueError('Exact serialized source-line binding failed')
NATIVE_FIELDS = ('world_sim_time', 'state_physics_world_time', 'body_lin_vel', 'body_ang_vel', 'rpy', 'qd', 'applied_torque', 'q_target', 'fault', 'actor_inferred_this_frame', 'body_clearance', 'contacts', 'command', 'command_expired', 'requested', 'ordinary_zero_before_actor_observation')

def native_projection(row):
    return {k: row.get(k) for k in NATIVE_FIELDS}

def status_projection(row):
    fields = ('request_id', 'state', 'waypoint_index', 'obstacle_hold', 'obstacle_stops', 'obstacle_resumes', 'aligned_obstacle_resumes', 'command', 'tilt_hold', 'accepted_trajectory_id', 'pose_age', 'cloud_age', 'raw_imu_age')
    d = {k: row.get(k) for k in fields}
    d['dynamic_stop_event'] = copy.deepcopy(row.get('dynamic_stop_event'))
    cascade = row.get('cascade_parking') or {}
    d['cascade_parking'] = {key: copy.deepcopy(cascade.get(key)) for key in ('mode', 'ordinary_parking', 'ordinary_parking_stage', 'ordinary_terminal_zero_event', 'fixed_goal', 'fixed_goal_sha256', 'path_sha256', 'failure_latched', 'protection_active', 'feedback_pose_stamp_ns', 'compute_clock_ns')}
    if 'terminal_reference' in cascade:
        from r105_terminal_reference import FIELDS
        d['cascade_parking'].update({key: copy.deepcopy(cascade.get(key)) for key in FIELDS})
    d['ordinary_terminal_failure_latched'] = copy.deepcopy(row.get('ordinary_terminal_failure_latched'))
    d['execution_bridge_safety'] = {'state': (row.get('execution_bridge_safety') or {}).get('state')}
    for field in ('known_scene_localization', 'known_scene_runtime_status', 'known_scene_scan_epoch'):
        d[field] = copy.deepcopy(row.get(field))
    return d

def native_ns(row):
    value = row.get('state_physics_world_time', row.get('world_sim_time'))
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('Native record has no original physical timestamp')
    return round(value * 1000000000.0)

def native_safe(row, standing=False):
    if row.get('fault') is not None or row.get('actor_inferred_this_frame') is not True:
        return False
    try:
        rpy = np.asarray(row['rpy'], dtype=float)
        v = np.asarray(row['body_lin_vel'], dtype=float)
        w = np.asarray(row['body_ang_vel'], dtype=float)
        qd = np.asarray(row['qd'], dtype=float)
        tau = np.asarray(row['applied_torque'], dtype=float)
        target = np.asarray(row['q_target'], dtype=float)
        if any((not np.isfinite(x).all() for x in (rpy, v, w, qd, tau, target))):
            return False
        if rpy.shape != (3,) or v.shape != (3,) or w.shape != (3,) or (qd.shape != (12,)) or (tau.shape != (12,)) or (target.shape != (12,)):
            return False
        if not math.isfinite(row['body_clearance']) or not math.isfinite(row['contacts']['body']) or max(abs(rpy[:2])) > 0.65 or (row['body_clearance'] < 0.18) or (row['contacts']['body'] > 0) or (max(abs(qd)) > 30.001) or (max(abs(tau)) > 23.50001):
            return False
        command = np.asarray(row['command'], dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            return False
        if standing and (np.linalg.norm(v[:2]) > 0.03 or np.linalg.norm(w) > 0.05 or sum((row['contacts'].get(k, 0) > 0 for k in ('FR', 'FL', 'RR', 'RL'))) < 2 or (np.linalg.norm(row['command']) > 1e-10)):
            return False
        return True
    except (ValueError, TypeError, KeyError):
        return False

def read_pcd(path):
    raw = Path(path).read_bytes()
    header, payload = raw.split(b'DATA binary\n', 1)
    fields = {}
    for line in header.decode('ascii').splitlines():
        if line and (not line.startswith('#')):
            parts = line.split()
            fields[parts[0]] = parts[1:]
    count = int(fields['POINTS'][0])
    if fields.get('FIELDS') != ['x', 'y', 'z', 'rgb'] or fields.get('SIZE') != ['4'] * 4 or fields.get('TYPE', [])[:3] != ['F'] * 3 or (fields['TYPE'][3] not in ('F', 'U')) or (len(payload) != count * 16):
        raise ValueError('Actual saved map PCD structure/count differs')
    points = np.frombuffer(payload, dtype=[('xyz', '<f4', (3,)), ('rgb', '<u4')])
    if not np.isfinite(points['xyz']).all():
        raise ValueError('Saved map contains invalid geometry')
    return dict(point_count=count, unique_colors=int(len(np.unique(points['rgb'] & 16777215))), file_sha256=raw_sha(raw), bytes=len(raw))

def verify_rgb(run, request_id, service_result, requested_epoch_s):
    if not service_result['success']:
        raise ValueError('Actual map save service failed: ' + service_result['message'])
    meta, meta_source = checked_json(run, 'map_metadata.json')
    save = meta.get('save', {})
    p = inside(run, save['filename'])
    if p.parent != Path(run).resolve():
        raise ValueError('Saved RGB PCD must belong to this actual run')
    stats = read_pcd(p)
    health = save.get('healthy_sensor_evidence', {})
    ages = health.get('ages', {})
    if meta.get('run_id') != Path(run).name or meta.get('frame_id') != 'camera_init' or meta.get('ground_truth_used') is not False or (meta.get('reference_map_loaded') is not False) or (meta.get('source_topic') != '/cloud_registered') or ('/camera/image_color' not in meta.get('color_source', '')) or (save.get('complete') is not True) or (save.get('saved_at', -math.inf) < requested_epoch_s) or (stats['point_count'] < 500) or (stats['point_count'] != save.get('point_count')) or (stats['unique_colors'] < 8) or (meta.get('observed_rgb_samples', 0) < 500) or (meta.get('capacity_rejections') != 0) or (health.get('error') is not None) or (health.get('slam_healthy') is not True) or (health.get('camera_healthy') is not True) or (not all((meta.get('counts', {}).get(k, 0) > 0 for k in ('camera', 'lidar', 'imu', 'odom', 'colored_cloud')))) or (not all((type(ages.get(k)) in (int, float) and math.isfinite(ages[k]) and (0 <= ages[k] < 2) for k in ('odom', 'lidar', 'imu', 'full_cloud', 'camera', 'colored_cloud')))):
        raise ValueError('Actual current-run RGB map/count/color/provenance/freshness gate failed')
    proof = dict(metadata=meta, metadata_source=meta_source, actual_pcd=stats, service_response=service_result, save_requested_epoch_s=requested_epoch_s, navigation_ground_truth_used=False)
    evidence = seal(run, 'mission46_rgb_save_evidence.json', proof)
    return receipt(run, 'teacher_mission46_rgb_save/v1', MAP_CHECKS, evidence, request_id=request_id, point_count=stats['point_count'], unique_colors=stats['unique_colors'], observed_rgb_samples=meta['observed_rgb_samples'], capacity_rejections=0, ground_truth_used=False, reference_map_loaded=False, source_topic=meta['source_topic'], color_source=meta['color_source'], saved_file=str(p), saved_file_sha256=stats['file_sha256'], sensor_ages_s=ages)

def bind_zero_qualification_sources(run, anchor, status_row, native_records):
    """Save exact actual sources preceding T0; never select a later zero frame."""
    event = anchor['event']
    proof = anchor['proof']
    clock = anchor['start_stamp_ns']
    if zero_qualification(status_row['data'], Path(run).name, event['request_id'], clock) != anchor:
        raise ValueError('Current callback does not carry the original T0 qualification')
    first = next((r for r in native_records if round(r['data']['world_sim_time'] * 1000000000.0) == proof['input_clock_ns']), None)
    if first is None:
        raise ValueError('Actual first zero input native frame is absent')
    verify_line(first)
    zero = validate_zero_observation(first['data'].get('ordinary_zero_before_actor_observation'), event, proof['input_clock_ns'])
    snapshot, snapshot_source = checked_json(run, 'closed_loop_executor_ack.json')
    ack = next((r for r in snapshot['entries'] if r.get('stamp_ns') == proof['input_clock_ns'] and r.get('sequence') == proof['ack_sequence']), None)
    if ack is None:
        raise ValueError('Actual first zero ACK is absent from live causal snapshot')
    checked = validate_zero_ack(ack, event, clock)
    if any((proof.get(k) != v for k, v in checked.items())) or ack['ordinary_zero_before_actor_observation'] != zero:
        raise ValueError('Original T0 proof does not match actual native input and actual ACK')
    if first['data'].get('command') != [0.0, 0.0, 0.0] or first['data'].get('requested') != [0.0, 0.0, 0.0]:
        raise ValueError('First actual zero input/request differs')
    preserved = seal(run, 'mission46_first_zero_ACK_snapshot.json', snapshot)
    return seal(run, 'mission46_first_zero_qualification_sources.json', dict(qualification_status=copy.deepcopy(status_row), first_input_native=copy.deepcopy(first), actual_ACK=copy.deepcopy(ack), preserved_ACK_snapshot=preserved, live_ACK_snapshot_at_read=snapshot_source, navigation_ground_truth_used=False))

def verify_zero_qualification_sources(run, anchor):
    sources, source_receipt = checked_json(run, 'mission46_first_zero_qualification_sources.json')
    row = sources['qualification_status']
    event = anchor['event']
    clock = anchor['start_stamp_ns']
    if zero_qualification(row['data'], Path(run).name, event['request_id'], clock) != anchor:
        raise ValueError('Independent actual T0 qualification replay differs')
    tail = FileTail(Path(run) / 'mission46_actual_navigation_statuses.jsonl')
    found = None
    while True:
        records = tail.poll()
        if not records:
            break
        found = next((r for r in records if r['data'] == row), None)
        if found is not None:
            break
    if found is None:
        raise ValueError('Original qualification callback source is absent')
    verify_line(found)
    native = sources['first_input_native']
    verify_line(native)
    zero = validate_zero_observation(native['data'].get('ordinary_zero_before_actor_observation'), event, round(native['data']['world_sim_time'] * 1000000000.0))
    saved = sources['preserved_ACK_snapshot']
    snapshot, _ = checked_json(run, saved['source_evidence_file'], saved['source_evidence_sha256'])
    ack = sources['actual_ACK']
    checked = validate_zero_ack(ack, event, clock)
    if ack not in snapshot['entries'] or ack['ordinary_zero_before_actor_observation'] != zero or any((anchor['proof'].get(k) != v for k, v in checked.items())) or (native['data'].get('command') != [0.0, 0.0, 0.0]) or (native['data'].get('requested') != [0.0, 0.0, 0.0]):
        raise ValueError('Actual T0 native/ACK source tuple differs')
    return dict(sources=source_receipt, original_qualification_status_line=found)

def verify_pose10hz_parking(run, request_id, start_ns, pose_rows, native_records, status_rows, origin, anchor, qualification_sources, evidence_basename):
    """Same pose-only grid as the live owner; native motion is diagnostic only."""
    from parking_pose10hz import PoseGrid, SOURCE_AGE_NS, committed_poses
    end = start_ns + 5000000000
    grid = PoseGrid(start_ns, origin)
    poses = sorted((r for r in pose_rows if origin['stamp_ns'] <= r['stamp_ns'] <= end + SOURCE_AGE_NS), key=lambda r: r['stamp_ns'])
    source_lines = dict(source='actual independent SLAM callbacks', original_callback_count=len(poses), required_grid_end_ns=end, derived_commit_identity_gate=False)
    poses = [r for r in poses if r['stamp_ns'] > origin['stamp_ns']]
    for row in poses:
        if not 0 <= row['received_sim_ns'] - row['stamp_ns'] <= SOURCE_AGE_NS or not 0 <= row['received_wall_ns'] - row['source_received_wall_ns'] <= SOURCE_AGE_NS:
            raise ValueError('Parking 10Hz data unavailable: original source expired')
        grid.observe(row)
    sampled = grid.evidence()
    if not sampled['data_available']:
        raise ValueError('Parking 10Hz data unavailable: missing complete fixed-window samples')
    if sampled['motion_failure']:
        raise ValueError('Parking 10Hz fixed-anchor displacement/yaw exceeded')
    statuses = [r for r in status_rows if start_ns <= r['received_sim_ns'] <= end]
    if not statuses:
        raise ValueError('Parking control evidence unavailable')
    stamps = [r['received_sim_ns'] for r in statuses]
    if stamps[0] - start_ns > SOURCE_AGE_NS or end - stamps[-1] > SOURCE_AGE_NS or any((b - a > SOURCE_AGE_NS for a, b in zip(stamps, stamps[1:]))):
        raise ValueError('Parking zero-command/status source evidence unavailable')
    generations = set()
    assisted = json.loads((Path(run) / 'navigation_profile.json').read_text()).get('known_scene_localization_contract', {}).get('enabled')
    for row in statuses:
        d = row['data']
        cascade = d.get('cascade_parking') or {}
        owner = cascade.get('ordinary_parking') or {}
        if d.get('state') != 'succeeded' or d.get('request_id') != request_id or cascade.get('mode') != 'ordinary_zero_hold' or cascade.get('protection_active') or cascade.get('failure_latched') or (d.get('ordinary_terminal_failure_latched') is not None) or owner.get('failure') or (cascade.get('ordinary_terminal_zero_event') != anchor['event']) or d.get('tilt_hold') or d.get('obstacle_hold') or ((d.get('execution_bridge_safety') or {}).get('state') != 'ready') or (d.get('command') != [0.0, 0.0, 0.0]):
            raise ValueError('Parking independent control/source/protection fault')
        if assisted:
            m = d.get('known_scene_localization') or {}
            s = d.get('known_scene_runtime_status') or {}
            e = d.get('known_scene_scan_epoch') or {}
            if not m.get('control_allowed') or m.get('correction_stale') or m.get('correction_transition') or (not s.get('initialized')) or s.get('correction_transition') or (not e.get('ready')) or (not m.get('generation') == s.get('generation') == e.get('generation')):
                raise ValueError('Parking source coordinate generation unavailable')
            generations.add(m['generation'])
    if assisted and len(generations) != 1:
        raise ValueError('Parking source coordinate generation changed')
    natives = [r for r in native_records if start_ns <= native_ns(r['data']) <= end]
    for record in natives:
        verify_line(record)
        d = record['data']
        if not native_safe(d) or d.get('command_expired') is not False:
            raise ValueError('Independent native physical/control fault during parking')
        if d.get('command') != [0.0, 0.0, 0.0] or d.get('requested') != [0.0, 0.0, 0.0]:
            raise ValueError('Parking actual Actor input/request was not exact zero')
        validate_zero_observation(d.get('ordinary_zero_before_actor_observation'), anchor['event'], round(d['world_sim_time'] * 1000000000.0))
    diagnostics = dict(native_samples=len(natives), native_planar_speed_max_mps=max((float(np.linalg.norm(r['data']['body_lin_vel'][:2])) for r in natives), default=None), native_wz_max_radps=max((abs(r['data']['body_ang_vel'][2]) for r in natives), default=None), used_for_parking_decision=False)
    metrics = dict(slam_pose10hz_xy_drift_max_m=sampled['max_xy_from_anchor_m'], slam_pose10hz_yaw_drift_max_rad=sampled['max_abs_yaw_from_anchor_rad'], adjacent_xy_max_m=sampled['max_adjacent_xy_m'], adjacent_yaw_abs_max_rad=sampled['max_abs_adjacent_yaw_rad'])
    proof = dict(schema='actual_pose10hz_fixed5_parking_evidence/v1', interval=[start_ns, end], samples_10hz=sampled, actual_slam_poses=poses, actual_controller_statuses=statuses, actual_slam_source_file='mission46_actual_slam_poses.jsonl', actual_source_description=source_lines, native_source_records=natives, native_motion_diagnostic=diagnostics, metrics=metrics, origin=origin, T0_origin_anchor=anchor, original_T0_source_replay=qualification_sources, parking_contract=ZERO_PARKING_MODE, navigation_ground_truth_used=False, high_frequency_motion_gate=False, independent_hard_fault_protection_retained=True)
    evidence = seal(run, evidence_basename, proof)
    return receipt(run, 'teacher_mission46_final_parking/v1', PARK_CHECKS, evidence, request_id=request_id, start_stamp_ns=start_ns, end_stamp_ns=end, metrics=metrics, parking_contract=ZERO_PARKING_MODE)

def verify_parking(run, request_id, start_ns, pose_rows, native_records, status_rows, origin, *, parking_contract='legacy_arrival_fixed5', anchor=None, evidence_basename='mission46_final_parking_evidence.json'):
    ordinary = parking_contract == ZERO_PARKING_MODE
    if ordinary:
        actual_anchor, anchor_receipt = checked_json(run, 'mission46_parking_T0_anchor.json')
        if actual_anchor.get('anchor') != anchor or anchor['start_stamp_ns'] != start_ns or actual_anchor.get('run_id') != Path(run).name or (actual_anchor.get('request_id') != request_id) or (origin != anchor['origin']):
            raise ValueError('First ordinary zero T0/origin binding differs')
        qualification_sources = verify_zero_qualification_sources(run, anchor)
        if anchor['proof'].get('parking_rule') == 'ordinary_pose10hz_zero_ack_fixed5/v1':
            return verify_pose10hz_parking(run, request_id, start_ns, pose_rows, native_records, status_rows, origin, anchor, qualification_sources, evidence_basename)
    end = start_ns + 5000000000
    poses = [r for r in pose_rows if start_ns <= r['stamp_ns'] <= end]
    natives = [r for r in native_records if start_ns <= native_ns(r['data']) <= end]
    statuses = [r for r in status_rows if start_ns <= r['received_sim_ns'] <= end]
    if len(poses) < 25 or len(natives) < 200 or len(statuses) < 20:
        raise ValueError('First5s hold has insufficient actual pose/native/status records')
    profile_path = Path(run) / 'navigation_profile.json'
    assisted = profile_path.exists() and json.loads(profile_path.read_text()).get('known_scene_localization_contract', {}).get('enabled')
    if assisted:
        generations = set()
        for row in statuses:
            data = row['data']
            m = data.get('known_scene_localization') or {}
            s = data.get('known_scene_runtime_status') or {}
            e = data.get('known_scene_scan_epoch') or {}
            if not m.get('control_allowed') or m.get('correction_stale') or m.get('correction_transition') or (not s.get('initialized')) or s.get('correction_transition') or (not e.get('ready')) or (not m.get('generation') == s.get('generation') == e.get('generation')):
                raise ValueError('Known-scene parking requires a fresh committed pose/cloud/planner epoch, never a correction hold')
            generations.add(m['generation'])
        if len(generations) != 1:
            raise ValueError('Parking5s crosses a localization correction epoch')

    def cover(stamps, gap):
        return stamps[0] - start_ns <= gap and end - stamps[-1] <= gap and np.all(np.diff(stamps) > 0) and (max(np.diff(stamps)) <= gap)
    if not cover([r['stamp_ns'] for r in poses], 200000000) or not cover([native_ns(r['data']) for r in natives], 30000000):
        raise ValueError('Actual first5s parking source coverage/gap failed')
    p = np.asarray([r['position'] for r in poses])
    q = np.asarray([r['quaternion_xyzw'] for r in poses])
    base = np.asarray(origin['position'])
    yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    q0 = origin['quaternion_xyzw']
    yaw0 = math.atan2(2 * (q0[3] * q0[2] + q0[0] * q0[1]), 1 - 2 * (q0[1] ** 2 + q0[2] ** 2))
    xy = float(np.max(np.linalg.norm(p[:, :2] - base[:2], axis=1)))
    yd = float(np.max(np.abs(np.arctan2(np.sin(yaw - yaw0), np.cos(yaw - yaw0)))))
    vv = max((float(np.linalg.norm(r['data']['body_lin_vel'][:2])) for r in natives))
    wz = max((abs(r['data']['body_ang_vel'][2]) for r in natives))
    for r in natives:
        verify_line(r)
    angular = np.unwrap([r['data']['rpy'][2] for r in natives])
    ts = np.array([native_ns(r['data']) for r in natives]) / 1000000000.0
    yawdot = float(np.max(np.abs(np.diff(angular) / np.diff(ts))))
    if xy > 0.05 or yd > 0.1 or vv > 0.08 or (wz > 0.1) or (yawdot > 0.1) or any((not native_safe(r['data']) or r['data'].get('command_expired') is not False for r in natives)) or any((r['data'].get('state') != 'succeeded' or r['data'].get('request_id') != request_id or (r['data'].get('cascade_parking') or {}).get('mode') != ('ordinary_zero_hold' if ordinary else 'active_hold') or (ordinary and r['data'].get('ordinary_terminal_failure_latched') is not None) or r['data'].get('tilt_hold') or r['data'].get('obstacle_hold') or ((r['data'].get('execution_bridge_safety') or {}).get('state') != 'ready') for r in statuses)) or any((not 0 <= r['received_sim_ns'] - r['stamp_ns'] <= 300000000 or not 0 <= r['received_wall_ns'] - r['source_received_wall_ns'] <= 300000000 for r in poses)):
        raise ValueError('Actual first5s parking drift/native/source/protection gate failed')
    if ordinary:
        if any((r['data'].get('command') != [0.0, 0.0, 0.0] or r['data'].get('requested') != [0.0, 0.0, 0.0] for r in natives)):
            raise ValueError('Ordinary first5s actual Teacher velocity input/request was not exact zero')
        event = anchor['event']
        for row in statuses:
            cascade = row['data'].get('cascade_parking') or {}
            owner = cascade.get('ordinary_parking') or {}
            if cascade.get('ordinary_terminal_zero_event') != event or owner.get('failure') or owner.get('stage') not in ('handoff_zero', 'zero_hold') or cascade.get('protection_active') or cascade.get('failure_latched'):
                raise ValueError('Ordinary zero owner/event changed or became protected during fixed5s')
        for row in natives:
            zero = row['data'].get('ordinary_zero_before_actor_observation')
            validate_zero_observation(zero, event, round(row['data']['world_sim_time'] * 1000000000.0))
    proof = dict(interval=[start_ns, end], actual_slam_poses=poses, native_source_records=natives, actual_controller_statuses=statuses, origin=origin, metrics=dict(slam_xy_drift_max_m=xy, slam_yaw_drift_max_rad=yd, native_planar_speed_max_mps=vv, native_wz_max_radps=wz, native_yaw_dot_max_radps=yawdot), navigation_ground_truth_used=False, native_use='offline parking safety only')
    proof.update(parking_contract=parking_contract, T0_origin_anchor=anchor if ordinary else None, original_T0_source_replay=qualification_sources if ordinary else None)
    evidence = seal(run, evidence_basename, proof)
    return receipt(run, 'teacher_mission46_final_parking/v1', LEGACY_PARK_CHECKS, evidence, request_id=request_id, start_stamp_ns=start_ns, end_stamp_ns=end, metrics=proof['metrics'], parking_contract=parking_contract)

def observe_native_tail(tail, previous_ns, maximum_chunks=4):
    """Bounded catch-up of original complete lines; no success over backlog."""
    latest = previous_ns
    fault = None
    chunks = 0
    for _ in range(maximum_chunks):
        records = tail.poll()
        chunks += 1
        for record in records:
            latest = native_ns(record['data'])
            if record['data'].get('fault') is not None and fault is None:
                fault = record
        if fault is not None:
            break
        if tail.path.exists() and tail.offset == tail.path.stat().st_size:
            break
        if not records:
            break
    caught_up = tail.path.exists() and tail.offset == tail.path.stat().st_size
    return dict(native_ns=latest, fault_record=fault, caught_up=caught_up, chunks=chunks, complete_source_bytes_checked=tail.offset)
