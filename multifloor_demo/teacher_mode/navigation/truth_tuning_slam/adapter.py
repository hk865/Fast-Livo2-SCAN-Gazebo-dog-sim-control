#!/usr/bin/env python3
"""Independent frozen-Controller binding to actual SLAM/IMU, fixed map route.

No Gazebo state/service, SCAN, joint command, or training API is imported.
The JSON consumer below is deliberately a new command contract; it cannot be
accepted as an old scan_slam envelope. Only the runner owns actuator integration.
"""
from __future__ import annotations

import argparse
from collections import deque
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time

import numpy as np

sys.dont_write_bytecode = True

SCHEMA = 'teacher_slam_fixed_route_command/v1'
TIMEOUT_S = .3
FUTURE_S = .05


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(obj):
    def numeric(value):
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError('Unsupported evidence value: '+type(value).__name__)
    return json.dumps(obj, sort_keys=True, separators=(',', ':'), allow_nan=False, default=numeric)


def rotation_xyzw(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .98 <= float(q @ q) <= 1.02:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def finite(values, width):
    a = np.asarray(values, dtype=float)
    if a.shape != (width,) or not np.isfinite(a).all():
        raise ValueError('Invalid finite vector')
    return a


def tilt(R):
    return max(abs(math.atan2(R[2, 1], R[2, 2])), abs(math.asin(float(np.clip(-R[2, 0], -1, 1)))))


def fresh(stamp_ns, received_wall, clock_ns, wall):
    return -FUTURE_S <= (clock_ns-stamp_ns)/1e9 <= TIMEOUT_S and 0 <= wall-received_wall <= TIMEOUT_S


def checked_file(config_path, item):
    path = Path(item['path'])
    if not path.is_absolute():
        path = config_path.parent / path
    path = path.resolve()
    expected = item['sha256']
    if not isinstance(expected, str) or len(expected) != 64 or sha(path) != expected:
        raise ValueError('Frozen input SHA mismatch: '+str(path))
    return path


def load_binding(config_path):
    """Validate original profile, explicit map registration and frozen source."""
    config_path = Path(config_path).resolve()
    c = json.loads(config_path.read_text())
    if c.get('schema') != 'teacher_slam_fixed_route_binding/v1' or c.get('simulation_only') is not True:
        raise ValueError('Only reviewed simulation-only bindings are admitted')
    core_path = checked_file(config_path, c['controller_source'])
    profile_path = checked_file(config_path, c['controller_profile'])
    scenario_path = checked_file(config_path, c['sensor_scenario'])
    if core_path.name != 'core.py':
        raise ValueError('Controller must be the explicitly archived core.py')
    p = json.loads(profile_path.read_text())
    spec = importlib.util.spec_from_file_location('frozen_slam_route_core_'+sha(core_path)[:12], core_path)
    core = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(core)
    s = json.loads(scenario_path.read_text())['sensors']['imu']
    imu_reference = s['orientation_reference']
    R_bi = rotation_xyzw(imu_reference['body_imu_quaternion'])
    R_wi = rotation_xyzw(imu_reference['world_quaternion'])
    route = c['fixed_route']
    xyz = np.asarray(route['points_xyz'], dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) < 2 or not np.isfinite(xyz).all():
        raise ValueError('Fixed map route must contain finite xyz points')
    registration = route['registration']
    if registration['source'] not in ('explicit_camera_init_route', 'actual_SLAM_map_registration', 'actual_SLAM_initial_body_relative_route'):
        raise ValueError('Route registration may not use live simulator truth or initial body yaw')
    if not isinstance(registration.get('evidence'), str) or not registration['evidence'].strip():
        raise ValueError('Document the actual map coordinate/registration evidence')
    relative = registration['source'] == 'actual_SLAM_initial_body_relative_route'
    if relative:
        if route['frame_id'] != 'initial_SLAM_body_horizontal' or np.linalg.norm(xyz[0]) > 1e-10:
            raise ValueError('Relative SLAM route must start at the measured body origin')
        if 'T_camera_init_from_route_frame' in registration:
            raise ValueError('Relative initial SLAM route must not have a truth-derived transform')
    elif route['frame_id'] != 'camera_init':
        T = np.asarray(registration['T_camera_init_from_route_frame'], dtype=float)
        if (T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-10)
            or not np.allclose(T[:3, :3].T@T[:3, :3], np.eye(3), atol=1e-8)
            or not math.isclose(np.linalg.det(T[:3, :3]), 1., abs_tol=1e-8)):
            raise ValueError('Map registration must be a frozen rigid SE3 transform')
        xyz = xyz@T[:3, :3].T+T[:3, 3]
    elif 'T_camera_init_from_route_frame' in registration:
        raise ValueError('Do not ambiguously transform an already camera_init route')
    effective = copy.deepcopy(p)
    effective['route_world_xyz'] = xyz.tolist()
    # Route substitution is explicit. Numeric gains, limits and selected rate
    # are preserved; inertial coordinates mean camera_init in this binding.
    finite(effective['base_com_offset'], 3)
    selected_hz = int(effective['feedback_hz'])
    if selected_hz > 10 and c.get('allow_reduced_source_frequency') is not True:
        raise ValueError('Actual SLAM is 10 Hz: explicitly review reduction of a >10 Hz calibration')
    if c.get('pose_topic', '/demo/slam/body_odom') != '/demo/slam/body_odom':
        raise ValueError('This adapter accepts only audited body-origin SLAM odometry')
    if c.get('imu_topic', '/livox/imu') != '/livox/imu' or c.get('imu_frame', s['frame']) != s['frame']:
        raise ValueError('IMU topic/frame must match the actual sensor scenario')
    if not 0 < float(c.get('maximum_gyro_pair_gap_s', .02)) <= .05:
        raise ValueError('Gyro pairing must remain causal and within 50 ms')
    if c.get('feedback_mode', 'mapped_slam') != 'mapped_slam':
        raise ValueError('IMU-propagated pose is not qualified by this mapped-SLAM adapter')
    if not 0 < float(c.get('maximum_tilt_rad', .65)) <= .65:
        raise ValueError('Invalid actual-attitude protection bound')
    core.Controller(effective)  # Exercise the frozen controller's own validation.
    receipt = {'schema': 'teacher_slam_fixed_route_source_receipt/v1',
               'config_path': str(config_path), 'config_sha256': sha(config_path),
               'adapter_sha256': sha(__file__), 'controller_source': str(core_path),
               'controller_sha256': sha(core_path), 'profile_source': str(profile_path),
               'profile_sha256': sha(profile_path), 'sensor_scenario': str(scenario_path),
               'sensor_scenario_sha256': sha(scenario_path), 'effective_profile': effective,
               'effective_profile_sha256': hashlib.sha256(canonical(effective).encode()).hexdigest(),
               'route_registration': registration, 'route_pending_actual_SLAM_anchor': relative, 'controller_frame': 'camera_init',
               'body_com_offset': effective['base_com_offset'],
               'body_from_imu_rotation': R_bi.tolist(), 'imu_orientation_reference_rotation': R_wi.tolist(),
               'selected_calibration_feedback_hz': selected_hz, 'actual_source_expected_hz': 10,
               'ground_truth_navigation': False, 'actual_validation': 'unverified'}
    return c, core, effective, R_bi, R_wi, receipt


class FreshBinding:
    """Pure input/state binding. Timer heartbeats never advance controller dwell."""
    def __init__(self, c, core, profile, R_bi, R_wi, receipt):
        self.c, self.profile, self.receipt = c, profile, receipt
        self.controller = core.Controller(profile)
        self.R_bi, self.R_wi = R_bi, R_wi
        self.gyros = deque(maxlen=512)
        self.pose = None
        self.last_pose_stamp = self.last_imu_stamp = -1
        self.clock_ns = None
        self.clock_wall = None
        self.last_clock_advance_wall = None
        self.first_source = None
        self.last_control_source = None
        self.last_used_source = None
        self.last_mode = 'waiting'
        self.command = np.zeros(3)
        self.fault = None
        self.hold = True
        self.recovery_stamps = deque(maxlen=2)
        self.sequence = self.update_count = 0
        self.update_stamps = []
        self.route_anchor = None
        self.completed_pose_stamp_ns = None

    def clock(self, ns, wall):
        if not isinstance(ns, int) or ns < 0:
            raise ValueError('Invalid original clock integer')
        if self.clock_ns is not None and ns < self.clock_ns:
            self.fault = 'clock_reset'; return
        if ns != self.clock_ns:
            self.last_clock_advance_wall = wall
        self.clock_ns, self.clock_wall = ns, wall

    def imu(self, row):
        stamp = row['stamp_ns']
        if row['frame_id'] != self.c.get('imu_frame', 'imu_link'):
            raise ValueError('Unexpected original IMU frame')
        if stamp == self.last_imu_stamp:
            return False
        if stamp < self.last_imu_stamp:
            self.fault = 'imu_header_backwards'; return False
        # sensor_msgs/Imu can validly provide gyro without an orientation
        # estimate. A covariance[0] of -1 forbids treating its quaternion as
        # an attitude measurement; body SLAM still supplies the tilt guard.
        orientation_available = row.get('orientation_available') is True
        R = rotation_xyzw(row['quaternion_xyzw']) if orientation_available else None
        gyro = self.R_bi@finite(row['angular_velocity_sensor'], 3)
        if not fresh(stamp, row['received_monotonic_wall'], row['callback_ros_clock_ns'], row['received_monotonic_wall']):
            raise ValueError('IMU original header is stale/future')
        item = {**row, 'gyro_body': gyro.tolist(),
                'tilt_rad': tilt(self.R_wi@R@self.R_bi.T) if R is not None else None,
                'attitude_source': 'declared_IMU_orientation' if R is not None else 'orientation_unavailable; tilt_from_SLAM_body_only'}
        self.gyros.append(item)
        self.last_imu_stamp = stamp
        return True

    def odom(self, row):
        stamp = row['stamp_ns']
        if row['frame_id'] != 'camera_init' or row['child_frame_id'] != 'demo_slam_body':
            raise ValueError('Pose must be actual body-origin camera_init odometry')
        if stamp == self.last_pose_stamp:
            return False
        if stamp < self.last_pose_stamp:
            self.fault = 'pose_header_backwards'; return False
        R = rotation_xyzw(row['quaternion_xyzw'])
        finite(row['position'], 3); finite(row['origin_linear_velocity_body'], 3)
        cov = finite(row['twist_covariance_diagonal'], 6)
        if np.any(cov >= 1e5):
            raise ValueError('Body odom finite-difference twist is not valid yet')
        if not fresh(stamp, row['received_monotonic_wall'], row['callback_ros_clock_ns'], row['received_monotonic_wall']):
            raise ValueError('SLAM original header is stale/future')
        self.pose = {**row, 'rotation': R.tolist(), 'tilt_rad': tilt(R)}
        self.last_pose_stamp = stamp
        self.recovery_stamps.append(stamp)
        return True

    def protect(self, reason):
        self.command[:] = 0
        self.controller.command[:] = 0
        if not self.hold:
            self.recovery_stamps.clear()
            x = self.controller
            x.integral[:] = 0.; x.velocity_integral[:] = 0.
            x.filtered = x.inner_filtered = None
            x.last_feedback = x.last_wall = None
            x.dwell = None; x.stop_since = None
            # A resumed controller measures stopping anew before any turn.
            if x.turn_phase is not None:
                x.turn_phase = 'pre_turn'
            x.last_mode = None
        self.hold = True
        self.last_mode = reason

    def tick(self, wall, graph_ok=True):
        reason = self.fault
        ns = self.clock_ns
        if reason is None and (ns is None or self.clock_wall is None or wall-self.clock_wall > TIMEOUT_S
                               or self.last_clock_advance_wall is None or wall-self.last_clock_advance_wall > TIMEOUT_S):
            reason = 'clock_unavailable_or_paused'
        if reason is None and not graph_ok:
            reason = 'publisher_graph_unverified_or_conflicting'
        if reason is None and (self.pose is None or not fresh(self.pose['stamp_ns'], self.pose['received_monotonic_wall'], ns, wall)):
            reason = 'SLAM_pose_stale_or_missing'
        gyro = None
        if reason is None:
            pose_stamp = self.pose['stamp_ns']
            gyro = next((g for g in reversed(self.gyros) if g['stamp_ns'] <= pose_stamp), None)
            gap = self.c.get('maximum_gyro_pair_gap_s', .02)
            if gyro is None or (pose_stamp-gyro['stamp_ns'])/1e9 > gap or not fresh(gyro['stamp_ns'], gyro['received_monotonic_wall'], ns, wall):
                reason = 'causal_IMU_pair_stale_or_missing'
            elif max(self.pose['tilt_rad'], gyro['tilt_rad'] if gyro['tilt_rad'] is not None else self.pose['tilt_rad']) > self.c.get('maximum_tilt_rad', .65):
                self.fault = reason = 'actual_attitude_limit_latched'
        if reason is None and self.hold:
            if len(self.recovery_stamps) < 2 or (self.recovery_stamps[-1]-self.recovery_stamps[0])/1e9 < .099999:
                reason = 'waiting_two_distinct_SLAM_headers'
            else:
                self.hold = False
        updated = False
        core_row = None
        if reason is not None:
            self.protect(reason)
        else:
            stamp = self.pose['stamp_ns']
            period_ns = round(1e9/min(10, int(self.profile['feedback_hz'])))
            due = self.last_control_source is None or stamp-self.last_control_source >= period_ns-2
            if stamp != self.last_used_source and due:
                if self.first_source is None:
                    self.first_source = stamp
                    if self.receipt['route_pending_actual_SLAM_anchor']:
                        R = np.asarray(self.pose['rotation'])
                        yaw = math.atan2(R[1, 0], R[0, 0])
                        c, s = math.cos(yaw), math.sin(yaw)
                        Rh = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                        relative = np.asarray(self.c['fixed_route']['points_xyz'])
                        actual_route = relative@Rh.T+np.asarray(self.pose['position'])
                        self.controller.route = actual_route.copy()
                        self.route_anchor = {'schema': 'teacher_fixed_route_actual_SLAM_anchor/v1',
                            'source': '/demo/slam/body_odom', 'original_stamp_ns': stamp,
                            'original_position_camera_init': self.pose['position'],
                            'original_quaternion_xyzw': self.pose['quaternion_xyzw'],
                            'received_monotonic_wall': self.pose['received_monotonic_wall'],
                            'heading_rad': yaw, 'horizontal_rotation': Rh.tolist(),
                            'relative_points_xyz': relative.tolist(), 'fixed_route_camera_init_xyz': actual_route.tolist(),
                            'navigation_ground_truth_used': False, 'route_reference_changes_after_anchor': 0,
                            'limitation': 'Body-relative measured SLAM route; not a registered Gazebo fixture centerline'}
                # Frozen core's truth ABI subtracts 5 ms. This ONLY neutralizes
                # that convention: its feedback_time is the original SLAM stamp.
                state = np.zeros(64)
                state[0] = stamp/1e9+.005
                state[1:4] = self.pose['position']
                q = self.pose['quaternion_xyzw']; state[4:8] = [q[3], *q[:3]]
                omega = np.asarray(gyro['gyro_body'])
                state[11:14] = omega
                state[8:11] = np.asarray(self.pose['origin_linear_velocity_body'])+np.cross(omega, self.controller.com_offset)
                elapsed = (stamp-self.first_source)/1e9
                if self.last_control_source is not None and stamp-self.last_control_source > 200000001:
                    self.controller.dwell = None; self.controller.stop_since = None
                # Emulate omitted 50Hz hold frames without computing on cached
                # pose. The core is invoked once per selected NEW source header.
                old = self.last_control_source
                n = 1 if old is None else max(1, round((stamp-old)/20e6))
                for _ in range(min(n-1, 15)):
                    e = self.controller.applied_command_estimate
                    e += np.clip(self.controller.command-e, -np.array([.6, .6, .8])*.02, np.array([.6, .6, .8])*.02)
                self.controller.frame = self.update_count*self.controller.stride-1
                try:
                    command, mode = self.controller.update(state, elapsed)
                    command = finite(command, 3)
                    if np.any(np.abs(command) > np.asarray(self.profile['command_limits'])+1e-9):
                        raise ValueError('Frozen controller emitted excessive command')
                    self.command, self.last_mode = command, mode
                    if self.controller.completed_t is not None and self.completed_pose_stamp_ns is None:
                        self.completed_pose_stamp_ns = stamp
                    core_row = copy.deepcopy(self.controller.row)
                    self.last_control_source = self.last_used_source = stamp
                    self.update_count += 1; self.update_stamps.append(stamp)
                    updated = bool(core_row.get('controller_updated'))
                except Exception as exc:
                    self.fault = 'controller_error:'+str(exc)
                    self.protect(self.fault)
        self.sequence += 1
        pose = self.pose
        out = {'schema': SCHEMA, 'sequence': self.sequence, 'source': 'slam_fixed_route',
               'mode': self.last_mode, 'controller_kind': 'frozen_truth_calibrated_controller_sensor_binding',
               'monotonic_wall': wall, 'clock_ns': ns, 'healthy': reason is None and self.fault is None and not self.hold,
               'stop_requested': bool(not np.any(self.command)), 'command': self.command.tolist(),
               'config_sha256': self.receipt['config_sha256'], 'controller_sha256': self.receipt['controller_sha256'],
               'profile_sha256': self.receipt['profile_sha256'], 'ground_truth_navigation': False,
               'pose_stamp_ns': None if pose is None else pose['stamp_ns'],
               'pose_received_monotonic_wall': None if pose is None else pose['received_monotonic_wall'],
               'gyro_stamp_ns': None if gyro is None else gyro['stamp_ns'],
               'gyro_received_monotonic_wall': None if gyro is None else gyro['received_monotonic_wall'],
               'control_pose_stamp_ns': self.last_control_source,
               'fresh_source_controller_updated': updated, 'reason': reason or self.fault,
               'command_semantics': 'body COM vx/vy and body angular wz; executor slew remains external',
               'selected_calibration_feedback_hz': self.profile['feedback_hz'],
               'source_feedback_ceiling_hz': 10, 'actual_validation': 'unverified'}
        out['controller_completed_elapsed_s'] = self.controller.completed_t
        out['first_control_pose_stamp_ns'] = self.first_source
        out['controller_completed_pose_stamp_ns'] = self.completed_pose_stamp_ns
        detail = {'envelope': out, 'core_original_row': core_row, 'pose_source': pose,
                  'paired_IMU_source': gyro, 'fixed_controller_frame': 'camera_init',
                  'actual_SLAM_route_anchor': self.route_anchor,
                  'core_native_phase_neutralization_s': .005,
                  'core_original_truth_metadata_is_not_the_deployment_source': True}
        return out, detail


def read_command(path, clock_ns, config_sha256, limits, sequence_state=None, wall=None):
    """Runner binding: dual TTL AND original feedback TTL, never heartbeat-only.

    Returns (velocity, rejected, reason). Rejection means Teacher cmd=0 through
    its normal policy/slew; it does not mean actor action=0 or joint-position hold.
    """
    live_wall = wall is None
    wall = time.monotonic() if live_wall else wall
    read_begin = wall
    attempt = {'read_begin_monotonic_wall': read_begin, 'read_monotonic_wall': wall, 'read_clock_ns': clock_ns, 'path': str(path),
               'status': 'reading', 'decoded_envelope': None}
    if sequence_state is not None:
        sequence_state['read_attempt'] = attempt
    try:
        raw = Path(path).read_bytes()
        attempt.update(raw_bytes_sha256=hashlib.sha256(raw).hexdigest(), raw_byte_count=len(raw))
        text = raw.decode('utf-8')
        attempt['raw_utf8'] = text
        def invalid_constant(token):
            raise ValueError('Nonfinite JSON constant: '+token)
        data = json.loads(text, parse_constant=invalid_constant)
        # A producer can atomically publish between read-begin and read_bytes.
        # Assess freshness after the version actually read exists, without
        # accepting negative source ages or rewriting any source timestamp.
        wall = time.monotonic() if live_wall else wall
        attempt.update(read_monotonic_wall=wall, read_completed_monotonic_wall=wall,
                       read_span_s=wall-read_begin)
        attempt['decoded_envelope'] = data
        if not isinstance(data, dict):
            raise ValueError('Command envelope is not an object')
        if data.get('schema') != SCHEMA or data.get('source') != 'slam_fixed_route' or data.get('ground_truth_navigation') is not False:
            raise ValueError('source/schema mismatch')
        if data['config_sha256'] != config_sha256 or data.get('healthy') is not True:
            raise ValueError('binding mismatch or protection hold')
        for name in ('clock_ns', 'pose_stamp_ns', 'gyro_stamp_ns', 'control_pose_stamp_ns'):
            if isinstance(data.get(name), bool) or not isinstance(data.get(name), int) or data[name] < 0:
                raise ValueError('Missing original integer timestamp: '+name)
        seq = data['sequence']
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 1:
            raise ValueError('invalid sequence')
        digest = hashlib.sha256(canonical(data).encode()).hexdigest()
        if sequence_state is not None:
            old = sequence_state.get('sequence', 0)
            if seq < old or seq == old and digest != sequence_state.get('sha256'):
                raise ValueError('sequence changed/backwards')
        if not fresh(data['clock_ns'], data['monotonic_wall'], clock_ns, wall):
            raise ValueError('producer TTL')
        for prefix in ('pose', 'gyro'):
            if not fresh(data[prefix+'_stamp_ns'], data[prefix+'_received_monotonic_wall'], clock_ns, wall):
                raise ValueError(prefix+' source TTL')
        command = finite(data['command'], 3)
        bound = finite(limits, 3)
        if np.any(np.abs(command) > bound+1e-9) or not isinstance(data.get('stop_requested'), bool):
            raise ValueError('command limits/stop invalid')
        if data['control_pose_stamp_ns'] is None or not -FUTURE_S <= (clock_ns-data['control_pose_stamp_ns'])/1e9 <= TIMEOUT_S:
            raise ValueError('controller input TTL')
        if sequence_state is not None:
            sequence_state.update(sequence=seq, sha256=digest, envelope=data)
        attempt['status'] = 'accepted'
        return np.zeros(3) if data['stop_requested'] else command, False, 'fresh actual SLAM/IMU fixed-route command'
    except (ValueError, TypeError, KeyError, OSError, OverflowError) as exc:
        if 'read_completed_monotonic_wall' not in attempt:
            wall = time.monotonic() if live_wall else wall
            attempt.update(read_monotonic_wall=wall,read_completed_monotonic_wall=wall,read_span_s=wall-read_begin)
        attempt.update(status='rejected', reason=str(exc))
        return np.zeros(3), True, str(exc)


class Recorder:
    def __init__(self, output, command_file):
        self.output = Path(output); self.output.mkdir(parents=True, exist_ok=True)
        self.command_file = Path(command_file)
        self.command_file.parent.mkdir(parents=True, exist_ok=True)
        self.q = queue.Queue(4096); self.error = None; self.counts = {}
        self.thread = threading.Thread(target=self.work, name='slam-binding-recorder', daemon=True)
        self.thread.start()

    def append(self, name, row, command=False):
        if self.error:
            raise RuntimeError(self.error)
        try:
            self.q.put_nowait((name, copy.deepcopy(row), command))
        except queue.Full:
            self.error = 'bounded evidence queue overflow'; raise RuntimeError(self.error)

    def work(self):
        files = {}
        try:
            while True:
                item = self.q.get()
                try:
                    if item is None:
                        break
                    name, row, command = item
                    line = canonical(row)
                    if command:
                        tmp = self.command_file.with_name(self.command_file.name+'.adapter.tmp')
                        tmp.write_text(line+'\n'); os.replace(tmp, self.command_file)
                    if name not in files:
                        files[name] = (self.output/name).open('a')
                    files[name].write(line+'\n'); files[name].flush()
                    self.counts[name] = self.counts.get(name, 0)+1
                finally:
                    self.q.task_done()
        except Exception as exc:
            self.error = repr(exc)
        finally:
            for f in files.values():
                f.close()

    def close(self):
        try:
            self.q.put(None, timeout=1)
            self.thread.join(5)
        except queue.Full:
            self.error = self.error or 'recorder could not close'
        if self.thread.is_alive():
            self.error = self.error or 'recorder did not drain'
        (self.output/'adapter_writer_receipt.json').write_text(canonical({
            'status': 'failed' if self.error else 'drained', 'error': self.error,
            'written_records': self.counts, 'pending_records': self.q.qsize()})+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--command-file', type=Path)
    parser.add_argument('--inspect', action='store_true', help='Validate frozen binding, print receipt; starts no ROS')
    args, ros_args = parser.parse_known_args()
    def initialization_failure(stage, exc):
        if args.output_dir is not None:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            (args.output_dir/'adapter_cleanup_receipt.json').write_text(canonical({
                'status': 'failed', 'initialization_stage': stage, 'primary_error': repr(exc),
                'cleanup_errors': [], 'node_was_constructed': False, 'zero_command_attempted': False})+'\n')
    try:
        c, core, profile, R_bi, R_wi, receipt = load_binding(args.config)
    except BaseException as exc:
        initialization_failure('frozen_binding_preflight', exc)
        raise
    if args.inspect:
        print(canonical(receipt)); return
    if args.output_dir is None or args.command_file is None:
        parser.error('Live adapter requires --output-dir and --command-file')
    if args.command_file.exists() or (args.output_dir/'adapter_source_receipt.json').exists():
        raise RuntimeError('Use a fresh run directory and new command file')
    # Imports are deliberately delayed: inspect and pure binding checks create
    # no ROS context and cannot interact with a running simulator.
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from rosgraph_msgs.msg import Clock as ClockMsg
        from geometry_msgs.msg import Twist
    except BaseException as exc:
        initialization_failure('ROS_dependency_import', exc)
        raise
    try:
        rclpy.init(args=ros_args)
    except BaseException as exc:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            initialization_failure('ROS_initialization', exc)
        raise
    partial_node = []
    class AdapterNode(Node):
        def __init__(self):
            super().__init__('teacher_slam_fixed_route_adapter')
            partial_node.append(self)
            self.binding = FreshBinding(c, core, profile, R_bi, R_wi, receipt)
            self.recorder = Recorder(args.output_dir, args.command_file)
            (args.output_dir/'adapter_source_receipt.json').write_text(canonical(receipt)+'\n')
            self.graph_good = False; self.graph_wall = -math.inf
            self.anchor_written = False
            self.pub = self.create_publisher(Twist, c.get('command_topic', '/demo/truth_tuning_slam/cmd_vel'), 1) if c.get('publish_twist') is True else None
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(ClockMsg, '/clock', self.on_clock, qos)
            self.create_subscription(Odometry, '/demo/slam/body_odom', self.on_odom, qos)
            self.create_subscription(Imu, '/livox/imu', self.on_imu, qos)
            self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.create_timer(.5, self.graph, clock=Clock(clock_type=ClockType.STEADY_TIME))
        def on_clock(self, msg):
            self.binding.clock(int(msg.clock.sec)*1000000000+int(msg.clock.nanosec), time.monotonic())
        def source_context(self, msg):
            if self.binding.clock_ns is None:
                raise ValueError('No actual clock yet')
            return {'stamp_ns': int(msg.header.stamp.sec)*1000000000+int(msg.header.stamp.nanosec),
                    'frame_id': msg.header.frame_id, 'received_monotonic_wall': time.monotonic(),
                    'callback_ros_clock_ns': self.binding.clock_ns}
        def on_odom(self, msg):
            try:
                p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
                row = {**self.source_context(msg), 'child_frame_id': msg.child_frame_id,
                       'position': [p.x, p.y, p.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w],
                       'origin_linear_velocity_body': [v.x, v.y, v.z],
                       'twist_covariance_diagonal': [msg.twist.covariance[7*i] for i in range(6)]}
                row['accepted'] = self.binding.odom(row)
                self.recorder.append('adapter_slam_poses.jsonl', row)
            except Exception as exc:
                self.binding.protect('rejected_SLAM_input')
                self.reject('SLAM', exc)
        def on_imu(self, msg):
            row = None
            try:
                q, w = msg.orientation, msg.angular_velocity
                row = {**self.source_context(msg), 'quaternion_xyzw': [q.x, q.y, q.z, q.w],
                       'angular_velocity_sensor': [w.x, w.y, w.z],
                       'orientation_available': bool(msg.orientation_covariance[0] >= 0),
                       'orientation_covariance': list(msg.orientation_covariance),
                       'angular_velocity_covariance': list(msg.angular_velocity_covariance)}
                row['accepted'] = self.binding.imu(row)
                self.recorder.append('adapter_IMU_inputs.jsonl', row)
            except Exception as exc:
                if row is not None:
                    self.recorder.append('adapter_IMU_inputs.jsonl', {**row, 'accepted': False, 'rejection': str(exc)})
                self.binding.protect('rejected_IMU_input')
                self.reject('IMU', exc)
        def reject(self, source, exc):
            try:
                self.recorder.append('adapter_rejections.jsonl', {'source': source, 'reason': str(exc),
                    'monotonic_wall': time.monotonic(), 'clock_ns': self.binding.clock_ns})
            except Exception:
                self.binding.fault = 'evidence_writer_failed'
        def graph(self):
            try:
                graph = {}
                for topic in ('/demo/slam/body_odom', '/livox/imu'):
                    graph[topic] = [{'node_name': x.node_name, 'namespace': x.node_namespace,
                                     'topic_type': x.topic_type} for x in self.get_publishers_info_by_topic(topic)]
                expected = c.get('expected_publishers', {'/demo/slam/body_odom': 'demo_slam_odom_adapter', '/livox/imu': 'ros_gz_bridge'})
                good = all(len(graph[t]) == 1 and graph[t][0]['node_name'] == expected[t] for t in graph)
                if self.pub is not None:
                    topic = c.get('command_topic', '/demo/truth_tuning_slam/cmd_vel')
                    nodes = self.get_publishers_info_by_topic(topic)
                    good = good and len(nodes) == 1 and nodes[0].node_name == self.get_name()
                self.graph_good, self.graph_wall = good, time.monotonic()
                self.recorder.append('adapter_publisher_graph.jsonl', {'graph': graph, 'valid': good,
                    'monotonic_wall': self.graph_wall, 'clock_ns': self.binding.clock_ns})
            except Exception as exc:
                self.graph_good = False; self.reject('graph', exc)
        def tick(self):
            if self.recorder.error:
                self.binding.fault = 'evidence_writer_failed'
            out, detail = self.binding.tick(time.monotonic(), self.graph_good and time.monotonic()-self.graph_wall <= 1.)
            try:
                self.recorder.append('adapter_commands.jsonl', out, command=True)
                if detail['core_original_row'] is not None:
                    self.recorder.append('adapter_controller_updates.jsonl', detail)
                if detail['actual_SLAM_route_anchor'] is not None and not self.anchor_written:
                    self.recorder.append('adapter_route_anchor.jsonl', detail['actual_SLAM_route_anchor'])
                    self.anchor_written = True
            except Exception:
                self.binding.fault = 'evidence_writer_failed'
                self.binding.protect(self.binding.fault)
            if self.pub is not None:
                msg = Twist(); msg.linear.x, msg.linear.y, msg.angular.z = map(float, self.binding.command)
                self.pub.publish(msg)
    node = None
    primary_error = None
    try:
        node = AdapterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exc:
        primary_error = repr(exc)
    finally:
        if node is None and partial_node:
            node = partial_node[0]
        errors = []
        out = None
        try:
            if node is not None and hasattr(node, 'binding'):
                node.binding.protect('shutdown')
                out, _ = node.binding.tick(time.monotonic(), False)
                # An ended producer also expires at the consumer. Optional
                # Twist is not enabled by the provided executable runner.
                if getattr(node, 'pub', None) is not None and rclpy.ok():
                    msg = Twist(); node.pub.publish(msg)
        except Exception as exc:
            errors.append('shutdown command: '+repr(exc))
        try:
            if node is not None and hasattr(node, 'recorder'):
                if out is not None and node.recorder.error is None:
                    node.recorder.append('adapter_commands.jsonl', out, command=True)
                node.recorder.close()
                if node.recorder.error:
                    errors.append('recorder: '+node.recorder.error)
        except Exception as exc:
            errors.append('recorder close: '+repr(exc))
        try:
            if out is not None:
                tmp = args.command_file.with_name(args.command_file.name+'.shutdown.tmp')
                tmp.write_text(canonical(out)+'\n'); os.replace(tmp, args.command_file)
        except Exception as exc:
            errors.append('final zero file: '+repr(exc))
        try:
            if node is not None:
                node.destroy_node()
        except Exception as exc:
            errors.append('destroy_node: '+repr(exc))
        finally:
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception as exc:
                    errors.append('rclpy.shutdown: '+repr(exc))
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir/'adapter_cleanup_receipt.json').write_text(canonical({
            'status': 'failed' if primary_error or errors else 'completed',
            'primary_error': primary_error, 'cleanup_errors': errors,
            'node_was_constructed': node is not None, 'zero_command_attempted': out is not None})+'\n')
        if primary_error or errors:
            raise RuntimeError(primary_error or '; '.join(errors))


if __name__ == '__main__':
    main()
