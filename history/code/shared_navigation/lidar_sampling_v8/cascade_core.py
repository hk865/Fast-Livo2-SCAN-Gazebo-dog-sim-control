"""Pure actual-SLAM/IMU/SCAN cascade. Returns bounded velocity, never actions.

All times are original integer nanoseconds supplied by the wrapper. No ROS,
simulator, Actor, clock sampling, or estimated Teacher slew is available here.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import numpy as np


SCHEMA = 'actual_slam_scan_header_event_cascade/v1'
DEFAULT_GAINS = dict(cross_kp=.8, cross_kd=.2, yaw_kp=1.3, yaw_kd=.15,
    velocity_kp_x=.6, velocity_ki_x=.5, velocity_kp_y=.4, velocity_ki_y=.3,
    rate_kp=.3, rate_ki=.2)
DEFAULT_COM = [.05523092034548942, -.001869001919385796, .006095299184261034]
PARKING = dict(capture_position_kp=.8, capture_position_kd=.2,
    capture_reference_xy_norm_limit_mps=.05, capture_command_limits_body=[.15,.07,.10],
    position_kp=.18, position_kd=.2, reference_xy_norm_limit_mps=.025,
    command_limits_body=[.06,.04,.10], yaw_kp=.65, yaw_kd=.18,
    reference_yaw_limit_radps=.07, position_deadband_m=.003, yaw_deadband_rad=.005,
    capture_xy_entry_m=.025, capture_xy_hold_m=.03,
    capture_yaw_entry_rad=.035, capture_yaw_hold_rad=.045,
    capture_actual_xy_speed_mps=.03, capture_actual_yawrate_radps=.06,
    capture_fresh_dwell_s=.6, first_fixed_parking_window_s=5.,
    maximum_xy_drift_m=.05, maximum_yaw_drift_rad=.1,
    maximum_native_xy_speed_mps=.08, maximum_native_Euler_yawrate_radps=.1,
    maximum_native_body_wz_radps=.1)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def sha(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def vector(value, n):
    result = np.asarray(value, float)
    if result.shape != (n,) or not np.isfinite(result).all():
        raise ValueError('Invalid finite vector')
    return result.copy()


def integer(value):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError('Nonnegative original integer timestamp required')
    return int(value)


def rotation(quaternion_wxyz):
    q = vector(quaternion_wxyz, 4)
    if not .98 <= float(q @ q) <= 1.02:
        raise ValueError('Invalid actual body quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
        [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
        [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def deadband(value, width):
    return np.sign(value) * np.maximum(abs(value)-width, 0.)


def bounded(raw, limits, forward_only=False):
    out = np.clip(raw, -limits, limits)
    if forward_only:
        out[0] = max(0., out[0])
    norm = np.linalg.norm(out[:2])
    if norm > limits[0]:
        out[:2] *= limits[0] / norm
    return out


class Controller:
    """One fixed goal, replaceable checked SCAN paths, source-header math.

    Ack is a causal *actual Teacher velocity input*, not observed velocity.
    Anti-windup uses its requested-minus-applied residual; this core never
    guesses how many actuator frames ran and never performs final slew.
    """
    def __init__(self, config, fixed_goal_xyz, fixed_goal_yaw_rad, goal_id):
        self.config = copy.deepcopy(config)
        self.g = copy.deepcopy(DEFAULT_GAINS)
        if config.get('gains', self.g) != self.g:
            raise ValueError('This candidate preserves the verified cascade gains')
        self.speed = float(config.get('desired_speed', .3))
        self.external_heading_gate = config.get('external_heading_gate', False)
        if not isinstance(self.external_heading_gate, bool):
            raise ValueError('Heading gate ownership must be explicit')
        self.limits = vector(config.get('command_limits', [.8,.35,.8]), 3)
        if not 0 < self.speed <= 1. or np.any(self.limits <= 0) or np.any(self.limits > [1.,.4,1.]):
            raise ValueError('Command outside archived Teacher boundary')
        self.com_offset = vector(config.get('base_com_offset', DEFAULT_COM), 3)
        if np.linalg.norm(self.com_offset) > .2:
            raise ValueError('Invalid body COM offset')
        self.ttl = .3
        self.gyro_gap = float(config.get('maximum_gyro_pair_gap_s', .02))
        self.goal_z_tolerance = float(config.get('goal_z_tolerance_m', .2))
        if not 0 < self.gyro_gap <= .05 or not 0 < self.goal_z_tolerance <= .3:
            raise ValueError('Unsafe source or floor boundary')
        self.goal = vector(fixed_goal_xyz, 3)
        self.goal_yaw = float(fixed_goal_yaw_rad)
        if not math.isfinite(self.goal_yaw) or not isinstance(goal_id, str) or not goal_id:
            raise ValueError('Explicit immutable map goal required')
        self.goal_id = goal_id
        self.goal_record = dict(position_world_xyz=self.goal.tolist(), heading_rad=self.goal_yaw,
            goal_id=goal_id, frame_id='camera_init')
        self.goal_sha = sha(self.goal_record)
        self.path = None
        self.path_id = None
        self.path_sha = None
        self.path_history = {}
        self.progress = 0.
        self.new_path = True
        self.replan_count = 0
        self.command = np.zeros(3)
        self.velocity_integral = np.zeros(3)
        self.hold_integral = np.zeros(3)
        self.filtered = None
        self.inner_filtered = None
        self.last_math_stamp = None
        self.last_math_wall = None
        self.last_seen_pose_stamp = None
        self.last_pose_hash = None
        self.last_clock = None
        self.last_wall = None
        self.last_ack_sequence = None
        self.last_ack_hash = None
        self.protected = True
        self.failure_latched = None
        self.protection_reason = 'initializing'
        self.recovery = []
        self.turn_phase = None
        self.turn_dwell = None
        self.capture_stamp = None
        self.capture_wall = None
        self.capture_clock = None
        self.arrival = None
        self.capture_dwell = None
        self.hold_stamp = None
        self.hold_wall = None
        self.hold_clock = None
        self.parking_interrupted = False
        self.row = {}

    def set_path(self, points_xyz, path_id, stamp_ns, received_wall_ns, frame_id='camera_init'):
        """Accept one actually checked SCAN ID. Generation time is provenance,
        not a 300ms path TTL; the wrapper continuously checks cloud/guard health.
        """
        points = np.asarray(points_xyz, float).copy()
        stamp, wall = integer(stamp_ns), integer(received_wall_ns)
        if (frame_id != 'camera_init' or not isinstance(path_id, str) or not path_id or
            points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or
            not np.isfinite(points).all()):
            raise ValueError('Finite registered directed SCAN path required')
        delta = np.diff(points, axis=0)
        lengths = np.linalg.norm(delta[:,:2], axis=1)
        if np.any(lengths < 1e-5):
            raise ValueError('Vertical/degenerate segment is not a ramp path')
        record = dict(points_xyz=points.tolist(), path_id=path_id, stamp_ns=stamp,
            received_wall_ns=wall, frame_id=frame_id)
        digest = sha(record)
        if path_id in self.path_history:
            if digest != self.path_history[path_id]:
                raise ValueError('An accepted SCAN path ID is immutable')
            return False
        if self.capture_stamp is not None:
            raise ValueError('Fixed endpoint capture cannot accept a new path')
        self.path, self.path_delta, self.lengths = points, delta, lengths
        self.cumulative = np.r_[0., np.cumsum(lengths)]
        self.path_record, self.path_id, self.path_sha = record, path_id, digest
        self.path_history[path_id] = digest
        self.progress = 0.
        self.new_path = True
        self.replan_count += 1
        # A replan is not a new velocity experiment: preserve PI and filters.
        return True

    def _project(self, position):
        # Nearest in xyz disambiguates overlapping floor xy. Horizontal arc
        # distance parameter gives signed horizontal cross and dz/ds grade.
        low = 0. if self.new_path else max(0., self.progress-.15)
        high = self.cumulative[-1] if self.new_path else min(self.cumulative[-1], self.progress+.8)
        best = None
        for i, (a, d, length) in enumerate(zip(self.path[:-1], self.path_delta, self.lengths)):
            if self.cumulative[i+1] < low or self.cumulative[i] > high:
                continue
            u0 = max(0., (low-self.cumulative[i])/length)
            u1 = min(1., (high-self.cumulative[i])/length)
            u = float(np.clip((position-a)@d/(d@d), u0, u1))
            point = a+u*d
            along = self.cumulative[i]+u*length
            distance2 = float((position-point)@(position-point))
            candidate = (distance2, float(along), i, point)
            if best is None or candidate[:3] < best[:3]:
                best = candidate
        _, nearest_s, i, point = best
        self.progress = max(self.progress, nearest_s)
        self.new_path = False
        tangent = self.path_delta[i,:2]/self.lengths[i]
        normal = np.array([-tangent[1], tangent[0]])
        cross = float((position[:2]-point[:2])@normal)
        return i, point, tangent, normal, cross, float(self.path_delta[i,2]/self.lengths[i]), nearest_s

    def _protect(self, reason, fatal=False):
        if fatal:
            self.failure_latched = reason
        if not self.protected or reason != self.protection_reason:
            self.velocity_integral[:] = 0.
            self.hold_integral[:] = 0.
            self.filtered = self.inner_filtered = None
            self.last_math_stamp = self.last_math_wall = None
            self.turn_phase = self.turn_dwell = self.capture_dwell = None
            self.recovery = []
            if self.hold_stamp is not None:
                self.parking_interrupted = True
        self.protected = True
        self.protection_reason = reason
        self.command[:] = 0.

    def _output(self, mode, clock, wall, updated=False, reason=None):
        row = copy.deepcopy(self.row)
        row.update(schema=SCHEMA, controller_updated=updated, mode=mode, reason=reason,
            compute_clock_ns=clock, compute_wall_ns=wall, command_body=self.command.tolist(),
            path_id=self.path_id, path_sha256=self.path_sha, replan_count=self.replan_count,
            fixed_goal=copy.deepcopy(self.goal_record), fixed_goal_sha256=self.goal_sha,
            progress_m=self.progress, feedback_source='actual SLAM body-origin odometry + causal actual IMU',
            navigation_ground_truth_used=False, actual_SLAM_SCAN_verified=False,
            final_slew_performed_here=False, parking_is_old_zero_command_protocol=False,
            parking_capture_pose_stamp_ns=self.capture_stamp, parking_capture_clock_ns=self.capture_clock,
            parking_capture_wall_ns=self.capture_wall, parking_hold_declared_pose_stamp_ns=self.hold_stamp,
            parking_hold_declared_clock_ns=self.hold_clock, parking_hold_declared_wall_ns=self.hold_wall,
            first_fixed_parking_window_s=5., parking_window_interrupted=self.parking_interrupted,
            arrival_capture_pose=copy.deepcopy(self.arrival), protection_active=self.protected)
        row['failure_latched'] = self.failure_latched
        self.row = copy.deepcopy(row)
        return self.command.copy(), row

    def _fresh(self, source, clock, wall):
        stamp, receipt = integer(source['stamp_ns']), integer(source['received_wall_ns'])
        if not 0 <= clock-stamp <= 300_000_000 or not 0 <= wall-receipt <= 300_000_000:
            raise ValueError('Source stale or future')
        return stamp, receipt

    def _pi(self, target, actual, dt, integral, limits, ack, forward_only, turn_only=False):
        kp = np.array([.6,.4,.3]); ki = np.array([.5,.3,.2])
        error = target-actual
        candidate = np.clip(integral+dt*error, -.5, .5)
        if turn_only:
            candidate[:2] = integral[:2]
        raw = target+kp*error+ki*candidate
        limited = bounded(raw, limits, forward_only)
        blocked_limits = error*(raw-limited) > 1e-9
        # The acknowledged request and applied input belong to the SAME actual
        # Teacher frame. Compare that residual, not a new request with old input.
        residual = vector(ack['requested_command_body'],3)-vector(ack['applied_command_body'],3)
        blocked_ack = error*residual > 1e-9
        candidate = np.where(blocked_limits | blocked_ack, integral, candidate)
        raw = target+kp*error+ki*candidate
        limited = bounded(raw, limits, forward_only)
        terms = dict(error=error.tolist(), P=(kp*error).tolist(), I=(ki*candidate).tolist(),
            integral_state=candidate.tolist(), integral_dt_s=dt,
            filtered_actual_body=actual.tolist(), blocked_limits=blocked_limits.tolist(),
            blocked_actual_ack=blocked_ack.tolist(), acknowledged_downstream_residual=residual.tolist(),
            ack_sequence=int(ack['sequence']), ack_stamp_ns=int(ack['stamp_ns']))
        terms['translation_integral_frozen'] = bool(turn_only)
        return limited, raw, candidate, terms

    def update(self, feedback, imu, ack, clock_ns, wall_ns, guard_reason=None, mode_override=None):
        """Return (requested velocity ndarray[3], independent diagnostic dict).

        mode_override: None/drive = geometric stop-turn-drive; turn = suppress
        translation; hold = protected zero; capture = bounded fixed-goal hold.
        Wrapper must reevaluate safety against the resulting actual direction.
        """
        clock, wall = integer(clock_ns), integer(wall_ns)
        if mode_override not in (None, 'drive', 'turn', 'hold', 'capture'):
            raise ValueError('Unknown explicit control mode')
        if (self.last_clock is not None and clock < self.last_clock) or (self.last_wall is not None and wall < self.last_wall):
            guard_reason = 'clock_backwards'
        self.last_clock, self.last_wall = clock, wall
        if self.failure_latched or guard_reason or mode_override == 'hold':
            reason = str(self.failure_latched or guard_reason or 'external_hold')
            self._protect(reason, fatal=reason=='clock_backwards')
            return self._output('protect',clock,wall,reason=self.protection_reason)
        try:
            if self.path is None:
                raise ValueError('No checked SCAN path')
            stamp, receipt = self._fresh(feedback,clock,wall)
            gyro_stamp, _ = self._fresh(imu,clock,wall)
            ack_stamp, _ = self._fresh(ack,clock,wall)
            if feedback.get('frame_id') != 'camera_init' or imu.get('frame_id') != 'body':
                raise ValueError('Explicit map pose and body gyro frames required')
            if not 0 <= stamp-gyro_stamp <= round(self.gyro_gap*1e9) or ack_stamp > stamp:
                raise ValueError('IMU or executed command is not causal to pose')
            if self.path_record['stamp_ns'] > stamp:
                raise ValueError('SCAN path generation is future to this pose')
            position = vector(feedback['position_world_xyz'],3)
            R = rotation(feedback['quaternion_wxyz'])
            origin_body = vector(feedback['origin_velocity_body'],3)
            gyro = vector(imu['angular_velocity_body'],3)
            roll = math.atan2(R[2,1],R[2,2]); pitch = math.asin(float(np.clip(-R[2,0],-1,1)))
            yaw = math.atan2(R[1,0],R[0,0])
            if max(abs(roll),abs(pitch)) > .65 or abs(math.cos(pitch)) < .4 or abs(math.cos(roll)) < .4:
                raise ValueError('Unsafe actual body attitude')
            ack_sequence = integer(ack['sequence'])
            request, applied = vector(ack['requested_command_body'],3), vector(ack['applied_command_body'],3)
            if np.any(abs(request)>[1.+1e-9,.4+1e-9,1.+1e-9]) or np.any(abs(applied)>[1.+1e-9,.4+1e-9,1.+1e-9]):
                raise ValueError('Executed velocity outside training command boundary')
            ack_hash = sha(dict(stamp_ns=ack_stamp, sequence=ack_sequence,
                received_wall_ns=int(ack['received_wall_ns']), requested=request.tolist(), applied=applied.tolist()))
            if self.last_ack_sequence is not None and (ack_sequence < self.last_ack_sequence or
                (ack_sequence == self.last_ack_sequence and ack_hash != self.last_ack_hash)):
                raise ValueError('Executed command acknowledgement changed or went backwards')
            self.last_ack_sequence, self.last_ack_hash = ack_sequence, ack_hash
            pose_hash = sha(dict(position=position.tolist(), quaternion=vector(feedback['quaternion_wxyz'],4).tolist(),
                velocity=origin_body.tolist(), receipt_ns=receipt))
            if self.last_seen_pose_stamp is not None and stamp < self.last_seen_pose_stamp:
                raise ValueError('Original SLAM header went backwards')
            duplicate = stamp == self.last_seen_pose_stamp
            if duplicate and pose_hash != self.last_pose_hash:
                raise ValueError('A repeated SLAM header changed data or receipt')
        except (ValueError, TypeError, KeyError) as error:
            reason = str(error)
            recoverable = reason in ('No checked SCAN path', 'Source stale or future',
                'IMU or executed command is not causal to pose', 'SCAN path generation is future to this pose')
            self._protect(reason, fatal=not recoverable)
            return self._output('protect',clock,wall,reason=self.protection_reason)
        # Check the old receipt BEFORE accepting the fresh one. A late update
        # does not erase a >300ms gap and falsely continue integral or dwell.
        if self.last_math_stamp is not None and (stamp-self.last_math_stamp > 300_000_000 or
            wall-self.last_math_wall > 300_000_000):
            self._protect('feedback_gap_before_fresh')
        if duplicate:
            return self._output('protect' if self.protected else self.row.get('mode','hold'),clock,wall,
                reason=self.protection_reason if self.protected else 'duplicate_SLAM_header')
        self.last_seen_pose_stamp, self.last_pose_hash = stamp, pose_hash
        if self.protected:
            self.recovery.append(stamp)
            self.recovery = self.recovery[-2:]
            if len(self.recovery) < 2:
                return self._output('recovering',clock,wall,reason=self.protection_reason)
            self.protected = False
            self.protection_reason = None
            self.last_math_stamp = self.last_math_wall = None
        dt = 0. if self.last_math_stamp is None else (stamp-self.last_math_stamp)/1e9
        self.last_math_stamp, self.last_math_wall = stamp, wall
        if dt > .20000001:
            self.capture_dwell = self.turn_dwell = None
        yawdot = (math.sin(roll)*gyro[1]+math.cos(roll)*gyro[2])/math.cos(pitch)
        origin_world = R@origin_body
        com_body = origin_body+np.cross(gyro,self.com_offset)
        measurement = np.r_[origin_world[:2],yawdot]
        actual = np.r_[com_body[:2],gyro[2]]
        self.filtered = measurement.copy() if self.filtered is None else self.filtered+dt/(.2+dt)*(measurement-self.filtered)
        self.inner_filtered = actual.copy() if self.inner_filtered is None else self.inner_filtered+dt/(.1+dt)*(actual-self.inner_filtered)
        segment, point, tangent, normal, cross, grade, nearest_s = self._project(position)
        remaining = float(self.cumulative[-1]-self.progress)
        goal_error = self.goal[:2]-position[:2]
        goal_distance = float(np.linalg.norm(goal_error))
        goal_z_error = float(self.goal[2]-position[2])
        endpoint_is_goal = np.linalg.norm(self.path[-1]-self.goal) <= .2
        heading = math.atan2(tangent[1],tangent[0])
        if endpoint_is_goal and remaining <= .15:
            heading = self.goal_yaw
        error_yaw = wrap(heading-yaw)
        capture_ready = (endpoint_is_goal and remaining <= .15 and goal_distance <= .12 and
            abs(goal_z_error) <= self.goal_z_tolerance and np.linalg.norm(origin_body[:2])<.08 and
            abs(yawdot)<.1 and abs(wrap(self.goal_yaw-yaw))<.2)
        entry_reset = False
        if self.capture_stamp is None and (capture_ready or mode_override=='capture'):
            if goal_distance>.3 or abs(goal_z_error)>self.goal_z_tolerance:
                self._protect('capture_fixed_goal_out_of_range')
                return self._output('protect',clock,wall,reason=self.protection_reason)
            self.capture_stamp, self.capture_wall, self.capture_clock = stamp, receipt, clock
            self.arrival = dict(position_world_xyz=position.tolist(),yaw_rad=yaw,pose_stamp_ns=stamp)
            self.hold_integral[:] = 0.
            self.capture_dwell = None
            entry_reset = True
        position_terms = dict(P=[0.,0.],D=[0.,0.])
        if self.capture_stamp is not None:
            if goal_distance>.3 or abs(goal_z_error)>self.goal_z_tolerance or abs(wrap(self.goal_yaw-yaw))>.6:
                self._protect('parking_fixed_target_lost')
                return self._output('protect',clock,wall,reason=self.protection_reason)
            xy_gate = .03 if self.capture_dwell is not None else .025
            yaw_gate = .045 if self.capture_dwell is not None else .035
            eligible = (goal_distance<=xy_gate and abs(wrap(self.goal_yaw-yaw))<=yaw_gate and
                np.linalg.norm(origin_body[:2])<.03 and abs(yawdot)<.06)
            if self.hold_stamp is None:
                self.capture_dwell = (stamp if self.capture_dwell is None else self.capture_dwell) if eligible else None
                if self.capture_dwell is not None and stamp-self.capture_dwell >= 600_000_000:
                    self.hold_stamp, self.hold_wall, self.hold_clock = stamp, receipt, clock
            mode = 'active_hold' if self.hold_stamp is not None else 'capture'
            prefix = '' if mode=='active_hold' else 'capture_'
            P = PARKING[prefix+'position_kp']*deadband(goal_error,.003)
            D = -PARKING[prefix+'position_kd']*self.filtered[:2]
            world_xy = P+D
            cap = PARKING[prefix+'reference_xy_norm_limit_mps']
            if np.linalg.norm(world_xy)>cap:
                world_xy *= cap/np.linalg.norm(world_xy)
            velocity_world = np.r_[world_xy, 0.]
            heading, error_yaw = self.goal_yaw, wrap(self.goal_yaw-yaw)
            yaw_P = .65*float(deadband(error_yaw,.005)); yaw_D = -.18*self.filtered[2]
            wref = float(np.clip(yaw_P+yaw_D,-.07,.07))
            limits = np.asarray(PARKING[prefix+'command_limits_body'])
            integral, integral_dt = self.hold_integral, dt
            position_terms = dict(P=P.tolist(),D=D.tolist())
        else:
            eligible = False
            if self.external_heading_gate:
                self.turn_phase = 'turn' if mode_override=='turn' else None
                self.turn_dwell = None
            if not self.external_heading_gate and self.turn_phase is None and abs(error_yaw)>.2:
                self.turn_phase='pre_turn'; self.turn_dwell=None
            if mode_override=='turn':
                self.turn_phase='turn'; self.turn_dwell=None
            elif self.turn_phase=='pre_turn':
                stopped = np.linalg.norm(origin_body[:2])<.04 and abs(yawdot)<.05
                self.turn_dwell = (stamp if self.turn_dwell is None else self.turn_dwell) if stopped else None
                if self.turn_dwell is not None and stamp-self.turn_dwell>=300_000_000:
                    self.turn_phase='turn'; self.turn_dwell=None
            elif self.turn_phase=='turn' and abs(error_yaw)<.08 and abs(yawdot)<.08:
                self.turn_phase='settle'; self.turn_dwell=stamp
            elif self.turn_phase=='settle':
                if abs(error_yaw)>.2:
                    self.turn_phase='turn'; self.turn_dwell=None
                elif self.turn_dwell is not None and stamp-self.turn_dwell>=300_000_000:
                    self.turn_phase=None; self.turn_dwell=None
            mode = self.turn_phase or 'drive'
            speed = min(self.speed, .9*max(remaining,0.), math.sqrt(.7*max(remaining-.05,0.)))
            lateral = float(np.clip(-.8*cross-.2*float(self.filtered[:2]@normal),-.2,.2))
            velocity_world = np.r_[speed*tangent+lateral*normal, speed*grade]
            yaw_P, yaw_D = 1.3*error_yaw, -.15*self.filtered[2]
            wref = yaw_P+yaw_D
            limits = self.limits
            integral, integral_dt = self.velocity_integral, dt if mode in ('drive','turn') else 0.
            if mode!='drive':
                velocity_world[:] = 0.
            if remaining<.05 and not endpoint_is_goal and mode=='drive':
                mode='path_end_hold'; velocity_world[:]=0.; integral_dt=0.
        body_wref = (wref*math.cos(pitch)-math.sin(roll)*gyro[1])/math.cos(roll)
        origin_reference = R.T@velocity_world
        com_reference = origin_reference+np.cross([0.,0.,body_wref],self.com_offset)
        target = np.r_[com_reference[:2],body_wref]
        requested, raw, candidate, pi = self._pi(target,self.inner_filtered,integral_dt,integral,limits,ack,
            mode=='drive', turn_only=mode=='turn')
        if self.capture_stamp is not None:
            self.hold_integral = candidate
        else:
            self.velocity_integral = candidate
        if mode=='turn':
            requested[:2]=0.
        elif mode in ('pre_turn','settle','path_end_hold'):
            requested[:]=0.; target[:]=0.; velocity_world[:]=0.
        self.command=requested
        self.row=dict(kind='controller_update',feedback_pose_stamp_ns=stamp,
            feedback_received_wall_ns=receipt,imu_stamp_ns=gyro_stamp,
            ack_stamp_ns=ack_stamp,header_dt_s=dt,source_expected_hz=self.config.get('feedback_expected_hz',10),
            integral_dt_s=integral_dt, segment=segment,nearest_projection_xyz=point.tolist(),
            nearest_projection_progress_m=nearest_s,path_length_m=float(self.cumulative[-1]),
            remaining_horizontal_arc_m=remaining,local_horizontal_tangent=tangent.tolist(),
            local_horizontal_normal=normal.tolist(),local_grade_dz_ds=grade,
            error_cross_m=cross,error_along_remaining_m=remaining,error_yaw_rad=error_yaw,
            reference_yaw_rad=heading,reference_velocity_world=velocity_world.tolist(),
            reference_COM_velocity_body=target.tolist(),raw_command_body=raw.tolist(),
            command_limits_body=np.asarray(limits).tolist(),velocity_PI=pi,
            position_terms=position_terms,yaw_PD=dict(P=yaw_P,D=yaw_D),
            measured_origin_velocity_world=origin_world.tolist(),measured_origin_velocity_body=origin_body.tolist(),
            measured_COM_velocity_body=com_body.tolist(),measured_Euler_yawrate_radps=yawdot,
            measured_body_omega=gyro.tolist(),body_roll_rad=roll,body_pitch_rad=pitch,
            goal_distance_xy_m=goal_distance,goal_z_error_m=goal_z_error,
            endpoint_is_fixed_goal=bool(endpoint_is_goal),capture_measurement_eligible=bool(eligible),
            capture_fresh_dwell_start_pose_stamp_ns=self.capture_dwell,
            capture_entry_integral_reset=entry_reset,command_saturated=(abs(raw-requested)>1e-9).tolist(),
            COM_reference_angular_assumption='Reference body omega_x/y=0; causal measured omega_y in Euler yaw conversion',
            source_hz_is_math_update_hz=True,parking_active_command_need_not_be_zero=True)
        return self._output(mode,clock,wall,updated=True)
