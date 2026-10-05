"""Simulation-only static analytic curve tracking; no actuator or ROS access.

API: Controller(profile).update(native64_state, simulation_elapsed_s) ->
(requested_body_vx_vy_wz, mode). The actor worker still owns command slewing;
the native TeacherActuator remains the only joint-force writer.
"""
import hashlib
import json
import math
import time

import numpy as np


GAUSS_X, GAUSS_W = np.polynomial.legendre.leggauss(64)
SLEW = np.array([.6, .6, .8])
DEFAULT_GAINS = dict(cross_kp=.8, cross_kd=.2, cross_ki=0., yaw_kp=1.3,
                     yaw_kd=.15, yaw_ki=0., velocity_kp_x=.6, velocity_ki_x=.5,
                     velocity_kp_y=.4, velocity_ki_y=.3, rate_kp=.3, rate_ki=.2)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def rotation(q):
    w, x, y, z = np.asarray(q, float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def parameter_hash(parameters):
    return hashlib.sha256(json.dumps(parameters, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


class StaticPath:
    """Finite flat world curve parameterized by true arc length, not time.

    Circle position is elementary analytic. S heading is A sin^3(2pi u/L);
    position is the definite integral of [cos(theta), sin(theta)] in arc length.
    A 64-point Gauss rule evaluates that analytic integral; derivatives and
    curvature are direct analytic formulas. No polyline drives the controller.
    """
    def __init__(self, parameters):
        self._parameters = json.loads(json.dumps(parameters, allow_nan=False))
        p = self._parameters
        self.sha256 = parameter_hash(p)
        self.kind = p['kind']
        if self.kind not in ('circle', 's_curve'):
            raise ValueError('Only circle and s_curve are supported')
        self.origin = np.asarray(p['origin_world_xyz'], float).copy()
        if self.origin.shape != (3,) or not np.isfinite(self.origin).all():
            raise ValueError('Invalid static world origin')
        self.origin.flags.writeable = False
        self.heading0 = float(p['heading0_rad'])
        self.entry = float(p['entry_straight_m'])
        self.exit = float(p['exit_straight_m'])
        self.sign = int(p['direction_sign'])
        if self.sign not in (-1, 1) or min(self.entry, self.exit) < .5:
            raise ValueError('Tangent entry/exit and signed direction required')
        if self.kind == 'circle':
            self.radius = float(p['radius_m'])
            if not math.isfinite(self.radius) or self.radius <= 0:
                raise ValueError('Positive explicit circle radius required')
            if not math.isclose(float(p['turn_angle_rad']),2*math.pi,rel_tol=0,abs_tol=1e-12):
                raise ValueError('Exactly one full circle required')
            self.curve_length = 2*math.pi*self.radius
            self.maximum_abs_curvature = 1/self.radius
        else:
            self.curve_length = float(p['curve_length_m'])
            self.amplitude = float(p['heading_amplitude_rad'])
            if self.curve_length <= 1 or not 0 < self.amplitude <= 1.2:
                raise ValueError('S length >1m and 0<heading amplitude<=1.2rad required')
            self.maximum_abs_curvature = 4*math.pi*self.amplitude/(math.sqrt(3)*self.curve_length)
        if not all(math.isfinite(v) for v in (self.heading0, self.entry, self.exit, self.curve_length)):
            raise ValueError('Nonfinite path')
        self.length = self.entry+self.curve_length+self.exit
        co, si = math.cos(self.heading0), math.sin(self.heading0)
        self.world_rotation = np.array([[co, -si], [si, co]])
        self.world_rotation.flags.writeable = False
        self.curve_end_local = self._curve_position(np.array([self.curve_length]))[0]
        self.final_xyz = self.position(self.length)
        # Local projection prevents jumping across a full-circle overlap.
        self.forward_window = min(.6, self.curve_length*.2) if self.kind == 'circle' else .6
        self.backward_window = min(.25, self.curve_length*.1) if self.kind == 'circle' else .25

    @property
    def parameters(self):
        return json.loads(json.dumps(self._parameters))

    def _curve_position(self, u):
        u = np.asarray(u, float).reshape(-1)
        if self.kind == 'circle':
            phase = u/self.radius
            return np.column_stack((self.radius*np.sin(phase),
                                    self.sign*self.radius*(1-np.cos(phase))))
        node = .5*u[:, None]*(GAUSS_X[None, :]+1)
        theta = self.sign*self.amplitude*np.sin(2*np.pi*node/self.curve_length)**3
        return .5*u[:, None]*np.column_stack((np.cos(theta)@GAUSS_W, np.sin(theta)@GAUSS_W))

    def position(self, s):
        raw = np.asarray(s, float)
        u = np.clip(raw.reshape(-1), 0., self.length)
        local = np.column_stack((np.minimum(u, self.entry), np.zeros(len(u))))
        curve = (u > self.entry) & (u <= self.entry+self.curve_length)
        local[curve] = self._curve_position(u[curve]-self.entry)+np.array([self.entry, 0.])
        after = u > self.entry+self.curve_length
        local[after] = self.curve_end_local+np.column_stack((self.entry+u[after]-self.entry-self.curve_length,
                                                           np.zeros(after.sum())))
        world = local@self.world_rotation.T+self.origin[:2]
        result = np.column_stack((world, np.full(len(u), self.origin[2])))
        return result[0] if raw.ndim == 0 else result.reshape(raw.shape+(3,))

    def geometry(self, s):
        s = float(np.clip(s, 0., self.length))
        theta = 0.; curvature = 0.
        if self.entry <= s < self.entry+self.curve_length:
            u = s-self.entry
            if self.kind == 'circle':
                theta = self.sign*u/self.radius
                curvature = self.sign/self.radius
            else:
                k = 2*math.pi/self.curve_length
                theta = self.sign*self.amplitude*math.sin(k*u)**3
                curvature = self.sign*3*self.amplitude*k*math.sin(k*u)**2*math.cos(k*u)
        elif s >= self.entry+self.curve_length and self.kind == 'circle':
            theta = self.sign*2*math.pi
        heading = self.heading0+theta
        tangent = np.array([math.cos(heading), math.sin(heading), 0.])
        normal = np.array([-tangent[1], tangent[0], 0.])
        return self.position(s), tangent, normal, heading, curvature

    def project(self, position, previous_progress):
        """Geometric local nearest point, with monotone admitted arc progress.

        The neighborhood is in meters along the immutable path, not a clock
        prediction. If the vehicle loses this neighborhood, errors grow instead
        of teleporting to a later loop. Backward motion cannot undo lap progress.
        """
        previous_progress = float(np.clip(previous_progress, 0., self.length))
        lower = max(0., previous_progress-self.backward_window)
        upper = min(self.length, previous_progress+self.forward_window)
        xy = np.asarray(position, float)[:2]
        grid = np.linspace(lower, upper, 33)
        distances = np.sum((self.position(grid)[:, :2]-xy)**2, axis=1)
        current = float(grid[np.argmin(distances)])
        best = float(distances.min())
        for _ in range(12):
            r, tangent, normal, _, curvature = self.geometry(current)
            e = xy-r[:2]
            denominator = 1-curvature*float(e@normal[:2])
            if denominator <= .05:
                break
            trial = float(np.clip(current+float(e@tangent[:2])/denominator, lower, upper))
            value = float(np.sum((self.position(trial)[:2]-xy)**2))
            if value > best+1e-14:
                trial = .5*(trial+current)
                value = float(np.sum((self.position(trial)[:2]-xy)**2))
            if value > best+1e-14:
                break
            step = abs(trial-current);current = trial;best = value
            if step < 1e-10:
                break
        progress = max(previous_progress, current)
        return progress, {'geometric_nearest_arc_m': current,
                          'admitted_progress_arc_m': progress,
                          'projection_search_interval_m': [lower, upper],
                          'nearest_xy_distance_m': math.sqrt(best)}


class Controller:
    def __init__(self, profile):
        self.p = json.loads(json.dumps(profile, allow_nan=False))
        self.path = StaticPath(self.p['path'])
        if self.p['path_parameters_sha256'] != self.path.sha256:
            raise ValueError('Immutable path parameter hash mismatch')
        self.hz = int(self.p['feedback_hz'])
        if self.hz not in (10, 25, 50):
            raise ValueError('Only exact 50Hz decimations:10/25/50')
        self.stride = 50//self.hz
        self.speed = float(self.p['desired_speed'])
        if not 0 < self.speed <= .8:
            raise ValueError('Prospective curve speed cap 0.8m/s')
        self.limits = np.asarray(self.p['command_limits'], float)
        if self.limits.shape != (3,) or np.any(self.limits <= 0) or np.any(self.limits > [.8,.35,.8]):
            raise ValueError('Prospective curve command caps [.8,.35,.8]')
        self.com_offset = np.asarray(self.p['base_com_offset'], float)
        if self.com_offset.shape != (3,) or not np.isfinite(self.com_offset).all():
            raise ValueError('Archived COM offset required')
        self.g = DEFAULT_GAINS | self.p.get('gains', {})
        if not all(math.isfinite(v) and 0<=v<=4 for v in self.g.values()):
            raise ValueError('Invalid gains')
        self.command = np.zeros(3);self.applied_command_estimate = np.zeros(3)
        self.integral = np.zeros(3);self.velocity_integral = np.zeros(3)
        self.filtered = None;self.inner_filtered = None
        self.frame = -1;self.progress = 0.;self.completed_t = None;self.dwell = None
        self.last_feedback = None;self.last_wall = None;self.last_world = None
        self.fault = None;self.row = {};self.waypoint_events = []

    def _zero_row(self, s, t, mode):
        final = self.path.final_xyz
        self.command[:] = 0.
        self.row = {**self.row, 'kind':'controller_hold', 'controller_updated':False,
                    'mode':mode, 'control_t_s':float(s[0]), 'feedback_time_s':float(s[0]-.005),
                    'elapsed_s':t, 'completed_t_s':self.completed_t,
                    'feedback_hz':self.hz, 'design':'curvature_cascade_pi',
                    'command_body':[0.,0.,0.], 'raw_command_body':[0.,0.,0.],
                    'v_reference_body':[0.,0.,0.], 'reference_velocity_world':[0.,0.,0.],
                    'reference_xy':final[:2].tolist(), 'reference_yaw':self.path.heading0,
                    'path_parameters_sha256':self.path.sha256,
                    'arc_progress_m':self.progress, 'path_length_m':self.path.length,
                    'active_integral_dt_s':0., 'feedback_measurement_fresh':False,
                    'route_reference_changes':0, 'counts_as_SLAM_navigation':False,
                    'fault':self.fault}

    def update(self, s, t):
        s = np.asarray(s, float);t = float(t)
        if s.shape != (64,) or not np.isfinite(s).all() or np.linalg.norm(s[4:8]) < .9:
            raise ValueError('Invalid native64 state')
        if self.last_world is not None and s[0] <= self.last_world:
            raise ValueError('Nonincreasing native clock')
        self.last_world = float(s[0]);self.frame += 1
        stamp = float(s[0]-.005);now = time.monotonic()
        receipt_gap = 0. if self.last_wall is None else now-self.last_wall
        updated = self.frame%self.stride == 0
        if t < 3. or self.completed_t is not None or self.fault is not None:
            mode = 'initializing' if t < 3. else 'parking' if self.completed_t is not None else self.fault
            self._zero_row(s,t,mode)
        elif updated:
            dt = 0. if self.last_feedback is None else stamp-self.last_feedback
            if dt < 0 or dt > .30000001 or receipt_gap > .3:
                self.fault = 'feedback_timeout';self.integral[:]=0.;self.velocity_integral[:]=0.
                self._zero_row(s,t,self.fault)
            else:
                self.last_feedback = stamp;self.last_wall = now
                R = rotation(s[4:8]);yaw = math.atan2(R[1,0],R[0,0])
                roll = math.atan2(R[2,1],R[2,2]);pitch = math.asin(np.clip(-R[2,0],-1,1))
                if abs(math.cos(pitch)) < .4 or abs(math.cos(roll)) < .4:
                    raise ValueError('Unsafe yaw kinematics')
                yawdot = (math.sin(roll)*s[12]+math.cos(roll)*s[13])/math.cos(pitch)
                origin_body = s[8:11]-np.cross(s[11:14],self.com_offset)
                origin_world = R@origin_body
                self.progress,projection = self.path.project(s[1:4],self.progress)
                point,tangent,normal,heading,curvature = self.path.geometry(self.progress)
                cross = float((s[1:3]-point[:2])@normal[:2])
                along_residual = float((s[1:3]-point[:2])@tangent[:2])
                eyaw = wrap(heading-yaw)
                measurement = np.r_[origin_world[:2],yawdot]
                alpha = 1. if self.filtered is None else dt/(.2+dt)
                self.filtered = measurement.copy() if self.filtered is None else self.filtered+alpha*(measurement-self.filtered)
                remaining = self.path.length-self.progress
                distance = float(np.linalg.norm(s[1:3]-self.path.final_xyz[:2]))
                radius = .14 if self.dwell is not None else .12
                eligible = (self.progress>=self.path.length-.15 and distance<=radius and
                            np.linalg.norm(origin_body[:2])<.08 and abs(yawdot)<.1 and abs(eyaw)<.2)
                mode = 'drive'
                if eligible:
                    self.dwell = t if self.dwell is None else self.dwell
                    mode = 'goal_dwell'
                    if t-self.dwell>=.6:
                        self.completed_t=t;mode='parking'
                else:
                    self.dwell=None
                active_dt = dt if mode=='drive' else 0.
                curvature_speed_cap = math.inf if abs(curvature)<1e-12 else .8*.8/abs(curvature)
                speed = min(self.speed,.8,curvature_speed_cap,.9*max(remaining,0.),
                            math.sqrt(.7*max(remaining-.05,0.)))
                g=self.g
                self.integral[1]=np.clip(self.integral[1]-active_dt*cross,-.3,.3)
                self.integral[2]=np.clip(self.integral[2]+active_dt*eyaw,-.3,.3)
                lateral = (-g['cross_kp']*cross-g['cross_kd']*float(self.filtered[:2]@normal[:2])+
                           g['cross_ki']*self.integral[1])
                velocity_world=speed*tangent+np.clip(lateral,-.20,.20)*normal
                yaw_ff=curvature*speed
                denominator=1-curvature*cross
                reference_yawdot = curvature*float(self.filtered[:2]@tangent[:2])/max(.25,denominator)
                yaw_P=g['yaw_kp']*eyaw
                yaw_D=g['yaw_kd']*(reference_yawdot-self.filtered[2])
                wref=yaw_ff+yaw_P+yaw_D+g['yaw_ki']*self.integral[2]
                body_wref=(wref*math.cos(pitch)-math.sin(roll)*s[12])/math.cos(roll)
                origin_ref_body=R.T@velocity_world
                com_ref=origin_ref_body+np.cross(np.array([0.,0.,body_wref]),self.com_offset)
                target=np.r_[com_ref[:2],body_wref]
                actual=np.r_[s[8:10],s[13]]
                beta=1. if self.inner_filtered is None else dt/(.1+dt)
                self.inner_filtered=actual.copy() if self.inner_filtered is None else self.inner_filtered+beta*(actual-self.inner_filtered)
                ev=target-self.inner_filtered
                kp=np.array([g['velocity_kp_x'],g['velocity_kp_y'],g['rate_kp']])
                ki=np.array([g['velocity_ki_x'],g['velocity_ki_y'],g['rate_ki']])
                candidate=np.clip(self.velocity_integral+active_dt*ev,-.5,.5)
                raw=target+kp*ev+ki*candidate
                clipped=np.clip(raw,-self.limits,self.limits)
                blocked_axis=ev*(raw-clipped)>0
                candidate=np.where(blocked_axis,self.velocity_integral,candidate)
                next_slew=self.applied_command_estimate+np.clip(clipped-self.applied_command_estimate,-SLEW*.02,SLEW*.02)
                blocked_slew=ev*(raw-next_slew)>1e-9
                candidate=np.where(blocked_slew,self.velocity_integral,candidate)
                self.velocity_integral=candidate
                raw=target+kp*ev+ki*candidate
                if mode!='drive':
                    raw[:]=0.;target[:]=0.;velocity_world[:]=0.
                clipped=np.clip(raw,-self.limits,self.limits)
                if mode=='drive':clipped[0]=max(0.,clipped[0])
                planar=np.linalg.norm(clipped[:2])
                if planar>self.limits[0]:clipped[:2]*=self.limits[0]/planar
                self.command=clipped
                self.row={'kind':'controller_update','controller_updated':True,
                          'control_t_s':float(s[0]),'feedback_time_s':stamp,'elapsed_s':t,
                          'header_dt_s':dt,'active_integral_dt_s':active_dt,'feedback_hz':self.hz,
                          'design':'curvature_cascade_pi','mode':mode,'segment':0,
                          'reference_xy':point[:2].tolist(),'reference_yaw':heading,
                          'reference_velocity_world':velocity_world.tolist(),
                          'error_cross':cross,'error_along':-along_residual,'error_yaw':eyaw,
                          'arc_progress_m':self.progress,'path_length_m':self.path.length,
                          'remaining_arc_m':remaining,'endpoint_xy_distance_m':distance,
                          'path_curvature_1pm':curvature,'planned_tangential_speed_mps':speed,
                          'curvature_speed_cap_mps':None if math.isinf(curvature_speed_cap) else curvature_speed_cap,
                          'yaw_feedforward_radps':yaw_ff,'reference_geometric_yaw_rate_radps':reference_yawdot,
                          'yaw_PD':{'P':yaw_P,'D':yaw_D},'v_reference_body':target.tolist(),
                          'command_body':clipped.tolist(),'raw_command_body':raw.tolist(),
                          'velocity_PI':{'error':ev.tolist(),'P':(kp*ev).tolist(),'I':(ki*candidate).tolist(),
                                         'filtered_actual_body':self.inner_filtered.tolist(),
                                         'blocked_axis':blocked_axis.tolist(),'blocked_slew':blocked_slew.tolist()},
                          'position_terms':{'P':[0.,-g['cross_kp']*cross,yaw_P],
                                            'I':[0.,g['cross_ki']*self.integral[1],g['yaw_ki']*self.integral[2]],
                                            'D':[0.,-g['cross_kd']*float(self.filtered[:2]@normal[:2]),yaw_D]},
                          'measured_origin_velocity_world':origin_world.tolist(),
                          'measured_COM_velocity_body':s[8:11].tolist(),'measured_yaw_rate':yawdot,
                          'COM_reference_angular_assumption':'Desired body omega_xy=0 for COM lever arm; measured body omega_y is used in Euler yaw-rate conversion',
                          'saturated':(abs(raw-clipped)>1e-9).tolist(),
                          'completed_t_s':self.completed_t,'goal_entry_radius_m':.12,'goal_hold_radius_m':.14,
                          'progress_required_before_final_dwell_m':self.path.length-.15,
                          'projection':projection,'path_parameters_sha256':self.path.sha256,
                          'route_reference_changes':0,'feedback_measurement_fresh':True,
                          'feedback_source':'Gazebo native truth; isolated curve calibration only',
                          'counts_as_SLAM_navigation':False,'fault':None}
        else:
            self.row={**self.row,'kind':'controller_hold','controller_updated':False,
                      'control_t_s':float(s[0]),'elapsed_s':t,'command_body':self.command.tolist(),
                      'feedback_measurement_fresh':False}
        sim_age=stamp-self.last_feedback if self.last_feedback is not None else 0.
        wall_age=now-self.last_wall if self.last_wall is not None else 0.
        if self.completed_t is None and t>=3. and (sim_age>.3 or wall_age>.3):
            self.fault='feedback_timeout';self.integral[:]=0.;self.velocity_integral[:]=0.
            self._zero_row(s,t,self.fault)
        self.applied_command_estimate+=np.clip(self.command-self.applied_command_estimate,-SLEW*.02,SLEW*.02)
        self.row.update(feedback_age_sim_s=sim_age,feedback_age_wall_s=wall_age,
                        previous_fresh_wall_receipt_gap_s=receipt_gap,
                        estimated_teacher_command_body=self.applied_command_estimate.tolist())
        return self.command.copy(),self.row['mode']

# BEGIN ACTIVE PARKING V2: the entire preceding v1 source is byte-identical.
from pathlib import Path

V1_DRIVE_SOURCE_SHA256 = '82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d'
V1_DRIVE_SOURCE_SIZE_BYTES = 21098
FrozenV1Controller = Controller


def parking_target_record(path):
    return dict(position_world_xyz=path.final_xyz.tolist(), heading0_rad=path.heading0,
                path_parameters_sha256=path.sha256)


def _deadband(value, width):
    value = np.asarray(value, float)
    return np.sign(value)*np.maximum(abs(value)-width, 0.)


class Controller(FrozenV1Controller):
    """Unchanged analytic driving, followed by bounded active endpoint hold.

    Capture and hold both use the prospective fixed endpoint, not a moving
    measured-pose anchor. The first arrived pose is immutable diagnostic data.
    This API only returns Teacher body velocity commands; never joint actions.
    """
    def __init__(self, profile):
        super().__init__(profile)
        if hashlib.sha256(Path(__file__).read_bytes()[:V1_DRIVE_SOURCE_SIZE_BYTES]).hexdigest()!=V1_DRIVE_SOURCE_SHA256:
            raise ValueError('Frozen v1 drive prefix differs')
        self.parking=json.loads(json.dumps(self.p['active_parking'],allow_nan=False))
        if self.parking['schema']!='fixed_endpoint_active_parking/v1':
            raise ValueError('Explicit active parking contract required')
        # Preserve the exact prospective target and its byte-level canonical
        # SHA. NumPy versions may differ in quadrature by a few femtometres;
        # recomputation is only a bounded numerical consistency check.
        expected_target=parking_target_record(self.path)
        self._target=json.loads(json.dumps(self.parking['target'],allow_nan=False))
        self._target_sha=parameter_hash(self._target)
        endpoint=np.asarray(self._target['position_world_xyz'],float)
        if (self.parking['target_sha256']!=self._target_sha or
            self.parking['target_kind']!='prospective_fixed_path_endpoint' or
            endpoint.shape!=(3,) or not np.isfinite(endpoint).all() or
            self._target['path_parameters_sha256']!=expected_target['path_parameters_sha256'] or
            np.max(abs(endpoint-np.asarray(expected_target['position_world_xyz'])))>1e-12 or
            not math.isfinite(float(self._target['heading0_rad'])) or
            abs(wrap(self._target['heading0_rad']-expected_target['heading0_rad']))>1e-12):
            raise ValueError('Parking target must be the immutable prospective path endpoint')
        self.hold_limits=np.asarray(self.parking['command_limits_body'],float)
        if (self.hold_limits.shape!=(3,) or not np.isfinite(self.hold_limits).all() or
            np.any(self.hold_limits<=0) or np.any(self.hold_limits>[.06,.04,.10])):
            raise ValueError('Active parking command envelope exceeded')
        positive=('position_kp','position_kd','yaw_kp','yaw_kd','reference_xy_norm_limit_mps',
            'reference_yaw_limit_radps','capture_xy_entry_m','capture_xy_hold_m',
            'capture_yaw_entry_rad','capture_yaw_hold_rad','capture_actual_xy_speed_mps',
            'capture_actual_yawrate_radps','capture_fresh_dwell_s','target_loss_radius_m',
            'target_loss_heading_rad')
        if any(not math.isfinite(float(self.parking[k])) or float(self.parking[k])<=0 for k in positive):
            raise ValueError('Nonpositive active parking parameter')
        if (self.parking['reference_xy_norm_limit_mps']>.025 or self.parking['reference_yaw_limit_radps']>.07 or
            self.parking['capture_xy_entry_m']>.025 or self.parking['capture_xy_hold_m']>.03 or
            self.parking['capture_yaw_entry_rad']>.035 or self.parking['capture_yaw_hold_rad']>.045 or
            self.parking['capture_actual_xy_speed_mps']>.03 or self.parking['capture_actual_yawrate_radps']>.06 or
            self.parking['capture_fresh_dwell_s']<.6 or self.parking['target_loss_radius_m']>.3 or
            self.parking['target_loss_heading_rad']>.6):
            raise ValueError('Active parking/capture safety boundary loosened')
        if (self.parking['position_kp']>1 or self.parking['position_kd']>.5 or
            self.parking['yaw_kp']>1.3 or self.parking['yaw_kd']>.3 or
            not 0<=self.parking['position_deadband_m']<=.005 or
            not 0<=self.parking['yaw_deadband_rad']<=.008):
            raise ValueError('Parking gain/deadband outside candidate boundary')
        fixed_contract=dict(first_fixed_parking_window_s=5.,maximum_xy_drift_m=.05,
            maximum_yaw_drift_rad=.1,maximum_native_xy_speed_mps=.08,
            maximum_native_Euler_yawrate_radps=.1,feedback_TTL_sim_and_wall_s=.3,
            command_slew_per_s=[.6,.6,.8],actual_command_must_be_zero=False,
            continuous_Teacher_inference_required=True,action_zero_is_parking=False,
            SLAM_verified=False,real_robot_verified=False)
        if any(self.parking.get(k)!=v for k,v in fixed_contract.items()):
            raise ValueError('Frozen parking numerical/source contract differs')
        self.hold_integral=np.zeros(3)
        self.capture_stamp=None;self.hold_stamp=None;self.capture_dwell_start=None
        self.arrival_pose=None;self.capture_elapsed=None;self.capture_world=None
        self.hold_elapsed=None;self.hold_world=None

    def _metadata(self, entry_reset=False):
        self.row.update(active_parking_enabled=True,
            parking_reference_kind='prospective_fixed_path_endpoint',
            parking_target_world_xyz=list(self._target['position_world_xyz']),
            parking_target_yaw_rad=self._target['heading0_rad'],parking_target_sha256=self._target_sha,
            parking_capture_elapsed_s=self.capture_elapsed,parking_capture_world_time_s=self.capture_world,
            parking_capture_state_time_s=self.capture_stamp,
            parking_hold_declared_elapsed_s=self.hold_elapsed,parking_hold_declared_world_time_s=self.hold_world,
            parking_hold_declared_state_time_s=self.hold_stamp,
            arrival_capture_position_world=None if self.arrival_pose is None else list(self.arrival_pose['position_world_xyz']),
            arrival_capture_yaw_rad=None if self.arrival_pose is None else self.arrival_pose['yaw_rad'],
            entry_integral_reset=bool(entry_reset),parking_command_limits_body=self.hold_limits.tolist(),
            v1_drive_source_prefix_sha256=V1_DRIVE_SOURCE_SHA256,
            v1_drive_source_prefix_size_bytes=V1_DRIVE_SOURCE_SIZE_BYTES,
            parking_is_old_zero_command_protocol=False)

    def _limit_hold(self, raw):
        limited=np.clip(raw,-self.hold_limits,self.hold_limits)
        norm=np.linalg.norm(limited[:2])
        if norm>self.hold_limits[0]:limited[:2]*=self.hold_limits[0]/norm
        return limited

    def _hold_fresh(self, s, t, stamp, dt, entry_reset=False):
        R=rotation(s[4:8]);yaw=math.atan2(R[1,0],R[0,0])
        roll=math.atan2(R[2,1],R[2,2]);pitch=math.asin(np.clip(-R[2,0],-1,1))
        if abs(math.cos(pitch))<.4 or abs(math.cos(roll))<.4:
            raise ValueError('Unsafe active parking yaw kinematics')
        yawdot=(math.sin(roll)*s[12]+math.cos(roll)*s[13])/math.cos(pitch)
        origin_body=s[8:11]-np.cross(s[11:14],self.com_offset)
        origin_world=R@origin_body
        error_world=np.asarray(self._target['position_world_xyz'])[:2]-s[1:3]
        error_yaw=wrap(self._target['heading0_rad']-yaw)
        distance=float(np.linalg.norm(error_world))
        if distance>self.parking['target_loss_radius_m'] or abs(error_yaw)>self.parking['target_loss_heading_rad']:
            self.fault='parking_target_lost';self.hold_integral[:]=0.
            self._zero_row(s,t,self.fault);self._metadata(entry_reset)
            return
        measurement=np.r_[origin_world[:2],yawdot]
        actual=np.r_[s[8:10],s[13]]
        # The first capture frame was already filtered by the unchanged drive
        # controller. Never count that physical measurement twice.
        if not entry_reset:
            alpha=1. if self.filtered is None else dt/(.2+dt)
            self.filtered=measurement.copy() if self.filtered is None else self.filtered+alpha*(measurement-self.filtered)
            beta=1. if self.inner_filtered is None else dt/(.1+dt)
            self.inner_filtered=actual.copy() if self.inner_filtered is None else self.inner_filtered+beta*(actual-self.inner_filtered)
        xy_radius=self.parking['capture_xy_hold_m'] if self.capture_dwell_start is not None else self.parking['capture_xy_entry_m']
        yaw_radius=self.parking['capture_yaw_hold_rad'] if self.capture_dwell_start is not None else self.parking['capture_yaw_entry_rad']
        eligible=(distance<=xy_radius and abs(error_yaw)<=yaw_radius and
                  np.linalg.norm(origin_body[:2])<self.parking['capture_actual_xy_speed_mps'] and
                  abs(yawdot)<self.parking['capture_actual_yawrate_radps'])
        if self.hold_stamp is None:
            if eligible:
                if self.capture_dwell_start is None:self.capture_dwell_start=stamp
                if stamp-self.capture_dwell_start>=self.parking['capture_fresh_dwell_s']-1e-9:
                    self.hold_stamp=stamp;self.hold_world=float(s[0]);self.hold_elapsed=t;self.completed_t=t
                    self.waypoint_events.append(dict(kind='active_hold_declared',state_physics_time_s=stamp,
                        world_time_s=float(s[0]),elapsed_s=t,target_sha256=self._target_sha))
            else:self.capture_dwell_start=None
        mode='active_hold' if self.hold_stamp is not None else 'capture'
        position_P=self.parking['position_kp']*_deadband(error_world,self.parking['position_deadband_m'])
        position_D=-self.parking['position_kd']*self.filtered[:2]
        velocity_world=np.r_[position_P+position_D,0.]
        xy_norm=np.linalg.norm(velocity_world[:2]);xy_limit=self.parking['reference_xy_norm_limit_mps']
        if xy_norm>xy_limit:velocity_world[:2]*=xy_limit/xy_norm
        yaw_P=self.parking['yaw_kp']*float(_deadband(error_yaw,self.parking['yaw_deadband_rad']))
        yaw_D=-self.parking['yaw_kd']*self.filtered[2]
        desired_yawdot=float(np.clip(yaw_P+yaw_D,-self.parking['reference_yaw_limit_radps'],self.parking['reference_yaw_limit_radps']))
        body_wref=(desired_yawdot*math.cos(pitch)-math.sin(roll)*s[12])/math.cos(roll)
        origin_ref_body=R.T@velocity_world
        com_ref=origin_ref_body+np.cross(np.array([0.,0.,body_wref]),self.com_offset)
        target=np.r_[com_ref[:2],body_wref];ev=target-self.inner_filtered
        g=self.g;kp=np.array([g['velocity_kp_x'],g['velocity_kp_y'],g['rate_kp']])
        ki=np.array([g['velocity_ki_x'],g['velocity_ki_y'],g['rate_ki']])
        candidate=np.clip(self.hold_integral+dt*ev,-.5,.5)
        raw=target+kp*ev+ki*candidate;limited=self._limit_hold(raw)
        blocked_axis=ev*(raw-limited)>0
        candidate=np.where(blocked_axis,self.hold_integral,candidate)
        next_slew=self.applied_command_estimate+np.clip(limited-self.applied_command_estimate,-SLEW*.02,SLEW*.02)
        blocked_slew=ev*(raw-next_slew)>1e-9
        candidate=np.where(blocked_slew,self.hold_integral,candidate)
        self.hold_integral=candidate
        raw=target+kp*ev+ki*candidate;limited=self._limit_hold(raw)
        self.command=limited
        self.progress,projection=self.path.project(s[1:4],self.progress)
        self.row=dict(kind='controller_update',controller_updated=True,
            control_t_s=float(s[0]),feedback_time_s=stamp,elapsed_s=t,header_dt_s=dt,
            active_integral_dt_s=dt,feedback_hz=self.hz,design='curvature_cascade_pi_active_hold',
            mode=mode,segment=0,reference_xy=self._target['position_world_xyz'][:2],
            reference_yaw=self._target['heading0_rad'],reference_velocity_world=velocity_world.tolist(),
            error_cross=float(-error_world@self.path.geometry(self.progress)[2][:2]),
            error_along=float(error_world@self.path.geometry(self.progress)[1][:2]),error_yaw=error_yaw,
            arc_progress_m=self.progress,path_length_m=self.path.length,remaining_arc_m=self.path.length-self.progress,
            endpoint_xy_distance_m=distance,path_curvature_1pm=0.,planned_tangential_speed_mps=0.,
            yaw_feedforward_radps=0.,reference_geometric_yaw_rate_radps=desired_yawdot,
            yaw_PD=dict(P=yaw_P,D=yaw_D),position_terms=dict(P=position_P.tolist(),D=position_D.tolist(),I=[0.,0.]),
            v_reference_body=target.tolist(),command_body=limited.tolist(),raw_command_body=raw.tolist(),
            velocity_PI=dict(error=ev.tolist(),P=(kp*ev).tolist(),I=(ki*candidate).tolist(),
                filtered_actual_body=self.inner_filtered.tolist(),blocked_axis=blocked_axis.tolist(),blocked_slew=blocked_slew.tolist()),
            measured_origin_velocity_world=origin_world.tolist(),measured_origin_velocity_body=origin_body.tolist(),
            measured_COM_velocity_body=s[8:11].tolist(),measured_yaw_rate=yawdot,
            COM_reference_angular_assumption='Desired body omega_xy=0 for COM lever arm; measured omega_y used in Euler conversion',
            saturated=(abs(raw-limited)>1e-9).tolist(),completed_t_s=self.completed_t,
            capture_fresh_dwell_start_state_time_s=self.capture_dwell_start,capture_measurement_eligible=bool(eligible),
            parking_reference_yaw_rate_radps=desired_yawdot,parking_reference_velocity_world=velocity_world.tolist(),
            parking_position_error_world=error_world.tolist(),parking_heading_error_rad=error_yaw,
            goal_entry_radius_m=.12,goal_hold_radius_m=.14,progress_required_before_final_dwell_m=self.path.length-.15,
            projection=projection,path_parameters_sha256=self.path.sha256,route_reference_changes=0,
            feedback_measurement_fresh=True,feedback_source='Gazebo native truth; active endpoint parking calibration only',
            counts_as_SLAM_navigation=False,fault=None)
        self._metadata(entry_reset)

    def update(self, s, t):
        s=np.asarray(s,float);t=float(t)
        if self.capture_stamp is None:
            estimate_before=self.applied_command_estimate.copy()
            command,mode=super().update(s,t)
            if mode=='goal_dwell' and self.fault is None:
                self.capture_stamp=float(s[0]-.005);self.capture_world=float(s[0]);self.capture_elapsed=t
                R=rotation(s[4:8]);self.arrival_pose=dict(position_world_xyz=s[1:4].tolist(),
                    yaw_rad=math.atan2(R[1,0],R[0,0]))
                self.hold_integral[:]=0.;self.dwell=None;self.completed_t=None
                # Parent performed its zero-command slew once; replace that
                # estimate with exactly one Teacher slew toward active output.
                self.applied_command_estimate=estimate_before
                dt=float(self.row['header_dt_s'])
                receipt_gap=float(self.row['previous_fresh_wall_receipt_gap_s'])
                self._hold_fresh(s,t,self.capture_stamp,dt,entry_reset=True)
                self.applied_command_estimate+=np.clip(self.command-self.applied_command_estimate,-SLEW*.02,SLEW*.02)
                self.row.update(estimated_teacher_command_body=self.applied_command_estimate.tolist(),
                    feedback_age_sim_s=0.,feedback_age_wall_s=0.,previous_fresh_wall_receipt_gap_s=receipt_gap)
                return self.command.copy(),self.row['mode']
            self._metadata(False)
            return command,mode
        if s.shape!=(64,) or not np.isfinite(s).all() or np.linalg.norm(s[4:8])<.9:
            raise ValueError('Invalid native64 active parking state')
        if self.last_world is not None and s[0]<=self.last_world:
            raise ValueError('Nonincreasing active parking native clock')
        self.last_world=float(s[0]);self.frame+=1
        stamp=float(s[0]-.005);now=time.monotonic()
        receipt_gap=0. if self.last_wall is None else now-self.last_wall
        dt=0. if self.last_feedback is None else stamp-self.last_feedback
        updated=self.frame%self.stride==0
        if self.fault is None and (dt<0 or dt>.30000001 or receipt_gap>.3):
            self.fault='feedback_timeout';self.hold_integral[:]=0.;self.integral[:]=0.;self.velocity_integral[:]=0.
        if self.fault is not None:
            self._zero_row(s,t,self.fault);self._metadata(False)
        elif updated:
            self.last_feedback=stamp;self.last_wall=now
            self._hold_fresh(s,t,stamp,dt,False)
        else:
            self.row={**self.row,'kind':'controller_hold', 'controller_updated':False,
                'control_t_s':float(s[0]),'elapsed_s':t,'command_body':self.command.tolist(),
                'feedback_measurement_fresh':False}
            self._metadata(False)
        sim_age=stamp-self.last_feedback if self.last_feedback is not None else 0.
        wall_age=now-self.last_wall if self.last_wall is not None else 0.
        self.applied_command_estimate+=np.clip(self.command-self.applied_command_estimate,-SLEW*.02,SLEW*.02)
        self.row.update(feedback_age_sim_s=sim_age,feedback_age_wall_s=wall_age,
            previous_fresh_wall_receipt_gap_s=receipt_gap,
            estimated_teacher_command_body=self.applied_command_estimate.tolist())
        return self.command.copy(),self.row['mode']
