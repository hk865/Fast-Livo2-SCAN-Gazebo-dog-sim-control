"""Teacher stop/heading transitions proven from fresh SLAM and raw body gyro.

No truth, timer-only standstill, CHAMP phase reset or IMU yaw localization.

Independent simulation experiment: allow profile turn caps up to 0.6 rad/s.
The NAV 0.6 cap has no physical validation yet. At cap 0.3 all transition
mathematics, measured-stop conditions and 300 ms freshness are unchanged.
"""
from collections import deque
import math


class TeacherHeadingGate:
    def __init__(self,profile):
        self.max_turn_rate=profile['max_yaw_rate_radps']
        if not math.isfinite(self.max_turn_rate)or not 0<self.max_turn_rate<=.6:raise ValueError('Simulation experimental turn limit must be finite and within (0,0.6]rad/s; NAV0.6 is not physically validated')
        self.config=profile['teacher_transition'];self.history=deque(maxlen=64)
        self.last_stamp=None;self.reset()
    @staticmethod
    def error(target,yaw):return math.atan2(math.sin(target-yaw),math.cos(target-yaw))
    def reset(self):
        self.phase='pre_turn';self.heading=None;self.settle_until=None;self.outside_since=None
        # Goal/replan resets heading, not the real observed standstill history.
    def observe(self,stamp_s,wall_s,velocity,slam_wz,body_gyro,gyro_stamp_s,zero_command):
        values=list(velocity)+[stamp_s,wall_s,slam_wz,gyro_stamp_s]+list(body_gyro)
        if len(velocity)!=3 or len(body_gyro)!=3 or not all(math.isfinite(x)for x in values):return False
        if self.last_stamp is not None and stamp_s<=self.last_stamp:return False
        self.last_stamp=stamp_s;c=self.config
        valid=(zero_command and math.hypot(*velocity[:2])<=c['stop_planar_speed_mps']
            and abs(slam_wz)<=c['stop_slam_yaw_rate_radps']
            and math.sqrt(sum(x*x for x in body_gyro))<=c['stop_body_gyro_radps']
            and abs(stamp_s-gyro_stamp_s)<=c['gyro_pose_pair_max_s'])
        if not valid or self.history and stamp_s-self.history[-1][0]>c['stop_pose_gap_max_s']:
            self.history.clear()
        if valid:self.history.append((stamp_s,wall_s))
        return valid
    def stopped(self,sim_time,wall_time):
        c=self.config
        return (len(self.history)>=c['stop_min_pose_samples']
            and 0<=wall_time-self.history[-1][1]<.3
            and -.05<=sim_time-self.history[-1][0]<.3
            and self.history[-1][0]-self.history[0][0]>=c['stop_window_sim_s']-1e-9)
    def resume_after_stop(self,yaw,fresh_path_heading,stopped_duration):
        # Caller retains unchanged cloud-clearance/new-SCAN guard. A duration
        # alone never establishes physical stopping for the Teacher.
        self.reset();self.heading=fresh_path_heading
        return False
    def update(self,sim_time,yaw,current_path_heading,wall_time=None):
        import time
        wall_time=time.monotonic()if wall_time is None else wall_time;c=self.config
        current_error=self.error(current_path_heading,yaw)
        if self.phase=='drive':
            if abs(current_error)<=c['drive_heading_max_rad']:
                return True,max(-.08,min(.08,.5*current_error))
            self.reset()  # Stop immediately before a substantial new heading.
        if self.phase=='pre_turn':
            if not self.stopped(sim_time,wall_time):return False,0.
            self.phase='align';self.heading=current_path_heading
        if self.heading is None:self.heading=current_path_heading
        error=self.error(self.heading,yaw)
        if self.phase=='align':
            if abs(error)>c['turn_complete_rad']:
                return False,max(-self.max_turn_rate,min(self.max_turn_rate,1.3*error))
            self.phase='settle'
        if self.phase=='settle':
            if abs(current_error)>c['drive_heading_max_rad']:
                self.phase='pre_turn';self.heading=None;return False,0.
            if not self.stopped(sim_time,wall_time):return False,0.
            self.phase='drive';self.heading=None
            return True,max(-.08,min(.08,.5*current_error))
        return False,0.
    def evidence(self,sim_time,wall_time):
        return {'transition':'teacher_measured_stop_heading_v1','phase':self.phase,
            'measured_stop':self.stopped(sim_time,wall_time),'stop_valid_pose_samples':len(self.history),
            'stop_first_stamp_s':self.history[0][0]if self.history else None,
            'stop_last_stamp_s':self.history[-1][0]if self.history else None,
            'navigation_heading_source':'SLAM body yaw only','gyro_use':'stop/tilt only; never localization'}
