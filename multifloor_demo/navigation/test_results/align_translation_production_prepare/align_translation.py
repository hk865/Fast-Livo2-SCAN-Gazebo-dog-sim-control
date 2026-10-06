"""Excluded ALIGN-only measured-SLAM anchor PD candidate; no ROS or GT.

This estimator observes raw, monotonically advancing native Header samples.
Its .4s least-squares slope has about .2s effective measurement lag. It is not
instantaneous gait feedback, and no integral or predicted plant is present.
"""
from collections import deque
import math
import numpy as np

class AlignTranslation:
    CONFIG=dict(kp_per_s=.8,kd=.4,max_abs_vx_m_s=.10,slew_m_s2=.2,
        window_ns=400000000,min_fit_span_ns=350000000,min_raw_observations=3,
        min_fit_samples=4,max_pose_gap_ns=200000000,max_pose_age_ns=250000000,
        position_fit_rms_limit_m=.02,anchor_deadband_m=.01,velocity_deadband_m_s=.015,
        mixed_yaw_cap_rad_s=.08,vy_m_s=0.,integral_enabled=False)
    def __init__(self):self.reset('initial')
    def reset(self,reason):
        self.context=None;self.anchor=None;self.last_stamp=None;self.last_wall=None
        self.samples=deque(maxlen=16);self.last_output_stamp=None;self.last_output_wall=None;self.command=0.
        self.latest=dict(valid=False,reason=reason,vx_m_s=0.,ground_truth_consumed=False)
    @staticmethod
    def ns(t):
        if type(t) is not int or t<0:raise ValueError('integer nonnegative native nanoseconds required')
        return t
    @staticmethod
    def geometry(p,R):
        p=np.asarray(p,dtype=float);R=np.asarray(R,dtype=float)
        if p.shape!=(3,) or R.shape!=(3,3) or not np.isfinite(p).all() or not np.isfinite(R).all():raise ValueError('finite raw position/rotation required')
        if np.max(np.abs(R.T@R-np.eye(3)))>1e-6 or abs(np.linalg.det(R)-1.)>1e-6:raise ValueError('proper raw rotation required')
        return p.copy(),R.copy()
    def begin(self,context,stamp_ns,p,R,received_wall_ns):
        if not isinstance(context,tuple) or len(context)!=4:raise ValueError('immutable request/goal/traj/reference context required')
        p,R=self.geometry(p,R);t=self.ns(stamp_ns);wall=self.ns(received_wall_ns)
        self.reset('waiting_fresh_fit');self.context=context;self.anchor=p.copy()
        self.last_stamp=t;self.last_wall=wall;self.samples.append((t,p,R))
    def observe(self,context,stamp_ns,p,R,received_wall_ns):
        t=self.ns(stamp_ns);wall=self.ns(received_wall_ns);p,R=self.geometry(p,R)
        if self.context is None:return False
        if context!=self.context:self.reset('foreign_identity');return False
        if t<=self.last_stamp:return False  # Duplicate never refreshes age/count.
        if wall<self.last_wall or t-self.last_stamp>self.CONFIG['max_pose_gap_ns']:
            self.reset('raw_gap_or_wall_reset');return False
        self.last_stamp=t;self.last_wall=wall;self.samples.append((t,p,R))
        while len(self.samples)>1 and self.samples[0][0]<t-self.CONFIG['window_ns']-1:self.samples.popleft()
        return True
    @staticmethod
    def deadband(x,width):return math.copysign(max(0.,abs(x)-width),x)
    def output(self,context,now_ns,now_wall_ns,*,phase,protected=False):
        now=self.ns(now_ns);wall=self.ns(now_wall_ns)
        def invalid(reason,reset=False):
            if reset:self.reset(reason)
            else:
                self.command=0.;self.last_output_stamp=now;self.last_output_wall=wall
                self.latest=dict(valid=False,reason=reason,vx_m_s=0.,ground_truth_consumed=False)
            return 0.
        if phase!='align' or protected:return invalid('phase_or_protection',True)
        if context!=self.context or self.context is None:return invalid('identity_or_no_anchor',True)
        if ((self.last_output_stamp is not None and now<self.last_output_stamp)
                or (self.last_output_wall is not None and wall<self.last_output_wall)):
            return invalid('output_clock_reset',True)
        if now<self.last_stamp or wall<self.last_wall:return invalid('future_or_clock_reset',True)
        age=now-self.last_stamp;wall_age=wall-self.last_wall
        if max(age,wall_age)>self.CONFIG['max_pose_age_ns']:return invalid('stale_raw_pose',True)
        rows=list(self.samples)
        if len(rows)<max(self.CONFIG['min_raw_observations'],self.CONFIG['min_fit_samples']):return invalid('waiting_fresh_fit')
        span=rows[-1][0]-rows[0][0]
        if span<self.CONFIG['min_fit_span_ns']:return invalid('waiting_fit_span')
        x=np.array([(r[0]-rows[-1][0])/1e9 for r in rows]);X=np.column_stack([np.ones(len(rows)),x])
        P=np.array([r[1] for r in rows]);fit=np.linalg.lstsq(X,P,rcond=None)[0]
        rms=float(np.sqrt(np.mean(np.sum((P-X@fit)**2,axis=1))))
        if not math.isfinite(rms) or rms>self.CONFIG['position_fit_rms_limit_m']:return invalid('fit_residual')
        R=rows[-1][2];e=float((R.T@(self.anchor-rows[-1][1]))[0]);v=float((R.T@fit[1])[0])
        target=self.CONFIG['kp_per_s']*self.deadband(e,self.CONFIG['anchor_deadband_m'])-self.CONFIG['kd']*self.deadband(v,self.CONFIG['velocity_deadband_m_s'])
        target=float(np.clip(target,-self.CONFIG['max_abs_vx_m_s'],self.CONFIG['max_abs_vx_m_s']))
        dt=0. if self.last_output_stamp is None else max(0.,min(.1,(now-self.last_output_stamp)/1e9))
        self.command+=float(np.clip(target-self.command,-self.CONFIG['slew_m_s2']*dt,self.CONFIG['slew_m_s2']*dt))
        self.last_output_stamp=now;self.last_output_wall=wall
        self.latest=dict(valid=True,reason='raw_pose_anchor_pd',stamp_ns=self.last_stamp,
            anchor=self.anchor.tolist(),context=self.context,sample_count=len(rows),span_ns=span,
            effective_delay_ns=int(rows[-1][0]-sum(r[0] for r in rows)//len(rows)),
            pose_age_ns=age,pose_wall_age_ns=wall_age,fit_rms_m=rms,
            anchor_body_x_error_m=e,measured_body_vx_m_s=v,requested_vx_m_s=target,
            vx_m_s=float(self.command),ground_truth_consumed=False)
        return float(self.command)
