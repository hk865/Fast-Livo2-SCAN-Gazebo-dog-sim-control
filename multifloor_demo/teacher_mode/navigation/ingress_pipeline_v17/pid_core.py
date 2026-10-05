"""Header-event PID of measured SLAM XY/yaw into bounded body velocity only."""
import math
import numpy as np


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def slew(previous, desired, dt, acceleration):
    previous=np.asarray(previous,dtype=float);desired=np.asarray(desired,dtype=float)
    result=previous+np.clip(desired-previous,-np.asarray(acceleration)*max(0.,min(dt,.1)),
                            np.asarray(acceleration)*max(0.,min(dt,.1)))
    if np.linalg.norm(desired[:2])<1e-9:result[:2]=0.
    return result


class HeaderPID:
    def __init__(self,profile):
        self.config=profile['pid'];self.limits=np.array([profile['max_speed_mps'],profile['max_lateral_speed_mps'],profile['max_yaw_rate_radps']])
        if np.any(self.limits<=0)or np.any(self.limits>np.array([.3,.2,.3])):raise ValueError('PID exceeds frozen Teacher command range')
        for k,v in self.config.items():
            if isinstance(v,(int,float))and (not math.isfinite(v)or v<0):raise ValueError('Invalid PID gain/limit')
        self.last_stamp=None;self.last_key=None;self.filtered=None;self.resets=0;self.reset('startup')
    def reset(self,reason):
        # Preserve the source watermark: a protected duplicate cannot be used
        # as another dt event or resurrect an old nonzero output.
        self.integral=np.zeros(3);self.command=np.zeros(3);self.last_key=None
        self.filtered=None;self.reset_reason=reason;self.resets+=1
    def update(self,stamp_ns,pose,yaw,world_velocity,body_wz,target,heading,mode,key,clock_ns):
        values=np.r_[pose,yaw,world_velocity,body_wz,target,heading]
        if not isinstance(stamp_ns,int)or not np.isfinite(values).all():
            self.reset('invalid_measurement');raise ValueError('Invalid measured PID input')
        if not -50_000_000<=clock_ns-stamp_ns<300_000_000:
            self.reset('stale_measurement');return self.command.copy(),{'updated':False,'reason':'stale_measurement'}
        if self.last_stamp is not None and stamp_ns<self.last_stamp:
            self.reset('backward_header');raise ValueError('PID source clock moved backward')
        if self.last_stamp==stamp_ns:
            # A new protected mode/setpoint cannot re-use the old output.
            if mode=='hold' or key!=self.last_key:self.reset('duplicate_after_transition')
            return self.command.copy(),{'updated':False,'reason':'duplicate_header','header_dt_s':0.}
        dt=0. if self.last_stamp is None else (stamp_ns-self.last_stamp)/1e9
        self.last_stamp=stamp_ns
        if dt>self.config['header_gap_max_s']:
            self.reset('header_gap');dt=0.
        if key!=self.last_key:self.reset('goal_or_mode_change');self.last_key=key;dt=0.
        if mode=='hold':
            self.reset('hold');self.last_key=key
            return self.command.copy(),{'updated':True,'reason':'hold','header_dt_s':dt,'integral':self.integral.tolist()}
        if mode not in ('drive','turn'):raise ValueError('Unknown PID mode')
        measurement=np.r_[np.asarray(world_velocity)[:2],float(body_wz)]
        alpha=1. if self.filtered is None else dt/(self.config['derivative_filter_tau_s']+dt)
        self.filtered=measurement.copy()if self.filtered is None else self.filtered+alpha*(measurement-self.filtered)
        error=np.r_[np.asarray(target)[:2]-np.asarray(pose)[:2],wrap(heading-yaw)]
        if mode=='turn':error[:2]=0.;self.integral[:2]=0.
        kp=np.array([self.config['position_kp']]*2+[self.config['yaw_kp']])
        ki=np.array([self.config['position_ki']]*2+[self.config['yaw_ki']])
        kd=np.array([self.config['position_kd']]*2+[self.config['yaw_kd']])
        bound=np.array([self.config['position_integral_limit']]*2+[self.config['yaw_integral_limit']])
        candidate=np.clip(self.integral+dt*error,-bound,bound)
        c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s],[s,c]])
        def calculate(integral):
            p=kp*error;i=ki*integral;d=-kd*self.filtered
            if mode=='turn':p[:2]=i[:2]=d[:2]=0.
            world=p+i+d;body=np.r_[rotation.T@world[:2],world[2]]
            clipped=np.clip(body,-self.limits,self.limits)
            # A forward checked path does not authorize reversing away from it.
            if mode=='drive':clipped[0]=max(0.,clipped[0])
            axis_clipped=clipped.copy()
            norm=np.linalg.norm(clipped[:2])
            if norm>self.limits[0]:clipped[:2]*=self.limits[0]/norm
            return p,i,d,world,body,clipped,axis_clipped
        p,i,d,world,raw,clipped,axis_clipped=calculate(candidate)
        error_body=rotation.T@error[:2]
        blocked_body=error_body*(raw[:2]-axis_clipped[:2])>1e-12
        # Reject only the increment that worsens that body-axis saturation.
        # A saturated forward axis must not prevent lateral-bias integration.
        # The final planar-norm limit preserves direction and is deliberately
        # separate from per-axis integral rejection.
        increment_body=rotation.T@(candidate[:2]-self.integral[:2])
        increment_body=np.where(blocked_body,0.,increment_body)
        candidate[:2]=np.clip(self.integral[:2]+rotation@increment_body,-bound[:2],bound[:2])
        blocked_yaw=error[2]*(raw[2]-clipped[2])>1e-12
        if blocked_yaw:candidate[2]=self.integral[2]
        blocked=np.r_[blocked_body,blocked_yaw];self.integral=candidate
        p,i,d,world,raw,clipped,axis_clipped=calculate(candidate);self.command=clipped
        return clipped.copy(),{'updated':True,'header_dt_s':dt,'mode':mode,'error_world_xy_yaw':error.tolist(),
            'error_body_xy':(rotation.T@error[:2]).tolist(),'filtered_measured_world_vxy_body_wz':self.filtered.tolist(),
            'derivative_filter_alpha':alpha,'P_world_xy_yaw':p.tolist(),'I_world_xy_yaw':i.tolist(),
            'D_world_xy_yaw':d.tolist(),'integral':candidate.tolist(),'integral_limits':bound.tolist(),
            'antiwindup_blocked_axes':blocked.tolist(),'raw_body_command':raw.tolist(),
            'clipped_body_command':clipped.tolist(),'reset_reason':self.reset_reason,'reset_count':self.resets,
            'source':'actual raw SLAM XY/yaw/twist and actual transformed IMU gyro; no simulator navigation state'}
