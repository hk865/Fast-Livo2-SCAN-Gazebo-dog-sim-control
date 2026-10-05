"""Staging only: bounded terrain-relative body roll/pitch feedback, no pose control.

All positions below are measured foot positions in BODY coordinates from FK.
The module never receives robot world position, SLAM pose, or a navigation goal.
"""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class Config:
    enabled: bool = False
    kp: float = .25
    kd: float = .04
    angle_limit: float = .06
    rate_limit: float = .25
    error_tau: float = .12
    gyro_tau: float = .04
    terrain_tau: float = 1.0
    terrain_age_limit: float = 1.2
    support_age_limit: float = .20
    terrain_max_tilt: float = .25
    plane_residual_limit: float = .008

def plane_normal(points):
    """Reject two feet/collinear feet: a diagonal alone cannot observe a plane."""
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<3 or not np.isfinite(points).all():
        return None
    centered=points-points.mean(axis=0)
    _,s,vh=np.linalg.svd(centered,full_matrices=False)
    if len(s)<2 or s[1]<.025:return None
    n=vh[-1]
    if n[2]<0:n=-n
    return n,float(np.max(np.abs(centered@n)))

def finite_rotation(R):
    R=np.asarray(R)
    return R.shape==(3,3) and np.isfinite(R).all() and np.allclose(R.T@R,np.eye(3),atol=1e-5) and abs(np.linalg.det(R)-1)<1e-5

def relative_tilt(rotation_world_body,normal_world):
    n=rotation_world_body.T@normal_world
    return np.array([math.atan2(n[1],n[2]),math.atan2(-n[0],math.hypot(n[1],n[2]))])

class Feedback:
    def __init__(self,config=Config()):
        self.c=config;self.last_stamp=None;self.plane_stamp=None;self.support_stamp=None
        self.last_stamp_ns=None;self.plane_stamp_ns=None;self.support_stamp_ns=None
        self.normal_world=None;self.error_filtered=np.zeros(2);self.gyro_filtered=np.zeros(2)
        self.output=np.zeros(2);self.failed=None;self.established=False;self.plane_updates=0
        self.diagnostic={}
    def fail(self,reason):
        self.failed=reason;self.output=np.zeros(2)
    def observe_support(self,stamp,rotation_world_body,feet_body,foot_velocity_body,contacts,omega_body,*,stamp_ns=None):
        # ROS callers provide the original integer header, not an epoch float
        # reconstructed here. The fallback supports standalone math fixtures.
        ns=round(stamp*1e9) if stamp_ns is None else stamp_ns
        feet=np.asarray(feet_body);vf=np.asarray(foot_velocity_body);mask=np.asarray(contacts,dtype=bool)
        if (feet.shape!=(4,3) or vf.shape!=(4,3) or mask.shape!=(4,) or not np.isfinite(feet).all()
                or not np.isfinite(vf).all() or not finite_rotation(rotation_world_body)
                or np.asarray(omega_body).shape!=(3,) or not np.isfinite(omega_body).all()):
            self.fail('invalid measured foot geometry');return
        ids=np.flatnonzero(mask)
        if len(ids)>=2:self.support_stamp=stamp;self.support_stamp_ns=ns
        # Estimate body velocity from stance-foot kinematics only. It is a
        # no-slip hypothesis/diagnostic, never a ground-truth measurement or
        # an additional velocity-controller input in this prototype.
        candidates=-(vf[ids]+np.cross(np.asarray(omega_body),feet[ids]))
        velocity=np.median(candidates,axis=0) if len(ids) else None
        scatter=float(np.max(np.linalg.norm(candidates-velocity,axis=1))) if len(ids) else None
        self.diagnostic.update(contact_count=len(ids),body_velocity_contact_estimate=None if velocity is None else velocity.tolist(),
            velocity_candidate_scatter=scatter,velocity_scope='stance FK + actual gyro; no-slip assumption; diagnostic only')
        fit=plane_normal(feet[ids])
        if fit is None:return
        normal_body,residual=fit
        normal_world=rotation_world_body@normal_body
        if normal_world[2]<0:normal_world=-normal_world
        if residual>self.c.plane_residual_limit or normal_world[2]<math.cos(self.c.terrain_max_tilt):return
        if self.plane_stamp_ns is not None and ns<=self.plane_stamp_ns:return
        if self.normal_world is None:self.normal_world=normal_world
        else:
            dt=(ns-self.plane_stamp_ns)/1e9;alpha=1-math.exp(-dt/self.c.terrain_tau)
            n=(1-alpha)*self.normal_world+alpha*normal_world;self.normal_world=n/np.linalg.norm(n)
        self.plane_stamp=stamp;self.plane_stamp_ns=ns;self.plane_updates+=1
        self.diagnostic.update(plane_residual=residual,normal_world=self.normal_world.tolist(),plane_updates=self.plane_updates)
    def step(self,stamp,rotation_world_body,omega_body,*,actual_moving,inputs_fresh=True,stamp_ns=None):
        ns=round(stamp*1e9) if stamp_ns is None and math.isfinite(stamp) else stamp_ns
        if not math.isfinite(stamp) or (self.last_stamp_ns is not None and ns<self.last_stamp_ns-1):
            self.fail('simulation time reversed or invalid')
        dt=0. if self.last_stamp_ns is None or ns is None else (ns-self.last_stamp_ns)/1e9
        self.last_stamp=stamp;self.last_stamp_ns=ns
        if not self.c.enabled:
            self.output=np.zeros(2);return self.output.copy(),'disabled'
        if self.failed:return np.zeros(2),'failed'
        if not finite_rotation(rotation_world_body) or np.asarray(omega_body).shape!=(3,) or not np.isfinite(omega_body).all():
            self.fail('invalid actual IMU');return np.zeros(2),'failed'
        supported=self.support_stamp_ns is not None and 0<=ns-self.support_stamp_ns<=round(self.c.support_age_limit*1e9)
        plane_ok=self.plane_stamp_ns is not None and 0<=ns-self.plane_stamp_ns<=round(self.c.terrain_age_limit*1e9)
        ready=inputs_fresh and supported and plane_ok
        if not ready:
            self.output=np.zeros(2)
            if self.established:self.fail('measured IMU/joints/support/terrain stale')
            return self.output.copy(),'failed' if self.failed else 'waiting'
        self.established=True
        error=relative_tilt(rotation_world_body,self.normal_world)
        if dt>0:
            self.error_filtered+=(1-math.exp(-dt/self.c.error_tau))*(error-self.error_filtered)
            self.gyro_filtered+=(1-math.exp(-dt/self.c.gyro_tau))*(np.asarray(omega_body)[:2]-self.gyro_filtered)
        self.diagnostic.update(tilt_relative_to_support=error.tolist(),filtered_error=self.error_filtered.tolist(),
            filtered_gyro=self.gyro_filtered.tolist(),plane_age=(ns-self.plane_stamp_ns)/1e9,support_age=(ns-self.support_stamp_ns)/1e9,
            plane_age_ns=ns-self.plane_stamp_ns,support_age_ns=ns-self.support_stamp_ns)
        if not actual_moving:
            # Exact native neutral posture is required by the separately
            # verified stop adapter; that adapter provides joint C0 return.
            self.output=np.zeros(2);return self.output.copy(),'neutral_for_stop'
        target=np.clip(-self.c.kp*self.error_filtered-self.c.kd*self.gyro_filtered,-self.c.angle_limit,self.c.angle_limit)
        self.output+=np.clip(target-self.output,-self.c.rate_limit*min(max(dt,0),.02),self.c.rate_limit*min(max(dt,0),.02))
        return self.output.copy(),'active'
