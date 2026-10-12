"""Simulation-only causal truth-feedback route/velocity controller.

No ROS, simulator pose write, joint command, or training operation is available.
The only output is a bounded Teacher velocity command. References are immutable.
"""
import math
import time
import numpy as np


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def rotation(q):
    w,x,y,z=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


class Controller:
    def __init__(self, p):
        self.p=p;self.route=np.asarray(p['route_world_xyz'],dtype=float)
        self.hz=int(p['feedback_hz'])
        if self.hz not in (5,10,25,50):raise ValueError('Only exact native50Hz decimations are admitted')
        self.stride=50//self.hz;self.frame=-1;self.segment=0
        if self.route.ndim!=2 or self.route.shape[1]!=3 or len(self.route)<2 or not np.isfinite(self.route).all():raise ValueError('Invalid fixed route')
        if np.any(np.linalg.norm(np.diff(self.route[:,:2],axis=0),axis=1)<.1):raise ValueError('Degenerate route')
        self.limits=np.asarray(p.get('command_limits',[1.,.4,1.]))
        if self.limits.shape!=(3,) or np.any(self.limits<=0) or np.any(self.limits>[1.,.4,1.]):raise ValueError('Beyond archived command range')
        self.g=dict(cross_kp=.8,cross_kd=.2,cross_ki=0.,yaw_kp=1.3,yaw_kd=.15,yaw_ki=0.,
                    velocity_kp_x=.6,velocity_ki_x=.5,velocity_kp_y=.4,velocity_ki_y=.3,
                    rate_kp=.3,rate_ki=.2)
        self.g.update(p.get('gains',{}))
        if not all(math.isfinite(v) and 0<=v<=4 for v in self.g.values()):raise ValueError('Unbounded gains')
        if p['design'] not in ('legacy_position','path_pd','cascade_pi','open_loop'):raise ValueError('Unknown design')
        self.command=np.zeros(3);self.last_feedback=None;self.last_wall=None
        self.integral=np.zeros(3);self.velocity_integral=np.zeros(3);self.filtered=None
        self.last_mode=None;self.completed_t=None;self.dwell=None;self.row={}
        self.turn_phase=None;self.stop_since=None;self.inner_filtered=None
        self.applied_command_estimate=np.zeros(3)
        self.com_offset=np.asarray(p['base_com_offset'])
        self.waypoint_events=[]

    def update(self,s,t):
        self.frame+=1
        if not np.isfinite(s).all() or np.linalg.norm(s[4:8])<.9:raise ValueError('Nonfinite truth feedback')
        now=time.monotonic();stamp=float(s[0]-.005)
        updated=self.frame%self.stride==0
        if t<3. or self.completed_t is not None:
            self.command[:]=0.;updated=False
            mode='initializing' if t<3. else'parking'
            a,b=self.route[self.segment:self.segment+2];heading=math.atan2(b[1]-a[1],b[0]-a[0])
            self.row={**self.row,'kind':'controller_hold','mode':mode,'controller_updated':False,
                      'command_body':[0.,0.,0.],'raw_command_body':[0.,0.,0.],
                      'reference_xy':b[:2].tolist(),'reference_yaw':heading,
                      'reference_velocity_world':[0.,0.,0.],'v_reference_body':[0.,0.,0.],
                      'control_t_s':float(s[0]),'feedback_time_s':stamp,'elapsed_s':t,'completed_t_s':self.completed_t,
                      'feedback_hz':self.hz,'design':self.p['design'],'segment':self.segment,
                      'route_reference_changes':0,'counts_as_SLAM_navigation':False}
            self.applied_command_estimate+=np.clip(-self.applied_command_estimate,-np.array([.6,.6,.8])*.02,np.array([.6,.6,.8])*.02)
            return self.command.copy(),mode
        if updated:
            dt=0. if self.last_feedback is None else stamp-self.last_feedback
            if dt<0 or dt>.30000001:raise ValueError('Truth feedback clock gap')
            self.last_feedback=stamp;self.last_wall=now
            R=rotation(s[4:8]);yaw=math.atan2(R[1,0],R[0,0]);roll=math.atan2(R[2,1],R[2,2]);pitch=math.asin(np.clip(-R[2,0],-1,1))
            if abs(math.cos(pitch))<.4:raise ValueError('Unsafe yaw kinematics')
            yawdot=(math.sin(roll)*s[12]+math.cos(roll)*s[13])/math.cos(pitch)
            com_body=s[8:11];origin_body=com_body-np.cross(s[11:14],self.com_offset)
            origin_world=R@origin_body
            a,b=self.route[self.segment:self.segment+2];delta=b[:2]-a[:2]
            L=np.linalg.norm(delta);tan=delta/L;normal=np.array([-tan[1],tan[0]])
            along=float((s[1:3]-a[:2])@tan);cross=float((s[1:3]-a[:2])@normal);remaining=L-along
            distance=float(np.linalg.norm(b[:2]-s[1:3]))
            if distance<.12 and self.segment<len(self.route)-2:
                self.segment+=1;self.integral[:]=0.;self.velocity_integral[:]=0.;self.filtered=None
                self.waypoint_events.append({'segment':self.segment,'elapsed_s':t,'physics_time_s':stamp})
                a,b=self.route[self.segment:self.segment+2];delta=b[:2]-a[:2];L=np.linalg.norm(delta)
                tan=delta/L;normal=np.array([-tan[1],tan[0]])
                along=float((s[1:3]-a[:2])@tan);cross=float((s[1:3]-a[:2])@normal)
                remaining=L-along;distance=float(np.linalg.norm(b[:2]-s[1:3]))
            heading=math.atan2(tan[1],tan[0]);eyaw=wrap(heading-yaw)
            if self.p['design']!='open_loop' and self.turn_phase is None and abs(eyaw)>.20:
                self.turn_phase='pre_turn';self.stop_since=None
            if self.turn_phase=='pre_turn':
                stopped=np.linalg.norm(origin_body[:2])<.04 and abs(yawdot)<.05
                self.stop_since=(t if self.stop_since is None else self.stop_since)if stopped else None
                if self.stop_since is not None and t-self.stop_since>=.3:self.turn_phase='turn';self.stop_since=None
            elif self.turn_phase=='turn' and abs(eyaw)<.08 and abs(yawdot)<.08:
                self.turn_phase='settle';self.stop_since=t
            elif self.turn_phase=='settle' and t-self.stop_since>=.3:
                self.turn_phase=None;self.stop_since=None
            mode=self.turn_phase or'drive'
            if mode!=self.last_mode:
                self.integral[:]=0.;self.velocity_integral[:]=0.;self.filtered=None;self.inner_filtered=None;dt=0.;self.last_mode=mode
            measurement=np.r_[origin_world[:2],yawdot]
            alpha=1. if self.filtered is None else dt/(.2+dt)
            self.filtered=measurement.copy()if self.filtered is None else self.filtered+alpha*(measurement-self.filtered)
            reference_xy=a[:2]+tan*np.clip(along+.8,0,L)
            err=np.r_[reference_xy-s[1:3],eyaw]
            final=self.segment==len(self.route)-2
            goal_radius=.14 if self.dwell is not None else .12
            if final and distance<=goal_radius and np.linalg.norm(origin_body[:2])<.08 and abs(yawdot)<.1 and abs(eyaw)<.2:
                self.dwell=t if self.dwell is None else self.dwell
                mode='goal_dwell';self.command[:]=0.
                if t-self.dwell>=.6:self.completed_t=t;mode='parking'
            else:self.dwell=None
            speed=min(float(self.p['desired_speed']), .9*max(remaining,0.), math.sqrt(.7*max(remaining-.05,0.)))
            # Collinear grade changes do not require a stop at each geometric knot.
            if not final:
                next_delta=self.route[self.segment+2,:2]-b[:2]
                if abs(wrap(math.atan2(next_delta[1],next_delta[0])-heading))<.1:
                    speed=float(self.p['desired_speed'])
            velocity_world=np.r_[speed*tan, speed*(b[2]-a[2])/L]
            g=self.g
            active_dt=0. if mode in ('parking','goal_dwell','pre_turn','settle')else dt
            effective_limits=self.limits.copy()
            if self.p['design']=='legacy_position':effective_limits[0]=min(effective_limits[0],float(self.p['desired_speed']))
            outer_I=np.array([0.,0.,0.]);outer_P=np.zeros(3);outer_D=np.zeros(3)
            if self.p['design']=='legacy_position':
                kp=np.array([.6,.6,1.1]);ki=np.array([.08,.08,.04]);kd=np.array([.12,.12,.15])
                candidate=np.clip(self.integral+active_dt*err,[-.4,-.4,-.3],[.4,.4,.3])
                outer_P=kp*err;outer_I=ki*candidate;outer_D=-kd*self.filtered
                world=outer_P+outer_I+outer_D
                target_body=np.r_[(R.T@np.r_[world[:2],0.])[:2],world[2]]
                raw=target_body.copy();vel_terms={}
                # Reject increments that increase saturation in the same world direction.
                limited=np.clip(raw,-effective_limits,effective_limits)
                bodyerr=(R.T@np.r_[err[:2],0.])[:2]
                blocked=bodyerr*(raw[:2]-limited[:2])>0
                ibody=(R.T@np.r_[candidate[:2]-self.integral[:2],0.])[:2];ibody[blocked]=0.
                candidate[:2]=self.integral[:2]+(R@np.r_[ibody,0.])[:2]
                if err[2]*(raw[2]-limited[2])>0:candidate[2]=self.integral[2]
                self.integral=candidate
                outer_I=ki*candidate;world=outer_P+outer_I+outer_D
                raw=np.r_[(R.T@np.r_[world[:2],0.])[:2],world[2]]
            else:
                self.integral[1]=np.clip(self.integral[1]-active_dt*cross,-.3,.3)
                self.integral[2]=np.clip(self.integral[2]+active_dt*eyaw,-.3,.3)
                lateral=-g['cross_kp']*cross-g['cross_kd']*float(self.filtered[:2]@normal)+g['cross_ki']*self.integral[1]
                velocity_world[:2]+=np.clip(lateral,-.20,.20)*normal
                wref=g['yaw_kp']*eyaw-g['yaw_kd']*self.filtered[2]+g['yaw_ki']*self.integral[2]
                if mode in ('turn','pre_turn','settle'):velocity_world[:]=0.
                # Position is the link origin; velocity command is COM. Preserve lever arm semantics.
                origin_ref_body=R.T@velocity_world
                body_wref=(wref*math.cos(pitch)-math.sin(roll)*s[12])/math.cos(roll)
                com_ref=origin_ref_body+np.cross(np.array([0.,0.,body_wref]),self.com_offset)
                target_body=np.r_[com_ref[:2],body_wref]
                if self.p['design']=='open_loop':target_body=np.array([speed,0.,0.])
                raw=target_body.copy();vel_terms={}
                if self.p['design']=='cascade_pi':
                    actual=np.r_[com_body[:2],s[13]]
                    beta=1. if self.inner_filtered is None else dt/(.10+dt)
                    self.inner_filtered=actual.copy()if self.inner_filtered is None else self.inner_filtered+beta*(actual-self.inner_filtered)
                    ev=target_body-self.inner_filtered
                    kp=np.array([g['velocity_kp_x'],g['velocity_kp_y'],g['rate_kp']])
                    ki=np.array([g['velocity_ki_x'],g['velocity_ki_y'],g['rate_ki']])
                    candidate=np.clip(self.velocity_integral+active_dt*ev,-.5,.5)
                    raw=target_body+kp*ev+ki*candidate
                    clipped=np.clip(raw,-self.limits,self.limits)
                    candidate=np.where(ev*(raw-clipped)>0,self.velocity_integral,candidate)
                    slewed=self.applied_command_estimate+np.clip(clipped-self.applied_command_estimate,-np.array([.6,.6,.8])*.02,np.array([.6,.6,.8])*.02)
                    candidate=np.where(ev*(raw-slewed)>1e-9,self.velocity_integral,candidate)
                    self.velocity_integral=candidate;raw=target_body+kp*ev+ki*candidate
                    vel_terms={'error':ev.tolist(),'P':(kp*ev).tolist(),'I':(ki*candidate).tolist(),'filtered_actual_body':self.inner_filtered.tolist()}
            if mode=='turn':raw[:2]=0.
            if mode in ('parking','goal_dwell','pre_turn','settle'):
                raw[:]=0.;target_body[:]=0.;velocity_world[:]=0.
            clipped=np.clip(raw,-effective_limits,effective_limits)
            if mode=='drive':clipped[0]=max(0.,clipped[0])
            norm=np.linalg.norm(clipped[:2])
            if norm>effective_limits[0]:clipped[:2]*=effective_limits[0]/norm
            self.command=clipped
            self.row={'kind':'controller_update','controller_updated':True,'control_t_s':float(s[0]),
                'feedback_time_s':stamp,'elapsed_s':t,'header_dt_s':dt,'feedback_hz':self.hz,
                'design':self.p['design'],'mode':mode,'segment':self.segment,
                'reference_xy':reference_xy.tolist(),'reference_yaw':heading,'reference_velocity_world':velocity_world.tolist(),
                'error_cross':cross,'error_along':float((reference_xy-s[1:3])@tan),'error_yaw':eyaw,
                'v_reference_body':target_body.tolist(),'command_body':clipped.tolist(),'raw_command_body':raw.tolist(),
                'velocity_PI':vel_terms,'position_terms':{'P':outer_P.tolist(),'I':outer_I.tolist(),'D':outer_D.tolist()},
                'measured_origin_velocity_world':origin_world.tolist(),'measured_COM_velocity_body':com_body.tolist(),
                'measured_yaw_rate':yawdot,'saturated':(abs(raw-clipped)>1e-9).tolist(),
                'completed_t_s':self.completed_t,'route_reference_changes':0,
                'goal_entry_radius_m':.12,'goal_hold_radius_m':.14,'active_integral_dt_s':active_dt,
                'feedback_source':'Gazebo native physics truth; intentionally controls this calibration only',
                'counts_as_SLAM_navigation':False}
        sim_age=stamp-self.last_feedback if self.last_feedback is not None else 0.
        wall_age=now-self.last_wall if self.last_wall is not None else 0.
        if sim_age>.3 or wall_age>.3:
            self.command[:]=0.;self.integral[:]=0.;self.velocity_integral[:]=0.
            self.row['mode']='feedback_timeout';self.row['command_body']=[0.,0.,0.]
        if not updated:
            self.row={**self.row,'kind':'controller_hold','controller_updated':False,
                'control_t_s':float(s[0]),'elapsed_s':t,'command_body':self.command.tolist()}
        self.row.update(feedback_age_sim_s=sim_age,feedback_age_wall_s=wall_age)
        self.applied_command_estimate+=np.clip(self.command-self.applied_command_estimate,-np.array([.6,.6,.8])*.02,np.array([.6,.6,.8])*.02)
        self.row['estimated_teacher_command_body']=self.applied_command_estimate.tolist()
        return self.command.copy(),self.row['mode']
