"""Bounded measured-pose response observer, no ROS/GT or command outputs.

Thresholds are excluded design candidates, not validated control settings.
Fitted angular rate approximates a short SO(3) interval in current body axes;
world heading rate is separately fitted to unwrapped measured heading.
"""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

NS=1_000_000_000

class MeasuredMotionObserver:
    def __init__(self,window_ns=400_000_000,min_span_ns=350_000_000,
                 max_pose_gap_ns=150_000_000,max_command_gap_ns=100_000_000,
                 max_pose_age_ns=250_000_000,pos_rms_limit_m=.02,angle_rms_limit_rad=.04):
        self.window_ns=int(window_ns);self.min_span_ns=int(min_span_ns)
        self.max_pose_gap_ns=int(max_pose_gap_ns);self.max_command_gap_ns=int(max_command_gap_ns)
        self.max_pose_age_ns=int(max_pose_age_ns)
        if min(self.window_ns,self.min_span_ns,self.max_pose_gap_ns,self.max_command_gap_ns,self.max_pose_age_ns)<=0 or self.min_span_ns>self.window_ns:
            raise ValueError('invalid observer windows')
        self.pos_rms_limit_m=float(pos_rms_limit_m);self.angle_rms_limit_rad=float(angle_rms_limit_rad)
        if not math.isfinite(self.pos_rms_limit_m) or not math.isfinite(self.angle_rms_limit_rad) or min(self.pos_rms_limit_m,self.angle_rms_limit_rad)<=0:raise ValueError('invalid fit gates')
        self.poses=deque(maxlen=64);self.commands=deque(maxlen=1024)
        self.rejected={'pose':0,'command':0};self.last_invalid=None

    @staticmethod
    def ns(x):
        if isinstance(x,(bool,np.bool_)) or not isinstance(x,(int,np.integer)) or x<0:raise ValueError('native nonnegative integer ns required')
        return int(x)

    @staticmethod
    def vector(v,n):
        x=np.array(v,dtype=float)
        if x.shape!=(n,) or not np.isfinite(x).all():raise ValueError('invalid finite vector')
        return x

    @staticmethod
    def mode(c):
        if abs(c[0])>.015:return 'walk' if c[0]>0 else 'reverse'
        if abs(c[1])>.015:return 'lateral'
        if abs(c[2])>.001:return 'turn'
        return 'zero'

    def observe_pose(self,stamp_ns,position,quaternion,received_wall_ns,
                     frame_id='camera_init',child_frame_id='demo_slam_body'):
        t=self.ns(stamp_ns);wall=self.ns(received_wall_ns)
        p=self.vector(position,3);q=self.vector(quaternion,4)
        if max(abs(p))>1000 or abs(np.linalg.norm(q)-1.)>.01 or frame_id!='camera_init' or child_frame_id!='demo_slam_body':raise ValueError('invalid pose frame/rotation')
        if self.poses and t<=self.poses[-1]['t']:
            self.rejected['pose']+=1
            if t<self.poses[-1]['t']:
                self.poses.clear();self.commands.clear();self.last_invalid='clock_reset'
            return False
        self.poses.append(dict(t=t,wall=wall,p=p,R=Rotation.from_quat(q).as_matrix()))
        # Retain one measured support point before the window boundary.
        # Fit a boundary interpolation instead of dropping 100ms of support
        # because an actual native stamp differs by 1ns. No extrapolation.
        while len(self.poses)>1 and self.poses[1]['t']<=t-self.window_ns:self.poses.popleft()
        self.last_invalid=None
        return True

    def observe_applied_command(self,stamp_ns,command,adapter_state='driving',context_id=None,execution_mode=None):
        t=self.ns(stamp_ns);c=self.vector(command,3)
        mode=self.mode(c) if execution_mode is None else execution_mode
        if not isinstance(adapter_state,str) or not isinstance(mode,str):raise ValueError('actual command mode/state required')
        row=dict(t=t,c=c,mode=mode,state=adapter_state,context=context_id)
        if self.commands and t<=self.commands[-1]['t']:
            self.rejected['command']+=1
            if t==self.commands[-1]['t'] and np.array_equal(c,self.commands[-1]['c']) and (mode,adapter_state,context_id)==(self.commands[-1]['mode'],self.commands[-1]['state'],self.commands[-1]['context']):return False
            # Never silently overwrite a same-stamp transition or reorder it.
            self.commands.clear();self.last_invalid='ambiguous_command_stamp';return False
        self.commands.append(row)
        return True

    def snapshot(self,now_ros_ns,now_wall_ns):
        now=self.ns(now_ros_ns);wall=self.ns(now_wall_ns)
        out=dict(available=False,valid=False,invalid_reason=self.last_invalid or 'waiting',
                 sample_count=len(self.poses),ground_truth_consumed=False,
                 control_output_generated=False,feedback_eligible=False,thresholds_status='unvalidated excluded design',
                 rejected_counts=self.rejected.copy())
        def invalid(reason):out['invalid_reason']=reason;return out
        if not self.poses:return out
        ps=list(self.poses);a,b=ps[0],ps[-1];end=b['t'];span=end-a['t']
        out.update(stamp_ns=end,window_start_ns=a['t'],dt_ns=span,age_ns=now-end,wall_age_ns=wall-b['wall'])
        if now<end or wall<b['wall']:return invalid('future_or_clock_reset')
        if now-end>self.max_pose_age_ns or wall-b['wall']>self.max_pose_age_ns:return invalid('stale_pose')
        if len(ps)<4 or span<self.min_span_ns:return invalid('insufficient_pose_span')
        if any(y['t']-x['t']>self.max_pose_gap_ns for x,y in zip(ps,ps[1:])):return invalid('pose_gap')
        support_start=a['t'];left=end-self.window_ns
        interpolated=a['t']<left
        if interpolated:
            next_pose=ps[1];alpha=(left-a['t'])/(next_pose['t']-a['t'])
            Rp=Slerp([0.,1.],Rotation.from_matrix(np.array([a['R'],next_pose['R']])))(alpha).as_matrix()
            boundary=dict(t=left,wall=a['wall'],p=(1-alpha)*a['p']+alpha*next_pose['p'],R=Rp)
            ps=[boundary]+ps[1:];a=boundary;span=end-left
        out.update(window_start_ns=a['t'],dt_ns=span,window_start_interpolated=interpolated,
                   measured_support_start_stamp_ns=support_start)
        cs=[c for c in self.commands if c['t']<=end]
        before=[i for i,c in enumerate(cs) if c['t']<=a['t']]
        if not before:return invalid('command_coverage_missing')
        cs=cs[before[-1]:]
        if a['t']-cs[0]['t']>self.max_command_gap_ns or end-cs[-1]['t']>self.max_command_gap_ns:return invalid('command_stale')
        if any(y['t']-x['t']>self.max_command_gap_ns for x,y in zip(cs,cs[1:])):return invalid('command_gap')
        identities={(c['mode'],c['state'],c['context']) for c in cs}
        if len(identities)!=1:return invalid('execution_transition')
        mode,state,context=next(iter(identities));out.update(execution_mode=mode,adapter_state=state,context_id=context,
                                                         observed_motion_ready=state=='driving' and mode in ('walk','turn'))
        effective=sum(x['t'] for x in ps)//len(ps)
        out.update(effective_stamp_ns=effective,effective_measurement_age_ns=now-effective)
        t=np.array([(x['t']-end)/NS for x in ps]);X=np.column_stack([t,np.ones(len(t))])
        P=np.array([x['p'] for x in ps]);fit=np.linalg.lstsq(X,P,rcond=None)[0]
        R=b['R'];Q=np.array([Rotation.from_matrix(x['R']@R.T).as_rotvec() for x in ps])
        if max(np.linalg.norm(Q,axis=1))>.6:return invalid('large_rotation_window')
        if any(np.hypot(x['R'][0,0],x['R'][1,0])<.2 for x in ps):return invalid('heading_singularity')
        rotfit=np.linalg.lstsq(X,Q,rcond=None)[0]
        yaw=np.unwrap([math.atan2(x['R'][1,0],x['R'][0,0]) for x in ps]);yf=np.linalg.lstsq(X,yaw,rcond=None)[0]
        posres=float(np.sqrt(np.mean(np.sum((P-X@fit)**2,axis=1))))
        rotres=float(np.sqrt(np.mean(np.sum((Q-X@rotfit)**2,axis=1))))
        yawres=float(np.sqrt(np.mean((yaw-X@yf)**2)))
        bodyv=R.T@fit[0];bodyw=R.T@rotfit[0]
        dt=(b['t']-ps[-2]['t'])/NS
        rawv=R.T@(b['p']-ps[-2]['p'])/dt
        raww=R.T@Rotation.from_matrix(R@ps[-2]['R'].T).as_rotvec()/dt
        integral=np.zeros(3)
        for i,c in enumerate(cs):
            left=max(a['t'],c['t']);right=min(end,cs[i+1]['t'] if i+1<len(cs) else end)
            if right>left:integral+=c['c']*(right-left)/span
        measured=np.array([bodyv[0],bodyv[1],bodyw[2]])
        out.update(available=True,raw_body_twist=dict(linear=rawv.tolist(),angular=raww.tolist()),
                   window_body_twist=dict(linear=bodyv.tolist(),angular=bodyw.tolist()),
                   heading_rate_world=float(yf[0]),actual_command_average=integral.tolist(),
                   response_error_body=(measured-integral).tolist(),
                   signed_body_response=(measured*integral).tolist(),
                   fit_residuals=dict(position_rms_m=posres,rotation_rms_rad=rotres,heading_rms_rad=yawres))
        if posres>self.pos_rms_limit_m or max(rotres,yawres)>self.angle_rms_limit_rad:return invalid('fit_residual_high')
        out.update(valid=True,invalid_reason=None)
        return out
