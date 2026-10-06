"""Excluded preregistered event supervisor. No ROS, GT, velocity outputs or PID.

Events require integration by an authorized controller; this is not deployed.
New source provenance cannot replace existing SCAN collision/progress validation.
"""
import math,os,pathlib,sys
import numpy as np
if os.environ.get('DEMO_TEST_ROOT'):
    sys.path.insert(0,str(pathlib.Path(os.environ['DEMO_TEST_ROOT'])/'navigation'))
else:
    sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]))
from trajectory_contract import TrajectoryAssociation

def angle(x):return math.atan2(math.sin(x),math.cos(x))
def ns(t):
    if type(t) is not int or t<0:raise ValueError('original integer native time required')
    return t

def position(p):
    q=np.asarray(p,dtype=float)
    if q.shape!=(3,) or not np.isfinite(q).all():raise ValueError('finite original SLAM position required')
    return q.copy()

class TurnDriftSupervisor:
    LIMITS=dict(planar_drift_m=.25,path_locked_offset_rad=.20,persistence_ns=400_000_000,
                max_pose_gap_ns=200_000_000,max_pose_age_ns=250_000_000,
                zero_before_replan_ns=1_000_000_000,max_adapter_age_ns=250_000_000)
    def __init__(self,require_path_offset=True,profile="original_025_04"):
        if profile not in ("original_025_04", "displacement_015_02"):raise ValueError("unknown preregistered profile")
        self.profile=profile
        self.LIMITS=dict(type(self).LIMITS)
        if profile=="displacement_015_02":
            if require_path_offset:raise ValueError("015 profile has no direction AND")
            self.LIMITS.update(planar_drift_m=.15,persistence_ns=200_000_000,min_raw_observations=3)
        else:self.LIMITS["min_raw_observations"]=3
        if type(require_path_offset) is not bool:raise ValueError('explicit candidate profile boolean required')
        self.require_path_offset=require_path_offset
        self.state='inactive';self.last_stamp=None;self.since=None;self.direction_sign=None
        self.events=[];self.zero_since=None;self.latest_zero=False;self.reference=None;self.last_execution_stamp=None;self.qualifying_samples=0
        self.association=TrajectoryAssociation()
    def begin(self,context,pose_stamp_ns,raw_position,locked_heading,body_goal):
        if self.state not in ('inactive','ready_for_existing_gate'):raise ValueError('must finish actual stop/new source handoff')
        if not isinstance(context,tuple) or len(context)!=4:raise ValueError('request,goal,traj,immutable reference required')
        if not math.isfinite(locked_heading):raise ValueError('finite heading required')
        self.context=context;self.anchor=position(raw_position);self.last_stamp=ns(pose_stamp_ns)
        self.heading=float(locked_heading);self.body_goal=position(body_goal);self.since=None;self.direction_sign=None;self.qualifying_samples=0
        self.state='observing';self.reference=context[3];self.zero_since=None;self.latest_zero=False
    def observe(self,stamp_ns,raw_position,current_path_heading,context,*,phase,actual_command,
                adapter_state,protected=False,pose_age_ns=0):
        t=ns(stamp_ns);p=position(raw_position)
        if not math.isfinite(current_path_heading):raise ValueError('finite current checked path heading required')
        c=np.asarray(actual_command,dtype=float)
        if c.shape!=(3,) or not np.isfinite(c).all():raise ValueError('actual [vx,vy,yaw] required')
        if self.state!='observing':return dict(state=self.state,event=None,reason='latched_or_inactive')
        if t<=self.last_stamp:return dict(state=self.state,event=None,reason='old_or_duplicate_pose')
        gap=t-self.last_stamp;self.last_stamp=t
        if context!=self.context:
            self.state='await_zero_idle';self.stop_stamp=t;self.since=None
            e=dict(event='stop_for_source_change',stamp_ns=t,context=self.context)
            self.events.append(e);return dict(state=self.state,event=e,reason='cannot_follow_unmatched_turn_source')
        drift=float(np.linalg.norm((p-self.anchor)[:2]));offset=angle(current_path_heading-self.heading)
        result=dict(state=self.state,event=None,raw_planar_drift_m=drift,path_locked_offset_rad=offset)
        pure_turn=c[0]==0. and c[1]==0. and abs(c[2])>1e-8 and adapter_state=='walk'
        if protected or phase!='align' or not pure_turn or type(pose_age_ns) is not int or not 0<=pose_age_ns<=self.LIMITS['max_pose_age_ns'] or gap>self.LIMITS['max_pose_gap_ns']:
            self.since=None;self.direction_sign=None;self.qualifying_samples=0;result['reason']='protection_mode_age_or_gap';return result
        sign=(1 if offset>0 else -1) if self.require_path_offset else 0
        if drift<self.LIMITS['planar_drift_m'] or (self.require_path_offset and abs(offset)<self.LIMITS['path_locked_offset_rad']):
            self.since=None;self.direction_sign=None;self.qualifying_samples=0;result['reason']='below_preregistered_geometry';return result
        if self.direction_sign!=sign:self.since=t;self.direction_sign=sign;self.qualifying_samples=0
        if self.since is None:self.since=t
        self.qualifying_samples+=1
        result['raw_observations']=self.qualifying_samples
        result['persistence_ns']=t-self.since
        if t-self.since<self.LIMITS['persistence_ns'] or self.qualifying_samples<self.LIMITS['min_raw_observations']:result['reason']='waiting_coherent_geometry';return result
        self.state='await_zero_idle';self.stop_stamp=t;self.zero_since=None;self.latest_zero=False
        e=dict(event='stop_for_turn_drift',stamp_ns=t,context=self.context,raw_position=p.tolist(),
               raw_anchor=self.anchor.tolist(),raw_planar_drift_m=drift,path_locked_offset_rad=offset,
               persistence_ns=t-self.since,raw_observations=self.qualifying_samples,profile=self.profile,limits=dict(self.LIMITS,require_path_offset=self.require_path_offset))
        self.events.append(e);return dict(result,state=self.state,event=e,reason='request_exact_zero_no_velocity_generated')
    def execution(self,stamp_ns,actual_command,*,adapter_state,nominal_calibrated,adapter_stamp_ns,
                  adapter_age_ns,protected=False):
        t=ns(stamp_ns);a=ns(adapter_stamp_ns)
        c=np.asarray(actual_command,dtype=float)
        if c.shape!=(3,) or not np.isfinite(c).all():raise ValueError('finite actual command required')
        if self.state!='await_zero_idle':return None
        if t<self.stop_stamp:return None
        if self.last_execution_stamp is not None:
            if t<=self.last_execution_stamp:return None
            if t-self.last_execution_stamp>self.LIMITS['max_pose_gap_ns']:self.zero_since=None
        self.last_execution_stamp=t
        # The actual adapter's fresh IDLE invariant is its zero-output proof.
        # Upstream safe==0 alone cannot start the actual-zero dwell.
        if (any(c) or protected or nominal_calibrated is not True or adapter_state!='idle'
                or a<self.stop_stamp or type(adapter_age_ns) is not int
                or not 0<=adapter_age_ns<=self.LIMITS['max_adapter_age_ns']):
            self.zero_since=None;self.latest_zero=False;return None
        if self.zero_since is None:self.zero_since=t
        self.latest_zero=True
        if t-self.zero_since<self.LIMITS['zero_before_replan_ns']:return None
        self.state='await_reference_declaration'
        e=dict(event='request_fresh_checked_SCAN',stamp_ns=t,actual_zero_since_ns=self.zero_since,
               actual_idle_stamp_ns=a,body_goal=self.body_goal.tolist(),context=self.context)
        self.events.append(e);return e
    def declare_reference(self,stamp,body_goal):
        if self.state!='await_reference_declaration':raise ValueError('no fresh reference before real zero/idle')
        if len(stamp)!=2 or any(type(x) is not int or x<0 for x in stamp) or stamp[1]>=1_000_000_000:raise ValueError('invalid immutable reference stamp')
        if tuple(stamp)<=tuple(self.context[3]):raise ValueError('reference must advance original request')
        if max(abs(position(body_goal)-self.body_goal))>1e-8:raise ValueError('original goal must remain unchanged')
        self.association.request(stamp,body_goal);self.state='await_paired_source';self.reference=tuple(stamp)
    def metadata(self,value):return self._pair(self.association.add_metadata(value)) if self.state=='await_paired_source' else None
    def spline(self,value):return self._pair(self.association.add_spline(value)) if self.state=='await_paired_source' else None
    def _pair(self,matched):
        if matched is None:return None
        msg,meta=matched
        if int(msg.traj_id)<=int(self.context[2]):self.association.reject('fresh local trajectory must advance single live planner');return None
        self.state='ready_for_existing_gate'
        e=dict(event='fresh_source_identity_valid',reference_stamp=list(self.reference),traj_id=int(msg.traj_id),
            body_goal=self.body_goal.tolist(),requires_original_collision_progress_pose_tilt_and_heading_gate=True)
        self.events.append(e);return e
