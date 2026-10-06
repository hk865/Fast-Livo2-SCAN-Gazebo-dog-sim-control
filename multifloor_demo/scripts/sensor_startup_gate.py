"""Static IMU startup policy; uses sensors and fixed world reference, never GT."""
from collections import deque
import math

def normalized(q):
    n=math.sqrt(sum(v*v for v in q))
    if not all(math.isfinite(v) for v in q) or n<1e-8:raise ValueError('invalid orientation')
    return tuple(v/n for v in q)

def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)

def rotate(q,v):
    return multiply(multiply(q,(*v,0.)),(-q[0],-q[1],-q[2],q[3]))[:3]

class StaticImuWindow:
    WARM_SECONDS=10.
    WINDOW_SECONDS=3.
    MIN_SAMPLES=300
    MAX_GAP=.030001
    MAX_GYRO=.03
    MAX_ORIENTATION_CHANGE=.01
    MAX_ACCEL_STD=.5
    MAX_MEAN_GRAVITY_ERROR=.25
    MAX_GRAVITY_ANGLE=.01
    GRAVITY=9.81

    def __init__(self,world_reference=(0.,0.,0.,1.)):
        self.reference=normalized(world_reference)
        self.samples=deque();self.last_stamp=None;self.last_wall=None
        self.bridge=None;self.bridge_wall=None;self.reason='waiting_for_imu'
        self.resets=0;self.metrics={}

    def reset(self,reason):
        if self.samples:self.resets+=1
        self.samples.clear();self.reason=reason

    def update_bridge(self,data,wall):
        self.bridge=data;self.bridge_wall=wall
        if not self.bridge_ready(wall):self.reset('bridge_not_ready_or_commanded')

    def bridge_ready(self,wall):
        if self.bridge_wall is None or wall-self.bridge_wall>.30 or not isinstance(self.bridge,dict):return False
        commands=[self.bridge.get(key) for key in ('requested','safe')]
        return (self.bridge.get('state')=='ready' and
                all(isinstance(v,(list,tuple)) and len(v)==3 for v in commands) and
                all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<1e-6 for values in commands for v in values))

    def update(self,stamp,wall,quaternion,gyro,acceleration,orientation_available=True):
        if not orientation_available or not all(math.isfinite(v) for v in (stamp,wall,*gyro,*acceleration)):
            self.reset('invalid_imu');return
        try:q=multiply(self.reference,normalized(quaternion))
        except ValueError:self.reset('invalid_orientation');return
        previous=self.last_stamp
        self.last_stamp=stamp;self.last_wall=wall
        if previous is not None and (stamp<=previous or stamp-previous>self.MAX_GAP):
            self.reset('imu_time_gap_or_nonincreasing_stamp');return
        if stamp<self.WARM_SECONDS:self.reset('warming_up');return
        if not self.bridge_ready(wall):self.reset('bridge_not_ready_or_commanded');return
        speed=math.sqrt(sum(v*v for v in gyro))
        if speed>self.MAX_GYRO:self.reset('angular_motion');return
        if self.samples:
            dot=min(1.,abs(sum(a*b for a,b in zip(q,self.samples[0]['q']))))
            if 2*math.acos(dot)>self.MAX_ORIENTATION_CHANGE:self.reset('orientation_motion')
        self.samples.append(dict(stamp=stamp,q=q,world_acc=rotate(q,acceleration),gyro=speed))
        while len(self.samples)>1 and self.samples[1]['stamp']<=stamp-self.WINDOW_SECONDS:self.samples.popleft()
        self.reason='collecting_static_window'

    def ready(self,clock,wall):
        if not self.bridge_ready(wall):self.reset('bridge_not_ready_or_commanded');return False
        if self.last_wall is None or wall-self.last_wall>.30 or self.last_stamp is None or not -.02<=clock-self.last_stamp<=.10:
            self.reset('imu_not_fresh');return False
        if len(self.samples)<self.MIN_SAMPLES or self.samples[-1]['stamp']-self.samples[0]['stamp']<self.WINDOW_SECONDS-1e-8:return False
        rows=list(self.samples);n=len(rows)
        mean=[sum(r['world_acc'][i] for r in rows)/n for i in range(3)]
        std=math.sqrt(sum(sum((r['world_acc'][i]-mean[i])**2 for i in range(3)) for r in rows)/n)
        norm=math.sqrt(sum(v*v for v in mean));gravity_error=abs(norm-self.GRAVITY)
        gravity_angle=math.acos(max(-1.,min(1.,mean[2]/norm))) if norm>1e-8 else math.pi
        orientation_range=max(2*math.acos(min(1.,abs(sum(a*b for a,b in zip(r['q'],rows[0]['q']))))) for r in rows)
        self.metrics=dict(samples=n,first_stamp=rows[0]['stamp'],last_stamp=rows[-1]['stamp'],
            duration=rows[-1]['stamp']-rows[0]['stamp'],mean_world_acceleration=mean,
            acceleration_std_norm=std,mean_gravity_norm_error=gravity_error,
            gravity_direction_error=gravity_angle,max_gyro=max(r['gyro'] for r in rows),
            orientation_range=orientation_range,resets=self.resets)
        if std>self.MAX_ACCEL_STD:self.reason='acceleration_variation';return False
        if gravity_error>self.MAX_MEAN_GRAVITY_ERROR:self.reason='acceleration_not_gravity';return False
        if gravity_angle>self.MAX_GRAVITY_ANGLE:self.reason='acceleration_orientation_disagree';return False
        if orientation_range>self.MAX_ORIENTATION_CHANGE:self.reason='orientation_motion';return False
        self.reason='stable_imu_and_ready_bridge';return True

    def report(self):
        return dict(reason=self.reason,metrics=self.metrics,buffered_samples=len(self.samples),
            thresholds={k:v for k,v in vars(type(self)).items() if k.isupper()},
            world_reference=list(self.reference),source='real IMU and bridge status; no ground truth')
