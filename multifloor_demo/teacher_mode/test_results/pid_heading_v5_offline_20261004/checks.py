#!/usr/bin/env python3
"""Actual controller methods and immutable route source, without ROS startup."""
import ast,copy,hashlib,importlib.util,json,math,sys,time
from pathlib import Path
from types import SimpleNamespace
sys.dont_write_bytecode=True
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
PID=ROOT/'navigation/pid_mode'
sys.path.insert(0,str(PID));sys.path.insert(0,str(ROOT.parent/'navigation'))
sys.path.insert(0,str(ROOT/'navigation'))
from route_leg_heading import RouteLegLedger,RouteLegError,apply_route_leg
from pid_core import HeaderPID,slew
from pid_scope import profile_for,runtime_files
from control_core import follow_trajectory
from teacher_transition import TeacherHeadingGate


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def rot(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def q(yaw):return [0,0,math.sin(yaw/2),math.cos(yaw/2)]


class Parent:
    def measured_region_arrival(self):
        self.parent_arrival_calls+=1
        return self.parent_arrival_result
    def control(self,now):
        return self.parent_control(now)


def actual_class(path):
    # Compile the exact on-disk class AST with a fake parent. No ROS imports,
    # nodes, transports, subscriptions or timer callbacks are started.
    tree=ast.parse(Path(path).read_text(),filename=str(path))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='PIDNavigation')
    scope={'TeacherNavigation':Parent,'HeaderPID':HeaderPID,'slew':slew,'math':math,'np':np,
        'time':SimpleNamespace(monotonic=lambda:100.),'RouteLegLedger':RouteLegLedger,
        'RouteLegError':RouteLegError,'apply_route_leg':apply_route_leg,'follow_trajectory':follow_trajectory}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),scope)
    return scope['PIDNavigation']


def equal(a,b):
    if isinstance(a,np.ndarray)or isinstance(b,np.ndarray):return np.array_equal(a,b)
    if isinstance(a,dict):return a.keys()==b.keys()and all(equal(a[k],b[k])for k in a)
    if isinstance(a,(tuple,list)):return len(a)==len(b)and all(equal(x,y)for x,y in zip(a,b))
    return a==b


def run():
    checks=[]
    def check(name,value,detail=None):
        row={'name':name,'passed':bool(value)}
        if detail is not None:row['detail']=detail
        checks.append(row)
        if not value:raise AssertionError(name)
    def rejects(name,fn):
        try:fn()
        except (RouteLegError,RuntimeError):check(name,True)
        else:check(name,False)
    baseline=json.loads((ROOT/'test_results/pid_navigation_v5_heading_baseline_20261004/manifest.json').read_text())
    old_path=Path(baseline['sources'][str(PID/'controller.py')]['snapshot'])
    old=actual_class(old_path);new=actual_class(PID/'controller.py')
    profile=profile_for('flat_short');variant=profile_for('flat_short','teacher','route_leg')
    added={'heading_reference','heading_reference_contract'}
    changed={'experiment','pid_runtime_revision'}
    for name in ('flat_short','flat_long','flat_roundtrip'):
        original=profile_for(name);v5=profile_for(name,'teacher','route_leg')
        check(name+' gains/limits/route/criteria preserved',
            {k:v for k,v in v5.items()if k not in added|changed}=={k:v for k,v in original.items()if k not in changed})
        check(name+' default profile exact old bytes decoded',original==json.loads((PID/'profiles'/f'{name}.json').read_text()))
    check('flat_straight alias exact route-leg flat_long',profile_for('flat_straight','teacher','route_leg')==profile_for('flat_long','teacher','route_leg'))
    rejects('CHAMP cannot select route-leg',lambda:profile_for('flat_short','champ','route_leg'))
    rejects('ramp cannot select route-leg',lambda:profile_for('ramp_up','teacher','route_leg'))
    rejects('unknown yaw source cannot select',lambda:profile_for('flat_short','teacher','world_x'))

    ledger=RouteLegLedger();start=np.array([2.,3.,.3]);goal=np.array([8.,3.,.3])
    ref,created=ledger.freeze('route',0,start,goal,1_000_000_000,1_100_000_000,q(0.))
    check('immutable source header and original pose frozen',created and ref.heading==0. and ref.activation_pose_stamp_ns==1_000_000_000)
    for angle in (-2.3,-.7,.4,2.8):
        rotation=rot(angle);translation=np.array([11.,-4.,2.])
        r,_=RouteLegLedger().freeze('route',0,rotation@start+translation,rotation@goal+translation,
            1_000_000_000,1_100_000_000,q(angle))
        check('rotation/translation yaw covariance '+str(angle),abs(wrap(r.heading-ref.heading-angle))<1e-14)
    check('caller array mutation cannot alter reference',
        isinstance(ref.start,tuple)and isinstance(ref.goal,tuple))
    start[0]=999.;goal[1]=999.
    check('mutable source arrays copied',ref.start==(2.,3.,.3)and ref.goal==(8.,3.,.3))
    r,created=ledger.freeze('route',0,ref.start,ref.goal,2_000_000_000,2_100_000_000,q(.8))
    check('duplicate/replan cannot refresh reference stamp/yaw',r is ref and not created and r.activation_pose_stamp_ns==1_000_000_000)
    rejects('same identity changed start rejected',lambda:ledger.get('route',0,[2.01,3,.3],ref.goal))
    rejects('same identity changed goal rejected',lambda:ledger.get('route',0,ref.start,[8.01,3,.3]))
    r,_=ledger.freeze('route',1,[8,3,.3],[2,3,.3],2_000_000_000,2_100_000_000,q(0))
    check('return leg pi direction',abs(abs(r.heading)-math.pi)<1e-14)
    check('new waypoint has distinct original activation header',r.activation_pose_stamp_ns==2_000_000_000 and len(ledger.references)==2)
    rejects('missing activation refuses',lambda:ledger.get('missing',0,[0,0,0],[1,0,0]))
    cases=[('zeroXY', [0,0,0],[0,0,1],1_000_000_000,1_100_000_000,q(0)),
        ('nanstart',[float('nan'),0,0],[1,0,0],1_000_000_000,1_100_000_000,q(0)),
        ('infinitegoal',[0,0,0],[float('inf'),0,0],1_000_000_000,1_100_000_000,q(0)),
        ('malformedstart',None,[1,0,0],1_000_000_000,1_100_000_000,q(0)),
        ('nonnumericstart',['oops',0,0],[1,0,0],1_000_000_000,1_100_000_000,q(0)),
        ('badquaternion',[0,0,0],[1,0,0],1_000_000_000,1_100_000_000,[0,0,0,0]),
        ('exact300ms',[0,0,0],[1,0,0],1_000_000_000,1_300_000_000,q(0)),
        ('futureover50ms',[0,0,0],[1,0,0],1_000_000_000,949_999_999,q(0)),
        ('missingstamp',[0,0,0],[1,0,0],-1,1_100_000_000,q(0))]
    for label,a,b,stamp,clock,quat in cases:
        rejects('reject invalid '+label,lambda a=a,b=b,stamp=stamp,clock=clock,quat=quat:
            RouteLegLedger().freeze('bad',0,a,b,stamp,clock,quat))

    # Exact V4 follower parity, with curved and rotated finite SCAN samples.
    rng=np.random.default_rng(42);follow_parity=0
    for i in range(1000):
        yaw=rng.uniform(-math.pi,math.pi);pose=np.r_[rng.uniform(-.3,.3,2),0.]
        xs=np.linspace(-.5,2,30);samples=np.c_[xs,.15*np.sin(xs*(i%5+1)),np.zeros(30)]
        f=SimpleNamespace(profile=profile,pose=pose)
        args=(pose,rot(yaw),samples,np.array([2.,0,0]))
        kwargs={'tracking_pose':pose+1,'gate_translation':False,'return_steering':True}
        a=old.follow_checked_trajectory(f,*args,**kwargs);b=new.follow_checked_trajectory(f,*args,**kwargs)
        if equal(a,b):follow_parity+=1
    check('V4 follower exact parity 1000 cases',follow_parity==1000)

    # Exercise actual new follower with the immutable raw-SLAM direction.
    f=new.__new__(new);f.profile=variant;f.route_leg_ledger=RouteLegLedger()
    f.request_id='curve';f.waypoint_index=0;f.segment_start=np.zeros(3);f.waypoints=np.array([[6.,0,0]])
    f.pose=np.array([.5,.1,0]);f.pose_stamp=1_000_000_000;f.pose_quaternion=q(.2);f.rotation=rot(.2)
    f.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_100_000_000))
    records=[];f.run=Path('/offline/never_created');f.evidence=SimpleNamespace(append=lambda path,row:records.append((path.name,copy.deepcopy(row))))
    f.parent_arrival_calls=0;f.parent_arrival_result=(False,False)
    result=f.measured_region_arrival()
    check('activation hook preserves measured dwell result',result==(False,False)and f.parent_arrival_calls==1)
    check('reference frozen before follower with original header',len(records)==1 and records[0][1]['activation_pose_stamp_ns']==1_000_000_000)
    headings=[]
    for bend in (.1,.6):
        x=np.linspace(0,6,40);samples=np.c_[x,bend*x*x/6,np.zeros(40)]
        previous=follow_trajectory(f.pose,f.rotation,samples,f.waypoints[0],tracking_pose=f.pose,gate_translation=False,return_steering=True)
        current=f.follow_checked_trajectory(f.pose,f.rotation,samples,f.waypoints[0],tracking_pose=f.pose+100,gate_translation=False,return_steering=True)
        headings.append(current[3]['scan_heading_rad'])
        check('curve '+str(bend)+' keeps exact checked SCAN target/direction/exhaustion',np.array_equal(current[2],previous[2])and
            all(current[3][k]==previous[3][k]for k in ('target','projection','direction','nearest_index','target_index','exhausted')))
        check('curve '+str(bend)+' yaw uses leg while original SCAN logged',current[3]['heading']==0. and
            current[3]['scan_heading_rad']==previous[3]['heading']and abs(current[3]['error']+.2)<1e-14)
    check('SCAN curvature changed yet frozen yaw stays fixed',headings[0]!=headings[1])
    f.pose_stamp=1_200_000_000;f.measured_region_arrival()
    check('mode/replan observed later pose does not reissue activation receipt',len(records)==1 and f.route_leg_ledger.get('curve',0,f.segment_start,f.waypoints[0]).activation_pose_stamp_ns==1_000_000_000)
    rejects('nonfinite original SCAN heading denied',lambda:apply_route_leg(ref,{'heading':float('nan'),'error':0},0))

    # Default PID output/diagnostic/log parity on identical measured inputs.
    def pid_fake(profile,i):
        yaw=.4*math.sin(i);ns=10_000_000_000;phase=('drive','pre_turn','settle')[i%3]
        return SimpleNamespace(profile=profile,pid=HeaderPID(profile),pid_velocity=np.array([.03,.01,0]),
            gyro_body=[0.,0.,.02],gyro_wall=100.,gyro_stamp=10.,pid_pose_stamp=ns,pose_stamp=ns,
            rotation=rot(yaw),pose=np.array([.2,.01,0]),pose_quaternion=q(yaw),
            steering={'heading':.25,'direction':[1.,0.],'exhausted':False},heading_gate=SimpleNamespace(phase=phase,heading=.25),
            obstacle_hold=False,request_id='test',waypoint_index=0,waypoints=np.array([[1,.2,0]]),command=[.05,.02,.01],last_command_time=9.95,
            pid_records=0,pid_row=None,active_trajectory_id=7,trajectory_archive_reference='checked.npz',
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=ns)),state='running',alignment_hold=False)
    pid_parity=0
    for i in range(1000):
        a=pid_fake(profile,i);b=copy.deepcopy(a)
        x=old.select_pid_velocity(a,np.array([.1,0]),.01,np.array([1,.2,0]),i%2==0)
        y=new.select_pid_velocity(b,np.array([.1,0]),.01,np.array([1,.2,0]),i%2==0)
        if equal(x,y)and equal(a.pid_row,b.pid_row)and equal(a.steering,b.steering):pid_parity+=1
    check('V4 actual PID output/receipt exact parity 1000 cases',pid_parity==1000)
    f=pid_fake(variant,0);f.steering.update(heading=ref.heading,heading_reference=ref.evidence(),scan_heading_rad=.7,scan_heading_error_rad=.5)
    new.select_pid_velocity(f,np.array([.1,0]),.01,np.array([1,.2,0]),True)
    check('new PID receipt retains original SCAN and immutable source',f.pid_row['heading_reference']==ref.evidence()and f.pid_row['scan_heading_rad']==.7)
    check('new yaw PID uses leg not original SCAN heading',f.pid_row['heading']==ref.heading and abs(f.pid_row['pid']['error_world_xy_yaw'][2]-wrap(ref.heading-f.pid_row['yaw']))<1e-14)
    expected=f.rotation[:2,:2]@f.pid_prepared_output[:2]
    check('new final slewed direction still guarded',np.array_equal(expected,np.asarray(f.steering['direction'])))
    f.record_native_guard=lambda *args:setattr(f,'pending_guard_row',{})or(False,None,0)
    new.evaluate_motion_guard(f,[],[],[],[],[])
    check('new exact guard-to-PID/source association',f.pending_guard_row['pid_sequence']==f.pid_row['sequence']and f.pending_guard_row['pid_control_pose_stamp_ns']==f.pose_stamp)
    gate=TeacherHeadingGate(variant)
    for t in (1.,1.15,1.3):gate.observe(t,100.,[0,0,0],0,[0,0,0],t,True)
    allowed,rate=gate.update(1.3,.5,ref.heading,100.)
    check('actual measured HeadingGate locks same route-leg reference',not allowed and gate.phase=='align'and gate.heading==ref.heading and rate<0)
    f=pid_fake(variant,0);f.rotation=rot(.5);f.heading_gate=gate
    f.steering.update(heading=ref.heading,heading_reference=ref.evidence(),scan_heading_rad=.7,scan_heading_error_rad=.2)
    new.select_pid_velocity(f,np.zeros(2),rate,np.array([1,.2,0]),False)
    check('align PID uses exact gate leg reference with zero XY',f.pid_row['heading']==gate.heading and f.pid_row['mode']=='turn'and f.pid_row['desired_body_command'][:2]==[0.,0.]and f.pid_row['desired_body_command'][2]<0)
    p=HeaderPID(variant);zero=np.zeros(3);key=('route',0,'drive')
    a,d=p.update(10_000_000_000,zero,0,zero,0,[1,.1,0],ref.heading,'drive',key,10_000_000_000)
    a,d=p.update(10_100_000_000,zero,0,zero,0,[1,.1,0],ref.heading,'drive',key,10_100_000_000);stored=p.integral.copy()
    b,d=p.update(10_100_000_000,zero,0,zero,0,[1,.1,0],ref.heading,'drive',key,10_100_000_000)
    check('V5 duplicate PID header unchanged',np.array_equal(a,b)and np.array_equal(p.integral,stored)and not d['updated'])
    watermark=p.last_stamp;p.reset('hold')
    b,d=p.update(10_100_000_000,zero,0,zero,0,[1,.1,0],ref.heading,'drive',key,10_100_000_000)
    check('V5 reset keeps watermark/duplicate cannot resume',p.last_stamp==watermark and np.array_equal(b,zero))

    # Invalid actual activation through the actual new control exception branch.
    bad=new.__new__(new);bad.profile=variant;bad.state='running';bad.request_id='bad';bad.waypoint_index=0
    bad.pose_stamp=1_000_000_000;bad.pid=HeaderPID(variant);bad.run=Path('/offline/never_created')
    bad.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_100_000_000))
    failed=[];stops=[];bad.evidence=SimpleNamespace(append=lambda p,r:failed.append(copy.deepcopy(r)))
    bad.reset_region_arrival=lambda reason:stops.append(reason)
    bad.publish_command=lambda:stops.append('exact_zero')
    bad.parent_control=lambda now:RouteLegLedger().freeze('bad',0,zero,zero,1_000_000_000,1_100_000_000,q(0))
    bad.control(100.)
    check('invalid activation locks failed and exact zero',bad.state=='failed'and stops==['failed','exact_zero']and len(failed)==1)
    check('invalid receipt declares no truth/no heading fallback',failed[0]['navigation_ground_truth_used']is False and 'exact zero'in failed[0]['action'])
    old_default=new.__new__(new);old_default.profile=profile;old_default.parent_control=lambda now:('unchanged',now)
    check('V4 control dispatch exact old parent',old_default.control(17.)==('unchanged',17.))

    # New runtime source additions are frozen by scope; all original unrelated
    # V4 sources remain byte-exact except root-owned runner and these 2 files.
    refs=runtime_files();check('scope freezes helper and all three profiles',PID/'route_leg_heading.py'in refs and
        all(PID/'profiles_heading_v5'/f'{n}.json'in refs for n in ('flat_short','flat_long','flat_roundtrip')))
    unchanged=[];mismatches=[]
    allowed={str(PID/'controller.py'),str(PID/'pid_scope.py'),str(ROOT/'scripts/run_test.py')}
    for path,row in baseline['sources'].items():
        if path in allowed:continue
        if sha(path)!=row['sha256']:mismatches.append(path)
        else:unchanged.append(path)
    check('unrelated V4 frozen actor/physics/profiles/source bytes preserved',not mismatches,{'unchanged_count':len(unchanged),'mismatches':mismatches})
    sources=[PID/'controller.py',PID/'pid_scope.py',PID/'route_leg_heading.py']+sorted((PID/'profiles_heading_v5').glob('*.json'))
    report={'schema':'teacher_pid_route_leg_heading_v5_offline/v1','status':'passed','check_count':len(checks),
        'checks':checks,'source_hashes':{str(p):sha(p)for p in sources},'script_sha256':sha(__file__),
        'baseline_manifest_sha256':sha(ROOT/'test_results/pid_navigation_v5_heading_baseline_20261004/manifest.json'),
        'controller_execution':'exact on-disk class AST with fake parent, actual HeaderPID and original shared follow_trajectory',
        'actual_ros_nodes_started':False,'actual_physics_started':False,'physical_navigation_verified':False,
        'V4_follow_exact_cases':follow_parity,'V4_PID_exact_cases':pid_parity,'unrelated_source_count_unchanged':len(unchanged)}
    output=HERE/'offline_checks.json'
    with output.open('x')as stream:json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({'status':'passed','checks':len(checks),'report':str(output),'sha256':sha(output)}))


if __name__=='__main__':run()
