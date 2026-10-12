#!/usr/bin/env python3
"""ROS-free production-method replay: continuous proof, provenance, transactions.

Reads three small frozen R1 accepted trajectory files. It never launches ROS,
reads ground truth, or changes original experiment/code files.
"""
import argparse,ast,copy,hashlib,json,math,sys,tempfile,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.interpolate import BSpline
from cascade_core import Controller
from path_admission import (FrozenFence,candidate,payload,replayable,certify_curve,
    remaining_certificate,split_bezier,MAX_CONTROL_POINTS)

HERE=Path(__file__).resolve().parent
TEACHER=HERE.parent.parent
DEMO=TEACHER.parent
sys.path.insert(0,str(DEMO/'navigation'))
from control_core import trajectory_has_progress
R1=TEACHER/'runs/20261006_202609_closed_loop_cascade_clock_hold_V26_height_diag_actual_r1_d530'
FIXTURE=TEACHER/'test_results/path_admission_v27_20261006/review/NEAREST_SWITCH_COUNTEREXAMPLE.json'
CONFIG={'horizontal_radius_m':.45,'height_error_m':.3}
REPORT={'schema':'path_admission_V28_capture_production_replay/v1','tests':[],
    'simulation_launched':False,'navigation_ground_truth_used':False,'fixture_run':str(R1),
    'limitations':'ROS-free production-method replay validates admission transactions and proofs; not physical navigation success.'}

def check(name,condition,**evidence):
    REPORT['tests'].append(dict(name=name,passed=bool(condition),**replayable(evidence)))
    if not condition:raise AssertionError(name+': '+str(evidence))

def message(value):
    return NS(order=value['order'],traj_id=value.get('traj_id',1),
        start_time=NS(sec=value.get('start_time',[0,0])[0],nanosec=value.get('start_time',[0,0])[1]),
        pos_pts=[NS(x=p[0],y=p[1],z=p[2])for p in value['pos_pts']],knots=value['knots'])

def cubic(control):return dict(order=3,pos_pts=np.asarray(control).tolist(),knots=[0]*4+[1]*4)

def proof_tests():
    anchor=json.loads((R1/'navigation_anchor.json').read_text());route=anchor['registered_route_camera_init_xyz']
    actual=[]
    for identity,expected in ((79,True),(80,True),(81,False)):
        file=R1/'navigation_trajectories'/f'{identity:06d}_trajectory_{identity}.json'
        document=json.loads(file.read_text());value=document['metadata']['trajectory']
        timings=[];result=None
        for _ in range(100):
            result=certify_curve(value,route,CONFIG);timings.append(result['elapsed_wall_ms'])
            if result['allowed']!=expected:raise AssertionError('actual'+str(identity)+str(result))
        check('actual_'+str(identity)+'_continuous_accept' if expected else 'actual_81_continuous_reject',
            result['allowed']==expected and (expected or result['reason']=='outside_fence'),
            trajectory_file=str(file),trajectory_file_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
            certificate=result,benchmark_ms=dict(min=min(timings),median=float(np.median(timings)),p95=float(np.percentile(timings,95)),max=max(timings)))
        actual.append(document)
    straight=[[0,0,0],[1,0,0]]
    # Runtime .08s sampling misses the entire 2ms bump. Continuous interval
    # proof must reject horizontal and vertical peaks inside that tiny span.
    for axis,amplitude in ((1,1.),(2,.8)):
        knots=[0]*4+[.039]*3+[.041]*3+[1]*4
        control=np.zeros((10,3));control[:,0]=np.linspace(0,1,10)
        control[4:6,axis]=amplitude
        value=dict(order=3,pos_pts=control.tolist(),knots=knots)
        samples=BSpline(knots,control,3)(np.linspace(0,1,int(1/.08)+1))
        proof=certify_curve(value,straight,CONFIG)
        check('narrow_between_runtime_samples_axis'+str(axis),not proof['allowed'] and proof['reason']=='outside_fence'
            and float(np.max(np.abs(samples[:,axis])))<1e-12,
            certificate=proof,sample_count=len(samples),sampled_max=float(np.max(np.abs(samples[:,axis]))))
    fixture=json.loads(FIXTURE.read_text());route=fixture['registered_route_points'];c=np.asarray(fixture['control_ball_center'])
    point=np.asarray(fixture['interior_point'])
    # Constant curve at external counterexample's unsafe interior point.
    # A broad patch around center has both nearest segments as possibilities.
    proof=certify_curve(cubic([point]*4),route,CONFIG)
    check('external_nearest_segment_switch_rejected',not proof['allowed'],fixture_file=str(FIXTURE),
        fixture_sha256=hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),certificate=proof)
    # Curve has safe midpoint c but unsafe endpoint point; one-center-nearest
    # proof incorrectly accepts its hull. All possible segment proof rejects.
    control=np.asarray([point,-point/3,-point/3,point])
    proof=certify_curve(cubic(control),route,CONFIG)
    check('all_possible_nearest_not_only_center',not proof['allowed'],certificate=proof)
    valid=cubic([[0,0,0],[1/3,0,0],[2/3,0,0],[1,0,0]])
    for name,changes in [('nan_coeff',{'pos_pts':[[math.nan,0,0]]*4}),
            ('inf_knots',{'knots':[0]*4+[math.inf]*4}),
            ('finite_extreme_coeff',{'pos_pts':[[5e307]*3]*4}),
            ('finite_extreme_knots',{'knots':[-1e308]*4+[1e308]*4}),
            ('noncubic',{'order':2}),('zero_duration',{'knots':[0]*8})]:
        value=dict(valid,**changes);proof=certify_curve(value,straight,CONFIG)
        check(name+'_failclosed',not proof['allowed'],certificate=proof)
    proof=certify_curve(valid,[[0]*3,[1e308]*3],CONFIG)
    check('finite_extreme_route_failclosed',not proof['allowed'],certificate=proof)
    proof=certify_curve(valid,straight,CONFIG,wall_budget_s=1e-12)
    check('wall_extraction_budget_failclosed',not proof['allowed'] and 'budget' in proof['reason'],certificate=proof)
    proof=certify_curve(valid,straight,CONFIG,max_patches=1)
    check('patch_budget_failclosed',not proof['allowed'] and proof['reason']=='patch_budget_exhausted',certificate=proof)
    # 512 controls with 509 spans: bounded failclosed, no unlimited dense scan.
    n=MAX_CONTROL_POINTS;control=np.zeros((n,3));control[:,0]=np.linspace(0,1,n);control[:,1]=.4499999
    knots=[0]*4+np.linspace(0,1,n-2)[1:-1].tolist()+[1]*4
    timings=[]
    for _ in range(10):
        proof=certify_curve(dict(order=3,pos_pts=control.tolist(),knots=knots),straight,CONFIG)
        timings.append(proof['elapsed_wall_ms'])
    check('worst_long_near_boundary_bounded',not proof['allowed'] and 'budget' in proof['reason']
        and max(timings)<100,certificate=proof,max_ms=max(timings),note='20ms cooperative wall bound; input extraction is included; scheduling can add overhead')
    return actual,anchor

class FakeEvidence:
    def __init__(self):self.rows=[];self.error=None
    def enqueue(self,work):work()
    def append(self,path,row):self.rows.append((str(path),copy.deepcopy(row)))

def actual_acceptance_layers():
    # Execute the actual shared acceptance and Teacher accepted archive methods,
    # retaining their lexical super() classes, without importing ROS.
    shared_tree=ast.parse((HERE/'shared_controller.py').read_text())
    wrapper_tree=ast.parse((HERE/'teacher_wrapper.py').read_text())
    shared=copy.deepcopy(next(n for n in shared_tree.body if isinstance(n,ast.ClassDef)and n.name=='Navigation'))
    shared.name='SharedAcceptance';shared.bases=[]
    shared.body=[n for n in shared.body if isinstance(n,ast.FunctionDef)and n.name in ('accept_spline','mark_execution_path_exhausted')]
    teacher=copy.deepcopy(next(n for n in wrapper_tree.body if isinstance(n,ast.ClassDef)and n.name=='TeacherNavigation'))
    teacher.name='TeacherAcceptance';teacher.bases=[ast.Name(id='SharedAcceptance',ctx=ast.Load())]
    teacher.body=[n for n in teacher.body if isinstance(n,ast.FunctionDef)and n.name in ('accept_spline','mark_execution_path_exhausted')]
    namespace=dict(np=np,BSpline=BSpline,trajectory_has_progress=trajectory_has_progress,hashlib=hashlib,json=json)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[shared,teacher],type_ignores=[])),str(HERE/'shared_controller.py'),'exec'),namespace)
    return namespace['TeacherAcceptance']

class FakeBase(actual_acceptance_layers()):
    def accept_spline(self,msg,metadata):
        self.super_calls+=1;before=self.replans
        super().accept_spline(msg,metadata)
        if self.replans>before:self.accepted_archives.append(msg.traj_id)

METHODS={'admission_identity','retain_admitted_path','archive_admission','accept_spline','control_admission_hold','control_parking','request_admission_replan_if_due','record_segment_activation','path_execution_eligibility','capture_replacement_eligibility','clear_committed_exhausted_capture','ensure_cascade'}
def production_class():
    tree=ast.parse((HERE/'controller.py').read_text())
    cls=next(node for node in tree.body if isinstance(node,ast.ClassDef)and node.name=='PIDNavigation')
    cls=copy.deepcopy(cls);cls.body=[node for node in cls.body if isinstance(node,ast.FunctionDef)and node.name in METHODS]
    module=ast.Module(body=[cls],type_ignores=[])
    namespace=dict(TeacherNavigation=FakeBase,copy=copy,hashlib=hashlib,json=json,math=math,time=time,Path=Path,np=np,
        BSpline=BSpline,Controller=Controller,FrozenFence=FrozenFence,candidate=candidate,payload=payload,replayable=replayable,
        certify_curve=certify_curve,remaining_certificate=remaining_certificate,trajectory_has_progress=trajectory_has_progress)
    exec(compile(ast.fix_missing_locations(module),str(HERE/'controller.py'),'exec'),namespace)
    return namespace['PIDNavigation']

def setup_controller(cls,run,profile,document):
    c=cls();c.run=run;c.profile=profile;c.path_admission_fence=FrozenFence(run,profile)
    c.path_admission_active=None;c.path_admission_hold=False;c.path_admission_replan_pending=False;c.path_admission_sequence=0
    c.path_admission_last_decision=None;c.request_id=document['request_id'];c.waypoint_index=9
    c.waypoints=np.tile(np.asarray(document['metadata']['body_goal']),(10,1));c.arrival_radius=.22
    c.trajectory_association=NS(reference_stamp=tuple(document['reference_stamp']),accepted_highwater_id=79)
    c.pose=np.asarray(document['metadata']['trajectory']['pos_pts'][0]);c.pose_stamp=12345
    c.replans=0;c.samples=None;c.active_trajectory_id=None;c.active_reference_stamp=None
    c.cascade=None;c.cascade_key=None;c.cascade_last_row={'numeric_PI_sentinel':[1,2,3]}
    c.path_receipt=None;c.cascade_path_id='untouched';c.reference_replan_pending=False
    c.super_calls=0;c.accepted_archives=[];c.evidence=FakeEvidence();c.state='running'
    c.pose_updated=c.cloud_updated=c.bridge_updated=time.monotonic();c.bridge_safety={'state':'ready'}
    c.raw_imu=NS(fresh=lambda now,ros:True);c.obstacle_hold=c.tilt_hold=False
    c.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000))
    c.path_pub=NS(publish=lambda path:None);c.make_path=lambda samples:samples
    c.degenerate_splines=0;c.last_spline_rejected=None;c.planning_start_state=None
    c.zero_count=0
    def zero():c.zero_count+=1
    c.publish_command=zero;c.pid_guard_needs_evaluation=False
    return c

def snapshot(c):
    return dict(active=c.active_trajectory_id,reference=c.active_reference_stamp,replans=c.replans,
        samples=None if c.samples is None else c.samples.copy(),cascade=copy.deepcopy(c.cascade_last_row),
        receipt=copy.deepcopy(c.path_receipt),archives=copy.deepcopy(c.accepted_archives),super_calls=c.super_calls)

def unchanged(a,b):
    aa=dict(a);bb=dict(b);sa=aa.pop('samples');sb=bb.pop('samples')
    return aa==bb and (sa is None and sb is None or sa is not None and sb is not None and np.array_equal(sa,sb))

def transaction_tests(actual,anchor):
    cls=production_class();profile=json.loads((HERE/'profiles/repair_prefix270.json').read_text())
    with tempfile.TemporaryDirectory(prefix='V27_admission_replay_')as temporary:
        run=Path(temporary);anchor=copy.deepcopy(anchor);anchor['run_dir']=str(run)
        (run/'navigation_anchor.json').write_text(json.dumps(anchor)+'\n')
        (run/'navigation_scene_axis_registration.json').write_bytes((R1/'navigation_scene_axis_registration.json').read_bytes())
        c=setup_controller(cls,run,profile,actual[0]);callback_timings=[]
        for document in actual[:2]:
            c.trajectory_association.reference_stamp=tuple(document['reference_stamp'])
            c.pose=np.asarray(document['metadata']['trajectory']['pos_pts'][0]);c.pose_updated=c.cloud_updated=c.bridge_updated=time.monotonic()
            started=time.monotonic();c.accept_spline(message(document['metadata']['trajectory']),document['metadata']);callback_timings.append((time.monotonic()-started)*1000)
        check('production_actual79_80_admitted',c.active_trajectory_id==80 and c.replans==2 and c.accepted_archives==[79,80],
            decisions=[row for path,row in c.evidence.rows if path.endswith('path_admission_decisions.jsonl')])
        before=snapshot(c);c.pose=c.samples[0].copy();c.pose_updated=c.cloud_updated=c.bridge_updated=time.monotonic()
        c.trajectory_association.reference_stamp=tuple(actual[2]['reference_stamp']);c.trajectory_association.accepted_highwater_id=81
        started=time.monotonic();c.accept_spline(message(actual[2]['metadata']['trajectory']),actual[2]['metadata']);callback_timings.append((time.monotonic()-started)*1000)
        check('production81_transaction_preserves_path_PI_archives',unchanged(before,snapshot(c))
            and c.trajectory_association.accepted_highwater_id==81 and not c.path_admission_hold,
            decision=c.path_admission_last_decision,callback_ms=callback_timings)
        c.trajectory_association.reference_stamp=(actual[1]['reference_stamp'][0]+1,actual[1]['reference_stamp'][1])
        check('active_reference_retained_pending_reference_separate',tuple(c.active_reference_stamp)==tuple(actual[1]['reference_stamp'])
            and c.active_reference_stamp!=c.trajectory_association.reference_stamp)
        c.trajectory_association.reference_stamp=tuple(actual[2]['reference_stamp'])
        # A normal drive diagnostic clears the legacy flag. The independent
        # admission request must survive until a real reference is published.
        c.reference_replan_pending=False;c.last_reference=10.;c.reference_requests=0
        def request_plan():c.reference_requests+=1;c.last_reference=13.001
        c.request_plan=request_plan;c.request_admission_replan_if_due(12.999)
        check('retained_rejection_replan_waits_3s',c.reference_requests==0 and c.path_admission_replan_pending)
        c.request_admission_replan_if_due(13.001)
        check('retained_rejection_normal_drive_still_replans',c.reference_requests==1 and not c.path_admission_replan_pending)
        rejected=json.loads(sorted((run/'path_admission_candidates').glob('*.json'))[-1].read_text())
        check('rejected81_payload_replayable_saved',rejected['payload']==actual[2]['metadata']['trajectory']
            and not rejected['decision']['admitted'] and rejected['decision']['association_payload_id']==81
            and rejected['decision']['active_trajectory_id']==80)
        # Metadata cannot be relabeled onto an old payload by a new reference.
        before=snapshot(c);bad=copy.deepcopy(actual[2]['metadata']);bad['reference_stamp']=[1,2]
        c.accept_spline(message(actual[2]['metadata']['trajectory']),bad)
        check('metadata_mismatch_no_execution_mutation',unchanged(before,snapshot(c)) and c.path_admission_hold and c.zero_count>0,
            decision=c.path_admission_last_decision)
        # Source loss forbids old route retention even when curve remains safe.
        before=snapshot(c);c.cloud_updated=time.monotonic()-.301
        c.accept_spline(message(actual[2]['metadata']['trajectory']),actual[2]['metadata'])
        check('retention_requires_actual_cloud_300ms',unchanged(before,snapshot(c)) and c.path_admission_hold,decision=c.path_admission_last_decision)
        c.cloud_updated=c.pose_updated=c.bridge_updated=time.monotonic();c.request_id+=':different-request'
        c.accept_spline(message(actual[2]['metadata']['trajectory']),actual[2]['metadata'])
        check('retention_requires_same_request',c.path_admission_hold,decision=c.path_admission_last_decision)
        c.request_id=actual[0]['request_id'];c.waypoints[9,0]+=.01
        bad=copy.deepcopy(actual[2]['metadata']);bad['body_goal']=c.waypoints[9].tolist()
        c.accept_spline(message(bad['trajectory']),bad)
        check('retention_requires_same_fixed_goal',c.path_admission_hold,decision=c.path_admission_last_decision)
        c.waypoints[9]=actual[0]['metadata']['body_goal'];c.request_id=actual[0]['request_id']
        # Missing/changed frozen anchor cannot retain a previously legal curve.
        original=(run/'navigation_anchor.json').read_bytes();modified=json.loads(original);modified['yaw']+=.01
        (run/'navigation_anchor.json').write_text(json.dumps(modified)+'\n')
        before=snapshot(c);c.accept_spline(message(actual[2]['metadata']['trajectory']),actual[2]['metadata'])
        check('frozen_anchor_change_failclosed',unchanged(before,snapshot(c)) and c.path_admission_hold
            and 'frozen_anchor_changed' in c.path_admission_last_decision['reason'],decision=c.path_admission_last_decision)
        (run/'navigation_anchor.json').write_bytes(original)
        (run/'navigation_anchor.json').unlink();c.accept_spline(message(actual[2]['metadata']['trajectory']),actual[2]['metadata'])
        check('missing_actual_sensor_anchor_failclosed',c.path_admission_hold and 'FileNotFoundError' in c.path_admission_last_decision['reason'])
        (run/'navigation_anchor.json').write_bytes(original)
        # Finite extreme candidate cannot mutate path or enter the accepted archive.
        v=copy.deepcopy(actual[2]['metadata']['trajectory']);v['pos_pts']=[[5e307]*3]*len(v['pos_pts'])
        m=copy.deepcopy(actual[2]['metadata']);m['trajectory']=v
        before=snapshot(c);c.accept_spline(message(v),m)
        check('overflow_production_rejection_preserves_state',unchanged(before,snapshot(c)) and c.path_admission_hold,decision=c.path_admission_last_decision)
        # Deadline in protected admission hold is unchanged, even without an
        # executable new path. The numeric PI sentinel stays untouched.
        c.goals=[NS(legacy=False,timeout_sim_s=90)]*10;c.segment_start=c.pose.copy();c.segment_started_ros=0
        c.reset_region_arrival=lambda reason:setattr(c,'arrival_reset_reason',reason)
        c.control_admission_hold(time.monotonic())
        check('90s_deadline_active_during_admission_hold',c.state=='failed' and c.arrival_reset_reason=='failed'
            and c.cascade_last_row==before['cascade'])
        # Final parking uses the old admitted reference and never rewrites the
        # association of pending planner output. Status retains this provenance
        # alongside the same final active id after shared arrival clears it.
        c.state='succeeded';c.waypoint_index=10;c.active_reference_stamp=None
        fixed=tuple(actual[1]['reference_stamp']);pending=(fixed[0]+7,fixed[1])
        c.trajectory_association.reference_stamp=pending
        c.final_bundle=dict(index=9,start=c.pose.copy(),trajectory_id=80,array='old80.npz',
            path_receipt=copy.deepcopy(c.path_receipt),reference_stamp=fixed)
        c.cascade=NS(goal_yaw=0.);c.rotation=np.eye(3);c.paired_imu={'stamp_ns':1}
        c.pose_updated=c.cloud_updated=time.monotonic();c.apply_tilt_guard=lambda now,ros:False
        c.select_pid_velocity=lambda *args,**kw:(np.zeros(2),0.)
        c.cloud=np.zeros((0,3));c.pending_guard_row=None;c.command=[0,0,0]
        seen=[]
        def guard(*args):
            seen.append(dict(active=c.active_reference_stamp,pending=c.trajectory_association.reference_stamp))
            return False,None,0
        c.evaluate_motion_guard=guard;c.publish_command=lambda *args,**kw:None
        c.control_parking(time.monotonic())
        check('final_parking_guard_uses_admitted_reference_pending_unchanged',
            c.trajectory_association.reference_stamp==pending and c.active_reference_stamp==fixed
            and c.waypoint_index==10 and seen==[dict(active=fixed,pending=pending)],observed=seen)
        # First fresh pose during an admission hold starts the original goal
        # timeout, and archives its exact source so arrival does not invent a
        # later activation pose when a valid path eventually appears.
        c.state='running';c.waypoint_index=9;c.segment_start=None;c.segment_references={}
        c.feedback={'stamp_ns':c.pose_stamp};c.stale_since=None;c.pose_timeout=c.cloud_timeout=.3
        c.control_admission_hold(time.monotonic())
        check('admission_hold_preserves_exact_segment_activation_source',
            c.segment_started_ros==100 and c.segment_references[(c.request_id,9)]['source_pose_stamp_ns']==c.pose_stamp
            and np.array_equal(c.segment_start,c.pose),activation=c.segment_references[(c.request_id,9)])
        REPORT['callback_benchmark_ms']=callback_timings


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);args=parser.parse_args()
    try:
        actual,anchor=proof_tests();transaction_tests(actual,anchor)
        REPORT['passed']=True
    except Exception as error:
        REPORT['passed']=False;REPORT['error']=type(error).__name__+': '+str(error)
        if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(REPORT,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
        raise
    REPORT['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in
        [HERE/'path_admission.py',HERE/'controller.py',HERE/'shared_controller.py',HERE/'teacher_wrapper.py',Path(__file__)]}
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(REPORT,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
    print(json.dumps(dict(passed=REPORT['passed'],checks=len(REPORT['tests']),output=str(args.output),callback_ms=REPORT['callback_benchmark_ms'])))

if __name__=='__main__':main()
