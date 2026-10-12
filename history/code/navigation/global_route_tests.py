"""Finite V33 regressions using production methods; no ROS or source-run writes."""
import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np
from cascade_core import Controller
from global_route_reference import (contract,SegmentGuides,StableProjectionController,
    direction_check,admission_direction_check)
from pure_tests import sources
from pid_core import slew

HERE=Path(__file__).resolve().parent
SETTINGS=contract({'global_route_reference_contract':{'enabled':True}})
CONFIG=dict(desired_speed=.2,external_heading_gate=True,spatial_reference=dict(enabled=True,arc_length_m=.2))


def method(file,name,extra=None,replace_super=False):
    tree=ast.parse((HERE/file).read_text())
    function=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    if replace_super:
        class FixtureBase(ast.NodeTransformer):
            def visit_Call(self,node):
                if (isinstance(node.func,ast.Attribute) and node.func.attr=='control'
                        and isinstance(node.func.value,ast.Call) and isinstance(node.func.value.func,ast.Name)
                        and node.func.value.func.id=='super'):
                    return ast.Call(func=ast.Name(id='fixture_base_control',ctx=ast.Load()),args=node.args,keywords=node.keywords)
                return self.generic_visit(node)
        function=FixtureBase().visit(function)
    namespace=dict(np=np,math=math,copy=copy,hashlib=hashlib,json=json)
    namespace.update(extra or {})
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),
        str(HERE/file)+':production_method','exec'),namespace)
    return namespace[name]


def core(path=None,position=(0.,0.,.32)):
    c=StableProjectionController(CONFIG,[4.,0.,.32],0.,'goal')
    c.set_path(path or [[0.,0.,.32],[4.,0.,.32]],'scan1',0,0,position=np.asarray(position))
    return c


class GlobalReferenceChecks(unittest.TestCase):
    def test_contract_default_off_and_invalid_inputs(self):
        self.assertFalse(contract({})['enabled'])
        for value in ({'enabled':'yes'},{'enabled':True,'guide_spacing_m':False},
                      {'enabled':True,'max_guide_points':1000},{'enabled':True,'direction_audit_arc_m':float('nan')}):
            with self.assertRaises(ValueError):contract({'global_route_reference_contract':value})

    def test_fixed_guides_end_at_original_goal_without_editing_goal(self):
        ledger=SegmentGuides();goal=[5.,0.,.82];start=[0.,0.,.32]
        points,receipt=ledger.remaining(('request',7),start,goal,[0.,0.,.32],SETTINGS)
        np.testing.assert_allclose(points,[[1.,0.,.42],[2.,0.,.52],[3.,0.,.62],[4.,0.,.72],goal],rtol=0,atol=1e-15)
        self.assertEqual(points[-1],goal)
        self.assertEqual(goal,[5.,0.,.82]);self.assertTrue(receipt['guide_only_not_arrival'])
        self.assertFalse(receipt['collision_or_support_certified'])

    def test_actual_on60_near_terminal_helper_keeps_only_exact_original_goal(self):
        # First ON60 request had length 1.00865m: its 1m helper was only
        # 8.64mm from the mandatory goal. Regress actual camera_init geometry.
        start=[0.002241295914149961,0.021215919002596867,0.0053291244935592934]
        goal=[-0.007955120008738954,-0.9873690270198282,0.003191722996237475]
        frozen_goal=goal.copy();delta=np.array(goal)-start;length=np.linalg.norm(delta[:2])
        self.assertGreater(length,1.);self.assertLess(length,1.01)
        helper=np.array(start)+delta/length
        self.assertLess(np.linalg.norm(helper-goal),.009)
        points,receipt=SegmentGuides().remaining(('actual_ON60',0),start,goal,start,SETTINGS)
        self.assertEqual(points,[frozen_goal]);self.assertEqual(goal,frozen_goal)
        self.assertEqual(receipt['terminal_helpers_coalesced'],1)
        self.assertEqual(receipt['terminal_helper_min_distance_m'],.5)
        self.assertTrue(receipt['guide_only_not_arrival'])
        self.assertFalse(receipt['collision_or_support_certified'])

    def test_long_segment_terminal_coalesce_keeps_fixed_grid_and_progress(self):
        start=[0.,0.,.32];goal=[5.2,0.,.84];ledger=SegmentGuides()
        first,receipt=ledger.remaining(('long',0),start,goal,start,SETTINGS)
        expected=[[1.,0.,.42],[2.,0.,.52],[3.,0.,.62],[4.,0.,.72],goal]
        np.testing.assert_allclose(first,expected,rtol=0,atol=1e-15)
        self.assertEqual(first[-1],goal);self.assertEqual(receipt['terminal_helpers_coalesced'],1)
        later,_=ledger.remaining(('long',0),start,goal,[2.2,0.,.54],SETTINGS)
        np.testing.assert_allclose(later,expected[2:],rtol=0,atol=1e-15)
        self.assertEqual(later[-1],goal)
        backward,receipt=ledger.remaining(('long',0),start,goal,[1.8,0.,.5],SETTINGS)
        self.assertEqual(backward,later);self.assertEqual(receipt['planning_progress_m'],2.2)

    def test_terminal_coalesce_threshold_is_strict_and_goal_stays_exact(self):
        for length,expected,removed in [(1.49,[1.49],1),(1.5,[1.,1.5],0),
                                        (1.51,[1.,1.51],0),(2.01,[1.,2.01],1)]:
            goal=[length,0.,0.]
            points,receipt=SegmentGuides().remaining(('threshold',0),[0.,0.,0.],goal,[0.,0.,0.],SETTINGS)
            self.assertEqual([p[0] for p in points],expected)
            self.assertEqual(points[-1],goal);self.assertEqual(receipt['terminal_helpers_coalesced'],removed)

    def test_guide_progress_never_regresses_and_goal_identity_resets(self):
        ledger=SegmentGuides()
        _,first=ledger.remaining(('r',0),[0,0,0],[5,0,.5],[2.2,.1,.22],SETTINGS)
        points,second=ledger.remaining(('r',0),[0,0,0],[5,0,.5],[1.9,.1,.19],SETTINGS)
        self.assertEqual(first['planning_progress_m'],second['planning_progress_m'])
        self.assertEqual(points[0],[3.,0.,.3])
        _,third=ledger.remaining(('r',1),[5,0,.5],[0,0,0],[5,0,.5],SETTINGS)
        self.assertEqual(third['planning_progress_m'],0.)
        with self.assertRaises(ValueError):ledger.remaining(('r',1),[5,0,.5],[1,0,0],[5,0,.5],SETTINGS)

    def test_guide_count_and_nonfinite_geometry_fail_closed(self):
        with self.assertRaises(ValueError):SegmentGuides().remaining(('r',0),[0,0,0],[200,0,0],[0,0,0],SETTINGS)
        with self.assertRaises(ValueError):SegmentGuides().remaining(('r',0),[0,0,0],[1,0,0],[float('nan'),0,0],SETTINGS)

    def test_direction_audit_does_not_replace_curve_or_claim_chord_safe(self):
        path=[[0,0,.32],[-.05,-.15,.32],[.25,-.15,.32],[4,0,.32]]
        c=core(path);before=c.path.copy()
        check=direction_check(c,c.preview_reference([0,0,.32]),.2,SETTINGS)
        self.assertFalse(check['allowed']);self.assertFalse(check['audit_chord_is_execution_path'])
        np.testing.assert_array_equal(c.path,before)
        with self.assertRaises(ValueError):direction_check(c,c.preview_reference([0,0,.32]),.21,SETTINGS)

    def test_straight_ramp_and_fixed_goal_tail_remain_eligible(self):
        c=StableProjectionController(CONFIG,[4,0,.72],0,'ramp')
        c.set_path([[0,0,.32],[4,0,.72]],'ramp',0,0)
        for point in ([0,0,.32],[2,0,.52],[3.95,0,.715]):
            check=direction_check(c,c.preview_reference(point),.2,SETTINGS)
            self.assertTrue(check['allowed'])

    def test_same_path_projection_cannot_backtrack(self):
        c=core();c._project(np.array([1.,0.,.32]));progress=c.progress
        reference=c.preview_reference([.9,0,.32])
        self.assertGreaterEqual(reference['nearest_s_m'],progress)
        self.assertEqual(c.progress,progress)
        c._project(np.array([.9,0,.32]));self.assertEqual(c.progress,progress)

    def test_new_trajectory_identity_resets_projection_not_arrival(self):
        c=core();c._project(np.array([1.,0,.32]))
        c.set_path([[0,0,.32],[4,0,.32]],'scan2',1,1,position=np.array([1.,0,.32]))
        self.assertEqual(c.progress,0.);self.assertTrue(c.new_path)
        self.assertTrue(c.path_handoff['projection_progress_reset_for_new_identity'])
        self.assertIsNone(c.capture_stamp)

    def test_preview_and_controller_use_exact_same_heading_and_no_preview_integration(self):
        c=core();pose=np.array([.2,.01,.32])
        c.update(*sources(100_000_000,position=pose),mode_override='drive')
        before=copy.deepcopy(c.__dict__);reference=c.preview_reference(pose)
        self.assertEqual(c.progress,before['progress'])
        np.testing.assert_array_equal(c.velocity_integral,before['velocity_integral'])
        _,row=c.update(*sources(200_000_000,position=pose),mode_override='drive')
        self.assertAlmostEqual(reference['heading_rad'],row['reference_yaw_rad'],places=12)

    def test_production_follow_keeps_checked_target_while_incoherent_holds(self):
        c=core([[0,0,.32],[-.05,-.15,.32],[.25,-.15,.32],[4,0,.32]])
        original=(np.array([.2,0]),0.,np.array([.6,0,.32]),
            dict(heading=0.,error=0.,direction=[1.,0.],exhausted=False))
        records=[];node=NS(cascade=c,pose=np.array([0,0,.32]),rotation=np.eye(3),
            request_id='r',waypoint_index=0,pose_stamp=10,feedback=None,run=Path('/unused'),
            global_reference_settings=SETTINGS,profile={'teacher_transition':{'drive_heading_max_rad':.2}},
            ensure_cascade=lambda:None,get_clock=lambda:NS(now=lambda:NS(nanoseconds=20)),
            evidence=NS(append=lambda p,row:records.append(row)),reference_replan_pending=False)
        follow=method('controller.py','follow_checked_trajectory',
            dict(follow_trajectory=lambda *a,**k:copy.deepcopy(original),direction_check=direction_check))
        result=follow(node,return_steering=True)
        np.testing.assert_array_equal(result[2],original[2])
        self.assertTrue(node.reference_consistency_hold);self.assertTrue(node.reference_replan_pending)
        self.assertFalse(result[3]['direction_coherence']['original_path_replaced'])
        self.assertEqual(result[3]['heading'],c.preview_reference(node.pose)['heading_rad'])

    def test_disabled_reference_preserves_frozen_v32_commands_and_diagnostics(self):
        # Compare actual production selection methods, not a rewritten model.
        from types import SimpleNamespace
        fixed_time=SimpleNamespace(monotonic_ns=lambda:10_205_000_000)
        old=method('../map_surface_v32/controller.py','select_pid_velocity',dict(slew=slew,time=fixed_time))
        new=method('controller.py','select_pid_velocity',dict(slew=slew,time=fixed_time))
        for phase,allow in [('drive',True),('align',False),('pre_turn',False),('settle',False)]:
            feedback,imu,ack,clock,wall=sources(200_000_000)
            def fixture():
                c=Controller(CONFIG,[4.,0.,.32],0.,'goal')
                c.set_path([[0,0,.32],[4,0,.32]],'scan1',0,0)
                c.update(*sources(100_000_000),mode_override='drive')
                return NS(cascade=c,get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock)),ensure_cascade=lambda:None,
                    obstacle_hold=False,heading_gate=NS(phase=phase,heading=0.),steering=dict(direction=[1,0]),get_ack=lambda:ack,
                    feedback=feedback,paired_imu=imu,state='running',command=[0,0,0],last_command_time=.15,rotation=np.eye(3),
                    control_clock_hold=NS(guard_required=False),pid_records=0,pose_stamp=200_000_000,path_receipt=None,
                    pose=np.array([0,0,.32]),pose_quaternion=[0,0,0,1],request_id='r',waypoint_index=0,active_trajectory_id=1,
                    trajectory_archive_reference='scan.npy',waypoints=[np.array([4,0,.32])],reference_consistency_hold=False)
            a,b=fixture(),fixture()
            ra=old(a,np.zeros(2),0.,np.array([.5,0,.32]),allow)
            rb=new(b,np.zeros(2),0.,np.array([.5,0,.32]),allow)
            np.testing.assert_array_equal(ra[0],rb[0]);self.assertEqual(ra[1],rb[1])
            self.assertEqual(a.cascade_last_row,b.cascade_last_row)

    def test_disabled_admission_hold_preserves_frozen_hold_behavior(self):
        old=method('../map_surface_v32/controller.py','control_admission_hold')
        new=method('controller.py','control_admission_hold')
        for stamp,wall in [(10_000_000_000,100.),(10_000_000_000,101.),(91_000_000_000,100.)]:
            a,b=self.hold_node(stamp=stamp),self.hold_node(stamp=stamp)
            a.global_reference_settings=b.global_reference_settings={'enabled':False}
            old(a,wall);new(b,wall)
            for key in ('state','message','command','zero_count','reset_reasons','checks','obstacle_hold',
                        'obstacle_clear_since','segment_started_ros'):
                self.assertEqual(getattr(a,key),getattr(b,key))

    def test_direction_hold_survives_cascade_protect_and_keeps_replan_pending(self):
        feedback,imu,ack,clock,wall=sources(200_000_000)
        c=core();c.update(*sources(100_000_000),mode_override='drive')
        node=NS(cascade=c,get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock)),ensure_cascade=lambda:None,
            obstacle_hold=False,heading_gate=NS(phase='drive',heading=None),steering=dict(direction=[1,0]),get_ack=lambda:ack,
            feedback=feedback,paired_imu=imu,state='running',command=[0,0,0],last_command_time=.15,rotation=np.eye(3),
            control_clock_hold=NS(guard_required=False),pid_records=0,pose_stamp=200_000_000,path_receipt=None,
            pose=np.array([0,0,.32]),pose_quaternion=[0,0,0,1],request_id='r',waypoint_index=0,active_trajectory_id=1,
            trajectory_archive_reference='scan.npy',waypoints=[np.array([4,0,.32])],reference_consistency_hold=True,
            reference_replan_pending=True)
        select=method('controller.py','select_pid_velocity',dict(slew=slew,time=NS(monotonic_ns=lambda:wall)))
        velocity,rate=select(node,np.array([.2,0.]),.1,np.array([.5,0.,.32]),False)
        np.testing.assert_array_equal(velocity,np.zeros(2));self.assertEqual(rate,0.)
        self.assertEqual(node.cascade_last_row['mode'],'protect');self.assertTrue(node.reference_replan_pending)

    def hold_node(self,stamp=10_000_000_000,stopped=True):
        node=NS(state='running',path_admission_hold=True,global_reference_settings=SETTINGS,segment_start=np.array([0,0,.32]),
            segment_started_ros=0.,goals=[NS(legacy=False,timeout_sim_s=90.)],waypoint_index=0,
            pose=np.array([.2,0,.32]),waypoints=[np.array([4,0,.32])],pose_stamp=stamp,
            pose_timeout=.3,cloud_timeout=.3,pose_updated=100.,cloud_updated=100.,
            raw_imu=NS(fresh=lambda w,s:True),apply_tilt_guard=lambda w,s:False,stale_since=None,
            pid_guard_needs_evaluation=False,last_obstacle_check=99.,obstacle_hold=True,
            obstacle_clear_since=None,obstacle_hold_started_ros=5.,obstacle_resumes=0,
            obstacle_resume_pending=False,steering=None,cloud=object(),samples=np.array([[0,0,.32],[4,0,.32]]),
            execution_path_exhaustion=None,pending_guard_row=None,command=[.2,0,0],run=Path('/unused'),
            evidence=NS(error=None),message='waiting',zero_count=0,reset_reasons=[],records=[],checks=0)
        node.clock=stamp
        node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=node.clock))
        node.reset_region_arrival=lambda reason:node.reset_reasons.append(reason)
        def zero():node.command=[0.,0.,0.];node.zero_count+=1
        node.publish_command=zero
        node.heading_gate=NS(stopped=lambda s,w:stopped,reset=lambda:None)
        def guard(*args):
            node.checks+=1;self.assertEqual(node.command,[0.,0.,0.]);node.pending_guard_row={}
            return node.guard_result
        node.guard_result=(False,None,0);node.evaluate_motion_guard=guard
        node.evidence.append=lambda p,row:node.records.append(row)
        return node

    def test_admission_hold_checks_clearance_but_never_releases_path_or_arrives(self):
        n=self.hold_node();control=method('controller.py','control_admission_hold')
        for stamp in (10.,10.5,11.01):
            n.clock=round(stamp*1e9);wall=100.+stamp-10.;n.pose_updated=n.cloud_updated=wall
            control(n,wall)
        self.assertEqual(n.checks,3);self.assertFalse(n.obstacle_hold)
        self.assertTrue(n.path_admission_hold);self.assertIsNone(n.samples)
        self.assertFalse(n.obstacle_resume_pending);self.assertEqual(n.command,[0.,0.,0.])
        self.assertEqual(n.segment_started_ros,0.)
        self.assertEqual(n.reset_reasons,['path_admission_hold']*3)
        self.assertTrue(all(r['admission_hold_observation_only'] for r in n.records))

    def test_admission_hold_requires_actual_stopping_and_blocked_resets_clear(self):
        control=method('controller.py','control_admission_hold');n=self.hold_node(stopped=False)
        control(n,100.);self.assertIsNone(n.obstacle_clear_since);self.assertTrue(n.obstacle_hold)
        n=self.hold_node();control(n,100.);n.clock=10_500_000_000;n.pose_updated=n.cloud_updated=100.5
        n.guard_result=(True,.4,1);control(n,100.5)
        self.assertIsNone(n.obstacle_clear_since);self.assertTrue(n.obstacle_hold)

    def test_admission_hold_stale_and_deadline_remain_protected(self):
        control=method('controller.py','control_admission_hold');n=self.hold_node()
        control(n,101.);self.assertEqual(n.checks,0);self.assertTrue(n.obstacle_hold)
        n=self.hold_node(stamp=91_000_000_000);control(n,100.)
        self.assertEqual(n.state,'failed');self.assertIn('时间限制',n.message);self.assertEqual(n.checks,0)
        self.assertEqual(n.command,[0.,0.,0.])

    def test_wrapper_does_not_overwrite_failure_or_reference_hold_message(self):
        for state,reference_hold in [('failed',False),('running',True),('succeeded',False)]:
            node=NS(state=state,message='original cause',evidence=NS(error=None),profile={},
                obstacle_hold=False,pending_guard_row=None,reference_consistency_hold=reference_hold,
                heading_gate=NS(phase='pre_turn'))
            control=method('teacher_wrapper.py','control',dict(fixture_base_control=lambda now:None),replace_super=True)
            control(node,1.)
            self.assertEqual(node.message,'original cause')

    def test_disabled_cloud_archive_preserves_live_input_and_explicit_missing_evidence(self):
        rows=[];cloud=np.ones((10,3));receipt=dict(filtered_xyz_float64_sha256='live-hash')
        n=NS(cloud_archive_records=0,registered_cloud_archive_mode='disabled',guard_cloud=cloud,
            guard_cloud_receipt=receipt,cloud_stamp=123,run=Path('/unused'),
            evidence=NS(append=lambda p,row:rows.append(row)))
        msg=NS(header=NS(frame_id='camera_init'),data=b'rawbytes')
        method('controller.py','archive_cloud')(n,msg)
        self.assertIs(n.guard_cloud,cloud);self.assertIsNone(receipt['cloud_array_file'])
        self.assertFalse(receipt['cloud_archive_collected']);self.assertIsNone(receipt['cloud_archive_sequence'])
        self.assertFalse(rows[0]['cloud_replay_verified']);self.assertTrue(rows[0]['live_guard_input_preserved'])


def replay_saved(source,output):
    """Review actual recorded geometry; no counterfactual closed-loop PASS."""
    source=Path(source);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    path_rows={}
    path_file=source/'cascade_path_history.jsonl';history=source/'navigation_pid_history.jsonl'
    for line in path_file.open():
        row=json.loads(line)
        if row['waypoint_index']==7:path_rows[row['path_id']]=row
    cores={};summary={};compact=[]
    for line in history.open():
        row=json.loads(line)
        if row['waypoint_index']!=7:continue
        projection=row['heading_gate_reference']['steering']['cascade_projection']
        pid=projection['path_id'];p=path_rows.get(pid)
        if p is None:continue
        if pid not in cores:
            c=StableProjectionController(CONFIG,row['goal'],0.,'recorded_goal')
            c.set_path(p['points_xyz'],pid,0,0,position=np.array(row['control_pose']));cores[pid]=c
        # The recorded V32 heading is checked against the same original curve.
        # This intentionally avoids inventing motion after a rejected path.
        check=direction_check(cores[pid],projection['reference'],.2,SETTINGS)
        item=summary.setdefault(str(p['trajectory_id']),dict(samples=0,rejected=0,first_source_s=None,
            last_source_s=None,min_difference_rad=None,max_difference_rad=None,path_sha256=cores[pid].path_sha))
        item['samples']+=1;item['rejected']+=int(not check['allowed'])
        stamp=row['source_pose_stamp_ns']/1e9
        if item['first_source_s'] is None:item['first_source_s']=stamp
        item['last_source_s']=stamp
        difference=abs(check['difference_rad']) if check['difference_rad'] is not None else None
        if difference is not None:
            item['min_difference_rad']=difference if item['min_difference_rad'] is None else min(item['min_difference_rad'],difference)
            item['max_difference_rad']=difference if item['max_difference_rad'] is None else max(item['max_difference_rad'],difference)
        if p['trajectory_id']==68:
            compact.append(dict(source_s=stamp,phase=row['heading_gate_reference']['phase'],
                pose_stamp_ns=row['source_pose_stamp_ns'],allowed=check['allowed'],difference_rad=check['difference_rad']))
    report=dict(schema='v33_saved_curve_direction_regression/v1',source_run=str(source),
        source_files_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (path_file,history)},
        original_maximum_direction_difference_rad=.2,results_by_trajectory=summary,
        saved_traj68_samples=len(compact),closed_loop_pass=False,simulation_started=False,
        truth_used=False,point_cloud_copied=False,
        limitation='Only saved curve reference admission was audited. Rejection/zero hold is not physical arrival or repaired navigation.')
    (output/'SAVED_PATH_REPLAY.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    with (output/'TRAJ68_DIRECTION_COMPACT.jsonl').open('w') as f:
        for row in compact:f.write(json.dumps(row,allow_nan=False)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source-run');parser.add_argument('--results')
    args=parser.parse_args()
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(GlobalReferenceChecks)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    if args.source_run:
        if not args.results:parser.error('--results required with --source-run')
        report=replay_saved(args.source_run,args.results)
        print(json.dumps(dict(tests_run=result.testsRun,finite_tests_pass=result.wasSuccessful(),
            traj68=report['results_by_trajectory'].get('68'),closed_loop_pass=False),indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)
