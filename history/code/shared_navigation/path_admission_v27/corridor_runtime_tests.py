"""Finite worker/ownership/lifecycle tests; no ROS, Gazebo, or safety proof."""
import ast
import copy
from collections import deque
import json
import math
import os
import argparse
from pathlib import Path
from types import SimpleNamespace, ModuleType
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from corridor_runtime import CorridorObserver, MAX_SNAPSHOT_BYTES, decode_snapshot, sha_text
from controller_reference_tests import wrapper_method
from clock_hold import ControlClockHold


def snapshot(revision=1):
    return dict(schema='teacher_scan_local_snapshot/v1',frame_id='camera_init',revision=revision,
        shape_xyz=[1,1,1],origin_index_xyz=[0,0,0],states='0',surface_points_xyz=[[0.,0.,0.]],
        resolution_m=.1,cell_observation_age_ms=[0],raw_occupancy_not_robot_inflated=True,
        source='actual_registered_lidar_raycast',navigation_ground_truth_used=False,
        complete=True,stamp_ns=1_000_000_000,source_cloud_stamp_ns=1_000_000_000,
        source_sensor_pose_stamp_ns=1_000_000_000)


def inputs(revision=1):
    s=snapshot(revision)
    path=dict(points_xyz=[[0.,0.,.32],[1.,0.,.32]],path_id='checked',path_sha256='a'*64,
        frame_id='camera_init',layer_id='floor1',stamp_ns=1_000_000_000)
    state=dict(position_world_xyz=[0.,0.,.32],yaw_rad=0.,speed_mps=.2,progress_m=0.,stamp_ns=1_000_000_000)
    return s,path,state,1_000_000_000,5_000_000_000,sha_text(json.dumps(s))


class RuntimeDecoderChecks(unittest.TestCase):
    def test_valid_snapshot_and_hash_do_not_change_source_bytes(self):
        original=json.dumps(snapshot(),indent=2)
        self.assertEqual(decode_snapshot(original),snapshot())
        self.assertEqual(len(sha_text(original)),64)
        self.assertNotEqual(sha_text(original),sha_text(json.dumps(snapshot())))

    def test_nonfinite_literals_and_overflowed_json_numbers_are_rejected(self):
        raw=json.dumps(snapshot())
        for number in ('NaN','Infinity','-Infinity','1e309','-1e309'):
            with self.subTest(number=number):
                damaged=raw.replace('"surface_points_xyz": [[0.0, 0.0, 0.0]]',
                    '"surface_points_xyz": [['+number+', 0.0, 0.0]]')
                self.assertNotEqual(damaged,raw)
                with self.assertRaises(ValueError):decode_snapshot(damaged)

    def test_oversize_shapes_states_and_surface_arrays_are_rejected(self):
        with self.assertRaises(ValueError):decode_snapshot(' '* (MAX_SNAPSHOT_BYTES+1))
        bad=[]
        for shape in ([0,1,1],[True,1,1],[257,1,1],[64,64,17]):
            item=snapshot();item['shape_xyz']=shape;bad.append(item)
        item=snapshot();item['states']='3';bad.append(item)
        item=snapshot();item['states']='00';bad.append(item)
        item=snapshot();item['surface_points_xyz']=[[0,0,0]]*12001;bad.append(item)
        for item in bad:
            with self.subTest(item_shape=item['shape_xyz'],points=len(item['surface_points_xyz'])):
                with self.assertRaises(ValueError):decode_snapshot(json.dumps(item))

    def test_deeply_nested_extra_payload_is_bounded(self):
        item=snapshot();nested=0
        for _ in range(30):nested=[nested]
        item['unexpected_nested_payload']=nested
        with self.assertRaises(ValueError):decode_snapshot(json.dumps(item))


class RuntimeWorkerChecks(unittest.TestCase):
    def test_one_owned_job_and_busy_counts_without_backlog(self):
        started=threading.Event();release=threading.Event();seen=[]
        def worker(path,snap,support,state,limits,now):
            started.set();self.assertTrue(release.wait(2.));seen.append((path,snap,state,limits,now))
            return {'schema':'teacher_corridor_certificate/v1','status':'blocked'}
        limits={'horizon_m':1.2};observer=CorridorObserver(limits,worker)
        args=list(inputs())
        try:
            self.assertTrue(observer.submit(*args));self.assertTrue(started.wait(2.))
            for _ in range(20):self.assertFalse(observer.submit(*inputs(2)))
            self.assertEqual(observer.submitted,1);self.assertEqual(observer.busy_snapshots,20)
            self.assertIsNone(observer.poll())
            args[0]['surface_points_xyz'][0][0]=99.
            args[1]['points_xyz'][0][0]=99.;args[2]['position_world_xyz'][0]=99.;limits['horizon_m']=99.
            release.set();result=observer.close()
            self.assertEqual(observer.completed,1)
            self.assertEqual(seen[0][0]['points_xyz'][0][0],0.)
            self.assertEqual(seen[0][1]['surface_points_xyz'][0][0],0.)
            self.assertEqual(seen[0][2]['position_world_xyz'][0],0.)
            self.assertEqual(seen[0][3]['horizon_m'],1.2)
            self.assertEqual(result['observation']['position_world_xyz'][0],0.)
            self.assertFalse(result['control_authority']);self.assertTrue(result['shadow_only'])
            self.assertFalse(observer.submit(*inputs(2)));self.assertIsNone(observer.close())
        finally:release.set();observer.close()

    def test_nonincreasing_revisions_are_rejected_after_completed_poll(self):
        observer=CorridorObserver({},lambda *args:{'status':'unavailable'})
        try:
            self.assertTrue(observer.submit(*inputs(3)))
            observer.future.result(timeout=2.);self.assertIsNotNone(observer.poll())
            for revision in (3,2,-1,True,1.5):
                with self.subTest(revision=revision):
                    with self.assertRaises(ValueError):observer.submit(*inputs(revision))
            self.assertTrue(observer.submit(*inputs(4)))
            self.assertEqual(observer.close()['observation']['map_revision'],4)
        finally:observer.close()

    def test_worker_exception_returns_unavailable_with_source_bindings(self):
        def worker(*args):raise RuntimeError('intentional worker error')
        observer=CorridorObserver({},worker)
        try:
            observer.submit(*inputs());result=observer.close()
            self.assertEqual(result['status'],'unavailable')
            self.assertIn('intentional worker error',result['reason'])
            self.assertEqual(result['observation']['path_id'],'checked')
            self.assertFalse(result['control_authority'])
        finally:observer.close()

    def test_invalid_worker_results_cannot_escape_the_shadow_boundary(self):
        for malformed in (None,17,['bad'],{'status':'certified','nonfinite':float('nan')}):
            with self.subTest(result=malformed):
                observer=CorridorObserver({},lambda *args:malformed)
                try:
                    observer.submit(*inputs());result=observer.close()
                    self.assertEqual(result['status'],'unavailable')
                    self.assertFalse(result['control_authority'])
                finally:observer.close()

    def test_worker_input_mutation_cannot_rewrite_observation_identity(self):
        def worker(path,snap,support,state,limits,now):
            path['path_id']='changed_by_worker';state['position_world_xyz'][0]=99.
            return {'status':'unavailable'}
        observer=CorridorObserver({},worker)
        try:
            observer.submit(*inputs());result=observer.close()
            self.assertEqual(result['observation']['path_id'],'checked')
            self.assertEqual(result['observation']['position_world_xyz'][0],0.)
        finally:observer.close()

    def test_future_infrastructure_error_is_an_unavailable_diagnostic(self):
        def failure():raise RuntimeError('synthetic future failure')
        observer=CorridorObserver({},lambda *args:{'status':'unavailable'})
        try:
            observer.future=SimpleNamespace(done=lambda:True,result=failure)
            result=observer.poll()
            self.assertEqual(result['status'],'unavailable')
            self.assertFalse(result['control_authority'])
            self.assertEqual(observer.completed,1)
            self.assertIsNone(observer.poll())
        finally:observer.close()

    def test_worker_cannot_mutate_limits_of_a_later_job(self):
        observed=[]
        def worker(path,snap,support,state,limits,now):
            observed.append(limits['horizon_m']);limits['horizon_m']=99.
            return {'status':'unavailable'}
        observer=CorridorObserver({'horizon_m':1.2},worker)
        try:
            observer.submit(*inputs(1));observer.future.result(timeout=2.);observer.poll()
            observer.submit(*inputs(2));observer.close()
            self.assertEqual(observed,[1.2,1.2])
        finally:observer.close()

    def test_close_drains_owned_inflight_work_and_is_idempotent(self):
        started=threading.Event();release=threading.Event();closing=threading.Event();results=[]
        def worker(*args):
            started.set();self.assertTrue(release.wait(2.));return {'status':'unavailable'}
        observer=CorridorObserver({},worker)
        closer=None
        try:
            observer.submit(*inputs());self.assertTrue(started.wait(2.))
            def finish():closing.set();results.append(observer.close())
            closer=threading.Thread(target=finish);closer.start();self.assertTrue(closing.wait(2.))
            self.assertEqual(results,[])
            release.set();closer.join(2.);self.assertFalse(closer.is_alive())
            self.assertEqual(observer.completed,1);self.assertEqual(observer.submitted,1)
            self.assertEqual(results[0]['observation']['sequence'],1)
            self.assertIsNone(observer.poll());self.assertIsNone(observer.close())
        finally:
            release.set()
            if closer is not None:closer.join(2.)
            observer.close()


class RuntimeControllerBoundaryChecks(unittest.TestCase):
    @staticmethod
    def poll(node,now):
        node._poll_corridor=lambda stamp:wrapper_method('_poll_corridor',dict(time=time))(node,stamp)
        return wrapper_method('poll_corridor',dict(time=time))(node,now)

    def test_certificate_content_has_no_command_authority(self):
        malicious=dict(status='certified',command=[9,9,9],control_authority=True,shadow_only=False)
        observer=CorridorObserver({},lambda *args:malicious)
        observer.submit(*inputs());observer.future.result(timeout=2.)
        records=[]
        node=SimpleNamespace(corridor_observer=observer,command=[.2,.01,.03],state='idle',
            cascade=SimpleNamespace(path_id='new-active-path'),corridor_last_certificate=None,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_100_000_000)),
            evidence=SimpleNamespace(append=lambda p,row:records.append(row)),run=Path('/unused'))
        try:
            self.poll(node,3.)
            self.assertEqual(node.command,[.2,.01,.03]);self.assertEqual(node.state,'idle')
            self.assertFalse(records[0]['control_authority']);self.assertTrue(records[0]['shadow_only'])
            self.assertEqual(records[0]['active_path_id_at_receipt'],'new-active-path')
            self.assertEqual(records[0]['observation']['path_id'],'checked')
        finally:observer.close()

    def test_shadow_poll_failure_does_not_escape_into_navigation_control(self):
        def failure():raise RuntimeError('synthetic future infrastructure failure')
        records=[]
        node=SimpleNamespace(corridor_observer=SimpleNamespace(poll=failure),command=[.2,.01,.03],state='idle',
            cascade=None,corridor_last_certificate=None,get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_100_000_000)),
            evidence=SimpleNamespace(append=lambda p,row:records.append(row)),run=Path('/unused'))
        self.poll(node,3.)
        self.assertEqual(node.command,[.2,.01,.03]);self.assertEqual(node.state,'idle')
        self.assertTrue(records)
        self.assertFalse(records[-1]['control_authority'])

    def test_unvalidated_control_modes_and_retention_are_rejected(self):
        tree=ast.parse((Path(__file__).parent/'controller.py').read_text())
        parent=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PIDNavigation')
        init=next(n for n in parent.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        candidate=ast.ClassDef(name='Candidate',bases=[ast.Name(id='FakeBase',ctx=ast.Load())],keywords=[],body=[init],decorator_list=[])
        class FakeBase:
            def __init__(self,run):self.profile=run
        namespace=dict(FakeBase=FakeBase,ResetBookkeeping=lambda:None,deque=deque,math=math,ControlClockHold=ControlClockHold)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[candidate],type_ignores=[])),'production_controller_init','exec'),namespace)
        for config in ({'mode':'active'},{'mode':'shadow','retain_valid_plan':True},{'mode':'off','retain_valid_plan':True}):
            with self.subTest(config=config):
                with self.assertRaises(ValueError):namespace['Candidate']({'corridor':config})

    def test_shadow_close_exception_does_not_skip_original_writer_and_ros_cleanup(self):
        flags=[];records=[]
        def failure():raise RuntimeError('synthetic shadow shutdown failure')
        node=SimpleNamespace(corridor_observer=SimpleNamespace(close=failure),
            publish_command=lambda:flags.append('zero'),
            event_archive=SimpleNamespace(close=lambda:flags.append('event_archive_closed')),
            evidence=SimpleNamespace(append=lambda p,row:records.append(row),close=lambda:flags.append('evidence_closed'),error=None),
            pid_records=0,guard_sequence=0,imu_records=0,cloud_archive_records=0,cloud_callback_records=0,
            control_clock_hold=SimpleNamespace(records=0),publication_records=1,counts={'commands':1},
            destroy_node=lambda:flags.append('node_destroyed'))
        ros=ModuleType('rclpy');ros.init=lambda **kw:None;ros.spin=lambda candidate:None;ros.ok=lambda:True
        ros.try_shutdown=lambda:flags.append('ros_shutdown')
        ros.executors=SimpleNamespace(ExternalShutdownException=type('FakeShutdown',(Exception,),{}))
        main=wrapper_method('main',dict(argparse=argparse,Path=Path,os=os,json=json,PIDNavigation=lambda run:node))
        with tempfile.TemporaryDirectory() as directory,patch.dict(sys.modules,{'rclpy':ros}),patch.dict(os.environ,{},clear=False),patch.object(sys,'argv',['controller','--run',directory]):
            main()
            self.assertEqual(flags,['zero','event_archive_closed','evidence_closed','node_destroyed','ros_shutdown'])
            self.assertTrue(any('shadow_close:' in record.get('reason','') for record in records))
            self.assertEqual(json.loads((Path(directory)/'navigation_pid_writer_receipt.json').read_text())['status'],'drained')


if __name__=='__main__':unittest.main(verbosity=2)
