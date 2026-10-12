"""Finite observer tests only; synthetic event stubs are not live observations."""
import json
import ast
import hashlib
import os
import pickle
import queue
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import known_scene_temporal_capture as M
from known_scene_diagnostic_capture import BoundedSourceFrames
from rclpy import serialization


def event(ns, accepted=True, quality=True):
    messages = {r: SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9)), data=b'xyz') for r in M.ROLES}
    provenance = dict(integer_stamp_join=True, stage='frozen_post_LIO_single_owner_publication', source_membership_verified=True)
    result = dict(source_ns=ns, accepted=accepted, evidence_verified=accepted,
                  reason='test_original_accept' if accepted else 'test_original_reject',
                  best=dict(quality_passed=quality, rays=dict(abs_p95_m=.01)),
                  policy=dict(ray_abs_p95_max_m=.35), candidates=[dict(rejections=[] if accepted else ['ray_p95'])])
    identity = [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    result.update(input_C_world_odom=identity,T_odom_body=identity,T_odom_body_source_ns=ns)
    return dict(source_ns=ns, source_messages=messages, provenance=provenance,
                original_received_wall_ns={r: ns + 10 for r in M.ROLES}), result


class SelectionTests(unittest.TestCase):
    def observer(self):
        # No thread starts, no ROS/Gazebo nodes, no source serialization.
        with patch.object(M.threading.Thread, 'start'):
            return M.TemporalFrames(Path(tempfile.gettempdir()) / 'R29_V34_baseline_temporal_TEST_ONLY')

    def send(self, observer, ns, accepted=True, quality=True):
        job, result = event(ns, accepted, quality)
        before = json.dumps(result, sort_keys=True)
        observer.consider(job, result, None, 1)
        self.assertEqual(before, json.dumps(result, sort_keys=True))
        return job, result

    def test_qualified_history_remains_memory_only(self):
        o = self.observer()
        self.send(o, 1_000_000_000)
        self.send(o, 2_000_000_000)
        self.assertEqual(o.last_qualified['source_ns'], 2_000_000_000)
        self.assertEqual(o.pending.qsize(), 0)
        self.assertIsNone(o.first_rejection)

    def test_quality_without_accepted_is_not_anchor(self):
        o = self.observer()
        self.send(o, 1_000_000_000, accepted=False)
        self.assertIsNone(o.last_qualified)
        self.assertEqual(o.pending.qsize(), 0)

    def test_accepted_without_full_ray_quality_is_not_anchor(self):
        o = self.observer()
        self.send(o, 1_000_000_000, quality=False)
        self.assertIsNone(o.last_qualified)

    def test_four_distinct_groups_with_strict_3s_boundary(self):
        o = self.observer()
        self.send(o, 1_000_000_000)
        self.send(o, 2_000_000_000, False)
        self.send(o, 3_000_000_000, False)
        self.send(o, 4_000_000_000, False)
        self.assertNotIn('first_query_after_anchor_3s', o.role_coverage)
        self.send(o, 5_000_000_000, False)
        self.assertEqual(len(o.selected), 4)
        self.assertIn('first_query_after_anchor_3s', o.role_coverage)

    def test_one_independent_group_can_cover_two_roles(self):
        o = self.observer()
        self.send(o, 77_090_000_000)
        self.send(o, 79_135_000_000, False)
        self.send(o, 81_150_000_000, False)
        self.assertEqual(len(o.selected), 3)
        self.assertEqual(o.pending.queue[-1][1], ('next_independent_query', 'first_query_after_anchor_3s'))

    def test_first_rejection_can_already_be_after_3s(self):
        o = self.observer()
        self.send(o, 1_000_000_000)
        self.send(o, 4_100_000_000, False)
        self.assertIn('first_query_after_anchor_3s', o.pending.queue[-1][1])

    def test_duplicate_stamp_does_not_fabricate_independence(self):
        o = self.observer()
        self.send(o, 1_000_000_000)
        self.send(o, 2_000_000_000, False)
        self.send(o, 2_000_000_000, False)
        self.assertEqual(len(o.selected), 2)
        self.assertIn('Repeated/backward', o.errors[-1]['reason'])

    def test_queue_full_is_nonblocking_observer_only_failure(self):
        o = self.observer()
        self.send(o, 1_000_000_000)
        for i in range(4): o.pending.put_nowait((None, ('test',)))
        self.send(o, 2_000_000_000, False)
        self.assertEqual(o.pending.qsize(), 4)
        self.assertEqual(len(o.selected), 0)
        self.assertIn('Full', o.errors[-1]['reason'])

    def test_worker_later_top_level_fields_do_not_change_snapshot(self):
        o = self.observer()
        _, result = self.send(o, 1_000_000_000)
        result['parent_generation'] = 999
        result['worker_completed_wall_ns'] = 10
        self.assertNotIn('parent_generation', o.last_qualified['result_core'])
        self.assertNotIn('worker_completed_wall_ns', o.last_qualified['result_core'])

    def test_oversize_source_is_not_retained_or_truncated(self):
        o = self.observer()
        job, result = event(1_000_000_000)
        job['source_messages']['raw'].data = b'x' * (1024 * 1024 + 1)
        o.consider(job, result, None, 1)
        self.assertIsNone(o.last_qualified)
        self.assertIn('Source payload too large', o.errors[-1]['reason'])

    def test_init_failure_preserves_original_collector(self):
        with tempfile.TemporaryDirectory(prefix='R29_observer_test_') as tmp:
            with patch.dict(os.environ, GO2_BASELINE_TEMPORAL_CAPTURE='R29_bounded_baseline'), patch.object(M, 'TemporalFrames', side_effect=ValueError('test-only initialization fault')):
                original = BoundedSourceFrames(tmp)
            self.assertIsNone(original.temporal)
            self.assertEqual(original.maximum_bytes, 8 * 1024 * 1024)
            self.assertTrue((Path(tmp) / 'known_scene_temporal_observer_error.json').is_file())

    def test_close_failure_preserves_original_final_receipt(self):
        with tempfile.TemporaryDirectory(prefix='R29_observer_test_') as tmp:
            with patch.dict(os.environ, GO2_BASELINE_TEMPORAL_CAPTURE='', clear=False): original = BoundedSourceFrames(tmp)
            original.temporal = SimpleNamespace(close=lambda: (_ for _ in ()).throw(ValueError('test-only close fault')))
            receipt = original.close()
            self.assertTrue(receipt['final'])
            self.assertEqual(receipt['saved'], [])


class AsyncWriterTests(unittest.TestCase):
    def prepare(self, tmp):
        run = Path(tmp) / 'R29_V34_baseline_temporal_TEST_ONLY'
        run.mkdir()
        for name in ('known_scene_reference.json','known_scene_runtime_witness.json'):
            (run / name).write_text('{"fixture":"TEST_ONLY"}')
        return run

    def test_async_full_groups_drain_and_final_receipt(self):
        with tempfile.TemporaryDirectory(prefix='R29_async_test_') as tmp:
            run = self.prepare(tmp)
            with patch.object(serialization,'serialize_message',side_effect=pickle.dumps), patch.object(serialization,'deserialize_message',side_effect=lambda b, cls:pickle.loads(b)):
                o = M.TemporalFrames(run)
                job, result = event(1_000_000_000)
                o.consider(job,result,None,1)
                job, result = event(2_000_000_000,False)
                before = json.dumps(result,sort_keys=True)
                o.consider(job,result,None,1)
                self.assertTrue(o.close())
            self.assertEqual(before,json.dumps(result,sort_keys=True))
            self.assertEqual(len(o.saved),2)
            self.assertEqual(o.errors,[])
            receipt=json.loads((run/'known_scene_temporal_progress.json').read_text())
            self.assertTrue(receipt['final'])
            self.assertTrue(receipt['writer_queue_drained'])
            self.assertEqual(len(receipt['saved']),2)
            self.assertLess(max(o.usage().values()),20*1024*1024)
            for group in o.saved:
                p=Path(group['path']); meta=json.loads((p/'GROUP.json').read_text())
                for role, row in meta['files'].items():
                    raw=(p/row['path']).read_bytes()
                    self.assertEqual(len(raw),row['bytes'])
                    self.assertEqual(hashlib.sha256(raw).hexdigest(),row['sha256'])

    def test_oversize_serialization_is_missing_not_saved_coverage(self):
        with tempfile.TemporaryDirectory(prefix='R29_async_test_') as tmp:
            run=self.prepare(tmp)
            with patch.object(serialization,'serialize_message',return_value=b'x'*(1024*1024+1)),patch.object(serialization,'deserialize_message',side_effect=lambda b, cls:sent_messages[0]):
                o=M.TemporalFrames(run)
                job,result=event(1_000_000_000)
                # All fake messages equal; roundtrip succeeds, then whole-CDR
                # admission fails. This is a synthetic budget test, not a scan.
                sent_messages=[job['source_messages']['raw']]
                o.consider(job,result,None,1)
                job,result=event(2_000_000_000,False)
                sent_messages[0]=job['source_messages']['raw']
                o.consider(job,result,None,1)
                self.assertTrue(o.close())
            self.assertEqual(o.saved,[])
            self.assertTrue(o.errors)
            receipt=json.loads((run/'known_scene_temporal_progress.json').read_text())
            self.assertEqual(receipt['saved_role_coverage'],[])
            self.assertTrue(receipt['queued_role_coverage'])


class LauncherBudgetTests(unittest.TestCase):
    def function(self):
        p=Path(M.__file__).parent/'run.py'
        node=next(x for x in ast.parse(p.read_text()).body if isinstance(x,ast.FunctionDef) and x.name=='baseline_temporal_capture_limits')
        scope=dict(os=os,json=json,Path=Path,sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest())
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(p),'exec'),scope)
        return scope['baseline_temporal_capture_limits']

    def test_disabled_preserves_whole_plan(self):
        plan=dict(wall_budget_s=4590,profile=dict(original=True));before=json.dumps(plan,sort_keys=True)
        with patch.dict(os.environ,GO2_BASELINE_TEMPORAL_CAPTURE=''):
            self.function()(plan,Path('/unused'),Path(M.__file__).parent)
        self.assertEqual(before,json.dumps(plan,sort_keys=True))

    def test_enabled_only_caps_original_guardian_wall_plan(self):
        profile=dict(original=True,source_age_ns=3_000_000_000)
        plan=dict(wall_budget_s=4590,profile=profile)
        with patch.dict(os.environ,GO2_BASELINE_TEMPORAL_CAPTURE='R29_bounded_baseline'):
            self.function()(plan,Path('/tmp/R29_V34_baseline_temporal_TEST_ONLY'),Path(M.__file__).parent)
        self.assertEqual(plan['wall_budget_s'],238.)
        self.assertIs(plan['profile'],profile)
        self.assertFalse(plan['bounded_baseline_temporal_capture']['total_physical_stop_hard_bound_claimed'])

    def test_enabled_wrong_run_label_refuses_before_launch(self):
        with patch.dict(os.environ,GO2_BASELINE_TEMPORAL_CAPTURE='R29_bounded_baseline'):
            with self.assertRaises(ValueError):self.function()(dict(wall_budget_s=4590),Path('/tmp/wrong'),Path(M.__file__).parent)


if __name__ == '__main__':
    unittest.main()
