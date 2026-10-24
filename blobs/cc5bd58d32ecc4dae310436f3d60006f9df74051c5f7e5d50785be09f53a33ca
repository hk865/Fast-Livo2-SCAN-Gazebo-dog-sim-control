"""Finite production-branch tests; no ROS, Teacher inference or simulation."""
import ast
import copy
import gzip
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

HERE = Path(__file__).resolve().parent
OLD = HERE.parent / 'corridor_tracking_v23_curvature_full46'
DEMO = HERE.parents[2]
sys.path.insert(0, str(DEMO / 'navigation'))
from control_core import trajectory_has_progress
from replan_policy import low_command_is_exhausted_path
from pure_tests import controller, sources


def branch(path):
    tree = ast.parse(path.read_text())
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
                and 'now - self.last_reference > 3.0' in ast.unparse(n.test)
                and 'np.linalg.norm(velocity)' in ast.unparse(n.test))
    wrapper = ast.parse('def replay(self,velocity,yaw_rate,now,goal):\n    pass').body[0]
    wrapper.body = [copy.deepcopy(node)]
    namespace = dict(np=np, trajectory_has_progress=trajectory_has_progress,
        low_command_is_exhausted_path=low_command_is_exhausted_path)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[])),
                 'actual_production_low_command_branch', 'exec'), namespace)
    return namespace['replay']


def fake(mode='recovering', exhausted=False, path=None, position=(0.,0.,.32)):
    node = SimpleNamespace(alignment_hold=False, last_reference=0., samples=np.asarray(
        path if path is not None else [[0.,0.,.32],[4.,0.,.32]]),
        pose=np.asarray(position), arrival_radius=.22, cascade_last_row=dict(mode=mode),
        steering=dict(exhausted=exhausted), published=0, requested=0)
    node.publish_command = lambda: setattr(node, 'published', node.published + 1)
    node.request_plan = lambda: setattr(node, 'requested', node.requested + 1)
    return node


class RecoveryReplanChecks(unittest.TestCase):
    def setUp(self):
        self.old = branch(OLD / 'shared_controller.py')
        self.new = branch(HERE / 'shared_controller.py')

    def test_old_actual_recovery_zero_trigger_reproduced_new_path_preserved(self):
        compact_path = HERE.parents[1] / 'test_results/curvature_full46_20261006/57fa_diagnosis/NINTH_TENTH_PID_COMPACT.json.gz'
        compact = json.load(gzip.open(compact_path, 'rt'))
        rows = {r['pid_sequence']: r for r in compact['rows']}
        run = Path(compact['run']); goal = np.asarray(json.loads((run / 'navigation_request.json').read_text())['goals'][9]['center'])
        for sequence in (5006, 5284, 5658, 7392):
            with self.subTest(sequence=sequence):
                row = rows[sequence]
                self.assertEqual(row['phase'], 'drive'); self.assertEqual(row['mode'], 'recovering')
                np.testing.assert_array_equal(row['command_after_slew'], [0.,0.,0.])
                with np.load(run / row['source_array_file'], allow_pickle=False) as archive:
                    path = archive['samples']
                old = fake(path=path, position=row['pose']); new = fake(path=path, position=row['pose'])
                preserved = new.samples
                self.old(old, np.zeros(2), 0., 4., goal)
                self.new(new, np.zeros(2), 0., 4., goal)
                self.assertIsNone(old.samples); self.assertEqual(old.requested, 1)
                self.assertIs(new.samples, preserved); self.assertEqual(new.requested, 0)
                self.assertEqual(new.published, 0)

    def test_all_protected_zero_modes_do_not_clear_even_aged_path(self):
        for mode in ('protect', 'recovering', 'pre_turn', 'settle', 'reference_constraint_hold'):
            for exhausted in (False, True):
                with self.subTest(mode=mode, exhausted=exhausted):
                    node = fake(mode, exhausted); original = node.samples
                    self.new(node, np.zeros(2), 0., 40., np.array([4.,0.,.32]))
                    self.assertIs(node.samples, original); self.assertEqual(node.requested, 0)

    def test_cascade_recovery_zero_then_second_fresh_drive_remains_original(self):
        c = controller()
        c.update(*sources(100_000_000), mode_override='hold')
        first, row = c.update(*sources(200_000_000), mode_override='drive')
        self.assertEqual(row['mode'], 'recovering'); np.testing.assert_array_equal(first, [0.,0.,0.])
        node = fake(row['mode']); original = node.samples
        self.new(node, first[:2], first[2], 4., np.array([4.,0.,.32]))
        self.assertIs(node.samples, original); self.assertEqual(node.requested, 0)
        second, row = c.update(*sources(300_000_000), mode_override='drive')
        self.assertEqual(row['mode'], 'drive'); self.assertGreater(second[0], .015)
        self.assertTrue(row['controller_updated'])

    def test_genuine_exhausted_path_and_geometrically_no_progress_replan(self):
        for mode in ('drive', 'path_end_hold'):
            for exhausted, path in ((True, None), (False, [[0.,0.,.32],[0.,0.,.32]])):
                with self.subTest(mode=mode, exhausted=exhausted):
                    node = fake(mode, exhausted, path)
                    self.new(node, np.zeros(2), 0., 4., np.array([4.,0.,.32]))
                    self.assertIsNone(node.samples); self.assertEqual(node.requested, 1)
                    self.assertEqual(node.published, 1)

    def test_zero_command_alone_is_not_geometric_progress_loss(self):
        node = fake('drive'); original = node.samples
        self.new(node, np.zeros(2), 0., 40., np.array([4.,0.,.32]))
        self.assertIs(node.samples, original); self.assertEqual(node.requested, 0)

    def test_actual_cascade_path_end_hold_preserves_original_replan_boundary(self):
        c = controller(goal=(4.,0.,.32))
        c.set_path([[0.,0.,.32],[3.,0.,.32]],'local-horizon',0,0)
        c.update(*sources(100_000_000,position=(2.96,0.,.32)),mode_override='drive')
        command,row = c.update(*sources(200_000_000,position=(2.96,0.,.32)),mode_override='drive')
        self.assertEqual(row['mode'],'path_end_hold')
        self.assertAlmostEqual(row['remaining_horizontal_arc_m'],.04)
        np.testing.assert_array_equal(command[:2],[0.,0.])
        node=fake(row['mode'],False,[[0.,0.,.32],[3.,0.,.32]],(2.96,0.,.32))
        self.new(node,command[:2],command[2],4.,np.array([4.,0.,.32]))
        self.assertIsNone(node.samples);self.assertEqual(node.requested,1)

    def test_alignment_age_and_command_thresholds_still_required(self):
        for align, age, velocity, yaw in ((True,4.,[0.,0.],0.),(False,3.,[0.,0.],0.),
                (False,4.,[.015,0.],0.),(False,4.,[0.,0.],.05)):
            node = fake('drive', True); node.alignment_hold = align; original = node.samples
            self.new(node, np.asarray(velocity), yaw, age, np.array([4.,0.,.32]))
            self.assertIs(node.samples, original); self.assertEqual(node.requested, 0)

    def test_stale_source_still_zero_and_cannot_use_replan_fix_to_move(self):
        c = controller(); c.update(*sources(100_000_000)); c.update(*sources(200_000_000))
        args = list(sources(300_000_000)); args[-1] += 1_000_000_000
        command, row = c.update(*args, mode_override='drive')
        self.assertEqual(row['mode'], 'protect'); np.testing.assert_array_equal(command, [0.,0.,0.])
        node = fake(row['mode']); original = node.samples
        self.new(node, command[:2], command[2], 4., np.array([4.,0.,.32]))
        self.assertIs(node.samples, original); self.assertEqual(node.requested, 0)

    def test_invalid_progress_metadata_rejected(self):
        for value in (None, 0, np.bool_(False)):
            with self.assertRaises(ValueError):
                low_command_is_exhausted_path(cascade_mode='drive', exhausted=value, has_geometric_progress=True)

    def test_changed_shared_production_statements_only_new_predicate(self):
        old = ast.parse((OLD/'shared_controller.py').read_text())
        new = ast.parse((HERE/'shared_controller.py').read_text())
        new.body = [n for n in new.body if not (isinstance(n, ast.ImportFrom) and n.module == 'replan_policy')]
        a = next(n for n in ast.walk(old) if isinstance(n,ast.If) and 'np.linalg.norm(velocity)' in ast.unparse(n.test))
        b = next(n for n in ast.walk(new) if isinstance(n,ast.If) and 'low_command_is_exhausted_path' in ast.unparse(n.test))
        self.assertIsInstance(b.test, ast.BoolOp)
        self.assertEqual(ast.dump(ast.BoolOp(op=ast.And(), values=b.test.values[:-1])), ast.dump(a.test))
        self.assertEqual(ast.dump(a.body[0]), ast.dump(b.body[0]))
        b.test = copy.deepcopy(a.test)
        self.assertEqual(ast.dump(new), ast.dump(old))

    def test_exhausted_path_deadline_constraint_replan_and_guards_unchanged(self):
        old = ast.parse((OLD/'shared_controller.py').read_text()); new = ast.parse((HERE/'shared_controller.py').read_text())
        def body(tree, token):
            return ast.dump(next(n for n in ast.walk(tree) if isinstance(n, ast.If) and token in ast.unparse(n.test)))
        for token in ("self.steering['exhausted']", 'timeout_sim_s', 'stale', 'self.obstacle_hold'):
            self.assertEqual(body(old,token), body(new,token))
        self.assertEqual((OLD/'controller.py').read_bytes(), (HERE/'controller.py').read_bytes())
        for name in ('cascade_core.py','spatial_reference.py','transition_gate.py','clock_hold.py',
                'teacher_wrapper.py','bridge.py','worker.py','route_fence.py','mission46_guard.py'):
            self.assertEqual((OLD/name).read_bytes(), (HERE/name).read_bytes())

    def test_new_gate_cannot_accept_parent_gate_or_missing_helper_binding(self):
        import corridor_preflight
        profile = json.loads((HERE/'profiles/curvature_original46_on.json').read_text())
        parent = json.loads((OLD/'CURVATURE_V23_PREFLIGHT.json').read_text())
        gate = HERE/'RECOVERY_REPLAN_V24_PREFLIGHT.json'
        original_read = Path.read_text
        original_is_file = Path.is_file
        def read(path, *args, **kwargs):
            return json.dumps(document) if path == gate else original_read(path,*args,**kwargs)
        def is_file(path):
            return True if path == gate else original_is_file(path)
        document = copy.deepcopy(parent)
        with patch.object(Path,'read_text',read), patch.object(Path,'is_file',is_file):
            with self.assertRaisesRegex(RuntimeError,'incomplete|inherited'):
                corridor_preflight.verify_preflight(HERE,profile)
        document.update(schema='recovery_replan_v24_full46_preflight/v1', candidate_root=str(HERE),
                        checks={k:True for k in corridor_preflight.REQUIRED})
        # The old binding map omits new production helper; the reader must fail
        # at mandatory dependency validation before accepting any old receipt.
        with patch.object(Path,'read_text',read), patch.object(Path,'is_file',is_file):
            with self.assertRaisesRegex(RuntimeError,'mandatory.*incomplete'):
                corridor_preflight.verify_preflight(HERE,profile)


if __name__ == '__main__':
    unittest.main(verbosity=2)
