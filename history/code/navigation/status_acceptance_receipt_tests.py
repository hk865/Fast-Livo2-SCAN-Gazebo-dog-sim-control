"""Finite CPU contract checks; synthetic run dirs, no ROS/model/Actor run."""
import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from status_acceptance_receipt import (StatusAcceptanceReceipt, restore_acceptance,
    restore_status, canonical, FILENAME, MAX_BYTES)


HERE = Path(__file__).resolve().parent
BASE = Path('/__LOCAL_HOME__/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/engineering_v34_continuous_known_map')
R23 = BASE.parents[1] / 'runs/20261008_190114_closed_loop_cascade_clock_hold_R23_V34_serial0_request_substages_full46_8fc7'
ACTUAL = json.loads((R23 / 'navigation_status.json').read_text())
BEFORE = HERE / 'TEACHER_WRAPPER_BEFORE.py'
if not BEFORE.exists():
    BEFORE = R23 / 'sources/navigation/engineering_v34_continuous_known_map/teacher_wrapper.py'
if hashlib.sha256(BEFORE.read_bytes()).hexdigest() != '498f665a6c06a627a520e3e1c260d8ab663cb40d8a5d461e1835ee0f48a9161b':
    raise ValueError('Pinned actual RecordedPublisher preimage changed')
FIXTURE_BILL = []


def load_publisher(path):
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RecordedPublisher')
    ns = dict(json=json, copy=copy, String=lambda **kw: SimpleNamespace(**kw),
              time=SimpleNamespace(monotonic=lambda: 123.0))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), ns)
    return ns['RecordedPublisher']


class ReceiptTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='r24_receipt_cpu_')
        self.run = Path(self.tmp.name).resolve()
        self.status = copy.deepcopy(ACTUAL)
        self.receipt = self.status['execution_bridge_safety']['acceptance']
        # Synthetic scope identity for a CPU fixture, never a verified run.
        self.receipt['run_dir'] = str(self.run)
        self.receipt['path'] = str(self.run / 'navigation_scope.json')
        self.helper = StatusAcceptanceReceipt(self.run, self.receipt, enabled=True)

    def tearDown(self):
        files=[p.lstat() for p in self.run.iterdir()]
        FIXTURE_BILL.append(dict(test=self.id(),files_at_cleanup=len(files),
            logical_at_cleanup=sum(s.st_size for s in files),
            allocated_at_cleanup=sum(s.st_blocks*512 for s in files),
            synthetic_directory_removed=True))
        self.tmp.cleanup()

    def test_01_complete_save_hash_value_restore_and_original_untouched(self):
        before = canonical(self.status)
        projected = self.helper.project(self.status)
        self.assertNotEqual(projected, self.status)
        self.assertEqual(canonical(self.status), before)
        raw = (self.run / FILENAME).read_bytes()
        self.assertEqual(raw, canonical(self.receipt) + b'\n')
        ref = projected['execution_bridge_safety']['acceptance']
        self.assertEqual(ref['sha256'], self.receipt['sha256'])
        self.assertEqual(hashlib.sha256(raw).hexdigest(), ref['receipt_ref']['file_sha256'])
        self.assertEqual(hashlib.sha256(canonical(self.receipt)).hexdigest(), ref['receipt_ref']['receipt_sha256'])
        self.assertEqual(canonical(restore_status(projected, self.run)), before)
        st = (self.run / FILENAME).stat()
        self.assertLessEqual(st.st_size, MAX_BYTES)
        self.assertLessEqual(st.st_blocks * 512, MAX_BYTES)
        self.assertEqual(st.st_mode & 0o777, 0o600)

    def test_02_disabled_is_exact_noop_and_no_file(self):
        off = StatusAcceptanceReceipt(self.run, self.receipt, enabled=False)
        self.assertIs(off.project(self.status), self.status)
        self.assertFalse((self.run / FILENAME).exists())

    def test_03_entire_receipt_mismatch_never_saved(self):
        self.status['execution_bridge_safety']['acceptance']['profile']['cascade']['gains']['cross_kp'] += .001
        self.assertIs(self.helper.project(self.status), self.status)
        self.assertFalse((self.run / FILENAME).exists())

    def test_04_bool_int_mismatch_falls_back(self):
        self.status['execution_bridge_safety']['acceptance']['experimental'] = 1
        self.assertIs(self.helper.project(self.status), self.status)
        self.assertFalse((self.run / FILENAME).exists())

    def test_05_missing_receipt_never_saved(self):
        del self.status['execution_bridge_safety']['acceptance']
        self.assertIs(self.helper.project(self.status), self.status)
        self.assertFalse((self.run / FILENAME).exists())

    def test_06_exclusive_collision_retains_inline_without_overwrite(self):
        path = self.run / FILENAME
        path.write_bytes(b'owned-by-someone-else')
        self.assertIs(self.helper.project(self.status), self.status)
        self.assertEqual(path.read_bytes(), b'owned-by-someone-else')
        self.assertFalse(self.helper.saved)

    def test_07_failed_save_retains_inline_without_false_reference(self):
        with mock.patch('status_acceptance_receipt.os.open', side_effect=PermissionError('fixture denied')):
            self.assertIs(self.helper.project(self.status), self.status)
        self.assertFalse((self.run / FILENAME).exists())
        self.assertFalse(self.helper.saved)

    def test_08_readback_failure_retains_inline(self):
        with mock.patch('status_acceptance_receipt._read_bounded', return_value=b'bad incomplete receipt'):
            self.assertIs(self.helper.project(self.status), self.status)
        self.assertTrue((self.run / FILENAME).exists())
        self.assertFalse(self.helper.saved)

    def test_09_saved_tamper_and_delete_reject_ref_and_fall_back(self):
        projected = self.helper.project(self.status)
        path = self.run / FILENAME
        path.write_bytes(b'{}\n')
        with self.assertRaises(ValueError): restore_status(projected, self.run)
        self.assertIs(self.helper.project(self.status), self.status)
        path.unlink()
        with self.assertRaises(OSError): restore_status(projected, self.run)
        self.assertIs(self.helper.project(self.status), self.status)

    def test_10_reference_path_scope_hash_and_symlink_rejected(self):
        projected = self.helper.project(self.status)
        ref = projected['execution_bridge_safety']['acceptance']
        for key, value in [('file', '../outside.json'), ('file_sha256', '0'*64), ('receipt_sha256', '0'*64), ('bytes', True)]:
            bad = copy.deepcopy(ref);bad['receipt_ref'][key] = value
            with self.assertRaises(ValueError): restore_acceptance(bad, self.run)
        bad = copy.deepcopy(ref);bad['sha256'] = '1'*64
        with self.assertRaises(ValueError): restore_acceptance(bad, self.run)
        path = self.run / FILENAME; other = self.run / 'another.json';path.rename(other);path.symlink_to(other)
        with self.assertRaises(OSError): restore_acceptance(ref, self.run)
        self.assertIs(self.helper.project(self.status), self.status)

    def test_11_large_receipt_and_wrong_run_keep_inline(self):
        huge = copy.deepcopy(self.receipt);huge['fixture_padding'] = 'x' * MAX_BYTES
        helper = StatusAcceptanceReceipt(self.run, huge, enabled=True)
        status = copy.deepcopy(self.status);status['execution_bridge_safety']['acceptance'] = huge
        self.assertIs(helper.project(status), status)
        wrong = copy.deepcopy(self.receipt);wrong['run_dir'] = '/tmp/other-fixture-run'
        self.assertIs(StatusAcceptanceReceipt(self.run, wrong, enabled=True).project(self.status), self.status)
        self.assertFalse((self.run / FILENAME).exists())

    def test_12_actual_publisher_publication_recording_all_dynamic_values_unchanged(self):
        def exercise(path, helper):
            published=[];appended=[];atomic=[]
            bridge_before = copy.deepcopy(self.status['execution_bridge_safety'])
            node = SimpleNamespace(profile=self.receipt['profile'],status_acceptance_receipt=helper,
                bridge_safety=bridge_before,
                trajectory_association=SimpleNamespace(reference_stamp=None),
                heading_gate=SimpleNamespace(evidence=lambda *args: copy.deepcopy(self.status['teacher_transition']),max_turn_rate=.6),
                get_clock=lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)),
                evidence=SimpleNamespace(append=lambda p,d:appended.append(d),atomic=lambda p,d:atomic.append(d)))
            load_publisher(path)(SimpleNamespace(publish=lambda m:published.append(json.loads(m.data))),self.run,node).publish(
                SimpleNamespace(data=json.dumps(self.status)))
            self.assertEqual(canonical(node.bridge_safety), canonical(bridge_before))
            return published,appended,atomic
        before = exercise(BEFORE, self.helper)
        after = exercise(HERE / 'teacher_wrapper.py', self.helper)
        for originals, projected in zip(before, after):
            self.assertEqual(len(originals), len(projected))
            for a,b in zip(originals, projected):
                self.assertEqual(canonical(a), canonical(restore_status(b,self.run)))
        off = exercise(HERE / 'teacher_wrapper.py', StatusAcceptanceReceipt(self.run,self.receipt,enabled=False))
        self.assertEqual(canonical(before), canonical(off))
        tree=ast.parse((HERE/'teacher_wrapper.py').read_text())
        init=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='TeacherNavigation').body[0]
        assignment=next(n for n in init.body if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Attribute) and t.attr=='status_acceptance_receipt' for t in n.targets))
        expression=next(k.value for k in assignment.value.keywords if k.arg=='enabled')
        for profile,expected in [({'mission46_required':False,'continuous_route_contract':{'enabled':True}},False),
                ({'mission46_required':True,'continuous_route_contract':{'enabled':False}},False),
                ({'mission46_required':True,'continuous_route_contract':{'enabled':True}},True)]:
            enabled=eval(compile(ast.Expression(expression),'candidate-enable','eval'),{'self':SimpleNamespace(profile=profile)})
            self.assertEqual(enabled,expected)

    def test_13_original_compact_codecs_and_nested_mission_restore(self):
        tree=ast.parse((BASE/'compact_archives.py').read_text())
        selected=[n for n in tree.body if (isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in ('canonical','digest','Codec','decode'))
            or (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('FORMAT','STATIC_PATHS') for t in n.targets))]
        ns=dict(json=json,copy=copy,hashlib=hashlib)
        exec(compile(ast.Module(body=selected,type_ignores=[]),'original-compact-cpu','exec'),ns)
        projected=self.helper.project(self.status)
        for name,row in [('navigation_status.jsonl',projected),('mission46_status_history.jsonl',{'navigation':projected,'stage':'fixture-only','current_goals':self.status.get('goals_definitions')})]:
            codec=ns['Codec'](name,{'fixture':'not-an-actual-run'})
            text=codec.encode(row)+codec.footer()
            decoded=list(ns['decode'](text.splitlines(True),expected_sources={'fixture':'not-an-actual-run'},expected_original_filename=name))
            self.assertEqual(canonical(decoded[0]),canonical(row))
            recovered=restore_status(decoded[0] if 'navigation' not in row else decoded[0]['navigation'],self.run)
            self.assertEqual(canonical(recovered),canonical(self.status))
        # The pinned actual original verify_scope return expression reconstructs
        # exactly the archived bridge receipt; no live scope checks are skipped.
        scope_raw=(R23/'navigation_scope.json').read_bytes();scope=json.loads(scope_raw)
        f=next(n for n in ast.parse((BASE/'pid_scope.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='verify_scope')
        returned=next(n for n in reversed(f.body) if isinstance(n,ast.Return)).value
        checkpoint=ACTUAL['execution_bridge_safety']['acceptance']['checkpoint_sha256']
        result=eval(compile(ast.Expression(returned),'original-return-expression','eval'),dict(
            path=R23/'navigation_scope.json',digest=hashlib.sha256(scope_raw).hexdigest(),run=R23,p=scope['profile'],FROZEN_SHA=checkpoint))
        self.assertEqual(canonical(result),canonical(ACTUAL['execution_bridge_safety']['acceptance']))


if __name__ == '__main__':
    result=unittest.main(verbosity=2,exit=False).result
    print('FIXTURE_BILL_JSON='+json.dumps(dict(tests=result.testsRun,rows=FIXTURE_BILL,
        no_old_run_writes=True,synthetic_temp_directories_only=True,
        maximum_temporary_logical_bound_per_test=MAX_BYTES*2+4096,
        maximum_temporary_allocated_bound_per_test=MAX_BYTES*2+4096)))
    raise SystemExit(0 if result.wasSuccessful() else 1)
