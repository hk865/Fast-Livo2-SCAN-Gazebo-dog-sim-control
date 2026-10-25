"""Finite offline integration checks; no ROS initialization or physical launch.

Exercises observer failures around the unchanged geometry call and actual
NativeGuardAudit/control_core geometry. No input cloud is archived by the probe.
"""
from contextlib import contextmanager
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

HERE = Path(__file__).resolve().parent
DEMO = HERE.parents[3]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Scope:
    def __init__(self, fault=None, suppress=False):
        self.fault = fault
        self.suppress = suppress
        self.entered = self.exited = 0
        self.exception = None
    def __enter__(self):
        self.entered += 1
        if self.fault == 'enter':
            raise OSError('injected observer enter')
    def __exit__(self, kind, error, traceback):
        self.exited += 1
        self.exception = error
        if self.fault == 'exit':
            raise OSError('injected observer exit')
        return self.suppress


class RuntimeTraceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='r89-trace-tests-')
        self.addCleanup(self.tmp.cleanup)
        self.zone = Scope()
        self.span = Scope()
        self.threads = NS(start=Mock(return_value=dict(enabled=True, running=True)),
            stop=Mock(return_value=dict(enabled=True, running=False, stop_complete=True)),
            zone=Mock(return_value=self.zone))
        self.timeline = NS(span=Mock(return_value=self.span))
        with patch.dict(sys.modules, r89_thread_probe=self.threads,
                        r87_diagnostic_probe=self.timeline):
            self.trace = load('_r89_runtime_trace_test', HERE/'r89_runtime_trace.py')
        self.result = object()
        self.original = Mock(return_value=self.result)
        self.node = NS(native_guard_audit=NS(native_obstacle=self.original, calls=[]),
                       pose_stamp=123456, run=Path(self.tmp.name))
        self.inputs = (np.zeros((3, 3)), np.zeros(3), np.ones(3), np.zeros((2, 3)))
        self.keyword = object()

    def install(self):
        self.trace.install(self.node)
        return self.node.native_guard_audit.native_obstacle

    def invoke(self, function):
        return function(*self.inputs, custom_option=self.keyword)

    def assert_call_identity(self):
        self.original.assert_called_once()
        for actual, expected in zip(self.original.call_args.args, self.inputs):
            self.assertIs(actual, expected)
        self.assertIs(self.original.call_args.kwargs['custom_option'], self.keyword)

    def test_disabled_sampler_does_not_access_audit_or_write(self):
        self.threads.start.return_value = dict(enabled=False, running=False)
        self.threads.stop.return_value = dict(enabled=False)
        receipt = self.trace.install(NS())
        self.assertFalse(receipt['enabled'])
        self.trace.finish(NS())
        self.threads.stop.assert_called_once()
        self.assertFalse(any(self.trace._ERRORS.values()))

    def test_failed_sampler_start_does_not_replace_geometry(self):
        self.threads.start.return_value = dict(enabled=True, running=False, started=True)
        self.trace.install(self.node)
        self.assertIs(self.node.native_guard_audit.native_obstacle, self.original)
        self.trace.finish(self.node)
        self.threads.stop.assert_called_once()

    def test_raised_start_error_attempts_cleanup_without_control_exception(self):
        self.threads.start.side_effect = RuntimeError('injected startup failure')
        result = self.trace.install(self.node)
        self.assertEqual(result['start_error'], 'RuntimeError')
        self.threads.stop.assert_called_once()
        self.assertIs(self.node.native_guard_audit.native_obstacle, self.original)

    def test_invalid_start_receipt_and_missing_audit_stop_sampler(self):
        for receipt, node in ((None, self.node), (dict(running=True), NS())):
            with self.subTest(receipt=receipt):
                self.threads.start.return_value = receipt
                result = self.trace.install(node)
                self.assertIn('installation_error', result)
                self.assertEqual(self.trace._binding, None)
        self.assertEqual(self.threads.stop.call_count, 2)

    def test_geometry_invoked_once_with_identical_inputs_keywords_and_return(self):
        observed = self.install()
        result = self.invoke(observed)
        self.assertIs(result, self.result)
        self.assert_call_identity()
        self.assertEqual(self.zone.entered, 1)
        self.assertEqual(self.zone.exited, 1)
        self.assertEqual(self.span.entered, 1)
        self.assertEqual(self.span.exited, 1)
        self.assertEqual(self.timeline.span.call_args.kwargs,
            dict(source_ns=123456, corridor_index=0, cloud_points=3, route_points=2))

    def test_original_exception_object_and_baseexception_cannot_be_suppressed(self):
        for error in (ValueError('geometry rejection'), KeyboardInterrupt('original interruption')):
            with self.subTest(error=type(error).__name__):
                self.zone.suppress = self.span.suppress = True
                self.original.reset_mock()
                self.original.side_effect = error
                observed = self.install()
                try:
                    self.invoke(observed)
                except BaseException as actual:
                    self.assertIs(actual, error)
                else:
                    self.fail('observer suppressed the original geometry error')
                self.assert_call_identity()
                self.assertIs(self.zone.exception, error)
                self.assertIs(self.span.exception, error)

    def test_observer_factory_enter_exit_failures_preserve_success_and_exception(self):
        for kind in ('zone', 'timeline'):
            for fault in ('factory', 'enter', 'exit'):
                for reject in (False, True):
                    with self.subTest(kind=kind, fault=fault, reject=reject):
                        self.original.reset_mock()
                        expected = ValueError('original geometry failure') if reject else None
                        self.original.side_effect = expected
                        scope = Scope(fault=fault)
                        factory = Mock(return_value=scope)
                        if fault == 'factory':
                            factory.side_effect = OSError('injected factory failure')
                        self.threads.zone = factory if kind == 'zone' else Mock(return_value=Scope())
                        self.timeline.span = factory if kind == 'timeline' else Mock(return_value=Scope())
                        observed = self.install()
                        if reject:
                            try:
                                self.invoke(observed)
                            except ValueError as actual:
                                self.assertIs(actual, expected)
                            else:
                                self.fail('lost original failure')
                        else:
                            self.assertIs(self.invoke(observed), self.result)
                        self.assert_call_identity()
                        key = kind + ('_exit' if fault == 'exit' else '_enter')
                        self.assertGreater(self.trace._ERRORS[key], 0)

    def test_metadata_failure_does_not_change_geometry_call(self):
        class BadLength:
            def __len__(self):
                raise RuntimeError('metadata size failure')
        self.inputs = (BadLength(), self.inputs[1], self.inputs[2], self.inputs[3])
        self.assertIs(self.invoke(self.install()), self.result)
        self.assert_call_identity()
        self.assertEqual(self.trace._ERRORS['metadata'], 1)

    def test_duplicate_install_does_not_nest_or_restart(self):
        first = self.install()
        result = self.trace.install(self.node)
        self.assertTrue(result['already_installed'])
        self.assertIs(first, self.node.native_guard_audit.native_obstacle)
        self.threads.start.assert_called_once()
        self.invoke(first)
        self.assert_call_identity()
        self.assertEqual(self.zone.entered, 1)

    def test_stop_unconditionally_cleans_and_restores_original_after_switch_change(self):
        self.install()
        with patch.dict(os.environ, R89_THREAD_PROBE='0'):
            result = self.trace.finish(self.node)
        self.assertTrue(result['stop_complete'])
        self.assertTrue(result['corridor_wrapper_restored'])
        self.assertIs(self.node.native_guard_audit.native_obstacle, self.original)
        self.threads.stop.assert_called_once()
        saved = json.loads((self.node.run/'r89_thread_timeline.json').read_text())
        self.assertFalse(any(saved['integration_errors'].values()))

    def test_stop_error_does_not_skip_restore_or_hide_cleanup_status(self):
        self.install()
        self.threads.stop.side_effect = RuntimeError('injected stop failure')
        result = self.trace.finish(self.node)
        self.assertFalse(result['stop_complete'])
        self.assertEqual(result['stop_error'], 'RuntimeError')
        self.assertIs(self.node.native_guard_audit.native_obstacle, self.original)
        self.assertEqual(result['integration_errors']['stop'], 1)

    def test_stop_timeout_evidence_is_preserved(self):
        self.install()
        self.threads.stop.return_value = dict(enabled=True, running=True, stop_complete=False)
        result = self.trace.finish(self.node)
        self.assertTrue(result['running'])
        self.assertFalse(result['stop_complete'])

    def test_serialization_write_and_stderr_faults_do_not_skip_stop(self):
        for snapshot in (dict(enabled=True, bad=float('nan')), dict(enabled=True), None):
            with self.subTest(snapshot=snapshot):
                self.threads.stop.return_value = snapshot
                with patch.object(Path, 'write_text', side_effect=OSError('injected disk fault')),\
                     patch.object(sys, 'stderr', NS(write=Mock(side_effect=OSError('stderr closed')))):
                    self.trace.finish(self.node)
        self.assertEqual(self.threads.stop.call_count, 3)
        self.assertEqual(self.trace._ERRORS['write'], 3)
        self.assertEqual(self.trace._ERRORS['stderr'], 3)

    def test_finish_does_not_overwrite_another_geometry_replacement(self):
        self.install()
        changed = object()
        self.node.native_guard_audit.native_obstacle = changed
        result = self.trace.finish(self.node)
        self.assertFalse(result['corridor_wrapper_restored'])
        self.assertIs(self.node.native_guard_audit.native_obstacle, changed)

    def test_real_native_guard_two_calls_and_results_are_unchanged(self):
        core = load('_r89_unchanged_core', DEMO/'navigation/control_core.py')
        guard = load('_r89_unchanged_audit', DEMO/'teacher_mode/navigation/guard_audit.py')
        pose = np.array([0., 0., .35])
        target = np.array([1., 0., .35])
        direction = np.array([1., .2])
        route = np.array([[0., 0., .35], [1., 0., .35], [2., 0., .5]])
        cases = (None, np.empty((0, 3)), np.array([[.4, 0., .35], [.5, 0., .4], [.6, 0., .45]]),
                 np.array([[.4, 0., -.2], [.5, 0., -.3], [.6, 0., -.4]]))
        for cloud in cases:
            with self.subTest(cloud=None if cloud is None else cloud.shape):
                baseline = guard.NativeGuardAudit(core.steering_obstacle_ahead)
                expected = baseline(cloud, pose, target, direction, route)
                actual_audit = guard.NativeGuardAudit(core.steering_obstacle_ahead)
                original = actual_audit.native_obstacle
                calls = []
                def checked(c, p, t, r, **kwargs):
                    self.assertIs(c, cloud)
                    self.assertIs(p, pose)
                    self.assertIs(r, route)
                    result = original(c, p, t, r, **kwargs)
                    calls.append(result)
                    return result
                actual_audit.native_obstacle = checked
                self.node.native_guard_audit = actual_audit
                self.trace.install(self.node)
                actual = actual_audit(cloud, pose, target, direction, route)
                self.assertEqual(actual, expected)
                self.assertEqual(len(calls), 2)
                self.assertEqual(len(actual_audit.calls), 2)
                for index in range(2):
                    self.assertIs(actual_audit.calls[index][1], calls[index])
                    self.assertEqual(actual_audit.calls[index][1], baseline.calls[index][1])
                    np.testing.assert_array_equal(actual_audit.calls[index][0], baseline.calls[index][0])
                self.trace.finish(self.node)

    def test_actual_probe_switch_matrix_stops_own_sampler_and_keeps_geometry(self):
        for r87 in ('0', '1'):
            for r89 in ('0', '1', 'invalid'):
                with self.subTest(r87=r87, r89=r89), patch.dict(os.environ,
                        R87_DIAGNOSTIC_PROBE=r87, R89_THREAD_PROBE=r89):
                    sampler = load('_r89_actual_sampler', HERE/'r89_thread_probe.py')
                    with patch.dict(sys.modules, r89_thread_probe=sampler):
                        diagnostic = load('_r89_actual_timeline', HERE/'r87_diagnostic_probe.py')
                        with patch.dict(sys.modules, r87_diagnostic_probe=diagnostic):
                            trace = load('_r89_actual_trace', HERE/'r89_runtime_trace.py')
                    original = Mock(return_value=self.result)
                    node = NS(native_guard_audit=NS(native_obstacle=original, calls=[]),
                              pose_stamp=123, run=Path(self.tmp.name))
                    try:
                        receipt = trace.install(node)
                        self.assertIs(self.invoke(node.native_guard_audit.native_obstacle), self.result)
                        original.assert_called_once()
                        if r89 == '1':
                            self.assertTrue(receipt['running'])
                            self.assertIsNot(node.native_guard_audit.native_obstacle, original)
                        else:
                            self.assertIs(node.native_guard_audit.native_obstacle, original)
                    finally:
                        snapshot = trace.finish(node)
                    self.assertIs(node.native_guard_audit.native_obstacle, original)
                    if r89 == '1':
                        self.assertTrue(snapshot['stop_complete'])
                        self.assertFalse(sampler.PROBE.thread.is_alive())
                        self.assertFalse(any(snapshot['integration_errors'].values()))


if __name__ == '__main__':
    unittest.main()
