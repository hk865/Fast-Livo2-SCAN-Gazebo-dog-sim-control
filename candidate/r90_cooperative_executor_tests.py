"""Finite offline dispatch/cleanup regressions; no ROS or Gazebo startup.

The fake executor isolates sole-spinner ordering and original exception identity.
It does not establish DDS throughput, callback freshness or physical safety;
those require the separate real-executor comparison, especially at 1000 Hz IMU.
"""
import copy
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import r90_cooperative_executor as experiment


def profile():
    return dict(r74_cooperative_executor=dict(enabled=True, schema='r74_spin_once_zero_yield/v1'),
        r74_teacher_substage_timing=dict(enabled=True, schema='r74_bounded_teacher_substage_timing/v1'),
        r73_minimal_diagnostic_baseline=dict(enabled=True, simulation_only=True),
        diagnostic_observe_only=dict(enabled=True), r85_formal_regression=dict(enabled=False),
        navigation_ground_truth_used=False, actor_kind='student')


class Clock:
    def __init__(self):
        self.wall = 100_000_000
        self.cpu = 10_000_000
        self.events = []
        self.sleep_error = None
        self.sleep_hook = None
    def monotonic_ns(self):
        return self.wall
    def thread_time_ns(self):
        return self.cpu
    def advance(self, wall, cpu=0):
        self.wall += wall
        self.cpu += cpu
    def sleep(self, seconds):
        self.events.append(('yield', seconds))
        if self.sleep_error is not None:
            raise self.sleep_error
        self.advance(round(seconds * 1e9))
        if self.sleep_hook is not None:
            self.sleep_hook()


class Base:
    def __init__(self, *args, **kwargs):
        self.constructor_args = args
        self.constructor_kwargs = kwargs
        self.context_ok = True
        self.context = NS(ok=lambda: self.context_ok)
        self.calls = []
        self.on_spin = None
        self.shutdown_calls = []
        self.shutdown_result = object()
        self.shutdown_error = None
    def spin_once(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if self.on_spin is not None:
            self.on_spin()
    def shutdown(self, *args, **kwargs):
        self.shutdown_calls.append((args, kwargs, self._r74_cooperative_stop.is_set()))
        if self.shutdown_error is not None:
            raise self.shutdown_error
        return self.shutdown_result


class PositiveYieldTests(unittest.TestCase):
    def setUp(self):
        self.environment = patch.dict(os.environ,
            R90_SPINNER_POSITIVE_YIELD='1', GO2_R73_MINIMAL_SIMULATION='1')
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.clock = Clock()
        self.time_patch = patch.object(experiment, 'time', self.clock)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)

    def make(self, **kwargs):
        return experiment.make_executor(Base, profile(), **kwargs)

    def finite_dispatch(self, executor, count, *, wall_ns=10_000_000, cpu_ns=3_000_000):
        def dispatch():
            self.clock.events.append(('dispatch', len(executor.calls)))
            self.clock.advance(wall_ns, cpu_ns)
            if len(executor.calls) >= count:
                executor.context_ok = False
        executor.on_spin = dispatch

    def test_absent_and_zero_switch_delegate_exact_original_factory(self):
        selected_profile = profile()
        expected = object()
        for option in (None, '0'):
            with self.subTest(option=option), patch.object(experiment.original, 'make_executor',
                    return_value=expected) as delegate:
                os.environ.pop(experiment.KEY, None)
                if option is not None:
                    os.environ[experiment.KEY] = option
                self.assertIs(experiment.make_executor(Base, selected_profile, num_threads=2), expected)
                delegate.assert_called_once_with(Base, selected_profile, num_threads=2)

    def test_off_uses_actual_original_zero_yield_class(self):
        os.environ[experiment.KEY] = '0'
        with patch.object(experiment.original, 'time', self.clock):
            executor = self.make()
            self.finite_dispatch(executor, 2)
            executor.spin()
        self.assertFalse(experiment.receipt(executor)['enabled'])
        self.assertFalse(hasattr(executor, 'r90_statistics'))
        self.assertEqual([event for event in self.clock.events if event[0] == 'yield'],
                         [('yield', 0), ('yield', 0)])

    def test_invalid_switch_rejected_before_any_executor_construction(self):
        for option in ('', 'yes', 'true', '2', '0.001', ' 1'):
            with self.subTest(option=option), patch.object(experiment.original, 'make_executor') as delegate:
                os.environ[experiment.KEY] = option
                base = Mock()
                with self.assertRaises(ValueError):
                    experiment.make_executor(base, profile())
                delegate.assert_not_called()
                base.assert_not_called()

    def test_on_requires_exact_original_selected_simulation_contract(self):
        variations = [None, {}, dict(r74_cooperative_executor=dict(enabled=False,
            schema='r74_spin_once_zero_yield/v1'))]
        for key, value in (
                ('r74_cooperative_executor', dict(enabled=1, schema='r74_spin_once_zero_yield/v1')),
                ('r74_cooperative_executor', dict(enabled=True, schema='wrong')),
                ('r74_teacher_substage_timing', dict(enabled=False, schema='r74_bounded_teacher_substage_timing/v1')),
                ('r73_minimal_diagnostic_baseline', dict(enabled=False)),
                ('diagnostic_observe_only', dict(enabled=False)),
                ('navigation_ground_truth_used', True), ('actor_kind', 'other')):
            value_profile = profile()
            value_profile[key] = value
            variations.append(value_profile)
        for value in variations:
            with self.subTest(profile=value):
                base = Mock()
                with self.assertRaises(ValueError):
                    experiment.make_executor(base, value)
                base.assert_not_called()
        with patch.dict(os.environ, GO2_R73_MINIMAL_SIMULATION='0'):
            with self.assertRaises(ValueError):
                self.make()

    def test_on_preserves_constructor_thread_count_and_profile(self):
        value = profile()
        before = copy.deepcopy(value)
        executor = experiment.make_executor(Base, value, num_threads=2)
        self.assertEqual(executor.constructor_args, ())
        self.assertEqual(executor.constructor_kwargs, {'num_threads': 2})
        self.assertEqual(value, before)
        self.assertIs(type(executor).spin_once, Base.spin_once)

    def test_dispatch_precedes_exactly_one_positive_yield_per_return(self):
        executor = self.make()
        self.finite_dispatch(executor, 3)
        executor.spin()
        self.assertGreater(experiment.YIELD_SECONDS, 0)
        self.assertEqual(self.clock.events, [
            ('dispatch', 1), ('yield', experiment.YIELD_SECONDS),
            ('dispatch', 2), ('yield', experiment.YIELD_SECONDS),
            ('dispatch', 3), ('yield', experiment.YIELD_SECONDS)])
        self.assertEqual(executor.calls, [((), {})] * 3)

    def test_context_false_or_preexisting_stop_prevents_dispatch(self):
        for stopped in (False, True):
            with self.subTest(stopped=stopped):
                executor = self.make()
                if stopped:
                    executor._r74_cooperative_stop.set()
                else:
                    executor.context_ok = False
                executor.spin()
                self.assertEqual(executor.calls, [])
                self.assertEqual(executor.r90_statistics()['spin_calls'], 0)
        self.assertEqual(self.clock.events, [])

    def test_stop_during_spin_or_yield_never_dispatches_again(self):
        for during in ('spin', 'yield'):
            with self.subTest(during=during):
                executor = self.make()
                if during == 'spin':
                    executor.on_spin = executor._r74_cooperative_stop.set
                else:
                    self.clock.sleep_hook = executor._r74_cooperative_stop.set
                executor.spin()
                self.assertEqual(len(executor.calls), 1)
                self.assertEqual(executor.r90_statistics()['yield_calls'], 1)
                self.clock.sleep_hook = None

    def test_original_spin_exception_identity_preserved_without_yield(self):
        for error in (ValueError('callback failed'), KeyboardInterrupt('original interrupt')):
            with self.subTest(error=type(error).__name__):
                executor = self.make()
                def dispatch():
                    self.clock.advance(6_000_000, 2_000_000)
                    raise error
                executor.on_spin = dispatch
                try:
                    executor.spin()
                except BaseException as actual:
                    self.assertIs(actual, error)
                else:
                    self.fail('original spin error was suppressed')
                self.assertEqual(len(executor.calls), 1)
                stats = executor.r90_statistics()
                self.assertEqual(stats['spin_exceptions'], 1)
                self.assertEqual(stats['spin_calls'], 1)
                self.assertEqual(stats['yield_calls'], 0)
        self.assertEqual(self.clock.events, [])

    def test_sleep_error_is_not_silently_retried_or_swallowed(self):
        executor = self.make()
        error = KeyboardInterrupt('original sleep interrupt')
        self.clock.sleep_error = error
        try:
            executor.spin()
        except BaseException as actual:
            self.assertIs(actual, error)
        else:
            self.fail('sleep error swallowed')
        self.assertEqual(len(executor.calls), 1)
        self.assertEqual(len(self.clock.events), 1)

    def test_shutdown_sets_stop_before_base_and_preserves_arguments_return(self):
        executor = self.make()
        sentinel = object()
        result = executor.shutdown(sentinel, timeout_sec=.25)
        self.assertIs(result, executor.shutdown_result)
        self.assertEqual(executor.shutdown_calls, [((sentinel,), {'timeout_sec': .25}, True)])
        executor.spin()
        self.assertEqual(executor.calls, [])

    def test_shutdown_base_exception_preserved_and_stop_remains_set(self):
        executor = self.make()
        error = RuntimeError('base shutdown failure')
        executor.shutdown_error = error
        try:
            executor.shutdown(timeout_sec=.1)
        except RuntimeError as actual:
            self.assertIs(actual, error)
        else:
            self.fail('shutdown error swallowed')
        self.assertTrue(executor._r74_cooperative_stop.is_set())

    def test_statistics_separate_spin_and_yield_wall_from_thread_cpu(self):
        executor = self.make()
        self.finite_dispatch(executor, 3)
        executor.spin()
        stats = executor.r90_statistics()
        self.assertEqual(stats['spin_calls'], 3)
        self.assertEqual(stats['spin_wall_ns'], 30_000_000)
        self.assertEqual(stats['spin_cpu_ns'], 9_000_000)
        self.assertEqual(stats['spin_wall_max_ns'], 10_000_000)
        self.assertEqual(stats['spin_cpu_max_ns'], 3_000_000)
        self.assertEqual(stats['yield_calls'], 3)
        self.assertEqual(stats['yield_wall_ns'], round(experiment.YIELD_SECONDS * 1e9) * 3)
        self.assertEqual(stats['yield_wall_max_ns'], round(experiment.YIELD_SECONDS * 1e9))
        self.assertEqual(stats['slow_intervals'], [])

    def test_slow_window_bounded_and_evictions_visible(self):
        executor = self.make()
        count = experiment.SLOW_CAPACITY + 17
        self.finite_dispatch(executor, count, wall_ns=experiment.SLOW_NS, cpu_ns=20_000_000)
        with patch.object(Path, 'write_text', side_effect=AssertionError('hot-path write')):
            executor.spin()
        stats = executor.r90_statistics()
        self.assertEqual(stats['slow_records'], count)
        self.assertEqual(stats['slow_evicted'], 17)
        self.assertEqual(len(stats['slow_intervals']), experiment.SLOW_CAPACITY)
        self.assertEqual(stats['spin_wall_ns'], experiment.SLOW_NS * count)
        self.assertTrue(all(row['wall_ns'] == experiment.SLOW_NS for row in stats['slow_intervals']))
        self.assertTrue(all(row['thread_cpu_ns'] == 20_000_000 for row in stats['slow_intervals']))
        self.assertEqual(stats['slow_intervals'][0]['begin_ns'], 100_000_000 + 17 *
                         (experiment.SLOW_NS + round(experiment.YIELD_SECONDS * 1e9)))

    def test_receipt_describes_actual_instance_after_environment_change(self):
        executor = self.make()
        os.environ[experiment.KEY] = '0'
        receipt = experiment.receipt(executor)
        self.assertTrue(receipt['enabled'])
        self.assertEqual(receipt['environment_value'], '0')
        self.assertEqual(receipt['yield_seconds'], experiment.YIELD_SECONDS)
        self.assertEqual(receipt['actual_executor_module'], 'r90_cooperative_executor')
        self.assertEqual(receipt['actual_executor_class'], type(executor).__qualname__)

    def test_finish_records_bounded_statistics_only_after_execution(self):
        executor = self.make()
        self.finite_dispatch(executor, 2)
        with tempfile.TemporaryDirectory(prefix='r90-dispatch-tests-') as directory:
            node = NS(run=Path(directory))
            executor.spin()
            self.assertEqual(list(node.run.iterdir()), [])
            experiment.finish(node, executor)
            data = json.loads((node.run/'r90_spinner_statistics.json').read_text())
            self.assertEqual(data['statistics']['spin_calls'], 2)
            self.assertTrue(data['selection']['enabled'])

    def test_finish_write_error_is_reported_without_rethrow(self):
        executor = self.make()
        node = NS(run=Path('/unused-test-path'))
        with patch.object(Path, 'write_text', side_effect=OSError('injected output failure')):
            experiment.finish(node, executor)
        self.assertTrue(node._r90_statistics_write_failed)

    def test_finish_off_never_accesses_node_or_writes(self):
        with patch.object(Path, 'write_text', side_effect=AssertionError('disabled write')):
            experiment.finish(object(), object())

    def test_diagnostic_clock_failure_before_dispatch_does_not_skip_original(self):
        executor = self.make()
        self.finite_dispatch(executor, 1)
        real_read = self.clock.monotonic_ns
        calls = 0
        def failed_once():
            nonlocal calls
            calls += 1
            if calls == 1:
                raise OSError('injected diagnostic clock failure')
            return real_read()
        with patch.object(self.clock, 'monotonic_ns', side_effect=failed_once):
            executor.spin()
        self.assertEqual(len(executor.calls), 1)

    def test_diagnostic_clock_failure_cannot_replace_original_spin_error(self):
        executor = self.make()
        original = ValueError('the actual callback failure')
        def dispatch():
            raise original
        executor.on_spin = dispatch
        real_read = self.clock.thread_time_ns
        calls = 0
        def failed_after_call():
            nonlocal calls
            calls += 1
            if calls > 1:
                raise OSError('injected diagnostic exit clock failure')
            return real_read()
        with patch.object(self.clock, 'thread_time_ns', side_effect=failed_after_call):
            try:
                executor.spin()
            except BaseException as actual:
                self.assertIs(actual, original)
            else:
                self.fail('actual callback exception was suppressed')
        self.assertEqual(len(executor.calls), 1)

    def test_finish_diagnostic_lookup_and_failure_flag_are_nonthrowing(self):
        class BrokenStatistics:
            @property
            def r90_statistics(self):
                raise OSError('injected statistics lookup failure')
        experiment.finish(object(), BrokenStatistics())
        with patch.object(Path, 'write_text', side_effect=OSError('injected write failure')):
            experiment.finish(object(), self.make())


if __name__ == '__main__':
    unittest.main()
