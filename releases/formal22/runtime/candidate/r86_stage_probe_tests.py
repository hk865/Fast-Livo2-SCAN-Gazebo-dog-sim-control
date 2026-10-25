"""Focused offline sampling checks; no ROS initialization or simulation."""
import ast
from collections import deque
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import unittest
from unittest.mock import patch

import r86_stage_probe as probe

HERE = Path(__file__).resolve().parent


def timing_type(source=None):
    source = (HERE / 'controller.py').read_text() if source is None else source
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == '_TeacherSubstageTiming')
    scope = dict(time=time, deque=deque, json=json, os=os,
                 _r86_probe_stages=probe.STAGES, _r86_probe_sample=probe.sample)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), '<isolated-timing-class>', 'exec'), scope)
    return scope['_TeacherSubstageTiming'], scope


class ProbeTests(unittest.TestCase):
    def test_all_four_stages_preserve_real_call_and_optional_failures(self):
        cls, scope = timing_type()
        observer=cls(Path('/unused'),wall_clock=lambda:101,cpu_clock=lambda:23)
        call=observer.begin('admission_body',original_ros_ns=7,original_wall_ns=8)
        for stage in probe.STAGES:observer.mark(call,stage)
        observer.end(call,ValueError)
        rows=observer.snapshot()['markers']
        self.assertEqual(sum('r86_execution_sample' in r for r in rows),4)
        self.assertEqual(call['exception_type'],'ValueError')
        self.assertEqual(call['original_ros_clock_ns'],7)
        for owner,attr in ((probe.threading,'get_native_id'),(probe.time,'thread_time_ns'),
                           (probe.time,'process_time_ns'),(probe.resource,'getrusage')):
            with patch.object(owner,attr,side_effect=OSError):
                row=probe.sample()
            self.assertTrue(row['errors'])
        for clocks in ([OSError(),202],[101,OSError()]):
            with patch.object(probe.time,'monotonic_ns',side_effect=clocks):
                observer.mark(call,'remember_native_guard_after')
        self.assertEqual(observer.markers_recorded,8)
        self.assertEqual(observer.error_count,0)

    def test_sample_read_order_and_double_wall_boundary(self):
        order=[]
        def reader(name,value):
            def f(*args):order.append(name);return value
            return f
        usage=type('Usage',(),{'ru_'+k:1 for k in ('nvcsw','nivcsw','minflt','majflt','inblock','oublock')})()
        with patch.object(probe.time,'monotonic_ns',side_effect=reader('wall',100)), \
             patch.object(probe.threading,'get_native_id',side_effect=reader('tid',2)), \
             patch.object(probe.time,'thread_time_ns',side_effect=reader('thread',3)), \
             patch.object(probe.time,'process_time_ns',side_effect=reader('process',4)), \
             patch.object(probe.resource,'getrusage',side_effect=reader('rusage',usage)):
            row=probe.sample()
        self.assertEqual(order,['wall','tid','thread','process','rusage','wall'])
        self.assertEqual(row['wall_begin_ns'],row['wall_end_ns'])

    def test_optional_failure_is_unknown(self):
        with patch.object(probe.time, 'process_time_ns', side_effect=OSError):
            row = probe.sample()
        self.assertIsNone(row['process_cpu_ns'])
        self.assertEqual(row['errors']['process_cpu_ns'], 'OSError')
        self.assertLessEqual(row['wall_begin_ns'], row['wall_end_ns'])

    def test_target_stages_only_and_failure_does_not_drop_original(self):
        cls, scope = timing_type()
        def unavailable():
            raise RuntimeError('optional probe failure')
        scope['_r86_probe_sample'] = unavailable
        observer = cls(Path('/unused'), wall_clock=lambda: 101, cpu_clock=lambda: 23)
        observer.mark(None, 'get_ack_before')
        observer.mark(None, 'native_geometry_guard_before')
        self.assertEqual(observer.markers_recorded, 2)
        self.assertEqual(observer.error_count, 0)
        self.assertNotIn('r86_execution_sample_error', observer.first[0])
        self.assertEqual(observer.first[1]['r86_execution_sample_error'], 'RuntimeError')
        self.assertEqual(observer.first[1]['monotonic_wall_ns'], 101)
        self.assertEqual(observer.first[1]['thread_cpu_ns'], 23)

    def test_capacity_and_no_hot_path_io(self):
        cls, _ = timing_type()
        observer = cls(Path('/unused'))
        with patch('builtins.open', side_effect=AssertionError('hot I/O')), \
                patch.object(os, 'open', side_effect=AssertionError('hot I/O')):
            for _ in range(5000):
                observer.mark(None, 'remember_native_guard_before')
        value = observer.snapshot()
        self.assertEqual(len(value['markers']), observer.FIRST + observer.LAST)
        self.assertEqual(value['r86_execution_sampling']['sample_count'], 5000)
        self.assertEqual(value['r86_execution_sampling']['sample_error_count'], 0)
        self.assertLess(len(json.dumps(value)), 5_000_000)
        self.assertTrue(value['overflowed'])


def percentile(values, q):
    values = sorted(values)
    index = (len(values) - 1) * q
    lo = int(index)
    return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (index - lo)


def benchmark():
    baseline = subprocess.check_output(['git', 'show', '6f9e657:candidate/controller.py'],
                                       cwd=HERE, text=True)
    old, _ = timing_type(baseline)
    new, _ = timing_type()
    results = {}
    # Interleave the same stage/retention work; no serialization during timing.
    observers = {'R20_original': old(Path('/unused')), 'R86_observed': new(Path('/unused'))}
    timings = {key: [] for key in observers}
    for iteration in range(6000):
        for key in (tuple(observers) if iteration % 2 else tuple(reversed(observers))):
            start = time.monotonic_ns()
            observers[key].mark(None, 'native_geometry_guard_before')
            elapsed = time.monotonic_ns() - start
            if iteration >= 1000:
                timings[key].append(elapsed)
    for key, values in timings.items():
        results[key] = dict(n=len(values), median_ns=percentile(values, .5),
                            p95_ns=percentile(values, .95), p99_ns=percentile(values, .99),
                            maximum_ns=max(values))
    examples = {}
    for kind in ('sleep', 'current_thread_cpu', 'other_thread_cpu'):
        ready = threading.Event()
        go = threading.Event()
        def cpu_work():
            ready.set()
            go.wait()
            deadline = time.monotonic() + .03
            while time.monotonic() < deadline:
                sum(range(400))
        worker = None
        if kind == 'other_thread_cpu':
            worker = threading.Thread(target=cpu_work)
            worker.start()
            ready.wait()
        before = probe.sample()
        if kind == 'sleep':
            time.sleep(.03)
        elif kind == 'current_thread_cpu':
            go.set()
            cpu_work()
        else:
            go.set()
            worker.join()
        after = probe.sample()
        examples[kind] = dict(wall_ns=after['wall_begin_ns']-before['wall_end_ns'],
            thread_cpu_ns=after['thread_cpu_ns']-before['thread_cpu_ns'],
            process_cpu_ns=after['process_cpu_ns']-before['process_cpu_ns'],
            same_tid=before['native_tid']==after['native_tid'],
            nvcsw=after['thread_usage']['nvcsw']-before['thread_usage']['nvcsw'],
            nivcsw=after['thread_usage']['nivcsw']-before['thread_usage']['nivcsw'])
    sampled=[r['r86_execution_sample'] for r in observers['R86_observed'].snapshot()['markers']
             if 'r86_execution_sample' in r]
    spans=[r['wall_end_ns']-r['wall_begin_ns'] for r in sampled]
    sample_span=dict(n=len(spans),median_ns=percentile(spans,.5),p95_ns=percentile(spans,.95),
                     p99_ns=percentile(spans,.99),observed_max_ns=max(spans))
    return dict(schema='r86_probe_offline_measurement/v2', benchmark=results,
                retained_double_wall_span=sample_span,
                benchmark_scope='one complete mark(None, native_geometry_guard_before); not four endpoints',
                maximum_is_observed_not_a_latency_bound=True,
                semantic_examples=examples, control_or_navigation_PASS=False,
                physics_started=False, source=__file__)


if __name__ == '__main__':
    import sys
    if '--benchmark' in sys.argv:
        print(json.dumps(benchmark(), indent=2))
    else:
        unittest.main()
