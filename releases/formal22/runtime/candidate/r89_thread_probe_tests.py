"""Finite self-only observer tests and no-ROS overhead measurements."""
import argparse
import builtins
from contextlib import contextmanager
import hashlib
import io
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

import r89_thread_probe as probe


def stat_text(start=123, user=12, system=5):
    fields = ['S'] + ['0'] * 21
    fields[11], fields[12], fields[19] = str(user), str(system), str(start)
    return '321 (a tricky (thread) name) ' + ' '.join(fields) + '\n'


@contextmanager
def entries(names):
    yield iter(types.SimpleNamespace(name=str(name)) for name in names)


def fixture_read(path, limit=probe._READ_LIMIT):
    name = path.rsplit('/', 1)[-1]
    return {'stat': stat_text(), 'schedstat': '23000000 5000000 21\n',
            'status': 'Name:\tfixture\nvoluntary_ctxt_switches:\t9\nnonvoluntary_ctxt_switches:\t3\n',
            'wchan': 'futex_wait_queue\n'}[name]


class ThreadProbeTests(unittest.TestCase):
    def test_default_off_import_creates_no_thread_and_no_io(self):
        source = Path(probe.__file__).read_text()
        for option in (None, '0', 'invalid', '1'):
            namespace = {'__name__': '_r89_test_import'}
            with patch.dict(os.environ, {}, clear=False),\
                 patch.object(threading.Thread, 'start', side_effect=AssertionError('thread on import')),\
                 patch.object(builtins, 'open', side_effect=AssertionError('I/O on import')):
                os.environ.pop(probe.KEY, None)
                if option is not None:
                    os.environ[probe.KEY] = option
                exec(compile(source, probe.__file__, 'exec'), namespace)
                self.assertEqual(namespace['enabled'](), option == '1')
                self.assertIsNone(namespace['PROBE'])

    def test_constructor_hard_limits_and_proc_parsers(self):
        for kwargs in ({'capacity': 0}, {'capacity': 1025}, {'capacity': True},
                       {'max_threads': 65}, {'max_threads': 0},
                       {'interval_ns': 19_999_999}, {'interval_ns': 50_000_001}):
            with self.assertRaises(ValueError):
                probe.ThreadProbe(**kwargs)
        parsed = probe._stat(stat_text())
        self.assertEqual(parsed, dict(comm='a tricky (thread) name', state='S',
            user_ticks=12, system_ticks=5, starttime_ticks=123))
        for value in ('', '123 broken', '123 (short) S 1 2'):
            with self.assertRaises(ValueError):
                probe._stat(value)
        with self.assertRaises(ValueError):
            probe._schedstat('1 -2 3')
        with self.assertRaises(ValueError):
            probe._status('Name: missing counts')

    def test_read_size_limit_and_read_only_mode(self):
        with patch.object(builtins, 'open', return_value=io.StringIO('a' * 17)) as opened:
            with self.assertRaises(ValueError):
                probe._read('/proc/self/task/321/stat', limit=16)
        self.assertEqual(opened.call_args.args[0], '/proc/self/task/321/stat')
        self.assertEqual(opened.call_args.args[1], 'r')

    def test_thread_cap_and_identity_bracket_and_locations_no_locals(self):
        recorder = probe.ThreadProbe(capacity=3, max_threads=2)
        secret_payload = 'DO_NOT_CAPTURE_this_local_value_98278'
        main = types.SimpleNamespace(native_id=321, ident=threading.get_ident(), name='fixture-owner')
        with patch.object(probe.os, 'scandir', side_effect=lambda path: entries([321, 322, 323])),\
             patch.object(probe, '_read', side_effect=fixture_read),\
             patch.object(probe.threading, 'enumerate', return_value=[main]):
            recorder._sample(time.monotonic_ns())
        row = recorder.rows[0]
        self.assertEqual(len(row['threads']), 2)
        self.assertEqual(row['omitted_task_count'], 1)
        self.assertEqual(recorder.omitted_thread_observations, 1)
        task = row['threads'][0]
        self.assertEqual(task['python_thread_name'], 'fixture-owner')
        self.assertEqual(task['starttime_ticks'], 123)
        self.assertEqual(task['cpu_end'], dict(user_ticks=12, system_ticks=5))
        self.assertEqual(task['schedstat']['runtime_ns'], 23000000)
        self.assertTrue(0 < len(task['python_stack']) <= 12)
        encoded = json.dumps(recorder.stop())
        self.assertNotIn(secret_payload, encoded)
        self.assertNotIn('f_locals', encoded)
        self.assertEqual(task['wchan'], 'futex_wait_queue')

    def test_native_tid_reuse_does_not_mix_counters(self):
        recorder = probe.ThreadProbe(capacity=3)
        count = 0
        def reused_read(path):
            nonlocal count
            if path.endswith('/stat'):
                count += 1
                return stat_text(start=count)
            return fixture_read(path)
        with patch.object(probe.os, 'scandir', side_effect=lambda path: entries([321])),\
             patch.object(probe, '_read', side_effect=reused_read):
            recorder._sample(time.monotonic_ns())
        self.assertEqual(recorder.identity_changed, 1)
        self.assertEqual(recorder.rows[0]['threads'], [])

    def test_permission_and_disappearing_task_degrade_with_counts(self):
        recorder = probe.ThreadProbe(capacity=3)
        def denied(path):
            if path.endswith('/stat'):
                return stat_text()
            raise PermissionError('fixture denied')
        with patch.object(probe.os, 'scandir', side_effect=lambda path: entries([321])),\
             patch.object(probe, '_read', side_effect=denied):
            recorder._sample(time.monotonic_ns())
        self.assertEqual(len(recorder.rows[0]['threads']), 1)
        for name in ('schedstat', 'status', 'wchan'):
            self.assertEqual(recorder.errors[name + ':denied'], 1)
            self.assertIsNone(recorder.rows[0]['threads'][0][name])
        with patch.object(probe.os, 'scandir', side_effect=PermissionError('fixture denied')):
            recorder._sample(time.monotonic_ns())
        with patch.object(probe.os, 'scandir', side_effect=lambda path: entries([321])),\
             patch.object(probe, '_read', side_effect=FileNotFoundError('gone')):
            recorder._sample(time.monotonic_ns())
        self.assertEqual(recorder.errors['task_list:denied'], 1)
        self.assertEqual(recorder.errors['stat:gone'], 1)
        self.assertEqual(recorder.recorded, 3)

    def test_stack_depth_string_bounds_and_no_frame_retention(self):
        fake = types.SimpleNamespace(f_code=types.SimpleNamespace(co_filename='f' * 300,
            co_name='n' * 200), f_lineno=42, f_back=None)
        head = fake
        for _ in range(20):
            head = types.SimpleNamespace(f_code=fake.f_code, f_lineno=42, f_back=head)
        locations = probe._locations(head)
        self.assertEqual(len(locations), 12)
        self.assertTrue(all(len(value[0]) == 240 and len(value[1]) == 120 for value in locations))
        self.assertTrue(all(type(value[2]) is int for value in locations))

    def test_freeze_and_longest_zone_windows_bounded_and_independent(self):
        recorder = probe.ThreadProbe(capacity=4)
        for index in range(7):
            recorder._record(dict(sequence=index))
        recorder.freeze('first', 12)
        recorder._record(dict(sequence=7))
        original = recorder.frozen
        recorder.freeze('second', 13)
        recorder.requests['slow'] = dict(duration_ns=250_000_000, stage='native',
            entered_wall_ns=1, exited_wall_ns=250_000_001)
        for index in range(8, 25):
            recorder._record(dict(sequence=index))
        self.assertIs(recorder.frozen, original)
        self.assertEqual(recorder.freeze_trigger['reason'], 'first')
        self.assertEqual(len(recorder.rows), 4)
        self.assertEqual(len(recorder.frozen), 4)
        self.assertEqual(len(recorder.slow_window), 4)
        self.assertEqual(len(recorder.frozen_post), 5)
        self.assertEqual(len(recorder.slow_post), 5)
        recorder.requests['slow'] = dict(duration_ns=400_000_000, stage='native',
            entered_wall_ns=2, exited_wall_ns=400_000_002)
        recorder._record(dict(sequence=25))
        self.assertEqual(recorder.slow_trigger['duration_ns'], 400_000_000)
        self.assertEqual(len(recorder.slow_post), 0)
        self.assertIs(recorder.frozen, original)
        snapshot = recorder.stop()
        self.assertEqual(snapshot['recorded'], 26)
        self.assertEqual(snapshot['evicted'], 22)

    def test_zone_nested_business_exception_and_observer_failure_transparency(self):
        recorder = probe.ThreadProbe(capacity=4)
        tid = threading.get_native_id()
        with recorder.zone('outer', 1):
            outer = recorder.zones[tid]
            with self.assertRaisesRegex(RuntimeError, 'business error'):
                with recorder.zone('inner', 2):
                    self.assertEqual(recorder.zones[tid]['stage'], 'inner')
                    raise RuntimeError('business error')
            self.assertIs(recorder.zones[tid], outer)
        self.assertEqual(recorder.zones, {})
        with patch.object(probe.time, 'monotonic_ns', side_effect=OSError('clock fixture')):
            with recorder.zone('broken observer'):
                result = 4
            recorder.freeze('broken observer')
        self.assertEqual(result, 4)
        self.assertGreater(recorder.errors['zone:other'], 0)

    def test_zone_freeze_hot_paths_have_no_io_or_lock_and_slow_request_at_exit(self):
        recorder = probe.ThreadProbe(capacity=4)
        forbidden = AssertionError('hot path performed I/O')
        with recorder.guard,\
             patch.object(builtins, 'open', side_effect=forbidden),\
             patch.object(builtins, 'print', side_effect=forbidden),\
             patch.object(probe.os, 'scandir', side_effect=forbidden),\
             patch.object(probe.sys, '_current_frames', side_effect=forbidden),\
             patch.object(recorder, '_cpu_snapshot', return_value={'fixture': True}),\
             patch.object(probe.time, 'monotonic_ns', side_effect=[100, 101, 200_000_101, 200_000_102]):
            with recorder.zone('native_geometry', 91):
                pass
            recorder.freeze('fault', 91)
        self.assertEqual(recorder.requests['slow']['duration_ns'], 200_000_001)
        self.assertEqual(recorder.requests['freeze']['source_ns'], 91)
        snapshot = recorder.stop()
        self.assertTrue(snapshot['slow_trigger']['exit_capture'])
        self.assertTrue(snapshot['anomaly_trigger']['exit_capture'])

    def test_zone_capacity_and_record_lock_drop_are_explicit(self):
        recorder = probe.ThreadProbe(capacity=4, max_threads=1)
        recorder.zones[-1] = dict(stage='fixture-other')
        with recorder.zone('over-capacity'):
            self.assertEqual(len(recorder.zones), 1)
        self.assertEqual(recorder.zone_drops, 1)
        with recorder.guard:
            recorder._record(dict(sequence=1))
            snapshot = recorder.stop(timeout=0)
        self.assertEqual(recorder.dropped_lock_contention, 1)
        self.assertTrue(snapshot['snapshot_unavailable_lock_contention'])

    def test_startup_failure_and_thread_start_failure_never_escape(self):
        recorder = probe.ThreadProbe(capacity=4)
        with patch.object(probe, '_read', side_effect=PermissionError('denied')),\
             patch.object(builtins, 'open', side_effect=PermissionError('denied')),\
             patch.object(probe.importlib, 'import_module', side_effect=ImportError('absent')):
            startup = recorder._startup()
        self.assertTrue(startup['captured_once'])
        self.assertFalse(startup['changed_thread_limits'])
        self.assertEqual(startup['threadpool_status'], 'ImportError')
        with patch.object(recorder, '_startup', return_value=startup),\
             patch.object(threading.Thread, 'start', side_effect=RuntimeError('no threads')):
            self.assertFalse(recorder.start())
        self.assertTrue(recorder.finished)
        self.assertEqual(recorder.errors['thread_start:other'], 1)
        self.assertTrue(recorder.stop()['stop_complete'])

    def test_native_library_introspection_is_once_and_never_changes_limits(self):
        recorder = probe.ThreadProbe(capacity=4)
        calls = []
        module = types.SimpleNamespace(threadpool_info=lambda: calls.append('info') or
            [dict(internal_api='openblas', num_threads=1, irrelevant='private', filepath='/tmp/libblas.so')])
        with patch.object(recorder, '_run', return_value=None),\
             patch.object(probe.importlib, 'import_module', return_value=module):
            self.assertTrue(recorder.start())
            self.assertFalse(recorder.start())
            snapshot = recorder.stop()
        self.assertEqual(calls, ['info'])
        self.assertEqual(snapshot['startup']['threadpools'][0]['num_threads'], 1)
        self.assertNotIn('irrelevant', snapshot['startup']['threadpools'][0])
        self.assertFalse(snapshot['startup']['changed_thread_limits'])

    def test_real_self_process_only_sampling_roles_cleanup_and_cpu_accounting(self):
        recorder = probe.ThreadProbe(capacity=8, interval_ns=20_000_000)
        before_threads = {t.ident for t in threading.enumerate()}
        reads = []
        original = probe._read
        def audited(path, *args, **kwargs):
            reads.append(path)
            return original(path, *args, **kwargs)
        with patch.object(probe, '_read', side_effect=audited):
            self.assertTrue(recorder.start())
            with recorder.zone('native_geometry_fixture', 91):
                time.sleep(.08)
            recorder.freeze('fixture', 91)
            time.sleep(.04)
            snapshot = recorder.stop()
        self.assertTrue(snapshot['stop_complete'])
        self.assertFalse(snapshot['running'])
        self.assertGreaterEqual(snapshot['recorded'], 3)
        self.assertGreater(snapshot['sample_cpu_ns'], 0)
        self.assertGreaterEqual(snapshot['sampler_lifetime_cpu_ns'], snapshot['sample_cpu_ns'])
        self.assertTrue(all(path.startswith('/proc/self/') for path in reads))
        self.assertEqual(snapshot['sample_errors'], 0)
        tasks = [task for row in snapshot['rows'] for task in row['threads']]
        self.assertTrue(any(task['native_tid'] == threading.get_native_id() for task in tasks))
        self.assertTrue(any(task.get('zone', {}).get('stage') == 'native_geometry_fixture' for task in tasks))
        self.assertTrue(any(task['python_thread_name'] == 'r89-thread-probe' for task in tasks))
        self.assertEqual({t.ident for t in threading.enumerate()}, before_threads)
        json.dumps(snapshot, allow_nan=False)

    def test_sampler_overrun_counts_missed_slots_without_burst_catchup(self):
        recorder = probe.ThreadProbe(interval_ns=20_000_000, capacity=4)
        visits = []
        def slow(scheduled):
            visits.append(scheduled)
            time.sleep(.046)
            if len(visits) == 2:
                recorder.stop_event.set()
        with patch.object(recorder, '_sample', side_effect=slow):
            recorder._run()
        self.assertGreaterEqual(recorder.missed_intervals, 4)
        self.assertGreaterEqual(visits[1] - visits[0], 60_000_000)
        self.assertTrue(recorder.finished)

    def test_stop_timeout_is_reported_without_killing_target_thread(self):
        recorder = probe.ThreadProbe(capacity=4)
        entered, release = threading.Event(), threading.Event()
        def delayed():
            entered.set()
            release.wait(2)
        with patch.object(recorder, '_startup', return_value={}),\
             patch.object(recorder, '_run', side_effect=delayed):
            try:
                self.assertTrue(recorder.start())
                self.assertTrue(entered.wait(1))
                before = time.monotonic()
                snapshot = recorder.stop(timeout=.005)
                self.assertLess(time.monotonic() - before, .25)
                self.assertFalse(snapshot['stop_complete'])
                self.assertTrue(snapshot['running'])
            finally:
                release.set()
                self.assertTrue(recorder.stop()['stop_complete'])

    def test_capture_timestamps_bracket_stacks_and_task_reads(self):
        recorder = probe.ThreadProbe(capacity=3)
        with patch.object(probe.os, 'scandir', side_effect=lambda path: entries([321])),\
             patch.object(probe, '_read', side_effect=fixture_read):
            recorder._sample(time.monotonic_ns())
        row = recorder.rows[0]
        item = row['threads'][0]
        ordered = [row['wall_begin_ns'], item['wall_begin_ns'],
            item['stat_begin_read_end_ns'], row['thread_map_capture_begin_ns'],
            row['thread_map_capture_end_ns'], row['python_frames_capture_begin_ns'],
            row['zones_capture_begin_ns'], row['zones_capture_end_ns'],
            row['python_frames_capture_end_ns'], item['schedstat_wall_begin_ns'],
            item['schedstat_wall_end_ns'], item['status_wall_begin_ns'],
            item['status_wall_end_ns'], item['wchan_wall_begin_ns'],
            item['wchan_wall_end_ns'], item['stat_end_read_begin_ns'],
            item['wall_end_ns'], row['wall_end_ns']]
        self.assertEqual(ordered, sorted(ordered))

    def test_module_wrappers_preserve_business_exception_off_and_on(self):
        for recorder in (None, probe.ThreadProbe(capacity=4)):
            with patch.object(probe, 'PROBE', recorder):
                with self.assertRaisesRegex(ValueError, 'original business'):
                    with probe.zone('some-stage', 1):
                        raise ValueError('original business')
                probe.freeze('fault', 1)
                self.assertIsInstance(probe.status(), dict)
                self.assertIsInstance(probe.stop(), dict)

    def test_linux_clock_self_exited_and_foreign_thread_group(self):
        if sys.platform != 'linux':
            self.skipTest('Linux encoded CPU clock test')
        before = time.thread_time_ns()
        value = probe._thread_clock_ns(threading.get_native_id())
        after = time.thread_time_ns()
        self.assertLessEqual(before, value)
        self.assertLessEqual(value, after)
        tids = []
        worker = threading.Thread(target=lambda: tids.append(threading.get_native_id()))
        worker.start()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        # join can precede final kernel task release. Do not mistake the brief
        # still-existing task and its final CPU counter for an invalid clock.
        deadline = time.monotonic()+1
        while os.path.exists('/proc/self/task/'+str(tids[0])) and time.monotonic() < deadline:
            time.sleep(.001)
        self.assertFalse(os.path.exists('/proc/self/task/'+str(tids[0])))
        with self.assertRaises(OSError):
            probe._thread_clock_ns(tids[0])
        child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(.1)'],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            with self.assertRaises(OSError):
                probe._thread_clock_ns(child.pid)
        finally:
            child.wait(timeout=2)

    def test_boundary_cpu_only_registered_tasks_stale_and_exit_unknown(self):
        recorder = probe.ThreadProbe(capacity=4)
        recorder.clock_available = True
        now = time.monotonic_ns()
        recorder.registry = tuple(dict(native_tid=tid, starttime_ticks=11,
            confirmed_wall_ns=now-age, registry_generation=1)
            for tid, age in ((123, 0), (124, 0), (125, 3_000_000_000)))
        def clock(tid):
            if tid == 124:
                raise OSError('exited fixture')
            return tid * 1000
        with patch.object(probe, '_thread_clock_ns', side_effect=clock) as called:
            snapshot = recorder._cpu_snapshot()
        self.assertEqual([call.args[0] for call in called.call_args_list], [123, 124])
        self.assertEqual(snapshot['threads'][0]['cpu_ns'], 123000)
        self.assertEqual(snapshot['threads'][1]['unknown'], 'OSError')
        self.assertEqual(snapshot['threads'][2]['unknown'], 'stale_registry')
        self.assertTrue(snapshot['identity_at_clock_read_unverified'])
        self.assertEqual(recorder.boundary_clock_reads, 1)
        self.assertEqual(recorder.boundary_clock_unknown, 2)

    def test_zone_cpu_records_capacity_first_anomaly_longest_and_post_windows(self):
        recorder = probe.ThreadProbe(capacity=4)
        with patch.object(recorder, '_cpu_snapshot', return_value={'fixture': True}):
            for _ in range(probe.ZONE_CAPACITY+3):
                with recorder.zone('bounded-zone'):
                    pass
            recorder.freeze('first')
            recorder._record(dict(sequence=1))
            recorder.requests['slow'] = dict(duration_ns=400_000_000)
            recorder._record(dict(sequence=2))
            for _ in range(10):
                with recorder.zone('post'):
                    pass
        self.assertEqual(recorder.zone_recorded, probe.ZONE_CAPACITY+13)
        self.assertEqual(recorder.zone_evicted, 13)
        self.assertEqual(len(recorder.zone_rows), probe.ZONE_CAPACITY)
        self.assertEqual(len(recorder.frozen_zones), probe.ZONE_CAPACITY)
        self.assertEqual(len(recorder.slow_zones), probe.ZONE_CAPACITY)
        self.assertEqual(len(recorder.frozen_zone_post), 5)
        self.assertEqual(len(recorder.slow_zone_post), 5)


def distribution(values):
    ordered = sorted(values)
    return dict(n=len(values), median=statistics.median(values),
                minimum=min(values), maximum=max(values),
                p95=ordered[int(.95 * (len(ordered) - 1))])


def contention_benchmark():
    """18 owned fixture workers plus main and sampler; no unrelated tasks read."""
    ready = threading.Barrier(19)
    go, measured, release = threading.Event(), threading.Event(), threading.Event()
    records = []
    deadline = []
    completed = []
    def worker(index, busy):
        tid = threading.get_native_id()
        ready.wait()
        go.wait()
        before, cpu = time.monotonic_ns(), time.thread_time_ns()
        if busy:
            while time.monotonic_ns() < deadline[0]:
                sum(range(200))
        else:
            measured.wait(2)
        ended, spent = time.monotonic_ns(), time.thread_time_ns() - cpu
        records.append(dict(native_tid=tid, role='busy' if busy else 'sleeping', index=index,
            measured_wall_begin_ns=before, measured_wall_end_ns=ended,
            exact_thread_cpu_ns=spent))
        completed.append(index)
        release.wait(4)  # Keep identities alive for the post-window counter sample.
    workers = [threading.Thread(target=worker, args=(index, index >= 16),
        name='fixture-busy-' + str(index) if index >= 16 else 'fixture-sleep-' + str(index))
        for index in range(18)]
    recorder = probe.ThreadProbe(capacity=128)
    def rss():
        with open('/proc/self/statm') as stream:
            return int(stream.read().split()[1]) * os.sysconf('SC_PAGE_SIZE')
    try:
        for worker_thread in workers:
            worker_thread.start()
        ready.wait(timeout=3)
        rss_before = rss()
        recorder.start()
        time.sleep(.08)
        begin, process = time.monotonic_ns(), time.process_time_ns()
        deadline.append(begin + 1_000_000_000)
        with recorder.zone('fixture_two_busy_threads', 0):
            go.set()
            time.sleep(1)
            measured.set()
            while len(completed) < 18 and time.monotonic_ns() - begin < 2_000_000_000:
                time.sleep(.01)
        end, process_delta = time.monotonic_ns(), time.process_time_ns() - process
        recorder.freeze('owned_18_worker_contention_fixture')
        time.sleep(.15)
        # Saturate the bounded zone ring with the same 20 registered tasks,
        # outside the measured workload. No duplicate CPU snapshot is attached
        # to the periodic task rows; these are independent bounded records.
        for _ in range(probe.ZONE_CAPACITY+5):
            with recorder.zone('fixture_fill_zone_retention'):
                pass
        rss_after = rss()
        snapshot = recorder.stop()
    finally:
        measured.set()
        go.set()
        release.set()
        recorder.stop()
        for worker_thread in workers:
            worker_thread.join(timeout=2)
    roles = {item['native_tid']: item for item in records}
    observed = {}
    for row in snapshot['rows']:
        for task in row['threads']:
            if task['native_tid'] in roles and task.get('schedstat') is not None:
                observed.setdefault(task['native_tid'], []).append(task)
    comparison = []
    for tid, exact in sorted(roles.items()):
        tasks = observed.get(tid, [])
        measured_delta = None
        if len(tasks) >= 2 and tasks[0]['starttime_ticks'] == tasks[-1]['starttime_ticks']:
            measured_delta = tasks[-1]['schedstat']['runtime_ns'] - tasks[0]['schedstat']['runtime_ns']
        comparison.append(dict(exact, observed_samples=len(tasks),
            observed_schedstat_delta_ns=measured_delta,
            observed_minus_exact_ns=None if measured_delta is None else measured_delta-exact['exact_thread_cpu_ns'],
            first_read_begin_ns=tasks[0]['schedstat_wall_begin_ns'] if tasks else None,
            last_read_end_ns=tasks[-1]['schedstat_wall_end_ns'] if tasks else None,
            exact_interval_contained=bool(tasks and tasks[0]['schedstat_wall_end_ns'] <= exact['measured_wall_begin_ns']
                and tasks[-1]['schedstat_wall_begin_ns'] >= exact['measured_wall_end_ns'])))
    busy_rows = [row for row in snapshot['rows'] if row['wall_begin_ns'] < end and row['wall_end_ns'] > begin]
    window_end = begin + 400_000_000
    stack_in_400ms = [row for row in snapshot['rows']
        if row['python_frames_capture_begin_ns'] >= begin and row['python_frames_capture_end_ns'] <= window_end]
    return dict(fixture_workers=18, sleeping_workers=16, busy_workers=2,
        total_expected_tasks_including_main_and_sampler=20,
        exact_interval_wall_begin_ns=begin, exact_interval_wall_end_ns=end,
        measured_process_cpu_ns=process_delta,
        exact_worker_cpu_sum_ns=sum(row['exact_thread_cpu_ns'] for row in records),
        rss_before_probe_bytes=rss_before, rss_after_full_zone_ring_bytes=rss_after,
        rss_delta_bytes=rss_after-rss_before,
        rss_basis='Same process, 18 fixture workers already created at baseline; after 1024-record zone ring filled. '
                  'Excludes JSON string construction, includes Python allocator and all observer state; observed, not RSS bound.',
        serialized_snapshot_bytes=len(json.dumps(snapshot, separators=(',', ':')).encode()),
        comparisons=comparison, stop_complete=snapshot['stop_complete'],
        all_fixture_workers_joined=all(not worker_thread.is_alive() for worker_thread in workers),
        sample_wall_ns_during_load=distribution([row['wall_end_ns']-row['wall_begin_ns'] for row in busy_rows]),
        complete_python_stack_captures_first_400ms=len(stack_in_400ms),
        missed_intervals=snapshot['missed_intervals'],
        sampler_lifetime_cpu_ns=snapshot['sampler_lifetime_cpu_ns'],
        boundary_clock_reads=snapshot['boundary_clock_reads'],
        boundary_clock_unknown=snapshot['boundary_clock_unknown'],
        boundary_wall_ns=snapshot['boundary_wall_ns'], boundary_cpu_ns=snapshot['boundary_cpu_ns'],
        zone_recorded=snapshot['zone_recorded'], zone_evicted=snapshot['zone_evicted'],
        zone_cpu_evidence=next((row for row in snapshot['slow_zones']
            if row['stage']=='fixture_two_busy_threads'), None),
        errors=snapshot['errors'], rows=snapshot['rows'],
        caveat='Exact fixture thread CPU covers its stated interval; proc deltas cover broader bracketed reads. '
               'Positive deltas include setup/wakeup/parking outside the exact interval. Sparse Python sampling '
               'does not establish absence of activity between frames, native-stack locations, or GIL ownership.')


def benchmark():
    """Finite no-ROS measurement, idle and CPU-busy Python peer, both intervals.

    Measures sampler's own CPU including recording. Includes deadline misses:
    a blocked Python sampler is not independent evidence of GIL ownership.
    """
    result = dict(schema='r89_thread_probe_overhead/v1', pid=os.getpid(),
        source_sha256=hashlib.sha256(Path(probe.__file__).read_bytes()).hexdigest(),
        cpu_affinity=sorted(os.sched_getaffinity(0)), python=sys.version, executable=sys.executable,
        no_ros_or_simulator=True, observed_not_performance_bound=True,
        switched_behavior=False, cases=[])
    for interval in (50_000_000, 20_000_000):
        for busy in (False, True):
            recorder = probe.ThreadProbe(interval_ns=interval, capacity=128)
            start_wall, start_process = time.monotonic_ns(), time.process_time_ns()
            recorder.start()
            begin = time.monotonic_ns()
            if busy:
                with recorder.zone('fixture_cpu_loop', 0):
                    while time.monotonic_ns() - begin < 1_000_000_000:
                        sum(range(200))
            else:
                time.sleep(1)
            recorder.freeze('benchmark_end')
            snapshot = recorder.stop()
            elapsed = time.monotonic_ns() - start_wall
            rows = snapshot['rows']
            result['cases'].append(dict(interval_ns=interval, python_peer_busy=busy,
                elapsed_wall_ns=elapsed, process_cpu_ns=time.process_time_ns()-start_process,
                sampler_lifetime_cpu_ns=snapshot['sampler_lifetime_cpu_ns'],
                sampler_cpu_percent_of_wall=100*snapshot['sampler_lifetime_cpu_ns']/elapsed,
                sample_wall_ns=distribution([row['wall_end_ns']-row['wall_begin_ns'] for row in rows]),
                sample_cpu_ns=distribution([row['sampler_thread_cpu_ns'] for row in rows]),
                scheduled_lateness_ns=distribution([row['wall_begin_ns']-row['scheduled_ns'] for row in rows]),
                missed_intervals=snapshot['missed_intervals'],
                stop_complete=snapshot['stop_complete'], errors=snapshot['errors'],
                startup=snapshot['startup'],
                retained_json_bytes=len(json.dumps(snapshot, separators=(',', ':')).encode())))
    raw = {}
    for enabled in (False, True):
        recorder = probe.ThreadProbe(capacity=4) if enabled else None
        samples = []
        with patch.object(probe, 'PROBE', recorder):
            for _ in range(5):
                cpu = time.thread_time_ns()
                for index in range(10000):
                    with probe.zone('benchmark_zone', index):
                        pass
                samples.append((time.thread_time_ns()-cpu)/10000)
        raw['on' if enabled else 'off'] = distribution(samples)
    result['zone_thread_cpu_ns_per_call'] = raw
    result['owned_18_worker_contention'] = contention_benchmark()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path)
    parser.add_argument('--skip-tests', action='store_true')
    args = parser.parse_args()
    if not args.skip_tests:
        result = unittest.TextTestRunner(verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(ThreadProbeTests))
        if not result.wasSuccessful():
            raise SystemExit(1)
    if args.benchmark:
        report = benchmark()
        args.benchmark.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print('Self-only finite benchmark:', args.benchmark)


if __name__ == '__main__':
    main()
