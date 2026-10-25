"""T4: default-off slow-guard window probe, no Node/ROS/physical state."""
import json
import threading
import time
import unittest
from unittest.mock import patch

import r86_stage_probe as probe


class SampleTests(unittest.TestCase):
    def test_sample_schema_and_optional_counter_failures(self):
        row = probe.sample()
        self.assertEqual(row['schema'], 'r86_execution_sample/v1')
        self.assertIsInstance(row['wall_begin_ns'], int)
        self.assertIsInstance(row['wall_end_ns'], int)
        self.assertGreaterEqual(row['wall_end_ns'], row['wall_begin_ns'])
        self.assertIsInstance(row['native_tid'], int)
        self.assertIsInstance(row['thread_cpu_ns'], int)
        self.assertIsInstance(row['process_cpu_ns'], int)
        self.assertIsInstance(row['thread_usage']['nvcsw'], int)
        self.assertEqual(row['errors'], {})
        with patch.object(probe, 'resource', None):
            degraded = probe.sample()
        self.assertIsNone(degraded['thread_usage'])
        self.assertIn('thread_usage', degraded['errors'])
        self.assertIsInstance(degraded['wall_end_ns'], int)

    def test_thread_snapshot_is_bounded_and_gc_snapshot_is_scalar(self):
        threads = probe._threads()
        self.assertIsNotNone(threads)
        self.assertLessEqual(len(threads), probe.MAX_THREADS)
        row = next(iter(threads.values()))
        self.assertIn('utime_ticks', row)
        self.assertIn('nvcsw', row)
        before = probe._threads()[threading.get_native_id()]
        for _ in range(20):
            time.sleep(.0005)
        after = probe._threads()[threading.get_native_id()]
        self.assertIsNotNone(before['nvcsw'])
        self.assertGreaterEqual(after['nvcsw'] - before['nvcsw'], 5)
        trees = probe._gc()
        self.assertEqual(len(trees['stats']), 3)
        self.assertEqual(len(trees['count']), 3)
        json.dumps(dict(threads=threads, gc=trees), allow_nan=False)

    def test_dump_parser_is_bounded(self):
        text = ''.join('Thread 0x%x (most recent call first):\n  File "x.py", line 1\n\n'
                       % index for index in range(300))
        parsed = probe._parse_dump(text.encode())
        self.assertEqual(parsed['block_count'], 100)
        self.assertIsNone(parsed['text'])
        big = b'x' * (probe.GUARD_DUMP_MAX_BYTES + 4096)
        self.assertTrue(probe._parse_dump(big)['truncated'])
        self.assertLessEqual(probe._parse_dump(big)['dump_bytes'],
                             probe.GUARD_DUMP_MAX_BYTES)


class WindowTests(unittest.TestCase):
    def test_off_by_default_is_a_noop(self):
        with patch.object(probe, 'ENABLED', False):
            self.assertIsNone(probe.guard_window_begin())
            self.assertIsNone(probe.guard_window_end(None))

    def test_slow_guard_captures_threads_gc_and_bounded_stacks(self):
        holder = threading.Lock()
        ready, go = threading.Event(), threading.Event()
        def peer():
            holder.acquire()
            ready.set()
            go.wait(1)
            started = time.monotonic()
            while time.monotonic() - started < .15:
                pass
            holder.release()
            time.sleep(.5)
        thread = threading.Thread(target=peer, daemon=True)
        thread.start()
        self.assertTrue(ready.wait(1))
        with patch.object(probe, 'ENABLED', True):
            window = probe.guard_window_begin()
            self.assertIsNotNone(window)
            self.assertTrue(window['armed'])
            # Main thread waits on a lock held by the CPU-burning peer: wall
            # time is large, its own thread CPU stays small, peer CPU is large.
            go.set()
            holder.acquire()
            holder.release()
            call = {}
            summary = probe.guard_window_end(window, call)
        thread.join(2)
        self.assertIs(summary, call['r86_guard_window'])
        self.assertGreaterEqual(summary['wall_ns'], 100_000_000)
        self.assertLess(summary['thread_cpu_ns'], 80_000_000)
        self.assertIsNotNone(summary['thread_delta'])
        self.assertIsNotNone(summary['gc_delta'])
        peer_tid = str(thread.native_id)
        self.assertIn(peer_tid, summary['thread_delta'])
        self.assertGreater(summary['thread_delta'][peer_tid]['cpu_ns'], 100_000_000)
        stacks = summary['stacks']
        self.assertGreaterEqual(stacks['block_count'], 1)
        self.assertLessEqual(stacks['block_count'], probe.GUARD_DUMP_MAX_BLOCKS)
        self.assertIn('Thread 0x', stacks['text'])
        self.assertIn('File "', stacks['text'])
        self.assertLessEqual(len(probe.recent_windows()), probe.RECENT_WINDOWS)

    def test_fast_guard_emits_no_stack_dump(self):
        with patch.object(probe, 'ENABLED', True):
            window = probe.guard_window_begin()
            summary = probe.guard_window_end(window)
        self.assertLess(summary['wall_ns'], probe.GUARD_FIRST_DUMP_S * 1e9)
        self.assertEqual(summary['stacks']['block_count'], 0)
        self.assertEqual(summary['stacks']['dump_bytes'], 0)


if __name__ == '__main__':
    unittest.main()
