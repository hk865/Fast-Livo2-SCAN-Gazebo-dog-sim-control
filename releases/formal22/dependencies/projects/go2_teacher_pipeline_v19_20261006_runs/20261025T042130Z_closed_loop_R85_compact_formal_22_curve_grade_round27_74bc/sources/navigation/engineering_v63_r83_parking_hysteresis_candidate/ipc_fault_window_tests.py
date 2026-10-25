"""Finite scalar diagnostics tests; no Actor, ROS, native IPC or simulation."""
import hashlib
import collections
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import ipc_evidence
import ipc_fault_window
from ipc_fault_window import CONTRACT, FaultWindow, RecordingWriter, run_original_worker


def row(seq, failed=False, pad=0):
    return json.dumps(dict(schema="teacher_ipc_frame_wall_timing/v1", frame_sequence=seq,
                          response_sent=not failed, outcome="response_send_failed" if failed else "response_sent",
                          native_clock_ns=seq * 20000000, padding="x" * pad),
                      separators=(",", ":")) + "\n"


class Writer:
    def __init__(self, admitted=True):
        self.rows = []
        self.admitted = admitted
    def submit(self, text, *, sequence=None):
        self.rows.append((text, sequence))
        return self.admitted


class FaultWindowTests(unittest.TestCase):
    def test_full_original_prefix_does_not_omit_terminal_failure(self):
        capture = FaultWindow()
        original = Writer(admitted=False)
        wrapped = RecordingWriter(original, capture)
        for seq in range(2000):
            self.assertIs(wrapped.submit(row(seq), sequence=seq), False)
        failed = row(2000, True)
        self.assertIs(wrapped.submit(failed, sequence=2000), False)
        data = capture.payload("synthetic", {})
        self.assertIn(failed.encode(), data)
        self.assertEqual(original.rows[-1], (failed, 2000))
        footer = json.loads(data.splitlines()[-1])
        self.assertEqual(footer["saved_sequence_max"], 2000)
        self.assertFalse(footer["full_run_capture_complete"])

    def test_whole_rows_and_memory_and_disk_bounds(self):
        capture = FaultWindow()
        for seq in range(1000):
            capture.record(row(seq, seq % 90 == 0, 31000), seq)
        self.assertLessEqual(capture.recent_bytes, CONTRACT["maximum_recent_bytes"])
        self.assertLessEqual(capture.critical_bytes, CONTRACT["maximum_critical_bytes"])
        self.assertLessEqual(len(capture.recent), CONTRACT["maximum_recent_records"])
        self.assertLessEqual(len(capture.critical), CONTRACT["maximum_critical_records"])
        capture.record(row(1000, True, 40000), 1000)
        self.assertEqual(capture.oversize, 1)
        payload = capture.payload("synthetic", {})
        self.assertLessEqual(len(payload), CONTRACT["maximum_file_bytes"])
        for line in payload.splitlines():
            json.loads(line)
        with tempfile.TemporaryDirectory() as directory:
            target = capture.save(directory, {})
            self.assertLessEqual(target.stat().st_size, CONTRACT["maximum_file_bytes"])
            self.assertLessEqual(target.stat().st_blocks * 512, CONTRACT["maximum_file_bytes"])
            with self.assertRaises(FileExistsError):
                capture.save(directory, {})

    def test_original_result_exception_and_recording_failure_preserved(self):
        capture = FaultWindow()
        wrapped = RecordingWriter(Writer(), capture)
        with patch.object(capture, "record", side_effect=OSError("synthetic optional error")):
            self.assertIs(wrapped.submit(row(1), sequence=1), True)
        self.assertEqual(capture.record_errors, 1)
        original = Writer()
        with patch.object(original, "submit", side_effect=BrokenPipeError("original failure")):
            with self.assertRaisesRegex(BrokenPipeError, "original failure"):
                RecordingWriter(original, capture).submit(row(2, True), sequence=2)
        self.assertEqual(capture.critical[-1][0], 2)

    def test_frozen_frame_timing_row_is_byte_identical(self):
        def exercise(writer):
            timer = ipc_evidence.FrameTiming(writer)
            counter = iter(range(100, 1000, 10))
            with patch.object(ipc_evidence.time, "monotonic_ns", side_effect=lambda: next(counter)):
                timer.begin_receive()
                timer.received(1230000000)
                with timer.measure("synthetic_inference"):
                    pass
                timer.finish("response_sent", response_sent=True)
            return writer
        baseline = exercise(Writer())
        capture = FaultWindow()
        actual = Writer()
        exercise(RecordingWriter(actual, capture))
        self.assertEqual(actual.rows, baseline.rows)
        self.assertEqual(capture.recent[-1][1], baseline.rows[-1][0].encode())

    def test_worker_failure_restores_original_timer(self):
        module = types.SimpleNamespace(FrameTiming=ipc_evidence.FrameTiming)
        original = module.FrameTiming
        capture = FaultWindow()
        def fake_worker():
            timer = module.FrameTiming(Writer())
            timer.begin_receive()
            timer.received(999)
            timer.finish("response_send_failed", BrokenPipeError("synthetic"))
            raise BrokenPipeError("synthetic worker error")
        with self.assertRaisesRegex(BrokenPipeError, "synthetic worker error"):
            run_original_worker(fake_worker, module, capture)
        self.assertIs(module.FrameTiming, original)
        self.assertEqual(capture.timers_created, 1)
        self.assertEqual(len(capture.critical), 1)

    def test_saved_digest_and_no_producer_disk_io(self):
        capture = FaultWindow()
        with patch.object(Path, "open", side_effect=AssertionError("hot path disk I/O")):
            for seq in range(150):
                capture.record(row(seq), seq)
            capture.record(row(150, True), 150)
        payload = capture.payload("synthetic", {"example": "abc"})
        lines = payload.splitlines(keepends=True)
        footer = json.loads(lines[-1])
        self.assertEqual(footer["original_record_bytes_sha256"], hashlib.sha256(b"".join(lines[1:-1])).hexdigest())
        self.assertEqual(json.loads(lines[-2])["frame_sequence"], 150)

    def test_payload_bound_holds_at_each_append(self):
        peaks = {"recent": 0, "critical": 0}
        class ProbedDeque(collections.deque):
            def __init__(self, name):
                super().__init__()
                self.name = name
            def append(self, item):
                super().append(item)
                peaks[self.name] = max(peaks[self.name], sum(len(body) for _, body in self))
        capture = FaultWindow()
        capture.recent = ProbedDeque("recent")
        capture.critical = ProbedDeque("critical")
        for seq in range(20):
            capture.record(row(seq, True, 32000), seq)
        self.assertLessEqual(peaks["recent"], CONTRACT["maximum_recent_bytes"])
        self.assertLessEqual(peaks["critical"], CONTRACT["maximum_critical_bytes"])

    def test_main_keeps_worker_exception_when_evidence_and_stderr_fail(self):
        def fake_worker():
            raise RuntimeError("original worker exception")
        fake_module = types.SimpleNamespace(main=fake_worker)
        with patch.dict("sys.modules", {"worker": fake_module}), \
             patch("sys.argv", ["ipc_fault_window.py", "--run", "/synthetic/run"]), \
             patch.object(Path, "read_bytes", return_value=b"synthetic source"), \
             patch.object(FaultWindow, "save", side_effect=OSError("synthetic write failure")), \
             patch("builtins.print", side_effect=BrokenPipeError("synthetic stderr failure")):
            with self.assertRaisesRegex(RuntimeError, "original worker exception"):
                ipc_fault_window.main()


if __name__ == "__main__":
    unittest.main()
