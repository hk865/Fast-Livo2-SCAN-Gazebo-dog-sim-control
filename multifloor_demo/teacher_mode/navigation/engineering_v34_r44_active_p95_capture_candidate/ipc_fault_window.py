"""Scalar-only IPC end window; the frozen worker still owns all decisions.

The runner explicitly selects this diagnostic entrypoint. Original timing rows
and their writer admission result are forwarded unchanged. This recorder does
no disk I/O during the actuator loop. A small end window is written only after
the original worker's finally block; it cannot establish full-run coverage.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
from pathlib import Path
import sys

CONTRACT = {
    "schema": "teacher_scalar_ipc_fault_window/v1",
    "maximum_file_bytes": 512 * 1024,
    "maximum_recent_bytes": 256 * 1024,
    "maximum_recent_records": 128,
    "maximum_critical_bytes": 128 * 1024,
    "maximum_critical_records": 4,
    "maximum_record_bytes": 32 * 1024,
    "maximum_temporary_record_bytes": 32 * 1024,
    "memory_scope": "retained scalar bytes and one bounded encoded row; Python object and original worker memory excluded",
    "capture_mode": "bounded_memory_then_post_worker_exit_file",
    "native_watchdog_ms": 200,
    "source_deadline_ms": 300,
    "full_run_capture_claimed": False,
    "algorithm_protocol_or_actuator_decisions_changed": False,
}


class FaultWindow:
    def __init__(self):
        self.recent = collections.deque()
        self.critical = collections.deque()
        self.recent_bytes = self.critical_bytes = 0
        self.attempted = self.oversize = self.record_errors = self.evicted = 0
        self.timers_created = 0

    def record(self, text, sequence):
        self.attempted += 1
        if isinstance(text, (str, bytes)) and len(text) > CONTRACT["maximum_record_bytes"]:
            self.oversize += 1
            return
        # Frozen FrameTiming uses json.dumps's default ensure_ascii=True.
        # Rejecting a foreign encoding is optional diagnostics failure only.
        body = text.encode("ascii") if isinstance(text, str) else text
        if not isinstance(body, bytes):
            raise TypeError("Timing row must be immutable bytes")
        if len(body) > CONTRACT["maximum_record_bytes"]:
            self.oversize += 1
            return
        item = (sequence, body)
        while self.recent and (self.recent_bytes + len(body) > CONTRACT["maximum_recent_bytes"] or
                               len(self.recent) >= CONTRACT["maximum_recent_records"]):
            self.recent_bytes -= len(self.recent.popleft()[1])
            self.evicted += 1
        self.recent.append(item)
        self.recent_bytes += len(body)
        # Frozen FrameTiming emits compact JSON with this exact boolean field.
        # The original immutable row is kept, rather than reconstructed.
        if b'"response_sent":false' in body:
            while self.critical and (self.critical_bytes + len(body) > CONTRACT["maximum_critical_bytes"] or
                                     len(self.critical) >= CONTRACT["maximum_critical_records"]):
                self.critical_bytes -= len(self.critical.popleft()[1])
            self.critical.append(item)
            self.critical_bytes += len(body)

    def payload(self, run, source_hashes):
        unique = {}
        conflicts = 0
        for seq, body in (*self.recent, *self.critical):
            if seq in unique and unique[seq] != body:
                conflicts += 1
            unique[seq] = body
        records = [unique[k] for k in sorted(unique)]
        digest = hashlib.sha256(b"".join(records)).hexdigest()
        header = dict(kind="header", contract=CONTRACT, run=str(run),
                      source_sha256=source_hashes, timers_created=self.timers_created)
        footer = dict(kind="footer", attempted=self.attempted, evicted=self.evicted,
                      oversize=self.oversize, diagnostic_record_errors=self.record_errors,
                      saved_records=len(records), sequence_conflicts=conflicts,
                      saved_sequence_min=min(unique) if unique else None,
                      saved_sequence_max=max(unique) if unique else None,
                      original_record_bytes_sha256=digest,
                      bounded_window_final=True, full_run_capture_complete=False,
                      native_parking_verified=False)
        encode = lambda x: (json.dumps(x, separators=(",", ":"), allow_nan=False) + "\n").encode()
        result = encode(header) + b"".join(records) + encode(footer)
        if len(result) > CONTRACT["maximum_file_bytes"]:
            raise ValueError("Fault-window output exceeds its declared bound")
        return result

    def save(self, run, source_hashes):
        payload = self.payload(run, source_hashes)
        path = Path(run) / "ipc_fault_window.jsonl"
        with path.open("xb") as stream:
            if stream.write(payload) != len(payload):
                raise OSError("Partial fault-window write")
        return path


class RecordingWriter:
    def __init__(self, original, capture):
        self.original = original
        self.capture = capture

    def submit(self, text, *, sequence=None):
        try:
            return self.original.submit(text, sequence=sequence)
        finally:
            try:
                self.capture.record(text, sequence)
            except Exception:
                self.capture.record_errors += 1


def run_original_worker(worker_main, timing_module, capture):
    original = timing_module.FrameTiming

    class RecordingTimer(original):
        def __init__(self, writer):
            super().__init__(RecordingWriter(writer, capture))
            capture.timers_created += 1

    timing_module.FrameTiming = RecordingTimer
    try:
        return worker_main()
    finally:
        timing_module.FrameTiming = original


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--run", type=Path, required=True)
    args, _ = parser.parse_known_args()
    import ipc_evidence
    import worker
    capture = FaultWindow()
    try:
        return run_original_worker(worker.main, ipc_evidence, capture)
    finally:
        # Optional evidence failure cannot replace an actuator-loop exception.
        try:
            here = Path(__file__).resolve().parent
            hashes = {str(here / name): hashlib.sha256((here / name).read_bytes()).hexdigest()
                      for name in ("worker.py", "ipc_evidence.py", "ipc_fault_window.py")}
            capture.save(args.run, hashes)
        except Exception as error:
            try:
                print("IPC fault-window evidence unavailable: " + type(error).__name__ + ": " + str(error),
                      file=sys.stderr)
            except BaseException:
                pass  # Optional warning cannot replace the worker's termination.


if __name__ == "__main__":
    main()
