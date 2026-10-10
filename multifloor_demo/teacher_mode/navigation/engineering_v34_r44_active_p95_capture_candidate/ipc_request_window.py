"""Observe original request operations; never cache or replace their results."""
from __future__ import annotations

import argparse
import collections
from contextlib import contextmanager
import gc
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
from gc_startup_partition import MAX_STARTUP_METADATA_BYTES, STARTUP_CONTRACT, StartupPartition

CONTRACT = {
    "schema": "teacher_original_request_substages/v3",
    "maximum_file_bytes": 512 * 1024,
    "maximum_recent_bytes": 256 * 1024,
    "maximum_recent_records": 128,
    "maximum_critical_bytes": 128 * 1024,
    "maximum_critical_records": 4,
    "maximum_record_bytes": 32 * 1024,
    "maximum_operation_groups": 16,
    "capture_mode": "original_operations_forwarded_then_post_worker_exit_file",
    "native_watchdog_ms": 200,
    "source_deadline_ms": 300,
    "original_source_checks_or_reads_skipped": False,
    "original_result_exception_or_control_decisions_replaced": False,
    "original_timing_rows_modified": False,
    "memory_scope": "retained encoded scalar rows; excludes Python objects and original worker RSS",
    "performance_non_regression_verified": False,
    "json_call_ordinal_and_input_length_recorded": True,
    "GC_observer_same_thread_active_frame_only": True,
    "GC_schedule_threshold_or_enabled_modified": True,
    "GC_heap_partition_strategy_modified": True,
    "GC_threshold_or_enabled_modified": False,
    "GC_observer_modifies_GC": False,
    "startup_heap_partition_capability": STARTUP_CONTRACT,
    "maximum_startup_metadata_bytes": MAX_STARTUP_METADATA_BYTES,
    "maximum_GC_generation_groups": 3,
}


class RequestWindow:
    def __init__(self, clock=time.monotonic_ns, cpu_clock=time.thread_time_ns):
        self.clock = clock
        self.cpu_clock = cpu_clock
        self.frame = None
        self.active = False
        self.recent = collections.deque()
        self.critical = collections.deque()
        self.recent_bytes = self.critical_bytes = 0
        self.attempted = self.evicted = self.oversize = self.diagnostic_errors = 0
        self.timers_created = 0
        self.owner_thread_id = threading.get_ident()
        self.pending_gc = {}
        self.ignored_foreign_gc = 0
        self.startup_partition = None

    def begin(self, sequence):
        self.active = False
        self.frame = {"schema": CONTRACT["schema"], "frame_sequence": sequence,
                      "native_clock_ns": None, "operations": {}, "request_calls": 0,
                      "json_load_calls": 0, "GC_generations": {}}

    def observe_gc(self, phase, info):
        """Record actual default-GC callbacks; never request or defer collection."""
        try:
            if threading.get_ident() != self.owner_thread_id:
                self.ignored_foreign_gc += 1
                return
            generation = info['generation']
            if type(generation) is not int or generation not in (0, 1, 2):
                return
            if phase == 'start':
                if self.frame is not None:
                    self.pending_gc[generation] = (self.frame['frame_sequence'],
                        self.clock(), self.cpu_clock(), self.active)
            elif phase == 'stop' and generation in self.pending_gc:
                sequence, wall, cpu, active = self.pending_gc.pop(generation)
                if self.frame is None or self.frame['frame_sequence'] != sequence:
                    return
                wall, cpu = self.clock() - wall, self.cpu_clock() - cpu
                row = self.frame['GC_generations'].setdefault(str(generation),
                    dict(calls=0, total_wall_ns=0, total_thread_cpu_ns=0,
                         maximum_wall_ns=0, request_active_calls=0,
                         collected=0, uncollectable=0))
                row['calls'] += 1
                row['total_wall_ns'] += wall
                row['total_thread_cpu_ns'] += cpu
                row['maximum_wall_ns'] = max(row['maximum_wall_ns'], wall)
                row['request_active_calls'] += int(active)
                row['collected'] += info['collected']
                row['uncollectable'] += info['uncollectable']
        except Exception:
            self.diagnostic_errors += 1

    def received(self, clock_ns):
        if self.frame is not None:
            self.frame["native_clock_ns"] = clock_ns

    @contextmanager
    def request_scope(self):
        if self.frame is None:
            yield
            return
        try:
            start, cpu_start = self.clock(), self.cpu_clock()
        except Exception:
            self.diagnostic_errors += 1
            yield
            return
        self.active = True
        self.frame["request_calls"] += 1
        self.frame["request_start_wall_ns"] = start
        try:
            yield
        finally:
            self.active = False
            try:
                self.frame["request_wall_ns"] = self.clock() - start
                self.frame["request_thread_cpu_ns"] = self.cpu_clock() - cpu_start
            except Exception:
                self.diagnostic_errors += 1

    def call(self, group, target, function, *args, **kwargs):
        if not self.active or self.frame is None:
            return function(*args, **kwargs)
        try:
            start = self.clock()
        except Exception:
            self.diagnostic_errors += 1
            return function(*args, **kwargs)
        error = False
        try:
            return function(*args, **kwargs)
        except BaseException:
            error = True
            raise
        finally:
            try:
                elapsed = self.clock() - start
                group = str(group)[:64]
                if group not in self.frame["operations"] and len(self.frame["operations"]) >= CONTRACT["maximum_operation_groups"]:
                    raise ValueError("Optional operation group budget exhausted")
                row = self.frame["operations"].setdefault(group,
                    {"calls": 0, "total_wall_ns": 0, "maximum_wall_ns": 0, "errors": 0})
                row["calls"] += 1
                row["total_wall_ns"] += elapsed
                row["errors"] += int(error)
                if elapsed >= row["maximum_wall_ns"]:
                    row["maximum_wall_ns"] = elapsed
                    row["maximum_target"] = str(target)[:120]
            except Exception:
                self.diagnostic_errors += 1

    def finish(self, outcome, response_sent):
        frame, self.frame = self.frame, None
        self.active = False
        if frame is None:
            return
        self.attempted += 1
        frame.update(outcome=str(outcome)[:80], response_sent=response_sent,
                     operation_totals_overlap=True, full_run_capture_complete=False)
        body = (json.dumps(frame, separators=(",", ":"), allow_nan=False) + "\n").encode("ascii")
        if len(body) > CONTRACT["maximum_record_bytes"]:
            self.oversize += 1
            return
        item = (frame["frame_sequence"], body)
        while self.recent and (self.recent_bytes + len(body) > CONTRACT["maximum_recent_bytes"] or
                               len(self.recent) >= CONTRACT["maximum_recent_records"]):
            self.recent_bytes -= len(self.recent.popleft()[1])
            self.evicted += 1
        self.recent.append(item)
        self.recent_bytes += len(body)
        if not response_sent or frame.get("request_wall_ns", 0) >= 100_000_000:
            while self.critical and (self.critical_bytes + len(body) > CONTRACT["maximum_critical_bytes"] or
                                     len(self.critical) >= CONTRACT["maximum_critical_records"]):
                self.critical_bytes -= len(self.critical.popleft()[1])
            self.critical.append(item)
            self.critical_bytes += len(body)

    def save(self, run, sources):
        unique = dict((*self.recent, *self.critical))
        rows = [unique[k] for k in sorted(unique)]
        header = dict(kind="header", run=str(run), contract=CONTRACT,
                      source_sha256=sources, timers_created=self.timers_created)
        footer = dict(kind="footer", attempted=self.attempted, evicted=self.evicted,
                      oversize=self.oversize, diagnostic_errors=self.diagnostic_errors,
                      saved_records=len(rows), sequence_min=min(unique) if unique else None,
                      sequence_max=max(unique) if unique else None,
                      original_scalar_rows_sha256=hashlib.sha256(b"".join(rows)).hexdigest(),
                      bounded_window_final=True, full_run_capture_complete=False,
                      native_parking_verified=False)
        header['GC_observer'] = dict(owner_thread_id=self.owner_thread_id,
            actual_thresholds=list(gc.get_threshold()), actual_enabled=gc.isenabled(),
            schedule_modified_by_observer=False, foreign_threads_ignored=True)
        footer['ignored_foreign_thread_GC_callbacks'] = self.ignored_foreign_gc
        if self.startup_partition is not None:
            startup, final = self.startup_partition.receipt_parts()
            header['startup_heap_partition'] = startup
            footer['startup_heap_partition_final'] = final
        encode = lambda x: (json.dumps(x, separators=(",", ":"), allow_nan=False) + "\n").encode()
        payload = encode(header) + b"".join(rows) + encode(footer)
        if len(payload) > CONTRACT["maximum_file_bytes"]:
            raise ValueError("Request-substage file exceeds declared bound")
        path = Path(run) / "ipc_request_substages.jsonl"
        with path.open("xb") as stream:
            if stream.write(payload) != len(payload):
                raise OSError("Partial request-substage write")
        return path


def observed_path(original, capture):
    class ObservedPath(type(original())):
        def stat(self, *args, **kwargs):
            return capture.call("worker.Path.stat", self.name, super().stat, *args, **kwargs)

        def read_bytes(self, *args, **kwargs):
            label = self.name if self.name == "navigation_command.json" else "other"
            return capture.call("worker.Path.read_bytes/" + label, self.name,
                                super().read_bytes, *args, **kwargs)

        def read_text(self, *args, **kwargs):
            label = self.name if self.name in ("navigation_status.json", "mission46_status.json") else "other"
            return capture.call("worker.Path.read_text/" + label, self.name,
                                super().read_text, *args, **kwargs)
    return ObservedPath


@contextmanager
def observe_default_gc(capture):
    callback = capture.observe_gc
    gc.callbacks.append(callback)
    try:
        yield
    finally:
        # Remove only this exact addition, retaining pre-existing and newly
        # registered callbacks and their order.
        for index in range(len(gc.callbacks) - 1, -1, -1):
            if gc.callbacks[index] is callback:
                del gc.callbacks[index]
                break


@contextmanager
def instrument(worker, timing_module, provider_class, recording_class, capture):
    restored = []

    def replace(obj, name, value):
        owned = name in vars(obj)
        old = getattr(obj, name)
        restored.append((obj, name, old, owned))
        setattr(obj, name, value)

    def wrapped_method(original, label):
        def observed(self, *args, **kwargs):
            target = args[0] if label == "terrain._read_json" and args else label
            return capture.call(label, target, original, self, *args, **kwargs)
        return observed

    original_timer = timing_module.FrameTiming

    class ObservedTimer(original_timer):
        def __init__(self, writer):
            super().__init__(writer)
            capture.timers_created += 1

        def begin_receive(self):
            result = super().begin_receive()
            try:
                capture.begin(self.sequence)
            except Exception:
                capture.diagnostic_errors += 1
            return result

        def received(self, clock_ns):
            result = super().received(clock_ns)
            try:
                capture.received(clock_ns)
            except Exception:
                capture.diagnostic_errors += 1
            return result

        @contextmanager
        def measure(self, name):
            with super().measure(name):
                if name == "command_request_and_status":
                    with capture.request_scope():
                        yield
                else:
                    yield

        def finish(self, outcome, error=None, *, response_sent=False):
            present = self.frame is not None
            try:
                return super().finish(outcome, error, response_sent=response_sent)
            finally:
                if present:
                    try:
                        capture.finish(outcome, response_sent)
                    except Exception:
                        capture.diagnostic_errors += 1

    original_json = worker.json

    class ObservedJSON:
        def __getattr__(self, name):
            return getattr(original_json, name)

        def loads(self, *args, **kwargs):
            if not capture.active or capture.frame is None:
                return original_json.loads(*args, **kwargs)
            try:
                capture.frame['json_load_calls'] += 1
                ordinal = capture.frame['json_load_calls']
                value = args[0] if args else kwargs.get('s')
                size = len(value) if type(value) in (str, bytes, bytearray) else None
                label = 'ordinal=%s;input_length=%s' % (ordinal, size)
            except Exception:
                capture.diagnostic_errors += 1
                return capture.call("worker.json.loads", "loads", original_json.loads, *args, **kwargs)
            return capture.call("worker.json.loads", "loads",
                capture.call, "worker.json.loads/" + str(ordinal), label,
                original_json.loads, *args, **kwargs)

    original_canonical = worker.canonical
    try:
        replace(worker, "Path", observed_path(worker.Path, capture))
        replace(worker, "json", ObservedJSON())
        replace(worker, "canonical", lambda *a, **k:
                capture.call("worker.canonical", "canonical", original_canonical, *a, **k))
        replace(timing_module, "FrameTiming", ObservedTimer)
        for name in ("check_switch", "_read_json", "evidence"):
            replace(provider_class, name, wrapped_method(getattr(provider_class, name), "terrain." + name))
        replace(recording_class, "project_evidence",
                wrapped_method(recording_class.project_evidence, "recording.project_evidence"))
        yield
    finally:
        for obj, name, old, owned in reversed(restored):
            if owned:
                setattr(obj, name, old)
            else:
                delattr(obj, name)


def main(*, owned_cli=False):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--run", type=Path, required=True)
    args, _ = parser.parse_known_args()
    import ipc_evidence
    import ipc_fault_window
    import worker
    from mission46_terrain import MissionTerrainProvider
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from evidence_runtime_v32.worker_logging import RecordingContext
    capture = RequestWindow()
    partition = StartupPartition.for_run(args.run, owned_cli=owned_cli)
    capture.startup_partition = partition
    try:
        with observe_default_gc(capture), instrument(worker, ipc_evidence, MissionTerrainProvider, RecordingContext, capture):
            return partition.run(ipc_fault_window.main)
    finally:
        try:
            here = Path(__file__).resolve().parent
            names = ("worker.py", "ipc_evidence.py", "ipc_fault_window.py", "ipc_request_window.py",
                     "mission46_terrain.py", "terrain_provider.py", "gc_startup_partition.py")
            sources = {str(here / n): hashlib.sha256((here / n).read_bytes()).hexdigest() for n in names}
            other = here.parent / "evidence_runtime_v32/worker_logging.py"
            sources[str(other)] = hashlib.sha256(other.read_bytes()).hexdigest()
            capture.save(args.run, sources)
        except Exception as error:
            try:
                print("Request-substage evidence unavailable: " + type(error).__name__ + ": " + str(error), file=sys.stderr)
            except BaseException:
                pass


if __name__ == "__main__":
    main(owned_cli=True)
