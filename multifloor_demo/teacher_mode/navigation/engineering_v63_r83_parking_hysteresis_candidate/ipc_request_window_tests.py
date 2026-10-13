import hashlib
import gc
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import ipc_evidence
import ipc_fault_window
from ipc_request_window import CONTRACT, RequestWindow, instrument, observed_path, observe_default_gc


class RequestWindowTests(unittest.TestCase):
    def fixture(self):
        events = []
        worker = types.SimpleNamespace(Path=Path, json=json, canonical=lambda x: json.dumps(x))

        class ProviderBase:
            def _read_json(self, name):
                events.append(("read", name))
                return {"name": name}, "original-receipt"

            def evidence(self):
                events.append(("evidence",))
                return self.original_evidence

        class Provider(ProviderBase):
            def check_switch(self):
                self._read_json("mission46_terrain_request.json")
                return self.original_switch

        class Recording:
            def project_evidence(self, value):
                events.append(("project", value))
                return value

        return worker, Provider, Recording, events

    def test_every_guard_stat_executes_and_changed_source_still_rejects(self):
        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "source.py"
            f.write_text("original")
            count = []

            class CountingPath(type(Path())):
                def stat(self, *a, **k):
                    count.append(str(self))
                    return super().stat(*a, **k)

            c = RequestWindow()
            O = observed_path(CountingPath, c)
            before = f.stat()
            c.begin(1)
            with c.request_scope():
                changed = []
                for _ in range(745):
                    s = O(f).stat()
                    changed.append((s.st_size, s.st_mtime_ns) != (before.st_size, before.st_mtime_ns))
            self.assertEqual(len(count), 745)
            self.assertFalse(any(changed))
            self.assertEqual(c.frame["operations"]["worker.Path.stat"]["calls"], 745)
            f.write_text("changed original source size")
            with c.request_scope():
                s = O(f).stat()
            self.assertNotEqual((s.st_size, s.st_mtime_ns), (before.st_size, before.st_mtime_ns))
            f.unlink()
            with c.request_scope(), self.assertRaises(FileNotFoundError):
                O(f).stat()
            self.assertEqual(len(count), 747)

    def test_forwarded_objects_side_effects_and_inherited_attributes_restore(self):
        w, P, R, events = self.fixture()
        c = RequestWindow()
        original = {"worker": dict(vars(w)), "provider": dict(vars(P)),
                    "recording": dict(vars(R)), "timer": ipc_evidence.FrameTiming}
        provider = P()
        provider.original_evidence, provider.original_switch = object(), object()
        payload = {"original": 1}
        c.begin(1)
        with instrument(w, ipc_evidence, P, R, c), c.request_scope():
            self.assertIs(provider.check_switch(), provider.original_switch)
            self.assertIs(provider.evidence(), provider.original_evidence)
            self.assertIs(R().project_evidence(payload), payload)
            self.assertEqual(w.json.loads('{"value":1}'), {"value": 1})
            self.assertEqual(w.canonical(payload), json.dumps(payload))
        self.assertEqual(events, [("read", "mission46_terrain_request.json"), ("evidence",), ("project", payload)])
        self.assertEqual(dict(vars(w)), original["worker"])
        self.assertEqual(dict(vars(P)), original["provider"])
        self.assertEqual(dict(vars(R)), original["recording"])
        self.assertIs(ipc_evidence.FrameTiming, original["timer"])

    def test_fresh_reads_no_cache_and_original_exception_identity(self):
        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "navigation_command.json"
            c = RequestWindow()
            O = observed_path(Path, c)
            c.begin(1)
            with c.request_scope():
                f.write_bytes(b"first")
                self.assertEqual(O(f).read_bytes(), b"first")
                f.write_bytes(b"second")
                self.assertEqual(O(f).read_bytes(), b"second")
            error = RuntimeError("original-operation-error")

            def fail():
                raise error

            with c.request_scope():
                with self.assertRaises(RuntimeError) as raised:
                    c.call("failure", "original", fail)
            self.assertIs(raised.exception, error)

    def test_optional_clock_and_group_failures_do_not_override_results_or_errors(self):
        c = RequestWindow()
        c.begin(1)
        token = object()
        error = RuntimeError("original-error")

        def fail():
            raise error

        with c.request_scope():
            for n in range(CONTRACT["maximum_operation_groups"]):
                self.assertIs(c.call(str(n), "target", lambda: token), token)
            self.assertIs(c.call("over_budget", "target", lambda: token), token)
            with self.assertRaises(RuntimeError) as raised:
                c.call("another_over_budget", "target", fail)
            self.assertIs(raised.exception, error)
            c.clock = lambda: (_ for _ in ()).throw(OSError("optional clock failed"))
            self.assertIs(c.call("no_clock", "target", lambda: token), token)
            with self.assertRaises(RuntimeError) as raised:
                c.call("no_clock", "target", fail)
            self.assertIs(raised.exception, error)
        self.assertGreater(c.diagnostic_errors, 0)

    def test_original_timing_rows_are_identical_and_worker_exception_restores_patches(self):
        w, P, R, _ = self.fixture()

        class Writer:
            def __init__(self):
                self.rows = []

            def submit(self, text, *, sequence=None):
                self.rows.append((sequence, text))
                return "original-writer-result"

        def emit(writer):
            timer = ipc_evidence.FrameTiming(writer)
            timer.begin_receive()
            timer.received(123)
            with timer.measure("command_request_and_status"):
                w.json.loads('{"x":1}')
            timer.finish("response_sent", response_sent=True)

        a, b = Writer(), Writer()
        c = RequestWindow(clock=lambda: 1000, cpu_clock=lambda: 1000)
        originals = (w.Path, w.json, w.canonical, ipc_evidence.FrameTiming)
        with patch.object(ipc_evidence.time, "monotonic_ns", return_value=1000):
            emit(a)
            with instrument(w, ipc_evidence, P, R, c):
                capture = ipc_fault_window.FaultWindow()
                ipc_fault_window.run_original_worker(lambda: emit(b), ipc_evidence, capture)
                self.assertEqual(capture.attempted, 1)
            self.assertEqual(a.rows, b.rows)
        error = RuntimeError("original worker fault")
        with self.assertRaises(RuntimeError) as raised:
            with instrument(w, ipc_evidence, P, R, c):
                raise error
        self.assertIs(raised.exception, error)
        self.assertEqual((w.Path, w.json, w.canonical, ipc_evidence.FrameTiming), originals)
        self.assertEqual(c.timers_created, 1)

    def test_bounded_terminal_retention_no_hot_disk_and_no_overwrite(self):
        c = RequestWindow()
        with tempfile.TemporaryDirectory() as td:
            for n in range(1000):
                c.begin(n + 1)
                c.received(n * 20_000_000)
                with c.request_scope():
                    for k in range(CONTRACT["maximum_operation_groups"]):
                        c.call(str(k), "a" * 200, lambda: None)
                c.finish("response_sent", True)
                self.assertLessEqual(c.recent_bytes, CONTRACT["maximum_recent_bytes"])
                self.assertLessEqual(len(c.recent), CONTRACT["maximum_recent_records"])
                self.assertEqual(list(Path(td).iterdir()), [])
            c.begin(1001)
            c.received(20_000_000_000)
            c.finish("response_send_failed", False)
            for n in range(1002, 1300):
                c.begin(n)
                c.finish("response_sent", True)
            f = c.save(td, {})
            raw = f.read_bytes()
            lines = raw.splitlines(keepends=True)
            rows = [json.loads(x) for x in lines]
            self.assertTrue(any(x.get("frame_sequence") == 1001 for x in rows))
            self.assertEqual(hashlib.sha256(b"".join(lines[1:-1])).hexdigest(), rows[-1]["original_scalar_rows_sha256"])
            self.assertLessEqual(len(raw), CONTRACT["maximum_file_bytes"])
            self.assertLessEqual(f.stat().st_blocks * 512, CONTRACT["maximum_file_bytes"])
            with self.assertRaises(FileExistsError):
                c.save(td, {})

    def test_json_decode_failure_remains_original_and_post_exit_save_failure_preserves_worker_fault(self):
        w, P, R, _ = self.fixture()
        c = RequestWindow()
        c.begin(1)
        with instrument(w, ipc_evidence, P, R, c), c.request_scope():
            with self.assertRaises(json.JSONDecodeError):
                w.json.loads("invalid original JSON")
        import ipc_request_window as module
        error = RuntimeError("original worker termination")
        fake_worker = w
        fake_provider = types.SimpleNamespace(MissionTerrainProvider=P)
        fake_recording = types.SimpleNamespace(RecordingContext=R)

        def fail():
            raise error

        class FailingStderr:
            def write(self, *_):
                raise RuntimeError("optional stderr failed")

        with tempfile.TemporaryDirectory() as td, patch.dict("sys.modules", {
                "worker": fake_worker, "mission46_terrain": fake_provider,
                "evidence_runtime_v32.worker_logging": fake_recording}), \
                patch.object(ipc_fault_window, "main", fail), \
                patch("sys.argv", ["test", "--run", td]), patch("sys.stderr", FailingStderr()), \
                patch.object(module.Path, "read_bytes", return_value=b"finite fake source receipt"), \
                patch.object(RequestWindow, "save", side_effect=OSError("actual optional save failure")) as saved:
            with self.assertRaises(RuntimeError) as raised:
                module.main()
            saved.assert_called_once()
        self.assertIs(raised.exception, error)


class StartupIntegrationTests(unittest.TestCase):
    fixture = RequestWindowTests.fixture

    def test_main_partition_surrounds_original_launcher_and_binds_helper_source(self):
        import ipc_request_window as module
        from gc_startup_partition_tests import FakeGC
        from gc_startup_partition import StartupPartition
        w, P, R, events = self.fixture()
        fake = FakeGC(events)
        partition = StartupPartition({"enabled": True, "reason": "finite_fake"}, owned_cli=True, gc_module=fake,
            importer=lambda name: events.append(("import", name)),
            getpid=lambda: 321, get_ident=lambda: 654, rss=lambda: 100)
        token = object()
        originals = (w.Path, w.json, w.canonical, ipc_evidence.FrameTiming)
        argv = ["test", "--run", "PLACEHOLDER", "--socket", "original-socket"]
        def original():
            events.append("original_launcher")
            self.assertEqual(fake.count, 42)
            self.assertEqual(__import__("sys").argv, argv)
            self.assertIsNot(w.Path, originals[0])
            self.assertIsNot(ipc_evidence.FrameTiming, originals[3])
            ipc_evidence.FrameTiming(types.SimpleNamespace(write=lambda row: None))
            return token
        with tempfile.TemporaryDirectory() as td:
            argv[2] = td
            with patch.dict("sys.modules", {"worker": w,
                    "mission46_terrain": types.SimpleNamespace(MissionTerrainProvider=P),
                    "evidence_runtime_v32.worker_logging": types.SimpleNamespace(RecordingContext=R)}), \
                    patch.object(module.StartupPartition, "for_run", return_value=partition) as selected, \
                    patch.object(ipc_fault_window, "main", original), patch("sys.argv", argv), \
                    patch.object(module.Path, "read_bytes", return_value=b"finite source bytes"):
                self.assertIs(module.main(owned_cli=True), token)
                selected.assert_called_once_with(Path(td), owned_cli=True)
            rows = [json.loads(x) for x in (Path(td) / "ipc_request_substages.jsonl").read_text().splitlines()]
            self.assertEqual(rows[0]["timers_created"], 1)
            self.assertTrue(rows[0]["startup_heap_partition"]["applied"])
            self.assertFalse(rows[-1]["startup_heap_partition_final"]["restored"])
            self.assertTrue(rows[-1]["startup_heap_partition_final"]["retained_until_process_exit"])
            helper = str(Path(module.__file__).resolve().parent / "gc_startup_partition.py")
            self.assertEqual(rows[0]["source_sha256"][helper], hashlib.sha256(b"finite source bytes").hexdigest())
        self.assertLess(events.index(("import", "torch")), events.index("original_launcher"))
        self.assertNotIn("unfreeze", events)
        self.assertEqual((w.Path, w.json, w.canonical, ipc_evidence.FrameTiming), originals)

    def test_main_final_diagnostic_and_sidecar_failures_keep_original_worker_exception(self):
        import ipc_request_window as module
        from gc_startup_partition_tests import FakeGC
        from gc_startup_partition import StartupPartition
        w, P, R, _ = self.fixture()
        fake = FakeGC(); fake.unfreeze_error = RuntimeError("must never unfreeze")
        partition = StartupPartition({"enabled": True}, owned_cli=True, gc_module=fake, importer=lambda _: None,
            getpid=lambda: 321, get_ident=lambda: 654, rss=lambda: 100)
        def bad_final():
            raise RuntimeError("final diagnostic error")
        partition._finish = bad_final
        error = RuntimeError("original worker exception identity")
        def original():
            raise error
        class BadStderr:
            def write(self, *_):
                raise RuntimeError("stderr error")
        with tempfile.TemporaryDirectory() as td, patch.dict("sys.modules", {"worker": w,
                "mission46_terrain": types.SimpleNamespace(MissionTerrainProvider=P),
                "evidence_runtime_v32.worker_logging": types.SimpleNamespace(RecordingContext=R)}), \
                patch.object(module.StartupPartition, "for_run", return_value=partition), \
                patch.object(ipc_fault_window, "main", original), patch("sys.argv", ["test", "--run", td]), \
                patch.object(module.Path, "read_bytes", return_value=b"finite source bytes"), \
                patch.object(RequestWindow, "save", side_effect=OSError("optional evidence failure")), \
                patch("sys.stderr", BadStderr()):
            with self.assertRaises(RuntimeError) as raised:
                module.main(owned_cli=True)
        self.assertIs(raised.exception, error)
        self.assertFalse(partition.footer["restored"])
        self.assertIn("final_diagnostic_error", partition.footer)
        self.assertNotIn("unfreeze", fake.events)

    def test_v3_policy_capability_and_actual_OFF_receipt_with_existing_file_cap(self):
        from gc_startup_partition_tests import FakeGC
        from gc_startup_partition import MAX_STARTUP_METADATA_BYTES, StartupPartition
        fake = FakeGC()
        partition = StartupPartition({"enabled": False, "reason": "profile_off"}, gc_module=fake,
            importer=lambda _: self.fail("OFF cannot import mandatory dependency"))
        partition.run(lambda: None)
        c = RequestWindow(); c.startup_partition = partition
        for seq in range(1, 301):
            c.begin(seq)
            for group in range(CONTRACT["maximum_operation_groups"]):
                c.call(str(group), "bounded", lambda: None)
            c.finish("response_sent", True)
        with tempfile.TemporaryDirectory() as td:
            f = c.save(td, {})
            raw = f.read_bytes(); lines = raw.splitlines(keepends=True)
            head, foot = json.loads(lines[0]), json.loads(lines[-1])
            self.assertEqual(CONTRACT["schema"], "teacher_original_request_substages/v3")
            self.assertTrue(CONTRACT["GC_schedule_threshold_or_enabled_modified"])
            self.assertTrue(CONTRACT["GC_heap_partition_strategy_modified"])
            self.assertFalse(CONTRACT["GC_threshold_or_enabled_modified"])
            self.assertFalse(CONTRACT["GC_observer_modifies_GC"])
            self.assertFalse(head["startup_heap_partition"]["applied"])
            self.assertEqual(fake.events, [])
            self.assertLessEqual(len(json.dumps({"startup_heap_partition": head["startup_heap_partition"],
                "startup_heap_partition_final": foot["startup_heap_partition_final"]},
                separators=(",", ":")).encode()), MAX_STARTUP_METADATA_BYTES)
            self.assertLessEqual(len(raw), 512 * 1024)
            self.assertLessEqual(f.stat().st_blocks * 512, 512 * 1024)
            self.assertEqual(hashlib.sha256(b"".join(lines[1:-1])).hexdigest(), foot["original_scalar_rows_sha256"])
            self.assertEqual((CONTRACT["native_watchdog_ms"], CONTRACT["source_deadline_ms"]), (200, 300))
        # Imported launcher defaults OFF even with an enabled profile selection.
        import ipc_request_window as module
        w, P, R, _ = self.fixture()
        token = object()
        fake = FakeGC()
        def select(run, *, owned_cli):
            return StartupPartition({"enabled": True}, owned_cli=owned_cli, gc_module=fake,
                importer=lambda _: self.fail("imported main cannot import dependency"))
        with tempfile.TemporaryDirectory() as td, patch.dict("sys.modules", {"worker": w,
                "mission46_terrain": types.SimpleNamespace(MissionTerrainProvider=P),
                "evidence_runtime_v32.worker_logging": types.SimpleNamespace(RecordingContext=R)}), \
                patch.object(module.StartupPartition, "for_run", side_effect=select) as selected, \
                patch.object(ipc_fault_window, "main", return_value=token), \
                patch("sys.argv", ["test", "--run", td]), \
                patch.object(module.Path, "read_bytes", return_value=b"finite source bytes"):
            self.assertIs(module.main(), token)
            selected.assert_called_once_with(Path(td), owned_cli=False)
        self.assertEqual(fake.events, [])


class DefaultGCObserverTests(unittest.TestCase):
    fixture = RequestWindowTests.fixture

    def test_actual_callback_values_wall_CPU_generation_and_request_phase(self):
        wall, cpu = iter((100, 200)), iter((10, 50))
        c = RequestWindow(clock=lambda: next(wall), cpu_clock=lambda: next(cpu))
        c.begin(1)
        c.active = True
        c.observe_gc('start', {'generation': 2})
        c.observe_gc('stop', {'generation': 2, 'collected': 7, 'uncollectable': 0})
        row = c.frame['GC_generations']['2']
        self.assertEqual((row['total_wall_ns'], row['total_thread_cpu_ns']), (100, 40))
        self.assertEqual((row['calls'], row['request_active_calls'], row['collected']), (1, 1, 7))
        self.assertEqual(c.pending_gc, {})

    def test_foreign_thread_and_outside_frame_callbacks_are_not_misattributed(self):
        c = RequestWindow()
        c.observe_gc('start', {'generation': 0})
        self.assertEqual(c.pending_gc, {})
        c.begin(1)
        with patch('ipc_request_window.threading.get_ident', return_value=c.owner_thread_id + 1):
            c.observe_gc('start', {'generation': 2})
            c.observe_gc('stop', {'generation': 2, 'collected': 0, 'uncollectable': 0})
        self.assertEqual(c.frame['GC_generations'], {})
        self.assertEqual(c.pending_gc, {})
        self.assertEqual(c.ignored_foreign_gc, 2)

    def test_restoration_preserves_existing_and_new_callbacks_and_original_fault(self):
        original = tuple(gc.callbacks)
        thresholds, enabled = gc.get_threshold(), gc.isenabled()
        additional = lambda phase, info: None
        error = RuntimeError('original worker termination')
        try:
            with self.assertRaises(RuntimeError) as raised:
                with observe_default_gc(RequestWindow()):
                    self.assertEqual(tuple(gc.callbacks[:-1]), original)
                    gc.callbacks.append(additional)
                    raise error
            self.assertIs(raised.exception, error)
            self.assertEqual(tuple(gc.callbacks), (*original, additional))
            self.assertEqual((gc.get_threshold(), gc.isenabled()), (thresholds, enabled))
        finally:
            gc.callbacks.remove(additional)
        self.assertEqual(tuple(gc.callbacks), original)

    def test_JSON_ordinals_forward_each_original_object_error_and_argument_once(self):
        w, P, R, _ = self.fixture()
        calls, token, error = [], object(), ValueError('original decoder failure')
        def loads(*args, **kwargs):
            calls.append((args, kwargs))
            if args[0] == 'fail':
                raise error
            return token
        w.json = types.SimpleNamespace(loads=loads, dumps=json.dumps)
        c = RequestWindow()
        c.begin(1)
        with instrument(w, ipc_evidence, P, R, c), c.request_scope():
            self.assertIs(w.json.loads('{}'), token)
            self.assertIs(w.json.loads(b'[]'), token)
            with self.assertRaises(ValueError) as raised:
                w.json.loads('fail')
        self.assertIs(raised.exception, error)
        self.assertEqual(calls, [(('{}',), {}), ((b'[]',), {}), (('fail',), {})])
        self.assertEqual(c.frame['json_load_calls'], 3)
        self.assertEqual(c.frame['operations']['worker.json.loads']['calls'], 3)
        self.assertEqual(c.frame['operations']['worker.json.loads/3']['errors'], 1)
        self.assertIn('input_length=2', c.frame['operations']['worker.json.loads/1']['maximum_target'])

    def test_optional_GC_clock_errors_preserve_original_JSON_error(self):
        w, P, R, _ = self.fixture()
        c = RequestWindow()
        c.begin(1)
        def fail_clock():
            raise RuntimeError('optional clock error')
        c.clock = fail_clock
        c.observe_gc('start', {'generation': 2})
        self.assertGreater(c.diagnostic_errors, 0)
        c.active = True
        with instrument(w, ipc_evidence, P, R, c), self.assertRaises(json.JSONDecodeError):
            w.json.loads('invalid unchanged JSON')

    def test_all_generation_metrics_stay_in_retained_and_file_caps_with_terminal(self):
        c = RequestWindow(clock=lambda: 1, cpu_clock=lambda: 1)
        for sequence in range(1, 1001):
            c.begin(sequence)
            for generation in (0, 1, 2, 3):
                c.observe_gc('start', {'generation': generation})
                c.observe_gc('stop', {'generation': generation, 'collected': 0, 'uncollectable': 0})
            self.assertEqual(len(c.frame['GC_generations']), 3)
            c.finish('response_sent', True)
            self.assertLessEqual(c.recent_bytes, CONTRACT['maximum_recent_bytes'])
        c.begin(1001)
        c.finish('original worker fault', False)
        with tempfile.TemporaryDirectory() as td:
            f = c.save(td, {})
            raw = f.read_bytes()
            lines = raw.splitlines(keepends=True)
            records = [json.loads(line) for line in lines]
            self.assertLessEqual(len(raw), CONTRACT['maximum_file_bytes'])
            self.assertEqual(records[0]['GC_observer']['actual_thresholds'], list(gc.get_threshold()))
            self.assertFalse(records[0]['GC_observer']['schedule_modified_by_observer'])
            self.assertTrue(any(row.get('frame_sequence') == 1001 for row in records))
            self.assertEqual(records[-1]['original_scalar_rows_sha256'], hashlib.sha256(b''.join(lines[1:-1])).hexdigest())


if __name__ == "__main__":
    unittest.main()
