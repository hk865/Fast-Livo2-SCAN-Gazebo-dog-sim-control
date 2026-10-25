"""Finite CPU-only policy checks; dependency import is always a fake.

The separate isolated lifecycle fixture calls gc.collect only in test code.
It preserves any preexisting permanent objects and exits without unfreeze.
No Torch/Actor/model/JIT/GPU is imported or loaded by these checks.
"""
import ast
import gc
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import gc_startup_partition as module
from gc_startup_partition import (EXPECTED_CHECKPOINT_SHA256, EXPECTED_THRESHOLDS, MAX_STARTUP_METADATA_BYTES,
                                  STARTUP_CONTRACT, StartupPartition, StartupPartitionError,
                                  peek_enabled)


class FakeGC:
    def __init__(self, events=None):
        self.events = events if events is not None else []
        self.count, self.enabled, self.thresholds = 0, True, EXPECTED_THRESHOLDS
        self.freeze_error = self.unfreeze_error = None
        self.freeze_partial = False

    def get_freeze_count(self):
        self.events.append("count")
        return self.count

    def isenabled(self):
        self.events.append("enabled")
        return self.enabled

    def get_threshold(self):
        self.events.append("thresholds")
        return self.thresholds

    def freeze(self):
        self.events.append("freeze")
        if self.freeze_error is not None:
            if self.freeze_partial:
                self.count = 42
            raise self.freeze_error
        self.count += 42

    def unfreeze(self):
        self.events.append("unfreeze")
        if self.unfreeze_error is not None:
            raise self.unfreeze_error
        self.count = 0


class StartupPartitionTests(unittest.TestCase):
    def fixture(self, selection=None, importer=None, fake=None):
        events = [] if fake is None else fake.events
        fake = fake or FakeGC(events)
        if importer is None:
            def importer(name):
                events.append(("import", name))
                return object()
        p = StartupPartition(selection or {"enabled": True, "reason": "finite_fake"}, owned_cli=True,
                             gc_module=fake, importer=importer,
                             getpid=lambda: 321, get_ident=lambda: 654, rss=lambda: 100)
        return p, fake, events

    @staticmethod
    def scope(root):
        return {"schema": "teacher_closed_loop_navigation_scope/v1", "allowed": True,
                "checkpoint_sha256": EXPECTED_CHECKPOINT_SHA256,
                "controller_kind": "teacher", "navigation_ground_truth_used": False,
                "navigation_is_verified": False, "status": "experimental_unverified",
                "run_dir": str(Path(root).resolve()), "references": {"bad": "bad hash"},
                "profile": {"mission46_required": True,
                            "continuous_route_contract": {"enabled": True}}}

    def test_OFF_no_dependency_import_or_GC_API_and_exact_forwarding(self):
        p, fake, events = self.fixture({"enabled": False, "reason": "profile_off"})
        arg, answer = object(), object()
        self.assertIs(p.run(lambda value, *, key: answer if value is key else None,
                            arg, key=arg), answer)
        error = RuntimeError("original OFF error")
        def fail():
            raise error
        with self.assertRaises(RuntimeError) as raised:
            p.run(fail)
        self.assertIs(raised.exception, error)
        self.assertEqual(events, [])
        self.assertFalse(p.header["applied"])
        self.assertEqual(p.footer["phase"], "off_no_GC_API_or_dependency_import")
        p, _, events = self.fixture()
        p.owned_cli = False
        self.assertIs(p.run(lambda: answer), answer)
        self.assertEqual(events, [])

    def test_missing_malformed_basic_invalid_and_boolean_gates_are_noop(self):
        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "navigation_scope.json"
            self.assertFalse(peek_enabled(td)["enabled"])
            for raw in (b"not JSON", b"[]", b"{}"):
                f.write_bytes(raw)
                self.assertFalse(peek_enabled(td)["enabled"])
            variants = []
            for field, value in (("allowed", False), ("controller_kind", "other"),
                                 ("run_dir", "other"), ("navigation_ground_truth_used", True),
                                 ("checkpoint_sha256", "bad checkpoint")):
                s = self.scope(td); s[field] = value; variants.append(s)
            s = self.scope(td); del s["checkpoint_sha256"]; variants.append(s)
            for profile in ({}, {"mission46_required": True},
                            {"mission46_required": 1, "continuous_route_contract": {"enabled": True}},
                            {"mission46_required": True, "continuous_route_contract": {"enabled": 1}}):
                s = self.scope(td); s["profile"] = profile; variants.append(s)
            for s in variants:
                f.write_text(json.dumps(s))
                selection = peek_enabled(td)
                p, _, events = self.fixture(selection)
                self.assertIsNone(p.run(lambda: None))
                self.assertFalse(selection["enabled"])
                self.assertEqual(events, [])
                original_error = RuntimeError("original Invalid closed-loop executor scope")
                def original_guard():
                    raise original_error
                with self.assertRaises(RuntimeError) as raised:
                    p.run(original_guard)
                self.assertIs(raised.exception, original_error)
                self.assertEqual(events, [])

    def test_scope_peek_bound_and_config_only_receipt(self):
        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "navigation_scope.json"
            f.write_text(json.dumps(self.scope(td)))
            selection = peek_enabled(td)
            self.assertTrue(selection["enabled"])
            self.assertFalse(selection["scope_authorization"])
            self.assertFalse(selection["reference_hashes_verified_by_peek"])
            self.assertEqual(selection["peek_bytes"], f.stat().st_size)
            # Tiny patched bound tests the same bounded-read rejection branch.
            with patch.object(module, "MAX_SCOPE_PEEK_BYTES", 32):
                self.assertEqual(peek_enabled(td)["reason"], "scope_peek_over_bound")
            self.assertEqual(module.MAX_SCOPE_PEEK_BYTES, 2 * 1024 * 1024)

    def test_single_preload_before_main_once_CLI_freeze_and_retention(self):
        p, fake, events = self.fixture()
        token = object()
        def original(arg, *, key):
            self.assertIs(arg, key)
            events.append("main")
            self.assertEqual(fake.count, 42)
            return token
        arg = object()
        self.assertIs(p.run(original, arg, key=arg), token)
        self.assertLess(events.index(("import", "torch")), events.index("freeze"))
        self.assertLess(events.index("freeze"), events.index("main"))
        self.assertEqual(events.count(("import", "torch")), 1)
        self.assertEqual(events.count("freeze"), 1)
        self.assertEqual(events.count("unfreeze"), 0)
        self.assertEqual((p.header["owner_pid"], p.header["owner_thread_id"]), (321, 654))
        self.assertEqual(p.header["phase"], "startup_before_original_main")
        self.assertTrue(p.header["applied"])
        self.assertFalse(p.footer["restored"])
        self.assertTrue(p.footer["retained_until_process_exit"])
        self.assertEqual((fake.enabled, fake.thresholds, fake.count), (True, EXPECTED_THRESHOLDS, 42))
        before = list(events)
        with self.assertRaises(StartupPartitionError):
            p.run(original, arg, key=arg)
        self.assertEqual(events, before)

    def test_original_exception_identity_and_finally_record_without_thaw(self):
        p, fake, events = self.fixture()
        error = ValueError("original worker fault")
        def original():
            events.append("main")
            raise error
        with self.assertRaises(ValueError) as raised:
            p.run(original)
        self.assertIs(raised.exception, error)
        self.assertEqual(fake.count, 42)
        self.assertFalse(p.footer["restored"])
        self.assertTrue(p.footer["retained_until_process_exit"])
        self.assertEqual(p.footer["original_error"]["message"], str(error))

    def test_preexisting_permanent_generation_is_preserved_and_never_thawed(self):
        fake = FakeGC(); fake.count = 375
        p, _, events = self.fixture(fake=fake)
        p.run(lambda: None)
        self.assertEqual(fake.count, 417)
        self.assertEqual(p.header["bootstrap"]["freeze_count"], 375)
        self.assertEqual(p.header["after_dependency_import"]["freeze_count"], 375)
        self.assertEqual(p.footer["final_state"]["freeze_count"], 417)
        self.assertNotIn("unfreeze", events)
        self.assertEqual(events.count("freeze"), 1)
        self.assertEqual(events.count(("import", "torch")), 1)
        self.assertTrue(p.header["applied"])
        self.assertTrue(p.footer["original_main_called"])

    def test_dependency_import_error_retains_identity_and_no_freeze(self):
        error = ImportError("mandatory dependency unavailable")
        def importer(name):
            self.assertEqual(name, "torch")
            raise error
        p, _, events = self.fixture(importer=importer)
        with self.assertRaises(ImportError) as raised:
            p.run(lambda: self.fail("prestart failure must abort"))
        self.assertIs(raised.exception, error)
        self.assertNotIn("freeze", events)
        self.assertNotIn("unfreeze", events)
        self.assertFalse(p.footer["original_main_called"])

    def test_dependency_created_permanent_generation_is_retained(self):
        fake = FakeGC()
        def importer(_):
            fake.count = 9
        p, _, events = self.fixture(importer=importer, fake=fake)
        p.run(lambda: None)
        self.assertEqual(fake.count, 51)
        self.assertEqual(p.header["after_dependency_import"]["freeze_count"], 9)
        self.assertEqual(events.count("freeze"), 1)
        self.assertNotIn("unfreeze", events)

    def test_disabled_or_changed_threshold_prestart_aborts_without_modification(self):
        for enabled, thresholds in ((False, EXPECTED_THRESHOLDS), (True, (1, 2, 3))):
            with self.subTest(enabled=enabled, thresholds=thresholds):
                fake = FakeGC(); fake.enabled, fake.thresholds = enabled, thresholds
                p, _, events = self.fixture(fake=fake)
                with self.assertRaises(StartupPartitionError):
                    p.run(lambda: self.fail("must abort"))
                self.assertEqual((fake.enabled, fake.thresholds), (enabled, thresholds))
                self.assertNotIn(("import", "torch"), events)
                self.assertNotIn("freeze", events)
                self.assertNotIn("unfreeze", events)
        fake = FakeGC(); fake.count = -1
        p, _, events = self.fixture(fake=fake)
        with self.assertRaises(StartupPartitionError):
            p.run(lambda: self.fail("negative permanent count must abort"))
        self.assertNotIn("freeze", events)
        self.assertEqual(fake.count, -1)

    def test_post_import_changed_threshold_or_enabled_aborts_before_freeze(self):
        for enabled, thresholds in ((False, EXPECTED_THRESHOLDS), (True, (1, 2, 3))):
            with self.subTest(enabled=enabled, thresholds=thresholds):
                fake = FakeGC()
                def importer(_):
                    fake.enabled, fake.thresholds = enabled, thresholds
                p, _, events = self.fixture(fake=fake, importer=importer)
                with self.assertRaises(StartupPartitionError):
                    p.run(lambda: self.fail("must abort"))
                self.assertEqual((fake.enabled, fake.thresholds), (enabled, thresholds))
                self.assertNotIn("freeze", events)
                self.assertNotIn("unfreeze", events)

    def test_failed_freeze_preserves_error_without_thawing_an_unproven_partition(self):
        for partial in (False, True):
            with self.subTest(partial=partial):
                fake = FakeGC(); fake.freeze_error = RuntimeError("freeze failed")
                fake.freeze_partial = partial
                p, _, events = self.fixture(fake=fake)
                with self.assertRaises(RuntimeError) as raised:
                    p.run(lambda: self.fail("no original loop after failed freeze"))
                self.assertIs(raised.exception, fake.freeze_error)
                self.assertEqual(events.count("unfreeze"), 0)
                self.assertEqual(fake.count, 42 if partial else 0)
                self.assertTrue(p.header["failed_freeze_partition_ownership_unknown"])
                self.assertFalse(p.header["applied"])
                self.assertFalse(p.footer["original_main_called"])

    def test_final_diagnostic_failure_never_covers_original_return_or_error(self):
        for fails in (False, True):
            with self.subTest(fails=fails):
                fake = FakeGC(); fake.unfreeze_error = RuntimeError("must never be called")
                p, _, _ = self.fixture(fake=fake)
                def bad_final():
                    raise RuntimeError("final diagnostic failed")
                p._finish = bad_final
                token, error = object(), RuntimeError("original fault")
                def original():
                    if fails:
                        raise error
                    return token
                if fails:
                    with self.assertRaises(RuntimeError) as raised:
                        p.run(original)
                    self.assertIs(raised.exception, error)
                else:
                    self.assertIs(p.run(original), token)
                self.assertFalse(p.footer["restored"])
                self.assertEqual(p.footer["final_diagnostic_error"]["message"], "final diagnostic failed")
                self.assertNotIn("unfreeze", fake.events)

    def test_owner_change_aborts_start_or_records_foreign_final_owner_without_thaw(self):
        p, fake, events = self.fixture()
        p.get_ident = lambda: 999
        with self.assertRaises(StartupPartitionError):
            p.run(lambda: self.fail("foreign owner must not start"))
        self.assertNotIn("freeze", events)
        p, fake, events = self.fixture()
        token = object()
        def original():
            p.getpid = lambda: 999
            return token
        self.assertIs(p.run(original), token)
        self.assertNotIn("unfreeze", events)
        self.assertEqual(fake.count, 42)
        self.assertIn("owner_error", p.footer)
        self.assertFalse(p.footer["restored"])

    def test_bad_reference_hash_still_reaches_original_guard_and_original_exception(self):
        with tempfile.TemporaryDirectory() as td:
            (Path(td) / "navigation_scope.json").write_text(json.dumps(self.scope(td)))
            p, fake, events = self.fixture(peek_enabled(td))
            error = RuntimeError("original full source verifier rejects bad hash")
            def original():
                events.append("original_full_guard")
                raise error
            with self.assertRaises(RuntimeError) as raised:
                p.run(original)
            self.assertIs(raised.exception, error)
            self.assertIn("original_full_guard", events)
            self.assertFalse(p.header["selection"]["scope_authorization"])
            self.assertEqual(fake.count, 42)

    def test_bounded_metadata_stdlib_source_and_no_forbidden_production_GC_API(self):
        p, _, _ = self.fixture()
        p.run(lambda: None)
        head, foot = p.receipt_parts()
        self.assertLessEqual(len(json.dumps({"startup_heap_partition": head,
                                            "startup_heap_partition_final": foot},
                                           separators=(",", ":")).encode()), MAX_STARTUP_METADATA_BYTES)
        self.assertEqual(MAX_STARTUP_METADATA_BYTES, 8192)
        self.assertTrue(STARTUP_CONTRACT["heap_partition_strategy_modified"])
        self.assertFalse(STARTUP_CONTRACT["threshold_or_enabled_modified"])
        self.assertFalse(STARTUP_CONTRACT["performance_or_memory_non_regression_verified"])
        tree = ast.parse(Path(module.__file__).read_text())
        forbidden = {"collect", "disable", "enable", "set_threshold", "unfreeze"}
        self.assertFalse(any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                             and n.func.attr in forbidden for n in ast.walk(tree)))
        imports = {n.names[0].name for n in ast.walk(tree) if isinstance(n, ast.Import)}
        self.assertNotIn("torch", imports)
        self.assertNotIn("numpy", imports)
        self.assertTrue(all(n in sys.stdlib_module_names for n in imports))
        self.assertEqual(head["bootstrap"]["peak_rss_kib"], 100)
        p, _, _ = self.fixture()
        def bad_rss():
            raise RuntimeError("\U0001f600" * 10000)
        p.rss = bad_rss
        p.run(lambda: None)
        self.assertLessEqual(len(json.dumps(p.receipt_parts()).encode()), MAX_STARTUP_METADATA_BYTES)
        for key, value in {"requires_dedicated_cli": True, "preexisting_permanent_objects_preserved": True,
                "unfreeze_calls_per_owned_process": 0, "retained_until_process_exit": True,
                "cleanup_restores_original_permanent_graph": False, "freeze_calls_per_owned_process": 1,
                "threshold_or_enabled_modified": False}.items():
            self.assertEqual(STARTUP_CONTRACT[key], value)

    def test_real_isolated_startup_cycles_live_cycles_and_process_exit_lifecycle(self):
        code = r'''
import gc,json,sys,weakref
from gc_startup_partition import StartupPartition,EXPECTED_THRESHOLDS
assert 'torch' not in sys.modules
assert gc.get_freeze_count()>=0 and gc.isenabled() and gc.get_threshold()==EXPECTED_THRESHOLDS
existing=gc.get_freeze_count()
class Cycle: pass
startup=Cycle();startup.self=startup;startup_ref=weakref.ref(startup)
baseline=(gc.get_threshold(),gc.isenabled())
events=[]
p=StartupPartition({'enabled':True,'reason':'isolated_fake_dependency'},owned_cli=True,importer=lambda n:events.append(n))
def original():
    global startup
    startup=None
    gc.collect() # Test fixture only: permanent startup cycle must be ignored.
    assert startup_ref() is not None
    live=Cycle();live.self=live;live_ref=weakref.ref(live);live=None
    gc.collect() # Test fixture only: new live cycle remains collectible.
    assert live_ref() is None
    return 'original-result'
assert p.run(original)=='original-result'
assert events==['torch'] and 'torch' not in sys.modules
assert gc.get_freeze_count()>=existing and (gc.get_threshold(),gc.isenabled())==baseline
gc.collect() # Test fixture only: startup cycle remains permanent until exit.
assert startup_ref() is not None
assert p.header['applied'] and not p.footer['restored']
assert p.footer['retained_until_process_exit'] and p.footer['unfreeze_calls']==0
print(json.dumps({'startup_live_retained_lifecycle':True,'model_or_torch_loaded':False,
                  'existing_permanent_count':existing,'final_permanent_count':gc.get_freeze_count(),
                  'unfreeze_called':False}))
'''
        result = subprocess.run([sys.executable, "-B", "-c", code],
                                cwd=Path(module.__file__).parent, capture_output=True,
                                text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(json.loads(result.stdout)["startup_live_retained_lifecycle"])


if __name__ == "__main__":
    unittest.main()
