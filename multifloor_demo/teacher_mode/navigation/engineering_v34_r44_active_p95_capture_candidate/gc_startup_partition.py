# Public learning snapshot 2026-10-10: privacy/path metadata or documentation links adjusted; control algorithms and numeric gates unchanged.
"""One dedicated CLI startup partition; this deliberately changes GC lifecycle.

Only the mandatory policy dependency is warmed; no Actor/model is loaded here.
Startup cycles can persist until process exit. Live allocations retain normal
automatic GC. Existing permanent objects are retained and never thawed. This
is a prospective whole-process policy, not a watchdog/performance claim.
"""
from __future__ import annotations

import gc
import hashlib
import importlib
import json
import os
from pathlib import Path
import resource
import threading

MAX_STARTUP_METADATA_BYTES = 8 * 1024
MAX_SCOPE_PEEK_BYTES = 2 * 1024 * 1024
EXPECTED_THRESHOLDS = (700, 10, 10)
EXPECTED_CHECKPOINT_SHA256 = ''
STARTUP_CONTRACT = {
    "schema": "teacher_dedicated_CLI_startup_heap_partition/v2",
    "enable_condition": "owned_cli=true AND mission46_required=true AND continuous_route_contract.enabled=true",
    "config_peek_is_scope_authorization": False,
    "mandatory_dependency_preloaded": "torch",
    "phase": "after_source_imports_before_original_worker_main",
    "freeze_calls_per_owned_process": 1,
    "requires_dedicated_cli": True,
    "preexisting_permanent_objects_preserved": True,
    "unfreeze_calls_per_owned_process": 0,
    "retained_until_process_exit": True,
    "cleanup_restores_original_permanent_graph": False,
    "preexisting_permanent_generation": "retain_nonnegative_actual_count",
    "retain_until_process_exit": True,
    "unfreeze_called": False,
    "selective_restoration_available": False,
    "required_enabled": True,
    "required_thresholds": list(EXPECTED_THRESHOLDS),
    "heap_partition_strategy_modified": True,
    "threshold_or_enabled_modified": False,
    "startup_cycles_may_persist_until_process_exit": True,
    "all_original_reads_guards_and_JSON_loads_retained": True,
    "performance_or_memory_non_regression_verified": False,
    "maximum_metadata_bytes": MAX_STARTUP_METADATA_BYTES,
}


class StartupPartitionError(RuntimeError):
    """A required startup precondition failed before the original worker loop."""


def peek_enabled(run):
    """Bounded config selection only; the worker still verifies every source.

    Missing/malformed/basic invalid input is a no-op so the original worker
    reports its original error. Reference/hash validity is deliberately not
    duplicated or substituted by this optional selection read.
    """
    try:
        root = Path(run).resolve()
        with (root / "navigation_scope.json").open("rb") as stream:
            raw = stream.read(MAX_SCOPE_PEEK_BYTES + 1)
        if len(raw) > MAX_SCOPE_PEEK_BYTES:
            return {"enabled": False, "reason": "scope_peek_over_bound"}
        scope = json.loads(raw)
        if not isinstance(scope, dict):
            return {"enabled": False, "reason": "scope_not_object"}
        basic = (scope.get("schema") == "teacher_closed_loop_navigation_scope/v1"
                 and scope.get("allowed") is True
                 and scope.get("controller_kind") == "teacher"
                 and scope.get("checkpoint_sha256") == EXPECTED_CHECKPOINT_SHA256
                 and scope.get("navigation_ground_truth_used") is False
                 and scope.get("navigation_is_verified") is False
                 and scope.get("status") == "experimental_unverified"
                 and scope.get("run_dir") == str(root))
        profile = scope.get("profile")
        if not basic or not isinstance(profile, dict):
            return {"enabled": False, "reason": "scope_basic_shape_invalid"}
        route = profile.get("continuous_route_contract")
        enabled = (profile.get("mission46_required") is True
                   and isinstance(route, dict) and route.get("enabled") is True)
        return {"enabled": enabled, "reason": "selected" if enabled else "profile_off",
                "peek_bytes": len(raw), "peek_sha256": hashlib.sha256(raw).hexdigest(),
                "scope_authorization": False, "reference_hashes_verified_by_peek": False}
    except Exception:
        return {"enabled": False, "reason": "scope_unavailable_or_malformed"}


def _error(error):
    try:
        message = str(error).encode("ascii", "backslashreplace")[:192].decode("ascii")
    except BaseException:
        message = "<message unavailable>"
    name = type(error).__name__.encode("ascii", "backslashreplace")[:64].decode("ascii")
    return {"type": name, "message": message}


class StartupPartition:
    def __init__(self, selection, *, owned_cli=False, gc_module=gc, importer=importlib.import_module,
                 getpid=os.getpid, get_ident=threading.get_ident, rss=None):
        self.selection = dict(selection)
        self.owned_cli = owned_cli is True
        self.gc = gc_module
        self.importer = importer
        self.getpid, self.get_ident = getpid, get_ident
        self.rss = rss or (lambda: int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
        self.owner_pid, self.owner_thread_id = getpid(), get_ident()
        self.started = False
        self.freeze_returned = False
        self.header = {"schema": STARTUP_CONTRACT["schema"], "selection": self.selection,
                       "owner_pid": self.owner_pid, "owner_thread_id": self.owner_thread_id,
                       "owned_cli": self.owned_cli, "retain_until_process_exit": False,
                       "selective_restoration_available": False,
                       "phase": "startup_before_original_main", "applied": False,
                       "dependency_preload_calls": 0, "freeze_calls": 0}
        self.footer = {"schema": STARTUP_CONTRACT["schema"], "phase": "not_started",
                       "unfreeze_calls": 0, "restored": False, "original_main_called": False,
                       "retained_until_process_exit": False,
                       "selective_restoration_available": False}

    @classmethod
    def for_run(cls, run, *, owned_cli=False):
        return cls(peek_enabled(run), owned_cli=owned_cli)

    def _require_owner(self):
        if (self.getpid(), self.get_ident()) != (self.owner_pid, self.owner_thread_id):
            raise StartupPartitionError("Startup partition owner PID/thread changed")

    def _snapshot(self):
        result = {"freeze_count": int(self.gc.get_freeze_count()),
                  "thresholds": list(self.gc.get_threshold()), "enabled": self.gc.isenabled()}
        try:
            result["peak_rss_kib"] = int(self.rss())
        except BaseException as error:
            result["peak_rss_error"] = _error(error)
        return result

    def _safe_snapshot(self):
        try:
            return self._snapshot()
        except BaseException as error:
            return {"snapshot_error": _error(error)}

    @staticmethod
    def _require_state(state):
        if state["enabled"] is not True or tuple(state["thresholds"]) != EXPECTED_THRESHOLDS:
            raise StartupPartitionError("Automatic GC enabled/threshold startup precondition failed")
        if state["freeze_count"] < 0:
            raise StartupPartitionError("Permanent generation count must be nonnegative")

    def _start(self):
        self._require_owner()
        self.header["bootstrap"] = self._snapshot()
        self._require_state(self.header["bootstrap"])
        self.header["dependency_preload_calls"] = 1
        self.importer("torch")
        self._require_owner()
        self.header["after_dependency_import"] = self._snapshot()
        self._require_state(self.header["after_dependency_import"])
        # Existing permanent objects are intentionally retained. The standard
        # API cannot restore only this attempt, so this CLI policy never thaws.
        self.header["freeze_calls"] = 1
        try:
            self.gc.freeze()
        except BaseException:
            self.header["failed_freeze_partition_ownership_unknown"] = True
            raise
        self.freeze_returned = True
        self.header["retain_until_process_exit"] = True
        self._require_owner()
        self.header["after_freeze"] = self._snapshot()
        if (self.header["after_freeze"]["enabled"] is not True
                or tuple(self.header["after_freeze"]["thresholds"]) != EXPECTED_THRESHOLDS):
            raise StartupPartitionError("Automatic GC changed during startup partition")
        self.header["applied"] = True

    def _finish(self):
        self.footer["phase"] = "after_original_main_or_prestart_failure"
        self.footer["final_state"] = self._safe_snapshot()
        self.footer["retained_until_process_exit"] = self.freeze_returned
        try:
            self._require_owner()
            self.footer["final_owner_pid"] = self.getpid()
            self.footer["final_owner_thread_id"] = self.get_ident()
        except BaseException as error:
            self.footer["owner_error"] = _error(error)
        # No thaw, threshold change, collection or GC enable/disable occurs.
        # Optional evidence errors never replace a worker outcome.

    def run(self, original, *args, **kwargs):
        if not self.owned_cli or not self.selection.get("enabled"):
            self.footer.update(phase="off_no_GC_API_or_dependency_import",
                               original_main_called=True)
            return original(*args, **kwargs)
        if self.started:
            raise StartupPartitionError("Startup partition may run only once")
        self.started = True
        try:
            self._start()
            self.footer["original_main_called"] = True
            return original(*args, **kwargs)
        except BaseException as error:
            self.footer["original_error"] = _error(error)
            self.footer["error_phase"] = ("original_main" if self.footer["original_main_called"]
                                          else "prestart")
            raise
        finally:
            # Even optional final diagnostics failing unexpectedly must preserve
            # the exact original result or exception object.
            try:
                self._finish()
            except BaseException as error:
                self.footer["final_diagnostic_error"] = _error(error)

    def receipt_parts(self):
        parts = {"startup_heap_partition": self.header,
                 "startup_heap_partition_final": self.footer}
        if len(json.dumps(parts, separators=(",", ":"), allow_nan=False).encode()) > MAX_STARTUP_METADATA_BYTES:
            raise ValueError("Startup partition metadata exceeds declared bound")
        return self.header, self.footer
