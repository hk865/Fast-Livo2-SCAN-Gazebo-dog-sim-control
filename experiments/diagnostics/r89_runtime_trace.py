"""Default-off integration of bounded, process-local R89 thread evidence.

Only wraps the existing two obstacle calls and forwards their unchanged results.
The original arrays are neither copied nor retained. All disk output is exit-only.
Diagnostic Exception failures cannot skip, repeat or suppress a geometry call.
"""
from contextlib import contextmanager
from functools import wraps
import json
import sys

import experiments.diagnostics.r87_diagnostic_probe as timeline
import experiments.diagnostics.r89_thread_probe as threads


_ERRORS = {key: 0 for key in ('start', 'install', 'metadata', 'zone_enter',
    'zone_exit', 'timeline_enter', 'timeline_exit', 'stop', 'restore', 'write', 'stderr')}
_binding = None


def _fault(key):
    # Fixed keys and integer counters only; no payload, traceback or exception
    # object is retained. This observer must have no control authority.
    _ERRORS[key] += 1


@contextmanager
def _observe(kind, factory, *args, **kwargs):
    try:
        scope = factory(*args, **kwargs)
        scope.__enter__()
    except Exception:
        _fault(kind + '_enter')
        yield
        return
    error = (None, None, None)
    try:
        yield
    except BaseException:
        error = sys.exc_info()
        raise
    finally:
        try:
            # Ignore suppression requests: only the original call determines
            # whether its exception propagates. A failed observer exit cannot
            # replace either a successful return or the original exception.
            scope.__exit__(*error)
        except Exception:
            _fault(kind + '_exit')
        finally:
            del error


def _stop():
    try:
        return threads.stop()
    except Exception as error:
        _fault('stop')
        return dict(enabled=True, stop_complete=False, stop_error=type(error).__name__)


def install(node):
    global _binding
    if _binding is not None:
        return dict(enabled=True, installed=True, already_installed=True,
                    integration_errors=dict(_ERRORS))
    try:
        receipt = threads.start()
    except Exception as error:
        _fault('start')
        stopped = _stop()
        return dict(enabled=False, running=False, start_error=type(error).__name__,
                    cleanup=stopped, integration_errors=dict(_ERRORS))
    try:
        # A selected switch does not establish successful startup. The sampler
        # reports the live thread before any geometry callable is replaced.
        if not receipt.get('running'):
            return receipt
        audit = node.native_guard_audit
        original = audit.native_obstacle

        @wraps(original)
        def observed(cloud, pose, target, route, **kwargs):
            source_ns = None
            fields = {}
            try:
                source_ns = node.pose_stamp
                fields = dict(corridor_index=len(audit.calls),
                              cloud_points=len(cloud) if cloud is not None else 0,
                              route_points=len(route))
            except Exception:
                _fault('metadata')
            with _observe('zone', threads.zone, 'teacher.native_corridor', source_ns=source_ns):
                with _observe('timeline', timeline.span, 'teacher.native_corridor',
                              source_ns=source_ns, **fields):
                    # This is the sole invocation, with the original object
                    # identities, keyword values and return/exception object.
                    return original(cloud, pose, target, route, **kwargs)

        audit.native_obstacle = observed
        _binding = (audit, original, observed)
        return receipt
    except Exception as error:
        _fault('install')
        return dict(receipt if isinstance(receipt, dict) else {},
                    installation_error=type(error).__name__, cleanup=_stop(),
                    integration_errors=dict(_ERRORS))


def finish(node):
    """Best effort after callback join, including exceptional shutdown paths."""
    global _binding
    # Stop does not depend on the environment retaining its startup value.
    snapshot = _stop()
    binding, _binding = _binding, None
    restored = False
    if binding is not None:
        try:
            audit, original, observed = binding
            if audit.native_obstacle is observed:
                audit.native_obstacle = original
                restored = True
        except Exception:
            _fault('restore')
    try:
        if not isinstance(snapshot, dict):
            raise TypeError('invalid sampler snapshot')
        if not snapshot.get('enabled') and binding is None and not any(_ERRORS.values()):
            return snapshot
        snapshot = dict(snapshot, integration_errors=dict(_ERRORS),
                        corridor_wrapper_restored=restored)
        (node.run/'r89_thread_timeline.json').write_text(
            json.dumps(snapshot, allow_nan=False)+'\n')
    except Exception as error:
        _fault('write')
        try:
            print('R89 thread evidence flush failed: '+type(error).__name__+': '+str(error),
                  file=sys.stderr)
        except Exception:
            _fault('stderr')
    return snapshot
