"""Default-OFF scalar state transitions; original gates remain sole authority.

The selected node's original shared stale function is delegated exactly once.
TLS observes only subpredicates it actually evaluates (including short circuits).
No readiness/freshness is re-evaluated and no hot-path file I/O is performed.
"""
from collections import deque
from functools import wraps
import json
import os
from pathlib import Path
import threading
import time

KEY = 'R102_PROGRESS_TRANSITIONS'
CAPACITY = 2048
MAX_KEYS = 128
MAX_FILE_BYTES = 20_000_000
MAX_MEMORY_BYTES = 4 * 1024 * 1024
OWNERS = ('shared_sources', 'admission_hold', 'admission_verdict', 'known_hold', 'admission_state')
ROW_COLUMNS = ('event_seq', 'owner', 'state', 'reason', 'sim_ns', 'mono_ns',
    'pose_ns', 'cloud_ns', 'active_trajectory', 'admission_seq', 'candidate_trajectory',
    'publication_seq', 'path_hold', 'replan_pending', 'bridge_state', 'evaluated_subpredicates',
    'waypoint', 'cascade_protected', 'cascade_reason', 'source_inputs_after_original')
_TLS = threading.local()
_INSTALLATION = None


def _scalar(value):
    if value is None or type(value) is bool:
        return value
    if type(value) is int:
        return value if -(1 << 63) <= value < (1 << 63) else None
    if type(value) is str:
        # All reason/identity fields have bounded ASCII storage, including Unicode input.
        return value[:64].encode('ascii','backslashreplace')[:64].decode('ascii')
    return None


def _finite_seconds_ns(value):
    # No source time is substituted for an absent ROS clock anchor.
    if type(value) in (int, float) and 0 <= value < 9e9:
        return round(value * 1e9)
    return None


class Recorder:
    def __init__(self):
        self.pid = os.getpid()
        self.rows = deque(maxlen=CAPACITY)
        self.counts = {}
        self.last = {}
        self.serial = self.calls = self.evicted = self.key_overflow = 0
        self.errors = self.closed_calls = self.clock_reversals = 0
        self.cost_calls = self.cost_wall_ns = self.cost_cpu_ns = self.cost_max_wall_ns = 0
        self.closed = False
        self.export_attempted = False
        self.receipt = {'installed': False, 'restored': False}
        self.missing_exact_direct_hold_subpredicates = True

    def observe(self, node, owner, state, reason, sim_ns=None, steps=(), decision=None, source_inputs=()):
        """Nonthrowing, fixed-schema observation of already available scalars."""
        start = time.perf_counter_ns()
        cpu = time.thread_time_ns()
        try:
            if self.closed or self.pid != os.getpid():
                self.closed_calls += 1
                return
            self.calls += 1
            mono = time.monotonic_ns()
            owner = OWNERS.index(owner)
            state, reason = _scalar(state), _scalar(reason)
            sim_ns = _scalar(sim_ns)
            steps = tuple((name, _scalar(result)) for name, result in steps[:4])
            key = (owner, state, reason)
            count = self.counts.get(key)
            if count is None:
                if len(self.counts) >= MAX_KEYS:
                    self.key_overflow += 1
                else:
                    # samples, observed SIM duration, MONO duration, backwards, absent-SIM intervals
                    count = self.counts[key] = [0, 0, 0, 0, 0]
            if count is not None:
                count[0] += 1
            trajectory = _scalar(getattr(node, 'active_trajectory_id', None))
            last_decision = getattr(node, 'path_admission_last_decision', None)
            last_decision = last_decision if type(last_decision) is dict else {}
            decision = decision if type(decision) is dict else last_decision
            admission_seq = _scalar(decision.get('sequence'))
            candidate = _scalar(decision.get('association_payload_id'))
            hold = _scalar(getattr(node, 'path_admission_hold', None))
            replan = _scalar(getattr(node, 'path_admission_replan_pending', None))
            bridge = getattr(node, 'bridge_safety', None)
            bridge = _scalar(bridge.get('state')) if type(bridge) is dict else None
            core = getattr(node, 'cascade', None)
            protected = _scalar(getattr(core, 'protected', None))
            core_reason = _scalar(getattr(core, 'protection_reason', None))
            # Header/counter progress alone must not turn every tick into a transition.
            identity = (state, reason, trajectory, hold, replan, bridge, protected, core_reason)
            if owner == OWNERS.index('admission_verdict'):
                identity += (admission_seq, candidate)
            previous = self.last.get(owner)
            if previous is not None:
                _, old_key, old_sim, old_mono = previous
                bucket = self.counts.get(old_key)
                if bucket is not None:
                    bucket[2] += max(0, mono-old_mono)
                    if type(sim_ns) is int and type(old_sim) is int:
                        if sim_ns < old_sim:
                            bucket[3] += 1
                            self.clock_reversals += 1
                        else:
                            bucket[1] += sim_ns-old_sim
                    else:
                        bucket[4] += 1
            self.last[owner] = (identity, key, sim_ns, mono)
            if previous is not None and previous[0] == identity:
                return
            self.serial += 1
            self.evicted += int(len(self.rows) == CAPACITY)
            self.rows.append((self.serial, owner, state, reason, sim_ns, mono,
                _scalar(getattr(node, 'pose_stamp', None)), _scalar(getattr(node, 'cloud_stamp', None)),
                trajectory, admission_seq, candidate, _scalar(getattr(node, 'publication_records', None)),
                hold, replan, bridge, steps, _scalar(getattr(node, 'waypoint_index', None)),
                protected, core_reason, source_inputs))
        except Exception:
            self.errors += 1
        finally:
            elapsed = time.perf_counter_ns()-start
            self.cost_calls += 1
            self.cost_wall_ns += elapsed
            self.cost_cpu_ns += time.thread_time_ns()-cpu
            self.cost_max_wall_ns = max(self.cost_max_wall_ns, elapsed)

    def snapshot(self):
        return dict(schema='r102_bounded_progress_transitions/v1', switch=KEY, enabled=True,
            pid=self.pid, capacity=CAPACITY, maximum_reason_keys=MAX_KEYS,
            maximum_memory_bytes=MAX_MEMORY_BYTES, maximum_file_bytes=MAX_FILE_BYTES,
            calls=self.calls, transitions=self.serial, evicted=self.evicted,
            errors=self.errors, key_overflow=self.key_overflow, closed_calls=self.closed_calls,
            clock_reversals=self.clock_reversals, owner_names=OWNERS, row_columns=ROW_COLUMNS,
            rows=list(self.rows), counts=[(key, value) for key, value in self.counts.items()],
            count_columns=('samples', 'observed_sim_ns', 'observed_mono_ns', 'sim_backwards', 'missing_sim_intervals'),
            unclosed_owner_intervals=list(self.last), installation=dict(self.receipt),
            observer_cost=dict(calls=self.cost_calls, wall_ns=self.cost_wall_ns,
                thread_cpu_ns=self.cost_cpu_ns, maximum_wall_ns=self.cost_max_wall_ns,
                includes='scalar bookkeeping only; excludes original predicates, wrapper dispatch and exit encoding/write'),
            hot_path_io=False, original_predicates_called_once=True, readiness_recomputed=False,
            source_input_columns=('caller_now_mono_ns','pose_stamp_ns','pose_updated_mono_ns','pose_timeout_ns',
                'cloud_stamp_ns','cloud_updated_mono_ns','cloud_timeout_ns','raw_imu_stamp_ns',
                'raw_imu_updated_mono_ns','raw_imu_timeout_ns'),
            string_storage='at most 64 ASCII bytes per field, Unicode escaped then truncated; details remain in original R73/admission evidence',
            source_snapshot_scope='sequential scalars after original aggregate returned; not an atomic predicate read snapshot',
            missing_exact_direct_hold_subpredicates=self.missing_exact_direct_hold_subpredicates,
            duration_scope='per owner left observation to next observation; owners overlap; final open intervals excluded; not physical motion duration',
            publication_and_ack_evidence='reuse complete r73_publications / r73_control / r73_selected_ack, verify writer counts',
            behavior_authority=False)


def note(node, owner, state, reason, sim_ns=None, decision=None):
    probe = getattr(node, '_r102_progress', None)
    if probe is not None:
        probe.observe(node, owner, state, reason, sim_ns, decision=decision)


def _predicate_wrapper(original, name):
    @wraps(original)
    def wrapped(node, *args, **kwargs):
        result = original(node, *args, **kwargs)
        context = getattr(_TLS, 'context', None)
        if context is not None and context[0] is node:
            # Only three original subpredicates exist in control_sources_stale.
            try:
                if len(context[1]) < 3:
                    context[1].append((name if name != 'pose_cloud' else args[0], result))
            except Exception:
                probe = getattr(node, '_r102_progress', None)
                if probe is not None:probe.errors += 1
        return result
    return wrapped


def _stale_wrapper(original):
    @wraps(original)
    def wrapped(node, now, ros_now, original_stale):
        probe = getattr(node, '_r102_progress', None)
        if probe is None:
            return original(node, now, ros_now, original_stale)
        previous = getattr(_TLS, 'context', None)
        context = (node, [])
        _TLS.context = context
        try:
            result = original(node, now, ros_now, original_stale)
        except BaseException:
            # Restore TLS and let the exact original exception escape.
            raise
        else:
            steps = context[1]
            reason = next((name for name, fresh in steps if fresh is False),
                'original_aggregate_only' if not steps else 'all_evaluated_fresh')
            probe.observe(node, 'shared_sources', result, reason,
                _finite_seconds_ns(ros_now), [('original_stale', original_stale), *steps],
                source_inputs=(_finite_seconds_ns(now), _scalar(getattr(node,'pose_stamp',None)),
                    _finite_seconds_ns(getattr(node,'pose_updated',None)),
                    _finite_seconds_ns(getattr(node,'pose_timeout',None)),
                    _scalar(getattr(node,'cloud_stamp',None)),
                    _finite_seconds_ns(getattr(node,'cloud_updated',None)),
                    _finite_seconds_ns(getattr(node,'cloud_timeout',None)),
                    _finite_seconds_ns(getattr(getattr(node,'raw_imu',None),'stamp',None)),
                    _finite_seconds_ns(getattr(getattr(node,'raw_imu',None),'updated',None)),
                    _finite_seconds_ns(getattr(getattr(node,'raw_imu',None),'timeout',None))))
            return result
        finally:
            _TLS.context = previous
    return wrapped


def install(node):
    global _INSTALLATION
    option = os.environ.get(KEY, '0')
    if option not in ('0', '1'):
        raise ValueError(KEY+' must be 0 or 1')
    if _INSTALLATION is not None:
        raise RuntimeError('R102 permits one installation per process until restored')
    node._r102_progress = None
    if option == '0':
        return None
    import experiments.candidates.minimal_simulation_baseline as minimal
    import control.navigation_node as teacher_wrapper
    probe = Recorder()
    bindings = [(minimal, 'pose_cloud_fresh', _predicate_wrapper(minimal.pose_cloud_fresh, 'pose_cloud')),
        (minimal, 'raw_imu_fresh', _predicate_wrapper(minimal.raw_imu_fresh, 'raw_imu')),
        (teacher_wrapper.base, 'control_sources_stale', _stale_wrapper(teacher_wrapper.base.control_sources_stale))]
    saved = []
    for module, name, wrapper in bindings:
        original = getattr(module, name)
        saved.append((module, name, original, wrapper))
        setattr(module, name, wrapper)
    node._r102_progress = probe
    _INSTALLATION = (node, saved)
    probe.receipt.update(installed=True, installed_pid=os.getpid(), owner_node_id=id(node),
        targets=[module.__name__+'.'+name for module, name, _, _ in saved],
        original_call_count_policy='exactly once; original exceptions propagate',
        scope='TLS identity restricts subpredicate records to selected node shared-control call')
    return probe


def finish(node):
    """Called after the existing executor join; restore before exit-only write."""
    global _INSTALLATION
    probe = getattr(node, '_r102_progress', None)
    if probe is None or probe.export_attempted:
        return
    probe.export_attempted = True
    probe.closed = True
    try:
        if _INSTALLATION is None or _INSTALLATION[0] is not node:
            probe.receipt['restore_error'] = 'selected_installation_missing'
        else:
            conflicts = []
            for module, name, original, wrapper in reversed(_INSTALLATION[1]):
                if getattr(module, name) is wrapper:
                    setattr(module, name, original)
                else:
                    conflicts.append(module.__name__+'.'+name)
            probe.receipt.update(restored=not conflicts, restore_conflicts=conflicts,
                restored_pid=os.getpid(), after_existing_executor_join=True)
            _INSTALLATION = None
        encoded = json.dumps(probe.snapshot(), allow_nan=False, separators=(',', ':'))+'\n'
        if len(encoded.encode()) > MAX_FILE_BYTES:
            raise RuntimeError('R102 exit file exceeds bound')
        with (Path(node.run)/'r102_progress_transitions.json').open('x') as stream:
            stream.write(encoded)
    except Exception as error:
        node._r102_progress_finish_error = type(error).__name__+':'+str(error)[:160]
        # Existing controller cleanup must still run if diagnostic export fails.
        return False
