"""Classify the original CPU exception; permit only bounded, post-physics close.

No process signals, sampling, file writes, navigation decisions or guard changes.
"""
from dataclasses import dataclass
import math

CPU_MESSAGE = 'Shared CPU load guard exceeded; only own run will be stopped'
MAXIMUM_WAIT_S = 30.0
FINISH_RESERVE_S = 35.0
CLEANUP_LIMIT_S = 300.0


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class CPULoadStopEvidence:
    original_error: RuntimeError
    original_guard: object
    run_id: str
    sample_monotonic: float
    logical_cpus: int
    load1: float
    non_cpu_eligible: bool
    non_cpu_reason: str

    def record(self):
        return dict(schema='r100_original_cpu_resource_stop/v1',
            original_exception_type=type(self.original_error).__name__,
            original_message=str(self.original_error), run_id=self.run_id,
            sample_monotonic=self.sample_monotonic, logical_cpus=self.logical_cpus,
            load1=self.load1, original_threshold=.9*self.logical_cpus,
            non_cpu_eligible=self.non_cpu_eligible, non_cpu_reason=self.non_cpu_reason,
            navigation_PASS=False)


def _classify(error, guard, sample, run_id):
    if type(error) is not RuntimeError or str(error) != CPU_MESSAGE:
        return None
    cpus, load, stamp = sample['logical_cpus'], sample['load_1_5_15'][0], sample['monotonic_wall']
    if (type(cpus) is not int or cpus <= 0 or not _finite(load)
            or not _finite(stamp) or load <= .9*cpus or not isinstance(run_id, str) or not run_id):
        return None
    ram = sample.get('memory_KiB', {}).get('MemAvailable')
    gpu = sample.get('gpu', {})
    used, total = gpu.get('used_MiB'), gpu.get('total_MiB')
    if not _finite(ram) or ram < 4*1024*1024:
        reason = 'same sample RAM headroom unavailable or insufficient'
    elif (not _finite(used) or not _finite(total) or used < 0 or total <= 0
            or used > min(14000, total-2048)):
        reason = 'same sample GPU headroom unavailable or insufficient'
    else:
        reason = None
    return CPULoadStopEvidence(error, guard, run_id, float(stamp), cpus,
                               float(load), reason is None, reason)


def checked_resource_guard(guard, sample, run_id):
    """Call the unchanged guard once; preserve its exact exception and traceback."""
    try:
        return guard(sample)
    except Exception as error:
        try:
            evidence = _classify(error, guard, sample, run_id)
            if evidence is not None:
                error._r100_cpu_stop = evidence
                if getattr(error, 'observation', None) is None:
                    error.observation = evidence.record()
        except Exception:
            pass  # Diagnostics must not replace the original guard exception.
        raise


def cpu_resource_stop(error, run_id, guard):
    evidence = getattr(error, '_r100_cpu_stop', None)
    if (type(evidence) is CPULoadStopEvidence and evidence.original_error is error
            and type(error) is RuntimeError and str(error) == CPU_MESSAGE
            and evidence.original_guard is guard and evidence.run_id == run_id):
        return evidence
    return None


def archive_budget(evidence, *, now, cleanup_started, runner_deadline,
                   guardian_deadline, hard_utc_deadline):
    """Fixed absolute bounds, never extending any original deadline."""
    result = dict(schema='r100_cpu_archive_budget/v1', eligible=False,
        maximum_wait_s=0.0, deadline_monotonic=None, reason=None,
        configured_maximum_s=MAXIMUM_WAIT_S, finish_reserve_s=FINISH_RESERVE_S,
        cleanup_limit_s=CLEANUP_LIMIT_S, navigation_PASS=False)
    if type(evidence) is not CPULoadStopEvidence:
        result['reason'] = 'no original typed CPU stop'
    elif not evidence.non_cpu_eligible:
        result['reason'] = evidence.non_cpu_reason
    elif not all(_finite(v) for v in (now, cleanup_started, runner_deadline,
                                      guardian_deadline, hard_utc_deadline)):
        result['reason'] = 'original hard deadline unavailable or invalid'
    else:
        deadlines = dict(cleanup=cleanup_started+CLEANUP_LIMIT_S, runner=runner_deadline,
                         guardian=guardian_deadline, hard_utc=hard_utc_deadline)
        deadline = min(now+MAXIMUM_WAIT_S, *(v-FINISH_RESERVE_S for v in deadlines.values()))
        result.update(original_deadlines_monotonic=deadlines,
                      deadline_monotonic=deadline, maximum_wait_s=max(0.0, deadline-now))
        result['eligible'] = deadline > now
        result['reason'] = None if result['eligible'] else 'original deadline finish reserve exhausted'
    return result


def fresh_motion_stop_reason(cleanup, identities, validate_members):
    """Revalidate registered command/physics groups at every grace poll."""
    physics = cleanup.get('gazebo', {})
    if not (physics.get('started') is True and physics.get('returncode') == 0
            and physics.get('remaining_owned_group_members') == []):
        return 'native physics normal stop not confirmed'
    try:
        for role, saved in identities.items():
            if role == 'worker':
                continue
            record = cleanup.get(role, {})
            if (not saved or record.get('started') is not True or record.get('returncode') != 0
                    or record.get('remaining_owned_group_members') != []
                    or record.get('identity_mismatch') or record.get('identity_unavailable')):
                return 'registered command/physics cleanup incomplete: '+role
            if validate_members(saved):
                return 'registered command/physics group still live: '+role
    except Exception as error:
        return 'fresh command/physics identity check rejected: '+type(error).__name__+': '+str(error)
    return None
