"""Observe worker EOF/archive close after owned physics stop, never signal.

The guardian is not the worker parent: exit code stays UNVERIFIED. Original
owner monitoring and group escalation remain in owned_guardian.py. This adds
no runtime continuation and no navigation acceptance authority.
"""
import hashlib
import json
import math
from pathlib import Path
import time

MAX_GRACE_S = 30.0
ARCHIVE_RESERVE_BYTES = 160_000_000
MAX_NATIVE_S = 1500
MAX_WALL_S = 9090
RUN_CAP_BYTES = 10_000_000_000
SHARED_HARD_BYTES = 80_000_000_000
SHARED_SOFT_BYTES = 79_463_129_088
R85_SHARED_HARD_BYTES = 130_000_000_000
R85_SHARED_SOFT_BYTES = R85_SHARED_HARD_BYTES - 536_870_912
R85_MAX_WALL_S = 4590
R85_DEADLINE_UTC = '2026-10-15T12:00:00+00:00'
NATIVE_TAIL_BYTES = 128 * 1024
ALLOWED_REASONS = {'parent_identity_lost', 'parent_heartbeat_stale'}
PRECEDING_ROLES = ('shadow_sampler','navigation_stack','capture','bridge','gazebo')


def _pipeline_drained(pipeline):
    if not isinstance(pipeline,dict): return False
    if (pipeline.get('normal_completed') is not True or pipeline.get('error') is not None
            or pipeline.get('gate_errors') != []): return False
    summary = pipeline.get('summary')
    return bool(isinstance(summary,dict) and summary.get('normal_completed') is True
        and all(type(summary.get(k)) is int and summary[k]>=0 for k in ('accepted','delivered','committed'))
        and summary['accepted']==summary['delivered']==summary['committed']
        and all(summary.get(k)==0 for k in ('pending','inflight','ready','bytes'))
        and summary.get('failure')=='' and summary.get('uncommitted_packets')==[])


def _preceding_empty(roles,cleanup,validate_members):
    if 'gazebo' not in roles: raise ValueError('native_physics_was_not_registered')
    for role in PRECEDING_ROLES:
        if role not in roles: continue
        record=cleanup.get(role)
        if (not isinstance(record,dict) or record.get('error') is not None
                or record.get('remaining_members') != []):
            raise ValueError('preceding_cleanup_not_confirmed_'+role)
        if validate_members(roles[role]): raise ValueError('preceding_owned_group_live_'+role)


def _r85_limits(run,owner,plan):
    """Full-route or explicit unverified timing lifecycle; never NAV PASS."""
    profile=plan.get('profile',{})
    session=profile.get('r85_session',{})
    from r85_session_guard import session_deadline_utc
    from r85_resource_renewal import retained_hard_bytes
    expected_session=dict(enabled=True,schema='r85_pct_scan_student_session/v1',
        deadline_utc=session_deadline_utc(profile),simulation_only=True,
        maximum_retained_bytes=retained_hard_bytes(profile,R85_SHARED_HARD_BYTES),maximum_round_bytes=RUN_CAP_BYTES)
    if any(session.get(k)!=v for k,v in expected_session.items()):
        raise ValueError('R85_session_archive_contract_changed')
    duration=plan.get('duration_s');profile_duration=profile.get('duration_s');wall=plan.get('wall_budget_s')
    if (type(duration)not in (int,float) or not math.isfinite(duration) or not 30<=duration<=MAX_NATIVE_S
            or type(profile_duration)not in (int,float) or not math.isfinite(profile_duration)
            or profile_duration!=duration
            or type(wall)not in (int,float) or not math.isfinite(wall)
            or not duration+120<=wall<=R85_MAX_WALL_S):
        raise ValueError('R85_frozen_shorter_native_or_wall_bound_invalid')
    from r109_speed_tiers import configuration as speed_configuration
    speed_tier=speed_configuration(profile)
    expected_speed=.2 if speed_tier is None else speed_tier['cruise_mps']
    if (plan.get('schema')!='student_pct_scan_closed_loop_runtime_plan/v1'
            or profile.get('actor_kind')!='student' or profile.get('controller_kind')!='student'
            or profile.get('navigation_ground_truth_used')is not False
            or profile.get('max_speed_mps')!=expected_speed
            or profile.get('cascade',{}).get('desired_speed')!=expected_speed):
        raise ValueError('R85_original_wall_native_actor_or_speed_limit_changed')
    verified_short=False
    if 'r74_short_timing_horizon' in profile:
        from r85_preflight import validate_short_timing_horizon
        verified_short=validate_short_timing_horizon(profile)
    task=profile.get('r85_task',{})
    task_max=task.get('maximum_task_sim_s')
    if (task.get('kind')!='f1_to_f3' or type(task_max)not in (int,float)
            or not math.isfinite(task_max) or not 0<task_max
            or (task_max>duration and not verified_short)
            or task.get('progress_window_sim_s')!=90 or task.get('minimum_progress_m')!=.5):
        raise ValueError('R85_original_full_route_progress_contract_changed')
    # The independent owner binds its plan before admitting any role. Also
    # check the archived profile/task/manifest rather than trusting live files.
    for name,expected in [('runtime_plan.json',owner.get('runtime_plan_sha256')),
                          ('source_manifest.json',plan.get('source_manifest_sha256')),
                          ('r85_task_contract.json',plan.get('r85_task_contract_sha256'))]:
        path=Path(run)/name
        if path.is_symlink() or not path.is_file() or path.stat().st_size>2*1024*1024:
            raise ValueError('R85_archive_binding_not_regular_or_bounded_'+name)
        if not isinstance(expected,str) or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise ValueError('R85_archive_binding_changed_'+name)
    path=Path(run)/'input_profile.json'
    if path.is_symlink() or not path.is_file() or path.stat().st_size>2*1024*1024:
        raise ValueError('R85_archived_profile_missing_or_unbounded')
    if json.loads(path.read_text())!=profile:
        raise ValueError('R85_archived_profile_changed')


def _limits(run,owner,plan):
    if plan.get('profile',{}).get('r85_session',{}).get('enabled')is True:
        _r85_limits(run,owner,plan)
        end=owner.get('wall_deadline')
        if type(end) not in (int,float) or not math.isfinite(end):
            raise ValueError('owner_wall_deadline_missing')
        return float(end)
    startup=plan.get('R33_original_startup')
    if not isinstance(startup,dict): raise ValueError('original_startup_contract_missing')
    path=Path(startup['contract_path'])
    if path.is_symlink() or not path.is_file() or path.stat().st_size>128*1024:
        raise ValueError('startup_contract_not_regular_or_bounded')
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=startup.get('contract_sha256'):
        raise ValueError('startup_contract_hash_changed')
    contract=json.loads(raw)
    expected=dict(max_native_seconds=MAX_NATIVE_S,max_wall_seconds=MAX_WALL_S,
                  new_output_cap_bytes=RUN_CAP_BYTES)
    if any(startup.get(k)!=v or contract.get(k)!=v for k,v in expected.items()):
        raise ValueError('original_runtime_limit_changed_or_unknown')
    if (contract.get('shared_hard_bytes')!=SHARED_HARD_BYTES
            or plan.get('wall_budget_s')!=MAX_WALL_S or plan.get('duration_s')!=MAX_NATIVE_S):
        raise ValueError('original_wall_native_shared_limit_changed')
    end=owner.get('wall_deadline')
    if type(end) not in (int,float) or not math.isfinite(end): raise ValueError('owner_wall_deadline_missing')
    return float(end)


def _last_physics_step(run):
    """Bounded exact tail read; missing/truncated/changed native evidence vetoes."""
    path=Path(run)/'actuator.jsonl'
    if path.is_symlink() or not path.is_file(): raise ValueError('native_physics_tail_missing')
    before=path.stat();start=max(0,before.st_size-NATIVE_TAIL_BYTES)
    with path.open('rb') as stream:
        stream.seek(start);raw=stream.read(NATIVE_TAIL_BYTES)
    after=path.stat()
    keys=('st_dev','st_ino','st_size','st_mtime_ns')
    if any(getattr(before,k)!=getattr(after,k) for k in keys): raise ValueError('native_tail_still_changing')
    if not raw or not raw.endswith(b'\n'): raise ValueError('native_tail_incomplete')
    lines=raw.splitlines(keepends=True)
    offset=start
    if start:
        offset+=len(lines.pop(0))  # First partial line is never parsed/credited.
    latest=None
    for line in lines:
        data=json.loads(line)
        if data.get('kind')=='physics_step':
            if data.get('fault') != 0 or data.get('terminating') is not False:
                raise ValueError('actual_original_native_fault_or_termination')
            t=data.get('t')
            if type(t) not in (int,float) or not math.isfinite(t) or not 0<=t<MAX_NATIVE_S:
                raise ValueError('original_native1500s_limit_or_invalid_clock')
            latest=dict(source_file=str(path),source_offset=offset,source_length=len(line),
                source_line_sha256=hashlib.sha256(line).hexdigest(),file_bytes=before.st_size,
                data={k:data.get(k) for k in ('kind','t','dt','iteration','mode','fault','terminating')})
        offset+=len(line)
    if latest is None: raise ValueError('no_real_physics_step_in_bounded_tail')
    return latest


def observe_archive_close(run,reason,owner,plan,roles,cleanup,pipeline,
        storage_check,previous_storage,identity,validate_members,*,usage_check=None,clock=None,sleep=None):
    """Add at most30s of observations; gates veto before any extra sleep.

    Original storage_check runs synchronously. Its I/O is not preempted; an
    overrun cancels as soon as it returns and earns no additional archive wait.
    No process poll(), exit status inference, signals, retries or budget reset.
    """
    clock=clock or time.monotonic;sleep=sleep or time.sleep
    started=clock();deadline=started+MAX_GRACE_S
    receipt=dict(schema='guardian_worker_archive_close/v1',maximum_grace_s=MAX_GRACE_S,
        archive_reserve_bytes=ARCHIVE_RESERVE_BYTES,started_monotonic_wall=started,
        observed_wait_wall_s=0.,signals_sent_by_observer=[],group_exit_observed=False,
        worker_returncode='UNVERIFIED',navigation_verified=False,reason=None,
        storage_callback_overrun=False,usage_callback_overrun=False,additional_sleep_after_overrun_s=0.)
    def finish(why,exit_observed=False):
        receipt.update(reason=why,group_exit_observed=exit_observed,
                       observed_wait_wall_s=max(0.,clock()-started))
        return receipt
    if reason not in ALLOWED_REASONS: return finish('skip_non_owner_loss_or_hard_stop_reason')
    if 'worker' not in roles: return finish('skip_worker_not_registered')
    if not _pipeline_drained(pipeline): return finish('skip_pipeline_normal_drain_not_confirmed')
    if (Path(run)/'R33_DIAGNOSTIC_STOP.json').exists():
        return finish('skip_existing_original_supervisor_stop')
    try:
        wall_deadline=_limits(run,owner,plan)
        r85=plan.get('profile',{}).get('r85_session',{}).get('enabled')is True
        from r85_resource_renewal import retained_hard_bytes
        shared_hard=retained_hard_bytes(plan['profile'],R85_SHARED_HARD_BYTES) if r85 else SHARED_HARD_BYTES
        shared_soft=shared_hard-536_870_912 if r85 else SHARED_SOFT_BYTES
        wall_limit_reason='cancel_R85_original4590s_wall_limit' if r85 else 'cancel_original9090s_wall_limit'
        receipt['runtime_contract']='R85_original_low_speed_full_route' if r85 else 'R33_original_startup'
        if usage_check is None:
            from storage_guard import usage as measure_usage
            usage_check=lambda:measure_usage([Path(run)])
        previous=previous_storage
        while True:
            if (Path(run)/'R33_DIAGNOSTIC_STOP.json').exists():
                return finish('cancel_original_supervisor_stop_recorded')
            # Recheck identities, absence of physics and all original bounds.
            from owned_guardian import same
            current_parent=identity(owner['parent']['pid'])
            if current_parent is not None:
                if not same(current_parent,owner['parent']):
                    return finish('cancel_owner_identity_anomaly')
                if current_parent.get('state')!='Z' and reason=='parent_identity_lost':
                    return finish('cancel_owner_loss_reason_inconsistent')
            _preceding_empty(roles,cleanup,validate_members)
            rows=validate_members(roles['worker'])
            if clock()>=wall_deadline: return finish(wall_limit_reason)
            if clock()>=deadline: return finish('archive_grace_expired')
            receipt['last_real_native_physics_step']=_last_physics_step(run)
            if clock()>=wall_deadline: return finish(wall_limit_reason)
            if clock()>=deadline: return finish('archive_grace_expired')
            usage=usage_check()
            if clock()>=deadline or clock()>=wall_deadline:
                receipt['usage_callback_overrun']=True
                receipt['storage_callback_overrun']=True
                return finish('cancel_run_usage_callback_exceeded_grace_or_original_wall')
            if (not isinstance(usage,dict) or any(type(usage.get(k)) is not int or usage[k]<0
                    for k in ('logical_bytes','allocated_bytes'))):
                return finish('cancel_run_usage_unknown')
            measured=max(usage['logical_bytes'],usage['allocated_bytes'])
            receipt['last_run_usage']=dict(logical_bytes=usage['logical_bytes'],allocated_bytes=usage['allocated_bytes'])
            if measured+ARCHIVE_RESERVE_BYTES>=RUN_CAP_BYTES:
                return finish('cancel_original10GB_cap_with_archive_reserve')
            callback_start=clock()
            snapshot=storage_check(previous)
            callback_end=clock()
            receipt['last_storage_callback_wall_s']=max(0.,callback_end-callback_start)
            if callback_end>=deadline or callback_end>=wall_deadline:
                receipt['storage_callback_overrun']=True
                return finish('cancel_storage_callback_exceeded_grace_or_original_wall')
            if (not isinstance(snapshot,dict) or snapshot.get('hard_bytes')!=shared_hard
                    or snapshot.get('soft_stop_bytes')!=shared_soft
                    or snapshot.get('stop_reasons')!=[]
                    or type(snapshot.get('conservative_session_growth_bytes')) is not int):
                return finish('cancel_original_storage_gate_or_unknown_snapshot')
            if r85:
                round_budget=snapshot.get('R85_round_budget',{})
                if (snapshot.get('budget_basis')!='current_retained_experiment_data'
                        or snapshot.get('historical_counter_used_for_capacity')is not False
                        or snapshot.get('current_retained_budget_bytes')!=snapshot['conservative_session_growth_bytes']
                        or round_budget.get('hard_B')!=RUN_CAP_BYTES
                        or round_budget.get('hard_stop')is not False
                        or round_budget.get('predictive_soft_stop')is not False
                        or type(round_budget.get('projected_full_stop_B'))is not int
                        or round_budget['projected_full_stop_B']+ARCHIVE_RESERVE_BYTES>=RUN_CAP_BYTES):
                    return finish('cancel_R85_current_retained_or_round_archive_reserve')
            charged=snapshot['conservative_session_growth_bytes']
            receipt['last_storage_gate']=dict(charged_bytes=charged,hard_bytes=snapshot['hard_bytes'],
                soft_stop_bytes=snapshot['soft_stop_bytes'],stop_reasons=snapshot['stop_reasons'])
            if charged<0 or charged+ARCHIVE_RESERVE_BYTES>=shared_soft:
                return finish(('cancel_R85_renewed160GB_soft_gate_with_archive_reserve' if shared_hard==160000000000
                               else 'cancel_R85_original130GB_soft_gate_with_archive_reserve') if r85
                              else 'cancel_original80GB_soft_predictive_gate_with_archive_reserve')
            previous=snapshot
            # Budget callbacks may take time; reread ownership before declaring
            # absence. The guardian cannot convert that absence into returncode0.
            _preceding_empty(roles,cleanup,validate_members)
            if (Path(run)/'R33_DIAGNOSTIC_STOP.json').exists():
                return finish('cancel_original_supervisor_stop_recorded')
            rows=validate_members(roles['worker'])
            if not rows: return finish('worker_owned_group_exit_observed',True)
            leader=identity(roles['worker']['pid'])
            if leader is None: return finish('cancel_worker_leader_identity_unavailable')
            if not same(leader,roles['worker']): return finish('cancel_worker_identity_mismatch')
            now=clock()
            if now>=wall_deadline: return finish(wall_limit_reason)
            if now>=deadline: return finish('archive_grace_expired')
            sleep(min(.05,deadline-now,wall_deadline-now))
    except Exception as error:
        return finish('archive_observation_rejected: '+type(error).__name__+': '+str(error))


def inspect_archive_after_recovery(run,reason,roles,cleanup,pipeline,validate_members):
    """Extra diagnostic only; do not rewrite manifest, status or damaged files."""
    receipt=dict(schema='guardian_worker_archive_integrity/v1',navigation_verified=False,
        worker_returncode='UNVERIFIED',artifact_repaired=False,integrity=None,error=None)
    try:
        if reason not in ALLOWED_REASONS or not _pipeline_drained(pipeline):
            raise ValueError('diagnostic_skipped_original_reason_or_pipeline_gate')
        _preceding_empty(roles,cleanup,validate_members)
        row=cleanup.get('worker')
        if (not isinstance(row,dict) or row.get('error') is not None
                or row.get('remaining_members')!=[] or validate_members(roles['worker'])):
            raise ValueError('worker_owned_exit_not_confirmed')
        from worker_archive_shutdown import inspect_worker_archive
        receipt['integrity']=inspect_worker_archive(run)
    except Exception as error:
        receipt['error']=type(error).__name__+': '+str(error)
    return receipt
