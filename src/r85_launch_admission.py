"""Admission for an owned R85 launch, never historical acceptance or outcome.

The SLAM include executes inside the registered navigation_stack process.
This module checks ownership without launching, signalling, or changing a lock.
"""
from pathlib import Path
import datetime as dt
import hashlib
import json
import math
import os
import stat
import time

DEADLINE = dt.datetime(2026, 10, 15, 12, tzinfo=dt.timezone.utc)
LOCK = Path('/tmp/go2_pid_physics_serial.lock')
IDENTITY_KEYS = ('pid', 'start_ticks', 'pgid', 'session', 'boot_id')


def fail(message):
    raise RuntimeError('R85 launch admission: ' + message)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict):
        fail('JSON document required: ' + str(path))
    return value


def process_identity(pid, boot_id):
    raw = Path(f'/proc/{pid}/stat').read_text()
    fields = raw[raw.rfind(')') + 2:].split()
    return dict(pid=int(pid), state=fields[0], ppid=int(fields[1]),
                pgid=int(fields[2]), session=int(fields[3]),
                start_ticks=int(fields[19]), boot_id=boot_id)


def same_identity(actual, saved):
    return bool(isinstance(actual, dict) and isinstance(saved, dict)
                and actual.get('state') not in ('Z', 'X')
                and all(actual.get(k) == saved.get(k) for k in IDENTITY_KEYS)
                and all(type(actual.get(k)) is int and actual[k] > 0
                        for k in ('pid', 'start_ticks', 'pgid', 'session')))


def held_serial_lock(raw, lock_stat, root_pid):
    """Match an actual granted whole-file exclusive flock, including device."""
    target = (os.major(lock_stat.st_dev), os.minor(lock_stat.st_dev), lock_stat.st_ino)
    for line in raw.splitlines():
        fields = line.split()
        if len(fields) != 8 or fields[1:4] != ['FLOCK', 'ADVISORY', 'WRITE']:
            continue
        try:
            device = fields[5].split(':')
            identity = (int(device[0], 16), int(device[1], 16), int(device[2], 10))
            if (int(fields[4]) == root_pid and identity == target
                    and fields[6:] == ['0', 'EOF']):
                return True
        except (ValueError, IndexError):
            continue
    return False


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_owned_runtime(run, here, profile, plan, documents, actual,
                           cmdlines, environment, locks_raw, lock_stat,
                           monotonic_wall, utc_now):
    """Pure guard: all process/clock/lock observations are explicit inputs."""
    run = Path(run).resolve(); here = Path(here).resolve()
    owner = documents['owner']; nav = documents['navigation']; permit = documents['permit']
    ready = documents['ready']; parent = owner.get('parent', {})
    request = documents['runner_request']
    token = owner.get('token')
    if (not isinstance(token, str) or len(token) != 32
            or any(c not in '0123456789abcdef' for c in token)):
        fail('invalid owner token')
    if owner.get('allowed_roles', []).count('navigation_stack') != 1:
        fail('navigation role not uniquely authorized')
    if (nav.get('role') != 'navigation_stack' or nav.get('owned_run') != str(run)
            or nav.get('ownership_token') != token or permit.get('token') != token
            or permit.get('identity') != nav or ready.get('token') != token):
        fail('navigation registration/permit token differs')
    if (not same_identity(actual.get('root'), parent)
            or not same_identity(actual.get('navigation'), nav)
            or not same_identity(actual.get('guardian'), ready.get('identity'))):
        fail('actual root/navigation/guardian identity differs')
    if not all(row.get('boot_id') == parent.get('boot_id')
               for row in (nav, ready['identity'])):
        fail('boot identity differs')
    for name, row in [('root', parent), ('navigation', nav), ('guardian', ready['identity'])]:
        if row['pid'] != row['pgid'] or row['pid'] != row['session']:
            fail(name + ' is not its recorded process-group/session leader')
    if (actual['root'].get('ppid') != parent.get('ppid')
            or nav.get('ppid') != parent['pid']
            or actual['navigation'].get('ppid') != parent['pid']
            or ready['identity'].get('ppid') != parent['pid']
            or actual['guardian'].get('ppid') != parent['pid']
            or nav['start_ticks'] < parent['start_ticks']
            or ready['identity']['start_ticks'] < parent['start_ticks']):
        fail('actual parent/root launch ancestry differs')
    root_cmd = cmdlines.get('root', [])
    # Both supported invocation forms are ordinary Python CLI execution.
    offset = 2 if len(root_cmd) > 1 and root_cmd[1] == '-B' else 1
    if (len(root_cmd) <= offset or Path(root_cmd[offset]).resolve() != here / 'run.py'
            or 'python' not in Path(root_cmd[0]).name
            or '--prepare-only' in root_cmd):
        fail('actual root command is not the active candidate runner')
    if '--prepared-run' in root_cmd:
        index = root_cmd.index('--prepared-run')
        if index + 1 >= len(root_cmd) or Path(root_cmd[index + 1]).resolve() != run:
            fail('actual root prepared-run target differs')
    elif (request.get('prepare_only') is not False
          or root_cmd[offset:] != request.get('argv')):
        fail('actual root CLI differs from current run creation request')
    if (request.get('schema') != 'r85_closed_loop_runner_request/v1'
            or request.get('run') != str(run)
            or request.get('simulation_only') is not True):
        fail('current runner request scope/run differs')
    nav_cmd = cmdlines.get('navigation', [])
    wanted = ['launch', str(here / 'stack.launch.py'), 'run_dir:=' + str(run)]
    if not any(nav_cmd[i:i + 3] == wanted for i in range(len(nav_cmd) - 2)):
        fail('actual navigation command is not the candidate stack/run')
    guardian_cmd = cmdlines.get('guardian', [])
    wanted = [str(here / 'owned_guardian.py'), '--watch', str(run)]
    if not any(guardian_cmd[i:i + 3] == wanted for i in range(len(guardian_cmd) - 2)):
        fail('actual guardian command differs')
    if (plan.get('run') != str(run) or plan.get('profile') != profile
            or plan.get('allowed') is not True or plan.get('simulation_only') is not True
            or plan.get('real_robot') is not False or plan.get('navigation_ground_truth_used') is not False
            or plan.get('schema') != 'student_pct_scan_closed_loop_runtime_plan/v1'):
        fail('runtime plan scope/run/profile differs')
    if type(plan.get('ros_domain')) is not int or not 1 <= plan['ros_domain'] <= 232:
        fail('private ROS domain required')
    expected_env = dict(ROS_DOMAIN_ID=str(plan['ros_domain']),
        GZ_PARTITION='teacher_closed_loop_' + run.name, DEMO_RUN_DIR=str(run),
        TEACHER_OWNED_RUN_TOKEN=token)
    if any(environment.get(k) != v for k, v in expected_env.items()):
        fail('actual navigation domain/partition/run/owner environment differs')
    recorded_env = documents['environment']
    if any(recorded_env.get(k) != v for k, v in expected_env.items()
           if k != 'TEACHER_OWNED_RUN_TOKEN'):
        fail('recorded navigation environment differs')
    for key, value in plan.get('navigation_stack_diagnostic_environment', {}).items():
        if environment.get(key) != value or recorded_env.get(key) != value:
            fail('actual/recorded SLAM diagnostic environment differs: ' + key)
    for name in ('guardian_heartbeat', 'runner_heartbeat'):
        heartbeat = documents[name]
        stamp = heartbeat.get('monotonic_wall')
        if not _finite(stamp) or not 0 <= monotonic_wall - stamp < 3.:
            fail(name + ' is stale or from the future')
    if documents['guardian_heartbeat'].get('state') != 'RUNNING':
        fail('guardian is not RUNNING')
    budget = plan.get('wall_budget_s'); stop = owner.get('wall_deadline')
    if (not _finite(budget) or not _finite(stop) or budget <= 0
            or not 0 < stop - monotonic_wall <= budget + 1.):
        fail('actual Guardian time plan expired or exceeds frozen wall budget')
    from runtime_guard import session_deadline_utc,session_deadline
    if (profile.get('r85_session', {}).get('enabled') is not True
            or profile['r85_session'].get('deadline_utc') != session_deadline_utc(profile)
            or utc_now.tzinfo is None):
        fail('explicit session time scope differs')
    effective_deadline = session_deadline(profile)
    if (effective_deadline is not None
            and (utc_now >= effective_deadline
                  or stop - monotonic_wall > (effective_deadline - utc_now).total_seconds())):
        fail('overnight deadline exceeded by current time plan')
    if not stat.S_ISREG(lock_stat.st_mode) or not held_serial_lock(locks_raw, lock_stat, parent['pid']):
        fail('actual runner does not hold the shared exclusive physics flock')
    return dict(schema='r85_owned_launch_admission/v1', run_id=run.name,
                root_identity=actual['root'], navigation_identity=actual['navigation'],
                guardian_identity=actual['guardian'], serial_lock=str(LOCK),
                physics_lock_owner_pid=parent['pid'], simulation_only=True,
                old_R83_acceptance_inherited=False, navigation_is_verified=False)


def verify_launch(run, here, profile=None):
    """Verify frozen inputs and live ownership immediately before SLAM launch."""
    run = Path(run).resolve(); here = Path(here).resolve()
    from pid_scope import verify_scope
    from r85_preflight import validate_profile
    receipt = verify_scope(run / 'navigation_scope.json')
    actual_profile = read_json(run / 'navigation_profile.json')
    if profile is not None and profile != actual_profile:
        fail('caller profile differs from frozen run profile')
    profile = actual_profile
    validate_profile(profile, here)
    if receipt.get('profile') != profile or receipt.get('controller_kind') != 'student':
        fail('verified Student scope/profile differs')
    plan = read_json(run / 'runtime_plan.json')
    files = dict(owner='guardian_owner.json', navigation='owned_roles/navigation_stack.identity.json',
                 permit='owned_roles/navigation_stack.permit.json', ready='guardian_ready.json',
                 guardian_heartbeat='guardian_heartbeat.json', runner_heartbeat='runner_heartbeat.json',
                 environment='navigation_stack_effective_environment.json',
                 runner_request='runner_request.json')
    documents = {key: read_json(run / name) for key, name in files.items()}
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    ids = dict(root=documents['owner']['parent']['pid'], navigation=os.getpid(),
               guardian=documents['ready']['identity']['pid'])
    actual = {key: process_identity(pid, boot) for key, pid in ids.items()}
    cmdlines = {key: [s.decode() for s in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if s]
                for key, pid in ids.items()}
    result = validate_owned_runtime(run, here, profile, plan, documents, actual, cmdlines,
        os.environ, Path('/proc/locks').read_text(), LOCK.lstat(), time.monotonic(),
        dt.datetime.now(dt.timezone.utc))
    # Recheck PID reuse or reparenting across hashes/proc reads before returning.
    for key, pid in ids.items():
        after = process_identity(pid, boot)
        if not same_identity(after, actual[key]) or after['ppid'] != actual[key]['ppid']:
            fail('process identity changed during launch admission: ' + key)
    result.update(actual_utc=dt.datetime.now(dt.timezone.utc).isoformat(),actual_monotonic_wall=time.monotonic())
    return result
