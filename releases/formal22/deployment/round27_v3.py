#!/usr/bin/env python3
"""One reviewed, concrete Round27 trial. Only execute starts the original runner.

No registry writes, business changes, automatic retries, deletion, or navigation PASS.
See README.md for the SHA-pinned specification and independent review formats.
"""
from contextlib import ExitStack, contextmanager
from pathlib import Path
import argparse
import ast
import copy
import datetime as dt
import fcntl
import hashlib
import importlib.util
import json
import os
import re
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True
A = Path(__file__).resolve().parents[1]
D = A.parent
OLD = D / 'structural_recovery_round26_20261023'
SELF = Path(__file__).resolve()
HELPERS = (SELF, OLD / 'admission.py', OLD / 'next_round_deployment.py')
LIMITS = dict(maximum_retained_bytes=170000000000, maximum_round_bytes=10000000000,
              additional_reserved_bytes=10000000000, maximum_runtime_wall_s=2400,
              maximum_cleanup_wall_s=300)


def require(value, reason):
    if not value:
        raise ValueError(reason)


def read(path):
    return json.loads(Path(path).read_text())


def raw(value):
    return (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n').encode()


def typed_equal(left, right):
    """JSON object order is irrelevant; every key, value and type remains exact."""
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return set(left) == set(right) and all(typed_equal(left[k], right[k]) for k in left)
    if isinstance(left, list):
        return len(left) == len(right) and all(typed_equal(a, b) for a, b in zip(left, right))
    return left == right


def actor_snapshot_matches(run, candidate, key, file_name, actor, bound):
    """Verify the original Actor loader's run-local snapshot and frozen origin."""
    run = Path(run).resolve()
    origin = Path(candidate).resolve() / file_name
    snapshot_map = read(run / 'navigation_source_snapshots.json')
    manifest = read(run / 'source_manifest.json')
    references = read(run / 'navigation_scope.json')['references']
    entry = snapshot_map[str(origin)]
    snapshot = Path(entry['snapshot']).resolve()
    actual = Path(actor['actual_' + key + '_path']).resolve()
    if not snapshot.is_relative_to(run / 'sources/navigation') or actual != snapshot:
        return False
    expected = bound[str(origin)]
    hashes = (sha(origin), sha(actual), actor['actual_' + key + '_sha256'], entry['sha256'],
              manifest[str(origin)], manifest[str(snapshot.relative_to(run / 'sources'))],
              references[str(origin)], references[str(snapshot)])
    return all(value == expected for value in hashes)


def closure_summary(value):
    """Keep the complete closure on disk and emit bounded diagnostic context."""
    manifest = value['original_manifest']
    loading = value['actual_loading']
    return dict(schema='r118_actual_closure_summary/v2', run=value['run'],
        source_commit=value['source_commit'], closure_complete=value['closure_complete'],
        closure_checks=value['closure_checks'], actor_archive_verified=value['actor_archive_verified'],
        actual_loading_passed=loading['passed'],
        loading_failed_checks=[k for k, passed in loading.get('checks', {}).items() if not passed],
        error_count=len(value['errors']), runtime_error=str(manifest.get('error'))[:1000],
        first_failure=str(manifest.get('first_failure'))[:1000],
        pipeline_normal_completed=manifest.get('pipeline_normal_completed'),
        formal_navigation_PASS=False)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 ** 2), b''):
            h.update(chunk)
    return h.hexdigest()


def digest(value):
    return hashlib.sha256(value).hexdigest()


def write(path, value):
    with Path(path).open('xb') as stream:
        stream.write(raw(value))


def pin(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=sha(path), bytes=path.stat().st_size)


def pinned(value):
    path = Path(value['path']).resolve()
    require(path.is_file() and not path.is_symlink() and sha(path) == value['sha256'],
            'SHA-pinned input changed: ' + str(path))
    return path


def git(root, *args):
    return subprocess.check_output(['git', *args], cwd=root, text=True).strip()


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def functions(source):
    return [ast.dump(n, include_attributes=False) for n in ast.parse(source).body
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]


def files(candidate):
    return {str(p): sha(p) for p in sorted(candidate.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts and '.git' not in p.parts}


def window(authority, before_start=False):
    require(all(type(authority.get(k)) is int and authority[k] == v for k, v in LIMITS.items()),
            'Exact 170GB/10GB/10GB and 2400+300 limits required')
    start = dt.datetime.fromisoformat(authority['planned_start_UTC'])
    end = dt.datetime.fromisoformat(authority['deadline_utc'])
    require(start.tzinfo is not None and end.tzinfo is not None and
            end == start + dt.timedelta(seconds=2700), 'Window must be planned start plus 2700 seconds')
    require(authority.get('automatic_retry') is False and authority.get('automatic_renewal') is False,
            'No automatic retry or renewal')
    if before_start:
        require(dt.datetime.now(dt.timezone.utc) <= start,
                'Planned start passed; new reviewed trial required, no deadline shifting')
    return start, end


def spec_inputs(spec):
    candidate = Path(spec['candidate']).resolve()
    require(candidate.name == 'candidate' and candidate.is_relative_to(A / 'worktrees'),
            'Only a Round27 isolated candidate may be selected')
    trial = Path(spec['trial']).resolve()
    require(trial.parent == A / 'trials' and re.fullmatch(r'[A-Za-z0-9_-]+', trial.name),
            'Trial must be one new direct Round27 trials child')
    require(re.fullmatch(r'R118_[A-Za-z0-9_]+', spec['kind']), 'Concrete R118 kind required')
    require(type(spec['domain']) is int and 0 <= spec['domain'] <= 232, 'Invalid private DDS domain')
    require(re.fullmatch(r'[A-Za-z0-9_-]+', spec['label']), 'Explicit safe label required')
    environment = spec['environment']
    require(all(type(k) is str and type(v) is str for k, v in environment.items()), 'String environment required')
    require(spec.get('mode') in ('diagnostic', 'formal'), 'Explicit diagnostic/formal mode required')
    if spec['mode'] == 'diagnostic':
        require(environment.get('GO2_R73_MINIMAL_SIMULATION') == '1', 'Diagnostic minimal selector must be 1')
    else:
        require(environment.get('GO2_R73_MINIMAL_SIMULATION', '0') == '0', 'Formal minimal selector must be 0/unset')
    for key, value in dict(R99_CLOCK_FEEDBACK='0',
                           R103_SCAN_SHARED_STORAGE='0', R105_RETAINED_BUDGET_170GB='1',
                           R97_DIAGNOSTIC_QUEUE_BYTES='2097152', PYTHONDONTWRITEBYTECODE='1').items():
        require(environment.get(key) == value, 'Required environment differs: ' + key)
    for path, expected in spec.get('bound_files', {}).items():
        require(sha(path) == expected, 'Frozen external input changed: ' + path)
    require(spec.get('runtime_binding_review_required') is True, 'Root runtime_binding review must be explicit')
    require(spec.get('loading_expectations'), 'Exact root loading expectations required')
    return candidate, trial


def mode_check(spec, candidate, profile):
    original = load(candidate / 'diagnostic_observation.py', 'r118_original_formal_selector')
    formal = original.strict_formal_enabled(profile)
    if spec['mode'] == 'formal':
        require(formal is True, 'Original strict formal selector rejected this profile')
    else:
        require(formal is False and profile.get('diagnostic_observe_only', {}).get('enabled') is True,
                'Diagnostic mode requires diagnostic profile and cannot select formal scope')


def set_runtime_environment(environment):
    # Avoid implicit selection from launcher/admission import or an inherited shell.
    for key in list(os.environ):
        if key == 'GO2_R73_MINIMAL_SIMULATION' or re.match(r'^R[0-9]+_', key):
            os.environ.pop(key)
    os.environ.update(environment)


def propose(spec_path, output):
    spec_path = Path(spec_path).resolve()
    spec = read(spec_path)
    candidate, trial = spec_inputs(spec)
    require(not trial.exists(), 'Trial already exists')
    require(git(candidate.parent, 'rev-parse', 'HEAD') == spec['implementation_commit'] and
            not git(candidate.parent, 'status', '--porcelain'), 'Implementation must be frozen and clean')
    start = dt.datetime.fromisoformat(spec['planned_start_UTC'])
    authority = dict(LIMITS, schema='r118_concrete_single_round_authority/v1',
                     planned_start_UTC=spec['planned_start_UTC'],
                     deadline_utc=(start + dt.timedelta(seconds=2700)).isoformat(),
                     automatic_retry=False, automatic_renewal=False, max_physical_attempts=1,
                     R99_feedback='0', S1='0', kind=spec['kind'], domain=spec['domain'],
                     root_scope=pin(A / 'AUTHORITY.json'), specification=pin(spec_path))
    window(authority, before_start=True)
    # The historical helper is used only for its pure resource-preview function.
    common = load(OLD / 'next_round_deployment.py', 'r118_resource_preview')
    name, proof_raw, resource_source, basis = common.resource_preview(
        candidate, spec['kind'], authority['deadline_utc'], pin(spec_path))
    require(functions(resource_source) == functions((candidate / 'r85_resource_renewal.py').read_text()),
            'Resource function AST changed')
    profile = copy.deepcopy(read(pinned(spec['profile'])))
    require(not profile.get('r86_expired_prefix', {}).get('enabled', False), 'S1 must remain disabled')
    scan = copy.deepcopy(read(pinned(spec['scan_contract'])))
    scan['source_root'] = str(candidate)
    require(scan.get('actual_loaded_binary_witness_required') is True, 'Actual SCAN binary witness required')
    dependencies = {str(Path(scan[k]).resolve()): sha(scan[k]) for k in ('executable', 'config', 'build_receipt')}
    policy = read(candidate / 'POLICY_SELECTION_CONTRACT.json')
    policy['candidate_origin'] = str(candidate / 'policy_candidate/worker.py')
    profile['r85_session'].update(scan_workspace_contract=str(candidate / 'R73_SCAN_RUNTIME_CONTRACT.json'),
        scan_workspace_contract_sha256=digest(raw(scan)), resource_renewal_proof=str(candidate / name),
        resource_renewal_proof_sha256=digest(proof_raw), deadline_utc=authority['deadline_utc'],
        maximum_retained_bytes=170000000000, maximum_round_bytes=10000000000)
    profile['r85_resource_renewal'] = dict(enabled=True, kind=json.loads(proof_raw)['kind'])
    profile['raw_storage_budget']['max_run_bytes'] = 170000000000
    profile['diagnostic_observe_only'].update(user_authorized_work_deadline_utc=authority['deadline_utc'],
                                            work_deadline_basis=basis)
    mode_check(spec, candidate, profile)
    contents = {name: proof_raw.decode(), 'r85_resource_renewal.py': resource_source,
                'R73_SCAN_RUNTIME_CONTRACT.json': raw(scan).decode(),
                'POLICY_SELECTION_CONTRACT.json': raw(policy).decode()}
    proposal = dict(schema='r118_metadata_proposal/v1', specification=pin(spec_path),
        candidate=str(candidate), trial=str(trial), implementation_commit=spec['implementation_commit'],
        authority=authority, profile=profile, profile_sha256=digest(raw(profile)),
        resource_basis=basis, dependencies=dependencies, original_candidate_files=files(candidate),
        proposed_files={k: dict(content=v, sha256=digest(v.encode())) for k, v in contents.items()},
        helper_sha256={str(SELF): sha(SELF), str(OLD / 'next_round_deployment.py'): sha(OLD / 'next_round_deployment.py')},
        actual_loading_verified=False, prepared=False, simulation_started=False, formal_navigation_PASS=False)
    write(output, proposal)
    print(json.dumps(dict(phase='proposal_only', proposal=pin(output))), flush=True)


def apply(proposal_path, review_path, review_sha256):
    require(sha(review_path) == review_sha256, 'Proposal review SHA differs')
    proposal = read(proposal_path)
    review = read(review_path)
    require(review.get('passed') is True and review.get('proposal_sha256') == sha(proposal_path),
            'Independent exact proposal review missing')
    require(review.get('verified_file_sha256', {}).get(str(SELF)) == sha(SELF), 'Tool not independently reviewed')
    spec = read(pinned(proposal['specification']))
    require(spec.get('software_fixture_only') is not True, 'Software-only fixture cannot be applied')
    candidate, trial = spec_inputs(spec)
    window(proposal['authority'], before_start=True)
    require(str(candidate) == proposal['candidate'] and str(trial) == proposal['trial'], 'Proposal paths differ')
    require(git(candidate.parent, 'rev-parse', 'HEAD') == proposal['implementation_commit'] and
            not git(candidate.parent, 'status', '--porcelain'), 'Implementation changed')
    require(files(candidate) == proposal['original_candidate_files'], 'Implementation bytes changed')
    for path, expected in dict(proposal['dependencies'], **proposal['helper_sha256']).items():
        require(sha(path) == expected, 'Proposal dependency/helper changed: ' + path)
    resource = proposal['proposed_files']['r85_resource_renewal.py']['content']
    require(functions(resource) == functions((candidate / 'r85_resource_renewal.py').read_text()),
            'Resource function AST changed')
    require(set(proposal['proposed_files']) == {spec['kind'] + '_RESOURCE_ADMIN_SINGLE_ROUND.json',
        'r85_resource_renewal.py', 'R73_SCAN_RUNTIME_CONTRACT.json', 'POLICY_SELECTION_CONTRACT.json'},
        'Only administrative metadata may be applied')
    for name, item in proposal['proposed_files'].items():
        require(Path(name).name == name and digest(item['content'].encode()) == item['sha256'], 'Invalid proposal content')
    # mkdir is atomic: a failed or interrupted application consumes this trial.
    trial.mkdir()
    write(trial / 'APPLY_INTENT.json', dict(proposal=pin(proposal_path), review=pin(review_path)))
    for name, item in proposal['proposed_files'].items():
        target = candidate / name
        if name == spec['kind'] + '_RESOURCE_ADMIN_SINGLE_ROUND.json':
            with target.open('x') as stream:
                stream.write(item['content'])
        else:
            require(not target.is_symlink(), 'Refuse metadata symlink')
            target.write_text(item['content'])
        require(sha(target) == item['sha256'], 'Applied metadata SHA mismatch')
    write(trial / 'PROFILE.json', proposal['profile'])
    write(trial / 'AUTHORITY.json', proposal['authority'])
    write(trial / 'SPECIFICATION.json', spec)
    write(trial / 'APPLIED.json', dict(proposal=pin(proposal_path), review=pin(review_path),
        candidate=str(candidate), candidate_files=files(candidate), simulation_started=False))
    print(json.dumps(dict(phase='metadata_applied_not_prepared', trial=str(trial),
                          next='Root commits metadata and reviews exact final deployment/binding')), flush=True)


def frozen(trial, require_window=True):
    trial = Path(trial).resolve()
    spec = read(trial / 'SPECIFICATION.json')
    candidate, selected_trial = spec_inputs(spec)
    require(trial == selected_trial, 'Trial identity differs')
    authority = read(trial / 'AUTHORITY.json')
    window(authority, before_start=require_window)
    pinned(authority['root_scope'])
    require(read(pinned(authority['specification'])) == spec, 'Copied specification differs from pinned original')
    applied = read(trial / 'APPLIED.json')
    pinned(applied['proposal']); pinned(applied['review'])
    review = read(trial / 'DEPLOYMENT_REVIEW.json')
    require(review.get('passed') is True and review.get('runtime_binding_reviewed') is True,
            'Independent final deployment/runtime_binding review missing')
    head = git(candidate.parent, 'rev-parse', 'HEAD')
    require(head == review['deployment_commit'] and not git(candidate.parent, 'status', '--porcelain'),
            'Final deployment not clean/frozen')
    require(git(candidate.parent, 'merge-base', '--is-ancestor', spec['implementation_commit'], head) == '',
            'Deployment does not descend from selected implementation')
    allowed = {'candidate/' + name for name in read(applied['proposal']['path'])['proposed_files']}
    allowed.add('candidate/r86_runtime_binding.py')
    changed = set(git(candidate.parent, 'diff', '--name-only', spec['implementation_commit'], head).splitlines())
    require(changed <= allowed, 'Deployment changed business source after implementation freeze: ' + str(changed - allowed))
    current = files(candidate)
    require(review.get('candidate_files') == current, 'Final reviewed candidate bytes changed')
    for path in (*HELPERS, trial / 'PROFILE.json', trial / 'AUTHORITY.json', trial / 'SPECIFICATION.json'):
        require(review.get('verified_file_sha256', {}).get(str(path)) == sha(path), 'Final review pin differs: ' + str(path))
    proposal = read(applied['proposal']['path'])
    require(read(trial / 'PROFILE.json') == proposal['profile'] and authority == proposal['authority'],
            'Applied profile or authority changed')
    mode_check(spec, candidate, read(trial / 'PROFILE.json'))
    for path, expected in proposal['dependencies'].items():
        require(sha(path) == expected, 'Actual dependency changed: ' + path)
    for name, item in proposal['proposed_files'].items():
        require(sha(candidate / name) == item['sha256'], 'Applied administrative metadata changed: ' + name)
    require(current[str(candidate / 'r86_runtime_binding.py')] ==
            review.get('runtime_binding_sha256'), 'Exact root runtime_binding hash not reviewed')
    environment = dict(spec['environment'], R87_CANDIDATE_COMMIT=head)
    bound = dict(current, **{str(p): sha(p) for p in (*HELPERS, trial / 'PROFILE.json', trial / 'AUTHORITY.json',
        trial / 'SPECIFICATION.json', trial / 'DEPLOYMENT_REVIEW.json')})
    bound.update(spec.get('bound_files', {})); bound.update(proposal['dependencies'])
    return spec, candidate, head, environment, bound


def admission_module():
    sys.path.insert(0, str(OLD))
    return load(OLD / 'admission.py', 'r118_original_admission')


def command(spec, candidate, trial, admission):
    return ['/usr/bin/python3', '-B', str(candidate / 'run.py'), '--profile', str(trial / 'PROFILE.json'),
        '--label', spec['label'], '--domain', str(spec['domain']), '--real-time-factor', '1.0',
        '--surface-validity', '0', '--cpu-python', spec.get('cpu_python', '/opt/experiment/user/isaacsim/kit/python/bin/python3'),
        '--run-storage-root', str(admission.R)]


def lifecycle(phase, trial):
    trial = Path(trial).resolve()
    spec, candidate, head, environment, bound = frozen(trial)
    require(Path('/proc/1/comm').read_text().strip() == 'systemd', 'Host namespace required')
    admission = admission_module()
    set_runtime_environment(environment)
    with ExitStack() as stack:
        for path in admission.LOCKS[1:]:
            stream = stack.enter_context(open(path, 'r'))
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        require(not (trial / 'EXECUTION_INTENT.json').exists(), 'Physical attempt already consumed')
        fresh = admission.inspect(domain=spec['domain'], verify_archives=False,
            outer_locks_owned=True, maximum_retained_bytes=170000000000)
        write(trial / ('ADMISSION_' + phase.upper() + '.json'), fresh)
        require(fresh['admitted'], 'Fresh admission refused: ' + str(fresh['stop_reasons']))
        cmd = command(spec, candidate, trial, admission)
        if phase == 'prepare':
            write(trial / 'PREPARE_INTENT.json', dict(command=cmd, source_commit=head, UTC=dt.datetime.now(dt.timezone.utc).isoformat()))
            with (trial / 'PREPARE_OUTPUT.log').open('x') as output:
                result = subprocess.run(cmd + ['--prepare-only'], env=dict(os.environ), stdout=output,
                                        stderr=subprocess.STDOUT, timeout=120)
            require(result.returncode == 0, 'Original prepare failed; preserve log and use a new trial')
            run = Path(json.loads((trial / 'PREPARE_OUTPUT.log').read_text().strip().splitlines()[-1])['run']).resolve()
            require(run.parent == admission.R and spec['label'] in run.name, 'Unexpected new run')
            plan = read(run / 'runtime_plan.json')
            require(plan['profile'] == read(trial / 'PROFILE.json') and plan['ros_domain'] == spec['domain'] and
                    0 < plan['wall_budget_s'] <= 2400, 'Prepared plan differs or exceeds bound')
            write(trial / 'DEPLOYMENT_FREEZE.json', dict(source_commit=head, bound=bound,
                environment=environment, run=str(run), runtime_plan_sha256=sha(run / 'runtime_plan.json'), command=cmd))
            with (trial / 'RUN_PATH.txt').open('x') as output:
                output.write(str(run) + '\n')
            print(json.dumps(dict(phase='prepared_not_started', run=str(run), source_commit=head)), flush=True)
            return
        freeze = read(trial / 'DEPLOYMENT_FREEZE.json')
        run = Path(freeze['run'])
        require(run.parent == admission.R and freeze['source_commit'] == head and freeze['bound'] == bound and
                freeze['environment'] == environment and freeze['command'] == cmd and
                sha(run / 'runtime_plan.json') == freeze['runtime_plan_sha256'] and
                not (run / 'worker_ready').exists() and not (run / 'runtime_manifest.json').exists(),
                'Prepared run/source changed or execution already attempted')
        window(read(trial / 'AUTHORITY.json'), before_start=True)
        write(trial / 'EXECUTION_INTENT.json', dict(freeze, UTC=dt.datetime.now(dt.timezone.utc).isoformat(), no_auto_repeat=True))
        execute_owned(cmd, trial, run, candidate, head, stack)


@contextmanager
def candidate_import_scope(candidate):
    """Own candidate imports, including lazy cleanup imports, until exit completes."""
    candidate = Path(candidate).resolve()
    roots = {path.stem for path in candidate.glob('*.py') if path.stem.isidentifier()}
    roots.update(path.name for path in candidate.iterdir()
                 if path.is_dir() and (path / '__init__.py').is_file())
    def selected(name):
        return name.partition('.')[0] in roots
    saved_modules = {name: module for name, module in sys.modules.items() if selected(name)}
    path_object = sys.path
    saved_path = list(path_object)
    try:
        for name in list(sys.modules):
            if selected(name):
                del sys.modules[name]
        path_object.insert(0, str(candidate))
        yield
    finally:
        for name in list(sys.modules):
            if selected(name):
                del sys.modules[name]
        sys.modules.update(saved_modules)
        path_object[:] = saved_path
        sys.path = path_object


def execute_owned(cmd, trial, run, candidate, head, stack):
    with candidate_import_scope(candidate):
        return _execute_owned(cmd, trial, run, candidate, head, stack)


def _execute_owned(cmd, trial, run, candidate, head, stack):
    from owned_guardian import ident, same, validated_members
    # Original stop routine is preserved; its runner group is the new session only.
    original = load(candidate / 'run.py', 'r118_original_stop_runner')
    log = stack.enter_context((trial / 'ENTRY_OUTPUT.log').open('x'))
    start = time.monotonic()
    child = None; saved = None; stop_requested = False; forced = None; failure = None; cleanup_at = None
    cleanup_errors = []
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    def interrupted(signum, frame):
        raise InterruptedError('Outer launcher received signal ' + str(signum))
    for sig in handlers:
        signal.signal(sig, interrupted)
    try:
        child = subprocess.Popen(cmd + ['--prepared-run', str(run)], env=dict(os.environ),
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        saved = ident(child.pid)
        require(saved and saved['pid'] == saved['pgid'] == saved['session'], 'New runner session identity invalid')
        write(trial / 'ENTRY_IDENTITY.json', dict(runner=saved, source_commit=head,
            source_file=str(candidate / 'run.py'), source_sha256=sha(candidate / 'run.py'), run=str(run)))
        print(json.dumps(dict(phase='original_guarded_runner_started', runner_pid=child.pid,
            runner_identity=saved, run=str(run), source_commit=head)), flush=True)
        while child.poll() is None:
            elapsed = time.monotonic() - start
            if elapsed >= 2400 and not stop_requested:
                require(same(ident(child.pid), saved), 'Runner identity changed; no signal')
                os.kill(child.pid, signal.SIGINT); stop_requested = True; cleanup_at = time.monotonic()
                write(trial / 'OUTER_RUNTIME_STOP.json', dict(elapsed_s=elapsed, signal=int(signal.SIGINT), target=saved))
            if elapsed >= 2670:
                forced = original.stop_owned(child, saved)
                break
            time.sleep(.5)
    except BaseException as error:
        failure = type(error).__name__ + ': ' + str(error)
    finally:
        for sig in handlers:
            signal.signal(sig, signal.SIG_IGN)
        try:
            if child is not None and child.poll() is None:
                if saved and same(ident(child.pid), saved):
                    if not stop_requested:
                        os.kill(child.pid, signal.SIGINT); stop_requested = True; cleanup_at = time.monotonic()
                    deadline = min(start + 2670, (cleanup_at or time.monotonic()) + 270)
                    while child.poll() is None and time.monotonic() < deadline:
                        time.sleep(.25)
                    if child.poll() is None:
                        forced = original.stop_owned(child, saved)
                else:
                    cleanup_errors.append('Runner identity unavailable/changed; no signal sent')
            if child is not None and child.poll() is not None:
                child.wait(timeout=1)
        except BaseException as error:
            cleanup_errors.append(type(error).__name__ + ': ' + str(error))
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
    groups = {}; errors = cleanup_errors
    for path in (run / 'owned_roles').glob('*.identity.json'):
        try:
            groups[path.name] = validated_members(read(path))
        except Exception as error:
            errors.append(str(path) + ': ' + repr(error))
    outcome = dict(schema='r118_bounded_runner_exit/v1', run=str(run), source_commit=head,
        returncode=None if child is None else child.returncode, wall_s=time.monotonic() - start,
        outer_stop_requested=stop_requested, outer_forced_stop=forced, outer_failure=failure,
        runner_still_alive=child is not None and child.poll() is None, remaining_owned_groups=groups,
        identity_errors=errors, no_other_process_signals=True, no_auto_retry=True, navigation_PASS=False)
    write(trial / 'ENTRY_EXIT.json', outcome)
    print(json.dumps(outcome), flush=True)
    if failure or errors or child is None or child.returncode != 0 or any(groups.values()):
        raise SystemExit(2)


def verify_loaded(trial, write_receipt=True, output=None):
    trial = Path(trial).resolve()
    spec, candidate, head, environment, bound = frozen(trial, require_window=False)
    freeze = read(trial / 'DEPLOYMENT_FREEZE.json')
    require(freeze['bound'] == bound and freeze['environment'] == environment, 'Prepared deployment changed')
    run = Path(freeze['run']); checks = {}; errors = []; receipts = {}
    def check(name, operation):
        try:
            checks[name] = bool(operation())
        except Exception as error:
            checks[name] = False; errors.append(name + ': ' + repr(error))
    def receipt(name):
        if name not in receipts:
            receipts[name] = read(run / name)
        return receipts[name]
    def source_matches(path, expected_sha, expected_path=None):
        p = Path(path).resolve()
        return (expected_path is None or str(p) == str(expected_path)) and bound.get(str(p)) == expected_sha == sha(p)
    def modules_match():
        modules = receipt('r86_runtime_binding.json')['modules']
        return bool(modules) and all(source_matches(v['path'], v['sha256']) for v in modules.values())
    check('controller_modules_frozen', modules_match)
    check('candidate_commit', lambda: receipt('r86_runtime_binding.json')['clock_feedback']['candidate_commit'] == head)
    check('R99_OFF', lambda: receipt('r86_runtime_binding.json')['clock_feedback']['environment_value'] == '0' and
          receipt('r86_runtime_binding.json')['clock_feedback']['enabled'] is False)
    for name, key, file_name in (('actor_adapter', 'adapter', 'student_actor_adapter.py'),
                                  ('actor_contract', 'contract', 'newstudent_pid_contract.json')):
        check(name, lambda key=key, file_name=file_name: actor_snapshot_matches(
            run, candidate, key, file_name, receipt('student_actual_actor_loaded.json'), bound))
    profile = read(trial / 'PROFILE.json')
    if 'r109_speed_tier' in profile:
        tier = profile['r109_speed_tier']
        check('actual_speed_helper', lambda: source_matches(
            receipt('student_actual_actor_loaded.json')['actual_speed_tier_helper_path'],
            receipt('student_actual_actor_loaded.json')['actual_speed_tier_helper_sha256'], candidate / 'r109_speed_tiers.py'))
        check('actual_speed_selection_and_bounds', lambda:
            typed_equal(receipt('student_actual_actor_loaded.json')['r109_speed_tier'], tier) and
            typed_equal(tier, receipt('r86_runtime_binding.json')['r109_speed_tier']['selected_profile']) and
            typed_equal(receipt('student_actual_actor_loaded.json')['effective_student_command_bounds'],
                        [tier['student_longitudinal_limit_mps'], .2, .3]))
    def worker_matches():
        worker = receipt('worker_executor_sources_loaded.json')
        return (bool(worker['modules']) and all(source_matches(v['actual_module_file'], v['actual_sha256']) and
            v['actual_sha256'] == v['snapshot_sha256'] == sha(v['snapshot']) for v in worker['modules'].values()) and
            worker['actual_worker_function']['code_filename'] == str(candidate / 'worker.py') and
            worker['entry_argv'][0] == str(candidate / 'ipc_request_window.py'))
    check('worker_modules_and_entry', worker_matches)
    scan_contract = read(candidate / 'R73_SCAN_RUNTIME_CONTRACT.json')
    check('scan_binary', lambda: receipt('scan_loaded_binary.json')['verified'] is True and
          receipt('scan_loaded_binary.json')['actual_executable_path'] == scan_contract['executable'] and
          source_matches(scan_contract['executable'], receipt('scan_loaded_binary.json')['actual_executable_sha256']))
    def startup_matches():
        rows = [json.loads(line) for line in (run / 'scan_epoch_history.jsonl').read_text().splitlines()]
        selected = [r for r in rows if r.get('event') == 'child_startup_joint_identity']
        return bool(selected) and all(r.get('proof', {}).get('accepted') is True for r in selected)
    check('scan_actual_startup_identity', startup_matches)
    # Root binds exact selected flags, modules and actual instance fields. Values
    # use JSON type-sensitive equality, so true cannot stand in for integer 1.
    for index, expectation in enumerate(spec['loading_expectations']):
        name = 'root_expectation_' + str(index)
        def expected(e=expectation):
            value = receipt(e['receipt'])
            for component in e['path']:
                value = value[component]
            return typed_equal(value, e['equals'])
        check(name, expected)
    value = dict(schema='r118_actual_loading/v1', UTC=dt.datetime.now(dt.timezone.utc).isoformat(),
        run=str(run), deployment=head, passed=all(checks.values()), checks=checks, errors=errors,
        receipt_sha256={name: sha(run / name) for name in receipts}, actual_trigger_claimed=False,
        navigation_PASS=False)
    if write_receipt:
        destination = Path(output).resolve() if output else trial / 'ACTUAL_LOADING.json'
        require(destination.parent == trial, 'Actual loading receipt must remain in this trial')
        write(destination, value)
        print(json.dumps(value), flush=True)
    return value


def closure(trial):
    trial = Path(trial).resolve()
    spec = read(trial / 'SPECIFICATION.json')
    candidate = Path(spec['candidate']).resolve()
    require(trial.parent == A / 'trials' and str(trial) == spec['trial'], 'Closure trial identity differs')
    admission = admission_module()
    from owned_guardian import ident, same, validated_members
    from worker_archive_shutdown import inspect_worker_archive
    freeze = read(trial / 'DEPLOYMENT_FREEZE.json'); run = Path(freeze['run'])
    errors = []; roles = {}; checks = {}
    def evidence(path):
        try:
            return read(path)
        except Exception as error:
            errors.append(str(path) + ': ' + repr(error))
            return {}
    manifest = evidence(run / 'runtime_manifest.json')
    entry = evidence(trial / 'ENTRY_EXIT.json')
    identity = evidence(trial / 'ENTRY_IDENTITY.json')
    for path in (run / 'owned_roles').glob('*.identity.json'):
        try:
            roles[path.name] = validated_members(read(path))
        except Exception as error:
            errors.append(str(path) + ': ' + repr(error))
    try:
        fresh = admission.inspect(domain=spec['domain'], verify_archives=False, maximum_retained_bytes=170000000000)
    except Exception as error:
        fresh = {}; errors.append('fresh_admission: ' + repr(error))
    try:
        checks['runner_absent'] = bool(identity and not same(ident(identity['runner']['pid']), identity['runner']))
    except Exception as error:
        checks['runner_absent'] = False; errors.append('runner_identity: ' + repr(error))
    checks['owned_groups_absent'] = not any(roles.values()) and not errors
    checks['entry_exit_recorded'] = bool(entry) and entry.get('runner_still_alive') is False
    checks['private_domain_clear'] = fresh.get('domain', {}).get('empty') is True
    checks['no_matching_runtime'] = fresh.get('processes', {}).get('matching_runtime') == []
    checks['three_locks_available'] = len(fresh.get('locks', [])) == 3 and all(v.get('available') is True for v in fresh['locks'])
    recorded = None; actual = None
    try:
        recorded = read(run / 'worker_archive_integrity.json')
        actual = inspect_worker_archive(run)
    except Exception as error:
        errors.append('new_run_actor_archive: ' + repr(error))
    archive_ok = bool(recorded and actual and actual.get('verified') is True and actual == recorded)
    try:
        loading = verify_loaded(trial, write_receipt=False)
    except Exception as error:
        loading = dict(passed=False, error=repr(error)); errors.append('actual_loading: ' + repr(error))
    value = dict(schema='r118_actual_closure/v1', UTC=dt.datetime.now(dt.timezone.utc).isoformat(),
        run=str(run), source_commit=freeze['source_commit'], closure_checks=checks,
        closure_complete=all(checks.values()), actor_archive_verified=archive_ok,
        worker_archive_recorded=recorded, worker_archive_fresh=actual, actual_loading=loading,
        live_owned_members=roles, errors=errors, entry_exit=entry, original_manifest=manifest,
        next_round_admission=fresh, signals_sent=False, source_or_data_deleted=False,
        formal_navigation_PASS=False)
    write(trial / 'POST_RUN_CLOSURE.json', value)
    print(json.dumps(closure_summary(value)), flush=True)
    if not value['closure_complete'] or not archive_ok or not loading['passed']:
        raise SystemExit(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='phase', required=True)
    p = sub.add_parser('propose'); p.add_argument('--spec', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('apply'); p.add_argument('--proposal', type=Path, required=True); p.add_argument('--review', type=Path, required=True); p.add_argument('--review-sha256', required=True)
    for phase in ('prepare', 'execute', 'verify-loaded', 'closure'):
        p = sub.add_parser(phase); p.add_argument('--trial', type=Path, required=True)
        if phase == 'verify-loaded': p.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        if args.phase == 'propose': propose(args.spec, args.output)
        elif args.phase == 'apply': apply(args.proposal, args.review, args.review_sha256)
        elif args.phase in ('prepare', 'execute'): lifecycle(args.phase, args.trial)
        elif args.phase == 'verify-loaded':
            if not verify_loaded(args.trial, output=args.output)['passed']: raise SystemExit(2)
        else: closure(args.trial)
    except Exception as error:
        target = getattr(args, 'trial', None)
        if target is not None:
            target = target.resolve()
            if target.parent == A / 'trials' and target.is_dir():
                write(target / (args.phase.upper() + '_FAILURE_' + str(time.time_ns()) + '.json'),
                    dict(phase=args.phase, error=type(error).__name__ + ': ' + str(error),
                         UTC=dt.datetime.now(dt.timezone.utc).isoformat(), automatic_retry=False))
        raise


if __name__ == '__main__':
    main()
