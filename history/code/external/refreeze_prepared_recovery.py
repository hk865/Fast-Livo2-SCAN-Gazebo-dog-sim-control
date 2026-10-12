"""Integration-owner reboot recovery freeze; same R39, no runtime execution."""
import datetime as dt
import hashlib
import json
import sys
from pathlib import Path

P = Path(__file__).resolve().parent
N = P.parent
T = N.parent
C = Path('/__LOCAL_HOME__/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/engineering_v34_r39_p1_association')
R = C.parents[1] / 'runs/20261009T225944Z_closed_loop_cascade_clock_hold_R39_P1_association_diagnostic_e790'
sys.dont_write_bytecode = True
sys.path.insert(0, str(C))
import corridor_preflight as cp
import pid_scope as scope
import policy_selection as policy
import run as runner
from full46_launch_contract import PROFILE_NAMES

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')

assert not (N / 'ONE_SIMULATION_LATCH.json').exists(), 'R39 already attempted'
assert not (R / 'R33_execution_request.json').exists(), 'Runtime request already exists'
assert json.loads((R / 'run_result.json').read_text())['status'] == 'prepared_unverified'
assert not (P / 'RECOVERY_SOURCE_FREEZE.json').exists(), 'Recovery already frozen'
root_review = json.loads((P / 'ROOT_RECOVERY_SOURCE_REVIEW.json').read_text())
assert root_review['review_passed'] is True and root_review['blocking_findings'] == []
for path, digest in root_review['source_sha256'].items():
    assert sha(path) == digest, path
old_gate = json.loads((P / 'BEFORE_R33_STARTUP_PREFLIGHT.json').read_text())
assert sha(C / cp.GATE_NAME) == sha(P / 'BEFORE_R33_STARTUP_PREFLIGHT.json')
changed = {str(C / name) for name in ('storage_guard.py', 'corridor_preflight.py')}
for path, digest in old_gate['verified_inputs_sha256'].items():
    if path not in changed:
        assert sha(path) == digest, 'Unreviewed input changed: ' + path
protected = C.parent / 'engineering_v34_r38_event_any_phase' / cp.GATE_NAME
assert sha(protected) == '8bcfa128f80cea0a5aa6efbd2afb65db79e6f3bca1814bfa9b7e6bed993b9ddb'
protected_gate = json.loads(protected.read_text())
for path, digest in protected_gate['verified_inputs_sha256'].items():
    assert sha(path) == digest, 'Protected R38 changed: ' + path

tests = [N / 'FINITE_CPU_RECEIPT.json', P / 'RECOVERY_FINITE_TEST_RECEIPT.json']
reviews = [N / 'INDEPENDENT_REVIEW_ADMISSION.json', N / 'ROOT_RELEASE_RECONCILIATION.json',
           N / 'reboot_review/FINAL_INDEPENDENT_RECOVERY_REVIEW.json', P / 'ROOT_RECOVERY_SOURCE_REVIEW.json']
gate = cp.freeze_source_review(C, tests, reviews, root_review_complete=True)
# Retain all older immutable proof bindings; update only the explicit recovery delta.
for path in old_gate['verified_inputs_sha256']:
    gate['verified_inputs_sha256'][path] = sha(path)
extra = [P / name for name in ('RECOVERY_AUTHORIZATION.json', 'MISSING_TMP_HISTORY.json',
    'RECOVERY_SOURCE_DIFF.patch', 'BEFORE_STORAGE_SESSION.json', 'BEFORE_storage_guard.py',
    'BEFORE_corridor_preflight.py', 'BEFORE_R33_STARTUP_PREFLIGHT.json',
    'BEFORE_R33_STARTUP_SOURCE_REVIEW.json', 'STORAGE_REBOOT_RECOVERY_CONTRACT.json',
    'storage_guard.py', 'corridor_preflight.py', 'storage_recovery.py', 'recovery_tests.py')]
extra += [Path(__file__).resolve(), N / 'reboot_review/INITIAL_READONLY_INTAKE.json']
for path in extra:
    gate['verified_inputs_sha256'][str(path)] = sha(path)
gate['verified_inputs_sha256'] = dict(sorted(gate['verified_inputs_sha256'].items()))
write(C / cp.GATE_NAME, gate)
for name in PROFILE_NAMES:
    cp.verify_preflight(C, json.loads((C / 'profiles' / name).read_text()))

backup = P / 'prepared_before_recovery'
for name in ('runtime_plan.json', 'navigation_scope.json', 'source_manifest.json',
             'navigation_source_snapshots.json', 'run_result.json'):
    assert (backup / name).is_file() and sha(backup / name) == sha(R / name), name
plan = json.loads((R / 'runtime_plan.json').read_text())
frozen = json.loads((R / 'navigation_scope.json').read_text())
refs = frozen['references']
snapshots = json.loads((R / 'navigation_source_snapshots.json').read_text())
manifest = json.loads((R / 'source_manifest.json').read_text())
selected = policy.selection()
updated = []
inputs = {Path(x).resolve() for x in scope.runtime_files(plan['profile'])}
inputs |= {Path(x) for x in refs if not Path(x).is_relative_to(R)}
for original in sorted(inputs):
    digest = sha(original)
    source = str(original)
    old_digest = refs.get(source)
    rel = policy.archive_relative(original, scope.ROOT, selected)
    target = R / 'sources' / rel
    if old_digest != digest or source not in snapshots:
        if target.exists():
            # Four modified canonical source snapshots have exact retained pre-recovery copies.
            assert original.parent == C and original.name in (
                'storage_guard.py', 'corridor_preflight.py', cp.GATE_NAME, cp.REVIEW_NAME), source
            assert sha(backup / ('snapshot_' + original.name)) == sha(target), source
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(original.read_bytes())
        updated.append({'source': source, 'old_sha256': old_digest, 'sha256': digest,
                        'snapshot': str(target)})
    assert sha(target) == digest
    refs[source] = digest
    refs[str(target)] = digest
    snapshots[source] = {'snapshot': str(target), 'sha256': digest}
    manifest[str(rel)] = digest
    manifest[source] = digest
write(R / 'source_manifest.json', manifest)
write(R / 'navigation_source_snapshots.json', snapshots)
for name in ('source_manifest.json', 'navigation_source_snapshots.json'):
    refs[str(R / name)] = sha(R / name)
write(R / 'navigation_scope.json', frozen)
plan['source_manifest_sha256'] = sha(R / 'source_manifest.json')
plan['scope_sha256'] = sha(R / 'navigation_scope.json')
plan['reboot_storage_recovery'] = {
    'contract_path': str(C / 'STORAGE_REBOOT_RECOVERY_CONTRACT.json'),
    'contract_sha256': sha(C / 'STORAGE_REBOOT_RECOVERY_CONTRACT.json'),
    'historical_floor_bytes': 47772774400,
    'same_prepared_run': True, 'historical_missing_records_restored': False,
    'navigation_localization_Actor_native_and_thresholds_unchanged': True,
    'authorization_file': str(P / 'RECOVERY_AUTHORIZATION.json')}
write(R / 'runtime_plan.json', plan)
write(R / 'run_result.json', dict(run=str(R), status='prepared_unverified', ROS_started=False,
    Gazebo_started=False, runtime_plan_sha256=sha(R / 'runtime_plan.json'),
    navigation_scope_sha256=sha(R / 'navigation_scope.json')))
verified = scope.verify_scope(R / 'navigation_scope.json')
runner.validate_profile(plan['profile'])
assert sha(T / 'STORAGE_SESSION.json') == sha(P / 'BEFORE_STORAGE_SESSION.json')
assert sha(protected) == '8bcfa128f80cea0a5aa6efbd2afb65db79e6f3bca1814bfa9b7e6bed993b9ddb'
receipt = dict(schema='R39_reboot_recovery_same_prepared_freeze/v1',
    utc=dt.datetime.now(dt.timezone.utc).isoformat(), status='PASS_LIMITED_SOURCE_FREEZE',
    run=str(R), attempt=0, execution_started=False, navigation_PASS=False,
    old_gate_bindings=len(old_gate['verified_inputs_sha256']),
    gate_bindings=len(gate['verified_inputs_sha256']), scope_references=len(refs),
    source_snapshots=len(snapshots), updated_source_snapshots=updated,
    old_prepared_metadata=str(backup), mutable_accounting_files_frozen_as_bytes=False,
    source_gate_sha256=sha(C / cp.GATE_NAME), source_review_sha256=sha(C / cp.REVIEW_NAME),
    runtime_plan_sha256=sha(R / 'runtime_plan.json'), scope_sha256=sha(R / 'navigation_scope.json'),
    source_manifest_sha256=sha(R / 'source_manifest.json'),
    shared_storage_session_sha256=sha(T / 'STORAGE_SESSION.json'),
    original47_77GB_historical_floor_retained=True,
    same_world_profile_sensors_capture_inputs_and_native_policy_bytes=True,
    protected_R38_sha256=sha(protected), all_five_profiles_verified=True)
write(P / 'RECOVERY_SOURCE_FREEZE.json', receipt)
print(json.dumps({k:v for k,v in receipt.items() if k != 'updated_source_snapshots'}, indent=2))
