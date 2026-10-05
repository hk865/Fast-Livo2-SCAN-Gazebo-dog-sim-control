#!/usr/bin/env python3
"""Post-completion read-only V44 ownership/protection/resource audit.

Prepare now, execute only after root confirms final run completion. No signal,
ROS/simulator launch, callback, actuator command or package/README writer exists.
"""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
HELPER = ROOT / 'test_results/final_protection_audit_20261004/executed_audit.py'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    if run.parent != ROOT / 'runs':
        raise ValueError('Expected one exact Teacher run directory')
    destination = OUT / 'audit.json'
    if destination.exists():
        raise RuntimeError('Never overwrite original audit evidence')
    # Require post-run receipts before hashing any finished-run evidence.
    for name in ('runtime_manifest.json', 'worker_result.json', 'navigation_guard_writer_receipt.json'):
        if not (run / name).exists():
            raise RuntimeError(f'Post-run receipt not yet available: {name}; do not audit a live writer')
    spec = importlib.util.spec_from_file_location('protection_readonly_helper', HELPER)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    previous = read(ROOT / 'test_results/final_protection_audit_20261004/audit.json')
    original_protection = previous['protected']
    def protected_value(value):
        if 'path' in value:
            return helper.file_entry(value['path'], value['sha256'])
        return {k: protected_value(v) for k, v in value.items()}
    protection = {k: protected_value(v) for k, v in original_protection.items()}
    freeze_path = ROOT / 'test_results/navigation_v44_measured_dwell_actual_guard_freeze.json'
    freeze = read(freeze_path)
    live_source_hashes = {name: helper.file_entry(ROOT / name, expected)
                         for name, expected in freeze['source_hashes'].items()}
    archived_sources = {name: helper.file_entry(run / 'sources' / name, expected)
                        for name, expected in freeze['source_hashes'].items() if (run / 'sources' / name).is_file()}
    not_archived = [name for name in freeze['source_hashes'] if name not in archived_sources]
    allowed_not_archived = ['navigation/dynamic/build/obstacle_pose_observer', 'simulation/build/libteacher_actuator.so', 'runs/acceptance.json']
    runtime, worker = read(run / 'runtime_manifest.json'), read(run / 'worker_result.json')
    owned = runtime.get('owned_processes', [])
    required_roles = ['worker', 'gazebo', 'bridge', 'capture', 'shadow', 'navigation_stack']
    exactly_one_owned = Counter(v['role'] for v in owned) == Counter(required_roles)
    clean_owned = exactly_one_owned and all(v.get('returncode') == 0 for v in owned)
    log_path = run / 'navigation_stack.log'
    log = re.sub(r'\x1b\[[0-9;]*m', '', log_path.read_text())
    started = [(a, int(b)) for a, b in re.findall(r'\[([^\[\]]+)\]: process started with pid \[(\d+)\]', log)]
    clean = [(a, int(b)) for a, b in re.findall(r'\[([^\[\]]+)\]: process has finished cleanly \[pid (\d+)\]', log)]
    died = [(a, int(b), int(c)) for a, b, c in re.findall(r'\[([^\[\]]+)\]: process has died \[pid (\d+), exit code (-?\d+)', log)]
    missing = sorted(set(started) - set(clean))
    unexpected = sorted(set(clean) - set(started))
    # No kill(pid,0) probe. Read /proc identities only; recorded PID reuse is not assumed away.
    recorded_pids = sorted({v['pid'] for v in owned} | {pid for _, pid in started})
    still_present = []
    run_processes = []
    training_candidates = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = proc.joinpath('cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
            cmd = ' '.join(argv)
            pid = int(proc.name)
            stat = proc.joinpath('stat').read_text()
            fields = stat[stat.rfind(')') + 2:].split()
            if pid in recorded_pids or str(run) in cmd or 'train_go2' in cmd or '/RL_for_unitree/' in cmd:
                entry = {'pid': pid, 'state': fields[0], 'ppid': int(fields[1]), 'starttime_ticks': int(fields[19]), 'argv': argv}
                if pid in recorded_pids:
                    still_present.append(entry)
                # Audit process itself has exact run arg; exclude its argv by basename.
                if str(run) in cmd and not any(Path(a).name == Path(__file__).name for a in argv):
                    run_processes.append(entry)
                if 'train_go2' in cmd or '/RL_for_unitree/' in cmd:
                    training_candidates.append(entry)
        except (OSError, IndexError, ValueError):
            continue
    guard_receipt = read(run / 'navigation_guard_writer_receipt.json')
    guard_path = run / 'navigation_guard_history.jsonl'
    # Read a small trace only after writer receipt + ownership/proc checks; no live logs rewritten.
    guard_lines = [json.loads(v) for v in guard_path.read_text().splitlines() if v.strip()]
    sequences = [v['sequence'] for v in guard_lines]
    guard_drained = (guard_receipt.get('status') == 'drained' and guard_receipt.get('queue_error') is None
        and sequences == list(range(1, int(guard_receipt['expected_records']) + 1)))
    policy = read(run / 'policy_manifest.json')
    asset = read(run / 'asset_manifest.json')
    with (run/'actuator.jsonl').open() as stream:
        actual_contract = json.loads(next(stream))
        actual_ownership = json.loads(next(stream))
    scope = read(run / 'navigation_scope.json')
    references = {path: helper.file_entry(path, expected) for path, expected in scope['references'].items()}
    evidence_names = ['runtime_manifest.json', 'worker_result.json', 'policy_manifest.json', 'asset_manifest.json',
        'navigation_stack.log', 'navigation_scope.json', 'navigation_guard_writer_receipt.json', 'navigation_guard_history.jsonl']
    def recursively_matches(value):
        if 'baseline_matches' in value:
            return value['baseline_matches']
        return all(recursively_matches(v) for v in value.values() if isinstance(v, dict))
    checks = {
        'protected_model_archives_camera_native_original_acceptance': recursively_matches(protection),
        'v44_frozen_live_runtime_source_hashes': all(v['baseline_matches'] for v in live_source_hashes.values()),
        'archived_runtime_sources_match_v44_freeze': all(v['baseline_matches'] for v in archived_sources.values()) and len(archived_sources) == 29,
        'three_preexisting_binary_acceptance_archive_omissions_explicit': set(not_archived) == set(allowed_not_archived),
        'v44_freeze_receipt_matches_authorized_sha': sha(freeze_path) == '714e75459d5a6087cd147498c955013f2d7d7a16899669b1f1046bd04681314f',
        'actual_scope_source_refs_still_match': all(v['baseline_matches'] for v in references.values()),
        'exact_six_owned_roles_all_exit_zero': clean_owned,
        'all_started_navigation_children_clean': bool(started) and not missing and not unexpected and not died,
        'no_recorded_owned_pid_still_present': not still_present,
        'no_live_process_argv_refers_to_completed_run': not run_processes,
        'runtime_without_error': runtime.get('error') is None and runtime.get('shadow_error') is None,
        'worker_completed_without_fault': worker.get('completed') is True and worker.get('fault') is None,
        'guard_writer_drained_all_sequences_accounted': guard_drained,
        'actor_cpu1_and_no_truth_navigation': policy.get('inference_device') == 'cpu'
            and policy.get('torch_threads') == 1 and policy.get('navigation_truth_used') is False,
        'native_only_joint_executor': asset.get('exclusive_writer') == 'teacher_sim::TeacherActuator'
            and all(v in asset.get('disabled', []) for v in ['CHAMP', 'body_stabilizer', 'ros2_control', 'JointTrajectoryController']),
        'actual_native_one_executor_no_other_writer': actual_contract.get('writer') == 'teacher_sim::TeacherActuator sole JointForceCmd writer' and actual_ownership.get('teacher_writers') == 1 and actual_ownership.get('passed') is True,
        'actual_native_PD_25_05_dt005_decimation4': actual_contract.get('kp') == 25 and actual_contract.get('kd') == .5 and actual_contract.get('expected_dt') == .005 and actual_contract.get('decimation') == 4,
        'simulation_only': runtime.get('real_robot') is False,
    }
    result = {'schema': 'teacher_post_v44_protection_ownership_audit/v1',
        'asof_utc': datetime.now(timezone.utc).isoformat(), 'run': str(run), 'audit_passed': all(checks.values()),
        'scope': 'Post-run read-only protection/ownership/source/resource audit; not a new physical/navigation acceptance or package manifest',
        'executed_source_sha256': sha(__file__), 'read_helper_sha256': sha(HELPER), 'checks': checks,
        'protected': protection, 'v44_freeze': helper.file_entry(freeze_path),
        'frozen_live_sources': live_source_hashes, 'archived_sources': archived_sources, 'not_archived_in_run_sources': not_archived, 'scope_refs': references,
        'ownership': {'required_roles': required_roles, 'owned': owned, 'started_navigation_children': started,
            'clean_navigation_children': clean, 'died_children': died, 'missing_clean_records': missing,
            'unexpected_clean_records': unexpected, 'recorded_pids_still_in_proc': still_present,
            'live_processes_matching_exact_run_argv': run_processes},
        'guard_writer': {'sequence_first_last': [sequences[0],sequences[-1]] if sequences else None, 'receipt': guard_receipt, 'actual_lines': len(guard_lines), 'exact_sequences': guard_drained},
        'current_resource_snapshot': helper.resource_snapshot(),
        'training_protection': {'training_candidate_processes_at_audit': training_candidates,
            'original_training_archive_and_checkpoint_hashes_match': recursively_matches(protection['training_archive']) and recursively_matches(protection['checkpoint']),
            'runtime_claim_training_untouched': runtime.get('training_process_untouched'),
            'audit_performed_signals_training_writes_or_training_launch': False,
            'limits': 'Read-only script sends no signals. Snapshot absence and matching frozen archive hashes do not independently prove all historical process actions; runner process control remains exact own-run scope.'},
        'actual_native_initial_receipts': {'contract': actual_contract, 'ownership': actual_ownership},
        'actual_navigation_acceptance_not_assigned': True,
        'active_command_scopes': {'yaw_rate_radps': scope['profile']['max_yaw_rate_radps'], 'experiment': scope['experiment'],
            'actor_and_physics_not_changed_by_logging': True},
        'evidence_hashes': {name: helper.file_entry(run / name) for name in evidence_names},
        'archive_limits': '29 frozen sources are original run sources copies. Three binaries/global acceptance have no source copies in this run; current SHA matches freeze and protection, but this audit does not falsely count them as archived executed binaries.',
        'global_acceptance_levels': read(ROOT / 'runs/acceptance.json')['levels'],
        'physical_dynamic_pass_claimed_by_this_audit': False,
    }
    shutil.copyfile(HELPER, OUT / 'read_helper.py')
    destination.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    (OUT / 'manifest.json').write_text(json.dumps({p.name: sha(p) for p in OUT.iterdir() if p.is_file()}, indent=2) + '\n')
    print(json.dumps({'passed': result['audit_passed'], 'checks': checks, 'run': run.name,
        'owned': len(owned), 'navigation_children': len(started), 'receipt_sha256': sha(destination), 'output': str(OUT)}))
    if not result['audit_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
