#!/usr/bin/env python3
"""Independent strict process-exit supplement; never changes frozen functional receipts."""
import argparse
import hashlib
import json
from pathlib import Path
import re


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evaluate(run, write=True):
    run = Path(run).resolve()
    runtime = json.loads((run / 'runtime_manifest.json').read_text())
    worker = json.loads((run / 'worker_result.json').read_text())
    original = run / 'summary_dynamic_obstacle_independent.json'
    before = original.read_bytes() if original.is_file() else None
    functional = json.loads(before) if before else {}
    roles = ['worker', 'gazebo', 'bridge', 'capture', 'shadow', 'navigation_stack']
    processes = runtime.get('owned_processes', [])
    by_role = {role: [p for p in processes if p.get('role') == role] for role in roles}
    complete = all(len(value) == 1 for value in by_role.values()) and len(processes) == len(roles)
    zero = complete and all(value[0].get('returncode') == 0 for value in by_role.values())
    log = run / 'navigation_stack.log'
    text = log.read_text(errors='replace') if log.exists() else ''
    starts = re.findall(r'\[INFO\] \[([^\]]+)\]: process started with pid \[(\d+)\]', text)
    exits = re.findall(r'\[INFO\] \[([^\]]+)\]: process has finished cleanly \[pid (\d+)\]', text)
    started, exited = set(starts), set(exits)
    unmatched = sorted(started - exited)
    died = re.findall(r'.*process has died.*', text)
    child_status = 'unverified' if not starts else 'failed' if unmatched or died else 'passed'
    owned_status = 'failed' if complete and not zero else 'passed' if zero else 'unverified'
    worker_ok = worker.get('completed') is True and not worker.get('fault') and runtime.get('error') is None
    statuses = [owned_status, child_status, 'passed' if worker_ok else 'failed']
    outcome = 'failed' if 'failed' in statuses else 'unverified' if 'unverified' in statuses else 'passed'
    result = {
        'schema': 1, 'status': outcome, 'scope': 'Complete owned simulation process shutdown, independent of physical functional success',
        'levels': {'all_owned_process_exit_zero': owned_status, 'all_started_navigation_children_clean_exit': child_status,
                   'worker_completed_without_fault': 'passed' if worker_ok else 'failed',
                   'dynamic_functional_frozen_receipt': functional.get('status', 'unverified'),
                   'raw_slam_two_regions_and_final_stop': functional.get('levels', {}).get('raw_slam_two_regions_and_final_stop', 'unverified'),
                   'complete_dynamic_run': outcome},
        'checks': {
            'all_owned_process_exit_zero': {'status': owned_status, 'required_roles': roles,
                'exactly_one_owned_process_per_role': complete, 'owned_processes': processes,
                'no_exit_code_exception_or_grace': True},
            'all_started_navigation_children_clean_exit': {'status': child_status, 'started': sorted(started),
                'finished_cleanly': sorted(exited), 'missing_clean_exit': unmatched, 'process_died_lines': died,
                'required': 'Every actual launch-started child identity must have a clean completion log; no signal exception'},
        },
        'run_dir': str(run), 'analyzer_sha256': sha(__file__),
        'frozen_functional_summary_sha256': sha(original) if before else None,
        'original_frozen_functional_receipt_bytes_preserved': original.read_bytes() == before if before else None,
        'input_sha256': {name: sha(run / name) for name in ['runtime_manifest.json', 'worker_result.json', 'navigation_stack.log'] if (run / name).exists()},
        'explanation': 'Frozen 0c7608 functional audit inherits common check requiring only worker/Gazebo exit zero; '
            'that original receipt is preserved. This supplement additionally requires every owned role and actual launched child to exit cleanly. '
            'Physical parking/recovery/region evidence does not imply clean shutdown.',
        'global_acceptance_overwritten': False,
    }
    if write:
        (run / 'summary_dynamic_cleanup_independent.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    result = evaluate(args.run)
    print(json.dumps({'status': result['status'], 'levels': result['levels'],
                      'missing_children': result['checks']['all_started_navigation_children_clean_exit']['missing_clean_exit']}))
