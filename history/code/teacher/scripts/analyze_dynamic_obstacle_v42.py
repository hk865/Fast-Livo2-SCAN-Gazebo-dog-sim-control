#!/usr/bin/env python3
"""Prospectively frozen V4.2 dynamic audit: same physical criteria plus strict cleanup.

Uses preserved 0c7608 functional logic, authorizes only the exact V4.2 runtime
freeze, and requires every owned process and launched child to exit cleanly.
Never overwrites any pre-existing functional receipt.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
FUNCTIONAL_SHA = '0c7608db35dbbfad4942179465e6a44376e52e949f276554eaa0a85d93723bf3'
RUNTIME_FREEZE_SHA = 'c3acec04da84ec1ec35d317db8235cef8e0de3692e0c84c10a9e154a4b75c819'
CANDIDATE = ROOT / 'test_results/dynamic_independent_candidate2_v42_20261004'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def evaluate(run, write=True):
    run = Path(run).resolve()
    destination = run / 'summary_dynamic_obstacle_independent.json'
    if write and destination.exists():
        raise ValueError('Refusing to overwrite a prior frozen dynamic receipt')
    functional_path = ROOT / 'scripts/analyze_dynamic_obstacle.py'
    if sha(functional_path) != FUNCTIONAL_SHA:
        raise ValueError('Preserved functional evaluator hash changed')
    freeze_path = CANDIDATE / 'runtime_v42_frozen.json'
    if sha(freeze_path) != RUNTIME_FREEZE_SHA:
        raise ValueError('Exact prospective runtime freeze changed')
    manifest = json.loads((CANDIDATE / 'freeze_manifest.json').read_text())
    cleanup_path = ROOT / 'scripts/audit_dynamic_runtime.py'
    if sha(cleanup_path) != manifest['cleanup_analyzer_sha256']:
        raise ValueError('Strict process-exit evaluator hash changed')
    functional = load(functional_path, 'dynamic_v42_preserved_functional')
    summary = functional.evaluate(run, write=False)
    if summary['status'] == 'inapplicable':
        return summary
    runtime = json.loads(freeze_path.read_text())
    mismatches = []
    resolved = {}
    for name, expected in runtime['source_hashes'].items():
        actual_path = functional.reference_path(run, str(ROOT / name))
        actual = sha(actual_path) if actual_path.is_file() else None
        resolved[name] = {'actual_path': str(actual_path), 'expected_sha256': expected, 'actual_sha256': actual}
        if actual != expected:
            mismatches.append(name)
    exact_status = 'failed' if mismatches else 'passed'
    cleanup = load(cleanup_path, 'dynamic_v42_strict_cleanup').evaluate(run, write=False)
    summary['checks']['exact_prospective_v42_runtime_sources'] = {
        'status': exact_status, 'passed': not mismatches, 'expected_runtime_freeze_sha256': RUNTIME_FREEZE_SHA,
        'runtime_source_checks': resolved, 'mismatches': mismatches,
        'authorization': 'Only the exact frozen direct-Node service-bridge cleanup version; physical criteria unchanged',
    }
    summary['checks']['all_owned_and_started_child_clean_exit'] = {
        'status': cleanup['status'], 'passed': cleanup['status'] == 'passed',
        'strict_process_checks': cleanup['checks'], 'no_signal_code_exception_or_grace': True,
        'worker_completed_without_fault': cleanup['levels']['worker_completed_without_fault'],
    }
    old_status = summary['status']
    summary['status'] = functional.verdict([old_status, exact_status, cleanup['status']])
    summary['levels'].update({'dynamic_functional': old_status, 'exact_v42_runtime_sources': exact_status,
        'all_owned_process_exit_zero': cleanup['levels']['all_owned_process_exit_zero'],
        'all_started_navigation_children_clean_exit': cleanup['levels']['all_started_navigation_children_clean_exit'],
        'complete_dynamic_run': summary['status']})
    summary['functional_analyzer_sha256'] = FUNCTIONAL_SHA
    summary['analyzer_sha256'] = sha(__file__)
    summary['cleanup_analyzer_sha256'] = sha(cleanup_path)
    summary['runtime_freeze_sha256'] = RUNTIME_FREEZE_SHA
    summary['criteria_change'] = 'Added exact prospective V4.2 runtime source authorization and required all-owned/child clean exits; no physical, clear, stopping, region, timeout or recovery criterion relaxed.'
    summary['input_sha256'].update({n: sha(run / n) for n in ['runtime_manifest.json', 'worker_result.json', 'navigation_stack.log']})
    if write:
        destination.write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--read-only', action='store_true')
    args = parser.parse_args()
    result = evaluate(args.run, write=not args.read_only)
    print(json.dumps({'status': result['status'], 'levels': result.get('levels'), 'errors': result.get('errors')}))
