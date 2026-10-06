#!/usr/bin/env python3
"""Version/source adapter plus post-abort reading; original acceptance stays intact."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
BASE = HERE / 'audit_curvature_prefix9.py'
BASE_SHA = 'ddb1b5b19f6abd5c051d8348a23b45d49cf7f7b0df35d9ab94d2ce85fe67f93a'
CANDIDATES = {
    'v21_curvature_serialization': HERE.parents[1] / 'navigation/corridor_tracking_v21_curvature',
    'v22_curvature_archive': HERE.parents[1] / 'navigation/corridor_tracking_v22_curvature_archive',
}


def process_readable_stop(runtime, audit):
    _, evidence = audit.owned_processes_stopped(runtime)
    owned = runtime.get('owned_processes') or []
    if not evidence or len(evidence) != len(owned):
        raise RuntimeError('All saved owner PID/start identities required')
    if any(x['same_process_live'] for x in evidence):
        raise RuntimeError('Live saved owner; post-abort reading refused')
    children = (runtime.get('launch_child_exit_evidence') or {}).get('children') or []
    if not children:
        raise RuntimeError('Archived launch-child exit evidence required')
    for child in children:
        pid = child.get('pid')
        if type(pid) is not int or (Path('/proc') / str(pid)).exists():
            raise RuntimeError('Launch child lacks absent-PID proof; reading refused')
    # Nonzero child exits remain failures in the original execution check.
    return True, evidence


def run_once(run, condition, output):
    spec = importlib.util.spec_from_file_location('curvature_v22_base_observer', BASE)
    base = importlib.util.module_from_spec(spec); spec.loader.exec_module(base)
    if base.sha(BASE) != BASE_SHA:
        raise RuntimeError('Reviewed base observer changed')
    run = Path(run).resolve()
    runtime = json.loads((run / 'runtime_manifest.json').read_text())
    profile = json.loads((run / 'navigation_profile.json').read_text())
    selector = profile.get('controller_selector')
    if selector not in ('corridor_tracking_v20', *CANDIDATES):
        raise RuntimeError('Unknown version/source adapter')
    original_load = base.load
    def load():
        legacy, audit = original_load()
        original_stop = audit.owned_processes_stopped
        if selector in CANDIDATES:
            candidate = CANDIDATES[selector]
            # Same exact geometry helpers; distinct live source root must match
            # this run's archived hashes in the unchanged verify_archive gate.
            for name in ('route.py', 'prefix_contract.py', 'mission46_profile.py'):
                if base.sha(candidate / name) != base.sha(audit.CORE / name):
                    raise RuntimeError('Geometry helper change requires a new evaluator')
            audit.CORE = candidate
        if runtime.get('all_owned_and_children_clean') is not True:
            def stopped(r):
                # Use the original identity reader without recursive patching.
                saved = audit.owned_processes_stopped
                audit.owned_processes_stopped = original_stop
                try: return process_readable_stop(r, audit)
                finally: audit.owned_processes_stopped = saved
            audit.owned_processes_stopped = stopped
        return legacy, audit
    base.load = load
    result = base.run_once(run, condition, output)
    # The original execution/worker/error gates prohibit PASS after an abort.
    if runtime.get('all_owned_and_children_clean') is not True and result['prefix9_limited_pass']:
        raise RuntimeError('Aborted runtime was incorrectly promoted')
    receipt = dict(schema='curvature_v22_source_and_abort_read_adapter/v1',
        run=str(run), selector=selector, adapter_sha256=base.sha(__file__),
        base_observer_sha256=BASE_SHA, original_acceptance_logic_changed=False,
        source_root=str(CANDIDATES[selector]) if selector in CANDIDATES else 'original V20',
        abnormal_cleanup_preserved=runtime.get('all_owned_and_children_clean') is not True,
        result=result, full46_pass=False)
    base.save(Path(output) / (run.name + '_VERSION_ADAPTER_RECEIPT.json'), receipt)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--condition', choices=['OFF', 'ON'], required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    result = run_once(args.run, args.condition, args.output_dir)
    print(json.dumps(result, indent=2))
    sys.exit(1 if result.get('error') else 0)
