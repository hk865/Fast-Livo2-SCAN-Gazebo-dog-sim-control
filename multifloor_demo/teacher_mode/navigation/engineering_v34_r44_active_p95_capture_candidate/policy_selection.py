# Public learning snapshot 2026-10-10: privacy/path metadata or documentation links adjusted; control algorithms and numeric gates unchanged.
"""Explicit private policy selection before source/scope freezing; no fallback."""
import hashlib
import json
from pathlib import Path
import sys
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASELINE_SHA = ''
CANDIDATE_SHA = ''
SCHEMA = 'teacher_explicit_private_policy_selection/v1'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def selection():
    path = HERE / 'POLICY_SELECTION_CONTRACT.json'
    raw = path.read_bytes()
    if len(raw) > 8192:
        raise RuntimeError('Private policy selection contract exceeds8KiB')
    data = json.loads(raw)
    baseline = ROOT / 'policy/worker.py'
    candidate = HERE / 'policy_candidate/worker.py'
    required = dict(schema=SCHEMA, baseline_origin=str(baseline), baseline_sha256=BASELINE_SHA,
                    candidate_origin=str(candidate), candidate_sha256=CANDIDATE_SHA,
                    selected_destination='policy/worker.py', baseline_destination='baseline_policy/worker.py',
                    fallback_allowed=False, original_policy_modified=False)
    if any(data.get(k) != v for k, v in required.items()):
        raise RuntimeError('Explicit private policy selection contract differs')
    if sha(data['authority_file']) != data.get('authority_sha256'):
        raise RuntimeError('Private policy selection authority bytes differ')
    if sha(baseline) != BASELINE_SHA or sha(candidate) != CANDIDATE_SHA:
        raise RuntimeError('Private or protected baseline policy source changed')
    data = dict(data)
    data.update(contract_path=str(path), contract_sha256=hashlib.sha256(raw).hexdigest())
    return data

def source_files():
    data = selection()
    return [Path(data['candidate_origin']), Path(data['contract_path']), Path(data['authority_file'])]

def archive_relative(original, root, data):
    original = Path(original).resolve()
    if original == Path(data['baseline_origin']):
        return Path(data['baseline_destination'])
    if original == Path(data['candidate_origin']):
        return Path(data['selected_destination'])
    return original.relative_to(root) if original.is_relative_to(root) else Path('external') / (sha(original)[:16] + '_' + original.name)

def binding(run):
    data = selection()
    return dict(schema='teacher_selected_policy_snapshot_binding/v1',
                selection_contract_path=data['contract_path'], selection_contract_sha256=data['contract_sha256'],
                baseline_origin=data['baseline_origin'], baseline_snapshot=str(Path(run)/'sources'/data['baseline_destination']),
                baseline_sha256=BASELINE_SHA, candidate_origin=data['candidate_origin'],
                selected_snapshot=str(Path(run)/'sources'/data['selected_destination']), selected_sha256=CANDIDATE_SHA,
                original_observation_and_contract=True, fallback_allowed=False)

def verify_binding(run, refs, snapshots, source_manifest):
    run = Path(run).resolve()
    expected = binding(run)
    path = run / 'selected_policy_binding.json'
    if json.loads(path.read_text()) != expected or refs.get(str(path)) != sha(path):
        raise RuntimeError('Selected policy snapshot binding changed or absent')
    for prefix in ('baseline', 'selected'):
        origin = expected['baseline_origin' if prefix == 'baseline' else 'candidate_origin']
        target = expected[prefix + '_snapshot']
        digest = expected[prefix + '_sha256']
        if refs.get(origin) != digest or refs.get(target) != digest or sha(target) != digest:
            raise RuntimeError('Selected policy source or archived bytes mismatch')
        if snapshots.get(origin) != dict(snapshot=target, sha256=digest):
            raise RuntimeError('Selected policy archive origin or destination differs')
        if source_manifest.get(str(Path(target).relative_to(run/'sources'))) != digest:
            raise RuntimeError('Selected policy absent from source manifest')
    return expected

def record_actual_loaded(run, legacy):
    run = Path(run).resolve()
    expected = binding(run)
    scope = json.loads((run/'navigation_scope.json').read_text())
    snapshots = json.loads((run/'navigation_source_snapshots.json').read_text())
    manifest = json.loads((run/'source_manifest.json').read_text())
    verify_binding(run, scope['references'], snapshots, manifest)
    observation = sys.modules[legacy.build_observation.__module__]
    paths = dict(worker=Path(legacy.__file__).resolve(), observation=Path(observation.__file__).resolve(),
                 observation_contract=Path(observation.__file__).resolve().with_name('contract.json'))
    if paths['worker'] != Path(expected['selected_snapshot']) or paths['observation'] != run/'sources/policy/observation.py':
        raise RuntimeError('Actually imported private policy/observation path differs')
    hashes = {k: sha(v) for k, v in paths.items()}
    if any(scope['references'].get(str(paths[k])) != h for k, h in hashes.items()):
        raise RuntimeError('Actually imported policy source hashes absent from frozen scope')
    if legacy.SHA != scope['checkpoint_sha256']:
        raise RuntimeError('Private policy checkpoint contract changed')
    receipt = dict(schema='teacher_actual_selected_policy_loaded/v1', actual_paths={k:str(v) for k,v in paths.items()},
                   actual_sha256=hashes, selected_binding=expected,
                   original_response_deadline_ms=200, source_and_SCAN_freshness_ms=300,
                   observation_module_cache_path_verified=True, before_Actor_load=True)
    with (run/'selected_policy_loaded.json').open('x') as stream:
        stream.write(json.dumps(receipt,sort_keys=True,indent=2)+'\n')
    return receipt
