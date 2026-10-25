"""Explicit private policy selection before source/scope freezing; no fallback."""
import package_paths as _package_paths
import hashlib
import json
from pathlib import Path
import sys
HERE = Path(__file__).resolve().parent
ROOT = _package_paths.TEACHER_ROOT  # Protected policy baseline; actual selected policy stays HERE.
BASELINE_SHA = '8d38a93c1049f4e7aff4788885b70ba3f5a6832addb7626ebfe992a0d128069f'
CANDIDATE_SHA = '70b9a7564c58be23ca070c6e8da8fbfedfa3cadd5eb7696aa1441f46206a83a5'
SCHEMA = 'teacher_explicit_private_policy_selection/v1'
STUDENT_EXPORT_SHA = 'd8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185'
STUDENT_CHECKPOINT_SHA = '7b59b8c7eafc9ffbc2dc32aaa2df3dadec3269d039338e4f93217bc73af66fd9'
TEACHER_CHECKPOINT_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'

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

def student_sources():
    adapter=HERE/'student_actor_adapter.py';contract=HERE/'newstudent_pid_contract.json'
    data=json.loads(contract.read_text())
    if (data.get('schema')!='student_runtime_exact45_fixed_pid/v2'
        or data.get('export_sha256')!=STUDENT_EXPORT_SHA
        or data.get('checkpoint_sha256')!=STUDENT_CHECKPOINT_SHA):
        raise RuntimeError('Explicit Student source/weight identity differs')
    model=Path(data['export_origin']).resolve()
    if sha(model)!=STUDENT_EXPORT_SHA:raise RuntimeError('Original Student JIT bytes changed')
    return dict(student_adapter=adapter,student_contract=contract,student_export=model)

def source_files():
    data = selection()
    return [Path(data['candidate_origin']), Path(data['contract_path']), Path(data['authority_file'])]+list(student_sources().values())

def archive_relative(original, root, data):
    original = Path(original).resolve()
    if original == Path(data['baseline_origin']):
        return Path(data['baseline_destination'])
    if original == Path(data['candidate_origin']):
        return Path(data['selected_destination'])
    # Keep actual clone modules adjacent in the immutable run layout.
    # Source reference identities remain their actual clone paths.
    if original.is_relative_to(HERE):
        return Path('navigation/engineering_v66_r85_turn_handoff_candidate') / original.relative_to(HERE)
    return original.relative_to(root) if original.is_relative_to(root) else Path('external') / (sha(original)[:16] + '_' + original.name)

def binding(run):
    data = selection()
    expected=dict(schema='explicit_actor_legacy_loop_snapshot_binding/v2',
                selection_contract_path=data['contract_path'], selection_contract_sha256=data['contract_sha256'],
                baseline_origin=data['baseline_origin'], baseline_snapshot=str(Path(run)/'sources'/data['baseline_destination']),
                baseline_sha256=BASELINE_SHA, candidate_origin=data['candidate_origin'],
                selected_snapshot=str(Path(run)/'sources'/data['selected_destination']), selected_sha256=CANDIDATE_SHA,
                original_observation_and_contract=True, fallback_allowed=False,
                historical_teacher_authority_role='legacy source evidence only; not Student runtime/performance authority')
    for role,origin in student_sources().items():
        origin=origin.resolve()
        target=Path(run)/'sources'/archive_relative(origin,ROOT,data)
        expected.update({role+'_origin':str(origin),role+'_snapshot':str(target),role+'_sha256':sha(origin)})
    return expected

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
    for role in ('student_adapter','student_contract','student_export'):
        origin,target,digest=(expected[role+'_'+key] for key in ('origin','snapshot','sha256'))
        if refs.get(origin)!=digest or refs.get(target)!=digest or sha(target)!=digest:
            raise RuntimeError('Student source/archived bytes mismatch: '+role)
        if snapshots.get(origin)!=dict(snapshot=target,sha256=digest):
            raise RuntimeError('Student source archive origin/destination differs: '+role)
        if source_manifest.get(str(Path(target).relative_to(run/'sources')))!=digest:
            raise RuntimeError('Student input absent from source manifest: '+role)
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
    actor_kind=scope.get('actor_kind','teacher')
    if legacy.SHA!=TEACHER_CHECKPOINT_SHA:
        raise RuntimeError('Original Teacher legacy-loop checkpoint identity changed')
    if actor_kind=='teacher' and scope['checkpoint_sha256']!=TEACHER_CHECKPOINT_SHA:
        raise RuntimeError('Actual Teacher Actor checkpoint scope differs')
    if actor_kind=='student' and (scope.get('checkpoint_sha256')!=STUDENT_CHECKPOINT_SHA
        or scope.get('export_sha256')!=STUDENT_EXPORT_SHA):
        raise RuntimeError('Actual Student Actor identity scope differs')
    if actor_kind not in ('teacher','student'):raise RuntimeError('Unknown selected Actor kind')
    receipt = dict(schema='explicit_actor_actual_legacy_sources_loaded/v2', actor_kind=actor_kind,
                   actual_paths={k:str(v) for k,v in paths.items()},
                   actual_actor_load_pending=True,legacy_teacher_loop_imported=True,
                   legacy_teacher_checkpoint_sha256=TEACHER_CHECKPOINT_SHA,
                   original_teacher_authority_not_student_runtime_authority=True,
                   actual_sha256=hashes, selected_binding=expected,
                   original_response_deadline_ms=200, source_and_SCAN_freshness_ms=300,
                   observation_module_cache_path_verified=True, before_Actor_load=True)
    with (run/'selected_policy_loaded.json').open('x') as stream:
        stream.write(json.dumps(receipt,sort_keys=True,indent=2)+'\n')
    return receipt
