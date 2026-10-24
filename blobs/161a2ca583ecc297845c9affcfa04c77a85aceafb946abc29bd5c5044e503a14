"""Independent source-bound V32 map support experiment; V28 navigation frozen."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

if not __debug__:
    raise RuntimeError('V32 preflight forbids optimized Python')

GATE_NAME='MAP_SURFACE_V32_PREFLIGHT.json'
REVIEW_NAME='MAP_SURFACE_V32_SOURCE_REVIEW.json'
REQUIRED=frozenset(('frozen_V28_navigation_preflight_still_valid',
    'all_unmodified_navigation_sources_exactly_V28','profile_normal_form_exactly_V28',
    'independent_V32_source_build_install_bound','surface_validity_runtime_flag_frozen_before_launch',
    'surface_quality_constants_and_original_lambda_min_retained',
    'bounded_support_diagnostic_birth_and_fault_windows',
    'bounded_logging_changes_not_control_relaxation','actual_loaded_binary_and_mode_witness_required',
    'original46_geometry_arrivals_and_safety_unchanged',
    'sole_actuator_and_300ms_protections_unchanged',
    'all_helpers_runner_sources_configs_review_bound',
    'finite_mechanism_tests_not_navigation_success','historical_navigation_pass_not_inherited'))
# Deployment adapters are explicitly changed; no control algorithm file is exempt.
CHANGED_ADAPTERS=frozenset(('corridor_preflight.py','full46_launch_contract.py','slam_workspace.py',
    'run.py','worker.py','pid_scope.py','slam.launch.py','stack.launch.py'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evidence_files(here):
    path=Path(here)/GATE_NAME
    if not path.is_file():
        raise RuntimeError('V32 independent prospective receipt is absent')
    d=json.loads(path.read_text())
    return [path,*[Path(x) for x in d.get('verified_inputs_sha256',{})]]


def verify_original_v28(here):
    original=Path(here).resolve().parent/'path_admission_v28_capture'
    code=('import json,sys; from pathlib import Path; '
          'here=Path(sys.argv[1]).resolve(); sys.path.insert(0,str(here)); '
          'from corridor_preflight import verify_preflight; '
          'p=json.loads((here/"profiles/repair_original46.json").read_text()); '
          'd=verify_preflight(here,p); '
          'print(json.dumps({k:d[k] for k in ("schema","candidate_root","status","actual_navigation_verified")}))')
    process=subprocess.run([sys.executable,'-B','-c',code,str(original)],
        capture_output=True,text=True,timeout=60,check=False)
    if process.returncode:
        raise RuntimeError('Frozen V28 navigation preflight changed: '+process.stderr[-2000:])
    row=json.loads(process.stdout)
    if (row.get('candidate_root')!=str(original)
        or row.get('schema')!='path_admission_v28_capture_preflight/v1'
        or row.get('status')!='PASS_LIMITED_CAPTURE_HANDOFF_EXPERIMENT'
        or row.get('actual_navigation_verified') is not False):
        raise RuntimeError('Wrong frozen V28 navigation receipt')
    return row


def unchanged_navigation_bindings(here):
    here=Path(here).resolve();original=here.parent/'path_admission_v28_capture'
    required=sorted(p.name for p in original.glob('*.py') if p.name not in CHANGED_ADAPTERS)
    if not required or not {'controller.py','shared_controller.py','cascade_core.py','bridge.py',
        'sensor_gate.py','spatial_reference.py','path_admission.py'}.issubset(required):
        raise RuntimeError('Incomplete frozen navigation control source set')
    rows={}
    for name in required:
        a=original/name;b=here/name
        digest=sha(a)
        if sha(b)!=digest:
            raise RuntimeError('V32 changed an unauthorized navigation source: '+name)
        rows[name]=dict(baseline=str(a.resolve()),candidate=str(b.resolve()),sha256=digest)
    return rows


def required_review_files(here,profile=None):
    here=Path(here).resolve()
    from full46_launch_contract import PROFILE_NAMES,baseline_profile_path
    from geometry_archive import geometry_helper_files
    from mission46_profile import required_source_files
    from slam_workspace import evidence_files as slam_evidence_files
    from scan_workspace import evidence_files as scan_evidence_files
    original=here.parent/'path_admission_v28_capture'
    required=list(here.glob('*.py'))+list((here/'profiles').glob('*.json'))
    required += [p for p in here.glob('*.json') if p.name!=GATE_NAME]
    required += list((here/'historical_receipts').glob('*.json'))
    required += [here/'README.md',*[baseline_profile_path(here,name) for name in PROFILE_NAMES],
        original/'PATH_ADMISSION_V28_CAPTURE_PREFLIGHT.json',
        original/'PATH_ADMISSION_V28_CAPTURE_SOURCE_REVIEW.json',
        *required_source_files(),*geometry_helper_files(here)]
    required += [Path(row['baseline']) for row in unchanged_navigation_bindings(here).values()]
    helper=here.parent/'evidence_runtime_v32'
    required += list(helper.glob('*.py'))+list(helper.glob('*.json'))
    if not {'bounded_binary.py','worker_logging.py','raw_capture.py'}.issubset({p.name for p in helper.glob('*.py')}):
        raise RuntimeError('V32 bounded evidence helpers are incomplete')
    required += [*slam_evidence_files(),*scan_evidence_files(profile)]
    root=here.parents[1]
    required += [root/'simulation/teacher_actuator.cpp',root/'simulation/build/libteacher_actuator.so',
        root/'policy/worker.py',root/'policy/observation.py',root/'policy/contract.json']
    from pid_scope import runtime_files
    from run import CHECKPOINT
    required += [p for p in runtime_files(profile) if p.resolve()!=here/GATE_NAME]
    required += [CHECKPOINT]
    return sorted(set(p.resolve() for p in required))


def verify_preflight(here,profile):
    here=Path(here).resolve()
    from full46_launch_contract import validate_full46,PROFILE_NAMES,profile_name
    validate_full46(profile,here)
    verify_original_v28(here)
    nav=unchanged_navigation_bindings(here)
    d=json.loads((here/GATE_NAME).read_text())
    if (d.get('schema')!='map_surface_v32_preflight/v1'
        or d.get('status')!='PASS_LIMITED_MAP_SURFACE_EXPERIMENT'
        or d.get('candidate_root')!=str(here) or d.get('allowed') is not True
        or d.get('actual_navigation_verified') is not False
        or d.get('historical_pass_inherited') is not False
        or d.get('allowed_profiles')!=list(PROFILE_NAMES)
        or d.get('root_review_complete') is not True or d.get('code_review_complete') is not True
        or d.get('actual_loaded_binary_and_mode_witness_required') is not True
        or set(d.get('checks',{}))!=REQUIRED
        or any(value is not True for value in d['checks'].values())):
        raise RuntimeError('V32 independent source review is pending or incomplete')
    review=json.loads((here/REVIEW_NAME).read_text())
    if (review.get('schema')!='map_surface_v32_source_review/v1'
        or review.get('candidate_root')!=str(here)
        or review.get('status')!='PASS_LIMITED_MAP_SURFACE_SOURCE_REVIEW'
        or review.get('actual_navigation_verified') is not False
        or review.get('historical_pass_inherited') is not False
        or review.get('root_review_complete') is not True or review.get('code_review_complete') is not True
        or review.get('unchanged_navigation_bindings')!=nav
        or set(review.get('checks',{}))!=REQUIRED
        or any(value is not True for value in review['checks'].values())):
        raise RuntimeError('V32 source review pending or navigation baseline changed')
    bindings=d.get('verified_inputs_sha256');evidence=review.get('review_evidence_sha256')
    required=required_review_files(here,profile)
    if (not isinstance(bindings,dict) or not bindings or any(str(p) not in bindings for p in required)
        or not isinstance(evidence,dict) or not evidence
        or any(bindings.get(name)!=digest for name,digest in evidence.items())):
        raise RuntimeError('V32 source/build/helper/runner/config/review evidence bindings incomplete')
    for name,digest in bindings.items():
        if not Path(name).is_absolute() or sha(name)!=digest:
            raise RuntimeError('V32 source or finite review evidence changed: '+name)
    if profile!=json.loads((here/'profiles'/profile_name(profile)).read_text()):
        raise RuntimeError('Only exact source-bound V32 profiles may run')
    return d


def freeze_source_review(here,review_evidence,*,root_review_complete=False,code_review_complete=False):
    """Called only after actual finite tests and root code review; never navigation PASS."""
    here=Path(here).resolve()
    if root_review_complete is not True or code_review_complete is not True:
        raise RuntimeError('V32 freeze requires completed root/code review of concrete evidence')
    from full46_launch_contract import PROFILE_NAMES,validate_full46
    verify_original_v28(here)
    nav=unchanged_navigation_bindings(here)
    from slam_workspace import verify_slam_workspace
    verify_slam_workspace()
    for name in PROFILE_NAMES:
        validate_full46(json.loads((here/'profiles'/name).read_text()),here)
    if not review_evidence:
        raise RuntimeError('V32 freeze needs actual mechanism/helper/runner review evidence')
    evidence={str(Path(p).resolve()):sha(p) for p in review_evidence}
    review=dict(schema='map_surface_v32_source_review/v1',candidate_root=str(here),
        status='PASS_LIMITED_MAP_SURFACE_SOURCE_REVIEW',actual_navigation_verified=False,
        historical_pass_inherited=False,root_review_complete=True,code_review_complete=True,
        checks={key:True for key in sorted(REQUIRED)},unchanged_navigation_bindings=nav,
        review_evidence_sha256=evidence)
    (here/REVIEW_NAME).write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n')
    required=set(required_review_files(here))|{Path(p) for p in evidence}
    gate=dict(schema='map_surface_v32_preflight/v1',candidate_root=str(here),
        status='PASS_LIMITED_MAP_SURFACE_EXPERIMENT',allowed=True,actual_navigation_verified=False,
        historical_pass_inherited=False,root_review_complete=True,code_review_complete=True,
        allowed_profiles=list(PROFILE_NAMES),actual_loaded_binary_and_mode_witness_required=True,
        checks={key:True for key in sorted(REQUIRED)},
        verified_inputs_sha256={str(p.resolve()):sha(p) for p in sorted(required)})
    (here/GATE_NAME).write_text(json.dumps(gate,ensure_ascii=False,indent=2)+'\n')
    for name in PROFILE_NAMES:
        verify_preflight(here,json.loads((here/'profiles'/name).read_text()))
    return gate
