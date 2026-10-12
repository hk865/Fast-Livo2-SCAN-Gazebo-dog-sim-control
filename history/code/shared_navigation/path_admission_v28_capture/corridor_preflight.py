"""Independent source-bound V28 repair gate; frozen V25/V26 remain evidence."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

if not __debug__:
    raise RuntimeError('V28 preflight forbids optimized Python')

GATE_NAME = 'PATH_ADMISSION_V28_CAPTURE_PREFLIGHT.json'
REVIEW_NAME = 'PATH_ADMISSION_V28_CAPTURE_SOURCE_REVIEW.json'
REQUIRED = frozenset(('original_V25_preflight_still_valid',
    'original_V26_receipts_bytes_unchanged', 'profile_normal_form_exactly_V25',
    'only_path_admission_algorithm_changed', 'whole_curve_route_fence_before_commit',
    'active_trajectory_reference_identity_preserved',
    'original46_geometry_arrivals_and_safety_unchanged',
    'sole_actuator_and_300ms_protections_unchanged',
    'recording_budget_change_not_control_relaxation',
    'all_candidate_sources_configs_binaries_and_review_evidence_bound',
    'cutoff_never_navigation_success', 'historical_navigation_pass_not_inherited'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evidence_files(here):
    path = Path(here)/GATE_NAME
    if not path.is_file():
        raise RuntimeError('V28 has no independent source-bound repair receipt')
    d = json.loads(path.read_text())
    return [path, *[Path(x) for x in d.get('verified_inputs_sha256', {})]]


def verify_original_v25(here):
    """Separate import namespace preserves original frozen V25 module binding."""
    original = Path(here).resolve().parent/'corridor_tracking_v25_terrain_event'
    code = ('import json,sys; from pathlib import Path; '
            'here=Path(sys.argv[1]).resolve(); sys.path.insert(0,str(here)); '
            'from corridor_preflight import verify_preflight; '
            'p=json.loads((here/"profiles/curvature_original46_on.json").read_text()); '
            'd=verify_preflight(here,p); '
            'print(json.dumps({"schema":d["schema"],"candidate_root":d["candidate_root"],'
            '"status":d["status"],"actual_navigation_verified":d["actual_navigation_verified"]}))')
    process = subprocess.run([sys.executable, '-B', '-c', code, str(original)],
                             capture_output=True, text=True, timeout=60, check=False)
    if process.returncode:
        raise RuntimeError('Frozen original V25 preflight is no longer valid: '+process.stderr[-2000:])
    row = json.loads(process.stdout)
    if (row.get('candidate_root') != str(original)
            or row.get('schema') != 'terrain_event_v25_full46_preflight/v1'
            or row.get('status') != 'PASS_LIMITED_NEW_EXPERIMENT'
            or row.get('actual_navigation_verified') is not False):
        raise RuntimeError('Wrong original V25 source review witness')
    return row


def verify_original_v26(here):
    """The copied old receipts are immutable historical references, never gates."""
    original = Path(here).resolve().parent/'height_diagnostic_v26'
    for name in ('DIAGNOSTIC_V26_PREFLIGHT.json', 'DIAGNOSTIC_V26_SOURCE_REVIEW.json'):
        if sha(Path(here)/'historical_receipts'/name) != sha(original/name):
            raise RuntimeError('Frozen original V26 receipt copy changed: '+name)


def required_review_files(here, profile=None):
    """Complete independently frozen candidate set; excludes the gate itself."""
    here = Path(here).resolve()
    from full46_launch_contract import baseline_profile_path
    from geometry_archive import geometry_helper_files
    from mission46_profile import required_source_files
    from slam_workspace import evidence_files as slam_evidence_files
    from scan_workspace import evidence_files as scan_evidence_files
    required = list(here.glob('*.py')) + list((here/'profiles').glob('*.json'))
    required += [p for p in here.glob('*.json') if p.name != GATE_NAME]
    required += list((here/'historical_receipts').glob('*.json'))
    required += [here/'README.md', baseline_profile_path(here),
        here.parent/'corridor_tracking_v25_terrain_event/TERRAIN_EVENT_V25_PREFLIGHT.json',
        *required_source_files(), *geometry_helper_files(here)]
    required += [here.parent/'height_diagnostic_v26'/name for name in
                 ('DIAGNOSTIC_V26_PREFLIGHT.json', 'DIAGNOSTIC_V26_SOURCE_REVIEW.json')]
    required += [*slam_evidence_files(), *scan_evidence_files(profile)]
    # Bind the unchanged native actuator and privileged policy as well.
    root = here.parents[1]
    required += [root/'simulation/teacher_actuator.cpp',
                 root/'simulation/build/libteacher_actuator.so',
                 root/'policy/worker.py', root/'policy/observation.py', root/'policy/contract.json']
    from pid_scope import runtime_files
    from run import CHECKPOINT
    required += [p for p in runtime_files(profile) if p.resolve() != here/GATE_NAME]
    required += [CHECKPOINT]
    return sorted(set(p.resolve() for p in required))


def verify_preflight(here, profile):
    here = Path(here).resolve()
    from full46_launch_contract import validate_full46, PROFILE_NAMES, profile_name
    validate_full46(profile, here)
    verify_original_v25(here)
    verify_original_v26(here)
    d = json.loads((here/GATE_NAME).read_text())
    if (d.get('schema') != 'path_admission_v28_capture_preflight/v1'
            or d.get('status') != 'PASS_LIMITED_CAPTURE_HANDOFF_EXPERIMENT'
            or d.get('candidate_root') != str(here) or d.get('allowed') is not True
            or d.get('actual_navigation_verified') is not False
            or d.get('historical_pass_inherited') is not False
            or d.get('allowed_profiles') != list(PROFILE_NAMES)
            or d.get('root_review_complete') is not True
            or d.get('code_review_complete') is not True
            or set(d.get('checks', {})) != REQUIRED
            or any(value is not True for value in d['checks'].values())):
        raise RuntimeError('Incomplete independent V28 path-admission source review')
    review = json.loads((here/REVIEW_NAME).read_text())
    if (review.get('schema') != 'path_admission_v28_capture_source_review/v1'
            or review.get('candidate_root') != str(here)
            or review.get('status') != 'PASS_LIMITED_CAPTURE_HANDOFF_SOURCE_REVIEW'
            or review.get('actual_navigation_verified') is not False
            or review.get('historical_pass_inherited') is not False
            or review.get('root_review_complete') is not True
            or review.get('code_review_complete') is not True
            or set(review.get('checks', {})) != REQUIRED
            or any(value is not True for value in review['checks'].values())):
        raise RuntimeError('V28 source review is pending or incomplete')
    bindings = d.get('verified_inputs_sha256')
    required = required_review_files(here, profile)
    evidence = review.get('review_evidence_sha256')
    if (not isinstance(bindings, dict) or not bindings
            or any(str(item) not in bindings for item in required)
            or not isinstance(evidence, dict) or not evidence
            or any(bindings.get(name) != digest for name, digest in evidence.items())):
        raise RuntimeError('V28 candidate/configuration/binary/review evidence bindings incomplete')
    for name, digest in bindings.items():
        if not Path(name).is_absolute() or sha(name) != digest:
            raise RuntimeError('V28 source or review evidence changed: '+name)
    if profile != json.loads((here/'profiles'/profile_name(profile)).read_text()):
        raise RuntimeError('Only exact source-bound V28 repair profiles may run')
    return d
