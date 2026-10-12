"""Independent source-bound V26 recording review; original V25 remains frozen."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

if not __debug__:
    raise RuntimeError('V26 preflight forbids optimized Python')

REQUIRED = frozenset(('original_V25_preflight_still_valid',
    'profile_normal_form_exactly_V25', 'only_recording_cutoff_budget_and_identity_changed',
    'original46_geometry_arrivals_and_safety_unchanged', 'sole_actuator_and_300ms_protections_unchanged',
    'new_diagnostic_logging_sources_bound', 'cutoff_never_navigation_success',
    'historical_navigation_pass_not_inherited'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evidence_files(here):
    path = Path(here)/'DIAGNOSTIC_V26_PREFLIGHT.json'
    if not path.is_file():
        raise RuntimeError('V26 has no independently reviewed recording-only source receipt')
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


def verify_preflight(here, profile):
    here = Path(here).resolve()
    from full46_launch_contract import validate_full46, baseline_profile_path, PROFILE_NAME
    from geometry_archive import geometry_helper_files
    from mission46_profile import required_source_files
    validate_full46(profile, here)
    verify_original_v25(here)
    path = here/'DIAGNOSTIC_V26_PREFLIGHT.json'
    d = json.loads(path.read_text())
    if (d.get('schema') != 'height_diagnostic_v26_preflight/v1'
            or d.get('status') != 'PASS_LIMITED_DIAGNOSTIC_EXPERIMENT'
            or d.get('candidate_root') != str(here) or d.get('allowed') is not True
            or d.get('actual_navigation_verified') is not False
            or d.get('historical_pass_inherited') is not False
            or d.get('allowed_profiles') != [PROFILE_NAME]
            or set(d.get('checks', {})) != REQUIRED
            or any(value is not True for value in d['checks'].values())):
        raise RuntimeError('Incomplete independent V26 recording-only review')
    required = list(here.glob('*.py')) + list((here/'profiles').glob('*.json'))
    required += [baseline_profile_path(here),
        here.parent/'corridor_tracking_v25_terrain_event/TERRAIN_EVENT_V25_PREFLIGHT.json',
        *required_source_files(), *geometry_helper_files(here)]
    from slam_workspace import evidence_files as slam_evidence_files
    from scan_workspace import evidence_files as scan_evidence_files
    required += [*slam_evidence_files(), *scan_evidence_files(profile)]
    bindings = d.get('verified_inputs_sha256')
    if (not isinstance(bindings, dict) or not bindings
            or any(str(item.resolve()) not in bindings for item in required)):
        raise RuntimeError('V26 candidate/configuration/geometry/binary review bindings incomplete')
    for name, digest in bindings.items():
        if not Path(name).is_absolute() or sha(name) != digest:
            raise RuntimeError('V26 source or review evidence changed: '+name)
    if profile != json.loads((here/'profiles'/PROFILE_NAME).read_text()):
        raise RuntimeError('Only exact source-bound V26 height_fault270 profile may run')
    return d
