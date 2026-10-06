"""A fresh V24 control/glue review is mandatory; V19 never authorizes V24."""
from pathlib import Path
from geometry_archive import geometry_helper_files
import hashlib
import json

if not __debug__:
    raise RuntimeError('V24 preflight forbids optimized Python')

REQUIRED = frozenset(('inherited_V19_SLAM_core_and_binary',
    'new_spatial_reference_finite_control', 'production_protection_and_publication_paths',
    'new_glue_profiles_and_fail_closed_gate', 'prospective_original_routes_and_limits',
    'no_historical_navigation_pass_inherited', 'native_scalar_JSON_and_stop_predicate_finite', 'five_geometry_helpers_direct_archive', 'full46_original_mission_and_safety', 'full46_launch_only_not_prefix', 'protected_zero_not_path_exhaustion'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evidence_files(here):
    path = Path(here)/'RECOVERY_REPLAN_V24_PREFLIGHT.json'
    if not path.is_file():
        raise RuntimeError('V24 control and glue have no fresh independently reviewed preflight')
    d = json.loads(path.read_text())
    return [path, *[Path(x) for x in d.get('verified_inputs_sha256', {})]]


def verify_preflight(here, profile):
    here = Path(here).resolve()
    path = here/'RECOVERY_REPLAN_V24_PREFLIGHT.json'
    if not path.is_file():
        raise RuntimeError('V24 requires a new finite control/protection/glue review; historical V19 PASS is not inherited')
    d = json.loads(path.read_text())
    if (d.get('schema') != 'recovery_replan_v24_full46_preflight/v1'
            or d.get('status') != 'PASS_LIMITED_NEW_EXPERIMENT' or d.get('candidate_root') != str(here)
            or d.get('allowed') is not True or d.get('actual_navigation_verified') is not False
            or d.get('historical_pass_inherited') is not False
            or set(d.get('checks', {})) != REQUIRED or any(v is not True for v in d['checks'].values())):
        raise RuntimeError('V24 finite control/protection review is incomplete or has inherited acceptance')
    allowed = d.get('allowed_profiles')
    if allowed != ['curvature_original46_on.json']:
        raise RuntimeError('V24 only authorizes the full original46 curvature ON experiment')
    expected = json.loads((here/'profiles'/allowed[0]).read_text())
    if (profile != expected or profile.get('controller_selector') != 'v24_recovery_replan_full46'
            or profile.get('scan_workspace_selector') != 'protected_baseline'
            or 'original46_prefix_regions' in profile or profile.get('mission46_required') is not True
            or profile.get('cascade', {}).get('spatial_reference', {}).get('curvature_feedforward_enabled') is not True
            or profile.get('cascade', {}).get('spatial_reference', {}).get('curvature_speed_limit_enabled') is not True):
        raise RuntimeError('Only exact full original46 V24 ON profile may run')
    from full46_launch_contract import validate_full46
    validate_full46(profile,here)
    p = profile.get('pipeline', {})
    if (p.get('schema') != 'pipeline_v19_contract/v1' or p.get('mode') not in ('serial', 'rx_decode', 'staged')
            or type(p.get('image_copy_opt')) is not int or p['image_copy_opt'] not in (0, 1)
            or p.get('max_items') != 512 or p.get('reserved_bytes') != 67108864
            or profile.get('pose_cloud_timeout_s') != .3 or profile.get('navigation_ground_truth_used') is not False
            or profile.get('lio_jacobian_parallelism', {}).get('threads') != 4
            or profile.get('vio_patch_parallel', {}).get('threads') != 1):
        raise RuntimeError('V24 original bounded pipeline, source freshness, LIO4/VIO1 or actual navigation contract changed')
    from slam_workspace import evidence_files as slam_evidence_files
    from scan_workspace import evidence_files as scan_evidence_files
    required = list(here.glob('*.py')) + list((here/'profiles').glob('*.json'))
    from full46_launch_contract import baseline_profile_path
    from mission46_profile import required_source_files
    required += [here/'RECOVERY_REPLAN_V24_CONTRACT.json', baseline_profile_path(here), *required_source_files(),
                 *geometry_helper_files(here), *slam_evidence_files(), *scan_evidence_files(profile)]
    review = d.get('control_review')
    full46_review = d.get('full46_review')
    receipts = d.get('finite_receipts')
    if (not isinstance(review, dict) or not isinstance(receipts, list) or not receipts
            or not isinstance(review.get('path'), str) or not review.get('sha256')
            or not isinstance(full46_review, dict) or not isinstance(full46_review.get('path'), str)
            or not full46_review.get('sha256')):
        raise RuntimeError('V24 must bind its new explicit controller review and actual finite receipts')
    required += [Path(review['path']), Path(full46_review['path'])]
    for receipt in receipts:
        if not isinstance(receipt, dict) or not isinstance(receipt.get('path'), str) or not receipt.get('sha256'):
            raise RuntimeError('Malformed V24 finite evidence binding')
        required.append(Path(receipt['path']))
    bindings = d.get('verified_inputs_sha256')
    if not isinstance(bindings, dict) or not bindings or any(str(x.resolve()) not in bindings for x in required):
        raise RuntimeError('V24 mandatory candidate/source/control/build/evidence bindings are incomplete')
    for name, digest in bindings.items():
        if not Path(name).is_absolute() or sha(name) != digest:
            raise RuntimeError('V24 reviewed source or receipt changed: ' + name)
    for receipt in [review, full46_review, *receipts]:
        if bindings.get(str(Path(receipt['path']).resolve())) != receipt['sha256']:
            raise RuntimeError('V24 declared finite receipt SHA is not the actually verified binding')
    fresh_replan_receipts = [json.loads(Path(row['path']).read_text()) for row in receipts
        if json.loads(Path(row['path']).read_text()).get('schema') == 'recovery_replan_v24_finite/v1']
    if len(fresh_replan_receipts) != 1:
        raise RuntimeError('V24 needs its own fresh finite replan receipt, not an inherited PASS')
    finite = fresh_replan_receipts[0]
    finite_pins = finite.get('source_bindings')
    if (finite.get('status') != 'PASS_FINITE_ONLY' or finite.get('passed') is not True
            or finite.get('actual_navigation_verified') is not False or finite.get('tests') != 61
            or finite.get('execution', {}).get('exit_code') != 0
            or not isinstance(finite_pins, dict) or not finite_pins
            or any(bindings.get(name) != digest for name, digest in finite_pins.items())):
        raise RuntimeError('V24 fresh finite results or exact tested source pins differ')
    archive = json.loads(Path(full46_review['path']).read_text())
    if (archive.get('schema') != 'recovery_replan_v24_full46_launch_review/v1'
            or archive.get('reviewed') is not True or archive.get('actual_navigation_verified') is not False
            or any(archive.get(key) is not True for key in ('control_math_unchanged',
                'profile_original46_geometry_unchanged', 'five_geometry_sources_direct', 'full46_only_authorization', 'original_safety_unchanged', 'protected_zero_not_path_exhaustion', 'genuine_exhausted_or_no_geometry_progress_replan_preserved'))):
        raise RuntimeError('V24 direct geometry archive revision has no independent finite review')
    archive_pins = archive.get('source_bindings')
    archive_mandatory = [here/'geometry_archive.py', here/'replan_policy.py', here/'shared_controller.py', here/'recovery_replan_tests.py', here/'full46_launch_contract.py', here/'run.py', here/'pid_scope.py', here/'corridor_preflight.py',
                         here/'profiles'/allowed[0], *geometry_helper_files(here)]
    if (not isinstance(archive_pins, dict) or not archive_pins
            or any(str(p.resolve()) not in archive_pins for p in archive_mandatory)
            or any(bindings.get(name) != digest for name, digest in archive_pins.items())):
        raise RuntimeError('V24 archive review source pins differ from its direct gate bindings')
    control = json.loads(Path(review['path']).read_text())
    if (control.get('schema') != 'curvature_v21_serialization_control_review/v1'
            or control.get('reviewed') is not True or control.get('actual_navigation_verified') is not False
            or control.get('original_geometry_timeout_dwell_or_safety_changed') is not False
            or control.get('serialization_and_intended_false_predicate_repair') is not True
            or control.get('numeric_formula_or_gains_changed') is not False
            or control.get('SLAM_CPP_changed') is not False
            or control.get('cross_frame_pipeline_changed') is not False):
        raise RuntimeError('V24 new reference requires an explicit limited control review')
    control_pins = control.get('reviewed_source_sha256', {})
    if not {'controller.py', 'cascade_core.py', 'spatial_reference.py'}.issubset(control_pins):
        raise RuntimeError('New V24 production control/reference pins are missing')
    for name, digest in control_pins.items():
        target = (here/name).resolve()
        if not target.is_relative_to(here) or sha(target) != digest:
            raise RuntimeError('Reviewed V24 control/reference source changed')
    if not any(profile == json.loads(p.read_text()) for p in (here/'profiles').glob('*.json')):
        raise RuntimeError('Only an exact source-bound reviewed V24 profile may run')
    return d
