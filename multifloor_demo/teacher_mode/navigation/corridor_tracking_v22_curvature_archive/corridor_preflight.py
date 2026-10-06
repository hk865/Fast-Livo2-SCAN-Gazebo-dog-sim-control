"""A fresh V22 control/glue review is mandatory; V19 never authorizes V22."""
from pathlib import Path
from geometry_archive import geometry_helper_files
import hashlib
import json

if not __debug__:
    raise RuntimeError('V22 preflight forbids optimized Python')

REQUIRED = frozenset(('inherited_V19_SLAM_core_and_binary',
    'new_spatial_reference_finite_control', 'production_protection_and_publication_paths',
    'new_glue_profiles_and_fail_closed_gate', 'prospective_original_routes_and_limits',
    'no_historical_navigation_pass_inherited', 'native_scalar_JSON_and_stop_predicate_finite', 'five_geometry_helpers_direct_archive'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evidence_files(here):
    path = Path(here)/'CURVATURE_V22_PREFLIGHT.json'
    if not path.is_file():
        raise RuntimeError('V22 control and glue have no fresh independently reviewed preflight')
    d = json.loads(path.read_text())
    return [path, *[Path(x) for x in d.get('verified_inputs_sha256', {})]]


def verify_preflight(here, profile):
    here = Path(here).resolve()
    path = here/'CURVATURE_V22_PREFLIGHT.json'
    if not path.is_file():
        raise RuntimeError('V22 requires a new finite control/protection/glue review; historical V19 PASS is not inherited')
    d = json.loads(path.read_text())
    if (d.get('schema') != 'curvature_v22_archive_preflight/v1'
            or d.get('status') != 'PASS_LIMITED_NEW_EXPERIMENT' or d.get('candidate_root') != str(here)
            or d.get('allowed') is not True or d.get('actual_navigation_verified') is not False
            or d.get('historical_pass_inherited') is not False
            or set(d.get('checks', {})) != REQUIRED or any(v is not True for v in d['checks'].values())):
        raise RuntimeError('V22 finite control/protection review is incomplete or has inherited acceptance')
    allowed = d.get('allowed_profiles')
    if allowed != ['curvature_original46_prefix9_on.json']:
        raise RuntimeError('V22 only authorizes the original9 bounded ON repair test')
    expected = json.loads((here/'profiles'/allowed[0]).read_text())
    if (profile != expected or profile.get('controller_selector') != 'v22_curvature_archive'
            or profile.get('scan_workspace_selector') != 'protected_baseline'
            or profile.get('original46_prefix_regions') != 9 or profile.get('mission46_required') is not False
            or profile.get('cascade', {}).get('spatial_reference', {}).get('curvature_feedforward_enabled') is not True
            or profile.get('cascade', {}).get('spatial_reference', {}).get('curvature_speed_limit_enabled') is not True):
        raise RuntimeError('Only exact bounded V22 ON repair profile may run')
    from prefix_contract import validate_prefix
    validate_prefix(profile)
    p = profile.get('pipeline', {})
    if (p.get('schema') != 'pipeline_v19_contract/v1' or p.get('mode') not in ('serial', 'rx_decode', 'staged')
            or type(p.get('image_copy_opt')) is not int or p['image_copy_opt'] not in (0, 1)
            or p.get('max_items') != 512 or p.get('reserved_bytes') != 67108864
            or profile.get('pose_cloud_timeout_s') != .3 or profile.get('navigation_ground_truth_used') is not False
            or profile.get('lio_jacobian_parallelism', {}).get('threads') != 4
            or profile.get('vio_patch_parallel', {}).get('threads') != 1):
        raise RuntimeError('V22 original bounded pipeline, source freshness, LIO4/VIO1 or actual navigation contract changed')
    from slam_workspace import evidence_files as slam_evidence_files
    from scan_workspace import evidence_files as scan_evidence_files
    required = list(here.glob('*.py')) + list((here/'profiles').glob('*.json'))
    required += [here/'CURVATURE_V22_CONTRACT.json', *geometry_helper_files(here), *slam_evidence_files(), *scan_evidence_files(profile)]
    review = d.get('control_review')
    archive_review = d.get('archive_review')
    receipts = d.get('finite_receipts')
    if (not isinstance(review, dict) or not isinstance(receipts, list) or not receipts
            or not isinstance(review.get('path'), str) or not review.get('sha256')
            or not isinstance(archive_review, dict) or not isinstance(archive_review.get('path'), str)
            or not archive_review.get('sha256')):
        raise RuntimeError('V22 must bind its new explicit controller review and actual finite receipts')
    required += [Path(review['path']), Path(archive_review['path'])]
    for receipt in receipts:
        if not isinstance(receipt, dict) or not isinstance(receipt.get('path'), str) or not receipt.get('sha256'):
            raise RuntimeError('Malformed V22 finite evidence binding')
        required.append(Path(receipt['path']))
    bindings = d.get('verified_inputs_sha256')
    if not isinstance(bindings, dict) or not bindings or any(str(x.resolve()) not in bindings for x in required):
        raise RuntimeError('V22 mandatory candidate/source/control/build/evidence bindings are incomplete')
    for name, digest in bindings.items():
        if not Path(name).is_absolute() or sha(name) != digest:
            raise RuntimeError('V22 reviewed source or receipt changed: ' + name)
    for receipt in [review, archive_review, *receipts]:
        if bindings.get(str(Path(receipt['path']).resolve())) != receipt['sha256']:
            raise RuntimeError('V22 declared finite receipt SHA is not the actually verified binding')
    archive = json.loads(Path(archive_review['path']).read_text())
    if (archive.get('schema') != 'curvature_v22_direct_archive_review/v1'
            or archive.get('reviewed') is not True or archive.get('actual_navigation_verified') is not False
            or any(archive.get(key) is not True for key in ('control_math_unchanged',
                'profile_control_geometry_unchanged', 'direct_five_geometry_sources', 'immutable_snapshot_required'))):
        raise RuntimeError('V22 direct geometry archive revision has no independent finite review')
    archive_pins = archive.get('source_bindings')
    archive_mandatory = [here/'geometry_archive.py', here/'pid_scope.py', here/'corridor_preflight.py',
                         here/'profiles'/allowed[0], *geometry_helper_files(here)]
    if (not isinstance(archive_pins, dict) or not archive_pins
            or any(str(p.resolve()) not in archive_pins for p in archive_mandatory)
            or any(bindings.get(name) != digest for name, digest in archive_pins.items())):
        raise RuntimeError('V22 archive review source pins differ from its direct gate bindings')
    control = json.loads(Path(review['path']).read_text())
    if (control.get('schema') != 'curvature_v21_serialization_control_review/v1'
            or control.get('reviewed') is not True or control.get('actual_navigation_verified') is not False
            or control.get('original_geometry_timeout_dwell_or_safety_changed') is not False
            or control.get('serialization_and_intended_false_predicate_repair') is not True
            or control.get('numeric_formula_or_gains_changed') is not False
            or control.get('SLAM_CPP_changed') is not False
            or control.get('cross_frame_pipeline_changed') is not False):
        raise RuntimeError('V22 new reference requires an explicit limited control review')
    control_pins = control.get('reviewed_source_sha256', {})
    if not {'controller.py', 'cascade_core.py', 'spatial_reference.py'}.issubset(control_pins):
        raise RuntimeError('New V22 production control/reference pins are missing')
    for name, digest in control_pins.items():
        target = (here/name).resolve()
        if not target.is_relative_to(here) or sha(target) != digest:
            raise RuntimeError('Reviewed V22 control/reference source changed')
    if not any(profile == json.loads(p.read_text()) for p in (here/'profiles').glob('*.json')):
        raise RuntimeError('Only an exact source-bound reviewed V22 profile may run')
    return d
