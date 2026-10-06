#!/usr/bin/env python3
"""Read-only local dashboard for recorded Teacher Gazebo tests.

No ROS publisher, simulation control, or training process is created here.
"""
from __future__ import annotations

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
import math
import os
import re
import stat
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlsplit


ROOT = Path(__file__).resolve().parents[1]

# Only these task-owned recording roots are external storage exceptions.
EXTERNAL_RUN_STORAGE = Path('/var/tmp/go2_teacher_simulation_20261005')
PARALLEL_RUN_STORAGE = Path('/var/tmp/go2_teacher_parallel_20261005')
PIPELINE_RUN_STORAGE = Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs')
EXTERNAL_RUN_NAME = re.compile(r'^[0-9]{8}_[0-9]{6}_closed_loop_cascade_[A-Za-z0-9_-]+_[0-9a-f]{4}$')


def recording_path(path, local_runs):
    try:
        path, local_runs = Path(path), Path(local_runs).resolve()
        resolved = path.resolve()
        if resolved.is_relative_to(local_runs):
            return resolved
        root = next((candidate for candidate in (EXTERNAL_RUN_STORAGE, PARALLEL_RUN_STORAGE, PIPELINE_RUN_STORAGE)
                     if resolved.is_relative_to(candidate)), None)
        if root is None:
            return None
        metadata = root.lstat()
        if (stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode)
                or metadata.st_uid != os.getuid() or stat.S_IMODE(metadata.st_mode) != 0o700
                or root.resolve() != root):
            return None
        relative = resolved.relative_to(root)
        if not relative.parts or not EXTERNAL_RUN_NAME.fullmatch(relative.parts[0]):
            return None
        child = root / relative.parts[0]
        child_metadata = child.lstat()
        if (stat.S_ISLNK(child_metadata.st_mode) or not stat.S_ISDIR(child_metadata.st_mode)
                or child_metadata.st_uid != os.getuid() or child.resolve() != child):
            return None
        lexical = path.absolute()
        if lexical.is_relative_to(local_runs):
            alias_parts = lexical.relative_to(local_runs).parts
            if not alias_parts or alias_parts[0] not in (child.name, 'latest'):
                return None
        alias = local_runs / child.name
        if not alias.is_symlink() or alias.resolve() != child:
            return None
        return resolved
    except (OSError, ValueError, TypeError, RuntimeError):
        return None


def recording_read_allowed(path):
    # Guard fixed recording JSON/telemetry paths against escaping symlinks.
    path = Path(path).absolute()
    local_runs = ROOT / 'runs'
    if any(path.is_relative_to(root) for root in (local_runs, EXTERNAL_RUN_STORAGE, PARALLEL_RUN_STORAGE, PIPELINE_RUN_STORAGE)):
        return recording_path(path, local_runs) is not None
    return True


CLOCK_HOLD_RECEIPT_SCHEMA = 'independent_actual_SLAM_SCAN_clock_hold_navigation/v1'
CLOCK_HOLD_CRITERIA_SHA256 = '9c2b5f2ff6a8f6525df662411ca51976ee3aa6196661f3b774b8a30bf4259f65'
CLOCK_HOLD_ADDITIONAL_CHECKS = frozenset((
    'prospective_clock_hold_criteria_and_archived_sources',
    'actual_clock_hold_event_integrity_and_short_freeze',
    'actual_clock_hold_stall_stale_reset_and_failure',
    'actual_clock_hold_nonzero_resume_requires_native_geometry',
    'actual_publication_slew_with_hold_chronology',
    'actual_executor_ack_and_PI_replay_with_explicit_hold_resets',
))
PUBLICATION_LEDGER_RECEIPT_SCHEMA = 'independent_actual_SLAM_SCAN_publication_ledger_navigation/v2'
PUBLICATION_LEDGER_CRITERIA_SHA256 = '756ad4c3f78a68f42483c9d016a7caa49e2bcbfd22f41595de4ac1675acb480c'
PUBLICATION_LEDGER_ADDITIONAL_CHECKS = CLOCK_HOLD_ADDITIONAL_CHECKS | frozenset((
    'actual_control_publication_ledger_complete_and_original',
))
PROSPECTIVE_RECEIPT_CONTRACTS = {
    CLOCK_HOLD_RECEIPT_SCHEMA: {
        'sha256': CLOCK_HOLD_CRITERIA_SHA256,
        'authority': 'test_results/lidar_density_rate_20261005/evaluation/clock_hold_v9/PROSPECTIVE_CRITERIA.json',
        'snapshot_name': 'PROSPECTIVE_CLOCK_HOLD_CRITERIA.json',
        'additional_checks': CLOCK_HOLD_ADDITIONAL_CHECKS,
    },
    PUBLICATION_LEDGER_RECEIPT_SCHEMA: {
        'sha256': PUBLICATION_LEDGER_CRITERIA_SHA256,
        'authority': 'test_results/lidar_density_rate_20261005/evaluation/publication_ledger_v11/PROSPECTIVE_CRITERIA.json',
        'snapshot_name': 'PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json',
        'candidates': ('navigation/lidar_sampling_v11/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json',
                       'navigation/lidar_sampling_v12/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json',
                       'navigation/lidar_sampling_v14/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json',
                       'navigation/parallel_lio_v15/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json',
                       'navigation/parallel_vio_v16/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json',
                       'navigation/ingress_pipeline_v17/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json',
                       'navigation/combined_compute_v18/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json'),
        'parent_sha256': CLOCK_HOLD_CRITERIA_SHA256,
        'additional_checks': PUBLICATION_LEDGER_ADDITIONAL_CHECKS,
    },
}

# Only immutable display inputs use this cache. A replacement or edit invalidates
# the digest even when its size and mtime are deliberately retained.
SOURCE_DIGEST_CACHE = {}
SOURCE_DIGEST_LOCK = threading.Lock()


def source_file_sha256(path):
    path = Path(path)
    def identity():
        stat = path.stat()
        return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
    before = identity()
    key = str(path.resolve())
    with SOURCE_DIGEST_LOCK:
        cached = SOURCE_DIGEST_CACHE.get(key)
    if cached and cached[0] == before:
        if identity() != before:
            raise OSError('Source changed while verifying cached SHA256')
        return cached[1]
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            hasher.update(block)
    digest = hasher.hexdigest()
    if identity() != before:
        raise OSError('Source changed while reading SHA256')
    with SOURCE_DIGEST_LOCK:
        SOURCE_DIGEST_CACHE[key] = (before, digest)
        while len(SOURCE_DIGEST_CACHE) > 256:
            SOURCE_DIGEST_CACHE.pop(next(iter(SOURCE_DIGEST_CACHE)))
    return digest


CLOCK_HOLD_REPLACED_CHECK = 'actual_executor_ack_and_cascade_PI_COM_PD_replay'
CLOCK_HOLD_REQUIRED_COMMON_CHECKS = frozenset((
    'execution_phase_status_and_cleanup_boundary', 'frozen_scope_and_archived_sources',
    'actual_causal_SLAM_IMU_cascade_updates', 'actual_checked_SCAN_trajectory_payloads',
    'fixed_goal_and_original_SCAN_bounded_projection', CLOCK_HOLD_REPLACED_CHECK,
    'actual_raw_cloud_bytes_fields_filtering', 'actual_cascade_movement_guard_raw_geometry',
    'actual_command_original_read_dual_TTL_slew', 'all_original_SLAM_3D_region_arrivals',
    'runtime_CPU_single_writer_complete_and_drained', 'native_continuous_200Hz',
    'native_physical_safety', 'native_force_velocity_and_support', 'exclusive_Teacher_CPU_identity',
    'actual_sensor_graph_actor247_CPU_joint_execution', 'independent_offline_actual_route_speed_heading',
    'new_first_declared_active_hold_fixed_5s', 'strict_nonflat_complete_contact_geometry',
))

TERRAIN_METADATA_RECEIPT_SCHEMA = 'independent_exact_terrain_status_payload_metadata_join/v2'
TERRAIN_METADATA_READER_SHA256 = 'cb2e8aa4b383f5feb26b7ba154158062a289d266b081d2d76e417944c9267c94'
TERRAIN_METADATA_V1_READER_SHA256 = '1d464d0a70e9b7a5484a9535727683cd8118743a2d2282afa792ff302cb06c53'
TERRAIN_METADATA_ADDITIONAL_CHECKS = frozenset((
    'all_other_original_common_v2_ramp_numeric_gates',
    'frozen_exact_archival_metadata_addition',
    'exact_unique_full_payload_and_original_causal_join',
    'all_original_terrain_switch_ray_and_native_conditions',
    'all_three_ancestor_byte_bindings_and_exact_mandatory_check_sets',
))
TERRAIN_CAUSAL_CHECK = 'original_source_authorized_causal_terrain_layer_switch'
TERRAIN_CAUSAL_ORIGINAL_REASON = 'MissingEvidence: Original provider status read not joined to raw status history'
TERRAIN_RAMP_REQUIRED_CHECKS = frozenset((
    'all_unchanged_original_common_gates', 'prospective_exact_ramp_contract',
    'actual_native_iteration_phase_continuity', 'actual_native_COM_and_origin_translation_consistency',
    'original_named_toe_world_contact_XYZ_normals_available', TERRAIN_CAUSAL_CHECK,
    'unchanged_first_active_hold_on_final_actual_landing',
)) | frozenset('native_recheck_' + name for name in (
    'native_continuous_200Hz', 'native_physical_safety',
    'native_force_velocity_and_support', 'exclusive_Teacher_CPU_identity',
)) | frozenset('leg_' + str(index) + '_' + ramp + '_' + suffix
    for index, ramp in enumerate(('ramp_12', 'ramp_23')) for suffix in (
        'complete_original_12m_fixture', 'actual_COM_entryoutside_full_axis_exitoutside',
        'all_named_feet_ordered_actual_top_support', 'first_destination_landing_continuous_support',
        'all_twelve_metre_bins_actual_support', 'whole_traversal_real_support_and_axis_safety',
        'actual_original_SLAM_destination_region_on_correct_physical_floor',
    ))


def finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def vector(value, size=3):
    if isinstance(value, dict):
        if isinstance(value.get('linear'), dict):
            linear, angular = value['linear'], value.get('angular', {})
            value = [linear.get('x'), linear.get('y'), angular.get('z')]
        else:
            keys = ('x', 'y', 'z') if any(k in value for k in ('x', 'y', 'z')) else ('vx', 'vy', 'wz')
            value = [value.get(k) for k in keys]
    if not isinstance(value, (list, tuple)) or len(value) < size:
        return None
    values = [finite_number(v) for v in value[:size]]
    return values if all(v is not None for v in values) else None


def first(row, names):
    for name in names:
        value = row.get(name)
        if value is not None:
            return value
    return None


def normalize(row):
    """Keep only display fields; never synthesize missing measurements."""
    if not isinstance(row, dict):
        return None
    stamp = finite_number(first(row, ('sim_time', 'sim', 't', 'time', 'stamp', 'clock')))
    if stamp is None and finite_number(row.get('stamp_ns')) is not None:
        stamp = float(row['stamp_ns']) / 1e9
    if stamp is None:
        return None
    command = vector(first(row, ('command', 'cmd', 'cmd_vel', 'requested', 'requested_command')))
    measured = vector(first(row, ('measured', 'measured_velocity', 'velocity_body', 'actual_velocity')))
    if measured is None:
        linear = vector(first(row, ('base_lin_vel', 'body_lin_vel', 'linear_velocity_body')))
        angular = vector(first(row, ('base_ang_vel', 'body_ang_vel', 'angular_velocity_body')))
        if linear is not None and angular is not None:
            measured = [linear[0], linear[1], angular[2]]
    position = vector(first(row, ('position', 'p', 'base_position', 'truth_position', 'ground_truth_position')))
    if position is None and isinstance(row.get('pose'), dict):
        position = vector(row['pose'].get('position'))
    rpy = vector(first(row, ('rpy', 'roll_pitch_yaw', 'attitude')))
    if rpy is None:
        roll, pitch, yaw = (finite_number(row.get(k)) for k in ('roll', 'pitch', 'yaw'))
        if None not in (roll, pitch, yaw):
            rpy = [roll, pitch, yaw]
    return {
        't': stamp, 'command': command, 'measured': measured, 'position': position,
        'rpy': rpy, 'contacts': first(row, ('contacts', 'foot_contacts', 'contact')),
        'state': first(row, ('state', 'mode', 'controller_state')),
        'test': first(row, ('test', 'test_name', 'case', 'phase')),
        'reason': first(row, ('reason', 'failure', 'error', 'fault')),
    }


def read_json(path):
    try:
        if not recording_read_allowed(path):
            return {}
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {'data': value}
    except (OSError, ValueError):
        return {}


def pid_receipt_view(directory):
    original = read_json(directory / 'summary_pid_navigation_independent.json')
    correction = read_json(directory / 'display_correction.json')
    selected = original
    verified = False
    if (correction.get('schema') == 'pid_analysis_display_correction/v1'
            and correction.get('original_filename') == 'summary_pid_navigation_independent.json'
            and correction.get('corrected_filename') in ('summary_pid_navigation_independent.corrected_v1.json',
                'summary_pid_navigation_independent.corrected_v2.json')
            and all(correction.get(k) is True for k in ('rawinputhashes_unchanged', 'analysis_only',
                'protocol_thresholds_unchanged', 'old_receipt_kept'))):
        corrected = read_json(directory / correction['corrected_filename'])
        version = 'v2' if correction['corrected_filename'].endswith('.corrected_v2.json') else 'v1'
        fix_path = ROOT / f'tests/pid_navigation/independent_receipt_fix_{version}.json'
        try:
            verified = bool(corrected and original and
                hashlib.sha256((directory / correction['original_filename']).read_bytes()).hexdigest() == correction['original_summary_sha256'] and
                hashlib.sha256((directory / correction['corrected_filename']).read_bytes()).hexdigest() == correction['corrected_summary_sha256'] and
                hashlib.sha256(fix_path.read_bytes()).hexdigest() == correction['fix_receipt_sha256'] and
                original.get('input_hashes') == corrected.get('input_hashes') and
                original.get('protocol') == corrected.get('protocol'))
        except (OSError, KeyError):
            verified = False
        if verified:
            selected = corrected
    return selected, original, {**correction, 'display_hashes_verified': verified} if correction else {}


def closed_loop_receipt(directory, filename, schema):
    """Select only this run's authored receipt; retain the original payload.

    This display check does not replay the evaluator or declare absent evidence
    successful. A copied receipt from another run and an internally inconsistent
    PASS remain unverified even when a filename looks authoritative.
    """
    path = directory / filename
    raw = read_json(path)
    if not raw:
        return {}
    errors = []
    if raw.get('schema') != schema:
        errors.append('Receipt schema does not match this result type')
    try:
        same_run = Path(raw.get('run', '')).resolve() == directory.resolve()
    except (TypeError, ValueError, OSError):
        same_run = False
    if not same_run:
        errors.append('Receipt belongs to another run')
    scope = raw.get('scope') or {}
    if schema != 'teacher_closed_loop_runtime_summary/v1':
        if not isinstance(scope, dict) or scope.get('navigation_ground_truth_used') is not False:
            errors.append('Actual sensor navigation source declaration is missing')
        checks = raw.get('checks')
        if not isinstance(checks, dict) or not checks:
            errors.append('Independent checks are missing')
            checks = {}
    else:
        checks = {}
    counts = {'passed': 0, 'failed': 0, 'unverified': 0, 'total': len(checks)}
    for item in checks.values():
        if isinstance(item, dict) and item.get('passed') is True and item.get('status') == 'passed':
            counts['passed'] += 1
        elif isinstance(item, dict) and item.get('passed') is False and item.get('status') == 'failed':
            counts['failed'] += 1
        else:
            counts['unverified'] += 1
    claimed = raw.get('status')
    if claimed == 'passed' and (not checks or counts['passed'] != counts['total']):
        errors.append('Claimed PASS contains missing or non-passing checks')
    if claimed == 'failed' and schema != 'teacher_closed_loop_runtime_summary/v1' and not counts['failed']:
        errors.append('Claimed FAIL has no explicit failed independent check')
    if claimed not in ('passed', 'failed', 'unverified'):
        errors.append('Independent result status is unsupported')
    try:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        errors.append('Receipt changed while reading')
        digest = None
    return {'filename': filename, 'sha256': digest, 'raw': raw,
            'valid_for_selected_run': not errors, 'validation_errors': errors,
            'status': claimed if not errors else 'unverified', 'counts': counts}


def closed_loop_clock_hold_receipt(directory, common):
    """Verify the prospective receipt's small source bindings, not its math.

    The independent evaluator performs replay. This display preserves its old
    common receipt, checks the exact permitted replacement and rejects copied,
    incomplete, changed-source or internally inconsistent result files.
    """
    filename = 'summary_closed_loop_clock_hold_independent.json'
    prospective = read_json(directory / filename)
    schema = prospective.get('schema') if isinstance(prospective, dict) else None
    contract = PROSPECTIVE_RECEIPT_CONTRACTS.get(schema)
    result = closed_loop_receipt(directory, filename, schema if contract else CLOCK_HOLD_RECEIPT_SCHEMA)
    if not result:
        return {}
    raw, errors = result['raw'], list(result['validation_errors'])
    if contract is None:
        result.update(valid_for_selected_run=False, status='unverified',
                      validation_errors=errors + ['Unsupported prospective receipt contract'])
        return result
    expected_sha256 = contract['sha256']
    is_ledger = schema == PUBLICATION_LEDGER_RECEIPT_SCHEMA
    original = common.get('raw', {})
    checks = raw.get('checks') if isinstance(raw.get('checks'), dict) else {}
    original_checks = original.get('checks') if isinstance(original.get('checks'), dict) else {}
    if not common.get('valid_for_selected_run') or raw.get('raw_common_receipt_sha256') != common.get('sha256'):
        errors.append('Original common receipt is missing, foreign or its SHA256 binding differs')
    if raw.get('raw_common_status') != original.get('status'):
        errors.append('Original common receipt status binding differs')
    if raw.get('source_bindings_verified') is not True or raw.get('unchanged_common_checks_verified') is not True:
        errors.append('Independent source or unchanged-common verification is not true')
    if raw.get('explicit_replaced_common_checks') != [CLOCK_HOLD_REPLACED_CHECK]:
        errors.append('Only the prospective PI bookkeeping replacement is permitted')
    if not CLOCK_HOLD_REQUIRED_COMMON_CHECKS.issubset(original_checks):
        errors.append('Complete original execution/source/guard/region/parking checks are missing')
    if set(checks) != set(original_checks) | contract['additional_checks']:
        errors.append('Prospective receipt is missing or adding undeclared independent checks')
    if any(checks.get(name) != item for name, item in original_checks.items()
           if name != CLOCK_HOLD_REPLACED_CHECK):
        errors.append('An unchanged common check payload or status was altered')
    inputs = original.get('verified_input_source_sha256') or {}
    if not isinstance(inputs, dict):
        inputs = {}
    if raw.get('verified_input_source_sha256', inputs) != inputs:
        errors.append('Original recorded-input bindings were changed')
    if raw.get('criteria') != original.get('criteria') or raw.get('scope') != original.get('scope'):
        errors.append('Original common criteria or navigation scope payload was altered')
    try:
        binding_names = ('navigation_scope.json', 'source_manifest.json', 'navigation_source_snapshots.json',
                         'navigation_profile.json', 'runtime_manifest.json', 'slam_loaded_binary.json')
        bindings = raw.get('source_bindings')
        if not isinstance(bindings, dict) or set(bindings) != set(binding_names):
            raise ValueError('Complete prospective source binding filenames are missing')
        for name in binding_names:
            if source_file_sha256(directory / name) != bindings[name]:
                raise ValueError('Prospective source binding mismatch: ' + name)
        for name in ('navigation_scope.json', 'source_manifest.json', 'navigation_source_snapshots.json',
                     'navigation_profile.json', 'navigation_request.json'):
            path = directory / name
            if source_file_sha256(path) != inputs.get(str(path.resolve())):
                raise ValueError('Original source binding mismatch: ' + name)
        scope = read_json(directory / 'navigation_scope.json')
        if (scope.get('schema') != 'teacher_closed_loop_navigation_scope/v1'
                or Path(scope.get('run_dir', '')).resolve() != directory.resolve()
                or scope.get('navigation_ground_truth_used') is not False):
            raise ValueError('Run source scope identity or actual-navigation declaration differs')
        criterion_path = ROOT / contract['authority']
        criterion_bytes = criterion_path.read_bytes()
        if (raw.get('criteria_sha256') != expected_sha256
                or source_file_sha256(criterion_path) != expected_sha256):
            raise ValueError('Prospective criterion SHA256 differs from the frozen contract')
        criterion = json.loads(criterion_bytes)
        if is_ledger:
            parent_path = ROOT / PROSPECTIVE_RECEIPT_CONTRACTS[CLOCK_HOLD_RECEIPT_SCHEMA]['authority']
            if (raw.get('parent_prospective_criteria_sha256') != contract['parent_sha256']
                    or criterion.get('parent_prospective_criteria_sha256') != contract['parent_sha256']
                    or source_file_sha256(parent_path) != contract['parent_sha256']
                    or set(criterion.get('required_extra_check_keys') or []) != contract['additional_checks']
                    or criterion.get('ledger_receipt_schema') != schema):
                raise ValueError('Publication-ledger child/parent criterion contract differs')
            if criterion.get('unchanged_common_numerical_criteria') != original.get('criteria'):
                raise ValueError('Publication-ledger common numerical gates were changed')
        if original.get('evaluator_sha256') != criterion.get('unchanged_common_evaluator_sha256'):
            raise ValueError('Original common evaluator identity differs from the prospective criterion')
        if scope.get('profile', {}).get('control_clock_contract') != criterion.get('frozen_control_clock_contract'):
            raise ValueError('Executed control-clock contract differs from the prospective criterion')
        refs = scope.get('references') or {}
        snapshots = read_json(directory / 'navigation_source_snapshots.json')
        matches = []
        for original_path, row in snapshots.items():
            if not isinstance(row, dict) or Path(original_path).name != contract['snapshot_name']:
                continue
            if is_ledger and Path(original_path).resolve() not in {(ROOT / candidate).resolve() for candidate in contract['candidates']}:
                raise ValueError('Publication-ledger criterion belongs to another candidate source')
            snapshot = Path(row.get('snapshot', '')).resolve()
            if (not snapshot.is_relative_to((directory / 'sources').resolve())
                    or row.get('sha256') != expected_sha256
                    or refs.get(original_path) != expected_sha256
                    or refs.get(str(snapshot)) != expected_sha256
                    or source_file_sha256(snapshot) != expected_sha256):
                raise ValueError('Executed prospective criterion snapshot/source binding differs')
            matches.append(snapshot)
        if len(matches) != 1:
            raise ValueError('Exactly one executed prospective criterion source is required')
        criterion_check = checks.get('prospective_clock_hold_criteria_and_archived_sources') or {}
        verified = criterion_check.get('verified_archived_sources')
        if (raw.get('criteria_binding_source') != str(matches[0])
                or criterion_check.get('criterion_snapshot') != str(matches[0])
                or criterion_check.get('criteria_sha256') != expected_sha256
                or criterion_check.get('common_evaluator_sha256') != criterion.get('unchanged_common_evaluator_sha256')
                or criterion_check.get('source_mismatches') != []
                or not isinstance(verified, dict) or not verified or set(verified) != set(snapshots)):
            raise ValueError('Prospective criterion/source verification payload is missing or inconsistent')
        for original_path, row in snapshots.items():
            observed = verified.get(original_path)
            archived_path = Path(row.get('snapshot', '')).resolve() if isinstance(row, dict) else None
            if (not isinstance(row, dict) or not isinstance(observed, dict)
                    or not archived_path.is_relative_to((directory / 'sources').resolve())
                    or observed.get('snapshot') != row.get('snapshot') or observed.get('sha256') != row.get('sha256')
                    or refs.get(original_path) != row.get('sha256')
                    or refs.get(row.get('snapshot')) != row.get('sha256')):
                raise ValueError('Prospective archived-source bindings differ from this run')
        if is_ledger:
            pinned_sources = criterion.get('pre_test_source_sha256')
            if not isinstance(pinned_sources, dict) or not pinned_sources:
                raise ValueError('Publication-ledger frozen producer source pins are missing')
            criterion_source = next(original for original, row in snapshots.items()
                                    if Path(row.get('snapshot', '')).resolve() == matches[0])
            candidate_directory = Path(criterion_source).parent.resolve()
            for original_path, expected in pinned_sources.items():
                name = Path(original_path).name
                source_name = Path(original_path).resolve() if name == 'evaluate_closed_loop.py' else candidate_directory / name
                source_matches = [(source, row) for source, row in snapshots.items()
                                  if Path(source).name == name]
                if len(source_matches) != 1:
                    raise ValueError('Exactly one archived publication-ledger producer source is required: ' + name)
                actual_source, row = source_matches[0]
                if (Path(actual_source).resolve() != source_name or not isinstance(row, dict)
                        or row.get('sha256') != expected
                        or source_file_sha256(Path(row.get('snapshot', ''))) != expected):
                    raise ValueError('Publication-ledger producer snapshot differs from the pre-test source')
        goals = read_json(directory / 'navigation_request.json').get('goals', [])
        phase = checks.get('execution_phase_status_and_cleanup_boundary', {})
        if phase.get('passed') is True or phase.get('status') == 'passed':
            states = phase.get('execution_states')
            if not isinstance(states, list) or not states or 'failed' in states:
                raise ValueError('A failed or missing actual execution phase cannot claim PASS')
        region = checks.get('all_original_SLAM_3D_region_arrivals', {})
        if region.get('passed') is True or region.get('status') == 'passed':
            arrivals = region.get('arrivals', [])
            if (len(goals) != 32 or len(arrivals) != 32
                    or [x.get('goal_id') for x in arrivals] != [x.get('goal_id') for x in goals]
                    or region.get('actual_last_state') != 'succeeded'
                    or not all(x.get('passed') is True and x.get('status') == 'passed' for x in arrivals)):
                raise ValueError('A partial region prefix cannot claim complete 32-region arrival')
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        errors.append(str(exc))
    result['validation_errors'] = errors
    result['valid_for_selected_run'] = not errors
    result['status'] = raw.get('status') if not errors else 'unverified'
    return result


def closed_loop_terrain_metadata_receipt(directory, common, clock_hold, ramp):
    """Display the separate reader correction without replacing an ancestor gate."""
    filename = 'summary_closed_loop_terrain_metadata_join_independent_v2.json'
    path = directory / filename
    raw = read_json(path)
    if not raw:
        return {}
    errors = []
    checks = raw.get('checks') if isinstance(raw.get('checks'), dict) else {}
    counts = {'passed': 0, 'failed': 0, 'unverified': 0, 'total': len(checks)}
    for item in checks.values():
        key = ('passed' if isinstance(item, dict) and item.get('status') == 'passed' and item.get('passed') is True
               else 'failed' if isinstance(item, dict) and item.get('status') == 'failed' and item.get('passed') is False
               else 'unverified')
        counts[key] += 1
    def require(condition, message):
        if not condition:
            raise ValueError(message)
    def same_payload(a, b):
        return json.dumps(a, sort_keys=True, separators=(',', ':'), allow_nan=False) == json.dumps(b, sort_keys=True, separators=(',', ':'), allow_nan=False)
    def passed(item):
        return isinstance(item, dict) and item.get('status') == 'passed' and item.get('passed') is True
    def verify_map(mapping):
        require(isinstance(mapping, dict) and bool(mapping), 'An original byte-binding map is missing')
        normalized = {}
        for name, expected in mapping.items():
            original_path = Path(name)
            actual = (original_path if original_path.is_absolute() else directory / original_path).resolve()
            require(actual.is_relative_to(directory.resolve()) and actual.is_file(), 'A byte binding is missing or outside this run')
            require(source_file_sha256(actual) == expected, 'Original source or recorded-input bytes changed: ' + str(actual))
            normalized[str(actual)] = expected
        require(len(normalized) == len(mapping), 'Duplicate canonical byte-binding paths')
        return normalized
    try:
        require(raw.get('schema') == TERRAIN_METADATA_RECEIPT_SCHEMA, 'Unsupported metadata-join receipt schema')
        require(Path(raw.get('run', '')).resolve() == directory.resolve(), 'Metadata-join receipt belongs to another run')
        require(raw.get('evaluator_sha256') == TERRAIN_METADATA_READER_SHA256, 'Unknown metadata-join reader')
        authority = ROOT / 'test_results/lidar_density_rate_20261005/evaluation'
        require(source_file_sha256(authority / 'audit_terrain_metadata_v2.py') == TERRAIN_METADATA_READER_SHA256
                and source_file_sha256(authority / 'audit_terrain_metadata.py') == TERRAIN_METADATA_V1_READER_SHA256,
                'Frozen metadata readers changed')
        require(set(checks) == TERRAIN_METADATA_ADDITIONAL_CHECKS, 'Exactly five mandatory metadata audit checks are required')
        require(raw.get('status') in ('passed', 'failed', 'unverified'), 'Unsupported metadata audit status')
        require(raw.get('passed') is (raw.get('status') == 'passed'), 'Metadata audit passed flag differs')
        require(raw.get('status') != 'passed' or counts['passed'] == 5, 'Claimed metadata PASS has a missing or non-passing check')
        require(raw.get('navigation_ground_truth_used') is False
                and raw.get('retrospective_reader_correction_explicit') is True
                and raw.get('original_ramp_status_not_relabelled') is True, 'Reader correction source or preservation declaration is missing')
        ancestors = {common.get('filename'): common, clock_hold.get('filename'): clock_hold, ramp.get('filename'): ramp}
        ancestor_names = {'summary_closed_loop_cascade_independent.json', 'summary_closed_loop_clock_hold_independent.json', 'summary_closed_loop_ramp_independent.json'}
        require(set(ancestors) == ancestor_names and set(raw.get('original_receipts') or {}) == ancestor_names,
                'All three exact original ancestor receipts are required')
        for name, result in ancestors.items():
            binding = raw['original_receipts'][name]
            require(result.get('valid_for_selected_run') is True and result.get('status') == 'unverified'
                    and Path(binding.get('path', '')).resolve() == (directory / name).resolve()
                    and binding.get('sha256') == result.get('sha256') and binding.get('status') == result['raw'].get('status'),
                    'Original ancestor identity, SHA256 or status differs: ' + name)
        c, v, r = common['raw'], clock_hold['raw'], ramp['raw']
        require(v.get('schema') == PUBLICATION_LEDGER_RECEIPT_SCHEMA, 'The original full-publication v2 ancestor is required')
        require(set(c['checks']) == CLOCK_HOLD_REQUIRED_COMMON_CHECKS
                and set(v['checks']) == CLOCK_HOLD_REQUIRED_COMMON_CHECKS | PUBLICATION_LEDGER_ADDITIONAL_CHECKS
                and set(r['checks']) == TERRAIN_RAMP_REQUIRED_CHECKS, 'Original 19/26/25 mandatory gate sets differ')
        nonflat = 'strict_nonflat_complete_contact_geometry'
        for receipt in (c, v):
            item = receipt['checks'][nonflat]
            require(item.get('status') == 'unverified' and item.get('passed') is None
                    and all(passed(value) for name, value in receipt['checks'].items() if name != nonflat),
                    'Another original common or publication gate is missing or non-passing')
        causal = r['checks'][TERRAIN_CAUSAL_CHECK]
        require(causal.get('status') == 'unverified' and causal.get('passed') is None
                and causal.get('reason') == TERRAIN_CAUSAL_ORIGINAL_REASON
                and all(passed(value) for name, value in r['checks'].items() if name != TERRAIN_CAUSAL_CHECK),
                'The original ramp has a different failure, missing gate or partial traversal')
        require(same_payload(v.get('raw_common_checks'), c['checks'])
                and v.get('explicit_replaced_common_checks') == [CLOCK_HOLD_REPLACED_CHECK]
                and same_payload(v['checks'][CLOCK_HOLD_REPLACED_CHECK], v['checks']['actual_executor_ack_and_PI_replay_with_explicit_hold_resets']),
                'Original check payload or the sole explicit PI replacement differs')
        require(r.get('ancestors', {}).get('common', {}).get('sha256') == common['sha256'], 'Ramp common ancestry differs')
        closure = raw.get('source_and_check_closure') or {}
        require(same_payload(closure, {k: value for k, value in checks['all_three_ancestor_byte_bindings_and_exact_mandatory_check_sets'].items() if k not in ('status', 'passed')}),
                'Metadata closure check payload differs')
        exact = closure.get('exact_original_check_sets') or {}
        require((exact.get('common_required_check_count'), exact.get('v2_required_check_count'), exact.get('ramp_required_check_count')) == (19, 26, 25)
                and set(exact.get('common_required_check_keys') or []) == CLOCK_HOLD_REQUIRED_COMMON_CHECKS
                and set(exact.get('v2_extra_check_keys') or []) == PUBLICATION_LEDGER_ADDITIONAL_CHECKS
                and set(exact.get('ramp_required_check_keys') or []) == TERRAIN_RAMP_REQUIRED_CHECKS
                and exact.get('all_required_passed_flags_verified') is True and exact.get('exact_original_common_payload_preserved') is True
                and exact.get('only_permitted_replaced_check') == CLOCK_HOLD_REPLACED_CHECK, 'Declared exact 19/26/25/7 closure differs')
        criterion = Path(closure.get('criterion_snapshot', '')).resolve()
        require(str(criterion) == v.get('criteria_binding_source') and criterion.is_relative_to((directory / 'sources').resolve())
                and closure.get('criterion_sha256') == PUBLICATION_LEDGER_CRITERIA_SHA256
                and source_file_sha256(criterion) == PUBLICATION_LEDGER_CRITERIA_SHA256, 'Metadata and original prospective criteria differ')
        snapshots = read_json(directory / 'navigation_source_snapshots.json')
        sources = {}
        for basename in ('teacher_wrapper.py', 'evaluate_ramp.py', 'evaluate_closed_loop.py'):
            found = [row for name, row in snapshots.items() if Path(name).name == basename]
            require(len(found) == 1, 'A unique archived metadata source is missing: ' + basename)
            row = found[0]; source = Path(row.get('snapshot', '')).resolve()
            require(source.is_relative_to((directory / 'sources').resolve()) and source_file_sha256(source) == row.get('sha256'), 'An archived metadata source changed')
            sources[basename] = source
        require(str(sources['evaluate_ramp.py']) == closure.get('required_common_set_source')
                and closure.get('required_common_set_source_sha256') == r.get('evaluator_sha256')
                and source_file_sha256(sources['evaluate_ramp.py']) == r.get('evaluator_sha256'), 'Frozen full-ramp evaluator differs')
        expected_maps = {}
        for name, fields in (('summary_closed_loop_cascade_independent.json', ('verified_input_source_sha256',)),
                             ('summary_closed_loop_clock_hold_independent.json', ('verified_input_source_sha256', 'clock_hold_verified_input_source_sha256', 'source_bindings')),
                             ('summary_closed_loop_ramp_independent.json', ('verified_input_source_sha256',))):
            for field in fields:
                expected_maps[name + ':' + field] = verify_map(ancestors[name]['raw'].get(field))
        require(same_payload(closure.get('all_three_ancestor_binding_maps_rehashed'), expected_maps), 'Ancestor byte maps are not completely closed')
        inputs = ('summary_closed_loop_cascade_independent.json', 'summary_closed_loop_clock_hold_independent.json', 'summary_closed_loop_ramp_independent.json',
                  'navigation_scope.json', 'navigation_source_snapshots.json', 'navigation_status.jsonl', 'terrain_provider_events.jsonl', 'terrain_provider_result.json',
                  'ramp_acceptance_contract.json', 'navigation_request.json', 'telemetry.jsonl', 'actuator.jsonl', 'terrain_target_manifest.json', 'alternate_terrain_target_manifest.json',
                  'summary_closed_loop_terrain_metadata_join_independent.json')
        own_map = verify_map(raw.get('verified_input_source_sha256'))
        require(set(own_map) == {str((directory / name).resolve()) for name in inputs} | {str(p) for p in sources.values()}, 'Required independent metadata input bindings are missing')
        earlier = raw.get('earlier_metadata_v1_receipt') or {}
        earlier_path = directory / 'summary_closed_loop_terrain_metadata_join_independent.json'
        original = read_json(earlier_path)
        require(Path(earlier.get('path', '')).resolve() == earlier_path.resolve() and earlier.get('sha256') == source_file_sha256(earlier_path)
                and earlier.get('status') == 'passed' and original.get('schema') == 'independent_exact_terrain_status_payload_metadata_join/v1'
                and original.get('evaluator_sha256') == TERRAIN_METADATA_V1_READER_SHA256
                and same_payload(original.get('proofs'), raw.get('proofs'))
                and all(same_payload(original.get('checks', {}).get(name), checks[name]) for name in TERRAIN_METADATA_ADDITIONAL_CHECKS - {'all_three_ancestor_byte_bindings_and_exact_mandatory_check_sets'}),
                'The preserved v1 bookkeeping proof differs')
        fields = ['monotonic_wall', 'ros_sim_time']
        producer = checks['frozen_exact_archival_metadata_addition']; join = checks['exact_unique_full_payload_and_original_causal_join']
        require(producer.get('snapshot') == str(sources['teacher_wrapper.py']) and producer.get('sha256') == own_map[str(sources['teacher_wrapper.py'])]
                and producer.get('archival_fields_only') == fields and join.get('removed_archival_fields') == fields
                and join.get('unique_original_payload_matches') == 1 and join.get('all_original_payload_fields_equal') is True
                and join.get('source_receipts_or_clock_freshness_refreshed') is False, 'Only the producer-proven two-field exact unique join is allowed')
        contract = read_json(directory / 'ramp_acceptance_contract.json')
        require(same_payload(r.get('prospective_contract'), contract) and same_payload(r.get('criteria'), contract.get('criteria'))
                and contract.get('schema') == 'prospective_actual_SLAM_SCAN_complete_ramp_contract/v1'
                and Path(contract.get('run', '')).resolve() == directory.resolve() and contract.get('navigation_ground_truth_used') is False
                and [x.get('ramp_model') for x in contract.get('segments', [])] == ['ramp_12', 'ramp_23']
                and contract.get('criteria', {}).get('nominal_horizontal_ramp_span_m') == 12.0
                and r['checks']['prospective_exact_ramp_contract'].get('original_contract_sha256') == own_map[str((directory / 'ramp_acceptance_contract.json').resolve())],
                'A pilot or incomplete ramp scope cannot be displayed as this audit')
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        errors.append(str(exc))
    return {'filename': filename, 'sha256': source_file_sha256(path), 'raw': raw,
            'valid_for_selected_run': not errors, 'validation_errors': errors,
            'status': raw.get('status') if not errors else 'unverified',
            'counts': counts if not errors else {'passed': 0, 'failed': 0, 'unverified': 1, 'total': 1},
            'display_note': '独立读取器只关联两个生产者证明的归档时间字段，原三份收据与全部数值门保持原样。仅为本轮静态 32 区域、两条完整 12 m 坡道和首次 5 秒停车；Actor 仍含 232 维特权观测。动态障碍、全传感器 Actor 与真机未验证。'}


def closed_loop_receipt_view(directory):
    mission_scope = read_json(directory / 'navigation_scope.json')
    mission_state = read_json(directory / 'mission46_status.json')
    mission_profile = mission_scope.get('profile')
    if ((mission_profile if isinstance(mission_profile, dict) else {}).get('mission46_required') is True
            or mission_state.get('schema') == 'teacher_original_mission46_state/v1'):
        failed = next((r for r in mission46_final_evaluations(directory) if r['status'] == 'failed'), None)
        return {'schema': 'teacher_closed_loop_dashboard_selection/v1',
                'status': 'failed' if failed else 'unverified',
                'counts': failed['counts'] if failed else {'passed': 0, 'failed': 0, 'unverified': 1, 'total': 1},
                'selection_kind': 'mission46_independent_failure' if failed else 'mission46_independent_pending',
                'selected_receipt_filename': failed['filename'] if failed else None,
                'selected_receipt_sha256': failed['sha256'] if failed else None,
                'selected_validation_summary': failed['raw'] if failed else {'checks': {'mission46_independent_acceptance_pending': {
                    'status': 'unverified', 'passed': None,
                    'reason': 'Original46 requires its own 18+14+14 mission evaluation; no V18/common result is inherited'}}},
                'display_note': ('本轮独立失败收据及全部来源哈希已核对；其余未验证项保持未验证。'
                                 if failed else '原始 46 区域任务须独立验收；阶段完成与运行时收据不代表完整导航通过。'),
                'navigation_ground_truth_used': False,
                'historical_results_are_not_this_run_acceptance': True}
    common = closed_loop_receipt(directory, 'summary_closed_loop_cascade_independent.json',
        'independent_actual_SLAM_SCAN_cascade_navigation/v1')
    ramp = closed_loop_receipt(directory, 'summary_closed_loop_ramp_independent.json',
        'independent_actual_SLAM_SCAN_complete_ramp_navigation/v1')
    runtime = closed_loop_receipt(directory, 'summary_closed_loop_navigation.json',
        'teacher_closed_loop_runtime_summary/v1')
    clock_hold = closed_loop_clock_hold_receipt(directory, common)
    terrain_metadata = closed_loop_terrain_metadata_receipt(directory, common, clock_hold, ramp)
    ledger_selected = (clock_hold.get('raw') or {}).get('schema') == PUBLICATION_LEDGER_RECEIPT_SCHEMA
    pilots = [closed_loop_receipt(directory, path.name,
        'independent_actual_SLAM_SCAN_complete_ramp_navigation/v1')
        for path in sorted(directory.glob('summary_closed_loop_ramp_independent.pilot_*.json'))]
    pilots = [receipt for receipt in pilots if receipt]
    scope = read_json(directory / 'navigation_scope.json')
    clock_profile = scope.get('profile') if isinstance(scope.get('profile'), dict) else {}
    clock_expected = bool(clock_profile.get('control_clock_contract'))
    affinity_criterion = str((ROOT / 'navigation/lidar_sampling_v14/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json').resolve())
    execution_variant = ('V14 固定绑核实验' if clock_profile.get('cpu_affinity_enabled') is True
                         and (scope.get('references') or {}).get(affinity_criterion) == PUBLICATION_LEDGER_CRITERIA_SHA256 else None)
    if not any((common, ramp, runtime, pilots, clock_hold)) and scope.get('schema') != 'teacher_closed_loop_navigation_scope/v1':
        return {}
    if clock_hold or clock_expected:
        selected, kind = clock_hold, 'clock_hold_independent' if clock_hold else 'clock_hold_pending'
    else:
        selected, kind = (ramp, 'ramp_independent') if ramp else (common, 'cascade_independent') if common else ({}, 'runtime_unverified')
    if selected:
        summary = dict(selected['raw'])
        if not selected['valid_for_selected_run']:
            # Render one explicit evidence rejection, rather than the foreign
            # or internally inconsistent receipt's apparent passing checks.
            summary = {'status': 'unverified', 'checks': {'selected_run_receipt_integrity': {
                'status': 'unverified', 'passed': None,
                'reason': '; '.join(selected['validation_errors'])}}}
        display_status = selected['status']
        counts = selected['counts'] if selected['valid_for_selected_run'] else {'passed': 0, 'failed': 0, 'unverified': 1, 'total': 1}
        note = ('本轮事前时钟保护联合独立验收；保留原共同收据，另外核对实际零速冻结、复位、恢复几何和发布时序。'
                if kind == 'clock_hold_independent' else '本轮独立验收；') + '仅对应所选 run 的冻结场景、来源和判据。最终进程清理状态不替代验收。'
    else:
        # A clean runtime, a succeeded controller status, a pilot diagnostic,
        # and historical global/truth-PID acceptance cannot certify navigation.
        display_status = 'unverified'
        counts = {'passed': 0, 'failed': 0, 'unverified': 1, 'total': 1}
        summary = {'status': 'unverified', 'checks': {'closed_loop_independent_acceptance_pending': {
            'status': 'unverified', 'passed': None,
            'reason': ('本轮事前时钟保护联合收据尚未完成；旧共同验收不含实际 hold 复核，不能代替。'
                       if clock_expected else '本轮真实 SLAM / SCAN 闭环尚无正式独立验收；模型接口、运行结束和历史结果不能代替。')}}}
        note = '运行记录待独立验收；当前不声明导航、完整坡道或多层通过。'
    if ledger_selected:
        note = '本轮事前时钟保护与完整实际发布账本联合独立验收；保留原共同收据，全部发布与来源须完整核对。仅对应所选 run 的冻结场景、来源和判据。最终进程清理状态不替代验收。'
    if execution_variant:
        note = execution_variant + ('；实际验收尚未完成。' if not clock_hold else '；') + note
    return {'schema': 'teacher_closed_loop_dashboard_selection/v1',
            'status': display_status, 'counts': counts, 'selection_kind': kind,
            'execution_variant': execution_variant,
            'selected_receipt_filename': selected.get('filename'),
            'selected_receipt_sha256': selected.get('sha256'),
            'selected_validation_summary': summary, 'display_note': note,
            'common': common, 'ramp': ramp, 'clock_hold': clock_hold, 'runtime': runtime,
            'terrain_metadata': terrain_metadata,
            'publication_ledger_contract': ledger_selected,
            'pilot_diagnostics': pilots,
            'navigation_ground_truth_used': False,
            'historical_results_are_not_this_run_acceptance': True,
            'shutdown_status_is_not_independent_acceptance': True}


MISSION46_PHASES = (('exploration', '探索', 18, 'exploring'),
                    ('return_origin', '返航', 14, 'returning'),
                    ('navigation_f1_f3', '导航', 14, 'navigating'))
MISSION46_RUNTIME_RECEIPTS = {
    'initialization': ('mission46_initialization_receipt.json', 'teacher_original_origin_initialization/v1',
        {'spawn_matches_original', 'exclusive_teacher_actuator', 'physical_standing',
         'stationary_imu_initialization', 'actual_slam_ready'}),
    'rgb_save': ('mission46_rgb_save_receipt.json', 'teacher_mission46_rgb_save/v1',
        {'actual_saved_rgb_file', 'current_run_rgb_provenance', 'fresh_sensor_evidence', 'not_capacity_truncated'}),
    'dynamic': ('mission46_dynamic_receipt.json', 'teacher_mission46_dynamic_obstacle/v1',
        {'actual_entity_motion', 'actual_slam_scene_trigger', 'fresh_scan_pre_roll', 'obstacle_stop', 'clearance', 'recovery'}),
    'parking': ('mission46_final_parking_receipt.json', 'teacher_mission46_final_parking/v1',
        {'first_five_seconds', 'actual_slam_hold_drift', 'native_motion_limits', 'source_freshness',
         'continuous_teacher_control', 'no_protection'}),
}
MISSION46_FINAL_SCHEMA = 'independent_original46_SLAM_SCAN_mission/v1'
MISSION46_FINAL_SOURCES = frozenset(('navigation_scope.json', 'source_manifest.json',
    'navigation_source_snapshots.json', 'navigation_profile.json', 'runtime_manifest.json',
    'worker_result.json', 'mission46_status.json', 'navigation_anchor.json'))
MISSION46_FINAL_CHECKS = frozenset(('original46_profile_and_archived_sources', 'whole_mission_terminal',
    'runtime_and_owned_cleanup', 'frozen_Teacher_CPU_sole_actuator', 'actual_loaded_SLAM_binary',
    'actual_SLAM_navigation_source', 'three_original_phase_requests',
    'exploration_original_regions_and_actual_dwell', 'return_origin_original_regions_and_actual_dwell',
    'navigation_f1_f3_original_regions_and_actual_dwell', 'actual_original_origin_standing_initialization',
    'three_causal_privileged_terrain_switches', 'actual_current_run_saved_RGB',
    'original_dynamic_stop_clear_recovery', 'final_first5s_SLAM_and_physical_hold',
    'native_physical_and_actuator_safety', 'pipeline_complete_ordered_context_valid_drain',
    'full_200Hz_native_actuator_trace_replay', 'full_publication_PI_and_all_SCAN_geometry_replay'))


def mission46_final_receipt(directory, filename):
    """Recognize source-bound independent failures; this viewer never certifies PASS.

    The independent evaluator performs replay. SHA reads use the stat-sensitive
    source cache, so unchanged final sources are not reread on each UI poll.
    """
    path = directory / filename
    def receipt_identity():
        st = path.stat()
        return (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns)
    try:
        before = receipt_identity()
    except OSError:
        return {}
    raw = read_json(path)
    if not raw:
        return {}
    errors, digest = [], None
    checks = raw.get('checks')
    try:
        run_matches = (isinstance(raw.get('run'), str) and Path(raw['run']).is_absolute()
                       and Path(raw['run']).resolve() == directory.resolve())
    except (OSError, ValueError, TypeError, RuntimeError):
        run_matches = False
    if raw.get('schema') != MISSION46_FINAL_SCHEMA or raw.get('run_id') != directory.name or not run_matches:
        errors.append('Independent schema or selected run/run_id identity differs')
    if (raw.get('simulation_only') is not True or raw.get('navigation_ground_truth_used') is not False
            or raw.get('real_robot_verified') is not False):
        errors.append('Explicit simulation and actual-SLAM navigation declarations are missing')
    counts = {'passed': 0, 'failed': 0, 'unverified': 0, 'total': len(checks) if isinstance(checks, dict) else 0}
    valid_checks = isinstance(checks, dict) and MISSION46_FINAL_CHECKS.issubset(checks)
    if valid_checks:
        for name, check in checks.items():
            if not isinstance(name, str) or not isinstance(check, dict):
                valid_checks = False
                break
            status, passed = check.get('status'), check.get('passed')
            if not ((status == 'passed' and passed is True) or (status == 'failed' and passed is False)
                    or (status == 'unverified' and passed is None)):
                valid_checks = False
                break
            counts[status] += 1
    if not valid_checks:
        errors.append('Independent checks are missing or do not have consistent three-state results')
    if raw.get('status') == 'failed' and (raw.get('passed') is not False or not counts['failed']):
        errors.append('Claimed failure has no explicit failed independent check')
    elif raw.get('status') not in ('failed', 'passed', 'unverified'):
        errors.append('Independent status is unsupported')
    bindings = raw.get('source_bindings')
    if not isinstance(bindings, dict) or not MISSION46_FINAL_SOURCES.issubset(bindings):
        errors.append('Mandatory final source bindings are missing')
    else:
        for name, expected in bindings.items():
            try:
                rel = Path(name)
                evidence = (directory / rel).resolve()
                if (not isinstance(name, str) or rel.is_absolute() or '..' in rel.parts
                        or not evidence.is_relative_to(directory.resolve())
                        or not isinstance(expected, str) or re.fullmatch(r'[0-9a-f]{64}', expected) is None
                        or not recording_read_allowed(evidence) or source_file_sha256(evidence) != expected):
                    raise ValueError('Final source path or bytes differ')
            except (OSError, ValueError, TypeError, RuntimeError):
                errors.append('Final source is missing, foreign or hash-mismatched: ' + str(name))
    try:
        digest = source_file_sha256(path)
        if receipt_identity() != before:
            raise OSError('Final receipt changed during display validation')
    except (OSError, ValueError, TypeError, RuntimeError):
        errors.append('Final receipt SHA could not be verified')
    failed = not errors and raw.get('status') == 'failed' and raw.get('passed') is False and counts['failed'] > 0
    return {'filename': filename, 'raw': raw, 'reported_status': raw.get('status'), 'sha256': digest,
            'status': 'failed' if failed else 'unverified', 'valid_for_selected_run': not errors,
            'validation_errors': errors, 'counts': counts, 'formal_acceptance': False,
            'failure_evidence_verified': failed, 'formal_pass_allowed': False,
            'reason': ('Independent failure and all bound source bytes are verified; unverified checks remain unverified'
                       if failed else '; '.join(errors) or 'PASS is not certified by this limited final-failure reader')}


def mission46_final_evaluations(directory):
    return [r for filename in ('summary_mission46_independent.json', 'mission46_final_evaluation.json',
            'mission46_independent_evaluation.json') if (r := mission46_final_receipt(directory, filename))]


def mission46_runtime_receipt(directory, filename, schema, required):
    raw = read_json(directory / filename)
    if not raw:
        return {}
    errors = []
    if raw.get('schema') != schema or raw.get('run_id') != directory.name:
        errors.append('Runtime receipt schema or selected run identity differs')
    if raw.get('navigation_ground_truth_used', raw.get('ground_truth_used')) is not False:
        errors.append('Actual-navigation source declaration is missing')
    checks = raw.get('checks')
    if (not isinstance(checks, dict) or not required.issubset(checks)
            or any(not isinstance(c, dict) or c.get('status') != 'passed' or c.get('passed') is not True
                   for c in checks.values()) or raw.get('passed') is not True
            or raw.get('binding_verified') is not True):
        errors.append('Runtime checks are missing or not all reported passing')
    try:
        evidence = Path(raw['source_evidence_file']).resolve()
        if (not evidence.is_relative_to(directory.resolve()) or not recording_read_allowed(evidence)
                or evidence.stat().st_size > 64 * 1024 * 1024
                or source_file_sha256(evidence) != raw['source_evidence_sha256']):
            raise ValueError('Evidence path or bytes differ')
    except (OSError, ValueError, KeyError, TypeError, RuntimeError):
        errors.append('Runtime evidence is missing, foreign or hash-mismatched')
    return {'filename': filename, 'raw': raw, 'source_binding_valid': not errors,
            'status': 'passed' if not errors else 'unverified', 'validation_errors': errors,
            'scope': 'runtime-reported checks and exact evidence bytes; no independent numerical replay',
            'formal_mission_acceptance': False}


def mission46_view(directory):
    scope = read_json(directory / 'navigation_scope.json')
    state = read_json(directory / 'mission46_status.json')
    profile = scope.get('profile')
    if not ((profile if isinstance(profile, dict) else {}).get('mission46_required') is True
            or state.get('schema') == 'teacher_original_mission46_state/v1'):
        return {}
    valid = (state.get('schema') == 'teacher_original_mission46_state/v1'
             and state.get('run_id') == directory.name and state.get('navigation_ground_truth_used') is False
             and state.get('expected_region_count') == 46)
    control = read_json(directory / 'mission46_control.json')
    control_valid = (control.get('schema') == 'teacher_mission46_control/v1'
                     and control.get('run_id') == directory.name
                     and control.get('navigation_ground_truth_used') is False and type(control.get('hold')) is bool)
    completed = state.get('completed_stages', []) if valid else []
    if not isinstance(completed, list):
        completed = []
    request = read_json(directory / 'navigation_request.json')
    nav = read_json(directory / 'navigation_status.json')
    if not nav:
        nav = state.get('navigation') if valid and isinstance(state.get('navigation'), dict) else {}
    bound_arrivals = []
    arrival_source_bound = False
    rid = nav.get('request_id')
    goals = request.get('goals')
    try:
        goal_hash = hashlib.sha256(json.dumps(goals, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        arrival_source_bound = (valid and isinstance(rid, str) and rid.startswith(directory.name + ':')
            and request.get('schema_version') == 2 and request.get('request_id') == rid
            and request.get('frame_id') == 'camera_init' and nav.get('frame_id') == 'camera_init'
            and nav.get('mode') == 'teacher' and nav.get('controller_kind') == 'teacher'
            and nav.get('navigation_ground_truth_used') is False
            and nav.get('goals_definitions') == goals and isinstance(goals, list) and bool(goals)
            and nav.get('goals_definition_sha256') == goal_hash
            and state.get('current_goals_sha256') == goal_hash
            and state.get('current_request') in (None, rid))
        if arrival_source_bound:
            definitions = {g['goal_id']: g for g in goals}
            for r in nav.get('region_arrivals', []):
                if not isinstance(r, dict) or r.get('goal_id') not in definitions:
                    continue
                definition = definitions[r['goal_id']].get('arrival') or {}
                stamp, begin, dwell, gap = [r.get(k) for k in ('stamp_ns', 'start_stamp_ns', 'dwell_ns', 'max_observation_gap_ns')]
                if (r.get('request_id') == rid and r.get('goals_definition_sha256') == goal_hash
                        and all(type(x) is int for x in (stamp, begin, dwell, gap))
                        and 0 <= begin <= stamp and dwell == stamp-begin
                        and dwell >= round(float(definition['dwell_sim_s'])*1e9) and 0 < gap <= 200000000
                        and r.get('region_inside') is True and r.get('control_region_inside') is True
                        and r.get('protected') is False and r.get('reason') == 'arrived'
                        and vector(r.get('raw_position')) is not None):
                    bound_arrivals.append(r)
    except (ValueError, KeyError, TypeError, OverflowError):
        arrival_source_bound = False
        bound_arrivals = []
    phases = []
    for name, label, total, stage in MISSION46_PHASES:
        arrived = total if stage in completed else 0
        if (stage not in completed and isinstance(rid, str) and f':{name}:' in rid
                and arrival_source_bound):
            arrived = min(total, len({r.get('goal_id') for r in bound_arrivals
                if isinstance(r, dict) and r.get('request_id') == rid
                and isinstance(r.get('goal_id'), str) and r['goal_id'].startswith(name + ':')}))
        phases.append({'name': name, 'label': label, 'expected': total, 'recorded_arrivals': arrived})
    receipts = {key: mission46_runtime_receipt(directory, *contract)
                for key, contract in MISSION46_RUNTIME_RECEIPTS.items()}
    finals = mission46_final_evaluations(directory)
    final_failure = next((r for r in finals if r['status'] == 'failed'), None)
    return {'schema': 'teacher_mission46_dashboard/v1', 'expected_region_count': 46,
            'phases': phases, 'recorded_arrivals': sum(p['recorded_arrivals'] for p in phases),
            'completed_stage_arrivals': sum(p['expected'] for p in phases
                if next(item[3] for item in MISSION46_PHASES if item[0] == p['name']) in completed),
            'arrival_source_bound': arrival_source_bound,
            'arrival_source': 'navigation_status.json bound to original request and goals hash',
            'stage': state.get('adapter_stage', state.get('stage', 'waiting_record')) if valid else 'waiting_valid_record',
            'state': state if valid else {}, 'state_valid_for_selected_run': valid,
            'state_validation_errors': [] if valid else ['No matching original46 state/source declaration for selected run'],
            'functional_sequence_completed': valid and state.get('functional_sequence_completed') is True,
            'control': control if control_valid else {}, 'control_valid_for_selected_run': control_valid,
            'runtime_receipts': receipts, 'final_evaluations': finals,
            'independent_status': 'failed' if final_failure else 'unverified',
            'selected_independent_failure': final_failure or {}, 'historical_acceptance_inherited': False,
            'navigation_is_certified': False,
            'display_note': '18 探索 + 14 返航 + 14 导航。到达数与 completed 是运行时功能记录，不代表完整独立验收；不继承 V18。动态障碍仅有服务 ACK，无独立 pose/info 观察。'}


def pipeline_view(directory):
    profile = read_json(directory / 'navigation_scope.json').get('profile') or {}
    if not isinstance(profile, dict):
        profile = {}
    summary = read_json(directory / 'fastlivo_debug/pipeline_v19_summary.json')
    contract = profile.get('pipeline') or {}
    if contract.get('schema') != 'pipeline_v19_contract/v1' and summary.get('schema') != 'staged_input_pipeline_v19/v1':
        return {}
    return {'schema': 'teacher_pipeline_v19_dashboard/v1', 'contract': contract, 'summary': summary,
            'summary_schema_valid': summary.get('schema') == 'staged_input_pipeline_v19/v1',
            'navigation_acceptance': False,
            'display_note': '所选运行的输入流水线与排空记录；正常退出不认证运动或导航。'}


PREFIX9_REPORT_SCHEMA = 'independent_original46_prefix9_actual_run/v1'
PREFIX9_REPORT_INDEX = 'test_results/corridor_tracking_v20_continue_20261006/viewer/PREFIX9_REPORT_INDEX.json'
PREFIX9_CANDIDATE_ROOTS = {
    'corridor_tracking_v20': 'navigation/corridor_tracking_v20',
    'v21_curvature_serialization': 'navigation/corridor_tracking_v21_curvature',
    'v22_curvature_archive': 'navigation/corridor_tracking_v22_curvature_archive',
}
PREFIX9_REQUIRED_CHECKS = frozenset(('immutable_source_archive', 'original_geometry_and_slam',
    'all_original_nine_dwells', 'first_fixed5s_parking', 'sampled_native_safety',
    'execution_contract', 'actual_pose_provenance', 'source_read_errors_absent'))
PREFIX9_RUN_SOURCES = frozenset(('runtime_manifest.json', 'navigation_profile.json',
    'navigation_anchor.json', 'navigation_request.json', 'source_manifest.json',
    'policy_manifest.json', 'asset_manifest.json', 'navigation_slam_contract.json',
    'navigation_slam_poses.jsonl', 'navigation_status.jsonl', 'navigation_pid_history.jsonl',
    'telemetry.jsonl', 'actuator.jsonl'))
PREFIX9_VIEW_CACHE = {}
PREFIX9_VIEW_LOCK = threading.Lock()


def _prefix9_identity(path):
    st = Path(path).stat()
    return (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns)


def prefix9_view(directory):
    """Display only a pinned independent prefix receipt, never a full-mission PASS.

    The first read checks all recorded source digests. Subsequent UI refreshes
    stat the same immutable dependencies; edits/replacements invalidate the
    result. This avoids cycling >256 source files through the shared hash cache
    and rereading multi-GB logs on every browser poll.
    """
    directory = Path(directory).resolve()
    profile = read_json(directory / 'navigation_profile.json')
    if (not isinstance(profile, dict) or profile.get('original46_prefix_regions') != 9
            or profile.get('controller_selector') not in PREFIX9_CANDIDATE_ROOTS):
        return {}
    view = dict(schema='teacher_prefix9_limited_dashboard/v1', status='unverified',
        valid_for_selected_run=False, limited_prefix9_pass=False, full46_pass=False,
        whole_200hz_physical_acceptance=False, corridor_control_verified=False,
        original_regions_expected=9, original_regions_verified=None, first5s_parking_verified=False,
        display_note='仅原始前 9 区域与首次固定 5 秒停车的独立有限验收；不代表 full46、全部 200 Hz 物理步骤或走廊控制通过。')
    index_path = ROOT / PREFIX9_REPORT_INDEX
    index = read_json(index_path)
    catalog = index.get('reports') if isinstance(index, dict) else None
    entry = catalog.get(directory.name) if isinstance(catalog, dict) else None
    if not isinstance(entry, dict):
        return {**view, 'validation_errors': ['No pinned independent prefix9 report for the selected run']}
    errors = []; dependencies = {}
    def require(condition, message):
        if not condition: raise ValueError(message)
    def track(path):
        path = Path(path).resolve(); dependencies[str(path)] = _prefix9_identity(path)
        return path
    try:
        require(index.get('schema') == 'teacher_prefix9_report_index/v1', 'Unsupported prefix9 report index')
        require(Path(entry.get('run_directory', '')).resolve() == directory, 'Report index belongs to another run')
        report_path = Path(entry.get('report_file', '')).resolve()
        require(report_path.is_relative_to((ROOT / 'test_results').resolve()), 'Prefix9 report must be an independent test-results artifact')
        require(report_path.suffix == '.json', 'Prefix9 report must be JSON')
        # Identity-only cache validation does not skip any recorded dependency.
        key = str(directory)
        with PREFIX9_VIEW_LOCK:
            cached = PREFIX9_VIEW_CACHE.get(key)
            if cached and cached['entry'] == entry:
                try:
                    if all(_prefix9_identity(p) == ident for p, ident in cached['dependencies'].items()):
                        return dict(cached['view'])
                except OSError:
                    pass
        track(index_path); track(report_path)
        expected = entry.get('report_sha256')
        require(isinstance(expected, str) and re.fullmatch(r'[0-9a-f]{64}', expected), 'Missing exact report SHA256')
        require(source_file_sha256(report_path) == expected, 'Independent prefix9 report bytes changed')
        raw = read_json(report_path)
        require(raw.get('schema') == PREFIX9_REPORT_SCHEMA, 'Unsupported prefix9 independent report schema')
        require(raw.get('run_id') == directory.name and Path(raw.get('run', '')).resolve() == directory,
                'Independent prefix9 report belongs to another selected run')
        checks = raw.get('checks')
        require(isinstance(checks, dict) and set(checks) == PREFIX9_REQUIRED_CHECKS
                and all(type(v) is bool for v in checks.values()), 'Prefix9 independent checks are incomplete')
        require(all(raw.get(k) is False for k in ('full46_pass', 'whole_200hz_physical_acceptance', 'corridor_control_verified')),
                'Prefix receipt attempts to upgrade the verified scope')
        claimed = raw.get('prefix9_limited_pass')
        require(type(claimed) is bool and claimed == all(checks.values()), 'Prefix9 PASS is inconsistent with its checks')
        require(not claimed or raw.get('result_status') == 'limited_pass', 'Unsupported prefix9 PASS status')
        regions = raw.get('original_regions') or {}; parking = raw.get('first5s_parking') or {}
        if claimed:
            receipts = regions.get('receipts') or []
            require(regions.get('passed') is True and regions.get('actual_receipt_count') == 9
                    and regions.get('raw_validated_count') == 9 and len(receipts) == 9
                    and [r.get('goal_id') for r in receipts] == [f'exploration:{i}' for i in range(9)]
                    and all(r.get('passed') is True and r.get('checks')
                        and all(v is True for v in r['checks'].values()) for r in receipts),
                    'Nine original raw-SLAM dwell receipts are not independently passed')
            interval = parking.get('clock_interval_ns')
            require(parking.get('passed') is True and parking.get('checks')
                    and all(v is True for v in parking['checks'].values())
                    and isinstance(interval, list) and len(interval) == 2
                    and all(type(v) is int for v in interval) and interval[1]-interval[0] == 5_000_000_000,
                    'First declared fixed five-second parking window is not independently passed')
            metrics = parking.get('metrics') or {}
            for name, limit in dict(slam_xy_drift_max_m=.05, slam_yaw_drift_max_rad=.1,
                    native_planar_speed_max_mps=.08, native_wz_max_radps=.1, native_yaw_dot_max_radps=.1).items():
                value = metrics.get(name)
                require(type(value) in (int, float) and math.isfinite(value) and 0 <= value <= limit,
                        'Original parking metric missing or exceeds unchanged limit: ' + name)
        bindings = raw.get('source_bindings')
        require(isinstance(bindings, list) and bool(bindings), 'Prefix9 report has no original source bindings')
        baseline_name = (raw.get('r3_comparison') or {}).get('baseline_run')
        baseline = Path(baseline_name).resolve() if isinstance(baseline_name, str) else None
        if baseline is not None:
            require(recording_path(baseline, ROOT / 'runs') == baseline, 'Comparison run is outside allowed recordings')
        candidate = ROOT / PREFIX9_CANDIDATE_ROOTS[profile['controller_selector']]
        helpers = {str((candidate / name).resolve()) for name in (
            'route.py', 'prefix_contract.py', 'mission46_profile.py')}
        helpers |= {str((ROOT.parent / name).resolve()) for name in ('navigation/goal_regions.py', 'mission/route_regions.py')}
        seen = set()
        for binding in bindings:
            require(isinstance(binding, dict) and isinstance(binding.get('file'), str), 'Malformed original source binding')
            source = Path(binding['file'])
            require(source.is_absolute(), 'Source path must be absolute')
            source = source.resolve(); digest = binding.get('sha256')
            require(str(source) not in seen, 'Duplicate original source binding')
            seen.add(str(source))
            in_run = source.is_relative_to(directory) or (baseline is not None and source.is_relative_to(baseline))
            require(in_run or str(source) in helpers, 'Foreign source is not part of this run or its declared comparison')
            require(not in_run or recording_read_allowed(source), 'Source escapes allowed recording roots')
            require(isinstance(digest, str) and re.fullmatch(r'[0-9a-f]{64}', digest), 'Malformed original source SHA256')
            track(source)
            require(source_file_sha256(source) == digest, 'Bound source bytes changed: ' + str(source))
        required = {str((directory / name).resolve()) for name in PREFIX9_RUN_SOURCES}
        required |= helpers
        if claimed: required.add(str((directory / 'worker_result.json').resolve()))
        require(required.issubset(seen), 'Mandatory selected-run raw sources are missing from report bindings')
        for source, identity in dependencies.items():
            require(_prefix9_identity(source) == identity, 'Evidence changed during prefix9 display verification')
        view.update(status='passed_limited' if claimed else 'not_passed', valid_for_selected_run=True,
            limited_prefix9_pass=claimed, original_regions_verified=regions.get('raw_validated_count'),
            first5s_parking_verified=parking.get('passed') is True,
            parking_metrics=parking.get('metrics') or {}, first5s_clock_interval_ns=parking.get('clock_interval_ns'),
            report_file=str(report_path), report_sha256=expected, verified_source_count=len(seen),
            checks=checks, validation_errors=[])
        with PREFIX9_VIEW_LOCK:
            PREFIX9_VIEW_CACHE[key] = dict(entry=entry, dependencies=dependencies, view=dict(view))
            while len(PREFIX9_VIEW_CACHE) > 8: PREFIX9_VIEW_CACHE.pop(next(iter(PREFIX9_VIEW_CACHE)))
        return view
    except (OSError, ValueError, TypeError, RuntimeError, AttributeError, KeyError) as error:
        errors.append(str(error))
    return {**view, 'validation_errors': errors}


def corridor_shadow_view(directory):
    """Read one complete bounded display record; never authorize its use."""
    profile = read_json(directory / 'navigation_scope.json').get('profile') or {}
    if not isinstance(profile, dict) or profile.get('controller_selector') != 'corridor_tracking_v20':
        return {}
    view = dict(schema='teacher_corridor_shadow_dashboard/v1', record_valid_for_selected_run=False,
        status='unavailable', control_authority=False, independent_acceptance=False,
        display_note='V20 走廊仅作 shadow 诊断，不控制路径保留或停车；这里只读原始记录，不代表独立验收或 46 区域通过。')
    path = directory / 'corridor_certificates.jsonl'
    try:
        if not recording_read_allowed(path):
            raise ValueError('corridor recording path escapes allowed roots')
        with path.open('rb') as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            offset = max(0, size - (256 << 10))
            stream.seek(offset)
            lines = stream.read(256 << 10).split(b'\n')
        # The final fragment is either empty or an in-progress record. Never
        # parse that fragment, and never parse a truncated first line.
        complete = lines[1:-1] if offset else lines[:-1]
        record = json.loads(next(line for line in reversed(complete) if line))
        binding = record.get('binding') or {}
        if (record.get('schema') != 'teacher_corridor_certificate/v1'
                or record.get('shadow_only') is not True
                or record.get('control_authority') is not False
                or record.get('navigation_ground_truth_used') is not False
                or not str(binding.get('path_id', '')).startswith(directory.name + ':')):
            raise ValueError('unsupported or foreign shadow record')
        view.update(record_valid_for_selected_run=True, status=record.get('status', 'unavailable'),
                    raw=record, filename=path.name, display_tail_bytes=min(size, 256 << 10))
    except (OSError, ValueError, TypeError, StopIteration, AttributeError) as error:
        view['display_error'] = str(error)
    return view


# The historical page is kept unchanged on disk. This additive read-only
# renderer understands the new receipt types without altering archived pages,
# camera selection, D/K display, RGB sources, or the actual SLAM/SCAN plot.
CLOSED_LOOP_PAGE_SCRIPT = r'''
<script>
(() => {
 const originalShow = show;
 const closedLoopNames = {
  execution_phase_status_and_cleanup_boundary:'运动阶段与进程清理边界',
  frozen_scope_and_archived_sources:'冻结范围、源码与存档哈希',
  actual_causal_SLAM_IMU_cascade_updates:'实际 SLAM / IMU 因果配对与反馈更新',
  actual_checked_SCAN_trajectory_payloads:'实际 SCAN 样条、原始数组与关联',
  fixed_goal_and_original_SCAN_bounded_projection:'固定目标与 SCAN 路径投影',
  actual_executor_ack_and_cascade_PI_COM_PD_replay:'实际执行回执与速度 PI / 路径 PD 重放',
  prospective_clock_hold_criteria_and_archived_sources:'事前时钟保护判据与本轮存档来源',
  actual_clock_hold_event_integrity_and_short_freeze:'实际同钟零速事件与短暂停顿冻结',
  actual_clock_hold_stall_stale_reset_and_failure:'实际长停顿、来源过期与保护复位',
  actual_clock_hold_nonzero_resume_requires_native_geometry:'非零恢复必须有新时钟与实际原生几何检查',
  actual_publication_slew_with_hold_chronology:'含零速 hold 的实际发布顺序与速度过渡',
  actual_executor_ack_and_PI_replay_with_explicit_hold_resets:'实际执行回执与显式 hold 复位的 PI 重放',
  actual_control_publication_ledger_complete_and_original:'全部实际命令发布的原始账本、计数与连续命令链',
  actual_raw_cloud_bytes_fields_filtering:'原始点云字节、字段与过滤来源',
  actual_cascade_movement_guard_raw_geometry:'实际最终命令方向的点云碰撞检查',
  actual_command_original_read_dual_TTL_slew:'实际读取、双时钟 300 ms 超时与命令过渡',
  all_original_SLAM_3D_region_arrivals:'全部目标区域的原始 SLAM 三维到达',
  runtime_CPU_single_writer_complete_and_drained:'CPU 推理、唯一执行器与完整退出',
  native_continuous_200Hz:'原始 200 Hz 物理记录连续性',
  native_force_velocity_and_support:'实际力矩、关节速度与足部支持',
  exclusive_Teacher_CPU_identity:'冻结 Teacher 身份与 CPU 唯一执行权',
  actual_sensor_graph_actor247_CPU_joint_execution:'实际传感器链、247 维 Actor 与关节执行',
  independent_offline_actual_route_speed_heading:'离线实测路线、速度与航向',
  new_first_declared_active_hold_fixed_5s:'首次声明后的固定 5 秒主动停车',
  strict_nonflat_complete_contact_geometry:'完整坡道接触几何（需专门验收）',
  all_unchanged_original_common_gates:'全部原闭环共同判据保持并通过',
  prospective_exact_ramp_contract:'事前冻结的完整坡道验收协议',
  original_source_authorized_causal_terrain_layer_switch:'实际 SLAM 来源触发的特权高度层切换',
  complete_original_required_ramp_evidence:'完整坡道、目的区域与停车证据',
  closed_loop_independent_acceptance_pending:'本轮正式独立验收待完成',
  selected_run_receipt_integrity:'所选 run 的收据来源与一致性'
 };
 const originalCheckName = checkName;
 checkName = name => closedLoopNames[name] || originalCheckName(name);
 const originalCheckDetail = checkDetail;
 checkDetail = item => {
  if(item.name==='all_original_SLAM_3D_region_arrivals'){
   const a=item.metrics?.arrivals||item.arrivals||[],ok=a.filter(x=>x.passed===true&&x.status==='passed').length;
   return `独立核对 ${ok} / ${a.length||32} 个原始 SLAM 三维目标区域；部分路线不代表完整多层通过。`;
  }
  if (!Object.hasOwn(closedLoopNames,item.name) && !String(item.name).startsWith('leg_') && !String(item.name).startsWith('native_recheck_')) return originalCheckDetail(item);
  return item.reason || item.failure_reason || item.metrics?.reason || item.notes ||
   (status(item)[1]==='pass'?'本轮原始来源与事前判据的独立核验通过':status(item)[1]==='fail'?'本轮预定判据未通过；展开查看原始实测指标':'本轮尚无完整证明；不计为通过');
 };
 const closedLoopCaption = data => {
  const result=data.closed_loop_result;if(!result||!Object.keys(result).length)return;
  const nav=data.navigation||{},state=nav.state||{};
  const prefix=data.prefix9_result||{};
  const prefixScope=prefix.valid_for_selected_run===true
    ? (prefix.limited_prefix9_pass===true?'；原前9区＋首固定5s停车独立有限通过（不代表全任务）':'；原前9区独立有限验收未通过') : '';
  $('navigationCaption').textContent=`实际 SLAM ${nav.poses?.records||0} 条 · SCAN ${nav.scan?.hash_verified?'样条 '+nav.scan.trajectory_id+'（原始数组哈希已核对）':'暂无已核对样条'} · 整任务独立验收：${status(result.status)[0]}${prefixScope}。原始最后运行状态：${state.status??state.state??'等待状态'}（可能包含进程清理，不替代验收）。此图不使用 Gazebo 真值；规划曲线不代表已实际到达。`;
 };
 const originalNavigationPlot=navigationPlot;
 navigationPlot=()=>{originalNavigationPlot();if(current)closedLoopCaption(current);};
 const receiptExpansion=new Map();let receiptExpansionRun=null;
 show = data => {
  const renderedRun=typeof data.run_id==='string'?data.run_id:null;
  if(!renderedRun||renderedRun!==receiptExpansionRun){receiptExpansion.clear();receiptExpansionRun=renderedRun;}
  else {$('closedLoopReceipts')?.querySelectorAll('details[data-receipt-key]').forEach(block=>receiptExpansion.set(block.dataset.receiptKey,block.open));}
  originalShow(data);
  const result=data.closed_loop_result;
  if (!result || !Object.keys(result).length) {const old=$('closedLoopReceipts');if(old)old.hidden=true;return;}
  const selected=result.selected_validation_summary||{};
  showTests(selected);
  $('legacyPanel').open=true;
  $('legacyTitle').textContent=result.publication_ledger_contract?'本轮真实 SLAM / SCAN 时钟保护与完整发布账本联合独立验收':result.selection_kind.startsWith('clock_hold_')?'本轮真实 SLAM / SCAN 事前时钟保护联合独立验收':result.selection_kind==='ramp_independent'?'本轮真实 SLAM / SCAN 完整坡道独立验收':'本轮真实 SLAM / SCAN 闭环独立验收';
  $('mainTitle').textContent='Teacher · 真实 SLAM / SCAN 闭环';
  $('mainSubtitle').textContent='实际传感器路线反馈、路径 PD 与速度 PI · CPU 推理 · 所选 run 的独立收据';
  $('pidCorrectionNote').hidden=false;
  $('pidCorrectionNote').textContent=result.display_note+(result.selected_receipt_filename?' 来源：'+result.selected_receipt_filename+' · SHA256 '+result.selected_receipt_sha256:'');
  closedLoopCaption(data);
  let panel=$('closedLoopReceipts');
  if(!panel){panel=document.createElement('section');panel.id='closedLoopReceipts';panel.className='panel tests';$('legacyPanel').after(panel);}
  panel.hidden=false;panel.replaceChildren();
  const heading=document.createElement('div');heading.className='panelhead';
  const title=document.createElement('h2');title.textContent='本轮收据与有限结论';heading.append(title);panel.append(heading);
  const rows=[['新增归档字段关联联合审计（原收据保留）',result.terrain_metadata],[result.publication_ledger_contract?'事前时钟保护与完整发布账本联合验收':'事前时钟保护联合验收',result.clock_hold],['原闭环共同判据'+(result.selection_kind.startsWith('clock_hold_')?'（不替代时钟保护联合验收）':''),result.common],['正式完整坡道验收',result.ramp],
   ...(result.pilot_diagnostics||[]).map(r=>['坡道先导诊断（不升级通过）',r]),['运行记录（不认证导航）',result.runtime]].filter(([,r])=>r&&Object.keys(r).length);
  const visibleReceiptKeys=new Set();
  for(const [label,r] of rows){const block=document.createElement('details'),head=document.createElement('summary');
   if(renderedRun&&typeof r.filename==='string'&&/^[0-9a-f]{64}$/.test(r.sha256||'')){
    const key=JSON.stringify([r.filename,r.sha256]);block.dataset.receiptKey=key;visibleReceiptKeys.add(key);block.open=receiptExpansion.get(key)===true;
   }
   let value=r.status;if(label.includes('不升级通过')&&value==='passed')value='unverified';if(label.includes('不认证导航'))value='unverified';
   head.textContent=label+' · '+status(value)[0]+' · '+r.filename;block.append(head);
   const note=document.createElement('p');note.className='caption';const c=r.counts||{};
   note.textContent=(c.total?`${c.total} 项：${c.passed} 通过 / ${c.failed} 失败 / ${c.unverified} 未验证。 `:'')+'SHA256 '+(r.sha256||'未核对')+(r.validation_errors?.length?'；显示完整性拒绝：'+r.validation_errors.join('；'):'')+(label.includes('不升级通过')?'；缺少事前协议的先导记录不能作为正式通过。':'')+(r.display_note?'；'+r.display_note:'');block.append(note);
   const raw=document.createElement('pre');raw.textContent=JSON.stringify(r.raw,null,2);block.append(raw);panel.append(block);
  }
  for(const key of receiptExpansion.keys())if(!visibleReceiptKeys.has(key))receiptExpansion.delete(key);
  const source=document.createElement('p');source.className='caption';source.textContent='导航反馈：实际 SLAM / IMU / 点云与实际 SCAN。Gazebo 真值仅供离线运动验收；Actor 仍有 232 维特权输入，15 维为已知命令与上一动作。完整三层、自动原地图配准、动态障碍及真机仅以各自正式收据为准。';panel.append(source);
  const raw=$('raw');let original={};try{original=JSON.parse(raw.textContent);}catch(e){}raw.textContent=JSON.stringify({...original,closed_loop_result:result},null,2);
 };
})();
</script>
'''


MISSION46_PAGE_SCRIPT = r'''
<script>
(() => {
 const previousShow=show;
 const baseTitle=$('mainTitle').textContent,baseSubtitle=$('mainSubtitle').textContent;
 let previousWasMission=false;
 const make=(tag,text,cls)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;return e;};
 const panel=(id,title)=>{let p=$(id);if(!p){p=make('section',undefined,'panel tests');p.id=id;$('legacyPanel').before(p);}p.hidden=false;p.replaceChildren();p.append(make('h2',title));return p;};
 const nativeNote=data=>{
  const t=data.telemetry||{};
  if(t.display_mode==='bounded_recent_window'){
   $('pathCaption').textContent+=` · 原始 native 曲线仅显示有界最近窗口；较早历史未展开（页面跳过 ${t.display_skipped_bytes||0} 字节，原日志保留）。`;
   $('runInfo').textContent+=` · 最新样本：${t.latest_source||'原日志'}；曲线末帧 ${fmt(t.archive_latest_t,2)} s。`;
  }
 };
 const missionCaption=data=>{
  const m=data.mission46;
  if(m&&Object.keys(m).length)$('navigationCaption').textContent+=` · Teacher46 ${m.stage}，已记录到达 ${m.recorded_arrivals}/46（18+14+14）；${m.independent_status==='failed'?'本轮独立结果 FAILED，未验证项保留':'整体独立验收未完成'}。`;
 };
 const previousNavigationPlot=navigationPlot;
 navigationPlot=()=>{previousNavigationPlot();if(current)missionCaption(current);};
 show=data=>{
  const hasMission=!!Object.keys(data.mission46||{}).length;
  if(previousWasMission&&!hasMission){$('mainTitle').textContent=baseTitle;$('mainSubtitle').textContent=baseSubtitle;}
  previousWasMission=hasMission;
  previousShow(data);nativeNote(data);
  const pipeline=data.pipeline_v19||{};
  if(Object.keys(pipeline).length){
   const p=panel('pipelineV19Panel','本轮 V19 输入流水线'),s=pipeline.summary||{},c=pipeline.contract||{};
   p.append(make('p',`模式 ${s.mode||c.mode||'等待记录'} · 图像复制选项 ${s.image_copy_opt??c.image_copy_opt??'—'} · accepted / delivered / committed：${s.accepted??'—'} / ${s.delivered??'—'} / ${s.committed??'—'} · pending ${s.pending??'—'}`));
   p.append(make('p',`${s.normal_completed===true?'正常排空记录已写入':s.failure?'流水线失败：'+s.failure:'等待最终排空记录'}。${pipeline.display_note}`,'caption'));
  }else if($('pipelineV19Panel'))$('pipelineV19Panel').hidden=true;
  const corridor=data.corridor_v20||{};
  if(Object.keys(corridor).length){
   const cp=panel('corridorV20Panel','V20 走廊 · 只读 shadow 诊断'),r=corridor.raw||{};
   cp.append(make('p',`最后完整记录：${corridor.record_valid_for_selected_run?corridor.status:'未验证来源'} · ${r.reason||corridor.display_error||'等待记录'} · sim ${fmt((r.recorded_clock_ns||0)/1e9,2)} s`));
   cp.append(make('p',corridor.display_note,'caption'));
   const detail=make('details');detail.append(make('summary','走廊原始诊断（不执行）'),make('pre',JSON.stringify(r,null,2)));cp.append(detail);
  }else if($('corridorV20Panel'))$('corridorV20Panel').hidden=true;
  const m=data.mission46||{};
  if(!Object.keys(m).length){if($('mission46Panel'))$('mission46Panel').hidden=true;return;}
  const p=panel('mission46Panel','本轮 Teacher46 · 原始多层任务');
  p.append(make('p',`阶段：${m.stage} · 有效区域回执 ${m.recorded_arrivals}/46 · ${m.phases.map(x=>`${x.label} ${x.recorded_arrivals}/${x.expected}`).join(' · ')} · 整阶段完成 ${m.completed_stage_arrivals}/46`));
  const failure=m.independent_status==='failed'&&m.selected_independent_failure?.failure_evidence_verified===true?m.selected_independent_failure:null;
  p.append(make('p',failure?'本轮独立结果 FAILED；未执行阶段和完整控制回放仍按原收据保持未验证。':m.functional_sequence_completed?'功能状态 completed 已记录；完整独立验收仍未完成。':'功能序列尚未完成；不以历史 V18 的通过替代。','caption'));
  const c=m.control||{};
  p.append(make('p',m.control_valid_for_selected_run?`最后协调记录：${c.hold?'速度暂停/停车保持':'允许当前路线'} · ${c.reason||''} · sim ${fmt((c.sim_ns||0)/1e9,2)} s`:'尚无绑定本轮的有效协调记录。'));
  const table=make('table'),head=make('tr');for(const x of ['运行时证据（不认证整体）','本轮来源','依据'])head.append(make('th',x));table.append(head);
  for(const [key,label]of [['initialization','初始化'],['rgb_save','当前 RGB 地图保存'],['dynamic','动态障碍停车恢复'],['parking','最终首次 5 秒停车']]){
   const r=m.runtime_receipts[key]||{},tr=make('tr');tr.append(make('td',label));tr.append(make('td',r.source_binding_valid?'来源哈希已核对，运行时报告通过':'尚无核对后的运行时证据'));
   tr.append(make('td',r.filename?(r.validation_errors||[]).join('; ')||`${r.filename}；未独立重算数值`:'等待本轮收据'));table.append(tr);
  }
  p.append(table);p.append(make('p',m.display_note,'caption'));
  for(const r of m.final_evaluations||[]){const d=make('details'),s=make('summary',`${r.filename} · ${r.failure_evidence_verified?'已核对本轮独立 FAILED':'未认证的最终验收记录'}`);d.append(s,make('p',`${r.reason} · final receipt SHA256 ${r.sha256||'未验证'}`,'caption'),make('pre',JSON.stringify(r.raw,null,2)));p.append(d);}
  if(!m.state_valid_for_selected_run)p.append(make('p',(m.state_validation_errors||[]).join('; '),'caption'));
  $('mainTitle').textContent='Teacher46 · 实际 SLAM / SCAN 多层任务';
  $('mainSubtitle').textContent='18 探索 + 14 返航 + 14 导航 · CPU Teacher · 本轮功能记录与独立验收分开';
  $('acceptanceStatus').textContent=failure?'46 任务独立结果：FAILED':'46 任务独立验收未完成';$('acceptanceStatus').className=failure?'badge fail':'badge';
  showLevels({});$('currentValidation').hidden=true;
  $('legacyTitle').textContent='本轮 46 任务整体独立验收';
  $('pidCorrectionNote').hidden=false;$('pidCorrectionNote').textContent=m.display_note;
  showTests(failure?failure.raw:{checks:{mission46_independent_acceptance_pending:{status:'unverified',passed:null,reason:'最终独立验收尚未完成；阶段 completed、运行时收据与旧 V18 结果均不升级整体通过。'}}});
  if($('closedLoopReceipts'))$('closedLoopReceipts').hidden=true;
  missionCaption(data);
  let raw={};try{raw=JSON.parse($('raw').textContent);}catch(e){}
  $('raw').textContent=JSON.stringify({...raw,pipeline_v19:pipeline,mission46:m},null,2);
 };
})();
</script>
'''


PREFIX9_PAGE_SCRIPT = r'''
<script>
(() => {
 const previousShow=show;
 const make=(tag,text,cls)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;return e;};
 show=data=>{
  previousShow(data);
  const r=data.prefix9_result||{};let p=$('prefix9Panel');
  if(!Object.keys(r).length){if(p)p.hidden=true;return;}
  if(!p){p=make('section',undefined,'panel tests');p.id='prefix9Panel';$('legacyPanel').before(p);}
  p.hidden=false;p.replaceChildren();p.append(make('h2','本轮 prefix9 · 独立有限验收'));
  const passed=r.valid_for_selected_run===true&&r.limited_prefix9_pass===true;
  p.append(make('p',passed?'原始 9 区域 + 首次固定 5 秒停车：有限通过':r.valid_for_selected_run?'有限验收未通过':'独立有限验收尚未核对',passed?'badge pass':'badge'));
  p.append(make('p',`原始区域 dwell：${r.original_regions_verified??'—'} / 9 · 首次固定 5 秒停车：${r.first5s_parking_verified?'已独立通过':'未验证'}`));
  const metrics=r.parking_metrics||{},xy=metrics.slam_xy_drift_max_m;
  if(typeof xy==='number'&&Number.isFinite(xy))p.append(make('p',`SLAM XY 最大漂移 ${xy.toFixed(5)} m · 原限值 0.05000 m；yaw 最大漂移 ${fmt(metrics.slam_yaw_drift_max_rad,5)} rad。`));
  p.append(make('p',r.display_note,'caption'));
  if(r.report_sha256)p.append(make('p',`所选 run 来源已核对 ${r.verified_source_count} 项 · 独立报告 SHA256 ${r.report_sha256}`,'caption'));
  if((r.validation_errors||[]).length)p.append(make('p',r.validation_errors.join('; '),'caption'));
  const detail=make('details');detail.append(make('summary','有限验收来源与范围'),make('pre',JSON.stringify(r,null,2)));p.append(detail);
  let raw={};try{raw=JSON.parse($('raw').textContent);}catch(e){}
  $('raw').textContent=JSON.stringify({...raw,prefix9_result:r},null,2);
  // This panel never modifies full46, clock-hold ledger, or global PASS badges.
 };
})();
</script>
'''


def dashboard_page():
    page = (ROOT / 'web/index.html').read_text(encoding='utf-8')
    return page.replace('</body>', CLOSED_LOOP_PAGE_SCRIPT + MISSION46_PAGE_SCRIPT + PREFIX9_PAGE_SCRIPT + '\n</body>').encode('utf-8')


def safe_json(value):
    """Nonfinite diagnostic values stay missing instead of breaking the UI."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    return value


class Telemetry:
    CHUNK_BYTES = 8 * 1024 * 1024

    def __init__(self):
        self.lock = threading.Lock()
        self.entries = {}

    def read(self, path, latest_window=False):
        with self.lock:
            try:
                if not recording_read_allowed(path):
                    raise OSError('Recording telemetry path escapes the allowed roots')
                stat = path.stat()
            except OSError:
                return {'rows': [], 'trajectory': [], 'latest': None, 'records': 0,
                        'invalid_records': 0, 'age_s': None, 'loading': False}
            key = str(path)
            entry = self.entries.get(key)
            identity = (stat.st_dev, stat.st_ino)
            if entry is None or entry['identity'] != identity or stat.st_size < entry['offset']:
                entry = {'identity': identity, 'offset': 0, 'partial': b'',
                         'rows': deque(maxlen=6000), 'trajectory': [], 'records': 0,
                         'invalid_records': 0, 'latest': None, 'display_skipped_bytes': 0}
                self.entries[key] = entry
                # Bound memory when the user browses many archived runs.
                while len(self.entries) > 6:
                    self.entries.pop(next(iter(self.entries)))
            try:
                read_budget = self.CHUNK_BYTES
                with path.open('rb') as stream:
                    if latest_window and stat.st_size - entry['offset'] > self.CHUNK_BYTES:
                        # Display only a bounded recent window. Never scan the
                        # complete native archive just to catch a live plot up.
                        target = stat.st_size - self.CHUNK_BYTES
                        entry['display_skipped_bytes'] += target - entry['offset']
                        entry.update(offset=target, partial=b'', latest=None, trajectory=[])
                        entry['rows'].clear()
                        stream.seek(target)
                        # A seek may begin inside a JSON line; do not report
                        # that deliberately omitted fragment as invalid data.
                        fragment = stream.readline(self.CHUNK_BYTES)
                        read_budget -= len(fragment)
                        entry['display_skipped_bytes'] += len(fragment)
                        entry['offset'] = stream.tell()
                    stream.seek(entry['offset'])
                    chunk = stream.read(read_budget)
                    entry['offset'] = stream.tell()
            except OSError:
                chunk = b''
            lines = (entry['partial'] + chunk).split(b'\n')
            entry['partial'] = lines.pop()
            # A partially written line is completed on the next poll.
            if len(entry['partial']) > 8 * 1024 * 1024:
                entry['partial'] = b''
                entry['invalid_records'] += 1
            for line in lines:
                if not line.strip():
                    continue
                try:
                    row = normalize(json.loads(line))
                except (ValueError, UnicodeDecodeError):
                    row = None
                if row is None:
                    entry['invalid_records'] += 1
                    continue
                entry['records'] += 1
                entry['latest'] = row
                entry['rows'].append(row)
                position = row['position']
                if position is not None:
                    point = [position[0], position[1], row['t']]
                    path_rows = entry['trajectory']
                    if not path_rows or math.hypot(point[0] - path_rows[-1][0], point[1] - path_rows[-1][1]) >= .015:
                        path_rows.append(point)
                    if len(path_rows) > 4000:
                        entry['trajectory'] = path_rows[::2]
            rows = list(entry['rows'])
            stride = max(1, math.ceil(len(rows) / 2000))
            displayed = rows[::stride]
            if rows and (not displayed or displayed[-1] is not rows[-1]):
                displayed.append(rows[-1])
            return {'rows': displayed, 'trajectory': entry['trajectory'], 'latest': entry['latest'],
                    'records': entry['records'], 'invalid_records': entry['invalid_records'],
                'age_s': None if entry['offset'] < stat.st_size else max(0., time.time() - stat.st_mtime),
                    'loading': entry['offset'] < stat.st_size,
                    'display_mode': 'bounded_recent_window' if latest_window else 'archive_backfill',
                    'display_skipped_bytes': entry['display_skipped_bytes'],
                    'display_chunk_limit_bytes': self.CHUNK_BYTES}


class Dashboard:
    def __init__(self, runs):
        self.runs = runs.resolve()
        self.telemetry = Telemetry()
        self.plan_cache = {}

    def navigation_view(self, directory):
        """Only measured SLAM and an archived actual SCAN payload are drawn."""
        poses = self.telemetry.read(directory / 'navigation_slam_poses.jsonl')
        request = read_json(directory / 'navigation_request.json')
        result = {'poses': poses, 'request': request,
                  'state': read_json(directory / 'navigation_status.json'),
                  'scan': {}, 'source': 'Actual sensor SLAM in camera_init; no Gazebo truth input'}
        files = sorted((directory / 'navigation_trajectories').glob('*_trajectory_*.json'))
        if not files:
            return result
        record = files[-1]
        try:
            key = (str(record), record.stat().st_mtime_ns)
            if key not in self.plan_cache:
                import numpy as np
                data = read_json(record)
                archive = self.inside(record.parent / data['array_file'])
                if archive is None or hashlib.sha256(archive.read_bytes()).hexdigest() != data['array_sha256']:
                    raise ValueError('Actual SCAN array hash mismatch')
                with np.load(archive, allow_pickle=False) as saved:
                    samples = saved['samples']
                    if samples.ndim != 2 or samples.shape[1] != 3 or not np.isfinite(samples).all():
                        raise ValueError('Actual SCAN samples malformed')
                    stride = max(1, math.ceil(len(samples) / 2000))
                    points = samples[::stride].tolist()
                self.plan_cache[key] = {'trajectory_id': data['trajectory_id'],
                    'waypoint_index': data['waypoint_index'], 'points': points,
                    'array_sha256': data['array_sha256'], 'source': data['source'],
                    'file': record.name, 'hash_verified': True}
                while len(self.plan_cache) > 6:
                    self.plan_cache.pop(next(iter(self.plan_cache)))
            result['scan'] = self.plan_cache[key]
        except (OSError, ValueError, KeyError, ImportError):
            result['scan'] = {'error': 'Actual SCAN payload not available or hash not verified'}
        return result

    def inside(self, path):
        return recording_path(path, self.runs)

    def directories(self):
        try:
            children = list(self.runs.iterdir())
        except OSError:
            return []
        result = []
        for child in children:
            resolved = self.inside(child)
            if child.name != 'latest' and resolved is not None and resolved.is_dir():
                try:
                    modified = child.stat().st_mtime
                    summary = read_json(child / 'summary.json')
                    state = read_json(child / 'state.json')
                    closed_loop = closed_loop_receipt_view(child)
                    result.append({'id': child.name, 'modified_at': modified,
                                   'status': closed_loop.get('status') if closed_loop else summary.get('status', state.get('state', state.get('status'))),
                                   'tests': closed_loop['counts']['total'] if closed_loop else len(summary.get('tests', [])) if isinstance(summary.get('tests'), (dict, list)) else 0})
                except OSError:
                    continue
        return sorted(result, key=lambda item: item['modified_at'], reverse=True)

    def resolve(self, run):
        if run == 'latest':
            latest = self.inside(self.runs / 'latest')
            if latest is not None and latest.is_dir():
                return latest
            directories = self.directories()
            return self.inside(self.runs / directories[0]['id']) if directories else None
        if not run or run in {'.', '..'} or '/' in run or '\\' in run:
            return None
        path = self.inside(self.runs / run)
        return path if path is not None and path.is_dir() else None

    def frame(self, directory):
        candidates = [p for p in (directory / 'frame.jpg', directory / 'frame.png')
                      if self.inside(p) is not None and p.is_file()]
        return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None

    def snapshot(self, run):
        directory = self.resolve(run)
        if directory is None:
            return {'available': False, 'run_id': None, 'state': {}, 'summary': {},
                    'telemetry': {'rows': [], 'trajectory': [], 'latest': None}, 'frame': {'available': False},
                    'acceptance': read_json(self.runs / 'acceptance.json')}
        state_path = directory / 'state.json'
        state = read_json(state_path)
        frame = self.frame(directory)
        vehicle_frame = self.frame(directory / 'vehicle_rgb')
        try:
            state_age = max(0., time.time() - state_path.stat().st_mtime)
        except OSError:
            state_age = None
        pipeline = pipeline_view(directory)
        native = self.telemetry.read(directory / 'telemetry.jsonl', latest_window=bool(pipeline))
        latest_state = normalize(state)
        native['archive_latest_t'] = (native.get('latest') or {}).get('t')
        if (pipeline and latest_state is not None
                and (native.get('latest') is None or latest_state['t'] >= native['latest']['t'])):
            native.update(latest=latest_state, age_s=state_age, latest_source='atomic state.json exact native sample')
        else:
            native['latest_source'] = 'original telemetry.jsonl exact native sample'
        original_functional = read_json(directory / 'summary_functional.json')
        reproduced = read_json(directory / 'summary_functional.reproduced.json')
        correction = read_json(directory / 'provenance_correction.json')
        corrected = bool(reproduced and correction.get('status') == 'reproduced_identical'
            and correction.get('all_checks_identical') is True and correction.get('all_metrics_identical') is True
            and original_functional.get('checks') == reproduced.get('checks')
            and original_functional.get('metrics') == reproduced.get('metrics'))
        ramp_original = read_json(directory / 'summary_full_ramp.json')
        stop_addendum = read_json(directory / 'full_ramp_stop_status_addendum.json')
        ramp_display = ramp_original
        if ramp_original and stop_addendum and stop_addendum.get('original_summary_sha256') == hashlib.sha256((directory / 'summary_full_ramp.json').read_bytes()).hexdigest():
            ramp_display = {**ramp_original, 'checks': {**ramp_original.get('checks', {}),
                'teacher_stop_on_destination_landing': stop_addendum['teacher_stop_on_destination_landing']},
                'display_note': 'Parking status uses separately archived clarification; original full-ramp receipt retained'}
        pid_display, pid_original, pid_correction = pid_receipt_view(directory)
        return {'available': True, 'run_id': directory.name, 'selected': run,
                'state': state, 'state_age_s': state_age,
                'summary': read_json(directory / 'summary.json'),
                'step_functional': reproduced if corrected else original_functional,
                'step_functional_original': original_functional,
                'step_provenance': correction,
                'step_functional_source': 'summary_functional.reproduced.json' if corrected else 'summary_functional.json',
                'step_safety': read_json(directory / 'supplemental_native_safety.json'),
                'full_ramp': ramp_display,
                'full_ramp_original': ramp_original,
                'full_ramp_stop_addendum': stop_addendum,
                'navigation_summary': read_json(directory / 'summary_navigation_independent.json'),
                'closed_loop_result': closed_loop_receipt_view(directory),
                'pipeline_v19': pipeline,
                'corridor_v20': corridor_shadow_view(directory),
                'prefix9_result': prefix9_view(directory),
                'mission46': mission46_view(directory),
                'pid_navigation': pid_display,
                'pid_navigation_original': pid_original,
                'pid_navigation_correction': pid_correction,
                'pid_scope': read_json(directory / 'navigation_scope.json') if (directory / 'pid_profile_input.json').is_file() else {},
                'targeted_ttl': read_json(directory / 'summary_ttl_dropout_independent.json'),
                'dynamic_obstacle': read_json(directory / 'summary_dynamic_obstacle_independent.json'),
                'runtime_processes': read_json(directory / 'runtime_manifest.json').get('owned_processes', []),
                'navigation': self.navigation_view(directory),
                'replay': {'available': (directory / 'frame_replay.mp4').is_file() or (directory / 'frame_replay.gif').is_file(),
                           'mp4': (directory / 'frame_replay.mp4').is_file(),
                           'gif': (directory / 'frame_replay.gif').is_file(),
                           'manifest': read_json(directory / 'frame_replay_manifest.json') or read_json(directory / 'step_retest_evidence_manifest.json'),
                           'scope': 'Archived actual Gazebo RGB in sensor timestamp order; no interpolated or synthesized frames'},
                'step_plot': (directory / 'step_functional.png').is_file(),
                'acceptance': read_json(self.runs / 'acceptance.json'),
                'current_validation': read_json(self.runs.parent / 'current_status.json'),
                'telemetry': native,
                'frame': {'available': frame is not None,
                          'age_s': max(0., time.time() - frame.stat().st_mtime) if frame else None,
                          'filename': frame.name if frame else None},
                'camera_info': {kind: read_json(directory / 'camera_info' / (kind + '.json')) for kind in ('overview','vehicle')},
                'vehicle_frame': {'available': vehicle_frame is not None,
                                  'age_s': max(0., time.time() - vehicle_frame.stat().st_mtime) if vehicle_frame else None,
                                  'filename': vehicle_frame.name if vehicle_frame else None},
                'diagnostic_source': 'Gazebo simulation truth for motion diagnostics; not a navigation input',
                'read_only': True}


def handler_for(dashboard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            # Polling should not bury simulation diagnostics in HTTP logs.
            if len(args) < 2 or str(args[1]) not in {'200', '304'}:
                super().log_message(format, *args)

        def send(self, data, content_type='application/json; charset=utf-8', status=200, headers=None):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def json(self, value, status=200):
            data = json.dumps(safe_json(value), ensure_ascii=False, allow_nan=False, default=str).encode('utf-8')
            self.send(data, status=status)

        def media(self, path, content_type):
            if path is None or dashboard.inside(path) is None or not path.is_file():
                self.json({'error': 'No recorded media'}, 404)
                return
            try:
                size = path.stat().st_size
                start, end, status = 0, size - 1, 200
                headers = {'Accept-Ranges': 'bytes'}
                value = self.headers.get('Range')
                if value:
                    # One byte range is enough for read-only HTML video seeking.
                    import re
                    match = re.fullmatch(r'bytes=(\d*)-(\d*)', value.strip())
                    if not match or not any(match.groups()):
                        self.send(b'', content_type, 416, {'Content-Range': f'bytes */{size}'})
                        return
                    first, last = match.groups()
                    if first:
                        start = int(first); end = min(size - 1, int(last)) if last else size - 1
                    else:
                        start = max(0, size - int(last))
                    if start > end or start >= size:
                        self.send(b'', content_type, 416, {'Content-Range': f'bytes */{size}'})
                        return
                    status = 206; headers['Content-Range'] = f'bytes {start}-{end}/{size}'
                with path.open('rb') as stream:
                    stream.seek(start); payload = stream.read(end - start + 1)
                self.send(payload, content_type, status, headers)
            except OSError:
                self.json({'error': 'Recorded media changed while reading'}, 404)

        def do_GET(self):
            parsed = urlsplit(self.path)
            run = parse_qs(parsed.query).get('run', ['latest'])[0]
            if parsed.path in {'/', '/index.html'}:
                try:
                    self.send(dashboard_page(), 'text/html; charset=utf-8')
                except OSError:
                    self.json({'error': 'Dashboard page is unavailable'}, 404)
            elif parsed.path == '/api/runs':
                self.json({'runs': dashboard.directories(), 'read_only': True})
            elif parsed.path == '/api/run':
                self.json(dashboard.snapshot(run))
            elif parsed.path == '/api/acceptance':
                self.json(read_json(dashboard.runs / 'acceptance.json'))
            elif parsed.path == '/api/frame':
                directory = dashboard.resolve(run)
                camera = parse_qs(parsed.query).get('camera', ['overview'])[0]
                if camera not in ('overview','vehicle'):
                    self.json({'error': 'Unknown camera'}, 400)
                    return
                frame = dashboard.frame(directory / 'vehicle_rgb' if camera == 'vehicle' else directory) if directory else None
                if frame is None or dashboard.inside(frame) is None:
                    self.json({'error': 'No recorded Gazebo frame'}, 404)
                    return
                try:
                    self.send(frame.read_bytes(), 'image/png' if frame.suffix == '.png' else 'image/jpeg')
                except OSError:
                    self.json({'error': 'Frame changed while reading'}, 404)
            elif parsed.path == '/api/replay':
                directory = dashboard.resolve(run)
                kind = parse_qs(parsed.query).get('kind', ['mp4'])[0]
                if kind not in {'mp4', 'gif'}:
                    self.json({'error': 'Unsupported recorded media type'}, 400)
                    return
                self.media(directory / ('frame_replay.' + kind) if directory else None, 'video/mp4' if kind == 'mp4' else 'image/gif')
            elif parsed.path == '/api/step_plot':
                directory = dashboard.resolve(run)
                self.media(directory / 'step_functional.png' if directory else None, 'image/png')
            else:
                self.json({'error': 'Not found'}, 404)

        def do_POST(self):
            self.json({'error': 'This dashboard is read-only'}, 405)

        do_PUT = do_POST
        do_DELETE = do_POST
        do_PATCH = do_POST
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8768)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--runs', type=Path, default=ROOT / 'runs')
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), handler_for(Dashboard(args.runs)))
    print(f'Teacher read-only dashboard: http://{args.host}:{args.port}/', flush=True)
    print(f'Recordings: {args.runs.resolve()}', flush=True)
    try:
        server.serve_forever(poll_interval=.3)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
