"""R73-only recording projection; no live control/source/ACK values are changed."""
from pathlib import Path
import math

# These append-only observation archives are not live atomic control files.
OPTIONAL = frozenset((
    'navigation_imu_history.jsonl', 'navigation_feedback_history.jsonl',
    'navigation_slam_poses.jsonl', 'navigation_cloud_history.jsonl',
    'navigation_cloud_callbacks.jsonl', 'navigation_cloud_xyz.jsonl',
    'navigation_heading_projection.jsonl', 'navigation_status.jsonl',
    'mission46_status_history.jsonl',
    'navigation_guard_continuity_history.jsonl', 'corridor_inputs.jsonl',
    'corridor_input_rejections.jsonl', 'corridor_certificates.jsonl',
    'navigation_cascade_paths.jsonl', 'cascade_path_history.jsonl'))


def small(value):
    if value is None or type(value) in (bool, int):
        return value
    if type(value) is float:
        return value if math.isfinite(value) else repr(value)
    if type(value) is str:
        return value[:256]
    if type(value) in (list, tuple) and len(value) <= 4:
        return [small(x) if type(x) not in (dict, list, tuple) else None for x in value]
    return None


def fields(value, names):
    return {k: small(value.get(k)) for k in names} if isinstance(value, dict) else {}


def control(row):
    out = fields(row, ('sequence', 'compute_ros_clock_ns', 'compute_monotonic_wall',
        'source_pose_stamp_ns', 'paired_imu_stamp_ns', 'request_id', 'trajectory_id',
        'mode', 'checked_target', 'goal', 'control_pose', 'desired_body_command',
        'prepared_after_slew_command', 'command_after_slew', 'state', 'stopped'))
    out.update(schema='r73_control_projection/v1', formal_audit_inherited=False)
    out['feedback'] = fields(row.get('feedback'), ('stamp_ns', 'received_wall_ns',
        'position_world_xyz', 'quaternion_wxyz', 'origin_velocity_body'))
    out['imu'] = fields(row.get('imu'), ('stamp_ns', 'received_wall_ns', 'angular_velocity_body'))
    out['core'] = fields(row.get('cascade'), ('compute_clock_ns', 'compute_wall_ns',
        'kind', 'controller_updated', 'math_step_executed', 'mode', 'reason',
        'protection_active', 'failure_latched', 'reference_yaw_rad', 'error_yaw_rad',
        'reference_COM_velocity_body', 'measured_COM_velocity_body', 'command_body'))
    # Eight existing diagnostics only; no geometry/reference copy or control math.
    cascade = row.get('cascade')
    cascade = cascade if isinstance(cascade, dict) else {}
    spatial = cascade.get('spatial_reference')
    spatial = spatial if isinstance(spatial, dict) else {}
    budget = cascade.get('curvature_speed_supervisor')
    budget = budget if isinstance(budget, dict) else {}
    speed = dict(remaining_horizontal_arc_m=cascade.get('remaining_horizontal_arc_m'),
        local_grade_dz_ds=cascade.get('local_grade_dz_ds'),
        curvature_per_m=spatial.get('curvature_per_m'),
        requested_speed_mps=budget.get('requested_speed_mps'),
        limited_speed_mps=budget.get('limited_speed_mps'),
        reference_budget_feasible=budget.get('reference_budget_feasible'),
        diagnostic_execution_continued=budget.get('diagnostic_execution_continued'),
        reason=budget.get('reason'))
    out['speed_budget'] = {key: small(value) if type(value) in
        (type(None), bool, int, float, str) else None for key, value in speed.items()}
    out['heading'] = fields(row.get('heading_gate_reference'),
        ('phase', 'locked_heading', 'controlled_braking', 'settle_control_stage'))
    return out


def selected_ack(row):
    ack = row.get('ack')
    if not isinstance(ack, dict):
        return dict(schema='r73_selected_actual_ack/v1', control_sequence=row.get('sequence'),
            ack_present=False, formal_audit_inherited=False)
    out = fields(ack, ('sequence', 'stamp_ns', 'received_wall_ns', 'ack_native_clock_ns',
        'requested_command_body', 'applied_command_body', 'source_scope_sha256'))
    out.update(schema='r73_selected_actual_ack/v1', control_sequence=row.get('sequence'),
        ack_present=True, formal_audit_inherited=False)
    out['actor'] = fields(ack.get('actual_actor_input'), ('frame_serial',
        'inference_sequence', 'input_clock_ns', 'actual_observation_command_body',
        'action_finite', 'hidden_finite'))
    return out


def publication(row):
    out = fields(row, ('sequence', 'associated_pid_sequence', 'publish_ros_clock_ns',
        'entry_monotonic_wall_ns', 'publish_started_monotonic_wall_ns', 'monotonic_wall_ns',
        'trigger', 'stopped_argument', 'command_before', 'command_after',
        'state_before', 'state_after', 'heading_phase', 'heading_reference'))
    out.update(schema='r73_actual_publication_projection/v1', formal_audit_inherited=False)
    names = ('obstacle_hold', 'tilt_hold', 'bridge_state', 'evidence_error',
        'clock_hold_holding', 'clock_hold_guard_required', 'pid_guard_needs_evaluation')
    out['flags_before'] = fields(row.get('protection_flags_before'), names)
    out['flags_after'] = fields(row.get('protection_flags_after'), names)
    return out


def guard(row):
    out = fields(row, ('sequence', 'pid_sequence', 'compute_ros_clock_ns',
        'compute_monotonic_wall', 'control_pose_stamp_ns', 'control_pose',
        'cloud_header_stamp_ns', 'cloud_received_monotonic_wall', 'checked_target',
        'command_after_control', 'zero_requested_after_control', 'state_after_control',
        'obstacle_hold_before', 'obstacle_hold_after', 'evidence_queue_error'))
    out.update(schema='r73_guard_projection/v1', formal_audit_inherited=False)
    out['result'] = fields(row.get('union_result'), ('blocked', 'clearance_m', 'point_count'))
    return out


def make_writer(base, run, *, full_evidence_after_endpoint=False):
    run = Path(run)
    class MinimalWriter(base):
        def append(self, path, data):
            path = Path(path)
            if path.parent != run:
                return super().append(path, data)
            name = path.name
            # Select before original producer serialization or archive deepcopy.
            if name == 'navigation_pid_history.jsonl':
                super().append(run/'r73_control.jsonl', control(data))
                return super().append(run/'r73_selected_ack.jsonl', selected_ack(data))
            if name == 'navigation_guard_history.jsonl':
                if full_evidence_after_endpoint is True:
                    return super().append(path, data)
                return super().append(run/'r73_control.jsonl', guard(data))
            if name == 'navigation_command_publications.jsonl':
                return super().append(run/'r73_publications.jsonl', publication(data))
            if name in OPTIONAL:
                return None
            # Preserve unknown archives and every original atomic/live consumer.
            return super().append(path, data)
    return MinimalWriter()
