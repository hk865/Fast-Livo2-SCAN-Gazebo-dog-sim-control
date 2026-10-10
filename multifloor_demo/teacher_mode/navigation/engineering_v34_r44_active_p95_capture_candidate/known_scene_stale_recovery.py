"""Fail-closed proposal for explicitly aborting an expired frozen transaction.

This function does not mutate correction state, change its target, or authorize
motion. The sole runtime owner logs ABORTED and starts a distinct frozen attempt
while transition remains true. Original continuation/release gates still apply.
"""
import copy
import math
import numpy as np
from known_scene_matcher import checked_transform, transform_distance

CONTRACT = dict(schema='known_scene_expired_transaction_recovery/v1',
    activation='expired_pending_AND_original_epoch_consistency_rejected',
    original_consistent_fresh_continuation_preserved=True,
    maximum_replacements_per_unreleased_transaction=2,
    original_match_age_ns=3_000_000_000,
    current_bundle_max_age_ns=300_000_000,
    physical_zero_stop_ns=100_000_000,
    original_origin_continuation_m=.08,original_continuation_angle_rad=.05,
    original_release_origin_and_frozen_body_m=.005,
    original_release_angle_rad=.003,
    replacement_is_successful_release=False,
    deadline_reset=False, frontend_reset=False, control_enabled_during_replacement=False)

def recovery_proposal(result, correction, pending_target, pending_body,
                      pending_source_ns, recovery_count, clock_ns,
                      complete_source_ns, complete_received_wall, now_wall,
                      scan_status, run_id, witness_sha256):
    """Return a bounded evidence event, or None; never partially renew TTL."""
    try:
        if (type(clock_ns) is not int or type(complete_source_ns) is not int
            or not 0 <= clock_ns-complete_source_ns < 300_000_000
            or not 0 <= now_wall-complete_received_wall < .3
            or type(recovery_count) is not int or not 0 <= recovery_count < 2):
            return None
        last=correction.last_match_ns;tick=correction.last_tick_ns
        if (type(last) is not int or type(tick) is not int
            or tick-last <= 3_000_000_000 or not 0 <= clock_ns-tick < 300_000_000):
            return None
        if (scan_status.get('schema')!='known_scene_SCAN_epoch_ready/v1'
            or scan_status.get('run_id')!=run_id or scan_status.get('ready') is not False
            or not 0 <= now_wall-float(scan_status['monotonic_wall']) < .3
            or type(scan_status.get('ros_clock_ns')) is not int
            or not 0 <= clock_ns-scan_status['ros_clock_ns'] < 300_000_000):
            return None
        ns=result.get('source_ns')
        if (type(ns) is not int or not ns > last or not ns > pending_source_ns
            or not 0 <= clock_ns-ns <= 3_000_000_000
            or result.get('accepted') is not True or result.get('evidence_verified') is not True
            or result.get('reason')!='confirmed_shadow_match'
            or result.get('parent_generation')!=correction.generation
            or result.get('robot_truth_pose_used') is not False
            or result.get('IMU_world_orientation_used') is not False
            or result.get('frontend_height_used_as_hypothesis') is not False
            or type(result.get('confirmations')) is not int or result['confirmations']<2
            or result.get('best',{}).get('quality_passed') is not True
            or result['best'].get('rejections')!=[]):
            return None
        provenance=result['provenance']
        flags=('point_time_verified','post_lio_association_verified','source_hashes_verified',
               'no_robot_truth_pose','integer_stamp_join','source_membership_verified')
        if (any(provenance.get(k) is not True for k in flags)
            or not isinstance(witness_sha256,str) or len(witness_sha256)!=64
            or provenance.get('witness_sha256')!=witness_sha256
            or provenance.get('stage')!='frozen_post_LIO_single_owner_publication'):
            return None
        applied=checked_transform(correction.applied)
        if not np.array_equal(checked_transform(result['input_C_world_odom']),applied):return None
        if type(pending_source_ns) is not int or pending_source_ns<=0:return None
        old_target=checked_transform(pending_target);old_body=checked_transform(pending_body)
        if not np.array_equal(checked_transform(correction.target),old_target):return None
        candidate=checked_transform(result['C_world_odom']);raw=checked_transform(result['T_odom_body'])
        matched=checked_transform(result['T_world_body'])
        if (result.get('T_odom_body_source_ns')!=ns
            or not np.allclose(candidate@raw,matched,rtol=0,atol=1e-7)):
            return None
        origin,angle=transform_distance(applied,old_target)
        body=float(np.linalg.norm((old_target@old_body)[:3,3]-(applied@old_body)[:3,3]))
        if not all(math.isfinite(v) for v in (origin,angle,body)):return None
        return dict(schema=CONTRACT['schema'],event='expired_transaction_ABORTED_replacement_proposed',
            run_id=run_id,old_attempt_source_ns=pending_source_ns,new_attempt_source_ns=ns,
            old_target_C_world_odom=old_target.tolist(),old_frozen_T_odom_body=old_body.tolist(),
            old_last_match_source_ns=last,old_terminal_bundle_source_ns=tick,
            old_residual=dict(origin_m=origin,frozen_body_m=body,rotation_rad=angle),
            preserved_applied_C_world_odom=applied.tolist(),generation=correction.generation,
            new_target_C_world_odom=candidate.tolist(),new_frozen_T_odom_body=raw.tolist(),
            replacement_number=recovery_count+1,successful_release=False,
            transition_continues=True,control_allowed=False,SCAN_ready=False,
            source_bundle_ns=complete_source_ns,witness_sha256=witness_sha256,
            exact_current_parent_and_input_C=True,deadline_reset=False,frontend_reset=False,
            new_match_provenance=copy.deepcopy(provenance),robot_truth_pose_used=False)
    except (KeyError,TypeError,ValueError,AttributeError,OverflowError):
        return None
