"""Explicit opt-in for the isolated R73 Gazebo diagnostic comparison only."""
import os


def enabled(profile):
    mode = profile.get('r73_minimal_diagnostic_baseline')
    if mode is None:
        return False
    if (type(mode) is dict and set(mode) == {'enabled'}
            and type(mode['enabled']) is bool and mode['enabled'] is False):
        return False
    if (type(mode) is not dict or set(mode) != {'enabled', 'simulation_only'}
            or any(type(mode[key]) is not bool for key in mode)):
        raise ValueError('Typed R73 simulation baseline opt-in required')
    if mode != {'enabled': True, 'simulation_only': True}:
        raise ValueError('Invalid R73 simulation baseline opt-in')
    if (os.environ.get('GO2_R73_MINIMAL_SIMULATION') != '1'
            or profile.get('diagnostic_observe_only', {}).get('enabled') is not True
            or profile.get('r85_formal_regression') != {'enabled': False}
            or profile.get('navigation_ground_truth_used') is not False
            or profile.get('actor_kind') != 'student'):
        raise ValueError('R73 baseline requires isolated simulation diagnostic entry')
    return True


# Local upper-layer adapters; the original control_core class is never modified.
def _finite_number(value):
    import math
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except (OverflowError, ValueError):
        return False


def wall_stale(node, field, now, receipt, limit, original_stale, *, future_tolerance=0.):
    if not enabled(getattr(node, 'profile', {})):
        return original_stale
    if not all(_finite_number(v) for v in (now, receipt, limit)) or limit <= 0:
        return True
    age = now-receipt
    if not _finite_number(age) or age < -future_tolerance:
        return True
    if age >= limit and original_stale:
        observations = getattr(node, '_r73_upper_wall_last_observations', None)
        if observations is None:
            observations = node._r73_upper_wall_last_observations = {}
        observations[field] = dict(age_s=age, original_limit_s=limit,
            checked_monotonic_wall=now, original_wall_limit_observed=True,
            original_timestamps_preserved=True, formal_acceptance_pass=False)
    return False


def wall_stale_ns(node, field, now_ns, receipt_ns, limit_ns, original_stale):
    if not enabled(getattr(node, 'profile', {})):
        return original_stale
    if (any(type(v) is not int or not _finite_number(v) for v in (now_ns, receipt_ns, limit_ns))
            or now_ns < 0 or receipt_ns < 0 or limit_ns <= 0 or now_ns < receipt_ns):
        return True
    return wall_stale(node, field, now_ns/1e9, receipt_ns/1e9, limit_ns/1e9, original_stale)


def raw_imu_fresh(node, now, ros_now, original_fresh):
    if not enabled(getattr(node, 'profile', {})):
        return original_fresh
    raw = node.raw_imu
    if (not all(_finite_number(v) for v in (ros_now, raw.stamp, raw.timeout))
            or raw.timeout <= 0 or not -.10 <= ros_now-raw.stamp <= raw.timeout
            or not _finite_number(raw.tilt)):
        return False
    return not wall_stale(node, 'raw_IMU_latest', now, raw.updated, raw.timeout,
                          not original_fresh)


def pose_cloud_fresh(node, source, now, ros_now, original_fresh):
    if not enabled(getattr(node, 'profile', {})):
        return original_fresh
    if source not in ('pose', 'cloud'):
        raise ValueError('Only original pose/cloud freshness is supported')
    stamp = getattr(node, source+'_stamp')
    receipt = getattr(node, source+'_updated')
    limit = getattr(node, source+'_timeout')
    if (type(stamp) is not int or stamp < 0
            or not all(_finite_number(v) for v in (ros_now, stamp, limit)) or limit <= 0):
        return False
    clock_ns, limit_ns = ros_now*1e9, limit*1e9
    if not _finite_number(clock_ns) or not _finite_number(limit_ns):
        return False
    age_ns = round(clock_ns)-stamp
    if not -50_000_000 <= age_ns <= round(limit_ns):
        return False
    return not wall_stale(node, 'SLAM_pose' if source == 'pose' else 'registered_cloud',
                          now, receipt, limit, not original_fresh)


def control_sources_stale(node, now, ros_now, original_stale):
    if not enabled(getattr(node, 'profile', {})):
        return original_stale
    for source in ('pose', 'cloud'):
        receipt = getattr(node, source+'_updated')
        limit = getattr(node, source+'_timeout')
        original_fresh = (all(_finite_number(v) for v in (now, receipt, limit))
                          and now-receipt <= limit)
        if not pose_cloud_fresh(node, source, now, ros_now, original_fresh):
            return True
    return not raw_imu_fresh(node, now, ros_now, node.raw_imu.fresh(now, ros_now))
