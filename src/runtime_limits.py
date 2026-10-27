"""Numeric resource bounds, independent of historical approvals and versions."""
import datetime as dt
HARD_BYTES=170000000000
ROUND_BYTES=10000000000
SOFT_BYTES=HARD_BYTES-10000000000
def configuration(profile):
    values=profile.get('runtime_limits',{})
    expected=dict(maximum_runtime_wall_s=2400,maximum_cleanup_wall_s=300,maximum_retained_bytes=HARD_BYTES,maximum_round_bytes=ROUND_BYTES,additional_reserved_bytes=10000000000)
    if any(type(values.get(k))is not int or values[k]!=v for k,v in expected.items()):raise ValueError('Resource safety limits changed')
    deadline=profile.get('r85_session',{}).get('deadline_utc')
    if deadline is None:raise ValueError('Runtime wall deadline missing')
    parsed=dt.datetime.fromisoformat(deadline)
    if parsed.tzinfo is None:raise ValueError('Runtime deadline needs a timezone')
    return dict(deadline_utc=deadline,maximum_retained_bytes=HARD_BYTES,maximum_round_bytes=ROUND_BYTES,soft_stop_bytes=SOFT_BYTES,finish_reserve_s=300)
def retained_hard_bytes(profile,legacy=HARD_BYTES):
    return HARD_BYTES if profile is None else configuration(profile)['maximum_retained_bytes']
