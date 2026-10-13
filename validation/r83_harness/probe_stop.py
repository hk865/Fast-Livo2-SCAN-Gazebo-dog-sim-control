"""Stop at declared finite diagnostic caps after original failure checks."""
from diagnostic_stop_policy import (NATIVE_CAP_REASON, WALL_CAP_REASON, OUTPUT_CAP_REASON,
    NATIVE_CAP_NS, ACQUISITION_WALL_CAP_S, OUTPUT_SOFT_B)
def diagnostic_stop(run,native_ns,elapsed,output_bytes,budget_warning=False):
    import time
    import datetime,json
    from pathlib import Path
    authority=json.loads((Path(__file__).resolve().parent/'AUTHORITY.json').read_text())
    if datetime.datetime.now(datetime.timezone.utc)>=datetime.datetime.fromisoformat(authority['deadline_UTC']):
        return dict(stop=True,reason='R77_night_cleanup_deadline',navigation_PASS=False,diagnostic_only=True,original_route_acceptance=False)
    reason=None
    if budget_warning or output_bytes>=OUTPUT_SOFT_B:reason=OUTPUT_CAP_REASON
    elif elapsed>=ACQUISITION_WALL_CAP_S:reason=WALL_CAP_REASON
    elif native_ns>=NATIVE_CAP_NS:reason=NATIVE_CAP_REASON
    if reason:return dict(stop=True,reason=reason,navigation_PASS=False,diagnostic_only=True,original_route_acceptance=False)
    return None
