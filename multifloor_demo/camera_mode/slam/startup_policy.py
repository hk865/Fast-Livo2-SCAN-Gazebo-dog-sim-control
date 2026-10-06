"""Camera-only startup policy, retaining the original stationary IMU maths."""
import math
from _shared import load_shared

StaticImuWindow = load_shared('static_imu', 'scripts/sensor_startup_gate.py').StaticImuWindow


def zero_twist(value):
    return isinstance(value, (list, tuple)) and len(value) == 6 and all(
        type(v) in (int, float) and math.isfinite(v) and abs(v) < 1e-6 for v in value)


class CameraStaticImuWindow(StaticImuWindow):
    def bridge_ready(self, wall):
        b = self.bridge
        return (self.bridge_wall is not None and 0 <= wall-self.bridge_wall <= .30
                and isinstance(b, dict) and b.get('schema') == 2 and b.get('mode') == 'camera'
                and b.get('state') in ('ready', 'hold') and b.get('imu_fresh') is True
                and zero_twist(b.get('requested')) and zero_twist(b.get('safe')))

    def report(self):
        result = super().report()
        result['source'] = 'real camera-rig IMU and camera six-axis zero-command safety; no ground truth'
        result['mode'] = 'camera'
        result['quadruped_prerequisites'] = []
        return result


class ObservationHealth:
    """Increasing original integer sensor stamps; no duplicates refresh age."""
    def __init__(self):
        self.stamps = {}
        self.walls = {}
        self.counts = {}
        self.rejected = {}

    def observe(self, name, stamp_ns, wall):
        if type(stamp_ns) is not int or stamp_ns < 0 or not math.isfinite(wall) \
                or stamp_ns <= self.stamps.get(name, -1):
            self.rejected[name] = self.rejected.get(name, 0) + 1
            return False
        self.stamps[name] = stamp_ns
        self.walls[name] = wall
        self.counts[name] = self.counts.get(name, 0) + 1
        return True

    def fresh(self, name, clock_ns, wall, wall_limit=2., sim_limit=2.):
        return (name in self.stamps and name in self.walls
                and 0 <= wall-self.walls[name] < wall_limit
                and -20_000_000 <= clock_ns-self.stamps[name] < round(sim_limit*1e9))

    def report(self, clock_ns, wall):
        return dict(counts=dict(self.counts), original_stamp_ns=dict(self.stamps),
                    wall_age={k: wall-v for k, v in self.walls.items()},
                    sim_age_ns={k: clock_ns-v for k, v in self.stamps.items()},
                    rejected_nonincreasing_or_invalid=dict(self.rejected))
