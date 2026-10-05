"""Exercise the actual coordinator callback without creating a ROS node."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from mission_server import Coordinator
from sensor_msgs.msg import Imu


class HeadingHistoryTest(unittest.TestCase):
    def history_at_rate(self, rate):
        stub = SimpleNamespace(lock=threading.RLock(), touch=lambda _: None,
                               imu_attitudes=deque())
        for tick in range(5*rate+1):
            msg = Imu()
            msg.header.stamp.sec = tick//rate
            msg.header.stamp.nanosec = (tick % rate)*(1_000_000_000//rate)
            msg.orientation.w = 1.
            Coordinator.on_imu(stub, msg)
        return stub

    def test_delayed_slam_pair_retained_at_both_rates(self):
        for rate in (100, 1000):
            with self.subTest(rate=rate):
                stub = self.history_at_rate(rate)
                stamps = [s for s, _ in stub.imu_attitudes]
                self.assertEqual(len(stamps), 4*rate+1)
                self.assertEqual(stamps[0], 1.)
                # A SLAM callback delayed .7 s still finds its exact IMU pair;
                # the calibration's existing 20 ms match rule is unchanged.
                self.assertLess(abs(min(stamps,key=lambda s:abs(s-4.3))-4.3),1e-9)
                self.assertNotIn(.9, stamps)

    def test_old_and_unavailable_attitudes_do_not_extend_history(self):
        stub = self.history_at_rate(1000)
        previous = list(stub.imu_attitudes)
        for sec, unavailable in ((4, False), (5, False), (6, True)):
            msg = Imu()
            msg.header.stamp.sec = sec
            msg.orientation.w = 1.
            msg.orientation_covariance[0] = -1. if unavailable else 0.
            Coordinator.on_imu(stub,msg)
        self.assertEqual(list(stub.imu_attitudes),previous)


if __name__ == '__main__':
    unittest.main()
