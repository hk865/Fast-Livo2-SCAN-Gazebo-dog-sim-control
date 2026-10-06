#!/usr/bin/env python3
"""Read-only sensor timing and fast IMU audit, separate from cloud processing."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2


class SensorAudit(Node):
    def __init__(self, output):
        super().__init__('demo_readonly_sensor_audit')
        self.stream = Path(output).open('w')
        self.previous = {}
        # Preserve 1000 Hz evidence across short writer/scheduler stalls. This
        # read-only queue is separate from the safety bridge's latest sample.
        imu_qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/livox/imu', self.imu, imu_qos)
        self.create_subscription(PointCloud2, '/livox/lidar', lambda m: self.write('lidar', m), qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_color', lambda m: self.write('camera', m), qos_profile_sensor_data)
        self.create_timer(1., self.stream.flush)

    def write(self, source, msg, **extra):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        previous = self.previous.get(source)
        self.previous[source] = stamp
        self.stream.write(json.dumps({'source': source, 'stamp': stamp,
            'wall_monotonic': time.monotonic(), 'frame_id': msg.header.frame_id,
            'stamp_delta_s': None if previous is None else stamp-previous, **extra})+'\n')

    def imu(self, msg):
        q, a, w = msg.orientation, msg.linear_acceleration, msg.angular_velocity
        self.write('imu', msg, quaternion=[q.x, q.y, q.z, q.w],
                   acceleration=[a.x, a.y, a.z], angular_velocity=[w.x, w.y, w.z],
                   orientation_available=bool(msg.orientation_covariance[0] != -1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args, ros = parser.parse_known_args()
    rclpy.init(args=ros)
    node = SensorAudit(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stream.flush()
        node.stream.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
