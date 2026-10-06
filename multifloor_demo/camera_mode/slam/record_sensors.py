#!/usr/bin/env python3
"""Passive camera-rig acquisition evidence with original integer headers."""
import argparse
import json
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, Image, PointCloud2
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args, ros = parser.parse_known_args()
    rclpy.init(args=ros)

    class SensorAudit(Node):
        def __init__(self):
            super().__init__('camera_readonly_sensor_audit')
            self.stream = args.output.open('x')
            self.previous = {}
            qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Imu, '/livox/imu', self.imu, qos)
            self.create_subscription(PointCloud2, '/livox/lidar', self.lidar, qos_profile_sensor_data)
            self.create_subscription(Image, '/camera/image_color', self.image, qos_profile_sensor_data)
            self.create_timer(1., self.stream.flush)

        def write(self, source, msg, **extra):
            stamp = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
            previous = self.previous.get(source)
            self.previous[source] = stamp
            self.stream.write(json.dumps(dict(source=source, stamp_ns=stamp, stamp=stamp/1e9,
                wall_monotonic=float(time.monotonic()), frame_id=str(msg.header.frame_id),
                stamp_delta_ns=None if previous is None else stamp-previous,
                stamp_delta_s=None if previous is None else (stamp-previous)/1e9, **extra))+'\n')

        def imu(self, msg):
            q, a, w = msg.orientation, msg.linear_acceleration, msg.angular_velocity
            self.write('imu', msg, quaternion=[float(v) for v in (q.x,q.y,q.z,q.w)],
                       acceleration=[float(v) for v in (a.x,a.y,a.z)],
                       angular_velocity=[float(v) for v in (w.x,w.y,w.z)],
                       orientation_available=bool(msg.orientation_covariance[0] != -1))

        def lidar(self, msg):
            self.write('lidar', msg, points=int(msg.width)*int(msg.height), point_step=int(msg.point_step),
                       actual_payload_bytes=int(len(msg.data)), fields=[str(f.name) for f in msg.fields])

        def image(self, msg):
            self.write('camera', msg, width=int(msg.width), height=int(msg.height), encoding=str(msg.encoding),
                       step=int(msg.step), actual_payload_bytes=int(len(msg.data)))

    node = SensorAudit()
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
