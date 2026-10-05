#!/usr/bin/env python3
"""QoS relay of the actual scan; no quadruped self-envelope deletion.

All field bytes, row layout and acquisition stamps remain the original message.
The real camera rig's near returns may remain in its scans. This is explicit,
and is not silently replaced with the old Go2 body/leg bounding-box mask.
"""
import argparse
import json
import time


def valid_cloud(msg, expected_frame):
    fields = {f.name: f for f in msg.fields}
    return (msg.header.frame_id == expected_frame and msg.width*msg.height > 0
            and msg.point_step > 0 and msg.row_step >= msg.width*msg.point_step
            and len(msg.data) >= msg.height*msg.row_step
            and all(n in fields and fields[n].datatype == 7 and fields[n].count == 1
                    and 0 <= fields[n].offset <= msg.point_step-4 for n in ('x', 'y', 'z')))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame', required=True)
    args, ros = parser.parse_known_args()
    rclpy.init(args=ros)

    class Relay(Node):
        def __init__(self):
            super().__init__('camera_slam_actual_lidar_relay')
            self.publisher = self.create_publisher(PointCloud2, '/camera_demo/slam/lidar', 10)
            self.status = self.create_publisher(String, '/camera_demo/slam/lidar_status', 10)
            self.create_subscription(PointCloud2, '/livox/lidar', self.receive, qos_profile_sensor_data)
            self.last_stamp_ns = None
            self.received = 0
            self.rejected = 0
            self.last_wall = None
            self.create_timer(1., self.report)

        def receive(self, msg):
            ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
            if not valid_cloud(msg, args.frame) or ns <= (self.last_stamp_ns if self.last_stamp_ns is not None else -1):
                self.rejected += 1
                return
            self.publisher.publish(msg)
            self.last_stamp_ns = ns
            self.last_wall = time.monotonic()
            self.received += 1

        def report(self):
            self.status.publish(String(data=json.dumps(dict(mode='camera', received=self.received,
                rejected=self.rejected, original_stamp_ns=self.last_stamp_ns,
                wall_age=None if self.last_wall is None else time.monotonic()-self.last_wall,
                input='/livox/lidar', output='/camera_demo/slam/lidar', removed_points=0,
                point_fields_and_payload_unchanged=True, ground_truth_used=False))))

    node = Relay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
