#!/usr/bin/env python3
"""Actual ROS transport regression for the read-only IMU recorder."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main():
    root = Path(__file__).resolve().parents[1]
    output = root / 'test_results/sensor_audit_smoke.jsonl'
    rclpy.init()
    node = Node('sensor_audit_transport_probe')
    publisher = node.create_publisher(Imu, '/livox/imu', qos_profile_sensor_data)
    process = subprocess.Popen([sys.executable, str(root/'scripts/record_sensors.py'),
                                '--output', str(output)])
    result = {'test_kind': 'isolated ROS transport, no simulator/control',
              'passed': False}
    try:
        deadline = time.monotonic()+15
        while node.count_subscribers('/livox/imu') < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if process.poll() is not None:
                raise RuntimeError('Recorder exited before ROS discovery')
        if node.count_subscribers('/livox/imu') < 1:
            raise RuntimeError('Recorder subscriber did not become available')
        # Graph discovery can precede completion of the DDS endpoint match.
        settling_until = time.monotonic()+.5
        while time.monotonic() < settling_until:
            rclpy.spin_once(node, timeout_sec=.05)
        # Endpoint matching precedes publishing; a missing sample is a failure.
        for index in range(10):
            msg = Imu()
            msg.header.frame_id = 'imu_probe'
            msg.header.stamp.sec = index+1
            msg.orientation.w = 1.
            msg.orientation_covariance[0] = -1. if index == 0 else 0.
            msg.linear_acceleration.z = 9.81
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.05)
            time.sleep(.05)
        time.sleep(.5)
    finally:
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        code = process.wait(timeout=15)
        node.destroy_node()
        rclpy.try_shutdown()
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    result.update(samples=len(rows), exit_code=code,
                  passed=(code == 0 and len(rows) == 10 and
                          [r['stamp'] for r in rows] == list(range(1, 11)) and
                          all(type(r['orientation_available']) is bool for r in rows) and
                          rows[0]['orientation_available'] is False and
                          all(r['orientation_available'] for r in rows[1:])))
    (root/'test_results/sensor_audit_smoke.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
