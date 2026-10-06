#!/usr/bin/env python3
"""Actual ROS negative/control test for a briefly stalled 1000 Hz recorder."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission.processes import finish_owned_process


def trial(script, output, expected_depth):
    node = Node('audit_buffer_transport_probe')
    pub = node.create_publisher(Imu, '/livox/imu',
        QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT))
    process = subprocess.Popen([sys.executable, str(script), '--output', str(output)],
                               start_new_session=True)
    paused = False
    try:
        deadline = time.monotonic()+15
        while node.count_subscribers('/livox/imu') != 1 and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('audit recorder exited before discovery')
            rclpy.spin_once(node, timeout_sec=.05)
        time.sleep(.5)  # Complete DDS matching before this measured stream.
        endpoints = node.get_subscriptions_info_by_topic('/livox/imu')
        if len(endpoints) != 1:
            raise RuntimeError('unexpected subscriber inventory after matching')
        graph_depth = endpoints[0].qos_profile.depth
        start = time.monotonic()
        for index in range(500):
            if index == 100:
                process.send_signal(signal.SIGSTOP)
                paused = True
            if index == 180:
                process.send_signal(signal.SIGCONT)
                paused = False
            msg = Imu()
            msg.header.frame_id = 'audit_buffer_probe'
            msg.header.stamp.sec = 1
            msg.header.stamp.nanosec = index*1_000_000
            msg.orientation.w = 1.
            msg.linear_acceleration.z = 9.81
            pub.publish(msg)
            remaining = start+(index+1)*.001-time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        time.sleep(.7)
    finally:
        if paused and process.poll() is None:
            process.send_signal(signal.SIGCONT)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        finish_owned_process(process, grace=10, terminate_timeout=3, kill_timeout=3)
        node.destroy_node()
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    expected = [1+i*1_000_000*1e-9 for i in range(500)]
    actual = [row['stamp'] for row in rows]
    return dict(script=str(script.resolve()),
        script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
        configured_subscription_depth=expected_depth, graph_reported_depth=graph_depth,
        sent=500, received=len(rows),
        exact_ordered_stream=actual == expected,
        missing_stamps=[s for s in expected if s not in set(actual)],
        return_code=process.returncode, owned_group_clean=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    out = args.output_dir.resolve()
    legacy = out/'record_sensors_depth5.py'
    if not legacy.is_file():
        raise RuntimeError('archived original depth5 recorder required')
    rclpy.init()
    try:
        baseline = trial(legacy, out/'baseline.jsonl', 5)
        current = trial(ROOT/'scripts/record_sensors.py', out/'current.jsonl', 2000)
    finally:
        rclpy.try_shutdown()
    result = dict(scope='ROS77 synthetic transport only; no physics, navigation, or sensor rewrite. '
        '500 unique 1ms stamps published over 0.5s; owned recorder paused for ~80ms.',
        baseline=baseline, current=current,
        passed=not baseline['exact_ordered_stream'] and current['exact_ordered_stream']
            and baseline['return_code'] == current['return_code'] == 0,
        test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('scope', 'passed')}, ensure_ascii=False))
    print('received:', baseline['received'], current['received'])
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
