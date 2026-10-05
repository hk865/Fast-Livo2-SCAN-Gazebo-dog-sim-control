#!/usr/bin/env python3
"""Pure ROS, no simulator/control: exact owned recorder payload and clean exit."""
import json
import os
from pathlib import Path as FilePath
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

ROOT = FilePath(__file__).resolve().parents[1]
OUT = ROOT / 'test_results' / 'recorder_trajectory_smoke.jsonl'
rclpy.init()
node = rclpy.create_node('navigation_recorder_trajectory_smoke')
path_pub = node.create_publisher(Path, '/demo/navigation/path',
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
metadata_pub = node.create_publisher(String, '/demo/navigation/trajectory_metadata', 10)
status_pub = node.create_publisher(String, '/demo/navigation/status', 10)
clock_pub = node.create_publisher(Clock, '/clock', 10)
metadata = {'reference_stamp': [100, 25], 'requested_body_goal': [1., 2., .3],
    'effective_body_goal': [1., 1.8, .3], 'trajectory': {'order': 3, 'traj_id': 8,
    'start_time': [101, 1], 'pos_pts': [[0., 0., .1], [.4, .5, .2], [1., 2., .3]],
    'knots': [-.3, -.2, -.1, 0., .1, .2, .3]}}
points = [[0., 0., .1], [.4, .5, .2], [1., 2., .3]]
steering = {'stamp': 102., 'odom_stamp': 101.9, 'traj_id': 8,
    'target': [.4, .5, .2], 'heading': 1.1, 'error': .05,
    'direction': [.3, .8], 'exhausted': False}
path = Path()
path.header.frame_id = 'camera_init'
path.header.stamp.sec = 101
path.header.stamp.nanosec = 100
for xyz in points:
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
    pose.pose.orientation.w = 1.
    path.poses.append(pose)
clock = Clock()
clock.clock.sec = 102
process = subprocess.Popen(['bash', str(ROOT / 'run.sh'), 'trace', '--output',
    str(OUT), '--duration', '10', '--cloud-interval', '0'],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
try:
    deadline = time.monotonic() + 5.
    while time.monotonic() < deadline:
        clock_pub.publish(clock)
        metadata_pub.publish(String(data=json.dumps(metadata)))
        status_pub.publish(String(data=json.dumps({'steering': steering})))
        path_pub.publish(path)
        rclpy.spin_once(node, timeout_sec=.05)
        if OUT.exists() and OUT.stat().st_size and OUT.with_name(OUT.stem+'_trajectories.jsonl').exists():
            captured = OUT.with_name(OUT.stem+'_trajectories.jsonl').read_text()
            feedback = OUT.read_text()
            # Metadata callbacks flush immediately; status is written at 10 Hz.
            # Wait for that actual feedback write before stopping the recorder.
            if ('scan_metadata' in captured and 'accepted_path' in captured
                    and '"steering"' in feedback):
                break
    process.send_signal(signal.SIGINT)
    stdout, stderr = process.communicate(timeout=5.)
    entries = [json.loads(line) for line in OUT.with_name(OUT.stem+'_trajectories.jsonl').read_text().splitlines()]
    rows = [json.loads(line) for line in OUT.read_text().splitlines()]
    checks = {
        'complete numeric Bspline payload preserved': any(r.get('metadata') == metadata for r in entries),
        'complete actual accepted path preserved': any(r.get('points') == points and r.get('header_stamp') == [101,100] for r in entries),
        'actual controller steering preserved': any(r.get('navigation',{}).get('steering') == steering for r in rows),
        'clean SIGINT without traceback': process.returncode == 0 and 'Traceback' not in stderr,
    }
    report = {'passed': all(checks.values()), 'domain': int(os.environ['ROS_DOMAIN_ID']),
        'scope': 'synthetic ROS topics and read-only recorder only; no simulator/controller/GT',
        'checks': checks, 'trajectory_events': len(entries), 'feedback_rows': len(rows),
        'stdout': stdout.strip(), 'stderr': stderr.strip()}
    (ROOT/'test_results'/'recorder_trajectory_smoke_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    assert report['passed']
finally:
    if process.poll() is None:
        process.kill()
        process.wait()
    node.destroy_node()
    rclpy.try_shutdown()
