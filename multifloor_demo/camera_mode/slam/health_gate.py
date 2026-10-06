#!/usr/bin/env python3
"""Two real-input gates; only the post-SLAM gate permits camera motion.

No GT, CM, gait, joint, body-contact or foot topic is subscribed. Both phases
retain the original warmup, three-second static IMU/gravity thresholds.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time

from _shared import HERE, load_shared, verify_sources
from startup_policy import CameraStaticImuWindow, ObservationHealth


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, Image, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', required=True, choices=('sensors', 'slam'))
    parser.add_argument('--scenario', default=str(HERE.parent/'simulation/scenario.json'))
    parser.add_argument('--run-dir', default=os.environ.get('DEMO_RUN_DIR'), required=False)
    parser.add_argument('--timeout-wall', type=float, default=150.)
    args, ros = parser.parse_known_args()
    if not args.run_dir or not 1 <= args.timeout_wall <= 150:
        parser.error('New run-dir and timeout-wall within 1..150 seconds are required')
    verify_sources()
    scenario = json.loads(Path(args.scenario).read_text())
    if scenario.get('mode') != 'camera':
        parser.error('Explicit camera scenario is required')
    sensors = scenario['sensors']
    policy = CameraStaticImuWindow(sensors['imu']['orientation_reference']['world_quaternion'])
    health = ObservationHealth()
    geometry = load_shared('cloud_geometry', 'slam/geometry.py')
    rclpy.init(args=ros)
    node = Node('camera_' + args.phase + '_startup_gate')
    startup_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, '/camera_demo/startup', startup_qos)
    clock_ns = None
    clock_wall = None
    status = {}
    status_wall = {}
    rejected = {}
    reason = 'waiting_for_real_sensors'
    passed = False

    def reject(name):
        rejected[name] = rejected.get(name, 0)+1

    def ns(msg):
        return msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec

    def clock(msg):
        nonlocal clock_ns, clock_wall
        stamp = msg.clock.sec*1_000_000_000 + msg.clock.nanosec
        if clock_ns is not None and stamp < clock_ns:
            policy.reset('clock_reversed')
            health.stamps.clear(); health.walls.clear()
        clock_ns, clock_wall = stamp, time.monotonic()

    def imu(msg):
        if msg.header.frame_id != sensors['imu']['frame']:
            reject('imu_frame'); policy.reset('invalid_imu_frame'); return
        now = time.monotonic()
        stamp = ns(msg)
        q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        valid = msg.orientation_covariance[0] != -1 and all(math.isfinite(v) for v in
            (q.x, q.y, q.z, q.w, w.x, w.y, w.z, a.x, a.y, a.z))
        if not valid or not health.observe('imu', stamp, now):
            reject('imu'); policy.reset('invalid_or_nonincreasing_imu'); return
        policy.update(stamp/1e9, now, (q.x, q.y, q.z, q.w), (w.x, w.y, w.z),
                      (a.x, a.y, a.z), msg.orientation_covariance[0] != -1)

    def image(msg):
        if msg.header.frame_id != sensors['camera']['frame'] or msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8') \
                or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*3 or len(msg.data) < msg.step*msg.height:
            reject('camera'); return
        if not health.observe('camera', ns(msg), time.monotonic()): reject('camera')

    def lidar(msg):
        from lidar_relay import valid_cloud
        if not valid_cloud(msg, sensors['lidar']['frame']): reject('lidar'); return
        if not health.observe('lidar', ns(msg), time.monotonic()): reject('lidar')

    def safety(msg):
        try:
            data = json.loads(msg.data)
            policy.update_bridge(data, time.monotonic())
        except (TypeError, ValueError):
            policy.bridge = None
            policy.reset('invalid_camera_safety')

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'demo_slam_body' \
                or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) \
                or q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < 1e-12:
            reject('odom'); return
        if not health.observe('odom', ns(msg), time.monotonic()): reject('odom')

    def colored(msg):
        try:
            xyz, rgb = geometry.cloud_records(msg)
            if msg.header.frame_id != 'camera_init' or len(xyz) == 0 or len(rgb) != len(xyz):
                raise ValueError('invalid actual RGB SLAM cloud')
        except (ValueError, TypeError): reject('colored_cloud'); return
        if not health.observe('colored_cloud', ns(msg), time.monotonic()): reject('colored_cloud')

    def receive_status(name, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict): raise ValueError('invalid status')
            status[name], status_wall[name] = data, time.monotonic()
        except (TypeError, ValueError): reject(name)

    qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
        node.create_subscription(Imu, '/livox/imu', imu, qos),
        node.create_subscription(Image, '/camera/image_color', image, qos_profile_sensor_data),
        node.create_subscription(PointCloud2, '/livox/lidar', lidar, qos_profile_sensor_data),
        node.create_subscription(String, '/demo/control/safety', safety, 10),
    ]
    if args.phase == 'slam':
        subscriptions += [
            node.create_subscription(Odometry, '/demo/slam/body_odom', odom, qos_profile_sensor_data),
            node.create_subscription(PointCloud2, '/cloud_registered', colored, qos_profile_sensor_data),
            node.create_subscription(String, '/demo/slam/status', lambda m: receive_status('slam', m), 10),
            node.create_subscription(String, '/demo/slam/map_status', lambda m: receive_status('map', m), 10),
        ]
    start = time.monotonic()
    last_report = 0.
    try:
        while rclpy.ok() and time.monotonic()-start < args.timeout_wall:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            ready = clock_ns is not None and clock_wall is not None and now-clock_wall < .30
            ready = ready and all(health.fresh(n, clock_ns, now) for n in ('imu', 'lidar', 'camera'))
            ready = ready and health.counts.get('camera', 0) >= 10 and health.counts.get('lidar', 0) >= 10
            ready = ready and policy.ready(clock_ns/1e9, now)
            reason = policy.reason
            if ready and args.phase == 'slam':
                ready = all(health.fresh(n, clock_ns, now) and health.counts.get(n, 0) >= 2
                            for n in ('odom', 'colored_cloud'))
                ready = ready and all(n in status_wall and 0 <= now-status_wall[n] < 2 for n in ('slam', 'map'))
                ready = ready and status.get('slam', {}).get('healthy') is True \
                    and status['slam'].get('ground_truth_used') is False \
                    and status.get('map', {}).get('slam_healthy') is True \
                    and status['map'].get('camera_healthy') is True \
                    and status['map'].get('ground_truth_used') is False \
                    and status['map'].get('reference_map_loaded') is False \
                    and status['map'].get('error') is None
                reason = 'real_sensor_SLAM_RGB_ready' if ready else 'waiting_for_initialized_sensor_SLAM_RGB'
            if ready:
                passed = True
                break
            if now-last_report >= .5:
                pub.publish(String(data=json.dumps(dict(schema=1, mode='camera', phase=args.phase,
                    state='waiting', reason=reason))))
                last_report = now
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        reason = 'interrupted'
    finally:
        now = time.monotonic()
        state = ('ready' if args.phase == 'slam' else 'sensors_ready') if passed else 'failed'
        report = dict(schema_version=1, mode='camera', phase=args.phase, state=state, passed=passed,
            reason=reason, clock_ns=clock_ns, wall_elapsed=now-start, static_imu=policy.report(),
            observed=health.report(clock_ns or 0, now), rejected=rejected, status=status,
            ground_truth_used=False, quadruped_prerequisites=[])
        directory = Path(args.run_dir)
        directory.mkdir(parents=True, exist_ok=True)
        (directory/('startup_'+args.phase+'.json')).write_text(json.dumps(report, indent=2)+'\n')
        if node.context.ok():
            pub.publish(String(data=json.dumps(report)))
            # Process the outgoing publication before the required gate exits.
            rclpy.spin_once(node, timeout_sec=.05)
        print('Camera startup gate: '+json.dumps(report), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
