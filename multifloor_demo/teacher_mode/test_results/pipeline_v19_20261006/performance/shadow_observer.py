#!/usr/bin/env python3
"""Read-only standalone SLAM odometry/clock observer; no sensor/control writer."""
import argparse
import json
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--out', type=Path, required=True)
    args, rosargs = p.parse_known_args(); out = args.out.resolve()
    if not out.is_dir(): raise ValueError('Prepared owned output directory required')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    rclpy.init(args=rosargs)
    stream = (out/'shadow_poses.jsonl').open('x')
    class Observer(Node):
        def __init__(self):
            super().__init__('pipeline_v19_shadow_observer')
            self.clock_ns = None; self.clock_count = 0; self.pose_count = 0
            self.create_subscription(Clock,'/clock',self.clock,QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Odometry,'/aft_mapped_to_init',self.pose,1000)
        def clock(self, msg):
            self.clock_ns = msg.clock.sec*1_000_000_000+msg.clock.nanosec; self.clock_count += 1
        def pose(self, msg):
            self.pose_count += 1
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            row = {'sequence':self.pose_count,'receipt_monotonic_ns':time.monotonic_ns(),
                   'source_ns':msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                   'latest_observed_clock_ns':self.clock_ns,'frame_id':msg.header.frame_id,
                   'child_frame_id':msg.child_frame_id,'position':[p.x,p.y,p.z],
                   'quaternion_xyzw':[q.x,q.y,q.z,q.w],'linear_velocity':[v.x,v.y,v.z],
                   'pose_covariance':list(msg.pose.covariance)}
            stream.write(json.dumps(row, separators=(',',':'),allow_nan=False)+'\n')
    node = Observer()
    (out/'observer_ready.json').write_text(json.dumps({'schema':'V19_shadow_observer_ready/v1',
        'subscriptions':['/clock','/aft_mapped_to_init'],'actuation_or_truth_subscribed':False})+'\n')
    try: rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException): pass
    finally:
        stream.flush(); stream.close()
        (out/'OBSERVER_RECEIPT.json').write_text(json.dumps({'schema':'V19_shadow_observer/v1',
            'pose_count':node.pose_count,'clock_count':node.clock_count,'last_clock_ns':node.clock_ns,
            'actuation_or_truth_subscribed':False,'clock_age_definition':'Latest observer clock minus odometry header; observer scheduling included, not estimator current-clock ground truth'},indent=2)+'\n')
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__': main()
