#!/usr/bin/env python3
"""Actual Teacher IMU/RGB, best-effort input to reliable FAST-LIVO2 QoS.

Header, acquisition stamp, frame, dimensions and payload are unchanged. Overview
RGB is never a VIO input. No simulator truth subscription or pose API exists.
"""
import argparse
import json
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import String
    parser=argparse.ArgumentParser(description=__doc__)
    _,ros=parser.parse_known_args()
    rclpy.init(args=ros)
    class Relay(Node):
        def __init__(self):
            super().__init__('teacher_slam_actual_sensor_relay')
            self.counts={'imu':0,'image':0};self.rejected={'imu':0,'image':0};self.stamps={};self.walls={}
            self.pubs={}
            for name,typ,src,dst in [('imu',Imu,'/livox/imu','/demo/teacher/slam/imu'),
                                     ('image',Image,'/demo/camera','/demo/teacher/slam/image')]:
                self.pubs[name]=self.create_publisher(typ,dst,10)
                self.create_subscription(typ,src,lambda m,n=name:self.receive(n,m),qos_profile_sensor_data)
            self.status=self.create_publisher(String,'/demo/teacher/slam/relay_status',10)
            self.create_timer(1.,self.report)
        def receive(self,name,msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if stamp<=self.stamps.get(name,-1):self.rejected[name]+=1;return
            self.pubs[name].publish(msg)
            self.stamps[name]=stamp;self.walls[name]=time.monotonic();self.counts[name]+=1
        def report(self):
            self.status.publish(String(data=json.dumps(dict(source='actual Teacher sensor messages',
                counts=self.counts,rejected=self.rejected,stamp_ns=self.stamps,
                wall_age={n:time.monotonic()-v for n,v in self.walls.items()},
                field_payload_and_stamp_unchanged=True,overview_used=False,ground_truth_used=False))))
    node=Relay()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
