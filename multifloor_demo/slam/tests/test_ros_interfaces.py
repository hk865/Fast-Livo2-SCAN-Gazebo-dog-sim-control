#!/usr/bin/env python3
"""Real ROS transport/service test with explicit synthetic message fixtures.

This is an interface test, not evidence of sensor SLAM accuracy. Run separately
from a demo, e.g. ROS_DOMAIN_ID=74 python3 tests/test_ros_interfaces.py.
"""
import json
import struct
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from odom_adapter import OdomAdapter
from map_archive import MapArchive


def run():
    rclpy.init()
    test = Node('slam_interface_fixture_test')
    executor = SingleThreadedExecutor()
    root=Path(__file__).resolve().parents[1]
    report={'test_kind':'ROS transport with synthetic fixtures; not SLAM accuracy', 'checks':{}}
    with tempfile.TemporaryDirectory(prefix='go2_slam_interface_') as temporary:
        adapter=OdomAdapter(root.parent/'simulation/scenario.json')
        archive=MapArchive(temporary,.08)
        for node in [test,adapter,archive]:executor.add_node(node)
        outputs={'body':[],'lidar':[],'map':[],'status':[]}
        subscriptions=[]
        for key,topic,typ in [('body','/demo/slam/body_odom',Odometry),('lidar','/demo/slam/lidar_odom',Odometry),
                             ('map','/demo/slam/map_status',String),('status','/demo/slam/status',String)]:
            subscriptions.append(test.create_subscription(typ,topic,lambda msg,k=key:outputs[k].append(msg),10))
        raw_pub=test.create_publisher(Odometry,'/aft_mapped_to_init',10)
        rgb_pub=test.create_publisher(PointCloud2,'/cloud_registered',10)
        sensor_publishers=[test.create_publisher(t,n,10) for n,t in [('/livox/lidar',PointCloud2),('/livox/imu',Imu),('/camera/image_color',Image),('/cloud_registered_full',PointCloud2)]]
        start=time.monotonic()
        while time.monotonic()-start<.5:executor.spin_once(timeout_sec=.02)
        for index in range(16):
            raw=Odometry();raw.header.frame_id='camera_init';raw.header.stamp.sec=100+index//10
            raw.header.stamp.nanosec=(index%10)*100000000
            raw.pose.pose.position.x=index*.01
            raw.pose.pose.orientation.w=1.
            raw_pub.publish(raw)
            cloud=PointCloud2();cloud.header=raw.header;cloud.height=1;cloud.width=3
            cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])]
            cloud.fields.append(PointField(name='rgb',offset=12,datatype=PointField.UINT32,count=1))
            cloud.point_step=16;cloud.row_step=48;cloud.is_dense=True
            cloud.data=b''.join(struct.pack('<fffI',x,y,z,rgb) for x,y,z,rgb in [(1,2,3,0xff0000),(2,3,4,0x00ff00),(3,4,5,0x0000ff)])
            rgb_pub.publish(cloud)
            for pub,typ in zip(sensor_publishers,[PointCloud2,Imu,Image,PointCloud2]):
                msg=typ();msg.header=raw.header;pub.publish(msg)
            until=time.monotonic()+.05
            while time.monotonic()<until:executor.spin_once(timeout_sec=.005)
        # Duplicate pose timestamps are rejected and cannot create infinite velocity.
        raw_pub.publish(raw)
        for _ in range(20):executor.spin_once(timeout_sec=.01)
        report['checks']['body_transport']=len(outputs['body'])>=10
        report['checks']['lidar_transport']=len(outputs['lidar'])>=10
        body=outputs['body'][-1];lidar=outputs['lidar'][-1]
        report['checks']['same_sensor_stamp']=body.header.stamp==lidar.header.stamp==raw.header.stamp
        report['checks']['precise_lidar_offset']=abs(lidar.pose.pose.position.x-body.pose.pose.position.x-.2)<1e-9 and abs(lidar.pose.pose.position.z-body.pose.pose.position.z-.1177)<1e-9
        report['checks']['valid_body_velocity']=abs(body.twist.twist.linear.x-.1)<1e-6
        report['checks']['reject_duplicate_timestamp']=adapter.counts['rejected_timestamp']==1
        client=test.create_client(Trigger,'/demo/slam/save_map')
        assert client.wait_for_service(timeout_sec=2)
        future=client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future,timeout_sec=3)
        report['checks']['save_service_success']=future.done() and future.result().success
        metadata=json.loads((Path(temporary)/'map_metadata.json').read_text())
        report['checks']['current_run_rgb_map']=metadata['point_count']==3 and metadata['rgb_points']==3 and metadata['run_id']==Path(temporary).name
        binary=(Path(temporary)/metadata['binary_filename']).read_bytes()
        report['checks']['binary_size_and_real_colors']=len(binary)==48 and binary[12:15]==bytes([255,0,0])
        report['checks']['pcd_saved']=Path(future.result().message).is_file() and metadata['save']['complete']
        report['checks']['no_reference_map_or_ground_truth']=not metadata['reference_map_loaded'] and not metadata['ground_truth_used']
        report['checks']['saved_sensor_health_evidence']=metadata['save']['healthy_sensor_evidence']['slam_healthy'] \
            and metadata['save']['healthy_sensor_evidence']['camera_healthy'] \
            and all(age<2 for age in metadata['save']['healthy_sensor_evidence']['ages'].values())
        until=time.monotonic()+4.5
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.01)
            if outputs['map'] and not json.loads(outputs['map'][-1].data)['slam_healthy']:
                break
        report['checks']['message_staleness_detected']=not json.loads(outputs['map'][-1].data)['slam_healthy']
        camera_received=archive.received['camera']
        replay=Image();replay.header=raw.header;sensor_publishers[2].publish(replay)
        until=time.monotonic()+.1
        while time.monotonic()<until:executor.spin_once(timeout_sec=.005)
        report['checks']['replay_cannot_refresh_camera_health']=archive.received['camera']==camera_received \
            and archive.counts['rejected_camera_timestamp']>=1
        stale_save=client.call_async(Trigger.Request())
        executor.spin_until_future_complete(stale_save,timeout_sec=3)
        report['checks']['stale_map_save_rejected']=stale_save.done() and not stale_save.result().success
        report['checks']['last_good_pcd_preserved_after_rejected_save']=(Path(temporary)/'colored_map.pcd').is_file()
        for node in [archive,adapter,test]:executor.remove_node(node);node.destroy_node()
    rclpy.shutdown()
    report['passed']=all(report['checks'].values())
    output=root/'tests/ros_interface_result.json';output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(run())
