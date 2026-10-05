#!/usr/bin/env python3
"""Actual ROS77 mission callbacks and persisted regional receipts; no physics."""
import copy, hashlib, json, os, sys, tempfile, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from mission_server import Coordinator
from mission.route_regions import add_route_regions
from navigation.goal_regions import parse_goal


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='77'
    rclpy.init();server=Coordinator();server.timer.cancel();fixture=Node('mission_region_transport_fixture')
    pubs=[fixture.create_publisher(Odometry,'/demo/slam/body_odom',qos_profile_sensor_data),
          fixture.create_publisher(Odometry,'/demo/ground_truth',qos_profile_sensor_data),
          fixture.create_publisher(String,'/demo/navigation/status',10)]
    def until(condition):
        end=time.monotonic()+4
        while not condition() and time.monotonic()<end:rclpy.spin_once(server,timeout_sec=.02)
        assert condition(), 'actual ROS callback/discovery timeout'
    checks=[]
    try:
        until(lambda:all(p.get_subscription_count() for p in pubs))
        qos=server.subs[1].qos_profile
        assert qos.depth==2000 and qos.reliability==ReliabilityPolicy.BEST_EFFORT and qos.history==HistoryPolicy.KEEP_LAST
        checks.append('actual truth observer BEST_EFFORT KEEP_LAST 2000; no control input')
        with tempfile.TemporaryDirectory(prefix='mission_region_ros_') as d:
            directory=Path(d);server.run_dir=directory
            server.audit_stream=(directory/'pose_audit.jsonl').open('w')
            server.navigation_stream=(directory/'navigation_audit.jsonl').open('w')
            scenario=add_route_regions({'exploration':[[0,-1,0],[5,2,.3]],'return_origin':[[0,0,0]],'navigation_f1_f3':[[0,7,2.4]]})
            server.mission.start('ros-interface',scenario);server.mission.origin=[0.,0.,0.]
            server.mission.enter('exploring',0.,'transport fixture only');server.mission.request_route('exploration',0.)
            for pub in pubs[:2]:
                msg=Odometry();msg.header.frame_id='camera_init';msg.header.stamp.sec=1790000000;msg.header.stamp.nanosec=123456789
                msg.pose.pose.position.x=5.;msg.pose.pose.position.y=2.;msg.pose.pose.position.z=.3;msg.pose.pose.orientation.w=1.;pub.publish(msg)
            until(lambda:server.counts['slam_pose'] and server.counts['ground_truth'])
            server.audit_stream.flush();rows=[json.loads(x) for x in (directory/'pose_audit.jsonl').read_text().splitlines()]
            assert len(rows)==2 and all(x['stamp_ns']==1790000000123456789 for x in rows)
            checks.append('raw SLAM/truth integer header ns preserved at epoch timestamp')
            m=server.mission;receipts=[]
            for i,g in enumerate(m.current_goals):
                receipts.append(dict(request_id=m.current_request,goal_id=g['goal_id'],goals_definition_sha256=m.current_goals_sha256,
                    stamp_ns=(i+1)*1000000000,start_stamp_ns=(i+1)*1000000000-400000000,dwell_ns=400000000,
                    raw_position=g['center'],region_inside=True,protected=False,reason='arrived',max_observation_gap_ns=200000000,
                    arrival_definition=g['arrival'],control_region_inside=True,
                    control_arrival_definition=parse_goal(g).control_arrival_definition()))
            valid=dict(request_id=m.current_request,state='succeeded',goals_definition_sha256=m.current_goals_sha256,
                goals_definitions=copy.deepcopy(m.current_goals),region_arrivals=receipts,total=2,waypoint_index=2)
            stale=copy.deepcopy(valid);stale['request_id']='old-request';pubs[2].publish(String(data=json.dumps(stale)))
            until(lambda:server.counts['navigation']==1);assert m.stage=='exploring'
            checks.append('stale completion cannot advance actual mission callback')
            old_request=m.current_request
            pubs[2].publish(String(data=json.dumps(valid)));until(lambda:m.stage=='returning')
            server.navigation_stream.flush();navrows=[json.loads(x) for x in (directory/'navigation_audit.jsonl').read_text().splitlines()]
            assert len(navrows)==2 and navrows[-1]['stage']=='exploring' and navrows[-1]['current_request']==old_request and navrows[-1]['status']==valid
            assert m.current_request!=old_request and m.goal_region['arrival']['radius_m']==.22
            checks.append('complete native NAV receipt persisted before immediate stage/request transition')
            assert navrows[-1]['status'].get('dynamic_obstacle_verified') is None
            checks.append('recorded native status preserved before Mission augments obstacle evidence')
            server.audit_stream.close();server.navigation_stream.close();server.audit_stream=server.navigation_stream=None
        report=dict(passed=True,physics=False,scope=__doc__,checks=checks,source_SHA256={x:hashlib.sha256((ROOT/x).read_bytes()).hexdigest() for x in ['scripts/mission_server.py','mission/state_machine.py','mission/route_regions.py','navigation/goal_regions.py']})
        target=ROOT/'test_results/mission_regions/actual_ros_inner_result.json';target.parent.mkdir(exist_ok=True);target.write_text(json.dumps(report,indent=2)+'\n')
        print('Actual ROS mission region checks:',len(checks),'PASS')
    finally:
        server.destroy_node();fixture.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
