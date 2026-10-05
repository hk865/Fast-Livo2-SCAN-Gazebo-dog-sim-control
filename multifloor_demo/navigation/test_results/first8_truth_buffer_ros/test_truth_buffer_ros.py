#!/usr/bin/env python3
"""ROS77 transport-only reproduction using the real frozen first8 QoS method."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry

ROOT=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo')
SOURCE=ROOT/'navigation/test_results/first8_component_staging/probe_first_eight.py'
EXPECTED_SHA='6639252fba6fb930d0ca4b0f4be76926bc3b84456a7b029b8b92e51aac573b62'
sys.path.insert(0,str(ROOT))
from mission.processes import finish_owned_process,group_running

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def actual_receiver_class(source):
    tree=ast.parse(source.read_text())
    cls=next(c for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='FirstEightProbe')
    method=next(m for m in cls.body if isinstance(m,ast.FunctionDef) and m.name=='create_subscription')
    actual=ast.ClassDef(name='ActualReceiver',bases=[ast.Name(id='Node',ctx=ast.Load())],
        keywords=[],body=[method],decorator_list=[])
    module=ast.fix_missing_locations(ast.Module(body=[actual],type_ignores=[]))
    scope={'Node':Node,'QoSProfile':QoSProfile,'ReliabilityPolicy':ReliabilityPolicy}
    exec(compile(module,str(source),'exec'),scope)
    return scope['ActualReceiver'],ast.get_source_segment(source.read_text(),method)

def receive(a):
    assert sha(SOURCE)==EXPECTED_SHA,'Frozen first8 source changed'
    rclpy.init()
    cls,_=actual_receiver_class(SOURCE)
    node=(cls if a.mode=='candidate' else Node)('truth_buffer_'+a.mode)
    with a.output.open('w',buffering=1) as log:
        def callback(m):
            log.write(json.dumps(dict(stamp_ns=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec,
                frame_id=m.header.frame_id,child_frame_id=m.child_frame_id,
                index=m.pose.pose.position.x,orientation_w=m.pose.pose.orientation.w,
                twist_x=m.twist.twist.linear.x))+'\n')
        truth_sub=node.create_subscription(Odometry,'/demo/ground_truth',callback,qos_profile_sensor_data)
        # The actual method must preserve the standard latest-pose queue.
        pose_sub=node.create_subscription(Odometry,'/demo/pose',lambda m:None,qos_profile_sensor_data)
        a.output.with_suffix('.subscription.json').write_text(json.dumps(dict(
            truth_depth=truth_sub.qos_profile.depth,truth_reliability=truth_sub.qos_profile.reliability.name,
            pose_depth=pose_sub.qos_profile.depth,pose_reliability=pose_sub.qos_profile.reliability.name),indent=2)+'\n')
        try:rclpy.spin(node)
        except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
        finally:node.destroy_node();rclpy.try_shutdown()

def trial(out,mode,depth):
    node=Node('truth_buffer_publisher_'+mode)
    pub=node.create_publisher(Odometry,'/demo/ground_truth',
        QoSProfile(depth=2000,reliability=ReliabilityPolicy.BEST_EFFORT))
    log=out/(mode+'.jsonl')
    with (out/(mode+'.process.log')).open('w') as process_log:
        proc=subprocess.Popen([sys.executable,'-B',str(Path(__file__).resolve()),
            '--receiver','--mode',mode,'--output',str(log)],stdout=process_log,
            stderr=subprocess.STDOUT,start_new_session=True)
        paused=False
        try:
            deadline=time.monotonic()+15
            while node.count_subscribers('/demo/ground_truth')!=1 and time.monotonic()<deadline:
                if proc.poll() is not None:raise RuntimeError('Owned receiver exited before DDS discovery')
                rclpy.spin_once(node,timeout_sec=.02)
            time.sleep(.5)
            endpoints=node.get_subscriptions_info_by_topic('/demo/ground_truth')
            unchanged=node.get_subscriptions_info_by_topic('/demo/pose')
            assert len(endpoints)==len(unchanged)==1,'Unexpected ROS77 endpoint inventory'
            graph_depth=endpoints[0].qos_profile.depth
            graph_reliability=endpoints[0].qos_profile.reliability.name
            pose_depth=unchanged[0].qos_profile.depth
            pose_reliability=unchanged[0].qos_profile.reliability.name
            start=time.monotonic();pause_start=None;pause_end=None
            for index in range(500):
                if index==100:
                    proc.send_signal(signal.SIGSTOP);paused=True;pause_start=time.monotonic()
                if index==180:
                    proc.send_signal(signal.SIGCONT);paused=False;pause_end=time.monotonic()
                m=Odometry();m.header.stamp.sec=1;m.header.stamp.nanosec=index*1_000_000
                m.header.frame_id='truth_buffer_unique_stream';m.child_frame_id='body'
                m.pose.pose.position.x=float(index);m.pose.pose.orientation.w=1.
                m.twist.twist.linear.x=float(index)+.25
                pub.publish(m)
                delay=start+(index+1)*.001-time.monotonic()
                if delay>0:time.sleep(delay)
            time.sleep(.7)
        finally:
            if paused and proc.poll() is None:proc.send_signal(signal.SIGCONT)
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
            finish_owned_process(proc,grace=10,terminate_timeout=3,kill_timeout=3)
            clean=not group_running(proc.pid);node.destroy_node()
    rows=[json.loads(line) for line in log.read_text().splitlines()]
    actual=[r['stamp_ns'] for r in rows];expected=[1_000_000_000+i*1_000_000 for i in range(500)]
    fields=all(r['frame_id']=='truth_buffer_unique_stream' and r['child_frame_id']=='body'
        and r['orientation_w']==1. and r['index']==(r['stamp_ns']-1_000_000_000)//1_000_000
        and r['twist_x']==r['index']+.25 for r in rows)
    local=json.loads(log.with_suffix('.subscription.json').read_text())
    return dict(mode=mode,configured_depth=depth,actual_subscription=local,actual_graph_depth=graph_depth,
        actual_graph_reliability=graph_reliability,unchanged_pose_depth=pose_depth,
        unchanged_pose_reliability=pose_reliability,sent=500,received=len(rows),
        stamps_unique=len(set(actual))==len(actual),strict_ordered=all(a<b for a,b in zip(actual,actual[1:])),
        exact_500_ordered=actual==expected,fields_preserved=fields,
        missing_stamp_ns=[s for s in expected if s not in set(actual)],
        pause_wall_seconds=pause_end-pause_start,return_code=proc.returncode,owned_group_clean=clean)

def main():
    p=argparse.ArgumentParser();p.add_argument('--receiver',action='store_true')
    p.add_argument('--mode',choices=['baseline','candidate']);p.add_argument('--output',type=Path)
    p.add_argument('--output-dir',type=Path)
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='77':raise RuntimeError('Only isolated ROS77 authorized')
    if a.receiver:return receive(a)
    assert sha(SOURCE)==EXPECTED_SHA
    out=a.output_dir.resolve() if a.output_dir else Path(__file__).resolve().parent
    out.mkdir(parents=True,exist_ok=True)
    (out/'first8_source_before.py').write_bytes(SOURCE.read_bytes())
    _,method=actual_receiver_class(SOURCE);(out/'extracted_actual_method.txt').write_text(method+'\n')
    rclpy.init()
    try:
        baseline=trial(out,'baseline',5)
        candidate=trial(out,'candidate',2000)
    finally:rclpy.try_shutdown()
    checks=dict(frozen_actual_source=sha(SOURCE)==EXPECTED_SHA,
        baseline_drops=not baseline['exact_500_ordered'],candidate_full=candidate['exact_500_ordered'],
        actual_subscription_qos=baseline['actual_subscription']['truth_depth']==5
            and candidate['actual_subscription']['truth_depth']==2000,
        best_effort=baseline['actual_graph_reliability']==candidate['actual_graph_reliability']=='BEST_EFFORT',
        nontruth_qos_unchanged=baseline['actual_subscription']['pose_depth']==candidate['actual_subscription']['pose_depth']==5
            and baseline['actual_subscription']['pose_reliability']==candidate['actual_subscription']['pose_reliability']=='BEST_EFFORT',
        ordered_unique=all(r['stamps_unique'] and r['strict_ordered'] for r in (baseline,candidate)),
        fields_preserved=all(r['fields_preserved'] for r in (baseline,candidate)),
        owned_clean=all(r['owned_group_clean'] and r['return_code']==0 for r in (baseline,candidate)))
    result=dict(scope='ROS77 actual Odom transport and actual first8 subscription method only; no physics/control. '
        'The original physical A remains FAIL; no original truth or .15s acceptance contract is altered.',
        first8_source_sha256=EXPECTED_SHA,test_sha256=sha(Path(__file__)),baseline=baseline,
        candidate=candidate,checks=checks,passed=all(checks.values()),
        graph_depth_limit='RMW graph reports depth 0 for both endpoints; actual Node subscription.qos_profile depths and measured transport are recorded. No graph depth assertion claimed.')
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],baseline_received=baseline['received'],
        candidate_received=candidate['received'],checks=checks),indent=2))
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
