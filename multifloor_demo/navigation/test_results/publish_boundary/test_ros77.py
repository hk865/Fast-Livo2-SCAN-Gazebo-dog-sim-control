#!/usr/bin/env python3
"""Owned ROS77 actual publication/DDS test, no simulator or body commands."""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
import statistics
from pathlib import Path

HERE=Path(__file__).resolve().parent

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def consumer(output):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import JointState
    rclpy.init();node=Node('publish_boundary_consumer');received=[]
    def record(m):
        received.append(dict(header_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec,
            frame=m.header.frame_id,names=list(m.name),position=list(m.position),
            velocity=list(m.velocity),effort=list(m.effort)))
    subscription=node.create_subscription(JointState,'/joint_states',record,
        QoSProfile(depth=2000,reliability=ReliabilityPolicy.BEST_EFFORT))
    until=time.monotonic()+12
    while time.monotonic()<until and len(received)<500:rclpy.spin_once(node,timeout_sec=.02)
    output.write_text(json.dumps(dict(rows=received,actual_QoS_depth=subscription.qos_profile.depth))+'\n')
    node.destroy_node();rclpy.shutdown()
    return 0 if len(received)==500 else 3

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--consumer',type=Path);args=parser.parse_args()
    if args.consumer:return consumer(args.consumer)
    if os.environ.get('ROS_DOMAIN_ID')!='77':raise ValueError('Explicit isolated ROS77 required')
    output=HERE/'ros77_v1';output.mkdir(exist_ok=False)
    result=dict(scope=__doc__,checks={},runs={},source_sha256={name:sha(HERE/name) for name in
        ('rcl_publish_audit.cpp','librcl_publish_audit.so','publisher.cpp','publisher','test_ros77.py')})
    expected=[dict(header_ns=1378123456789+i*1000000,frame='actual_jstate_fixture',
        names=['lf_hip_joint','rf_hip_joint'],position=[i/10.,-i/10.],
        velocity=[i/20.,-i/20.],effort=[i/30.,-i/30.]) for i in range(500)]
    for hooked in (False,True):
        name='hook' if hooked else 'no_hook';folder=output/name;folder.mkdir()
        consumed=folder/'consumer.json';captured=folder/'publication'
        env=os.environ.copy();env.pop('LD_PRELOAD',None)
        env['DEMO_PUBLISH_AUDIT_OUTPUT']=str(captured)
        children=[]
        try:
            with (folder/'consumer.log').open('w') as log:
                cp=subprocess.Popen([sys.executable,str(__file__),'--consumer',str(consumed)],env=env,stdout=log,stderr=subprocess.STDOUT)
                children.append(cp)
                publish_env=env.copy()
                if hooked:publish_env['LD_PRELOAD']=str(HERE/'librcl_publish_audit.so')
                with (folder/'publisher.log').open('w') as plog:
                    pp=subprocess.Popen([str(HERE/'publisher')],env=publish_env,stdout=plog,stderr=subprocess.STDOUT)
                    children.append(pp);pub_rc=pp.wait(timeout=12)
                sub_rc=cp.wait(timeout=14)
            data=json.loads(consumed.read_text())
            result['runs'][name]=dict(publisher_return=pub_rc,consumer_return=sub_rc,
                                     received=len(data['rows']),actual_QoS_depth=data['actual_QoS_depth'],
                                     owned_children_clean=all(c.poll() is not None for c in children))
            result['checks'][name+'_500_exact_fields_order']=data['rows']==expected
            result['checks'][name+'_process_exit0']=pub_rc==sub_rc==0
            timings=next(json.loads(s) for s in (folder/'publisher.log').read_text().splitlines()
                         if s.startswith('{"cpp_publish_elapsed_ns"'))
            elapsed=sorted(timings['cpp_publish_elapsed_ns'])
            result['runs'][name]['cpp_publish_elapsed_median_ns']=statistics.median(elapsed)
            result['runs'][name]['cpp_publish_elapsed_p95_ns']=elapsed[int(.95*(len(elapsed)-1))]
            result['checks'][name+'_C_type_actual_publish_success']=(timings['C_type_init_ret']==timings['C_type_publish_ret']==timings['C_type_fini_ret']==0)
            captures=list(folder.glob('publication.*.jsonl'))
            if not hooked:
                result['checks']['no_hook_has_no_capture']=not captures
            else:
                if len(captures)!=1:raise ValueError('Expected one actual publisher PID capture')
                actual_capture=captures[0]
                records=[json.loads(s) for s in actual_capture.read_text().splitlines()]
                summary=records[0];pubs=[r for r in records if r['kind']=='publisher']
                cpp_pubs=[p for p in pubs if p['cpp_sensor_msgs_JointState_verified'] is True]
                calls=[r for r in records if r['kind']=='publish'];handles={p['handle'] for p in cpp_pubs}
                result['runs'][name]['capture_summary']=summary
                result['checks']['actual_topic_handle_cpp_ABI_verified']=(len(cpp_pubs)==1 and cpp_pubs[0]['topic']=='/joint_states')
                result['checks']['C_type_same_topic_not_cast_or_captured']=(len(pubs)==2 and len(cpp_pubs)==1 and len(calls)==500)
                result['checks']['actual_owned_PID_file']=(summary['pid']==pp.pid and actual_capture.name==f'publication.{pp.pid}.jsonl')
                result['checks']['all_and_only_joint_states_captured']=(len(calls)==500 and [r['header_ns'] for r in calls]==[r['header_ns'] for r in expected] and all(r['handle'] in handles for r in calls) and summary['unmatched_topic_calls']>=500)
                result['checks']['unchanged_header_and_actual_return']=(all(r['header_ns']==r['header_after_ns'] and r['ret']==0 and r['return_wall_ns']>=r['enter_wall_ns'] for r in calls))
                result['checks']['no_overflow_or_incomplete']=(summary['overflow']==summary['incomplete']==0)
                result['runs'][name]['capture_SHA256']=sha(actual_capture)
        finally:
            for child in children:
                if child.poll() is None:child.terminate()
            for child in children:
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
    result['runs']['median_elapsed_difference_ns']=(result['runs']['hook']['cpp_publish_elapsed_median_ns']-result['runs']['no_hook']['cpp_publish_elapsed_median_ns'])
    result['checks']['hook_median_publish_time_below_100us']=(result['runs']['hook']['cpp_publish_elapsed_median_ns']<100000)
    result['passed']=all(result['checks'].values())
    result['limitations']=['No-hook and hook use fresh separate publishers; this validates payload preservation, not identical timing.',
        'Capture is written on normal process exit only. SIGKILL/crash produces no trustworthy complete evidence.',
        'Capacity65536; overflow/incomplete or unknown CPP type prevents complete-boundary claims.',
        'Publication enter is after fixed-memory observer preparation, not a ros2_control update timestamp.',
        'No publication-source, consumer, or physical safety timeout is modified.']
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
