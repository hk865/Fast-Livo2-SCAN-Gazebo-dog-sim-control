#!/usr/bin/env python3
"""Isolated synthetic startup-node contract test; no physical acceptance claim."""
import json,os,pathlib,signal,subprocess,tempfile,time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu,Image
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

ROOT=pathlib.Path(__file__).resolve().parents[1]

def main():
    if int(os.environ.get('ROS_DOMAIN_ID','0'))!=78:raise RuntimeError('This synthetic fixture requires unused ROS_DOMAIN_ID=78')
    rclpy.init();n=Node('startup_gate_synthetic_fixture')
    ip=n.create_publisher(Imu,'/livox/imu',10);cp=n.create_publisher(Clock,'/clock',10)
    camera=n.create_publisher(Image,'/camera/image_color',10);bridge=n.create_publisher(String,'/demo/control/safety',10)
    directory=pathlib.Path(tempfile.mkdtemp(prefix='startup_gate_',dir=ROOT/'test_results'))
    log=(directory/'stdout.log').open('w');env=dict(os.environ,DEMO_RUN_DIR=str(directory))
    proc=subprocess.Popen(['python3',str(ROOT/'scripts/wait_sensors.py')],env=env,stdout=log,stderr=log,start_new_session=True)
    checks={};start=time.monotonic()
    try:
        while ip.get_subscription_count()<1 and time.monotonic()-start<8:rclpy.spin_once(n,timeout_sec=.01)
        if ip.get_subscription_count()<1:raise RuntimeError('startup node did not subscribe')
        for i in range(1701):
            t=i*.01;stamp_sec=int(t);stamp_ns=round((t-stamp_sec)*1e9)
            c=Clock();c.clock.sec=stamp_sec;c.clock.nanosec=stamp_ns;cp.publish(c)
            m=Imu();m.header.stamp=c.clock;m.orientation.w=1.;m.linear_acceleration.z=9.81
            if 10<=t<=12:m.angular_velocity.x=.08;m.linear_acceleration.x=.65
            ip.publish(m)
            if i%5==0:
                s=String();s.data=json.dumps({'state':'hold' if 10<=t<=12 else 'ready','requested':[0,0,0],'safe':[0,0,0]});bridge.publish(s)
            if i%10==0:
                image=Image();image.header.stamp=c.clock;image.height=image.width=1;image.encoding='rgb8';image.step=3;image.data=[0,0,0];camera.publish(image)
            if i==1200:checks['did_not_start_during_recovery']=proc.poll() is None
            if proc.poll() is not None:break
            # Faster-than-real replay; message timestamps still span real policy duration.
            rclpy.spin_once(n,timeout_sec=.003)
        proc.wait(timeout=5);log.flush()
        report=json.loads((directory/'startup_gate.json').read_text())
        checks['clean_exit']=proc.returncode==0
        checks['passed_after_three_seconds_static']=report['passed'] and report['metrics']['first_stamp']>12 and report['metrics']['duration']>=3-1e-8
        checks['metrics_and_source_saved']=report['metrics']['samples']>=300 and report['source'].endswith('no ground truth')
        result={'scope':'ROS78 synthetic node contract only; no physical proof','checks':checks,'passed':all(checks.values()),'artifact_directory':str(directory),'startup_report':report}
        (ROOT/'test_results/startup_gate_ros.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='startup_report'},indent=2))
        if not result['passed']:raise RuntimeError('startup ROS fixture failed')
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close();n.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
