#!/usr/bin/env python3
"""Exercise real controller methods on fake state, without rclpy.init/nodes."""
import json
import argparse
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import numpy as np
from std_msgs.msg import String
from controller import TeacherNavigation
from control_core import RawImuTilt
from teacher_transition import TeacherHeadingGate


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path);args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];profile=json.loads((root/'navigation/flat_relative_roundtrip.json').read_text())
    checks={}
    def check(name,condition):
        checks[name]={'passed':bool(condition)}
        if not condition:raise AssertionError(name)
    raw=RawImuTilt(np.eye(3));raw.update([0,0,0,1],1.,10.,1.)
    heading=TeacherHeadingGate(profile);heading.phase='drive'
    f=NS(raw_imu=raw,rotation=np.eye(3),max_tilt=0.,bridge_safety={'state':'hold'},
         bridge_updated=10.,evidence=NS(error=None),state='running',tilt_stops=0,
         tilt_hold=False,tilt_source=None,tilt_clear_since=None,pose_updated=10.,pose_timeout=.3,
         heading_gate=heading,samples=np.ones((3,3)),last_reference=0.)
    check('communication_hold_immediately_protects',TeacherNavigation.apply_tilt_guard(f,10.05,1.05))
    check('hold_is_not_measured_tilt_or_heading_reset',f.tilt_stops==0 and not f.tilt_hold and heading.phase=='drive'and f.samples is not None)
    f.bridge_safety={'state':'ready'};f.bridge_updated=10.1
    check('fresh_recovery_retains_checked_path_and_heading',not TeacherNavigation.apply_tilt_guard(f,10.1,1.1)and f.samples is not None and heading.phase=='drive')
    raw.tilt=.31
    check('actual_high_tilt_unchanged_threshold_protects',TeacherNavigation.apply_tilt_guard(f,10.1,1.1)and f.tilt_hold and f.tilt_stops==1)
    raw.tilt=.51
    check('actual_severe_tilt_latches_failed',TeacherNavigation.apply_tilt_guard(f,10.1,1.1)and f.state=='failed')
    raw.tilt=0.;f.state='running';f.bridge_safety={'state':'failed'}
    check('bridge_integrity_fault_latches_failed',TeacherNavigation.apply_tilt_guard(f,10.1,1.1)and f.state=='failed')
    f.bridge_safety={'state':'ready'};f.state='running';f.tilt_hold=False
    check('raw_imu_expiry_protects',TeacherNavigation.apply_tilt_guard(f,10.4,1.4))
    count=[0]
    def zero():count[0]+=1;f.command=[0.,0.,0.]
    f.state='running';f.command=[.2,0,0];f.apply_tilt_guard=lambda *a:True
    f.reset_region_arrival=lambda *a:None;f.publish_command=zero;f.publish_status=lambda:None
    f.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000))
    for _ in range(200):
        TeacherNavigation.on_bridge_safety(f,String(data=json.dumps({'mode':'teacher','source':'scan_slam','state':'hold','monotonic_wall':time.monotonic()})))
    check('200_bridge_hold_messages_stop_once_without_echo_loop',count[0]==1 and f.command==[0.,0.,0.])
    TeacherNavigation.on_bridge_safety(f,String(data=json.dumps({'mode':'teacher','source':'scan_slam','state':'ready','monotonic_wall':time.monotonic()-1.})))
    check('stale_health_status_not_rejuvenated_on_receive',f.bridge_safety['state']=='hold')
    receipt={'schema':1,'status':'passed','checks':checks,'starts_ros_nodes':False,'rclpy_init_called':False,
        'starts_simulation':False,'runtime_validation':'unverified','scope':'Actual controller method fault checks using synthetic state only'}
    output=args.output or root/'test_results/navigation_interface_v4_controller_method_checks.json'
    output.write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n');print(json.dumps({'status':'passed','checks':len(checks),'output':str(output)}))


if __name__=='__main__':main()
