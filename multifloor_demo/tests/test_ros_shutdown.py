#!/usr/bin/env python3
"""Actual ROS78 nodes, shutdown races and live error propagation; no physics."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros

ROOT=Path(__file__).resolve().parents[1]
FILES=['simulation/control_bridge.py','simulation/joint_reference_adapter.py',
    'simulation/obstacle_controller.py','navigation/controller.py','slam/self_echo_filter.py',
    'slam/odom_adapter.py','slam/map_archive.py']


def module(path):
    sys.path.insert(0,str(path.parent))
    spec=importlib.util.spec_from_file_location('shutdown_'+path.stem,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    return m


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='78'
    destination=ROOT/'test_results/ros_shutdown';destination.mkdir(exist_ok=True)
    checks=[]
    for name in FILES:
        with tempfile.TemporaryDirectory(prefix='ros_shutdown_fixture_') as directory:
            live=Path(directory)/'live';live.mkdir()
            os.environ['DEMO_RUN_DIR']=str(live)
            m=module(ROOT/name)
            with patch('rclpy.spin',side_effect=_ros.RCLError('deliberate live-context fixture error')):
                raised=False
                try:m.main()
                except _ros.RCLError:raised=True
                assert raised,name+' swallowed live-context failure'
                checks.append(dict(file=name,check='live-context RCLError propagates',passed=True))
            def closed_context(node):
                rclpy.try_shutdown()
                raise _ros.RCLError('deliberate closed-context wait-set fixture race')
            closed=Path(directory)/'closed';closed.mkdir()
            os.environ['DEMO_RUN_DIR']=str(closed)
            with patch('rclpy.spin',side_effect=closed_context):m.main()
            checks.append(dict(file=name,check='closed-context race exits normally and flushes',passed=True))
    # These processes execute their real main/spin loops; no synthetic spin.
    for trial in range(3):
        processes=[]
        with tempfile.TemporaryDirectory(prefix='ros_sigint_fixture_') as directory:
            try:
                for index,name in enumerate(FILES):
                    case=Path(directory)/str(index);case.mkdir()
                    log=destination/f'sigint_{trial}_{index}.log'
                    stream=log.open('w')
                    proc=subprocess.Popen([sys.executable,str(ROOT/name)],
                        env=dict(os.environ,DEMO_RUN_DIR=str(case)),stdout=stream,stderr=subprocess.STDOUT,
                        start_new_session=True)
                    processes.append((name,proc,stream,log))
                # Allow process imports/ROS endpoints; this bounded wait is local
                # fixture setup, not a simulated robot timing assertion.
                time.sleep(1.2)
                for name,proc,_,_ in processes:
                    assert proc.poll() is None,name+' failed before SIGINT'
                    proc.send_signal(signal.SIGINT)
                for name,proc,stream,log in processes:
                    code=proc.wait(timeout=8);stream.close()
                    output=log.read_text()
                    assert code==0 and 'Traceback' not in output,(name,trial,code,output[-1500:])
                    checks.append(dict(file=name,check='real SIGINT exits zero without traceback',trial=trial,passed=True))
            finally:
                for _,proc,stream,_ in processes:
                    if proc.poll() is None:
                        os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
                    if not stream.closed:stream.close()
    report=dict(scope=__doc__,physics=False,passed=True,checks=checks,
        source_SHA256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in FILES})
    (destination/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Actual ROS shutdown checks:',len(checks),'PASS')


if __name__=='__main__':main()
