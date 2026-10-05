#!/usr/bin/env python3
"""Own one actual domain73 component process group; never run concurrently."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(os.environ['DEMO_TEST_ROOT']).resolve()
sys.path.insert(0,str(ROOT))
from mission.processes import finish_owned_process
from mission.artifacts import snapshot_sources,snapshot_runtime

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--joint-stop-adapter',action='store_true',help='test-only; explicit owner decision after its physical A/B, default disabled')
    args=parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='73',GZ_PARTITION='go2_dynamic_component_'+str(time.time_ns()),
        DEMO_RUN_DIR=str(out),ROS_LOG_DIR=str(out/'ros_logs'),DEMO_TEST_ROOT=str(ROOT),
        DEMO_TEST_JOINT_STOP_ADAPTER='1' if args.joint_stop_adapter else '0')
    # Prevent inherited test-only rate/gait assets from silently changing this run.
    env.pop('DEMO_TEST_ROBOT_URDF',None);env.pop('DEMO_TEST_GAIT_CONFIG',None)
    hashes={}
    for name in ('probe_dynamic_navigation.py','dynamic_probe.launch.py','run_dynamic_component.py'):
        source=Path(__file__).resolve().parent/name
        data=source.read_bytes();(out/name).write_bytes(data);hashes[name]=hashlib.sha256(data).hexdigest()
    wrapper=Path(__file__).resolve().with_suffix('.sh')
    if wrapper.is_file():
        data=wrapper.read_bytes();(out/wrapper.name).write_bytes(data);hashes[wrapper.name]=hashlib.sha256(data).hexdigest()
    scenario=json.loads((ROOT/'simulation/scenario.json').read_text())
    scenario['test_only_scope']='actual north2 then east2 dynamic collidable obstacle component; original limits unchanged'
    (out/'scenario.json').write_text(json.dumps(scenario,indent=2)+'\n')
    (out/'driver_manifest.json').write_text(json.dumps(dict(scope=__doc__,driver_sha256=hashes,
        ros_domain=73,partition=env['GZ_PARTITION'],joint_stop_adapter=args.joint_stop_adapter,
        sole_body_command_publisher='demo_navigation',initial_spawn_world=[0,0,.30],
        world_axis_offsets=[[0,2,0],[2,2,0]],NAV_each_waypoint_sim_s=90,
        raw_arrival_m=.22,independent_joint_arrival_m=.30,obstacle_timing=[8,20,8]),indent=2)+'\n')
    os.environ.update(env);snapshot_sources(ROOT,out);snapshot_runtime(out)
    started=time.monotonic()
    with (out/'stack.log').open('w') as log:
        process=subprocess.Popen(['ros2','launch',str(out/'dynamic_probe.launch.py')],
            env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:process.wait(timeout=450)
        except (subprocess.TimeoutExpired,KeyboardInterrupt):
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
        finally:
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGINT)
                except ProcessLookupError:pass
            finish_owned_process(process,grace=15,terminate_timeout=5,kill_timeout=5)
    path=out/'dynamic_result.json'
    result=json.loads(path.read_text()) if path.is_file() else dict(passed=False,failure='component result missing; inspect actual stack log')
    cleanup=dict(owned_process_group=process.pid,return_code=process.returncode,owned_group_clean=True,wall_seconds=time.monotonic()-started)
    (out/'process_cleanup.json').write_text(json.dumps(cleanup,indent=2)+'\n')
    print(json.dumps(dict(output_directory=str(out),passed=result.get('passed'),failure=result.get('failure'),
        missing_acceptance=result.get('missing_acceptance'),**cleanup),indent=2))
    return 0 if result.get('passed') else 1

if __name__=='__main__':raise SystemExit(main())
