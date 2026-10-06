#!/usr/bin/env python3
"""Production launch handlers with synthetic exit events; no nodes or Gazebo."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
from launch import LaunchContext
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.events.process import ProcessExited

ROOT=Path(__file__).resolve().parents[1]


def load(path):
    spec=importlib.util.spec_from_file_location(path.stem.replace('.','_'),path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.generate_launch_description()


def event(action,returncode=0):
    return ProcessExited(action=action,returncode=returncode,name='isolated_fixture',
        cmd=['fixture'],cwd=None,env=None,pid=os.getpid())


def emits_shutdown(actions):
    return any(isinstance(a,EmitEvent) and isinstance(a.event,Shutdown) for a in actions)


def main():
    # Generating descriptions only resolves package/config locations. It does
    # not start a ROS context, execute an action or register a live handler.
    os.environ['DEMO_JOINT_STOP_ADAPTER']='1'
    results=[]
    context=LaunchContext()
    simulation=load(ROOT/'simulation/simulation.launch.py')
    handlers=[a.event_handler for a in simulation.entities if isinstance(a,RegisterEventHandler)]
    processes=[a for a in simulation.entities if isinstance(a,ExecuteProcess)]
    required=[]
    for action in processes:
        matching=[h for h in handlers if h.matches(event(action))]
        if matching:
            assert len(matching)==1
            for code in [0,1,-9]:
                assert emits_shutdown(matching[0].handle(event(action,code),context))
            required.append(action)
    assert len(required)==4 # Actual Gazebo, CHAMP, command bridge and adapter.
    results.append('four required physics/control actions stop stack on any exit')
    assert len(processes)>len(required)
    results.append('normal create/spawner exits are not required process failures')
    with tempfile.TemporaryDirectory(prefix='required_launch_fixture_') as directory:
        os.environ['DEMO_RUN_DIR']=directory
        stack=load(ROOT/'scripts/stack.launch.py')
        handlers=[a.event_handler for a in stack.entities if isinstance(a,RegisterEventHandler)]
        processes=[a for a in stack.entities if isinstance(a,ExecuteProcess)]
        deferred=[]
        for action in processes:
            for handler in handlers:
                if handler.matches(event(action)):
                    output=handler.handle(event(action),context)
                    deferred.extend(a for a in output if isinstance(a,ExecuteProcess))
        assert len(deferred)==1
        navigator=deferred[0]
        matching=[h for h in handlers if h.matches(event(navigator))]
        assert len(matching)==1
        for code in [0,1,-9]:
            assert emits_shutdown(matching[0].handle(event(navigator,code),context))
        results.append('actual deferred navigation action stops stack on any exit')
    report=dict(scope=__doc__,physics=False,passed=True,checks=results,
        source_SHA256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            for name in ['simulation/simulation.launch.py','scripts/stack.launch.py']})
    target=ROOT/'test_results/production_control_staging/required_launch_lifecycle.json'
    target.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
