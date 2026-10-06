"""Import the future standard navigation layout at a deep archive, no ROS node."""
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
DEMO = HERE.parents[2]
ARCHIVE = HERE / 'archive_standard' / 'nested' / 'evidence' / 'source' / 'multifloor_demo'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    paths = []
    for name in ('drift_controller.py', 'turn_drift.py', 'guard_configuration.json'):
        src, dst = HERE / name, ARCHIVE / 'navigation' / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst); paths.append((src, dst))
    for name in ('controller.py', 'control_core.py', 'trajectory_contract.py',
                 'event_archive.py', 'goal_regions.py'):
        src, dst = DEMO / 'navigation' / name, ARCHIVE / 'navigation' / name
        shutil.copyfile(src, dst); paths.append((src, dst))
    for src, dst in ((DEMO / 'simulation/scenario.json', ARCHIVE / 'simulation/scenario.json'),
                     (HERE / 'stack.launch.py', ARCHIVE / 'scripts/stack.launch.py')):
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst); paths.append((src, dst))
    # This process only imports modules. It does not call init, instantiate Node,
    # generate a launch, publish, or create a DDS endpoint.
    child = r'''
import importlib.util,json,os,pathlib,sys
from types import SimpleNamespace
root=pathlib.Path(sys.argv[1]);sys.path.insert(0,str(root/'navigation'))
assert 'DEMO_TEST_ROOT' not in os.environ
import drift_controller as entry
import controller, control_core, trajectory_contract, event_archive, goal_regions, turn_drift
assert not entry.rclpy.ok()
modules=[entry,controller,control_core,trajectory_contract,event_archive,goal_regions,turn_drift]
checks={'no_test_root_environment':True, 'no_ros_context':not entry.rclpy.ok(),
        'standard_root':entry.ROOT==root,
        'all_navigation_dependencies_archive_local':all(pathlib.Path(m.__file__).parent==root/'navigation' for m in modules),
        'default_scenario_archive_local':(controller.FilePath(controller.__file__).resolve().parent.parent/'simulation/scenario.json').is_file()}
cfg=json.loads((root/'navigation/guard_configuration.json').read_text())
g=turn_drift.TurnDriftSupervisor(profile=cfg['profile'],require_path_offset=cfg['require_path_offset'])
stub=SimpleNamespace(drift_pending=False,drift=None,drift_events=0,drift_stop_clock_ns=None,
                     drift_stop_count=None,drift_log_dropped=0,drift_log=[])
status=entry.NavigationDrift.drift_status(stub)
checks['selected_profile_matches_declared_limits']=cfg['limits']==dict(g.LIMITS,require_path_offset=False)==status['limits']
checks['selected_profile_matches_native_status']=cfg['profile']==status['candidate']
checks['scenario_sensor_extrinsic_available']='orientation_reference' in json.loads((root/'simulation/scenario.json').read_text())['sensors']['imu']
print(json.dumps({'checks':checks,'passed':all(checks.values()),'root':str(root),
     'module_files':{m.__name__:m.__file__ for m in modules},'rclpy_ok':entry.rclpy.ok(),
     'context_initialized':False,'dds_nodes_created':False},sort_keys=True))
assert all(checks.values())
'''
    env = dict(os.environ); env.pop('DEMO_TEST_ROOT', None)
    completed = subprocess.run([sys.executable, '-c', child, str(ARCHIVE)],
                               env=env, text=True, capture_output=True, timeout=20)
    (HERE / 'archive_import_stdout.log').write_text(completed.stdout)
    (HERE / 'archive_import_stderr.log').write_text(completed.stderr)
    result = json.loads(completed.stdout) if completed.stdout.strip() else {}
    result['exit_code'] = completed.returncode
    result['archive_copy_sha256'] = {str(dst.relative_to(ARCHIVE)):sha(dst) for src,dst in paths}
    result['copies_byte_identical'] = all(sha(src)==sha(dst) for src,dst in paths)
    # Confirm the relocated stack resolves the same standard demo root and entry.
    tree = ast.parse((ARCHIVE / 'scripts/stack.launch.py').read_text())
    result['stack_uses_standard_root_and_navigation_entry'] = (
        "Path(__file__).resolve().parents[1]" in ast.unparse(tree)
        and "root / 'navigation/drift_controller.py'" in ast.unparse(tree))
    result['passed'] = (result.get('passed',False) and result['copies_byte_identical']
                       and result['stack_uses_standard_root_and_navigation_entry']
                       and completed.returncode==0)
    (HERE / 'archive_import_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'passed':result['passed'],'checks':len(result.get('checks',{})),
                      'exit_code':completed.returncode,'copy_count':len(paths)}))
    return not result['passed']

if __name__=='__main__':
    raise SystemExit(main())
