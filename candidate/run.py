#!/usr/bin/env python3
"""Own one actual SLAM/SCAN cascade CPU Teacher run; simulation only.

Prepare without ROS/Gazebo:
  python3 -B navigation/path_admission_v28_capture/run.py --profile repair_prefix270 --label V28_prepare270 --prepare-only
Run after source review (root/user only):
  python3 -B navigation/path_admission_v28_capture/run.py --profile repair_prefix270 --label V28_capture270 --domain 91
Root first evaluates the270s prefix; repair_original46 repeats the original
1500s mission only when root judges the prefix useful. Both exact profiles are
source-bound in one V28 gate; no historical full46 pass is inherited.
The ROS/configuration interpreter is system Python. --cpu-python selects the
already tested PyTorch interpreter for the CPU-only Actor: system Python here
has ROS but does not have torch. No dependency installation or RL process edit.
Preparation and runtime failures receive fresh run identities and receipts.
Historical acceptance is an immutable source, never a launch-success gate.
"""
from __future__ import annotations
import package_paths as _package_paths
import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
ROOT=_package_paths.TEACHER_ROOT  # Read-only original assets/dependencies; executed source remains HERE.
DEMO=ROOT.parent
SYSTEM_PYTHON=Path('/usr/bin/python3')
DEFAULT_CPU_PYTHON=Path('/home/hyh001/IsaacLab/_isaac_sim/kit/python/bin/python3')
ROS_BASE=Path('/opt/ros/jazzy/setup.bash')
SLAM_UNDERLAY=DEMO/'slam/ros2_ws/install/setup.bash'
from slam_workspace import SLAM_WORKSPACE, SLAM_SOURCE_ROOT, PRIVATE as PRIVATE_SLAM, verify_slam_workspace
from ipc_fault_window import CONTRACT as IPC_FAULT_WINDOW_CONTRACT
from ipc_request_window import CONTRACT as REQUEST_SUBSTAGE_CONTRACT
SLAM_OVERLAY=SLAM_WORKSPACE/'install/local_setup.bash'
SCAN_OVERLAY=DEMO/'navigation/ros2_ws/install/setup.bash'
BRIDGE=Path('/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge')
CHECKPOINT=Path('/home/hyh001/projects/1.Project/RL_for_unitree/logs/rsl_rl/rl_unitree_go2_aer_height_distribution/2026-10-01_17-23-23_STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A-FORMAL-4096ENV-3000ITER-SEED42/model_1000.pt')
MODEL_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
REQUIRED_ROLES=('worker','bridge','capture','navigation_stack','gazebo')
EXPECTED_CHILDREN=11


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write_new(path,value):
    with Path(path).open('x') as stream:
        json.dump(value,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')


def failure_record(error,phase):
    record=dict(phase=phase,monotonic_ns=time.monotonic_ns(),
        kind='exception' if isinstance(error,Exception) else 'reported_outcome',
        error=f'{type(error).__name__}: {error}' if isinstance(error,Exception) else error)
    if isinstance(error,Exception):
        record.update(exception_type=type(error).__name__,message=str(error))
        try:observation=getattr(error,'observation',None)
        except Exception as observation_error:
            record['observation_read_error']=type(observation_error).__name__
        else:
            if observation is not None:record['observation']=observation
    return record


def write_failure_evidence(run,name,value):
    # Exceptional/cleanup path only. Evidence failure must not replace the
    # original exception or prevent the original cleanup actions.
    try:write_new(Path(run)/name,value)
    except Exception as error:
        try:print(f'Failure evidence unavailable ({name}): {type(error).__name__}: {error}',file=sys.stderr,flush=True)
        except Exception:pass


def exceptional_runtime_manifest(run,plan,error):
    context=None;context_error=None
    try:context=json.loads((Path(run)/'runtime_failure_context.json').read_text())
    except (OSError,ValueError) as read_error:context_error=f'{type(read_error).__name__}: {read_error}'
    first=context.get('first_failure') if isinstance(context,dict) else None
    escaped=failure_record(error,'execute_escape')
    cleanup_failures=[escaped] if isinstance(context,dict) else []
    try:cleanup_failures=[json.loads((Path(run)/'guardian_close_failure.json').read_text())]
    except (OSError,ValueError):pass
    return {**plan,'error':escaped['error'],
        'first_failure':first,'cleanup_failures':cleanup_failures,'escaped_failure':escaped,
        'failure_context_read_error':context_error,'owned_processes':[],
        'ownership_at_cleanup_start':context.get('owned_processes') if isinstance(context,dict) else None,
        'final_owned_process_exitcodes_known':False,
        'runtime_started':context.get('runtime_started') if isinstance(context,dict) else None,
        'runtime_status':'failed','all_owned_and_children_clean':False,
        'training_processes_signaled':False,'real_robot':False}


def diagnostic_full_evidence_archive_allowed(profile,startup_stop,run,execution_request,task_contract):
    """Pure identity/predicate check for the original bounded EOF archive grace."""
    if not isinstance(profile,dict) or not isinstance(startup_stop,dict):return False
    diagnostic=profile.get('diagnostic_observe_only')
    if (not isinstance(diagnostic,dict) or diagnostic.get('enabled')is not True
            or diagnostic.get('full_evidence_after_endpoint')is not True
            or startup_stop.get('reason')!='actual_R85_diagnostic_terminated'):return False
    run=Path(run)
    if (not isinstance(execution_request,dict) or execution_request.get('run')!=str(run)
            or not isinstance(task_contract,dict)
            or task_contract.get('schema')!='r85_pct_scan_student_task/v1'
            or task_contract.get('run_id')!=run.name or task_contract.get('kind')!='f1_to_f3'):return False
    result=startup_stop.get('diagnostic_result')
    if (not isinstance(result,dict) or result.get('run_id')!=run.name
            or type(result.get('request_id'))is not str or not result['request_id']
            or result['request_id']!=run.name+':r85:f1_to_f3:1'
            or result.get('state')!='diagnostic_evidence_complete'
            or result.get('diagnostic_full_evidence_observed')is not True
            or result.get('terminal')is not True or result.get('passed')is not False
            or result.get('formal_full_route_pass')is not False):return False
    predicates=result.get('actual_original_predicates')
    return isinstance(predicates,dict) and all(predicates.get(key)is True for key in(
        'endpoint_control_band_dwell','fixed5_parking','current_run_RGB_map',
        'dynamic_protocol','dynamic_same_read_capture'))


def baseline_temporal_capture_limits(plan,run,here):
    """Optional diagnostic stop budget; original profile/localization untouched."""
    if os.environ.get('GO2_BASELINE_TEMPORAL_CAPTURE')!='R29_bounded_baseline':return
    path=Path(here)/'KNOWN_SCENE_TEMPORAL_CAPTURE_CONTRACT.json'
    c=json.loads(path.read_text())
    if (c.get('schema')!='known_scene_baseline_temporal_capture/v1'
            or c.get('required_run_label') not in Path(run).name
            or c.get('original_matcher_sha256')!=sha(Path(here)/'known_scene_matcher.py')
            or c.get('stop_request_wall_cap_s')!=238 or c.get('stop_request_sim_cap_s')!=120
            or c.get('maximum_allocated_and_logical_bytes')!=20*1024*1024):
        raise ValueError('Wrong optional bounded baseline capture input')
    plan['wall_budget_s']=min(plan['wall_budget_s'],float(c['stop_request_wall_cap_s']))
    plan['bounded_baseline_temporal_capture']=dict(contract_path=str(path.resolve()),
        contract_sha256=sha(path),original_matcher_sha256=c['original_matcher_sha256'],
        maximum_observer_bytes=c['maximum_allocated_and_logical_bytes'],
        original_guardian_uses_shorter_wall_budget=True,localization_or_profile_changed=False,
        stop_request_sim_cap_s=c['stop_request_sim_cap_s'],total_physical_stop_hard_bound_claimed=False)


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    obj=importlib.util.module_from_spec(spec);sys.modules[name]=obj;spec.loader.exec_module(obj)
    return obj


def profile_path(value):
    p=Path(value).expanduser()
    if not p.is_file():p=HERE/'profiles'/(value if value.endswith('.json') else value+'.json')
    if not p.is_file():raise ValueError('Unknown new closed-loop profile '+value)
    return p.resolve()


def structural(element,exclude_model_pose=False):
    """Preserve all robot physics/sensor fields while ignoring XML indentation."""
    return (element.tag,tuple(sorted(element.attrib.items())),(element.text or '').strip(),
        tuple(structural(c) for c in element if not(exclude_model_pose and c.tag=='pose')))


def validate_profile(p):
    if p.get('r85_session',{}).get('enabled') is True:
        from r85_preflight import validate_profile as validate_r85_profile
        return validate_r85_profile(p,HERE)
    if p.get('controller_kind')!='teacher' or p.get('navigation_ground_truth_used') is not False:
        raise ValueError('Actual SLAM/SCAN Teacher scope required')
    if p.get('review_basis')!='explicit_user_closed_loop_controller_actual_SLAM_SCAN_multifloor_simulation':
        raise ValueError('Missing explicit new closed-loop scope')
    if p.get('pose_cloud_timeout_s')!=.3:raise ValueError('Source freshness must remain300ms')
    from clock_hold import CONTRACT
    if p.get('control_clock_contract')!=CONTRACT:raise ValueError('Prospective exact-zero clock hold contract differs')
    if p.get('diagnostic_scope',{}).get('detail_window_sim_s')!=[1600,1601]:
        raise ValueError('V33 dense solver payload disabled via window outside1500s cap [1600,1601]')
    from corridor_preflight import verify_preflight
    from full46_launch_contract import validate_full46
    validate_full46(p,HERE)
    verify_preflight(HERE,p)
    if type(p.get('boundary_timing',False)) is not bool:
        raise ValueError('Boundary timing must be explicit true/false')
    duration=float(p['duration_s']);spawn=p['spawn'];asset=p['expected_asset'];cascade=p['cascade']
    if not math.isfinite(duration) or not 30<=duration<=1500:raise ValueError('Invalid bounded simulation duration')
    if len(spawn)!=4 or not all(math.isfinite(float(v))for v in spawn) or asset['spawn']!=spawn:
        raise ValueError('Explicit matching physical spawn required')
    limits=cascade['command_limits']
    if len(limits)!=3 or not all(math.isfinite(float(x))and 0<float(x)<=y for x,y in zip(limits,[1,.4,1])):
        raise ValueError('Invalid bounded Teacher command envelope')
    overview=p.get('overview_pose',[8,3,3,0,.65,-math.pi/2])
    if len(overview)!=6 or not all(math.isfinite(float(v))for v in overview):raise ValueError('Invalid observation camera pose')
    if cascade.get('navigation_ground_truth_used') is not False:raise ValueError('Truth controller prohibited')
    if cascade.get('feedback_expected_hz')!=p['sensor_sampling']['camera_hz'] or cascade.get('feedback_ttl_sim_and_wall_s')!=.3:
        raise ValueError('Explicit camera-driven nominal source rate and unchanged300ms required')
    return duration,overview


def run_command(command,log):
    with Path(log).open('x')as stream:
        subprocess.run(command,check=True,stdout=stream,stderr=subprocess.STDOUT)


def prepare(run,args):
    selected=profile_path(args.profile);p=json.loads(selected.read_text());duration,overview=validate_profile(p)
    r85=p.get('r85_session',{}).get('enabled') is True
    if not r85 and (selected.name!='engineering_nav46.json' or args.shadow_sample_contract is not None):raise RuntimeError('R33 permits only the original46 startup profile without an extra collector')
    startup=json.loads((HERE/'R33_SHORT_STARTUP_CONTRACT.json').read_text())
    if not r85 and (startup.get('candidate_root')!=str(HERE) or startup.get('maximum_simulator_attempts')!=1 or startup.get('GT_carrier_allowed')is not False):raise RuntimeError('R33 startup authorization/source contract differs')
    verify_slam_workspace()
    if PRIVATE_SLAM and p['pipeline']['mode']!='serial':raise RuntimeError('Private IMU priority requires serial pipeline')
    if args.surface_validity!=p['engineering_contract']['surface_validity_flag']:
        raise RuntimeError('V33 freezes surface validity0 for engineering isolation')
    from scan_workspace import scan_contract
    scan_workspace,scan_contract_path,scan_contract_data=scan_contract(p)
    scan_overlay=scan_workspace/'install/setup.bash'
    if not 1<=args.domain<=232 or not .1<=args.real_time_factor<=1:raise ValueError('Private domain/RTF invalid')
    for f in (SYSTEM_PYTHON,args.cpu_python,ROS_BASE,SLAM_UNDERLAY,SLAM_OVERLAY,scan_overlay,BRIDGE,CHECKPOINT):
        if not f.is_file():raise RuntimeError('Existing runtime dependency missing '+str(f))
    if sha(CHECKPOINT)!=MODEL_SHA:raise RuntimeError('Frozen Teacher model hash changed')
    # Dependency-location inspection does not load the Actor or create ROS nodes.
    probe=subprocess.run([str(args.cpu_python),'-B','-c',
        'import importlib.util,json,sys;print(json.dumps({"python":sys.executable,"version":sys.version,"torch_spec":None if importlib.util.find_spec("torch")is None else importlib.util.find_spec("torch").origin,"numpy_spec":None if importlib.util.find_spec("numpy")is None else importlib.util.find_spec("numpy").origin}))'],
        check=True,capture_output=True,text=True,timeout=20)
    interpreter=json.loads(probe.stdout.strip())
    if interpreter['torch_spec']is None or interpreter['numpy_spec']is None:
        raise RuntimeError('Selected CPU interpreter cannot import frozen Actor dependencies')
    write_new(run/'cpu_interpreter_inspection.json',interpreter)
    write_new(run/'input_profile.json',p)
    prep=[str(SYSTEM_PYTHON),'-B',str(ROOT/'simulation/prepare.py'),'--output',str(run),
          '--terrain',p['expected_asset']['terrain'],'--sensors','--camera-rate','10',
          '--render-engine','ogre2','--real-time-factor',str(args.real_time_factor),
          '--overview-fov',str(math.radians(80)),'--overview-pose',*map(str,overview)]
    run_command(prep,run/'prepare.log')
    tree=ET.parse(run/'world.sdf');robot=tree.getroot().find("world/model[@name='go2']")
    if robot is None:raise RuntimeError('Prepared Teacher model absent')
    before=structural(robot,True);x,y,z,yaw=p['spawn']
    robot.find('pose').text=f'{x} {y} {z} 0 0 {yaw}'
    if structural(robot,True)!=before:raise RuntimeError('Unexpected robot mutation')
    native_path=Path(startup['native_guard_private_reuse_path'])
    if sha(native_path)!=startup['native_guard_private_reuse_sha256']:raise RuntimeError('Frozen actual-contact native guard changed')
    actuator_plugins=[q for q in robot.findall('plugin') if 'TeacherActuator' in q.get('name','')]
    if len(actuator_plugins)!=1:raise RuntimeError('R33 requires the original sole native actuator')
    actuator_plugins[0].set('filename',str(native_path))
    ET.indent(tree);tree.write(run/'world.sdf',encoding='unicode')
    from sampling import override_sensors
    sampling_receipt=override_sensors(run,p)
    asset=json.loads((run/'asset_manifest.json').read_text())
    asset.update(spawn=p['spawn'],spawn_override=True,scenario_label=args.label,world_sha256=sha(run/'world.sdf'),
                 sensor_contract_sha256=sha(run/'sensor_contract.json'),sensor_sampling_override=sampling_receipt,
                 actual_native_guard_path=str(native_path),actual_native_guard_sha256=sha(native_path),
                 native_guard_reused_unchanged=True,Actor_and_PD_sources_modified=bool(r85),
                 original_native_PD_modified=False,selected_Actor_changed_to_Student=bool(r85))
    # The only rewritten input is this fresh, unlaunched generated asset receipt.
    (run/'asset_manifest.json').write_text(json.dumps(asset,indent=2)+'\n')
    names=[c.get('name','')for c in robot.findall('plugin')]
    if sum('TeacherActuator'in n for n in names)!=1 or any('ros2_control'in n.lower() or 'champ'in n.lower()for n in names):
        raise RuntimeError('Exactly one native Teacher actuator required')
    if asset.get('physics_step_s')!=.005 or asset.get('decimation')!=4:raise RuntimeError('Native50/200Hz contract changed')
    actual_com=[float(v)for v in asset['base_inertial_pose'].split()][:3]
    if len(p['cascade']['base_com_offset'])!=3 or any(abs(float(a)-b)>1e-9 for a,b in zip(p['cascade']['base_com_offset'],actual_com)):
        raise RuntimeError('Controller COM differs from unchanged physical asset')
    run_command([str(SYSTEM_PYTHON),'-B',str(ROOT/'tests/cloud_transport_probe_20261004/prepare_transport.py'),
                 '--run',str(run),'--profile','shm_64m'],run/'cloud_transport_prepare.log')
    from known_scene_reference_builder import build as build_known_scene_reference
    write_new(run/'known_scene_reference.json',build_known_scene_reference(run/'world.sdf'))
    from scan_runtime_parameters import parameters as scan_parameters
    write_new(run/'v34_scan_overrides.yaml',{'scan_planner_node':{'ros__parameters':scan_parameters(p)}})
    # Bind event capture before the one-time source archive/scope freeze.
    capture=json.loads((HERE/'EVENT_CAPTURE_CONTRACT_TEMPLATE.json').read_text())
    capture['run_id']=run.name
    capture['authorization']['delegation']='R44 original-route10GB proposed single round with durable owner; source-bound observer capture is evidence only; no shadow scoring'
    if r85:capture['authorization']['delegation']='R85 explicit bounded overnight PCT_SCAN_Student session; source-bound observer capture is evidence only; no inherited R83 or 46-region PASS'
    capture['authorization']['runtime_started_by_reviewer']=False
    for item in capture['source_bindings']:
        if sha(item['path'])!=item['sha256']:raise RuntimeError('Frozen event source binding changed: '+item['role'])
    for role,path in [('diagnostic_stop_owner',HERE/('r85_preflight.py' if r85 else 'first_goal_stop.py')),
                      ('sensor_contract',run/'sensor_contract.json')]:
        capture['source_bindings'].append(dict(role=role,path=str(path.resolve()),sha256=sha(path)))
    write_new(run/'known_scene_event_capture_contract.json',capture)
    if r85:
        from r85_preflight import prepare_task_inputs
        prepare_task_inputs(run,p,HERE)
    scope=module(HERE/'pid_scope.py','closed_loop_prepare_'+uuid.uuid4().hex)
    receipt=scope.prepare(run,p)
    ramp_contract=None
    if p.get('ramp_segments'):
        script=ROOT/'test_results/closed_loop_navigation_20261005/acceptance_audit/evaluate_ramp.py'
        segments=','.join(f"{s['model']}:{s['direction']}:{s['destination_goal_index']}"for s in p['ramp_segments'])
        run_command([str(SYSTEM_PYTHON),'-B',str(script),'--run',str(run),'--prepare-contract',
            '--segments',segments],run/'ramp_contract_prepare.log')
        ramp_contract=sha(run/'ramp_acceptance_contract.json')
    scope_runtime=json.loads((run/'navigation_scope.json').read_text()).get('runtime',{})
    expected_children=(14 if p.get('mission46_required') else EXPECTED_CHILDREN+1)-int(not p['engineering_recording']['raw_source_capture'])+1
    if r85 and p['r85_task']['kind']=='f1_to_f3':expected_children+=2
    if scope_runtime.get('required_owned_roles')!=list(REQUIRED_ROLES)or scope_runtime.get('expected_launch_children')!=expected_children:
        raise RuntimeError('New scope and runner process contracts disagree')
    binary_contract={name:{'path':str(path.resolve()),'sha256':sha(path)} for name,path in (
        ('executable',SLAM_WORKSPACE/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping'),
        ('core',SLAM_WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so'))}
    detail_window=p['diagnostic_scope']['detail_window_sim_s']
    diagnostic_environment={'FASTLIVO_DIAGNOSTIC_DIR':str(run/'fastlivo_diagnostics'),
                            'FASTLIVO_DIAGNOSTIC_BEGIN':str(detail_window[0]),
                            'FASTLIVO_DIAGNOSTIC_END':str(detail_window[1]),
                            'FASTLIVO_BOUNDARY_TIMING':'1' if p.get('boundary_timing',False) else '0',
                            'FASTLIVO_LIO_JACOBIAN_THREADS':'4',
                            'FASTLIVO_VIO_PATCH_THREADS':'1',
                            'FASTLIVO_PIPELINE_MODE':p['pipeline']['mode'],
                            'FASTLIVO_SERIAL_IMU_PRIORITY':'1' if PRIVATE_SLAM else '0',
                            'FASTLIVO_IMAGE_COPY_OPT':str(p['pipeline']['image_copy_opt']),
                            'FASTLIVO_SURFACE_VALIDITY':str(args.surface_validity),
                            'FASTLIVO_SURFACE_DIAG_WINDOWS':'0:15,209:215',
                            'FASTLIVO_SURFACE_DIAG_BOUNDS':'14:18,-1:3,0:1.5',
                            'FASTLIVO_SURFACE_DIAG_MAX_BYTES':str(64*1024*1024)}
    plan={'schema':'teacher_closed_loop_runtime_plan/v1','run':str(run),'simulation_only':True,'real_robot':False,
          'allowed':True,'navigation_ground_truth_used':False,'actor_privileged_dimensions':232,
          'profile_path':str(selected),'profile_sha256':sha(selected),'profile':p,
          'required_owned_roles':list(REQUIRED_ROLES),'expected_launch_children':expected_children,'scope_runtime':scope_runtime,
          'duration_s':duration,'real_time_factor':args.real_time_factor,'wall_budget_s':duration/args.real_time_factor*3+90,
          'ramp_contract_sha256':ramp_contract,
          'ros_domain':args.domain,'actor_python':str(args.cpu_python.resolve()),'system_python':str(SYSTEM_PYTHON),
          'actor_device':'cpu','actor_threads':1,'physics_step_s':.005,'Actor_hz':50,'native_PD_hz':200,
          'exclusive_writer':'teacher_sim::TeacherActuator','checkpoint_path':str(CHECKPOINT),'model_sha256':MODEL_SHA,
          'native_plugin_sha256':sha(native_path),'native_plugin_path':str(native_path),
           'protected_production_native_plugin_sha256':sha(ROOT/'simulation/build/libteacher_actuator.so'),
          'source_manifest_sha256':sha(run/'source_manifest.json'),'scope_sha256':receipt['sha256'],
          'world_sha256':sha(run/'world.sdf'),'cloud_transport_manifest_sha256':sha(run/'cloud_transport_manifest.json'),
          'run_storage_contract':json.loads((run/'run_storage_contract.json').read_text()),
          'runtime_started':False,'renderer':'hardware/Ogre2','compatibility_motion_gate_applies':False,
          'slam_pipeline_contract':json.loads((HERE/'PIPELINE_V19_CONTRACT.json').read_text()),
          'slam_multicore_contract':json.loads((HERE/'COMBINED_COMPUTE_CONTRACT.json').read_text()),
          'slam_logging_acceleration_contract':json.loads((HERE/'LOGGING_ACCELERATION_CONTRACT.json').read_text()),
          'slam_lockfree_contract':json.loads((HERE/'LOCKFREE_RESIDUAL_CONTRACT.json').read_text()),
          'slam_boundary_timing_contract':json.loads((HERE/'BOUNDARY_TIMING_CONTRACT.json').read_text()),
          'slam_binary_contract':binary_contract,'navigation_stack_diagnostic_environment':diagnostic_environment,
          'independent_SLAM_workspace':str(SLAM_WORKSPACE.resolve()),'SLAM_build_copied':False,
           'SLAM_existing_V32_build_reused_read_only':False,'SLAM_CPP_changed_by_V33':True,
            'R33_full_source_rebuilt_timestamp_codec':True,
           'engineering_contract':p['engineering_contract'],'engineering_recording':p['engineering_recording'],
          'surface_validity_experiment':{'enabled':bool(args.surface_validity), 'flag':args.surface_validity,
              'source_semantics':'new V32 binary both flags; baseline0 retains original plane acceptance',
              'backend_connected':False,'navigation_thresholds_changed':False,
              'prospective_math_gate':'N>=6, lambda_mid/lambda_max>=.05, sum(r_i^2/max(n^Tvar_i n,1e-8))/(N-3)<=4; original lambda_min<.005 preserved',
              'calibrated_chi_squared_claim':False,'guaranteed_old_plane_rejection':False},
          'scan_workspace_contract':{'path':str(scan_contract_path),'sha256':sha(scan_contract_path),
              'workspace':str(scan_workspace),'data':scan_contract_data},
          'scan_binary_contract':{'path':str((scan_workspace/'install/scan_planner/lib/scan_planner/scan_planner_node').resolve()),
              'sha256':sha(scan_workspace/'install/scan_planner/lib/scan_planner/scan_planner_node')},
          'diagnostic_prefix_only':p['path_admission_contract']['prefix_only'],
          'path_admission_contract':p['path_admission_contract'],
          'scalar_ipc_fault_window':dict(IPC_FAULT_WINDOW_CONTRACT, entrypoint=str(HERE/'ipc_fault_window.py'),
              entrypoint_sha256=sha(HERE/'ipc_fault_window.py'), artifact=str(run/'ipc_fault_window.jsonl')),
          'scalar_original_request_substages':dict(REQUEST_SUBSTAGE_CONTRACT, entrypoint=str(HERE/'ipc_request_window.py'),
              entrypoint_sha256=sha(HERE/'ipc_request_window.py'), artifact=str(run/'ipc_request_substages.jsonl')),
          'runtime_commands':{'worker':[str(args.cpu_python.resolve()),'-B',str(HERE/'ipc_request_window.py'),'--run',str(run)],
                              'navigation_stack':['ros2','launch',str(HERE/'stack.launch.py'),'run_dir:='+str(run)]},
          'limitations':'Plan is launch provenance only, never a route/SLAM/Sim2Sim pass'}
    if PRIVATE_SLAM:
        plan.update(independent_SLAM_source_root=str(SLAM_SOURCE_ROOT),
            SLAM_existing_V32_build_reused_read_only=False,SLAM_CPP_changed_by_V33=False,
            R33_full_source_rebuilt_timestamp_codec=False,SLAM_private_full_source_build=True,
            SLAM_runtime_acceptance_inherited=False,
            slam_native_transport_variant=dict(schema='serial_imu_priority_transport_selection/v1',
                selection=dict(PRIVATE_SLAM[0]),native_source_commit=(PRIVATE_SLAM[1]['original_native_commit'] if PRIVATE_SLAM[1].get('schema')=='published_formal22_native571_build/v1' else PRIVATE_SLAM[1]['candidate_commit']),
                environment_flag='FASTLIVO_SERIAL_IMU_PRIORITY',environment_value='1',mode='serial',
                build_manifest_is_not_runtime_PASS=True,actual_loaded_witness_required=True,
                actual_group_INIT_required=True,new_source_root=True))
        plan['surface_validity_experiment']['source_semantics']='Private rebuilt R73 source; original surface algorithm and flag unchanged'
    contract_path=getattr(args,'shadow_sample_contract',None)
    if contract_path is not None:
        contract_path=Path(contract_path).resolve()
        if contract_path.stat().st_size>65536:raise ValueError('Oversized bounded sampling contract')
        c=json.loads(contract_path.read_text())
        if c.get('schema')!='go2_R6_bounded_eight_observation_contract/v1' or c['disk_contract']['total_allocated_and_logical_cap_bytes']!=24*1024**2:
            raise ValueError('Exact user-approved24MiB shadow contract required')
        if c.get('control_allowed')is not False:raise ValueError('Shadow must not control')
        plan['bounded_shadow_sampling']={'contract_path':str(contract_path),'contract_sha256':sha(contract_path),'groups':[7,8],
            'maximum_total_bytes':24*1024**2,'collector_source_sha256':sha(HERE/'bounded_shadow_capture.py'),'control_allowed':False}
        plan['wall_budget_s']=min(plan['wall_budget_s'],480.)
    plan['event_aware_source_capture']=dict(contract_path=str(run/'known_scene_event_capture_contract.json'),contract_sha256=sha(run/'known_scene_event_capture_contract.json'),window_native_seconds=15,groups=3,total_logical_and_allocated_cap_bytes=16*1024**2,diagnostic_only=True,profile_diagnostic_metadata='historical2groups8MiB retained; effective event contract overrides capture diagnostics only',file_sha256_kind='exact_indented_file_bytes',group_contract_sha256_kind='canonical_sorted_compact_JSON_with_newline')
    plan['selected_policy_binding']=json.loads((run/'selected_policy_binding.json').read_text())
    if r85:
        from r85_session_guard import remaining_seconds,FINISH_RESERVE_S,work_deadline_withdrawn
        from policy_selection import STUDENT_CHECKPOINT_SHA,STUDENT_EXPORT_SHA
        plan.update(schema='student_pct_scan_closed_loop_runtime_plan/v1',
            actor_kind='student',navigation_controller_kind='student',actor_privileged_dimensions=30,
            actor_observation_dimensions=45,native_sensor_proxy_dimensions=30,
            inner_velocity_PID_privileged_native_feedback=True,
            checkpoint_path=None,training_checkpoint_sha256=STUDENT_CHECKPOINT_SHA,
            checkpoint_role='training identity only; actual runtime loads archived deterministic JIT export',
            export_path=json.loads((HERE/'newstudent_pid_contract.json').read_text())['export_origin'],
            actual_actor_artifact_path=json.loads((run/'selected_policy_binding.json').read_text())['student_export_snapshot'],
            model_sha256=STUDENT_EXPORT_SHA,export_sha256=STUDENT_EXPORT_SHA,
            legacy_teacher_checkpoint_role='unused actor; original observation/source provenance only',
            r85_task_contract_sha256=sha(run/'r85_task_contract.json'),
            old_R83_acceptance_inherited=False,old46_region_sequence_is_not_this_task=True)
        from diagnostic_observation import enabled as diagnostic_enabled
        from minimal_simulation_baseline import enabled as minimal_enabled
        if minimal_enabled(p):
            plan['wall_budget_s']=min(plan['wall_budget_s'],2400.)
            plan['R73_minimal_simulation']=dict(source_root=str(HERE),strict_acceptance_inherited=False,maximum_wall_s=2400)
        else:
            from diagnostic_observation import strict_formal_enabled
            if strict_formal_enabled(p):
                plan['wall_budget_s']=min(plan['wall_budget_s'],2400.)
                plan['R85_formal_bounded_simulation']=dict(source_root=str(HERE),strict_acceptance_inherited=False,maximum_wall_s=2400)
        preparation_reserve=60.
        withdrawn=work_deadline_withdrawn(p)
        if not withdrawn:
            plan['wall_budget_s']=min(plan['wall_budget_s'],remaining_seconds(p)-FINISH_RESERVE_S-preparation_reserve)
        user_deadline=p.get('diagnostic_observe_only',{}).get('user_authorized_work_deadline_utc')
        plan['self_imposed_work_deadline_applied']=not withdrawn and user_deadline is None
        if user_deadline is not None:
            plan['explicit_user_work_deadline_applied']=True
            plan['effective_work_deadline_utc']=user_deadline
        if not duration+120<=plan['wall_budget_s']<=4590:
            raise RuntimeError('Finite original per-run native/wall contract does not fit')
    else:
        plan['R33_original_startup']=dict(contract_path=str(HERE/'R33_SHORT_STARTUP_CONTRACT.json'),contract_sha256=sha(HERE/'R33_SHORT_STARTUP_CONTRACT.json'),max_native_seconds=startup['max_native_seconds'],max_wall_seconds=startup['max_wall_seconds'],new_output_cap_bytes=startup['new_output_cap_bytes'],original_profile_duration_preserved=duration,no_GT_carrier=True)
        plan['wall_budget_s']=float(startup['max_wall_seconds'])
        from mission46_profile import required_source_files as actual_mission_sources
        actual_sources=actual_mission_sources()
        declared_sources=p['mission46_required_source_files']
        if len(actual_sources)!=len(declared_sources):raise RuntimeError('R33 profile source relocation count differs')
        relocation=[]
        for declared,actual in zip(declared_sources,actual_sources):
            expected=HERE.with_name('engineering_v34_continuous_known_map')/actual.relative_to(HERE) if actual.is_relative_to(HERE) else actual
            if Path(declared).resolve()!=expected.resolve():raise RuntimeError('R33 unexpected original profile source path metadata')
            relocation.append(dict(original_profile_declared_path=declared,actual_executed_source_path=str(actual.resolve()),
                actual_source_sha256=sha(actual),original_declared_source_sha256=sha(declared)))
        plan['R33_original_profile_source_path_metadata_relocation']=relocation
        baseline_profile=json.loads((Path(startup['base_R36_source_root'])/'profiles/engineering_nav46.json').read_text())
        budget_expected=json.loads(json.dumps(baseline_profile))
        budget_expected['raw_storage_budget']['max_run_bytes']=130_000_000_000
        budget_expected['raw_budget_bytes']=130_000_000_000
        from parking_contract import SPEC as approved_ordinary_parking_SPEC
        budget_expected['ordinary_terminal_parking_contract']=approved_ordinary_parking_SPEC
        from navigation_stop_window import SPEC as approved_navigation_settle_SPEC
        budget_expected['navigation_settle_statistics_contract']=approved_navigation_settle_SPEC
        if p!=budget_expected:raise RuntimeError('R80 permits only explicit130GB budget and approved staged parking contract; original other profile fields must remain')
        plan['R33_original_profile_bytes_retained']=False
        plan['R80_original_profile_explicit_budget_and_parking_delta']={'raw_budget_bytes':[50_000_000_000,130_000_000_000],'raw_storage_budget.max_run_bytes':[50_000_000_000,130_000_000_000]}
        plan['R80_approved_parking_contract']=approved_ordinary_parking_SPEC
        plan['R81_approved_new_navigation_settle_statistics_contract']=approved_navigation_settle_SPEC
        plan['R81_original_continuous300ms_predicate_retained_and_not_relabelled']=True
        plan['R80_original_geometry_source_quality_and_safety_thresholds_preserved']=True
    write_new(run/'runtime_plan.json',plan)
    return plan


def ros_environment(scan_overlay=SCAN_OVERLAY):
    # SCAN's generated setup may source the old core underlay again. Apply the
    # independent overlay's local_setup LAST, without replaying its underlays.
    r=subprocess.run(['bash','-c','set -e; source "$1" >/dev/null; source "$2" >/dev/null; source "$3" >/dev/null; source "$4" >/dev/null; env -0',
        'closed-loop-ros-env',str(ROS_BASE),str(SLAM_UNDERLAY),str(scan_overlay),str(SLAM_OVERLAY)],check=True,capture_output=True)
    return dict(x.decode().split('=',1)for x in r.stdout.split(b'\0')if b'='in x)


def identity(pid):
    try:
        raw=Path(f'/proc/{pid}/stat').read_text();fields=raw[raw.rfind(')')+2:].split()
        return {'pid':pid,'state':fields[0],'ppid':int(fields[1]),'pgid':int(fields[2]),'session':int(fields[3]),'start_ticks':int(fields[19])}
    except (OSError,ValueError):return None


def group_members(pgid):
    rows=[]
    for p in Path('/proc').iterdir():
        if p.name.isdigit():
            r=identity(int(p.name))
            if r and r['pgid']==pgid and r['state']!='Z':rows.append(r)
    return rows


def verify_loaded_slam(run,plan,navigation_identity):
    """Witness the launched child's actual executable and mapped core before physics.

    Scope hashing alone does not prove which shared library the dynamic loader
    selected. No process outside this runner's saved navigation group is used.
    Any old/different core is a protected failure, never a fallback.
    """
    expected=plan['slam_binary_contract'];deadline=time.monotonic()+30
    receipt={'schema':'teacher_v7_loaded_slam_binary/v1','run':str(run),
             'expected':expected,'navigation_group_identity':navigation_identity,
             'verified':False,'simulation_only':True,'before_physics':True}
    try:
        while time.monotonic()<deadline:
            if plan['profile'].get('r85_session',{}).get('enabled'):
                current=identity(navigation_identity['pid'])
                if not current or current.get('state')=='Z':
                    errors=[line for line in (run/'navigation_stack.log').read_text(errors='replace').splitlines()if '[ERROR]'in line]
                    raise RuntimeError('Owned navigation_stack exited before binary witness: '+(errors[-1]if errors else'no launch ERROR line'))
            candidates=[]
            for member in group_members(navigation_identity['pgid']):
                try:
                    exe_raw=os.readlink(f"/proc/{member['pid']}/exe")
                    if Path(exe_raw.removesuffix(' (deleted)')).name=='fastlivo_mapping':
                        candidates.append((member,exe_raw))
                except (OSError,ValueError):continue
            if len(candidates)>1:raise RuntimeError('Multiple mapping executables in owned navigation group')
            if not candidates:
                time.sleep(.05);continue
            member,exe_raw=candidates[0];pid=member['pid']
            receipt.update(mapping_identity=member,actual_executable_path=exe_raw)
            if exe_raw.endswith(' (deleted)')or str(Path(exe_raw).resolve())!=expected['executable']['path']:
                raise RuntimeError('Mapping executable is not the frozen V7 independent binary')
            maps_raw=Path(f'/proc/{pid}/maps').read_bytes();core_rows=[];loaded_paths=set()
            for line in maps_raw.decode().splitlines():
                fields=line.split(maxsplit=5)
                if len(fields)==6 and 'libfast_livo2_core.so' in Path(fields[5].removesuffix(' (deleted)')).name:
                    core_rows.append(line);loaded_paths.add(fields[5])
            if not loaded_paths:
                time.sleep(.05);continue
            receipt.update(core_mapping_rows=core_rows,loaded_core_paths=sorted(loaded_paths),
                           process_maps_sha256=hashlib.sha256(maps_raw).hexdigest())
            if len(loaded_paths)!=1:raise RuntimeError('More than one core library mapped by V7 executable')
            core_raw=next(iter(loaded_paths))
            if core_raw.endswith(' (deleted)')or str(Path(core_raw).resolve())!=expected['core']['path']:
                raise RuntimeError('Protected failure: V7 mapping loaded the old or an unexpected core')
            actual_exe_sha=sha(f'/proc/{pid}/exe');actual_core_sha=sha(core_raw)
            receipt.update(actual_executable_sha256=actual_exe_sha,actual_core_sha256=actual_core_sha)
            if actual_exe_sha!=expected['executable']['sha256']or actual_core_sha!=expected['core']['sha256']:
                raise RuntimeError('Actual mapping/core binary differs from frozen preparation hash')
            if PRIVATE_SLAM:
                with (run/'navigation_stack.log').open('r',errors='replace')as stream:startup=stream.read(2*1024*1024)
                values=re.findall(r'\[ INIT \] rgb_img publisher created, node=laserMapping serial_imu_priority=([01])',startup)
                if not values:time.sleep(.05);continue
                if values!=['1']:raise RuntimeError('Actual native IMU callback-group selection differs')
                receipt['actual_serial_imu_priority_group']=True
            mode_keys=('FASTLIVO_SERIAL_IMU_PRIORITY','FASTLIVO_SURFACE_VALIDITY','FASTLIVO_SURFACE_DIAG_WINDOWS','FASTLIVO_SURFACE_DIAG_BOUNDS','FASTLIVO_SURFACE_DIAG_MAX_BYTES')
            launch_environment=json.loads((run/'navigation_stack_effective_environment.json').read_text())
            if launch_environment.get('DEMO_RUN_DIR')!=str(run.resolve()):raise RuntimeError('Owned launch DEMO_RUN_DIR differs from this run')
            receipt.update(source_bound_run_dir=str(run.resolve()),launch_environment_receipt_file=str((run/'navigation_stack_effective_environment.json').resolve()),launch_environment_receipt_sha256=sha(run/'navigation_stack_effective_environment.json'))
            actual_mode={key:launch_environment.get(key) for key in mode_keys}
            expected_mode={key:plan['navigation_stack_diagnostic_environment'][key] for key in mode_keys}
            receipt.update(actual_surface_launch_environment=actual_mode,expected_surface_environment=expected_mode,
                source_bound_launch_environment_verified=actual_mode==expected_mode,
                process_environment_directly_read=False,
                mode_provenance='Owned launch checks exact environment before Node creation; child inherits it, frozen source changes only DEMO_RUN_DIR')
            if actual_mode!=expected_mode:raise RuntimeError('Owned SLAM launch mode differs from frozen experiment')
            after=identity(pid)
            if not after or after['start_ticks']!=member['start_ticks']or after['pgid']!=navigation_identity['pgid']:
                raise RuntimeError('Mapping process identity changed during binary witness')
            receipt.update(verified=True,witness_monotonic_wall_ns=time.monotonic_ns())
            write_new(run/'slam_loaded_binary.json',receipt);return receipt
        raise TimeoutError('V7 loaded mapping/core witness unavailable within30wall seconds')
    except Exception as error:
        receipt.update(error=f'{type(error).__name__}: {error}',witness_monotonic_wall_ns=time.monotonic_ns())
        write_new(run/'slam_loaded_binary.json',receipt)
        raise


def verify_loaded_scan(run,plan,navigation_identity):
    """Recheck the frozen planner executable in this owned group before physics."""
    expected=plan['scan_binary_contract'];deadline=time.monotonic()+30
    receipt=dict(schema='teacher_v20_loaded_SCAN_binary/v1',run=str(run),expected=expected,
        navigation_group_identity=navigation_identity,verified=False,before_physics=True)
    try:
        while time.monotonic()<deadline:
            if plan['profile'].get('r85_session',{}).get('enabled'):
                current=identity(navigation_identity['pid'])
                if not current or current.get('state')=='Z':
                    errors=[line for line in (run/'navigation_stack.log').read_text(errors='replace').splitlines()if '[ERROR]'in line]
                    raise RuntimeError('Owned navigation_stack exited before binary witness: '+(errors[-1]if errors else'no launch ERROR line'))
            candidates=[]
            for member in group_members(navigation_identity['pgid']):
                try:
                    raw=os.readlink(f"/proc/{member['pid']}/exe")
                    if Path(raw.removesuffix(' (deleted)')).name=='scan_planner_node':candidates.append((member,raw))
                except (OSError,ValueError):continue
            if len(candidates)>1:raise RuntimeError('Multiple SCAN planners in owned navigation group')
            if not candidates:time.sleep(.05);continue
            member,raw=candidates[0];pid=member['pid'];actual=sha(f'/proc/{pid}/exe')
            receipt.update(planner_identity=member,actual_executable_path=raw,actual_executable_sha256=actual)
            if raw.endswith(' (deleted)')or str(Path(raw).resolve())!=expected['path']or actual!=expected['sha256']:
                raise RuntimeError('Actual SCAN planner differs from the frozen selected workspace')
            after=identity(pid)
            if not after or after['start_ticks']!=member['start_ticks']or after['pgid']!=navigation_identity['pgid']:
                raise RuntimeError('SCAN planner identity changed during binary witness')
            receipt.update(verified=True,witness_monotonic_wall_ns=time.monotonic_ns())
            write_new(run/'scan_loaded_binary.json',receipt);return receipt
        raise TimeoutError('Owned SCAN planner witness unavailable within30wall seconds')
    except Exception as error:
        receipt.update(error=type(error).__name__+': '+str(error));write_new(run/'scan_loaded_binary.json',receipt)
        raise


def stop_owned(proc,saved,parent_first=False,*,absolute_deadline=None):
    if proc is None:return {'started':False}
    if saved is None:return {'started':True,'identity_unavailable':True,'sent_signals':[],'returncode':proc.poll()}
    started=time.monotonic()
    deadline=started+(23. if parent_first else 13.)  # Existing INT/TERM/KILL wait total.
    if absolute_deadline is not None:
        if type(absolute_deadline) not in (int,float) or not math.isfinite(absolute_deadline):
            raise ValueError('Owned cleanup absolute deadline must be finite')
        deadline=min(deadline,absolute_deadline)
    sent=[];parent=identity(proc.pid)
    if saved.get('ownership_token'):
        from owned_guardian import validated_members
        validated_members(saved)
    if parent and parent['start_ticks']!=saved['start_ticks']:
        return {'identity_mismatch':True,'sent_signals':[],'returncode':proc.poll()}
    for sig,timeout in ((signal.SIGINT,15 if parent_first else 5),(signal.SIGTERM,5),(signal.SIGKILL,3)):
        if not group_members(saved['pgid']):break
        if saved.get('ownership_token'):validated_members(saved)
        try:
            if parent_first and sig==signal.SIGINT and proc.poll()is None:os.kill(proc.pid,sig)
            else:os.killpg(saved['pgid'],sig)
            sent.append(int(sig));until=min(time.monotonic()+timeout,deadline)
            while time.monotonic()<until:
                proc.poll()
                if not group_members(saved['pgid']):break
                time.sleep(min(.1,max(0.,until-time.monotonic())))
        except ProcessLookupError:break
    remaining=group_members(saved['pgid'])
    confirmation=dict(deadline_monotonic=deadline,wait_attempted=False,timed_out=False)
    if not remaining:
        # Empty non-zombie group and a reaped Popen are separate observations.
        # Revalidate the same owner before waiting; never manufacture an exit code.
        remaining=validated_members(saved) if saved.get('ownership_token') else group_members(saved['pgid'])
        current=identity(proc.pid)
        if current and any(current.get(k)!=saved.get(k) for k in ('pid','start_ticks','pgid','session')):
            return {'identity_mismatch':True,'sent_signals':sent,'returncode':proc.poll()}
        if not remaining and proc.poll() is None:
            remaining_s=max(0.,deadline-time.monotonic())
            if remaining_s>0.:
                confirmation['wait_attempted']=True
                try:proc.wait(timeout=remaining_s)
                except subprocess.TimeoutExpired:confirmation['timed_out']=True
                except ChildProcessError as error:confirmation['error']=type(error).__name__+': '+str(error)
        if saved.get('ownership_token'):validated_members(saved)
    # Refresh after the final group scan/wait rather than returning a stale cache.
    remaining=validated_members(saved) if saved.get('ownership_token') else group_members(saved['pgid'])
    rc=proc.poll()
    if confirmation.get('error'):rc=None  # A failed reap is not an observed exit code.
    confirmation.update(ended_monotonic_wall=time.monotonic(),returncode=rc)
    return {'started':True,'sent_signals':sent,'returncode':rc,'remaining_owned_group_members':remaining,
            'returncode_confirmation':confirmation}


def resource(pids=()):
    mem={}
    for line in Path('/proc/meminfo').read_text().splitlines():
        k,v=line.split(':',1)
        if k in ('MemTotal','MemAvailable'):mem[k]=int(v.split()[0])
    d={'monotonic_wall':time.monotonic(),'logical_cpus':os.cpu_count(),'load_1_5_15':list(os.getloadavg()),'memory_KiB':mem}
    try:
        raw=subprocess.check_output(['nvidia-smi','--query-gpu=memory.used,memory.total,utilization.gpu','--format=csv,noheader,nounits'],text=True,timeout=5)
        d['gpu_raw']=raw.strip();a=raw.strip().splitlines()[0].split(',');d['gpu']={'used_MiB':float(a[0]),'total_MiB':float(a[1]),'utilization_percent':float(a[2])}
    except (OSError,subprocess.SubprocessError,ValueError)as e:d['gpu_error']=repr(e)
    if pids:
        d['owned_processes']=subprocess.run(['ps','-p',','.join(map(str,pids)),'-o','pid,ppid,%cpu,%mem,rss,args'],capture_output=True,text=True).stdout
    return d


def resource_guard(d):
    if d['memory_KiB']['MemAvailable']<4*1024*1024:raise RuntimeError('Less than4GiB free RAM; only own run will be stopped')
    if d['load_1_5_15'][0]>.9*d['logical_cpus']:raise RuntimeError('Shared CPU load guard exceeded; only own run will be stopped')
    if 'gpu'not in d:raise RuntimeError('Cannot verify required renderer GPU headroom')
    gpu=d['gpu']
    if gpu['used_MiB']>min(14000,gpu['total_MiB']-2048):raise RuntimeError('Less than audited GPU headroom; only own renderer will be stopped')


def require_private_domain(domain):
    # Query a private DDS graph without reading any process environment.
    probe=ros_environment();probe['ROS_DOMAIN_ID']=str(domain)
    result=subprocess.run(['ros2','node','list','--no-daemon'],env=probe,capture_output=True,text=True,timeout=10)
    if result.returncode or result.stdout.strip():
        raise RuntimeError('Private DDS domain query failed or existing nodes are present: '+result.stdout[-400:]+result.stderr[-400:])


def verify_loaded_native(run,plan,owned):
    deadline=time.monotonic()+15;expected=Path(plan['native_plugin_path']).resolve()
    while time.monotonic()<deadline:
        paths=set();proof=[]
        for member in group_members(owned['pgid']):
            try:raw=Path('/proc',str(member['pid']),'maps').read_text()
            except OSError:continue
            rows=[line for line in raw.splitlines() if 'libteacher_actuator.so' in line]
            for line in rows:
                fields=line.split(maxsplit=5)
                if len(fields)==6:paths.add(fields[5])
            if rows:proof.append(dict(process=member,actual_map_rows=rows,maps_sha256=hashlib.sha256(raw.encode()).hexdigest()))
        if paths:
            if paths!={str(expected)} or sha(expected)!=plan['native_plugin_sha256']:
                raise RuntimeError('Owned Gazebo loaded unexpected native actuator')
            contract_line=None
            log=run/'actuator.jsonl'
            if log.is_file():
                with log.open() as f:
                    for _ in range(8):
                        line=f.readline()
                        if not line:break
                        row=json.loads(line)
                        if row.get('kind')=='actuator_contract':contract_line=row;break
            if contract_line is not None:
                if contract_line.get('diagnostic_native_collision_guard')!='actual_ContactSensorData_200Hz_v1':
                    raise RuntimeError('Actual native actuator lacks its frozen contact guard')
                receipt=dict(schema='R33_actual_native_loaded_guard/v1',verified=True,actual_library_path=str(expected),
                    actual_library_sha256=sha(expected),owned_identity=owned,actual_process_maps=proof,
                    actual_contract_line=contract_line,source_reused_unchanged=True,no_GT_carrier=True)
                write_new(run/'R33_native_loaded_guard.json',receipt);return receipt
        time.sleep(.05)
    raise TimeoutError('R33 actual native loading/contact marker witness unavailable')


def launch_child_receipt(log,expected=EXPECTED_CHILDREN):
    text=Path(log).read_text(errors='replace')if Path(log).exists()else''
    started=re.findall(r'\[([^\[\]]+)\]: process started with pid \[(\d+)\]',text)
    clean={int(x)for x in re.findall(r'process has finished cleanly \[pid (\d+)\]',text)}
    died=re.findall(r'process has died \[pid (\d+), exit code ([^,\]]+)',text)
    unique={int(pid):action for action,pid in started}
    rows=[{'pid':pid,'action':name,'clean_exit':pid in clean}for pid,name in unique.items()]
    return {'expected_children':expected,'started_children':len(unique),'children':rows,'died':died,
            'all_expected_children_clean':len(unique)==expected and len(started)==len(unique)and not died and set(unique)<=clean}


def execute(run,plan):
    from diagnostic_observation import enabled as diagnostic_enabled,record as diagnostic_record
    overview_exit_recorded=False;overview_missing_recorded=False
    diagnostic_config=plan['profile'].get('diagnostic_observe_only',{})
    capture_requested=not (isinstance(diagnostic_config,dict)
        and diagnostic_config.get('recording_requested') is False)
    actual_expected_roles=set((*REQUIRED_ROLES,*(['shadow_sampler']if plan.get('bounded_shadow_sampling')else[])))
    if not capture_requested:actual_expected_roles.remove('capture')
    env=ros_environment(Path(plan['scan_workspace_contract']['workspace'])/'install/setup.bash');transport=json.loads((run/'cloud_transport_manifest.json').read_text())
    for k in transport['remove_environment_keys']:env.pop(k,None)
    env.update(transport['environment'])
    for k in plan['navigation_stack_diagnostic_environment']:
        env.pop(k,None)  # Diagnostic settings belong only to navigation_stack.
    sock='/tmp/teacher_closed_loop_'+uuid.uuid4().hex[:12]+'.sock'
    env.update(ROS_DOMAIN_ID=str(plan['ros_domain']),GZ_IP='127.0.0.1',GZ_PARTITION='teacher_closed_loop_'+run.name,
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'simulation/build'),TEACHER_SOCKET=sock,TEACHER_ACTUATOR_LOG=str(run/'actuator.jsonl'),
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',ROS_LOG_DIR=str(run/'ros_logs'))
    env.pop('LIBGL_ALWAYS_SOFTWARE',None);env.pop('GALLIUM_DRIVER',None)
    selected_keys=('ROS_DOMAIN_ID','ROS_LOG_DIR','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION',
                   'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','GZ_PARTITION','GZ_IP','DISPLAY',
                   'LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS',
                   'AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH')
    write_new(run/'effective_environment.json',{k:env.get(k)for k in selected_keys})
    from storage_guard import observe as storage_observe, Watchdog
    from r100_resource_observer import Observer as ResourceObserver
    from r100_resource_stop import checked_resource_guard,cpu_resource_stop,archive_budget,fresh_motion_stop_reason
    resource_observer=ResourceObserver()
    before=resource();before['r100_resource_observer']=resource_observer.sample()
    before['storage']=storage_observe(run,plan['profile'],starting=True,count_files=True);write_new(run/'resources_before.json',before)
    children={};identities={};handles=[];cleanup={};error=None;first_failure=None;sensors_ready=False;natural_timeout=False;loaded_slam=None;loaded_scan=None;guardian=None;sample_complete=False;startup_stop=None;native_last_ns=0;native_tail=None
    cpu_stop_evidence=None;archive_deadlines=dict(runner=None,guardian=None,hard_utc=None)
    try:
        utc_deadline=dt.datetime.fromisoformat(plan['profile']['r85_session']['deadline_utc'])
        if utc_deadline.tzinfo is not None:
            archive_deadlines['hard_utc']=time.monotonic()+(utc_deadline-dt.datetime.now(dt.timezone.utc)).total_seconds()
    except (KeyError,TypeError,ValueError):pass  # Missing bound denies only the new CPU grace.
    stream=(run/'owned_resources.jsonl').open('x',buffering=1)
    def start(role,command,child_env=None):
        log=(run/(role+'.log')).open('x');handles.append(log)
        child,saved=guardian.start_role(role,command,env if child_env is None else child_env,log)
        children[role]=child;identities[role]=saved
        if identities[role]is None:raise RuntimeError('Could not bind owned process identity '+role)
        return child
    def check_started_dependencies(message):
        nonlocal overview_exit_recorded
        for role,child in children.items():
            if child.poll() is None:continue
            if role=='capture' and diagnostic_enabled(plan['profile']):
                # Only a role already admitted by Guardian.start_role may be
                # observed here. Registration/ownership failures above still
                # raise; no unowned child or invented identity is tolerated.
                if identities.get(role) is None:raise RuntimeError('Unregistered overview capture dependency')
                if not overview_exit_recorded:
                    diagnostic_record(run,plan['profile'],'overview_capture_process',
                        'original_overview_capture_exit_would_abort_navigation',
                        dict(role=role,returncode=child.returncode,phase=message,
                            actual_monotonic_wall=time.monotonic(),observer_only=True,
                            overview_used_for_vio=False,actual_sensor_and_required_process_guards_changed=False))
                    overview_exit_recorded=True
            else:raise RuntimeError(message)
    def interrupted(sig,frame):raise InterruptedError('Runner interrupted; cleanup is restricted to saved owned process groups')
    old_handlers={s:signal.signal(s,interrupted)for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    watchdog=None
    try:
        watchdog=Watchdog(run,plan['profile'],lambda:os.kill(os.getpid(),signal.SIGTERM)).start()
        if plan['profile'].get('r85_session',{}).get('enabled'):
            sys.path.insert(0,str(HERE))
            import owned_guardian as actual_guardian_module
            if Path(actual_guardian_module.__file__).resolve()!=HERE/'owned_guardian.py':
                raise RuntimeError('R85 Guardian module source differs from candidate')
            write_new(run/'r85_runtime_module_resolution.json',dict(
                owned_guardian_file=str(Path(actual_guardian_module.__file__).resolve()),
                owned_guardian_sha256=sha(actual_guardian_module.__file__),
                candidate_module_priority_restored_after_legacy_storage_import=True))
        from owned_guardian import Guardian
        guardian=Guardian(run,plan)
        try:
            owner=json.loads((run/'guardian_owner.json').read_text())
            if (owner.get('token')==guardian.token and owner.get('parent')==guardian.parent
                    and owner.get('runtime_plan_sha256')==sha(run/'runtime_plan.json')):
                archive_deadlines['guardian']=owner.get('wall_deadline')
        except (OSError,ValueError,TypeError):pass
        checked_resource_guard(resource_guard,before,str(run))
        require_private_domain(plan['ros_domain'])
        scope=module(HERE/'pid_scope.py','closed_loop_execute_'+uuid.uuid4().hex)
        scope.verify_scope(run/'navigation_scope.json')
        if sha(run/'source_manifest.json')!=plan['source_manifest_sha256']or sha(run/'world.sdf')!=plan['world_sha256']:
            raise RuntimeError('Run preparation changed before execution')
        worker=start('worker',[plan['actor_python'],'-B',str(HERE/'ipc_request_window.py'),'--run',str(run),'--socket',sock],{**env,'CUDA_VISIBLE_DEVICES':''})
        deadline=time.monotonic()+30
        while not(run/'worker_ready').exists():
            if worker.poll()is not None:raise RuntimeError('CPU Actor exited before native IPC readiness')
            if time.monotonic()>deadline:raise TimeoutError('CPU Actor readiness exceeded30wall seconds')
            time.sleep(.05)
        start('bridge',[str(BRIDGE),'--ros-args','-r','__node:=ros_gz_bridge','-p','config_file:='+str(run/'sensor_bridge.yaml')])
        if capture_requested:
            start('capture',[str(SYSTEM_PYTHON),'-B',str(ROOT/'scripts/capture.py'),'--run',str(run),
                             '--topic','/demo/teacher/overview','--archive-stride',str(plan['profile']['engineering_recording']['overview_archive_stride'])])
        else:
            diagnostic_record(run,plan['profile'],'overview_capture_selection','capture_not_requested',
                dict(recording_requested=False,independent_overview_process_started=False,
                    original_required_owned_roles=list(REQUIRED_ROLES),
                    actual_expected_owned_roles=sorted(actual_expected_roles),
                    actual_ROS_GZ_camera_and_LIVO_sources_preserved=True,overview_used_for_vio=False))
        navigation_env={**env,'DEMO_RUN_DIR':str(run),**plan['navigation_stack_diagnostic_environment']}
        write_new(run/'navigation_stack_effective_environment.json',{
            k:navigation_env.get(k)for k in (*selected_keys,'DEMO_RUN_DIR',*plan['navigation_stack_diagnostic_environment'])})
        start('navigation_stack',['ros2','launch',str(HERE/'stack.launch.py'),'run_dir:='+str(run)],navigation_env)
        if plan.get('bounded_shadow_sampling'):
            capture=plan['bounded_shadow_sampling']
            if sha(capture['contract_path'])!=capture['contract_sha256']or sha(HERE/'bounded_shadow_capture.py')!=capture['collector_source_sha256']:
                raise RuntimeError('Bounded sampling input changed')
            start('shadow_sampler',[str(SYSTEM_PYTHON),'-B',str(HERE/'bounded_shadow_capture.py'),'--run',str(run),'--contract',capture['contract_path']],navigation_env)
        time.sleep(.5)
        check_started_dependencies('Owned dependency exited before physics started')
        loaded_slam=verify_loaded_slam(run,plan,identities['navigation_stack'])
        loaded_scan=verify_loaded_scan(run,plan,identities['navigation_stack'])
        check_started_dependencies('Owned dependency exited during V7 binary witness')
        gazebo=start('gazebo',['gz','sim','-s','-r',str(run/'world.sdf'),*(['--seed','42'] if plan['profile'].get('r85_session',{}).get('enabled') else [])])
        from known_scene_witness import record as record_known_scene_witness
        verify_loaded_native(run,plan,identities['gazebo'])
        from mission46_runtime_evidence import FileTail,native_projection,native_ns
        from startup_trace import startup_stop_decision
        from storage_guard import usage
        native_tail=FileTail(run/'telemetry.jsonl',project=native_projection)
        record_known_scene_witness(run,plan,identities,HERE,SLAM_WORKSPACE,source_root=SLAM_SOURCE_ROOT)
        write_new(run/'owned_processes_started.json',{role:{**identities[role],'role':role}for role in children})
        camera_deadline=time.monotonic()+30;wall_deadline=time.monotonic()+plan['wall_budget_s'];last_resource=-math.inf;last_storage=-math.inf
        archive_deadlines['runner']=wall_deadline
        while worker.poll()is None:
            now=time.monotonic()
            for role,child in children.items():
                if role=='shadow_sampler':
                    rc=child.poll()
                    if rc==0:
                        result=json.loads((run/'bounded_shadow_samples/RESULT.json').read_text())
                        if result.get('complete')is not True:raise RuntimeError('Sampler exit without complete receipt')
                        sample_complete=True
                    elif rc is not None:raise RuntimeError('Bounded shadow sampler incomplete; inspect its receipt')
                elif role not in ('worker','gazebo')and child.poll()is not None:
                    if role=='capture' and diagnostic_enabled(plan['profile']):
                        if not overview_exit_recorded:
                            diagnostic_record(run,plan['profile'],'overview_capture_process',
                                'original_overview_capture_exit_would_abort_navigation',
                                dict(role=role,returncode=child.returncode,actual_monotonic_wall=now,
                                    observer_only=True,overview_used_for_vio=False,
                                    actual_sensor_and_required_process_guards_changed=False))
                            overview_exit_recorded=True
                    else:raise RuntimeError(role+' exited while Actor remained active')
            if sample_complete:break
            from first_goal_stop import observe_native_tail,first_goal_runtime_decision
            observed_native=observe_native_tail(native_tail,native_last_ns)
            native_last_ns=observed_native['native_ns']
            current_run_storage=usage([run])
            if plan['profile'].get('r85_session',{}).get('enabled') is True:
                from r85_preflight import runtime_stop_decision
                decision=runtime_stop_decision(run,observed_native)
                if decision['stop']:
                    startup_stop=decision
                    write_new(run/'R85_TASK_STOP.json',dict(decision,actual_native_ns=native_last_ns,actual_monotonic_wall=now))
                    break
            else:
                decision=first_goal_runtime_decision(run,observed_native,
                    now-(wall_deadline-plan['wall_budget_s']),max(current_run_storage['allocated_bytes'],current_run_storage['logical_bytes']))
                if decision['stop']:
                    startup_stop=decision;write_new(run/'R33_DIAGNOSTIC_STOP.json',dict(decision,actual_native_ns=native_last_ns,actual_monotonic_wall=now));break
            sensors_ready=(run/'frame_source.json').exists()
            if capture_requested and now>camera_deadline and not sensors_ready:
                if diagnostic_enabled(plan['profile']):
                    if not overview_missing_recorded:
                        diagnostic_record(run,plan['profile'],'overview_capture_frame',
                            'original_overview_frame_timeout_would_abort_navigation',
                            dict(original_wait_wall_s=30.,actual_monotonic_wall=now,frame_source_present=False,
                                observer_only=True,overview_used_for_vio=False,
                                actual_sensor_and_required_process_guards_changed=False))
                        overview_missing_recorded=True
                else:raise TimeoutError('Actual overview camera failed to publish in30wall seconds')
            if now-last_resource>=2:
                sample=resource([p.pid for p in children.values()if p.poll()is None])
                sample['r100_resource_observer']=resource_observer.sample(identities)
                count_storage=now-last_storage>=1
                sample['storage']=storage_observe(run,plan['profile'],count_files=count_storage)
                if count_storage:last_storage=now
                stream.write(json.dumps(sample)+'\n');checked_resource_guard(resource_guard,sample,str(run));last_resource=now
            if gazebo.poll()is not None:
                worker.wait(timeout=10);break
            if now>wall_deadline:raise TimeoutError('Owned closed-loop run exceeded frozen wall budget')
            time.sleep(.2)
        if plan['profile'].get('r85_session',{}).get('enabled') is True and native_tail is not None:
            # The worker can finish between polls. Recheck only original native
            # fault events before accepting any endpoint/completion observation;
            # shutdown EOF or native pose is not a failure cause or nav input.
            from first_goal_stop import observe_native_tail
            from r85_preflight import runtime_stop_decision
            terminal_native=observe_native_tail(native_tail,native_last_ns)
            native_last_ns=terminal_native['native_ns']
            if terminal_native['fault_record'] is not None:
                startup_stop=runtime_stop_decision(run,terminal_native)
                write_new(run/'R85_NATIVE_FAULT_PRIORITY_STOP.json',dict(startup_stop,
                    actual_native_ns=native_last_ns,actual_monotonic_wall=time.monotonic(),
                    prior_stop_record_preserved=(run/'R85_TASK_STOP.json').exists(),
                    check_before_pipeline_shutdown=True,native_pose_used_for_navigation=False))
        if startup_stop:
            error=('R85 actual task stop: 'if plan['profile'].get('r85_session',{}).get('enabled')else'R33 intentional diagnostic stop: ')+startup_stop['reason']+'; no autonomous route PASS'
        elif sample_complete:
            error='Bounded shadow data collected; navigation horizon intentionally interrupted, no navigation PASS'
        else:
            if worker.returncode!=0:raise RuntimeError('CPU Teacher worker returned nonzero')
            result=json.loads((run/'worker_result.json').read_text())
            if result.get('fault')is not None:error='Actor/native physical fault: '+str(result['fault'])
            try:gazebo.wait(timeout=8)
            except subprocess.TimeoutExpired:natural_timeout=True;error=error or 'Gazebo failed natural exit after done/damping'
    except Exception as e:
        cpu_stop_evidence=cpu_resource_stop(e,str(run),resource_guard)
        error=f'{type(e).__name__}: {e}';first_failure=failure_record(e,'runtime')
    finally:
        cleanup_started=time.monotonic()
        cleanup_deadline=min([cleanup_started+300.]+[v for v in archive_deadlines.values()
            if type(v) in (int,float) and math.isfinite(v)])
        if error and first_failure is None:first_failure=failure_record(error,'runtime_outcome')
        write_failure_evidence(run,'runtime_failure_context.json',dict(first_failure=first_failure,
            runtime_started=bool(children),owned_processes=dict(identities),
            monotonic_ns=time.monotonic_ns(),phase='before_original_cleanup'))
        for s in old_handlers:signal.signal(s,signal.SIG_IGN)
        from pipeline_lifecycle import request_normal_stop
        pipeline_stop=request_normal_stop(run,loaded_slam,identities.get('navigation_stack',{}),identity)
        if not pipeline_stop['normal_completed']:error=error or 'V19 normal pipeline drain failed: '+str(pipeline_stop['error'])
        for role in ('shadow_sampler','navigation_stack','capture','bridge','gazebo'):
            cleanup[role]=stop_owned(children.get(role),identities.get(role),parent_first=role=='navigation_stack',
                                     absolute_deadline=cleanup_deadline)
        # Native physics is stopped before waiting for the unmodified Actor's
        # normal EOF/final archive; hard gates and original escalation remain.
        from worker_archive_shutdown import await_archive_close,inspect_worker_archive
        from owned_guardian import validated_members,ident as worker_archive_identity
        cpu_archive_budget=archive_budget(cpu_stop_evidence,now=time.monotonic(),
            cleanup_started=cleanup_started,runner_deadline=archive_deadlines['runner'],
            guardian_deadline=archive_deadlines['guardian'],hard_utc_deadline=archive_deadlines['hard_utc'])
        def archive_abort_reason():
            normal_diagnostic=bool(startup_stop and startup_stop.get('reason')in(
                'actual_original_mission_failed','actual_original_navigation_failed','actual_R85_task_failed','actual_R85_navigation_status.json_failed','actual_R85_r85_task_status.json_failed'))
            diagnostic=plan['profile'].get('diagnostic_observe_only')
            if (not normal_diagnostic and isinstance(diagnostic,dict)
                    and diagnostic.get('enabled')is True and diagnostic.get('full_evidence_after_endpoint')is True
                    and isinstance(startup_stop,dict) and startup_stop.get('reason')=='actual_R85_diagnostic_terminated'):
                try:
                    execution_request=json.loads((run/'r85_execution_request.json').read_text())
                    task_contract=json.loads((run/'r85_task_contract.json').read_text())
                    normal_diagnostic=diagnostic_full_evidence_archive_allowed(plan['profile'],startup_stop,run,execution_request,task_contract)
                except (OSError,ValueError,TypeError):normal_diagnostic=False
            if not(cpu_archive_budget['eligible'] or normal_diagnostic or sample_complete or children.get('worker')is not None and children['worker'].poll()is not None):
                return 'exceptional/hard shutdown uses original immediate escalation'
            if not pipeline_stop.get('normal_completed'):
                return 'original pipeline normal drain failed'
            for role,result in cleanup.items():
                if(result.get('remaining_owned_group_members')or result.get('identity_mismatch')
                        or result.get('identity_unavailable')or result.get('started')and result.get('returncode')!=0):
                    return 'original owned role cleanup incomplete: '+role
            if watchdog and watchdog.error:return 'storage watchdog: '+str(watchdog.error)
            if guardian and guardian.error:return 'owned guardian: '+str(guardian.error)
            if cpu_archive_budget['eligible']:
                if time.monotonic()>=cpu_archive_budget['deadline_monotonic']:
                    return 'CPU archive deadline or finish reserve exhausted'
                motion_reason=fresh_motion_stop_reason(cleanup,identities,validated_members)
                if motion_reason:return motion_reason
            if archive_deadlines['runner'] is None or time.monotonic()>=archive_deadlines['runner']:
                return 'original frozen wall deadline'
            return None
        worker_archive_close=await_archive_close(children.get('worker'),identities.get('worker'),
            cleanup.get('gazebo',{}),worker_archive_identity,validated_members,abort_reason=archive_abort_reason,
            maximum_wait_s=cpu_archive_budget['maximum_wait_s'] if cpu_stop_evidence is not None else 30.,
            absolute_deadline=cpu_archive_budget['deadline_monotonic'] if cpu_stop_evidence is not None else cleanup_deadline)
        worker_archive_close['original_cpu_stop']=cpu_stop_evidence.record() if cpu_stop_evidence is not None else None
        worker_archive_close['cpu_archive_budget']=cpu_archive_budget
        cleanup['worker']=stop_owned(children.get('worker'),identities.get('worker'),absolute_deadline=cleanup_deadline)
        worker_archive_integrity=inspect_worker_archive(run)if'worker'in children else None
        if worker_archive_integrity is not None and not worker_archive_integrity['verified']:
            error=error or 'Worker archive incomplete: '+str(worker_archive_integrity['error'])
        write_new(run/'worker_archive_close.json',worker_archive_close)
        write_new(run/'worker_archive_integrity.json',worker_archive_integrity)
        Path(sock).unlink(missing_ok=True);stream.close()
        for f in handles:f.close()
        try:guardian_receipt=guardian.close()if guardian else None
        except Exception as e:
            write_failure_evidence(run,'guardian_close_failure.json',failure_record(e,'guardian.close'))
            raise
        actual=[{'role':role,**(identities[role]or{'pid':child.pid,'identity_unavailable':True}),'returncode':child.returncode}for role,child in children.items()]
        child_proof=launch_child_receipt(run/'navigation_stack.log',expected=plan['expected_launch_children'])
        roles_complete=set(children)==actual_expected_roles
        clean=roles_complete and all(x['returncode']==0 for x in actual)and child_proof['all_expected_children_clean']and all(not x.get('remaining_owned_group_members')and not x.get('identity_mismatch')and not x.get('identity_unavailable')for x in cleanup.values())
        if not clean:error=error or 'Owned role/launch child cleanup incomplete or nonzero'
        manifest={**plan,'error':error,'first_failure':first_failure,'owned_processes':actual,'cleanup':cleanup,
             'independent_owned_guardian':guardian_receipt,'bounded_shadow_sampling_complete':sample_complete,
             'R33_actual_diagnostic_stop':startup_stop,'R33_last_actual_native_ns':native_last_ns,
             'worker_archive_close':worker_archive_close,'worker_archive_integrity':worker_archive_integrity,
            'slam_loaded_binary_verified':bool(loaded_slam and loaded_slam.get('verified')),
            'slam_loaded_binary_receipt_sha256':sha(run/'slam_loaded_binary.json')if(run/'slam_loaded_binary.json').is_file()else None,
            'scan_loaded_binary_verified':bool(loaded_scan and loaded_scan.get('verified')),
            'scan_loaded_binary_receipt_sha256':sha(run/'scan_loaded_binary.json')if(run/'scan_loaded_binary.json').is_file()else None,
            'pipeline_normal_stop_sha256':sha(run/'pipeline_normal_stop.json'),
            'pipeline_normal_completed':pipeline_stop['normal_completed'],
            'required_owned_roles_complete':roles_complete,'all_owned_and_children_clean':clean,
            'original_required_owned_roles':list(REQUIRED_ROLES),
            'actual_expected_owned_roles':sorted(actual_expected_roles),
            'overview_capture_requested':capture_requested,
            'overview_capture_selection':'original_capture_mode'if capture_requested else'explicitly_not_requested',
            'overview_frame_source_present':sensors_ready,
            'launch_child_exit_evidence':child_proof,'gazebo_partition':env['GZ_PARTITION'],
            'native_plugin_sha256':plan['native_plugin_sha256'],'plugin_sha256':plan['native_plugin_sha256'],
             'protected_production_native_plugin_sha256':sha(ROOT/'simulation/build/libteacher_actuator.so'),
             'R33_native_loaded_guard_verified':(run/'R33_native_loaded_guard.json').is_file() and json.loads((run/'R33_native_loaded_guard.json').read_text()).get('verified') is True,
             'R33_native_loaded_guard_receipt_sha256':sha(run/'R33_native_loaded_guard.json') if (run/'R33_native_loaded_guard.json').is_file() else None,
            'source_manifest_sha256':sha(run/'source_manifest.json'),'runtime_plan_sha256':sha(run/'runtime_plan.json'),
            'gazebo_natural_exit_grace_expired':natural_timeout,'actual_sensors_ready':sensors_ready,
            'runtime_started':bool(children),'runtime_status':'failed'if error else'completed_requires_independent_navigation_evaluation',
            'training_processes_signaled':False,'actor_started':'worker'in children,'navigation_controller_kind':plan['profile']['controller_kind'],
            'high_level_source':'Actual SLAM body pose and causal IMU gyro, actual registered cloud and SCAN checked path',
            'truth_navigation_used':False,'actual_feedback_hz':'Measured from unique source headers; nominal camera-driven source '+str(plan['profile']['sensor_sampling']['camera_hz'])+'Hz; unchanged20Hz wall ticker is not new feedback'}
        if watchdog:
            watchdog.close()
            manifest['storage_watchdog_error']=watchdog.error
            manifest['storage_last_observation']=watchdog.last
            if watchdog.error:
                manifest['error']=error=error or watchdog.error
                manifest['runtime_status']='failed'
        if plan['profile'].get('r85_session',{}).get('enabled'):
            manifest.update(R85_actual_task_stop=startup_stop,old_R83_acceptance_inherited=False,old46_PASS=False)
        write_new(run/'runtime_manifest.json',manifest)
        after=resource();after['r100_resource_observer']=resource_observer.sample(identities)
        write_new(run/'resources_after.json',after)
        write_new(run/'summary_closed_loop_navigation.json',{'schema':('r85_student_closed_loop_runtime_summary/v1'if plan['profile'].get('r85_session',{}).get('enabled')else'teacher_closed_loop_runtime_summary/v1'),
            'status':'failed'if error else'unverified','run':str(run),'runtime_error':error,
            'levels':{'runtime':'failed'if error else'completed','navigation':'unverified','multifloor':'unverified','Sim2Sim':'unverified','real_robot':'unverified'},
            'scope_sha256':plan['scope_sha256'],'reason':'Original-source/SCAN/arrival/active-hold/native offline independent evaluation required',
            'compatibility_stand_window_diagnostic_is_navigation_acceptance':False})
        for s,handler in old_handlers.items():signal.signal(s,handler)
    return error


def r85_main(args):
    from r85_session_guard import acquire,remaining_seconds,FINISH_RESERVE_S,work_deadline_withdrawn
    from run_storage import create_run,verify_run
    from storage_guard import register
    # The legacy budget proxy prepends its private dependency directory.
    # New R85 runtime modules must resolve to this source-bound candidate.
    sys.path.insert(0,str(HERE))
    profile=json.loads(profile_path(args.profile).read_text())
    validate_profile(profile)
    lock=None
    try:
        if not args.prepare_only:
            lock,ownership=acquire(profile)
        if args.prepared_run is not None:
            run=args.prepared_run.resolve();verify_run(run,ROOT)
            if (run/'runtime_manifest.json').exists()or(run/'worker_ready').exists():raise RuntimeError('R85 prepared run already attempted')
            plan=json.loads((run/'runtime_plan.json').read_text())
            if plan['profile']!=profile or plan['ros_domain']!=args.domain or plan['real_time_factor']!=args.real_time_factor:
                raise RuntimeError('R85 prepared invocation differs')
            module(HERE/'pid_scope.py','R85_prepared_source').verify_scope(run/'navigation_scope.json')
        else:
            label=''.join(x for x in args.label if x.isalnum()or x in '_-')or'R85'
            stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            run,_=create_run(ROOT,f'{stamp}_closed_loop_R85_{label}_{uuid.uuid4().hex[:4]}',args.run_storage_root)
            register(run)
            write_new(run/'runner_request.json',dict(schema='r85_closed_loop_runner_request/v1',run=str(run),
                argv=sys.argv,runner_sha256=sha(__file__),prepare_only=args.prepare_only,
                simulation_only=True,no_other_process_signals_authorized=True))
            try:plan=prepare(run,args)
            except Exception as e:
                failure=dict(schema='r85_prepare_failure/v1',run=str(run),status='failed',
                    error=type(e).__name__+': '+str(e),ROS_started=False,Gazebo_started=False,Actor_started=False)
                write_new(run/'prepare_failure.json',failure);write_new(run/'run_result.json',failure)
                print(json.dumps(failure,ensure_ascii=False),flush=True);raise
        if args.prepare_only:
            outcome=dict(run=str(run),status='prepared_unverified',ROS_started=False,Gazebo_started=False,
                runtime_plan_sha256=sha(run/'runtime_plan.json'),scope_sha256=plan['scope_sha256'])
        else:
            if not work_deadline_withdrawn(profile) and plan['wall_budget_s']>remaining_seconds(profile)-FINISH_RESERVE_S:
                raise RuntimeError('Prepared round no longer fits overnight finish reserve')
            write_new(run/'r85_physics_ownership.json',ownership)
            write_new(run/'r85_execution_request.json',dict(run=str(run),runner_sha256=sha(__file__),
                runtime_plan_sha256=sha(run/'runtime_plan.json'),seed=42,old_single_attempt_latch_used=False))
            try:error=execute(run,plan)
            except Exception as e:
                failure=exceptional_runtime_manifest(run,plan,e);error=failure['error']
                if not(run/'runtime_manifest.json').exists():
                    write_new(run/'runtime_manifest.json',failure)
            outcome=dict(run=str(run),status='failed'if error else'runtime_completed_navigation_unverified',
                runtime_error=error,runtime_manifest_sha256=sha(run/'runtime_manifest.json'))
        write_new(run/('r85_prepare_result.json'if args.prepare_only else'run_result.json'),outcome)
        print(json.dumps(outcome,ensure_ascii=False),flush=True)
        if outcome.get('runtime_error'):raise SystemExit(1)
    finally:
        if lock is not None:lock.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--profile',required=True,help='New closed-loop profile name or JSON path')
    ap.add_argument('--label',default='V28_capture_prefix270');ap.add_argument('--domain','--ros-domain',type=int,default=86)
    ap.add_argument('--cpu-python',type=Path,default=DEFAULT_CPU_PYTHON)
    ap.add_argument('--real-time-factor',type=float,default=1.)
    ap.add_argument('--surface-validity',type=int,choices=(0,1),required=True,help='Prospective V32 switch: 0 frozen acceptance baseline, 1 support-consistency candidate; captured before physics')
    ap.add_argument('--run-storage-root',type=Path,default=None,help='Optional dedicated V19 root declared in run_storage.py; default project/runs')
    ap.add_argument('--shadow-sample-contract',type=Path,default=None,help='User-approved whole-frame24MiB capture; groups7/8 only, no control adoption')
    ap.add_argument('--prepared-run',type=Path,default=None)
    ap.add_argument('--prepare-only','--no-launch',action='store_true');args=ap.parse_args();args.cpu_python=args.cpu_python.expanduser().resolve()
    if json.loads(profile_path(args.profile).read_text()).get('r85_session',{}).get('enabled') is True:
        return r85_main(args)
    launch_authority=json.loads((HERE/'R33_SHORT_STARTUP_CONTRACT.json').read_text())
    if not args.prepare_only and launch_authority.get('runtime_launch_authorized')is not True:
        raise RuntimeError('R44 preparation only: budget/runtime approval pending; no run may start')
    if not args.prepare_only:
        launch_latch=Path(launch_authority['source_stage'])/'ONE_SIMULATION_LATCH.json'
        if launch_latch.exists():raise RuntimeError('R44 authorized single simulation attempt already consumed')
        # A source-review PASS and the exact current user authority are both
        # required before creating any run, registration, alias or execution request.
        from corridor_preflight import verify_preflight
        verify_preflight(HERE,json.loads(profile_path(args.profile).read_text()))
    label=''.join(c for c in args.label if c.isalnum()or c in '_-')or'candidate'
    from run_storage import create_run,verify_run
    from storage_guard import SESSION,register
    if not SESSION.exists():raise RuntimeError('R33 must preserve the existing shared storage baseline')
    if args.prepared_run is not None:
        run=args.prepared_run.resolve();verify_run(run,ROOT)
        if (run/'runtime_manifest.json').exists() or (run/'worker_ready').exists():raise RuntimeError('Prepared R33 run already executed')
        plan=json.loads((run/'runtime_plan.json').read_text())
        if (plan.get('ros_domain')!=args.domain or plan.get('real_time_factor')!=args.real_time_factor
            or plan.get('profile_path')!=str(profile_path(args.profile))):raise RuntimeError('Prepared R33 invocation differs')
        validate_profile(plan['profile']);verify_slam_workspace()
        module(HERE/'pid_scope.py','R33_prepared_source').verify_scope(run/'navigation_scope.json')
        write_new(run/'R33_execution_request.json',dict(argv=sys.argv,plan_sha256=sha(run/'runtime_plan.json'),runner_sha256=sha(__file__)))
    else:
        stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        run,storage_contract=create_run(ROOT,f'{stamp}_closed_loop_cascade_clock_hold_{label}_{uuid.uuid4().hex[:4]}',args.run_storage_root)
        register(run)
        write_new(run/'runner_request.json',{'schema':'teacher_closed_loop_runner_request/v1','run':str(run),
            'argv':sys.argv,'runner_sha256':sha(__file__),'prepare_only':args.prepare_only,'simulation_only':True,
            'wall_begin':time.monotonic(),'no_other_process_signals_authorized':True})
        try:plan=prepare(run,args)
        except Exception as e:
            failure={'schema':'teacher_closed_loop_prepare_failure/v1','run':str(run),'status':'failed',
                'error':f'{type(e).__name__}: {e}','ROS_started':False,'Gazebo_started':False,'Actor_started':False,
                'owned_processes':[],'runner_sha256':sha(__file__),'source_manifest_present':(run/'source_manifest.json').exists()}
            write_new(run/'prepare_failure.json',failure);write_new(run/'run_result.json',failure)
            print(json.dumps(failure,ensure_ascii=False),flush=True);raise SystemExit(1)
    latest=ROOT/'runs/latest_engineering_v34_r44_active_p95_capture_candidate';tmp=ROOT/'runs'/('latest_engineering_v34_r44_active_p95_capture_candidate.'+uuid.uuid4().hex[:8]+'.tmp')
    tmp.symlink_to(run.name);tmp.replace(latest)
    if args.prepare_only:
        outcome={'run':str(run),'status':'prepared_unverified','ROS_started':False,'Gazebo_started':False,
            'runtime_plan_sha256':sha(run/'runtime_plan.json'),'navigation_scope_sha256':plan['scope_sha256']}
    else:
        latch=Path(json.loads((HERE/'R33_SHORT_STARTUP_CONTRACT.json').read_text())['source_stage'])/'ONE_SIMULATION_LATCH.json'
        write_new(latch,dict(run=str(run),runner_sha256=sha(__file__),parent_authorization_thread='01a119af-6f52-71ee-a0aa-e8f82a9e50f3',maximum_attempts=1))
        try:error=execute(run,plan)
        except Exception as e:
            failure=exceptional_runtime_manifest(run,plan,e);error=failure['error']
            if not(run/'runtime_manifest.json').exists():
                write_new(run/'runtime_manifest.json',failure)
        outcome={'run':str(run),'status':'failed'if error else'runtime_completed_navigation_unverified','runtime_error':error,
                 'runtime_manifest_sha256':sha(run/'runtime_manifest.json'),'independent_navigation_validation':'unverified'}
    write_new(run/'run_result.json',outcome);print(json.dumps(outcome,ensure_ascii=False),flush=True)
    if not args.prepare_only and outcome['runtime_error']:raise SystemExit(1)


if __name__=='__main__':main()
