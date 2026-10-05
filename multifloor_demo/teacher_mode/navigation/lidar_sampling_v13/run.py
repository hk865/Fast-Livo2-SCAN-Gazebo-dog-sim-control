#!/usr/bin/env python3
"""Own one actual SLAM/SCAN cascade CPU Teacher run; simulation only.

Prepare without ROS/Gazebo:
  /usr/bin/python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/lidar_sampling_v13/run.py --profile l64_r30_c10_210 --label preflight --prepare-only
Run after source review (root/user only):
  /usr/bin/python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/lidar_sampling_v13/run.py --profile l64_r30_c10_210 --label actual --domain 86
The ROS/configuration interpreter is system Python. --cpu-python selects the
already tested PyTorch interpreter for the CPU-only Actor: system Python here
has ROS but does not have torch. No dependency installation or RL process edit.
Preparation and runtime failures receive fresh run identities and receipts.
Historical acceptance is an immutable source, never a launch-success gate.
"""
from __future__ import annotations
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
ROOT=HERE.parents[1]
DEMO=ROOT.parent
SYSTEM_PYTHON=Path('/usr/bin/python3')
DEFAULT_CPU_PYTHON=Path('/home/hyh001/IsaacLab/_isaac_sim/kit/python/bin/python3')
ROS_BASE=Path('/opt/ros/jazzy/setup.bash')
SLAM_UNDERLAY=DEMO/'slam/ros2_ws/install/setup.bash'
SLAM_WORKSPACE=HERE/'slam_ws'
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
    if p.get('controller_kind')!='teacher' or p.get('navigation_ground_truth_used') is not False:
        raise ValueError('Actual SLAM/SCAN Teacher scope required')
    if p.get('review_basis')!='explicit_user_closed_loop_controller_actual_SLAM_SCAN_multifloor_simulation':
        raise ValueError('Missing explicit new closed-loop scope')
    if p.get('pose_cloud_timeout_s')!=.3:raise ValueError('Source freshness must remain300ms')
    from clock_hold import CONTRACT
    if p.get('control_clock_contract')!=CONTRACT:raise ValueError('Prospective exact-zero clock hold contract differs')
    if p.get('diagnostic_scope',{}).get('detail_window_sim_s')!=[115,118]:
        raise ValueError('V12 allows only the prospective 3s diagnostic window')
    logging_preflight=json.loads((HERE/'logging_acceleration_preflight.json').read_text())
    if logging_preflight.get('status')!='PASS_LIMITED_BUILD_AND_NUMERIC':
        raise ValueError('V11 is source prepared only; independent build and numeric fixture are required')
    cache_preflight=json.loads((HERE/'DESKEW_REUSE_PREFLIGHT.json').read_text())
    if cache_preflight.get('status')!='PASS_LIMITED_BUILD_AND_NUMERIC':
        raise ValueError('V13 source only; requires independent build and exact-dt numeric fixtures')
    lockfree_preflight=json.loads((HERE/'LOCKFREE_RESIDUAL_PREFLIGHT.json').read_text())
    if lockfree_preflight.get('status')!='PASS_LIMITED_BUILD_AND_NUMERIC':
        raise ValueError('V12 requires independent lockfree4 build and finite numeric fixtures')
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
    if not 1<=args.domain<=232 or not .1<=args.real_time_factor<=1:raise ValueError('Private domain/RTF invalid')
    for f in (SYSTEM_PYTHON,args.cpu_python,ROS_BASE,SLAM_UNDERLAY,SLAM_OVERLAY,SCAN_OVERLAY,BRIDGE,CHECKPOINT):
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
    ET.indent(tree);tree.write(run/'world.sdf',encoding='unicode')
    from sampling import override_sensors
    sampling_receipt=override_sensors(run,p)
    asset=json.loads((run/'asset_manifest.json').read_text())
    asset.update(spawn=p['spawn'],spawn_override=True,scenario_label=args.label,world_sha256=sha(run/'world.sdf'),
                 sensor_contract_sha256=sha(run/'sensor_contract.json'),sensor_sampling_override=sampling_receipt)
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
    if scope_runtime.get('required_owned_roles')!=list(REQUIRED_ROLES)or scope_runtime.get('expected_launch_children')!=EXPECTED_CHILDREN:
        raise RuntimeError('New scope and runner process contracts disagree')
    binary_contract={name:{'path':str(path.resolve()),'sha256':sha(path)} for name,path in (
        ('executable',SLAM_WORKSPACE/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping'),
        ('core',SLAM_WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so'))}
    detail_window=p['diagnostic_scope']['detail_window_sim_s']
    diagnostic_environment={'FASTLIVO_DIAGNOSTIC_DIR':str(run/'fastlivo_diagnostics'),
                            'FASTLIVO_DIAGNOSTIC_BEGIN':str(detail_window[0]),
                            'FASTLIVO_DIAGNOSTIC_END':str(detail_window[1]),
                            'FASTLIVO_BOUNDARY_TIMING':'1' if p.get('boundary_timing',False) else '0',
                            'FASTLIVO_DESKEW_REUSE':'1' if p.get('deskew_transform_reuse',False) else '0'}
    plan={'schema':'teacher_closed_loop_runtime_plan/v1','run':str(run),'simulation_only':True,'real_robot':False,
          'allowed':True,'navigation_ground_truth_used':False,'actor_privileged_dimensions':232,
          'profile_path':str(selected),'profile_sha256':sha(selected),'profile':p,
          'required_owned_roles':list(REQUIRED_ROLES),'expected_launch_children':EXPECTED_CHILDREN,'scope_runtime':scope_runtime,
          'duration_s':duration,'real_time_factor':args.real_time_factor,'wall_budget_s':duration/args.real_time_factor*3+90,
          'ramp_contract_sha256':ramp_contract,
          'ros_domain':args.domain,'actor_python':str(args.cpu_python.resolve()),'system_python':str(SYSTEM_PYTHON),
          'actor_device':'cpu','actor_threads':1,'physics_step_s':.005,'Actor_hz':50,'native_PD_hz':200,
          'exclusive_writer':'teacher_sim::TeacherActuator','checkpoint_path':str(CHECKPOINT),'model_sha256':MODEL_SHA,
          'native_plugin_sha256':sha(ROOT/'simulation/build/libteacher_actuator.so'),
          'source_manifest_sha256':sha(run/'source_manifest.json'),'scope_sha256':receipt['sha256'],
          'world_sha256':sha(run/'world.sdf'),'cloud_transport_manifest_sha256':sha(run/'cloud_transport_manifest.json'),
          'run_storage_contract':json.loads((run/'run_storage_contract.json').read_text()),
          'runtime_started':False,'renderer':'hardware/Ogre2','compatibility_motion_gate_applies':False,
          'slam_multicore_contract':json.loads((HERE/'LIO_MULTICORE_CONTRACT.json').read_text()),
          'slam_logging_acceleration_contract':json.loads((HERE/'LOGGING_ACCELERATION_CONTRACT.json').read_text()),
          'slam_lockfree_contract':json.loads((HERE/'LOCKFREE_RESIDUAL_CONTRACT.json').read_text()),
          'slam_deskew_reuse_contract':json.loads((HERE/'DESKEW_REUSE_CONTRACT.json').read_text()),
          'slam_boundary_timing_contract':json.loads((HERE/'BOUNDARY_TIMING_CONTRACT.json').read_text()),
          'slam_binary_contract':binary_contract,'navigation_stack_diagnostic_environment':diagnostic_environment,
          'diagnostic_prefix_only':bool(p.get('diagnostic_scope')),
          'runtime_commands':{'worker':[str(args.cpu_python.resolve()),'-B',str(HERE/'worker.py'),'--run',str(run)],
                              'navigation_stack':['ros2','launch',str(HERE/'stack.launch.py'),'run_dir:='+str(run)]},
          'limitations':'Plan is launch provenance only, never a route/SLAM/Sim2Sim pass'}
    write_new(run/'runtime_plan.json',plan)
    return plan


def ros_environment():
    # SCAN's generated setup may source the old core underlay again. Apply the
    # independent overlay's local_setup LAST, without replaying its underlays.
    r=subprocess.run(['bash','-c','set -e; source "$1" >/dev/null; source "$2" >/dev/null; source "$3" >/dev/null; source "$4" >/dev/null; env -0',
        'closed-loop-ros-env',str(ROS_BASE),str(SLAM_UNDERLAY),str(SCAN_OVERLAY),str(SLAM_OVERLAY)],check=True,capture_output=True)
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


def stop_owned(proc,saved,parent_first=False):
    if proc is None:return {'started':False}
    if saved is None:return {'started':True,'identity_unavailable':True,'sent_signals':[],'returncode':proc.poll()}
    sent=[];parent=identity(proc.pid)
    if parent and parent['start_ticks']!=saved['start_ticks']:
        return {'identity_mismatch':True,'sent_signals':[],'returncode':proc.poll()}
    for sig,timeout in ((signal.SIGINT,15 if parent_first else 5),(signal.SIGTERM,5),(signal.SIGKILL,3)):
        if not group_members(saved['pgid']):break
        try:
            if parent_first and sig==signal.SIGINT and proc.poll()is None:os.kill(proc.pid,sig)
            else:os.killpg(saved['pgid'],sig)
            sent.append(int(sig));until=time.monotonic()+timeout
            while time.monotonic()<until:
                proc.poll()
                if not group_members(saved['pgid']):break
                time.sleep(.1)
        except ProcessLookupError:break
    proc.poll()
    return {'started':True,'sent_signals':sent,'returncode':proc.returncode,'remaining_owned_group_members':group_members(saved['pgid'])}


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
    """Refuse sharing an active ROS control domain; never signal its processes."""
    for p in Path('/proc').iterdir():
        if not p.name.isdigit()or int(p.name)==os.getpid():continue
        try:
            argv=p.joinpath('cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace')
            if not any(x in argv for x in ('parameter_bridge','fastlivo_mapping','scan_planner_node','odom_adapter.py','ros2 launch')):continue
            values=dict(x.split(b'=',1)for x in p.joinpath('environ').read_bytes().split(b'\x00')if b'='in x)
            if values.get(b'ROS_DOMAIN_ID')==str(domain).encode():
                raise RuntimeError('ROS domain already owned by active process '+p.name+'; choose another --domain')
        except (OSError,ValueError):continue


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
    env=ros_environment();transport=json.loads((run/'cloud_transport_manifest.json').read_text())
    for k in transport['remove_environment_keys']:env.pop(k,None)
    env.update(transport['environment'])
    for k in ('FASTLIVO_DIAGNOSTIC_DIR','FASTLIVO_DIAGNOSTIC_BEGIN','FASTLIVO_DIAGNOSTIC_END','FASTLIVO_BOUNDARY_TIMING','FASTLIVO_DESKEW_REUSE'):
        env.pop(k,None)  # Diagnostic settings belong only to navigation_stack.
    sock='/tmp/teacher_closed_loop_'+uuid.uuid4().hex[:12]+'.sock'
    env.update(ROS_DOMAIN_ID=str(plan['ros_domain']),GZ_IP='127.0.0.1',GZ_PARTITION='teacher_closed_loop_'+run.name,
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'simulation/build'),TEACHER_SOCKET=sock,TEACHER_ACTUATOR_LOG=str(run/'actuator.jsonl'),
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
    env.pop('LIBGL_ALWAYS_SOFTWARE',None);env.pop('GALLIUM_DRIVER',None)
    selected_keys=('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION',
                   'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','GZ_PARTITION','GZ_IP','DISPLAY',
                   'LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS',
                   'AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH')
    write_new(run/'effective_environment.json',{k:env.get(k)for k in selected_keys})
    before=resource();write_new(run/'resources_before.json',before)
    children={};identities={};handles=[];cleanup={};error=None;sensors_ready=False;natural_timeout=False;loaded_slam=None
    stream=(run/'owned_resources.jsonl').open('x',buffering=1)
    def start(role,command,child_env=None):
        log=(run/(role+'.log')).open('x');handles.append(log)
        child=subprocess.Popen(command,env=env if child_env is None else child_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children[role]=child;identities[role]=identity(child.pid)
        if identities[role]is None:raise RuntimeError('Could not bind owned process identity '+role)
        return child
    def interrupted(sig,frame):raise InterruptedError('Runner interrupted; cleanup is restricted to saved owned process groups')
    old_handlers={s:signal.signal(s,interrupted)for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        resource_guard(before)
        require_private_domain(plan['ros_domain'])
        scope=module(HERE/'pid_scope.py','closed_loop_execute_'+uuid.uuid4().hex)
        scope.verify_scope(run/'navigation_scope.json')
        if sha(run/'source_manifest.json')!=plan['source_manifest_sha256']or sha(run/'world.sdf')!=plan['world_sha256']:
            raise RuntimeError('Run preparation changed before execution')
        worker=start('worker',[plan['actor_python'],'-B',str(HERE/'worker.py'),'--run',str(run),'--socket',sock],{**env,'CUDA_VISIBLE_DEVICES':''})
        deadline=time.monotonic()+30
        while not(run/'worker_ready').exists():
            if worker.poll()is not None:raise RuntimeError('CPU Actor exited before native IPC readiness')
            if time.monotonic()>deadline:raise TimeoutError('CPU Actor readiness exceeded30wall seconds')
            time.sleep(.05)
        start('bridge',[str(BRIDGE),'--ros-args','-r','__node:=ros_gz_bridge','-p','config_file:='+str(run/'sensor_bridge.yaml')])
        start('capture',[str(SYSTEM_PYTHON),'-B',str(ROOT/'scripts/capture.py'),'--run',str(run),
                         '--topic','/demo/teacher/overview','--archive-stride','1'])
        navigation_env={**env,'DEMO_RUN_DIR':str(run),**plan['navigation_stack_diagnostic_environment']}
        write_new(run/'navigation_stack_effective_environment.json',{
            k:navigation_env.get(k)for k in (*selected_keys,'DEMO_RUN_DIR','FASTLIVO_DIAGNOSTIC_DIR',
                                           'FASTLIVO_DIAGNOSTIC_BEGIN','FASTLIVO_DIAGNOSTIC_END','FASTLIVO_BOUNDARY_TIMING','FASTLIVO_DESKEW_REUSE')})
        start('navigation_stack',['ros2','launch',str(HERE/'stack.launch.py'),'run_dir:='+str(run)],navigation_env)
        time.sleep(.5)
        if any(p.poll()is not None for p in children.values()):raise RuntimeError('Owned dependency exited before physics started')
        loaded_slam=verify_loaded_slam(run,plan,identities['navigation_stack'])
        if any(p.poll()is not None for p in children.values()):raise RuntimeError('Owned dependency exited during V7 binary witness')
        gazebo=start('gazebo',['gz','sim','-s','-r',str(run/'world.sdf')])
        write_new(run/'owned_processes_started.json',{role:{**identities[role],'role':role}for role in children})
        camera_deadline=time.monotonic()+30;wall_deadline=time.monotonic()+plan['wall_budget_s'];last_resource=-math.inf
        while worker.poll()is None:
            now=time.monotonic()
            for role,child in children.items():
                if role not in ('worker','gazebo')and child.poll()is not None:raise RuntimeError(role+' exited while Actor remained active')
            sensors_ready=(run/'frame_source.json').exists()
            if now>camera_deadline and not sensors_ready:raise TimeoutError('Actual overview camera failed to publish in30wall seconds')
            if now-last_resource>=2:
                sample=resource([p.pid for p in children.values()if p.poll()is None]);stream.write(json.dumps(sample)+'\n');resource_guard(sample);last_resource=now
            if gazebo.poll()is not None:
                worker.wait(timeout=10);break
            if now>wall_deadline:raise TimeoutError('Owned closed-loop run exceeded frozen wall budget')
            time.sleep(.2)
        if worker.returncode!=0:raise RuntimeError('CPU Teacher worker returned nonzero')
        result=json.loads((run/'worker_result.json').read_text())
        if result.get('fault')is not None:error='Actor/native physical fault: '+str(result['fault'])
        try:gazebo.wait(timeout=8)
        except subprocess.TimeoutExpired:natural_timeout=True;error=error or 'Gazebo failed natural exit after done/damping'
    except Exception as e:error=f'{type(e).__name__}: {e}'
    finally:
        for s in old_handlers:signal.signal(s,signal.SIG_IGN)
        for role in ('navigation_stack','capture','bridge','gazebo','worker'):
            cleanup[role]=stop_owned(children.get(role),identities.get(role),parent_first=role=='navigation_stack')
        Path(sock).unlink(missing_ok=True);stream.close()
        for f in handles:f.close()
        actual=[{'role':role,**(identities[role]or{'pid':child.pid,'identity_unavailable':True}),'returncode':child.returncode}for role,child in children.items()]
        child_proof=launch_child_receipt(run/'navigation_stack.log')
        roles_complete=set(children)==set(REQUIRED_ROLES)
        clean=roles_complete and all(x['returncode']==0 for x in actual)and child_proof['all_expected_children_clean']and all(not x.get('remaining_owned_group_members')and not x.get('identity_mismatch')and not x.get('identity_unavailable')for x in cleanup.values())
        if not clean:error=error or 'Owned role/launch child cleanup incomplete or nonzero'
        manifest={**plan,'error':error,'owned_processes':actual,'cleanup':cleanup,
            'slam_loaded_binary_verified':bool(loaded_slam and loaded_slam.get('verified')),
            'slam_loaded_binary_receipt_sha256':sha(run/'slam_loaded_binary.json')if(run/'slam_loaded_binary.json').is_file()else None,
            'required_owned_roles_complete':roles_complete,'all_owned_and_children_clean':clean,
            'launch_child_exit_evidence':child_proof,'gazebo_partition':env['GZ_PARTITION'],
            'native_plugin_sha256':sha(ROOT/'simulation/build/libteacher_actuator.so'),'plugin_sha256':plan['native_plugin_sha256'],
            'source_manifest_sha256':sha(run/'source_manifest.json'),'runtime_plan_sha256':sha(run/'runtime_plan.json'),
            'gazebo_natural_exit_grace_expired':natural_timeout,'actual_sensors_ready':sensors_ready,
            'runtime_started':bool(children),'runtime_status':'failed'if error else'completed_requires_independent_navigation_evaluation',
            'training_processes_signaled':False,'actor_started':'worker'in children,'navigation_controller_kind':'teacher',
            'high_level_source':'Actual SLAM body pose and causal IMU gyro, actual registered cloud and SCAN checked path',
            'truth_navigation_used':False,'actual_feedback_hz':'Measured from unique source headers; nominal camera-driven source '+str(plan['profile']['sensor_sampling']['camera_hz'])+'Hz; unchanged20Hz wall ticker is not new feedback'}
        write_new(run/'runtime_manifest.json',manifest)
        write_new(run/'resources_after.json',resource())
        write_new(run/'summary_closed_loop_navigation.json',{'schema':'teacher_closed_loop_runtime_summary/v1',
            'status':'failed'if error else'unverified','run':str(run),'runtime_error':error,
            'levels':{'runtime':'failed'if error else'completed','navigation':'unverified','multifloor':'unverified','Sim2Sim':'unverified','real_robot':'unverified'},
            'scope_sha256':plan['scope_sha256'],'reason':'Original-source/SCAN/arrival/active-hold/native offline independent evaluation required',
            'compatibility_stand_window_diagnostic_is_navigation_acceptance':False})
        for s,handler in old_handlers.items():signal.signal(s,handler)
    return error


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--profile',required=True,help='New closed-loop profile name or JSON path')
    ap.add_argument('--label',default='candidate');ap.add_argument('--domain','--ros-domain',type=int,default=86)
    ap.add_argument('--cpu-python',type=Path,default=DEFAULT_CPU_PYTHON)
    ap.add_argument('--real-time-factor',type=float,default=1.)
    ap.add_argument('--run-storage-root',type=Path,default=None,help='Explicit owned canonical root: /var/tmp/go2_teacher_simulation_20261005; default project/runs')
    ap.add_argument('--prepare-only','--no-launch',action='store_true');args=ap.parse_args();args.cpu_python=args.cpu_python.expanduser().resolve()
    label=''.join(c for c in args.label if c.isalnum()or c in '_-')or'candidate'
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    from run_storage import create_run
    run,storage_contract=create_run(ROOT,f'{stamp}_closed_loop_cascade_clock_hold_{label}_{uuid.uuid4().hex[:4]}',args.run_storage_root)
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
    latest=ROOT/'runs/latest';tmp=ROOT/'runs'/('latest.'+uuid.uuid4().hex[:8]+'.tmp')
    tmp.symlink_to(run.name);tmp.replace(latest)
    if args.prepare_only:
        outcome={'run':str(run),'status':'prepared_unverified','ROS_started':False,'Gazebo_started':False,
            'runtime_plan_sha256':sha(run/'runtime_plan.json'),'navigation_scope_sha256':plan['scope_sha256']}
    else:
        try:error=execute(run,plan)
        except Exception as e:
            error=f'{type(e).__name__}: {e}'
            if not(run/'runtime_manifest.json').exists():
                write_new(run/'runtime_manifest.json',{**plan,'error':error,'owned_processes':[],
                    'runtime_status':'initialization_failed','all_owned_and_children_clean':False,
                    'training_processes_signaled':False,'real_robot':False})
        outcome={'run':str(run),'status':'failed'if error else'runtime_completed_navigation_unverified','runtime_error':error,
                 'runtime_manifest_sha256':sha(run/'runtime_manifest.json'),'independent_navigation_validation':'unverified'}
    write_new(run/'run_result.json',outcome);print(json.dumps(outcome,ensure_ascii=False),flush=True)
    if not args.prepare_only and outcome['runtime_error']:raise SystemExit(1)


if __name__=='__main__':main()
