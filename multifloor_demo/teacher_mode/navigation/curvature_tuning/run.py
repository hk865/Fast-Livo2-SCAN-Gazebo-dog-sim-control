#!/usr/bin/env python3
"""Isolated immutable plant runner. Only the root operator launches actual runs."""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[1]
CPU_PYTHON='/home/hyh001/IsaacLab/_isaac_sim/kit/python/bin/python3'


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def load_plan(path):
    plan=json.loads(path.read_text())
    if plan.get('schema')!='teacher_continuous_turn_plant_plan/v1': raise ValueError('Wrong finite plant plan schema')
    for name,digest in plan['source_hashes'].items():
        if sha(name)!=digest: raise RuntimeError('Frozen plan source changed: '+name)
    protocol=json.loads((HERE/'protocol.json').read_text())
    if sha(HERE/'protocol.json')!=plan['protocol_sha256']: raise ValueError('Protocol mismatch')
    if plan['speed_mps'] not in protocol['speeds_mps'] or plan['yaw_rate_radps'] not in protocol['yaw_rates_radps']:
        raise ValueError('Outside frozen plant envelope')
    if plan['timing']!=protocol['timing'] or plan['fixture']!=protocol['fixture'] or plan['model_sha256']!=protocol['model_sha256']:
        raise ValueError('Plan differs from prospective fixture/timing/model')
    return plan


def structural_fingerprint(element):
    """All SDF content except XML layout tails/indentation remains significant."""
    return (element.tag,tuple(sorted(element.attrib.items())),(element.text or '').strip(),
            tuple(structural_fingerprint(child) for child in element))


def _prepare_into(args):
    plan=load_plan(args.plan.resolve())
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    signed='p' if plan['yaw_rate_radps']>0 else 'n'
    directory=args.preparation_directory
    shutil.copy2(args.plan,directory/'curvature_plan.json')
    manifest={}
    def archive(src,dst):
        src=Path(src); dest=directory/'sources'/dst; dest.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(src,dest)
        manifest[str(dst)]={'path':str(dest),'sha256':sha(dest),'original_path':str(src.resolve()),'original_sha256':sha(src)}
    for name in ('plant_worker.py','prepare_plan.py','run.py','evaluate_radius.py','protocol.json','README.md','pure_tests.py'):
        archive(HERE/name,Path('curvature')/name)
    for name in ('worker.py','observation.py','contract.json'):
        archive(Path(plan['policy_source_directory'])/name,Path('policy')/name)
    for name in ('teacher_actuator.cpp','build/libteacher_actuator.so','prepare.py'):
        archive(ROOT/'simulation'/name,Path('native')/Path(name).name)
    if args.camera: archive(ROOT/'scripts/capture.py',Path('observer/capture.py'))
    for source,digest in plan['source_hashes'].items():
        key='original_'+hashlib.sha256(source.encode()).hexdigest()[:16]
        dest=Path('original_refs')/(key+'_'+Path(source).name)
        archive(Path(source),dest)
        manifest[str(dest)]['frozen_plan_digest']=digest
    cmd=[sys.executable,str(ROOT/'simulation/prepare.py'),'--output',str(directory),'--terrain','flat',
         '--real-time-factor',str(args.real_time_factor),'--render-engine','ogre2']
    if args.camera: cmd+=['--camera-only','--camera-rate','2','--overview-fov',str(math.radians(80)),
                            '--overview-pose','0','-18','12','0','.55',str(math.pi/2)]
    with (directory/'prepare.log').open('w') as log: subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
    tree=ET.parse(directory/'world.sdf'); world=tree.getroot().find('world')
    go2=world.find("model[@name='go2']")
    if go2 is None: raise ValueError('Missing original Go2 model')
    original_robot=ET.tostring(go2,encoding='unicode')
    original_structure=structural_fingerprint(go2)
    for m in list(world.findall('model')):
        if m.get('name') not in ('go2','teacher_overview_camera'): world.remove(m)
    for include in list(world.findall('include')): world.remove(include)
    go2.find('pose').text='0 0 .4 0 0 0'
    ground=ET.fromstring('<model name="teacher_plant_ground"><static>true</static><pose>0 0 -.1 0 0 0</pose><link name="ground"><collision name="collision"><geometry><box><size>60 60 .2</size></box></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision><visual name="visual"><geometry><box><size>60 60 .2</size></box></geometry><material><ambient>.55 .55 .55 1</ambient><diffuse>.55 .55 .55 1</diffuse></material></visual></link></model>')
    world.append(ground)
    actuator=go2.findall("plugin[@name='teacher_sim::TeacherActuator']")
    if len(actuator)!=1 or any('control' in p.get('name','').lower() for p in go2.findall('plugin')):
        raise ValueError('Exclusive actuator violated')
    # Only the pose differs from prepared Go2. No inertial, joint, actuator,
    # collision or sensor data on the robot may change in the plant fixture.
    comparison=ET.fromstring(ET.tostring(go2,encoding='unicode')); comparison.find('pose').text=ET.fromstring(original_robot).findtext('pose')
    if structural_fingerprint(comparison)!=original_structure: raise ValueError('Unexpected robot mutation')
    ET.indent(tree); tree.write(directory/'world.sdf',encoding='unicode')
    asset=json.loads((directory/'asset_manifest.json').read_text()); asset.update(spawn=[0,0,.4,0],world_sha256=sha(directory/'world.sdf'),
        fixture_scope='New 60x60m plane; not original multifloor map',original_multifloor_geometry=False)
    (directory/'asset_manifest.json').write_text(json.dumps(asset,indent=2)+'\n')
    (directory/'fixture_manifest.json').write_text(json.dumps({'schema':'teacher_continuous_turn_plane/v1',
        'fixture':plan['fixture'],'robot_changes':['spawn pose only'],'ground_friction_ode_mu':1.,
        'model_names':[m.get('name') for m in world.findall('model')],'world_sha256':sha(directory/'world.sdf'),
        'robot_pre_spawn_sha256':hashlib.sha256(original_robot.encode()).hexdigest(),
        'robot_structural_fingerprint_sha256':hashlib.sha256(json.dumps(original_structure).encode()).hexdigest(),
        'camera_observer_only':bool(args.camera),'no_base_servo':True,'no_physics_math_change':True},indent=2)+'\n')
    for name in ('world.sdf','asset_manifest.json','fixture_manifest.json','curvature_plan.json','sensor_contract.json','sensor_bridge.yaml'):
        path=directory/name
        if path.exists(): manifest['input_'+name]={'path':str(path),'sha256':sha(path),'generated_input':True}
    (directory/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (directory/'prepared_receipt.json').write_text(json.dumps({'status':'prepared_only','actual_validation':'unverified',
        'plan_sha256':sha(directory/'curvature_plan.json'),'source_manifest_sha256':sha(directory/'source_manifest.json'),
        'simulation_started':False,'camera':bool(args.camera),'real_time_factor':args.real_time_factor},indent=2)+'\n')
    return directory,plan


def prepare(args):
    # A fresh directory is allocated before validation so every failed prepare
    # identifies its own artifacts. No earlier or frozen run is ever changed.
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    args.preparation_directory=ROOT/'runs'/f'{stamp}_plant_radius_prepare_{uuid.uuid4().hex[:4]}'
    args.preparation_directory.mkdir()
    try:
        return _prepare_into(args)
    except BaseException as exc:
        failure={'schema':'teacher_continuous_turn_prepare_failure/v1','status':'failed','run':str(args.preparation_directory),
                 'plan':str(args.plan.resolve()),'error':repr(exc),'simulation_started':False,'limited_preparation_artifacts_only':True}
        for name in ('prepare_failure.json','prepared_receipt.json'):
            (args.preparation_directory/name).write_text(json.dumps(failure,indent=2)+'\n')
        print(json.dumps(failure),file=sys.stderr,flush=True)
        raise


def stop_owned(proc):
    if proc is None or proc.poll() is not None: return
    for sig,timeout in ((signal.SIGINT,8),(signal.SIGTERM,3),(signal.SIGKILL,3)):
        try: os.killpg(proc.pid,sig);proc.wait(timeout=timeout);return
        except ProcessLookupError:return
        except subprocess.TimeoutExpired:pass


def execute(directory,plan,args):
    token=uuid.uuid4().hex[:16]
    env={**os.environ,'GZ_IP':'127.0.0.1','GZ_PARTITION':'plant_'+token,'GZ_SIM_SYSTEM_PLUGIN_PATH':str(directory/'sources/native'),
         'TEACHER_SOCKET':'/tmp/teacher_plant_'+token+'.sock','TEACHER_ACTUATOR_LOG':str(directory/'actuator.jsonl'),
         'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','MKL_NUM_THREADS':'1','ROS_DOMAIN_ID':str(args.ros_domain),
         'ROS_LOCALHOST_ONLY':'1','PYTHONDONTWRITEBYTECODE':'1'}
    env.pop('LIBGL_ALWAYS_SOFTWARE',None);env.pop('GALLIUM_DRIVER',None)
    owned={};logs=[];error=None
    def launch(role,argv,extra=None):
        log=(directory/(role+'.log')).open('w');logs.append(log)
        owned[role]=subprocess.Popen(argv,env={**env,**(extra or {})},stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def resource():
        data={'monotonic_wall':time.monotonic()}
        for key,argv in [('gpu',['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader']),('processes',['ps','-eo','pid,ppid,pcpu,pmem,rss,args'])]:
            try:data[key]=subprocess.check_output(argv,text=True)
            except Exception as exc:data[key+'_unavailable']=repr(exc)
        return data
    (directory/'resources_before.json').write_text(json.dumps(resource(),indent=2))
    (directory/'effective_environment.json').write_text(json.dumps({k:env[k] for k in ('GZ_IP','GZ_PARTITION','GZ_SIM_SYSTEM_PLUGIN_PATH','TEACHER_SOCKET','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','OMP_NUM_THREADS')},indent=2)+'\n')
    def interrupted(signum,frame): raise InterruptedError('Operator interrupted only this owned plant run')
    old_handlers={sig:signal.signal(sig,interrupted) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        launch('worker',[CPU_PYTHON,str(directory/'sources/curvature/plant_worker.py'),'--run',str(directory),'--socket',env['TEACHER_SOCKET']],{'CUDA_VISIBLE_DEVICES':''})
        deadline=time.monotonic()+30
        while not (directory/'worker_ready').exists():
            if owned['worker'].poll() is not None:raise RuntimeError('CPU Teacher failed before ready')
            if time.monotonic()>deadline:raise TimeoutError('CPU Teacher ready timeout')
            time.sleep(.05)
        if args.camera:
            launch('bridge',['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge','--ros-args','-r','__node:=ros_gz_bridge','-p','config_file:='+str(directory/'sensor_bridge.yaml')])
            launch('capture',[sys.executable,str(directory/'sources/observer/capture.py'),'--run',str(directory),'--topic','/demo/teacher/overview','--archive-stride','1'])
        launch('gazebo',['gz','sim','-s','-r',str(directory/'world.sdf')])
        deadline=time.monotonic()+plan['timing']['duration_s']/args.real_time_factor*3+60
        while owned['worker'].poll() is None:
            if owned['gazebo'].poll() is not None:
                if owned['gazebo'].returncode!=0:raise RuntimeError('Gazebo abnormal exit')
                owned['worker'].wait(timeout=10);break
            for role,p in owned.items():
                if role not in ('worker','gazebo') and p.poll() is not None:raise RuntimeError(role+' exited during run')
            if time.monotonic()>deadline:raise TimeoutError('Bounded plant wall timeout')
            time.sleep(.1)
        if owned['worker'].returncode!=0:raise RuntimeError('CPU worker abnormal exit')
        owned['gazebo'].wait(timeout=8)
    except BaseException as exc:error=repr(exc)
    finally:
        for sig in old_handlers:signal.signal(sig,signal.SIG_IGN)
        for role in ('capture','bridge','gazebo','worker'):stop_owned(owned.get(role))
        Path(env['TEACHER_SOCKET']).unlink(missing_ok=True)
        for log in logs:log.close()
        (directory/'resources_after.json').write_text(json.dumps(resource(),indent=2))
        (directory/'runtime_manifest.json').write_text(json.dumps({'schema':'teacher_continuous_turn_runtime/v1','error':error,
            'owned_processes':[{'role':role,'pid':p.pid,'returncode':p.returncode} for role,p in owned.items()],
            'expected_owned_roles':['worker','gazebo']+(['bridge','capture'] if args.camera else []),
            'exclusive_writer':'teacher_sim::TeacherActuator','native_plugin_sha256':sha(directory/'sources/native/libteacher_actuator.so'),
            'source_manifest_sha256':sha(directory/'source_manifest.json'),'plan_sha256':sha(directory/'curvature_plan.json'),
            'model_sha256':plan['model_sha256'],'CPU_inference':True,'camera':bool(args.camera),'real_time_factor':args.real_time_factor,
            'counts_as_SLAM_navigation':False,'PID_tracking_claim':False,'training_processes_signaled':False,'real_robot':False},indent=2)+'\n')
        for sig,handler in old_handlers.items():signal.signal(sig,handler)
    subprocess.run([sys.executable,str(directory/'sources/curvature/evaluate_radius.py'),'--run',str(directory)],check=True)
    return error


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--plan',type=Path,required=True);ap.add_argument('--prepare-only',action='store_true')
    ap.add_argument('--camera',action='store_true');ap.add_argument('--ros-domain',type=int,default=84)
    ap.add_argument('--real-time-factor',type=float,default=1.)
    args=ap.parse_args()
    if not .1<=args.real_time_factor<=1 or not 0<=args.ros_domain<=232:ap.error('Invalid isolated runtime settings')
    directory,plan=prepare(args)
    error=None if args.prepare_only else execute(directory,plan,args)
    print(json.dumps({'run':str(directory),'prepared_only':args.prepare_only,'runtime_error':error}),flush=True)
    if error:raise SystemExit(1)


if __name__=='__main__':main()
