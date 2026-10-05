#!/usr/bin/env python3
"""Create isolated static-curve truth runs on a new unobstructed plant plane; no SLAM navigation."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
PYTHON='/home/hyh001/IsaacLab/_isaac_sim/kit/python/bin/python3'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def stop(proc):
    if proc is None or proc.poll()is not None:return
    for sig,timeout in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,3)):
        try:os.killpg(proc.pid,sig);proc.wait(timeout=timeout);return
        except ProcessLookupError:return
        except subprocess.TimeoutExpired:pass


def run(profile,camera=False):
    # Root freezes all inputs before any actual curve trial. Reject source drift.
    expected=profile.get('frozen_source_hashes',{})
    if not expected:raise ValueError('Prospectively frozen curve sources required')
    for filename,digest in expected.items():
        if sha(Path(filename))!=digest:raise ValueError('Frozen curve source changed: '+filename)
    if not profile.get('feasibility',{}).get('status','').startswith('prospective_margin_from_'):
        raise ValueError('Actual signed radius evidence required before curve execution')
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    name=f"{stamp}_truth_pid_{profile['scenario']}_{profile['design']}_{profile['feedback_hz']}hz_{uuid.uuid4().hex[:4]}"
    directory=ROOT/'runs'/name;directory.mkdir()
    (directory/'truth_profile_input.json').write_text(json.dumps(profile,indent=2)+'\n')
    prepare=[sys.executable,str(ROOT/'simulation/prepare.py'),'--output',str(directory),'--terrain',profile['terrain'],
             '--real-time-factor','1','--render-engine','ogre2']
    camera_rate=float(profile.get('camera_rate_hz',2))
    if not 1<=camera_rate<=20:raise ValueError('Observer camera rate must be 1..20 Hz')
    if camera:prepare+=['--camera-only','--camera-rate',str(camera_rate),'--overview-pose',*map(str,profile.get('overview_pose',[7,-3,2,0,.4,1.2]))]
    subprocess.run(prepare,check=True,stdout=(directory/'prepare.log').open('w'))
    tree=ET.parse(directory/'world.sdf');model=tree.getroot().find("world/model[@name='go2']")
    model.find('pose').text=' '.join(map(str,[*profile['spawn'][:3],0,0,profile['spawn'][3]]))
    if camera:
        tree.getroot().find("world/model[@name='teacher_overview_camera']/link/sensor/update_rate").text=str(camera_rate)
        sensor_path=directory/'sensor_contract.json'
        sensor_contract=json.loads(sensor_path.read_text())
        sensor_contract['overview']['rate_hz']=camera_rate
        sensor_path.write_text(json.dumps(sensor_contract,indent=2)+'\n')
    # Isolated characterization fixture. Original SLAM5 map is preserved.
    world=tree.getroot().find('world')
    for item in list(world.findall('model')):
        if item.get('name') not in ('go2','teacher_overview_camera'):world.remove(item)
    for item in list(world.findall('include')):world.remove(item)
    ground=ET.fromstring('<model name="teacher_plant_ground"><static>true</static><pose>0 0 -.1 0 0 0</pose><link name="ground"><collision name="collision"><geometry><box><size>60 60 .2</size></box></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision><visual name="visual"><geometry><box><size>60 60 .2</size></box></geometry><material><ambient>.55 .55 .55 1</ambient><diffuse>.55 .55 .55 1</diffuse></material></visual></link></model>')
    world.append(ground)
    ET.indent(tree);tree.write(directory/'world.sdf',encoding='unicode')
    asset=json.loads((directory/'asset_manifest.json').read_text());asset.update(spawn=profile['spawn'],world_sha256=sha(directory/'world.sdf'),original_multifloor_geometry=False,fixture_scope='new60x60 characterizationplane; exactanalytic staticcurves; notSLAM5 route')
    (directory/'asset_manifest.json').write_text(json.dumps(asset,indent=2)+'\n')
    profile={**profile,'base_com_offset':[float(v)for v in asset['base_inertial_pose'].split()][:3],
             'uses_truth_for_control':True,'counts_as_SLAM_navigation':False}
    if profile.get('terrain_target_manifest'):
        shutil.copy2(ROOT/profile['terrain_target_manifest'],directory/'terrain_target_manifest.json')
    (directory/'truth_profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    refs={}
    sources=[(ROOT/'policy'/name,Path('policy')/name)for name in ('worker.py','observation.py','contract.json')]
    sources += [(HERE/name,Path('truth')/name)for name in ('core.py','worker.py','run.py','evaluate.py','protocol.json','profiles.py','pure_tests.py','curve_receipt.py','test_curve_receipt.py','README_CURVE_RECEIPT.md')if (HERE/name).exists()]
    sources += [(ROOT/'simulation'/name,Path('native')/Path(name).name)for name in ('teacher_actuator.cpp','build/libteacher_actuator.so','prepare.py')]
    if camera:sources += [(ROOT/'scripts/capture.py',Path('observer/capture.py'))]
    for source,relative in sources:
        dest=directory/'sources'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest);refs[str(relative)]=sha(dest)
    (directory/'source_manifest.json').write_text(json.dumps(refs,indent=2)+'\n')
    env={**os.environ,'GZ_IP':'127.0.0.1','GZ_PARTITION':'truth_'+uuid.uuid4().hex[:16],
         'GZ_SIM_SYSTEM_PLUGIN_PATH':str(directory/'sources/native'),
         'TEACHER_SOCKET':'/tmp/teacher_truth_'+uuid.uuid4().hex[:12]+'.sock',
         'TEACHER_ACTUATOR_LOG':str(directory/'actuator.jsonl'),
         'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','MKL_NUM_THREADS':'1','ROS_DOMAIN_ID':'79','ROS_LOCALHOST_ONLY':'1'}
    if camera:env.pop('LIBGL_ALWAYS_SOFTWARE',None);env.pop('GALLIUM_DRIVER',None)
    children={};error=None
    resource=lambda:{'wall':time.monotonic(),'gpu':subprocess.check_output(['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader'],text=True),
        'processes':subprocess.check_output(['ps','-eo','pid,ppid,pcpu,pmem,rss,args'],text=True)}
    (directory/'resources_before.json').write_text(json.dumps(resource(),indent=2))
    def interrupt(signum,frame):raise InterruptedError('Only owned truth experiment will be stopped')
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:
        children['worker']=subprocess.Popen([PYTHON,str(directory/'sources/truth/worker.py'),'--run',str(directory),'--socket',env['TEACHER_SOCKET']],
             env={**env,'CUDA_VISIBLE_DEVICES':''},stdout=(directory/'worker.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+30
        while not (directory/'worker_ready').exists():
            if children['worker'].poll()is not None:raise RuntimeError('CPU Teacher failed before ready')
            if time.monotonic()>deadline:raise TimeoutError('Teacher initialization timeout')
            time.sleep(.05)
        if camera:
            children['bridge']=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(directory/'sensor_bridge.yaml')],env=env,
                stdout=(directory/'bridge.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
            children['capture']=subprocess.Popen([sys.executable,str(directory/'sources/observer/capture.py'),'--run',str(directory),'--topic','/demo/teacher/overview','--archive-stride','1'],env=env,
                stdout=(directory/'capture.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(.7)
        children['gazebo']=subprocess.Popen(['gz','sim','-s','-r',str(directory/'world.sdf')],env=env,stdout=(directory/'gazebo.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+profile['duration_s']*3+60
        while children['worker'].poll()is None:
            if children['gazebo'].poll()is not None:
                if children['gazebo'].returncode!=0:raise RuntimeError('Gazebo exited abnormally before controller')
                # Native done response can stop Gazebo before worker flushes its
                # arrays/receipt. Wait for the owned worker, never SIGINT it here.
                try:children['worker'].wait(timeout=10)
                except subprocess.TimeoutExpired:raise RuntimeError('Gazebo exited without completed worker receipt')
                break
            if time.monotonic()>deadline:raise TimeoutError('Truth experiment wall deadline')
            time.sleep(.5)
        if children['worker'].returncode!=0:raise RuntimeError('Teacher worker exited abnormally')
        time.sleep(.1)
        try:children['gazebo'].wait(timeout=8)
        except subprocess.TimeoutExpired:error='Gazebo natural termination timeout'
    except Exception as exc:error=f'{type(exc).__name__}: {exc}'
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        for role in ('capture','bridge','gazebo','worker'):stop(children.get(role))
        Path(env['TEACHER_SOCKET']).unlink(missing_ok=True)
        (directory/'resources_after.json').write_text(json.dumps(resource(),indent=2))
        (directory/'runtime_manifest.json').write_text(json.dumps({'scope':'Isolated60x60plane curvePID characterization; no SLAM5 or SLAM navigation claim',
            'run':str(directory),'error':error,'owned_processes':[{'role':k,'pid':v.pid,'returncode':v.returncode}for k,v in children.items()],
            'exclusive_writer':'teacher_sim::TeacherActuator','native_plugin_sha256':sha(directory/'sources/native/libteacher_actuator.so'),
            'frozen_model_sha256':'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34',
            'source_manifest_sha256':sha(directory/'source_manifest.json'),'truth_profile_sha256':sha(directory/'truth_profile.json'),
            'uses_truth_for_control':True,'counts_as_SLAM_navigation':False,'training_processes_signaled':False,'real_robot':False},indent=2)+'\n')
        signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    if (directory/'sources/truth/evaluate.py').exists():
        subprocess.run([sys.executable,str(directory/'sources/truth/evaluate.py'),'--run',str(directory)],check=True,
                       stdout=(directory/'evaluation.log').open('w'),stderr=subprocess.STDOUT)
    if profile['terrain']!='flat' and (directory/'sources/truth/terrain_receipt.py').exists():
        subprocess.run([sys.executable,str(directory/'sources/truth/terrain_receipt.py'),'--run',str(directory)],check=True,
                       stdout=(directory/'terrain_evaluation.log').open('w'),stderr=subprocess.STDOUT)
    print(json.dumps({'run':str(directory),'runtime_error':error}),flush=True)
    return directory


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--profile',type=Path,required=True);p.add_argument('--camera',action='store_true')
    a=p.parse_args();run(json.loads(a.profile.read_text()),a.camera)
