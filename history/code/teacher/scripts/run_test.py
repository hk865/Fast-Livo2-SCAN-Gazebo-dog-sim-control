#!/usr/bin/env python3
"""Own only this test's children; keep training and camera service untouched."""
import argparse, datetime, hashlib, json, os, signal, subprocess, sys, time, uuid, shutil
import xml.etree.ElementTree as E
from pathlib import Path
from evaluate import evaluate
ROOT=Path(__file__).resolve().parents[1]
PYTHON=Path('/__LOCAL_HOME__/IsaacLab/_isaac_sim/kit/python/bin/python3')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stop(proc):
    if proc is None:return
    if proc.poll()is not None:return
    try:os.killpg(proc.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:os.killpg(proc.pid,signal.SIGTERM)
        except ProcessLookupError:return
        try:proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            proc.wait(timeout=3)
def resource():
    return {'processes':subprocess.check_output(['ps','-eo','pid,ppid,%cpu,%mem,rss,args'],text=True),
            'gpu':subprocess.check_output(['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader'],text=True)}
def main():
    p=argparse.ArgumentParser();p.add_argument('tests',nargs='*',default=['stand']);p.add_argument('--sensors',action='store_true');p.add_argument('--camera-only',action='store_true');p.add_argument('--shadow',action='store_true');p.add_argument('--noisy',action='store_true');p.add_argument('--repeat',type=int,default=1)
    p.add_argument('--spawn',type=float,nargs=4,metavar=('X','Y','Z','YAW'));p.add_argument('--label',default='')
    args=p.parse_args();suite=[]
    if args.shadow and not args.sensors:p.error('--shadow requires --sensors; no unavailable input is fabricated')
    for repeat in range(args.repeat):
        for test in args.tests:
            stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
            run=ROOT/'runs'/f'{stamp}_{test}{"_"+args.label if args.label else ""}_r{repeat+1}_{uuid.uuid4().hex[:4]}';run.mkdir(parents=True)
            latest=ROOT/'runs/latest';tmp=ROOT/'runs/latest.tmp'
            if tmp.exists()or tmp.is_symlink():tmp.unlink()
            tmp.symlink_to(run.name);tmp.replace(latest)
            terrain=test if test in ['ramp_up','ramp_down','step05','step10']else 'flat'
            prepare=[sys.executable,str(ROOT/'simulation/prepare.py'),'--output',str(run),'--terrain',terrain]
            if args.sensors:prepare+=['--sensors']
            if args.camera_only:prepare+=['--camera-only']
            subprocess.run(prepare,check=True,stdout=(run/'prepare.log').open('w'))
            if args.spawn:
                tree=E.parse(run/'world.sdf');x,y,z,yaw=args.spawn
                tree.getroot().find("world/model[@name='go2']/pose").text=f'{x} {y} {z} 0 0 {yaw}'
                E.indent(tree);tree.write(run/'world.sdf',encoding='unicode')
                asset=json.loads((run/'asset_manifest.json').read_text());asset['spawn']=args.spawn
                asset['spawn_override']=True;asset['scenario_label']=args.label
                asset['world_sha256']=sha(run/'world.sdf');(run/'asset_manifest.json').write_text(json.dumps(asset,indent=2)+'\n')
            baseline=resource();(run/'resources_before.json').write_text(json.dumps(baseline,indent=2))
            sourcefiles=[f for f in ROOT.rglob('*')if f.is_file()and not any(d in f.parts for d in ['runs','build','__pycache__','test_results'])]
            sources={str(f.relative_to(ROOT)):sha(f)for f in sourcefiles}
            for f in sourcefiles:
                dest=run/'sources'/f.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest)
            (run/'source_manifest.json').write_text(json.dumps(sources,indent=2))
            env={**os.environ,'ROS_DOMAIN_ID':'79','ROS_LOCALHOST_ONLY':'1','GZ_IP':'127.0.0.1',
                 'GZ_PARTITION':'teacher_'+run.name,'LIBGL_ALWAYS_SOFTWARE':'1','GALLIUM_DRIVER':'llvmpipe',
                 'GZ_SIM_SYSTEM_PLUGIN_PATH':str(ROOT/'simulation/build')+':'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH',''),
                 'TEACHER_SOCKET':'/tmp/teacher_'+uuid.uuid4().hex[:12]+'.sock','TEACHER_ACTUATOR_LOG':str(run/'actuator.jsonl'),
                 'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
            worker_cmd=[str(PYTHON),str(ROOT/'policy/worker.py'),'--run',str(run),'--test',test,'--socket',env['TEACHER_SOCKET']]
            if args.noisy:worker_cmd+=['--noisy']
            worker=gazebo=capture=bridge=shadow=resource_stream=None;error=shadow_error=None;gazebo_grace_expired=False;plugin_hash=sha(ROOT/'simulation/build/libteacher_actuator.so')
            try:
                worker=subprocess.Popen(worker_cmd,env={**env,'CUDA_VISIBLE_DEVICES':''},stdout=(run/'worker.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+20
                while not (run/'worker_ready').exists():
                    if worker.poll()is not None:raise RuntimeError('Policy failed before IPC ready')
                    if time.monotonic()>deadline:raise TimeoutError('Policy initialization timeout')
                    time.sleep(.05)
                if args.sensors or args.camera_only:
                    bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(run/'sensor_bridge.yaml')],env=env,stdout=(run/'bridge.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                    capture=subprocess.Popen([sys.executable,str(ROOT/'scripts/capture.py'),'--run',str(run),'--topic','/demo/teacher/overview'],env=env,stdout=(run/'capture.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                if args.shadow:
                    shadow=subprocess.Popen([sys.executable,str(ROOT/'scripts/sensor_shadow.py'),'--run',str(run),'--output',str(run/'sensor_shadow'),'--duration','185','--ros-domain','79'],
                                            env=env,stdout=(run/'shadow.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                    deadline=time.monotonic()+20
                    while not (run/'sensor_shadow/ready').exists():
                        if shadow.poll()is not None:raise RuntimeError('Read-only shadow observer failed before ready')
                        if time.monotonic()>deadline:raise TimeoutError('Read-only shadow observer initialization timeout')
                        time.sleep(.05)
                if bridge:
                    # DDS discovery occurs before the first physics samples.
                    time.sleep(.5)
                    if bridge.poll()is not None:raise RuntimeError('Sensor bridge failed before simulation')
                cmd=['gz','sim','-s','-r',str(run/'world.sdf')]
                # Mesa llvmpipe uses the inherited X display. EGL headless
                # explicitly selects the NVIDIA device and crashed Mesa here;
                # those failed runs are retained rather than hidden.
                gazebo=subprocess.Popen(cmd,env=env,stdout=(run/'gazebo.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+180
                resource_stream=(run/'process_resources.jsonl').open('w',buffering=1);resource_last=-float('inf')
                while worker.poll()is None:
                    if shadow and shadow.poll()is not None:shadow_error='Read-only shadow observer exited before policy completed'
                    if time.monotonic()-resource_last>=2:
                        owned=[proc.pid for proc in [worker,gazebo,bridge,capture,shadow]if proc and proc.poll()is None]
                        ps=subprocess.run(['ps','-p',','.join(map(str,owned)),'-o','pid,ppid,%cpu,%mem,rss,args'],text=True,stdout=subprocess.PIPE)
                        resource_stream.write(json.dumps({'monotonic_wall':time.monotonic(),'owned_processes':ps.stdout})+'\n');resource_last=time.monotonic()
                    if gazebo.poll()is not None:
                        try:worker.wait(timeout=2)
                        except subprocess.TimeoutExpired:raise RuntimeError('Gazebo exited before policy test finished')
                        break
                    if time.monotonic()>deadline:raise TimeoutError('Simulation test wall timeout')
                    time.sleep(.5)
                resource_stream.close()
                if worker.returncode:raise RuntimeError('Policy test process failed')
                # Retain two more physics steps so terminate/damping is visible.
                time.sleep(.1)
                # Let renderer resources finish naturally before sending a
                # cleanup signal. Exit failures remain visible to evaluation.
                try:gazebo.wait(timeout=8 if args.sensors or args.camera_only else 2)
                except subprocess.TimeoutExpired:gazebo_grace_expired=True
            except Exception as e:error=f'{type(e).__name__}: {e}'
            finally:
                if resource_stream and not resource_stream.closed:resource_stream.close()
                if shadow and shadow.poll()is None:
                    time.sleep(.5) # drain actual ROS messages before final causal comparison
                    stop(shadow)
                if shadow and (not (run/'sensor_shadow/summary.json').exists()or shadow.returncode):
                    shadow_error=shadow_error or 'Read-only shadow observer did not finish successfully; see shadow.log'
                for proc in [capture,bridge,gazebo,worker]:stop(proc)
                try:Path(env['TEACHER_SOCKET']).unlink()
                except FileNotFoundError:pass
                (run/'resources_after.json').write_text(json.dumps(resource(),indent=2))
                manifest={'run':str(run),'test':test,'owned_processes':[{ 'role':role,'pid':proc.pid,'returncode':proc.returncode}for role,proc in [('worker',worker),('gazebo',gazebo),('bridge',bridge),('capture',capture),('shadow',shadow)]if proc],
                          'error':error,'ros_domain':79,'gazebo_partition':env['GZ_PARTITION'],
                          'sensors':args.sensors,'camera_only':args.camera_only,'shadow_requested':args.shadow,'shadow_error':shadow_error,
                          'gazebo_natural_exit_grace_expired':gazebo_grace_expired,
                          'plugin_sha256':plugin_hash,
                          'training_process_untouched':True,'real_robot':False}
                (run/'runtime_manifest.json').write_text(json.dumps(manifest,indent=2))
                summary=evaluate(run);suite.append({'run':str(run),'summary':summary})
                print(json.dumps({'run':str(run),'test':test,'result':summary['tests'][0]},ensure_ascii=False),flush=True)
    (ROOT/'runs/suite_latest.json').write_text(json.dumps(suite,indent=2,ensure_ascii=False)+'\n')
if __name__=='__main__':main()
