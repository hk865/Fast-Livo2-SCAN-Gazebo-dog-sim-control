#!/usr/bin/env python3
"""Own only this test's children; keep training and camera service untouched."""
import argparse, datetime, hashlib, json, os, signal, subprocess, sys, time, uuid, shutil
from pathlib import Path
from evaluate import evaluate
ROOT=Path(__file__).resolve().parents[1]
PYTHON=Path('/__LOCAL_HOME__/IsaacLab/_isaac_sim/kit/python/bin/python3')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stop(proc):
    if proc is None:return
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
    p=argparse.ArgumentParser();p.add_argument('tests',nargs='*',default=['stand']);p.add_argument('--sensors',action='store_true');p.add_argument('--camera-only',action='store_true');p.add_argument('--noisy',action='store_true');p.add_argument('--repeat',type=int,default=1)
    args=p.parse_args();suite=[]
    for repeat in range(args.repeat):
        for test in args.tests:
            stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
            run=ROOT/'runs'/f'{stamp}_{test}_r{repeat+1}_{uuid.uuid4().hex[:4]}';run.mkdir(parents=True)
            latest=ROOT/'runs/latest';tmp=ROOT/'runs/latest.tmp'
            if tmp.exists()or tmp.is_symlink():tmp.unlink()
            tmp.symlink_to(run.name);tmp.replace(latest)
            terrain=test if test in ['ramp_up','ramp_down','step05','step10']else 'flat'
            prepare=[sys.executable,str(ROOT/'simulation/prepare.py'),'--output',str(run),'--terrain',terrain]
            if args.sensors:prepare+=['--sensors']
            if args.camera_only:prepare+=['--camera-only']
            subprocess.run(prepare,check=True,stdout=(run/'prepare.log').open('w'))
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
            worker=gazebo=capture=bridge=None;error=None;plugin_hash=sha(ROOT/'simulation/build/libteacher_actuator.so')
            try:
                worker=subprocess.Popen(worker_cmd,env={**env,'CUDA_VISIBLE_DEVICES':''},stdout=(run/'worker.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+20
                while not (run/'worker_ready').exists():
                    if worker.poll()is not None:raise RuntimeError('Policy failed before IPC ready')
                    if time.monotonic()>deadline:raise TimeoutError('Policy initialization timeout')
                    time.sleep(.05)
                cmd=['gz','sim','-s','-r',str(run/'world.sdf')]
                # Mesa llvmpipe uses the inherited X display. EGL headless
                # explicitly selects the NVIDIA device and crashed Mesa here;
                # those failed runs are retained rather than hidden.
                gazebo=subprocess.Popen(cmd,env=env,stdout=(run/'gazebo.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                if args.sensors or args.camera_only:
                    bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/demo/camera@sensor_msgs/msg/Image[gz.msgs.Image'],env=env,stdout=(run/'bridge.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                    capture=subprocess.Popen([sys.executable,str(ROOT/'scripts/capture.py'),'--run',str(run)],env=env,stdout=(run/'capture.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+180
                while worker.poll()is None:
                    if gazebo.poll()is not None:
                        try:worker.wait(timeout=2)
                        except subprocess.TimeoutExpired:raise RuntimeError('Gazebo exited before policy test finished')
                        break
                    if time.monotonic()>deadline:raise TimeoutError('Simulation test wall timeout')
                    time.sleep(.5)
                if worker.returncode:raise RuntimeError('Policy test process failed')
                # Retain two more physics steps so terminate/damping is visible.
                time.sleep(.1)
            except Exception as e:error=f'{type(e).__name__}: {e}'
            finally:
                for proc in [capture,bridge,gazebo,worker]:stop(proc)
                try:Path(env['TEACHER_SOCKET']).unlink()
                except FileNotFoundError:pass
                (run/'resources_after.json').write_text(json.dumps(resource(),indent=2))
                manifest={'run':str(run),'test':test,'owned_processes':[{ 'pid':proc.pid,'returncode':proc.returncode}for proc in [worker,gazebo,bridge,capture]if proc],
                          'error':error,'ros_domain':79,'gazebo_partition':env['GZ_PARTITION'],
                          'plugin_sha256':plugin_hash,
                          'training_process_untouched':True,'real_robot':False}
                (run/'runtime_manifest.json').write_text(json.dumps(manifest,indent=2))
                summary=evaluate(run);suite.append({'run':str(run),'summary':summary})
                print(json.dumps({'run':str(run),'test':test,'result':summary['tests'][0]},ensure_ascii=False),flush=True)
    (ROOT/'runs/suite_latest.json').write_text(json.dumps(suite,indent=2,ensure_ascii=False)+'\n')
if __name__=='__main__':main()
