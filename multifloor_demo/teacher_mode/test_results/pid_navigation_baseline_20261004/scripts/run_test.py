#!/usr/bin/env python3
"""Own only this test's children; keep training and camera service untouched."""
import argparse, datetime, hashlib, json, os, signal, subprocess, sys, time, uuid, shutil
import importlib.util
import xml.etree.ElementTree as E
from pathlib import Path
from evaluate import evaluate
ROOT=Path(__file__).resolve().parents[1]
PYTHON=Path('/home/hyh001/IsaacLab/_isaac_sim/kit/python/bin/python3')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stop(proc,process_group=True):
    if proc is None:return
    if proc.poll()is not None:return
    try:
        if process_group:os.killpg(proc.pid,signal.SIGINT)
        else:os.kill(proc.pid,signal.SIGINT)
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
    termination_signal=None
    def request_cleanup(signum,frame):
        nonlocal termination_signal
        termination_signal=signum
        raise InterruptedError(f'Runner received signal {signum}; cleaning up only owned children')
    for signum in (signal.SIGTERM,signal.SIGINT):signal.signal(signum,request_cleanup)
    p=argparse.ArgumentParser();p.add_argument('tests',nargs='*',default=['stand']);p.add_argument('--sensors',action='store_true');p.add_argument('--camera-only',action='store_true');p.add_argument('--shadow',action='store_true');p.add_argument('--noisy',action='store_true');p.add_argument('--repeat',type=int,default=1)
    p.add_argument('--spawn',type=float,nargs=4,metavar=('X','Y','Z','YAW'));p.add_argument('--label',default='')
    p.add_argument('--overview-pose',type=float,nargs=6,default=None,metavar=('X','Y','Z','ROLL','PITCH','YAW'))
    p.add_argument('--overview-fov',type=float,default=None)
    p.add_argument('--camera-rate',type=float,default=None)
    p.add_argument('--real-time-factor',type=float,default=1.)
    p.add_argument('--renderer',choices=['software','hardware'])
    p.add_argument('--render-engine',choices=['ogre','ogre2'])
    p.add_argument('--schedule',type=Path,help='Independent frozen command protocol; never used for navigation')
    p.add_argument('--terrain-target-manifest',type=Path)
    p.add_argument('--navigation-stack',action='store_true',help='Finite scoped experiment with actual SLAM/SCAN inputs')
    p.add_argument('--navigation-dynamic',action='store_true',help='Finite flat experiment with one real Gazebo moving obstacle; implies navigation stack')
    p.add_argument('--navigation-dynamic-turn20',action='store_true',help='Explicit calibrated dynamic-only 0.2 rad/s alignment profile; implies dynamic mode')
    p.add_argument('--navigation-dynamic-dwell',action='store_true',help='Independent dynamic profile that stops on confirmed original measured region dwell; implies turn20 dynamic mode')
    p.add_argument('--slam-only-stack',action='store_true',help='Actual stationary SLAM sensor validation without navigation commands')
    p.add_argument('--navigation-duration',type=float,default=None)
    p.add_argument('--sensor-feedback',action='store_true')
    args=p.parse_args();suite=[]
    if args.navigation_dynamic_dwell:args.navigation_dynamic_turn20=True
    if args.navigation_dynamic_turn20:args.navigation_dynamic=True
    if args.navigation_duration is None:args.navigation_duration=240. if args.navigation_dynamic else 120.
    if args.navigation_dynamic:
        if args.slam_only_stack or args.sensor_feedback:p.error('Dynamic fixture uses the separately frozen privileged policy and actual sensor navigation profile')
        args.navigation_stack=True
    if args.navigation_stack:
        if args.tests!=['navigation']or args.schedule or args.terrain_target_manifest:p.error('Scoped navigation requires exactly navigation, no scripted schedule or terrain override')
        args.sensors=True;args.shadow=True
    if args.slam_only_stack:
        if args.navigation_stack or args.tests!=['stand']:p.error('SLAM-only stack requires stand, without navigation')
        args.sensors=True;args.shadow=True
    if args.sensor_feedback:args.sensors=True
    schedule=json.loads(args.schedule.read_text())if args.schedule else None
    if schedule and 'navigation'in args.tests:p.error('Scripted schedules cannot drive navigation tests')
    if args.shadow and not args.sensors:p.error('--shadow requires --sensors; no unavailable input is fabricated')
    # Sensor modes default to the actual verified graphics path. Bare physics
    # keeps its prior environment and does not allocate a graphics context.
    args.renderer=args.renderer or ('hardware'if args.sensors or args.camera_only else 'software')
    args.render_engine=args.render_engine or ('ogre2'if args.renderer=='hardware'else 'ogre')
    for repeat in range(args.repeat):
        for test in args.tests:
            stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
            run=ROOT/'runs'/f'{stamp}_{test}{"_"+args.label if args.label else ""}_r{repeat+1}_{uuid.uuid4().hex[:4]}';run.mkdir(parents=True)
            latest=ROOT/'runs/latest';tmp=ROOT/'runs/latest.tmp'
            if tmp.exists()or tmp.is_symlink():tmp.unlink()
            tmp.symlink_to(run.name);tmp.replace(latest)
            terrain=schedule.get('terrain','flat')if schedule else test if test in ['ramp_up','ramp_down','step05','step10','step05_continue','step10_continue']else 'flat'
            if schedule:shutil.copy2(args.schedule,run/'independent_protocol.json')
            if args.terrain_target_manifest:shutil.copy2(args.terrain_target_manifest,run/'terrain_target_manifest.json')
            prepare=[sys.executable,str(ROOT/'simulation/prepare.py'),'--output',str(run),'--terrain',terrain]
            prepare+=['--render-engine',args.render_engine]
            prepare+=['--real-time-factor',str(args.real_time_factor)]
            if args.camera_rate is not None or args.navigation_stack:prepare+=['--camera-rate',str(args.camera_rate if args.camera_rate is not None else 10.)]
            if args.overview_pose is not None:prepare+=['--overview-pose',*[str(v)for v in args.overview_pose]]
            if args.overview_fov is not None:prepare+=['--overview-fov',str(args.overview_fov)]
            if args.sensors:prepare+=['--sensors']
            if args.camera_only:prepare+=['--camera-only']
            subprocess.run(prepare,check=True,stdout=(run/'prepare.log').open('w'))
            spawn=args.spawn or (schedule.get('spawn')if schedule else None)
            if spawn:
                tree=E.parse(run/'world.sdf');x,y,z,yaw=spawn
                tree.getroot().find("world/model[@name='go2']/pose").text=f'{x} {y} {z} 0 0 {yaw}'
                E.indent(tree);tree.write(run/'world.sdf',encoding='unicode')
                asset=json.loads((run/'asset_manifest.json').read_text());asset['spawn']=spawn
                asset['spawn_override']=True;asset['scenario_label']=args.label
                asset['world_sha256']=sha(run/'world.sdf');(run/'asset_manifest.json').write_text(json.dumps(asset,indent=2)+'\n')
            if args.navigation_dynamic:
                subprocess.run([sys.executable,str(ROOT/'navigation/dynamic/prepare.py'),'--stage','asset','--run',str(run)],check=True,
                               stdout=(run/'dynamic_prepare.log').open('w'),stderr=subprocess.STDOUT)
            if args.navigation_stack or args.slam_only_stack:
                config_cmd=[sys.executable,str(ROOT/'navigation/scoped_profile.py'),'--run',str(run)]
                if args.slam_only_stack:config_cmd+=['--slam-only']
                if args.navigation_dynamic:config_cmd+=['--dynamic']
                if args.navigation_dynamic_turn20:config_cmd+=['--dynamic-turn20']
                if args.navigation_dynamic_dwell:config_cmd+=['--dynamic-dwell']
                subprocess.run(config_cmd,check=True,stdout=(run/'navigation_prepare.log').open('w'),stderr=subprocess.STDOUT)
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
            if args.renderer=='hardware':
                # Only this run's graphics environment changes. The actor is
                # separately forced to CPU; the training environment is untouched.
                env.pop('LIBGL_ALWAYS_SOFTWARE',None);env.pop('GALLIUM_DRIVER',None)
            renderer_env={key:env.get(key)for key in ['DISPLAY','WAYLAND_DISPLAY','LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER','__GLX_VENDOR_LIBRARY_NAME','__NV_PRIME_RENDER_OFFLOAD','CUDA_VISIBLE_DEVICES']}
            render_manifest={'mode':args.renderer,'engine':args.render_engine,'graphics_environment':renderer_env,
                             'actor_device':'cpu','hardware_startup_ready_timeout_s':30 if args.renderer=='hardware'else None,
                             'actual_ready_scope':'IMU/joints/clock/raw_lidar/vehicle_RGB/overview_RGB callbacks'if args.shadow else 'actual overview RGB; other sensor availability requires --shadow'}
            if args.renderer=='software'and args.render_engine=='ogre'and(args.sensors or args.camera_only):
                render_manifest['known_limitation']='Explicit diagnostic mode: historical software Ogre1 sensor runs SIGSEGV at natural Gazebo cleanup; not the default sensor path'
            (run/'rendering_manifest.json').write_text(json.dumps(render_manifest,indent=2)+'\n')
            worker_cmd=[str(PYTHON),str(ROOT/'policy/worker.py'),'--run',str(run),'--test',test,'--socket',env['TEACHER_SOCKET']]
            if args.noisy:worker_cmd+=['--noisy']
            if schedule:worker_cmd+=['--schedule',str(run/'independent_protocol.json')]
            if args.terrain_target_manifest:worker_cmd+=['--terrain-target-manifest',str(run/'terrain_target_manifest.json')]
            if args.navigation_stack:worker_cmd+=['--command-file',str(run/'navigation_command.json'),'--acceptance',str(run/'navigation_scope.json'),'--navigation-duration',str(args.navigation_duration)]
            if args.sensor_feedback:worker_cmd+=['--sensor-feedback']
            worker=gazebo=capture=bridge=shadow=navstack=feedback=resource_stream=None;error=shadow_error=None;gazebo_grace_expired=False;plugin_hash=sha(ROOT/'simulation/build/libteacher_actuator.so')
            actual_sensors_ready=False;gpu_samples=[];gpu_guard_mb=None
            if args.renderer=='hardware':
                gpu_guard_mb=min(14000,int(baseline['gpu'].split()[0])+2048)
                render_manifest['gpu_memory_guard_total_mb']=gpu_guard_mb
                (run/'rendering_manifest.json').write_text(json.dumps(render_manifest,indent=2)+'\n')
            try:
                worker=subprocess.Popen(worker_cmd,env={**env,'CUDA_VISIBLE_DEVICES':''},stdout=(run/'worker.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+20
                while not (run/'worker_ready').exists():
                    if worker.poll()is not None:raise RuntimeError('Policy failed before IPC ready')
                    if time.monotonic()>deadline:raise TimeoutError('Policy initialization timeout')
                    time.sleep(.05)
                if args.sensors or args.camera_only:
                    bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(run/'sensor_bridge.yaml')],env=env,stdout=(run/'bridge.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                    capture=subprocess.Popen([sys.executable,str(ROOT/'scripts/capture.py'),'--run',str(run),'--topic','/demo/teacher/overview','--archive-stride','1'if test.endswith('_continue')else'5'],env=env,stdout=(run/'capture.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                if args.shadow:
                    shadow_duration=max(185,args.navigation_duration/args.real_time_factor+60 if args.navigation_stack else float(schedule['duration_s'])/args.real_time_factor+60 if schedule else 185)
                    shadow=subprocess.Popen([sys.executable,str(ROOT/'scripts/sensor_shadow.py'),'--run',str(run),'--output',str(run/'sensor_shadow'),'--duration',str(shadow_duration),'--ros-domain','79'],
                                            env=env,stdout=(run/'shadow.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                    deadline=time.monotonic()+20
                    while not (run/'sensor_shadow/ready').exists():
                        if shadow.poll()is not None:raise RuntimeError('Read-only shadow observer failed before ready')
                        if time.monotonic()>deadline:raise TimeoutError('Read-only shadow observer initialization timeout')
                        time.sleep(.05)
                if args.sensor_feedback:
                    feedback=subprocess.Popen([sys.executable,str(ROOT/'scripts/sensor_feedback.py'),'--run',str(run)],env=env,
                                              stdout=(run/'sensor_feedback.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                if bridge:
                    # DDS discovery occurs before the first physics samples.
                    time.sleep(.5)
                    if bridge.poll()is not None:raise RuntimeError('Sensor bridge failed before simulation')
                if args.navigation_stack or args.slam_only_stack:
                    launch_env={**env,'DEMO_RUN_DIR':str(run)}
                    launch_file='slam.launch.py'if args.slam_only_stack else 'stack.launch.py'
                    launch_shell='source /opt/ros/jazzy/setup.bash\nsource '+str(ROOT.parent/'slam/ros2_ws/install/setup.bash')+'\nsource '+str(ROOT.parent/'navigation/ros2_ws/install/setup.bash')+'\nexec ros2 launch '+str(ROOT/'navigation'/launch_file)+' run_dir:='+str(run)
                    navstack=subprocess.Popen(['bash','-c',launch_shell],env=launch_env,stdout=(run/'navigation_stack.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                cmd=['gz','sim','-s','-r',str(run/'world.sdf')]
                # Mesa llvmpipe uses the inherited X display. EGL headless
                # explicitly selects the NVIDIA device and crashed Mesa here;
                # those failed runs are retained rather than hidden.
                gazebo=subprocess.Popen(cmd,env=env,stdout=(run/'gazebo.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+max(180,args.navigation_duration*3+60 if args.navigation_stack else float(schedule['duration_s'])*3+60 if schedule else 180)
                sensor_deadline=time.monotonic()+30;gpu_last=-float('inf')
                resource_stream=(run/'process_resources.jsonl').open('w',buffering=1);resource_last=-float('inf')
                while worker.poll()is None:
                    if args.renderer=='hardware':
                        actual_sensors_ready=(run/'sensor_shadow/actual_sensor_ready.json').exists()if args.shadow else (run/'frame_source.json').exists()
                        if (args.sensors or args.camera_only)and not actual_sensors_ready and time.monotonic()>sensor_deadline:
                            raise TimeoutError('Hardware renderer did not produce actual sensor/RGB ready within 30 s')
                        if time.monotonic()-gpu_last>=2:
                            value=subprocess.check_output(['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader'],text=True)
                            gpu_samples.append({'monotonic_wall':time.monotonic(),'gpu':value});gpu_last=time.monotonic()
                            if int(value.split()[0])>gpu_guard_mb:raise RuntimeError('Hardware diagnostic GPU resource guard exceeded; stopping own run')
                    if shadow and shadow.poll()is not None:shadow_error='Read-only shadow observer exited before policy completed'
                    if time.monotonic()-resource_last>=2:
                        owned=[proc.pid for proc in [worker,gazebo,bridge,capture,shadow,navstack,feedback]if proc and proc.poll()is None]
                        ps=subprocess.run(['ps','-p',','.join(map(str,owned)),'-o','pid,ppid,%cpu,%mem,rss,args'],text=True,stdout=subprocess.PIPE)
                        resource_stream.write(json.dumps({'monotonic_wall':time.monotonic(),'owned_processes':ps.stdout})+'\n');resource_last=time.monotonic()
                    if gazebo.poll()is not None:
                        try:worker.wait(timeout=2)
                        except subprocess.TimeoutExpired:raise RuntimeError('Gazebo exited before policy test finished')
                        break
                    if navstack and navstack.poll()is not None:raise RuntimeError('Navigation stack exited before policy test finished')
                    if feedback and feedback.poll()is not None:raise RuntimeError('Actual sensor feedback broker exited before policy test finished')
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
                # Complete cleanup and write a failed receipt even if the caller
                # sends repeated termination signals. Never signal shared jobs.
                for signum in (signal.SIGTERM,signal.SIGINT):signal.signal(signum,signal.SIG_IGN)
                if resource_stream and not resource_stream.closed:resource_stream.close()
                if shadow and shadow.poll()is None:
                    time.sleep(.5) # drain actual ROS messages before final causal comparison
                    stop(shadow)
                if shadow and (not (run/'sensor_shadow/summary.json').exists()or shadow.returncode):
                    shadow_error=shadow_error or 'Read-only shadow observer did not finish successfully; see shadow.log'
                stop(navstack,process_group=False) # launch forwards once; avoid double SIGINT in child cleanup
                for proc in [feedback,capture,bridge,gazebo,worker]:stop(proc)
                try:Path(env['TEACHER_SOCKET']).unlink()
                except FileNotFoundError:pass
                (run/'resources_after.json').write_text(json.dumps(resource(),indent=2))
                manifest={'run':str(run),'test':test,'owned_processes':[{ 'role':role,'pid':proc.pid,'returncode':proc.returncode}for role,proc in [('worker',worker),('gazebo',gazebo),('bridge',bridge),('capture',capture),('shadow',shadow),('navigation_stack',navstack),('sensor_feedback',feedback)]if proc],
                          'error':error,'ros_domain':79,'gazebo_partition':env['GZ_PARTITION'],
                          'sensors':args.sensors,'camera_only':args.camera_only,'shadow_requested':args.shadow,'shadow_error':shadow_error,
                          'gazebo_natural_exit_grace_expired':gazebo_grace_expired,
                          'rendering':render_manifest,'actual_sensors_ready':actual_sensors_ready,'gpu_resource_samples':gpu_samples,
                          'plugin_sha256':plugin_hash,
                          'training_process_untouched':True,'real_robot':False}
                (run/'runtime_manifest.json').write_text(json.dumps(manifest,indent=2))
                summary=evaluate(run);suite.append({'run':str(run),'summary':summary})
                if test.endswith('_continue'):
                    module_name='teacher_functional_'+uuid.uuid4().hex
                    evaluator_path=run/'sources/scripts/evaluate_step_functional.py'
                    spec=importlib.util.spec_from_file_location(module_name,evaluator_path)
                    if spec is None or spec.loader is None:raise RuntimeError('Archived functional evaluator cannot be loaded')
                    evaluator_module=importlib.util.module_from_spec(spec)
                    sys.modules[module_name]=evaluator_module
                    spec.loader.exec_module(evaluator_module)
                    functional=evaluator_module.evaluate(run)
                    suite[-1]['functional_summary']=functional
                    console={'run':str(run),'test':test,'functional_result':functional['tests'][0],
                             'legacy_result':summary['tests'][0],
                             'legacy_scope':'Compatibility record only; historical command timing and height-gain protocol do not apply to the independent functional retest'}
                else:console={'run':str(run),'test':test,'result':summary['tests'][0]}
                print(json.dumps(console,ensure_ascii=False),flush=True)
            for signum in (signal.SIGTERM,signal.SIGINT):signal.signal(signum,request_cleanup)
            if termination_signal is not None:raise SystemExit(128+termination_signal)
    (ROOT/'runs/suite_latest.json').write_text(json.dumps(suite,indent=2,ensure_ascii=False)+'\n')
if __name__=='__main__':main()
