#!/usr/bin/env python3
"""Explicitly requested V19 SLAM-only replay, owned processes/private domains.

No Gazebo, model, controller, training, simulator truth, relay, actuator or SCAN.
The physical-run gate is not invoked/bypassed: this is a separate limited replay.
"""
import argparse
import csv
import hashlib
import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from input_identity import TOPICS, load_index, sha
from read_stages import COLUMNS, LIFE_COLUMNS, analyze, read_csv, read_summary

if not __debug__: raise RuntimeError('Never execute validation under -O/PYTHONOPTIMIZE')
HERE = Path(__file__).resolve().parent
TEACHER = HERE.parents[2]
PROJECT = TEACHER.parents[1]
WS = TEACHER/'navigation/pipeline_v19/slam_ws'
CORE = WS/'install/fast_livo2_core/lib/libfast_livo2_core.so'
MAPPER = WS/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping'
CORE_SHA = 'ec5ccfab27028108841d720447e35e902b148b89ffc56cc23f108f25ee8c0e2a'
FRESH_SHA = '6323b7a4d18f358380e6d0e0a6c5c5e7e18160c3cb59128392bd413772d335c5'


def require(value, text):
    if not value: raise RuntimeError(text)


def write_new(path, value):
    with Path(path).open('x') as f: json.dump(value,f,indent=2,allow_nan=False); f.write('\n')


def identity(pid):
    try:
        raw = Path(f'/proc/{pid}/stat').read_text(); fields = raw[raw.rfind(')')+2:].split()
        return {'pid':pid,'state':fields[0],'ppid':int(fields[1]),'pgid':int(fields[2]),'session':int(fields[3]),'start_ticks':int(fields[19])}
    except (FileNotFoundError, ProcessLookupError): return None


def owned_signal(proc, saved, sig, group=True):
    now = identity(proc.pid)
    if now is None: return
    require(now['start_ticks'] == saved['start_ticks'] and now['pgid'] == saved['pgid'], 'Owned process identity changed')
    if group: os.killpg(saved['pgid'],sig)
    else: os.kill(proc.pid,sig)


def members(pgid):
    result = []
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            x = identity(int(path.name))
            if x and x['pgid'] == pgid and x['state'] != 'Z': result.append(x)
    return result


def stop(proc, saved, mapper=False):
    sent = []
    if mapper and proc.poll() is None:
        owned_signal(proc,saved,signal.SIGUSR1,False); sent.append(int(signal.SIGUSR1))
        try: proc.wait(timeout=12)
        except subprocess.TimeoutExpired: pass
    for sig,wait in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,2)):
        if not members(saved['pgid']): break
        owned_signal(proc,saved,sig); sent.append(int(sig))
        try: proc.wait(timeout=wait)
        except subprocess.TimeoutExpired: continue
    proc.poll()
    return {'returncode':proc.returncode,'sent_signals':sent,'remaining_owned_members':members(saved['pgid'])}


def sample_process(saved):
    pid = saved['pid']; result = {'pid':pid,'threads':{}}
    if not identity(pid): return result
    for thread in Path(f'/proc/{pid}/task').iterdir():
        try:
            raw=(thread/'stat').read_text(); x=raw[raw.rfind(')')+2:].split()
            status=(thread/'status').read_text(); context={}
            for line in status.splitlines():
                if line.startswith(('voluntary_ctxt_switches:','nonvoluntary_ctxt_switches:')):
                    k,v=line.split(':',1);context[k]=int(v)
            result['threads'][thread.name]={'utime_ticks':int(x[11]),'stime_ticks':int(x[12]),
                'processor':int(x[36]),'name':(thread/'comm').read_text().strip(),**context}
        except (FileNotFoundError, ProcessLookupError): pass
    return result


def require_private_domain(domain):
    wanted=('ROS_DOMAIN_ID='+str(domain)).encode()
    for path in Path('/proc').iterdir():
        if not path.name.isdigit() or int(path.name)==os.getpid():continue
        try:
            values=(path/'environ').read_bytes().split(b'\0')
            if wanted in values:raise RuntimeError('Refuse occupied replay domain '+str(domain)+' pid '+path.name)
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture-run',type=Path,required=True);p.add_argument('--index-dir',type=Path,required=True)
    p.add_argument('--mode',choices=('serial','rx_decode','staged'),required=True)
    p.add_argument('--rate',type=float,default=1.);p.add_argument('--domain',type=int,choices=range(201,206),required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--execute',action='store_true')
    args=p.parse_args();capture=args.capture_run.resolve();index=args.index_dir.resolve();out=args.out.resolve()
    require(0<args.rate<=3,'Headroom rate outside prospective1–3 scope')
    require(not out.exists(),'Use a new independent replay output directory')
    fresh=HERE/'finite_math/FRESH_MATH_FP_TEAM_LOADER_RECEIPT.json'
    require(sha(fresh)==FRESH_SHA and json.loads(fresh.read_text())['status']=='PASS_LIMITED_FRESH_KERNEL_REGRESSION','Fresh finite math binding unavailable')
    require(sha(CORE)==CORE_SHA and MAPPER.is_file(),'New private core/mapping executable unavailable/changed')
    corpus=json.loads((index/'CAPTURED_INPUT_MANIFEST.json').read_text())
    require(Path(corpus['bag']).resolve()==capture/'sensor_input_bag' and sha(index/'captured_inputs.jsonl')==corpus['index_sha256'],'Foreign/changed corpus identity')
    expected=load_index(index/'captured_inputs.jsonl')
    env=os.environ.copy()
    for key in list(env):
        if key.startswith(('FASTLIVO_','V15_','V16_','GZ_')) or key in ('LD_PRELOAD','DEMO_RUN_DIR','CYCLONEDDS_URI'):
            env.pop(key,None)
    transport=json.loads((capture/'navigation_stack_effective_environment.json').read_text())
    env.update({k:v for k,v in transport.items() if k in ('RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','FASTRTPS_DEFAULT_PROFILES_FILE') and v is not None})
    env.update(ROS_DOMAIN_ID=str(args.domain),DEMO_RUN_DIR=str(out),
        FASTLIVO_PIPELINE_MODE=args.mode,FASTLIVO_IMAGE_COPY_OPT='0',FASTLIVO_DIAGNOSTIC_DIR=str(out/'fastlivo_diagnostics'),
        FASTLIVO_DIAGNOSTIC_BEGIN='115',FASTLIVO_DIAGNOSTIC_END='118',FASTLIVO_BOUNDARY_TIMING='1',
        FASTLIVO_LIO_JACOBIAN_THREADS='4',FASTLIVO_VIO_PATCH_THREADS='1',OMP_NUM_THREADS='1',
        OMP_DYNAMIC='FALSE',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=':'.join([str(CORE.parent),str(PROJECT/'slam5_navigation/ros2_ws/install/vikit_common/lib'),
        str(PROJECT/'slam5_navigation/ros2_ws/install/vikit_ros/lib'),'/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',
        '/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu'])
    out.mkdir(parents=True,mode=0o700)
    for name in ('navigation_fastlivo.yaml','navigation_camera.yaml'):
        with (out/name).open('xb') as f:f.write((capture/name).read_bytes())
    ros=['--ros-args','-p','use_sim_time:=true']
    commands={
        'camera_parameters':['/opt/ros/jazzy/lib/demo_nodes_cpp/parameter_blackboard',*ros,'--params-file',str(out/'navigation_camera.yaml')],
        'mapping':['taskset','-c','0,2,4,6',str(MAPPER),*ros,'--params-file',str(out/'navigation_fastlivo.yaml')],
        'observer':['/usr/bin/python3','-B',str(HERE/'shadow_observer.py'),'--out',str(out),*ros],
        'player':['ros2','bag','play',str(capture/'sensor_input_bag'),'--rate',str(args.rate),'--read-ahead-queue-size','256',
                  '--delay','1','--disable-keyboard-controls','--wait-for-all-acked','1000']}
    plan={'schema':'V19_standalone_SLAM_replay_plan/v1','scope':'SLAM-only input bag replay; no physics/Teacher/controller/navigation',
        'capture_run':str(capture),'out':str(out),'mode':args.mode,'rate':args.rate,'domain':args.domain,
        'commands':commands,'core_sha256':CORE_SHA,'mapper_sha256':sha(MAPPER),'fresh_math_receipt_sha256':FRESH_SHA,
        'corpus_manifest_sha256':sha(index/'CAPTURED_INPUT_MANIFEST.json'),'corpus_index_sha256':corpus['index_sha256'],
        'reader_sha256':sha(HERE/'read_stages.py'),'runner_sha256':sha(Path(__file__)),
        'observer_sha256':sha(HERE/'shadow_observer.py'),
        'configuration_sha256':{name:sha(out/name) for name in ('navigation_fastlivo.yaml','navigation_camera.yaml')},
        'environment':{k:v for k,v in env.items() if k.startswith(('ROS_','FASTLIVO_','OMP_','MKL_','OPENBLAS_')) or k in ('LD_LIBRARY_PATH','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','DEMO_RUN_DIR')},
        'same_bag_CDR_content':True,'same_cross_topic_production_order_claimed':False,
        'native_wall_timer_schedule_same_as_capture_or_other_rate':False,'exact_full_frontend_math_replay':'NOT_IMPLEMENTED'}
    write_new(out/'PLAN.json',plan)
    if not args.execute:
        print(json.dumps({'status':'prepared_not_executed','out':str(out)}));return 0
    require_private_domain(args.domain)
    children={};identities={};handles={};cleanup={};error=None;loaded=None;samples=[]
    def start(role):
        handles[role]=(out/(role+'.log')).open('x')
        children[role]=subprocess.Popen(commands[role],env=env,stdout=handles[role],stderr=subprocess.STDOUT,start_new_session=True)
        identities[role]=identity(children[role].pid);require(identities[role] is not None,'Owned process identity missing')
    def wait_file(path,timeout=20):
        until=time.monotonic()+timeout
        while not path.exists():
            require(all(c.poll()is None for c in children.values()),'Owned role exited before readiness')
            require(time.monotonic()<until,'Readiness timeout '+str(path));time.sleep(.05)
    try:
        start('camera_parameters');time.sleep(.5);start('observer');wait_file(out/'observer_ready.json')
        start('mapping');wait_file(out/'fastlivo_debug/pipeline_v19_events.csv')
        pid=children['mapping'].pid;maps=Path(f'/proc/{pid}/maps').read_text()
        paths={line.split()[-1] for line in maps.splitlines() if 'libfast_livo2_core.so' in line}
        require(paths=={str(CORE.resolve())},'Actual mapping loaded foreign core')
        executable=Path(f'/proc/{pid}/exe').resolve();require(executable==MAPPER.resolve(),'Actual mapping executable mismatch')
        loaded={'schema':'V19_shadow_loaded_binary/v1','identity':identities['mapping'],'before_playback':True,
                'actual_executable':str(executable),'actual_executable_sha256':sha(executable),
                'actual_core_paths':sorted(paths),'actual_core_sha256':sha(CORE),'maps_sha256':hashlib.sha256(maps.encode()).hexdigest()}
        write_new(out/'LOADED_BINARY.json',loaded)
        start('player');deadline=time.monotonic()+max(120,100/args.rate);last_sample=0
        while children['player'].poll()is None:
            require(all(children[role].poll()is None for role in ('mapping','camera_parameters','observer')),'SLAM-only dependency exited during replay')
            require(time.monotonic()<deadline,'Replay wall deadline exceeded')
            if time.monotonic()-last_sample>=1:
                last_sample=time.monotonic();samples.append({'wall_ns':time.monotonic_ns(),
                    'owned_roles':{role:sample_process(identities[role])for role in children}})
            time.sleep(.1)
        require(children['player'].returncode==0,'Bag player returned nonzero')
        # Let already delivered input complete. This fixed post-play wait is
        # recorded and is not a missing/loss waiver; final input counts decide.
        time.sleep(2)
    except Exception as e:error=type(e).__name__+': '+str(e)
    finally:
        for role in ('mapping','player','observer','camera_parameters'):
            if role in children:cleanup[role]=stop(children[role],identities[role],mapper=role=='mapping')
        for handle in handles.values():handle.close()
    write_new(out/'PROCESS_SAMPLES.json',{'schema':'V19_shadow_process_samples/v1','CLK_TCK':os.sysconf('SC_CLK_TCK'),'samples':samples})
    runtime={'schema':'V19_standalone_SLAM_replay_runtime/v1','error':error,'scope':plan['scope'],
             'owned_processes':identities,'cleanup':cleanup,'loaded_binary':loaded,
             'all_owned_processes_clean':set(cleanup)==set(commands) and all(x['returncode']==0 and not x['remaining_owned_members']for x in cleanup.values()),
             'physics_or_Teacher_or_controller_started':False}
    write_new(out/'RUNTIME.json',runtime)
    if error is not None or not runtime['all_owned_processes_clean']:
        print(json.dumps({'status':'failed_runtime','error':error,'out':str(out)}));return 1
    summary=read_summary(out/'fastlivo_debug/pipeline_v19_summary.json')
    rows=read_csv(out/'fastlivo_debug/pipeline_v19_events.csv',COLUMNS)
    stage=analyze(rows,read_csv(out/'fastlivo_debug/pipeline_v19_lifecycle.csv',LIFE_COLUMNS,('event',)),summary)
    stage['input_sha256']={str(path):sha(path)for path in (out/'fastlivo_debug/pipeline_v19_events.csv',out/'fastlivo_debug/pipeline_v19_lifecycle.csv',out/'fastlivo_debug/pipeline_v19_summary.json')}
    write_new(out/'STAGE_RECEIPT.json',stage)
    topic_kind={'/demo/teacher/slam/imu':1,'/demo/slam/lidar_filtered':2,'/demo/teacher/slam/image':3}
    expected_headers=[(topic_kind[r['topic']],r['source_ns'])for r in expected if r['topic']in topic_kind]
    observed_headers=[(r['kind'],r['source_ns'])for r in rows if r['kind']in (1,2,3)]
    from collections import Counter
    matched=Counter(expected_headers)==Counter(observed_headers)
    write_new(out/'INPUT_HEADER_MATCH.json',{'schema':'V19_bag_to_actual_admission_headers/v1',
        'same_sensor_header_multiset':matched,'same_captured_record_sensor_order':expected_headers==observed_headers,
        'expected_sensor_count':len(expected_headers),'observed_sensor_count':len(observed_headers),
        'expected_order_sha256':hashlib.sha256(json.dumps(expected_headers,separators=(',',':')).encode()).hexdigest(),
        'observed_order_sha256':hashlib.sha256(json.dumps(observed_headers,separators=(',',':')).encode()).hexdigest(),
        'body_bytes_at_receiver_proven':False,'original_capture_mapper_order_proven':False,
        'exact_full_frontend_math':'UNVERIFIED','bag_input_CDR_index_sha256':corpus['index_sha256']})
    print(json.dumps({'status':stage['status'],'headers_complete':matched,'out':str(out),'actual_navigation_PASS':False}))
    return 0 if matched and stage['status']=='passed_limited_trace_and_lifecycle' else 1


if __name__=='__main__':raise SystemExit(main())
