#!/usr/bin/env python3
"""Isolated actual LIVMapper semantic fixtures; explicit cue required to build/run."""
from __future__ import annotations
import argparse,hashlib,json,os,shlex,struct,subprocess,datetime
from pathlib import Path
HERE=Path(__file__).resolve().parent
TEACHER=HERE.parents[2]
PROJECT=TEACHER.parents[1]
WORKSPACE=TEACHER/'navigation/ingress_pipeline_v17/slam_ws'
SEALED=Path('/var/tmp/go2_teacher_simulation_20261005/20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def append(path,data):
    with path.open('x')as f:f.write(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def raw_rows(path):
    result=[]
    with path.open('rb')as f:
        if f.read(16)!=b'FLIVODIAG0001LE\0':raise ValueError('Unexpected diagnostic format')
        while True:
            h=f.read(56)
            if not h:break
            if len(h)!=56:raise ValueError('Truncated raw header')
            v=struct.unpack('<7Q',h);p=f.read(8*v[-1])
            if len(p)!=8*v[-1]:raise ValueError('Truncated raw payload')
            result.append((v,struct.unpack('<'+'d'*v[-1],p)))
    return result
def main():
    a=argparse.ArgumentParser();a.add_argument('--compile',action='store_true');a.add_argument('--run',action='store_true');a.add_argument('--output',type=Path,default=HERE/'v17_semantic_production_fixtures');args=a.parse_args()
    out=args.output.resolve()
    if not out.is_relative_to(HERE):raise ValueError('Only owned evaluation output')
    source=HERE/'v17_header_exception_fixture.cpp';lib=WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so'
    flags={}
    for line in(WORKSPACE/'build/fast_livo2_core/CMakeFiles/fast_livo2_core.dir/flags.make').read_text().splitlines():
        if line.startswith('CXX_'):k,val=line.split('=',1);flags[k.strip()]=shlex.split(val)
    vikit=PROJECT/'slam5_navigation/ros2_ws/install';executable=out/'fixture'
    cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],str(source),str(lib),str(vikit/'vikit_common/lib/libvikit_common.so'),str(vikit/'vikit_ros/lib/libvikit_ros.so'),'/usr/lib/x86_64-linux-gnu/libpcl_common.so','/usr/lib/x86_64-linux-gnu/libopencv_core.so','/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so','/opt/ros/jazzy/lib/librclcpp.so','/opt/ros/jazzy/lib/librcutils.so','/opt/ros/jazzy/lib/librcl.so','-Wl,-rpath,'+str(lib.parent),'-Wl,-rpath-link,/opt/ros/jazzy/lib','-o',str(executable)]
    env=dict(os.environ);env.update(ROS_DOMAIN_ID='231',OMP_NUM_THREADS='4',OMP_DYNAMIC='FALSE',OPENBLAS_NUM_THREADS='1',FASTLIVO_BOUNDARY_TIMING='0',FASTLIVO_DIAGNOSTIC_BEGIN='0',FASTLIVO_DIAGNOSTIC_END='20')
    env['LD_LIBRARY_PATH']=':'.join([str(lib.parent),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu','/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',str(vikit/'vikit_common/lib'),str(vikit/'vikit_ros/lib'),env.get('LD_LIBRARY_PATH','')])
    if not args.compile and not args.run:
        print(json.dumps({'status':'PREPARED_NOT_EXECUTED','compile_argv':cmd,'ROS_DOMAIN_ID':231,'scope':'No subscriptions/spin/mapping/Gazebo/model. Actual candidate LIVMapper constructor+direct/receiver/commit only.'}));return
    out.mkdir(parents=True,exist_ok=True)
    cpp=WORKSPACE/'src/fast_livo2_core/src/LIVMapper.cpp';header=WORKSPACE/'src/fast_livo2_core/include/fast_livo2_core/core/LIVMapper.h';queue=header.parent/'ordered_ingress.h'
    if lib.stat().st_mtime_ns<max(x.stat().st_mtime_ns for x in(cpp,header,queue)):raise ValueError('Candidate library predates repaired headers/source; root must rebuild before fixture execution')
    if args.compile:
        result=subprocess.run(cmd,env=env,text=True,capture_output=True);(out/'compile.log').write_text(result.stdout+result.stderr)
        receipt=dict(schema='actual_v17_semantic_fixture_build/v1',created_UTC=datetime.datetime.now(datetime.timezone.utc).isoformat(),compile_argv=cmd,returncode=result.returncode,source_sha256={str(x):sha(x)for x in(source,cpp,header,queue,WORKSPACE/'build/fast_livo2_core/CMakeFiles/fast_livo2_core.dir/flags.make')},library=str(lib.resolve()),library_sha256=sha(lib),env={k:env[k]for k in('ROS_DOMAIN_ID','LD_LIBRARY_PATH','OMP_NUM_THREADS','OMP_DYNAMIC','OPENBLAS_NUM_THREADS')},meaning='Fixture build only, no semantic or physical pass')
        if result.returncode==0:receipt['executable_sha256']=sha(executable)
        append(out/'build_receipt.json',receipt)
        if result.returncode:raise RuntimeError(result.stderr[-4000:])
    if not args.run:return
    build=json.loads((out/'build_receipt.json').read_text())
    if sha(executable)!=build.get('executable_sha256')or sha(lib)!=build['library_sha256']or any(sha(Path(x))!=h for x,h in build['source_sha256'].items()):raise ValueError('Built fixture/source/library changed')
    config=out/'navigation.yaml';camera=out/'camera.yaml'
    if not config.exists():config.write_bytes((SEALED/'navigation_fastlivo.yaml').read_bytes())
    snap=json.loads((SEALED/'navigation_source_snapshots.json').read_text());camera_ref=next(x for k,x in snap.items()if k.endswith('/camera_mode/slam/camera.yaml'))
    camera_src=Path(camera_ref['snapshot'])
    if sha(camera_src)!=camera_ref['sha256']:raise ValueError('Camera source binding changed')
    if not camera.exists():camera.write_bytes(camera_src.read_bytes())
    cases=['duplicate_bad','near_duplicate_bad','backward_bad','under_20ms_bad','fresh_bad','fresh_valid','fresh_empty','cloud_standard_filter']
    results=[];pairs=[]
    for case in cases:
        pair=[]
        for mode in('direct','queued'):
            folder=out/f'{case}_{mode}';folder.mkdir(exist_ok=False)
            argv=[str(executable),mode,case,str(folder),'--ros-args','--params-file',str(config),'--params-file',str(camera)]
            proc=subprocess.run(argv,env=env,text=True,capture_output=True,timeout=120);(folder/'execution.log').write_text(proc.stdout+proc.stderr)
            row=dict(case=case,mode=mode,returncode=proc.returncode,command=argv,execution_log_sha256=sha(folder/'execution.log'))
            if proc.returncode==0:
                row.update(canonical_sha256=sha(folder/'canonical.bin'),error_sha256=sha(folder/'callback_error.txt')if (folder/'callback_error.txt').exists()else None)
                stats=json.loads((folder/'raw_diagnostics/writer_stats.json').read_text())
                row['raw_writer_final_lossless']=stats.get('final')is True and stats.get('dropped')==0 and stats.get('writer_io_failed')is False
                r=raw_rows(folder/'raw_diagnostics/records.bin');rawkind=2 if case=='cloud_standard_filter'else 3;source=[x for x in r if x[0][0]==rawkind]
                if len(source)!=1:raise ValueError('Fixture lacks unique actual original source diagnostic')
                row['raw_source_ns']=source[0][0][2];row['raw_receipt_wall']=source[0][1][0]
                if mode=='queued'and case!='fresh_bad':
                    text=(folder/'fixture_ingress.csv').read_text().strip().split(',')
                    row['queued_original_source_and_receipt_exact']=int(text[2])==source[0][0][2]and float(text[3])==source[0][1][0]
                    if not row['queued_original_source_and_receipt_exact']:raise ValueError('Refreshed source receipt or changed original source header')
            results.append(row);pair.append(row)
        same=all(x['returncode']==0 for x in pair)and pair[0].get('canonical_sha256')==pair[1].get('canonical_sha256')and pair[0].get('error_sha256')==pair[1].get('error_sha256')and all(x.get('raw_writer_final_lossless')for x in pair)
        pairs.append(dict(case=case,passed=same,scope='Actual direct original callback branch vs worker decode + owner commit; selected source/header/error semantics only'))
    folder=out/'hilti_rejected';folder.mkdir(exist_ok=False);argv=[str(executable),'queued','hilti_rejected',str(folder),'--ros-args','--params-file',str(config),'--params-file',str(camera)]
    proc=subprocess.run(argv,env=env,text=True,capture_output=True,timeout=120);(folder/'execution.log').write_text(proc.stdout+proc.stderr)
    guard=dict(case='literal_hilti_configuration_rejected',passed=proc.returncode==0,returncode=proc.returncode,execution_log_sha256=sha(folder/'execution.log'))
    receipt=dict(schema='independent_actual_v17_selected_semantic_fixtures/v1',status='PASS_LIMITED_SEMANTICS'if all(x['passed']for x in pairs)and guard['passed']else'FAILED',scope='Selected actual production receive/commit callback data/header/error semantics. No complete estimator trajectory, performance, runtime queue order generalization or navigation pass.',build_receipt_sha256=sha(out/'build_receipt.json'),config_sha256=sha(config),camera_sha256=sha(camera),source_configuration_run=str(SEALED),case_pairs=pairs,negative_guard=guard,processes=results,physics_started=False,model_loaded=False,publishers_subscriptions_or_spin_initialized=False,evaluator_sha256=sha(Path(__file__)))
    append(out/'semantic_receipt.json',receipt);print(json.dumps({'status':receipt['status'],'receipt':str(out/'semantic_receipt.json'),'sha256':sha(out/'semantic_receipt.json')}))
if __name__=='__main__':main()
