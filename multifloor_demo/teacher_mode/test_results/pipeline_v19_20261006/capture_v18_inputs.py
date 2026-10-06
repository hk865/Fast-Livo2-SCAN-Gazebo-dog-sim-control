#!/usr/bin/env python3
"""One new actual V18 simulation plus bounded native MCAP sensor capture.

Recorder overhead makes this an input acquisition run, not a speed baseline.
Only the recorder created here and baseline runner's owned groups are signaled.
"""
from pathlib import Path
import argparse,datetime,hashlib,json,os,signal,subprocess,sys,threading,time,uuid
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
V18=ROOT/'navigation/combined_compute_v18'
sys.path.insert(0,str(V18))
import run as baseline
from run_storage import create_run

def main():
 p=argparse.ArgumentParser();p.add_argument('--domain',type=int,default=88);a=p.parse_args()
 stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
 run,_=create_run(ROOT,f'{stamp}_closed_loop_cascade_clock_hold_v19_corpus_capture_{uuid.uuid4().hex[:4]}')
 args=argparse.Namespace(profile=str(HERE/'capture_v18_profile60.json'),label='v19_corpus_capture',domain=a.domain,cpu_python=baseline.DEFAULT_CPU_PYTHON.resolve(),real_time_factor=1.)
 baseline.write_new(run/'runner_request.json',{'schema':'v19_fresh_sensor_corpus_request/v1','run':str(run),'prepare_only':False,'source_helper_sha256':baseline.sha(__file__),'baseline_runner_sha256':baseline.sha(V18/'run.py'),'simulation_only':True,'recorder_overhead_not_speed_baseline':True})
 recorder=None;ident=None;watch=None;finished=threading.Event();lock=threading.Lock();log=None;stopped=None
 state={'schema':'v19_sensor_capture_lifecycle/v1','status':'preparing','run':str(run),'bag':str(run/'sensor_input_bag'),'max_bag_bytes':6*2**30,'max_capture_wall_s':240,'stop_reason':None,'error':None,'simulation_only':True,'historical_raw_not_used':True}
 def stop(reason):
  nonlocal stopped
  with lock:
   if recorder is None or stopped is not None:return
   state['stop_reason']=reason
   stopped=baseline.stop_owned(recorder,ident,parent_first=True)
   state['recorder_stop']=stopped
 def monitor():
  begin=time.monotonic()
  while not finished.wait(.5):
   size=sum(x.stat().st_size for x in(run/'sensor_input_bag').rglob('*')if x.is_file())if(run/'sensor_input_bag').exists()else 0
   state['bag_bytes_last_observed']=size
   if size>state['max_bag_bytes']:stop('6GiB capture budget reached');return
   if time.monotonic()-begin>240:stop('240s wall capture budget reached');return
   if recorder.poll()is not None:
    state['unexpected_recorder_exit']=recorder.returncode;return
 try:
  plan=baseline.prepare(run,args)
  env=baseline.ros_environment();transport=json.loads((run/'cloud_transport_manifest.json').read_text())
  for k in transport['remove_environment_keys']:env.pop(k,None)
  env.update(transport['environment']);env.update(ROS_DOMAIN_ID=str(a.domain),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
  baseline.require_private_domain(a.domain)
  topics=['/demo/slam/lidar_filtered','/demo/teacher/slam/imu','/demo/teacher/slam/image','/clock']
  command=['ros2','bag','record','--storage','mcap','--storage-preset-profile','zstd_fast','--max-cache-size','67108864','--max-bag-size','1073741824','--disable-keyboard-controls','--output',str(run/'sensor_input_bag'),'--topics',*topics]
  state.update(command=command,topics=topics,domain=a.domain,ready_before_physics=True)
  log=(run/'sensor_input_recorder.log').open('x');recorder=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  ident=baseline.identity(recorder.pid)
  if not ident:raise RuntimeError('Cannot bind own recorder process')
  state['recorder_identity']=ident;time.sleep(1.)
  if recorder.poll()is not None:raise RuntimeError('Recorder exited before simulation')
  watch=threading.Thread(target=monitor,name='bounded_capture_monitor',daemon=True);watch.start()
  state['status']='recording_actual_V18_simulation';baseline.write_new(run/'sensor_capture_setup.json',dict(state));print(json.dumps({'actual_run':str(run),'domain':a.domain}),flush=True)
  error=baseline.execute(run,plan)
  state['baseline_runtime_error']=error
  if error:raise RuntimeError(error)
  state['status']='captured_requires_input_identity_audit'
 except Exception as error:
  state.update(status='failed',error=type(error).__name__+': '+str(error))
 finally:
  finished.set()
  if watch:watch.join(timeout=1.)
  stop('simulation completed or failed')
  if log:log.close()
  state['bag_bytes_final']=sum(x.stat().st_size for x in(run/'sensor_input_bag').rglob('*')if x.is_file())if(run/'sensor_input_bag').exists()else 0
  if stopped and(stopped.get('returncode')!=0 or stopped.get('remaining_owned_group_members')):
   state.update(status='failed',error=state.get('error')or'Recorder did not finish cleanly')
  baseline.write_new(run/'sensor_capture_lifecycle.json',state)
  baseline.write_new(run/'run_result.json',{'run':str(run),'status':state['status'],'runtime_error':state['error'],'capture_receipt_sha256':baseline.sha(run/'sensor_capture_lifecycle.json'),'independent_navigation_validation':'unverified','captured_run_is_uninstrumented_speed_baseline':False})
  print(json.dumps(state,ensure_ascii=False),flush=True)
 if state['status']=='failed':raise SystemExit(1)
if __name__=='__main__':main()
