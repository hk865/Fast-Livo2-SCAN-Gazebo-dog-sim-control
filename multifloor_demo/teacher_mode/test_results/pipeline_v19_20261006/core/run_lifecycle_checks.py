from pathlib import Path
import hashlib,json,subprocess,os,csv
if not __debug__:raise RuntimeError('Optimized Python validation forbidden')
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[4];WS=ROOT/'multifloor_demo/teacher_mode/navigation/pipeline_v19/slam_ws';old=ROOT/'slam5_navigation/ros2_ws/install';ros=Path('/opt/ros/jazzy/lib');h=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();lib=WS/'install/fast_livo2_core/lib/libfast_livo2_core.so';env={**os.environ,'ROS_DOMAIN_ID':'230','OMP_NUM_THREADS':'4','OMP_DYNAMIC':'FALSE','OPENBLAS_NUM_THREADS':'1','FASTLIVO_BOUNDARY_TIMING':'0','LD_LIBRARY_PATH':':'.join([str(lib.parent),str(old/'vikit_common/lib'),str(old/'vikit_ros/lib'),'/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib','/home/hyh001/projects/third_party/Livox-SDK2/install/lib',str(ros),str(ros/'x86_64-linux-gnu')])};rows=[]
for mode in ['rx_decode','staged']:
 for case,opt in [('normalbusy',0),('normalbusy',1),('overflow',0),('contextstop',0)]:
  folder=HERE/'lifecycle_outputs_atomic_final'/f'{mode}_{case}_copy{opt}';folder.mkdir(parents=True,exist_ok=False);cmd=[str(HERE/'production_fixture'),'queued',case,str(folder),'--ros-args','--params-file',str(HERE/'navigation_fixture.yaml'),'--params-file',str(HERE/'camera_fixture.yaml')];r=subprocess.run(cmd,env={**env,'FASTLIVO_PIPELINE_MODE':mode,'FASTLIVO_IMAGE_COPY_OPT':str(opt)},capture_output=True,text=True,timeout=30);(folder/'execution.log').write_text(r.stdout+r.stderr);assert r.returncode==0,(mode,case,r.returncode,r.stdout,r.stderr)
  summaries=list(folder.rglob('pipeline_v19_summary.json'));assert len(summaries)==1;summary=json.loads(summaries[0].read_text());trace=summaries[0].parent/'pipeline_v19_events.csv';events=list(csv.DictReader(trace.open()));life=list(csv.DictReader((summaries[0].parent/'pipeline_v19_lifecycle.csv').open()));assert summary['schema']=='staged_input_pipeline_v19/v1'
  if case=='normalbusy':
   assert summary['normal_completed']is True and summary['context_valid_at_drain']is True and summary['accepted']==summary['delivered']==summary['committed']==128 and summary['pending']==summary['inflight']==summary['ready']==summary['bytes']==summary['canceled']==summary['rejected_capacity']==summary['closed_rejections']==0 and not summary['failure']
   assert len(events)==128 and [int(x['sequence'])for x in events]==list(range(1,129))
   order=[x['event']for x in life];assert order==['start','receiver_cancel_begin','receiver_joined','admission_close','decoders_joined','owner_drain_begin','owner_drain_complete']
   barrier=int(next(x for x in life if x['event']=='admission_close')['wall_ns']);assert all(int(x['raw_enqueue_wall_ns'])<barrier for x in events)
   assert {int(x['kind'])for x in events}=={0,1,2,3}
   for x in events:
    if int(x['kind'])in (0,1):assert all(int(x[k])==0 for k in ['decoder_pop_wall_ns','decode_begin_wall_ns','decode_end_wall_ns','decode_tid','decode_cpu_begin_ns','decode_cpu_end_ns'])
    else:
     assert int(x['receipt_wall_ns'])<=int(x['raw_enqueue_wall_ns'])<=int(x['decoder_pop_wall_ns'])<=int(x['decode_begin_wall_ns'])<=int(x['decode_end_wall_ns'])<=int(x['ready_enqueue_wall_ns'])<=int(x['owner_pop_wall_ns'])<=int(x['commit_end_wall_ns'])
     assert int(x['decode_tid'])==(int(x['receive_tid'])if mode=='rx_decode'else int(x['decode_tid']))
   if mode=='staged':assert len({x['decode_tid']for x in events if int(x['kind'])in(2,3)})==2 and all(x['decode_tid']!=x['receive_tid']for x in events if int(x['kind'])in(2,3))
  elif case=='overflow':assert summary['normal_completed']is False and summary['accepted']==0 and summary['rejected_capacity']==1 and summary['rejected_packet']['source_ns']==11000000000 and 'capacity'in summary['failure']
  else:assert summary['normal_completed']is False and summary['context_valid_at_drain']is False and summary['accepted']==summary['pending']==summary['canceled']==1 and summary['delivered']==summary['committed']==0 and len(summary['uncommitted_packets'])==1 and summary['uncommitted_packets'][0]['source_ns']==11000000000
  rows.append({'mode':mode,'case':case,'copy_opt':opt,'returncode':r.returncode,'summary':str(summaries[0]),'summary_sha256':h(summaries[0]),'trace_sha256':h(trace),'summary_data':summary})
receipt={'schema':'v19_production_lifecycle_checks/v1','status':'PASS_LIMITED_RX_BUSY_DRAIN_AND_FAILURE_PRESERVATION','process_runs':len(rows),'cases':rows,'actual_ROS_executor_busy_callback':True,'no_Gazebo_no_model':True,'library_sha256':h(lib),'fixture_source_sha256':h(HERE/'production_fixture.cpp')};(HERE/'PRODUCTION_LIFECYCLE_CHECKS.json').write_text(json.dumps(receipt,indent=2)+'\n');print('production lifecycle',len(rows),'passed')
