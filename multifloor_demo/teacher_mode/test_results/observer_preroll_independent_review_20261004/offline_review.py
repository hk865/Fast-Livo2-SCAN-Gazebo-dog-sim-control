#!/usr/bin/env python3
"""Independent actual observer method/ring review. Writes only this audit directory."""
from pathlib import Path
import ast, copy, hashlib, json, time, importlib.util, datetime
from collections import deque
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
 if (OUT/'review.json').exists():raise RuntimeError('Never overwrite independent evidence')
 sources=OUT/'sources';sources.mkdir(exist_ok=False)
 paths={'observer.py':ROOT/'navigation/dynamic/dynamic_obstacle.py','ring.py':ROOT/'navigation/dynamic/observer_preroll.py','schema.json':ROOT/'navigation/dynamic/observer_preroll_schema.json','prepare.py':ROOT/'navigation/dynamic/prepare.py','runtime_io.py':ROOT/'navigation/runtime_io.py'}
 for k,p in paths.items():(sources/k).write_bytes(p.read_bytes())
 current=ast.parse((sources/'observer.py').read_text());old=ast.parse((OUT/'old_v44_observer.py').read_text());checks={}
 def named(tree,name):return next(x for x in tree.body if isinstance(x,(ast.ClassDef,ast.FunctionDef)) and x.name==name)
 def astsame(a,b):return ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False)
 for name in ('Tail','fresh','Trigger','Program','verify_scope','sha'):
  checks['original_'+name+'_AST_identical']=astsame(named(current,name),named(old,name))
 cm=named(current,'main');om=named(old,'main')
 newfixture=next(x for x in cm.body if isinstance(x,ast.ClassDef) and x.name=='Fixture');oldfixture=next(x for x in om.body if isinstance(x,ast.ClassDef) and x.name=='Fixture')
 method=lambda fixture,name:next(x for x in fixture.body if isinstance(x,ast.FunctionDef) and x.name==name)
 nt=copy.deepcopy(method(newfixture,'tick'));ot=method(oldfixture,'tick')
 for x in ast.walk(nt):
  if isinstance(x,ast.If) and ast.unparse(x.test)=="args.role == 'sensor_observer'":
   x.body=[y for y in x.body if not(isinstance(y,ast.Expr) and isinstance(y.value,ast.Call) and ast.unparse(y.value.func)=='self.flush_pre_roll')]
 checks['mover_tick_after_observer_only_branch_AST_identical']=astsame(nt,ot)
 checks['observer_sensor_archive_flush_no_publish_or_service_or_truth_pose_calls']=not any(
  isinstance(x,ast.Call) and isinstance(x.func,ast.Attribute) and x.func.attr in ('publish','call_async','create_publisher','set_position','apply_force')
  for name in ('sensor','archive_xyz','flush_pre_roll') for x in ast.walk(method(newfixture,name)))
 ringnamespace={'deque':deque,'copy':copy}
 exec(compile((sources/'ring.py').read_text(),str(sources/'ring.py'),'exec'),ringnamespace)
 Ring=ringnamespace['ActualCloudPreRoll']
 arr=lambda n=2:np.arange(n*3,dtype='<f4').reshape(n,3).copy()
 row=lambda stamp,h='rawhash':{'source':'cloud','stamp_ns':stamp,'data_sha256':h,'received_monotonic_wall':5.123,'frame_id':'camera_init','navigation_input':False,'fields':[{'name':'x'}]}
 r=Ring();x=arr();before=x.tobytes();meta=row(2**53+123);r.add(meta,x);meta['fields'][0]['name']='changed';d=r.drain()
 checks['integer_stamp_above2pow53_and_old_received_wall_preserved']=d[0][0]['stamp_ns']==2**53+123 and d[0][0]['received_monotonic_wall']==5.123
 checks['original_dtype_shape_Cbytes_unchanged_readonly']=d[0][1].dtype==np.dtype('<f4') and d[0][1].shape==(2,3) and d[0][1].tobytes()==before and not d[0][1].flags.writeable
 checks['metadata_deepcopy_and_single_drain']=d[0][0]['fields'][0]['name']=='x' and r.drain()==[] and r.bytes==0
 r=Ring();r.add(row(0),arr());r.add(row(1000000000),arr());checks['exact1s_retained']=len(r.entries)==2
 r.add(row(1000000001),arr());checks['1s_plus1ns_old_eviction_explicit']=len(r.entries)==2 and r.evicted_expired==1
 r=Ring();r.add(row(10),arr());v=r.add(row(10),arr());checks['same_stamp_same_raw_payload_dedup_only']=v is False and len(r.entries)==1 and r.duplicates==1
 def failed(call):
  try:call()
  except RuntimeError:return True
  return False
 checks['same_stamp_changed_rawpayload_fails']=failed(lambda:r.add(row(10,'different'),arr())) and r.error is not None
 r=Ring();r.add(row(10),arr());checks['backward_original_stamp_fails_not_silent_reorder']=failed(lambda:r.add(row(9),arr())) and r.error is not None
 checks['float_stamp_rejected_not_rounded']=failed(lambda:Ring().add(row(1.1),arr()))
 checks['negative_stamp_rejected']=failed(lambda:Ring().add(row(-1),arr()))
 r=Ring(max_frames=2);r.add(row(1),arr());r.add(row(2),arr());checks['frame_capacity_overflow_fails_with_retained_window']=failed(lambda:r.add(row(3),arr())) and len(r.entries)==2
 r=Ring(max_bytes=24);r.add(row(1),arr());checks['byte_capacity_overflow_fails']=failed(lambda:r.add(row(2),arr())) and r.bytes==24
 checks['bounded_default1s32frames16MiB']=Ring().receipt()['span_sim_s']==1. and Ring().max_frames==32 and Ring().max_bytes==16*1024*1024
 class Writer:
  error=None
  def __init__(self):self.jobs=[];self.rows=[]
  def enqueue(self,f):self.jobs.append(f)
  def append(self,p,v):self.rows.append((Path(p),json.loads(json.dumps(v,allow_nan=False))))
 class Clock:
  def now(self):return SimpleNamespace(nanoseconds=13240000000)
 wall=[100.];timefake=SimpleNamespace(monotonic=lambda:wall[0])
 def readpoints(msg,field_names,skip_nans):
  assert field_names==('x','y','z') and skip_nans is True
  return msg.xyz
 ns={'np':np,'json':json,'hashlib':hashlib,'time':timefake,'point_cloud2':SimpleNamespace(read_points_numpy=readpoints),'args':SimpleNamespace(role='sensor_observer')}
 chosen=[copy.deepcopy(method(newfixture,k)) for k in ('sensor','archive_xyz','flush_pre_roll','tick')]
 clazz=ast.ClassDef(name='ActualObserver',bases=[],keywords=[],decorator_list=[],body=chosen)
 exec(compile(ast.fix_missing_locations(ast.Module(body=[clazz],type_ignores=[])),str(sources/'observer.py'),'exec'),ns)
 Observer=ns['ActualObserver'];testdir=OUT/'mock_sensor_run';testdir.mkdir(exist_ok=False);ns['run']=testdir
 def make():
  o=Observer();o.evidence=Writer();o.pre_roll=Ring();o.pre_roll_flushed=False;o.saved_xyz={};o.observer_error=None;o.counts={};o.last_rgb={};o.directory=testdir/'dynamic_sensor_evidence';o.directory.mkdir(exist_ok=True);o.get_clock=lambda:Clock();return o
 def state(phase,clear=None):
  (testdir/'dynamic_obstacle_state.json').write_text(json.dumps({'phase':phase,'sim_time':13.24,'monotonic_wall':110.,'clear_start_sim_s':clear}))
 def msg(stamp,xyz=None):
  a=arr() if xyz is None else xyz
  return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp//10**9,nanosec=stamp%10**9),frame_id='camera_init'),xyz=a,data=a.tobytes(),width=len(a),height=1,point_step=12,row_step=12*len(a),is_bigendian=False,fields=[SimpleNamespace(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate('xyz')])
 o=make();state('waiting');m=msg(13199999999);o.sensor('cloud',m)
 checks['waiting_actualcloud_no_immediate_disk_xyz']=len(o.pre_roll.entries)==1 and not o.evidence.jobs
 wall[0]=110.;state('entering');o.tick(0) if False else o.tick()
 flushed=[v for p,v in o.evidence.rows if v.get('xyz_file')]
 checks['observer20ms_tick_flushes_preentry_without_newcloud']=o.pre_roll_flushed and len(o.evidence.jobs)==1 and len(flushed)==1 and flushed[0]['stamp_ns']==13199999999
 saved=flushed[0]
 checks['enterflush_original_acquisition_and_receivedwall_not_refreshed']=saved['received_monotonic_wall']==100. and saved['stamp_ns']==13199999999 and saved['pre_roll_flush_monotonic_wall']==110. and saved['fixture_phase']=='waiting'
 checks['enterflush_dtype_shape_decodedhash_and_rawhash_exact']=saved['decoded_xyz_dtype']=='float32' and saved['decoded_xyz_shape']==[2,3] and saved['decoded_xyz_sha256']==hashlib.sha256(m.xyz.tobytes()).hexdigest() and saved['data_sha256']==hashlib.sha256(m.data).hexdigest()
 for f in o.evidence.jobs:f()
 with np.load(o.directory/saved['xyz_file'])as z:checks['compressed_npz_exact_original_xyz_dtype_shape_values']=np.array_equal(z['xyz'],m.xyz) and z['xyz'].dtype==m.xyz.dtype and z['xyz'].shape==m.xyz.shape
 checks['original_source_cloud_array_not_changed_or_marked_readonly']=m.xyz.flags.writeable and m.xyz.tobytes()==arr().tobytes()
 before=len(o.evidence.jobs);o.flush_pre_roll();checks['flush_is_once_single_xyz_write']=len(o.evidence.jobs)==before and o.pre_roll.flushed_frames==1
 # When entry is noticed inside the cloud callback, the last metadata row must still name exact XYZ.
 o=make();state('waiting');o.sensor('cloud',msg(13099999999));state('entering');o.sensor('cloud',msg(13199999999));latest=o.evidence.rows[-1][1]
 checks['entry_callback_lastrow_retains_xyz_shapehash_despite_duplicate_archive']=latest.get('xyz_file')=='cloud_13199999999.npz' and latest['decoded_xyz_shape']==[2,3] and latest['archive_duplicate'] is True and len(o.evidence.jobs)==2
 for phase in ['entering','blocking','leaving']:
  o=make();o.pre_roll_flushed=True;state(phase);o.sensor('cloud',msg(15000000000));o.sensor('raw_lidar',msg(15000000000));checks['original_active_'+phase+'_cloud_and_raw_lidar_still_archived']=len(o.evidence.jobs)==2
 o=make();o.pre_roll_flushed=True;state('clear',20.);o.sensor('cloud',msg(25000000000));o.sensor('cloud',msg(25000000001));checks['original_clear_plus5s_inclusive_boundary_retained']=len(o.evidence.jobs)==1
 o=make();o.pre_roll_flushed=True;state('waiting');o.sensor('raw_lidar',msg(15000000000));checks['raw_lidar_waiting_no_unsolicited_archive_change']=not o.evidence.jobs
 o=make();o.pre_roll_flushed=True;state('blocking');o.sensor('cloud',msg(15000000000));checks['archived_same_stamp_changed_payload_fails_explicitly']=failed(lambda:o.sensor('cloud',msg(15000000000,arr()+1))) and o.observer_error is not None
 # Actual shared writer drains these operations once; queue overflow and disk failures remain explicit.
 spec=importlib.util.spec_from_file_location('observer_audit_runtime_io',sources/'runtime_io.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
 w=mod.EvidenceWriter();records=[]
 for i in range(32):w.enqueue(lambda i=i:records.append(i))
 w.close();checks['actual_single_evidence_writer_drain_order_complete']=records==list(range(32)) and w.completed==32 and w.error is None and not w.thread.is_alive()
 # Full lifecycle does not create any ROS, simulator, navigation output or process signals.
 result={'schema':'actual_observer_preroll_independent_offline_audit/v1','asof_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'passed':all(checks.values()),'checks':checks,'actual_closed_loop_or_coverage_pass_asserted':False,'source_hashes':{k:{'path':str(p),'snapshot_sha256':sha(sources/k),'current_equals_snapshot':sha(p)==sha(sources/k)} for k,p in paths.items()},'script_sha256':sha(__file__),'original_v44_observer_sha256':sha(OUT/'old_v44_observer.py'),'scope':'Actual ring class and observer sensor/archive/flush/tick AST methods with synthetic ROS-shaped arrays and phase files, actual bounded EvidenceWriter; no ROS/sim, signals, navigation output, original source/log writes','limits':['Synthetic point arrays validate retention fidelity and method flow, not actual future acquisition completeness.','Raw PointCloud2 payload SHA and decoded XYZ SHA are distinct; no invented payload reconstruction is claimed.','Cache evicts only samples older than1s; capacity exhaustion/backward/conflict fails explicitly rather than hiding current-window loss.','V44 missing pre-entry XYZ stays failed and unchanged; V45 actual fresh coverage still requires a new run.']}
 (OUT/'review.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({'passed':result['passed'],'checks':checks,'receipt_sha256':sha(OUT/'review.json')}))
 if not result['passed']:raise SystemExit(1)
if __name__=='__main__':main()
