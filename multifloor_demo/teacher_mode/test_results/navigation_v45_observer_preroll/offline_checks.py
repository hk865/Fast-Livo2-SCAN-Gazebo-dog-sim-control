#!/usr/bin/env python3
"""Actual passive sensor methods and bounded original-XYZ ring, no ROS init."""
from pathlib import Path
import ast,hashlib,json,math,sys,tempfile,time
import numpy as np
ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation';sys.path.insert(0,str(NAV))
from dynamic.observer_preroll import ActualCloudPreRoll
from runtime_io import EvidenceWriter
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
checks={}
def check(n,c,e=None):
 checks[n]={'passed':bool(c),'evidence':e}
 if not c:raise AssertionError(n)
def row(stamp,value='a'):
 return {'source':'cloud','stamp_ns':stamp,'received_monotonic_wall':10.,'frame_id':'camera_init','data_sha256':value*64}
cloud=np.array([[.1,.2,.3],[1.,2.,3.]],dtype=np.float32)
c=ActualCloudPreRoll();c.add(row(12_200_000_000),cloud.copy());c.add(row(13_199_999_999),cloud.copy())
first=c.entries[0][0].copy();items=c.drain()
check('full_one_second_original_source_span_retained',len(items)==2 and items[0][0]==first)
check('original_xyz_dtype_shape_bits_immutable',items[1][1].dtype==np.float32 and items[1][1].shape==(2,3)and items[1][1].tobytes()==cloud.tobytes()and not items[1][1].flags.writeable)
check('drain_once_no_duplicate_archive',c.drain()==[]and c.flushed_frames==2 and c.bytes==0)
c=ActualCloudPreRoll();c.add(row(1_000_000_000),cloud.copy());c.add(row(2_000_000_000),cloud.copy());c.add(row(2_000_000_001),cloud.copy())
check('only_expired_source_time_evicted',len(c.entries)==2 and c.evicted_expired==1 and c.entries[0][0]['stamp_ns']==2_000_000_000)
c=ActualCloudPreRoll();c.add(row(1),cloud.copy());check('same_stamp_same_payload_deduplicated',not c.add(row(1),cloud.copy())and len(c.entries)==1 and c.duplicates==1)
for name,setup,stamp,hashchar in [
 ('backward',lambda:ActualCloudPreRoll(),0,'a'),
 ('conflicting_duplicate',lambda:ActualCloudPreRoll(),1,'b'),
 ('frame_overflow',lambda:ActualCloudPreRoll(max_frames=1),2,'a'),
 ('byte_overflow',lambda:ActualCloudPreRoll(max_bytes=cloud.nbytes),2,'a')]:
 c=setup();c.add(row(1),cloud.copy());caught=False
 try:c.add(row(stamp,hashchar),cloud.copy())
 except RuntimeError:caught=True
 check(name+'_is_explicit_failed_evidence',caught and bool(c.error))

source=NAV/'dynamic/dynamic_obstacle.py';tree=ast.parse(source.read_text())
main=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name=='main')
fixture=next(x for x in main.body if isinstance(x,ast.ClassDef)and x.name=='Fixture')
methods=[x for x in fixture.body if isinstance(x,ast.FunctionDef)and x.name in ('sensor','archive_xyz','flush_pre_roll')]
klass=ast.ClassDef(name='OfflinePassiveFixture',bases=[],keywords=[],body=methods,decorator_list=[])
program=ast.fix_missing_locations(ast.Module(body=[klass],type_ignores=[]))
with tempfile.TemporaryDirectory()as tmp:
 run=Path(tmp);directory=run/'dynamic_sensor_evidence';directory.mkdir()
 env={'run':run,'time':time,'hashlib':hashlib,'json':json,'np':np,'point_cloud2':point_cloud2}
 exec(compile(program,str(source),'exec'),env)
 f=env['OfflinePassiveFixture']();f.counts={};f.directory=directory;f.pre_roll=ActualCloudPreRoll();f.pre_roll_flushed=False
 f.saved_xyz={};f.observer_error=None;f.last_rgb={};f.evidence=EvidenceWriter()
 def state(phase,t):
  (run/'dynamic_obstacle_state.json').write_text(json.dumps({'phase':phase,'sim_time':t,'monotonic_wall':time.monotonic(),'clear_start_sim_s':26.}))
 def msg(stamp,xyz=cloud):
  sec,nanosec=divmod(stamp,1_000_000_000)
  return point_cloud2.create_cloud_xyz32(Header(frame_id='camera_init',stamp=Time(sec=sec,nanosec=nanosec)),xyz)
 state('waiting',13.22)
 raw=msg(13_199_999_999);actual_raw_sha=hashlib.sha256(bytes(raw.data)).hexdigest()
 f.sensor('cloud',raw);before=f.pre_roll.entries[0][0].copy()
 check('waiting_actual_method_caches_only_no_xyz_file',not list(directory.glob('*.npz'))and before['stamp_ns']==13_199_999_999)
 state('entering',13.24);f.flush_pre_roll();f.evidence.close()
 payload=np.load(directory/'cloud_13199999999.npz')['xyz'];logs=[json.loads(x)for x in (directory/'sensor_inputs.jsonl').read_text().splitlines()]
 archived=next(x for x in logs if x.get('archive_kind')=='pre_roll')
 check('actual_enter_flush_preserves_preentry40ms_header',archived['stamp_ns']==13_199_999_999 and archived['pre_roll_flush_fixture_sim_time']==13.24)
 check('actual_enter_flush_does_not_rejuvenate_received_wall',archived['received_monotonic_wall']==before['received_monotonic_wall']and archived['original_source_stamps_preserved'])
 check('original_frame_raw_payload_hash_preserved',archived['frame_id']=='camera_init'and archived['data_sha256']==actual_raw_sha)
 check('actual_npz_xyz_shape_dtype_bytes_exact',payload.dtype==cloud.dtype and payload.shape==cloud.shape and payload.tobytes()==cloud.tobytes())
 check('actual_preroll_metadata_is_waiting_not_faked_entering',archived['fixture_phase']=='waiting'and archived['pre_roll_flush_fixture_phase']=='entering')
 f.evidence=EvidenceWriter();state('blocking',14.2);f.sensor('cloud',msg(14_200_000_000));f.sensor('cloud',msg(14_200_000_000))
 state('leaving',24.2);f.sensor('cloud',msg(24_200_000_000));state('clear',26.);f.sensor('cloud',msg(31_000_000_000));f.sensor('cloud',msg(31_000_000_001));f.evidence.close()
 check('original_active_and_clear5s_archives_unchanged',all((directory/f'cloud_{s}.npz').exists()for s in (14_200_000_000,24_200_000_000,31_000_000_000))and not(directory/'cloud_31000000001.npz').exists())
 logs=[json.loads(x)for x in (directory/'sensor_inputs.jsonl').read_text().splitlines()]
 duplicate=[x for x in logs if x['stamp_ns']==14_200_000_000][-1]
 check('duplicate_active_row_keeps_xyz_reference_and_hash',duplicate['archive_duplicate']and duplicate['xyz_file']=='cloud_14200000000.npz'and duplicate['decoded_xyz_sha256']==hashlib.sha256(cloud.tobytes()).hexdigest())
 check('unique_npz_single_writer_no_duplicate_archive',len(list(directory.glob('*.npz')))==4 and len(f.saved_xyz)==4 and f.evidence.error is None)

 # First observed active callback flushes its pre-roll and must keep the latest
 # per-stamp sensor row's file reference, rather than overwrite it with no XYZ.
 f=env['OfflinePassiveFixture']();f.counts={};f.directory=directory/'callback';f.directory.mkdir()
 f.pre_roll=ActualCloudPreRoll();f.pre_roll_flushed=False;f.saved_xyz={};f.observer_error=None;f.last_rgb={};f.evidence=EvidenceWriter()
 state('waiting',13.1);f.sensor('cloud',msg(13_100_000_000));state('entering',13.24);f.sensor('cloud',msg(13_300_000_000));f.evidence.close()
 logs=[json.loads(x)for x in (f.directory/'sensor_inputs.jsonl').read_text().splitlines()]
 last=[x for x in logs if x['stamp_ns']==13_300_000_000][-1]
 check('first_active_callback_latest_metadata_keeps_xyz_file',last['xyz_file']=='cloud_13300000000.npz'and bool(last['decoded_xyz_sha256']))

old=ROOT/'runs/20261004_102555_navigation_slam_scan_dynamic_flat_v44_dwell_r1_1536/sources/navigation/dynamic/dynamic_obstacle.py'
oldtree=ast.parse(old.read_text());oldmain=next(x for x in oldtree.body if isinstance(x,ast.FunctionDef)and x.name=='main')
oldfixture=next(x for x in oldmain.body if isinstance(x,ast.ClassDef)and x.name=='Fixture')
dump=lambda n:ast.dump(n,include_attributes=False)
for name in ('Tail','fresh','Trigger','Program','verify_scope'):
 check('control_fixture_'+name+'_AST_unchanged',dump(next(x for x in tree.body if getattr(x,'name',None)==name))==dump(next(x for x in oldtree.body if getattr(x,'name',None)==name)))
oldtick=next(x for x in oldfixture.body if getattr(x,'name',None)=='tick');newtick=next(x for x in fixture.body if getattr(x,'name',None)=='tick')
check('mover_tick_all_after_observer_branch_AST_unchanged',[dump(x)for x in oldtick.body[3:]]==[dump(x)for x in newtick.body[3:]])
check('observer_methods_contain_no_control_or_service_calls',not any(isinstance(x,ast.Attribute)and x.attr in ('publish','call_async','create_client')for method in methods for x in ast.walk(method)))
freeze=json.loads((ROOT/'test_results/navigation_v44_measured_dwell_actual_guard_freeze.json').read_text())
protected=('navigation/controller.py','navigation/teacher_transition.py','navigation/guard_audit.py','navigation/scoped_profile.py','navigation/stack.launch.py',
 'navigation/dynamic/flat_dynamic_profile_dwell.json','navigation/dynamic/flat_dynamic_profile_turn20.json','navigation/dynamic/protocol.json',
 'policy/worker.py','policy/observation.py','policy/contract.json','simulation/prepare.py','simulation/build/libteacher_actuator.so','scripts/run_test.py','runs/acceptance.json')
check('all_V44_control_profiles_physics_runner_hashes_unchanged',all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==freeze['source_hashes'][n]for n in protected))
receipt={'schema':1,'status':'passed','checks':checks,'ROS_initialized':False,'ROS_nodes_started':False,'Gazebo_started':False,
 'synthetic_test_clouds_are_not_actual_run_evidence':True,'runtime_evidence_completeness':'unverified until new actual run',
 'V44_actual1536_modified':False,'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [source,NAV/'dynamic/observer_preroll.py',NAV/'dynamic/observer_preroll_schema.json',Path(__file__)]}}
out=Path(__file__).with_name('offline_checks.json');out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':'passed','checks':len(checks),'receipt':str(out)}))
