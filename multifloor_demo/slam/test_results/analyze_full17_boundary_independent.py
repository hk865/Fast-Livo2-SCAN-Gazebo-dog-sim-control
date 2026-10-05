#!/usr/bin/env python3
"""Offline read-only audit of archived startup boundaries; no ROS context or physics."""
from pathlib import Path
import collections, difflib, hashlib, json, math, struct
import numpy as np
BASE=Path(__file__).resolve().parents[2]
R=BASE/'simulation/test_results/20261001_full17_startup_boundary_v2'
OUT=BASE/'slam/test_results'
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def named_rows(a,eid,phase=1):return a[(a[:,2]==eid)&(a[:,3]==phase)]
report={'scope':'One cold-start diagnostic; independent offline archived data audit, no control or production changes.', 'run':str(R),'formal_full17_failure_preserved':str(BASE/'runs/20261001_232612_ad2317'), 'diagnostic_result':'NO_ADAPTER_TIMEOUT_REPRODUCED; not a fix or full-demo PASS'}
traces={};arrays={};pairs={}
for role in ['adapter','bridge']:
 m=read(R/(role+'_boundaries.json'));a=np.load(R/(role+'_boundaries.npy'));arrays[role]=a
 starts={};done={};errors=[]
 for row in a:
  call=int(row[4]);phase=int(row[3])
  if phase==1:
   if call in starts: errors.append(['duplicate_enter',call])
   starts[call]=row
  else:
   if call in done:errors.append(['duplicate_completion',call])
   done[call]=row
 unmatched=sorted(starts.keys()-done.keys());orphan=sorted(done.keys()-starts.keys());events={}
 rolepairs=[]
 for call,b in starts.items():
  if call not in done:continue
  e=done[call]
  if b[2]!=e[2] or e[1]<b[1]:errors.append(['bad_pair',call])
  rolepairs.append((b,e))
 for name,eid in m['events'].items():
  z=[(b,e) for b,e in rolepairs if int(b[2])==eid]
  b,e=max(z,key=lambda q:q[1][1]-q[0][1]);ent=named_rows(a,eid)
  post=ent[ent[:,6]>=10_000_000_000]
  if len(post)>1:
   i=int(np.argmax(np.diff(post[:,1])));gap={'duration_ns':int(post[i+1,1]-post[i,1]),'before_entry':post[i].tolist(),'after_entry':post[i+1].tolist()}
  else:gap=None
  events[name]={'calls':len(z),'max_call_duration_ns':int(e[1]-b[1]),'max_call':{'entry':b.tolist(),'completion':e.tolist()},'max_post10_entry_gap':gap}
 traces[role]={'rows':len(a),'unique_calls':len(starts),'sequence_contiguous':bool(np.array_equal(a[:,0],np.arange(len(a)))),'pair_errors':errors,'unmatched':unmatched,'orphan':orphan,'overflow':m['overflow'],'active_call_stack':m['active_call_stack'],'marker_errors':m['marker_errors'],'marker_worker_clean':m['marker_worker_clean'],'first_bridge_failure':m['first_bridge_failure'],'exceptions':m['exceptions'],'events':events}
 pairs[role]=rolepairs
report['trace_pairing']=traces
q=traces['adapter']['events']['callback.clock']['max_post10_entry_gap'];lo=q['before_entry'][1];hi=q['after_entry'][1]
report['common_pause']={'adapter_clock_entry_gap_ns':hi-lo,'wall_begin_ns':lo,'wall_end_ns':hi,'callback_sim_semantics':'Latest callback state only; it is not the current message header.', 'actual_calls_inside':{}}
for role in ['adapter','bridge']:
 a=arrays[role];m=read(R/(role+'_boundaries.json'));report['common_pause']['actual_calls_inside'][role]={name:int(np.count_nonzero((a[:,2]==eid)&(a[:,3]==1)&(a[:,1]>lo)&(a[:,1]<hi))) for name,eid in m['events'].items()}
sources=collections.defaultdict(list); commands=collections.Counter();targets={};statefailed=[]
for line in (R/'sensor_audit.jsonl').read_text().splitlines():
 d=json.loads(line);sources[d['source']].append(d)
im=sources['imu'];stamps=[int(round(x['stamp']*1e9)) for x in im];post=[x for x in im if x['stamp']>=10]
gapmax=max(zip(post,post[1:]),key=lambda q:q[1]['wall_monotonic']-q[0]['wall_monotonic'])
def tilt(q):return math.acos(max(-1.,min(1.,1-2*(q[0]*q[0]+q[1]*q[1]))))
report['official_IMU']={'count':len(im),'first_stamp_ns':stamps[0],'last_stamp_ns':stamps[-1],'unique_strict_1ms':len(set(stamps))==len(stamps) and all(b-a==1_000_000 for a,b in zip(stamps,stamps[1:])),'max_tilt':max(tilt(x['quaternion']) for x in im),'max_post10_tilt':max(tilt(x['quaternion']) for x in post),'max_post10_wall_gap_s':gapmax[1]['wall_monotonic']-gapmax[0]['wall_monotonic'],'gap_before':gapmax[0],'gap_after':gapmax[1]}
for line in (R/'joint_stop_adapter.jsonl').read_text().splitlines():
 d=json.loads(line);k=d['kind']
 if k in ('actual_champ_command','requested_safe'):
  commands[k]+=1
  if any(v!=0 for v in d['value']):statefailed.append(['nonzero_command',d])
 if k in ('raw_target','filtered_target'):
  if k not in targets: targets[k]={'count':0,'initial':d['positions'],'constant':True}
  targets[k]['count']+=1;targets[k]['constant'] &= d['positions']==targets[k]['initial']
 if d.get('state')=='failed':statefailed.append(['adapter_failed',d])
report['actual_command_and_targets']={'commands':dict(commands),'targets':targets,'violations':statefailed}
prev={};gaps={};lost=[];cdr_counts=collections.Counter()
with (R/'actuator/actuator_startup.cdrlog').open('rb') as f:
 assert f.read(12)==b'STARTUPCDR1\n'
 while b:=f.read(8):
  n,c=struct.unpack('<II',b);h=json.loads(f.read(n));f.seek(c,1);k=h['kind'];cdr_counts[k]+=1
  if k=='middleware_lost':lost.append(h)
  if k in prev and (h.get('header_stamp_ns') or h.get('clock_ns',0))>=10_000_000_000:
   gap=h['wall_monotonic_ns']-prev[k]['wall_monotonic_ns']
   if gap>gaps.get(k,{}).get('gap_ns',-1):gaps[k]={'gap_ns':gap,'before':prev[k],'after':h}
  prev[k]=h
report['observer']={'counts':dict(cdr_counts),'lost_events':lost,'largest_post10_gaps':{k:gaps[k] for k in ['imu','clock','joint','jtc','adapter','actual_command','truth']},'full_cross_stream_capture':False,'boundary_note':'At observer joint/adapter gap resumption, actual DDS publication sequence remains consecutive, but DDS receive is delayed about 330ms. Later actual lost callbacks and sequence jumps make this observer incomplete. The Bridge native Python callback stream is independently continuous and cannot be replaced by this observer.'}
cleanup=read(R/'process_cleanup.json');formal=BASE/'runs/20261001_232612_ad2317';manifest=read(R/'runtime_manifest.json');fm=read(formal/'runtime_manifest.json');sm=read(R/'source_manifest.json')
maps=read(R/'actual_gz_process.json');selected=manifest['artifacts']['controller_manager'];actualCM=any(selected in p['libraries'] for p in maps)
report['provenance']={'source_manifest_equal_full17':sm==read(formal/'source_manifest.json'),'artifacts_equal_full17':manifest['artifacts']==fm['artifacts'],'adapter_settings_equal_full17':manifest['joint_stop_adapter']==fm['joint_stop_adapter'],'timing_config_equal_full17':manifest['controller_timing_configuration']==fm['controller_timing_configuration'],'actual_Gazebo_CM_matches_selected':actualCM,'cleanup':cleanup}
meta=read(R/'map_metadata.json');pc=np.fromfile(R/meta['binary_filename'],dtype=np.dtype({'names':['x','y','z','r','g','b','pad'],'formats':['<f4']*3+['u1']*4,'offsets':[0,4,8,12,13,14,15],'itemsize':16}))
report['SLAM_map_diagnostic']={'live_points':len(pc),'metadata_count_match':len(pc)==meta['point_count'],'finite_xyz':all(bool(np.all(np.isfinite(pc[k]))) for k in ['x','y','z']),'rgb_distinct_triplets':len(set(zip(pc['r'].tolist(),pc['g'].tolist(),pc['b'].tolist()))),'slam_healthy':meta['slam_healthy'],'camera_healthy':meta['camera_healthy'],'capacity_rejections':meta['capacity_rejections'],'save_requested':meta['save']['requested'],'formal_route_or_saved_map_acceptance_performed':False}
report['checks']={'all_boundary_calls_pair':all(not t['pair_errors'] and not t['unmatched'] and not t['orphan'] and t['sequence_contiguous'] for t in traces.values()),'all_trace_buffers_complete':all(not t['overflow'] and not t['active_call_stack'] and not t['marker_errors'] and t['marker_worker_clean'] for t in traces.values()),'no_bridge_failure_observed':traces['bridge']['first_bridge_failure'] is None,'all_logged_body_commands_zero':not statefailed and sum(commands.values())>0,'all_logged_joint_targets_constant':all(t['constant'] for t in targets.values()),'same_original_source_and_control_runtime':all(report['provenance'][k] for k in ['source_manifest_equal_full17','artifacts_equal_full17','adapter_settings_equal_full17','timing_config_equal_full17','actual_Gazebo_CM_matches_selected']),'owned_clean_and_unchanged':all(cleanup[k] for k in ['owned_group_clean','source_unchanged','runtime_unchanged','staging_unchanged']),'official_IMU_complete_1ms':report['official_IMU']['unique_strict_1ms']}
report['limitations']=['Added tracing and CDR subscriptions change scheduling; a fresh readonly Coordinator executes original subscriptions/JPEG but excludes Mission.tick, active routes and HTTP serving.','No 337ms Adapter-only failure reproduced. One 119ms IMU/clock pause occurred while Adapter watchdog/status and Bridge status intake continued. This does not establish a fix or unique cause of Full17.','Python enter/return elapsed includes off-CPU scheduling. No native DDS/controller trigger hook ran here, so CPU/DDS root cause is unproven.','Observer reports loss, and its delayed DDS messages must not be substituted for missing rows or used to infer Bridge status timeout.','300ms adapter deadline, original sensor/command deadlines and all guard thresholds remain unchanged.']
report['input_sha256']={f:sha(R/f) for f in ['source_manifest.json','runtime_manifest.json','process_cleanup.json','adapter_boundaries.npy','bridge_boundaries.npy','adapter_boundaries.json','bridge_boundaries.json','sensor_audit.jsonl','joint_stop_adapter.jsonl','actuator/observer_result.json','actuator/actuator_startup.cdrlog','startup_gate.json','map_metadata.json']}
(OUT/'oct2_full17_startup_boundary_v2_independent.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
# Correct the prior static review's missed callback arity, preserving original failed evidence.
a=BASE/'simulation/test_results/20261001_full17_startup_boundary_v1/staging/diagnostic_stack.launch.py';b=R/'staging/diagnostic_stack.launch.py';fixture=BASE/'simulation/test_results/startup_boundary_ros78_v4/launch_ready/result.json'
correction={'scope':'Independent correction to prior static Ready; the OnProcessIO arity error was missed by static review. Original v1 FAIL remains unchanged.','old_signature_failure_preserved':str(a.parent.parent),'diagnostic_launch_diff':list(difflib.unified_diff(a.read_text().splitlines(),b.read_text().splitlines())),'actual_launch_fixture':read(fixture),'actual_fixture_sha256':sha(fixture),'current_launch_sha256':sha(b),'controller_wrapper_sha256':sha(R/'staging/boundary_trace.py'),'same_original_control_trace_sha256':'4615d777fd5c884f92bbaaab568238669ab2a98b92c6473698924805df876e34','conclusion':'Only unused context argument made optional; actual LaunchService with same Coordinator/Observer and no-physics include passed 5 checks. v2 completed original production cold stack without recreating Full17 timeout; this is not a repair of that failure.'}
(OUT/'oct2_startup_boundary_OnProcessIO_correction_review.json').write_text(json.dumps(correction,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'report':str(OUT/'oct2_full17_startup_boundary_v2_independent.json'),'checks':report['checks'],'commands':report['actual_command_and_targets']['commands'],'common_pause_callcounts':report['common_pause']['actual_calls_inside']},indent=2))
