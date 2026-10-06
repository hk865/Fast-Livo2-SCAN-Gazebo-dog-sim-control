"""Independent original46 file/physical evaluator. Never writes into the run.

The original region geometry/dwell implementation and frozen receipt verifier
are reused only after their archived/current source bytes have been checked.
Missing future phases stay UNVERIFIED; known runtime/route faults remain FAIL.
"""
from __future__ import annotations
import argparse,bisect,copy,csv,hashlib,importlib.util,json,math,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
TOP=HERE.parents[2]
V19=TOP/'navigation/pipeline_v19'
sys.path.insert(0,str(V19))
from mission46_profile import FROZEN_SHA,ORIGINAL_SCENARIO_SHA,ROUTE_COUNTS,validate_profile
from mission.state_machine import Mission
from navigation.goal_regions import parse_goal,contains_control
import mission46_runtime_evidence as evidence


def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for data in iter(lambda:f.read(1<<20),b''):h.update(data)
 return h.hexdigest()

def read(p):return json.loads(Path(p).read_text())
def rows(p):
 with Path(p).open()as f:
  for line in f:
   if line.strip():yield json.loads(line)
def ck(value,**details):return dict(status='unverified'if value is None else'passed'if value else'failed',passed=value,**details)
def wrapped(operation):
 try:return ck(True,result=operation())
 except FileNotFoundError as err:return ck(None,error=str(err))
 except Exception as err:return ck(False,error=type(err).__name__+': '+str(err))
def module(p,name):
 spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def snapshot(run,basename,required_sha=None):
 candidates=[(source,row)for source,row in read(run/'navigation_source_snapshots.json').items()if Path(source).name==basename]
 if not candidates:raise FileNotFoundError('No archived source '+basename)
 hashes={row['sha256']for _,row in candidates}
 if len(hashes)!=1 or required_sha is not None and hashes!={required_sha}:raise ValueError('Ambiguous/foreign source '+basename)
 for source,row in candidates:
  p=Path(row['snapshot']).resolve()
  if not p.is_relative_to(run/'sources')or sha(p)!=row['sha256']:raise ValueError('Archived source binding differs '+basename)
 return Path(candidates[0][1]['snapshot'])

def source_closure(run,scope):
 errors=[];n=0
 for source,row in read(run/'navigation_source_snapshots.json').items():
  p=Path(row['snapshot']).resolve()
  if not p.is_relative_to(run/'sources')or not p.is_file()or sha(p)!=row['sha256']or scope['references'].get(source)!=row['sha256']or scope['references'].get(str(p))!=row['sha256']:errors.append(source)
  n+=1
 for path,expected in scope['references'].items():
  p=Path(path).resolve()
  if p.is_relative_to(run)and(not p.is_file()or sha(p)!=expected):errors.append(path)
 for path in (V19/'mission46_profile.py',TOP.parent/'mission/state_machine.py',TOP.parent/'navigation/goal_regions.py',V19/'mission46_runtime_evidence.py'):
  if sha(path)!=sha(snapshot(run,path.name)):errors.append('Reused reader module changed '+str(path))
 if errors:raise ValueError(errors[:10])
 return dict(archived_source_count=n,run_local_reference_hashes_verified=True,current_pure_modules_match_archives=True)

def bound_proof(run,name,schema,required):
 receipt=read(run/name)
 if receipt.get('schema')!=schema or receipt.get('run_id')!=run.name or receipt.get('binding_verified')is not True or receipt.get('passed')is not True:raise ValueError('Receipt identity/flag differs '+name)
 checks=receipt.get('checks',{})
 if set(checks)!=set(required)or any(c.get('passed')is not True or c.get('status')!='passed'for c in checks.values()):raise ValueError('Receipt mandatory checks differ '+name)
 p=Path(receipt['source_evidence_file']).resolve()
 if not p.is_relative_to(run)or sha(p)!=receipt['source_evidence_sha256']:raise ValueError('Receipt actual proof changed '+name)
 return receipt,read(p)

def readonly_seal(run,name,data):
 p=Path(run)/name
 if read(p)!=data:raise ValueError('Independent recomputation differs from archived proof '+name)
 return dict(source_evidence_file=str(p),source_evidence_sha256=sha(p))

def initialization(run,scope):
 r,p=bound_proof(run,'mission46_initialization_receipt.json','teacher_original_origin_initialization/v1',evidence.INIT_CHECKS)
 out=evidence.verify_initialization(run,scope,p['actual_pose_records'],p['actual_imu_records'],p['native_safety_source_records'],p['scene_axis_registration'],p['sensor_health'])
 if out!=r:raise ValueError('Initialization recomputed receipt differs')
 return dict(actual_imu_samples=len(p['actual_imu_records']),actual_SLAM_samples=len(p['actual_pose_records']),native_safety_records=len(p['native_safety_source_records']),hidden_bias_state_inspected=False)

def phases(run,profile,anchor,poses,laststatus):
 m=Mission();m.scenario=profile['original_scenario'];m.run_id=run.name;m.origin=anchor['origin'];m.heading_alignment=anchor['scene_axis_registration']['heading_receipt']
 # The registration receipt field is intentionally taken from the frozen
 # run's actual once-only measured axis registration, not simulator yaw.
 expected={};actual={};completion={};partial={};errors=[];requestnames=[]
 for ordinal,(phase,count)in enumerate(ROUTE_COUNTS.items(),1):
  action=m.request_route(phase,0)[0];rid=action['request_id'];expected[phase]=dict(schema_version=2,request_id=rid,frame_id=action['frame_id'],goals=action['goals'])
  path=run/'mission46_requests'/f'{phase}_{ordinal}.json'
  if not path.is_file():completion[phase]=ck(None,reason='Phase not executed',expected_regions=count);continue
  request=read(path);requestnames.append(path.name)
  if request!=expected[phase]:errors.append(phase+' original geometry/goal identity changed')
  actual[phase]=request;s=laststatus.get(rid);receipts=(s or{}).get('region_arrivals')or[];partial[phase]=len(receipts)
  complete=bool(s and s.get('state')=='succeeded' and m.region_completion_valid(s,s.get('pose')))
  gaps=[];dwellerrors=[];stamps=sorted(poses)
  for raw,receipt in zip(request['goals'],receipts):
   goal=parse_goal(raw);start=receipt['start_stamp_ns'];end=receipt['stamp_ns'];window=stamps[bisect.bisect_left(stamps,start):bisect.bisect_right(stamps,end)]
   if not window or window[0]!=start or window[-1]!=end or len(window)<2 or max(np.diff(window))>200_000_000 or any(not contains_control(goal,poses[t]['position'])for t in window):dwellerrors.append(receipt.get('goal_id'))
   terminal=poses.get(end)
   if terminal is None or terminal['position']!=receipt['raw_position']:dwellerrors.append(receipt.get('goal_id')+' end raw pose')
   gaps.extend(np.diff(window).tolist())
  completion[phase]=ck(complete and not dwellerrors,completed_regions=len(receipts),required_regions=count,actual_source_dwell_errors=dwellerrors,max_source_gap_ns=max(gaps)if gaps else None,request_id=rid,final_state=(s or{}).get('state'))
 return ck(not errors and len(actual)==3,errors=errors,phase_request_files=requestnames,phase_geometry_preserved=True,phase_region_checks=completion,partial_arrival_counts=partial),completion

def rgb(run):
 r,p=bound_proof(run,'mission46_rgb_save_receipt.json','teacher_mission46_rgb_save/v1',evidence.MAP_CHECKS)
 saved=Path(r['saved_file']).resolve()
 if not saved.is_relative_to(run)or sha(saved)!=r['saved_file_sha256']:raise ValueError('Current-run saved RGB PCD binding differs')
 stat=evidence.read_pcd(saved);meta=p['metadata'];health=meta['save']['healthy_sensor_evidence'];ages=health['ages']
 if stat!=p['actual_pcd']or stat['point_count']!=r['point_count']or stat['unique_colors']<8 or stat['point_count']<500 or meta.get('capacity_rejections')!=0 or meta.get('observed_rgb_samples',0)<500 or meta.get('run_id')!=run.name or meta.get('ground_truth_used')is not False or meta.get('reference_map_loaded')is not False or meta.get('source_topic')!='/cloud_registered'or'/camera/image_color'not in meta.get('color_source','')or not p['service_response']['success']or meta['save']['saved_at']<p['save_requested_epoch_s']or not all(0<=ages[k]<2 for k in('odom','lidar','imu','full_cloud','camera','colored_cloud'))or health.get('error')is not None or health.get('slam_healthy')is not True or health.get('camera_healthy')is not True:raise ValueError('Actual saved RGB count/color/source/health gate differs')
 return dict(point_count=stat['point_count'],unique_colors=stat['unique_colors'],saved_SHA=stat['file_sha256'],historical_metadata_source_SHA=p['metadata_source']['sha256'],metadata_capture='Frozen runtime proof contains the actual metadata at save; latest live map_metadata may later be overwritten',independent_saved_geometry_and_color_verified=True)

def terrain(run,scope,final_ns):
 results=[];expected=[('exploring','exploration:11','lower12','upper23'),('returning','return_origin:5','upper23','lower12'),('navigating','navigation_f1_f3:7','lower12','upper23')]
 for i,(phase,goal,before,after)in enumerate(expected,1):
  path=run/f'mission46_terrain_evidence_{i}.json';p=read(path);intent=p['intent'];pending={k:v for k,v in intent.items()if k!='kind'}
  if(pending['phase'],pending['goal_id'],pending['from_layer'],pending['to_layer'])!=(phase,goal,before,after):raise ValueError('Original terrain transition order differs')
  for key,payload in [('status_read',p['actual_original_navigation_status']),('intent_read',intent)]:
   if json.loads(p[key]['raw_utf8'])!=payload:raise ValueError('Terrain captured original payload differs')
  # Earlier atomic ACK files are superseded. Reconstruct only the join argument
  # from the immutable actual provider proof; do not call it an archived ACK.
  ack=dict(pending,source_evidence_file=str(path),source_evidence_sha256=sha(path),effective_sim_ns=p['effective_sim_ns'])
  out=evidence.verify_terrain(run,scope,ack,pending,final_ns)
  results.append(dict(goal_id=goal,effective_ns=out['effective_sim_ns'],ray_count=out['ray_count'],max_ray_difference_m=out['maximum_ray_difference_m'],source_sha256=sha(path)))
 final=read(run/'mission46_terrain_ack.json')
 if final['source_evidence_sha256']!=results[-1]['source_sha256']or final['goal_id']!=expected[-1][1]:raise ValueError('Final actual ACK differs from third proof')
 return dict(transitions=results,earlier_atomic_ACKs_superseded=True,application_evidence='Immutable actual provider file-read/ray/effective-clock proofs, not fabricated historic ACK records')

def dynamic(run):
 r,p=bound_proof(run,'mission46_dynamic_receipt.json','teacher_mission46_dynamic_obstacle/v1',evidence.DYNAMIC_CHECKS)
 guards=p['native_guard_pre_roll']+p['continuous_clear_guard']+[x['guard_source']for x in p['native_guard_replay']]
 guards=list({(x['source_file'],x['source_offset']):x for x in guards}.values());guards.sort(key=lambda x:x['data']['compute_ros_clock_ns'])
 out=evidence.verify_dynamic(run,r['request_id'],p['actual_controller_held_statuses']+p['actual_recovered_statuses'],p['measured_stop_window']+p['recovery_actual_poses'],guards,[p['actual_trigger']]+p['actual_service_acknowledgments'])
 if out!=r:raise ValueError('Independent dynamic receipt differs')
 return dict(replayed_guard_inputs=len(p['native_guard_replay']),actual_service_ACK_count=len(p['actual_service_acknowledgments']),recovery_progress_m=p['recovery_progress_m'],independent_entity_pose_observed=False,evidence_scope=p['actual_entity_evidence'])

def parking(run):
 r,p=bound_proof(run,'mission46_final_parking_receipt.json','teacher_mission46_final_parking/v1',evidence.PARK_CHECKS)
 out=evidence.verify_parking(run,r['request_id'],r['start_stamp_ns'],p['actual_slam_poses'],p['native_source_records'],p['actual_controller_statuses'],p['origin'])
 if out!=r:raise ValueError('Independent first5s parking receipt differs')
 return dict(interval_ns=p['interval'],metrics=p['metrics'])

def lifecycle(run):
 summary=read(run/'fastlivo_debug/pipeline_v19_summary.json');stop=read(run/'pipeline_normal_stop.json');errors=[]
 if summary.get('schema')!='staged_input_pipeline_v19/v1'or summary.get('normal_completed')is not True or summary.get('context_valid_at_drain')is not True:errors.append('schema/normal/context')
 count=summary.get('accepted')
 if type(count)is not int or count<=0 or summary.get('delivered')!=count or summary.get('committed')!=count:errors.append('accepted/delivered/committed')
 for key in('pending','inflight','ready','bytes','canceled','rejected_capacity','closed_rejections'):
  if type(summary.get(key))is not int or summary[key]!=0:errors.append(key)
 if summary.get('failure')!=''or summary.get('uncommitted_packets')!=[]:errors.append('failure/uncommitted')
 if stop.get('normal_completed')is not True or stop.get('error')is not None or stop.get('summary_sha256')!=sha(run/'fastlivo_debug/pipeline_v19_summary.json')or stop.get('summary')!=summary:errors.append('actual normal stop binding')
 n=0;rx=set();owner=set();decoder={};previous=0
 with(run/'fastlivo_debug/pipeline_v19_events.csv').open()as f:
  for row in csv.DictReader(f):
   n+=1;seq=int(row['sequence']);kind=int(row['kind']);t=[int(row[k])for k in('receipt_wall_ns','raw_enqueue_wall_ns','ready_enqueue_wall_ns','owner_pop_wall_ns','commit_end_wall_ns')]
   if seq!=n or t!=sorted(t)or t[-1]<previous:errors.append('trace order/causality '+str(n))
   previous=t[-1];rx.add(int(row['receive_tid']));owner.add(int(row['owner_tid']));decoder.setdefault(kind,set()).add(int(row['decode_tid']))
 if n!=count or len(rx)!=1 or len(owner)!=1 or rx==owner:errors.append('actual trace count/owner/RX')
 if errors:raise ValueError(errors[:10])
 return dict(actual_trace_commits=n,summary=summary,receiver_TIDs=sorted(rx),owner_TIDs=sorted(owner),decoder_TIDs={str(k):sorted(v)for k,v in decoder.items()},cleanup_exemption=False)

def heading_reference_consistency(run,profile):
 """Check the explicit new reference join, without calling runtime control."""
 source=snapshot(run,'cascade_core.py');controller_source=snapshot(run,'controller.py')
 revision03=(sha(source)=='ba139551805511fbf04b2fbc52fcc8feb9ea9bd60c0fa18251e7d7a37c93e9c9'and sha(controller_source)=='f0e848191e7fb6e109c41cad54475613a439f4c2015a16d9159120fd7a87ee57')
 errors=[];turns=0;drive=0;maximum=0.;legacy_conflicts=[]
 for row in rows(run/'navigation_pid_history.jsonl'):
  c=row.get('cascade')or{};gate=row.get('heading_gate_reference')or{}
  if c.get('controller_updated')is not True:continue
  mode=c.get('mode');ref=c.get('reference_yaw_rad')
  if mode not in ('drive','turn','capture','active_hold'):continue
  tangent=c['local_horizontal_tangent'];ungated=math.atan2(tangent[1],tangent[0])
  if c.get('endpoint_is_fixed_goal')and c['remaining_horizontal_arc_m']<=.15:ungated=c['fixed_goal']['heading_rad']
  if mode in ('capture','active_hold'):expected=c['fixed_goal']['heading_rad']
  elif mode=='turn'and gate.get('phase')=='align':
   turns+=1;locked=gate.get('locked_heading')
   if type(locked)not in(int,float)or not math.isfinite(locked):errors.append([row['sequence'],'Missing actual locked heading']);continue
   expected=locked
   if revision03:
    if c.get('reference_yaw_source')!='external_heading_gate_locked_reference'or c.get('external_turn_heading_rad')!=locked or abs(c.get('ungated_reference_yaw_rad',float('inf'))-ungated)>1e-9:errors.append([row['sequence'],'Revision03 explicit lock/path provenance differs'])
   elif abs(math.atan2(math.sin(ref-locked),math.cos(ref-locked)))>1e-9:legacy_conflicts.append([row['sequence'],ref,locked])
  else:expected=ungated;drive+=1
  diff=abs(math.atan2(math.sin(ref-expected),math.cos(ref-expected)));maximum=max(maximum,diff)
  if diff>1e-9:errors.append([row['sequence'],'Effective yaw reference differs',ref,expected])
 return ck(None if not turns else not errors,revision03_source_pins_verified=revision03,actual_align_turn_updates=turns,other_reference_updates=drive,maximum_effective_reference_error_rad=maximum,errors=errors[:30],legacy_replan_conflicts=legacy_conflicts[:10],runtime_Controller_called=False,full_PI_math_not_asserted=True)

def evaluate(run):
 run=run.resolve();checks={};scope=read(run/'navigation_scope.json');profile=read(run/'navigation_profile.json');final=read(run/'mission46_status.json');runtime=read(run/'runtime_manifest.json');worker=read(run/'worker_result.json');anchor=read(run/'navigation_anchor.json')
 checks['original46_profile_and_archived_sources']=wrapped(lambda:(validate_profile(profile),source_closure(run,scope))[1])
 checks['whole_mission_terminal']=ck(final.get('run_id')==run.name and final.get('stage')=='completed'and final.get('functional_sequence_completed')is True and final.get('completed_region_count')==46,stage=final.get('stage'),message=final.get('message'),completed_phase_regions=final.get('completed_region_count'))
 checks['runtime_and_owned_cleanup']=ck(runtime.get('error')is None and runtime.get('all_owned_and_children_clean')is True and worker.get('completed')is True,error=runtime.get('error'),owned_clean=runtime.get('all_owned_and_children_clean'),worker_samples=worker.get('samples'))
 loaded=read(run/'slam_loaded_binary.json');policy=read(run/'policy_manifest.json');asset=read(run/'asset_manifest.json')
 checks['frozen_Teacher_CPU_sole_actuator']=ck(scope.get('checkpoint_sha256')==policy.get('checkpoint_sha256')==runtime.get('model_sha256')==FROZEN_SHA and policy.get('inference_device')=='cpu'and policy.get('torch_threads')==1 and runtime.get('exclusive_writer')==asset.get('exclusive_writer')=='teacher_sim::TeacherActuator'and asset.get('spawn')==[0.,0.,.3,0.],model_SHA=runtime.get('model_sha256'),Actor_privileged_dimensions=232)
 checks['actual_loaded_SLAM_binary']=ck(loaded.get('run')==str(run)and loaded.get('verified')is True and loaded.get('actual_core_sha256')==loaded['expected']['core']['sha256']and loaded.get('actual_executable_sha256')==loaded['expected']['executable']['sha256'],actual_core_SHA=loaded.get('actual_core_sha256'),actual_executable_SHA=loaded.get('actual_executable_sha256'))
 poses={r['stamp_ns']:r for r in rows(run/'navigation_slam_poses.jsonl')};navs={};groundtruth=[]
 for r in rows(run/'navigation_status.jsonl'):
  groundtruth.append(r.get('navigation_ground_truth_used'));rid=r.get('request_id')
  if rid and (rid not in navs or len(r.get('region_arrivals')or[])>=len(navs[rid].get('region_arrivals')or[])):
   if rid not in navs or navs[rid].get('state')!='succeeded':navs[rid]=r
 checks['actual_SLAM_navigation_source']=ck(bool(poses)and all(r.get('frame_id')=='camera_init'and r.get('child_frame_id')=='demo_slam_body'for r in poses.values())and all(v is False for v in groundtruth)and profile.get('navigation_ground_truth_used')is False,SLAM_pose_count=len(poses),navigation_ground_truth_used=False)
 region_result=wrapped(lambda:phases(run,profile,anchor,poses,navs))
 if region_result['passed']is True:
  allrequests,phasechecks=region_result['result'];checks['three_original_phase_requests']=allrequests
  for phase,c in phasechecks.items():checks[phase+'_original_regions_and_actual_dwell']=c
 else:
  checks['three_original_phase_requests']=region_result
  for phase in ROUTE_COUNTS:checks[phase+'_original_regions_and_actual_dwell']=ck(None,reason='Phase source unavailable')
 before=evidence.seal;evidence.seal=readonly_seal
 try:
  checks['actual_original_origin_standing_initialization']=wrapped(lambda:initialization(run,scope))
  checks['three_causal_privileged_terrain_switches']=wrapped(lambda:terrain(run,scope,int(final['sim_ns'])))
  checks['actual_current_run_saved_RGB']=wrapped(lambda:rgb(run))
  checks['original_dynamic_stop_clear_recovery']=wrapped(lambda:dynamic(run))
  checks['final_first5s_SLAM_and_physical_hold']=wrapped(lambda:parking(run))
 finally:evidence.seal=before
 nativeerrors=[];samples=0;execution=[];maxqd=maxtau=maxroll=maxpitch=0.;minimum=math.inf
 for row in rows(run/'telemetry.jsonl'):
  samples+=1;execution.append({k:row.get(k)for k in('world_sim_time','command','requested')})
  if row.get('fault')is not None:nativeerrors.append([row['sim_time'],row['fault']])
  if row.get('actor_inferred_this_frame')is True:
   if not evidence.native_safe(row):nativeerrors.append([row['sim_time'],'Native safety/finite/contact/actuator limit'])
   if row.get('outer_navigation_ground_truth_used')is not False:nativeerrors.append([row['sim_time'],'Native consumer navigation truth disclosure'])
   maxqd=max(maxqd,max(map(abs,row['qd'])));maxtau=max(maxtau,max(map(abs,row['applied_torque'])));maxroll=max(maxroll,abs(row['rpy'][0]));maxpitch=max(maxpitch,abs(row['rpy'][1]));minimum=min(minimum,row['body_clearance'])
 checks['native_physical_and_actuator_safety']=ck(samples==worker.get('samples')and samples>0 and not nativeerrors,samples=samples,errors=nativeerrors[:30],max_abs_qd_radps=maxqd,max_abs_torque_Nm=maxtau,max_abs_roll_rad=maxroll,max_abs_pitch_rad=maxpitch,min_clearance_m=minimum)
 checks['pipeline_complete_ordered_context_valid_drain']=wrapped(lambda:lifecycle(run))
 checks['actual_heading_reference_consistency']=heading_reference_consistency(run,profile)
 checks['full_200Hz_native_actuator_trace_replay']=ck(None,reason='Only original50Hz policy/native snapshots checked here; strict200Hz actuator trace has not been replayed')
 checks['full_publication_PI_and_all_SCAN_geometry_replay']=ck(None,reason='Full source/ACK/PI/publication/geometry replay not completed by this mission reader; original numerical gates retained')
 statuses=[v['status']for v in checks.values()];status='failed'if'failed'in statuses else'unverified'if'unverified'in statuses else'passed'
 return dict(schema='independent_original46_SLAM_SCAN_mission/v1',run=str(run),run_id=run.name,status=status,passed=status=='passed',checks=checks,simulation_only=True,navigation_ground_truth_used=False,actor_privileged_dimensions=232,real_robot_verified=False,
  evaluator_sha256=sha(__file__),source_bindings={n:sha(run/n)for n in('navigation_scope.json','source_manifest.json','navigation_source_snapshots.json','navigation_profile.json','runtime_manifest.json','worker_result.json','mission46_status.json','navigation_anchor.json')},
  limitations=['Functional46 receipt recomputation; complete command-publication/PI and SCAN trajectory geometry replay is not yet added; explicit revision03 heading reference is checked separately to this reader, so overall formal control/navigation acceptance remains separately unverified.','Dynamic entity evidence uses actual SetEntityPose execution ACKs, not an independent pose/info observer.','Native position is used only for Actor privilege and offline physical safety, never navigation.'],formal_control_math_and_all_path_guard_replay_complete=False)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 if not(args.run/'run_result.json').is_file():raise SystemExit('Run writer has not completed')
 out=evaluate(args.run);args.output.parent.mkdir(parents=True,exist_ok=True)
 with args.output.open('x')as f:json.dump(out,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
 print(json.dumps(dict(output=str(args.output),status=out['status'],checks={k:v['status']for k,v in out['checks'].items()}),ensure_ascii=False))
