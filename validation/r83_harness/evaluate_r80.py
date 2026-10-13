# Publication 2026-10-13: local paths redacted; numeric/control behavior retained; local admission is not transferred.
"""Post-run independent reads and original-predicate replay. Never starts a node.

Original verifier seal is replaced in this process by exact comparison of the
already saved bytes; this evaluator cannot write to the actual simulation run.
Only the resulting evaluation is created under the task stage, after closure.
"""
from pathlib import Path
import copy, datetime, hashlib, json, sys

T=Path(__file__).resolve().parent.parent
S=Path(__file__).resolve().parent
C=Path('./multifloor_demo/teacher_mode/navigation/engineering_v63_r83_parking_hysteresis_candidate')
R=Path(json.loads((S/'PREPARED_RUN.json').read_text())['run'])
assert S.name=='R83_PARKING_HYSTERESIS_STAGE' and 'closed_loop_R83_parking_hysteresis_' in R.name
D=C.parents[2]
sys.path[:0]=[str(S),str(C),str(C.parent),str(D),str(D/'navigation')]
from compact_archives import open_history
from continuous_route import completion_valid
from navigation.goal_regions import contains, contains_control, parse_goal
import mission46_runtime_evidence as original

def read(p): return json.loads(Path(p).read_text())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
 return h.hexdigest()
def lines(p):
 if not Path(p).exists(): return
 with Path(p).open() as f:
  for n,line in enumerate(f,1):
   if line.strip(): yield n,json.loads(line)

manifest=read(R/'runtime_manifest.json')
outcome=read(S/'RUN_OUTCOME.json')
host=read(S/'HOST_PROCESS_CLOSURE.json')
budget=read(S/'POSTROUND_BUDGET.json')
assert host['all_owned_and_guardian_groups_absent'], 'Only evaluate after actual host closure'
integrity={};latest={};completed={};first_failed=None
for row in open_history(R/'mission46_status_history.jsonl',integrity=integrity,require_complete=True):
 n=row.get('navigation') or {};rid=n.get('request_id')
 if row.get('stage')=='failed' and first_failed is None:first_failed=copy.deepcopy(row)
 if not rid:continue
 latest[rid]=copy.deepcopy(n)
 if n.get('state')=='succeeded' and rid not in completed:
  completed[rid]=dict(sim_ns=row['sim_ns'],original_check=completion_valid(n,n['goals_definitions'],rid,n['goals_definition_sha256'],n.get('pose')))
last_nav=read(R/'navigation_status.json')
if last_nav.get('request_id'):latest[last_nav['request_id']]=last_nav
requests=[row for _,row in lines(R/'mission46_requests.jsonl')]
expected=[g['goal_id'] for row in requests for g in row['request']['goals']]
request_ids=[row['request']['request_id'] for row in requests]
receipts=[r for rid in request_ids for r in latest.get(rid,{}).get('region_arrivals',[])]
goal_map={g['goal_id']:parse_goal(g) for row in requests for g in row['request']['goals']}
hard=[dict(receipt=r,source_rows=0,first=None,last=None,passed=True) for r in receipts if r.get('reason')=='arrived']
needed={r['stamp_ns'] for r in receipts};positions={}
for _,p in lines(R/'navigation_slam_poses.jsonl'):
 stamp=p['stamp_ns']
 if stamp in needed:positions[stamp]=p['position']
 for window in hard:
  r=window['receipt']
  if r['start_stamp_ns']<=stamp<=r['stamp_ns']:
   window['passed'] &= contains(goal_map[r['goal_id']],p['position']) and contains_control(goal_map[r['goal_id']],p['position']) and (window['last'] is None or 0<stamp-window['last']<=200_000_000)
   window['source_rows']+=1;window['first']=stamp if window['first'] is None else window['first'];window['last']=stamp
for w in hard:
 r=w['receipt'];w['original_spatial_dwell_verified']=bool(w['passed'] and w['source_rows']>=3 and w['first']==r['start_stamp_ns'] and w['last']==r['stamp_ns'] and w['last']-w['first']>=400_000_000)
native=dict(rows=0,last_physics_s=None,last_actor_clock_s=None,first_fault=None,first_unsafe=None)
for n,row in lines(R/'telemetry.jsonl'):
 native['rows']+=1;native['last_physics_s']=row['state_physics_world_time'];native['last_actor_clock_s']=row['world_sim_time']
 if row.get('fault') is not None and native['first_fault'] is None:native['first_fault']=dict(line=n,fault=row['fault'])
 if not original.native_safe(row) and native['first_unsafe'] is None:native['first_unsafe']=dict(line=n,physics_s=row['state_physics_world_time'])

comparisons={}
def compare_saved_seal(run,basename,data):
 p=original.inside(run,basename);raw=original.json_bytes(data)
 if p.read_bytes()!=raw:raise ValueError('Original replay differs from saved canonical proof: '+basename)
 comparisons[basename]=dict(passed=True,sha256=sha(p),bytes=len(raw))
 return dict(source_evidence_file=str(p),source_evidence_sha256=hashlib.sha256(raw).hexdigest())
original.seal=compare_saved_seal
def attempt(name,fn):
 try:return dict(passed=True,result=fn())
 except Exception as e:return dict(passed=False,error=type(e).__name__+': '+str(e))

scope=read(R/'navigation_scope.json')
def initialization():
 p=read(R/'mission46_initialization_evidence.json')
 result=original.verify_initialization(R,scope,p['actual_pose_records'],p['actual_imu_records'],p['native_safety_source_records'],p['scene_axis_registration'],p['sensor_health'])
 assert result==read(R/'mission46_initialization_receipt.json')
 return dict(receipt_matches=True,proof=comparisons['mission46_initialization_evidence.json'])
initial=attempt('initialization',initialization)

def terrain():
 actions=[row for _,row in lines(R/'mission46_actions.jsonl')]
 switches=[row for row in actions if row.get('action',{}).get('kind')=='terrain_switch']
 releases=[row for row in actions if row.get('action',{}).get('reason')=='verified_terrain_ack']
 assert len(switches)==len(releases) and 0<len(switches)<=3
 assert [row['action']['phase']for row in switches]==['exploring','returning','navigating'][:len(switches)]
 results=[]
 for i,(switch,release) in enumerate(zip(switches,releases),1):
  observed=S/f'TERRAIN_ACK_OBSERVED_{i}.json';meta=read(S/f'TERRAIN_ACK_OBSERVATION_{i}.json')
  assert sha(observed)==meta['source_raw_sha256']==meta['stage_copy_sha256']
  ack=read(observed);pending=copy.deepcopy(switch['action']);pending.pop('kind')
  assert release['action']['request_id']==ack['request_id']
  assert switch['sim_ns']<=ack['effective_sim_ns']<=release['sim_ns']
  checked=original.verify_terrain(R,scope,ack,pending,release['sim_ns'])
  assert checked['passed'] and checked['ray_count']==187 and checked['maximum_ray_difference_m']<=1e-9
  results.append(dict(goal_id=ack['goal_id'],request_id=ack['request_id'],phase=ack['phase'],effective_sim_ns=ack['effective_sim_ns'],original_release_sim_ns=release['sim_ns'],ray_count=checked['ray_count'],maximum_ray_difference_m=checked['maximum_ray_difference_m'],source_evidence_sha256=ack['source_evidence_sha256'],observed_ACK_sha256=sha(observed)))
 return results
terrain_result=attempt('terrain',terrain)

def rgb():
 receipt=read(R/'mission46_rgb_save_receipt.json');p=read(receipt['source_evidence_file']);stats=original.read_pcd(receipt['saved_file']);meta=p['metadata'];save=meta['save'];health=save['healthy_sensor_evidence'];ages=health['ages']
 assert sha(receipt['source_evidence_file'])==receipt['source_evidence_sha256'] and stats==p['actual_pcd'] and stats['file_sha256']==receipt['saved_file_sha256']
 assert receipt['passed'] and receipt['ground_truth_used'] is False and meta['run_id']==R.name and meta['ground_truth_used'] is False and meta['reference_map_loaded'] is False
 assert meta['frame_id']=='camera_init' and meta['source_topic']=='/cloud_registered' and '/camera/image_color' in meta['color_source'] and p['service_response']['success']
 assert save['complete'] and save['saved_at']>=p['save_requested_epoch_s'] and stats['point_count']>=500 and stats['point_count']==save['point_count'] and stats['unique_colors']>=8
 assert meta['observed_rgb_samples']>=500 and meta['capacity_rejections']==0 and health['error'] is None and health['slam_healthy'] and health['camera_healthy']
 assert all(meta['counts'][k]>0 for k in ('camera','lidar','imu','odom','colored_cloud')) and all(0<=ages[k]<2 for k in ('odom','lidar','imu','full_cloud','camera','colored_cloud'))
 return dict(actual_pcd=stats,source_proof_sha256=receipt['source_evidence_sha256'],historical_metadata_snapshot_checked=True)
rgb_result=attempt('rgb',rgb)

def dynamic():
 receipt=read(R/'mission46_dynamic_receipt.json');p=read(receipt['source_evidence_file']);assert sha(receipt['source_evidence_file'])==receipt['source_evidence_sha256']
 saved_guards=p['native_guard_pre_roll']+p['continuous_clear_guard']+[r['guard_source'] for r in p['native_guard_replay']]
 # The proof repeats clear guard records in both continuity and replay lists.
 # Reconstruct the original unique serialized records before original checks.
 guards={ (r['source_file'],r['source_offset']):r for r in saved_guards }
 guard_rows=sorted(guards.values(),key=lambda r:r['data']['compute_ros_clock_ns'])
 result=original.verify_dynamic(R,p['request_id'],p['actual_controller_held_statuses']+p['actual_recovered_statuses'],p['measured_stop_window']+p['recovery_actual_poses'],guard_rows,[p['actual_trigger']]+p['actual_service_acknowledgments'])
 assert result==receipt
 return dict(receipt_matches=True,proof=comparisons['mission46_dynamic_evidence.json'],service_execution_ACKs=len(p['actual_service_acknowledgments']),recovery_progress_m=p['recovery_progress_m'],actual_entity_evidence=p['actual_entity_evidence'])
dynamic_result=attempt('dynamic',dynamic)

def parking():
 receipt=read(R/'mission46_final_parking_receipt.json');p=read(receipt['source_evidence_file']);assert sha(receipt['source_evidence_file'])==receipt['source_evidence_sha256']
 actions=[]
 for _,record in lines(R/'mission46_actions.jsonl'):
  for action in ([record['action']] if 'action' in record else record.get('actions',[])):
   if action.get('kind')=='final_ordinary_zero_hold':actions.append(action)
 assert actions and all(a['start_stamp_ns']==actions[0]['start_stamp_ns'] for a in actions)
 assert p['interval']==[actions[0]['start_stamp_ns'],actions[0]['start_stamp_ns']+5_000_000_000]
 result=original.verify_parking(R,receipt['request_id'],p['interval'][0],p['actual_slam_poses'],p['native_source_records'],p['actual_controller_statuses'],p['origin'],parking_contract=p['parking_contract'],anchor=p['T0_origin_anchor'])
 assert result==receipt
 return dict(receipt_matches=True,first_final_ordinary_zero_hold_action=actions[0],interval=p['interval'],metrics=p['metrics'],pose_rows=len(p['actual_slam_poses']),native_rows=len(p['native_source_records']),status_rows=len(p['actual_controller_statuses']),proof=comparisons['mission46_final_parking_evidence.json'])
parking_result=attempt('parking',parking)

status=read(R/'mission46_status.json');mission_errors=[dict(line=n,**row)for n,row in lines(R/'mission46_failures.jsonl')];errors=[dict(line=n,**row)for n,row in lines(R/'navigation_controller_errors.jsonl')]
freeze=read(S/'SOURCE_FREEZE.json');mismatch=[p for p,h in freeze['bindings'].items()if not Path(p).is_file()or sha(p)!=h]
independent={}
for name in ('INDEPENDENT_FINAL_NAVIGATION_REVIEW.json','INDEPENDENT_TERMINAL_ARCHIVE_REVIEW.json','INDEPENDENT_FINAL_LOCALIZATION_REVIEW.json','INDEPENDENT_FINAL_RESOURCE_REVIEW.json'):
 p=S/name
 review=read(p)if p.exists()else {}
 independent[name]=dict(present=p.exists(),passed=(review.get('review_passed')is True or review.get('evidence_review_completed')is True),meaning='Completed accurate evidence review; separate runtime/archive/navigation predicates below must also pass',sha256=sha(p)if p.exists()else None)
criteria=dict(actual_original_sequence_completed=status['stage']=='completed'and status['functional_sequence_completed'],full_ordered_46=len(expected)==46 and len(receipts)==46 and [r['goal_id']for r in receipts]==expected,
 original_40_ordinary_6_hard=sum(r.get('reason')=='passed'for r in receipts)==40 and len(hard)==6,all_original_phase_completion_checks=len(completed)==3 and all(v['original_check']for v in completed.values()),
 actual_slam_receipt_positions=all(positions.get(r['stamp_ns'])==r['raw_position']for r in receipts),all_original_hard_spatial_dwell=len(hard)==6 and all(w['original_spatial_dwell_verified']for w in hard),
 initialization=initial['passed'],RGB=rgb_result['passed'],three_terrain_switches=status['terrain_switch_count']==3 and terrain_result['passed'],dynamic=dynamic_result['passed'],first_fixed_five_second_parking=parking_result['passed'],
 no_original_controller_or_task_error=not errors and not mission_errors and first_failed is None and outcome['error']is None and not manifest.get('independent_owned_guardian',{}).get('error'),no_native_fault_or_unsafe=native['first_fault']is None and native['first_unsafe']is None,
 non_GT=status['navigation_ground_truth_used']is False,normal_pipeline_drain=manifest['pipeline_normal_completed'],typed_capture_close=manifest['typed_capture_close_verified'],
 actor_archive=manifest['worker_archive_integrity'].get('verified')is True,all_owned_and_children_clean=manifest['all_owned_and_children_clean'],runtime_closure=manifest['R77_runtime_closure_completed']is True,
 no_cleanup_errors=not manifest['cleanup_errors'],independent_host_closed=host['all_owned_and_guardian_groups_absent']and (host['all_simulation_roles_absent'] and not host['other_simulation_roles'] and not host['original_owner_pid_present']),frozen_inputs_unchanged=not mismatch,
 budget=not budget['stop_reasons'] and budget['conservative_session_growth_bytes']<130000000000 and budget['R77_round_budget']['charged_B']<10000000000,independent_audits=all(d['passed']for d in independent.values()))
criteria['approved_new_parking_contract']=parking_result['passed']and (parking_result.get('result',{}).get('first_final_ordinary_zero_hold_action',{}).get('anchor',{}).get('contract')=='ordinary_zero_ack_fixed5')
criteria['actual_prepared_dependency_closure']=read(S/'PREPARED_DEPENDENCY_CLOSURE.json')['passed']
e=dict(schema='R80_actual_staged_parking_full_route_evaluation/v1',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),run=str(R),candidate=str(C),strict_full_autonomous_navigation_PASS=all(criteria.values()),criteria=criteria,
 completed_original_region_count=len(receipts),ordinary_regions=sum(r.get('reason')=='passed'for r in receipts),hard_regions=len(hard),expected_order=expected,actual_order=[r['goal_id']for r in receipts],phase_completion=completed,hard_source_windows=hard,
 original_status=dict(stage=status['stage'],message=status['message'],sim_ns=status['sim_ns'],functional_sequence_completed=status['functional_sequence_completed'],terrain_switch_count=status['terrain_switch_count']),first_failed_mission=first_failed,original_task_operation_errors=mission_errors,controller_errors=errors,native=native,
 initialization_replay=initial,terrain_replay=terrain_result,RGB=rgb_result,dynamic_replay=dynamic_result,parking_replay=parking_result,compact_history_integrity=integrity,frozen_input_mismatches=mismatch,independent_reviews=independent,
 closure=dict(normal_pipeline=manifest['pipeline_normal_completed'],typed_capture=manifest['typed_capture_close_verified'],worker=manifest['worker_archive_integrity'],runtime_completed=manifest['R77_runtime_closure_completed'],cleanup_errors=manifest['cleanup_errors']),
 budget=dict(round_charged_B=budget['R77_round_budget']['charged_B'],shared_charged_B=budget['conservative_session_growth_bytes'],stop_reasons=budget['stop_reasons']),
 limitations=['Original hard dwell is spatial/control-region dwell, not an added physical-stop definition.','SetEntityPose ACKs are service execution evidence; independent entity pose/info observer was not collected.','RGB metadata is the saved contemporaneous proof snapshot, not the later mutable live metadata file.','First actual identity group is not formal complete R6; simulation instantaneous point-time model is not mechanical LiDAR certification.','Native require_complete_imu=false remains frozen; healthy localization does not prove every original update complete.'],
 source_evidence_sha256={str(p):sha(p)for p in [R/'runtime_manifest.json',R/'mission46_status.json',R/'mission46_status_history.compact.jsonl',R/'navigation_status.compact.jsonl',R/'navigation_slam_poses.jsonl',R/'telemetry.jsonl',S/'RUN_OUTCOME.json',S/'SOURCE_FREEZE.json',S/'HOST_PROCESS_CLOSURE.json',S/'POSTROUND_BUDGET.json']})
with(S/'EVALUATION.json').open('x')as f:json.dump(e,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
print(json.dumps(dict(strict_PASS=e['strict_full_autonomous_navigation_PASS'],regions=len(receipts),criteria=criteria,sha256=sha(S/'EVALUATION.json')),ensure_ascii=False))
