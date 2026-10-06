"""Original v8 core evidence only. No ROS, CDR scan or payload decoding."""
from pathlib import Path
import hashlib,importlib.util,json,sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal
RUN=ROOT/'simulation/test_results/20261002_downhill_disabled_envelope_v8'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=read(RUN/'downhill_result.json');m=read(RUN/'fixture_manifest.json');runtime=read(RUN/'runtime_manifest.json');terminal=read(RUN/'terminal_receipt.json');full=read(RUN/'full_control_runtime_evidence.json');rpc=read(RUN/'control_parameter_readback.json');footer=read(RUN/'actuator/observer_result.json')
spec=importlib.util.spec_from_file_location('v8_immutable_region_contract',RUN/'staging/downhill_region_contract.py');C=importlib.util.module_from_spec(spec);spec.loader.exec_module(C)
start,end=r['origin_stamp_ns'],r['terminal_stamp_ns'];goals=tuple(parse_goal(g) for g in r['goals_definitions'])
ev=C.evaluate_regions(goals,r['request_id'],r['statuses'],r['poses'],r['truth'],start,end)
cal=ev['initial_fixed_SE3_evaluation_only'];rot=Rotation.from_matrix(cal['rotation_world_from_slam'])
raw=[x for x in r['raw_imu'] if start<=x['stamp_ns']<=end];lookup={x['stamp_ns']:x for x in raw}
gaps=[dict(previous_ns=a['stamp_ns'],next_ns=b['stamp_ns'],gap_ns=b['stamp_ns']-a['stamp_ns']) for a,b in zip(raw,raw[1:]) if b['stamp_ns']-a['stamp_ns']!=1000000]
euler=lambda q:float(np.max(np.abs(Rotation.from_quat(q).as_euler('xyz')[:2])))
guards=[];keys=set()
for row in r['aggregate_execution_safety_events']:
 s=row['safety'];key=(s['state'],s.get('holds'))
 if s['state'] in ('hold','failed') and s['reason'].startswith('imu_tilt_') and key not in keys:
  keys.add(key);guards.append(row)
crossings={}
for name,t in [('first_raw_hold',next(x['stamp_ns'] for x in raw if x['tilt']>=.30)),('first_raw_failed',next(x['stamp_ns'] for x in raw if x['tilt']>=.50))]+[(f'hold_{row["safety"]["holds"]}_executed_header',round(row['safety']['imu_stamp']*1e9)) for row in guards if row['safety']['state']=='hold']:
 x=lookup[t];truth=C.bounded_pair(t,r['truth']);crossings[name]=dict(imu_Header_ns=t,original_raw_tilt=x['tilt'],independent_raw_Euler_metric=euler(x['quaternion']),same_Header_GT_Euler_metric=euler(Rotation.from_matrix(truth[1]).as_quat()) if truth else None,GT_bracket_indices=list(truth[2]) if truth else None)
orient=[]
for p in r['poses']:
 if start<=p['stamp_ns']<=end:
  truth=C.bounded_pair(p['stamp_ns'],r['truth'])
  if truth is not None:orient.append(float((Rotation.from_matrix(truth[1]).inv()*rot*Rotation.from_quat(p['q'])).magnitude()))
adapter=[]
for line in (RUN/'joint_stop_adapter.jsonl').open():
 x=json.loads(line)
 if x['kind'] in ('zero_edge','native_zero_ack','return_complete') and 107.5<=x['sim']<=118.5:adapter.append(x)
bodymodes=[];bodyidentity=[];bodyfailed=[]
for line in (RUN/'body_feedback.jsonl').open():
 x=json.loads(line)
 if type(x.get('enabled')) is bool:
  bodymodes.append(x['enabled']);bodyidentity.append(x.get('actual_published_body_pose_roll_pitch',x.get('body_pose_roll_pitch'))==[0,0])
 if x.get('failed') is True:bodyfailed.append(x)
actual=[];rpc_exact=True
fields={1:'bool_value',2:'integer_value',3:'double_value'}
for service,s in rpc['services'].items():
 successes=[a for a in s['attempts'] if a.get('actual_response')]
 a=successes[-1];vals=a['actual_response']['values'];names=a['request_names'];rpc_exact &=len(vals)==len(names)==len(s['expected']) and s['passed']
 for n,v in zip(names,vals):
  typ,value=s['expected'][n];match=v['type']==typ and v[fields[typ]]==value;rpc_exact &= match
  actual.append(dict(service=service,name=n,type=v['type'],value=v[fields[typ]],matches_declared=match))
phase=[]
for row in guards:
 t=row['sim'];before=max((x for x in r['statuses'] if x['sim']<=t),key=lambda x:x['sim']);after=min((x for x in r['statuses'] if x['sim']>=t),key=lambda x:x['sim']);phase.append(dict(guard_callback_clock=t,IMU_Header=row['safety']['imu_stamp'],preceding_status=dict(sim=before['sim'],phase=before['status']['alignment_phase'],state=before['status']['state']),following_status=dict(sim=after['sim'],phase=after['status']['alignment_phase'],state=after['status']['state'])))
frozen=read(ROOT/'test_results/full19_freeze/source_manifest.json')['sha256'];selected=runtime['selected_gait'];prior=runtime['qualified_phase2_gait'];actualgait=read(RUN/'startup_gait_runtime.json');sources=selected['source_sha256'];checks={}
def check(k,v):checks[k]=bool(v)
check('original20_checks_and_1of2_imuFAIL_unchanged',len(r['checks'])==20 and not r['passed'] and r['failure']=='aggregate execution safety failed: imu_tilt_at_least_0.50' and sum(x['receipt'] is not None for x in ev['regions'])==1)
check('first_original80p9_to81p4_same_rawinner_GTouter_window',ev['regions'][0]['passed'] and ev['regions'][0]['receipt']['start_stamp_ns']==80900000000 and ev['regions'][0]['receipt']['stamp_ns']==81400000000 and len(ev['regions'][0]['observations'])==6)
check('second_missing_no_later_window_or_vacuous_PASS',not ev['regions'][1]['passed'] and ev['regions'][1]['receipt'] is None and not ev['checks']['ordered_two_original_receipt_windows'] and not ev['checks']['actual_NAV_succeeded'])
check('immutable_original_hash_and_sensor_timestamp_contract_valid',all(ev['checks'][x] for x in ('actual_matching_goal_hash_and_definitions','native_receipts_immutable','native_sensor_timestamps_valid')))
check('canonical_original2_geometry_and_deadlines_match',r['goals_definitions']==[x.definition() for x in C.downhill_goals(read(ROOT/'simulation/scenario.json'),r['origin'],r['heading_alignment'])] and r['unchanged_contract']==dict(per_goal_timeout_sim_s=90,raw_inner_dwell_sim_s=.4,max_raw_gap_sim_s=.2,independent_GT_outer='same preregistered outer region',truth_bracket_max_s=.15,imu_hold_rad=.3,imu_fail_rad=.5,component_cleanup_bound_sim_s=220))
check('original994_active_GT_paired_one_initial_SE3',ev['coverage']['passed'] and ev['coverage']['active_pose_rows']==ev['coverage']['matched_pose_rows']==994 and not ev['coverage']['unpaired_native_stamps'] and cal['stamp_ns']==start)
check('independent_recomputed_evaluation_equals_original_saved',ev==r['region_evaluation'])
check('same_Header_true_failed_tilt_preserved',crossings['first_raw_failed']['imu_Header_ns']==117389000000 and crossings['first_raw_failed']['same_Header_GT_Euler_metric']>=.49)
check('three_original_hold_actions_and_failed_guard_safezero',len(guards)==4 and [x['safety']['holds'] for x in guards]==[1,2,3,3] and all(x['safety']['safe']==[0,0,0] for x in guards))
check('three_critical_zero_ACK_return_sequences_observed',len(adapter)==9 and sum(x['kind']=='zero_edge' for x in adapter)==sum(x['kind']=='native_zero_ack' for x in adapter)==sum(x['kind']=='return_complete' for x in adapter)==3)
check('final_fail_after_last_return_completed_adapter_idle',next(x for x in adapter if x['kind']=='return_complete' and x['sim']>116)['sim']==117.281 and guards[-1]['safety']['joint_adapter']['state']=='idle' and end==117389000000)
check('actual64_RPC_original_D1_and_all_values_types_exact',rpc['passed'] and len(actual)==64 and rpc_exact and len([a for a in actual if a['name'].endswith('.d') and a['type']==3 and a['value']==1.])==12)
check('feedback_disabled_actual_mode_and_identity_pose',bool(bodymodes) and not any(bodymodes) and all(bodyidentity) and not bodyfailed)
check('original_return_point30_floor_no_private_slow_wrapper',not m['slow_return_candidate'] and 'joint_stop_core.py' not in m['executed_staging_sha256'] and r['checks']['actual_safe_zero_and_nominal_return'])
check('five_actual_startup_gates_all_pass',all(read(RUN/n)['passed'] for n in ('controller_ready_gate.json','control_parameter_readback.json','startup_control_runtime.json','startup_gait_runtime.json','startup_gate.json')))
check('actual_new_gait_maps_c839_a984_all6_pass',actualgait['passed'] and len(actualgait['checks'])==6 and all(actualgait['checks'].values()) and selected['executable']['sha256']=='c839ee6489566793b5c5da180f60d381ede11895865139e4d01324c6fbf3ec67' and selected['library']['sha256']=='a9845d38769129abc75b6af0703b424f1b1308349c9caf3f9f0eba2367da8324')
check('qualified_prior2527_60ae_separately_preserved',prior['executable']['sha256']=='2527d4c9e64b7562a57e6ba8e9559c98b3828eae9463af16df7e78880138df64' and prior['library']['sha256']=='60ae6c054c143ec7751297906cbd9f9ff0b3a6d9eb48084ad7dc8f6deeb9d1f8')
check('ownedclean_runtime16_startup_prestop_source36_alltrue',terminal['checks']['ownedclean'] and full['passed'] and len(full['checks'])==16 and all(full['checks'].values()))
check('all12_end_executed_archives_actual_bytes_exact',len(m['executed_staging_sha256'])==12 and all(sha(RUN/'staging'/p)==d for p,d in m['executed_staging_sha256'].items()))
check('private_source36_leg703a_phase9d0_and4provenance_unchanged',len(sources)==36 and all(sha(Path(selected['source_root'])/p)==d for p,d in sources.items()) and sources['champ/include/champ/leg_controller/leg_controller.h']=='703a6ca811cf718434c98c078f61d1ac3f9dc2d8baf3c9c6e107f920e4a390cb' and all(sha(Path(selected[k]['path']))==selected[k]['sha256'] for k in ('frozen_build_receipt','frozen_source_archive_receipt','frozen_patch','deployment_manifest')))
check('original329_current_bytes_and_manifest_unchanged',len(frozen)==329 and m['source_sha256']==frozen==read(RUN/'source_manifest.json')['sha256'] and all(sha(ROOT/p)==d for p,d in frozen.items()))
check('original_no_body_contacts_legal_observed_foot_pairs',r['checks']['no_body_contacts'] and r['checks']['four_feet_preregistered_floor_or_ramp_only'])
check('observer_footer_all_received_retained_no_error_or_overwrite',footer['failure'] is None and footer['overwritten_events']==0 and footer['total_events']==footer['retained_events']==852264)
report=dict(scope=__doc__,evidence_ready=all(checks.values()),check_count=len(checks),checks=checks,false_checks=[k for k,v in checks.items() if not v],original_motion_passed=r['passed'],original_failure=r['failure'],original_acceptance_checks=r['checks'],original_acceptance_true_count=sum(r['checks'].values()),origin_Header_ns=start,terminal_observerClock_ns=end,original_active_peak=r['active_max_imu_tilt'],active_holds=r['active_bridge_holds'],independent_original_regions=ev,same_Header_raw_and_GT_crossings=crossings,actual_guard_onsets=guards,phase_observation_brackets=phase,original_small_adapter_events=adapter,active_raw_Header_count=len(raw),active_raw_non1ms_gaps=gaps,active_raw_missing_1ms_samples=sum(x['gap_ns']//1000000-1 for x in gaps if x['gap_ns']>1000000),full_native_1ms_driver_sampling_claim=not gaps,one_initial_SE3_orientation=dict(samples=len(orient),RMSE_rad=float(np.sqrt(np.mean(np.array(orient)**2))),max_rad=max(orient)),actual_parameters=actual,actual_body_mode_rows=len(bodymodes),actual_body_pose_identity_rows=sum(bodyidentity),first_body_internal_failed_tick=bodyfailed[0] if bodyfailed else None,recording_footer_only={k:footer[k] for k in ('total_events','retained_events','overwritten_events','middleware_lost_events','failure')},terminal_receipt=terminal,input_sha256={n:sha(RUN/n) for n in ('downhill_result.json','fixture_manifest.json','source_manifest.json','runtime_manifest.json','body_feedback.jsonl','joint_stop_adapter.jsonl','terminal_receipt.json','full_control_runtime_evidence.json','control_parameter_readback.json','startup_gait_runtime.json','startup_control_runtime.json','actuator/observer_result.json')},limits=['Evidence Ready preserves original14/20 required checks true and original1/2 FAIL. It is not physical PASS or startup-envelope adoption.','First original window80.9..81.4s remains rawinner/GTouter joint PASS; second has no receipt. No later window substitutes it.','First raw.30 Header107.995 and Bridge callbackclock107.996 are separate clocks; Adapter logged zero107.995. Their displayed sim values cannot establish cross-node submillisecond wall order.','Second .30 occurs after original zero111.876/ACK111.879, during return. Last return completes117.281, before failed IMU Header117.389 by108ms; finalfailure cannot be described as unfinished last return.','Driver raw IMU2ms gap37.802..37.804s is preserved as one missing sample. No observer footer loss callback does not establish all producer/DDS messages were received. Shared metadata is owned by Sim; this audit does not scan or deserialize CDR.','Foot contact messages observed only on allowed surfaces; no messages is not proof of a foot airborne or continuously supported.','The leg envelope scales current Body gait offset and attenuates internal discontinuities. Actualmaps certify binary loading, not contact stability, exact tracking, applied torque or unique cause.','v8 returns D1.5 to originalD1 versus v7 while selecting envelope; disabled feedback/originalpoint30 are explicit. Fresh states and scheduling differ, preventing only-envelope attribution versusv7.'])
P=ROOT/'slam/test_results/downhill_v8_core_independent_final.json';P.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(path=str(P),sha256=sha(P),evidence_ready=report['evidence_ready'],checks=len(checks),false=report['false_checks'],original=r['passed'],coverage=ev['coverage'],crossings=crossings,rawgaps=gaps)))
