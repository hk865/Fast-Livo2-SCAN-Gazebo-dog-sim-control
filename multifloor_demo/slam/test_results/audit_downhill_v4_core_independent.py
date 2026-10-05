"""Immutable B v4 core evidence only; no ROS, CDR scan or payload decoding."""
from pathlib import Path
import hashlib, importlib.util, json, sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal
run=ROOT/'simulation/test_results/20261002_downhill_b_enabled_v4'
load=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=load(run/'downhill_result.json');m=load(run/'fixture_manifest.json')
spec=importlib.util.spec_from_file_location('v4_archived_downhill_contract',run/'staging/downhill_region_contract.py')
C=importlib.util.module_from_spec(spec);spec.loader.exec_module(C)
start,end=r['origin_stamp_ns'],r['terminal_stamp_ns']
ev=C.evaluate_regions(tuple(parse_goal(g) for g in r['goals_definitions']),r['request_id'],r['statuses'],r['poses'],r['truth'],start,end)
R=Rotation.from_matrix(ev['initial_fixed_SE3_evaluation_only']['rotation_world_from_slam'])
raw=[x for x in r['raw_imu'] if start<=x['stamp_ns']<=end]
metric=lambda q:float(np.max(np.abs(Rotation.from_quat(q).as_euler('xyz')[:2])))
crossings={}
for label,t in [('first_hold',44724000000),('second_hold',72250000000),('first_failed',72325000000)]:
    row=next(x for x in raw if x['stamp_ns']==t);gt=C.bounded_pair(t,r['truth'])
    crossings[label]={'stamp_ns':t,'original_raw_metric_rad':row['tilt'],'independent_raw_Euler_metric_rad':metric(row['quaternion']),
        'same_stamp_GT_Euler_metric_rad':metric(Rotation.from_matrix(gt[1]).as_quat()) if gt else None,
        'GT_bracket_indices':list(gt[2]) if gt else None}
orientation=[]
for p in r['poses']:
    if not start<=p['stamp_ns']<=end:continue
    gt=C.bounded_pair(p['stamp_ns'],r['truth'])
    if gt is not None:orientation.append(float((Rotation.from_matrix(gt[1]).inv()*R*Rotation.from_quat(p['q'])).magnitude()))
gap=[{'previous_ns':a['stamp_ns'],'next_ns':b['stamp_ns'],'gap_ns':b['stamp_ns']-a['stamp_ns']} for a,b in zip(raw,raw[1:]) if b['stamp_ns']-a['stamp_ns']!=1000000]
events=[]
for line in (run/'joint_stop_adapter.jsonl').open():
    x=json.loads(line)
    if x['kind'] in ['zero_edge','native_zero_ack','return_complete'] and (44.7<=x['sim']<=45.1 or 72.2<=x['sim']<=72.7):events.append(x)
    elif x['kind']=='actual_champ_command' and x['sim'] in [44.718,44.724,72.246,72.25]:events.append(x)
body=[];modes=[];failed=[]
for line in (run/'body_feedback.jsonl').open():
    x=json.loads(line)
    if type(x.get('enabled')) is bool:modes.append(x['enabled'])
    if x.get('failed') is True:failed.append(x)
    if x.get('predicates',{}).get('latest_imu_stamp_ns') in [44724000000,72250000000,72325000000]:body.append(x)
terminal=load(run/'terminal_receipt.json');runtime=load(run/'full_control_runtime_evidence.json')
frozen=load(ROOT/'test_results/full19_freeze/source_manifest.json')['sha256'];observer=load(run/'actuator/observer_result.json')
checks={
 'original_motion_FAIL_0of2_preserved':not r['passed'] and 'imu_tilt_at_least_0.50' in r['failure'] and all(x['receipt'] is None and x['passed'] is False for x in ev['regions']),
 'ordered_windows_false_not_vacuous_pass':ev['checks']['ordered_two_original_receipt_windows'] is False and ev['checks']['actual_NAV_succeeded'] is False,
 'original_hash_definitions_and_native_timestamp_contract_valid':all(ev['checks'][k] for k in ['actual_matching_goal_hash_and_definitions','native_receipts_immutable','native_sensor_timestamps_valid']),
 'full_active_GT_under_single_initial_SE3':ev['coverage']['passed'] and ev['coverage']['active_pose_rows']==ev['coverage']['matched_pose_rows']==519,
 'independent_raw_and_same_time_GT_confirm_active_failed_tilt':crossings['first_failed']['original_raw_metric_rad']>=.50 and crossings['first_failed']['same_stamp_GT_Euler_metric_rad']>=.49,
 'actual_two_hold_zeros_ACK_return_observed':len([x for x in events if x['kind']=='zero_edge'])==2 and len([x for x in events if x['kind']=='native_zero_ack'])==2 and len([x for x in events if x['kind']=='return_complete'])==2,
 'second_failed_tick_during_return_terminal_before_completion':72255000000<end<72595000000,
 'original_safezero_nominal_contract_reported':r['checks']['actual_safe_zero_and_nominal_return'] is True,
 'actual_body_mode_enabled_no_internal_feedback_failed_tick':bool(modes) and all(modes) and not failed,
 'original_all_five_actual_startup_gates_pass':all(load(run/n)['passed'] for n in ['controller_ready_gate.json','control_parameter_readback.json','startup_control_runtime.json','startup_gait_runtime.json','startup_gate.json']),
 'ownedclean_runtime16_source12_bindings_pass':terminal['checks']['ownedclean'] and runtime['passed'] and len(runtime['checks'])==16 and all(runtime['checks'].values()) and len(m['executed_staging_sha256'])==12 and all(sha(run/'staging'/n)==v for n,v in m['executed_staging_sha256'].items()),
 'original329_current_bytes_unchanged':m['source_sha256']==frozen==load(run/'source_manifest.json')['sha256'] and all(sha(ROOT/n)==v for n,v in frozen.items()),
 'original_no_body_contacts_and_legal_observed_foot_pairs':r['checks']['no_body_contacts'] and r['checks']['four_feet_preregistered_floor_or_ramp_only'],
 'observer_footer_no_capture_failure_or_overwrite':observer['failure'] is None and observer['overwritten_events']==0,
}
report=dict(scope=__doc__,evidence_ready=all(checks.values()),check_count=len(checks),checks=checks,original_passed=r['passed'],original_failure=r['failure'],origin_ns=start,terminal_ns=end,
 active_original_raw_peak=r['active_max_imu_tilt'],active_bridge_holds=r['active_bridge_holds'],original_checks=r['checks'],original_regions=ev,
 same_stamp_actual_guard_crossings=crossings,active_raw_header_count=len(raw),active_raw_non_1ms_gaps=gap,
 orientation_single_initial_SE3={'samples':len(orientation),'RMSE_rad':float(np.sqrt(np.mean(np.array(orientation)**2))),'max_rad':max(orientation)},
 nearest_status_phase_evidence=[{'sample_sim':min(r['statuses'],key=lambda x:abs(x['sim']-t))['sim'],'crossing_sim':t,'phase':min(r['statuses'],key=lambda x:abs(x['sim']-t))['status']['alignment_phase']} for t in [44.724,72.25,72.325]],
 original_adapter_events=events,selected_body_predicate_rows=body,observer_footer_only=observer,terminal_receipt=terminal,
 input_sha256={n:sha(run/n) for n in ['downhill_result.json','fixture_manifest.json','source_manifest.json','runtime_manifest.json','body_feedback.jsonl','joint_stop_adapter.jsonl','terminal_receipt.json','full_control_runtime_evidence.json','actuator/observer_result.json']},
 limits=['All checks certify preserved evidence, not physical PASS. 0/2 receipts, original tilt failure and no succeeded status remain.','First .30+zero 44.724, ACK44.736 and completion45.078; second .30+zero72.250, ACK72.255; .50 fails72.325 before return completion72.595. Actual instability precedes protective zero, so return duration is not established as its unique cause.','Adapter logs use its float simulation clock and original raw uses integer IMU Header; displayed same times are not proof of callback order at sub-millisecond resolution.','Predicates/body contacts are original logs. No CDR metadata or actual JTC/foot payload decoding was performed for this audit; observed sparse contact messages do not prove every foot continuously supported.','v3-to-v4 changes include both bridge arming and deferred FK; differing fresh-run scheduling and microscopic state prevent a single causal attribution.','The .75 return candidate requires its own physical outcome; reference smoothness and command effort are not applied torque or proven actual joint constraints.'])
p=ROOT/'slam/test_results/downhill_v4_B_core_independent_final.json';p.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps({'path':str(p),'sha256':sha(p),'checks':len(checks),'ready':report['evidence_ready'],'false':[k for k,v in checks.items() if not v],'GTcoverage':ev['coverage'],'rawgaps':gap,'crossings':crossings}))
