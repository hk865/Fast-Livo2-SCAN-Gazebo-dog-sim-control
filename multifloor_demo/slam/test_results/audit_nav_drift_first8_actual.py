#!/usr/bin/env python3
"""Offline actual first8 audit. Reads original records; no ROS node or publishers."""
from pathlib import Path
import json,importlib.util,sys,math,collections,bisect
import numpy as np
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('audit_helper',P/'analyze_feedback_first4_ab_independent.py');H=importlib.util.module_from_spec(spec);spec.loader.exec_module(H)
run=H.ROOT/'simulation/test_results/20261002_nav_drift_first8_disabled'
r=H.load(run/'first_eight_result.json')
spec=importlib.util.spec_from_file_location('actual_eight_contract',run/'staging/eight_region_contract.py');C=importlib.util.module_from_spec(spec);spec.loader.exec_module(C)
e=C.evaluate_regions(tuple(H.parse_goal(g) for g in r['goals_definitions']),r['request_id'],r['statuses'],r['poses'],r['truth'],r['origin_stamp_ns'],r['terminal_stamp_ns'])
print(json.dumps({'eight_regions_passed':e['passed'],'precision':e['precision'],'coverage':e['coverage']}),flush=True)
R=np.array(e['initial_fixed_SE3_evaluation_only']['rotation_world_from_slam']);T=np.array(e['initial_fixed_SE3_evaluation_only']['translation'])
raw=[x for x in r['raw_imu'] if r['origin_stamp_ns']<=x['stamp_ns']<=r['terminal_stamp_ns']]
raw_stamps=[x['stamp_ns'] for x in raw];raw_gaps=[b-a for a,b in zip(raw_stamps,raw_stamps[1:])]
crossings={str(th):next(({'stamp_ns':x['stamp_ns'],'tilt_rad':H.metric(x['quaternion'])[0],'roll_pitch_rad':H.metric(x['quaternion'])[1:],'raw':x} for x in raw if H.metric(x['quaternion'])[0]>=th),None) for th in [.2,.3,.5]}
attitude=[]
for x in r['poses']:
 if not r['origin_stamp_ns']<=x['stamp_ns']<=r['terminal_stamp_ns']:continue
 gt=C.bounded_pair(x['stamp_ns'],r['truth'])
 if gt is None:continue
 q=Rotation.from_matrix(gt[1]);est=Rotation.from_matrix(R)*Rotation.from_quat(x['q'])
 attitude.append({'stamp_ns':x['stamp_ns'],'error_rad':float((q.inv()*est).magnitude()),'GT_raw_Euler_metric_rad':H.metric(q.as_quat())[0]})
first=None;following=None
for line in (run/'first_eight_events.jsonl').open():
 x=json.loads(line)
 if x['kind']!='aggregate_execution_safety':continue
 if first is None and x['data'].get('reason')=='imu_tilt_at_least_0.30':first=x
 elif first is not None and following is None and x['data'].get('state')=='ready':following=x
if first is None:raise RuntimeError('no first actual hold despite original hold evidence')
ns=first['received_stamp_ns'];wall=first['wall']
prior=next((s for s in reversed(r['statuses']) if s['received_stamp_ns']<=ns),None)
window=[ns-1_000_000_000,ns+2_000_000_000]
raw_near=[x for x in raw if window[0]<=x['stamp_ns']<=window[1]]
gt_near=[x for x in r['truth'] if window[0]<=x['stamp_ns']<=window[1]]
slam_near=[x for x in r['poses'] if window[0]<=x['stamp_ns']<=window[1]]
cmds=[x for x in r['command_evidence'] if window[0]<=x['received_stamp_ns']<=window[1]]
cmd_transition={}
for source in ['requested','safe','actuator']:
 rows=[x for x in r['command_evidence'] if x['source']==source]
 before=[x for x in rows if x['wall']<wall]
 after=[x for x in rows if x['wall']>=wall]
 zero=next((x for x in after if all(v==0 for v in x['value'])),None)
 cmd_transition[source]={'last_before_hold_callback':before[-1] if before else None,'first_after_hold_callback':after[0] if after else None,'first_exact_zero_after_hold_callback':zero,'zero_after_callback_wall_delay_ms':None if zero is None else (zero['wall']-wall)*1000}
# Native header-based selected critical truth snapshots under the one fixed transform.
critical=[]
for target in [ns-1_000_000_000,ns-300_000_000,ns,ns+300_000_000,ns+1_000_000_000]:
 gt=C.bounded_pair(target,r['truth'])
 closest=min(raw_near,key=lambda x:abs(x['stamp_ns']-target))
 critical.append({'target_stamp_ns':target,'bounded_truth_available':gt is not None,'truth_world_position':None if gt is None else gt[0].tolist(),'truth_world_roll_pitch':None if gt is None else H.metric(Rotation.from_matrix(gt[1]).as_quat())[1:],'raw_IMU_stamp_ns':closest['stamp_ns'],'raw_roll_pitch':H.metric(closest['quaternion'])[1:]})
footer=H.load(run/'actuator/observer_result.json');ranges=footer['retained_topic_ranges']
missing_first_hold={k:{'retained_first_header_ns':ranges[k]['first_header_ns'],'retained_last_header_ns':ranges[k]['last_header_ns'],'first_hold_complete_original_CDR_retained':bool(ranges[k]['first_header_ns'] is not None and ranges[k]['first_header_ns']<=ns<=ranges[k]['last_header_ns'])} for k in ['imu','measured_joint','legacy_joint','jtc','foot_lf','foot_rf','foot_lh','foot_rh']}
print(json.dumps({'first_hold_ns':ns,'first_hold_waypoint_index':prior['status']['waypoint_index'],'CDR_first_hold_retained':missing_first_hold}),flush=True)
cdr=H.audit_cdr(run,r['eighth_segment_previous_receipt_stamp_ns'],r['terminal_stamp_ns'])
native=H.audit_native(run,r)
active_slice_bad=[x for x in native['generic_slice_bad'] if r['origin_stamp_ns']<=round(float(x['camera'])*1e9)<=r['terminal_stamp_ns']]
report={'verdict':'Original strict component FAIL retained: eight ordered original region windows pass, but active raw tilt >=.30 produced one safety hold. No full demo PASS or new physics run.',
 'original_result_passed':r['passed'],'original_missing_acceptance':r['missing_acceptance'],'independent_eight_regions':e,
 'active_raw_IMU':{'count':len(raw),'span_ns':[raw_stamps[0],raw_stamps[-1]],'max_original_Euler_tilt_rad':max(H.metric(x['quaternion'])[0] for x in raw),'unexpected_1ms_header_gaps':sorted(set(v for v in raw_gaps if v!=1_000_000)),'first_crossings':crossings},
 'aligned_attitude':{'rmse_rad':float(np.sqrt(np.mean([x['error_rad']**2 for x in attitude]))),'max_rad':max(x['error_rad'] for x in attitude),'near_first_hold':[x for x in attitude if window[0]<=x['stamp_ns']<=window[1]]},
 'first_actual_hold':{'actual_bridge_callback':first,'next_actual_ready_callback':following,'prior_actual_NAV':prior,'critical_original_inputs':critical,'raw_IMU_nearby_rows':len(raw_near),'GT_nearby_rows':len(gt_near),'SLAM_nearby_rows':len(slam_near),'actual_commands_transitions':cmd_transition,'raw_IMU_nearby_max_tilt_rad':max(H.metric(x['quaternion'])[0] for x in raw_near),'native_sensor_CDR_availability':missing_first_hold,'limits':['Actual Bridge callback received_stamp_ns is probe latest-clock association; actual imu_stamp identifies the exact safety input.','Requested/safe/actual Twist are headerless; callback wall delay and recorded latest-clock are observed transport boundaries, not exact applied velocity.','193sim JTC/measured-q/feet full payloads were overwritten by the retained suffix starting about270sim. No effort/velocity/reference causality can be inferred at the first hold.']},
 'eighth_actual_CDR':cdr,'native_FAST_IMU':native,'active_generic_slice_bad':active_slice_bad,'cleanup':H.load(run/'process_cleanup.json'),'manifest':H.load(run/'extension_manifest.json'),
 'input_SHA256':{str(s):H.sha(run/s) for s in ['first_eight_result.json','first_eight_events.jsonl','actuator/observer_result.json','actuator/actuator_suffix.cdrlog','fastlivo_debug/imu.txt','stack.log','process_cleanup.json','extension_manifest.json']},
 'limits':['No state correction, GT feedback, threshold change, later replacement receipt window, synthetic contact or overwritten raw payload reconstruction.','Sparse actual contact headers are not an invented continuous full-contact timeline.','Output.effort is controller command, not applied measured torque.','DDS lost callback footer is global; retained sequence integrity and interval callbacks are checked separately.']}
dest=P/'oct2_NAV_drift_first8_actual_independent.json';dest.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'report':str(dest),'regions':e['passed'],'cdr_fixed_complete':cdr['all_fixed_period_sensor_JTC_headers_complete'],'cdr_payload_errors':len(cdr['full_original_payload_errors']),'native_missing':native['missing_native_1ms'],'native_raw_unconsumed':native['raw_observed_but_native_unconsumed_in_native_span'],'active_slice_bad':active_slice_bad}),flush=True)
