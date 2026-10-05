#!/usr/bin/env python3
"""Offline actual true candidate audit. Original FAIL stays immutable; no ROS node."""
from pathlib import Path
import json,importlib.util,hashlib,collections
import numpy as np
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('actual_helpers',P/'analyze_feedback_first4_ab_independent.py');H=importlib.util.module_from_spec(spec);spec.loader.exec_module(H)
run=H.ROOT/'simulation/test_results/20261002_jtc_desired_first8_b_true';r=H.load(run/'first_eight_result.json')
spec=importlib.util.spec_from_file_location('actual_eight',run/'staging/eight_region_contract.py');C=importlib.util.module_from_spec(spec);spec.loader.exec_module(C)
goals=tuple(H.parse_goal(g) for g in r['goals_definitions']);evaluation=C.evaluate_regions(goals,r['request_id'],r['statuses'],r['poses'],r['truth'],r['origin_stamp_ns'],r['terminal_stamp_ns'])
R=np.asarray(evaluation['initial_fixed_SE3_evaluation_only']['rotation_world_from_slam']);T=np.asarray(evaluation['initial_fixed_SE3_evaluation_only']['translation'])
att=[]
for x in r['poses']:
 if not r['origin_stamp_ns']<=x['stamp_ns']<=r['terminal_stamp_ns']:continue
 gt=C.bounded_pair(x['stamp_ns'],r['truth'])
 if gt is None:continue
 actual=Rotation.from_matrix(gt[1]);est=Rotation.from_matrix(R)*Rotation.from_quat(x['q']);att.append({'stamp_ns':x['stamp_ns'],'error_rad':float((actual.inv()*est).magnitude())})
raw=[x for x in r['raw_imu'] if r['origin_stamp_ns']<=x['stamp_ns']<=r['terminal_stamp_ns']];gaps=[b['stamp_ns']-a['stamp_ns'] for a,b in zip(raw,raw[1:])]
first_hold=next((x for x in raw if H.metric(x['quaternion'])[0]>=.3),None)
terminal=[x for x in r['statuses'] if x['status'].get('request_id')==r['request_id'] and x['status'].get('state')=='failed']
last=[x for x in r['poses'] if x['stamp_ns']<=r['terminal_stamp_ns']][-1];gt=C.bounded_pair(last['stamp_ns'],r['truth']);gt_slam=None if gt is None else R.T@(gt[0]-T)
print(json.dumps({'original_passed':r['passed'],'regions':[x['passed'] for x in evaluation['regions']],'precision':evaluation['precision'],'coverage':evaluation['coverage'],'first_rawhold':None if first_hold is None else first_hold['stamp_ns']}),flush=True)
cdr=H.audit_cdr(run,r['origin_stamp_ns'],r['terminal_stamp_ns']);native=H.audit_native(run,r);start,end=r['origin_stamp_ns'],r['terminal_stamp_ns'];active_generic_bad=[x for x in native['generic_slice_bad'] if start<=round(float(x['camera'])*1e9)<=end]
readback=H.load(run/'jtc_parameter_readback.json');manifest=H.load(run/'jtc_manifest.json');cleanup=H.load(run/'process_cleanup.json');maps=H.load(run/'actual_gz_process.json');selected=[v for x in maps for v in x['libraries'] if Path(v['path']).name in ('libjoint_trajectory_controller.so','libcontroller_manager.so','libcontrol_toolbox.so','libcontroller_interface.so')]
near=[x for x in r['poses'] if r['terminal_stamp_ns']-1_000_000_000<=x['stamp_ns']<=r['terminal_stamp_ns']]
from navigation.goal_regions import contains,contains_control
end_samples=[]
for x in near:
 paired=C.bounded_pair(x['stamp_ns'],r['truth']);pos=None if paired is None else R.T@(paired[0]-T)
 end_samples.append({'stamp_ns':x['stamp_ns'],'raw_position':x['p'],'raw_inner':contains_control(goals[0],x['p']),'GT_outer':False if pos is None else contains(goals[0],pos),'raw_center_error_m':float(np.linalg.norm(np.array(x['p'])-goals[0].center))})
report={'verdict':'Original B true first8 FAIL retained: zero original receipt windows passed, first original90sim deadline expired. No tilt hold or body contact, no rescue by later position/window.',
 'original_passed':r['passed'],'original_failure':r['failure'],'original_missing_acceptance':r['missing_acceptance'],'independent_original_regions':evaluation,'original_final_failed_NAV':terminal[0] if terminal else None,
 'active_raw_IMU':{'rows':len(raw),'span_ns':[raw[0]['stamp_ns'],raw[-1]['stamp_ns']],'unexpected_1ms_header_gaps':sorted(set(v for v in gaps if v!=1_000_000)),'max_original_Euler_rad':max(H.metric(x['quaternion'])[0] for x in raw),'first_at_least030':first_hold,'no_firsthold_payload_needed':first_hold is None},
 'aligned_attitude':{'rmse_rad':float(np.sqrt(np.mean([x['error_rad']**2 for x in att]))),'max_rad':max(x['error_rad'] for x in att)},
 'first_goal_terminal_one_second_positions_diagnostic_only':end_samples,'first_attempted_segment_actual_CDR':cdr,'native_FAST_IMU':native,'active_generic_bad':active_generic_bad,'actual_BOOL_startup_readback':readback,'actual_loaded_control_libraries':selected,'cleanup':cleanup,'manifest':manifest,
 'original_data_immutable_SHA256':{n:H.sha(run/n) for n in ['first_eight_result.json','jtc_parameter_readback.json','process_cleanup.json','jtc_manifest.json','actuator/observer_result.json','actuator/actuator_suffix.cdrlog','fastlivo_debug/imu.txt','stack.log']},
 'limits':['B is the fresh explicittrue candidate with common readback/recording changes. Old900k run had different diagnostic timing and natural startup state; not an onlyflag comparison.','Same initial SE3 used once for all region/precision checks, GT never control feedback.','No first>=.30 event exists in B, so no firsthold mechanical claim.','Header continuity/actual CDR state is not an independent hardware applied-force or native ControllerInterface period hook. JTC outputeffort is commanded.','Any internal IMU missing timestamps or global DDS loss remain explicit, never filled or claimed as unique timeout cause.']}
out=P/'oct2_JTC_desired_first8_B_true_actual_independent.json';out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'report':str(out),'coverage':evaluation['coverage'],'last_region_error':evaluation['regions'][-1].get('errors'),'cdr_fixed_complete':cdr['all_fixed_period_sensor_JTC_headers_complete'],'cdr_payload_errors':len(cdr['full_original_payload_errors']),'native_missing':native['missing_native_1ms'],'native_raw_unconsumed':native['raw_observed_but_native_unconsumed_in_native_span'],'native_active_sync_bad':native['active_sync_bad'],'active_generic_bad':active_generic_bad}),flush=True)
