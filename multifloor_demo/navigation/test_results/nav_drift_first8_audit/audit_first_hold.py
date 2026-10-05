"""Original native first-hold context. No control, GT correction, or missing JTC reconstruction."""
import json,hashlib,bisect
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
RUN=HERE.parents[2]/'simulation/test_results/20261002_nav_drift_first8_disabled'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=json.loads((RUN/'first_eight_result.json').read_text())
guard=json.loads((RUN/'nav_drift_guard.json').read_text())
poses=r['poses'];pt=[x['stamp_ns'] for x in poses]
frame=r['region_evaluation']['initial_fixed_SE3_evaluation_only']
R=np.array(frame['rotation_world_from_slam']);T=np.array(frame['translation'])
thresholds={}
for limit in [.20,.25,.30,.35]:
    match=next((x for x in r['raw_imu'] if x['stamp_ns']>=r['origin_stamp_ns'] and x['tilt']>=limit),None)
    thresholds[str(limit)]=match
first=thresholds['0.3'];onset=first['stamp_ns'];ns0,ns1=onset-2000000000,onset+2000000000
states=[x for x in r['statuses'] if ns0<=x['received_stamp_ns']<=ns1]
navrows=[]
for x in states:
    n=x['status'];navrows.append(dict(received_stamp_ns=x['received_stamp_ns'],state=n.get('state'),index=n.get('waypoint_index'),
        phase=n.get('alignment_phase'),command=n.get('command'),tilt=n.get('raw_imu_tilt'),tilt_stops=n.get('tilt_stops'),
        protected=n.get('protection'),obstacle_hold=n.get('obstacle_hold'),accepted_trajectory_id=n.get('accepted_trajectory_id'),
        reference=n.get('trajectory_reference_stamp'),steering=n.get('steering'),turn_drift_guard=n.get('turn_drift_guard')))
commands=[];edges=[]
for line in (RUN/'joint_stop_adapter.jsonl').open():
    x=json.loads(line)
    if ns0<=round(x['sim']*1e9)<=ns1:
        if x['kind']=='actual_champ_command':commands.append(x)
        elif x['kind'] in ('zero_edge','native_zero_ack','return_complete'):edges.append(x)
before=[x for x in commands if round(x['sim']*1e9)<=onset]
after=[x for x in commands if round(x['sim']*1e9)>onset]
firstzero=next((x for x in after if not any(x['value'])),None)
last_motion=next((x for x in reversed(before) if any(x['value'])),None)
local_thresholds={str(limit):next((x for x in r['raw_imu'] if ns0<=x['stamp_ns']<=onset+1000000000 and x['tilt']>=limit),None) for limit in [.20,.25,.30,.35]}
slampose=[x for x in poses if ns0<=x['stamp_ns']<=ns1]
# GT evaluation is kept separate; original single initial frame is unchanged.
def bounded(t):
    truth=r['truth'];times=[x['stamp_ns'] for x in truth];i=bisect.bisect_left(times,t)
    if i<len(times) and times[i]==t:return np.array(truth[i]['p'])
    if i==0 or i==len(times) or times[i]-times[i-1]>150000000:return None
    return np.array(truth[i-1]['p'])+(t-times[i-1])/(times[i]-times[i-1])*(np.array(truth[i]['p'])-truth[i-1]['p'])
pose_evidence=[]
for x in slampose:
    gt=bounded(x['stamp_ns']);pose_evidence.append(dict(stamp_ns=x['stamp_ns'],raw_slam=x['p'],raw_slam_rpy=Rotation.from_quat(x['q']).as_euler('xyz').tolist(),
        fixed_SE3_world=(R@x['p']+T).tolist(),independent_truth_world=None if gt is None else gt.tolist(),
        independent_error_m=None if gt is None else float(np.linalg.norm(R@x['p']+T-gt))))
stop_events=[x for x in guard['events'] if x['event']=='exact_zero_published']
last_stop=next((x for x in reversed(stop_events) if x['callback_clock_ns']<=onset),None)
next_stop=next((x for x in stop_events if x['callback_clock_ns']>onset),None)
regions=r['region_evaluation']['regions'];eighth=regions[7]
a=json.loads((HERE/'result.json').read_text())
result=dict(scope=__doc__,original_component_passed=r['passed'],original_missing_acceptance=r['missing_acceptance'],
    original_fail_preserved=r['passed'] is False,origin_stamp_ns=r['origin_stamp_ns'],terminal_stamp_ns=r['terminal_stamp_ns'],
    first_active_thresholds=thresholds,local_thresholds_within_two_seconds_before_hold=local_thresholds,first_tilt_hold_stamp_ns=onset,actual_preceding_adapter_emit=before[-1] if before else None,actual_last_nonzero_adapter_emit=last_motion,
    first_observed_actual_zero_after_raw_threshold=firstzero,adapter_stop_edges=edges,native_nav_status_context=navrows,
    original_raw_pose_and_independent_truth_evaluation=pose_evidence,initial_fixed_SE3_evaluation_only=frame,
    previous_drift_stop=last_stop,next_drift_stop=next_stop,
    drift_handoff_count=len(a['handoffs']),all_14_handoff_checks=all(x['passed'] for x in a['handoffs']),
    original_region_windows=[dict(goal_id=x['goal_id'],receipt=x['receipt'],passed=x['passed']) for x in regions],
    eighth_original_receipt=eighth['receipt'],eighth_elapsed_from_previous_receipt_s=(eighth['receipt']['stamp_ns']-regions[6]['receipt']['stamp_ns'])/1e9,
    eighth_deadline_bound_s=90,eighth_original_window_passed=eighth['passed'],
    eighth_interruptions=[x for x in a['handoffs'] if x['goal_index']==7],
    input_sha256={n:sha(RUN/n) for n in ['first_eight_result.json','nav_drift_guard.json','joint_stop_adapter.jsonl','feedback_navigation_trajectories.jsonl','sensor_audit.jsonl']},
    limitations=['Native Adapter emit is actual publisher evidence, not an actuator receive timestamp or physical no-slip assumption.',
        'Raw IMU thresholds use original sensor Header stamps. Command log sim is cached Adapter clock; no precise cross-topic callback delay is inferred.',
        'Actuator CDR suffix begins after the first hold (reported269.956sim); no JTC/joint/contact states are fabricated for193sim.',
        'Status/steering is lower rate cached evidence. Last observed drive phase is not an exact internal phase transition trace.',
        'Single initial SE3 GT here evaluates only actual motion/SLAM fidelity and never enters control.'])
(HERE/'first_hold_and_eighth.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(first_raw_tilt_hold_stamp_ns=onset,threshold_stamps={k:v['stamp_ns'] for k,v in thresholds.items() if v},
    preceding_actual_emit=result['actual_preceding_adapter_emit'],first_zero=firstzero,
    last_drift=last_stop['callback_clock_ns'],next_drift=next_stop['callback_clock_ns'],
    eight_elapsed=result['eighth_elapsed_from_previous_receipt_s'],all14=result['all_14_handoff_checks'],
    component_passed=r['passed'],file_sha256=sha(HERE/'first_hold_and_eighth.json')),indent=2))
