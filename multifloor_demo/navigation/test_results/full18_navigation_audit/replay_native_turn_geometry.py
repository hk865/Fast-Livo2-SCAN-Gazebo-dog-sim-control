"""Reconstruct same frozen SCAN path + production EMA on native raw SLAM.

No GT input; verify reconstructed geometry against actual embedded controller
diagnostics before drawing a bounded-reference decision timing comparison.
"""
import hashlib
import json
import math
import pathlib
import sys
import numpy as np
from turn_target_candidate import BoundedTurnReference,angle

HERE=pathlib.Path(__file__).resolve().parent;DEMO=HERE.parents[2]
RUN=DEMO/'runs/20261002_001559_882ddb'
sys.path.insert(0,str(DEMO/'navigation'))
from control_core import TrackingPositionFilter,follow_trajectory,rotation_xyzw
poses=sorted([json.loads(s) for s in (RUN/'pose_audit.jsonl').read_text().splitlines()
             if json.loads(s)['source']=='slam'],key=lambda x:x['stamp_ns'])
feedback=[json.loads(s) for s in (RUN/'feedback_navigation.jsonl').read_text().splitlines()]
trajectories=[json.loads(s) for s in (RUN/'feedback_navigation_trajectories.jsonl').read_text().splitlines()]
metadata=next(x['metadata'] for x in trajectories if x['kind']=='scan_metadata' and x['metadata']['trajectory']['traj_id']==11)
paths=[x for x in trajectories if x['kind']=='accepted_path' and x['receipt_sim_time']>=metadata['trajectory']['start_time'][0]+metadata['trajectory']['start_time'][1]/1e9]
path=paths[0];samples=np.asarray(path['points']);goal=np.asarray(metadata['body_goal'])
tracking=TrackingPositionFilter();native={};timestamps=[]
for x in poses:
    tracked=tracking.update(np.asarray(x['p']),x['stamp_ns']/1e9)
    rotation=rotation_xyzw(x['q'])
    _,_,_,steering=follow_trajectory(np.asarray(x['p']),rotation,samples,goal,
                tracking_pose=tracked,gate_translation=False,return_steering=True)
    native[x['stamp_ns']]={'pose':x['p'],'tracking_pose':tracked.tolist(),
        'stamp_ns':x['stamp_ns'],'yaw':math.atan2(rotation[1,0],rotation[0,0]),'steering':steering}
    timestamps.append(x['stamp_ns'])

diagnostic_checks=[]
for x in feedback:
    n=x.get('navigation',{});s=n.get('steering')
    if not s or s.get('trajectory_id')!=11 or n.get('tilt_hold'):continue
    expected=round(s['odom_stamp']*1e9)
    key=min(timestamps,key=lambda t:abs(t-expected))
    if abs(key-expected)>2:continue
    calc=native[key]['steering']
    diagnostic_checks.append({'native_stamp_ns':key,'heading_error':angle(calc['heading']-s['heading']),
        'direction_maxabs':max(abs(a-b) for a,b in zip(calc['direction'],s['direction'])),
        'control_yaw_error':angle(native[key]['yaw']-(s['heading']-s['error']))})
inputs=[native[x['stamp_ns']] for x in poses if 112.8<=x['stamp_ns']/1e9<=119.8]
locked=2.939656740266147
key=('20261002_001559_882ddb:exploration:1','exploration:3',11,tuple(metadata['reference_stamp']))
cases=[]
for rate in [.02,.04,.06]:
    c=BoundedTurnReference(key,locked,inputs[0]['stamp_ns']-100_000_000,rate_rad_s=rate)
    trace=[];first=None
    for x in inputs:
        r=c.update(x['stamp_ns'],x['steering']['heading'],x['yaw'],key)
        trace.append({'stamp_ns':x['stamp_ns'],'input_heading':x['steering']['heading'],'yaw':x['yaw'],'candidate':r})
        if first is None and r['can_start_original_settle']:
            first=trace[-1]
    cases.append({'rate_rad_s':rate,'first_hypothetical_settle_native_sample':first,'trace':trace})
result={'scope':'excluded same-curve/native-odom/production-EMA open-loop replay; no body resimulation',
    'native_samples':len(inputs),'path_receipt':{k:path[k] for k in ['receipt_sim_time','header_stamp']},
    'traj_id':11,'reference_stamp':metadata['reference_stamp'],'body_goal':metadata['body_goal'],
    'diagnostic_checks':diagnostic_checks,
    'max_heading_difference_to_actual_diagnostic':max(abs(x['heading_error']) for x in diagnostic_checks),
    'max_direction_difference_to_actual_diagnostic':max(x['direction_maxabs'] for x in diagnostic_checks),
    'max_yaw_difference_to_actual_diagnostic':max(abs(x['control_yaw_error']) for x in diagnostic_checks),
    'cases':cases,
    'limits':{'total_reference_offset_rad':.30,'consistency_deadband_rad':.05,'persistence_ns':350000000,
              'native_gap_ns':200000000,'original_settle_error_rad':.10,'original_drive_gate_rad':.20},
    'limitations':['Sparse status-only replay separately has observation gaps and cannot establish readiness',
      'Full raw SLAM native pose and exact accepted path reconstruct the missing geometry samples',
      'An earlier hypothetical zero would alter future body state; replay is not a physical outcome or PASS',
      'No changed timeout, region, collision, tilt, rate cap, gait or state-machine duration'],
    'sha256':{name:hashlib.sha256((RUN/name).read_bytes()).hexdigest() for name in ['feedback_navigation.jsonl','pose_audit.jsonl','feedback_navigation_trajectories.jsonl']}}
(HERE/'bounded_reference_native_geometry_replay.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'samples':len(inputs),'diagnostics':len(diagnostic_checks),
    'max_heading_diff':result['max_heading_difference_to_actual_diagnostic'],
    'max_direction_diff':result['max_direction_difference_to_actual_diagnostic'],
    'cases':[{'rate':x['rate_rad_s'],'first':x['first_hypothetical_settle_native_sample']} for x in cases]}))
