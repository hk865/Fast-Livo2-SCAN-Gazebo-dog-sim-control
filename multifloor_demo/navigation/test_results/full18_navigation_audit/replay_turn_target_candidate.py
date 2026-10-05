"""Open-loop decision comparison on original recorded steering inputs only.

The recorded body is NOT resimulated after a hypothetical earlier stop. This
cannot prove a change would avoid falling. Native raw SLAM stamps provide the
only timing progress; reconstructed control yaw is checked against raw q.
"""
import bisect
import hashlib
import json
import math
import pathlib
from turn_target_candidate import BoundedTurnReference,angle

HERE=pathlib.Path(__file__).resolve().parent
RUN=HERE.parents[2]/'runs/20261002_001559_882ddb'
poses=[json.loads(x) for x in (RUN/'pose_audit.jsonl').read_text().splitlines()]
poses=sorted([x for x in poses if x['source']=='slam'],key=lambda x:x['stamp_ns'])
times=[x['stamp_ns'] for x in poses]
feedback=[json.loads(x) for x in (RUN/'feedback_navigation.jsonl').read_text().splitlines()]
inputs=[];seen=set();mismatches=[]
for x in feedback:
    n=x.get('navigation',{});s=n.get('steering')
    if (n.get('waypoint_index')!=3 or n.get('alignment_phase')!='align'
            or n.get('tilt_hold') or not s or s.get('trajectory_id')!=11):continue
    expected=round(s['odom_stamp']*1e9)
    index=bisect.bisect_left(times,expected)
    options=poses[max(0,index-1):index+2]
    p=min(options,key=lambda a:abs(a['stamp_ns']-expected))
    if abs(p['stamp_ns']-expected)>2:
        mismatches.append({'stamp':s['odom_stamp'],'reason':'no native audit stamp within 2ns'});continue
    if p['stamp_ns'] in seen:continue
    seen.add(p['stamp_ns'])
    qx,qy,qz,qw=p['q'];qnorm=sum(v*v for v in p['q'])
    raw_yaw=math.atan2(2*(qw*qz+qx*qy),qnorm-2*(qy*qy+qz*qz))
    used_yaw=angle(s['heading']-s['error'])
    if abs(angle(raw_yaw-used_yaw))>1e-9:
        mismatches.append({'stamp':p['stamp_ns'],'reason':'raw yaw differs from live steering input','error':angle(raw_yaw-used_yaw)})
    inputs.append({'stamp_ns':p['stamp_ns'],'control_stamp':s['stamp'],
        'desired_heading':s['heading'],'yaw':used_yaw,'locked_heading':n['locked_heading'],
        'raw_tilt_status':n['raw_imu_tilt_rad'],
        'reference_key':(n['request_id'],n['current_goal']['goal_id'],11,tuple(n['trajectory_reference_stamp']))})
results=[]
for rate in [.02,.04,.06]:
    first=inputs[0]
    candidate=BoundedTurnReference(first['reference_key'],first['locked_heading'],
                                    first['stamp_ns']-100_000_000,rate_rad_s=rate)
    traces=[];first_settle=None
    for x in inputs:
        r=candidate.update(x['stamp_ns'],x['desired_heading'],x['yaw'],x['reference_key'])
        traces.append(dict(input=x,result=r))
        if first_settle is None and r['can_start_original_settle']:
            first_settle={'native_odom_stamp_ns':x['stamp_ns'],'sampled_control_stamp':x['control_stamp'],
                'reference_heading':r['reference_heading'],'reference_error':r['reference_error'],
                'current_path_error':r['current_path_error'],'raw_tilt_status':x['raw_tilt_status'],
                'time_before_actual_raw_hold_using_control_stamp_s':119.823-x['control_stamp']}
    results.append({'rate_rad_s':rate,'max_offset_rad':.30,'coherence_rad':.05,
        'persistence_ns':350_000_000,'first_hypothetical_settle_decision':first_settle,'trace':traces})
result={'scope':'excluded geometry-only open-loop decision replay',
    'input_samples':len(inputs),'native_stamp_or_yaw_mismatches':mismatches,
    'current_source_thresholds':{'turn_settle_error_rad':.10,'drive_error_rad':.20,
         'persistent_drive_error_duration_s':.5,'severe_error_rad':.55,'pure_turn_yaw_cap_rad_s':.12},
    'actual_first_hold_sim':119.823,'actual_first_fail_sim':120.104,'results':results,
    'limitations':['Recorded pose/heading trajectory remains the original commanded one',
      'Feedback status snapshots omit some live control callbacks; first decision is a sampled upper timing bound',
      'No synthetic body response or GT inputs','No proof earlier settle prevents fall',
      'Original pre-turn/one-second settle and all actual motion/tilt/collision/arrival contracts are untouched'],
    'sha256':{name:hashlib.sha256((RUN/name).read_bytes()).hexdigest()
          for name in ['feedback_navigation.jsonl','pose_audit.jsonl']}}
(HERE/'bounded_reference_sparse_status_replay.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'samples':len(inputs),'mismatches':len(mismatches),
    'rates':[{'rate':x['rate_rad_s'],'first':x['first_hypothetical_settle_decision']} for x in results]}))
