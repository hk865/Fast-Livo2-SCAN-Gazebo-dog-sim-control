"""Original A=false NAV deadline/region context; no new physics, GT feedback or CDR inference."""
from pathlib import Path
import ast,hashlib,json,sys
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal,contains_control,contains
RUN=ROOT/'simulation/test_results/20261002_jtc_desired_first8_a_false'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=json.loads((RUN/'first_eight_result.json').read_text())
guard=json.loads((RUN/'nav_drift_guard.json').read_text())
goal=parse_goal(r['goals_definitions'][7]);start=r['eighth_segment_previous_receipt_stamp_ns'];end=r['terminal_stamp_ns']
poses=[p for p in r['poses'] if start<=p['stamp_ns']<=end]
inside=[p for p in poses if contains_control(goal,p['p'])]
stops=[]
for i,x in enumerate(guard['events']):
    if x['event']!='exact_zero_published' or x['waypoint_index']!=7:continue
    j=next((j for j in range(i+1,len(guard['events'])) if guard['events'][j]['event']=='exact_zero_published'),len(guard['events']))
    part=guard['events'][i+1:j]
    ref=next((x for x in part if x['event']=='fresh_reference_requested'),None)
    accept=next((x for x in part if x['event']=='fresh_checked_path_accepted'),None)
    stops.append(dict(raw_trigger_stamp_ns=x['trigger']['stamp_ns'],callback_stop_stamp_ns=x['callback_clock_ns'],
        planar_drift_m=x['trigger']['raw_planar_drift_m'],reference=ref,accepted=accept))
last=poses[-1];axis_error=np.asarray(goal.axes)@(np.asarray(last['p'])-goal.center)
failure=next((x for x in r['statuses'] if x['status'].get('state')=='failed' and x['status'].get('request_id')==r['request_id']),None)
post=[p for p in r['poses'] if p['stamp_ns']>end and contains_control(goal,p['p'])]
written=[x.attr for x in ast.walk(ast.parse((RUN/'staging/nav_drift_controller.py').read_text())) if isinstance(x,ast.Attribute) and isinstance(x.ctx,ast.Store)]
checks=dict(original_failed_result_kept=r['passed'] is False,
    original_first7_receipt_windows=all(x['passed'] for x in r['region_evaluation']['regions'][:7]),
    original_eighth_missing_receipt=r['region_evaluation']['regions'][7]['receipt'] is None,
    no_raw_inner_before_actual_terminal=not inside,
    last_along_error_over_point25=abs(axis_error[0])>.25 and abs(axis_error[1])<=.20 and abs(axis_error[2])<=.07,
    original_no_active_imu_hold=r['active_bridge_holds']==0 and r['active_max_imu_tilt']<.30,
    original_timeout_reported=bool(failure) and '仿真时间限制' in failure['status']['message'],
    unchanged_wrapper_deadline=not any(x in written for x in ['segment_start','segment_started_ros','goals','waypoint_index']),
    five_fresh_checked_current_goal_handoffs=len(stops)==5 and all(x['reference'] and x['accepted']
        and x['reference']['waypoint_index']==x['accepted']['waypoint_index']==7
        and x['reference']['request_id']==x['accepted']['request_id']==r['request_id']
        and x['accepted']['callback_clock_ns']>=x['reference']['callback_clock_ns'] for x in stops))
checks={k:bool(v) for k,v in checks.items()}
result=dict(passed=all(checks.values()),checks=checks,scope=__doc__,original_component_passed=False,
    original_failure=r['failure'],original_missing_acceptance=r['missing_acceptance'],
    origin_stamp_ns=r['origin_stamp_ns'],terminal_stamp_ns=end,previous_original_receipt_stamp_ns=start,
    first_observed_goal8_status_stamp_ns=r['eighth_segment_entry_stamp_ns'],
    actual_failed_status=failure,declared_goal8=goal.definition(),original_active_inner_samples=len(inside),
    original_last_active_pose=last,original_last_active_axis_error_m=axis_error.tolist(),
    last_outer_inside=contains(goal,last['p']),last_inner_inside=contains_control(goal,last['p']),
    post_terminal_inner_samples_excluded=len(post),first_post_terminal_inner_stamp_ns=post[0]['stamp_ns'] if post else None,
    original_max_raw_tilt=r['active_max_imu_tilt'],original_tilt_holds=r['active_bridge_holds'],
    goal8_drift_handoffs=stops,whole_route_drift_count=guard['interruptions'],
    input_sha256={n:sha(RUN/n) for n in ['first_eight_result.json','nav_drift_guard.json','jtc_parameter_readback.json','process_cleanup.json','staging/nav_drift_controller.py','staging/turn_drift.py']},
    audit_sha256=sha(Path(__file__)),
    limitations=['Previous receipt/end and first 4Hz status do not constitute an exact internal segment-start timer readback.',
        'No physical output, torque, foot or timing root cause is inferred here. Full native JTC/CDR audit is independently owned.',
        'Raw post-terminal samples cannot repair an original missing receipt or timeout.',
        'A lower tilt peak than another fresh trial is not a parameter effect attribution; A uses the original false flag.'])
(HERE/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'],checks=checks,last_axis_error=axis_error.tolist(),post_terminal_excluded=len(post),result_sha256=sha(HERE/'result.json')),indent=2))
