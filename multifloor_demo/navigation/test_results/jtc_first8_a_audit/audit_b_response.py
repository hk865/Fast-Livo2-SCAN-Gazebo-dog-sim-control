"""Original B=true command/SLAM response. No controller, node or CDR replay.

Command times are native Adapter cached float clock values, not message headers.
Truth only evaluates observed motion using the original single initial SE3.
"""
from pathlib import Path
import ast,hashlib,json,sys
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal,contains_control
RUN=ROOT/'simulation/test_results/20261002_jtc_desired_first8_b_true'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=json.loads((RUN/'first_eight_result.json').read_text())
g=json.loads((RUN/'nav_drift_guard.json').read_text())
origin,end=r['origin_stamp_ns']/1e9,r['terminal_stamp_ns']/1e9
cmd=[]
for line in (RUN/'joint_stop_adapter.jsonl').open():
    v=json.loads(line)
    if v['kind']=='actual_champ_command':cmd.append(v)
assert all(a['sim']<=b['sim'] for a,b in zip(cmd,cmd[1:]))
intervals=[]
for a,b in zip(cmd,cmd[1:]):
    l,h=max(origin,a['sim']),min(end,b['sim'])
    if h<=l:continue
    c=a['value'];mode='walk' if c[0]!=0 or c[1]!=0 else 'turn' if c[5]!=0 else 'zero'
    intervals.append((l,h,mode,c))
assert intervals

def sampled(rows,times,Rmap=None,tmap=None,maxgap=.200000001):
    unique={x['stamp_ns']:x for x in rows}; rows=[unique[k] for k in sorted(unique)]
    t=np.asarray([x['stamp_ns']/1e9 for x in rows]);p=np.asarray([x['p'] for x in rows])
    rot=Rotation.from_quat([x['q'] for x in rows])
    assert times.min()>=t[0] and times.max()<=t[-1]
    i=np.searchsorted(t,times,side='right');i=np.clip(i,1,len(t)-1)
    assert np.max(t[i]-t[i-1])<=maxgap
    p=np.stack([np.interp(times,t,p[:,j]) for j in range(3)],axis=1)
    mats=Slerp(t,rot)(times).as_matrix()
    if Rmap is not None:
        p=(p-tmap)@Rmap;mats=np.einsum('ij,njk->nik',Rmap.T,mats)
    return p,mats

sampletimes=np.asarray([q for l,h,m,c in intervals for q in (l,(l+h)/2,h)])
frame=r['region_evaluation']['initial_fixed_SE3_evaluation_only']
rr=np.asarray(frame['rotation_world_from_slam']);tt=np.asarray(frame['translation'])
sources={}
for name,rows,rm,tm in [('slam',r['poses'],None,None),('truth_evaluation_only',r['truth'],rr,tt)]:
    p,mats=sampled(rows,sampletimes,rm,tm,maxgap=.150000001 if name=='truth_evaluation_only' else .200000001)
    p=p.reshape(-1,3,3);mats=mats.reshape(-1,3,3,3)
    dp=p[:,2]-p[:,0];body=np.einsum('nij,nj->ni',np.transpose(mats[:,1],(0,2,1)),dp)
    yaw=np.arctan2(mats[:,:,1,0],mats[:,:,0,0]);dyaw=np.arctan2(np.sin(yaw[:,2]-yaw[:,0]),np.cos(yaw[:,2]-yaw[:,0]))
    sources[name]=(dp,body,dyaw)

def total(indices):
    out={'duration_sim_s':sum(intervals[j][1]-intervals[j][0] for j in indices),
         'emitted_command_integral_vx_m':sum(intervals[j][3][0]*(intervals[j][1]-intervals[j][0]) for j in indices),
         'emitted_command_integral_wz_rad':sum(intervals[j][3][5]*(intervals[j][1]-intervals[j][0]) for j in indices)}
    for name,(dp,body,dyaw) in sources.items():
        out[name]={'delta_camera_init_m':np.sum(dp[indices],axis=0).tolist(),
                   'local_body_forward_m':float(np.sum(body[indices,0])),
                   'net_yaw_rad':float(np.sum(dyaw[indices]))}
    return out
modes={m:total([j for j,x in enumerate(intervals) if x[2]==m]) for m in ('walk','turn','zero')}
segments=[];j=0
while j<len(intervals):
    k=j+1
    while k<len(intervals) and intervals[k][2]==intervals[j][2]:k+=1
    segments.append(dict(mode=intervals[j][2],start=intervals[j][0],end=intervals[k-1][1],**total(list(range(j,k)))))
    j=k
goal=parse_goal(r['goals_definitions'][0]);activeposes=[p for p in r['poses'] if r['origin_stamp_ns']<=p['stamp_ns']<=r['terminal_stamp_ns']]
inside=[p for p in activeposes if contains_control(goal,p['p'])]
status=[x for x in r['statuses'] if x['status'].get('request_id')==r['request_id'] and x['status']['state']=='running']
trajectory=[]
for line in (RUN/'feedback_navigation_trajectories.jsonl').open():
    x=json.loads(line)
    if x['kind']=='scan_metadata':
        m=x['metadata'];pts=np.asarray(m['trajectory']['pos_pts'])
        trajectory.append(dict(traj_id=m['trajectory']['traj_id'],reference_stamp=m['reference_stamp'],
              body_goal=m['body_goal'],same_original_goal=bool(np.max(np.abs(np.asarray(m['body_goal'])-goal.center))<=1e-8),
              control_point_bounds=[pts.min(axis=0).tolist(),pts.max(axis=0).tolist()]))
stops=[]
events=g['events']
for i,x in enumerate(events):
    if x['event']!='exact_zero_published':continue
    nxt=next((j for j in range(i+1,len(events)) if events[j]['event']=='exact_zero_published'),len(events))
    part=events[i+1:nxt];ref=next((v for v in part if v['event']=='fresh_reference_requested'),None)
    accept=next((v for v in part if v['event']=='fresh_checked_path_accepted'),None)
    stops.append(dict(trigger_stamp_ns=x['trigger']['stamp_ns'],raw_anchor=x['trigger']['raw_anchor'],
       raw_position=x['trigger']['raw_position'],raw_drift_m=x['trigger']['raw_planar_drift_m'],
       stop_callback_ns=x['callback_clock_ns'],reference=ref,checked_accept=accept))
failed=next((x for x in r['statuses'] if x['status']['state']=='failed' and x['status'].get('request_id')==r['request_id']),None)
writes=[x.attr for x in ast.walk(ast.parse((RUN/'staging/nav_drift_controller.py').read_text())) if isinstance(x,ast.Attribute) and isinstance(x.ctx,ast.Store)]
checks=dict(original_component_FAIL_kept=r['passed'] is False,
    original_zero_receipts=all(x['receipt'] is None for x in r['region_evaluation']['regions']),
    original_raw_inner_never_entered=not inside,
    timeout90_actual_report=bool(failed) and '仿真时间限制' in failed['status']['message'] and 90<end-origin<90.3,
    no_tilt_hold_or_failure=r['active_bridge_holds']==0 and r['active_max_imu_tilt']<.30,
    all16_actual_metadata_same_original_goal=len(trajectory)==16 and all(x['same_original_goal'] for x in trajectory),
    no_navigation_identity_or_degenerate_reject=max(x['status']['trajectory_association_rejected'] for x in status)==0 and max(x['status']['degenerate_splines'] for x in status)==0,
    no_deadline_or_goal_write_in_guard=not any(x in writes for x in ('segment_started_ros','waypoint_index','goals','goals_definition_sha256')),
    all6_fresh_checked_same_goal_handoffs=len(stops)==6 and all(x['reference'] and x['checked_accept'] and x['reference']['waypoint_index']==x['checked_accept']['waypoint_index']==0 for x in stops))
checks={k:bool(v) for k,v in checks.items()}
result=dict(scope=__doc__,audit_checks_passed=all(checks.values()),checks=checks,original_component_passed=False,
    original_failure=r['failure'],original_result_sha256=sha(RUN/'first_eight_result.json'),
    origin_stamp_ns=r['origin_stamp_ns'],terminal_stamp_ns=r['terminal_stamp_ns'],
    single_original_fixed_SE3_evaluation_only=frame,mode_sums=modes,mode_segments=segments,
    original_goal=goal.definition(),active_inside_control_samples=len(inside),last_active_raw_pose=activeposes[-1],
    closest_active_raw_center_error_m=min(float(np.linalg.norm(np.asarray(p['p'])-goal.center)) for p in activeposes),
    active_raw_max_tilt=r['active_max_imu_tilt'],drift_stops=stops,actual_metadata=trajectory,
    actual_failed_status={k:failed['status'].get(k) for k in ('state','message','waypoint_index','pose','steering','command','alignment_phase','accepted_trajectory_id','region_arrival_evidence')},
    input_sha256={n:sha(RUN/n) for n in ('first_eight_result.json','nav_drift_guard.json','joint_stop_adapter.jsonl','feedback_navigation_trajectories.jsonl','jtc_parameter_readback.json','process_cleanup.json')},
    limitations=['Adapter native emitted commands do not establish actual CHAMP receive or joint torque. Cached float sim time is explicitly not an original message Header.',
       'Position/quaternion boundaries are interpolated only within native brackets (SLAM max .20s/GT max .15s), using the unchanged original initial SE3. GT is evaluation-only.',
       'No per-actuator/JTC mechanism is inferred; independently owned JTC/native input analysis is necessary for parameter effects.',
       'No replan trace can repair a missing raw arrival receipt or original timeout. No source, parameter, deadline or protection was changed.'])
(HERE/'b_response_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(audit_checks_passed=result['audit_checks_passed'],mode_sums=modes,segments=len(segments),
    closest_center=result['closest_active_raw_center_error_m'],last_pose=activeposes[-1]['p'],
    result_sha256=sha(HERE/'b_response_result.json')),indent=2))
