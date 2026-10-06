"""Original native SLAM/full-Bspline replay, not closed-loop resimulation.

Each original pure-turn segment is analyzed separately. After the first
hypothetical STOP the recorded future is counterfactual and never judged PASS.
"""
import bisect,hashlib,json,math,pathlib,sys
import numpy as np
from scipy.interpolate import BSpline
from turn_drift import TurnDriftSupervisor,angle
HERE=pathlib.Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'navigation'))
from control_core import TrackingPositionFilter,follow_trajectory,rotation_xyzw

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):return [json.loads(l) for l in p.open()]
def load(run,is_component):
    trace=rows(run/'feedback_navigation_trajectories.jsonl')
    if is_component:
        r=json.loads((run/'first_four_result.json').read_text());ps=r['poses'];statuses=r['statuses']
        status=[dict(n['status'],receipt_sim=n['sim']) for n in statuses]
    else:
        ps=[r for r in rows(run/'pose_audit.jsonl') if r['source']=='slam']
        status=[dict(r['status'],receipt_sim=r.get('sim_time')) for r in rows(run/'navigation_audit.jsonl')]
    metas={};curves={}
    for row in trace:
        if row['kind']!='scan_metadata':continue
        m=row['metadata'];v=m['trajectory'];k=(v['traj_id'],tuple(m['reference_stamp']));metas[k]=m
        knot=np.asarray(v['knots']);coeff=np.asarray(v['pos_pts']);begin,end=knot[v['order']],knot[-v['order']-1]
        curves[k]=BSpline(knot,coeff,v['order'])(np.linspace(begin,end,min(2000,max(2,int((end-begin)/.08)+1))))
    states=[]
    for n in status:
        s=n.get('steering')
        if not s or n.get('request_id') is None or n.get('trajectory_reference_stamp') is None:continue
        states.append((round(s['stamp']*1e9),n))
    states.sort(key=lambda x:x[0]);state_t=[t for t,n in states]
    cmd=[]
    for r in rows(run/'joint_stop_adapter.jsonl'):
        if r['kind']=='actual_champ_command':cmd.append((round(r['sim']*1e9),r))
    cmd=list(sorted({t:r for t,r in cmd}.items()));cmd_t=[t for t,r in cmd]
    turns=[];current=None
    for (t,r),(end,nextrow) in zip(cmd,cmd[1:]):
        c=r['value'];pure=abs(c[0])+abs(c[1])<1e-9 and abs(c[5])>1e-8
        if pure:
            if current is None:current=dict(start=t,end=end)
            else:current['end']=end
        elif current is not None:turns.append(current);current=None
    if current:turns.append(current)
    return ps,states,state_t,cmd,cmd_t,turns,metas,curves

def replay(name,run,is_component,require_path_offset=True):
    ps,states,state_t,cmd,cmd_t,turns,metas,curves=load(run,is_component)
    ps.sort(key=lambda p:p['stamp_ns']);ps_t=[p['stamp_ns'] for p in ps]
    filter=TrackingPositionFilter();tracked={}
    for p in ps:tracked[p['stamp_ns']]=filter.update(np.asarray(p['p']),p['stamp_ns']/1e9)
    checks=[];match={}
    for control_t,n in states:
        s=n['steering'];expected=round(s['odom_stamp']*1e9);i=bisect.bisect_left(ps_t,expected)
        candidates=ps_t[max(0,i-1):min(len(ps_t),i+2)]
        if not candidates:continue
        exact=min(candidates,key=lambda t:abs(t-expected))
        if abs(exact-expected)>2:continue
        p=ps[bisect.bisect_left(ps_t,exact)];k=(n['accepted_trajectory_id'],tuple(n['trajectory_reference_stamp']))
        if k not in curves:continue
        _,_,_,calc=follow_trajectory(np.asarray(p['p']),rotation_xyzw(p['q']),curves[k],np.asarray(metas[k]['body_goal']),tracking_pose=tracked[exact],gate_translation=False,return_steering=True)
        if n.get('tilt_hold'):continue
        checks.append(dict(pose_stamp_ns=exact,control_stamp_ns=control_t,
            heading_difference=angle(calc['heading']-s['heading']),direction_maxabs=max(abs(a-b) for a,b in zip(calc['direction'],s['direction']))))
        match.setdefault(exact,[]).append(control_t)
    episodes=[]
    for ti,turn in enumerate(turns):
        inside=[(t,n) for t,n in states if turn['start']<=t<=turn['end'] and n.get('alignment_phase')=='align' and n.get('locked_heading') is not None and n.get('state')=='running']
        if not inside:continue
        first=inside[0][1];req,goalid=first['request_id'],first['current_goal']['goal_id'];k=(first['accepted_trajectory_id'],tuple(first['trajectory_reference_stamp']))
        if k not in curves:continue
        anchor_idx=bisect.bisect_right(ps_t,turn['start'])-1
        if anchor_idx<0 or turn['start']-ps_t[anchor_idx]>200_000_000:continue
        anchor=ps[anchor_idx];context=(req,goalid,k[0],k[1]);x=TurnDriftSupervisor(require_path_offset=require_path_offset)
        x.begin(context,anchor['stamp_ns'],anchor['p'],first['locked_heading'],metas[k]['body_goal'])
        trace=[];event=None
        for p in ps[anchor_idx+1:]:
            t=p['stamp_ns']
            if t>turn['end']:break
            si=bisect.bisect_right(state_t,t)-1;ci=bisect.bisect_right(cmd_t,t)-1
            if si<0 or ci<0:continue
            n=states[si][1];current=(n['accepted_trajectory_id'],tuple(n['trajectory_reference_stamp']))
            if current not in curves:continue
            _,_,_,st=follow_trajectory(np.asarray(p['p']),rotation_xyzw(p['q']),curves[current],np.asarray(metas[current]['body_goal']),tracking_pose=tracked[t],gate_translation=False,return_steering=True)
            c=cmd[ci][1]['value'];cc=(n['request_id'],n['current_goal']['goal_id'],current[0],current[1])
            result=x.observe(t,p['p'],st['heading'],cc,phase=n['alignment_phase'],actual_command=[c[0],c[1],c[5]],adapter_state=cmd[ci][1]['state'],protected=bool(n.get('tilt_hold') or n.get('obstacle_hold')),pose_age_ns=0)
            trace.append(dict(stamp_ns=t,raw_position=p['p'],current_path_heading=st['heading'],locked_heading=first['locked_heading'],yaw=math.atan2(rotation_xyzw(p['q'])[1,0],rotation_xyzw(p['q'])[0,0]),candidate=result))
            if result['event'] is not None:
                event=dict(result['event']);event['first_captured_original_control_using_this_pose_ns']=min(match.get(t,[]),default=None);break
        episodes.append(dict(original_turn_index=ti,original_emit_interval_ns=[turn['start'],turn['end']],
            original_raw_anchor=anchor,first_original_locked_heading=first['locked_heading'],source=context,
            candidate_event=event,trace_until_first_event=trace,
            counterfactual_after_event='not replayed; would require actual stop/native idle/new checked source and altered body physics' if event else None))
    output=dict(group=name,run=str(run),limits=dict(TurnDriftSupervisor.LIMITS,require_path_offset=require_path_offset),original_control_geometry_comparisons=len(checks),
      max_heading_difference=max((abs(c['heading_difference']) for c in checks),default=None),
      max_direction_difference=max((c['direction_maxabs'] for c in checks),default=None),
      original_turn_episodes=episodes,independent_original_turn_candidate_events=[e['candidate_event'] for e in episodes if e['candidate_event']],
      first_hypothetical_stop=next((e['candidate_event'] for e in episodes if e['candidate_event']),None),
      input_sha256={n:sha(run/n) for n in ['joint_stop_adapter.jsonl','feedback_navigation_trajectories.jsonl','first_four_result.json' if is_component else 'pose_audit.jsonl','feedback_navigation.jsonl']},
      limitations=['No GT input, no command/ROS node/physics or production edit.',
       'Native raw pose time/position preserved. Tracking filter and full metadata B-spline reconstructed with unchanged production code.',
       'Sparse original status state/identity is joined by embedded control clock, and geometry is independently checked against actual controller diagnostics.',
       'Candidate event time is a native measurement opportunity, not a measured new-command publication or demonstrated earlier stable outcome.',
       'Per-original-turn event count is diagnostic, not a new closed-loop trigger count after altered physics.',
       'Ideal pose receipt age only; callback wall not recorded in these original pose arrays. Live implementation must enforce original freshness.',
       'New source association validation does not replace original collision/progress/arrival/tilt controls.'])
    (HERE/(name+('_direction' if require_path_offset else '_displacement')+'_replay.json')).write_text(json.dumps(output,indent=2)+'\n')
    return dict(group=name,require_path_offset=require_path_offset,checks=len(checks),heading_max=output['max_heading_difference'],direction_max=output['max_direction_difference'],
        events=[dict(event=e['candidate_event']['event'],time=e['candidate_event']['stamp_ns']/1e9,original_turn=e['original_turn_index']) for e in episodes if e['candidate_event']])

if __name__=='__main__':
    result=[]
    for name,run,component in [('a_disabled',ROOT/'simulation/test_results/20261002_feedback_first4_a_disabled',True),('b_enabled',ROOT/'simulation/test_results/20261002_feedback_first4_b_enabled',True),('full18',ROOT/'runs/20261002_001559_882ddb',False)]:
        for profile in [True,False]:result.append(replay(name,run,component,profile))
    (HERE/'replay_summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
