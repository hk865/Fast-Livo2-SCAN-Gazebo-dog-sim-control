"""Owned-clean read-only goal8 geometry/velocity evidence; no control output.

Native pose Header stamps retain integer nanoseconds. Adapter command clocks
are historical cached sim times, not a transport Header or applied force.
GT is used only to independently assess fitted raw-SLAM velocities.
"""
import bisect
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation, Slerp

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def wrap(x):return float(np.arctan2(np.sin(x),np.cos(x)))
def stats(v):
    a=np.asarray(v,dtype=float)
    return dict(count=len(a),mean=None if not len(a) else float(a.mean()),
                median=None if not len(a) else float(np.median(a)),
                p05=None if not len(a) else float(np.percentile(a,5)),
                p95=None if not len(a) else float(np.percentile(a,95)),
                abs_p95=None if not len(a) else float(np.percentile(np.abs(a),95)),
                maximum=None if not len(a) else float(a.max()),
                minimum=None if not len(a) else float(a.min()))

def main(run,out):
    assert json.loads((run/'process_cleanup.json').read_text())['owned_group_clean']
    assert not out.exists()
    data=json.loads((run/'first_eight_result.json').read_text())
    motion=json.loads((out.parent/'lowD_first8_motion.json').read_text())
    start,end=355800000000,data['terminal_stamp_ns']
    poses=sorted({r['stamp_ns']:r for r in data['poses']}.values(),key=lambda r:r['stamp_ns'])
    pt=np.array([r['stamp_ns'] for r in poses],dtype=np.int64)
    pp=np.array([r['p'] for r in poses]);pr=Rotation.from_quat([r['q'] for r in poses]).as_matrix()
    statuses=[r for r in data['statuses'] if r['status'].get('request_id')==data['request_id']]
    st=[r['received_stamp_ns'] for r in statuses]
    commands=[]
    with (run/'joint_stop_adapter.jsonl').open() as f:
        for line in f:
            r=json.loads(line)
            if r['kind']=='actual_champ_command':
                c=r['value'];r['ns']=round(r['sim']*1e9)
                r['mode']='walk' if c[0]!=0 or c[1]!=0 else 'turn' if c[5]!=0 else 'zero'
                commands.append(r)
    ct=[r['ns'] for r in commands]
    def command(t):return commands[max(0,bisect.bisect_right(ct,int(t))-1)]
    def status(t):return statuses[max(0,bisect.bisect_right(st,int(t))-1)]['status']
    def pose(t):
        j=bisect.bisect_left(pt,int(t))
        if j<len(pt) and pt[j]==t:return pp[j],pr[j]
        if j==0 or j==len(pt) or pt[j]-pt[j-1]>200000000:return None
        a=(t-pt[j-1])/(pt[j]-pt[j-1]);p=(1-a)*pp[j-1]+a*pp[j]
        R=Slerp([0.,1.],Rotation.from_matrix(pr[j-1:j+1]))(a).as_matrix()
        return p,R
    goal=np.array(data['goals_definitions'][7]['center'])
    compact=[]
    for row in statuses:
        s=row['status'];t=row['received_stamp_ns']
        if s.get('waypoint_index')!=7 or not(start<=t<=end):continue
        z=pose(t);steer=s.get('steering');record=dict(received_stamp_ns=t,
            phase=s.get('alignment_phase'),trajectory_id=s.get('accepted_trajectory_id'),
            reference_stamp=s.get('trajectory_reference_stamp'),locked_heading=s.get('locked_heading'),
            steering=steer,raw_pose=s.get('pose'),tracking_pose=s.get('tracking_pose'),
            turn_drift_pending=s.get('turn_drift_guard',{}).get('pending'),
            obstacle_hold=s.get('obstacle_hold'),pose_age=s.get('pose_age'))
        if z:
            p,R=z;yaw=float(np.arctan2(R[1,0],R[0,0]));bearing=float(np.arctan2(goal[1]-p[1],goal[0]-p[0]))
            record.update(body_yaw=yaw,target_bearing=bearing,goal_error=(goal-p).tolist(),
                current_path_error=None if not steer else wrap(steer['heading']-yaw),
                path_minus_bearing=None if not steer else wrap(steer['heading']-bearing),
                locked_error=None if s.get('locked_heading') is None else wrap(s['locked_heading']-yaw))
        compact.append(record)
    segments=[]
    for s in motion['mode_segments']:
        if s['end']<=start/1e9 or s['start']>=end/1e9:continue
        rows=[r for r in compact if s['start']<=r['received_stamp_ns']/1e9<=s['end']]
        segments.append(dict(mode=s['mode'],start=s['start'],end=s['end'],
            duration=s['duration_sim_s'],GT_bodyforward=s['truth_evaluation_only']['local_body_forward_m'],
            GT_yaw_delta=s['truth_evaluation_only']['yaw_delta_rad'],
            emitted_wz_integral=s['emitted_command_wz_integral_rad'],
            status_count=len(rows),first=None if not rows else rows[0],last=None if not rows else rows[-1],
            path_error=stats([r['current_path_error'] for r in rows if r.get('current_path_error') is not None]),
            path_minus_bearing=stats([r['path_minus_bearing'] for r in rows if r.get('path_minus_bearing') is not None])))
    traj=[]
    with (run/'feedback_navigation_trajectories.jsonl').open() as f:
        for line in f:
            r=json.loads(line)
            if r['kind']!='scan_metadata' or not(start/1e9<=r['receipt_sim_time']<=end/1e9):continue
            md=r['metadata'];v=md['trajectory'];k=np.array(v['knots']);order=v['order']
            spl=BSpline(k,v['pos_pts'],order)
            samples=spl(np.linspace(k[order],k[-order-1],min(2000,max(2,int((k[-order-1]-k[order])/.08)+1))))
            j=0;arc=0.
            while j+1<len(samples) and arc<.8:
                arc+=float(np.linalg.norm(samples[j+1,:2]-samples[j,:2]));j+=1
            tangent=samples[j,:2]-samples[0,:2]
            p=np.array(md['start_state']['position']);bearing=float(np.arctan2(goal[1]-p[1],goal[0]-p[0]))
            head=float(np.arctan2(tangent[1],tangent[0]))
            traj.append(dict(receipt_sim=r['receipt_sim_time'],trajectory_id=v['traj_id'],
                reference_stamp=md['reference_stamp'],body_goal=md['body_goal'],
                adjusted_body_goal=md['adjusted_body_goal'],start_state=md['start_state'],
                initial_08m_tangent_heading=head,start_to_goal_bearing=bearing,
                tangent_minus_goal_bearing=wrap(head-bearing),sample_count=len(samples)))
    # Fit the last .4s of actual native raw poses. Never fit across long gaps,
    # command mode transitions, identities or outside the original active run.
    tt=sorted({r['stamp_ns']:r for r in data['truth']}.values(),key=lambda r:r['stamp_ns'])
    truth_t=np.array([r['stamp_ns'] for r in tt],dtype=np.int64)
    truth_p=np.array([r['p'] for r in tt])
    fixed=data['region_evaluation']['initial_fixed_SE3_evaluation_only']
    Rw=np.array(fixed['rotation_world_from_slam']);Tw=np.array(fixed['translation'])
    def truth_at(ts):
        idx=np.searchsorted(truth_t,ts,side='left');valid=[]
        for t,j in zip(ts,idx):
            valid.append((j<len(truth_t) and truth_t[j]==t) or
                         (0<j<len(truth_t) and truth_t[j]-truth_t[j-1]<=150000000))
        if not all(valid):return None
        q=np.column_stack([np.interp((ts-truth_t[0])/1e9,(truth_t-truth_t[0])/1e9,truth_p[:,i]) for i in range(3)])
        return (q-Tw)@Rw
    velocities=[];rejected={}
    for j in range(1,len(poses)):
        t=int(pt[j]);a=bisect.bisect_left(pt,t-400000001)
        ids=np.arange(a,j+1);ts=pt[ids];span=int(ts[-1]-ts[0])
        if t<data['origin_stamp_ns'] or t>end:continue
        reason=None
        if len(ids)<4 or span<350000000:reason='insufficient_span'
        elif np.max(np.diff(ts))>200000000:reason='pose_gap'
        c0=command(ts[0]);c1=command(t)
        source_cmds=commands[max(0,bisect.bisect_right(ct,int(ts[0]))-1):bisect.bisect_right(ct,t)]
        if reason is None and any(c['mode']!=c1['mode'] for c in source_cmds):reason='mode_transition'
        sx,sy=status(ts[0]),status(t)
        if reason is None and (sx.get('waypoint_index'),sx.get('accepted_trajectory_id'))!=(sy.get('waypoint_index'),sy.get('accepted_trajectory_id')):reason='identity_transition'
        if reason:
            rejected[reason]=rejected.get(reason,0)+1;continue
        x=(ts-ts[-1])/1e9;X=np.column_stack([np.ones(len(ids)),x]);coef=np.linalg.lstsq(X,pp[ids],rcond=None)[0]
        rms=float(np.sqrt(np.mean(np.sum((pp[ids]-X@coef)**2,axis=1))))
        body=pr[j].T@coef[1]
        secant=pr[j].T@((pp[j]-pp[j-1])/((pt[j]-pt[j-1])/1e9))
        truth=truth_at(ts);error=None
        if truth is not None:error=float((pr[j].T@(coef[1]-np.linalg.lstsq(X,truth,rcond=None)[0][1]))[0])
        velocities.append(dict(stamp_ns=t,mode=c1['mode'],goal_index=sy.get('waypoint_index'),
            phase=sy.get('alignment_phase'),rms_m=rms,fit_valid=rms<=.02,
            body_vx=float(body[0]),body_vy=float(body[1]),raw_secant_body_vx=float(secant[0]),
            GT_fit_evaluation_vx_error=error,fit_span_ns=span,effective_delay_ns=int(ts[-1]-ts.mean()),
            nominal_stamp_latency_ns=int(sy.get('pose_age',0)*1e9)))
    velocity_summary={}
    for mode in ['turn','walk','zero']:
        allrows=[r for r in velocities if r['mode']==mode]
        rows=[r for r in allrows if r['fit_valid']]
        velocity_summary[mode]=dict(candidates=len(allrows),valid=len(rows),
            body_vx=stats([r['body_vx'] for r in rows]),body_vy=stats([r['body_vy'] for r in rows]),
            fit_position_rms_m=stats([r['rms_m'] for r in allrows]),
            raw_secant_body_vx=stats([r['raw_secant_body_vx'] for r in allrows]),
            GT_fit_evaluation_vx_error=stats([r['GT_fit_evaluation_vx_error'] for r in rows if r['GT_fit_evaluation_vx_error'] is not None]),
            effective_delay_ns=stats([r['effective_delay_ns'] for r in rows]))
    v8=[r for r in velocities if r['goal_index']==7 and r['mode']=='turn' and r['fit_valid']]
    velocity_summary['goal8_turn']=dict(count=len(v8),body_vx=stats([r['body_vx'] for r in v8]),
        rms_m=stats([r['rms_m'] for r in v8]),GT_fit_evaluation_vx_error=stats([r['GT_fit_evaluation_vx_error'] for r in v8 if r['GT_fit_evaluation_vx_error'] is not None]))
    active_ages=[r['status']['pose_age'] for r in statuses if data['origin_stamp_ns']<=r['received_stamp_ns']<=end and 'pose_age' in r['status']]
    poslast=poses[bisect.bisect_right(pt,end)-1];axes=np.array(data['goals_definitions'][7]['arrival']['axes'])
    active8=[r for r in poses if start<=r['stamp_ns']<=end]
    closest=min(active8,key=lambda r:abs(float(axes[0]@(np.array(r['p'])-goal))))
    final=dict(raw_stamp_ns=poslast['stamp_ns'],local_goal_error=(axes@(np.array(poslast['p'])-goal)).tolist(),
        closest_along_raw=dict(stamp_ns=closest['stamp_ns'],local_goal_error=(axes@(np.array(closest['p'])-goal)).tolist()))
    answer=dict(scope='offline actual first8; original physical FAIL retained',
        run=str(run),source_sha256={n:sha(run/n) for n in ['first_eight_result.json','joint_stop_adapter.jsonl','feedback_navigation_trajectories.jsonl']},
        audit_source_sha256=sha(Path(__file__)),goal8_native_motion=motion['goal_motion'][7],
        goal8_status_snapshots=compact,goal8_mode_segments=segments,goal8_metadata=traj,
        goal8_final_region_error=final,velocity_04s_fit=velocity_summary,velocity_rejections=rejected,
        raw_pose_cached_status_wall_age_s=stats(active_ages),velocity_samples=velocities,
        limitations=['No GT enters an observer or command; independent GT comparison only.',
            'Native Adapter emission is not measured CHAMP reception or applied force.',
            'Status snapshots are 4Hz observations; they cannot establish exact gate transition timestamps.',
            'Native original pose gap442.8->443.1 retained; no other channel fills it.',
            'Velocity retrospective fit has .15-.2sim effective lag; callback/transport latency not captured by pose rows.',
            'No candidate control output is simulated and no counterfactual trajectory is claimed.'])
    out.write_text(json.dumps(answer,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(goal8_segment_count=len(segments),metadata_count=len(traj),velocity_summary=velocity_summary,
        final_region_error=final,pose_age_s=answer['raw_pose_cached_status_wall_age_s'],output_sha256=sha(out)),indent=2))

if __name__=='__main__':main(Path(sys.argv[1]).resolve(),Path(sys.argv[2]).resolve())
