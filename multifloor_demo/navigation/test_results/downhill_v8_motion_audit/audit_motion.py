"""Owned-clean v8 light-log motion/SCAN handoff audit. No ROS or CDR."""
import bisect
from collections import Counter
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from scipy.interpolate import BSpline

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
spec=importlib.util.spec_from_file_location('reuse_native_motion',ROOT/'navigation/test_results/champ_phase_dynamic_handoff_audit/audit_motion.py')
native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)

def run_audit(run,output):
    assert not output.exists(), 'Do not overwrite a previous audit'
    terminal=json.loads((run/'terminal_receipt.json').read_text())
    assert terminal['checks']['ownedclean'] is True
    d=json.loads((run/'downhill_result.json').read_text())
    assert d['origin_stamp_ns'] is not None and d['terminal_stamp_ns'] is not None
    req=d['request_id'];g=json.loads((run/'nav_drift_guard.json').read_text())
    assert g['dropped']==0
    events=[e for e in g['events'] if e.get('request_id')==req]
    adapter=[]
    for seq,line in enumerate((run/'joint_stop_adapter.jsonl').open(),1):
        x=json.loads(line)
        if x['kind'] in {'actual_champ_command','zero_edge','native_zero_ack','return_complete'}:
            x['native_file_line']=seq;adapter.append(x)
    commands=[x for x in adapter if x['kind']=='actual_champ_command']
    ns=lambda x:round(x['sim']*1e9)
    ct=[ns(x) for x in commands]
    assert all(a<=b for a,b in zip(ct,ct[1:]))
    start=max(d['origin_stamp_ns'],min(x['stamp_ns'] for x in d['poses']),min(x['stamp_ns'] for x in d['truth']),ct[0])
    end=min(d['terminal_stamp_ns'],max(x['stamp_ns'] for x in d['poses']),max(x['stamp_ns'] for x in d['truth']),ct[-1])
    supports=[];last=None;context=None
    for e in events:
        if e['event']=='arm':context=json.dumps(e['context'],sort_keys=True);last=None
        if e['event'] in {'exact_zero_published','fresh_reference_requested','fresh_checked_path_accepted'}:last=None;context=None
        if e['event'] not in {'arm','observe'}:continue
        cur=(e['callback_clock_ns'],e.get('pose_stamp_ns',e.get('anchor_stamp_ns')),context,e['waypoint_index'])
        if last and context and cur[2:]==last[2:] and 0<cur[1]-last[1]<=200000000 and 0<cur[0]-last[0]<=250000000:
            supports.append((last[0],cur[0],'ALIGN'))
        last=cur
    statuses=[x for x in d['statuses'] if x['status'].get('request_id')==req]
    identity=lambda s:(s.get('alignment_phase'),s.get('accepted_trajectory_id'),s.get('waypoint_index'),s.get('tilt_hold'))
    for a,b in zip(statuses,statuses[1:]):
        sa,sb=a['status'],b['status'];l,r=a['received_stamp_ns'],b['received_stamp_ns']
        if identity(sa)==identity(sb) and 0<r-l<=350000000 and sa.get('state')==sb.get('state')=='running' and sa.get('alignment_phase') in {'drive','pre_turn','settle'}:
            supports.append((l,r,sa['alignment_phase'].upper()))
    receipts=[x['receipt'] for x in d['region_evaluation']['regions'] if x.get('receipt')]
    goal_edges=[start]+[r['stamp_ns'] for r in receipts]+[end]
    points=sorted({start,end}|{t for t in ct if start<t<end}|{t for s in supports for t in s[:2] if start<t<end}|{t for t in goal_edges if start<t<end})
    iv=[];ss=sorted(supports);active=[];j=0
    for l,r in zip(points,points[1:]):
        while j<len(ss) and ss[j][0]<=l:active.append(ss[j]);j+=1
        active=[s for s in active if s[1]>=r]
        labs={s[2] for s in active if s[0]<=l and r<=s[1]}
        phase=next(iter(labs)) if len(labs)==1 else 'UNCERTAIN'
        ci=bisect.bisect_right(ct,l)-1;c=commands[ci]['value']
        mode='TRANSLATION' if c[0]!=0 or c[1]!=0 else 'ROTATION' if c[5]!=0 else 'ZERO'
        goal=min(bisect.bisect_right(goal_edges,l)-1,len(d['goals_definitions'])-1)
        iv.append(dict(l=l,r=r,phase=phase,mode=mode,c=c,goal=goal,command_gap_ns=ct[ci+1]-ct[ci] if ci+1<len(ct) else None))
    times=np.array([t/1e9 for v in iv for t in (v['l'],(v['l']+v['r'])/2,v['r'])])
    probes=np.array([t for v in iv for t in (v['l'],(v['l']+v['r'])//2,v['r'])],dtype=np.int64)
    valid=np.ones(len(iv),dtype=bool);motion={};gaps={}
    transform=d['region_evaluation']['initial_fixed_SE3_evaluation_only']
    for name,rows,xf,maxgap in [('SLAM',d['poses'],None,200000000),('GT_evaluation',d['truth'],transform,150000000)]:
        p,R,_=native.positions(rows,times,xf,100.)
        stamps=np.array(sorted({x['stamp_ns'] for x in rows}),dtype=np.int64)
        idx=np.clip(np.searchsorted(stamps,probes,side='right'),1,len(stamps)-1)
        bracket=stamps[idx]-stamps[idx-1]
        valid &= np.all(bracket.reshape(-1,3)<=maxgap,axis=1)
        gaps[name]=int(bracket.max())
        dp=p[:,2]-p[:,0];body=np.einsum('nij,nj->ni',np.transpose(R[:,1],(0,2,1)),dp)
        yaw=np.arctan2(R[:,:,1,0],R[:,:,0,0]);dy=np.arctan2(np.sin(yaw[:,2]-yaw[:,0]),np.cos(yaw[:,2]-yaw[:,0]))
        motion[name]=(dp,body,dy)
    for i,v in enumerate(iv):
        v['pose_valid']=bool(valid[i]);v['label']=v['phase']+'_'+v['mode'] if valid[i] else 'UNOBSERVED_POSE'
    def total(ix):
        o={'duration_s':sum((iv[i]['r']-iv[i]['l'])/1e9 for i in ix),
           'emitted_vx_integral_m':sum(iv[i]['c'][0]*(iv[i]['r']-iv[i]['l'])/1e9 for i in ix),
           'emitted_wz_integral_rad':sum(iv[i]['c'][5]*(iv[i]['r']-iv[i]['l'])/1e9 for i in ix)}
        for name,(dp,body,dy) in motion.items():
            o[name]=None if any(not iv[i]['pose_valid'] for i in ix) else dict(body_forward_m=float(body[ix,0].sum()),body_lateral_m=float(body[ix,1].sum()),delta_camera_init_m=dp[ix].sum(axis=0).tolist(),yaw_delta_rad=float(dy[ix].sum()))
        return o
    groups={label:total([i for i,v in enumerate(iv) if v['label']==label]) for label in sorted({v['label'] for v in iv})}
    sign=lambda v:1 if v>0.005 else -1 if v<-.005 else 0
    segments=[];i=0
    while i<len(iv):
        v=iv[i];key=(v['label'],v['goal'],sign(v['c'][5]));j=i+1
        while j<len(iv) and (iv[j]['label'],iv[j]['goal'],sign(iv[j]['c'][5]))==key:j+=1
        segments.append(dict(start_ns=v['l'],end_ns=iv[j-1]['r'],phase_mode=key[0],goal_index=key[1],emitted_wz_sign=key[2],**total(list(range(i,j)))))
        i=j
    reverse=[s for s in segments if s['emitted_wz_sign'] and s['duration_s']>=.4 and s['phase_mode'] in {'ALIGN_TRANSLATION','DRIVE_TRANSLATION'} and s['GT_evaluation'] and s['SLAM'] and s['emitted_wz_sign']*s['GT_evaluation']['yaw_delta_rad']<0 and s['emitted_wz_sign']*s['SLAM']['yaw_delta_rad']<0]
    traj=[json.loads(l) for l in (run/'feedback_navigation_trajectories.jsonl').open()]
    md=[x for x in traj if x['kind']=='scan_metadata'];paths=[x for x in traj if x['kind']=='accepted_path' and x['points']]
    def reconstruct(m):
        v=m['trajectory'];k=np.array(v['knots']);o=v['order'];t=np.linspace(k[o],k[-o-1],min(2000,max(2,int((k[-o-1]-k[o])/.08)+1)))
        return BSpline(k,v['pos_pts'],o)(t)
    associations=[]
    for path in paths:
        opts=[]
        for row in md:
            v=row['metadata']['trajectory']
            # NAV make_path uses publish clock, not Bspline.start_time.
            # Pair the full payload by receipt proximity + reconstructed points.
            if abs(row['receipt_sim_time']-path['receipt_sim_time'])>.030:continue
            pp=reconstruct(row['metadata'])
            if np.shape(path['points'])==pp.shape:opts.append((float(np.max(np.abs(pp-np.array(path['points'])))),row))
        error,m=min(opts,key=lambda x:x[0]) if opts else (None,None)
        associations.append(dict(path_sim=path['receipt_sim_time'],matched=m is not None and error<=1e-12,max_reconstruction_error=error,reference_stamp=None if m is None else m['metadata']['reference_stamp'],traj_id=None if m is None else m['metadata']['trajectory']['traj_id'],body_goal=None if m is None else m['metadata']['body_goal']))
    edges=[i for i,x in enumerate(adapter) if x['kind']=='zero_edge'];raw={x['stamp_ns']:x['p'] for x in d['poses']}
    stops=[i for i,e in enumerate(events) if e['event']=='exact_zero_published'];handoffs=[]
    for count,i in enumerate(stops):
        stop=events[i];later=events[i+1:stops[count+1] if count+1<len(stops) else len(events)]
        ref=next((e for e in later if e['event']=='fresh_reference_requested'),None);accept=next((e for e in later if e['event']=='fresh_checked_path_accepted'),None)
        if ref is None or accept is None:
            handoffs.append(dict(stop_ns=stop['callback_clock_ns'],cause=stop['trigger']['event'],completed=False));continue
        edge_i=edges[stop['adapter_stop_count_before']]
        next_edge=next((e for e in edges if e>edge_i),len(adapter));section=adapter[edge_i:next_edge]
        edge=adapter[edge_i];ack=next(e for e in section if e['kind']=='native_zero_ack');idle=next(e for e in section if e['kind']=='return_complete')
        before=next(e for e in reversed(adapter[:edge_i]) if e['kind']=='actual_champ_command')
        resume_i=next((n for n in range(edge_i,len(adapter)) if adapter[n]['kind']=='actual_champ_command' and any(adapter[n]['value'])),None)
        held=[e for e in adapter[edge_i:resume_i] if e['kind']=='actual_champ_command']
        identity=accept['identity'];meta=next(x['metadata'] for x in md if x['metadata']['reference_stamp']==ref['reference_stamp'] and x['metadata']['trajectory']['traj_id']==identity['traj_id'])
        matching=[x for x in associations if x['traj_id']==identity['traj_id'] and x['reference_stamp']==ref['reference_stamp'] and abs(round(x['path_sim']*1e9)-accept['callback_clock_ns'])<=30000000]
        checks=dict(raw_trigger_exact=stop['trigger']['stamp_ns'] in raw and np.array_equal(stop['trigger']['raw_position'],raw[stop['trigger']['stamp_ns']]),native_zero_first=not any(before['value']),zero_until_resume=bool(held) and all(not any(e['value']) for e in held),counter_increment=ref['actual_adapter_ack']['counters']['stops']>stop['adapter_stop_count_before'],fresh_idle_ACK=ref['actual_adapter_ack']['state']=='idle' and ns(idle)<=ref['handoff']['actual_zero_since_ns'],idle_dwell_at_least_1sim=ref['callback_clock_ns']-ref['handoff']['actual_zero_since_ns']>=1000000000,same_goal_reference=np.allclose(meta['body_goal'],ref['handoff']['body_goal'],rtol=0,atol=1e-8) and identity['reference_stamp']==ref['reference_stamp'],published_path_exact=bool(matching) and all(x['matched'] for x in matching),original_pre_at_least_1sim=resume_i is not None and ns(adapter[resume_i])>=accept['callback_clock_ns']+1000000000)
        handoffs.append(dict(stop_ns=stop['callback_clock_ns'],goal_index=stop['waypoint_index'],cause=stop['trigger']['event'],details=stop['trigger'],completed=True,actual_first_zero_ns=ns(before),zero_edge_ns=ns(edge),ACK_ns=ns(ack),return_complete_ns=ns(idle),reference_ns=ref['callback_clock_ns'],checked_path_ns=accept['callback_clock_ns'],first_native_motion_ns=None if resume_i is None else ns(adapter[resume_i]),zero_emissions=len(held),checks={k:bool(v) for k,v in checks.items()}))
    activecmd=[x['value'] for x in commands if start<=ns(x)<=end];mixed=[c for c in activecmd if c[0]!=0 or c[1]!=0]
    sources=['downhill_result.json','joint_stop_adapter.jsonl','nav_drift_guard.json','feedback_navigation_trajectories.jsonl','terminal_receipt.json','fixture_manifest.json','runtime_manifest.json']
    result=dict(scope=__doc__,run=str(run),original_result=dict(passed=d['passed'],failure=d['failure'],origin_ns=d['origin_stamp_ns'],terminal_ns=d['terminal_stamp_ns'],region_receipts=receipts,active_peak_imu=d['active_max_imu_tilt']),
        observed_interval_ns=[start,end],excluded_prefix_ns=start-d['origin_stamp_ns'],excluded_terminal_tail_ns=d['terminal_stamp_ns']-end,original_single_initial_SE3=transform,
        phase_mode_groups=groups,continuous_phase_same_yaw_sign_segments=segments,mixed_reverse_observations_at_least_04s=reverse,
        phase_uncertain_s=sum((v['r']-v['l'])/1e9 for v in iv if v['phase']=='UNCERTAIN'),unobserved_pose_s=sum((v['r']-v['l'])/1e9 for v in iv if not v['pose_valid']),max_pose_bracket_ns=gaps,max_command_edge_gap_ns=max(v['command_gap_ns'] or 0 for v in iv),
        source_associations=associations,guard_handoffs=handoffs,stop_causes=dict(Counter(h['cause'] for h in handoffs)),
        checks=dict(owned_clean=terminal['checks']['ownedclean'],original_result_unchanged=True,all_guard_handoffs=len(handoffs)==g['interruptions'] and all(h['completed'] and all(h['checks'].values()) for h in handoffs),all_nonempty_accepted_paths_full_metadata_reconstructed=bool(associations) and all(a['matched'] for a in associations),all_running_goal_definition_hash_matches=all(x['status'].get('goals_definition_sha256')==d['goals_definition_sha256'] for x in statuses if x['status']['state']=='running'),mixed_yaw_cap=all(abs(c[5])<=.080000000001 for c in mixed),global_vx_cap=all(abs(c[0])<=.120000000001 for c in activecmd),vy_zero=all(c[1]==0 for c in activecmd)),
        bounds=dict(global_vx=max(abs(c[0]) for c in activecmd),mixed_yaw=max(abs(c[5]) for c in mixed)),input_sha256={n:native.sha(run/n) for n in sources},
        limitations=['Original component FAIL and original region windows are retained unchanged.','ALIGN support uses paired raw guard callbacks; other phases use cached adjacent native statuses, not exact phase-transition publisher entry. Conflicts and crossing transitions are UNCERTAIN.','Adapter sim is historical cached float clock, not a Header; emission is not direct CHAMP receive, applied velocity or applied effort.','SE3 is original initial transform only; GT evaluates motion and never enters control. No CDR scan or contact/JTC mechanism inferred.','Pose gaps are excluded from motion; final common-pose tail is not extrapolated.','Continuous same-sign mixed yaw response is descriptive for the observed window; no steady plant identification or generic sign-error claim.','Full metadata/path reconstruction validates association, not an independent collision optimization replay.'])
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed_light_checks=all(result['checks'].values()),checks=result['checks'],groups=groups,reverse_mixed_count=len(reverse),handoffs=len(handoffs),receipt_sha256=native.sha(output)),indent=2))

if __name__=='__main__':run_audit(Path(sys.argv[1]).resolve(),Path(sys.argv[2]).resolve())
