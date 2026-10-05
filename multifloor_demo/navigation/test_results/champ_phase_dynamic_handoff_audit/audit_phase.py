"""Supported ALIGN/DRIVE response, not a vx!=0 phase assumption. Offline only."""
import bisect,hashlib,importlib.util,json,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('native_motion_audit',HERE/'audit_motion.py')
native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def analyze(run, start_ns=None, end_ns=None):
    assert json.loads((run/'process_cleanup.json').read_text())['owned_group_clean']
    d=json.loads((run/'dynamic_result.json').read_text())
    d.setdefault('request_id',d['region_evaluation']['regions'][0]['receipt']['request_id'])
    left=max(d['origin_stamp_ns'],min(r['stamp_ns'] for r in d['poses']),min(r['stamp_ns'] for r in d['truth']))
    right=min(d['terminal_stamp_ns'],max(r['stamp_ns'] for r in d['poses']),max(r['stamp_ns'] for r in d['truth']))
    if start_ns is not None:left=max(left,start_ns)
    if end_ns is not None:right=min(right,end_ns)
    g=json.loads((run/'nav_drift_guard.json').read_text());assert g['dropped']==0
    supports=[];last=None;context=None
    for e in g['events']:
        if e['event']=='arm':context=json.dumps(e['context'],sort_keys=True)
        if e['event'] in ['exact_zero_published','fresh_reference_requested','fresh_checked_path_accepted']:
            last=None;context=None
        if e['event'] not in ['arm','observe']:continue
        stamp=e['anchor_stamp_ns'] if e['event']=='arm' else e['pose_stamp_ns']
        current=dict(t=e['callback_clock_ns'],raw=stamp,ctx=context,index=e['waypoint_index'])
        if last and current['ctx'] is not None and current['ctx']==last['ctx'] and current['index']==last['index']:
            if 0<current['raw']-last['raw']<=200000000 and 0<current['t']-last['t']<=250000000:
                supports.append((last['t'],current['t'],'ALIGN','original arm/observe callback evidence'))
        last=current
    statuses=[s for s in d['statuses'] if s['status'].get('request_id')==d['request_id']]
    for a,b in zip(statuses,statuses[1:]):
        sa,sb=a['status'],b['status'];l=a['received_stamp_ns'];r=b['received_stamp_ns']
        same=(sa.get('alignment_phase'),sa.get('accepted_trajectory_id'),sa.get('waypoint_index'))==(sb.get('alignment_phase'),sb.get('accepted_trajectory_id'),sb.get('waypoint_index'))
        if same and 0<r-l<=350000000 and sa.get('state')=='running' and sb.get('state')=='running':
            phase=sa.get('alignment_phase')
            if phase in ['drive','pre_turn','settle']:
                supports.append((l,r,phase.upper(),'matching adjacent actual status; transition duration >sampling bracket'))
    commands=[]
    with (run/'joint_stop_adapter.jsonl').open() as f:
        for line in f:
            r=json.loads(line)
            if r['kind']=='actual_champ_command':commands.append((round(r['sim']*1e9),r['value']))
    ct=[r[0] for r in commands]
    points={left,right}
    points.update(t for t in ct if left<t<right)
    points.update(t for s in supports for t in s[:2] if left<t<right)
    points=sorted(points);support_start=sorted(supports)
    # The supported windows are small in number; classify once per boundary
    # while retaining conflicting/unsupported phase boundaries explicitly.
    intervals=[];j=0;active=[]
    for l,r in zip(points,points[1:]):
        while j<len(support_start) and support_start[j][0]<=l:
            active.append(support_start[j]);j+=1
        active=[s for s in active if s[1]>=r]
        labels={s[2] for s in active if s[0]<=l and r<=s[1]}
        phase=next(iter(labels)) if len(labels)==1 else 'UNCERTAIN'
        c=commands[max(0,bisect.bisect_right(ct,l)-1)][1]
        mode='TRANSLATION' if c[0]!=0 or c[1]!=0 else 'ROTATION' if c[5]!=0 else 'ZERO'
        intervals.append((l,r,phase+'_'+mode,c))
    times=np.array([x/1e9 for l,r,_,_ in intervals for x in [l,(l+r)/2,r]])
    transformed=d['region_evaluation']['initial_fixed_SE3_evaluation_only']
    motion={};gaps={}
    for name,rows,xf,gap in [('SLAM',d['poses'],None,.2),('GT_evaluation',d['truth'],transformed,.15)]:
        p,R,bracket=native.positions(rows,times,xf,gap)
        dp=p[:,2]-p[:,0];body=np.einsum('nij,nj->ni',np.transpose(R[:,1],(0,2,1)),dp)
        yaw=np.arctan2(R[:,:,1,0],R[:,:,0,0]);dyaw=np.arctan2(np.sin(yaw[:,2]-yaw[:,0]),np.cos(yaw[:,2]-yaw[:,0]))
        motion[name]=(body,dp,dyaw);gaps[name]=bracket
    groups={}
    for label in sorted({r[2] for r in intervals}):
        ix=[j for j,r in enumerate(intervals) if r[2]==label]
        item=dict(duration_sim_s=sum((intervals[j][1]-intervals[j][0])/1e9 for j in ix),
            emitted_vx_integral_m=sum(intervals[j][3][0]*(intervals[j][1]-intervals[j][0])/1e9 for j in ix),
            emitted_wz_integral_rad=sum(intervals[j][3][5]*(intervals[j][1]-intervals[j][0])/1e9 for j in ix))
        for name,(body,dp,dyaw) in motion.items():item[name]=dict(body_forward_m=float(body[ix,0].sum()),
            body_lateral_m=float(body[ix,1].sum()),delta_camera_init_m=dp[ix].sum(axis=0).tolist(),yaw_delta_rad=float(dyaw[ix].sum()))
        groups[label]=item
    active_cmd=[c for t,c in commands if left<=t<=right]
    mixed=[c for c in active_cmd if c[0]!=0 or c[1]!=0]
    pdstatus=[s['status']['turn_drift_guard']['align_translation'] for s in statuses if 'align_translation' in s['status'].get('turn_drift_guard',{})]
    return dict(run=str(run),original_passed=d['passed'],original_receipts=d['region_evaluation']['regions'],
        observed_interval_ns=[left,right],excluded_terminal_suffix_ns=d['terminal_stamp_ns']-right,
        active_peak_imu=d['active_max_imu_tilt'],drift_stops=g['interruptions'],
        groups=groups,pose_bracket_max_s=gaps,initial_single_SE3=transformed,
        guard_supported_ALIGN_windows=[s for s in supports if s[2]=='ALIGN'],
        phase_support_duration_s={phase:sum(v['duration_sim_s'] for k,v in groups.items() if k.startswith(phase+'_')) for phase in ['ALIGN','DRIVE','PRE_TURN','SETTLE','UNCERTAIN']},
        actual_emit_bounds=dict(mixed_count=len(mixed),mixed_vy_all_zero=all(c[1]==0 for c in mixed),
            mixed_abs_yaw_max=0. if not mixed else max(abs(c[5]) for c in mixed),
            active_abs_vx_max=max(abs(c[0]) for c in active_cmd)),
        PD_actual_configuration=None if not pdstatus else pdstatus[-1]['config'],
        source_sha256={n:sha(run/n) for n in ['dynamic_result.json','joint_stop_adapter.jsonl','nav_drift_guard.json']})

if __name__=='__main__':
    output=Path(sys.argv[2]);assert not output.exists()
    result=dict(scope=__doc__,candidate=analyze(Path(sys.argv[1]).resolve()),
        boundaries=['Original guard paired-callback support; sampled actual-status boundaries remain uncertain.',
        'No GT feedback, CDR decoding, ROS or new motion. Native emissions are not direct CHAMP receive/applied velocity.'])
    output.write_text(json.dumps(result,indent=2)+'\n')
    c=result['candidate'];print(json.dumps(dict(phase_duration=c['phase_support_duration_s'],bounds=c['actual_emit_bounds']),indent=2))
