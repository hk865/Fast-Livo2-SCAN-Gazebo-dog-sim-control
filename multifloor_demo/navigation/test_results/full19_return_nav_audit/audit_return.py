"""Bounded owned-clean Full19 NAV audit. No ROS/CDR/control or truth refit."""
import bisect, hashlib, importlib.util, json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
RUN=ROOT/'runs/20261002_084329_14d9cc'
spec=importlib.util.spec_from_file_location('old_positions',ROOT/'navigation/test_results/champ_phase_first8_motion_audit/audit_motion.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
f=[json.loads(l) for l in (HERE/'bounded_feedback_navigation.jsonl').open()]
a=[json.loads(l) for l in (HERE/'bounded_joint_stop_adapter.jsonl').open()]
g=json.loads((RUN/'nav_drift_guard.json').read_text())
req='20261002_084329_14d9cc:return_origin:2'
supports=[];last=None;context=None
for e in g['events']:
 if e.get('request_id')!=req:continue
 if e['event']=='arm':context=json.dumps(e['context']);last=None
 if e['event'] in ('exact_zero_published','fresh_reference_requested','fresh_checked_path_accepted'):last=None;context=None
 if e['event'] not in ('arm','observe'):continue
 cur=(e['callback_clock_ns'],e.get('pose_stamp_ns',e.get('anchor_stamp_ns')),context,e['waypoint_index'])
 if last and context and cur[2:]==last[2:] and 0<cur[1]-last[1]<=200000000 and 0<cur[0]-last[0]<=250000000:supports.append((last[0],cur[0],'ALIGN'))
 last=cur
for x,y in zip(f,f[1:]):
 n,m=x['navigation'],y['navigation'];ident=lambda v:(v.get('alignment_phase'),v.get('accepted_trajectory_id'),v.get('waypoint_index'),v.get('tilt_hold'))
 if n.get('request_id')==m.get('request_id')==req and ident(n)==ident(m) and 0<y['sim_time']-x['sim_time']<=.35:
  if n['alignment_phase'] in ('drive','pre_turn','settle'):supports.append((round(x['sim_time']*1e9),round(y['sim_time']*1e9),n['alignment_phase'].upper()))
poses={round(x['slam']['stamp']*1e9):dict(stamp_ns=round(x['slam']['stamp']*1e9),p=x['slam']['pose'],q=x['slam']['quaternion']) for x in f}
truth={round(x['ground_truth']['stamp']*1e9):dict(stamp_ns=round(x['ground_truth']['stamp']*1e9),p=x['ground_truth']['pose'],q=x['ground_truth']['quaternion']) for x in f}
cmd=[(round(x['sim']*1e9),x['value']) for x in a if x['kind']=='actual_champ_command'];ct=[x[0] for x in cmd]
left=809100000000;right=min(856137000000,max(poses),max(truth));points=sorted({left,right}|{t for t in ct if left<t<right}|{t for s in supports for t in s[:2] if left<t<right})
iv=[];ss=sorted(supports);j=0;active=[]
for l,r in zip(points,points[1:]):
 while j<len(ss) and ss[j][0]<=l:active.append(ss[j]);j+=1
 active=[s for s in active if s[1]>=r];labs={s[2] for s in active if s[0]<=l and r<=s[1]}
 phase=next(iter(labs)) if len(labs)==1 else 'UNCERTAIN';c=cmd[bisect.bisect_right(ct,l)-1][1]
 mode='TRANSLATION' if c[0]!=0 or c[1]!=0 else 'ROTATION' if c[5]!=0 else 'ZERO';iv.append((l,r,phase+'_'+mode,c))
times=np.array([t/1e9 for l,r,_,_ in iv for t in (l,(l+r)/2,r)])
acc=json.loads((RUN/'acceptance.json').read_text());xf=acc['trajectory'];trans={'rotation_world_from_slam':xf['rotation_world_from_slam'],'translation':xf['translation_world_from_slam']}
mov={};gaps={};valid=np.ones(len(iv),dtype=bool)
for name,rows,tr,gap in [('SLAM',list(poses.values()),None,.2),('GT_evaluation',list(truth.values()),trans,.15)]:
 p,R,b=old.positions(rows,times,tr,1.0);ordered=np.array(sorted(x['stamp_ns'] for x in rows));probe=np.array([t for l,r,_,_ in iv for t in (l,(l+r)//2,r)]);idx=np.clip(np.searchsorted(ordered,probe,side='right'),1,len(ordered)-1);valid &= np.all((ordered[idx]-ordered[idx-1]).reshape(-1,3)<=round(gap*1e9),axis=1);dp=p[:,2]-p[:,0];body=np.einsum('nij,nj->ni',np.transpose(R[:,1],(0,2,1)),dp);yaw=np.arctan2(R[:,:,1,0],R[:,:,0,0]);dy=np.arctan2(np.sin(yaw[:,2]-yaw[:,0]),np.cos(yaw[:,2]-yaw[:,0]));mov[name]=(dp,body,dy);gaps[name]=b
iv=[(l,r,label if valid[i] else 'UNOBSERVED_POSE',c) for i,(l,r,label,c) in enumerate(iv)]
out={}
for label in sorted({x[2] for x in iv}):
 ix=[j for j,x in enumerate(iv) if x[2]==label];o={'duration_s':sum((iv[j][1]-iv[j][0])/1e9 for j in ix),'cmd_vx_integral_m':sum(iv[j][3][0]*(iv[j][1]-iv[j][0])/1e9 for j in ix),'cmd_yaw_integral_rad':sum(iv[j][3][5]*(iv[j][1]-iv[j][0])/1e9 for j in ix)}
 for k,(dp,b,dy) in mov.items():o[k]=None if label=='UNOBSERVED_POSE' else{'body_forward_m':float(b[ix,0].sum()),'body_lateral_m':float(b[ix,1].sum()),'delta_camera_init_m':dp[ix].sum(axis=0).tolist(),'yaw_delta_rad':float(dy[ix].sum())}
 out[label]=o
stops=[e for e in g['events'] if e.get('request_id')==req and e['event']=='exact_zero_published']; hand=[]
for s in stops:
 t=s['callback_clock_ns']/1e9;later=[e for e in g['events'] if e.get('request_id')==req and e['callback_clock_ns']>s['callback_clock_ns']];ref=next(e for e in later if e['event']=='fresh_reference_requested');accept=next(e for e in later if e['event']=='fresh_checked_path_accepted');rt=ref['callback_clock_ns']/1e9
 edge=next(e for e in a if e['kind']=='zero_edge' and e['sim']>=t-.001);ack=next(e for e in a if e['kind']=='native_zero_ack' and e['sim']>=edge['sim']);idle=next(e for e in a if e['kind']=='return_complete' and e['sim']>=ack['sim']);zeros=[e for e in a if e['kind']=='actual_champ_command' and t<=e['sim']<=accept['callback_clock_ns']/1e9]
 hand.append({'stop_sim':t,'cause':s['trigger']['event'],'details':s['trigger'],'edge':edge['sim'],'ACK':ack['sim'],'return_complete':idle['sim'],'reference':rt,'accepted_path':accept['callback_clock_ns']/1e9,'zero_emissions':len(zeros),'all_zero':all(all(v==0 for v in e['value']) for e in zeros),'idle_before_ref_s':rt-idle['sim'],'new_counter':ref['actual_adapter_ack']['counters']['stops']>s['adapter_stop_count_before'],'same_goal_ref_identity':accept['identity']['reference_stamp']==ref['reference_stamp'] and accept['identity']['body_goal']==ref['handoff']['body_goal']})
result={'scope':__doc__,'observed_goal1_ns':[left,right],'failure_ns':856137000000,'excluded_tail_ns':856137000000-right,'phase_support':'ALIGN paired fresh raw guard callbacks; other phases matching adjacent cached NAV snapshots, not exact change time. Conflicts/transition brackets UNCERTAIN.','groups':out,'pose_brackets_s':gaps,'single_initial_SE3_from_original_acceptance':{'initial_pair_stamp':xf['initial_pair_stamp'],'rotation_world_from_slam':xf['rotation_world_from_slam'],'translation_world_from_slam':xf['translation_world_from_slam']},'excluded_pose_bracket_duration_s':sum((r-l)/1e9 for l,r,label,c in iv if label=='UNOBSERVED_POSE'),'four_guard_handoffs':hand,'bounds':{'global_vx':max(abs(c[0]) for t,c in cmd if left<=t<=right),'mixed_yaw':max(abs(c[5]) for t,c in cmd if left<=t<=right and c[0]!=0),'vy_zero':all(c[1]==0 for t,c in cmd if left<=t<=right)},'limitations':['Original failed result unchanged.','Exact raw threshold headers provided by independent IMU audit; cached NAV phase is not native publish-entry trace.','No full CDR or raw Bspline collision recomputation.','Historical floating Adapter sim and recorder pose stamps used only for motion diagnostics, not canonical dwell.']}
(HERE/'motion_handoff_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'groups':out,'handoffs':hand,'pose_gaps':gaps,'bounds':result['bounds']},ensure_ascii=False))
