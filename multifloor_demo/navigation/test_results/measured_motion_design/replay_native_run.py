#!/usr/bin/env python3
"""Passive native-header SLAM replay. No ROS/GT/control output or source edits."""
import bisect,hashlib,json,sys
from collections import Counter
from pathlib import Path
import numpy as np
from motion_observer import MeasuredMotionObserver as O
RUN=Path(sys.argv[1]).resolve();HERE=Path(__file__).parent

def rows(p):
    for line in p.open():
        try:yield json.loads(line)
        except json.JSONDecodeError:continue

def stats(a):
    if not a:return {'count':0}
    a=np.array(a);return dict(count=len(a),mean=np.mean(a,axis=0).tolist(),p05=np.quantile(a,.05,axis=0).tolist(),p95=np.quantile(a,.95,axis=0).tolist())

native_poses=[p for p in rows(RUN/'pose_audit.jsonl') if p['source']=='slam']
commands=[dict(t=round(c['sim']*1e9),value=[c['value'][0],c['value'][1],c['value'][5]],state=c['state']) for c in rows(RUN/'joint_stop_adapter.jsonl') if c['kind']=='actual_champ_command']
commands.sort(key=lambda c:c['t'])
contexts={}
for n in rows(RUN/'navigation_audit.jsonl'):
    s=n['status'];steer=s.get('steering')
    if steer:contexts[round(steer['stamp']*1e9)]=(s['request_id'],s['waypoint_index'],s.get('accepted_trajectory_id'))
ct=sorted(contexts)
o=O();j=0;reasons=Counter();samples=[];modes={}
for p in native_poses:
    t=p['stamp_ns']
    while j<len(commands) and commands[j]['t']<=t:
        c=commands[j];k=bisect.bisect_right(ct,c['t'])-1
        context=contexts[ct[k]] if k>=0 else None
        o.observe_applied_command(c['t'],c['value'],c['state'],context)
        j+=1
    # No callback wall stamp exists in pose_audit: ideal-age replay only.
    o.observe_pose(t,p['p'],p['q'],t)
    if p['stage'] not in ('exploring','returning_origin','navigating'):continue
    s=o.snapshot(t,t);s['stage']=p['stage'];samples.append(s)
    reasons[s['invalid_reason'] or 'valid']+=1
    if s['available']:
        a=modes.setdefault(s['execution_mode'],dict(raw=[],window=[],applied=[],valid_count=0))
        a['raw'].append([s['raw_body_twist']['linear'][0],s['raw_body_twist']['angular'][2]])
        a['window'].append([s['window_body_twist']['linear'][0],s['window_body_twist']['angular'][2],s['heading_rate_world']])
        a['applied'].append(s['actual_command_average'])
        a['valid_count']+=int(s['valid'])
result=dict(scope=__doc__,run=str(RUN),native_pose_count=len(native_poses),active_snapshot_count=len(samples),reasons=dict(reasons),
            observer_source_sha256=hashlib.sha256((HERE/'motion_observer.py').read_bytes()).hexdigest(),
            poses_native_integer_stamp=True,actual_command_timestamp_source='Adapter receipt simulation clock archived as float; reconstructed integer ns, not a message Header.',
            context_alignment='Historical status steering clock matched to applied-command receipt clock; original clocks archived as float, status can be sparse.',
            callback_wall_age_evaluated=False,ground_truth_consumed=False,command_output_generated=False,
            modes={m:{k:stats(v) if isinstance(v,list) else v for k,v in a.items()} for m,a in modes.items()},
            limitations=['Window quality tags are excluded unvalidated design; no feedback enabled.',
                         'Native pose payload/frame stamps exact; source callback receipt-wall absent, replay sets ideal age only.',
                         'Live evidence can grow after snapshot; these are partial diagnostics, not run acceptance.'])
(HERE/'run15_native_observer.json').write_text(json.dumps(result,indent=2)+'\n')
with (HERE/'run15_native_observer_snapshots.jsonl').open('w') as f:
    for s in samples:f.write(json.dumps(s,allow_nan=False)+'\n')
print(json.dumps(dict(native_pose_count=len(native_poses),active_snapshot_count=len(samples),reasons=dict(reasons),modes=result['modes']),indent=2))
