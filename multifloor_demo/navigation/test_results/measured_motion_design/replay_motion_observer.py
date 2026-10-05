#!/usr/bin/env python3
"""Historical-float replay of a command-free observer. No native-stamp claim."""
import hashlib,json
from collections import Counter
from pathlib import Path
import numpy as np
from motion_observer import MeasuredMotionObserver as O
ROOT=Path(__file__).resolve().parents[3]
RUN=ROOT/'simulation/test_results/20261001_balance_pair2_a_disabled'
HERE=Path(__file__).parent

def rows(p):
    for line in p.open():yield json.loads(line)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dist(x):
    a=np.asarray(x)
    if not len(a):return {'count':0}
    return dict(count=len(a),mean=np.mean(a,axis=0).tolist(),median=np.median(a,axis=0).tolist(),p05=np.quantile(a,.05,axis=0).tolist(),p95=np.quantile(a,.95,axis=0).tolist())

def main():
    pose={};active=(float('inf'),float('inf'))
    for r in rows(RUN/'feedback_navigation.jsonl'):
        n=r.get('navigation',{})
        if n.get('state')=='running' and active[0]==float('inf'):active=(r['sim_time'],active[1])
        if n.get('waypoint_index')==8 and active[1]==float('inf'):active=(active[0],r['sim_time'])
        if 'slam' in r:
            p=r['slam'];pose.setdefault(p['stamp'],dict(t=round(p['stamp']*1e9),p=p['pose'],q=p['quaternion'],wall=round(r['wall_elapsed']*1e9)))
    commands=[dict(t=round(r['sim']*1e9),value=[r['value'][0],r['value'][1],r['value'][5]],state=r['state'])
              for r in rows(RUN/'joint_stop_adapter.jsonl') if r['kind']=='actual_champ_command']
    # Historical command records had duplicate receipt clock ticks; preserve
    # input order at a tick and observer rejects ambiguous same-stamp changes.
    commands.sort(key=lambda c:c['t'])
    o=O();j=0;reasons=Counter();mode_stats={};snapshots=[];raw_errors=[]
    for p in sorted(pose.values(),key=lambda p:p['t']):
        while j<len(commands) and commands[j]['t']<=p['t']:
            c=commands[j];o.observe_applied_command(c['t'],c['value'],c['state']);j+=1
        o.observe_pose(p['t'],p['p'],p['q'],p['wall'])
        if not active[0]<=p['t']/1e9<active[1]:continue
        s=o.snapshot(p['t'],p['wall']);snapshots.append(s)
        reasons[s['invalid_reason'] or 'valid']+=1
        if s['available']:
            raw=s['raw_body_twist'];window=s['window_body_twist'];command=s['actual_command_average']
            mode_stats.setdefault(s['execution_mode'],dict(raw=[],window=[],command=[],valid_count=0,negative_yaw_response_count=0))
            a=mode_stats[s['execution_mode']]
            a['raw'].append([raw['linear'][0],raw['linear'][1],raw['angular'][2]])
            a['window'].append([window['linear'][0],window['linear'][1],window['angular'][2],s['heading_rate_world']])
            a['command'].append(command)
            if s['valid']:
                a['valid_count']+=1
                if abs(command[2])>.02 and abs(window['angular'][2])>.02 and command[2]*window['angular'][2]<0:a['negative_yaw_response_count']+=1
    result=dict(scope=__doc__,run=str(RUN.relative_to(ROOT)),observer_source_sha256=sha(HERE/'motion_observer.py'),
                replay_source_sha256=sha(Path(__file__)),feedback_sha256=sha(RUN/'feedback_navigation.jsonl'),
                actual_adapter_command_sha256=sha(RUN/'joint_stop_adapter.jsonl'),historical_stamp_conversion='round(recorded float seconds*1e9); not original native sec/nsec.',
                historical_wall_age_scope='pose wall is first recorder output containing pose, not source callback receive wall; replay age0 does not validate transport freshness.',
                active_interval=active,pose_snapshot_count=len(snapshots),reasons=dict(reasons),
                modes={m:{k:dist(v) if isinstance(v,list) else v for k,v in a.items()} for m,a in mode_stats.items()},
                output_columns=dict(raw=['bodyvx','bodyvy','bodyomega_z'],window=['bodyvx','bodyvy','bodyomega_z','worldheadingrate'],command=['actualvx','actualvy','actualyaw']),
                no_ground_truth_input=True,no_command_output=True,no_production_modification=True,
                limitations=['Fit gates .02m/.04rad are unvalidated design values; coverage/residual tags do not guarantee control safety.',
                             'Window rate is averaged actual pose movement, not an instantaneous rate; phase changes are rejected.',
                             'Adapter actual state/command changes align; historical full SCAN context_id is not joined, left None rather than inventing exact identity.',
                             'Single historical strictly FAIL tilt run is unchanged; negative signed responses are diagnostics, not plant-sign proof.'])
    (HERE/'observer_historical_replay.json').write_text(json.dumps(result,indent=2)+'\n')
    with (HERE/'observer_historical_snapshots.jsonl').open('w') as f:
        for s in snapshots:f.write(json.dumps(s,allow_nan=False)+'\n')
    print(json.dumps(dict(pose_snapshot_count=len(snapshots),reasons=dict(reasons),modes=result['modes']),indent=2))
if __name__=='__main__':main()
