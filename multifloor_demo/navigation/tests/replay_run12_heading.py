#!/usr/bin/env python3
"""Replay actual archived SCAN tail and measured SLAM through both policies.

The archive preserves only the final 68-point path, unchanged after 524.774 sim
seconds. Earlier path history is unavailable. This is sampled input replay, not
a closed-loop prediction of physical progress or a claim that run12 now passes.
"""
import hashlib
import json
import math
from pathlib import Path
import sys
import tarfile
import types
import numpy as np

NAV=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(NAV))
import control_core as current

run=NAV.parent/'runs/20260930_200540_d9d7b3'
with tarfile.open(run/'source_snapshot.tar.gz') as archive:
    member=next(m for m in archive.getmembers() if m.name.endswith('navigation/control_core.py'))
    source=archive.extractfile(member).read()
original=types.ModuleType('archived_run12_control_core')
exec(compile(source,member.name,'exec'),original.__dict__)
path=np.array(json.loads((run/'mission.json').read_text())['planned_path'])
assert len(path)==68
poses=[]
for line in (run/'pose_audit.jsonl').read_text().splitlines():
    record=json.loads(line)
    if record['source']=='slam':poses.append(record)
tracking=original.TrackingPositionFilter()
filtered={}
old=original.HeadingGate();new=current.HeadingGate()
old.phase=new.phase='drive'
rows=[];old_stops=[];new_stops=[]
for p in poses:
    t=p['stamp'];raw=np.array(p['p']);point=tracking.update(raw,t)
    filtered[tuple(np.round(raw,10))]=point.copy()
    if not 524.774<=t<=527.9:continue
    rotation=current.rotation_xyzw(p['q'])
    yaw=math.atan2(rotation[1,0],rotation[0,0])
    _,_,old_target=original.follow_trajectory(raw,rotation,path,path[-1],tracking_pose=point,gate_translation=False)
    old_delta=old_target[:2]-point[:2]
    old_heading=math.atan2(old_delta[1],old_delta[0])
    _,_,new_target,info=current.follow_trajectory(raw,rotation,path,path[-1],tracking_pose=point,gate_translation=False,return_steering=True)
    assert np.array_equal(old_target,new_target),'policy must preserve the actual checked lookahead point'
    phases=(old.phase,new.phase)
    old_allowed,_=old.update(t,yaw,old_heading)
    new_allowed,_=new.update(t,yaw,info['heading'])
    if phases[0]=='drive' and old.phase=='pre_turn':old_stops.append(t)
    if phases[1]=='drive' and new.phase=='pre_turn':new_stops.append(t)
    rows.append(dict(stamp=t,old_error=old.error(old_heading,yaw),new_error=info['error'],
                     old_allowed=old_allowed,new_allowed=new_allowed,
                     checked_target_distance=float(np.linalg.norm(old_delta)),
                     new_correction_scale=info['correction_scale']))
errors=[]
for line in (run/'feedback_navigation.jsonl').read_text().splitlines():
    n=json.loads(line).get('navigation',{})
    if n.get('pose') is None or n.get('tracking_pose') is None:continue
    key=tuple(np.round(n['pose'],10))
    if key in filtered:errors.append(float(np.linalg.norm(filtered[key]-n['tracking_pose'])))
checks=dict(actual_tracking_filter_reconstruction=len(errors)>5000 and max(errors)<1e-10,
            original_tail_reproduces_discrete_turn_restart=len(old_stops)==1,
            fixed_scale_does_not_restart_on_same_inputs=not new_stops,
            no_new_scale_collapses_near_endpoint=all(r['new_correction_scale']==.8 for r in rows))
report=dict(scope=__doc__,no_ground_truth_inputs=True,passed=all(checks.values()),checks=checks,
            path_points=len(path),path_unchanged_after_sim=524.774,
            actual_observed_pre_turn=526.988,
            old_replay_pre_turn=old_stops,new_replay_pre_turn=new_stops,
            tracking_comparisons=len(errors),tracking_max_error=max(errors),
            old_max_heading_error=max(abs(r['old_error']) for r in rows),
            new_max_heading_error=max(abs(r['new_error']) for r in rows),
            old_translation_denied_samples=sum(not r['old_allowed'] for r in rows),
            new_translation_denied_samples=sum(not r['new_allowed'] for r in rows),
            source_archive_member=member.name,old_source_sha256=hashlib.sha256(source).hexdigest(),
            new_source_sha256=hashlib.sha256((NAV/'control_core.py').read_bytes()).hexdigest(),
            limitations=['sampled 10 Hz pose stamps; original live gate used 20 Hz ROS clock ticks',
                         'earlier trajectories were not archived; no exact whole-segment replay',
                         'counterfactual motion, sensor noise and replans would change in a live run',
                         'does not waive raw .22/.4 arrival, .30 joint acceptance, or 90 s timeout'],rows=rows)
output=NAV/'test_results/run12_heading_replay.json'
output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
assert report['passed'],report
