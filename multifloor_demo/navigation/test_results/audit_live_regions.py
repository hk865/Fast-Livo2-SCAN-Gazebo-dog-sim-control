#!/usr/bin/env python3
"""Readonly actual NAV receipts and native SLAM geometry. No GT or ROS."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
NAV=Path(__file__).resolve().parents[1];sys.path.insert(0,str(NAV))
from goal_regions import parse_goal,definitions_sha256,contains
RUN=Path(sys.argv[1]).resolve();OUT=Path(__file__).with_name('run15_region_navigation_review.json')

def rows(p):
    if not p.exists():return
    for line in p.open():
        try:yield json.loads(line)
        except json.JSONDecodeError:continue

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

mission=json.loads((RUN/'mission.json').read_text())
poses={r['stamp_ns']:r for r in rows(RUN/'pose_audit.jsonl') if r['source']=='slam'}
requests={};problems=[]
for r in rows(RUN/'navigation_audit.jsonl'):
    s=r['status'];rid=s.get('request_id')
    if not rid or not s.get('goals_definitions'):continue
    d=requests.setdefault(rid,dict(definitions=s['goals_definitions'],definition_hash=s['goals_definition_sha256'],receipts={},protected_status_stamps=[],latest_status=s))
    d['latest_status']=s
    if d['definitions']!=s['goals_definitions'] or d['definition_hash']!=s['goals_definition_sha256']:problems.append('mutated goal definitions: '+rid)
    for receipt in s.get('region_arrivals',[]):
        gid=receipt['goal_id'];prior=d['receipts'].get(gid)
        if prior is not None and prior!=receipt:problems.append('mutated receipt: '+rid+':'+gid)
        d['receipts'][gid]=receipt
    e=s.get('region_arrival_evidence')
    if e and (s.get('tilt_hold') or s.get('obstacle_hold') or s.get('execution_bridge_safety',{}).get('state')!='ready'):
        d['protected_status_stamps'].append(e['stamp_ns'])
reports=[]
for rid,d in requests.items():
    goals=tuple(parse_goal(g) for g in d['definitions']);h=definitions_sha256(goals)
    if h!=d['definition_hash']:problems.append('definition hash mismatch: '+rid)
    receipts=[];previous=-1
    for i,g in enumerate(goals):
        r=d['receipts'].get(g.goal_id)
        if r is None:continue
        checks=dict(goal_id_index_matches=r['waypoint_index']==i,full_definition_hash_matches=r['goals_definition_sha256']==h,
                    ordered_start_strictly_after_previous=r['start_stamp_ns']>previous,
                    integer_time=all(type(r[k]) is int for k in ('stamp_ns','start_stamp_ns','dwell_ns')),
                    elapsed_ns_matches=r['stamp_ns']-r['start_stamp_ns']==r['dwell_ns'],
                    dwell_at_least_declared=r['dwell_ns']>=round(g.dwell_sim_s*1e9),
                    receipt_not_protected=r.get('protected') is False and r.get('reason')=='arrived',
                    arrival_definition_matches=r['arrival_definition']==g.definition()['arrival'])
        points=[p for t,p in sorted(poses.items()) if r['start_stamp_ns']<=t<=r['stamp_ns']]
        checks['start_and_end_native_raw_pose_exist']=bool(points) and points[0]['stamp_ns']==r['start_stamp_ns'] and points[-1]['stamp_ns']==r['stamp_ns']
        checks['receipt_raw_position_matches_native_end']=bool(points) and points[-1]['p']==r['raw_position']
        checks['native_raw_pose_continuous_inside']=bool(points) and all(contains(g,p['p']) for p in points)
        maxgap=max((b['stamp_ns']-a['stamp_ns'] for a,b in zip(points,points[1:])),default=0)
        checks['native_observation_gap_within_contract']=maxgap<=r['max_observation_gap_ns']
        checks['no_observed_protected_status_during_receipt']=not any(r['start_stamp_ns']<=t<=r['stamp_ns'] for t in d['protected_status_stamps'])
        valid=all(checks.values())
        if not valid:problems.append('invalid receipt: '+rid+':'+g.goal_id)
        receipts.append(dict(goal_id=g.goal_id,index=i,checks=checks,passed=valid,receipt=r,native_pose_count=len(points),native_max_gap_ns=maxgap))
        previous=r['stamp_ns']
    s=d['latest_status'];reports.append(dict(request_id=rid,definition_hash=h,index=s['waypoint_index'],total=s['total'],state=s['state'],receipts=receipts,
        current_goal=s.get('current_goal'),arrival_evidence=s.get('region_arrival_evidence'),steering=s.get('steering'),phase=s.get('alignment_phase'),
        counts=dict(replans=s.get('replans'),obstacle_stops=s.get('obstacle_stops'),obstacle_resumes=s.get('obstacle_resumes'),tilt_stops=s.get('tilt_stops'),max_raw_tilt=s.get('raw_imu_max_tilt_rad'))))
result=dict(scope=__doc__,run=str(RUN),stage=mission['stage'],elapsed_s=mission['elapsed_s'],current_request=mission.get('current_request'),
            mission_current_hash=mission.get('current_goals_sha256'),requests=reports,problems=problems,
            source_sha256={n:sha(NAV/n) for n in ['controller.py','goal_regions.py','control_core.py']},
            scope_limitations=['NAV raw-stamp/region receipt audit only. GT/complete route/physical envelope evaluated independently.',
                              'Protected-status exclusion only official status samples, not proof of complete raw IMU event coverage.',
                              'Live files can grow after this read; no original result is modified.'])
OUT.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(stage=result['stage'],elapsed_s=result['elapsed_s'],problems=problems,
                     requests=[dict(request_id=r['request_id'],index=r['index'],total=r['total'],receipt_pass_count=sum(x['passed'] for x in r['receipts']),phase=r['phase'],counts=r['counts']) for r in reports]),indent=2))
