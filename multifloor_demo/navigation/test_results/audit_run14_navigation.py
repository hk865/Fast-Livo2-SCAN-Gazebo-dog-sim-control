#!/usr/bin/env python3
"""Read-only actual full14 NAV/Bspline audit; GT is independent yaw evaluation.

Status is sampled at 4 Hz. Phase/hold edges below are observation bounds,
not claims of exact controller or protection callback times.
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline

NAV = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAV))
from control_core import follow_trajectory, rotation_xyzw

RUN = NAV.parent/'runs/20261001_174341_6cd895'
OUT = NAV/'test_results/run14_navigation_decomposition.json'

def read(name):
    rows=[]
    for line in (RUN/name).open():
        try: rows.append(json.loads(line))
        except json.JSONDecodeError: pass
    return rows

def wrap(v): return math.atan2(math.sin(v), math.cos(v))

def sample(meta):
    t=meta['trajectory']; k=np.asarray(t['knots']); order=t['order']
    a,b=k[order],k[-order-1]
    return BSpline(k,np.asarray(t['pos_pts']),order)(np.linspace(a,b,min(2000,max(2,int((b-a)/.08)+1))))

def main():
    feedback=read('feedback_navigation.jsonl')
    trajectory=read('feedback_navigation_trajectories.jsonl')
    truth=[r for r in read('pose_audit.jsonl') if r.get('source')=='truth']
    truth_stamps=np.asarray([r['stamp'] for r in truth])
    mission=json.loads((RUN/'mission.json').read_text())
    heading=mission['heading_alignment']['yaw_camera_init_from_world']
    metadata={r['metadata']['trajectory']['traj_id']:r for r in trajectory if r['kind']=='scan_metadata'}
    curves={k:sample(r['metadata']) for k,r in metadata.items()}
    paths=[r for r in trajectory if r['kind']=='accepted_path']
    relevant=[]
    for r in feedback:
        n=r.get('navigation',{}); st=n.get('steering')
        if n.get('waypoint_index')==7 and st and n.get('state') in ['running','failed']:
            key=(n['state'],st['stamp'],n['alignment_phase'],n['tilt_hold'],n['tilt_stops'])
            if not relevant or key!=relevant[-1][0]: relevant.append((key,r))
    rows=[r for _,r in relevant]
    goal=np.asarray(metadata[rows[0]['navigation']['steering']['trajectory_id']]['metadata']['body_goal'])

    def details(r):
        n=r['navigation']; s=n['steering']; tid=s['trajectory_id']
        body=wrap(s['heading']-s['error'])
        tangent=math.atan2(s['target'][1]-s['projection'][1],s['target'][0]-s['projection'][0])
        gt=truth[int(np.argmin(abs(truth_stamps-s['odom_stamp'])))]
        R=rotation_xyzw(gt['q']); gtyaw=wrap(math.atan2(R[1,0],R[0,0])+heading)
        c,v=math.cos(body),math.sin(body); yaw_R=np.array([[c,-v,0],[v,c,0],[0,0,1]])
        repro=follow_trajectory(np.asarray(n['pose']),yaw_R,curves[tid],goal,
            tracking_pose=np.asarray(n['tracking_pose']),gate_translation=False,return_steering=True)[3]
        return dict(observed_sim=r['sim_time'],control_stamp=s['stamp'],odom_stamp=s['odom_stamp'],
            phase=n['alignment_phase'],state=n['state'],id=tid,pose=n['pose'],tracking_pose=n['tracking_pose'],
            heading=s['heading'],error=s['error'],body_yaw=body,path_tangent=tangent,
            cross_angle=wrap(s['heading']-tangent),independent_GT_yaw=gtyaw,
            yaw_minus_GT=wrap(body-gtyaw),GT_stamp_gap=gt['stamp']-s['odom_stamp'],
            raw_goal_distance=float(np.linalg.norm(np.asarray(n['pose'])-goal)),
            tilt_hold=n['tilt_hold'],tilt_stops=n['tilt_stops'],raw_tilt=n['raw_imu_tilt_rad'],
            tilt_source=n['tilt_source'],message=n['message'],
            reproduced_heading_difference=wrap(repro['heading']-s['heading']))

    phase_edges=[]; triggers=[]; curve_changes=[]; drive_begin=None; previous_key=None
    for r in rows:
        n=r['navigation']; s=n['steering']; key=(n['state'],n['alignment_phase'],n['tilt_hold'],s['trajectory_id'])
        if key!=previous_key: phase_edges.append(details(r)); previous_key=key
    for a,b in zip(rows,rows[1:]):
        na,nb=a['navigation'],b['navigation']; sa,sb=na['steering'],nb['steering']
        if nb['alignment_phase']=='drive' and na['alignment_phase']!='drive': drive_begin=b
        if na['alignment_phase']=='drive' and nb['alignment_phase']=='pre_turn':
            d=details(b); d['last_drive']=details(a)
            if drive_begin:
                begin=details(drive_begin);d['drive_begin']=begin
                for field in ['body_yaw','path_tangent','cross_angle','heading']:
                    d['delta_'+field]=wrap(d[field]-begin[field])
            triggers.append(d)
        if sa['trajectory_id']!=sb['trajectory_id']:
            d=details(b); c,v=math.cos(d['body_yaw']),math.sin(d['body_yaw']); R=np.array([[c,-v,0],[v,c,0],[0,0,1]])
            heads=[]
            for tid in [sa['trajectory_id'],sb['trajectory_id']]:
                steer=follow_trajectory(np.asarray(nb['pose']),R,curves[tid],goal,
                    tracking_pose=np.asarray(nb['tracking_pose']),gate_translation=False,return_steering=True)[3]
                heads.append(steer['heading'])
            curve_changes.append(dict(observed_sim=b['sim_time'],previous_id=sa['trajectory_id'],id=sb['trajectory_id'],
                phase=nb['alignment_phase'],same_pose_heading_change=wrap(heads[1]-heads[0])))

    curve_reports=[]
    for tid,row in metadata.items():
        m=row['metadata']
        if np.linalg.norm(np.asarray(m['body_goal'])-goal)>1e-8:continue
        p=curves[tid]; inc=np.diff(p[:,:2],axis=0); chord=p[-1,:2]-p[0,:2]; length=np.linalg.norm(chord)
        unit=chord/length
        candidates=[q for q in paths if abs(q['receipt_sim_time']-row['receipt_sim_time'])<.1 and np.asarray(q['points']).shape==p.shape]
        differences=[float(np.max(abs(np.asarray(q['points'])-p))) for q in candidates]
        curve_reports.append(dict(id=tid,receipt_sim=row['receipt_sim_time'],reference_stamp=m['reference_stamp'],
            body_goal=m['body_goal'],adjusted_body_goal=m['adjusted_body_goal'],start_state=m['start_state'],
            start=p[0].tolist(),end=p[-1].tolist(),arc_length=float(np.linalg.norm(inc,axis=1).sum()),
            chord_length=float(length),max_lateral_from_chord=float(abs((p[:,:2]-p[0,:2])@np.array([-unit[1],unit[0]])).max()),
            backtracking=float(np.minimum(inc@unit,0).sum()),actual_accepted_path_max_difference=min(differences) if differences else None))
    tilt_edges=[];old=None
    for r in rows:
        n=r['navigation']; key=(n['tilt_hold'],n['tilt_stops'],n['tilt_source'])
        if key!=old:tilt_edges.append(details(r));old=key
    report=dict(scope=__doc__,run=str(RUN),original_failure_preserved=True,goal=goal.tolist(),
        input_SHA256={name:hashlib.sha256((RUN/name).read_bytes()).hexdigest() for name in ['mission.json','feedback_navigation.jsonl','feedback_navigation_trajectories.jsonl','pose_audit.jsonl','acceptance.json']},
        actual_status_interval=[rows[0]['sim_time'],rows[-1]['sim_time']],first_status=details(rows[0]),last_status=details(rows[-1]),
        closest_status_raw_goal=min((details(r)['raw_goal_distance'],r['sim_time']) for r in rows),
        phase_edges=phase_edges,realignments=triggers,curve_changes=curve_changes,curves=curve_reports,tilt_edges=tilt_edges,
        maximum_running_heading_reproduction_difference=max(abs(details(r)['reproduced_heading_difference']) for r in rows if r['navigation']['state']=='running'),
        maximum_unprotected_drive_heading_reproduction_difference=max(abs(details(r)['reproduced_heading_difference'])
            for r in rows if r['navigation']['state']=='running' and not r['navigation']['tilt_hold']
            and r['navigation']['alignment_phase']=='drive'),
        maximum_running_sensor_GT_yaw_difference=max(abs(details(r)['yaw_minus_GT']) for r in rows if r['navigation']['state']=='running'),
        final_diagnostics={k:mission['navigation'].get(k) for k in ['degenerate_splines','trajectory_association_rejected','tilt_stops','obstacle_stops','raw_imu_max_tilt_rad']},
        independent_physics_report='simulation/test_results/20261001_full14_physics_audit.json')
    OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print('saved',OUT); print('interval',report['actual_status_interval'],'closest',report['closest_status_raw_goal'])
    print('heading_reproduction',report['maximum_running_heading_reproduction_difference'],'independentyaw',report['maximum_running_sensor_GT_yaw_difference'])
    for t in triggers: print('realign',t['observed_sim'],'id',t['id'],'err',t['error'],'body',t.get('delta_body_yaw'),'tangent',t.get('delta_path_tangent'),'cross',t.get('delta_cross_angle'),'tilt',t['tilt_hold'])
    print('changes',curve_changes)
    for e in tilt_edges: print('tilt',e['observed_sim'],e['tilt_hold'],e['tilt_stops'],e['tilt_source'],e['raw_tilt'],e['message'])
    for c in curve_reports: print('curve',c['id'],'arc',c['arc_length'],'chord',c['chord_length'],'cross',c['max_lateral_from_chord'],'backtrack',c['backtracking'],'payload_diff',c['actual_accepted_path_max_difference'])

if __name__=='__main__': main()
