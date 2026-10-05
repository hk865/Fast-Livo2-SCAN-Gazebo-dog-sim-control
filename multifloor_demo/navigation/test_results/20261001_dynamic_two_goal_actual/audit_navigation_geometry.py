#!/usr/bin/env python3
"""Read-only archived actual NAV audit. No ROS, publication, GT control or replay.

GT yaw independently checks actual sensor yaw at its own odometry stamp. Sparse
status snapshots bound transitions; they are not all 20-Hz controller samples.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.interpolate import BSpline

OUT = Path(__file__).resolve().parent
NAV = OUT.parents[1]
sys.path.insert(0, str(NAV))
from control_core import follow_trajectory, rotation_xyzw


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sample(m):
    t = m['trajectory']
    k = np.asarray(t['knots'])
    a, b = k[t['order']], k[-t['order']-1]
    return BSpline(k, np.asarray(t['pos_pts']), t['order'])(
        np.linspace(a, b, min(2000, max(2, int((b-a)/.08)+1))))


def main():
    result = json.loads((OUT/'dynamic_result.json').read_text())
    metadata = [json.loads(s) for s in (OUT/'trajectory_payloads.jsonl').open()]
    paths = [json.loads(s) for s in (OUT/'executed_paths.jsonl').open()]
    goal = np.asarray(result['goals'][1])
    curves, curve_reports = {}, []
    for row in metadata:
        m = row['metadata']
        t = m['trajectory']
        tid = t['traj_id']
        if tid in curves:
            continue
        pts = sample(m)
        curves[tid] = pts
        if np.linalg.norm(np.asarray(m['body_goal'])-goal) > 1e-6:
            continue
        stamp = t['start_time'][0]+t['start_time'][1]*1e-9
        # Metadata and actual path headers can differ by one callback-clock ms.
        # Preserve that delta separately; exact full point-array equality is
        # the stronger association here, not a claimed equal header stamp.
        candidates = [p for p in paths if abs(p['received_sim']-row['received_sim']) < .1
                       and np.asarray(p['points']).shape == pts.shape]
        differences = [float(np.max(np.abs(pts-np.asarray(p['points'])))) for p in candidates]
        chord = pts[-1,:2]-pts[0,:2]
        norm = np.linalg.norm(chord)
        unit = chord/norm
        lateral = (pts[:,:2]-pts[0,:2]) @ np.array([-unit[1],unit[0]])
        inc = np.diff(pts[:,:2], axis=0)
        curve_reports.append(dict(id=tid, received_sim=row['received_sim'], reference_stamp=m['reference_stamp'],
            body_goal=m['body_goal'], adjusted_body_goal=m['adjusted_body_goal'], start_state=m['start_state'],
            start=pts[0].tolist(), end=pts[-1].tolist(), xy_min=pts[:,:2].min(axis=0).tolist(),
            xy_max=pts[:,:2].max(axis=0).tolist(), xy_arc_length=float(np.linalg.norm(inc,axis=1).sum()),
            xy_chord_length=float(norm), max_chord_lateral=float(abs(lateral).max()),
            backtracking_along_chord=float(np.minimum(inc@unit,0).sum()),
            actual_accepted_path_max_difference=min(differences) if differences else None,
            metadata_to_path_header_stamp_differences=[p['stamp']-stamp for p in candidates]))

    statuses=[]
    for row in result['statuses']:
        s=row['status']
        if s['waypoint_index'] != 1 or not s.get('steering'):
            continue
        if not statuses or (row['sim'],s['steering']['stamp'],s['alignment_phase']) != (
                statuses[-1]['sim'],statuses[-1]['status']['steering']['stamp'],statuses[-1]['status']['alignment_phase']):
            statuses.append(row)
    truth=result['truth']
    truth_stamps=np.array([t['stamp'] for t in truth])
    offset=result['heading_alignment']['yaw_camera_init_from_world']

    def details(row):
        s=row['status']; st=s['steering']
        d=np.asarray(st['target'])[:2]-st['projection']
        tangent=math.atan2(d[1],d[0])
        body=wrap(st['heading']-st['error'])
        gt=truth[int(np.argmin(abs(truth_stamps-st['odom_stamp'])))]
        rot=rotation_xyzw(gt['q'])
        gt_yaw=wrap(math.atan2(rot[1,0],rot[0,0])+offset)
        c,v=math.cos(body),math.sin(body)
        rotation=np.array([[c,-v,0],[v,c,0],[0,0,1]])
        steer=follow_trajectory(np.asarray(s['pose']),rotation,curves[st['trajectory_id']],goal,
            tracking_pose=np.asarray(s['tracking_pose']),gate_translation=False,return_steering=True)[3]
        return dict(received_sim=row['sim'], control_stamp=st['stamp'], odom_stamp=st['odom_stamp'],
            id=st['trajectory_id'], raw_pose=s['pose'], tracking_pose=s['tracking_pose'],
            phase=s['alignment_phase'], heading=st['heading'], tangent=tangent,
            cross_track_angle=wrap(st['heading']-tangent), cross_track_m=float(.8*math.tan(wrap(st['heading']-tangent))),
            actual_body_yaw=body, independent_GT_body_yaw=gt_yaw,
            sensor_yaw_minus_GT=wrap(body-gt_yaw), GT_stamp_gap=gt['stamp']-st['odom_stamp'],
            error=st['error'], locked_heading=s['locked_heading'], heading_deviation_since=s['heading_deviation_since'],
            repro_heading_difference=wrap(steer['heading']-st['heading']),
            raw_goal_distance=float(np.linalg.norm(np.asarray(s['pose'])-goal)), obstacle_hold=s['obstacle_hold'])

    transitions=[]; last_key=None
    changes=[]; triggers=[]; drive_start=None
    for row in statuses:
        s=row['status']; st=s['steering']
        key=(s['state'],s['alignment_phase'],s['obstacle_hold'],st['trajectory_id'])
        if key != last_key:
            transitions.append(dict(state=s['state'],**details(row)))
            last_key=key
    for old,new in zip(statuses,statuses[1:]):
        a,b=old['status'],new['status']; sa,sb=a['steering'],b['steering']
        if b['alignment_phase']=='drive' and a['alignment_phase']!='drive':
            drive_start=new
        if a['alignment_phase']=='drive' and b['alignment_phase']=='pre_turn':
            info=details(new)
            info['last_observed_drive']=details(old)
            if drive_start is not None:
                begin=details(drive_start)
                info['drive_begin']=begin
                info['delta_body_yaw_since_drive_begin']=wrap(info['actual_body_yaw']-begin['actual_body_yaw'])
                info['delta_path_tangent_since_drive_begin']=wrap(info['tangent']-begin['tangent'])
                info['delta_cross_track_angle_since_drive_begin']=wrap(info['cross_track_angle']-begin['cross_track_angle'])
                info['delta_desired_heading_since_drive_begin']=wrap(info['heading']-begin['heading'])
            triggers.append(info)
        if sa['trajectory_id'] != sb['trajectory_id']:
            d=details(new); body=d['actual_body_yaw']
            c,v=math.cos(body),math.sin(body); rot=np.array([[c,-v,0],[v,c,0],[0,0,1]])
            args=(np.asarray(b['pose']),rot)
            previous=follow_trajectory(*args,curves[sa['trajectory_id']],goal,
                tracking_pose=np.asarray(b['tracking_pose']),gate_translation=False,return_steering=True)[3]
            current=follow_trajectory(*args,curves[sb['trajectory_id']],goal,
                tracking_pose=np.asarray(b['tracking_pose']),gate_translation=False,return_steering=True)[3]
            changes.append(dict(received_sim=new['sim'], previous_id=sa['trajectory_id'], id=sb['trajectory_id'],
                obstacle_hold=b['obstacle_hold'], phase=b['alignment_phase'],
                same_pose_heading_change=wrap(current['heading']-previous['heading']),
                old_heading=previous['heading'],new_heading=current['heading']))

    second_start=result['nav_confirmations'][0]['received_sim']
    failed=next(row for row in result['statuses'] if row['status']['state']=='failed')
    active_poses=[(float(np.linalg.norm(np.asarray(p['p'])-goal)),p['stamp']) for p in result['poses']
                  if second_start<=p['stamp']<=failed['sim']]
    box=result['obstacle_states']
    phases=[]; old=None
    for row in box:
        s=row['state']; phase=s['visible_phase']
        if phase!=old:
            phases.append(dict(received_sim=row['received_sim'],stamp=s['stamp'],phase=phase,position=s['position']))
            old=phase
    box_positions=np.asarray([b['state']['position'] for b in box])
    safe=[c for c in result['command_evidence'] if c['source']=='safe' and 102.409<=c['sim']<failed['sim']]
    walk_s=capped_s=0.
    for left,right in zip(safe,safe[1:]):
        dt=right['sim']-left['sim']
        if left['command'][0]>.015:
            walk_s+=dt
            if abs(left['command'][2])>=.039999:
                capped_s+=dt
    manifest=json.loads((OUT/'source_manifest.json').read_text())['sha256']
    nav_hashes={p:d for p,d in manifest.items() if p.startswith('navigation/')}
    navigation_source_differences={p:sha(NAV.parent/p) for p,d in nav_hashes.items()
                                  if not (NAV.parent/p).exists() or sha(NAV.parent/p)!=d}
    report=dict(scope=__doc__, archived_input_SHA256={f:sha(OUT/f) for f in ['dynamic_result.json','trajectory_payloads.jsonl','executed_paths.jsonl','feedback_navigation.jsonl','feedback_navigation_trajectories.jsonl']},
        production_remained_frozen=not navigation_source_differences,
        checked_navigation_source_files=len(nav_hashes), navigation_source_differences=navigation_source_differences,
        component_passed=result['passed'], checks=result['checks'],
        unchanged_limits=result['unchanged_limits'], second_waypoint_interval=[second_start,failed['sim']],
        second_raw_goal=goal.tolist(), first_actual_NAV_arrival=result['nav_confirmations'][0],
        failure=failed['status']['message'], final_observed_failed_status=details(failed),
        closest_second_raw_goal= min(active_poses), precision=result['initial_SE3_precision'],
        active_max_raw_IMU_tilt=result['active_max_imu_tilt'], active_bridge_holds=result['active_bridge_holds'],
        curves=curve_reports, actual_status_transitions=transitions,
        realignment_triggers=triggers, accepted_curve_changes=changes,
        box_phase_edges=phases, actual_acknowledged_box_max_displacement=float(np.linalg.norm(box_positions-box_positions[0],axis=1).max()),
        after_second_resume_safe_command_walk_duration_above_015=walk_s,
        after_second_resume_safe_command_yaw_cap_duration=capped_s,
        after_second_resume_safe_command_yaw_cap_fraction=capped_s/walk_s,
        maximum_sensor_GT_yaw_difference_at_realignment=max(abs(t['sensor_yaw_minus_GT']) for t in triggers),
        maximum_reproduced_running_heading_difference=max(abs(details(s)['repro_heading_difference']) for s in statuses if s['status']['state']=='running'),
        final_diagnostics={k:failed['status'][k] for k in ['degenerate_splines','trajectory_association_rejected','reference_requests','replans','last_spline_rejected']},
        conclusion=['Second waypoint genuinely fails its original 90s contract; first waypoint arrival and real moving-obstacle wait/replan/resume are recorded, not full Demo success.',
            'Every current-goal payload keeps the original adjusted goal unchanged; no degenerate or identity-rejected spline causes the post-clearance stall.',
            'After second clearance, actual accepted traj13 persists for ~42sim seconds. First three drive-to-pre_turn transitions occur on that same path, so repeated replans are not their trigger.',
            'Independent body yaw confirms the observed negative heading errors. Along-path movement reduces cross-track steering angle while body yaw increases; both contribute to crossing the unchanged .20rad persistent gate.',
            'Safe walking yaw is already at its .04rad/s cap for ~62% of sampled active walking time after second clearance. A gain-only increase at the same cap cannot repair those saturated intervals; a physical yaw-response component diagnosis is needed before changing limits.',
            'Command/physical displacement evidence must be assessed alongside this geometry report; no source edits, threshold changes, fitted/GT steering, or second simulation are applied.'])
    path=OUT/'navigation_geometry_audit.json'
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(path)
    print('interval',report['second_waypoint_interval'],'closest',report['closest_second_raw_goal'])
    print('yaw_difference',report['maximum_sensor_GT_yaw_difference_at_realignment'],'reproduction',report['maximum_reproduced_running_heading_difference'])
    print('box',report['box_phase_edges'],'displacement',report['actual_acknowledged_box_max_displacement'])
    for c in curve_reports:
        print('curve',c['id'],round(c['received_sim'],3),'arc',round(c['xy_arc_length'],3),'cross',round(c['max_chord_lateral'],3),'payload_diff',c['actual_accepted_path_max_difference'])
    for t in triggers:
        print('trigger',t['received_sim'],t['id'],'body_delta',t.get('delta_body_yaw_since_drive_begin'),'tangent_delta',t.get('delta_path_tangent_since_drive_begin'),'cross_delta',t.get('delta_cross_track_angle_since_drive_begin'),'error',t['error'])


if __name__=='__main__':
    main()
