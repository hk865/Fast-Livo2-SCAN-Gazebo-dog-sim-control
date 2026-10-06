#!/usr/bin/env python3
"""Read archived slope probes; never create nodes, publish or use GT feedback."""
import hashlib
import json
import math
import pathlib
import sys

import numpy as np
from scipy.interpolate import BSpline

NAV = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAV))
from control_core import follow_trajectory, rotation_xyzw


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def sample(metadata):
    t = metadata['trajectory']
    knots = np.array(t['knots'])
    begin, end = knots[t['order']], knots[-t['order']-1]
    return BSpline(knots, np.array(t['pos_pts']), t['order'])(
        np.linspace(begin, end, min(2000, max(2, int((end-begin)/.08)+1))))


def audit(directory):
    result = json.loads((directory/'slope_result.json').read_text())
    metadata = [json.loads(line) for line in (directory/'trajectory_payloads.jsonl').open()]
    paths = [json.loads(line) for line in (directory/'executed_paths.jsonl').open()]
    curves = {}
    curve_reports = []
    goal = np.array(result['goal'])
    for row in metadata:
        m = row['metadata']
        tid = m['trajectory']['traj_id']
        if tid in curves:
            continue
        p = sample(m)
        curves[tid] = p
        differences = [float(np.max(np.abs(p-np.array(r['points'])))) for r in paths
                       if np.array(r['points']).shape == p.shape
                       and abs(r['stamp']-(m['trajectory']['start_time'][0]+
                               m['trajectory']['start_time'][1]*1e-9)) < 1e-6]
        chord = p[-1, :2]-p[0, :2]
        unit = chord / np.linalg.norm(chord)
        lateral = (p[:, :2]-p[0, :2]) @ np.array([-unit[1], unit[0]])
        increments = np.diff(p[:, :2], axis=0)
        curve_reports.append(dict(trajectory_id=tid, received_sim=row['received_sim'],
            start=p[0].tolist(), end=p[-1].tolist(), xy_min=p[:, :2].min(axis=0).tolist(),
            xy_max=p[:, :2].max(axis=0).tolist(), xy_arc_length=float(np.linalg.norm(increments,axis=1).sum()),
            xy_chord_length=float(np.linalg.norm(chord)), max_chord_cross_track=float(abs(lateral).max()),
            backtracking_along_chord=float(np.minimum(increments@unit,0).sum()),
            sampled_payload_to_actual_path_max_difference=min(differences) if differences else None,
            requested_goal=m['body_goal'], adjusted_goal=m['adjusted_body_goal'],
            start_state=m['start_state']))
    statuses = [r['status'] for r in result['statuses'] if r['status'].get('steering')
                and r['status']['state']=='running']
    dedup = []
    for s in statuses:
        if not dedup or (s['steering']['stamp'],s['alignment_phase']) != (
                dedup[-1]['steering']['stamp'],dedup[-1]['alignment_phase']):
            dedup.append(s)
    truth = result['truth']
    stamps = np.array([r['stamp'] for r in truth])
    heading_offset = result['heading_alignment']['yaw_camera_init_from_world']
    triggers = []
    changes = []
    for old, new in zip(dedup,dedup[1:]):
        st = new['steering']
        desired, error = st['heading'],st['error']
        body = wrap(desired-error)
        target_vector = np.array(st['target'])[:2]-np.array(st['projection'])
        tangent = math.atan2(target_vector[1],target_vector[0])
        if old['alignment_phase']=='drive' and new['alignment_phase']=='pre_turn':
            # Direction/error used the most recent SLAM odometry, not the later
            # controller clock. Match its sensor stamp for independent yaw.
            near = truth[int(np.argmin(abs(stamps-st['odom_stamp'])))]
            r = rotation_xyzw(near['q'])
            truth_yaw = wrap(math.atan2(r[1,0],r[0,0])+heading_offset)
            triggers.append(dict(stamp=st['stamp'], trajectory_id=st['trajectory_id'],
                pose=new['pose'], tracking_pose=new['tracking_pose'],
                heading=desired, path_tangent=tangent, cross_track_angle=wrap(desired-tangent),
                measured_body_yaw=body, independent_gt_body_yaw=truth_yaw,
                independent_yaw_difference=wrap(body-truth_yaw), truth_stamp_gap=near['stamp']-st['odom_stamp'],
                current_error=error, previous_drive_stamp=old['steering']['stamp'],
                heading_deviation_since=new['heading_deviation_since'], raw_imu_tilt=new['raw_imu_tilt_rad']))
        oid, nid = old['steering']['trajectory_id'], st['trajectory_id']
        if oid != nid and oid in curves and nid in curves:
            c,s = math.cos(body),math.sin(body)
            rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])
            args=(np.array(new['pose']),rot)
            old_steer=follow_trajectory(*args,curves[oid],goal,
                tracking_pose=np.array(new['tracking_pose']),gate_translation=False,return_steering=True)[3]
            new_steer=follow_trajectory(*args,curves[nid],goal,
                tracking_pose=np.array(new['tracking_pose']),gate_translation=False,return_steering=True)[3]
            changes.append(dict(stamp=st['stamp'],previous_trajectory_id=oid,trajectory_id=nid,
                same_pose_heading_change=wrap(new_steer['heading']-old_steer['heading']),
                old_heading=old_steer['heading'],new_heading=new_steer['heading'],
                status_heading=new['steering']['heading'],
                reproduced_status_heading_difference=wrap(new_steer['heading']-st['heading'])))
    running_times = [s['steering']['stamp'] for s in dedup]
    start,end=min(running_times),max(running_times)
    start_truth=truth[int(np.argmin(abs(stamps-start)))]
    end_truth=truth[int(np.argmin(abs(stamps-end)))]
    pose_distance=[(float(np.linalg.norm(np.array(p['p'])-goal)),p['stamp'])
                   for p in result['poses'] if start<=p['stamp']<=end]
    return dict(scope='Read-only actual archived navigation/payload audit; GT yaw/position only independent evaluation.',
        production_source_sha256=result['source_sha256'], passed=result['passed'], failure=result['failure'],
        goal=goal.tolist(), unchanged_limits=result['unchanged_limits'],
        active_interval=[start,end], actual_world_displacement=(np.array(end_truth['p'])-start_truth['p']).tolist(),
        actual_world_start=start_truth['p'],actual_world_end=end_truth['p'],
        min_raw_goal_distance=min(pose_distance), raw_imu_max=result['active_max_imu_tilt'],
        active_bridge_holds=result['active_bridge_holds'], trajectory_association_rejected=max(s['trajectory_association_rejected'] for s in dedup),
        curve_reports=curve_reports, heading_realignment_triggers=triggers, same_pose_curve_changes=changes,
        max_gt_yaw_difference_at_trigger=max(abs(r['independent_yaw_difference']) for r in triggers),
        caveat='Status snapshots bound gate onset, not every controller tick. Same-pose curve deltas use exact accepted payload at the first observed status, not claim exact publish-time pose.')


def main():
    root=NAV.parent/'simulation/test_results'
    results={name:audit(root/name) for name in ['slope_gait_a_007','slope_gait_b_004']}
    results['conclusion'] = ('Both original 90s/.22m NAV probes failed. No huge east-to-west goal curve or terminal 180deg tangent reversal is present. '
        'Repeated .07 turns occur even with the same eastward curve; independent body yaw confirms real heading errors. '
        '.04 gets closer but still retreats after real heading errors and a late new-curve direction change. '
        'These component outcomes do not justify adopting .04 or raising yaw cap/gain.')
    output=NAV/'test_results/review_slope_gait_pair_navigation.json'
    output.write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    print(output)
    for k,r in results.items():
        if not isinstance(r,dict):continue
        print(k, json.dumps({key:r[key] for key in ['actual_world_displacement','min_raw_goal_distance','max_gt_yaw_difference_at_trigger','trajectory_association_rejected']},ensure_ascii=False))
        for c in r['curve_reports']:
            print('curve',c['trajectory_id'],'length',round(c['xy_arc_length'],4),'chord',round(c['xy_chord_length'],4),'cross',round(c['max_chord_cross_track'],4),'back',round(c['backtracking_along_chord'],4),'payloaddiff',c['sampled_payload_to_actual_path_max_difference'])
        print('triggers',[(round(t['stamp'],3),t['trajectory_id'],round(t['current_error'],4),round(t['path_tangent'],4),round(t['cross_track_angle'],4),round(t['measured_body_yaw'],4)) for t in r['heading_realignment_triggers']])


if __name__ == '__main__':
    main()
