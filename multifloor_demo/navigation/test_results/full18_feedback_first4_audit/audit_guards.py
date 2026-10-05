"""Read-only native raw-IMU gate and SCAN current-goal identity evidence."""
import hashlib,json,math,pathlib
import numpy as np
from scipy.spatial.transform import Rotation
HERE=pathlib.Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):return [json.loads(l) for l in p.open()]
reports={}
for suffix in ['a_disabled','b_enabled']:
    run=ROOT/'simulation/test_results'/('20261002_feedback_first4_'+suffix)
    data=json.loads((run/'first_four_result.json').read_text())
    origin,terminal=data['origin_stamp_ns'],data['terminal_stamp_ns']
    imu=[r for r in data['raw_imu'] if origin<=r['stamp_ns']<=terminal]
    cmds=[r for r in rows(run/'joint_stop_adapter.jsonl') if r['kind']=='actual_champ_command']
    metadata={};paths=[]
    for r in rows(run/'feedback_navigation_trajectories.jsonl'):
        if r['kind']=='scan_metadata':
            m=r['metadata'];metadata[(m['trajectory']['traj_id'],tuple(m['reference_stamp']))]=m
        elif r['kind']=='accepted_path':paths.append(r)
    states=[r for r in data['statuses'] if r['status'].get('request_id')==data['request_id']]
    identity=[];edges=[];last=None
    for row in states:
        n=row['status'];key=(n.get('alignment_phase'),n.get('tilt_hold'),n.get('state'))
        if key!=last:
            edges.append(dict(received_stamp_ns=row['received_stamp_ns'],phase=key[0],tilt_hold=key[1],state=key[2],
                locked_heading=n.get('locked_heading'),steering=n.get('steering'),message=n.get('message')));last=key
        tid=n.get('accepted_trajectory_id');ref=n.get('trajectory_reference_stamp');goal=n.get('current_goal')
        if n.get('state')=='running' and tid is not None and ref is not None:
            m=metadata.get((tid,tuple(ref)))
            valid=m is not None and goal is not None and max(abs(a-b) for a,b in zip(m['body_goal'],goal['center']))<1e-8
            identity.append(dict(received_stamp_ns=row['received_stamp_ns'],traj_id=tid,reference_stamp=ref,
                current_goal=goal['goal_id'] if goal else None,requested_body_goal_matches_current=valid))
    tilt_edges={}
    for t in [.2,.3,.5]:
        first=next((r for r in imu if r['tilt']>=t),None)
        if first:
            rpy=Rotation.from_quat(first['quaternion']).as_euler('xyz').tolist()
            tilt_edges[str(t)]=dict(stamp_ns=first['stamp_ns'],tilt=first['tilt'],roll_pitch_yaw=rpy,
                angular_velocity=first['angular_velocity'],wall_monotonic=first['wall'])
        else:tilt_edges[str(t)]=None
    hold=tilt_edges['0.3'];post=[r for r in cmds if hold and round(r['sim']*1e9)>=hold['stamp_ns']]
    last=states[-1]['status']
    inputs=['first_four_result.json','joint_stop_adapter.jsonl','feedback_navigation_trajectories.jsonl','process_cleanup.json','first4_manifest.json']
    reports[suffix]=dict(original_passed=data['passed'],original_failure=data['failure'],
      actual_fourth_segment_entered=data['fourth_segment_previous_receipt_stamp_ns'] is not None,
      original_receipts=[r.get('receipt') for r in data['region_evaluation']['regions'] if r.get('receipt')],
      metadata_count=len(metadata),accepted_path_count=len(paths),actual_status_identity_checks=len(identity),
      all_current_goal_reference_matches=bool(identity) and all(r['requested_body_goal_matches_current'] for r in identity),
      identity=identity,phase_edges=edges,raw_tilt_threshold_onsets=tilt_edges,
      all_actual_emitted_commands_zero_after_raw_hold=bool(post) and all(not any(r['value']) for r in post) if hold else None,
      emitted_rows_after_hold=len(post),first_command_after_hold=post[0] if post else None,
      max_active_raw_tilt=data['active_max_imu_tilt'],active_bridge_holds=data['active_bridge_holds'],
      final_native_nav_counts={k:last.get(k) for k in ['trajectory_association_rejected','degenerate_splines','obstacle_stops','replans','reference_requests']},
      accepted_paths=[dict(stamp=p['header_stamp'],received=p['receipt_sim_time'],points=len(p['points']),
          first=p['points'][0] if p['points'] else None,last=p['points'][-1] if p['points'] else None) for p in paths],
      input_sha256={f:sha(run/f) for f in inputs},cleanup=json.loads((run/'process_cleanup.json').read_text()))
result=dict(scope='read-only original native timeline; no new source, node, command or GT steering',groups=reports,
    limitations=['Status is lower-rate cached diagnostic; use its embedded steering and received native stamps, not recomputed targets.',
      'Adapter log observes actual emitted commands, not CHAMP native receipt or JTC internal actuator output.',
      'B never entered fourth region; fourth-segment CDR coverage must be not-applicable/not-observed, never passed.',
      'Measured initial yaw/calibration and motion histories differ; one A/B realization does not establish a universal feedback sign law.'],
    script_sha256=sha(pathlib.Path(__file__)))
(HERE/'native_guards.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(output=str(HERE/'native_guards.json'),sha256=sha(HERE/'native_guards.json'),
  identity={k:v['all_current_goal_reference_matches'] for k,v in reports.items()},counts={k:v['actual_status_identity_checks'] for k,v in reports.items()})))
