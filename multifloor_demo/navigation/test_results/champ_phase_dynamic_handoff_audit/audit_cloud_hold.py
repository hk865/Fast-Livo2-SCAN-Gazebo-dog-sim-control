"""Owned-clean native event/cloud/held/checked-path evidence. No ROS or CDR."""
import hashlib,importlib.util,json,sys
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'navigation'))
from control_core import steering_obstacle_ahead

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def ns(pair):return pair[0]*1_000_000_000+pair[1]
def audit(run):
    clean=json.loads((run/'process_cleanup.json').read_text());assert clean['owned_group_clean']
    d=json.loads((run/'dynamic_result.json').read_text());req=d['region_evaluation']['regions'][0]['receipt']['request_id']
    poses={p['stamp_ns']:p for p in d['poses']}
    statuses=[r for r in d['statuses'] if r['status'].get('request_id')==req]
    events=[]
    for p in (run/'navigation_events').glob('*.json'):
        e=json.loads(p.read_text());path=(p.parent/e['cloud_file']).resolve();assert path.parent==p.parent.resolve()
        z=np.load(path,allow_pickle=False)
        actual=steering_obstacle_ahead(z['cloud'],z['pose'],z['checked_target'],z['steering_direction'],z['route'])
        cloud=e['cloud_input'];index=e['edge_waypoint_index'];pose=poses[e['odom_stamp_ns']]
        checks=dict(committed_npz_sha=sha(path)==e['cloud_sha256'],finite_exact_Nx3=z['cloud'].shape==(cloud['filtered_points'],3) and np.isfinite(z['cloud']).all(),camera_init=cloud['frame_id']=='camera_init',filter_same_odom=cloud['filtering_body_stamp_ns']==e['odom_stamp_ns'],raw_same_native_pose=np.array_equal(z['pose'],pose['p']),filter_same_position=np.array_equal(z['pose'],cloud['filtering_body_pose']),filter_same_rotation=np.array_equal(z['rotation'],cloud['filtering_body_rotation']),current_goal=np.allclose(z['goal'],d['goals_definitions'][index]['center'],atol=1e-8,rtol=0),request_and_index=e['edge_request_id']==e['request_id']==req and e['waypoint_index']==index,guard_exact=actual[0]==e['guard_result'][0] and actual[2]==e['guard_result'][2] and abs(actual[1]-e['guard_result'][1])<=1e-12,zero_before_archive=e['zero_command']==[0.,0.,0.] and e['zero_command_stamp']==e['obstacle_edge_stamp'] and not e['protect_result'] and not e['tilt_hold'],native_cloud_pair_gap=abs(cloud['message_stamp_ns']-cloud['filtering_body_stamp_ns'])<=150000000,cloud_native_age=0<=round(e['guard_compute_stamp']*1e9)-cloud['message_stamp_ns']<=1000000000)
        events.append({'json_file':str(p),'json_sha256':sha(p),'npz_file':str(path),'npz_sha256':sha(path),'event':e,'reproduced_guard':[bool(actual[0]),actual[1],actual[2]],'checks':{k:bool(v) for k,v in checks.items()}})
    spec=importlib.util.spec_from_file_location('frozen_held',run/'staging/held_command_contract.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    meta=[json.loads(l) for l in (run/'trajectory_payloads.jsonl').open()]
    held=mod.audit_held_commands([dict(event=e['event']) for e in events],d['statuses'],meta,d['command_evidence'],req,d['goals'])
    assert held==d['held_command_timing_audit']
    adapter=[]
    for ordinal,l in enumerate((run/'joint_stop_adapter.jsonl').open()):
        x=json.loads(l)
        if x['kind'] in ['actual_champ_command','zero_edge','native_zero_ack','return_complete']:
            x['native_file_line']=ordinal+1;adapter.append(x)
    trajectories=[json.loads(l) for l in (run/'feedback_navigation_trajectories.jsonl').open()]
    metadata=[x for x in trajectories if x['kind']=='scan_metadata']
    paths=[x for x in trajectories if x['kind']=='accepted_path' and x['points']]
    path_proof=[]
    for x in paths:
        possibilities=[]
        for y in metadata:
            if abs(y['receipt_sim_time']-x['receipt_sim_time'])>.03:continue
            v=y['metadata']['trajectory'];k=np.asarray(v['knots']);o=v['order'];pts=BSpline(k,v['pos_pts'],o)(np.linspace(k[o],k[-o-1],min(2000,max(2,int((k[-o-1]-k[o])/.08)+1))))
            if pts.shape==np.shape(x['points']):possibilities.append((float(np.max(np.abs(pts-np.array(x['points'])))),y))
        assert possibilities
        error,y=min(possibilities,key=lambda a:a[0]);path_proof.append(dict(receipt_sim_time=x['receipt_sim_time'],metadata_receipt_sim_time=y['receipt_sim_time'],traj_id=y['metadata']['trajectory']['traj_id'],reference_stamp=y['metadata']['reference_stamp'],body_goal=y['metadata']['body_goal'],points=len(x['points']),max_error=error))
    obstacle_chains=[]
    for iv in held['intervals']:
        edge=next(e['event'] for e in events if e['event']['event_id']==iv['event_id']);ref=iv['confirmed_same_goal_executed_reference'];md=next(x for x in metadata if x['metadata']['reference_stamp']==ref['reference_stamp'] and x['metadata']['trajectory']['traj_id']==ref['accepted_trajectory_id']);path=next(x for x in path_proof if x['reference_stamp']==ref['reference_stamp'] and x['traj_id']==ref['accepted_trajectory_id']);start=iv['start'];end=path['receipt_sim_time'];zero=[x for x in adapter if x['kind']=='actual_champ_command' and start<=x['sim']<end];prezero=next(x for x in reversed(adapter) if x['kind']=='zero_edge' and x['sim']<=start);ack=next(x for x in adapter if x['kind']=='native_zero_ack' and x['native_file_line']>prezero['native_file_line']);idle=next(x for x in adapter if x['kind']=='return_complete' and x['native_file_line']>ack['native_file_line']);first=next(x for x in adapter if x['kind']=='actual_champ_command' and x['sim']>=end and any(x['value']));status=next(x for x in statuses if x['status'].get('accepted_trajectory_id')==ref['accepted_trajectory_id'] and x['status'].get('obstacle_resumes')==1 and x['status'].get('steering',{}).get('trajectory_id')==ref['accepted_trajectory_id']);s=status['status'];native_tzero_since=next(x for x in adapter if x['kind']=='actual_champ_command' and x['native_file_line']<prezero['native_file_line'] and not any(x['value']) and x['sim']>=prezero['sim']-.01)
        checks=dict(real_current_goal_hold=iv['valid_native_zero_edge'] and md['metadata']['body_goal']==ref['body_goal'],requested_and_safe_held_all_zero=iv['passed'],native_emits_through_checked_path_all_zero=bool(zero) and all(not any(x['value']) for x in zero),actual_zero_ACK_idle_before_obstacle=ack['sim']<idle['sim']<=start,new_ref_after_hold=ref['reference_time']>start and edge['reference_stamp']!=ref['reference_stamp'],idle_ge_one_second_before_ref=ref['reference_time']-idle['sim']>=1.,fresh_MD_checked_path_before_motion=ref['reference_time']<=md['receipt_sim_time']<=path['receipt_sim_time']<=first['sim'],reference_and_goal_same=md['metadata']['reference_stamp']==ref['reference_stamp'] and np.allclose(md['metadata']['body_goal'],d['goals_definitions'][edge['edge_waypoint_index']]['center'],rtol=0,atol=1e-8),path_reconstructed=path['max_error']<=1e-12,original_aligned_resume_observed=s['aligned_obstacle_resumes']==1 and s['alignment_phase']=='drive' and abs(s['steering']['error'])<=.20,not_new_pre_turn_claim=True)
        obstacle_chains.append(dict(event_id=iv['event_id'],held_start_sim=start,observed_clear_current_reference_sim=ref['reference_time'],native_zero_emits_held_to_checked_path=len(zero),upstream_held_zero_samples=iv['exact_zero_samples'],prior_corridor_zero_edge=prezero,native_ACK=ack,native_idle=idle,continuous_idle_before_fresh_ref_s=ref['reference_time']-idle['sim'],fresh_metadata_sim=md['receipt_sim_time'],checked_path_sim=path['receipt_sim_time'],resume_first_actual_native_emit=first,original_aligned_resume_status={'sim':status['sim'],'steering_error':s['steering']['error'],'aligned_obstacle_resumes':s['aligned_obstacle_resumes']},checks={k:bool(v) for k,v in checks.items()},scope='Obstacle edge occurs during prior corridor stop; no new Adapter zero edge is invented. Old aligned_resume gate reuses >=1s stopped duration when fresh direction error<=.20, rather than imposing a new full pre_turn.'))
    checks={'owned_clean':clean['owned_group_clean'],'all_native_event_checks':bool(events) and all(all(e['checks'].values()) for e in events),'held_original_audit_reproduced_exact':held==d['held_command_timing_audit'] and held['passed'],'all_obstacle_resume_chains':bool(obstacle_chains) and all(all(e['checks'].values()) for e in obstacle_chains),'all_recorded_nonempty_paths_reconstructed':len(path_proof)==len(paths) and all(x['max_error']<=1e-12 for x in path_proof)}
    return dict(scope=__doc__,run=str(run),ready=all(checks.values()),checks=checks,native_events=events,held=held,obstacle_chains=obstacle_chains,path_association=path_proof,source_sha256={n:sha(run/n) for n in ['dynamic_result.json','joint_stop_adapter.jsonl','trajectory_payloads.jsonl','feedback_navigation_trajectories.jsonl','staging/held_command_contract.py','process_cleanup.json']},control_core_sha256=sha(ROOT/'navigation/control_core.py'),limits=['Guard uses exact archived self-filtered array and paired raw SLAM context, not a later recorder cloud.','Native emission sim fields are cached clocks, not message headers/actual receive timestamps.','No GT input to cloud/held/identity audit. Event-point physical-box association and two-window GT validation are independently owned by SLAM reviewer.','No independent SCAN collision-map rerun, no direct raw Bspline topic recapture, no JTC effort/applied force inference.','Original aligned resume can enter DRIVE immediately after fresh checked path when stopped>=1s and measured path heading within.20; do not claim an extra new 1s pre_turn here.'])
if __name__=='__main__':
    run=Path(sys.argv[1]).resolve();out=Path(sys.argv[2]).resolve();assert not out.exists();result=audit(run);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'ready':result['ready'],'checks':result['checks'],'obstacle_chains':result['obstacle_chains'],'output_sha256':sha(out)},indent=2))
