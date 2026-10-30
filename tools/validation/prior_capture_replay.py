"""Offline convergence basin of unchanged PriorTracker, not online initialization acceptance."""
from pathlib import Path
import sys,json,time,collections
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from localization.backend.external_map import ExternalMap
from localization.backend.history import History
from localization.backend.tracking import PriorTracker
from localization.backend.geometry import inverse,pose_distance
RUN=ROOT/'runs/slam_multi_floor_repeat_20261030';REVIEW=ROOT/'docs/backend_evidence'

def replay():
    history=History.load(next((RUN/'backend_history').glob('epoch_*')))
    with(RUN/'backend_results.jsonl').open()as stream:accepted={r['source_ns']:r for r in map(json.loads,stream)if r.get('accepted')}
    available=[k for k in history.keyframes if k.stamp_ns in accepted]
    selected=[min(available,key=lambda k:abs(k.stamp_ns/1e9-time_s))for time_s in [7,35,160,225,335]]
    asset=ExternalMap.from_pcd(ROOT/'assets/building/building2_9.pcd');matcher=PriorTracker(asset)
    world_map=np.eye(4);world_map[:3,3]=[-.000002,-.000001,-.1]
    cases=[('nominal',0.,0.,0.,0.)]
    for radius in [.1,.25,.4,.55,.75,1.]:
        for dx,dy in [(radius,0),(-radius,0),(0,radius),(0,-radius)]:cases.append((f'XY_{radius:.2f}m',dx,dy,0.,0.))
    for degrees in [5,10,15,25]:
        for sign in [-1,1]:cases.append((f'yaw_{degrees}deg',0.,0.,0.,np.deg2rad(sign*degrees)))
    for dz in [.25,.5,1.5]:
        for sign in [-1,1]:cases.append((f'Z_{dz:.2f}m',0.,0.,sign*dz,0.))
    for radius in [.25,.4]:
        for sign in [-1,1]:
            for degrees in [-15,-10,10,15]:cases.append((f'X_{radius:.2f}m_yaw_{abs(degrees)}deg',sign*radius,0.,0.,np.deg2rad(degrees)))
    rows=[];frames=[]
    for k in selected:
        body=np.asarray(accepted[k.stamp_ns]['T_world_body']);nominal=inverse(world_map)@body@k.T_body_lidar
        frames.append(dict(sequence=k.sequence,stamp_ns=k.stamp_ns,nominal_world_body=body.tolist(),nominal_source='Actual geometry/temporal accepted backend result; no GT pose used to construct priors'))
        for group,dx,dy,dz,yaw in cases:
            prior=nominal.copy();prior[:3,3]+=[dx,dy,dz];prior[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()@prior[:3,:3]
            estimate=matcher.localize(k.points_lidar,prior)
            matched=None if estimate.T_map_lidar is None else world_map@estimate.T_map_lidar@inverse(k.T_body_lidar)
            rows.append(dict(sequence=k.sequence,stamp_ns=k.stamp_ns,group=group,delta_xyz_m=[dx,dy,dz],delta_yaw_deg=float(np.rad2deg(yaw)),accepted=estimate.accepted,reason=estimate.reason,fitness=estimate.fitness,rmse=estimate.rmse if np.isfinite(estimate.rmse)else None,elapsed_ms=estimate.elapsed_s*1000,T_world_body=None if matched is None else matched.tolist(),diagnostics=estimate.diagnostics))
        print(json.dumps(dict(stage='frame_finished',sequence=k.sequence,decisions=len(rows))),flush=True)
    return dict(scope='Offline single-frame convergence of unchanged local matcher; no ROS queue, temporal multi-frame admission, motion or navigation acceptance',run=RUN.name,priors_centered_on_accepted_map_matches_not_GT=True,frames=frames,decisions=rows)

def score_after_all_decisions(result):
    # GT is opened only after every prior perturbation has been independently decided.
    with(RUN/'native_pose_source.jsonl').open()as stream:truth=[json.loads(line)for line in stream]
    ns=np.array([r['stamp_ns']for r in truth],dtype=np.int64);xyz=np.array([r['position']for r in truth]);slerp=Slerp((ns-ns[0])/1e9,Rotation.from_quat([r['quaternion_wxyz'][1:]+r['quaternion_wxyz'][:1]for r in truth]))
    groups=collections.defaultdict(list)
    for r in result['decisions']:
        r['accurate_by_offline_GT']=False
        if r['accepted']:
            T=np.asarray(r['T_world_body']);GT=np.eye(4);GT[:3,3]=[np.interp(r['stamp_ns'],ns,xyz[:,axis])for axis in range(3)];GT[:3,:3]=slerp([(r['stamp_ns']-ns[0])/1e9]).as_matrix()[0]
            r['GT_position_error_m'],r['GT_orientation_error_rad']=pose_distance(T,GT)
            r['accurate_by_offline_GT']=r['GT_position_error_m']<=.1 and r['GT_orientation_error_rad']<=.05
        groups[r['group']].append(r)
    summary=[]
    for group,rows in groups.items():
        accepted=[r for r in rows if r['accepted']]
        summary.append(dict(group=group,trials=len(rows),accepted=len(accepted),accurate=sum(r['accurate_by_offline_GT']for r in rows),accepted_inaccurate=sum(not r['accurate_by_offline_GT']for r in accepted),maximum_GT_position_error_m=max([r['GT_position_error_m']for r in accepted],default=None),maximum_GT_orientation_error_rad=max([r['GT_orientation_error_rad']for r in accepted],default=None),reasons=dict(collections.Counter(r['reason']for r in rows))))
    result.update(GT_read_only_after_all_decisions=True,offline_accuracy_criterion=dict(position_m=.1,rotation_rad=.05,note='Independent reporting criterion, never substituted for runtime admission'),summary=summary)
    return result
if __name__=='__main__':
    result=score_after_all_decisions(replay());(REVIEW/'prior_capture_replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result['summary'],indent=2))
