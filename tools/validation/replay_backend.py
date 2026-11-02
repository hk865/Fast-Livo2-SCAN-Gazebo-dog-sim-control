"""Replay persisted SLAM keyframes; read GT only after all correction decisions."""
from pathlib import Path
import argparse,collections,json,time,sys
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from localization.backend.external_map import ExternalMap
from localization.backend.history import History
from localization.backend.tracking import PriorTracker
from localization.backend.correction import TemporalAdmission
from localization.backend.geometry import inverse,pose_distance

def replay(run,map_path):
    setup=json.loads((run/'experiment_setup.json').read_text())
    initial=np.eye(4);initial[:3,3]=setup['navigation_world_origin']
    world_map=np.eye(4);world_map[:3,3]=[-.000002,-.000001,-.1]
    began=time.monotonic();asset=ExternalMap.from_pcd(map_path);matcher=PriorTracker(asset)
    load_seconds=time.monotonic()-began;admission=TemporalAdmission();C=initial.copy();out=[]
    h=History.load(next((run/'backend_history').glob('epoch_*')))
    for k in h.keyframes:
        prior=inverse(world_map)@C@k.T_odom_body@k.T_body_lidar
        match=matcher.localize(k.points_lidar,prior)
        proposal=admission.consider(k,match,k.stamp_ns,world_map)
        if proposal['accepted']:C=np.asarray(proposal['C_world_odom'])
        out.append(dict(sequence=k.sequence,stamp_ns=k.stamp_ns,accepted=proposal['accepted'],
                        reason=proposal['reason'],registration_reason=match.reason,
                        registration_ms=match.elapsed_s*1000,fitness=match.fitness,
                        rmse=match.rmse if np.isfinite(match.rmse)else None,
                        observability=match.observability,diagnostics=match.diagnostics,
                        T_raw_world_body=(initial@k.T_odom_body).tolist(),
                        T_corrected_world_body=(C@k.T_odom_body).tolist()))
    return dict(map_startup_s=load_seconds,decisions=out)

def score_only_after_replay(run,result):
    truth=[json.loads(l)for l in(run/'native_pose_source.jsonl').open()]
    stamps=np.asarray([r['stamp_ns']for r in truth]);xyz=np.asarray([r['position']for r in truth]);rotation=Slerp((stamps-stamps[0])/1e9,Rotation.from_quat([r['quaternion_wxyz'][1:]+r['quaternion_wxyz'][:1]for r in truth]))
    for r in result['decisions']:
        stamp=r['stamp_ns'];GT=np.eye(4);GT[:3,3]=[np.interp(stamp,stamps,xyz[:,i])for i in range(3)];GT[:3,:3]=rotation([(stamp-stamps[0])/1e9]).as_matrix()[0]
        r['raw_error_GT_m'],r['raw_orientation_GT_rad']=pose_distance(np.asarray(r['T_raw_world_body']),GT)
        r['corrected_error_GT_m'],r['corrected_orientation_GT_rad']=pose_distance(np.asarray(r['T_corrected_world_body']),GT)
    rows=result['decisions'];times=[r['registration_ms']for r in rows]
    result['summary']=dict(keyframes=len(rows),accepted=sum(r['accepted']for r in rows),reasons=dict(collections.Counter(r['reason']for r in rows)),registration_ms=dict(p50=float(np.percentile(times,50)),p95=float(np.percentile(times,95)),maximum=max(times)),raw_RMSE_GT_m=float(np.sqrt(np.mean([r['raw_error_GT_m']**2 for r in rows]))),corrected_RMSE_GT_m=float(np.sqrt(np.mean([r['corrected_error_GT_m']**2 for r in rows]))),raw_maximum_GT_m=max(r['raw_error_GT_m']for r in rows),corrected_maximum_GT_m=max(r['corrected_error_GT_m']for r in rows),fixed_frame_no_fitted_alignment=True,GT_read_after_all_localization_decisions=True)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=score_only_after_replay(a.run,replay(a.run,a.map));a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result['summary'],ensure_ascii=False,indent=2))
