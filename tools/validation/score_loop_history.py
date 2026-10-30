"""Independent offline relative-pose score of actual persisted historical loop edges."""
from pathlib import Path
import argparse,json,sys
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from localization.backend.history import History
from localization.backend.geometry import inverse,pose_distance

def score(run):
    history=History.load(next((run/'backend_history').glob('epoch_*')))
    # All loop retrieval, geometric admission and optimization finished online
    # before this offline scorer ever opens ground truth.
    truth=[json.loads(line)for line in(run/'native_pose_source.jsonl').open()]
    times=np.asarray([r['stamp_ns']for r in truth],dtype=np.int64)
    xyz=np.asarray([r['position']for r in truth]);q=[r['quaternion_wxyz'][1:]+r['quaternion_wxyz'][:1]for r in truth]
    rotations=Slerp((times-times[0])/1e9,Rotation.from_quat(q))
    def reference(index):
        ns=history.keyframes[index].stamp_ns
        if not times[0]<=ns<=times[-1]:raise ValueError('GT does not bracket loop endpoint')
        T=np.eye(4);T[:3,3]=[np.interp(ns,times,xyz[:,a])for a in range(3)]
        T[:3,:3]=rotations([(ns-times[0])/1e9]).as_matrix()[0];return T
    rows=[]
    for e in history.edges:
        if e.kind!='loop':continue
        actual=inverse(reference(e.target))@reference(e.source)
        raw=inverse(history.keyframes[e.target].T_odom_body)@history.keyframes[e.source].T_odom_body
        fitted=inverse(history.poses[e.target])@history.poses[e.source]
        def errors(T):
            distance,angle=pose_distance(T,actual);return dict(translation_m=distance,rotation_rad=angle)
        rows.append(dict(target=e.target,source=e.source,quality=e.quality,geometric_measurement_error=errors(e.T_target_source),raw_relative_pose_error=errors(raw),optimized_relative_pose_error=errors(fitted)))
    graphs=[]
    for line in(run/'backend_results.jsonl').open():
        row=json.loads(line)
        if 'graph'in row:graphs.append(dict(source_sequence=row['sequence'],**row['graph']))
    result=dict(scope='Actual online historical cloud loops; offline relative-body-frame GT scoring, no fitted trajectory alignment or GT supplied to graph',history_revision=history.revision,verified_loop_edges=len(rows),graphs=graphs,loop_edges=rows)
    (run/'loop_history_score.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path);args=parser.parse_args();print(json.dumps(score(args.run),indent=2))
