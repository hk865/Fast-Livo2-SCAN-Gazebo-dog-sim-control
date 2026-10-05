#!/usr/bin/env python3
"""Audit the real upper-deck boundary defect against the recorded 42-run corpus."""
import importlib.util, hashlib, json, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'policy'))
import observation as current

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    old_path=ROOT/'test_results/raycast_boundary_v1/observation.py'
    spec=importlib.util.spec_from_file_location('old_boundary_observation',old_path)
    old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
    rows=[];maximum=archive_max=0.;frames=0
    for run in sorted((ROOT/'runs').glob('20261003_*')):
        if not run.is_dir()or run.name<'20261003_193541':continue
        asset=json.loads((run/'asset_manifest.json').read_text())if(run/'asset_manifest.json').exists()else{}
        if not asset or asset.get('sensors')or asset.get('camera_only')or asset.get('spawn_override'):continue
        if not(run/'telemetry.jsonl').exists()or not(run/'observations_actions.npz').exists():continue
        telemetry=[json.loads(x)for x in(run/'telemetry.jsonl').read_text().splitlines()]
        archive=np.load(run/'observations_actions.npz')['observations']
        old_terrain=old.TerrainHeightMap.from_sdf(run/'world.sdf');new_terrain=current.TerrainHeightMap.from_sdf(run/'world.sdf')
        last_action=np.zeros(12);peak=original_peak=0.
        for index,row in enumerate(telemetry):
            # Earlier formal rows predate world_sim_time logging. State time
            # is not consumed by build_observation; no timing is reconstructed.
            state=np.zeros(64);state[0]=row.get('world_sim_time',row['sim_time']);state[1:4]=row['position'];state[4:8]=row['quaternion_wxyz']
            state[8:11]=row['body_lin_vel'];state[11:14]=row['body_ang_vel'];state[14:26]=row['q'];state[26:38]=row['qd'];state[38:50]=row['applied_torque']
            before,_=old.build_observation(state,row['command'],last_action,old_terrain)
            after,_=current.build_observation(state,row['command'],last_action,new_terrain)
            peak=max(peak,float(np.max(abs(before-after))))
            original_peak=max(original_peak,float(np.max(abs(before-archive[index]))))
            last_action=np.zeros(12)if row['sim_time']<.1 else np.asarray(row['action'])
        frames+=len(telemetry);maximum=max(maximum,peak);archive_max=max(archive_max,original_peak)
        rows.append({'run':run.name,'frames':len(telemetry),'old_vs_corrected247_max_error':peak,'old_reconstruction_vs_recorded247_max_error':original_peak,
                     'telemetry_sha256':digest(run/'telemetry.jsonl'),'observations_actions_sha256':digest(run/'observations_actions.npz')})
    terrain=current.TerrainHeightMap.from_sdf(ROOT/'runs/20261003_194022_stand_original_origin_r1_dc76/world.sdf')
    previous=old.TerrainHeightMap.from_sdf(ROOT/'runs/20261003_194022_stand_original_origin_r1_dc76/world.sdf')
    starts=np.array([[0,-3e-11,20.4],[0,3e-11,20.4]])
    old_z,old_names=previous.raycast(starts);new_z,new_names=terrain.raycast(starts)
    boundary_ok=abs(new_z[0])<1e-8 and abs(new_z[1]-2.4)<1e-8 and abs(old_z[0]-2.4)<1e-8
    result={'status':'passed'if len(rows)==42 and maximum==0 and archive_max==0 and boundary_ok else'failed',
            'scope':'Offline recorded-state247 regression plus an actual-scene upper-deck edge counterexample; not new physical rollouts',
            'run_count':len(rows),'frame_count':frames,'all42_corrected247_vs_old_max_error':maximum,
            'old_reconstruction_vs_recorded247_max_error':archive_max,
            'boundary':{'starts':starts.tolist(),'previous_hit_z':old_z.tolist(),'corrected_hit_z':new_z.tolist(),
                        'previous_surfaces':old_names,'corrected_surfaces':new_names,'passed':bool(boundary_ok)},
            'sources':{'previous':digest(old_path),'corrected':digest(ROOT/'policy/observation.py'),'audit':digest(Path(__file__))},'runs':rows}
    path=ROOT/'test_results/raycast_boundary_v1/regression.json';path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items()if k!='runs'},indent=2))
    return 0 if result['status']=='passed'else 1
if __name__=='__main__':raise SystemExit(main())
