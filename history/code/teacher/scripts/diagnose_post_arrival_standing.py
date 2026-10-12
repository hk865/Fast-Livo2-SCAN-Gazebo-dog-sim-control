#!/usr/bin/env python3
"""Record long zero-command standing after the frozen final-stop window; no new gate."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1048576),b''):
            digest.update(chunk)
    return digest.hexdigest()


def evaluate(run,write=True):
    run=Path(run).resolve()
    output=run/'post_arrival_long_standing_diagnostic.json'
    if write and output.exists():
        raise ValueError('Refusing to overwrite an earlier standing diagnostic')
    original=(run/'summary_dynamic_obstacle_independent.json').read_bytes()
    summary=json.loads(original)
    start=summary['checks']['original_raw_slam_two_regions_and_final_stop']['final_stop']['world_time_window_s'][1]
    with np.load(run/'dynamic_obstacle_plot_arrays.npz') as arrays:
        mask=arrays['native_physics_world_time']>=start
        time=arrays['native_physics_world_time'][mask]
        speed=arrays['body_com_planar_speed'][mask]
        wz=arrays['body_wz'][mask]
        policy=arrays['policy_world_time']>=start
        actor_zero=bool(np.max(abs(arrays['actor_command'][policy]))<1e-8)
        requested_zero=bool(np.max(abs(arrays['requested'][policy]))<1e-8)
    positions=[];body_contact=0;fault=0;quaternions=[]
    with (run/'actuator.jsonl').open() as stream:
        for line in stream:
            record=json.loads(line)
            if record.get('kind')=='physics_step' and record['t']-.005>=start:
                positions.append(record['position'])
                quaternions.append(record['quaternion_wxyz'])
                body_contact+=int(record['contacts'][0]>0)
                fault+=int(record['fault']!=0)
    positions=np.asarray(positions)
    q=np.asarray(quaternions)
    heading=np.unwrap(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2)))
    result={'schema':1,'scope':'Post-arrival long zero-command standing diagnostic; does not replace frozen3s final-stop window',
        'status':'diagnostic_only','world_window_s':[float(start),float(time[-1])],
        'actual_actor_velocity_input_all_zero':actor_zero,'requested_velocity_all_zero':requested_zero,
        'max_planar_speed_mps':float(speed.max()),'planar_peak_world_s':float(time[np.argmax(speed)]),
        'max_abs_wz_radps':float(abs(wz).max()),
        'max_xy_departure_from_window_start_m':float(np.linalg.norm(positions[:,:2]-positions[0,:2],axis=1).max()),
        'max_yaw_departure_from_window_start_rad':float(abs(heading-heading[0]).max()),
        'body_contact_samples':body_contact,'native_fault_samples':fault,'frozen_final_stop_preserved':True,
        'original_receipt_bytes_preserved':(run/'summary_dynamic_obstacle_independent.json').read_bytes()==original,
        'input_sha256':{name:sha(run/name)for name in ['summary_dynamic_obstacle_independent.json','actuator.jsonl','dynamic_obstacle_plot_arrays.npz']},
        'analyzer_sha256':sha(__file__),
        'interpretation':'The frozen3s post-arrival final-stop test is unchanged. Later zero-command Teacher motion is '
            'diagnostic only. A single3s pass is not a claim of all240s stationary behavior or all command-dropout parking modes. '
            'No post-hoc standing threshold is added to the frozen dynamic verdict.'}
    if write:
        output.write_text(json.dumps(result,indent=2)+'\n')
        (run/'post_arrival_long_standing_diagnostic.executed.py').write_bytes(Path(__file__).read_bytes())
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--read-only',action='store_true')
    args=parser.parse_args();print(json.dumps(evaluate(args.run,not args.read_only)))
