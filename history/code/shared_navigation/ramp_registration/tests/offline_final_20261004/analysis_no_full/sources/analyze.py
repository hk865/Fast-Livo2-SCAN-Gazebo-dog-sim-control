#!/usr/bin/env python3
"""Offline actual-cloud registration receipt; never sends a navigation request."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
sys.dont_write_bytecode=True
from capture import atomic_json
from cloud import rotation
from geometry import register
HERE=Path(__file__).resolve().parent


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def analyze_capture(directory, output):
    directory=Path(directory).resolve();output=Path(output).resolve()
    if output.exists():raise ValueError('new registration output required; never overwrite old receipts')
    manifest_path=directory/'capture_manifest.json';manifest=json.loads(manifest_path.read_text())
    if manifest.get('schema')!='actual_ramp_cloud_capture/v1':raise ValueError('actual full-cloud capture manifest required')
    for name,expected in manifest['source_freeze']['source_hashes'].items():
        if digest(directory/'sources'/name)!=expected:raise ValueError('archived capture source changed: '+name)
    for name,ref in manifest['source_freeze'].get('source_contract_refs',{}).items():
        if digest(directory/'sources/dependencies'/name)!=ref['sha256']:raise ValueError('archived source/API contract changed: '+name)
    for path,expected in manifest.get('producer_configuration',{}).get('source_hashes',{}).items():
        if digest(directory/'producer_configuration'/Path(path).name)!=expected:raise ValueError('archived producer configuration changed: '+path)
    if manifest.get('errors') or not manifest.get('writer_drained') or manifest['submitted']!=manifest['processed']:
        raise ValueError('capture errors or undrained evidence: refusing registration')
    arrays=[];source_files={str(manifest_path):digest(manifest_path)};accepted=[]
    for row in manifest['clouds']:
        path=directory/row['archive_file']
        if digest(path)!=row['archive_sha256']:raise ValueError('original cloud archive hash changed: '+str(path))
        if not row['geometry_admitted']:continue
        if row.get('source_topic')!='/cloud_registered_full' or row.get('ground_truth_used')is not False:raise ValueError('nonactual full-cloud provenance')
        with np.load(path,allow_pickle=False)as data:
            if hashlib.sha256(data['raw'].tobytes()).hexdigest()!=row['raw_payload_sha256']:raise ValueError('original PointCloud2 bytes changed')
            xyz=data['xyz'].copy()
            if hashlib.sha256(xyz.tobytes()).hexdigest()!=row['decoded_xyz_sha256']:raise ValueError('decoded XYZ changed')
        # Standard declared body-frame self-return exclusion only; never adds
        # support, obstacle, floor or ramp points. Raw XYZ remains in archives.
        body=row['body_pose'];local=(xyz-np.asarray(body['position']))@rotation(body['quaternion'])
        self_box=(np.abs(local[:,0])<.55)&(np.abs(local[:,1])<.30)&(local[:,2]>-.40)&(local[:,2]<.30)
        xyz=xyz[~self_box];arrays.append(xyz);source_files[str(path)]=row['archive_sha256']
        accepted.append({'original_stamp_ns':row['original_stamp_ns'],'received_monotonic_wall':row['received_monotonic_wall'],
            'archive_file':row['archive_file'],'body_stamp_ns':body['original_stamp_ns'],'removed_self_box_points':int(self_box.sum())})
    if not arrays or manifest.get('gravity')is None or manifest.get('body_anchor')is None:raise ValueError('actual up/anchor/admitted clouds unavailable')
    xyz=np.concatenate(arrays)
    # Deterministic observed 4cm voxels, retaining a real observation per voxel.
    # Quantization is only an offline workload bound, never a sensor producer.
    anchor=manifest['body_anchor'];scaled=((xyz-np.asarray(anchor['position']))@rotation(anchor['quaternion']))/.04
    nearest=np.rint(scaled);eps=16*np.finfo(float).eps*np.maximum(1.,np.abs(scaled))
    scaled=np.where(np.abs(scaled-nearest)<=eps,nearest,scaled)
    key=np.floor(scaled).astype(np.int64);_,indices=np.unique(key,axis=0,return_index=True);xyz=xyz[np.sort(indices)]
    result=register(xyz,manifest['gravity']['up_camera_init'],manifest['body_anchor']['position'],
        complete_archive=manifest['status']=='observation_complete')
    result.update(capture_manifest_sha256=digest(manifest_path),input_source_hashes=source_files,
        analyzed_clouds=accepted,observed_downsample_m=.04,observed_downsample_frame='initial actual SLAM body pose',retained_observed_points=len(xyz),
        analysis_source_hashes={str(HERE/n):digest(HERE/n)for n in ('analyze.py','geometry.py','cloud.py','capture.py')},
        registration_control_enabled=False,navigation_request_emitted=False,
        limitations=['Plane/edge candidates need independent review; geometry does not establish motor/solver/SLAM accuracy.',
                    'Full candidate covers observed ramp seams, not an entire multifloor mission.',
                    'Recorded actual accelerometer calibration is stationary; no simulator or absolute IMU yaw input.',
                    'No unobserved opposite edge, floor, endpoint or route segment is filled.'])
    output.mkdir();np.savez(output/'observed_xyz.npz',xyz=xyz)
    (output/'sources').mkdir()
    for name in ('analyze.py','geometry.py','cloud.py','capture.py'):
        (output/'sources'/name).write_bytes((HERE/name).read_bytes())
    result['observed_xyz_artifact']={'path':str(output/'observed_xyz.npz'),'sha256':digest(output/'observed_xyz.npz')}
    canonical=json.dumps(result,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    result['registration_content_sha256']=hashlib.sha256(canonical).hexdigest()
    atomic_json(output/'registration.json',result)
    atomic_json(output/'receipt_hash.json',{'path':str(output/'registration.json'),'sha256':digest(output/'registration.json'),
        'status':result['status'],'full_route_eligible':result['full_route_eligible'],'navigation_verified':False})
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=analyze_capture(a.capture,a.output)
    print(json.dumps({'status':r['status'],'full_route_eligible':r['full_route_eligible'],'navigation_verified':False}))


if __name__=='__main__':main()
