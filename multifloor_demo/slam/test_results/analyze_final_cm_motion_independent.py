#!/usr/bin/env python3
"""Read archived CDR and text only; never initializes ROS or controls physics."""
import argparse
import bisect
import collections
from decimal import Decimal
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re

import numpy as np
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation, Slerp


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        while data:=f.read(4*1024*1024):h.update(data)
    return h.hexdigest()


def tilt(q):
    x,y,z,w=q
    return max(abs(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),
               abs(math.asin(max(-1.,min(1.,2*(w*y-z*x))))))


def nearest_truth(t,truth,stamps):
    i=bisect.bisect_left(stamps,t)
    if i<len(stamps) and stamps[i]==t:return np.array(truth[i]['p']),np.array(truth[i]['q'])
    if not 0<i<len(stamps) or stamps[i]-stamps[i-1]>150_000_000:return None
    alpha=(t-stamps[i-1])/(stamps[i]-stamps[i-1])
    p=(1-alpha)*np.array(truth[i-1]['p'])+alpha*np.array(truth[i]['p'])
    q=Slerp([0.,1.],Rotation.from_quat([truth[i-1]['q'],truth[i]['q']]))([alpha]).as_quat()[0]
    return p,q


def analyze(run):
    mpath=run/'staging/actuator_observer.py'
    spec=importlib.util.spec_from_file_location('readonly_motion_cdr',mpath)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    types={k:t for k,_,t in module.TOPICS}
    poses=collections.defaultdict(list); imu=[];imu_tilt=[]; jtc=[]; counts=collections.Counter()
    for h,cdr in module.read_capture(run/'actuator/actuator_startup.cdrlog'):
        kind=h['kind'];counts[kind]+=1
        if not cdr:continue
        t=h['header_stamp_ns']
        if kind=='jtc':jtc.append(t)
        if kind not in ('slam','truth','imu'):continue
        msg=deserialize_message(cdr,types[kind])
        if kind=='imu':
            q=msg.orientation;imu.append(t);imu_tilt.append(tilt([q.x,q.y,q.z,q.w]));continue
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        poses[kind].append(dict(t=t,p=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w]))
    motion=json.loads((run/'motion_result.json').read_text())
    active_start_ns=(int(round(motion['origin_stamp']*1e9)) if motion.get('origin_stamp') is not None
                     else min((x['t'] for x in poses['slam']),default=0))
    active_end_ns=(int(round(motion['complete_sim']*1e9)) if motion.get('complete_sim') is not None
                   else max((x['t'] for x in poses['slam']),default=0))
    truth=sorted(poses['truth'],key=lambda x:x['t']);stamps=[x['t'] for x in truth]
    active=[x for x in poses['slam'] if active_start_ns<=x['t']<=active_end_ns]
    transform=None;errors=[];unpaired=[]
    for p in active:
        match=nearest_truth(p['t'],truth,stamps)
        if match is None:unpaired.append(p['t']);continue
        gp,gq=match;sp=np.array(p['p'])
        if transform is None:
            rot=Rotation.from_quat(gq).as_matrix()@Rotation.from_quat(p['q']).as_matrix().T
            off=gp-rot@sp
            transform=dict(stamp_ns=p['t'],rotation_world_from_slam=rot.tolist(),translation=off.tolist())
        errors.append(float(np.linalg.norm(rot@sp+off-gp)))
    precision=dict(active_domain_ns=[active_start_ns,active_end_ns],single_initial_SE3=transform,
        fits=int(transform is not None),samples=len(errors),unpaired_truth_stamps_ns=unpaired,
        rmse_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,max_m=max(errors,default=None))
    raw_missing=[(a,b) for a,b in zip(imu,imu[1:]) if b-a!=1_000_000]
    jtc_deltas=collections.Counter(b-a for a,b in zip(jtc,jtc[1:]))
    internal=[];nul=0
    with (run/'fastlivo_debug/imu.txt').open('rb') as f:
        for line in f:
            nul+=line.count(b'\0')
            if line.strip():internal.append(int(Decimal(line.split()[0].decode())*1_000_000_000))
    logs=(run/'stack.log').read_text();sync=[];slices=[]
    for line in logs.splitlines():
        if '[DEMO_SYNC]' in line:
            z=dict(re.findall(r'(camera|lidar_newest|imu_newest|imu_last_used|imu_count|complete)=(-?[\d.]+)',line))
            if z:sync.append(z)
        elif '[DEMO_LIDAR_SLICE]' in line:
            z=dict(re.findall(r'(camera|previous|points|offset_min_ms|offset_max_ms|pending)=(-?[\d.]+)',line))
            if z:slices.append(z)
    first_camera_ns=int(Decimal(sync[0]['camera'])*1_000_000_000) if sync else None
    # The native IMU file explicitly logs (head stamp - first lidar time), not absolute stamps.
    internal_abs=[t+first_camera_ns for t in internal] if first_camera_ns is not None else []
    normalized=[int(round(t/1_000_000))*1_000_000 for t in internal_abs]
    internal_counts=collections.Counter(normalized)
    internal_missing=[]
    if normalized:
        internal_missing=[x for x in range(min(normalized),max(normalized)+1,1_000_000) if x not in internal_counts]
    cdr_imu_set=set(imu)
    unmatched=[x for x in internal_counts if x not in cdr_imu_set]
    official_imu=set()
    with (run/'sensor_audit.jsonl').open() as f:
        for line in f:
            row=json.loads(line)
            if row.get('source')=='imu':official_imu.add(int(round(row['stamp']*1e9)))
    official_missing_internal=[x for x in internal_counts if x not in official_imu]
    observed_used_missing=[x for x in cdr_imu_set if normalized and min(normalized)<=x<=max(normalized) and x not in internal_counts]
    sync_active=[x for x in sync if int(Decimal(x['camera'])*1_000_000_000)>=active_start_ns]
    sync_bad=[x for x in sync_active if x['complete']!='1' or int(Decimal(x['imu_last_used'])*1_000_000_000)<int(Decimal(x['camera'])*1_000_000_000)-2]
    slice_bad=[x for x in slices if x['pending']!='0' or abs(float(x['offset_min_ms'])-float(x['offset_max_ms']))>1e-6
               or abs(float(x['offset_min_ms'])-(float(x['camera'])-float(x['previous']))*1000)>2e-5]
    meta=json.loads((run/'map_metadata.json').read_text());binary=run/meta['binary_filename']
    dtype=np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,)),('pad','u1')])
    cloud=np.fromfile(binary,dtype=dtype)
    packed=cloud['rgb'][:,0].astype(np.uint32)<<16|cloud['rgb'][:,1].astype(np.uint32)<<8|cloud['rgb'][:,2]
    actualmaps=json.loads((run/'actual_gz_process.json').read_text());mapped_cm=[x for p in actualmaps for x in p['libraries'] if Path(x['path']).name=='libcontroller_manager.so']
    cleanup=json.loads((run/'process_cleanup.json').read_text());observer=json.loads((run/'actuator/observer_result.json').read_text())
    child_errors=[line for line in logs.splitlines() if 'process has died' in line or 'exit code 1' in line]
    result=dict(run=str(run),physics_result=motion['passed'],physics_checks=motion['checks'],independent_pose_precision=precision,
        raw_CDR_1000Hz=dict(samples=len(imu),unique=len(set(imu)),first_ns=min(imu,default=None),last_ns=max(imu,default=None),non_1ms_intervals=raw_missing,
            active_max_tilt=max((v for t,v in zip(imu,imu_tilt) if active_start_ns<=t<=active_end_ns),default=None)),
        actual_JTC_state_header_spacing_ns=dict(jtc_deltas),
        FAST_integrator_head_samples=dict(rows=len(internal),unique=len(internal_counts),duplicate_boundary_rows=len(internal)-len(internal_counts),null_bytes=nul,
            first_absolute_ns=min(normalized,default=None),last_absolute_ns=max(normalized,default=None),first_lidar_time_from_first_camera_ns=first_camera_ns,
            max_logged_conversion_quantization_ns=max((abs(t-u) for t,u in zip(internal_abs,normalized)),default=None),
            missing_grid_stamps_ns=internal_missing,CDR_raw_observed_but_not_integrated_in_covered_span=sorted(observed_used_missing),
            integrator_stamps_without_CDR_raw=sorted(unmatched),integrator_stamps_without_official_raw=sorted(official_missing_internal),
            scope='Native log writes actual consecutive IMU head midpoint input; adds only source-defined first lidar timestamp for time comparison. Nearest 1ms comparison is explicit, at most 2ns conversion quantization, not state/GT correction.'),
        generic_lidar_slice=dict(rows=len(slices),violations=slice_bad),active_sync=dict(rows=len(sync_active),incomplete_or_last_used_before_camera_gt2ns=sync_bad),
        real_RGBmap=dict(actual_count=len(cloud),metadata_count=meta['point_count'],unique_RGB_triplets=len(np.unique(packed)),finite_xyz=bool(np.isfinite(cloud['xyz']).all()),
            capacity_rejections=meta['capacity_rejections'],point_count_matches=len(cloud)==meta['point_count']==meta['rgb_points'],save=meta['save'],slam_healthy=meta['slam_healthy'],camera_healthy=meta['camera_healthy'],binary_sha256=sha(binary)),
        actual_mapped_CM=mapped_cm,cleanup=cleanup,observer=observer,child_shutdown_errors=child_errors,
        input_sha256={n:sha(run/n) for n in ['motion_result.json','process_cleanup.json','map_metadata.json','actuator/actuator_startup.cdrlog','fastlivo_debug/imu.txt']},
        limits=['All processing offline; no ROS context/nodes or physics.','Only one actual first SE3 per run; GT evaluation only.','JTC header spacing is observed state publication, not native ControllerInterface period or actuator-applied torque.','First .2m forward fourturn is a different component from original .22 NAV and .30 independent ordered waypoint contract.','Official float sensor_audit times are run-relative below 122s; CDR carries original integer header stamps.','No PCD map save was requested by these components and no full46/browser pass is claimed.','Ownedclean and launch exit0 do not imply every child exit0; actual child shutdown errors are listed.'])
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('out',type=Path);a=p.parse_args()
    result=analyze(a.run.resolve());a.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ['run','physics_result','independent_pose_precision','raw_CDR_1000Hz','actual_JTC_state_header_spacing_ns','FAST_integrator_head_samples','generic_lidar_slice','active_sync','real_RGBmap','child_shutdown_errors']},indent=2))
