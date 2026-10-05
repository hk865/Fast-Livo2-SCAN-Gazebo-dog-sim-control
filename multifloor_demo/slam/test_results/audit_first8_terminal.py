#!/usr/bin/env python3
"""Independent offline actual first8 audit; no ROS import or source mutation."""
import bisect
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation,Slerp

ROOT=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def match(t,times,positions):
    i=int(np.searchsorted(times,t))
    if i<len(times) and abs(times[i]-t)<1e-9:return positions[i].copy(),i,i
    if i==0 or i==len(times):return None
    if times[i]-times[i-1]<=0 or times[i]-times[i-1]>.15:return None
    lo,hi=i-1,i;alpha=(t-times[lo])/(times[hi]-times[lo])
    return positions[lo]+alpha*(positions[hi]-positions[lo]),lo,hi

def audit(p):
    r=read(p/'first_eight_result.json');scenario=read(p/'scenario.json')
    at=r['origin_stamp'];end=r['independent_truth_evaluation_interval'][1]
    poses=r['poses'];truth=r['truth'];times=np.array([x['stamp'] for x in truth]);tp=np.array([x['p'] for x in truth]);tq=np.array([x['q'] for x in truth])
    initial=match(at,times,tp)
    if initial is None:raise ValueError('No bounded actual initial GT pairing')
    actual,lo,hi=initial
    gtR=Rotation.from_quat(tq[lo]).as_matrix() if lo==hi else Slerp(times[[lo,hi]],Rotation.from_quat(tq[[lo,hi]]))([at]).as_matrix()[0]
    first=next(x for x in poses if x['stamp']==at)
    R=gtR@Rotation.from_quat(first['q']).as_matrix().T;translation=actual-R@first['p']
    offsets=np.array(scenario['exploration'][:8]);worldgoals=actual+offsets;goals=np.array(r['goals'])
    active=[x for x in poses if at<=x['stamp']<=end];precision=[];holes=[]
    for row in active:
        matched=match(row['stamp'],times,tp)
        if matched is None:holes.append(row['stamp']);continue
        precision.append(float(np.linalg.norm(R@row['p']+translation-matched[0])))
    statuses=r['statuses'];ids={x['status'].get('request_id') for x in statuses if x['status'].get('total')==8}
    if len(ids)!=1:raise ValueError('Actual request ID not uniquely bound to eight goals')
    request=next(iter(ids));nav=[x for x in statuses if x['status'].get('request_id')==request];nt=[x['sim'] for x in nav]
    transitions=[];previous=-1
    for row in nav:
        idx=row['status']['waypoint_index']
        if idx!=previous:transitions.append(dict(sim=row['sim'],index=idx,state=row['status']['state']));previous=idx
    sequential=[x['index'] for x in transitions]==list(range(9))
    joint=[];earliest=at
    for i,goal in enumerate(goals):
        confirmations=[x for x in nav if x['status']['waypoint_index']>i]
        stop=confirmations[0]['sim'] if confirmations else float('inf');evidence=None
        for row in active:
            t=row['stamp']
            if t<earliest or t>stop:continue
            si=bisect.bisect_right(nt,t)-1
            if si<0 or nav[si]['status']['waypoint_index']!=i:continue
            matched=match(t,times,tp)
            if matched is None:continue
            gt=matched[0];slam_error=float(np.linalg.norm(np.array(row['p'])-goal));truth_error=float(np.linalg.norm(gt-worldgoals[i]));rigid_error=float(np.linalg.norm(gt-(R@goal+translation)))
            if max(slam_error,truth_error,rigid_error)<=.30:
                evidence=dict(stamp=t,SLAM_error_m=slam_error,GT_error_m=truth_error,fixed_SE3_target_error_m=rigid_error,actual_observed_NAV_index=i,NAV_status_sim=nav[si]['sim']);break
        joint.append(dict(goal_index=i,goal=goal.tolist(),first_joint_evidence=evidence,actual_advance_sim=None if not confirmations else stop))
        if confirmations:earliest=stop
    active_raw=[x for x in r['raw_imu'] if at<=x['stamp']<=end];quat=np.array([x['quaternion'] for x in active_raw]);reference=scenario['sensors']['imu']['orientation_reference']
    rawR=Rotation.from_quat(reference['world_quaternion']).as_matrix()@Rotation.from_quat(quat).as_matrix()@Rotation.from_quat(reference['body_imu_quaternion']).as_matrix().T
    tilt=np.arccos(np.clip(rawR[:,2,2],-1,1));active_max=float(tilt.max())
    event_health=[x for x in r['aggregate_execution_safety_events'] if at<=x['sim']<=end];holds=max((x['safety']['holds'] for x in event_health),default=0)-min((x['safety']['holds'] for x in event_health),default=0)
    log=(p/'stack.log').read_text(errors='replace')
    sync=[dict(camera=float(m[1]),lidar=float(m[2]),newest=float(m[3]),used=float(m[4]),count=int(m[5]),complete=int(m[6])) for m in re.finditer(r'DEMO_SYNC\] camera=([\d.]+) lidar_newest=([\d.]+) imu_newest=([\d.]+) imu_last_used=([\d.]+) imu_count=(\d+) complete=(\d+)',log)]
    slices=[dict(camera=float(m[1]),previous=float(m[2]),minimum_ms=float(m[3]),maximum_ms=float(m[4]),pending=int(m[5])) for m in re.finditer(r'DEMO_LIDAR_SLICE\] camera=([\d.]+) previous=([\d.]+) points=\d+ offset_min_ms=([\d.]+) offset_max_ms=([\d.]+) pending=(\d+)',log)]
    official=[];parse_errors=0
    for line in (p/'sensor_audit.jsonl').open():
        try:x=json.loads(line)
        except json.JSONDecodeError:parse_errors+=1;continue
        if x['source']=='imu':official.append([round(x['stamp']*1e6),*x['angular_velocity'],*x['acceleration']])
    official=np.array(official);keys=official[:,0].astype('int64');v=official[:,1:];native=np.loadtxt(p/'fastlivo_debug/imu.txt');mat=np.loadtxt(p/'fastlivo_debug/mat_out.txt')
    firstpose=poses[0];mi=int(np.argmin(np.linalg.norm(mat[:,4:7]-firstpose['p'],axis=1)));offset=firstpose['stamp']-mat[mi,0]
    nk=np.rint((native[:,0]+offset)*1e6).astype('int64');tk=np.r_[nk[1:],round(sync[-1]['camera']*1e6)]
    hi=np.searchsorted(keys,nk);ti=np.searchsorted(keys,tk);ix=np.flatnonzero((hi<len(keys))&(ti<len(keys)));ix=ix[(keys[hi[ix]]==nk[ix])&(keys[ti[ix]]==tk[ix])]
    mean_difference=np.max(abs(native[ix,1:]-(v[hi[ix]]+v[ti[ix]])*.5),axis=1)
    missing=sorted(set(keys[(keys>=nk[0])&(keys<tk[-1])])-set(nk))
    officialgaps=[dict(a=float(keys[i]/1e6),b=float(keys[i+1]/1e6),delta_us=int(keys[i+1]-keys[i])) for i in np.flatnonzero(np.diff(keys)!=1000)]
    nativegaps=[dict(a=float(nk[i]/1e6),b=float(tk[i]/1e6),delta_us=int(tk[i]-nk[i])) for i in np.flatnonzero(tk-nk!=1000)]
    discrepancies=[]
    for a,b in zip(sync,sync[1:]):
        if b['camera']+2e-9<firstpose['stamp']:continue
        n=np.searchsorted(keys,round(b['camera']*1e6),side='right')-np.searchsorted(keys,round(a['camera']*1e6),side='right')
        if b['count']!=n:discrepancies.append(dict(batch=b,actual_official_count=int(n)))
    poseframes=[x for x in slices if x['camera']+2e-9>=firstpose['stamp']]
    badframes=[x for x in poseframes if x['minimum_ms']!=100. or x['maximum_ms']!=100. or x['pending']]
    metadata=read(p/'map_metadata.json');rgbfile=p/metadata['binary_filename'];rgb=np.frombuffer(rgbfile.read_bytes(),dtype=np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,)),('pad','u1')]))
    manifest=read(p/'first8_manifest.json');staging_checks={k:dict(archive_matches=sha(p/'staging'/k)==h,original_matches=sha(manifest['original_paths'][k])==h,sha256=h) for k,h in manifest['source_sha256'].items()}
    runtime=read(p/'runtime_manifest.json');runtime_checks={k:dict(matches=sha(a['path'])==a['sha256'],recorded_sha256=a['sha256']) for k,a in runtime['artifacts'].items()}
    record_fixed=r['initial_fixed_SE3_evaluation_only'];rot_delta=float(np.max(abs(R-np.array(record_fixed['rotation_world_from_slam']))));trans_delta=float(np.max(abs(translation-record_fixed['translation'])))
    errors=np.array(precision)
    independent=dict(all_eight_ordered_joint=sequential and all(x['first_joint_evidence'] for x in joint),full_active_truth_coverage=not holes,raw_tilt_envelope=active_max<.30,no_new_holds=holds==0)
    return dict(scope='Independent terminal read-only actual evidence audit. One initial SE3, GT evaluation only; no pose correction or result rewrites. Native/sample losses are reported, not assigned as physical causes.',run=p.name,original_result_sha256=sha(p/'first_eight_result.json'),original_passed=r['passed'],original_failed_checks=[k for k,v in r['checks'].items() if not v],independent_checks=independent,independent_passed=all(independent.values()),single_initial_SE3=dict(stamp=at,R=R.tolist(),translation=translation.tolist(),rotation_delta_to_recorded=rot_delta,translation_delta_to_recorded=trans_delta),trajectory=dict(actual_domain=[at,end],active_pose_count=len(active),paired_count=len(errors),unpaired_pose_stamps=holes,truth_pair_max_gap_s=.15,RMSE_m=float(np.sqrt(np.mean(errors**2))),max_error_m=float(errors.max()),final_error_m=float(errors[-1])),actual_NAV_request_id=request,NAV_ordered_transitions=transitions,ordered_joint_arrivals=joint,raw_IMU_physics=dict(active_max_tilt_rad=active_max,recorded_max_tilt=r['active_max_imu_tilt'],hold_increase=holds,recorded_active_holds=r['active_bridge_holds']),official_IMU=dict(count=len(keys),range_s=[float(keys[0]/1e6),float(keys[-1]/1e6)],non_1ms_gaps=officialgaps,invalid_json=parse_errors),native_IMU=dict(rows=len(native),relative_time_origin_s=offset,initial_state_match_error_m=float(np.linalg.norm(mat[mi,4:7]-firstpose['p'])),actual_raw_pair_rows_verified=len(ix),actual_prebias_mean_max_error=float(mean_difference.max()),unverified_pairs=len(native)-len(ix),missing_official_raw_heads_s=[float(x/1e6) for x in missing],non_1ms_pairs=nativegaps,NUL_bytes=(p/'fastlivo_debug/imu.txt').read_bytes().count(b'\0')),sync=dict(all_complete=all(x['complete'] for x in sync),newest_covers_camera=all(x['newest']+5e-9>=x['camera'] for x in sync),official_count_discrepancies=discrepancies),generic=dict(frames=len(poseframes),non_current_frames=badframes),RGB=dict(actual_points=len(rgb),finite_xyz=bool(np.isfinite(rgb['xyz']).all()),actual_colors=len(np.unique(rgb['rgb'],axis=0)),binary_sha256=sha(rgbfile),metadata=metadata,PCD_saved=bool(metadata['save'].get('complete'))),staging_hashes=staging_checks,runtime_hashes=runtime_checks)

if __name__=='__main__':
    p=Path(sys.argv[1]).resolve();out=Path(sys.argv[2]).resolve();d=audit(p);out.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n');print(json.dumps({k:d[k] for k in ('run','independent_passed','independent_checks','trajectory','official_IMU','native_IMU','sync')},indent=2))
