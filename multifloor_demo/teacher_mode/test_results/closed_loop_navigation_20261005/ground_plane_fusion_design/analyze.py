"""Read original d42e files; write new offline design receipts without overwrite."""
import hashlib
import json
from pathlib import Path
import numpy as np
from plane_observer import fit_candidate_planes,select_support_plane,vertical_observation
from pure_tests import run as pure_tests

BASE=Path(__file__).resolve().parent
RUN=BASE.parents[2]/'runs'/'20261005_145640_closed_loop_cascade_actual_multifloor_r1_d42e'
OUT=BASE/'final_v3'

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def write(n,a):
    with (OUT/n).open('x') as f:json.dump(a,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

def matrix(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def main():
    OUT.mkdir(exist_ok=False)
    anchor=json.loads((RUN/'navigation_anchor.json').read_text())
    initial_times=list(np.round(np.arange(6.,7.01,.1),1))
    requested_times=[30.,50.,80.,100.,120.,130.,160.,210.]
    times=initial_times+requested_times
    indices={t:None for t in times}
    with (RUN/'navigation_cloud_xyz.jsonl').open() as f:
        for l in f:
            x=json.loads(l);t=x['stamp_ns']/1e9
            for target in times:
                old=indices[target]
                if old is None or abs(t-target)<abs(old['stamp_ns']/1e9-target):indices[target]=x
    stamps={n for x in indices.values() for n in [x['stamp_ns'],x['filtering_body_stamp_ns']]}
    poses={}
    with (RUN/'navigation_slam_poses.jsonl').open() as f:
        for l in f:
            x=json.loads(l)
            if x['stamp_ns'] in stamps:poses[x['stamp_ns']]=x
    inputs={n:sha(RUN/n) for n in ['navigation_anchor.json','navigation_cloud_xyz.jsonl','navigation_slam_poses.jsonl','sensor_contract.json','navigation_request.json','navigation_fastlivo.yaml','source_manifest.json','runtime_manifest.json']}
    rows=[]
    for target in times:
        x=indices[target];pose=poses[x['stamp_ns']]
        p=np.load(RUN/x['array_file'],allow_pickle=False)
        actual_np=sha(RUN/x['array_file']);actual_raw=sha(RUN/x['raw_payload_file'])
        actual_decoded=hashlib.sha256(np.asarray(p,dtype='<f8').tobytes()).hexdigest()
        filter_pose=poses[x['filtering_body_stamp_ns']]
        filter_gap=x['stamp_ns']-x['filtering_body_stamp_ns']
        checks={'array_file_SHA':actual_np==x['array_sha256'],'rawpayload_SHA':actual_raw==x['raw_payload_sha256'],'decoded_float64_SHA':actual_decoded==x['filtered_xyz_float64_sha256'],'cloud_pose_stamp_exact':x['stamp_ns']==pose['stamp_ns'],'filter_pose_original_stamp_and_XYZ_exact':np.array_equal(x['filtering_body_pose'],filter_pose['position']),'filter_pose_causal_within150ms':0<=filter_gap<=150000000,'frame_camera_init':x['frame_id']==pose['frame_id']=='camera_init','point_count':list(p.shape)==x['shape']}
        if not all(checks.values()):raise ValueError(f'original cloud/source mismatch {target}: {checks}')
        candidates=fit_candidate_planes(p,pose['position'])
        selected=select_support_plane(candidates)
        rows.append(dict(requested_time_s=target,original_cloud_stamp_ns=x['stamp_ns'],original_cloud_index=x,original_raw_SLAM_pose=pose,original_self_filter_SLAM_pose=filter_pose,self_filter_source_gap_ns=filter_gap,source_checks=checks,candidates=candidates,selection=selected))
        inputs[x['array_file']]=actual_np;inputs[x['raw_payload_file']]=actual_raw
    initial=[x for x in rows if x['requested_time_s'] in initial_times]
    if not all(x['selection']['status']=='observed' for x in initial):raise ValueError('initial ground calibration ambiguous')
    ai=np.asarray(anchor['origin']);heights=[];normals=[]
    for x in initial:
        pl=x['selection']['selected'];n=np.asarray(pl['normal']);heights.append(float(-(n[:2]@ai[:2]+pl['offset'])/n[2]));normals.append(n)
    hinit=float(np.median(heights));ninit=np.median(normals,axis=0);ninit/=np.linalg.norm(ninit)
    dinit=float(-ninit@np.array([*ai[:2],hinit]))
    yaw=anchor['yaw'];R2=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    for x in rows:
        pos=np.asarray(x['original_raw_SLAM_pose']['position']);prior_scene_xy=np.array([1.2,2.])+R2.T@(pos[:2]-ai[:2])
        elev=0. if x['requested_time_s'] in initial_times else (float(np.clip(.1*(prior_scene_xy[0]-2.),0,1.2)) if x['requested_time_s']<100 else 1.2)
        x['static_map_elevation_prior_m']=elev;x['scene_prior_XY_from_actual_SLAM']=prior_scene_xy.tolist()
        x['horizontal_map_prior_observation']=vertical_observation(x['selection'],hinit,elev)
        x['prior_scope']='initial floor1 / lower12 slope / floor2 platform fixed surveyed map; no native input; not automatic floor classifier'
        if x['selection']['status']=='observed':
            measured=x['selection']['selected']['height_at_body_xy']
            expected_parallel=float((elev-dinit-ninit[:2]@pos[:2])/ninit[2])
            x['initial_observed_up_normal_sensitivity']={'normal':ninit.tolist(),'expected_plane_height':expected_parallel,'delta_z':expected_parallel-measured,'scope':'offline frame-tilt sensitivity only, distinct from original yaw-only map prior'}
    # Serialize cloud-only observations before reading any native state.
    write('cloud_only_observations.json',dict(schema='original_cloud_static_map_plane_observation/v1',inputs_sha256=inputs,initial={'sample_count':len(initial),'plane_z_at_frozen_anchor_xy_m':hinit,'initial_height_MAD_m':float(np.median(np.abs(np.asarray(heights)-hinit))),'h0_body_origin_clearance_m':float(ai[2]-hinit),'initial_observed_normal':ninit.tolist()},rows=rows,native_or_Gazebo_inputs_used=False))
    targets={float(x['requested_time_s']) for x in rows};native={}
    with (RUN/'actuator.jsonl').open() as f:
        for l in f:
            x=json.loads(l)
            if x.get('kind')!='physics_step':continue
            t=x['t']-.005
            for target in targets:
                if target not in native or abs(t-target)<abs(native[target]['effective_time_s']-target):native[target]={'effective_time_s':t,'state':x}
    inputs['actuator.jsonl']=sha(RUN/'actuator.jsonl')
    anch=native[6.7]['state'];sp=poses[anchor['pose_stamp_ns']]
    qnative=anch['quaternion_wxyz'];Rw=matrix([qnative[1],qnative[2],qnative[3],qnative[0]])
    Rcw=matrix(sp['quaternion'])@Rw.T
    world0=np.asarray(anch['position']);offline=[]
    for x in rows:
        t=x['requested_time_s'];truth=native[t];p=np.asarray(x['original_raw_SLAM_pose']['position'])
        expected=ai+Rcw@(np.asarray(truth['state']['position'])-world0)
        offline.append({'time_s':t,'native_effective_time_s':truth['effective_time_s'],'native_world_position':truth['state']['position'],'initial_once_SE3_offline_expected_camera_init_body_origin':expected.tolist(),'raw_SLAM_z':float(p[2]),'offline_vertical_bias_expected_minus_SLAM_m':float(expected[2]-p[2]),'horizontal_plane_observed_delta_z_m':x['horizontal_map_prior_observation']['delta_z'],'initial_normal_sensitivity_delta_z_m':x.get('initial_observed_up_normal_sensitivity',{}).get('delta_z'),'native_input_used_by_observer':False})
    tests=pure_tests();write('pure_tests.json',tests)
    write('native_OFFLINE_comparison.json',dict(schema='native_truth_only_offline_plane_observability_comparison/v1',initial_once_transform={'camera_init_from_world_rotation':Rcw.tolist(),'anchor_stamp_ns':anchor['pose_stamp_ns'],'native_anchor_effective_time_s':native[6.7]['effective_time_s'],'navigation_ground_truth_used':False},rows=offline,actuator_sha256=inputs['actuator.jsonl']))
    write('aggregate.json',dict(schema='ground_plane_fusion_offline_design/v1',status='offline_observability_demonstrated_not_runtime_verified',original_run=str(RUN),source_hashes={n:sha(BASE/n)for n in ['plane_observer.py','pure_tests.py','analyze.py']},inputs_sha256=inputs,source_check_all_passed=all(all(x['source_checks'].values())for x in rows),unique_support_observed_samples=sum(x['selection']['status']=='observed'for x in rows),sample_count=len(rows),initial_plane_height_m=hinit,initial_clearance_h0_m=float(ai[2]-hinit),pure_tests=tests,offline_comparison=offline,limits=['Original registered cloud and pose share the SLAM estimate; static map prior supplies independent elevation','Original map uses yaw-only gravity-aligned prior; measured initial plane tilt produces a separate sensitivity result','Known support-layer/elevation association must be derived from causal route/map observations; this offline case list is not a runtime layer selector','No navigation/SLAM/Actor/source/receipt changed; no live ground-truth feedback; no deployed fusion pass','All plane estimates precede any native read; native only evaluates original data offline']))
    print(json.dumps({'aggregate_sha256':sha(OUT/'aggregate.json'),'initial_plane_z':hinit,'samples':len(rows),'selected':[(x['time_s'],x['horizontal_plane_observed_delta_z_m'],x['initial_normal_sensitivity_delta_z_m'],x['offline_vertical_bias_expected_minus_SLAM_m'])for x in offline if x['time_s']>=100],'pure_checks':tests['status']}))


if __name__=='__main__':main()
