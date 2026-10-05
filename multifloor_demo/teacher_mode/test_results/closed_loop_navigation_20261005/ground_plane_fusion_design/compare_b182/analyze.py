"""Additional immutable original-cloud comparison, no runtime or native input."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE))
from plane_observer import fit_candidate_planes,select_support_plane
from frame_sensitivity import expected_plane_height
from analyze import matrix
ROOT=BASE.parents[2]
OUT=Path(__file__).resolve().parent/'final'
RUNS={'d42e':ROOT/'runs'/'20261005_145640_closed_loop_cascade_actual_multifloor_r1_d42e',
      'b182':ROOT/'runs'/'20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182'}

def sha(p):
    h=hashlib.sha256()
    with p.open('rb')as f:
        for c in iter(lambda:f.read(1024*1024),b''):h.update(c)
    return h.hexdigest()

def main():
    OUT.mkdir(exist_ok=False)
    result={};arrays={}
    for label,run in RUNS.items():
        anchor=json.loads((run/'navigation_anchor.json').read_text());t0=anchor['pose_stamp_ns']/1e9
        times=[t0,130.,135.,140.,160.,210.];idx={t:None for t in times}
        with (run/'navigation_cloud_xyz.jsonl').open()as f:
            for l in f:
                x=json.loads(l);t=x['stamp_ns']/1e9
                for target in times:
                    old=idx[target]
                    if old is None or abs(t-target)<abs(old['stamp_ns']/1e9-target):idx[target]=x
        stamps={n for x in idx.values()for n in [x['stamp_ns'],x['filtering_body_stamp_ns']]};poses={}
        with (run/'navigation_slam_poses.jsonl').open()as f:
            for l in f:
                x=json.loads(l)
                if x['stamp_ns']in stamps:poses[x['stamp_ns']]=x
        inputs={n:sha(run/n)for n in ['navigation_cloud_xyz.jsonl','navigation_slam_poses.jsonl','navigation_anchor.json','navigation_fastlivo.yaml']};rows=[]
        for target in times:
            x=idx[target];pose=poses[x['stamp_ns']];p=np.load(run/x['array_file'],allow_pickle=False);filter_pose=poses[x['filtering_body_stamp_ns']]
            ck={'array_SHA':sha(run/x['array_file'])==x['array_sha256'],'raw_SHA':sha(run/x['raw_payload_file'])==x['raw_payload_sha256'],'decoded_SHA':hashlib.sha256(np.asarray(p,dtype='<f8').tobytes()).hexdigest()==x['filtered_xyz_float64_sha256'],'cloud_stamp_raw_pose_exact':pose['stamp_ns']==x['stamp_ns'],'original_filter_pose_exact':np.array_equal(filter_pose['position'],x['filtering_body_pose']),'filter_pose_causal':0<=x['stamp_ns']-x['filtering_body_stamp_ns']<=150000000,'frame':x['frame_id']==pose['frame_id']=='camera_init'}
            if not all(ck.values()):raise ValueError(f'{label}/{target}: source mismatch {ck}')
            candidates=fit_candidate_planes(p,pose['position']);s=select_support_plane(candidates);row={'target_time_s':target,'source_header_stamp_ns':x['stamp_ns'],'original_cloud_index':x,'raw_SLAM_pose':pose,'original_source_checks':ck,'candidates':candidates,'selection':s}
            if s['status']=='observed':
                pl=s['selected'];n=np.asarray(pl['normal']);pos=np.asarray(pose['position']);local=np.linalg.norm(p[:,:2]-pos[:2],axis=1)<=2.5;local&=np.abs(p[:,2]-pos[2])<=3;inlier=local&(np.abs(p@n+pl['offset'])<=.015)
                relative=p[inlier]-pos;J=np.c_[np.cross(relative,n),np.tile(n,(len(relative),1))];sv=np.linalg.svd(J/np.sqrt(len(J)),compute_uv=False)
                row['single_plane_observability']={'jacobian_order':'rotationXYZ,translationXYZ','singular_values_normalized_by_sqrt_points':sv.tolist(),'rank_at_relative_tolerance1e10':int(np.sum(sv>sv[0]*1e-10)),'normal_translation_observable':True,'two_in_plane_translations_unobservable':True,'rotation_about_plane_normal_unobservable':True,'Hessian_is_geometry_proxy_not_actual_FASTLIVO_Hessian':True,'all_local_points':int(local.sum()),'ground_inlier_fraction':float(inlier.sum()/local.sum())}
                arrays[f'{label}_{target:g}_local']=p[local];arrays[f'{label}_{target:g}_support']=p[inlier]
            rows.append(row);inputs[x['array_file']]=sha(run/x['array_file']);inputs[x['raw_payload_file']]=sha(run/x['raw_payload_file'])
        initial=rows[0]['selection']
        if initial['status']!='observed':raise ValueError('initial plane missing')
        h0plane=initial['selected']['height_at_body_xy'];pair=anchor['scene_axis_registration']['exact_paired_sample_records'][-1];Rcb=matrix(pair['slam_body_quaternion']);Rwb=matrix(pair['imu_quaternion']);up=(Rcb@Rwb.T)[:,2]
        for row in rows:
            elev=0. if row is rows[0] else 1.2
            if row['selection']['status']=='observed':
                measured=row['selection']['selected']['height_at_body_xy'];expected=expected_plane_height(h0plane,anchor['origin'][:2],row['raw_SLAM_pose']['position'][:2],elev,up)
                row['known_floor_elevation_prior_m']=elev;row['horizontal_map_delta_z_m']=h0plane+elev-measured;row['actual_SLAM_IMU_up_axis_delta_z_m']=expected-measured
        result[label]={'run':str(run),'anchor':anchor,'up_axis_from_original_SLAM_IMU':up.tolist(),'input_hashes':inputs,'config_recorded':{},'raw_config_SHA':inputs['navigation_fastlivo.yaml'],'rows':rows}
    # Keep YAML field location exact without assumptions about dotted sections.
    for label,run in RUNS.items():
        cfg=json.loads((run/'navigation_fastlivo.yaml').read_text())['/**']['ros__parameters'];found={}
        for section,value in cfg.items():
            if isinstance(value,dict):
                for k in ['gravity_est_en','ba_bg_est_en','imu_rate_odom']:
                    if k in value:found[section+'.'+k]=value[k]
        result[label]['config_recorded']=found
    aggregate={'schema':'actual_two_run_ground_plane_geometry_observability/v1','status':'offline_observability_only','source_math_hashes':{n:sha(BASE/n)for n in ['plane_observer.py','frame_sensitivity.py']},'source_script_sha256':sha(Path(__file__)),'runs':result,'native_inputs_used':False,'SLAM_internal_Hessian_or_loop_keyframe_trace_available':False,'interpretation':['Known floor-height prior plus measured ground observes height normal to the support plane','A single plane does not determine two tangent translations or yaw about its normal','Many ground points are not evidence that actual SLAM optimizes height against this known prior','Other visible points cannot by themselves prove actual matched-feature or optimizer constraints','No source, runtime, original receipt, threshold or goal changed']}
    with (OUT/'aggregate.json').open('x')as f:json.dump(aggregate,f,indent=2,allow_nan=False)
    with (OUT/'actual_local_points.npz').open('xb')as f:np.savez_compressed(f,**arrays)
    print(json.dumps({'aggregate_sha256':sha(OUT/'aggregate.json'),'rows':{k:[{'t':x['target_time_s'],'SLAMz':x['raw_SLAM_pose']['position'][2],'plane':x['selection']['selected']['height_at_body_xy']if x['selection']['status']=='observed'else None,'delta':x.get('horizontal_map_delta_z_m'),'sensor_up_delta':x.get('actual_SLAM_IMU_up_axis_delta_z_m'),'rank':x.get('single_plane_observability',{}).get('rank_at_relative_tolerance1e10')}for x in v['rows']]for k,v in result.items()}}))

if __name__=='__main__':main()
