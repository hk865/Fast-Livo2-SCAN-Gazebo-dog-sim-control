import os
for key in ['OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS']:
    os.environ[key] = '1'
import json, hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[2]
RUN = BASE / 'runs/20261005_145640_closed_loop_cascade_actual_multifloor_r1_d42e'
CUTOFF = 221.0
sources = {}

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''): h.update(b)
    return h.hexdigest()

def prefix(name, time_fn):
    p=RUN/name; h=hashlib.sha256(); count=0; length=0
    with p.open('rb') as f:
        for b in f:
            a=json.loads(b)
            t=time_fn(a)
            if t is not None and t > CUTOFF: break
            h.update(b); length+=len(b); count+=1
            yield a
    sources[name]={'path':str(p),'consumed_prefix_bytes':length,'prefix_sha256':h.hexdigest(),'lines':count,'cutoff_sim_s':CUTOFF}

anchor=json.loads((RUN/'navigation_anchor.json').read_text())
registration=json.loads((RUN/'navigation_scene_axis_registration.json').read_text())
final=json.loads((RUN/'navigation_status.json').read_text())
profile=json.loads((RUN/'input_profile.json').read_text())
native=[]
for a in prefix('actuator.jsonl', lambda a: a.get('t',0)-.005 if a.get('kind')=='physics_step' else None):
    if a.get('kind')=='physics_step':
        native.append([a['t']-.005,*a['position'],*a['quaternion_wxyz'],*a['body_lin_vel_origin']])
native=np.array(native)
nt=native[:,0]; np_=native[:,1:4]
nrot=Rotation.from_quat(native[:,[5,6,7,4]])
nslerp=Slerp(nt,nrot)
feedback=[]
paired=[]
for a in prefix('navigation_feedback_history.jsonl', lambda a:a['stamp_ns']/1e9):
    feedback.append([a['stamp_ns']/1e9,*a['position_world_xyz'],*a['quaternion_wxyz']])
    imu=a.get('paired_imu')
    paired.append(imu)
feedback=np.array(feedback); t=feedback[:,0]; p=feedback[:,1:4]
srot=Rotation.from_quat(feedback[:,[5,6,7,4]])
native_interp=np.column_stack([np.interp(t,nt,np_[:,i]) for i in range(3)])
anchor_t=anchor['pose_stamp_ns']/1e9
native_anchor=np.array([np.interp(anchor_t,nt,np_[:,i]) for i in range(3)])
anchor_p=np.array(anchor['origin'])
R_yaw=np.array(registration['heading_receipt']['rotation_camera_init_from_world'])
calib=registration['exact_paired_sample_records']
R_pairs=np.array([(Rotation.from_quat(a['slam_body_quaternion'])*Rotation.from_quat(a['imu_quaternion']).inv()).as_matrix() for a in calib])
R_initial=Rotation.from_matrix(R_pairs).mean().as_matrix()
delta=native_interp-native_anchor
expected_yaw=anchor_p+delta@R_yaw.T
expected_full=anchor_p+delta@R_initial.T
residual=p-expected_full
instant_R=(srot*nslerp(t).inv())
orientation_delta=(instant_R*Rotation.from_matrix(R_initial).inv()).as_euler('xyz')
instant_euler=instant_R.as_euler('xyz')
available_pairs=[a for a in paired if a is not None]
pair_t=np.array([a['stamp_ns']/1e9 for a in available_pairs])
pair_rot=Rotation.from_quat([a['orientation_xyzw'] for a in available_pairs])
sensor_native_rotation_error=(pair_rot*nslerp(pair_t).inv()).magnitude()

status=[]; previous=None; first_failed=None
for a in prefix('navigation_status.jsonl', lambda a:a.get('ros_sim_time',0)):
    sig=(a.get('waypoint_index'),a.get('state'))
    if sig!=previous:
        status.append({'time':a.get('ros_sim_time'),'waypoint_index':a.get('waypoint_index'),'state':a.get('state'),'goal':a.get('current_goal'),'pose':a.get('pose'),'message':a.get('message'),'region_arrivals':a.get('region_arrivals')})
        previous=sig
    if a.get('state')=='failed' and first_failed is None: first_failed=status[-1]

def snapshot(tm):
    i=int(np.argmin(abs(t-tm)))
    goal_z=anchor_p[2]+1.2
    return {'time_s':float(t[i]),'raw_slam_xyz_m':p[i].tolist(),'native_world_xyz_m':native_interp[i].tolist(),
            'expected_slam_if_initial_frame_constant_xyz_m':expected_full[i].tolist(),
            'raw_minus_constant_initial_frame_xyz_m':residual[i].tolist(),
            'instant_camera_init_from_world_rpy_deg':np.rad2deg(instant_euler[i]).tolist(),
            'frame_orientation_change_rpy_deg':np.rad2deg(orientation_delta[i]).tolist(),
            'floor2_goal_z_m':float(goal_z),
            'floor2_z_gap_decomposition_m':{
                'body_origin_clearance_change_since_anchor':float(native_interp[i,2]-native_anchor[2]-1.2),
                'initial_constant_frame_tilt_contribution':float(expected_full[i,2]-expected_yaw[i,2]),
                'remaining_slam_position_error':float(residual[i,2]),
                'total_raw_z_minus_goal_z':float(p[i,2]-goal_z)}}

snapshots=[snapshot(tm) for tm in [6.7,40,60,80,100,110,120,130,140,150,160,180,200,219.9]]
windows=[]
for lo,hi in [(6.7,20),(100,110),(110,120),(120,130),(130,140),(140,150),(150,180),(180,219.9)]:
    m=(t>=lo)&(t<=hi)
    if not m.any(): continue
    windows.append({'start_s':lo,'end_s':hi,'sample_count':int(m.sum()),
                    'native_z_minmax_m':[float(native_interp[m,2].min()),float(native_interp[m,2].max())],
                    'raw_slam_z_minmax_m':[float(p[m,2].min()),float(p[m,2].max())],
                    'residual_z_firstlast_m':[float(residual[m,2][0]),float(residual[m,2][-1])],
                    'residual_z_slope_mps':float(np.polyfit(t[m],residual[m,2],1)[0]),
                    'raw_slam_z_slope_mps':float(np.polyfit(t[m],p[m,2],1)[0]),
                    'orientation_change_firstlast_rpy_deg':np.rad2deg(orientation_delta[m][[0,-1]]).tolist()})

metrics={'schema':'offline_slam_multifloor_height_diagnostic/v1','run':str(RUN),
         'scope':'offline native reference only; original navigation and height gate unchanged',
         'navigation_ground_truth_used_by_this_diagnostic':False,'diagnostic_native_reference_used':True,
         'analysis_cutoff_sim_s':CUTOFF,'native_samples':len(nt),'raw_slam_samples':len(t),
         'native_state_timestamp_rule':'physics_step.t - 0.005 s, prior state of request',
         'initial_anchor_time_s':anchor_t,'native_anchor_world_xyz_m':native_anchor.tolist(),
         'raw_slam_anchor_xyz_m':anchor_p.tolist(),
         'initial_rotation_from_actual_slam_and_imu_rpy_deg':Rotation.from_matrix(R_initial).as_euler('xyz',degrees=True).tolist(),
         'initial_yaw_only_rotation_rpy_deg':Rotation.from_matrix(R_yaw).as_euler('xyz',degrees=True).tolist(),
         'actual_imu_orientation_vs_native_rotation_error_rad':{'maximum':float(sensor_native_rotation_error.max()),'median':float(np.median(sensor_native_rotation_error))},
         'snapshots':snapshots,'windows':windows,'navigation_phase_changes':status,
         'first_original_navigation_failed':first_failed,'original_final_navigation_state':final['state'],
         'original_goal_height_half_span_m':.1,'sources':sources,
         'frozen_file_hashes':{name:sha(RUN/name) for name in ['navigation_anchor.json','navigation_scene_axis_registration.json','input_profile.json','navigation_fastlivo.yaml','sensor_contract.json','source_manifest.json','runtime_manifest.json','run_result.json']}}
pre=np.loadtxt(RUN/'fastlivo_debug/mat_pre.txt')
post=np.loadtxt(RUN/'fastlivo_debug/mat_out.txt')
pre_by_time={}; post_by_time={}
for a in pre: pre_by_time.setdefault(float(a[0]),[]).append(a)
for a in post: post_by_time.setdefault(float(a[0]),[]).append(a)
updates=[]; previous=None
for relative_time, group in sorted(post_by_time.items()):
    if relative_time+.01>CUTOFF:break
    lio=next((a for a in group if a[19]>0),None)
    if relative_time not in pre_by_time or lio is None:continue
    prediction=pre_by_time[relative_time][0]; result=group[-1]
    if previous is not None:
        updates.append([relative_time+.01,prediction[6]-previous[6],lio[6]-prediction[6],result[6]-lio[6],
                        prediction[6],lio[6],result[6],prediction[9],lio[9],result[9],lio[19],
                        prediction[9]-previous[9],lio[9]-prediction[9],result[9]-lio[9]])
    previous=result
updates=np.array(updates)
update_windows=[]
for lo,hi in [(130,140),(131.7,133.7),(131.7,133.3),(140,150),(150,220)]:
    a=updates[(updates[:,0]>lo)&(updates[:,0]<=hi)]
    update_windows.append({'start_exclusive_s':lo,'end_inclusive_s':hi,'updates':len(a),
                           'delta_z_sum_propagation_lio_vio_m':a[:,1:4].sum(axis=0).tolist(),
                           'delta_vz_sum_propagation_lio_vio_mps':a[:,11:14].sum(axis=0).tolist(),
                           'max_abs_vio_z_correction_m':float(abs(a[:,3]).max()),
                           'lidar_raw_point_count_range':[int(a[:,10].min()),int(a[:,10].max())]})
metrics['livo_update_decomposition']={'world_time_offset_s':.01,
    'offset_basis':'first feedback at 3.7 s equals first mat_out at relative 3.69 s, source position checked',
    'columns': ['world_time','dz_prediction','dz_lio_correction','dz_vio_correction','z_prediction','z_lio','z_vio','vz_prediction','vz_lio','vz_vio','raw_lidar_points','dvz_prediction','dvz_lio_correction','dvz_vio_correction'],
    'window_decomposition':update_windows,
    'limitations':'prediction inherits previous corrected velocity; this split is not a causal proof of raw IMU sensor error. mat logs have limited decimal precision. No solver Hessian or effective-plane directions archived.'}
metrics['debug_final_files_sha256']={name:sha(RUN/name) for name in ['fastlivo_debug/mat_pre.txt','fastlivo_debug/mat_out.txt','fastlivo_debug/imu.txt','navigation_stack.log']}
metrics['initialization_log_summary']={'stable_samples':605,'required_samples':600,'first_stamp_s':.58,'last_stamp_s':3.6,'span_s':3.02,
    'mean_acc_body_mps2':[-.263050,-.026124,9.800469], 'acc_std_body_mps2':[.023828,.027940,.040921], 'max_gyro_radps':.028850,
    'bias_g_assigned_by_stationary_initializer':[0,0,0], 'gravity_est_en':False,'ba_bg_est_en':False}
(HERE/'livo_update_arrays.npz').unlink(missing_ok=True)
np.savez_compressed(HERE/'livo_update_arrays.npz',updates=updates)
(HERE/'metrics.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False)+'\n')
np.savez_compressed(HERE/'diagnostic_arrays.npz',time=t,raw_slam_xyz=p,native_world_xyz=native_interp,expected_yaw_xyz=expected_yaw,expected_full_xyz=expected_full,residual_xyz=residual,orientation_delta_rpy=orientation_delta,instant_camera_world_rpy=instant_euler)
print(json.dumps({'initial_rpy_deg':metrics['initial_rotation_from_actual_slam_and_imu_rpy_deg'],'anchor_native':native_anchor.tolist(),'imu_native_error':metrics['actual_imu_orientation_vs_native_rotation_error_rad'],'end_snapshot':snapshots[-1], 'livo_update_decomposition':update_windows,'failed':first_failed},indent=2,ensure_ascii=False))
