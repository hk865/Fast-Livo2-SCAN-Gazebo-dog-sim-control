#!/usr/bin/env python3
"""Read only the once-extracted compact evidence; no original raw log access."""
from pathlib import Path
import json,gzip,bisect,hashlib,collections,math
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
OUT=Path(__file__).resolve().parent
read=lambda n:json.loads((OUT/n).read_text())
rows=lambda n:[json.loads(x)for x in gzip.open(OUT/n,'rt')]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(n,d):
 with (OUT/n).open('x')as f:f.write(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
native=rows('e8de_native50Hz_compact.jsonl.gz');pid=rows('e8de_regions11_12_pid.jsonl.gz');status=rows('e8de_regions11_12_status.jsonl.gz');frame=read('E8DE_FRAME_INPUTS.json');reg=frame['navigation_scene_axis_registration.json']
nt=np.asarray([round(r['state_physics_world_time']*1e9)for r in native],dtype=np.int64);assert np.all(np.diff(nt)>0)
def match(t):
 j=bisect.bisect_left(nt,t)
 if j<len(nt)and nt[j]==t:a=b=j
 elif 0<j<len(nt):a,b=j-1,j
 else:raise ValueError('extrapolation forbidden')
 assert nt[b]-nt[a]<=20_000_001
 w=0. if a==b else float((t-nt[a])/(nt[b]-nt[a]))
 p=(1-w)*np.asarray(native[a]['position'])+w*np.asarray(native[b]['position'])
 qa=np.roll(native[a]['quaternion_wxyz'],-1);qb=np.roll(native[b]['quaternion_wxyz'],-1)
 rr=Rotation.from_quat(qa)if a==b else Slerp([0.,1.],Rotation.from_quat([qa,qb]))([w])[0]
 return p,rr,dict(left_native_stamp_ns=int(nt[a]),right_native_stamp_ns=int(nt[b]),max_distance_to_bracket_ns=int(max(t-nt[a],nt[b]-t)),nearest_distance_ns=int(min(t-nt[a],nt[b]-t)),left_native_source_line=native[a]['source_line'],right_native_source_line=native[b]['source_line'],linear_position_and_SLerp_attitude=True)
initial=[]
for pair in reg['exact_paired_sample_records']:
 t=pair['slam_stamp_ns'];pn,rn,join=match(t);ps=np.asarray(pair['slam_source']['position']);rs=Rotation.from_quat(pair['slam_body_quaternion']);rcw=rs*rn.inv()
 initial.append((ps,pn,rcw,dict(stamp_ns=t,SLAM_camera_position=ps.tolist(),native_world_base_link_position=pn.tolist(),SLAM_quaternion_xyzw=pair['slam_body_quaternion'],native_quaternion_xyzw=rn.as_quat().tolist(),join=join)))
rcw=Rotation.from_quat([x[2].as_quat()for x in initial]).mean();R=rcw.as_matrix();translation=np.mean([ps-R@pn for ps,pn,_,_ in initial],axis=0)
Ryaw=np.asarray(reg['heading_receipt']['rotation_camera_init_from_world']);tyaw=np.mean([ps-Ryaw@pn for ps,pn,_,_ in initial],axis=0)
residuals=np.asarray([ps-(R@pn+translation)for ps,pn,_,_ in initial]);rotation_spread=[float((rcw.inv()*rot).magnitude())for _,_,rot,_ in initial]
poses={}
for r in status:
 a=r.get('region_arrival_evidence')or{};t=a.get('stamp_ns');p=a.get('raw_position')
 if t is not None and p is not None:poses[t]=dict(stamp_ns=t,position=p,waypoint_index=r['waypoint_index'],status_row=r['source_line'],status_source_line_sha256=r['source_line_sha256'])
for r in pid:
 f=r.get('feedback')or{};t=f.get('stamp_ns');p=f.get('position_world_xyz')
 if t is not None and p is not None:
  if t in poses:assert np.max(np.abs(np.asarray(poses[t]['position'])-p))<1e-10
  poses[t]=dict(stamp_ns=t,position=p,quaternion_wxyz=f['quaternion_wxyz'],waypoint_index=r['waypoint_index'],pid_row=r['source_line'],pid_source_line_sha256=r['source_line_sha256'])
joined=[]
for t,s in sorted(poses.items()):
 pn,rn,m=match(t);p=np.asarray(s['position']);expected=R@pn+translation;eyaw=Ryaw@pn+tyaw
 row=dict(**s,native_world_position=pn.tolist(),native_world_quaternion_xyzw=rn.as_quat().tolist(),native_position_in_fixed_initial_camera_frame=expected.tolist(),native_position_using_frozen_route_yaw_and_initial_translation=eyaw.tolist(),SLAM_minus_fixed_initial_native_m=(p-expected).tolist(),SLAM_minus_initial_yaw_native_m=(p-eyaw).tolist(),timejoin=m)
 if s.get('quaternion_wxyz'):
  rs=Rotation.from_quat(np.roll(s['quaternion_wxyz'],-1));error=rs*(rcw*rn).inv();row['SLAM_world_attitude_error_rotation_vector_rad']=error.as_rotvec().tolist();row['SLAM_world_attitude_error_angle_rad']=float(error.magnitude())
 joined.append(row)
with gzip.open(OUT/'e8de_same_time_frame_join.jsonl.gz','xt')as f:
 for row in joined:f.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n')
def nearest(t):return min(joined,key=lambda r:abs(r['stamp_ns']/1e9-t))
def period(a,b):
 rr=[r for r in joined if a<=r['stamp_ns']/1e9<=b];assert rr
 s=np.asarray([r['position']for r in rr]);n=np.asarray([r['native_position_in_fixed_initial_camera_frame']for r in rr]);nw=np.asarray([r['native_world_position']for r in rr]);e=s-n
 return dict(interval_s=[a,b],rows=len(rr),SLAM_z_m={'median':float(np.median(s[:,2])),'min':float(s[:,2].min()),'max':float(s[:,2].max())},native_world_base_z_m={'median':float(np.median(nw[:,2])),'min':float(nw[:,2].min()),'max':float(nw[:,2].max())},native_fixed_camera_z_m={'median':float(np.median(n[:,2])),'min':float(n[:,2].min()),'max':float(n[:,2].max())},SLAM_minus_fixed_native_z_m={'median':float(np.median(e[:,2])),'min':float(e[:,2].min()),'max':float(e[:,2].max())})
periods=[period(*x)for x in [(198,199.5),(200,203),(205,208),(212,215),(217,218),(219,220),(220.3,222),(225,230),(285,289.45)]]
changes=[];last=None
for r in status:
 if r['waypoint_index']!=11:continue
 c=r.get('cascade_parking')or{};key=(r['state'],r['degenerate_splines'],r['accepted_trajectory_id'],r['alignment_phase'],r['obstacle_hold'])
 if key!=last:
  changes.append({k:r[k]for k in ['source_line','source_offset','source_line_sha256','ros_sim_time','waypoint_index','state','pose','command','accepted_trajectory_id','degenerate_splines','alignment_phase','obstacle_hold','message','region_arrival_evidence']});last=key
pid_stages=collections.Counter((r['waypoint_index'],r['cascade'].get('mode'),r['cascade'].get('reason'))for r in pid)
no_path=[r for r in status if r['waypoint_index']==11 and r['degenerate_splines']>0 and r['state']=='running']
geom=read('E8DE_LAST3_SCAN_GEOMETRY.json');geometry=[]
for g in geom:
 a=np.asarray(g['arrays']['samples']['values']);arc=float(np.linalg.norm(np.diff(a[:,:2],axis=0),axis=1).sum());coeff=np.asarray(g['arrays']['coefficients']['values']);geometry.append(dict(path=g['path'],sha256=g['sha256'],samples_count=len(a),sample_start=a[0].tolist(),sample_end=a[-1].tolist(),sample_z_range_m=[float(a[:,2].min()),float(a[:,2].max())],horizontal_arc_m=arc,horizontal_extent_m=float(np.max(np.linalg.norm(a[:,:2]-a[0,:2],axis=1))),coefficients_all_constant=bool(np.ptp(coeff,axis=0).max()<1e-10)))
result=dict(schema='e8de_compact_same_time_diagnostic/v1',run=read('E8DE_STREAM_RECEIPT.json')['run'],original_full46_pass=False,frame_calibration={'method':'Fixed initial SE3 from actual 26 scene-registration SLAM pose/attitude pairs and bracketing native base_link pose. Mean relative rotation; mean position translation. Used offline only. Also record original frozen route yaw with initial fitted translation as sensitivity comparison.','initial_pair_count':len(initial),'initial_interval_ns':[initial[0][3]['stamp_ns'],initial[-1][3]['stamp_ns']],'rotation_camera_from_world':R.tolist(),'translation_camera_from_world_m':translation.tolist(),'relative_roll_pitch_yaw_rad':rcw.as_euler('xyz').tolist(),'initial_position_residual_max_abs_xyz_m':np.max(np.abs(residuals),axis=0).tolist(),'initial_position_residual_RMS_xyz_m':np.sqrt(np.mean(residuals**2,axis=0)).tolist(),'relative_rotation_spread_max_rad':max(rotation_spread),'native_position_is_base_link_origin':True,'COM_position_offset_subtracted':False,'initial_pairs':[x[3]for x in initial]},timejoin={'samples':len(joined),'no_extrapolation':True,'native_max_bracket_gap_ns':max(x['timejoin']['right_native_stamp_ns']-x['timejoin']['left_native_stamp_ns']for x in joined),'native_max_nearest_gap_ns':max(x['timejoin']['nearest_distance_ns']for x in joined),'native_max_distance_to_bracket_ns':max(x['timejoin']['max_distance_to_bracket_ns']for x in joined),'SLAM_position_sources':'Original PID feedback stamp+position and original status region_arrival_evidence stamp+raw_position, never status publication time substituted for SLAM pose time'},landmarks={str(t):nearest(t)for t in [199.455,203,208,215,217.67,220.08,220.28,223,230,289.444999999]},periods=periods,status_transitions=changes,pid_stage_counts=[[*k,v]for k,v in pid_stages.items()],after_first_degenerate_running_statuses={'count':len(no_path),'first_status_sim_s':no_path[0]['ros_sim_time'],'last_status_sim_s':no_path[-1]['ros_sim_time'],'max_absolute_command':max(abs(v)for r in no_path for v in r['command']),'obstacle_hold_any':any(r['obstacle_hold']for r in no_path),'last_degenerate_count':no_path[-1]['degenerate_splines'],'accepted_trajectory_id_values':sorted(set(str(r['accepted_trajectory_id'])for r in no_path))},last3_successfully_archived_SCAN_paths=geometry,limitations=['No actual occupancy/costmap or ESDF replay was captured by this compact extraction. A collision-related log does not identify the offending voxel or establish that SLAM height caused it.','The final successful archive is not a rejected EmergencyStop spline. Full original rejected spline coefficients were not saved by this archive path.','Fixed initial alignment and native interpolation quantify divergence and motion, but do not diagnose estimator gravity/bias or a specific map defect.','No 200Hz actuator replay or changed acceptance thresholds. Source previews and one complete scan counts are disclosed in E8DE_STREAM_RECEIPT.'],source_bindings={str(OUT/n):sha(OUT/n)for n in ['E8DE_STREAM_RECEIPT.json','E8DE_FRAME_INPUTS.json','e8de_native50Hz_compact.jsonl.gz','e8de_regions11_12_pid.jsonl.gz','e8de_regions11_12_status.jsonl.gz','E8DE_LAST3_SCAN_GEOMETRY.json','analyze_e8de_compact.py']})
save('E8DE_FRAME_AND_STOP_DIAGNOSIS.json',result)
print(json.dumps(dict(frame=result['frame_calibration']|{'initial_pairs':'seeJSON'},periods=periods,no_path=result['after_first_degenerate_running_statuses'],pathgeometry=geometry),ensure_ascii=False))
