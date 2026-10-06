import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS'):os.environ[k]='1'
from pathlib import Path
import csv,json,numpy as np
p=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/closed_loop_multifloor_v7/analysis/actual_r1_vio_v2')
def read(n):return [{k:float(v) if v not in ('','None') else np.nan for k,v in a.items()} for a in csv.DictReader((p/n).open())]
ph=read('estimator_stages.csv');lio=read('lio_information_comparison.csv');vi=read('vio_iterations.csv');im=read('imu_pair_propagation.csv');ret=read('vio_retrieval.csv');tr=read('vio_tracks.csv');raw=read('raw_imu.csv')
def st(vals):
 a=np.array(vals);a=a[np.isfinite(a)]
 return {'n':len(a),'min':float(a.min())if len(a)else None,'median':float(np.median(a))if len(a)else None,'mean':float(a.mean())if len(a)else None,'max':float(a.max())if len(a)else None}
def choose(rows,lo,hi):return[a for a in rows if lo*1e9<a['stamp_ns']<=hi*1e9]
res={'scope':'actual source diagnostic and previously derived CSV only; offline selected event windows, no original acceptance or runtime changes','windows':[]}
for lo,hi in [(120,125),(125,130),(130,131),(131,134),(134,140),(140,156.4)]:
 pp=choose(ph,lo,hi);ll=choose(lio,lo,hi);vv=choose(vi,lo,hi);ii=choose(im,lo,hi);rr=choose(ret,lo,hi);tt=choose(tr,lo,hi);aa=choose(raw,lo,hi)
 out={'window_start_exclusive_end_inclusive_s':[lo,hi],'phases':{}}
 for stage,n in [(2,'LIO'),(1,'VIO')]:
  rows=[a for a in pp if a['stage']==stage]
  out['phases'][n]={'n':len(rows),'prop_dz_m':sum(a['propagation_dz']for a in rows),'measurement_dz_m':sum(a['measurement_dz']for a in rows),'prop_dvz_mps':sum(a['propagation_dvz']for a in rows),'measurement_dvz_mps':sum(a['measurement_dvz']for a in rows),'vz_predicted':st([a['vz_predicted']for a in rows]),'P_pz_vz_predicted':st([a['P_pz_vz_predicted']for a in rows])}
 out.update(propagation_dz_velocity_m=sum(a['dz_velocity']for a in ii),propagation_dz_acceleration_m=sum(a['dz_acceleration']for a in ii),propagation_world_az_mean_std=[float(np.mean([a['world_az']for a in ii])),float(np.std([a['world_az']for a in ii]))],raw_acc_body_xyz_mean_std=[[float(np.mean([a[k]for a in aa]))for k in ['ax','ay','az']],[float(np.std([a[k]for a in aa]))for k in ['ax','ay','az']]],raw_acc_norm=st([a['accel_norm']for a in aa]),raw_gyro_norm=st([a['gyro_norm']for a in aa]),tracks=st([a['total_points']for a in rr]),ref_age_s=st([a['reference_age_s']for a in tt]),selected_grid=st([a['selected_grid']for a in rr]),reject_depth=sum(a['reject_depth']for a in rr),reject_SSE=sum(a['reject_SSE']for a in rr),reject_normal=sum(a['reject_normal']for a in rr),LIO_Mzz=st([a['Mzz']for a in ll]),LIO_schur_z=st([a['schur_z']for a in ll]),LIO_effective=st([a['effective_count']for a in ll]),VIO_Mzz=st([a['Mzz']for a in vv]),VIO_schur_z=st([a['schur_z']for a in vv]),VIO_candidate_mean_squared_error=st([a['error_candidate']for a in vv]),VIO_accepted=sum(a['accepted']== 1 for a in vv),VIO_rollback=sum(a['accepted']== 0 for a in vv))
 res['windows'].append(out)
 print(lo,hi,json.dumps(out,ensure_ascii=False))
(p/'failure_window_131_134.json').write_text(json.dumps(res,indent=2)+'\n')
