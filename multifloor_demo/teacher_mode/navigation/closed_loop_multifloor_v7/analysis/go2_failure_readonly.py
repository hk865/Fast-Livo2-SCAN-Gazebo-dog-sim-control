import os
for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
from pathlib import Path
import numpy as np,json,re
from scipy.spatial.transform import Rotation,Slerp
base=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs')
runs=['20261005_145640_closed_loop_cascade_actual_multifloor_r1_d42e','20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182','20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6']
windows=[(120,128),(128,131),(131,134),(134,140),(140,145)]
summary={'scope':'existing archive read only; native only offline comparison; no solver/SLAM input','raw_acceleration_archived':False,'imu_debug_semantics':'head/tail mean, raw scale before bias correction; world transformed below uses archived IMU orientation offline only, not SLAM or navigation'}
def stream(path,fn,cutoff=146):
 for l in path.open():
  a=json.loads(l);t=fn(a)
  if t>cutoff:break
  yield a
for name in runs:
 p=base/name; out={}
 imu=np.loadtxt(p/'fastlivo_debug/imu.txt');imu=imu[imu[:,0]+.01<146]
 # retain one record per head timestamp; repeated endpoints do not count twice
 _,ix=np.unique(imu[:,0],return_index=True);imu=imu[np.sort(ix)];imu[:,0]+=.01
 records=list(stream(p/'navigation_imu_history.jsonl',lambda a:a['stamp_ns']/1e9))
 qt=np.array([a['stamp_ns']/1e9 for a in records]);qr=Rotation.from_quat([a['orientation_xyzw'] for a in records]);rot=Slerp(qt,qr)
 good=(imu[:,0]+.0025>=qt[0])&(imu[:,0]+.0025<=qt[-1]);imu=imu[good]
 accworld=rot(imu[:,0]+.0025).apply(imu[:,4:7]);accworld[:,2]-=9.81
 pre=np.loadtxt(p/'fastlivo_debug/mat_pre.txt');post=np.loadtxt(p/'fastlivo_debug/mat_out.txt');pd={};od={}
 for a in pre:pd.setdefault(float(a[0]),[]).append(a)
 for a in post:od.setdefault(float(a[0]),[]).append(a)
 updates=[];previous=None
 for t,group in sorted(od.items()):
  if t+.01>146:break
  lio=next((a for a in group if a[19]>0),None)
  if t not in pd or lio is None:continue
  pred=pd[t][0];res=group[-1]
  if previous is not None:updates.append([t+.01,pred[6]-previous[6],lio[6]-pred[6],res[6]-lio[6],pred[9]-previous[9],lio[9]-pred[9],res[9]-lio[9],pred[9],lio[9],res[9],lio[19]])
  previous=res
 updates=np.array(updates)
 tele=[]
 for a in stream(p/'telemetry.jsonl',lambda a:a['world_sim_time']):
  if a['world_sim_time']<119:continue
  tele.append([a['state_physics_world_time'],*a['position'],*a['rpy'],*a['body_ang_vel'],*a['body_lin_vel'],*a['command'],a['contacts'].get('body',0),sum(v>0 for k,v in a['contacts'].items() if k!='body'),a['fault'] is not None,a['inference_ms'],a['state']])
 nt=np.array([a[:-1] for a in tele],dtype=float); states=[a[-1] for a in tele]
 clouds=list(stream(p/'navigation_cloud_history.jsonl',lambda a:a['stamp_ns']/1e9))
 poses=list(stream(p/'navigation_slam_poses.jsonl',lambda a:a['stamp_ns']/1e9))
 sync=[];slices=[]
 for l in (p/'navigation_stack.log').open():
  if '[DEMO_SYNC]' in l:
   a=dict(re.findall(r'(camera|lidar_newest|imu_newest|imu_last_used|imu_count|complete)=(-?[\d.]+)',l));sync.append(a)
  if '[DEMO_LIDAR_SLICE]' in l:
   a=dict(re.findall(r'(camera|previous|points|offset_min_ms|offset_max_ms|pending)=(-?[\d.]+)',l));slices.append(a)
 metrics=[]
 for lo,hi in windows:
  mi=(imu[:,0]>=lo)&(imu[:,0]<hi);mu=(updates[:,0]>lo)&(updates[:,0]<=hi);mn=(nt[:,0]>=lo)&(nt[:,0]<hi)
  mc=[a for a in clouds if lo<=a['stamp_ns']/1e9<hi];mp=[a for a in poses if lo<=a['stamp_ns']/1e9<hi]
  sy=[a for a in sync if lo<=float(a['camera'])<hi];sl=[a for a in slices if lo<=float(a['camera'])<hi]
  zn=nt[mn,3];nrp=nt[mn,4:6];acc=imu[mi,4:7];an=np.linalg.norm(acc,axis=1);aw=accworld[mi];upd=updates[mu]
  cm=np.array([a['sim_age_at_callback_s'] for a in mc]);pm=np.array([a['sim_age_at_callback_s'] for a in mp]);count=nt[mn]
  metrics.append({'window_s':[lo,hi], 'debug_avg_acc_body_mean_std_xyz':np.stack([acc.mean(0),acc.std(0)]).tolist(),'debug_avg_acc_norm_mean_std_min_max_mps2':[float(an.mean()),float(an.std()),float(an.min()),float(an.max())],'debug_avg_acc_world_minus_gravity_mean_std_xyz_offline':np.stack([aw.mean(0),aw.std(0)]).tolist(),'debug_avg_gyro_peak_norm_radps':float(np.linalg.norm(imu[mi,1:4],axis=1).max()),'dz_sum_predict_LIO_VIO_m':upd[:,1:4].sum(0).tolist(),'dvz_sum_predict_LIO_VIO_mps':upd[:,4:7].sum(0).tolist(),'vz_pred_LIO_last_minmax_mps':[[float(upd[:,k].min()),float(upd[:,k].max())] for k in [7,8,9]],'raw_lidar_points_minmax':[int(upd[:,10].min()),int(upd[:,10].max())], 'native_offline_z_first_last_min_max_m':[float(zn[0]),float(zn[-1]),float(zn.min()),float(zn.max())],'native_offline_abs_roll_pitch_peak_deg':np.rad2deg(np.abs(nrp).max(0)).tolist(),'native_offline_body_vz_mean_std_min_max_mps':[float(count[:,12].mean()),float(count[:,12].std()),float(count[:,12].min()),float(count[:,12].max())], 'native_offline_body_contacts_count':int((count[:,16]>0).sum()),'native_offline_any_foot_contact_fraction':float((count[:,17]>0).mean()),'worker_fault_frames':int(count[:,18].sum()),'native_gyro_peak_norm_radps':float(np.linalg.norm(count[:,7:10],axis=1).max()),'cmd_vx_minmax':[float(count[:,13].min()),float(count[:,13].max())],'cmd_wz_minmax':[float(count[:,15].min()),float(count[:,15].max())],'inference_ms_median_max':[float(np.median(count[:,19])),float(count[:,19].max())],'cloud_callback_sim_age_min_median_max_s':[float(cm.min()),float(np.median(cm)),float(cm.max())],'pose_callback_sim_age_min_median_max_s':[float(pm.min()),float(np.median(pm)),float(pm.max())],'cloud_pose_stamp_mismatch_count':sum(a['stamp_ns']!=a['nearest_slam_pose_stamp_ns'] for a in mc),'sync_incomplete_count':sum(int(a['complete'])!=1 for a in sy),'sync_imu_last_used_difference_peak_ms':max(abs(float(a['camera'])-float(a['imu_last_used']))*1000 for a in sy),'sync_imu_count_minmax':[min(int(a['imu_count']) for a in sy),max(int(a['imu_count']) for a in sy)],'instantaneous_slice_bad_count':sum(abs(float(a['offset_min_ms'])-float(a['offset_max_ms']))>1e-6 or abs(float(a['offset_min_ms'])-1000*(float(a['camera'])-float(a['previous'])))>2e-5 or int(a['pending'])!=0 for a in sl)})
 out['windows']=metrics;summary[name]=out
 print(name, json.dumps(metrics[2],ensure_ascii=False),flush=True)
Path('/tmp/go2_failure_readonly_metrics.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
