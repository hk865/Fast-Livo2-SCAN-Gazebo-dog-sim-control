#!/usr/bin/env python3
"""Offline native/body-vs-published SLAM comparison; truth never feeds navigation."""
from pathlib import Path
import argparse,csv,json,struct
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
p=args.run.resolve();out=args.output;out.mkdir(parents=True,exist_ok=True)
rows=[]
with (p/'actuator.jsonl').open() as stream:
 for line in stream:
  d=json.loads(line)
  if d.get('kind')!='physics_step':continue
  rows.append([d['t']-d['dt'],*d['position'],*d['quaternion_wxyz'],*d['body_lin_vel_origin'],*d['body_ang_vel'],d['contacts'][0],d['fault']])
n=np.array(rows);del rows
R=Rotation.from_quat(n[:,[5,6,7,4]]);rpy=R.as_euler('xyz');vw=R.apply(n[:,8:11])
raw=[json.loads(l) for l in (p/'navigation_slam_poses.jsonl').open()];s=np.array([[d['stamp_ns']*1e-9,*d['position'],*d['quaternion'],*d['body_velocity'],d['sim_age_at_callback_s']] for d in raw]);del raw
srpy=Rotation.from_quat(s[:,4:8]).as_euler('xyz');svw=Rotation.from_quat(s[:,4:8]).apply(s[:,8:11])
reg=json.loads((p/'navigation_scene_axis_registration.json').read_text());pair=reg['exact_paired_sample_records'][-1]
t0=pair['slam_stamp_ns']*1e-9;axis=np.array(reg['heading_receipt']['rotation_camera_init_from_world']);anchor=np.array(pair['slam_source']['position']);np0=np.array([np.interp(t0,n[:,0],n[:,i]) for i in [1,2,3]])
nt=(n[:,1:4]-np0)@axis.T+anchor
commands=[];failure=None
for line in (p/'navigation_command_history.jsonl').open():
 d=json.loads(line)
 if failure is None and d['state']=='failed':failure={k:d.get(k) for k in ['sim_time','slam_stamp_ns','reason','registered_route_fence','ages']}
 commands.append([d['sim_time'],*d['command'],0 if d['state']=='drive' else 1])
c=np.array(commands)
ft=failure['slam_stamp_ns']*1e-9 if failure else 156.4
# Explicit diagnostic window: 5s ending at first fence source; reference window 120–125s.
windows={name:(a,b) for name,a,b in [('before_failure',ft-5,ft),('broad_platform',130,ft),('old_V6_window',128.4,131.7),('exact_solver_window',131,134)]}
summary={'schema':'offline_physical_height_fault/v1','run':str(p),'truth_is_offline_only':True,'comparison_alignment':'frozen sensor yaw axes; native/body and actual SLAM body paired at final frozen calibration sample for offline relative comparison only','anchor_time_s':t0,'first_failed_envelope':failure,'windows':{}}
for name,(a,b)in windows.items():
 mask=(n[:,0]>a)&(n[:,0]<=b);ms=(s[:,0]>a)&(s[:,0]<=b);nn=n[mask];ss=s[ms]
 ni=np.array([np.interp(ss[:,0],n[:,0],nt[:,j])for j in range(3)]).T
 zerr=ss[:,3]-ni[:,2]
 summary['windows'][name]={'window_start_exclusive_end_inclusive_s':[a,b],'native_sample_count':int(mask.sum()),'native_delta_z_m':float(nn[-1,3]-nn[0,3]),'native_z_range_m':float(np.ptp(nn[:,3])),'native_roll_pitch_max_rad':np.max(np.abs(rpy[mask,:2]),axis=0).tolist(),'native_world_vz_min_max_mps':[float(vw[mask,2].min()),float(vw[mask,2].max())],'native_body_contact_max':float(nn[:,14].max()),'native_fault_max':float(nn[:,15].max()),'SLAM_delta_z_m':float(ss[-1,3]-ss[0,3]),'SLAM_roll_pitch_max_rad':np.max(np.abs(srpy[ms,:2]),axis=0).tolist(),'SLAM_world_vz_min_max_mps':[float(svw[ms,2].min()),float(svw[ms,2].max())],'SLAM_native_relative_z_error_start_end_m':[float(zerr[0]),float(zerr[-1])],'SLAM_source_callback_age_peak_s':float(ss[:,11].max())}
# Save compact physical trace without bringing native data to any control process.
header=['t','world_x','world_y','world_z','slam_aligned_native_x','slam_aligned_native_y','slam_aligned_native_z','roll','pitch','yaw','vx_world','vy_world','vz_world','body_contact','fault']
trace=np.c_[n[:,0:4],nt,rpy,vw,n[:,14:16]]
np.savetxt(out/'native_trace_200hz.csv',trace,delimiter=',',header=','.join(header),comments='')
np.savetxt(out/'slam_body_trace.csv',np.c_[s[:,0:4],srpy,svw,s[:,11]],delimiter=',',header='t,x,y,z,roll,pitch,yaw,vx_world,vy_world,vz_world,callback_age',comments='')
fig,axs=plt.subplots(4,1,figsize=(13,12),sharex=True)
axs[0].plot(n[:,0],nt[:,2],label='Gazebo body (offline aligned)');axs[0].plot(s[:,0],s[:,3],label='Published SLAM body');axs[0].set_ylabel('Height [m]');axs[0].legend()
axs[1].plot(n[:,0],vw[:,2],label='Actual world vz');axs[1].plot(s[:,0],svw[:,2],label='SLAM world vz');axs[1].set_ylabel('Vertical speed [m/s]');axs[1].legend()
axs[2].plot(n[:,0],np.rad2deg(rpy[:,0]),label='Actual roll');axs[2].plot(n[:,0],np.rad2deg(rpy[:,1]),label='Actual pitch');axs[2].plot(s[:,0],np.rad2deg(srpy[:,0]),'--',label='SLAM roll');axs[2].plot(s[:,0],np.rad2deg(srpy[:,1]),'--',label='SLAM pitch');axs[2].set_ylabel('Attitude [deg]');axs[2].legend(ncol=4)
axs[3].plot(c[:,0],c[:,1],label='Teacher vx command');axs[3].plot(c[:,0],c[:,3],label='Teacher wz command');axs[3].set_ylabel('Command [m/s, rad/s]');axs[3].set_xlabel('Original simulation/source time [s]');axs[3].legend()
for ax in axs:
 ax.axvline(ft,color='red',ls=':',label='First fence source');ax.grid(alpha=.2);ax.set_xlim(110,175)
fig.suptitle('V7 actual run: native truth is OFFLINE ONLY; SLAM/IMU/SCAN supplies navigation');fig.tight_layout();fig.savefig(out/'actual_height_velocity_attitude.png',dpi=160);plt.close(fig)
(out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+'\n');print(json.dumps(summary,ensure_ascii=False))
