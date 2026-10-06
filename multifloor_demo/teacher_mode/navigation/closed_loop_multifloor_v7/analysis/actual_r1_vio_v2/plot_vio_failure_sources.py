import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS'):os.environ[k]='1'
from pathlib import Path
import csv,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/closed_loop_multifloor_v7/analysis/actual_r1_vio_v2')
def read(n):return[{k:float(v)if v else np.nan for k,v in a.items()}for a in csv.DictReader((p/n).open())]
s=read('estimator_stages.csv');l=[a for a in s if a['stage']==2 and 120<=a['stamp_ns']/1e9<=140];v=[a for a in s if a['stage']==1 and 120<=a['stamp_ns']/1e9<=140]
li=read('lio_information_comparison.csv');vi=read('vio_iterations.csv');rr=read('vio_retrieval.csv')
f,ax=plt.subplots(4,1,figsize=(12,12),sharex=True)
for rows,k,label,color in [(l,'z_final','post LIO','#d1495b'),(v,'z_final','post VIO','#149c7b')]:ax[0].plot([a['stamp_ns']/1e9 for a in rows],[a[k]for a in rows],label=label,c=color,lw=1.7)
ax[0].set_ylabel('IMU state Z (camera_init) (m)');ax[0].legend();ax[0].set_title('Actual V7 source records: height failure 131–134 s; first guard at clock 133.13 s / source 133.1 s')
for rows,k,label,color in [(l,'vz_predicted','IMU predicted vz','#4169a1'),(l,'vz_final','post LIO vz','#d1495b'),(v,'vz_final','post VIO vz','#149c7b')]:ax[1].plot([a['stamp_ns']/1e9 for a in rows],[a[k]for a in rows],label=label,c=color,lw=1.3)
ax[1].set_ylabel('estimated world vz (m/s)');ax[1].legend(ncol=3)
for rows,k,label,color in [(l,'propagation_dz','propagation','#4169a1'),(l,'measurement_dz','LIO correction','#d1495b'),(v,'measurement_dz','VIO correction','#149c7b')]:ax[2].plot([a['stamp_ns']/1e9 for a in rows],[a[k] * 1000 for a in rows],label=label,c=color,lw=1.3)
ax[2].set_ylabel('actual stage delta Z (mm)');ax[2].legend(ncol=3)
rt=[a for a in rr if 120<=a['stamp_ns']/1e9<=140];ax[3].plot([a['stamp_ns']/1e9 for a in rt],[a['total_points']for a in rt],label='accepted visual tracks',c='#7f5aa2',lw=1.7);ax[3].set_ylabel('actual accepted tracks')
second=ax[3].twinx()
for rows,label,color in [(li,'LIO conditional Z information','#d1495b'),(vi,'VIO conditional Z information','#149c7b')]:
 by={}
 for a in rows:
  if 120<=a['stamp_ns']/1e9<=140 and a.get('schur_z',0)>0:by.setdefault(a['stamp_ns']/1e9,[]).append(a['schur_z'])
 second.semilogy(sorted(by),[np.median(by[t])for t in sorted(by)],label=label,c=color,lw=1)
second.set_ylabel('conditional Z information, per-stamp median');lines1,labels1=ax[3].get_legend_handles_labels();lines2,labels2=second.get_legend_handles_labels();ax[3].legend(lines1+lines2,labels1+labels2,loc='lower left',fontsize=8)
for a in ax:a.axvspan(131,134,color='#f8bb52',alpha=.18);a.grid(alpha=.22);a.set_xlim(120,140)
ax[-1].set_xlabel('actual absolute estimator source stamp (s)')
f.text(.5,.007,'Original solver states only. No native pose used. Information/rank are offline diagnostics, not a full fused observability or association certificate.',ha='center',fontsize=9)
f.tight_layout(rect=[0,.025,1,1]);f.savefig(p/'vio_failure_sources.png',dpi=170);plt.close(f)
