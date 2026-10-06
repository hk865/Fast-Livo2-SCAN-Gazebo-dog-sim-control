#!/usr/bin/env python3
"""Scientific plots from saved compact files only."""
from pathlib import Path
import json,gzip,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).resolve().parent
rows=lambda n:[json.loads(x)for x in gzip.open(O/n,'rt')]
j=rows('e8de_same_time_frame_join.jsonl.gz');s=rows('e8de_regions11_12_status.jsonl.gz');n=rows('e8de_native50Hz_compact.jsonl.gz');r=json.loads((O/'E8DE_FRAME_AND_STOP_DIAGNOSIS.json').read_text())
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})
t=np.array([x['stamp_ns']/1e9 for x in j]);zs=np.array([x['position'][2]for x in j]);zn=np.array([x['native_position_in_fixed_initial_camera_frame'][2]for x in j])
fig,ax=plt.subplots(figsize=(10,5.5));fig.subplots_adjust(bottom=.23,top=.84,right=.97,left=.1)
ax.plot(t,zs,lw=1.6,color='#0072b2',label='Actual SLAM z (camera_init)')
ax.plot(t,zn,lw=1.6,color='#d55e00',label='Native base_link z mapped by fixed initial SE3')
ax.axhline(1.2034672279979741,color='#555',ls=':',lw=1.2,label='Original region 12 center z')
ax.axvline(220.31,color='#991b1b',ls='--',lw=1.2,label='First status with rejected stop spline')
ax.set(xlim=(195,232),ylim=(.965,1.29),xlabel='Actual source / status simulation time (s)',ylabel='Body-origin height in camera_init (m)');ax.grid(alpha=.2);ax.legend(loc='lower left',fontsize=9)
fig.suptitle('V25 e8de: estimated height drops while physical base height stays stable',fontsize=13,y=.96)
fig.text(.1,.075,'Initial alignment uses 26 measured pairs (5.745–6.570 s); then remains fixed. Native snapshots: 50 Hz.\nPosition interpolation brackets <=20 ms; no COM position subtraction. Offline diagnosis only; original full46 result: FAILED, 11/46.',fontsize=9,va='center')
for ext in ('png','svg'):fig.savefig(O/('E8DE_HEIGHT_FIXED_FRAME.'+ext),dpi=180)
plt.close(fig)
rr=[x for x in s if x['waypoint_index']==11];tt=np.array([x['ros_sim_time']for x in rr]);cmd=np.array([x['command']for x in rr]);dd=np.array([x['degenerate_splines']for x in rr]);nn=[x for x in n if 198<=x['state_physics_world_time']<=296.7]
fig,(a,b)=plt.subplots(2,1,figsize=(10,6.7),sharex=True,gridspec_kw={'height_ratios':[2,1]});fig.subplots_adjust(bottom=.19,top=.87,hspace=.18,right=.97,left=.1)
a.plot(tt,cmd[:,0],lw=1.5,color='#0072b2',label='Published forward command vx')
a.plot([x['state_physics_world_time']for x in nn],[x['body_lin_vel'][0]for x in nn],lw=.75,alpha=.75,color='#d55e00',label='Native measured body COM vx (50 Hz)')
a.set(ylabel='Forward speed (m/s)');a.legend(loc='upper right',fontsize=9);a.grid(alpha=.2)
b.step(tt,dd,where='post',color='#6b21a8',lw=1.5,label='Rejected degenerate splines (cumulative)');b.set(ylabel='Rejected splines',xlabel='Recorded status / native physics simulation time (s)',xlim=(198,296.7),ylim=(-1,36));b.grid(alpha=.2);b.legend(loc='upper left',fontsize=9)
for ax in (a,b):
 ax.axvline(220.31,color='#991b1b',ls='--',lw=1.2);ax.axvline(289.61,color='#555',ls=':',lw=1.2);ax.axvspan(220.31,289.61,color='#991b1b',alpha=.055)
a.text(221.3,.22,'No accepted path; zero commands',color='#991b1b',fontsize=10)
fig.suptitle('V25 e8de, region 12: repeated stop splines until the original 90 s timeout',fontsize=13,y=.96)
fig.text(.1,.058,'Dashed: first recorded rejection at 220.31 s. Dotted: first failed status at 289.61 s.\n34 rejections; 457 subsequent running statuses have exactly zero command and no outer obstacle_hold.\nLast three NPZ archives contain valid moving paths; rejected stop-spline coefficients were not archived.',fontsize=9,va='center')
for ext in ('png','svg'):fig.savefig(O/('E8DE_REGION12_COMMAND_STOP.'+ext),dpi=180)
plt.close(fig)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
f=O/'PLOT_PROVENANCE.json';assert not f.exists();f.write_text(json.dumps({'schema':'e8de_compact_only_scientific_plots/v1','original_raw_read':False,'input_sha256':{str(O/x):sha(O/x)for x in ['e8de_same_time_frame_join.jsonl.gz','e8de_regions11_12_status.jsonl.gz','e8de_native50Hz_compact.jsonl.gz','E8DE_FRAME_AND_STOP_DIAGNOSIS.json','plot_e8de_compact.py']},'output_sha256':{str(p):sha(p)for p in sorted(O.glob('E8DE_*.png'))+sorted(O.glob('E8DE_*.svg'))}},indent=2)+'\n')
print('PLOTS_COMPLETE')
