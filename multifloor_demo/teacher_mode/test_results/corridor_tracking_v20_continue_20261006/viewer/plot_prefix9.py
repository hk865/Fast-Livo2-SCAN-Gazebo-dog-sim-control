#!/usr/bin/env python3
"""Scientific summary from the bound compact series, never large raw logs."""
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent;BASE=HERE.parent
NAME='20261006_134714_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r3_401b'
SERIES=BASE/(NAME+'_COMPACT_SERIES.json.gz')
REVIEW=BASE/(NAME+'_REGION9_COMPARISON_REVIEWED.json')
REPORT=BASE/(NAME+'_PREFIX9_ACTUAL_EVALUATION.json')
PINS={SERIES:'18553e168f128d40b8e281835aebb0d6e03b02162239bef1a211d89f1787ee17',
    REVIEW:'524c79f9b04df2ae00c52897b7d13fe7b16e9f1a9038e003f93a17586158b54e',
    REPORT:'dc4894e4d19ca1eeaa61c0e2cec48d6b8880cef72b47d3a3df961db2e534cf13'}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for path,expected in PINS.items():
    if sha(path)!=expected:raise ValueError('Plot input binding differs: '+str(path))
with gzip.open(SERIES,'rt')as source:series=json.load(source)
review=json.loads(REVIEW.read_text());report=json.loads(REPORT.read_text())
candidate=report['run'];baseline=report['r3_comparison']['baseline_run']
pc={x:i for i,x in enumerate(series['pid_columns'])};qc={x:i for i,x in enumerate(series['pose_columns'])}
colors={'V20':'#087e8b','V19':'#cd5c2e'}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,
    'axes.spines.right':False,'axes.grid':True,'grid.alpha':.2,'svg.fonttype':'none'})
fig=plt.figure(figsize=(15,10),facecolor='white')
grid=fig.add_gridspec(2,2,hspace=.35,wspace=.24,left=.075,right=.94,bottom=.145,top=.865)
axpath=fig.add_subplot(grid[0,0]);axspeed=fig.add_subplot(grid[1,0]);axpark=fig.add_subplot(grid[1,1])
heading_grid=grid[0,1].subgridspec(3,1,height_ratios=[6,1,1],hspace=.08)
axhead=fig.add_subplot(heading_grid[0]);mode_axes=[fig.add_subplot(heading_grid[i],sharex=axhead)for i in (1,2)]
plt.setp(axhead.get_xticklabels(),visible=False);plt.setp(mode_axes[0].get_xticklabels(),visible=False)
counts={}
for label,run,key,band in [('V19',baseline,'baseline',0),('V20',candidate,'candidate',1)]:
    interval=review[key]['observed_interval_s'];start,end=interval;data=series['runs'][run]
    poses=[r for r in data['poses']if start<=r[qc['pose_stamp_s']]<=end]
    pid=[r for r in data['pid']if r[pc['goal_index']]==8 and start<=r[pc['clock_s']]<=end]
    a=np.asarray([r[1:4]for r in poses],dtype=float);t=np.asarray([r[pc['clock_s']]-start for r in pid])
    axpath.plot(a[:,0],a[:,1],color=colors[label],lw=1.6,label=label+' actual SLAM')
    axpath.scatter(a[0,0],a[0,1],color=colors[label],s=24,zorder=4)
    axpath.scatter(a[-1,0],a[-1,1],color=colors[label],marker='s',s=35,zorder=4)
    goal=np.asarray(data['goals'][8]['center']);axpath.scatter(goal[0],goal[1],marker='*',s=130,color=colors[label],zorder=5)
    errors=np.array([abs(r[pc['inner_heading_error_rad']])if r[pc['inner_heading_error_rad']]is not None else np.nan for r in pid])
    axhead.plot(t,errors,color=colors[label],lw=1.2,label=label)
    command=np.array([np.hypot(r[pc['cmd_vx_mps']],r[pc['cmd_vy_mps']])if r[pc['cmd_vx_mps']]is not None and r[pc['cmd_vy_mps']]is not None else np.nan for r in pid])
    measured=np.array([np.hypot(r[pc['slam_origin_vx_mps']],r[pc['slam_origin_vy_mps']])if r[pc['slam_origin_vx_mps']]is not None and r[pc['slam_origin_vy_mps']]is not None else np.nan for r in pid])
    axspeed.plot(t,measured,color=colors[label],lw=.9,alpha=.8,label=label+' SLAM origin speed')
    axspeed.plot(t,command,color=colors[label],ls='--',lw=1.4,label=label+' command norm')
    modes=[r[pc['heading_phase']]for r in pid];mode_colors={'drive':'#087e8b','pre_turn':'#bbc4cb','align':'#cd5c2e','settle':'#edc362'}
    axis=mode_axes[band]
    for i in range(len(pid)-1):
        if 0<t[i+1]-t[i]<=.200000001:axis.axvspan(t[i],t[i+1],color=mode_colors.get(modes[i],'#bca4ce'),lw=0)
    axis.set_yticks([]);axis.set_ylabel(label,rotation=0,labelpad=22,va='center');axis.grid(False)
    counts[label]=dict(raw_slam_points=len(poses),pid_rows=len(pid),heading_error_samples=int(np.isfinite(errors).sum()))
axpath.set(title='A  Region 9: recorded SLAM trajectory',xlabel='camera_init x (m)',ylabel='camera_init y (m)')
axpath.legend(loc='lower right',frameon=True,fontsize=9)
axpath.text(.02,.035,'Circle: start  |  square: end  |  star: target\nEach run retains its independent SLAM registration.',transform=axpath.transAxes,fontsize=8.5,va='bottom')
axpath.set_ylim(1.72,2.08)
axhead.set(title='B  Heading error and recorded heading-gate phase',ylabel='|Heading error| (rad)',xlim=(0,91))
axhead.axhline(.2,color='#525c66',ls=':',lw=1.2,label='0.2 rad gate')
axhead.legend(loc='upper right',fontsize=8.5,ncol=3)
mode_axes[-1].set_xlabel('Time since original region-9 activation (s)')
mode_axes[0].text(.02,.5,'pre-turn: gray   align: orange   settle: gold   drive: teal',transform=mode_axes[0].transAxes,fontsize=7.5,va='center',bbox=dict(facecolor='white',alpha=.75,edgecolor='none'))
axspeed.set(title='C  Command and measured SLAM origin speed',xlabel='Time since original region-9 activation (s)',ylabel='Planar speed (m/s)',xlim=(0,91))
axspeed.legend(loc='upper right',fontsize=8.5,ncol=2)
start,end=[v/1e9 for v in report['first5s_parking']['clock_interval_ns']]
origin_stamp=report['first5s_parking']['origin_pose_stamp_ns']/1e9
rows=series['runs'][candidate]['poses'];origin=next(r for r in rows if r[0]==origin_stamp)
park=[r for r in rows if start<=r[0]<=end]
a=np.asarray(park,dtype=float);time=a[:,0]-start
drift=np.linalg.norm(a[:,1:3]-np.asarray(origin[1:3]),axis=1)*1000
def yaw(q):
    x,y,z,w=q
    return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
y0=yaw(origin[4:8]);yaw_drift=np.abs(np.array([np.arctan2(np.sin(yaw(r[4:8])-y0),np.cos(yaw(r[4:8])-y0))for r in park]))
axpark.plot(time,drift,color=colors['V20'],lw=1.7,label='SLAM XY drift')
other=axpark.twinx();other.grid(False);other.spines['right'].set_visible(True)
other.plot(time,yaw_drift,color='#6e51a3',lw=1.4,label='SLAM yaw drift')
other.set_ylabel('Yaw drift (rad)',color='#6e51a3');other.tick_params(axis='y',labelcolor='#6e51a3');other.set_ylim(0,.005)
axpark.set(title='D  First declared 5 s parking window (V20)',xlabel='Time from first active-hold declaration (s)',ylabel='XY drift (mm)',xlim=(0,5),ylim=(0,8))
axpark.text(.03,.95,'Recorded maxima: 6.55 mm XY / 0.00366 rad yaw\nLimits: 50 mm XY / 0.100 rad yaw',transform=axpark.transAxes,va='top',fontsize=9,
    bbox=dict(facecolor='white',edgecolor='#d1d7dc',alpha=.9))
fig.suptitle('V19 full mission vs V20 prefix9: different terminal policy',fontsize=17,fontweight='bold',y=.965)
fig.text(.5,.913,'Region 9: V19 timed out at 90.165 s (104 pre-arrival resets); V20 arrived in 23.625 s (0 pre-arrival resets).',ha='center',fontsize=11)
fig.text(.075,.035,'V20 ends and parks at region 9; V19 was intended to continue. These are observed runs, not a matched terminal-policy experiment.\nSources: bound compact PID/SLAM series only. No interpolation or fabricated samples. Limited prefix9 PASS; full46 and all 200 Hz physics remain unverified.',fontsize=9,color='#44515c')
files=[]
for ext in ('png','svg'):
    target=HERE/('V20_R3_PREFIX9_COMPARISON.'+ext);fig.savefig(target,dpi=180,metadata={'Creator':'Independent compact-series audit'} if ext=='svg' else None);files.append(target)
receipt=dict(schema='prefix9_scientific_plot/v1',inputs={str(k):v for k,v in PINS.items()},
    script_sha256=sha(__file__),outputs={str(f):sha(f)for f in files},counts=counts,
    raw_runtime_logs_read=False,missing_samples_filled=False,parking_points=len(park),
    parking_xy_drift_max_m=float(drift.max()/1000),parking_yaw_drift_max_rad=float(yaw_drift.max()),
    first5s_clock_interval_s=[start,end],full46_pass=False,whole_200hz_physics_verified=False)
(HERE/'PLOT_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt,indent=2))
