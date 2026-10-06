#!/usr/bin/env python3
"""Plot compact same-pass OFF/ON evidence only; no raw-log reads or acceptance upgrade."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

SERIES_SCHEMA = 'curvature_prefix9_same_pass_series/v1'
DIAGNOSTIC_SCHEMA = 'curvature_prefix9_diagnostic/v1'
COLORS = {'OFF':'#2563eb', 'ON':'#d97706'}


def require(ok, why):
    if not ok: raise ValueError(why)


def number(value):
    return float(value) if isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) else np.nan


def item(row, field, i):
    v=row.get(field)
    return number(v[i]) if isinstance(v,list) and len(v)>i else np.nan


def drive(row):
    return row.get('fresh_math') is True and row.get('mode')=='drive' and row.get('phase')=='drive'


def metric(values):
    a=np.array([number(v) for v in values],dtype=float)
    a=a[np.isfinite(a)]
    if not len(a):
        return dict(n=0,mean=None,mean_abs=None,rms=None,abs_p95=None,peak_abs=None)
    return dict(n=len(a),mean=float(a.mean()),mean_abs=float(np.abs(a).mean()),
                rms=float(np.sqrt(np.mean(a*a))),abs_p95=float(np.percentile(np.abs(a),95)),
                peak_abs=float(np.max(np.abs(a))))


def summarize_rows(rows):
    rows=[r for r in rows if drive(r)]
    def raw_error(r,i):
        target=item(r,'reference_COM_vx_vy_wz',i)
        actual=item(r,'measured_SLAM_COM_velocity',i) if i<2 else item(r,'measured_body_omega',2)
        return target-actual
    def planar_speed_error(r):
        a=np.array([item(r,'reference_COM_vx_vy_wz',i) for i in (0,1)])
        b=np.array([item(r,'measured_SLAM_COM_velocity',i) for i in (0,1)])
        return float(np.linalg.norm(a)-np.linalg.norm(b)) if np.isfinite(a).all() and np.isfinite(b).all() else np.nan
    result=dict(fresh_math_drive_rows=len(rows),
        inner_heading_error_rad=metric(r.get('inner_heading_error_rad') for r in rows),
        cross_raw_m=metric(r.get('cross_raw_m') for r in rows),
        cross_control_m=metric(r.get('cross_control_m') for r in rows),
        planar_COM_speed_error_mps=metric(planar_speed_error(r) for r in rows),
        raw_SLAM_COM_vx_error_mps=metric(raw_error(r,0) for r in rows),
        raw_SLAM_COM_vy_error_mps=metric(raw_error(r,1) for r in rows),
        raw_IMU_body_z_rate_error_radps=metric(raw_error(r,2) for r in rows),
        recorded_filtered_PI_vx_error_mps=metric(item(r,'PI_error',0) for r in rows),
        recorded_filtered_PI_vy_error_mps=metric(item(r,'PI_error',1) for r in rows),
        recorded_filtered_PI_body_z_rate_error_radps=metric(item(r,'PI_error',2) for r in rows),
        curvature_FF_raw_radps=metric(r.get('curvature_FF_raw_radps') for r in rows))
    return result


def load(series_path, diagnostic_path):
    series_path,diagnostic_path=Path(series_path),Path(diagnostic_path)
    content=series_path.read_bytes()
    s=json.loads(gzip.decompress(content)); d=json.loads(diagnostic_path.read_text())
    require(s.get('schema')==SERIES_SCHEMA and d.get('schema')==DIAGNOSTIC_SCHEMA,'unsupported compact schema')
    require(s['run']==d['run'] and s['run_id']==d['run_id'] and s['condition']==d['condition'],'foreign diagnostic')
    require(s['condition'] in COLORS,'expected OFF or ON')
    require(d['series']['sha256']==hashlib.sha256(content).hexdigest(),'compact series SHA does not match diagnostic')
    require(s.get('navigation_ground_truth_used') is False,'navigation provenance unavailable')
    for key in ('pid','native','poses','activation_events'):
        require(isinstance(s.get(key),list),f'missing compact {key}')
    s['_input']=dict(series=str(series_path.absolute()),series_sha256=hashlib.sha256(content).hexdigest(),
                    diagnostic=str(diagnostic_path.absolute()),diagnostic_sha256=hashlib.sha256(diagnostic_path.read_bytes()).hexdigest())
    s['_diagnostic']=d
    return s


def points(rows,fn,time_fn=lambda r:number(r.get('clock_ns'))/1e9,mask=None,max_gap=.3):
    xs,ys=[],[]; previous=None
    for row in rows:
        t=time_fn(row); y=number(fn(row)) if mask is None or mask(row) else np.nan
        if not np.isfinite(t): continue
        if previous is not None and (t<=previous or t-previous>max_gap):
            xs.append(np.nan);ys.append(np.nan)
        xs.append(t);ys.append(y);previous=t
    return xs,ys


def draw(ax,rows,fn,label,**kwargs):
    x,y=points(rows,fn,**{k:kwargs.pop(k) for k in ['time_fn','mask','max_gap'] if k in kwargs})
    if any(np.isfinite(y)):
        ax.plot(x,y,label=label,**kwargs)
        return True
    return False


def regions(ax,s):
    starts=[]
    for e in s['activation_events']:
        d=e.get('data',{}); t=number(d.get('control_stamp_ns'))/1e9; goal=d.get('waypoint_index')
        if np.isfinite(t) and isinstance(goal,int): starts.append((t,goal+1))
    if not starts: return
    end=max([number(r.get('clock_ns'))/1e9 for r in s['pid']],default=starts[-1][0])
    for i,(t,region) in enumerate(starts):
        ax.axvline(t,color='#94a3b8',lw=.5,alpha=.55)
        right=starts[i+1][0] if i+1<len(starts) else end
        ax.text((t+right)/2,.99,f'R{region}',transform=ax.get_xaxis_transform(),
                ha='center',va='top',fontsize=7,color='#475569')


def format_axis(ax,s,label):
    regions(ax,s);ax.set_ylabel(label);ax.grid(alpha=.17)
    handles,_=ax.get_legend_handles_labels()
    if handles: ax.legend(loc='lower right',fontsize=7,ncol=2)


def save(fig,output,name):
    paths=[]
    for ext in ('png','svg'):
        path=output/f'{name}.{ext}'
        fig.savefig(path,dpi=165,bbox_inches='tight');paths.append(path)
    plt.close(fig)
    return paths


def plot_all(runs,output):
    n=len(runs); artifacts=[]
    fig,axs=plt.subplots(1,n,figsize=(6*n,5),squeeze=False)
    for col,s in enumerate(runs):
        ax=axs[0,col]; pc=s['pose_columns']; ix,iy=pc.index('x_m'),pc.index('y_m')
        arr=np.array(s['poses'],dtype=float)
        ax.plot(arr[:,ix],arr[:,iy],color=COLORS[s['condition']],lw=1,label='actual SLAM body origin')
        for e in s['activation_events']:
            d=e.get('data',{});p=d.get('segment_start');idx=d.get('waypoint_index')
            if isinstance(p,list) and len(p)>=2 and isinstance(idx,int):
                ax.scatter(p[0],p[1],s=15,color='#111827');ax.annotate(f'R{idx+1} start',p[:2],fontsize=7,xytext=(3,3),textcoords='offset points')
        ax.set_title(s['condition']+' | independent camera_init SLAM frame')
        ax.set_xlabel('SLAM x (m)');ax.set_ylabel('SLAM y (m)');ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Actual SLAM trajectories | each run keeps its own registration; no truth navigation')
    artifacts+=save(fig,output,'slam_xy')

    fig,axs=plt.subplots(3,n,figsize=(7*n,9),squeeze=False,sharex='col')
    for col,s in enumerate(runs):
        rs=s['pid']; color=COLORS[s['condition']]
        axs[0,col].set_title(s['condition']+' | fresh_math drive only')
        draw(axs[0,col],rs,lambda r:r.get('kappa_per_m'),'signed kappa',mask=drive,color=color,lw=.8)
        draw(axs[1,col],rs,lambda r:r.get('curvature_FF_raw_radps'),'recorded raw curvature FF',mask=drive,color=color,lw=.8)
        for key,label,c in [('requested_speed_mps','limiter requested','#64748b'),('limited_speed_mps','limiter selected',color)]:
            draw(axs[2,col],rs,lambda r,k=key:(r.get('speed_supervisor') or {}).get(k),label,mask=drive,color=c,lw=.9)
        if not s['settings']['curvature_speed_limit_enabled']:
            axs[2,col].text(.02,.4,'Limiter disabled: no requested/selected samples',transform=axs[2,col].transAxes,fontsize=9)
        for row,label in enumerate(['curvature (1/m)','FF (rad/s)','supervisor speed (m/s)']): format_axis(axs[row,col],s,label)
        axs[-1,col].set_xlabel('this run simulation clock (s)')
    fig.suptitle('Curvature reference and speed supervision | gaps are unavailable, never filled with zero')
    artifacts+=save(fig,output,'curvature_speed')

    fig,axs=plt.subplots(3,n,figsize=(7*n,10),squeeze=False,sharex='col')
    for col,s in enumerate(runs):
        rs=s['pid'];nr=s['native'];axs[0,col].set_title(s['condition']+' | independent simulation clock')
        for i,label in enumerate(['COM body vx (m/s)','COM body vy (m/s)','body-z angular rate (rad/s)']):
            ax=axs[i,col]
            draw(ax,rs,lambda r,k=i:item(r,'command',k),'recorded PID after-slew command',color='#334155',lw=.8)
            draw(ax,rs,lambda r,k=i:item(r,'reference_COM_vx_vy_wz',k),'drive reference',mask=drive,color='#7c3aed',lw=.7)
            draw(ax,rs,lambda r,k=i:item(r,'measured_SLAM_COM_velocity',k) if k<2 else item(r,'measured_body_omega',2),
                 'SLAM COM / IMU body omega',color='#16a34a',lw=.65)
            draw(ax,nr,lambda r,k=i:item(r,'COM_velocity',k) if k<2 else item(r,'body_omega',2),
                 'native actual (offline only)',time_fn=lambda r:number(r.get('world_s')),color=COLORS[s['condition']],lw=.6,alpha=.8)
            draw(ax,nr,lambda r,k=i:item(r,'command',k),'native applied command',time_fn=lambda r:number(r.get('world_s')),
                 color='#ea580c',lw=.65,alpha=.65)
            format_axis(ax,s,label)
        axs[-1,col].set_xlabel('this run simulation clock (s)')
    fig.suptitle('Commands and measured response | native COM/body omega are simulator offline diagnostics')
    artifacts+=save(fig,output,'commands_velocity')

    fig,axs=plt.subplots(4,n,figsize=(7*n,12),squeeze=False,sharex='col')
    for col,s in enumerate(runs):
        rs=s['pid'];color=COLORS[s['condition']];axs[0,col].set_title(s['condition']+' | fresh_math drive only')
        draw(axs[0,col],rs,lambda r:r.get('inner_heading_error_rad'),'inner reference - heading',mask=drive,color=color,lw=.8)
        draw(axs[1,col],rs,lambda r:r.get('cross_raw_m'),'raw path cross error',mask=drive,color='#64748b',lw=.8)
        draw(axs[1,col],rs,lambda r:r.get('cross_control_m'),'spatial reference cross error',mask=drive,color=color,lw=.8)
        draw(axs[2,col],rs,lambda r:item(r,'PI_error',0),'filtered PI vx error',mask=drive,color=color,lw=.8)
        draw(axs[2,col],rs,lambda r:item(r,'PI_error',1),'filtered PI vy error',mask=drive,color='#16a34a',lw=.8)
        draw(axs[3,col],rs,lambda r:item(r,'PI_error',2),'filtered PI body-z rate error',mask=drive,color=color,lw=.8)
        for row,label in enumerate(['heading error (rad)','cross error (m)','velocity error (m/s)','rate error (rad/s)']):format_axis(axs[row,col],s,label)
        axs[-1,col].set_xlabel('this run simulation clock (s)')
    fig.suptitle('Fresh mathematical drive updates | inner heading is spatial reference, not locked turn reference')
    artifacts+=save(fig,output,'drive_errors')
    return artifacts


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--off-series',required=True,type=Path);parser.add_argument('--off-diagnostic',required=True,type=Path)
    parser.add_argument('--on-series',type=Path);parser.add_argument('--on-diagnostic',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();require(bool(args.on_series)==bool(args.on_diagnostic),'ON requires both compact inputs')
    runs=[load(args.off_series,args.off_diagnostic)];require(runs[0]['condition']=='OFF','OFF input is not OFF')
    if args.on_series:
        runs.append(load(args.on_series,args.on_diagnostic));require(runs[1]['condition']=='ON','ON input is not ON')
    args.output.mkdir(parents=True,exist_ok=True)
    require(not any(args.output.iterdir()),'output must be empty to avoid overwriting prior results')
    summary=dict(schema='curvature_compact_plot_summary/v1',actual_acceptance_inferred=False,full46_pass=False,
        scope='Descriptive sample-weighted metrics on fresh_math=true AND mode=drive AND phase=drive; no native/SLAM timestamp join.',
        missing='Unavailable values remain NaN in plots / null in JSON, never zero or forward-filled.',
        coordinate_and_clock='Independent camera_init SLAM coordinates and own simulation clocks; region markers use each activation journal.',
        units_and_errors='Raw speed error = norm(reference COM vx/vy)-norm(raw measured SLAM COM vx/vy); raw rate error = reference body-z minus raw IMU body omega-z. PI errors are recorded filtered values; body-z is not Euler yaw rate.',
        command_scope='PID command is recorded command_after_slew, not a complete publication ledger. Native applied command records actuator-side receipt; no new publication gate is inferred.',
        native_scope='Recorded native COM velocity/body omega shown offline only, at their own recorded world_s; not used for navigation or error acceptance.',
        runs=[])
    for s in runs:
        summary['runs'].append(dict(condition=s['condition'],run=s['run'],run_id=s['run_id'],settings=s['settings'],inputs=s['_input'],
            original_acceptance=s['_diagnostic']['original_acceptance'],metrics=summarize_rows(s['pid']),
            regions={str(i+1):summarize_rows([r for r in s['pid'] if r.get('goal')==i]) for i in range(9)}))
    paths=plot_all(runs,args.output)
    summary['artifacts']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    summary['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    out=args.output/'PLOT_SUMMARY.json';out.write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(args.output),'conditions':[s['condition'] for s in runs],
                      'drive_rows':[r['metrics']['fresh_math_drive_rows'] for r in summary['runs']],
                      'full46_pass':False},indent=2))


if __name__=='__main__':main()
