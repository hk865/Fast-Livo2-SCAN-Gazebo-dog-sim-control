#!/usr/bin/env python3
"""Scientific actual PID diagnosis figures. No ROS and no simulation controls."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for c in iter(lambda:f.read(1<<20),b''):h.update(c)
    return h.hexdigest()
def lines(p):
    with p.open() as f:return [json.loads(x) for x in f if x.strip()]
def spans(t,mask):
    begin=None;out=[]
    for i,v in enumerate(mask):
        if v and begin is None:begin=t[i]
        if begin is not None and (not v or i==len(t)-1):
            out.append((begin,t[i]));begin=None
    return out

def plot(run,version):
    summaryfile=run/f'summary_pid_navigation_independent.{version}.json'
    arrayfile=run/f'pid_navigation_independent_arrays.{version}.npz'
    summary=json.loads(summaryfile.read_text());pid=lines(run/'navigation_pid_history.jsonl');ex=lines(run/'telemetry.jsonl')
    request=json.loads((run/'navigation_request.json').read_text())
    with np.load(arrayfile) as z:a={k:z[k] for k in z.files}
    helperpath=Path(__file__).with_name('analyze_pid_navigation_v1_receipt_fix.py')
    spec=importlib.util.spec_from_file_location('plot_pid_helpers',helperpath);h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
    nt=a['native_world_time_s'];et=a['execution_world_time_s'];cmd=a['execution_command'];pt=np.asarray([x['compute_ros_clock_ns']/1e9 for x in pid])
    pi,pv=h.causal_index(pt,nt,.3);ei,ev=h.causal_index(et,nt,.020001)
    modes=np.asarray([pid[i]['mode'] for i in pi]);wp=np.asarray([pid[i]['waypoint_index'] for i in pi]);rot=h.rotation(a['native_quaternion_wxyz']);angles=h.rpy(a['native_quaternion_wxyz'])
    worldcmd=np.einsum('nij,nj->ni',rot,cmd[ei]);bodyvel=np.einsum('nji,nj->ni',rot,a['native_world_COM_velocity'])
    actualwz=np.asarray([x['body_ang_vel'][2] for x in ex]);expired=np.asarray([x.get('command_expired',False) for x in ex]);drive=a['drive_mask']
    vertices=a['planned_world_polyline'];direction=np.zeros((len(nt),2))
    for j in range(len(vertices)-1):
        v=vertices[j+1,:2]-vertices[j,:2];direction[wp==j]=v/np.linalg.norm(v)
    projected=np.einsum('ij,ij->i',a['native_world_COM_velocity'][:,:2],direction)
    projectedcmd=np.einsum('ij,ij->i',worldcmd[:,:2],direction)
    final=summary['checks'].get('fixed_final_zero_command_parking',{});window=final.get('world_window_s')
    active_limit=min(float(et[-1]),max(60,float(pt[-1])+8))
    fig,ax=plt.subplots(4,2,figsize=(16,13),constrained_layout=True)
    fig.suptitle(f"{summary['controller']} | {summary['case_id']} | strict status: {summary['status'].upper()}\n{run.name}",fontsize=14)
    raw=a['SLAM_position'];anchor=json.loads((run/'navigation_anchor.json').read_text())['origin']
    ax[0,0].plot(raw[:,0],raw[:,1],color='#087e8b',lw=1.4,label='Actual raw sensor SLAM')
    for j,g in enumerate(request['goals']):
        x,y=g['center'][:2];ax[0,0].add_patch(Circle((x,y),.17,fill=False,ls='--',color=['#df8f00','#7149b1'][j%2]));ax[0,0].plot(x,y,'+',color='black');ax[0,0].annotate(f'goal {j}',(x,y),xytext=(5,7),textcoords='offset points')
    ax[0,0].plot(anchor[0],anchor[1],'o',color='#087e8b',ms=5,label='Immutable SLAM anchor');ax[0,0].set_title('Navigation evidence: SLAM frame only');ax[0,0].set_xlabel('camera_init x (m)');ax[0,0].set_ylabel('camera_init y (m)');ax[0,0].set_aspect('equal',adjustable='datalim');ax[0,0].legend(fontsize=8)
    pos=a['native_position'];ax[0,1].plot(pos[:,0],pos[:,1],color='#315b99',lw=1.4,label='Actual native body origin (offline)');ax[0,1].plot(vertices[:,0],vertices[:,1],'k--',lw=1,label='Predefined physical test route');ax[0,1].set_title('Independent physical route: world frame, no truth control');ax[0,1].set_xlabel('Gazebo world x (m)');ax[0,1].set_ylabel('Gazebo world y (m)');ax[0,1].set_aspect('equal',adjustable='datalim');ax[0,1].legend(fontsize=8)
    ax[1,0].plot(nt,np.where(drive,projectedcmd,np.nan),color='#df8f00',lw=1.2,label='Actor command, projected on active leg');ax[1,0].plot(nt,np.where(drive,projected,np.nan),color='#087e8b',lw=.65,label='Actual COM speed, projected');ax[1,0].axhline(.05,color='#bb3232',ls='--',lw=1,label='0.5s mean gate: >0.05m/s');ax[1,0].set_title('Drive speed; a positive command is not actual movement');ax[1,0].set_ylabel('m/s');ax[1,0].legend(fontsize=8)
    failurefile=run/'pid_drive_failure_diagnostic.json'
    if failurefile.exists():
        for g in json.loads(failurefile.read_text())['groups']:ax[1,0].axvspan(g['first_failing_window_end_world_s'],g['last_failing_window_end_world_s'],color='#dd5555',alpha=.2)
    ax[1,1].plot(et,cmd[:,2],color='#df8f00',lw=1.3,label='Actor yaw command');ax[1,1].plot(et,actualwz,color='#087e8b',lw=.7,label='Actual body yaw rate');ax[1,1].set_title('Turn and stop response');ax[1,1].set_ylabel('rad/s');ax[1,1].legend(fontsize=8)
    ax[2,0].plot(nt,np.where(drive,a['drive_heading_error_rad'],np.nan),color='#7149b1',lw=1.1,label='Explicit PID drive only');ax[2,0].axhline(.35,color='#bb3232',ls='--',label='Frozen 0.35rad bound');ax[2,0].set_ylabel('absolute rad');ax[2,0].set_title('Heading: turn/hold is excluded from drive gate');ax[2,0].legend(fontsize=8)
    ax[2,1].plot(nt,a['lateral_error_m'],color='#315b99',lw=1,label='Bounded planned XY polyline distance');ax[2,1].axhline(.45,color='#bb3232',ls='--',label='Maximum error bound 0.45m');ax[2,1].set_ylabel('m');ax[2,1].set_title('Physical route error; active RMS gate remains 0.20m');ax[2,1].legend(fontsize=8)
    mode_map={'hold':0,'turn':1,'drive':2};mv=np.asarray([mode_map.get(x['mode'],-1) for x in pid]);ax[3,0].step(pt,mv,where='post',color='#7149b1',lw=1,label='Actual PID mode');ax[3,0].scatter(et[expired],np.full(expired.sum(),-.55),s=2,color='#bb3232',label='Consumer expired/unhealthy');ax[3,0].set_yticks([-.55,0,1,2],['protect','hold','turn','drive']);ax[3,0].set_title('Actual mode and causal protection, 0–active segment');ax[3,0].legend(fontsize=8,loc='upper right')
    ax[3,1].plot(et,cmd[:,0],color='#df8f00',lw=1,label='Actor body forward command');ax[3,1].plot(nt,bodyvel[:,0],color='#087e8b',lw=.7,label='Actual body COM forward speed');ax[3,1].set_title(f'Full {et[-1]:.1f}s record; formal parking highlighted');ax[3,1].set_xlabel('world simulation time (s)');ax[3,1].set_ylabel('m/s');ax[3,1].set_xlim(0,et[-1]);ax[3,1].legend(fontsize=8)
    if window:
        ax[3,1].axvspan(*window,color='#70b86a',alpha=.25);ax[3,1].annotate(f"fixed parking {window[0]:.3f}–{window[1]:.3f}s\nXY drift {final['translation_drift_m']:.6f}m",(window[-1],.14),xytext=(6,0),textcoords='offset points',fontsize=8)
    for axy in ax[1:,:].ravel():
        axy.grid(alpha=.2)
        if axy is not ax[3,1]:axy.set_xlim(0,active_limit);axy.set_xlabel('world simulation time (s)')
    for axy in ax[0,:]:axy.grid(alpha=.2)
    out=run/'pid_navigation_diagnostic.png';fig.savefig(out,dpi=160);plt.close(fig)
    j={'schema':'pid_actual_figure_provenance/v1','run':str(run),'figure':out.name,'figure_sha256':sha(out),'plot_script_sha256':sha(__file__),'data_generated_or_synthetic':False,'physics_control_changed':False,'frame_overlay_used':False,'scope':'Separate rawSLAM/navigation and Gazebo/offline physical panels; no fabricated alignment or navigation truth. Summary gates unchanged.','input_hashes':{p.name:sha(p) for p in [summaryfile,arrayfile,run/'navigation_pid_history.jsonl',run/'telemetry.jsonl',run/'navigation_request.json',run/'navigation_anchor.json']},'visual_QA':'pending independent pixel inspection'}
    (run/'pid_navigation_diagnostic_figure_manifest.json').write_text(json.dumps(j,indent=2)+'\n');print(json.dumps(j))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--version',default='corrected_v1');a=p.parse_args();plot(a.run.resolve(),a.version)
