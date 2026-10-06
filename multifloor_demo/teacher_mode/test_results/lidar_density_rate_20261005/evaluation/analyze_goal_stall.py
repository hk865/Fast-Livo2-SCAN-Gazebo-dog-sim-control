#!/usr/bin/env python3
"""Describe a failed measured waypoint without modifying transition gates."""
from __future__ import annotations
import argparse,csv,hashlib,json,math
from collections import Counter
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def rows(p):return [json.loads(l)for l in p.open()if l.strip()]
def dist(v):
    a=np.asarray(v,float);a=a[np.isfinite(a)]
    return dict(count=len(a),min=float(a.min()),p50=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max()))if len(a)else{'count':0}
def sha(p):
    h=hashlib.sha256()
    with p.open('rb')as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run',required=True,type=Path);ap.add_argument('--output',required=True,type=Path);a=ap.parse_args();r=a.run.resolve();a.output.mkdir(parents=True,exist_ok=True)
    statuses=rows(r/'navigation_status.jsonl');failed=next((d for d in statuses if d['state']=='failed'),None)
    if failed is None:raise ValueError('No actual failed waypoint; cannot fabricate a stall window')
    index=failed['waypoint_index'];start=failed['goal_activated_ros_clock_ns']*1e-9;end=failed['ros_sim_time']
    pid=[d for d in rows(r/'navigation_pid_history.jsonl')if d['waypoint_index']==index and start<=d['compute_ros_clock_ns']*1e-9<=end]
    pubs=[d for d in rows(r/'navigation_command_publications.jsonl')if start<=d['publish_ros_clock_ns']*1e-9<=end]
    native=[d for d in rows(r/'telemetry.jsonl')if start<=d['state_physics_world_time']<=end]
    curve=[]
    for d in pid:
        c=d['cascade'];imu=d.get('imu');omega=imu['angular_velocity_body'][2]if imu else math.nan
        curve.append([d['compute_ros_clock_ns']*1e-9, d['control_pose'][0],d['control_pose'][1],d['control_pose'][2],
            *d['command_after_slew'],omega,c.get('measured_Euler_yawrate_radps',math.nan),c.get('error_yaw_rad',math.nan),
            c.get('goal_distance_xy_m',math.nan),c.get('goal_z_error_m',math.nan),d['compute_ros_clock_ns']*1e-9-d['source_pose_stamp_ns']*1e-9])
    cc=np.asarray(curve,float);stop=[d for d in pid if d['heading_gate_reference']['phase']in('pre_turn','settle')]
    gi=[d['imu']['angular_velocity_body'][2]for d in stop if d.get('imu')]
    nr=np.array([[d['state_physics_world_time'],d['rpy'][2],d['body_ang_vel'][2],*d['position'],*d['command']]for d in native])
    transitions=[];prior=None
    for d in pid:
        phase=d['heading_gate_reference']['phase']
        if phase!=prior:transitions.append({'sim_s':d['compute_ros_clock_ns']*1e-9,'phase':phase,'PID_sequence':d['sequence']});prior=phase
    result=dict(schema='original_failed_waypoint_heading_and_motion_diagnostic/v1',run=str(r),waypoint_index=index,
        original_activated_failed_window_s=[start,end],actual_elapsed_s=end-start,first_failure_message=failed['message'],
        goal=failed['current_goal'],all_original_acceptance_gates_unchanged=True,navigation_ground_truth_used=False,
        native_truth_used_offline_only=True,PID_records=len(pid),mode_counts=dict(Counter(d['cascade']['mode']for d in pid)),
        reason_counts={str(k):v for k,v in Counter((d['cascade']['mode'],d['cascade']['reason'])for d in pid).items()},
        phase_counts=dict(Counter(d['heading_gate_reference']['phase']for d in pid)),phase_transitions=transitions,
        actual_publication_trigger_counts=dict(Counter(d['trigger']for d in pubs)),
        exact_zero_publications=sum(d['command_after']==[0.,0.,0.]for d in pubs),
        last_original_goal_XY_error_m=pid[-1]['cascade'].get('goal_distance_xy_m'),last_original_goal_Z_error_m=pid[-1]['cascade'].get('goal_z_error_m'),
        external_zero_gate_IMU_body_wz=dist(gi),external_zero_gate_absolute_IMU_body_wz_over_original005_count=sum(abs(v)>.05 for v in gi),
        actual_native_body_wz=dist(nr[:,2]),native_position_start_end=nr[[0,-1],3:6].tolist(),
        native_relative_yaw_change_rad=float(np.unwrap(nr[:,1])[-1]-np.unwrap(nr[:,1])[0]),
        interpretation='Direct evidence shows repeated pre_turn/align/settle zero/turn states with no sustained drive, so timeout is not proof of persistent SLAM source loss. Body-yaw drift under zero and strict measured stop/heading transitions can interact; this is an inference, not a unique policy-versus-gate cause proof. Numerical gates are preserved.',
        input_sha256={name:sha(r/name)for name in ['navigation_pid_history.jsonl','navigation_status.jsonl','navigation_command_publications.jsonl','telemetry.jsonl']},reader_sha256=sha(Path(__file__)))
    (a.output/'failed_waypoint_diagnosis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    np.savetxt(a.output/'failed_waypoint_PID_curve.csv',cc,delimiter=',',header='sim_s,SLAM_x,SLAM_y,SLAM_z,command_vx,command_vy,command_wz,IMU_body_wz,SLAM_Euler_yawdot,yaw_error,goal_XY_error,goal_Z_error,source_sim_age',comments='')
    np.savetxt(a.output/'failed_waypoint_native_offline_curve.csv',nr,delimiter=',',header='physical_s,native_yaw,native_body_wz,native_x,native_y,native_z,actual_actor_vx,actual_actor_vy,actual_actor_wz',comments='')
    fig,ax=plt.subplots(4,1,figsize=(13,10),sharex=True)
    ax[0].plot(cc[:,0],cc[:,10],label='Original SLAM XY goal error');ax[0].plot(cc[:,0],cc[:,11],label='Original SLAM Z goal error');ax[0].set_ylabel('Error [m]')
    ax[1].plot(cc[:,0],cc[:,9],label='Original heading error');ax[1].axhline(.1,ls=':',c='red');ax[1].axhline(-.1,ls=':',c='red');ax[1].set_ylabel('Yaw error [rad]')
    ax[2].plot(cc[:,0],cc[:,6],label='Actual controller wz');ax[2].plot(cc[:,0],cc[:,7],label='Actual causal IMU wz',alpha=.65);ax[2].plot(nr[:,0],nr[:,2],label='Native wz (OFFLINE)',alpha=.4);ax[2].axhline(.05,ls=':',c='red');ax[2].axhline(-.05,ls=':',c='red');ax[2].set_ylabel('Yaw rate [rad/s]')
    ax[3].plot(cc[:,0],cc[:,4],label='Controller vx');ax[3].plot(cc[:,0],cc[:,12],label='Original pose source age');ax[3].axhline(.3,ls=':',c='red');ax[3].set_ylabel('m/s / s');ax[3].set_xlabel('Actual ROS/source simulation time [s]')
    for x in ax:x.legend();x.grid(alpha=.25)
    fig.suptitle('Failed original waypoint: measured pose, command and heading/stop transitions; truth offline only');fig.tight_layout();fig.savefig(a.output/'failed_waypoint_heading_motion.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in result.items()if k not in ['phase_transitions','input_sha256','goal']},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
