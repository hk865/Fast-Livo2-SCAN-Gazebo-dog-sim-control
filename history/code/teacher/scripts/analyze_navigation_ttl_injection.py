#!/usr/bin/env python3
"""Independently audit one real command-producer pause and physical response.

This script is read-only apart from its own receipt/plot/NPZ outputs. It does
not signal processes, import ROS, change commands, or replace the ordinary
navigation audit's transient-TTL-unverified result with a global pass.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()

def read(path):
    return json.loads(Path(path).read_text())

def rows(path):
    with Path(path).open() as f:
        return [json.loads(line) for line in f if line.strip()]

def clean(value):
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, dict): return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [clean(v) for v in value]
    return value

def check(value, **evidence):
    return {'status': 'unverified' if value is None else 'passed' if bool(value) else 'failed',
            'passed': None if value is None else bool(value), **evidence}

def evaluate(run, fixture):
    run, fixture = Path(run).resolve(), Path(fixture).resolve()
    ordinary_bytes = (run/'summary_navigation_independent.json').read_bytes()
    generic_bytes = (run/'summary.json').read_bytes()
    ordinary = json.loads(ordinary_bytes)
    protocol = read(fixture/'protocol.json'); receipt = read(fixture/'receipt.json')
    events = rows(fixture/'events.jsonl'); telemetry = rows(run/'telemetry.jsonl')
    native = [r for r in rows(run/'actuator.jsonl') if r.get('kind') == 'physics_step']
    rules = protocol['rules']; checks = {}; metrics = {}; errors = []
    def one(kind, **fields):
        found = [r for r in events if r.get('kind') == kind and all(r.get(k) == v for k, v in fields.items())]
        if len(found) != 1: raise ValueError(f'{kind}/{fields}: expected one actual event, got {len(found)}')
        return found[0]
    trigger = one('trigger'); confirmed = one('signal_after', signal='SIGSTOP')
    release = one('signal_before', signal='SIGCONT'); released = one('signal_after', signal='SIGCONT')
    start = one('hold_start_after_confirmed_stop')
    frozen = receipt['original_frozen_envelope']; envelope = frozen['original_envelope']
    target = receipt['target']; identity = (target['pid'], target['starttime_ticks'])
    ident_ok = all((r['target']['pid'], r['target']['starttime_ticks']) == identity for r in events if r.get('kind') in {'signal_before', 'signal_after'})
    ident_ok &= receipt.get('recovery', {}).get('restored_same_process') is True
    reconstructed = (json.dumps(envelope, indent=2, ensure_ascii=False, allow_nan=False)+'\n').encode()
    hold = (release['clock_ns']-start['clock_ns'])/1e9
    fixture_ok = receipt['status'] == 'injection_completed' and not receipt.get('error') and not receipt.get('logging_errors') and str(run) == receipt['run'] and sha(fixture/'protocol.json') == receipt['protocol_sha256'] and sha(fixture/'executed_fixture.py') == receipt['fixture_sha256'] == protocol['fixture_sha256'] and sha(fixture/'events.jsonl') == receipt['events_sha256']
    checks['actual_single_producer_pause_and_restore'] = check(fixture_ok and ident_ok and confirmed['target']['state'] == 'T' and released['target']['state'] not in {'T', 't', 'Z', 'X', 'x'} and hold >= rules['hold_sim_ns']/1e9 and receipt.get('commands_or_joint_targets_written') is False and receipt.get('truth_navigation_used') is False, target_pid=target['pid'], target_starttime_ticks=target['starttime_ticks'], confirmed_stop_state=confirmed['target']['state'], restored_state=released['target']['state'], hold_clock_window_s=[start['clock_ns']/1e9, release['clock_ns']/1e9], actual_clock_hold_s=hold, kernel_identity_consistent=ident_ok)
    checks['original_file_bytes_frozen'] = check(hashlib.sha256(reconstructed).hexdigest() == frozen['sha256'] and len(reconstructed) == frozen['size_bytes'], original_file_sha256=frozen['sha256'], original_file_mtime_ns=frozen['mtime_ns'], original_file_bytes=frozen['size_bytes'], reconstructed_indent2_bytes_match=True, original_envelope_sequence=envelope['sequence'], original_source_sim_s=envelope['sim_time'], original_source_monotonic_wall=envelope['monotonic_wall'], note='File bytes SHA differs from canonical consumer SHA; source timestamps are retained rather than refreshed at SIGSTOP.')
    slam = rows(run/'navigation_slam_poses.jsonl')
    motion = [r for r in slam if trigger['motion_start_ns'] <= r['stamp_ns'] <= trigger['motion_last_ns']]
    mt = np.asarray([r['stamp_ns'] for r in motion], dtype=np.int64)
    mv = np.asarray([r['body_velocity'][0] for r in motion], float)
    trigger_ok = len(motion) >= 2 and mt[-1]-mt[0] >= rules['motion_dwell_ns'] and np.max(np.diff(mt)) <= rules['pose_gap_max_ns'] and np.min(mv) >= rules['forward_body_velocity_mps'] and motion[-1] == trigger['trigger_slam_row'] and envelope['healthy'] is True and envelope['requested'][0] >= rules['healthy_forward_command_mps']
    checks['actual_moving_measured_slam_trigger'] = check(trigger_ok, original_slam_stamp_ns=[int(mt[0]), int(mt[-1])], observed_motion_dwell_s=(mt[-1]-mt[0])/1e9, minimum_actual_slam_body_vx_mps=mv.min(), frozen_healthy_forward_request_mps=envelope['requested'][0], source='actual original measured SLAM; never Gazebo truth')
    canonical = hashlib.sha256(json.dumps(envelope, sort_keys=True, allow_nan=False).encode()).hexdigest()
    pt = np.asarray([r['world_sim_time'] for r in telemetry], float)
    cmd = np.asarray([r['command'] for r in telemetry], float)
    req = np.asarray([r['requested'] for r in telemetry], float)
    readwall = np.asarray([r['navigation_envelope']['read_monotonic_wall'] for r in telemetry], float)
    paused = (readwall >= confirmed['monotonic_wall']) & (readwall < release['monotonic_wall'])
    ids = np.flatnonzero(paused)
    if not len(ids): raise ValueError('No actual consumer samples while producer stopped')
    actual_frozen = all(telemetry[i]['navigation_envelope'].get('hash') == canonical and telemetry[i]['navigation_envelope'].get('sequence') == envelope['sequence'] and telemetry[i]['navigation_envelope'].get('envelope_sim_time') == envelope['sim_time'] and telemetry[i]['navigation_envelope'].get('envelope_monotonic_wall') == envelope['monotonic_wall'] for i in ids)
    checks['same_frozen_envelope_at_actual_consumer'] = check(actual_frozen, paused_policy_samples=len(ids), actual_world_window_s=[pt[ids[0]], pt[ids[-1]]], canonical_consumer_sha256=canonical, file_bytes_sha256=frozen['sha256'], source_timestamps_refreshed=False)
    stale = [i for i in ids if telemetry[i]['command_expired']]
    if not stale: raise ValueError('No stale consumer frame during actual pause')
    first = stale[0]; before = first-1
    stale_ids = [i for i in ids if i >= first]
    first_env = telemetry[first]['navigation_envelope']; previous_env = telemetry[before]['navigation_envelope']
    timeout = float(read(run/'navigation_profile.json')['pose_cloud_timeout_s'])
    age_valid = lambda e: -.05 <= e['wall_age_s'] < timeout and -.05 <= e['sim_age_s'] < timeout
    boundary_ok = age_valid(previous_env) and not age_valid(first_env) and not telemetry[before]['command_expired'] and all(telemetry[i]['command_expired'] and np.max(np.abs(req[i])) < 1e-8 for i in stale_ids)
    checks['first_actual_ttl_expiry_requests_zero'] = check(boundary_ok, timeout_wall_and_sim_s=timeout, original_source_sim_s=envelope['sim_time'], mathematical_sim_expiry_s=envelope['sim_time']+timeout, last_accepted_frame_world_s=pt[before], last_accepted_sim_age_s=previous_env['sim_age_s'], last_accepted_wall_age_s=previous_env['wall_age_s'], first_stale_frame_world_s=pt[first], first_stale_sim_age_s=first_env['sim_age_s'], first_stale_wall_age_s=first_env['wall_age_s'], stale_zero_requested_samples=len(stale_ids), note='Expiry is from original source stamps, not the older sensor-gate clock at confirmed SIGSTOP.')
    manifest = read(run/'policy_manifest.json'); delta = np.asarray(manifest['command_slew_acceleration'])*.02
    slew_error = max(float(np.max(np.abs(cmd[i]-(cmd[i-1]+np.clip(req[i]-cmd[i-1], -delta, delta))))) for i in stale_ids)
    zeros = [i for i in stale_ids if np.max(np.abs(cmd[i])) < 1e-8]
    if not zeros: raise ValueError('No zero actor input before command producer resumed')
    first_zero = zeros[0]
    checks['actual_teacher_brake_slew_to_zero'] = check(slew_error < 1e-9 and all(np.max(np.abs(cmd[i])) < 1e-8 for i in stale_ids if i >= first_zero), slew_max_error=slew_error, acceleration_limits_mps2_radps2=manifest['command_slew_acceleration'], first_stale_actor_input=cmd[first], first_zero_actor_input_world_s=pt[first_zero], observed_actor_brake_duration_s=pt[first_zero]-pt[first], note='Zero velocity actor input; action is continually inferred and is not set to zero.')
    nt = np.asarray([r['t']-.005 for r in native], float)
    pos = np.asarray([r['position'] for r in native], float)
    v = np.asarray([r['body_lin_vel_com'] for r in native], float)
    omega = np.asarray([r['body_ang_vel'] for r in native], float)
    q = np.asarray([r['quaternion_wxyz'] for r in native], float)
    w,x,y,z=q.T
    yaw = np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    limits = read(run/'sources/tests/protocol.json')['stand_stop']
    window_start = pt[first]+limits['settling_seconds']; window_end = window_start+3.
    nm = (nt >= window_start-1e-10) & (nt <= window_end+1e-10)
    tm = (pt >= window_start-1e-10) & (pt <= window_end+1e-10)
    ni = np.flatnonzero(nm); ti = np.flatnonzero(tm)
    if len(ni) < 599 or len(ti) < 149: raise ValueError('Incomplete predetermined3s stop window')
    p = pos[nm]; drift=float(np.max(np.linalg.norm(p[:,:2]-p[0,:2],axis=1)))
    yd=float(np.max(np.abs(yaw[nm]-yaw[nm][0]))); rms=np.sqrt(np.mean(v[nm,:2]**2,axis=0)); wrms=float(np.sqrt(np.mean(omega[nm,2]**2)))
    coverage=nt[ni[-1]]-nt[ni[0]] >= 3.-.005001 and np.max(np.diff(nt[ni])) <= .005001 and np.max(np.diff(pt[ti])) <= .020001 and window_end < release['clock_ns']/1e9
    actor=all(telemetry[i].get('actor_inferred_this_frame') is True and telemetry[i].get('state') not in {'support_hold','support_capture'} for i in ti)
    stop_ok=coverage and actor and np.max(np.abs(req[tm])) < 1e-8 and np.max(np.abs(cmd[tm])) < 1e-8 and drift <= limits['translation_drift_m'] and yd <= limits['yaw_drift_rad'] and np.max(rms) <= limits['linear_rms_mps'] and wrms <= limits['yaw_rate_rms_radps']
    contacts=np.asarray([r['contacts']for r in native],float)
    rp=np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1))])
    checks['predetermined_teacher_physical_stop_before_resume'] = check(stop_ok, world_time_window_s=[window_start,window_end], native_physical_state_time='PreUpdate t minus0.005s', native_samples=len(ni), actor_samples=len(ti), coverage_complete=coverage, continuous_actor=actor, actual_inference_device=manifest.get('inference_device'), checkpoint_sha256=manifest.get('checkpoint_sha256'), no_support_hold_or_capture=actor, actual_zero_requested_and_actor_command=bool(np.max(np.abs(req[tm]))<1e-8 and np.max(np.abs(cmd[tm]))<1e-8), body_contact_samples=int((contacts[nm,0]>0).sum()), missing_contact_samples=int((contacts[nm]<0).sum()), maximum_abs_roll_pitch_rad=float(np.max(np.abs(rp[nm]))), translation_drift_m=drift, yaw_drift_rad=yd, linear_body_com_rms_mps=rms, yaw_rate_rms_radps=wrms, original_stand_stop_limits=limits, measurement_duration_s=3., measured_before_producer_resumed=True)
    resumed=np.flatnonzero(readwall >= released['monotonic_wall'])
    healthy_motion=[i for i in resumed if telemetry[i]['navigation_envelope'].get('read_status')=='accepted' and telemetry[i]['navigation_envelope'].get('sequence',0)>envelope['sequence'] and np.linalg.norm(req[i,:2])>.015]
    new_motion_s=len(healthy_motion)*.02
    recovery_start=pt[healthy_motion[0]] if healthy_motion else None
    rm=nt >= recovery_start if recovery_start is not None else np.zeros(len(nt),bool)
    measured_s=float(np.sum(np.linalg.norm(v[rm,:2],axis=1)>.05)*.005)
    progression=float(np.max(np.linalg.norm(pos[rm,:2]-pos[rm,:2][0],axis=1))) if rm.any() else 0.
    regular_causal=ordinary['checks']['causal_worker_command_source']['status']=='passed' and ordinary['checks']['teacher_continuous_actor_and_command_input']['status']=='passed'
    checks['fresh_slam_command_and_physical_motion_recovered'] = check(regular_causal and new_motion_s>=.5 and measured_s>=.5 and progression>=.1, first_actual_accepted_nonzero_command_world_s=recovery_start, new_accepted_motion_command_s=new_motion_s, actual_native_motion_s=measured_s, post_recovery_physical_xy_excursion_m=progression, recovery_requirement='Same previously frozen finite-route motion minima: ≥.5s accepted nonzero commands, ≥.5s actual speed>.05m/s, ≥.1m physical excursion', actual_lineage_regular_audit_passed=regular_causal)
    checks['complete_finite_route_after_dropout'] = check(ordinary['status']=='passed' and ordinary['levels']['slam_region_arrival']=='passed' and ordinary['levels']['post_arrival_stop']=='passed', ordinary_finite_route_status=ordinary['status'], original_raw_slam_arrivals=ordinary['metrics']['arrivals'], ordinary_global_transient_TTL_status=ordinary['levels']['command_TTL_physical_stop'])
    safety = ordinary['checks']['native_physical_safety']
    checks['native_safety_and_runtime_continuous'] = check(safety['status']=='passed' and ordinary['checks']['actual_runtime_continuity']['status']=='passed', native_safety=safety, runtime=ordinary['checks']['actual_runtime_continuity'])
    if (run/'summary_navigation_independent.json').read_bytes()!=ordinary_bytes or (run/'summary.json').read_bytes()!=generic_bytes: raise ValueError('Existing summaries unexpectedly changed')
    # The primary verdict is this targeted parking/recovery event, not route arrival.
    required=[v['status']for k,v in checks.items()if k!='complete_finite_route_after_dropout']
    verdict='failed'if'failed'in required else'unverified'if'unverified'in required else'passed'
    summary = clean({'schema':1,'scope':'one_actual_V4_command_producer_SIGSTOP_drop_and_restore_simulation_only','status':verdict,'levels':{'targeted_dropout_stop_and_recovery':verdict,'targeted_physical_stop':checks['predetermined_teacher_physical_stop_before_resume']['status'],'healthy_motion_recovery':checks['fresh_slam_command_and_physical_motion_recovered']['status'],'finite_route_after_dropout':ordinary['levels']['finite_flat_navigation'],'ordinary_all_transient_TTL':ordinary['levels']['command_TTL_physical_stop']},'checks':checks,'errors':errors,'analyzer_sha256':sha(__file__),'run_dir':str(run),'fixture_dir':str(fixture),'source_hashes':{str(path):sha(path)for path in [fixture/'protocol.json',fixture/'executed_fixture.py',fixture/'receipt.json',fixture/'events.jsonl',run/'telemetry.jsonl',run/'actuator.jsonl',run/'policy_manifest.json',run/'navigation_slam_poses.jsonl',run/'navigation_profile.json',run/'sources/tests/protocol.json',run/'summary_navigation_independent.json',run/'summary.json']},'ordinary_summary_bytes_preserved':True,'ordinary_transient_TTL_result_preserved':ordinary['levels']['command_TTL_physical_stop'],'global_acceptance_overwritten':False,'global_levels_preserved':{'sim2sim':'failed','navigation_all_tasks':'unverified','dynamic_obstacle':'unverified','multifloor':'unverified','real_robot':'unverified'},'limitation':'One targeted producer dropout only; cannot generalize every transient timeout or all outage modes. True navigation uses SLAM/cloud; physical truth is read only after the run.'})
    output=run/'summary_ttl_dropout_independent.json'
    npz=run/'navigation_ttl_injection_physical.npz'
    np.savez_compressed(npz,stop_native_world_time_s=nt[nm],stop_position=pos[nm],stop_body_com_velocity=v[nm],stop_body_angular_velocity=omega[nm],stop_quaternion_wxyz=q[nm],stop_policy_world_time_s=pt[tm],stop_requested=req[tm],stop_actor_command=cmd[tm])
    fig,axs=plt.subplots(3,1,figsize=(11,8),sharex=True)
    axs[0].plot(nt,v[:,0],color='#416891',lw=.5,label='actual native body COM vx200Hz')
    axs[0].step(pt,req[:,0],where='post',color='#8b8b8b',lw=1,label='actual requested vx50Hz')
    axs[0].step(pt,cmd[:,0],where='post',color='#e28719',lw=1,label='actual actor vx50Hz')
    axs[0].set_ylabel('body vx (m/s)');axs[0].legend(fontsize=8)
    axs[1].plot(nt,omega[:,2],color='#416891',lw=.5,label='native body wz200Hz')
    axs[1].step(pt,cmd[:,2],where='post',color='#e28719',lw=1,label='actor wz50Hz')
    axs[1].set_ylabel('body wz (rad/s)');axs[1].legend(fontsize=8)
    ages=np.asarray([r['navigation_envelope'].get('sim_age_s',np.nan)for r in telemetry],float)
    axs[2].plot(pt,ages,color='#654d99',lw=1,label='age from original envelope sim stamp')
    axs[2].axhline(timeout,color='#b24444',ls=':',label='unchanged300ms TTL')
    axs[2].set_ylabel('source age (s)');axs[2].legend(fontsize=8)
    for ax in axs:
        ax.axvline(pt[first],color='#b24444',ls=':');ax.axvline(release['clock_ns']/1e9,color='#353535',ls=':')
        ax.axvspan(window_start,window_end,color='#5fb37a',alpha=.2);ax.grid(alpha=.2);ax.set_xlim(5,23)
    axs[-1].set_xlabel('actual world simulation time (s); native physical t=PreUpdate−.005s')
    fig.suptitle('Actual producer dropout: source TTL → Teacher braking → measured stop → resumed navigation')
    fig.tight_layout(rect=(0,0,1,.96));plot=run/'navigation_ttl_injection_physical.png';fig.savefig(plot,dpi=150);plt.close(fig)
    summary['artifacts_sha256']={str(npz):sha(npz),str(plot):sha(plot)}
    output.write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    return summary

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('fixture',type=Path)
    a=p.parse_args()
    try:
        result=evaluate(a.run,a.fixture)
    except (ValueError,KeyError,FileNotFoundError,IndexError) as e:
        result={'schema':1,'scope':'targeted_actual_dropout_stop_and_recovery','status':'unverified','checks':{},'errors':[str(e)],'analyzer_sha256':sha(__file__),'global_acceptance_overwritten':False}
        (a.run/'summary_ttl_dropout_independent.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'status':result['status'],'failed':[k for k,v in result['checks'].items()if v['status']=='failed'],'unverified':[k for k,v in result['checks'].items()if v['status']=='unverified'],'output':str(a.run.resolve()/'summary_ttl_dropout_independent.json')},ensure_ascii=False))
