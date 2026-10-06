#!/usr/bin/env python3
"""30s small-file observation only. No raw streams, signals or ROS nodes."""
from datetime import datetime, timezone
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path
import time

HERE = Path(__file__).resolve().parent
RUN = Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_182824_closed_loop_cascade_clock_hold_terrain_event_V25_full46_r1_isolated_e8de')


def read(name):
    path = RUN / name
    try:
        stat = path.stat()
        if stat.st_size > 1_000_000:
            raise RuntimeError('Refuse non-small observation file: ' + name)
        return json.loads(path.read_text()), dict(path=str(path), bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)
    except (OSError, ValueError, RuntimeError) as error:
        return {}, dict(path=str(path), error=str(error))


def append(name, data):
    with (HERE / name).open('a') as stream:
        stream.write(json.dumps(data, ensure_ascii=False, allow_nan=False) + '\n')


def proc_snapshot(pid, expected_ticks=None):
    try:
        path = Path('/proc') / str(pid) / 'stat'
        parts = path.read_text().rsplit(')', 1)[1].split()
        return dict(pid=pid, present=True, state=parts[0], ppid=int(parts[1]),
            pgid=int(parts[2]), session=int(parts[3]), start_ticks=int(parts[19]),
            cpu_user_ticks=int(parts[11]), cpu_system_ticks=int(parts[12]),
            saved_identity_matches=(int(parts[19]) == expected_ticks) if expected_ticks is not None else None)
    except FileNotFoundError:
        return dict(pid=pid, present=False, saved_identity_matches=False)


def isolation_receipt(setup):
    owned = setup['owned_processes_started.json']['data']
    worker = owned.get('worker', {})
    live_worker = proc_snapshot(worker.get('pid'), worker.get('start_ticks'))
    runner = proc_snapshot(worker.get('ppid'))
    parent = proc_snapshot(runner.get('ppid')) if runner.get('present') else {}
    command = None
    if runner.get('present'):
        command = (Path('/proc') / str(runner['pid']) / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    isolated = (live_worker.get('saved_identity_matches') is True and
        live_worker.get('ppid') == runner.get('pid') and runner.get('present') is True and
        runner['pid'] == runner['pgid'] == runner['session'] and
        (not parent.get('present') or parent['pgid'] != runner['pgid']))
    result = dict(schema='v25_setsID_runner_isolation_observation/v1', run=str(RUN),
        observation_only=True, observed_monotonic_wall_ns=time.monotonic_ns(),
        saved_worker=worker, actual_worker=live_worker, actual_runner=runner,
        actual_runner_parent=parent, actual_runner_cmdline=command,
        runner_pid_equals_pgid_equals_session=runner.get('present') is True and runner['pid'] == runner['pgid'] == runner['session'],
        isolation_observed=isolated, no_signal_or_process_mutation_performed=True,
        not_a_navigation_or_runtime_pass=True)
    (HERE / 'RUNNER_ISOLATION_RECEIPT.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    return result


def resource_snapshot(stage, setup, isolation):
    identities = dict(setup['owned_processes_started.json']['data'])
    for filename, key, role in [('slam_loaded_binary.json', 'mapping_identity', 'FAST_LIVO'),
                               ('scan_loaded_binary.json', 'planner_identity', 'SCAN')]:
        identity = setup[filename]['data'].get(key)
        if identity:
            identities[role] = identity
    if isolation.get('actual_runner', {}).get('present'):
        identities['runner'] = isolation['actual_runner']
    begin_wall = time.monotonic()
    before = {role: proc_snapshot(saved['pid'], saved.get('start_ticks')) for role, saved in identities.items()}
    time.sleep(1)
    end_wall = time.monotonic()
    after = {role: proc_snapshot(saved['pid'], saved.get('start_ticks')) for role, saved in identities.items()}
    for role, p in after.items():
        b = before[role]
        if b.get('saved_identity_matches') and p.get('saved_identity_matches') and b.get('start_ticks') == p.get('start_ticks'):
            p['sampled_CPU_cores'] = ((p['cpu_user_ticks'] + p['cpu_system_ticks']) -
                (b['cpu_user_ticks'] + b['cpu_system_ticks'])) / os.sysconf('SC_CLK_TCK') / (end_wall - begin_wall)
        else:
            p['sampled_CPU_cores'] = None
    query = subprocess.run(['/usr/bin/nvidia-smi', '--query-gpu=index,memory.used,memory.total,utilization.gpu,utilization.memory',
        '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=10)
    mem = {key: value.strip() for key, value in (line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
        if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree')}
    disks = {path: dict(total=shutil.disk_usage(path).total, used=shutil.disk_usage(path).used,
        free=shutil.disk_usage(path).free) for path in ['/home', '/var/tmp', str(RUN)]}
    row = dict(schema='v25_readonly_phase_resource/v1', run=str(RUN), stage=stage,
        sampled_utc=datetime.now(timezone.utc).isoformat(), sampled_monotonic_wall_ns=time.monotonic_ns(),
        loadavg=Path('/proc/loadavg').read_text().strip(), memory_KiB=mem, disk_bytes=disks,
        gpu=dict(query_returncode=query.returncode, query_stdout=query.stdout.strip(), query_stderr=query.stderr.strip()),
        owned_processes_before=before, owned_processes_after=after, CPU_sample_interval_wall_s=end_wall - begin_wall,
        observation_only=True, no_priority_affinity_signal_or_other_task_change=True)
    append('PHASE_RESOURCES.jsonl', row)
    print('RESOURCE ' + json.dumps(dict(stage=stage, gpu=query.stdout.strip(), home_free_GiB=disks['/home']['free']/2**30,
        observed_owned_CPU_cores={role: p['sampled_CPU_cores'] for role, p in after.items()}), ensure_ascii=False), flush=True)


def main():
    HERE.mkdir(parents=True, exist_ok=True)
    sample_path = HERE / 'STATUS_SAMPLES.jsonl'
    if sample_path.exists():
        raise RuntimeError('Refuse overwriting or mixing an observation session')
    receipt = dict(schema='v25_small_file_observation_setup/v1', run=str(RUN),
        interval_wall_s=30, observation_only=True, formal_acceptance=False,
        native_truth_position_used_for_navigation=False,
        source_scope='Small latest state/status/mission and immutable actual loaded binary receipts only. No PID/native raw stream scan.')
    for name in ('slam_loaded_binary.json', 'scan_loaded_binary.json', 'owned_processes_started.json'):
        data, source = read(name)
        if 'error' not in source:
            source['sha256'] = hashlib.sha256(Path(source['path']).read_bytes()).hexdigest()
        receipt[name] = dict(source=source, data=data)
    (HERE / 'SETUP_RECEIPT.json').write_text(json.dumps(receipt, indent=2) + '\n')
    isolation = isolation_receipt(receipt)
    print('ISOLATION ' + json.dumps(isolation, ensure_ascii=False), flush=True)
    resource_snapshot('observer_start', receipt, isolation)
    seen_arrivals = set()
    previous_activation = None
    previous_mission_phase = None
    started = time.monotonic()
    sequence = 0
    while time.monotonic() - started < 7200:
        native, native_source = read('state.json')
        nav, nav_source = read('navigation_status.json')
        mission, mission_source = read('mission46_status.json')
        control, control_source = read('mission46_control.json')
        cascade = nav.get('cascade_parking') or {}
        sequence += 1
        row = dict(schema='v25_small_status_observation/v1', sequence=sequence, run=str(RUN),
            sampled_utc=datetime.now(timezone.utc).isoformat(), sampled_monotonic_wall_ns=time.monotonic_ns(),
            sim_time=native.get('sim_time'), native_state=native.get('state'), native_fault=native.get('fault'),
            native_command_expired=native.get('command_expired'), native_command=native.get('command'),
            native_command_reason=native.get('command_reason'), native_requested=native.get('requested'),
            native_actor_terrain_provider=native.get('actor_terrain_provider'),
            native_measured_body_lin_vel=native.get('body_lin_vel'), native_measured_body_ang_vel=native.get('body_ang_vel'),
            native_rpy=native.get('rpy'), native_inference_ms=native.get('inference_ms'),
            mission_stage=mission.get('stage'), mission_message=mission.get('message'),
            mission_sim_ns=mission.get('sim_ns'), completed_stages=mission.get('completed_stages'),
            mission_declared_completed_region_count=mission.get('completed_region_count'),
            functional_sequence_completed=mission.get('functional_sequence_completed'),
            initialization_verified=mission.get('initialization_verified'), RGB_save_verified=mission.get('current_rgb_save_verified'),
            dynamic_verified=mission.get('original_dynamic_verified'), final_parking_verified=mission.get('final_parking_verified'),
            mission_layer=mission.get('layer'), terrain_switch_count=mission.get('terrain_switch_count'),
            terrain_pending=mission.get('terrain_pending'),
            runtime_errors=mission.get('last_runtime_errors'), mission_control_hold=control.get('hold'),
            mission_control_reason=control.get('reason'), request_id=nav.get('request_id'), nav_state=nav.get('state'),
            waypoint_index=nav.get('waypoint_index'), displayed_region_number=(nav['waypoint_index'] + 1)
                if isinstance(nav.get('waypoint_index'), int) and nav.get('waypoint_index',0) < nav.get('total',0) else None,
            phase_region_total=nav.get('total'), current_goal=nav.get('current_goal'),
            goal_activated_ros_clock_ns=nav.get('goal_activated_ros_clock_ns'),
            current_request_reported_arrivals=len(nav.get('region_arrivals') or []),
            actual_SLAM_pose=nav.get('pose'), actual_source_pose_age_wall_s=nav.get('pose_age'),
            actual_source_cloud_age_wall_s=nav.get('cloud_age'), command=nav.get('command'),
            alignment_phase=nav.get('alignment_phase'), locked_heading=nav.get('locked_heading'),
            cascade_mode=cascade.get('mode'), cascade_reason=cascade.get('reason'),
            cascade_reference_yaw_rad=cascade.get('reference_yaw_rad'), cascade_error_yaw_rad=cascade.get('error_yaw_rad'),
            cascade_goal_xy_distance_m=cascade.get('goal_distance_xy_m'),
            cascade_goal_z_error_m=cascade.get('goal_z_error_m'),
            raw_goal_z_error_m=(nav.get('pose')[2] - nav.get('current_goal').get('center')[2])
                if isinstance(nav.get('pose'), list) and len(nav['pose']) == 3 and isinstance(nav.get('current_goal'), dict)
                and isinstance(nav['current_goal'].get('center'), list) and len(nav['current_goal']['center']) == 3 else None,
            cascade_remaining_horizontal_arc_m=cascade.get('remaining_horizontal_arc_m'),
            cascade_path_id=cascade.get('path_id'), cascade_math_updated=cascade.get('controller_updated'),
            reference_requests=nav.get('reference_requests'), accepted_replans=nav.get('replans'),
            obstacle_hold=nav.get('obstacle_hold'), obstacle_stops=nav.get('obstacle_stops'),
            tilt_hold=nav.get('tilt_hold'), tilt_stops=nav.get('tilt_stops'), nav_message=nav.get('message'),
            sources=[native_source, nav_source, mission_source, control_source], observation_only=True)
        append('STATUS_SAMPLES.jsonl', row)
        phase = (row['mission_stage'], row['mission_layer'], row['terrain_switch_count'])
        if previous_mission_phase is not None and phase != previous_mission_phase:
            resource_snapshot('phase_change:' + str(phase), receipt, isolation)
        previous_mission_phase = phase
        activation = (row['request_id'], row['waypoint_index'], row['goal_activated_ros_clock_ns'])
        if activation != previous_activation:
            append('REGION_EVENTS.jsonl', dict(kind='observed_activation',
                observation_sequence=sequence, actual_reported_activation_ns=row['goal_activated_ros_clock_ns'],
                request_id=row['request_id'], waypoint_index=row['waypoint_index'],
                current_goal=row['current_goal'], sampled_sim_time=row['sim_time']))
            previous_activation = activation
        for arrival in nav.get('region_arrivals') or []:
            key = (arrival.get('request_id'), arrival.get('goal_id'), arrival.get('stamp_ns'))
            if key in seen_arrivals:
                continue
            seen_arrivals.add(key)
            observationally_valid = (arrival.get('request_id') == nav.get('request_id')
                and arrival.get('goals_definition_sha256') == nav.get('goals_definition_sha256')
                and arrival.get('region_inside') is True and arrival.get('control_region_inside') is True
                and arrival.get('protected') is False and arrival.get('dwell_ns',0) >= 400_000_000
                and arrival.get('reason') == 'arrived')
            append('REGION_EVENTS.jsonl', dict(kind='observed_actual_arrival_receipt',
                observation_sequence=sequence, sampled_sim_time=row['sim_time'], receipt=arrival,
                basic_current_request_binding_valid=observationally_valid, formal_acceptance=False))
        print(json.dumps({k: row[k] for k in ('sequence','sim_time','mission_stage','nav_state',
            'waypoint_index','displayed_region_number','current_request_reported_arrivals','alignment_phase',
            'cascade_mode','cascade_goal_xy_distance_m','raw_goal_z_error_m','reference_requests','accepted_replans','native_fault',
            'native_command_expired','native_command_reason')}, ensure_ascii=False), flush=True)
        if (RUN / 'runtime_manifest.json').is_file():
            manifest, source = read('runtime_manifest.json')
            final = dict(schema='v25_small_status_observer_terminal/v1', run=str(RUN),
                terminal_observation=row, runtime_manifest_source=source,
                runtime_status=manifest.get('runtime_status'), runtime_error=manifest.get('error'),
                all_owned_and_children_clean=manifest.get('all_owned_and_children_clean'),
                pipeline_normal_completed=manifest.get('pipeline_normal_completed'),
                actual_arrival_receipts_observed=len(seen_arrivals),
                observation_only=True, independent_full_acceptance_pending=True)
            resource_snapshot('runtime_terminal:' + str(final['runtime_status']), receipt, isolation)
            (HERE / 'TERMINAL_OBSERVATION.json').write_text(json.dumps(final, ensure_ascii=False, indent=2) + '\n')
            print('TERMINAL ' + json.dumps({k: final[k] for k in ('runtime_status','runtime_error',
                'all_owned_and_children_clean','pipeline_normal_completed','actual_arrival_receipts_observed')},ensure_ascii=False),flush=True)
            return
        time.sleep(30)
    raise RuntimeError('Observation bound reached; no process was modified')


if __name__ == '__main__':
    main()
