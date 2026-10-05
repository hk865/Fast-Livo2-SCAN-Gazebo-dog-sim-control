#!/usr/bin/env python3
"""Append-only actual SLAM/IMU fixed-route Teacher transfer acceptance.

This program is an offline reader. It imports no ROS, never publishes commands,
and uses Gazebo state only AFTER the original sensor command chain is audited.
Frozen truth-v2 numerical criteria are retained, not its truth-arrival claim.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
TRUTH = HERE.parent/'truth_tuning'
EVALUATOR_SHA = '7e386d004e2d1187753609b73f6b9f419c417aa35c1a30147d28385ffcd9fb8a'
PROTOCOL_SHA = 'e2308b7b7585f933081cf741bd8670e950607cea495c3f4eca293e33ae7796a8'
MODEL_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
SCHEMA = 'independent_actual_SLAM_fixed_route_teacher_transfer/v1'
COMMAND_SCHEMA = 'teacher_slam_fixed_route_command/v1'
TTL = .3
FUTURE_CLOCK_TOLERANCE = .05


class MissingEvidence(Exception):
    """Absent evidence is unverified, not an invented success or a physical fail."""


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def read_json(path):
    if not Path(path).is_file():
        raise MissingEvidence('Missing original file: '+str(path))
    return json.loads(Path(path).read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError('Nonfinite '+x)))


def rows(path, optional=False):
    if optional and not Path(path).exists():
        return []
    if not Path(path).is_file():
        raise MissingEvidence('Missing original JSONL: '+str(path))
    output = []
    with Path(path).open() as stream:
        for i, line in enumerate(stream, 1):
            if line.strip():
                try:
                    output.append(json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x))))
                except (ValueError, TypeError) as exc:
                    raise ValueError(f'{path}:{i}: invalid original JSON') from exc
    return output


def integer(value, name):
    if type(value) is not int or value < 0:
        raise ValueError('Original nonnegative integer required: '+name)
    return value


def vector(value, width):
    a = np.asarray(value, float)
    if a.shape != (width,) or not np.isfinite(a).all():
        raise ValueError('Invalid finite vector of width '+str(width))
    return a


def rotation_xyzw(value):
    x, y, z, w = vector(value, 4)
    n = x*x+y*y+z*z+w*w
    if abs(n-1) > .02:
        raise ValueError('Non-unit recorded quaternion')
    x, y, z, w = np.asarray([x,y,z,w])/math.sqrt(n)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def angle(a, b):
    return np.arctan2(np.sin(a-b), np.cos(a-b))


def distribution(values):
    a = np.asarray(values, float)
    return None if not len(a) else {'count': len(a), 'min': float(a.min()),
        'p50': float(np.percentile(a,50)), 'p95': float(np.percentile(a,95)), 'max': float(a.max())}


def original_index(data, source):
    """Index accepted original headers only; duplicated receipt walls cannot win."""
    indexed = {}
    last = -1
    rejected = 0
    for row in data:
        ns = integer(row['stamp_ns'], source+' stamp_ns')
        if type(row.get('accepted')) is not bool:
            raise ValueError('Actual accepted boolean missing: '+source)
        if not row['accepted']:
            rejected += 1
            continue
        if ns <= last:
            raise ValueError('Duplicate/backwards accepted original '+source+' header')
        last = ns
        wall = float(row['received_monotonic_wall'])
        if not math.isfinite(wall):
            raise ValueError('Nonfinite original receipt wall')
        indexed[ns] = row
    return indexed, rejected


def bind_original(envelope, poses, imus, read_clock_ns, read_wall, max_gyro_gap=.02):
    """Causal source join; producer /clock tolerance is separate from feedback."""
    pns = integer(envelope['pose_stamp_ns'], 'pose_stamp_ns')
    gns = integer(envelope['gyro_stamp_ns'], 'gyro_stamp_ns')
    cns = integer(envelope['control_pose_stamp_ns'], 'control_pose_stamp_ns')
    rns = integer(read_clock_ns, 'Teacher read_clock_ns')
    if gns > pns or pns-gns > round(max_gyro_gap*1e9):
        raise ValueError('IMU is not a causal bounded past pair of original SLAM pose')
    if pns > rns or cns > pns:
        raise ValueError('Future SLAM/controller feedback at actual Teacher read')
    if pns not in poses or gns not in imus or cns not in poses:
        raise MissingEvidence('Used header absent from accepted original source logs')
    p, g = poses[pns], imus[gns]
    ages = {}
    for prefix, raw in (('pose',p), ('gyro',g)):
        if envelope[prefix+'_received_monotonic_wall'] != raw['received_monotonic_wall']:
            raise ValueError('Heartbeat refreshed original '+prefix+' receipt wall')
        sim_age = (rns-raw['stamp_ns'])/1e9
        wall_age = read_wall-float(raw['received_monotonic_wall'])
        if not 0 <= sim_age <= TTL+1e-12 or not 0 <= wall_age <= TTL+1e-12:
            raise ValueError('Original '+prefix+' source exceeds dual 300ms TTL')
        ages[prefix+'_sim_s'], ages[prefix+'_wall_s'] = sim_age, wall_age
    if not 0 <= (rns-cns)/1e9 <= TTL+1e-12:
        raise ValueError('Last actually used controller pose exceeds 300ms TTL')
    producer_sim = (rns-integer(envelope['clock_ns'], 'producer clock_ns'))/1e9
    producer_wall = read_wall-float(envelope['monotonic_wall'])
    if not -FUTURE_CLOCK_TOLERANCE <= producer_sim <= TTL+1e-12 or not 0 <= producer_wall <= TTL+1e-12:
        raise ValueError('Original producer clock/wall exceeds contractual TTL')
    ages.update(producer_clock_sim_s=producer_sim, producer_wall_s=producer_wall,
                control_pose_sim_s=(rns-cns)/1e9, gyro_pair_gap_s=(pns-gns)/1e9)
    return p, g, poses[cns], ages


def validate_anchor(anchor, poses, relative):
    ns = integer(anchor['original_stamp_ns'], 'anchor original_stamp_ns')
    if ns not in poses:
        raise MissingEvidence('Anchor header is absent from accepted actual SLAM')
    p = poses[ns]
    if (anchor.get('source') != '/demo/slam/body_odom' or anchor.get('navigation_ground_truth_used') is not False
        or anchor.get('route_reference_changes_after_anchor') != 0):
        raise ValueError('Anchor provenance/reference mutation violation')
    for ak, pk in (('original_position_camera_init','position'),('original_quaternion_xyzw','quaternion_xyzw'),
                   ('received_monotonic_wall','received_monotonic_wall')):
        if anchor[ak] != p[pk]:
            raise ValueError('Anchor did not retain exact original SLAM '+pk)
    R = rotation_xyzw(p['quaternion_xyzw']); yaw = math.atan2(R[1,0],R[0,0])
    c,s = math.cos(yaw),math.sin(yaw)
    Rh = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    rel = np.asarray(relative, float)
    route = rel@Rh.T+vector(p['position'],3)
    if (not np.allclose(anchor['horizontal_rotation'],Rh,atol=1e-12,rtol=0)
        or abs(float(angle(anchor['heading_rad'],yaw)))>1e-12
        or not np.array_equal(np.asarray(anchor['relative_points_xyz']),rel)
        or not np.allclose(anchor['fixed_route_camera_init_xyz'],route,atol=1e-12,rtol=0)):
        raise ValueError('Frozen route was not exactly derived from original SLAM anchor')
    return route, p


def dwell_evidence(updates, poses, imus, route, R_bi, completed_ns):
    """Only distinct source rows can advance .6s dwell; never timer heartbeats."""
    start = None
    last = None
    used = []
    target_yaw = math.atan2(*(route[-1,:2]-route[-2,:2])[::-1])
    for detail in updates:
        core, env = detail['core_original_row'], detail['envelope']
        ns = integer(env['control_pose_stamp_ns'], 'dwell source')
        if ns > completed_ns:
            break
        if last is not None and ns <= last:
            raise ValueError('Duplicate controller input used for source dwell')
        if last is not None and ns-last > 200000001:
            start = None; used = []
        last = ns
        if core.get('controller_updated') is not True:
            continue
        p = poses[ns]; g = imus[env['gyro_stamp_ns']]
        R = rotation_xyzw(p['quaternion_xyzw'])
        roll=math.atan2(R[2,1],R[2,2]); pitch=math.asin(float(np.clip(-R[2,0],-1,1)))
        omega = R_bi@vector(g['angular_velocity_sensor'],3)
        yawdot=(math.sin(roll)*omega[1]+math.cos(roll)*omega[2])/math.cos(pitch)
        distance=float(np.linalg.norm(vector(p['position'],3)[:2]-route[-1,:2]))
        valid=(distance <= (.14 if start is not None else .12)+1e-12
               and np.linalg.norm(vector(p['origin_linear_velocity_body'],3)[:2]) < .08
               and abs(yawdot)<.1 and abs(float(angle(target_yaw,math.atan2(R[1,0],R[0,0]))))<.2
               and core['mode'] in ('goal_dwell','parking'))
        if valid:
            if start is None:
                start=ns; used=[]
            used.append(ns)
        else:
            start=None; used=[]
    return {'passed': bool(start is not None and used and used[-1] == completed_ns and (completed_ns-start)>=600000000),
            'source_start_ns':start, 'source_end_ns':completed_ns,
            'distinct_header_count':len(used), 'headers_ns':used,
            'duration_s':None if start is None else (completed_ns-start)/1e9,
            'entry_radius_m':.12, 'hysteresis_hold_radius_m':.14, 'maximum_header_gap_s':.2}


def source_hashes(run, execution, manifest):
    verified = {}; current = {}; missing=[]; mismatches=[]
    snapshots=execution.get('source_snapshots',{})
    for name,digest in manifest.items():
        p=Path(name)
        if p.is_absolute():
            original=p
            snap=snapshots.get(name)
            if snap:
                p=Path(snap['snapshot'])
                if snap.get('sha256')!=digest:
                    mismatches.append(name+': snapshot declaration')
            elif p.is_file():
                current[name]=sha(p)==digest
            else:
                missing.append(name); continue
            if original.is_file():
                current[name]=sha(original)==digest
        else:
            p=run/'sources'/p
        if not p.is_file():
            missing.append(str(p)); continue
        actual=sha(p); verified[str(p)]=actual
        if actual!=digest:
            mismatches.append(str(p))
    for name,digest in execution.get('immutable_generated_files',{}).items():
        p=Path(name)
        if not p.is_file():
            missing.append(str(p)); continue
        actual=sha(p); verified[str(p)]=actual
        if actual!=digest:
            mismatches.append(str(p))
    return verified,current,missing,mismatches


def child_cleanup(log, expected):
    if not Path(log).is_file():
        raise MissingEvidence('Original SLAM launch child log missing')
    text=Path(log).read_text(errors='replace')
    started=[int(x) for x in re.findall(r'process started with pid \[(\d+)\]',text)]
    clean=[int(x) for x in re.findall(r'process has finished cleanly \[pid (\d+)\]',text)]
    bad=re.findall(r'process has died[^\n]*',text)
    return {'passed':len(started)==expected and len(set(started))==expected and sorted(started)==sorted(clean) and not bad,
            'expected_count':expected,'original_started_pids':started,'original_clean_exit_pids':clean,'abnormal_exit_lines':bad}


def evaluate(run, *, write=True, suffix=None, helper_path=None, protocol_path=None):
    run=Path(run).resolve()
    if suffix is not None and not re.fullmatch(r'[A-Za-z0-9_-]+',suffix):
        raise ValueError('Unsafe receipt suffix')
    output=run/('summary_slam_transfer_independent'+('.'+suffix if suffix else '')+'.json')
    if write and output.exists():
        raise ValueError('Refusing to overwrite an existing independent receipt')
    helper_path=Path(helper_path or TRUTH/'evaluate.py').resolve()
    protocol_path=Path(protocol_path or TRUTH/'protocol.json').resolve()
    result={'schema':SCHEMA,'run':str(run),'status':'unverified','score':None,
        'scope':'Actual SLAM/IMU fixed body-relative route transfer; privileged CPU Teacher in simulation',
        'navigation_ground_truth_used':False,'actual_SLAM_fixed_route_verified':False,
        'SCAN_navigation_verified':False,'full_multifloor_navigation_verified':False,'real_robot_verified':False,
        'Sim2Sim_general_acceptance':False,'actor_privileged_observations':True,
        'absolute_original_scene_centerline':{'status':'unverified','reason':'Body-relative SLAM anchor is not an independently registered scene centerline'},
        'checks':{},'input_sha256':{},'receipt_suffix':suffix,
        'limitations':['Gazebo pose is used only for offline execution measurement, never to choose command, anchor, waypoint or source arrival.',
            'The actor retains 232 privileged observation dimensions: COM velocity, angular velocity, gravity, q/qd, applied torque and 187 static-terrain rays; only 3 command and 12 previous-action dimensions are controller-known.',
            'Mapped SLAM supplies at most 10 fresh headers per second; a selected 25Hz law is an explicitly reduced-rate migration with unchanged gains.',
            'No SCAN planner, cloud obstacle guard, full multifloor route, or hardware acceptance is established.',
            'Producer clock has the frozen 50ms future tolerance; actual used SLAM feedback and gyro pairing are checked strictly causal.',
            'Raw clock callbacks are not independently archived; envelope/read clocks establish normal-run ordering, not every clock-pause failure scenario.',
            'Native tau is a command fed to physics, not measured motor torque.']}
    def check(name,passed,*,unverified=False,**values):
        result['checks'][name]={'status':'passed' if passed else 'unverified' if unverified else 'failed',**values}
    def remember(path):
        p=Path(path)
        if p.is_file():result['input_sha256'][str(p.resolve())]=sha(p)
    try:
        if sha(helper_path)!=EVALUATOR_SHA or sha(protocol_path)!=PROTOCOL_SHA:
            raise ValueError('Frozen v2 native helper/protocol hash differs from prospective validator contract')
        remember(helper_path);remember(protocol_path);remember(__file__)
        protocol=read_json(protocol_path)
        spec=importlib.util.spec_from_file_location('_independent_frozen_truth_v2_native_reader',helper_path)
        common=importlib.util.module_from_spec(spec);spec.loader.exec_module(common)
        files=['slam_execution.json','slam_binding.json','source_manifest.json','frozen_controller_profile.json',
            'runtime_manifest.json','worker_result.json','policy_manifest.json','telemetry.jsonl',
            'slam_execution_commands.jsonl','actuator.jsonl','world.sdf','asset_manifest.json']
        for name in files:remember(run/name)
        execution=read_json(run/'slam_execution.json');binding=read_json(run/'slam_binding.json')
        manifest=read_json(run/'source_manifest.json');profile=read_json(run/'frozen_controller_profile.json')
        runtime=read_json(run/'runtime_manifest.json');worker=read_json(run/'worker_result.json')
        policy=read_json(run/'policy_manifest.json')
        folder=run/'SLAM_fixed_route'
        check('runtime_as_originally_recorded',runtime.get('error') is None
            and runtime.get('runtime_status')=='completed_requires_independent_route_evaluation',
            runtime_error=runtime.get('error'),original_runtime_status=runtime.get('runtime_status'))
        writer_preflight=read_json(folder/'adapter_writer_receipt.json')
        check('recorder_as_originally_recorded',writer_preflight.get('status')=='drained'
            and writer_preflight.get('error') is None and writer_preflight.get('pending_records')==0,
            original_writer_receipt=writer_preflight)
        rawfiles=['adapter_source_receipt.json','adapter_slam_poses.jsonl','adapter_IMU_inputs.jsonl',
            'adapter_commands.jsonl','adapter_controller_updates.jsonl','adapter_route_anchor.jsonl',
            'adapter_publisher_graph.jsonl','adapter_writer_receipt.json','adapter_cleanup_receipt.json']
        for name in rawfiles:remember(folder/name)
        source=read_json(folder/'adapter_source_receipt.json')
        poses_raw=rows(folder/'adapter_slam_poses.jsonl');imus_raw=rows(folder/'adapter_IMU_inputs.jsonl')
        poses,pose_rejected=original_index(poses_raw,'SLAM');imus,imu_rejected=original_index(imus_raw,'IMU')
        commands=rows(folder/'adapter_commands.jsonl');details=rows(folder/'adapter_controller_updates.jsonl')
        anchors=rows(folder/'adapter_route_anchor.jsonl');graphs=rows(folder/'adapter_publisher_graph.jsonl')
        telemetry=rows(run/'telemetry.jsonl');reads=rows(run/'slam_execution_commands.jsonl')
        if not poses or not imus or not details or len(telemetry)<2 or not reads:
            raise MissingEvidence('Insufficient actual sensor/controller/executor rows')
        verified,current,missing,mismatches=source_hashes(run,execution,manifest)
        check('executed_source_and_generated_input_hashes',not missing and not mismatches,
            unverified=bool(missing) and not mismatches,verified_archived_sources=verified,
            current_original_source_equality=current,missing=missing,mismatches=mismatches,
            historical_current_source_change_does_not_replace_archived_executed_source=True)
        effective=dict(source['effective_profile']);selected=dict(profile)
        effective.pop('route_world_xyz',None);selected.pop('route_world_xyz',None)
        selected_hz=int(profile['feedback_hz']);migration=binding.get('allow_reduced_source_frequency') is True
        check('explicit_unchanged_law_rate_migration',canonical(effective)==canonical(selected)
            and source['profile_sha256']==sha(run/'frozen_controller_profile.json')
            and source['config_sha256']==sha(run/'slam_binding.json')==execution['binding_sha256']
            and source['effective_profile_sha256']==hashlib.sha256(canonical(source['effective_profile']).encode()).hexdigest()
            and source['controller_sha256']==binding['controller_source']['sha256']
            and (selected_hz<=10 or migration) and execution['selected_calibration_hz']==selected_hz
            and execution['fresh_SLAM_expected_hz']==10,
            selected_calibration_hz=selected_hz,source_feedback_ceiling_hz=10,
            explicit_allow_source_rate_reduction=migration,gains_unchanged_except_route=True)
        scope_ok=(execution.get('schema')=='teacher_slam_fixed_route_execution/v1'
            and execution.get('navigation_ground_truth_used') is False and execution.get('real_robot') is False
            and execution.get('SCAN_started') is False and execution.get('CHAMP_started') is False
            and execution.get('old_PID_controller_started') is False
            and execution.get('exclusive_writer')=='teacher_sim::TeacherActuator'
            and execution.get('model_sha256')==MODEL_SHA and execution.get('actor_observations_remain_privileged') is True)
        plugins=ET.parse(run/'world.sdf').getroot().find("world/model[@name='go2']").findall('plugin')
        names=[p.get('name','') for p in plugins]
        scope_ok=scope_ok and sum('TeacherActuator' in name for name in names)==1 and not any('champ' in name.lower() or 'ros2_control' in name.lower() for name in names)
        checkpoint=Path(policy['checkpoint']);remember(checkpoint)
        check('scope_and_unique_cpu_actor_writer',scope_ok and policy.get('checkpoint_sha256')==MODEL_SHA and sha(checkpoint)==MODEL_SHA
            and policy.get('inference_device')=='cpu' and policy.get('torch_threads')==1,
            actual_model_plugins=names,actor_observation_source=policy.get('observation_source'),
            privileged_dimensions=232,controller_known_dimensions=15,SCAN_started=False,model_sha256=MODEL_SHA)
        # Raw source validation is independent of the adapter's accepted flag.
        source_errors=[]
        for ns,p in poses.items():
            if p['frame_id']!='camera_init' or p['child_frame_id']!='demo_slam_body':source_errors.append('SLAM frame')
            vector(p['position'],3);rotation_xyzw(p['quaternion_xyzw']);vector(p['origin_linear_velocity_body'],3)
            if np.any(vector(p['twist_covariance_diagonal'],6)>=1e5):source_errors.append('invalid first-difference velocity')
        for ns,g in imus.items():
            if g['frame_id']!=binding['imu_frame'] or type(g.get('orientation_available')) is not bool:source_errors.append('IMU frame/orientation availability metadata')
            if g.get('orientation_available') is True:rotation_xyzw(g['quaternion_xyzw'])
            vector(g['angular_velocity_sensor'],3)
        check('original_sensor_headers_and_frames',not source_errors,accepted_SLAM_headers=len(poses),accepted_IMU_headers=len(imus),
            explicitly_unaccepted_SLAM_rows=pose_rejected,explicitly_unaccepted_IMU_rows=imu_rejected,
            accepted_IMU_orientation_unavailable_rows=sum(g.get('orientation_available') is False for g in imus.values()),
            gyro_only_without_declared_orientation_is_allowed=True,violations=source_errors)
        if binding['fixed_route']['registration']['source']!='actual_SLAM_initial_body_relative_route':
            raise MissingEvidence('This validator version only accepts the prospective actual-body-relative fixed-route provider')
        if len(anchors)!=1:raise ValueError('Exactly one immutable original SLAM route anchor required')
        route,anchor_pose=validate_anchor(anchors[0],poses,binding['fixed_route']['points_xyz'])
        anchor_ns=anchors[0]['original_stamp_ns']
        check('one_original_SLAM_route_anchor',True,anchor=anchors[0],route_camera_init_xyz=route.tolist(),
            truth_registration_used=False,absolute_original_scene_centerline_verified=False)
        # Every produced healthy envelope must retain original sensor walls and causal gyro.
        by_seq={};prod_errors=[];graph_times=np.asarray([float(g['monotonic_wall']) for g in graphs])
        producer_clock=[];lastseq=0;lastclock=-1
        for env in commands:
            seq=integer(env['sequence'],'producer sequence')
            if seq<=lastseq:raise ValueError('Producer sequence duplicate/backwards')
            lastseq=seq;by_seq[seq]=env
            if env['clock_ns'] is not None:
                clock=integer(env['clock_ns'],'producer clock')
                if clock<lastclock:prod_errors.append('clock backwards')
                lastclock=clock
            if env.get('healthy') is not True:
                if np.any(vector(env['command'],3)):prod_errors.append('unhealthy producer emitted nonzero')
                continue
            if env.get('schema')!=COMMAND_SCHEMA or env.get('ground_truth_navigation') is not False:prod_errors.append('schema/truth')
            if env.get('config_sha256')!=execution['binding_sha256'] or env.get('profile_sha256')!=source['profile_sha256'] or env.get('controller_sha256')!=source['controller_sha256']:prod_errors.append('source hash')
            # Production has documented callback clock tolerance. Strict feedback
            # is established at the Teacher read below, not fabricated here.
            pns=integer(env['pose_stamp_ns'],'producer pose');gns=integer(env['gyro_stamp_ns'],'producer gyro')
            if gns>pns or pns-gns>round(binding.get('maximum_gyro_pair_gap_s',.02)*1e9):prod_errors.append('future/unpaired gyro')
            for prefix,index in (('pose',poses),('gyro',imus)):
                ns=env[prefix+'_stamp_ns']
                if ns not in index:prod_errors.append('missing original '+prefix);continue
                raw=index[ns]
                if env[prefix+'_received_monotonic_wall']!=raw['received_monotonic_wall']:prod_errors.append('refreshed receipt wall')
                a=(env['clock_ns']-ns)/1e9;w=env['monotonic_wall']-raw['received_monotonic_wall']
                if not -.05<=a<=.3+1e-12 or not 0<=w<=.3+1e-12:prod_errors.append('producer source TTL')
            gi=np.searchsorted(graph_times,env['monotonic_wall'],side='right')-1
            if gi<0 or env['monotonic_wall']-graph_times[gi]>1.+1e-9 or graphs[gi].get('valid') is not True:prod_errors.append('publisher graph stale/invalid')
        check('producer_original_headers_dual_TTL_and_graph',not prod_errors,violations=prod_errors[:50],
            violation_count=len(prod_errors),producer_rows=len(commands),clock_tolerance_s=[-.05,.3])
        scenario_path=Path(binding['sensor_scenario']['path'])
        if not scenario_path.is_absolute():scenario_path=run/scenario_path
        scenario=read_json(scenario_path);imu_scenario=scenario['sensors']['imu']
        R_bi=np.asarray(source['body_from_imu_rotation'],float)
        check('IMU_mount_and_COM_extrinsics_from_frozen_scenario',R_bi.shape==(3,3)
            and np.allclose(R_bi,rotation_xyzw(imu_scenario['orientation_reference']['body_imu_quaternion']),atol=1e-12,rtol=0)
            and binding['imu_frame']==imu_scenario['frame']
            and np.array_equal(np.asarray(source['body_com_offset']),np.asarray(profile['base_com_offset'])),
            source_scenario_sha256=sha(scenario_path),body_from_IMU_rotation=R_bi.tolist(),body_COM_offset=source['body_com_offset'])
        controls={};ct=[];mathstamps=[];controller_errors=[]
        for detail in details:
            env=detail['envelope'];core=detail['core_original_row'];ns=integer(env['control_pose_stamp_ns'],'control original stamp')
            if ns in controls:controller_errors.append('duplicate core invocation')
            controls[ns]=core;ct.append(ns)
            if by_seq.get(env['sequence'])!=env:controller_errors.append('detail envelope not original producer row')
            if detail['pose_source']['stamp_ns']!=ns or ns!=env['pose_stamp_ns']:controller_errors.append('controller pose mismatch')
            p=poses[ns];g=imus[env['gyro_stamp_ns']]
            for k in ('position','quaternion_xyzw','origin_linear_velocity_body','received_monotonic_wall'):
                if detail['pose_source'][k]!=p[k]:controller_errors.append('modified source pose '+k)
            if detail['paired_IMU_source']['angular_velocity_sensor']!=g['angular_velocity_sensor']:controller_errors.append('modified source gyro')
            omega=R_bi@vector(g['angular_velocity_sensor'],3)
            if not np.allclose(detail['paired_IMU_source']['gyro_body'],omega,atol=1e-12,rtol=0):controller_errors.append('gyro frame transform')
            if abs(float(core['feedback_time_s'])-ns/1e9)>1e-8 or abs(float(core['control_t_s'])-ns/1e9-.005)>1e-8:controller_errors.append('source timestamp replaced')
            if type(core.get('controller_updated')) is not bool:controller_errors.append('missing update boolean')
            if core.get('controller_updated') is True:
                mathstamps.append(ns)
                com=vector(p['origin_linear_velocity_body'],3)+np.cross(omega,vector(profile['base_com_offset'],3))
                if not np.allclose(core['measured_COM_velocity_body'],com,atol=1e-10,rtol=0):controller_errors.append('COM lever arm')
            if env['fresh_source_controller_updated'] is not core['controller_updated']:controller_errors.append('heartbeat update flag')
            if vector(core['command_body'],3).tolist()!=env['command']:controller_errors.append('command differs from actual core')
        gaps=np.diff(ct)/1e9;mgaps=np.diff(mathstamps)/1e9
        check('fresh_actual_source_updates_not_cached_timer_dwell',not controller_errors and len(mathstamps)>1
            and np.all(gaps>=1/min(10,selected_hz)-2e-9) and np.all(np.diff(ct)>0),
            actual_source_invocations=len(ct),actual_math_update_rows=len(mathstamps),nonmath_initial_or_parking_rows=len(ct)-len(mathstamps),
            source_invocation_gaps_s=distribution(gaps),math_update_gaps_s=distribution(mgaps),
            observed_math_rate_hz=(len(mathstamps)-1)/((mathstamps[-1]-mathstamps[0])/1e9) if len(mathstamps)>1 else None,
            selected_calibration_hz=selected_hz,actual_feedback_ceiling_hz=10,violations=controller_errors[:50])
        # Actual executor reads: original bytes, this-attempt decoded row, not cached accepted fallback.
        read_errors=[];ages={};readmap={};lastseq=0;lastdigest=None;accepted=0;rejected=0
        for row in reads:
            clock=integer(row['physics_clock_ns'],'executor native clock');attempt=row['actual_read_attempt'];env=row.get('actual_original_envelope')
            if clock in readmap:raise ValueError('Duplicate actual executor native clock')
            readmap[clock]=row
            if attempt.get('read_clock_ns')!=clock or attempt.get('read_monotonic_wall')!=row['read_monotonic_wall']:read_errors.append('attempt clock/wall replaced')
            if attempt.get('decoded_envelope')!=env:read_errors.append('current attempt relabeled as previous accepted envelope')
            if 'raw_utf8' in attempt:
                raw=attempt['raw_utf8'].encode('utf-8')
                if sha_bytes(raw)!=attempt['raw_bytes_sha256'] or len(raw)!=attempt['raw_byte_count'] or json.loads(raw)!=env:read_errors.append('raw byte identity')
            elif row.get('rejected') is not True:read_errors.append('accepted read has no original bytes')
            if row.get('navigation_ground_truth_used') is not False or row.get('changed_source_fingerprints'):read_errors.append('changed source/truth command')
            if row.get('rejected') is True:
                rejected+=1
                if np.any(vector(row['command'],3)):read_errors.append('rejected read did not request zero')
                continue
            accepted+=1
            if attempt.get('status')!='accepted' or env.get('healthy') is not True:read_errors.append('false accepted status')
            if by_seq.get(env['sequence'])!=env:read_errors.append('accepted bytes absent/different in original producer archive')
            digest=hashlib.sha256(canonical(env).encode()).hexdigest()
            if env['sequence']<lastseq or env['sequence']==lastseq and digest!=lastdigest:read_errors.append('repeated sequence changed content')
            lastseq,lastdigest=env['sequence'],digest
            _,_,_,sampleages=bind_original(env,poses,imus,clock,float(row['read_monotonic_wall']),binding.get('maximum_gyro_pair_gap_s',.02))
            for key,value in sampleages.items():ages.setdefault(key,[]).append(value)
            ages.setdefault('pose_relative_cached_physics_state_age_s',[]).append(sampleages['pose_sim_s']-.005)
            expected=np.zeros(3) if env['stop_requested'] else vector(env['command'],3)
            if not np.array_equal(vector(row['command'],3),expected):read_errors.append('accepted command changed')
            if env['control_pose_stamp_ns'] not in controls:read_errors.append('used control source missing actual core row')
        check('actual_Teacher_reads_original_bytes_and_causal_sensor_feedback',not read_errors and accepted>0,
            accepted_reads=accepted,rejected_reads=rejected,violations=read_errors[:50],age_distributions={k:distribution(v) for k,v in ages.items()},
            actual_pose_future_feedback_count=0,producer_clock_negative_age_count=int(sum(x<0 for x in ages.get('producer_clock_sim_s',[]))),
            contractual_producer_clock_tolerance_s=[-.05,.3],source_gyro_strictly_past_pose=True)
        # Telemetry command source must be the same actual read and unchanged external slew.
        world=np.asarray([float(r['world_sim_time']) for r in telemetry]);physical=np.asarray([float(r['state_physics_world_time']) for r in telemetry])
        elapsed=np.asarray([float(r['sim_time']) for r in telemetry]);requested=np.asarray([vector(r['requested'],3) for r in telemetry]);cmd=np.asarray([vector(r['command'],3) for r in telemetry])
        telemetry_errors=[];mapped=[];joined=[];oldcmd=np.zeros(3)
        for i,row in enumerate(telemetry):
            read=readmap.get(round(world[i]*1e9))
            if read is None:raise MissingEvidence('Actual Teacher frame lacks original command read row')
            if row.get('slam_command_read_evidence')!=read:telemetry_errors.append('telemetry read mismatch')
            if row.get('outer_navigation_ground_truth_used') is not False or row.get('privileged_actor_observations') is not True:telemetry_errors.append('incorrect source boundary')
            if not np.array_equal(requested[i],vector(read['command'],3)):telemetry_errors.append('requested mismatch')
            expected=oldcmd+np.clip(requested[i]-oldcmd,-np.array([.6,.6,.8])*.02,np.array([.6,.6,.8])*.02)
            if not np.allclose(cmd[i],expected,atol=1e-12,rtol=0):telemetry_errors.append('external command slew changed')
            oldcmd=cmd[i]
            env=read['actual_original_envelope']
            core=None if read['rejected'] or not isinstance(env,dict) else controls.get(env['control_pose_stamp_ns'])
            mapped.append(core);joined.append(env if not read['rejected'] else None)
        offset=float(world[0]-elapsed[0]);maxgap=float(np.max(np.diff(physical)))
        check('actual_read_to_Teacher_command_slew_and_time',not telemetry_errors and len(reads)==len(telemetry)
            and np.all(np.diff(world)>0) and maxgap<=protocol['time']['maximum_telemetry_gap_s']
            and np.max(abs(physical-(world-.005)))<=1e-8 and np.ptp(world-elapsed)<=1e-8,
            rows=len(telemetry),read_rows=len(reads),maximum_telemetry_gap_s=maxgap,violations=telemetry_errors[:50],
            actor_slew_per_second=[.6,.6,.8],physical_pose_offset_s=-.005)
        # Runtime requires all six owned roles AND original six SLAM child exits.
        required=set(execution['required_roles']);owned=runtime.get('owned_processes',[])
        clean=runtime.get('error') is None and set(r.get('role') for r in owned)==required and len(owned)==6 and all(r.get('returncode')==0 for r in owned)
        children=child_cleanup(run/'SLAM.log',int(execution['expected_SLAM_children']))
        check('six_owned_and_six_SLAM_children_clean',clean and children['passed'],owned_processes=owned,SLAM_children=children,
            runtime_error=runtime.get('error'),actor_device=runtime.get('actor_device'),training_processes_signaled=runtime.get('training_processes_signaled'))
        writer=read_json(folder/'adapter_writer_receipt.json');cleanup=read_json(folder/'adapter_cleanup_receipt.json')
        counts={name:len(rows(folder/name)) for name in writer.get('written_records',{})}
        check('source_recorder_drained_and_cleaned',writer.get('status')=='drained' and writer.get('error') is None
            and writer.get('pending_records')==0 and counts==writer.get('written_records') and cleanup.get('status')=='completed'
            and cleanup.get('primary_error') is None and not cleanup.get('cleanup_errors'),writer=writer,cleanup=cleanup,actual_record_counts=counts)
        # Arrival is reconstructed ONLY from original source headers and fixed SLAM map route.
        visits=[];cursor=anchor_ns
        ordered=list(poses)
        for number,goal in enumerate(route[1:],1):
            candidates=[ns for ns in ordered if ns>cursor and np.linalg.norm(vector(poses[ns]['position'],3)[:2]-goal[:2])<=.15]
            if not candidates:break
            ns=candidates[0];cursor=ns;visits.append({'waypoint_index':number,'original_pose_stamp_ns':ns,
                'original_position_camera_init':poses[ns]['position'],'goal_camera_init':goal.tolist(),
                'distance_m':float(np.linalg.norm(vector(poses[ns]['position'],3)[:2]-goal[:2]))})
        completed=[integer(e['controller_completed_pose_stamp_ns'],'completion stamp') for e in commands if e.get('controller_completed_pose_stamp_ns') is not None]
        completion=completed[0] if completed else None
        if completion is None:
            check('source_arrival_and_nonduplicated_dwell',False,unverified=True,ordered_visits=visits,reason='No completed original SLAM source header')
        else:
            dwell=dwell_evidence(details,poses,imus,route,R_bi,completion)
            comp_elapsed=[e['controller_completed_elapsed_s'] for e in commands if e.get('controller_completed_pose_stamp_ns') is not None]
            consistent=all(ns==completion for ns in completed) and all(abs(float(t)-(completion-anchor_ns)/1e9)<=1e-8 for t in comp_elapsed)
            check('source_arrival_and_nonduplicated_dwell',len(visits)==len(route)-1 and dwell['passed'] and consistent,
                ordered_visits=visits,dwell=dwell,completed_original_pose_stamp_ns=completion,
                navigation_ground_truth_arrival_used=False)
        observed=worker.get('controller_completed_observed_worker_elapsed_s')
        expected_end=None if observed is None else min(float(execution['duration_s']),float(observed)+float(execution['post_completion_s']))
        termination=completion is not None and observed is not None and expected_end is not None and worker.get('completed') is True and worker.get('fault') is None and worker.get('samples')==len(telemetry) and abs(float(worker['last_sim_time'])-elapsed[-1])<=1e-8 and abs(elapsed[-1]-expected_end)<=.021
        check('declared_source_completion_and_run_termination',termination,unverified=completion is None,
            controller_completed_source_stamp_ns=completion,worker_first_observed_completion_elapsed_s=observed,
            expected_end_elapsed_s=expected_end,actual_end_elapsed_s=float(elapsed[-1]),post_completion_s=execution['post_completion_s'])
        # Native evidence: same frozen helper, every original 200Hz sample.
        native,n=common.native_evidence(run);nt=n['state_t'];nr=common.rotation_wxyz(n['quaternion_wxyz'])
        ni=np.searchsorted(nt,physical);ni=np.clip(ni,0,len(nt)-1);prev=np.maximum(0,ni-1);ni=np.where(abs(nt[prev]-physical)<abs(nt[ni]-physical),prev,ni)
        pair_error=max(float(np.max(abs(n['position'][ni]-np.asarray([r['position'] for r in telemetry])))),
                       float(np.max(abs(n['body_com'][ni]-np.asarray([r['body_lin_vel'] for r in telemetry])))),
                       float(np.max(abs(n['qd'][ni]-np.asarray([r['qd'] for r in telemetry])))),
                       float(np.max(abs(n['quaternion_wxyz'][ni]-np.asarray([r['quaternion_wxyz'] for r in telemetry])))))
        check('independent_native_200Hz_coverage_and_phase',np.all(np.diff(n['iteration'])==1)
            and np.all(abs(n['dt']-.005)<=1e-8) and np.all(abs(np.diff(n['world_t'])-.005)<=1e-8)
            and np.max(abs(nt[ni]-physical))<=1e-8 and pair_error<=1e-8
            and nt[0]<=physical[0]+1e-8 and nt[-1]>=physical[-1]-1e-8,
            native_rows=len(native),maximum_native_gap_s=float(np.max(np.diff(nt))),maximum_state_pair_error=pair_error,
            state_phase='Native PreUpdate cached physical state = t-dt; command tau fed for current physics step')
        audit=nt>=offset+.1-.005;faults=[r for r in native if r.get('fault') not in (None,0,'')]
        limits=protocol['safety'];rp=float(abs(n['roll_pitch'][audit]).max());clear=float(n['clearance'][audit].min());tau=float(abs(n['tau'][audit]).max());qd=float(abs(n['qd'][audit]).max());contact=int((n['contacts'][audit,0]>0).sum())
        check('independent_200Hz_motion_safety',rp<=limits['maximum_abs_roll_pitch_rad'] and clear>=limits['minimum_body_clearance_m']
            and tau<=limits['maximum_abs_applied_torque_Nm'] and qd<=limits['maximum_abs_joint_velocity_radps'] and contact==0 and not faults and worker.get('fault') is None
            and all(r.get('fault') in (None,0,'') for r in telemetry),maximum_abs_roll_pitch_rad=rp,minimum_clearance_m=clear,
            maximum_abs_force_command_Nm=tau,maximum_abs_joint_velocity_radps=qd,body_contact_rows=contact,native_faults=len(faults),worker_fault=worker.get('fault'),thresholds=limits)
        # One offline yaw+translation alignment at original SLAM anchor. It never
        # feeds back, never optimizes against route, and is not scene registration.
        ai=int(np.searchsorted(nt,anchor_ns/1e9,side='right')-1)
        if ai<0 or anchor_ns/1e9-nt[ai]>.00500001:raise MissingEvidence('No causal native state within one step of original SLAM anchor for offline measurement')
        slam_yaw=float(anchors[0]['heading_rad']);native_yaw=math.atan2(nr[ai,1,0],nr[ai,0,0]);dyaw=float(angle(native_yaw,slam_yaw));c,s=math.cos(dyaw),math.sin(dyaw)
        A=np.array([[c,-s,0],[s,c,0],[0,0,1]]);translation=n['position'][ai]-A@vector(anchor_pose['position'],3)
        offline_route=route@A.T+translation
        result['offline_execution_alignment']={'source_anchor_stamp_ns':anchor_ns,'causal_native_physical_time_s':float(nt[ai]),
            'native_anchor_index':ai,'rotation_map_to_native_world':A.tolist(),'translation_map_to_native_world':translation.tolist(),
            'used_for_commands_or_source_arrival':False,'absolute_scene_centerline_registration':False}
        native_read_idx=np.searchsorted(world,n['world_t'],side='right')-1;bound=np.clip(native_read_idx,0,len(telemetry)-1)
        active=(native_read_idx>=0)&(n['world_t']-world[bound]<=.04100001)&(nt>=anchor_ns/1e9)
        mode=np.asarray(['protection_zero' if core is None else core['mode'] for core in mapped])
        refv=np.asarray([[0,0,0] if core is None else core.get('reference_velocity_world',[0,0,0]) for core in mapped],float)@A.T
        refyaw=np.asarray([0 if core is None else core.get('reference_yaw',0) for core in mapped])+dyaw
        speedref=np.linalg.norm(refv[bound,:2],axis=1);moving=active&(mode[bound]=='drive')&(speedref>.03)
        distances,_=common.polyline_distance(n['position'],offline_route)
        route_max=float(distances[active].max()) if active.any() else None
        rms=float(np.sqrt(np.mean(distances[moving]**2))) if moving.any() else None
        check('offline_actual_anchored_route_execution',moving.any() and route_max<=.20 and rms<=.08,
            maximum_xy_distance_m=route_max,drive_distance_rms_m=rms,limits_m=[.20,.08],
            route_frame='single causal anchor yaw/translation of actual SLAM fixed body-relative route; no fit to subsequent truth')
        nyaw=np.arctan2(nr[:,1,0],nr[:,0,0]);heading=abs(angle(nyaw,refyaw[bound]));hm=float(heading[moving].max()) if moving.any() else None
        check('offline_actual_drive_heading',hm is not None and hm<=.2,maximum_error_rad=hm,limit_rad=.2,
            peak_native_state_time_s=None if not moving.any() else float(nt[np.flatnonzero(moving)[np.argmax(heading[moving])]]))
        worldv=np.einsum('nij,nj->ni',nr,n['body_com']);direction=np.zeros((len(nt),2));np.divide(refv[bound,:2],speedref[:,None],out=direction,where=speedref[:,None]>1e-12)
        along=np.sum(worldv[:,:2]*direction,axis=1);first=nt[np.flatnonzero(moving)[0]] if moving.any() else math.inf
        stable=moving&(nt>=first+1)&(speedref>=.8*float(profile['desired_speed']))
        duration=float(np.sum(n['dt'][stable]));meanref=float(np.mean(speedref[stable])) if stable.any() else None
        error=float(np.mean(abs(along[stable]-speedref[stable]))) if stable.any() else None
        speedlimit=max(.05,.25*meanref) if meanref is not None else None
        check('offline_actual_stable_COM_speed',duration>=.5 and error is not None and error<=speedlimit,
            stable_duration_s=duration,desired_speed_mps=profile['desired_speed'],mean_reference_mps=meanref,
            mean_actual_projection_mps=float(np.mean(along[stable])) if stable.any() else None,
            mean_absolute_error_mps=error,limit_mps=speedlimit,source='Every native body COM velocity rotated by original quaternion')
        # Parking trigger is ORIGINAL SLAM completion plus first actually read
        # parking zero. Native truth is never used to select a later quiet window.
        eligible=[i for i,(core,env) in enumerate(zip(mapped,joined)) if completion is not None and core is not None
            and core['mode']=='parking' and env['pose_stamp_ns']>=completion
            and np.max(abs(requested[i]))<=1e-9 and np.max(abs(cmd[i]))<=1e-9
            and np.max(abs(vector(core['reference_velocity_world'],3)))<=1e-9]
        parking=None
        if eligible:
            pi=eligible[0];start=float(physical[pi]);end=start+5
            tm=(physical>=start-1e-8)&(physical<=end+1e-8);nm=(nt>=start-1e-8)&(nt<=end+1e-8)
            ps=[p for ns,p in poses.items() if start-1e-8<=ns/1e9<=end+1e-8]
            pt=np.asarray([p['stamp_ns']/1e9 for p in ps]);pp=np.asarray([p['position'] for p in ps]);py=np.asarray([math.atan2(rotation_xyzw(p['quaternion_xyzw'])[1,0],rotation_xyzw(p['quaternion_xyzw'])[0,0]) for p in ps])
            source_complete=len(ps)>1 and pt[0]<=start+.20000001 and pt[-1]>=end-.20000001 and np.max(np.diff(pt))<=.200000001
            native_complete=nm.any() and nt[nm][-1]>=end-1e-8 and np.sum(nm)>=1000
            drift=float(np.linalg.norm(n['position'][nm,:2]-n['position'][np.flatnonzero(nm)[0],:2],axis=1).max()) if nm.any() else None
            yd=float(abs(np.unwrap(nyaw[nm])-nyaw[nm][0]).max()) if nm.any() else None
            slamdrift=float(np.linalg.norm(pp[:,:2]-pp[0,:2],axis=1).max()) if len(ps) else None
            slamyaw=float(abs(np.unwrap(py)-py[0]).max()) if len(ps) else None
            zero=all(mapped[i] is not None and mapped[i]['mode']=='parking' and joined[i] is not None
                and np.max(abs(requested[i]))<=1e-9 and np.max(abs(cmd[i]))<=1e-9 for i in np.flatnonzero(tm))
            sourcegoal=all(np.linalg.norm(p[:2]-route[-1,:2])<=.15 for p in pp)
            parking={'physical_window_s':[start,end],'trigger_executor_index':pi,
                'trigger_original_SLAM_pose_stamp_ns':joined[pi]['pose_stamp_ns'],'native_rows':int(nm.sum()),'telemetry_rows':int(tm.sum()),
                'source_original_headers':len(ps),'source_maximum_gap_s':float(np.max(np.diff(pt))) if len(pt)>1 else None,
                'source_window_complete':bool(source_complete),'native_window_complete':bool(native_complete),
                'continuous_source_goal_within_15cm':bool(sourcegoal),'continuous_requested_and_actor_command_zero':bool(zero),
                'actual_native_xy_drift_m':drift,'actual_native_yaw_drift_rad':yd,'actual_SLAM_xy_drift_m':slamdrift,'actual_SLAM_yaw_drift_rad':slamyaw}
        parkpassed=parking is not None and parking['source_window_complete'] and parking['native_window_complete'] and parking['continuous_source_goal_within_15cm'] and parking['continuous_requested_and_actor_command_zero'] and parking['actual_native_xy_drift_m']<=.05 and parking['actual_native_yaw_drift_rad']<=.1 and parking['actual_SLAM_xy_drift_m']<=.05 and parking['actual_SLAM_yaw_drift_rad']<=.1
        check('first_source_arrival_fixed_5s_parking',parkpassed,unverified=parking is None or not parking['source_window_complete'] or not parking['native_window_complete'],
            parking=parking,limits_xy_m=.05,yaw_rad=.1,truth_selected_parking_window=False)
        result['route_profile']=profile;result['source_binding']=binding
    except MissingEvidence as exc:
        check('required_original_evidence',False,unverified=True,reason=str(exc))
    except (ValueError,TypeError,KeyError,OSError,IndexError,AttributeError,ZeroDivisionError) as exc:
        check('evidence_integrity_or_contract',False,reason=f'{type(exc).__name__}: {exc}')
    statuses=[r['status'] for r in result['checks'].values()]
    result['status']='failed' if 'failed' in statuses else 'unverified' if not statuses or 'unverified' in statuses else 'passed'
    result['actual_SLAM_fixed_route_verified']=result['status']=='passed'
    result['levels']={'interface_sources':result['checks'].get('actual_Teacher_reads_original_bytes_and_causal_sensor_feedback',{}).get('status','unverified'),
        'motion_control':result['checks'].get('independent_200Hz_motion_safety',{}).get('status','unverified'),
        'actual_SLAM_fixed_route_transfer':result['status'],'SCAN_full_multifloor':'unverified','hardware':'unverified','general_Sim2Sim':'unverified'}
    if write:
        with output.open('x') as stream:json.dump(result,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')
    return result


def sha_bytes(raw):
    return hashlib.sha256(raw).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--receipt-suffix')
    parser.add_argument('--native-helper',type=Path,help='Optional archived exact frozen v2 evaluator; SHA is mandatory')
    parser.add_argument('--protocol',type=Path,help='Optional archived exact v2 protocol; SHA is mandatory')
    args=parser.parse_args()
    receipt=evaluate(args.run,suffix=args.receipt_suffix,helper_path=args.native_helper,protocol_path=args.protocol)
    print(json.dumps({'run':str(args.run.resolve()),'status':receipt['status'],
                      'actual_SLAM_fixed_route_verified':receipt['actual_SLAM_fixed_route_verified'],'score':None}))
    return 0 if receipt['status']=='passed' else 2


if __name__=='__main__':
    sys.dont_write_bytecode=True
    raise SystemExit(main())
