#!/usr/bin/env python3
"""Evaluate frozen per-run evidence; incomplete or assisted coverage cannot pass."""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
JOINT_ORDER = [f'{leg}_{part}_joint' for leg in ('rf','lf','rh','lh') for part in ('hip','upper_leg','lower_leg')]
CONTACT_ORDER = ('body','FR','FL','RR','RL')
HOLD_PHASES = {'support_hold','support_capture'}
_HASH_CACHE = {}


def digest(path):
    path = Path(path)
    stat = path.stat()
    key = (str(path.resolve()), stat.st_mtime_ns, stat.st_size)
    if key not in _HASH_CACHE:
        _HASH_CACHE[key] = hashlib.sha256(path.read_bytes()).hexdigest()
    return _HASH_CACHE[key]


def read_json(path, errors, required=False):
    try:
        data = json.loads(path.read_text())
        if not isinstance(data, dict):
            raise ValueError('expected object')
        return data
    except (OSError, ValueError) as error:
        if required or path.exists():
            errors.append(f'{path.name}: {error}')
        return {}


def read_jsonl(path, errors):
    rows = []
    try:
        with path.open() as stream:
            for number, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                    if not isinstance(row, dict):
                        raise ValueError('expected object')
                    rows.append(row)
                except ValueError as error:
                    errors.append(f'{path.name}:{number}: {error}')
    except OSError as error:
        errors.append(f'{path.name}: {error}')
    return rows


def arr(rows, key, shape):
    data = np.asarray([r.get(key) for r in rows], dtype=float)
    expected = (len(rows),) + tuple(shape)
    if data.shape != expected or not np.isfinite(data).all():
        raise ValueError(f'{key} must be finite with shape {expected}; got {data.shape}')
    return data


def check(passed, **evidence):
    return {'passed': bool(passed), **evidence}


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def transform(element):
    values = np.fromstring(element.findtext('pose', '0 0 0 0 0 0'), sep=' ')
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('Invalid SDF pose')
    return values[:3], rotation(values[3:])


def box_geometry(world, model_name):
    model = ET.parse(world).getroot().find(f"world/model[@name='{model_name}']")
    if model is None:
        raise ValueError(f'Missing physical {model_name} model')
    link = model.find('link')
    collision = link.find('collision') if link is not None else None
    if collision is None:
        raise ValueError(f'Missing {model_name} collision')
    pos, rot = transform(model)
    for child in (link, collision):
        cp, cr = transform(child)
        pos, rot = pos + rot @ cp, rot @ cr
    size = np.fromstring(collision.findtext('geometry/box/size', ''), sep=' ')
    if size.shape != (3,) or not np.isfinite(size).all() or (size <= 0).any():
        raise ValueError(f'{model_name} must have a finite box collision')
    normal = rot[:,2]
    if abs(normal[2]) < 1e-8:
        raise ValueError('Support surface cannot be vertical')
    top = pos + normal*size[2]/2
    return {'name':model_name, 'center':pos, 'rotation':rot, 'size':size, 'normal':normal, 'top':top}


def surface_height(box, xy):
    xy = np.atleast_2d(xy)
    n, top = box['normal'], box['top']
    return (np.dot(n, top) - xy @ n[:2]) / n[2]


def inside_tread(box, positions):
    ground = np.column_stack((positions[:,:2], surface_height(box, positions[:,:2])))
    local = (ground-box['center']) @ box['rotation']
    return (np.abs(local[:,:2]) <= box['size'][:2]/2 + 1e-8).all(axis=1)


def support_groups(step, model_name):
    groups = set()
    token = re.compile(r'(?:^|::|/)' + re.escape(model_name) + r'(?:$|::|/)')
    for pair in step.get('contact_pairs', []):
        group = pair.get('group')
        if group in (1,2,3,4) and (token.search(str(pair.get('a',''))) or token.search(str(pair.get('b','')))):
            groups.add(int(group))
    return groups


def stop_metrics(mask, t, pos, velocity, attitude):
    if not mask.any():
        return {'samples':0}
    p = pos[mask]; yaw = np.unwrap(attitude[mask,2])
    return {'samples':int(mask.sum()), 'window_s':[float(t[mask][0]),float(t[mask][-1])],
            'translation_drift_m':float(np.linalg.norm(p[-1,:2]-p[0,:2])),
            'yaw_drift_rad':float(abs(yaw[-1]-yaw[0])),
            'velocity_rms':np.sqrt(np.mean(velocity[mask]**2,axis=0)).tolist()}


def evaluate(run):
    run = Path(run)
    errors=[]; reasons=[]; metrics={}; iface={}
    protocol_path=run/'sources/tests/protocol.json'
    if not protocol_path.exists():protocol_path=ROOT/'tests/protocol.json'
    protocol=read_json(protocol_path,errors,True)
    meta=read_json(run/'policy_manifest.json',errors,True)
    result=read_json(run/'worker_result.json',errors,True)
    runtime=read_json(run/'runtime_manifest.json',errors,True)
    assets=read_json(run/'asset_manifest.json',errors,True)
    manifest=read_json(run/'source_manifest.json',errors,True)
    rows=read_jsonl(run/'telemetry.jsonl',errors)
    native=read_jsonl(run/'actuator.jsonl',errors)
    name=meta.get('test',run.name)
    protocol_frozen=(run/'sources/tests/protocol.json').exists() or manifest.get('tests/protocol.json')==digest(protocol_path)
    timing=protocol.get('timing',{}) if protocol_frozen else {}
    duration_key='switch_duration_s'if name=='switch'else'step_duration_s'if name.startswith('step')else'stand_duration_s'if name=='stand'else'single_duration_s'
    legacy_duration=26 if name=='switch' else 20 if name.startswith('step') else 15 if name=='stand' else 18
    expected=float(meta.get('test_duration_s',timing.get(duration_key,legacy_duration)))
    command_end=float(meta.get('command_end_s',timing.get('step_command_end_s'if name.startswith('step')else'single_command_end_s',11)))
    contracts=[r for r in native if r.get('kind')=='actuator_contract']
    owners=[r for r in native if r.get('kind')=='model_plugin_ownership']
    steps=[r for r in native if r.get('kind')=='physics_step']
    exchanges=[r for r in native if r.get('kind')=='policy_exchange']
    native_faults=[r for r in native if r.get('kind')=='fault']
    normal_end=bool(steps and steps[-1].get('terminating') is True and exchanges and exchanges[-1].get('terminate')==1)
    contract=contracts[0]if len(contracts)==1 else {}

    # Interface is a conjunction of actual observation, inference, transport,
    # native actuation and provenance checks. Motion may independently fail.
    iface['cpu_frozen_checkpoint']=check(meta.get('checkpoint_sha256')==CHECKPOINT_SHA and meta.get('inference_device')=='cpu',
        device=meta.get('inference_device'),declared_sha256=meta.get('checkpoint_sha256'))
    try:
        model_hash=digest(meta['checkpoint'])
        iface['cpu_frozen_checkpoint']['actual_sha256']=model_hash
        iface['cpu_frozen_checkpoint']['passed'] &= model_hash==CHECKPOINT_SHA
    except (KeyError,OSError) as e:iface['cpu_frozen_checkpoint'].update(passed=False,error=str(e))
    source_verified=[];source_missing=[];source_mismatch=[]
    for rel,expected_hash in manifest.items():
        candidate=run/'sources'/rel
        if not candidate.exists():candidate=ROOT/rel
        try:
            if digest(candidate)==expected_hash:source_verified.append(rel)
            else:source_mismatch.append(rel)
        except (OSError,TypeError):source_missing.append(rel)
    core={'policy/worker.py','policy/observation.py','policy/contract.json','simulation/teacher_actuator.cpp',
          'simulation/prepare.py','scripts/run_test.py','tests/protocol.json'}
    iface['source_hashes']=check(bool(manifest) and core.issubset(manifest) and not source_missing and not source_mismatch,
        verified_count=len(source_verified),missing=source_missing,mismatched=source_mismatch,
        core_missing=sorted(core-set(manifest)),protocol_source=str(protocol_path),snapshot_present=(run/'sources').is_dir())
    try:
        iface['world_hash']=check(digest(run/'world.sdf')==assets.get('world_sha256'),actual=digest(run/'world.sdf'),declared=assets.get('world_sha256'))
    except OSError as e:iface['world_hash']=check(False,error=str(e))
    binary=ROOT/'simulation/build/libteacher_actuator.so'
    try:iface['plugin_hash']=check(digest(binary)==runtime.get('plugin_sha256'),actual=digest(binary),declared=runtime.get('plugin_sha256'))
    except OSError as e:iface['plugin_hash']=check(False,error=str(e))
    iface['native_exclusive_owner']=check(len(owners)==1 and owners[0].get('passed')is True and owners[0].get('teacher_writers')==1,
        evidence=owners)
    expected_contract={'kp':25.,'kd':.5,'effort_limit':23.5,'saturation_effort':23.5,'velocity_limit':30.,'expected_dt':.005,'decimation':4}
    matching=bool(contract) and all(isinstance(contract.get(k),(int,float)) and math.isclose(contract[k],v,rel_tol=0,abs_tol=1e-10)for k,v in expected_contract.items())
    iface['native_training_contract']=check(matching and contract.get('joint_order')==JOINT_ORDER and contract.get('joint_position_reset_count')==12 and contract.get('body_pose_resets')==0,
        actual={k:contract.get(k)for k in expected_contract},joint_order=contract.get('joint_order'),
        joint_position_reset_count=contract.get('joint_position_reset_count'),body_pose_resets=contract.get('body_pose_resets'),
        torque_source=contract.get('torque_source'),state_source=contract.get('state_source'))
    telemetry_valid=False; arrays={}; obs=None; actions=None
    try:
        if not rows:raise ValueError('No physics observation')
        for key,shape in [('sim_time',()),('position',(3,)),('command',(3,)),('requested',(3,)),('measured',(3,)),('rpy',(3,)),
                          ('body_clearance',()),('q',(12,)),('qd',(12,)),('action',(12,)),('applied_torque',(12,)),('inference_ms',())]:
            arrays[key]=arr(rows,key,shape)
        contacts=np.asarray([[r['contacts'][leg]for leg in CONTACT_ORDER]for r in rows],dtype=float)
        if contacts.shape!=(len(rows),5)or not np.isfinite(contacts).all():raise ValueError('Invalid named contact evidence')
        arrays['contacts']=contacts;telemetry_valid=True
    except (KeyError,TypeError,ValueError)as e:iface['finite_telemetry']=check(False,error=str(e));reasons.append(f'Invalid telemetry: {e}')
    else:iface['finite_telemetry']=check(True,samples=len(rows))
    try:
        with np.load(run/'observations_actions.npz',allow_pickle=False)as data:
            obs=np.asarray(data['observations']);actions=np.asarray(data['actions'])
        valid=(obs.shape==(len(rows),247)and actions.shape==(len(rows),12)and np.isfinite(obs).all()and np.isfinite(actions).all())
        action_error=float(np.max(np.abs(actions-arrays['action'])))if valid and telemetry_valid else None
        iface['observation_action_dimensions']=check(valid and telemetry_valid and action_error<=1e-6,
            observation_shape=list(obs.shape),action_shape=list(actions.shape),telemetry_samples=len(rows),action_max_error=action_error)
    except (OSError,KeyError,TypeError,ValueError)as e:iface['observation_action_dimensions']=check(False,error=str(e))
    if telemetry_valid:
        t=arrays['sim_time'];gaps=np.diff(t)
        time_valid=len(t)>1 and (gaps>0).all()and abs(t[0])<=1e-8 and np.max(gaps)<=.030001
        period_valid=time_valid and np.max(abs(gaps-.02))<=1e-8
        iface['policy_50hz']=check(period_valid,maximum_gap_s=float(np.max(gaps))if len(gaps)else None,
            maximum_period_error_s=float(np.max(abs(gaps-.02)))if len(gaps)else None)
        if not time_valid:reasons.append('Policy time is nonincreasing, missing its origin, or has gaps above 30ms')
        latency=arrays['inference_ms']
        iface['cpu_inference_samples']=check(len(latency)>=2 and (latency>=0).all()and result.get('samples')==len(rows),
            samples=len(latency),worker_samples=result.get('samples'),p95_ms=float(np.percentile(latency,95)))
    else:iface['policy_50hz']=check(False);iface['cpu_inference_samples']=check(False)
    native_valid=False;native_arrays={};native_time=None;offset=None
    try:
        if not steps or not exchanges:raise ValueError('No native PD steps or policy exchange')
        for key,shape in [('t',()),('dt',()),('q',(12,)),('qd',(12,)),('qtarget',(12,)),('pd_raw',(12,)),('tau',(12,)),('tau_lower',(12,)),('tau_upper',(12,))]:
            native_arrays[key]=arr(steps,key,shape)
        native_time=native_arrays['t'];ngaps=np.diff(native_time)
        ntime_ok=len(steps)>1 and(ngaps>0).all()and np.max(abs(ngaps-.005))<=1e-9 and np.max(abs(native_arrays['dt']-.005))<=1e-9
        offset=float(exchanges[0]['t'])
        exchange_time=arr(exchanges,'t',())
        exchange_match=telemetry_valid and len(exchanges)==len(rows) and np.max(abs(exchange_time-offset-arrays['sim_time']))<=1e-8
        iface['native_timing_and_transport']=check(ntime_ok and exchange_match,physics_steps=len(steps),policy_exchanges=len(exchanges),
            first_native_policy_time_s=offset,maximum_physics_period_error_s=float(np.max(abs(ngaps-.005)))if len(ngaps)else None,
            telemetry_exchange_alignment=exchange_match)
        native_valid=True
    except (KeyError,TypeError,ValueError)as e:iface['native_timing_and_transport']=check(False,error=str(e));reasons.append(f'Invalid native evidence: {e}')
    if native_valid and matching:
        qd=np.clip(native_arrays['qd'],-60.,60.)
        expected_lower=np.maximum(23.5*(-1.-qd/30.),-23.5)
        expected_upper=np.minimum(23.5*(1.-qd/30.),23.5)
        normal=np.array([not r.get('fault')and not r.get('terminating')for r in steps])
        pd=25.*(native_arrays['qtarget']-native_arrays['q'])-.5*native_arrays['qd']
        curve_error=max(float(np.max(abs(native_arrays['tau_lower']-expected_lower))),float(np.max(abs(native_arrays['tau_upper']-expected_upper))))
        pd_error=float(np.max(abs(native_arrays['pd_raw'][normal]-pd[normal])))if normal.any()else None
        applied=np.minimum(expected_upper,np.maximum(expected_lower,native_arrays['pd_raw']))
        torque_error=float(np.max(abs(applied-native_arrays['tau'])))
        iface['actual_pd_dcmotor_curve']=check(normal.any()and curve_error<=1e-8 and pd_error<=1e-8 and torque_error<=1e-8,
            envelope_error_Nm=curve_error,pd_error_Nm=pd_error,clipped_command_error_Nm=torque_error,
            torque_is='Native physics joint-force input; not independent hardware torque measurement')
    else:iface['actual_pd_dcmotor_curve']=check(False)
    # Validate that actual native targets carry actor outputs after startup.
    target_error=None;active_policy_samples=0
    if telemetry_valid and native_valid:
        default=np.array(contract.get('initial_q',[]))
        if default.shape==(12,):
            diffs=[];lookup={round(float(r['t']),8):r for r in steps}
            for row in rows:
                if row.get('state')=='initializing' or row.get('state')in HOLD_PHASES or row.get('fault'):continue
                step=lookup.get(round(row['sim_time']+offset,8))
                if not step or step.get('terminating')or step.get('fault'):continue
                target=default+.25*np.asarray(row['action'])
                diffs.append(float(np.max(abs(np.asarray(step['qtarget'])-target))))
            active_policy_samples=len(diffs);target_error=max(diffs)if diffs else None
    iface['actual_teacher_target_execution']=check(active_policy_samples>=2 and target_error is not None and target_error<=1e-6,
        samples=active_policy_samples,target_max_error_rad=target_error,excluded_assisted_phases=sorted(HOLD_PHASES))
    end_alignment=bool(normal_end and telemetry_valid and offset is not None and abs(steps[-1]['t']-offset-arrays['sim_time'][-1])<=1e-8)
    owned=runtime.get('owned_processes',[])
    main_exits=bool(len(owned)>=2 and all(p.get('returncode')==0 for p in owned[:2]))
    iface['actual_run_end']=check(end_alignment and main_exits and result.get('completed')is True and not runtime.get('error'),
        native_terminate=normal_end,terminal_time_alignment=end_alignment,main_process_returncodes=[p.get('returncode')for p in owned[:2]],runner_error=runtime.get('error'))
    iface['observation_provenance']=check(meta.get('navigation_truth_used')is False and 'Gazebo privileged' in meta.get('observation_source','') and 'COM' in contract.get('state_source',''),
        policy_source=meta.get('observation_source'),native_source=contract.get('state_source'),navigation_truth_used=meta.get('navigation_truth_used'))
    iface['evidence_readable']=check(not errors,errors=errors)
    interface=all(c['passed']for c in iface.values())
    provenance_checks={'source_hashes','plugin_hash'}
    interface_failures=[k for k,v in iface.items() if not v['passed']]
    interface_status='passed' if interface else 'unverified' if set(interface_failures).issubset(provenance_checks) else 'failed'

    if not rows:reasons.append('No completed physics observation')
    if errors:reasons.append('Evidence missing or malformed: '+'; '.join(errors))
    if runtime.get('error'):reasons.append('Runner error: '+str(runtime['error']))
    if result.get('fault'):reasons.append(str(result['fault']))
    if native_faults:reasons.append('Native actuator fault: '+', '.join(str(r.get('reason',r.get('code')))for r in native_faults))
    if any(r.get('fault',0) for r in steps):reasons.append('Native physics steps contain a latched fault')
    if not normal_end:reasons.append('Native termination/damping acknowledgement missing')
    if not main_exits:reasons.append('Policy or Gazebo did not exit normally')
    operational_failures=[k for k in interface_failures if k not in provenance_checks]
    if operational_failures:reasons.append('Operational interface checks failed: '+', '.join(operational_failures))
    if telemetry_valid and protocol:
        t,pos,cmd,v,rpy,contact,clear=(arrays[k]for k in('sim_time','position','command','measured','rpy','contacts','body_clearance'))
        active=t>=3
        phases=np.array([r.get('state','')for r in rows])
        hold=np.array([p in HOLD_PHASES for p in phases])
        metrics.update(duration_s=float(t[-1]),samples=len(rows),
            max_abs_roll_pitch_rad=float(abs(rpy[active,:2]).max())if active.any()else None,
            min_body_clearance_m=float(clear[active].min())if active.any()else None,
            body_contact_samples=int((contact[active,0]>0).sum()),unknown_contact_samples=int((contact[active]<0).sum()),
            max_applied_torque_Nm=float(abs(arrays['applied_torque']).max()),
            position_delta_m=(pos[-1]-pos[0]).tolist(),total_yaw_delta_rad=float(np.unwrap(rpy[:,2])[-1]-rpy[0,2]),
            assisted_phase_counts={p:int((phases==p).sum())for p in sorted(HOLD_PHASES)})
        if t[-1]<expected-.08:reasons.append('Test ended before required duration')
        if active.any():
            lim=protocol['motion_limits']
            if abs(rpy[active,:2]).max()>lim['max_abs_roll_pitch_rad']:reasons.append('Roll/pitch exceeds criterion')
            if clear[active].min()<lim['min_body_clearance_m']:reasons.append('Body clearance below criterion')
            if(contact[active,0]>0).any():reasons.append('Body ground/object contact')
            if(contact[active]<0).any():reasons.append('Contact evidence missing')
        else:reasons.append('No active motion sample')
        windows=[]if name=='stand'else[(5,10)]if name!='switch'else[(5,7.8),(10,12.8),(15,17.8)]
        track=[]
        for start,end in windows:
            mask=(t>=start)&(t<end)
            if mask.sum()<50:reasons.append(f'Incomplete tracking window {start}-{end}s');continue
            err=np.sqrt(np.mean((v[mask]-cmd[mask])**2,axis=0));mean=v[mask].mean(axis=0);ref=cmd[mask].mean(axis=0)
            track.append({'window_s':[start,end],'command_mean':ref.tolist(),'measured_mean':mean.tolist(),'rmse':err.tolist()})
            axis=int(np.argmax(abs(ref)))
            for k in range(3):
                thresh=protocol['tracking']['yaw_rate_rmse_radps']if k==2 else protocol['tracking']['linear_rmse_mps']
                if abs(ref[k])<.01:thresh=protocol['tracking']['cross_yaw_rms_radps']if k==2 else protocol['tracking']['cross_linear_rms_mps']
                if err[k]>thresh:reasons.append(f'Axis {k} tracking/drift exceeds {thresh}')
            if ref[axis]!=0 and mean[axis]*np.sign(ref[axis])<abs(ref[axis])*protocol['tracking']['minimum_requested_axis_ratio']:reasons.append('Requested direction or speed ratio failed')
        metrics['tracking']=track
        if name=='stand':stop=(t>=5)&(t<=14)
        elif name=='switch':stop=(t>=22)&(t<=26)
        elif name=='command_timeout':stop=(t>=11)&(t<=17.8)
        elif name.startswith('step'):stop=(t>=float(timing.get('step_stop_evaluation_start_s',18 if expected>=25 else 15)))&(t<=expected)
        else:stop=(t>=15)&(t<=expected)
        teacher_stop=stop &~hold;assisted_stop=stop &hold
        metrics['stop_teacher']=stop_metrics(teacher_stop,t,pos,v,rpy)
        metrics['stop_assisted']=stop_metrics(assisted_stop,t,pos,v,rpy)
        metrics['stop']=metrics['stop_teacher']
        if hold[stop].any()or(name=='stand'and hold.any()):
            reasons.append('Invalid stand/stop coverage: support capture/hold replaced continuous Teacher feedback')
        if teacher_stop.sum()>=50:
            sm=metrics['stop_teacher'];lim=protocol['stand_stop']
            if sm['translation_drift_m']>lim['translation_drift_m']:reasons.append('Teacher stand/stop translation drift')
            if sm['yaw_drift_rad']>lim['yaw_drift_rad']:reasons.append('Teacher stand/stop yaw drift')
            if max(sm['velocity_rms'][:2])>lim['linear_rms_mps']or sm['velocity_rms'][2]>lim['yaw_rate_rms_radps']:reasons.append('Teacher stand/stop residual velocity')
            if np.max(abs(cmd[teacher_stop]))>1e-6:reasons.append('Stand/stop window contains nonzero velocity command')
        else:reasons.append('Incomplete continuous Teacher stand/stop window')
        if name=='command_timeout':
            expired=np.array([r.get('command_expired')is True for r in rows]);ages=np.array([r.get('command_age_sim_s',np.nan)for r in rows],dtype=float)
            ttl_ok=np.isfinite(ages).all()and expired.any()and(t[expired][0]>=7.28-1e-8)and(t[expired][0]<=7.32+1e-8)and(ages[expired]>.3).all()and np.max(abs(arrays['requested'][expired]))<1e-8
            metrics['command_timeout']={'source':'simulated command producer inside worker; ROS command receiver remains unverified',
                'expired_samples':int(expired.sum()),'first_expired_sim_s':float(t[expired][0])if expired.any()else None,'passed':bool(ttl_ok)}
            if not ttl_ok:reasons.append('Command TTL expiry evidence missing or incorrect')
        if name in('ramp_up','ramp_down')or name.startswith('step'):
            try:
                box=box_geometry(run/'world.sdf','ramp_12'if name.startswith('ramp')else'teacher_low_step')
                start=(t>=2)&(t<3);end=t>=max(3,t[-1]-1.)
                if start.sum()<25 or end.sum()<25:raise ValueError('Missing settled terrain start/end window')
                p0=np.median(pos[start],axis=0);p1=np.median(pos[end],axis=0)
                nrel=native_time-offset if native_valid else np.array([])
                native_end=[s for s,nt in zip(steps,nrel)if nt>=max(3,t[-1]-1.)]
                supports=[support_groups(s,box['name'])for s in native_end]
                foot_union=set().union(*supports)if supports else set()
                terrain={'description':'10% physical ramp, not stairs'if name.startswith('ramp')else'Low-step ascent onto box tread; not complete obstacle traversal',
                    'start_median_position_m':p0.tolist(),'end_median_position_m':p1.tolist(),'height_gain_m':float(p1[2]-p0[2]),
                    'native_end_support_foot_groups':sorted(foot_union),'native_end_all_four_support_samples':sum(len(g)==4 for g in supports),
                    'native_end_samples':len(native_end),'end_inside_tread':bool(inside_tread(box,pos[end]).all())}
                metrics['terrain']=terrain
                if foot_union!={1,2,3,4}:reasons.append('Missing measured terrain support for one or more feet at the end')
                if not terrain['end_inside_tread']:reasons.append('Final body position is outside the physical terrain tread')
                if name.startswith('ramp'):
                    direction=1 if name=='ramp_up'else-1;progress=direction*(p1[0]-p0[0]);rise=surface_height(box,p1[None,:2])[0]-surface_height(box,p0[None,:2])[0]
                    terrain.update(progress_m=float(progress),expected_surface_height_change_m=float(rise),height_change_error_m=float(abs(p1[2]-p0[2]-rise)))
                    minimum=protocol['terrain']['up_ramp_min_x_progress_m'if name=='ramp_up'else'down_ramp_min_progress_m']
                    if progress<minimum:reasons.append('Insufficient measured ramp progress')
                    on_ramp=(t>=3)&(t<=min(command_end,t[-1]))
                    if not inside_tread(box,pos[on_ramp]).all():reasons.append('Active ramp trace leaves its physical x/y support footprint')
                    native_active=[s for s,nt in zip(steps,nrel)if 3<=nt<=min(command_end,t[-1])]
                    all_supported=set().union(*(support_groups(s,'ramp_12')for s in native_active))if native_active else set()
                    terrain['active_support_foot_groups']=sorted(all_supported)
                    if all_supported!={1,2,3,4}:reasons.append('Actual foot contacts do not confirm ramp support during motion')
                    if direction*(p1[2]-p0[2])<=0 or terrain['height_change_error_m']>protocol['stand_stop']['translation_drift_m']:
                        reasons.append('Measured height change does not follow the physical ramp surface')
                else:
                    height=.05 if name=='step05'else.1
                    minimum_gain=height*protocol['terrain']['step_body_height_gain_fraction']
                    terrain.update(step_height_m=height,minimum_height_gain_m=minimum_gain)
                    if p1[0]<protocol['terrain']['step_end_x_m']or p1[2]-p0[2]<minimum_gain:
                        reasons.append('Low step tread was not reached with settled measured height gain')
                        if command_end<=11 and p1[0]<protocol['terrain']['step_end_x_m']:
                            reasons.append('Historical command duration was insufficient to complete the unchanged low-step endpoint')
                    if not any(len(g)==4 for g in supports):reasons.append('No measured final four-foot support on the low-step tread')
            except (OSError,ET.ParseError,TypeError,ValueError)as e:reasons.append('Terrain evidence invalid: '+str(e))
    motion_pass=not reasons
    summary={'test':name,'levels':{'interface':interface_status,'motion':'passed'if motion_pass else'failed',
        'sim2sim':'unverified','navigation':'unverified','real_robot':'unverified'},'iface_checks':iface,
        'tests':[{'name':name,'status':'passed'if motion_pass else'failed','reason':'; '.join(dict.fromkeys(reasons)),'metrics':metrics}],
        'protocol_sha256':digest(protocol_path)if protocol_path.exists()else None,'protocol_source':str(protocol_path),
        'evaluator_sha256':digest(Path(__file__)),
        'scope':'Actual Gazebo physics with privileged policy observations; no navigation localization claim. Low-step ascent and ramps do not demonstrate stairs. Three independent repetitions remain required for full motion acceptance.'}
    summary=clean(summary)
    if name=='navigation':
        summary['generic_motion_diagnostic_status']=summary['tests'][0]['status']
        summary['requires_independent_navigation_evaluation']=True
        summary['tests'][0]['name']='Generic motion/safety diagnostic during navigation experiment'
        if motion_pass:
            summary['tests'][0]['status']='unverified';summary['levels']['motion']='unverified'
            summary['tests'][0]['reason']='Generic safety diagnostic passed; SLAM/SCAN movement, trajectories and arrival require independent route evidence'
    (run/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args()
    print(json.dumps(evaluate(args.run),ensure_ascii=False,allow_nan=False))
