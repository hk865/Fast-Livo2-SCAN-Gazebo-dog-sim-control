#!/usr/bin/env python3
"""Append-only complete physical ramp supplement for actual SLAM/SCAN navigation.

No ROS, simulator, policy inference, command publisher or process signal. Actual
Gazebo pose/contact data is used ONLY after execution, never to construct goals.
The original common nonflat UNVERIFIED receipt remains unchanged. Formal PASS
requires this exact source/contract archived before the first actual file read.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
SCHEMA = 'independent_actual_SLAM_SCAN_complete_ramp_navigation/v1'
CONTRACT_SCHEMA = 'prospective_actual_SLAM_SCAN_complete_ramp_contract/v1'
COMMON_SHA = '074b468581de568264f558e38f82ce25e809094843c6f86e2816130babf8099e'
APPLICABILITY = 'strict_nonflat_complete_contact_geometry'
REQUIRED_COMMON_CHECKS = {
    'execution_phase_status_and_cleanup_boundary', 'frozen_scope_and_archived_sources',
    'actual_causal_SLAM_IMU_cascade_updates', 'actual_checked_SCAN_trajectory_payloads',
    'fixed_goal_and_original_SCAN_bounded_projection', 'actual_executor_ack_and_cascade_PI_COM_PD_replay',
    'actual_raw_cloud_bytes_fields_filtering', 'actual_cascade_movement_guard_raw_geometry',
    'actual_command_original_read_dual_TTL_slew', 'all_original_SLAM_3D_region_arrivals',
    'runtime_CPU_single_writer_complete_and_drained', 'native_continuous_200Hz',
    'native_physical_safety', 'native_force_velocity_and_support', 'exclusive_Teacher_CPU_identity',
    'actual_sensor_graph_actor247_CPU_joint_execution', 'independent_offline_actual_route_speed_heading',
    'new_first_declared_active_hold_fixed_5s',
}
LEGS = {'FR': 'rf', 'FL': 'lf', 'RR': 'rh', 'RL': 'lh'}
CRITERIA = {
    'native_physics_step_s': .005, 'maximum_native_gap_s': .005001,
    'nominal_horizontal_ramp_span_m': 12.,
    'minimum_COM_distance_outside_each_seam_m': .25,
    'contact_local_xy_margin_m': .005, 'top_contact_normal_distance_m': .015,
    'minimum_abs_contact_normal_dot': .8,
    'foot_sphere_center_height_tolerance_m': .025,
    'each_foot_entry_ramp_exit_continuous_support_s': .1,
    'exit_landing_any_foot_continuous_support_s': .6,
    'exit_landing_simultaneous_four_foot_support_s': .1,
    'ramp_coverage_bin_m': 1., 'minimum_bin_actual_support_s': .1,
    'maximum_all_feet_unsupported_s': .3,
    'maximum_COM_lateral_ramp_axis_error_m': .45,
    'minimum_origin_clearance_m': .18, 'maximum_roll_pitch_rad': .65,
    'maximum_pose_origin_velocity_integral_error_m': .0001,
    'region_dwell_s': .6, 'region_header_max_gap_s': .2,
    'region_control_radius_m': .17, 'region_control_height_half_span_m': .1,
    'goal_timeout_sim_s': 90.,
    'source_ttl_sim_and_wall_s': .3, 'gyro_pose_causal_max_gap_s': .02,
    'parking_first_window_s': 5., 'parking_xy_drift_m': .05,
    'parking_yaw_drift_rad': .1, 'parking_origin_speed_peak_mps': .08,
    'parking_Euler_yawdot_and_body_wz_peak_radps': .1,
    'body_world_height_gain_ratio_is_acceptance_gate': False,
}
FIXTURES = {
    'ramp_12': {'center_y_m': 2., 'slope_z_per_world_x': .1,
                'low_x_landing': 'floor_1', 'high_x_landing': 'floor_2',
                'landing_top_z_m': [0., 1.2], 'world_x_seams_m': [2., 14.]},
    'ramp_23': {'center_y_m': 7., 'slope_z_per_world_x': -.1,
                'low_x_landing': 'floor_3', 'high_x_landing': 'floor_2',
                'landing_top_z_m': [2.4, 1.2], 'world_x_seams_m': [2., 14.]},
}


class MissingEvidence(Exception):
    pass


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''): h.update(block)
    return h.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def clean(value):
    if isinstance(value, np.ndarray): return clean(value.tolist())
    if isinstance(value, np.generic): return clean(value.item())
    if isinstance(value, dict): return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return None
    return value


def read(path):
    if not Path(path).is_file(): raise MissingEvidence('Missing ' + str(path))
    return json.loads(Path(path).read_text(), parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))


def lines(path):
    if not Path(path).is_file(): raise MissingEvidence('Missing ' + str(path))
    with Path(path).open() as stream:
        return [json.loads(s) for s in stream if s.strip()]


def check(value, **detail):
    return {'status': 'unverified' if value is None else 'passed' if value else 'failed',
            'passed': None if value is None else bool(value), **clean(detail)}


def overall(checks):
    status = [v['status'] for v in checks.values()]
    return 'failed' if 'failed' in status else 'unverified' if not status or 'unverified' in status else 'passed'


def array(rows, key, width=None):
    if any(key not in r for r in rows): raise MissingEvidence('Missing original native ' + key)
    a = np.asarray([r[key] for r in rows], float)
    if a.shape != ((len(rows),) if width is None else (len(rows), width)) or not np.isfinite(a).all():
        raise ValueError('Invalid finite native ' + key)
    return a


def module(path):
    name = '_closed_loop_ramp_common_' + sha(path)[:12]
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def planned_segments(text):
    """ramp_12:up:0,ramp_23:up:2; final component is original goal index."""
    out = []
    for value in text.split(','):
        fields = value.strip().split(':')
        if len(fields) != 3 or fields[0] not in FIXTURES or fields[1] not in ('up', 'down'):
            raise ValueError('Need explicit ramp_12|ramp_23:up|down:destination_goal_index')
        name, direction, index = fields
        if not index.isdecimal(): raise ValueError('Nonnegative original destination goal index required')
        f = FIXTURES[name]
        sign = (1 if f['slope_z_per_world_x'] > 0 else -1) * (1 if direction == 'up' else -1)
        start = f['low_x_landing'] if sign > 0 else f['high_x_landing']
        end = f['high_x_landing'] if sign > 0 else f['low_x_landing']
        out.append({'ramp_model': name, 'direction': direction, 'world_x_forward_sign': sign,
                    'entry_landing_model': start, 'exit_landing_model': end,
                    'destination_goal_index': int(index)})
    if not out: raise ValueError('Empty prospective ramp sequence')
    if any(b['destination_goal_index'] <= a['destination_goal_index'] for a, b in zip(out, out[1:])):
        raise ValueError('Successive ramps require increasing original destination goal indices')
    if any(a['exit_landing_model'] != b['entry_landing_model'] for a, b in zip(out, out[1:])):
        raise ValueError('Disconnected prospective physical landing sequence')
    return out


def prepare_contract(run, text):
    """Root-only explicit prelaunch preparation; never changes runtime or old evidence."""
    run = Path(run).resolve()
    outputs = [run/'ramp_acceptance_contract.json', run/'ramp_acceptance_source_manifest.json',
               run/'sources/acceptance/evaluate_ramp.py', run/'sources/acceptance/evaluate_closed_loop.py']
    if any(p.exists() for p in outputs): raise ValueError('Refusing to replace prospective acceptance evidence')
    if (run/'telemetry.jsonl').exists() or (run/'actuator.jsonl').exists():
        raise ValueError('Prospective prepare must precede all actual execution logging')
    for name in ('world.sdf', 'navigation_scope.json'):
        if not (run/name).is_file(): raise MissingEvidence('Prepare after immutable world/scope, before launching: ' + name)
    common = HERE/'evaluate_closed_loop.py'
    if sha(common) != COMMON_SHA: raise ValueError('Original common evaluator identity changed')
    segments = planned_segments(text)
    contract = {'schema': CONTRACT_SCHEMA, 'run': str(run), 'criteria': CRITERIA, 'fixtures': FIXTURES,
                'segments': segments, 'prepared_monotonic_wall': time.monotonic(),
                'evaluator_sha256': sha(__file__), 'common_evaluator_sha256': COMMON_SHA,
                'world_sha256': sha(run/'world.sdf'), 'navigation_scope_sha256': sha(run/'navigation_scope.json'),
                'navigation_ground_truth_used': False, 'Gazebo_geometry_used_only_offline': True,
                'scope': 'Complete explicitly selected ramp(s), actual SLAM/IMU/SCAN control, active endpoint hold; no stairs/hardware/general mission claim'}
    (run/'sources/acceptance').mkdir(parents=True, exist_ok=True)
    for source, dest in ((Path(__file__), outputs[2]), (common, outputs[3])):
        with dest.open('xb') as stream: stream.write(source.read_bytes())
    with outputs[0].open('x') as stream: stream.write(json.dumps(contract, indent=2, allow_nan=False)+'\n')
    manifest = {str(p.relative_to(run)): sha(p) for p in (outputs[0], outputs[2], outputs[3])}
    with outputs[1].open('x') as stream: stream.write(json.dumps(manifest, indent=2)+'\n')
    return {'contract': str(outputs[0]), 'contract_sha256': sha(outputs[0]), 'archived_sources': manifest,
            'formal_evaluation_requires_unchanged_original_common_receipt': True}


def ancestry(run, common_path):
    receipt = read(common_path)
    if receipt.get('schema') != 'independent_actual_SLAM_SCAN_cascade_navigation/v1':
        raise ValueError('Different original common receipt schema')
    if Path(receipt['run']).resolve() != run or receipt.get('evaluator_sha256') != COMMON_SHA:
        raise ValueError('Original common receipt belongs to a different run/source')
    hashes = receipt.get('verified_input_source_sha256')
    if not hashes: raise MissingEvidence('Original common input hash chain absent')
    changed = []
    for name, digest in hashes.items():
        p = Path(name).resolve()
        if run not in p.parents: raise ValueError('Common evidence path outside this run')
        if not p.is_file(): raise MissingEvidence('Missing ancestor input ' + name)
        if sha(p) != digest: changed.append(name)
    if changed: raise ValueError('Original common source inputs changed: ' + repr(changed))
    checks = receipt.get('checks', {})
    if not checks: raise MissingEvidence('Original common checks absent')
    shared = {k: v for k, v in checks.items() if k != APPLICABILITY}
    missing = REQUIRED_COMMON_CHECKS-set(shared)
    if missing: raise MissingEvidence('Original common required checks absent: '+repr(sorted(missing)))
    if APPLICABILITY in checks and checks[APPLICABILITY]['status'] != 'unverified':
        raise ValueError('Original nonflat applicability receipt has unexpected status')
    return receipt, check(None if any(v['status']=='unverified' for v in shared.values()) and not any(v['status']=='failed' for v in shared.values())
                          else all(v['status']=='passed' for v in shared.values()),
                          failed_checks=[k for k,v in shared.items() if v['status']=='failed'],
                          unverified_checks=[k for k,v in shared.items() if v['status']=='unverified'],
                          original_status=receipt.get('status'),
                          excluded_only_explicit_nonflat_applicability=APPLICABILITY,
                          all_original_real_safety_source_motion_arrival_parking_checks_preserved=True)


def contract_audit(run, execution):
    path = run/'ramp_acceptance_contract.json'
    contract = read(path)
    if contract.get('schema') != CONTRACT_SCHEMA or Path(contract['run']).resolve() != run:
        raise ValueError('Prospective ramp contract identity differs')
    if contract.get('criteria') != CRITERIA or contract.get('fixtures') != FIXTURES:
        raise ValueError('Prospective numeric/geometry acceptance changed')
    if contract.get('navigation_ground_truth_used') is not False or contract.get('Gazebo_geometry_used_only_offline') is not True:
        raise ValueError('Prospective source boundary differs')
    if contract['evaluator_sha256'] != sha(__file__) or contract['common_evaluator_sha256'] != COMMON_SHA:
        raise ValueError('Exact prospective evaluator source differs')
    if contract['world_sha256'] != sha(run/'world.sdf') or contract['navigation_scope_sha256'] != sha(run/'navigation_scope.json'):
        raise ValueError('Frozen physical scene/scope changed after protocol')
    manifest = read(run/'ramp_acceptance_source_manifest.json')
    for name in ('ramp_acceptance_contract.json', 'sources/acceptance/evaluate_ramp.py', 'sources/acceptance/evaluate_closed_loop.py'):
        if name not in manifest: raise MissingEvidence('Required immutable acceptance archive not declared')
        if sha(run/name) != manifest[name]: raise ValueError('Acceptance archive SHA differs: '+name)
    if sha(run/'sources/acceptance/evaluate_ramp.py') != sha(__file__) or sha(run/'sources/acceptance/evaluate_closed_loop.py') != COMMON_SHA:
        raise ValueError('Wrong executed independent helper archive')
    reads = [r.get('closed_loop_read_evidence',{}).get('actual_read_attempt',{}).get('read_monotonic_wall') for r in execution]
    reads = [float(v) for v in reads if isinstance(v, (float,int)) and math.isfinite(v)]
    if not reads: raise MissingEvidence('Original actual consumer receipt clock absent')
    before = type(contract.get('prepared_monotonic_wall')) in (float,int) and 0 < contract['prepared_monotonic_wall'] < min(reads)
    if not before: raise ValueError('Contract was not prepared before actual execution reads')
    # Revalidate planned semantics; neither names nor order may be silently inferred.
    text = ','.join(f"{v['ramp_model']}:{v['direction']}:{v['destination_goal_index']}" for v in contract['segments'])
    if planned_segments(text) != contract['segments']: raise ValueError('Invalid prospective ramp direction/goal identity')
    return contract, check(True,original_contract_sha256=sha(path),prepared_monotonic_wall=contract['prepared_monotonic_wall'],
                           first_original_consumer_read_wall=min(reads),prospective_segments=contract['segments'])


def pose(element):
    if element is None: raise MissingEvidence('Missing SDF geometric element')
    e = element.find('pose')
    vals = np.fromstring(e.text or '', sep=' ') if e is not None else np.zeros(6)
    if vals.shape != (6,) or not np.isfinite(vals).all(): raise ValueError('Invalid actual SDF pose')
    r,p,y=vals[3:]; cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    return vals[:3], np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                              [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def surface(world, name):
    model=world.find(f"world/model[@name='{name}']")
    if model is None or model.findtext('static','').lower() not in ('true','1'): raise MissingEvidence('Missing physical static surface '+name)
    links=[link for link in model.findall('link') if link.findall('collision')]
    if len(links)!=1 or len(links[0].findall('collision'))!=1: raise ValueError('Ambiguous physical terrain slab')
    link=links[0]; col=link.find('collision')
    for e in (model,link,col):
        if e.find('pose') is not None and e.find('pose').get('relative_to'): raise MissingEvidence('Unsupported terrain relative frame')
    mp,mr=pose(model); lp,lr=pose(link); cp,cr=pose(col)
    size=np.fromstring(col.findtext('geometry/box/size',''),sep=' ')
    if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0): raise MissingEvidence('Original terrain is not an auditable box')
    return {'name':name,'collision':f'{name}::{link.get("name")}::{col.get("name")}',
            'center':mp+mr@lp+mr@lr@cp,'rotation':mr@lr@cr,'half':size/2}


def axis_rotations(axis, angles):
    if axis.shape!=(3,) or not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-8: raise ValueError('Invalid physical joint axis')
    x,y,z=axis/np.linalg.norm(axis); k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    c,s=np.cos(angles)[:,None,None],np.sin(angles)[:,None,None]
    return c*np.eye(3)+(1-c)*np.outer([x,y,z],[x,y,z])+s*k


def foot_centers(world, native, order, rotation):
    """Measured-q FK is a geometric cross-check; raw contacts prove support."""
    model=world.find("world/model[@name='go2']")
    if model is None: raise MissingEvidence('Original physical go2 model absent')
    q=array(native,'q',12); base=array(native,'position',3); centers={};feet={}
    for leg,short in LEGS.items():
        p=base.copy();r=rotation.copy();parent='base_link';chain=[]
        for part in ('hip','upper_leg','lower_leg'):
            name=f'{short}_{part}_joint';joint=model.find(f"joint[@name='{name}']")
            if joint is None or joint.find('pose') is None: raise MissingEvidence('Missing actual joint '+name)
            child=joint.findtext('child')
            if joint.findtext('parent')!=parent or joint.find('pose').get('relative_to')!=parent: raise MissingEvidence('Unsupported joint frame')
            xyz,rr=pose(joint);p+=np.einsum('nij,j->ni',r,xyz);r=r@rr
            axis=joint.find('axis/xyz')
            if axis is None or axis.get('expressed_in'): raise MissingEvidence('Unsupported joint axis frame')
            r=r@axis_rotations(np.fromstring(axis.text,sep=' '),q[:,order.index(name)])
            link=model.find(f"link[@name='{child}']");lp=link.find('pose') if link is not None else None
            if lp is not None and lp.get('relative_to')!=name: raise MissingEvidence('Unsupported child-link frame')
            xyz,rr=pose(link);p+=np.einsum('nij,j->ni',r,xyz);r=r@rr;parent=child;chain.append(name)
        cols=[c for c in link.findall('collision') if f'{short}_foot_link_collision' in c.get('name','')]
        if len(cols)!=1: raise MissingEvidence('Ambiguous actual spherical foot collision')
        col=cols[0];radius=float(col.findtext('geometry/sphere/radius'))
        if not math.isfinite(radius) or radius<=0: raise ValueError('Invalid actual foot radius')
        if col.find('pose') is not None and col.find('pose').get('relative_to'): raise MissingEvidence('Nonlocal actual foot collision frame')
        xyz,_=pose(col);centers[leg]=p+np.einsum('nij,j->ni',r,xyz)
        feet[leg]={'collision':f'go2::{parent}::{col.get("name")}','radius_m':radius,'chain':chain}
    return centers,feet


def support_masks(native, centers, feet, surfaces):
    if any(type(row.get('contact_pairs')) is not list for row in native): raise MissingEvidence('Original actual contact_pairs absent')
    masks={name:{leg:np.zeros(len(native),bool) for leg in LEGS} for name in surfaces}
    malformed=[];relevant=0;qualified={name:0 for name in surfaces}
    # Group is only a consistency check: exact two collision names are authoritative.
    foot_by_collision={v['collision']:leg for leg,v in feet.items()}
    surface_by_collision={v['collision']:name for name,v in surfaces.items()}
    local={name:{leg:(centers[leg]-s['center'])@s['rotation'] for leg in LEGS} for name,s in surfaces.items()}
    for i,row in enumerate(native):
        for pair in row['contact_pairs']:
            names=[pair.get('a'),pair.get('b')]
            legs=[foot_by_collision[x] for x in names if x in foot_by_collision]
            slabs=[surface_by_collision[x] for x in names if x in surface_by_collision]
            if len(legs)!=1 or len(slabs)!=1: continue
            leg,name=legs[0],slabs[0];relevant+=1;s=surfaces[name];count=pair.get('points')
            xyz=np.asarray(pair.get('positions_world',[]),float);nn=np.asarray(pair.get('normals_world',[]),float)
            if type(count) is not int or count<1 or xyz.shape!=(count,3) or nn.shape!=(count,3) or not np.isfinite(xyz).all() or not np.isfinite(nn).all() or np.any(np.linalg.norm(nn,axis=1)<1e-8) or pair.get('group')!=list(LEGS).index(leg)+1:
                if len(malformed)<30: malformed.append({'native_index':i,'foot':leg,'surface':name})
                continue
            point=(xyz-s['center'])@s['rotation'];normal=nn/np.linalg.norm(nn,axis=1)[:,None]
            top=(abs(point[:,:2])<=s['half'][:2]-CRITERIA['contact_local_xy_margin_m']).all(1)&(abs(point[:,2]-s['half'][2])<=CRITERIA['top_contact_normal_distance_m'])&(abs(normal@s['rotation'][:,2])>=CRITERIA['minimum_abs_contact_normal_dot'])
            c=local[name][leg][i];radius=feet[leg]['radius_m']
            center_ok=(abs(c[:2])<=s['half'][:2]+radius).all() and abs(c[2]-s['half'][2]-radius)<=CRITERIA['foot_sphere_center_height_tolerance_m']
            if top.any() and center_ok: masks[name][leg][i]=True;qualified[name]+=int(top.sum())
    return masks,{'malformed_relevant_contact_examples':malformed,'relevant_exact_foot_surface_pairs':relevant,
                  'qualified_original_top_contact_points':qualified,
                  'support_source':'Original native exact named foot/slab collision pair, world XYZ and normalized normal; measured-q FK confirms sphere location',
                  'support_load_or_hardware_motor_torque_measured':False}


def span(mask,t):
    best=0.;start=None;interval=None
    for i,valid in enumerate(mask):
        if not valid:start=None;continue
        if start is None or (i and t[i]-t[i-1]>CRITERIA['maximum_native_gap_s']):start=float(t[i])
        duration=float(t[i]-start)
        if duration>best:best=duration;interval=[start,float(t[i])]
    return best,interval


def first_confirmation(mask,t,seconds):
    start=None
    for i,valid in enumerate(mask):
        if not valid:start=None;continue
        if start is None or (i and t[i]-t[i-1]>CRITERIA['maximum_native_gap_s']):start=float(t[i])
        if t[i]-start>=seconds-1e-8:return i,[start,float(t[i])]
    return None,None


def fixture_check(surfaces, segment):
    name=segment['ramp_model'];f=FIXTURES[name];r=surfaces[name];slope=f['slope_z_per_world_x']
    expected=np.array([-slope,0,1.])/math.sqrt(1+slope*slope)
    center=np.array([8.,f['center_y_m'],np.mean(f['landing_top_z_m'])-.05/math.sqrt(1+slope*slope)])
    landing_ok=all(np.allclose(surfaces[n]['rotation'],np.eye(3),atol=1e-10,rtol=0) and abs(surfaces[n]['center'][2]+surfaces[n]['half'][2]-z)<1e-9
                   for n,z in zip((f['low_x_landing'],f['high_x_landing']),f['landing_top_z_m']))
    return check(np.allclose(r['rotation'][:,2],expected,atol=1e-10,rtol=0) and np.allclose(r['center'],center,atol=1e-9,rtol=0)
                 and abs(r['half'][0]*2-(math.hypot(12,1.2)+.04))<1e-9 and abs(r['half'][1]*2-2)<1e-9 and landing_ok,
                 exact_actual_world_surfaces={n:surfaces[n] for n in (name,segment['entry_landing_model'],segment['exit_landing_model'])},
                 nominal_world_x_seams_m=[2.,14.],sloped_walkway_not_stairs=True)


def fixed_exit_support_end(common,goalindex):
    """Original source arrival/fixed first hold defines the exit proof horizon.

    A front toe may establish the first .6s landing continuity before rear toes
    finish leaving the ramp. Do not incorrectly confine every toe to that first
    short continuity interval; equally, do not search for a later quiet window.
    """
    arrivals=common['checks'].get('all_original_SLAM_3D_region_arrivals',{}).get('arrivals',[])
    if goalindex>=len(arrivals):raise MissingEvidence('No original requested destination region '+str(goalindex))
    arrival=arrivals[goalindex]
    pair=arrival.get('original_begin_end_ns')
    if type(pair) is not list or len(pair)!=2 or not all(type(v) is int and v>=0 for v in pair) or pair[1]<pair[0]:
        raise MissingEvidence('No original integer destination-region dwell stamps')
    end=pair[1]/1e9;basis='Original source destination-region dwell end'
    if goalindex==len(arrivals)-1:
        fixed=common['checks'].get('new_first_declared_active_hold_fixed_5s',{}).get('fixed_native_window_s')
        if fixed is None:raise MissingEvidence('Final exit support needs the original first declared hold window')
        if len(fixed)!=2 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in fixed) or abs(fixed[1]-fixed[0]-5.)>1e-8:
            raise ValueError('Original fixed first hold horizon differs')
        end=max(end,fixed[1]);basis='Original destination-region dwell and fixed first declared five-second hold end'
    return end,arrival,basis


def segment_audit(segment,index,n,com,centers,feet,surfaces,masks,common,cursor):
    checks={};metrics={};t=n['t'];rname=segment['ramp_model'];f=FIXTURES[rname];sign=segment['world_x_forward_sign']
    start,end=segment['entry_landing_model'],segment['exit_landing_model']
    entry_seam=2. if sign>0 else 14.;exit_seam=14. if sign>0 else 2.
    limit=CRITERIA['minimum_COM_distance_outside_each_seam_m']
    initial=(t>=max(.1,cursor))&(sign*(com[:,0]-entry_seam)<=-limit)
    entered=np.flatnonzero((t>=max(.1,cursor))&(sign*(com[:,0]-entry_seam)>=0))
    entry=int(entered[0]) if len(entered) else None
    exits=np.flatnonzero((t>=(t[entry] if entry is not None else math.inf))&(sign*(com[:,0]-exit_seam)>=limit))
    exit_index=int(exits[0]) if len(exits) else None
    crossed=entry is not None and exit_index is not None and exit_index>entry and initial.any()
    checks['complete_original_12m_fixture']=fixture_check(surfaces,segment)
    checks['actual_COM_entryoutside_full_axis_exitoutside']=check(crossed,
        actual_entry_crossing_world_s=float(t[entry]) if entry is not None else None,
        actual_exit_outside_world_s=float(t[exit_index]) if exit_index is not None else None,
        entry_outside_distance_m=limit,exit_outside_distance_m=limit,
        COM_source='Actual native base_link COM = original base pose + measured rotation × archived base_inertial_offset; not whole-robot aggregate COM')
    if not crossed:
        checks['complete_contact_traversal_and_destination']=check(None,reason='Complete physical seam crossing absent; surviving on a prefix is not full-ramp success')
        return checks,metrics,cursor
    initial&=t<t[entry]
    traversal=(t>=t[entry])&(t<=t[exit_index])
    # Landing confirmation starts with the first physical exit; never select a
    # later quiet window. Foot continuity is assessed against original 200Hz.
    landing=(t>=t[exit_index])
    any_exit=np.logical_or.reduce(list(masks[end].values()))
    landing_local=(com-surfaces[end]['center'])@surfaces[end]['rotation']
    on_landing=(abs(landing_local[:,:2])<=surfaces[end]['half'][:2]).all(1)
    destination_mask=landing&on_landing&any_exit&(sign*(com[:,0]-exit_seam)>=limit)
    confirmed,landing_interval=first_confirmation(destination_mask,t,.6)
    final_end=t[confirmed] if confirmed is not None else t[-1]
    goalindex=segment['destination_goal_index']
    foot_proof_end,arrival,foot_proof_basis=fixed_exit_support_end(common,goalindex)
    exit_window=landing&(t<=foot_proof_end+1e-9)
    feet_metrics={};ordered=True
    for leg in LEGS:
        ai,ap=first_confirmation(masks[start][leg]&initial,t,.1)
        bi,bp=first_confirmation(masks[rname][leg]&traversal,t,.1)
        ci,cp=first_confirmation(masks[end][leg]&exit_window,t,.1)
        ok=all(v is not None for v in (ai,bi,ci)) and ai<bi<ci
        ordered&=ok
        feet_metrics[leg]={'actual_ordered_entry_ramp_exit_confirmation_world_s':[float(t[v]) if v is not None else None for v in (ai,bi,ci)],
                           'entry_support_interval_s':ap,'ramp_support_interval_s':bp,'exit_support_interval_s':cp,**feet[leg]}
    all_exit=np.logical_and.reduce(list(masks[end].values()))
    four,four_span=span(all_exit&exit_window,t)
    checks['all_named_feet_ordered_actual_top_support']=check(ordered,feet=feet_metrics,
        fixed_exit_support_end_world_s=foot_proof_end,exit_support_horizon_source=foot_proof_basis,
        exit_continuity_first_confirmation_and_each_toe_confirmation_are_independent=True)
    checks['first_destination_landing_continuous_support']=check(confirmed is not None and four>=.1-1e-8,
        any_foot_support_confirmation_interval_s=landing_interval,minimum_continuous_any_foot_s=.6,
        simultaneous_four_feet_span_s=four,simultaneous_interval_s=four_span)
    bins=[]
    any_ramp=np.logical_or.reduce(list(masks[rname].values()))
    for edge in np.arange(2.,14.,1.):
        sel=traversal&any_ramp&(com[:,0]>=edge)&(com[:,0]<edge+1.)
        bins.append({'world_x_limits_m':[edge,edge+1.],'actual_native_top_support_seconds':float(sel.sum()*.005),
                     'native_samples':int(sel.sum())})
    checks['all_twelve_metre_bins_actual_support']=check(all(b['actual_native_top_support_seconds']>=.1-1e-8 for b in bins),bins=bins)
    selected=(t>=float(t[np.flatnonzero(initial)[0]]))&(t<=final_end)
    any_selected=np.logical_or.reduce([masks[name][leg] for name in (start,rname,end) for leg in LEGS])
    unsupported,unsupported_interval=span(selected&~any_selected,t)
    lateral=float(abs(com[traversal,1]-f['center_y_m']).max())
    progress=float(sign*(com[exit_index,0]-com[np.flatnonzero(initial)[0],0]))
    checks['whole_traversal_real_support_and_axis_safety']=check(unsupported<=.3+1e-9 and lateral<=.45,
        maximum_all_named_surface_feet_unsupported_s=unsupported,unsupported_interval_s=unsupported_interval,
        actual_COM_ramp_axis_lateral_error_max_m=lateral,actual_COM_projected_entry_exit_progress_m=progress)
    # Reuse the original source-defined arrival, then associate its original
    # headers with native state solely to verify which physical landing was used.
    begin,end_stamp=arrival['original_begin_end_ns'];begin/=1e9;end_stamp/=1e9
    interval=(t>=begin-1e-9)&(t<=end_stamp+1e-9)
    physically_inside=interval.any() and bool(on_landing[interval].all()) and bool((sign*(com[interval,0]-exit_seam)>=0).all())
    top=surfaces[end]['center'][2]+surfaces[end]['half'][2]
    clearance=n['pos'][interval,2]-top if interval.any() else np.array([])
    checks['actual_original_SLAM_destination_region_on_correct_physical_floor']=check(arrival.get('status')=='passed' and physically_inside and len(clearance)>0 and float(clearance.min())>=.18,
        original_goal_index=goalindex,original_goal_id=arrival.get('goal_id'),original_source_dwell_ns=arrival['original_begin_end_ns'],
        actual_physical_landing_model=end,actual_landing_top_z_m=top,
        minimum_actual_base_origin_z_above_destination_floor_m=float(clearance.min()) if len(clearance) else None,
        truth_used_only_to_assess_execution_after_source_arrival=True)
    metrics.update(feet=feet_metrics,entry_world_s=float(t[entry]),exit_outside_world_s=float(t[exit_index]),
                   first_destination_continuous_support_confirmed_world_s=float(t[confirmed]) if confirmed is not None else None,
                   actual_body_world_z_gain_m=float(n['pos'][exit_index,2]-n['pos'][entry,2]),
                   world_height_change_is_diagnostic_only=True)
    return checks,metrics,final_end


def terrain_switch_audit(run,n,execution,contract,base):
    """Actual SLAM-authorized Actor-only provider change, never navigation truth."""
    scope=read(run/'navigation_scope.json');config=scope.get('profile',{}).get('terrain_layer_switch')
    if len(contract['segments'])<2 and not config:
        return check(True,applicability='Single ramp; no terrain-provider switch was requested')
    if not config:raise MissingEvidence('A multi-ramp case needs an explicit frozen terrain-layer provider contract')
    event_rows=lines(run/'terrain_provider_events.jsonl');result=read(run/'terrain_provider_result.json')
    switches=[r for r in event_rows if r.get('event')=='switched_once']
    if result.get('status')!='switched' or result.get('failure') is not None or len(switches)!=1 or any(r.get('event')=='switch_refused_latched' for r in event_rows):
        return check(False,result_status=result.get('status'),failure=result.get('failure'),actual_switch_count=len(switches))
    event=switches[0]
    if any(r.get('navigation_ground_truth_used') is not False for r in event_rows):raise ValueError('Provider event crosses navigation source boundary')
    decoded={}
    for key in ('status_read','request_read'):
        record=event.get(key)
        if not record or type(record.get('raw_utf8')) is not str:raise MissingEvidence('Switch lacks original file read bytes '+key)
        raw=record['raw_utf8'].encode('utf-8')
        if hashlib.sha256(raw).hexdigest()!=record['raw_bytes_sha256']:raise ValueError('Switch original file bytes differ')
        decoded[key]=json.loads(raw)
    if decoded['request_read']!=read(run/'navigation_request.json'):raise ValueError('Provider used a different original navigation request')
    status=decoded['status_read'];status_rows=lines(run/'navigation_status.jsonl')
    if not any(canonical(row)==canonical(status) for row in status_rows):raise MissingEvidence('Original provider status read not joined to raw status history')
    arrival=event['original_region_arrival'];stamp=arrival['stamp_ns'];clock=event['native_request_clock_ns'];effective=event['native_state_effective_clock_ns']
    if type(stamp) is not int or type(clock) is not int or type(effective) is not int or effective!=clock-5_000_000 or stamp>clock:
        raise ValueError('Provider switched before its original source arrival or uses wrong native phase')
    if hashlib.sha256(canonical(arrival).encode()).hexdigest()!=event['region_arrival_sha256'] or arrival not in status.get('region_arrivals',[]):
        raise ValueError('Provider arrival differs from original source receipt')
    if arrival.get('goal_id')!='connector_mid' or config.get('completed_goal_id')!='connector_mid' or arrival.get('protected') is not False or arrival.get('region_inside') is not True or arrival.get('control_region_inside') is not True:
        raise ValueError('Provider was not authorized by the frozen measured connector region')
    if event['request_id']!=decoded['request_read']['request_id'] or event['goals_definition_sha256']!=status['goals_definition_sha256']:
        raise ValueError('Provider request/goals binding differs')
    k=int(np.argmin(abs(n['t']-effective/1e9)))
    if abs(n['t'][k]-effective/1e9)>1e-8:raise MissingEvidence('Switch native effective-time row absent')
    pos=np.asarray(event['actor_native_base_origin'],float);quat=np.asarray(event['actor_native_quaternion_wxyz'],float)
    if not np.allclose(pos,n['pos'][k],atol=1e-8,rtol=0) or not np.allclose(quat,n['q'][k],atol=1e-8,rtol=0):
        raise ValueError('Switch proof does not cite the actual native Actor-only sample')
    paths=[p for p in (run/'sources').rglob('observation.py') if p.parent.name=='policy']
    if len(paths)!=1:raise MissingEvidence('No unique immutable Actor terrain ray provider')
    observation=base.module(paths[0]);world=run/'world.sdf'
    initial=read(run/'terrain_target_manifest.json');alternate=read(run/'alternate_terrain_target_manifest.json')
    if set(initial['include_models'])!={'floor_1','ramp_12','floor_2'} or set(alternate['include_models'])!={'floor_2','ramp_23','floor_3'}:
        raise ValueError('Switch raw manifests differ from explicit lower12/upper23 providers')
    yaw=math.atan2(n['rot'][k,1,0],n['rot'][k,0,0]);c,s=math.cos(yaw),math.sin(yaw)
    xy=observation.SCAN_GRID_XY@np.array([[c,s],[-s,c]])+pos[:2]
    expected_starts=np.column_stack((xy,np.full(187,pos[2]+20.)))
    ray=event['ray_equivalence'];starts=np.asarray(ray['starts'],float)
    if starts.shape!=(187,3) or not np.isfinite(starts).all() or not np.allclose(starts,expected_starts,atol=1e-8,rtol=0):raise ValueError('Switch altered the frozen grid or +20m ray origin')
    a=observation.TerrainHeightMap.from_sdf(world,include_models=initial['include_models'])
    b=observation.TerrainHeightMap.from_sdf(world,include_models=alternate['include_models'])
    az,an=a.raycast(starts);bz,bn=b.raycast(starts)
    logged_a=np.asarray(ray['initial_hit_z'],float);logged_b=np.asarray(ray['alternate_hit_z'],float)
    replay=np.isfinite(az).all() and np.isfinite(bz).all() and az.shape==bz.shape==(187,) and logged_a.shape==logged_b.shape==(187,)
    if not replay:raise ValueError('Missing or nonfinite original shared-layer rays')
    shared=all(isinstance(name,str) and name.split('/')[0]=='floor_2' for name in list(an)+list(bn))
    errors=[float(abs(az-logged_a).max()),float(abs(bz-logged_b).max())]
    equality=float(abs(az-bz).max())
    rows_at=[r for r in execution if abs(float(r['world_sim_time'])-clock/1e9)<=1e-8]
    if len(rows_at)!=1:raise MissingEvidence('Actual Actor command-frame provider proof absent')
    applied=rows_at[0].get('actor_terrain_provider',{})
    good=(shared and equality<=1e-9 and max(errors)<=1e-9 and list(an)==ray['initial_collision_names'] and list(bn)==ray['alternate_collision_names']
          and ray.get('finite') is True and ray.get('shared_floor_2') is True
          and applied.get('current_provider')=='alternate' and applied.get('switched') is True and applied.get('failure') is None)
    return check(good,original_connector_arrival_stamp_ns=stamp,actual_native_request_clock_ns=clock,effective_physics_clock_ns=effective,
                 initial_manifest_sha256=sha(run/'terrain_target_manifest.json'),alternate_manifest_sha256=sha(run/'alternate_terrain_target_manifest.json'),
                 actual_187_ray_replay_errors_m=errors,max_actual_provider_hit_z_difference_m=equality,
                 all_actual_rays_on_shared_floor2=shared,identical_float64_bytes_diagnostic=az.tobytes()==bz.tobytes(),
                 native_state_only_checks_Actor_input_equivalence_and_is_not_navigation_feedback=True)


def evaluate(run,common_name='summary_closed_loop_cascade_independent.json',diagnostic_segments=None):
    run=Path(run).resolve();checks={};metrics={};inputs={};ancestor={};contract={};common={}
    def audit(name,fn):
        try:checks[name]=fn()
        except (MissingEvidence,KeyError,FileNotFoundError) as e:checks[name]=check(None,reason=type(e).__name__+': '+str(e))
        except (ValueError,TypeError,IndexError,AttributeError,ImportError,OSError) as e:checks[name]=check(False,reason=type(e).__name__+': '+str(e))
    try:
        common_path=run/common_name
        common,checks['all_unchanged_original_common_gates']=ancestry(run,common_path)
        ancestor={'common':{'path':str(common_path),'sha256':sha(common_path),'status':common['status'],
                            'evaluator_sha256':common['evaluator_sha256'],'checks':common['checks']}}
        execution=lines(run/'telemetry.jsonl')
        try:contract,checks['prospective_exact_ramp_contract']=contract_audit(run,execution)
        except MissingEvidence as e:
            checks['prospective_exact_ramp_contract']=check(None,reason=str(e),pilot_not_formally_pre_registered=True)
            if not diagnostic_segments:raise
            contract={'segments':planned_segments(diagnostic_segments)}
        source=run/'sources/acceptance/evaluate_closed_loop.py'
        if not source.is_file():source=HERE/'evaluate_closed_loop.py'
        if sha(source)!=COMMON_SHA:raise ValueError('Cannot use changed common native helper')
        base=module(source);n=base.native_state(run);native=n['steps'];t=n['t']
        # All physics safety/cadence/identity remains independently assessed, even
        # if the common receipt is failed or still geometrically unverified.
        checks.update({'native_recheck_'+k:v for k,v in base.native_audit(n).items()})
        iteration=array(native,'iteration');dt=array(native,'dt')
        checks['actual_native_iteration_phase_continuity']=check(len(t)>1 and np.all(np.diff(iteration)==1) and np.max(abs(dt-.005))<=1e-10 and np.all(np.diff(t)>0) and np.max(np.diff(t))<=.005001,
            original_native_samples=len(t),phase='Original PreUpdate state at t-dt, not t',maximum_gap_s=float(np.max(np.diff(t))))
        world=ET.parse(run/'world.sdf').getroot()
        model=world.find("world/model[@name='go2']")
        inertial=model.find("link[@name='base_link']/inertial") if model is not None else None
        offset,rr=pose(inertial)
        declared=np.asarray(n['contract']['base_com_offset_body_m'],float)
        com=n['pos']+np.einsum('nij,j->ni',n['rot'],offset)
        integration=float(abs(np.diff(n['pos'],axis=0)-n['world_origin'][1:]*np.diff(t)[:,None]).max())
        checks['actual_native_COM_and_origin_translation_consistency']=check(np.allclose(offset,declared,atol=1e-12,rtol=0) and np.allclose(rr,np.eye(3),atol=1e-12,rtol=0) and integration<=.0001,
            actual_base_link_COM_offset_body_m=offset,contract_offset_body_m=declared,
            maximum_native_pose_origin_velocity_integral_error_m=integration,
            base_origin_clearance_and_base_COM_progress_are_distinguished=True)
        centers,feet=foot_centers(world,native,n['contract']['joint_order'],n['rot'])
        names=set()
        for s in contract['segments']:names.update((s['ramp_model'],s['entry_landing_model'],s['exit_landing_model']))
        surfaces={name:surface(world,name) for name in names}
        masks,raw_support=support_masks(native,centers,feet,surfaces)
        checks['original_named_toe_world_contact_XYZ_normals_available']=check(None if raw_support['malformed_relevant_contact_examples'] else True,**raw_support)
        if raw_support['malformed_relevant_contact_examples'] or raw_support['relevant_exact_foot_surface_pairs']==0:
            raise MissingEvidence('Incomplete raw foot/surface contact geometry cannot prove or disprove all strict support gates')
        audit('original_source_authorized_causal_terrain_layer_switch',lambda:terrain_switch_audit(run,n,execution,contract,base))
        cursor=.1
        for index,segment in enumerate(contract['segments']):
            segment_checks,segment_metrics,cursor=segment_audit(segment,index,n,com,centers,feet,surfaces,masks,common,cursor)
            checks.update({f'leg_{index}_{segment["ramp_model"]}_{k}':v for k,v in segment_checks.items()})
            metrics[f'leg_{index}']={'prospective':segment,**segment_metrics}
        parking=common['checks'].get('new_first_declared_active_hold_fixed_5s',{})
        if parking.get('fixed_native_window_s') is None:raise MissingEvidence('Original first active-hold native window missing')
        begin,end=parking['fixed_native_window_s'];stop=(t>=begin-1e-9)&(t<=end+1e-9)
        finalname=contract['segments'][-1]['exit_landing_model']
        finalall=np.logical_and.reduce(list(masks[finalname].values()))
        finalspan,interval=span(finalall&stop,t)
        checks['unchanged_first_active_hold_on_final_actual_landing']=check(parking.get('status')=='passed' and stop.sum()>=1000 and finalspan>=.1-1e-8 and bool(finalall[np.flatnonzero(stop)[-1]]),
            original_fixed_window_s=[begin,end],native_samples=int(stop.sum()),actual_final_surface=finalname,
            simultaneous_four_foot_support_span_s=finalspan,interval_s=interval,
            original_all_numeric_drift_speed_yaw_commands_and_source_gates=parking)
        metrics['final_origin_height_above_actual_floor_m']=float(n['pos'][-1,2]-(surfaces[finalname]['center'][2]+surfaces[finalname]['half'][2]))
    except (MissingEvidence,KeyError,FileNotFoundError) as e:checks['complete_original_required_ramp_evidence']=check(None,reason=type(e).__name__+': '+str(e))
    except (ValueError,TypeError,IndexError,AttributeError,ImportError,OSError) as e:checks['valid_original_required_ramp_evidence']=check(False,reason=type(e).__name__+': '+str(e))
    for name in (common_name,'ramp_acceptance_contract.json','ramp_acceptance_source_manifest.json','world.sdf','navigation_scope.json','actuator.jsonl','telemetry.jsonl','navigation_request.json','navigation_anchor.json','navigation_pid_history.jsonl','navigation_slam_poses.jsonl','runtime_manifest.json','worker_result.json','sources/acceptance/evaluate_ramp.py','sources/acceptance/evaluate_closed_loop.py','terrain_provider_events.jsonl','terrain_provider_result.json','terrain_target_manifest.json','alternate_terrain_target_manifest.json'):
        p=run/name
        if p.is_file():inputs[str(p)]=sha(p)
    status=overall(checks)
    return clean({'schema':SCHEMA,'run':str(run),'status':status,'score':None,'checks':checks,'metrics':metrics,
        'criteria':CRITERIA,'prospective_contract':contract,'ancestors':ancestor,
        'evaluator_sha256':sha(__file__),'verified_input_source_sha256':inputs,
        'scope':{'actual_SLAM_SCAN_complete_selected_ramp_navigation':status,'simulation_only':True,
                 'navigation_ground_truth_used':False,'native_ground_truth_used_only_offline':True,
                 'privileged_Actor_dimensions':232,'controller_known_dimensions':15,
                 'full_real_sensor_Actor_deployment':'unverified','absolute_automatic_old_map_registration':'unverified',
                 'all_three_floors':'passed' if status=='passed' and len(contract.get('segments',[]))>=2 and {s['ramp_model'] for s in contract['segments']}=={'ramp_12','ramp_23'} else 'unverified',
                 'dynamic_obstacle_stop_resume':'unverified','true_stairs':'unverified',
                 'full_46_region_mission':'unverified','general_Sim2Sim':'unverified','hardware':'unverified'},
        'meaning':'Different explicit nonflat append-only contract. No body-height ratio, no truth-derived navigation goal, no changed original common receipt or post-selected quiet parking window.'})


def selftest():
    import tempfile
    assert check(None)['status']=='unverified' and overall({'a':check(False),'b':check(None)})=='failed'
    assert planned_segments('ramp_12:up:0,ramp_23:up:2')[1]['world_x_forward_sign']==-1
    for text in ('ramp_12:up:0,ramp_23:up:0','ramp_12:up:0,ramp_23:down:2','ramp_99:up:0'):
        try:planned_segments(text)
        except ValueError:pass
        else:raise AssertionError('Bad sequence accepted')
    t=np.arange(0,1.005,.005);good=np.ones(len(t),bool)
    assert first_confirmation(good,t,.6)[0]==120
    broken=good.copy();broken[50]=False
    assert first_confirmation(broken,t,.6)[0]==171
    assert first_confirmation(good[:100],t[:100],.6)[0] is None
    assert first_confirmation(good,t+np.where(t>=.3,.02,0),.6)[0]==180
    # True top support; touching the box side or an unnamed link never passes.
    s={'name':'floor','center':np.zeros(3),'rotation':np.eye(3),'half':np.ones(3),'collision':'floor::link::collision'}
    feet={leg:{'collision':f'go2::{leg}::foot','radius_m':.02} for leg in LEGS}
    centers={leg:np.array([[0.,0.,1.02]]) for leg in LEGS}
    pair={'a':feet['FR']['collision'],'b':s['collision'],'group':1,'points':1,'positions_world':[[0.,0.,1.]],'normals_world':[[0.,0.,1.]]}
    masks,quality=support_masks([{'contact_pairs':[pair]}],centers,feet,{'floor':s})
    assert masks['floor']['FR'][0] and not masks['floor']['FL'][0] and not quality['malformed_relevant_contact_examples']
    for field,value in [('positions_world',[[1.,0.,0.]]),('normals_world',[[1.,0.,0.]]),('a','go2::base::collision')]:
        changed={**pair,field:value};m,_=support_masks([{'contact_pairs':[changed]}],centers,feet,{'floor':s});assert not m['floor']['FR'][0]
    m,q=support_masks([{'contact_pairs':[{**pair,'normals_world':[]}]}],centers,feet,{'floor':s})
    assert not m['floor']['FR'][0] and q['malformed_relevant_contact_examples']
    # These original safety/drift gates are not loosened by the supplement.
    assert CRITERIA['parking_xy_drift_m']==.05 and CRITERIA['parking_yaw_drift_rad']==.1
    assert CRITERIA['goal_timeout_sim_s']==90 and not CRITERIA['body_world_height_gain_ratio_is_acceptance_gate']
    c={'checks':{'all_original_SLAM_3D_region_arrivals':{'arrivals':[
        {'original_begin_end_ns':[1_200_000_000,1_800_000_000]},
        {'original_begin_end_ns':[9_400_000_000,10_000_000_000]}]},
        'new_first_declared_active_hold_fixed_5s':{'fixed_native_window_s':[10.065,15.065]}}}
    assert fixed_exit_support_end(c,0)[0]==1.8 and fixed_exit_support_end(c,1)[0]==15.065
    # Rear toe's real .1s support after a front toe's first .6s continuity is
    # accepted only within the fixed source horizon; extension is not arbitrary.
    tt=np.arange(0,2.005,.005);front=(tt>=.2);rear=(tt>=1.0)&(tt<=1.2)
    first,_=first_confirmation(front,tt,.6);assert abs(tt[first]-.8)<1e-8
    assert span(rear&(tt<=tt[first]),tt)[0]==0 and span(rear&(tt<=fixed_exit_support_end(c,0)[0]),tt)[0]>=.1
    # Explicit prelaunch archive/copy/hash and refusal tests use only private
    # temporary fake evidence. They never construct a simulator or ROS node.
    with tempfile.TemporaryDirectory(prefix='closed_ramp_receipt_negative_') as folder:
        run=Path(folder)
        (run/'world.sdf').write_text('<sdf version="1.9"><world name="fake"/></sdf>')
        (run/'navigation_scope.json').write_text('{}')
        prepared=prepare_contract(run,'ramp_12:up:0')
        contract,verdict=contract_audit(run,[{'closed_loop_read_evidence':{'actual_read_attempt':{'read_monotonic_wall':time.monotonic()+1.}}}])
        assert verdict['status']=='passed' and prepared['contract_sha256']==sha(run/'ramp_acceptance_contract.json')
        try:prepare_contract(run,'ramp_12:up:0')
        except ValueError:pass
        else:raise AssertionError('Prelaunch protocol overwritten')
        try:contract_audit(run,[{'closed_loop_read_evidence':{'actual_read_attempt':{'read_monotonic_wall':contract['prepared_monotonic_wall']-1.}}}])
        except ValueError:pass
        else:raise AssertionError('Retrospective contract accepted')
        original={'schema':'independent_actual_SLAM_SCAN_cascade_navigation/v1','run':str(run),
                  'evaluator_sha256':COMMON_SHA,'verified_input_source_sha256':{str(run/'world.sdf'):sha(run/'world.sdf')},
                  'checks':{APPLICABILITY:check(None)},'status':'unverified'}
        fp=run/'fake_common.json';fp.write_text(json.dumps(original))
        try:ancestry(run,fp)
        except MissingEvidence:pass
        else:raise AssertionError('Only applicability and missing real checks became PASS')
        original['checks'].update({name:check(True) for name in REQUIRED_COMMON_CHECKS})
        original['checks']['native_physical_safety']=check(False)
        original['status']='failed';fp.write_text(json.dumps(original))
        _,verdict=ancestry(run,fp);assert verdict['status']=='failed'
        (run/'world.sdf').write_text('changed original input')
        try:ancestry(run,fp)
        except ValueError:pass
        else:raise AssertionError('Changed original input accepted')
    with tempfile.TemporaryDirectory(prefix='closed_ramp_receipt_live_refusal_') as folder:
        run=Path(folder);(run/'telemetry.jsonl').write_text('')
        try:prepare_contract(run,'ramp_12:up:0')
        except ValueError:pass
        else:raise AssertionError('Prepare after original execution file accepted')
    return {'status':'passed','meaningful_groups':17,'live_ROS_Gazebo_policy_operations':0}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path);p.add_argument('--prepare-contract',action='store_true')
    p.add_argument('--segments',help='Prospective explicit sequence: ramp_12:up:0,ramp_23:up:2')
    p.add_argument('--diagnostic-segments',help='Unregistered historical/pilot diagnostic only; never formal PASS')
    p.add_argument('--common-receipt',default='summary_closed_loop_cascade_independent.json')
    p.add_argument('--receipt-suffix');p.add_argument('--no-write',action='store_true');p.add_argument('--self-test',action='store_true')
    a=p.parse_args()
    if a.self_test:print(json.dumps(selftest()));return
    if a.run is None:p.error('--run required')
    if a.prepare_contract:
        if not a.segments:p.error('--prepare-contract requires --segments')
        print(json.dumps(prepare_contract(a.run,a.segments),indent=2));return
    if a.receipt_suffix and not a.receipt_suffix.replace('_','').replace('-','').isalnum():p.error('Invalid receipt suffix')
    filename='summary_closed_loop_ramp_independent'+('.'+a.receipt_suffix if a.receipt_suffix else '')+'.json'
    dest=a.run.resolve()/filename
    if not a.no_write and dest.exists():p.error('Refusing to replace prior receipt')
    result=evaluate(a.run,a.common_receipt,a.diagnostic_segments)
    if not a.no_write:
        with dest.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':result['status'],'receipt':None if a.no_write else str(dest),
                      'checks':{k:v['status'] for k,v in result['checks'].items()}},indent=2))


if __name__=='__main__':main()
