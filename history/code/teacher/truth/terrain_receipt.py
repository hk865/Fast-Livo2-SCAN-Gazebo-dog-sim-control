#!/usr/bin/env python3
"""Independent actual contact/surface receipt for truth-feedback terrain trials.

Only writes a new summary_truth_terrain.json. No simulator, ROS, command writer,
training operation or pose reset is available. Body world-height gain is never
an acceptance gate. Contact positions/normals must come from original physics.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
LEGS = {'FR': 'rf', 'FL': 'lf', 'RR': 'rh', 'RL': 'lh'}
SEMANTIC_SOURCE_SHA256 = {
    'tests/ramp_full_protocol.json': '1e7e1357604ca9d9ce6a9eb49eeef3079643867af777e26a1dee0954a7b8ce28',
    'tests/step_functional_protocol.json': '8b7a43c74b21c063f5b56984e2b63256ee9bb0db88fda5c2f59944bb8b70f431',
    'scripts/analyze_full_ramp.py': 'f7d420cf35332f0a84d9d3ed193d2926e0f1f52ff5bea73ca096c9e7dd36a172',
    'scripts/evaluate_step_functional.py': 'e4cfe4c4b6e4c22213c77922ff289bd0ec59e9e9753069ab582b91fc8b8ec6b5'}
CRITERIA = {'physics_dt_s': .005, 'max_native_gap_s': .005001, 'xy_contact_margin_m': .005,
            'top_normal_distance_tolerance_m': .015, 'minimum_abs_normal_dot': .8,
            'ramp_foot_center_height_tolerance_m': .025, 'step_foot_center_height_tolerance_m': .015,
            'continuous_support_s': .1, 'maximum_all_feet_unsupported_s': .3,
            'ramp_nominal_horizontal_span_m': 12., 'ramp_bin_m': 1., 'minimum_bin_support_s': .1,
            'ramp_minimum_forward_progress_m': 13.2, 'rolling_velocity_window_s': .5,
            'minimum_positive_world_forward_mean_mps': .05, 'minimum_positive_window_fraction': .9,
            'step_body_start_past_edge_m': .5, 'step_minimum_continuation_m': 1.,
            'step_minimum_continuation_duration_s': 3., 'initial_support_window_elapsed_s': [1.5, 3.]}
FIXTURES = {
    'ramp_12': {'y': 2., 'slope': .1, 'low_x_landing': 'floor_1', 'high_x_landing': 'floor_2', 'heights': [0., 1.2]},
    'ramp_23': {'y': 7., 'slope': -.1, 'low_x_landing': 'floor_3', 'high_x_landing': 'floor_2', 'heights': [2.4, 1.2]}}


class EvidenceMissing(ValueError):
    pass


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(1024*1024), b''):
            h.update(data)
    return h.hexdigest()


def read_lines(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def clean(value):
    if isinstance(value, np.ndarray): return clean(value.tolist())
    if isinstance(value, np.generic): return clean(value.item())
    if isinstance(value, dict): return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return None
    return value


def load_module(path, prefix):
    name = prefix + sha(path)[:12]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    sys.modules[name] = module
    try: spec.loader.exec_module(module)
    finally: sys.dont_write_bytecode = previous
    return module


def array(data, key, width=None):
    if any(key not in r for r in data): raise EvidenceMissing('Missing native ' + key)
    out = np.asarray([r[key] for r in data], float)
    shape = (len(data),) if width is None else (len(data), width)
    if out.shape != shape or not np.isfinite(out).all(): raise EvidenceMissing('Incomplete finite native ' + key)
    return out


def pose(element):
    if element is None: raise EvidenceMissing('Missing SDF element')
    e = element.find('pose')
    vals = np.fromstring(e.text or '', sep=' ') if e is not None else np.zeros(6)
    if vals.shape != (6,) or not np.isfinite(vals).all(): raise EvidenceMissing('Invalid SDF pose')
    r, p, y = vals[3:]
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return vals[:3], np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                              [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def axis_rotations(axis, angles):
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-8: raise EvidenceMissing('Invalid joint axis')
    axis = axis/np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    c, s = np.cos(angles)[:, None, None], np.sin(angles)[:, None, None]
    return c*np.eye(3)+(1-c)*np.outer(axis, axis)+s*k


def foot_centers(world, native, joint_order, rotation):
    """Spherical collision FK from measured q/pose, never inferred contact."""
    model = world.find("world/model[@name='go2']")
    if model is None: raise EvidenceMissing('Missing original go2 SDF model')
    q, base = array(native, 'q', 12), array(native, 'position', 3)
    centers, feet = {}, {}
    for leg, short in LEGS.items():
        p, r, parent, chain = base.copy(), rotation.copy(), 'base_link', []
        for part in ('hip', 'upper_leg', 'lower_leg'):
            name = f'{short}_{part}_joint'
            joint = model.find(f"joint[@name='{name}']")
            if joint is None or joint.find('pose') is None: raise EvidenceMissing('Missing actual joint ' + name)
            child = joint.findtext('child')
            if joint.findtext('parent') != parent or joint.find('pose').get('relative_to') != parent: raise EvidenceMissing('Unsupported joint frame ' + name)
            xyz, rr = pose(joint)
            p += np.einsum('nij,j->ni', r, xyz)
            r = r @ rr
            axis = joint.find('axis/xyz')
            if axis is None or axis.get('expressed_in'): raise EvidenceMissing('Unsupported joint-axis frame')
            r = r @ axis_rotations(np.fromstring(axis.text, sep=' '), q[:, joint_order.index(name)])
            link = model.find(f"link[@name='{child}']")
            lp = link.find('pose') if link is not None else None
            if lp is not None and lp.get('relative_to') != name: raise EvidenceMissing('Unsupported child frame')
            xyz, rr = pose(link)
            p += np.einsum('nij,j->ni', r, xyz)
            r = r @ rr
            parent = child
            chain.append(name)
        collisions = [c for c in link.findall('collision') if f'{short}_foot_link_collision' in c.get('name', '')]
        if len(collisions) != 1: raise EvidenceMissing('Expected exactly one actual spherical foot collision')
        col = collisions[0]
        radius = float(col.findtext('geometry/sphere/radius'))
        if not math.isfinite(radius) or radius<=0: raise EvidenceMissing('Invalid foot collision radius')
        if col.find('pose') is not None and col.find('pose').get('relative_to'): raise EvidenceMissing('Nonlocal foot collision pose')
        xyz, _ = pose(col)
        centers[leg] = p + np.einsum('nij,j->ni', r, xyz)
        feet[leg] = {'collision': f'go2::{parent}::{col.get("name")}', 'radius_m': radius, 'chain': chain}
    return centers, feet


def surface(world, name):
    model = world.find(f"world/model[@name='{name}']")
    if model is None or model.findtext('static', '').lower() not in ('true', '1'): raise EvidenceMissing('Missing actual static surface ' + name)
    links = [link for link in model.findall('link') if link.findall('collision')]
    if len(links)!=1 or len(links[0].findall('collision'))!=1: raise EvidenceMissing('Ambiguous actual surface collision ' + name)
    link, col = links[0], links[0].find('collision')
    for element in (model, link, col):
        if element.find('pose') is not None and element.find('pose').get('relative_to'): raise EvidenceMissing('Unsupported static relative frame')
    mp, mr = pose(model); lp, lr = pose(link); cp, cr = pose(col)
    size = np.fromstring(col.findtext('geometry/box/size', ''), sep=' ')
    if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0): raise EvidenceMissing('Nonbox actual terrain surface ' + name)
    return {'name': name, 'collision': f'{name}::{link.get("name")}::{col.get("name")}',
            'center': mp+mr@lp+mr@lr@cp, 'rotation': mr@lr@cr, 'half': size/2}


def support_masks(native, centers, feet, surfaces, kind):
    masks = {name: {leg: np.zeros(len(native), bool) for leg in LEGS} for name in surfaces}
    malformed = []; relevant = 0; qualified = {name: 0 for name in surfaces}
    if any(type(row.get('contact_pairs')) is not list for row in native): raise EvidenceMissing('Original native contact_pairs unavailable')
    for name, slab in surfaces.items():
        normal = slab['rotation'][:, 2]
        local = {leg: (centers[leg]-slab['center'])@slab['rotation'] for leg in LEGS}
        for i, row in enumerate(native):
            for pair in row['contact_pairs']:
                group = pair.get('group')
                if group not in (1, 2, 3, 4): continue
                leg = list(LEGS)[group-1]
                if {pair.get('a'), pair.get('b')} != {feet[leg]['collision'], slab['collision']}: continue
                relevant += 1
                count = pair.get('points')
                xyz = np.asarray(pair.get('positions_world', []), float)
                normals = np.asarray(pair.get('normals_world', []), float)
                if type(count) is not int or count<1 or xyz.shape!=(count, 3) or normals.shape!=(count, 3) or not np.isfinite(xyz).all() or not np.isfinite(normals).all() or np.any(np.linalg.norm(normals, axis=1)<1e-8):
                    if len(malformed)<20: malformed.append({'native_index': i, 'surface': name, 'foot': leg})
                    continue
                point = (xyz-slab['center'])@slab['rotation']
                nn = normals/np.linalg.norm(normals, axis=1)[:, None]
                top = (abs(point[:, :2])<=slab['half'][:2]-CRITERIA['xy_contact_margin_m']).all(axis=1) & (abs(point[:, 2]-slab['half'][2])<=CRITERIA['top_normal_distance_tolerance_m']) & (abs(nn@normal)>=CRITERIA['minimum_abs_normal_dot'])
                c = local[leg][i]; radius = feet[leg]['radius_m']
                tolerance = CRITERIA[kind+'_foot_center_height_tolerance_m']
                extra = radius if kind=='ramp' else 0.
                center_ok = (abs(c[:2])<=slab['half'][:2]+extra).all() and abs(c[2]-slab['half'][2]-radius)<=tolerance
                if top.any() and center_ok:
                    masks[name][leg][i] = True
                    qualified[name] += int(top.sum())
    return masks, {'malformed_relevant_contact_examples': malformed, 'malformed_relevant_contact_present': bool(malformed),
                   'relevant_exact_foot_surface_pairs': relevant, 'qualified_actual_top_points': qualified,
                   'support_source': 'Exact physical foot/surface collision pair plus original world contact XYZ/normal and measured-q spherical collision FK; missing normals never inferred',
                   'measured_vertical_motor_or_bearing_load_claimed': False}


def span(mask, t):
    best = 0.; start = None; interval = None
    for i, valid in enumerate(mask):
        if not valid: start = None; continue
        if start is None or (i and t[i]-t[i-1]>CRITERIA['max_native_gap_s']): start = float(t[i])
        length = float(t[i]-start)
        if length>best: best, interval = length, [start, float(t[i])]
    return best, interval


def confirmation(mask, t):
    begin = None
    for i, valid in enumerate(mask):
        if not valid: begin = None; continue
        if begin is None or (i and t[i]-t[i-1]>CRITERIA['max_native_gap_s']): begin = float(t[i])
        if t[i]-begin >= CRITERIA['continuous_support_s']-1e-8: return float(t[i])
    return None


def positive_fraction(velocity):
    width = int(round(CRITERIA['rolling_velocity_window_s']/CRITERIA['physics_dt_s']))
    rolling = np.convolve(velocity, np.ones(width)/width, mode='valid') if len(velocity)>=width else np.array([])
    return (float((rolling>CRITERIA['minimum_positive_world_forward_mean_mps']).mean()) if len(rolling) else None), len(rolling)


def evaluate(run, write=True):
    run = Path(run).resolve(); out = run/'summary_truth_terrain.json'
    if write and out.exists(): raise ValueError('Refusing to replace prior terrain evidence')
    result = {'schema': 'truth_teacher_actual_terrain_receipt/v1', 'run': str(run), 'status': 'unverified',
              'scope': 'Privileged simulator-feedback Teacher locomotion benchmark; not SLAM navigation, stairs or real robot',
              'navigation_ground_truth_used': True, 'SLAM_navigation_verified': False, 'real_robot_verified': False,
              'strict_terrain_motion_verified': False, 'score': None, 'checks': {}, 'metrics': {},
              'criteria': CRITERIA, 'semantic_source_sha256': SEMANTIC_SOURCE_SHA256,
              'world_body_height_used_as_pass_gate': False}
    source_paths = [Path(__file__).resolve(), run/'truth_profile.json', run/'world.sdf', run/'actuator.jsonl',
                    run/'control.jsonl', run/'telemetry.jsonl', run/'worker_result.json', run/'runtime_manifest.json']
    def check(name, value, **evidence):
        result['checks'][name] = {'status': 'unverified' if value is None else 'passed' if bool(value) else 'failed', **evidence}
    try:
        profile = json.loads((run/'truth_profile.json').read_text())
        worker = json.loads((run/'worker_result.json').read_text())
        evaluator_path = run/'sources/truth/evaluate.py'
        if not evaluator_path.exists(): raise EvidenceMissing('Missing immutable archived truth evaluator')
        base_module = load_module(evaluator_path, '_truth_terrain_base_')
        source_paths += [evaluator_path, evaluator_path.with_name('protocol.json'), run/'sources/policy/observation.py', run/'sources/policy/contract.json']
        # No writes from the common evaluator, and no modification of its old
        # strict-terrain-unverified receipt. This new receipt fills only that gate.
        base = base_module.evaluate_run(run, write=False)
        common = {k: v for k, v in base['checks'].items() if k!='strict_terrain_support_applicability'}
        result['common_benchmark_checks'] = common
        result['common_benchmark_original_status'] = base['status']
        hard_failed = [k for k, v in common.items() if v['status']=='failed' and k!='complete_parseable_required_evidence']
        common_value = False if hard_failed else True if common and all(v['status']=='passed' for v in common.values()) else None
        check('unchanged_common_truth_benchmark', common_value,
              failed_checks=[k for k, v in common.items() if v['status']=='failed'],
              unverified_checks=[k for k, v in common.items() if v['status']=='unverified'],
              motion_safety_parking_thresholds='Unchanged archived truth protocol v2; no historical .15m/.2rad stop relaxation')
        original = read_lines(run/'actuator.jsonl')
        native = [r for r in original if r.get('kind')=='physics_step']
        contracts = [r for r in original if r.get('kind')=='actuator_contract']
        if len(contracts)!=1: raise EvidenceMissing('Exactly one actual native actuator_contract required')
        contract = contracts[0]
        world = ET.parse(run/'world.sdf').getroot()
        t = array(native, 't')-array(native, 'dt')
        iteration = array(native, 'iteration')
        pos = array(native, 'position', 3)
        rot = base_module.rotation_wxyz(array(native, 'quaternion_wxyz', 4))
        world_v = np.einsum('nij,nj->ni', rot, array(native, 'body_lin_vel_com', 3))
        centers, feet = foot_centers(world, native, contract['joint_order'], rot)
        control = base_module.control_rows(read_lines(run/'control.jsonl'))
        ct = np.asarray([float(r['control_t_s']) for r in control])
        ci = np.searchsorted(ct, t, side='right')-1; bound = np.clip(ci, 0, len(control)-1)
        mode = np.asarray([control[i]['mode'] for i in bound])
        cmdbody = np.asarray([control[i]['command_body'] for i in bound], float)
        age = t-ct[bound]
        active = (ci>=0)&(age>=-1e-8)&(age<=.301)&(mode=='drive')&(np.linalg.norm(cmdbody[:, :2], axis=1)>.03)
        elapsed_offset = float(control[0]['control_t_s']-control[0]['elapsed_s'])
        elapsed = t-elapsed_offset
        initial = (elapsed>=CRITERIA['initial_support_window_elapsed_s'][0])&(elapsed<CRITERIA['initial_support_window_elapsed_s'][1])
        parking_info = base['checks'].get('fixed_final_goal_parking', {}).get('parking')
        stop = np.zeros(len(t), bool)
        if parking_info is not None:
            start, end = parking_info['window_s']; stop=(t>=start-1e-8)&(t<=end+1e-8)
        check('native_cadence_and_unassisted_contract', len(native)>1 and np.all(np.diff(iteration)==1) and
              np.max(np.diff(t))<=CRITERIA['max_native_gap_s'] and contract.get('body_pose_resets')==0 and
              contract.get('joint_position_reset_count')==12 and contract.get('writer')=='teacher_sim::TeacherActuator sole JointForceCmd writer' and
              contract.get('kp')==25 and contract.get('kd')==.5 and contract.get('expected_dt')==.005 and
              contract.get('decimation')==4 and contract.get('effort_limit')==23.5 and contract.get('velocity_limit')==30,
              native_rows=len(native), maximum_native_gap_s=float(np.max(np.diff(t))),
              body_pose_resets=contract.get('body_pose_resets'), joint_position_reset_count=contract.get('joint_position_reset_count'),
              measured_foot_center_source='FK of original run/world.sdf spherical collision chain, original native measured q/base pose at t-dt; not a direct foot_xyz field',
              contacts_authority='Original actual gz.msgs.Contact fields', physics_pose_time='native t-dt, dt=.005')
        route = np.asarray(profile['route_world_xyz'], float)
        fixture = profile.get('terrain_acceptance', {})
        terrain = str(profile['terrain'])
        kind = fixture.get('kind', 'step' if terrain.startswith('step') else 'ramp' if terrain.startswith('ramp') else None)
        if kind not in ('ramp', 'step'): raise EvidenceMissing('No prospective ramp/step fixture selected')
        result['terrain_kind'] = kind
        if kind=='ramp':
            ramp_name = fixture.get('ramp_model')
            if ramp_name is None:
                matches=[name for name, f in FIXTURES.items() if np.max(abs(route[:, 1]-f['y']))<.05]
                if len(matches)!=1: raise EvidenceMissing('Frozen route does not uniquely identify ramp')
                ramp_name=matches[0]
            if ramp_name not in FIXTURES: raise EvidenceMissing('Unsupported requested complete ramp fixture')
            f=FIXTURES[ramp_name]; sign=1 if route[-1, 0]>route[0, 0] else -1
            start_name=f['low_x_landing'] if sign>0 else f['high_x_landing']; end_name=f['high_x_landing'] if sign>0 else f['low_x_landing']
            if fixture.get('start_landing', start_name)!=start_name or fixture.get('destination_landing', end_name)!=end_name:
                raise ValueError('Declared landing order conflicts with frozen route direction')
            surfaces={name:surface(world, name) for name in (ramp_name, start_name, end_name)}
            ramp=surfaces[ramp_name]; normal=np.array([-f['slope'],0,1])/math.sqrt(1+f['slope']**2)
            expected_center=np.array([8.,f['y'],np.mean(f['heights'])-.05/math.sqrt(1+f['slope']**2)])
            landing_ok=all(np.allclose(surfaces[name]['rotation'],np.eye(3),atol=1e-10) and abs(surfaces[name]['center'][2]+surfaces[name]['half'][2]-height)<1e-9
                           for name,height in zip((f['low_x_landing'],f['high_x_landing']),f['heights']))
            geometry_ok=np.allclose(ramp['rotation'][:,2],normal,atol=1e-10) and np.allclose(ramp['center'],expected_center,atol=1e-9) and abs(2*ramp['half'][0]-(math.hypot(12,1.2)+.04))<1e-9 and abs(2*ramp['half'][1]-2)<1e-9 and landing_ok
            check('complete_original_ramp_fixture',geometry_ok,ramp_model=ramp_name,actual_surfaces=surfaces,
                  start_landing=start_name,destination_landing=end_name,nominal_world_x_seams_m=[2.,14.],horizontal_span_m=12.)
            masks, ge = support_masks(native, centers, feet, surfaces, 'ramp')
            check('actual_contact_geometry_available',None if ge['malformed_relevant_contact_present'] else True,**ge)
            each={}; ordered=True
            for leg in LEGS:
                a, ai=span(masks[start_name][leg]&initial,t);b, bi=span(masks[ramp_name][leg]&active,t);c, cinterval=span(masks[end_name][leg]&stop,t)
                ac=confirmation(masks[start_name][leg]&initial,t);bc=confirmation(masks[ramp_name][leg]&active,t);cc=confirmation(masks[end_name][leg]&stop,t)
                ok=all(v is not None for v in (ac,bc,cc)) and ac<bc<cc and min(a,b,c)>=.1-1e-8 and bool(masks[end_name][leg][-1])
                ordered=ordered and ok
                each[leg]={'initial_landing_span_s':a,'initial_interval_s':ai,'ramp_span_s':b,'ramp_interval_s':bi,'final_landing_span_s':c,'final_interval_s':cinterval,'ordered_support_confirmations_world_s':[ac,bc,cc],'final_destination_support':bool(masks[end_name][leg][-1]),**feet[leg]}
            check('each_foot_ordered_landing_ramp_destination_support',ordered if stop.any() else None,feet=each)
            ramp_any=np.logical_or.reduce(list(masks[ramp_name].values()));bins=[]
            for edge in np.arange(2.,14.,1.):
                selected=active&ramp_any&(pos[:,0]>=edge)&(pos[:,0]<=edge+1.)
                bins.append({'world_x_limits_m':[edge,edge+1.],'actual_supported_seconds':float(selected.sum()*.005)})
            check('entire_twelve_meter_contact_coverage',len(bins)==12 and all(b['actual_supported_seconds']>=.1-1e-8 for b in bins),bins=bins)
            indices=np.flatnonzero(active);progress=None;fraction=None;window_count=0;crossed=False;continuous_crossing=False
            if len(indices) and initial.any():
                progress=float(sign*(pos[indices[-1],0]-np.median(pos[initial,0])))
                fraction,window_count=positive_fraction(sign*world_v[active,0])
                crossed=bool((pos[active,0]<=2.).any() and (pos[active,0]>=14.).any())
                entry_seam=2. if sign>0 else 14.;exit_seam=14. if sign>0 else 2.
                entered=np.flatnonzero(active & (sign*(pos[:,0]-entry_seam)>=0))
                exited=np.flatnonzero(active & (sign*(pos[:,0]-exit_seam)>=0))
                if len(entered) and len(exited) and exited[0]>=entered[0]:
                    continuous_crossing=bool(active[entered[0]:exited[0]+1].all())
            check('full_ramp_and_landings_forward_motion',worker.get('truth_controller_completed_t_s') is not None and crossed and continuous_crossing and progress is not None and progress>=13.2 and fraction is not None and fraction>=.9,
                  actual_projected_forward_progress_m=progress,nominal_both_seams_crossed_during_active_translation=crossed,
                  continuous_nonzero_translation_between_actual_both_seams=continuous_crossing,
                  positive_rolling_velocity_window_fraction=fraction,rolling_windows=window_count,completion_elapsed_s=worker.get('truth_controller_completed_t_s'))
            any_support=np.logical_or.reduce([masks[name][leg] for name in surfaces for leg in LEGS])
            unsupported, ui=span((~any_support)&active,t)
            final=np.logical_and.reduce(list(masks[end_name].values()))
        else:
            step_name=fixture.get('step_model','teacher_low_step');slab=surface(world,step_name)
            surfaces={step_name:slab};front=slab['center'][0]-slab['half'][0];back=slab['center'][0]+slab['half'][0]
            ymin,ymax=slab['center'][1]-slab['half'][1],slab['center'][1]+slab['half'][1]
            top=slab['center'][2]+slab['half'][2]
            expected_height=.05 if terrain.startswith('step05') else .1 if terrain.startswith('step10') else fixture.get('height_m')
            check('continued_step_fixture',expected_height in (.05,.1) and np.allclose(slab['rotation'],np.eye(3),atol=1e-10) and abs(front-7)<1e-9 and abs(back-13)<1e-9 and abs(2*slab['half'][1]-1.4)<1e-9 and abs(top-expected_height)<1e-9,
                  actual_surface=slab,front_x_m=front,back_x_m=back,top_height_m=top,world_height_ratio_used=False)
            if route[-1,0]<=route[0,0]: raise ValueError('Only prospective forward step ascent is admitted')
            masks,ge=support_masks(native,centers,feet,surfaces,'step')
            check('actual_contact_geometry_available',None if ge['malformed_relevant_contact_present'] else True,**ge)
            support=masks[step_name];conf={};entry={}
            for leg in LEGS:
                c=centers[leg]
                crossings=np.flatnonzero((c[:-1,0]<front)&(c[1:,0]>=front)&(c[1:,0]>c[:-1,0]))+1
                crossings=[int(i) for i in crossings if active[i] and ymin<=c[i,1]<=ymax]
                entered=np.zeros(len(t),bool)
                if crossings:entered[crossings[0]:]=True
                confirm=confirmation(support[leg]&entered&active,t);conf[leg]=confirm
                duration,interval=span(support[leg]&entered&active,t)
                entry[leg]={'first_actual_forward_edge_crossing_world_s':float(t[crossings[0]]) if crossings else None,'first_continuous_true_tread_support_confirmation_world_s':confirm,'active_tread_support_span_s':duration,'interval_s':interval,**feet[leg]}
                check(leg+'_front_entry_and_true_tread_support',bool(crossings) and confirm is not None and duration>=.1-1e-8,**entry[leg])
            all_confirmed=max(conf.values()) if all(v is not None for v in conf.values()) else None
            past=np.flatnonzero(active&(pos[:,0]>=front+.5)&(pos[:,1]>=ymin)&(pos[:,1]<=ymax)&(t>=all_confirmed)) if all_confirmed is not None else []
            cont=np.zeros(len(t),bool);progress=None;cont_duration=None;fraction=None;windows=0;inside=None;continuous_command=False
            if len(past):
                first=int(past[0]);last=int(np.flatnonzero(active)[-1]);cont=(np.arange(len(t))>=first)&(np.arange(len(t))<=last)
                continuous_command=bool(active[cont].all())
                progress=float(pos[last,0]-pos[first,0]);cont_duration=float(t[last]-t[first]);fraction,windows=positive_fraction(world_v[cont,0])
                inside=bool(((pos[cont,0]>=front)&(pos[cont,0]<=back)&(pos[cont,1]>=ymin)&(pos[cont,1]<=ymax)).all())
            any_support=np.logical_or.reduce(list(support.values()));unsupported,ui=span((~any_support)&cont,t)
            check('ascent_then_continued_actual_forward_walking',progress is not None and progress>=1. and cont_duration>=3. and fraction is not None and fraction>=.9 and inside is True and continuous_command and unsupported<=.3,
                  all_four_tread_support_confirmed_world_s=all_confirmed,first_base_edge_plus_half_m_world_s=float(t[past[0]]) if len(past) else None,
                  actual_continued_forward_progress_m=progress,continued_duration_s=cont_duration,positive_rolling_velocity_window_fraction=fraction,
                  rolling_windows=windows,all_body_samples_inside_actual_tread=inside,
                  continuous_nonzero_translation_during_continuation=continuous_command,
                  maximum_no_foot_tread_support_s=unsupported)
            final=np.logical_and.reduce(list(support.values()))
            result['metrics']['feet']=entry
        check('no_extended_all_feet_unsupported',unsupported<=.3,maximum_unsupported_s=unsupported,interval_s=ui,maximum_allowed_s=.3)
        finalspan,finalinterval=span(final&stop,t)
        check('final_simultaneous_four_foot_destination_support',finalspan>=.1-1e-8 and bool(final[-1]) if stop.any() else None,
              fixed_v2_parking_window_s=parking_info.get('window_s') if parking_info else None,
              simultaneous_support_span_s=finalspan,interval_s=finalinterval,all_four_final_support=bool(final[-1]))
        result['metrics'].update(body_world_z_change_diagnostic_m=float(pos[-1,2]-pos[0,2]),body_world_z_change_used_for_acceptance=False,
                                 actual_native_rows=len(native),active_translation_rows=int(active.sum()),fixed_parking_native_rows=int(stop.sum()))
    except (OSError,KeyError,EvidenceMissing) as error:
        check('complete_required_terrain_evidence',None,reason=type(error).__name__+': '+str(error))
    except (ValueError,TypeError,IndexError,ImportError) as error:
        check('valid_required_terrain_evidence',False,reason=type(error).__name__+': '+str(error))
    failed=[k for k,v in result['checks'].items() if v['status']=='failed'];unverified=[k for k,v in result['checks'].items() if v['status']=='unverified']
    result['status']='failed' if failed else 'unverified' if unverified or not result['checks'] else 'passed'
    result['strict_terrain_motion_verified']=result['status']=='passed'
    result['failed_checks'],result['unverified_checks']=failed,unverified
    result['input_source_sha256']={str(path):sha(path) for path in source_paths if path.is_file()}
    result=clean(result)
    if write:
        with out.open('x') as stream:json.dump(result,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();r=evaluate(args.run)
    print(json.dumps({'run':str(args.run.resolve()),'status':r['status'],'strict_terrain_motion_verified':r['strict_terrain_motion_verified'],
                      'failed_checks':r['failed_checks'],'unverified_checks':r['unverified_checks'],'SLAM_navigation_verified':False}))


if __name__=='__main__':main()
