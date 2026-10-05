"""Prospective analytic curve profile constructors; no simulation launches.

No radius here is declared feasible. Final circle registration requires the
same-speed/same-sign actual plant evidence and a 1.5x geometric radius margin.
"""
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

try:
    from .core import StaticPath, parameter_hash
except ImportError:
    # Explicit adjacent source avoids accidentally importing the old v3 module
    # named "core" when a caller already has that mode loaded in sys.modules.
    spec=importlib.util.spec_from_file_location('go2_analytic_curvature_core',Path(__file__).with_name('core.py'))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    StaticPath,parameter_hash=module.StaticPath,module.parameter_hash


BASE_COM_OFFSET = [0.05523092034548942, -0.001869001919385796, 0.006095299184261034]
MODEL_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
APPROVED_PLANT_ANALYZER_SHA = 'b2663570a8ae9dcc53b26638a81e9b86389087e9cd488cbe22aa4e48a70ffc07'
APPROVED_PLANT_PROTOCOL_SHA = '2ad596f0c3815c8a12ba7f919b24f8087115ea9a510aa4fa7a8301300f459847'
REQUIRED_PLANT_CHECKS = (
    'frozen_sources_and_protocol','prospective_signed_plan_grid','complete_owned_runtime','explicit_unobstructed_plant_fixture',
    'exclusive_native_PD_DC_motor_contract','original_worker_native_state_phase','complete_monotonic_native_and_50Hz_policy',
    'frozen_CPU_Teacher_continuously_active','original_velocity_schedule_and_slew',
    'no_runtime_or_physical_fault','body_contact_clearance_and_attitude_safety',
    'all_native_torque_and_joint_speed_limits','actual_PD_speed_torque_curve_recomputed',
    'full_continuous_cruise_coverage','continuous_forward_and_yaw_response',
    'never_in_place_or_intermittent_stop_in_cruise','valid_stable_geometric_circle_arc',
    'fixed_5s_Teacher_zero_velocity_parking')
REQUIRED_PLANT_INPUTS = ('actuator.jsonl','telemetry.jsonl','plant_commands.jsonl',
    'observations_actions.npz','runtime_manifest.json','policy_manifest.json','worker_result.json',
    'source_manifest.json','fixture_manifest.json','curvature_plan.json','protocol.json')


def _sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def _validate_plant_source(item,speed,sign):
    """Validate the actual selected passed case and its original input hashes.

    The caller supplies the sweep minimum/boundary selection. This constructor
    does not infer global minimality or repeatability from a single plant run.
    """
    required=('status','desired_speed_mps','direction_sign','minimum_of_passed_tested_actual_radii_m',
              'source_path','source_sha256')
    if any(k not in item for k in required) or item['status']!='passed':
        raise ValueError('Passed actual plant receipt fields required')
    if item['desired_speed_mps']!=speed or item['direction_sign']!=sign:
        raise ValueError('Plant evidence must match speed and direction')
    path=Path(item['source_path'])
    if not path.is_absolute() or _sha(path)!=item['source_sha256']:
        raise ValueError('Actual plant source SHA mismatch')
    source=json.loads(path.read_text())
    if source['schema']!='teacher_continuous_turn_plant_independent/v1' or source['status']!='passed':
        raise ValueError('Actual plant source is not an independent PASS')
    if not set(REQUIRED_PLANT_CHECKS)<=set(source['checks']) or not all(c['status']=='passed' for c in source['checks'].values()):
        raise ValueError('Every original plant check must pass')
    if source['analyzer_sha256']!=APPROVED_PLANT_ANALYZER_SHA:
        raise ValueError('Plant analyzer is not the approved frozen revision')
    if not set(REQUIRED_PLANT_INPUTS)<=set(source['input_hashes']):
        raise ValueError('Missing mandatory original plant inputs')
    run=Path(source['run']).resolve()
    if path.parent.resolve()!=run:
        raise ValueError('Plant receipt run identity mismatch')
    for name,expected in source['input_hashes'].items():
        input_path=run/'sources/curvature/protocol.json' if name=='protocol.json' else run/name
        if _sha(input_path)!=expected:raise ValueError('Original plant input SHA mismatch: '+name)
    if source['input_hashes']['protocol.json']!=APPROVED_PLANT_PROTOCOL_SHA:
        raise ValueError('Plant protocol is not the approved frozen revision')
    manifest=json.loads((run/'source_manifest.json').read_text())
    for name,expected in [('curvature/evaluate_radius.py',APPROVED_PLANT_ANALYZER_SHA),
                          ('curvature/protocol.json',APPROVED_PLANT_PROTOCOL_SHA)]:
        if name not in manifest or manifest[name]['sha256']!=expected:
            raise ValueError('Missing approved archived plant source: '+name)
    for item_source in manifest.values():
        if _sha(item_source['path'])!=item_source['sha256']:
            raise ValueError('Archived plant source SHA mismatch')
    plan=json.loads((run/'curvature_plan.json').read_text())
    if plan['speed_mps']!=speed or (1 if plan['yaw_rate_radps']>0 else -1)!=sign:
        raise ValueError('Actual plant plan differs from evidence speed/sign')
    if plan['model_sha256']!=MODEL_SHA:
        raise ValueError('Plant model differs from frozen Teacher')
    if plan['protocol_sha256']!=APPROVED_PLANT_PROTOCOL_SHA:
        raise ValueError('Plant plan protocol is not approved')
    radius=float(item['minimum_of_passed_tested_actual_radii_m'])
    if radius<=0 or not math.isclose(radius,source['metrics']['actual_fitted_radius_m'],rel_tol=1e-10,abs_tol=1e-10):
        raise ValueError('Evidence radius must be fitted actual radius of selected passed case')
    return radius


def _profile(parameters, speed, hz, duration_s, scenario, evidence):
    path=StaticPath(parameters)
    if not 0<speed<=.8 or hz not in (10,25,50):
        raise ValueError('Curve prospective speed<=.8 and feedback10/25/50')
    count=math.ceil(path.length/.03)+1
    route=path.position(np.linspace(0,path.length,count)).tolist()
    return {'schema':'truth_teacher_static_curve_profile/v1','scenario':scenario,'terrain':'flat',
            'spawn':[parameters['origin_world_xyz'][0],parameters['origin_world_xyz'][1],
                     parameters['origin_world_xyz'][2]+.08,parameters['heading0_rad']],
            'route_world_xyz':route,
            'route_world_xyz_role':'visualization/legacy spatial diagnostic only; exact analytic path drives controller',
            'path':parameters,'path_parameters_sha256':parameter_hash(parameters),
            'path_length_m':path.length,'maximum_abs_curvature_1pm':path.maximum_abs_curvature,
            'desired_speed':float(speed),'feedback_hz':int(hz),'duration_s':float(duration_s),
            'post_completion_s':7.,'command_limits':[.8,.35,.8],'gains':{},
            'design':'curvature_cascade_pi','controller_revision':'new isolated analytic curvature feedforward v1',
            'base_com_offset':BASE_COM_OFFSET.copy(),'uses_truth_for_control':True,
            'counts_as_SLAM_navigation':False,'feedback_source':'native64 truth; world_time-.005',
            'curvature_feedforward':True,'nominal_yaw_feedforward_limit_fraction':.8,
            'continuous_tracking_has_pre_turn_gate':False,'path_time_parameterized':False,
            'model_training_changed':False,'real_robot_verified':False,
            'feasibility':evidence,'navigation_source_scope':'truth curve control calibration; not SLAM fusion'}


def circle_profile(radius_m, speed=.3, hz=25, direction_sign=1,
                   origin_world_xyz=(0.,0.,.32), heading0_rad=0.,
                   entry_straight_m=2.,exit_straight_m=2.,duration_s=150.,
                   minimum_radius_evidence=None):
    parameters={'kind':'circle','origin_world_xyz':list(origin_world_xyz),
                'heading0_rad':float(heading0_rad),'entry_straight_m':float(entry_straight_m),
                'exit_straight_m':float(exit_straight_m),'direction_sign':int(direction_sign),
                'radius_m':float(radius_m),'turn_angle_rad':2*math.pi}
    evidence={'status':'pending_actual_minimum_radius','circle_radius_is_tested_feasible':False,
              'radius_margin_factor':1.5,'claim':'No final feasible curvature selected before actual plant data.'}
    if minimum_radius_evidence is not None:
        source=json.loads(json.dumps(minimum_radius_evidence,allow_nan=False))
        rmin=_validate_plant_source(source,speed,direction_sign)
        if radius_m<1.5*rmin:
            raise ValueError('Circle requires >=1.5x minimum passed tested actual radius')
        evidence={'status':'prospective_margin_from_actual_plant','plant_source':source,
                  'radius_margin_factor':1.5,'circle_radius_is_tested_feasible':False,
                  'claim':'Margin-based planned geometry; closed-loop tracking remains unverified until actual run.'}
    return _profile(parameters,speed,hz,duration_s,'flat_continuous_circle',evidence)


def s_curve_profile(curve_length_m,heading_amplitude_rad,speed=.3,hz=25,direction_sign=1,
                    origin_world_xyz=(0.,0.,.32),heading0_rad=0.,entry_straight_m=2.,
                    exit_straight_m=2.,duration_s=150.,minimum_radius_evidence=None):
    parameters={'kind':'s_curve','origin_world_xyz':list(origin_world_xyz),
                'heading0_rad':float(heading0_rad),'entry_straight_m':float(entry_straight_m),
                'exit_straight_m':float(exit_straight_m),'direction_sign':int(direction_sign),
                'curve_length_m':float(curve_length_m),'heading_amplitude_rad':float(heading_amplitude_rad)}
    path=StaticPath(parameters)
    evidence={'status':'pending_actual_circle_tracking_and_radius','geometry_minimum_radius_m':1/path.maximum_abs_curvature,
              'claim':'Both signed-curvature plant and circle tracking must be checked before choosing final S amplitude/length.'}
    if minimum_radius_evidence is not None:
        # S has both signs. Root must supply the worse of same-speed actual
        # left/right passed plant radii, not a single favorable-direction case.
        left=minimum_radius_evidence['left'];right=minimum_radius_evidence['right']
        minimums=[_validate_plant_source(item,speed,sign) for sign,item in [(1,left),(-1,right)]]
        maximum=max(minimums)
        if 1/path.maximum_abs_curvature<1.5*maximum:
            raise ValueError('S geometry requires >=1.5x worse left/right actual tested radius')
        evidence={'status':'prospective_margin_from_both_actual_signs','plant_sources':minimum_radius_evidence,
                  'geometry_minimum_radius_m':1/path.maximum_abs_curvature,'margin_factor':1.5,
                  'minimum_of_passed_tested_actual_radii_worse_direction_m':maximum,
                  'claim':'Planned varying-curvature geometry; actual S tracking still unverified.'}
    return _profile(parameters,speed,hz,duration_s,'flat_continuous_s_curve',evidence)
