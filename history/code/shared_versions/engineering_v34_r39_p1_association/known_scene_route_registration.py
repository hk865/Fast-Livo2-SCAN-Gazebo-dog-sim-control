"""Freeze provided-map axes after actual matching, without IMU world attitude.

Raw IMU rates/acceleration only attest stationary sensor initialization. World
axes come from the given map and the canonical adapter's frozen T_nav_world.
This remains known-scene assisted local matching, not global uniqueness proof.
"""
import copy,hashlib,json,math,time
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from known_scene_matcher import checked_transform,transform_distance
from known_scene_runtime import proposal_body_effect
from known_scene_adapter import validate_runtime_witness
from path_admission import FrozenFence,digest

SCHEMA='teacher_known_scene_axis_registration/v1'
SOURCE='actual_known_scene_canonical_frame'
ROUTE_REFERENCE='original46 centers with provided-map world axes and initial measured canonical SLAM origin'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def canonical(data):return json.dumps(data,sort_keys=True,separators=(',',':'),allow_nan=False)

def validate_initial_frame(run,scope,frame):
    run=Path(run).resolve()
    for name in ('world.sdf','known_scene_reference.json','navigation_scenario.json','sensor_contract.json'):
        path=run/name
        if scope['references'].get(str(path))!=sha(path):raise ValueError('Unfrozen canonical frame input '+name)
    if (frame.get('schema')!='known_scene_canonical_initialization_frame/v1' or frame.get('run_id')!=run.name
            or type(frame.get('generation')) is not int or frame['generation']!=1
            or frame.get('IMU_world_orientation_used') is not False or frame.get('robot_truth_pose_used') is not False
            or frame.get('frontend_state_reset') is not False or frame.get('initial_commit_navigation_idle_verified') is not True
            or frame.get('reference_sha256')!=sha(run/'known_scene_reference.json')
            or frame.get('world_sha256')!=sha(run/'world.sdf')
            or frame.get('runtime_witness_sha256')!=sha(run/'known_scene_runtime_witness.json')):
        raise ValueError('Actual known-scene initialization identity differs')
    witness=validate_runtime_witness(json.loads((run/'known_scene_runtime_witness.json').read_text()),run.name)
    if not witness['valid']:raise ValueError('Unverified actual producer/stage')
    scenario=json.loads((run/'navigation_scenario.json').read_text())
    nav=np.eye(4);nav[:3,3]=-np.asarray(scenario['slam_origin_in_world'],dtype=float)
    if not np.array_equal(checked_transform(frame['T_navigation_world']),nav):
        raise ValueError('Canonical navigation axes differ from frozen provided world')
    rows=frame.get('actual_startup_results')
    if not isinstance(rows,list) or not 2<=len(rows)<=16:raise ValueError('Missing actual independent startup frames')
    valid=[]
    for row in rows:
        if (row.get('reason') not in ('await_independent_frame','confirmed_shadow_match')
                or row.get('evidence_verified') is not True):continue
        if (row.get('robot_truth_pose_used') is not False or row.get('IMU_world_orientation_used') is not False
                or row.get('frontend_height_used_as_hypothesis') is not False
                or row.get('reference_sha256')!=frame['reference_sha256']
                or row.get('reference_world_sha256')!=frame['world_sha256']
                or row.get('provenance',{}).get('witness_sha256')!=witness['witness_sha256']
                or any(row.get('provenance',{}).get(k)is not True for k in ('point_time_verified',
                    'post_lio_association_verified','source_hashes_verified','integer_stamp_join','source_membership_verified'))
                or not isinstance(row.get('best'),dict) or row['best'].get('rejections')!=[]
                or row['best'] not in row.get('candidates',[])
                or not np.allclose(checked_transform(row['best']['T_world_body']),checked_transform(row['T_world_body']),rtol=0,atol=1e-7)):
            raise ValueError('Startup matching source/quality not bound')
        proposal_body_effect(row,row['C_world_odom'])
        valid.append(row)
    if len(valid)<2:raise ValueError('Two verified independent point-cloud matches required')
    first,last=valid[-2:]
    if (last.get('accepted') is not True or last.get('confirmations',0)<2
            or last['source_ns']!=frame['source_ns']
            or not 150_000_000<=last['source_ns']-first['source_ns']<=10_000_000_000
            or transform_distance(checked_transform(first['C_world_odom']),checked_transform(last['C_world_odom']))[0]>.15
            or transform_distance(checked_transform(first['C_world_odom']),checked_transform(last['C_world_odom']))[1]>.08
            or not np.array_equal(checked_transform(frame['C_world_odom']),checked_transform(last['C_world_odom']))):
        raise ValueError('Actual independent confirmation/canonical commit differs')
    return dict(generation=1,T_navigation_world=nav.tolist(),C_world_odom=frame['C_world_odom'],
        confirmed_source_ns=[first['source_ns'],last['source_ns']],reference_sha256=frame['reference_sha256'],
        world_sha256=frame['world_sha256'],global_search_complete=False,calibrated_covariance=False,
        IMU_world_orientation_used=False,robot_truth_pose_used=False)

class KnownSceneAxisRegistration:
    def __init__(self,run,scope,writer):
        self.run=Path(run).resolve();self.scope=scope;self.writer=writer
        p=self.run/'sensor_contract.json'
        if scope['references'].get(str(p))!=sha(p):raise ValueError('Unfrozen mechanical IMU mount')
        self.imu_config=json.loads(p.read_text())['imu'];self.sensor_contract_sha256=sha(p)
        self.mount_rotation=Rotation.from_euler('xyz',self.imu_config['body_rpy']).as_matrix()
        self.imu_rows=deque(maxlen=512);self.pairs=deque(maxlen=32);self.all_pairs=[]
        self.last_imu_stamp=None;self.last_pair_stamp=None;self.receipt=None;self.frame=None;self.frame_proof=None
        self.reset_count=0;self.wait_reason='waiting_stationary_known_scene_canonical_frame'
    def reset(self,reason):
        if self.receipt is not None:return
        if self.pairs:self.reset_count+=1
        self.pairs.clear();self.last_pair_stamp=None;self.wait_reason=reason
    def observe_imu(self,row):
        stamp=row['stamp_ns'];clock=row['callback_ros_clock_ns']
        gyro=np.asarray(row['angular_velocity_sensor'],dtype=float);acc=np.asarray(row['linear_acceleration_sensor'],dtype=float)
        if (type(stamp)is not int or type(clock)is not int or not 0<=stamp<=clock or clock-stamp>=300_000_000
                or self.last_imu_stamp is not None and stamp<=self.last_imu_stamp
                or row['frame_id']!=self.imu_config['frame'] or gyro.shape!=(3,) or acc.shape!=(3,)
                or not np.isfinite(gyro).all() or not np.isfinite(acc).all()):return False
        # Attitude and its declared world reference are neither read nor used.
        source={k:copy.deepcopy(row[k]) for k in ('stamp_ns','callback_ros_clock_ns','received_wall_ns',
            'frame_id','angular_velocity_sensor','linear_acceleration_sensor','source_topic')}
        source['angular_velocity_body']=(self.mount_rotation@gyro).tolist()
        self.imu_rows.append(source);self.last_imu_stamp=stamp;return True
    def observe_pose(self,row,healthy,wall_ns):
        if self.receipt is not None:return False
        stamp=row['stamp_ns'];clock=row['callback_ros_clock_ns'];velocity=np.asarray(row['body_velocity'],dtype=float)
        if (not healthy or type(stamp)is not int or type(clock)is not int or not 0<=stamp<=clock
                or clock-stamp>=300_000_000 or velocity.shape!=(3,) or not np.isfinite(velocity).all()
                or np.linalg.norm(velocity)>.03 or not 0<=wall_ns-row['received_wall_ns']<300_000_000):
            self.reset('initial_canonical_SLAM_not_fresh_stationary');return False
        imu=next((x for x in reversed(self.imu_rows) if 0<=stamp-x['stamp_ns']<=20_000_000),None)
        if (imu is None or not 0<=wall_ns-imu['received_wall_ns']<300_000_000
                or np.linalg.norm(imu['angular_velocity_body'])>.05):
            self.reset('missing_causal_stationary_raw_IMU_rates');return False
        if self.last_pair_stamp is not None:
            if stamp<=self.last_pair_stamp:return False
            if stamp-self.last_pair_stamp>300_000_000:self.reset('initial_source_gap')
        pair=dict(sequence=len(self.all_pairs)+1,slam_stamp_ns=stamp,imu_stamp_ns=imu['stamp_ns'],
            pair_gap_ns=stamp-imu['stamp_ns'],slam_source=copy.deepcopy(row),IMU_rate_acceleration_source=copy.deepcopy(imu),
            IMU_world_orientation_used=False,navigation_ground_truth_used=False)
        self.pairs.append(pair);self.all_pairs.append(pair);self.last_pair_stamp=stamp
        self.writer.append(self.run/'nav_registration_pairs.jsonl',pair);return True
    def try_freeze(self,clock_ns,wall_ns):
        if self.receipt is not None:return self.receipt
        if len(self.pairs)<10 or self.pairs[-1]['slam_stamp_ns']-self.pairs[0]['slam_stamp_ns']<800_000_000:return None
        last=self.pairs[-1]
        if (not 0<=clock_ns-last['slam_stamp_ns']<300_000_000
                or not 0<=wall_ns-last['slam_source']['received_wall_ns']<300_000_000
                or not 0<=wall_ns-last['IMU_rate_acceleration_source']['received_wall_ns']<300_000_000):
            self.reset('canonical_initialization_source_expired');return None
        path=self.run/'known_scene_initialization_frame.json'
        if not path.exists():return None
        if self.frame is None:
            self.frame=json.loads(path.read_text());self.frame_proof=validate_initial_frame(self.run,self.scope,self.frame)
            self.frame_sha256=sha(path)
        if sha(path)!=self.frame_sha256:raise ValueError('Frozen canonical initialization changed')
        status=json.loads((self.run/'known_scene_status.json').read_text())
        if (status.get('run_id')!=self.run.name or status.get('initialized')is not True
                or status.get('generation')!=1 or status.get('correction_transition')is not False
                or not 0<=wall_ns/1e9-float(status.get('monotonic_wall',0))<.3
                or type(status.get('last_match_source_ns'))is not int
                or not 0<=clock_ns-status['last_match_source_ns']<=3_000_000_000):return None
        heading=dict(source=SOURCE,yaw_camera_init_from_world=0.,rotation_camera_init_from_world=np.eye(3).tolist(),
            ground_truth_used=False,IMU_world_orientation_used=False,
            purpose='provided-map axes of actual matched canonical navigation frame; raw IMU rates only for stop/init')
        raw=''.join(json.dumps(x,allow_nan=False,ensure_ascii=False)+'\n'for x in self.all_pairs).encode()
        self.receipt=dict(schema=SCHEMA,status='frozen',heading_receipt=heading,
            canonical_initialization_file=path.name,canonical_initialization_sha256=self.frame_sha256,
            canonical_frame_proof=self.frame_proof,stationary_source_records=list(self.pairs),
            paired_sources_file='nav_registration_pairs.jsonl',paired_sources_expected_file_sha256=hashlib.sha256(raw).hexdigest(),
            paired_sources_expected_records=len(self.all_pairs),sensor_contract_sha256=self.sensor_contract_sha256,
            frozen_ros_clock_ns=clock_ns,frozen_monotonic_wall_ns=wall_ns,
            stopped_initialization={'body_velocity_norm_max_mps':.03,'IMU_body_gyro_norm_max_radps':.05},
            pairing_causal=True,maximum_pair_gap_ns=20_000_000,minimum_samples=10,minimum_span_ns=800_000_000,
            ground_truth_navigation_used=False,IMU_world_orientation_used=False,
            meaning='axes from frozen given-map world→navigation transform, after actual same-source independent matching')
        return self.receipt

class KnownSceneFrozenFence(FrozenFence):
    """New provenance validation; original curve certificate/math stay shared."""
    def load(self):
        if self.profile.get('known_scene_localization_contract',{}).get('enabled')is not True:return super().load()
        ap=self.run/'navigation_anchor.json';rp=self.run/'navigation_scene_axis_registration.json'
        if ap.stat().st_size>2_000_000 or rp.stat().st_size>2_000_000:raise ValueError('known_scene_anchor_size_budget')
        raw=ap.read_bytes();regraw=rp.read_bytes();ah=hashlib.sha256(raw).hexdigest();rh=hashlib.sha256(regraw).hexdigest()
        a=json.loads(raw);r=json.loads(regraw)
        if (self.anchor_sha is not None and ah!=self.anchor_sha or self.registration_sha is not None and rh!=self.registration_sha
                or a.get('schema')!=1 or a.get('run_dir')!=str(self.run) or a.get('frame_id')!='camera_init'
                or a.get('source')!='/demo/slam/body_odom' or a.get('frozen_once')is not True
                or a.get('ground_truth_navigation_used')is not False or a.get('original_region_count')!=46
                or a.get('registered_route_reference')!=ROUTE_REFERENCE
                or a.get('scene_axis_registration_file')!=rp.name or a.get('scene_axis_registration_file_sha256')!=rh
                or a.get('scene_axis_registration')!=r or r.get('schema')!=SCHEMA or r.get('status')!='frozen'
                or r.get('ground_truth_navigation_used')is not False or r.get('IMU_world_orientation_used')is not False
                or r['heading_receipt'].get('source')!=SOURCE or r['heading_receipt'].get('IMU_world_orientation_used')is not False
                or a.get('yaw')!=0. or r['heading_receipt'].get('yaw_camera_init_from_world')!=0.):
            raise ValueError('invalid_frozen_provided_map_axis_anchor')
        fp=self.run/r['canonical_initialization_file']
        if fp.resolve().parent!=self.run or fp.name!='known_scene_initialization_frame.json' or sha(fp)!=r['canonical_initialization_sha256']:
            raise ValueError('unbound_canonical_initialization')
        frame=json.loads(fp.read_text());proof=r['canonical_frame_proof'];nav=checked_transform(frame['T_navigation_world'])
        if (frame.get('run_id')!=self.run.name or frame.get('generation')!=1
                or frame.get('IMU_world_orientation_used')is not False or frame.get('robot_truth_pose_used')is not False
                or frame.get('C_world_odom')!=proof.get('C_world_odom')
                or frame.get('T_navigation_world')!=proof.get('T_navigation_world')
                or not np.array_equal(nav[:3,:3],np.eye(3))):raise ValueError('canonical_world_axes_changed')
        origin=np.asarray(a['origin'],dtype=float);initial=a.get('initial_actual_slam_source')or{}
        if (origin.shape!=(3,) or not np.isfinite(origin).all()
                or initial.get('position')!=a['origin'] or initial.get('stamp_ns')!=a.get('pose_stamp_ns')
                or initial.get('source_topic')!='/demo/slam/body_odom' or initial.get('frame_id')!='camera_init'
                or initial.get('child_frame_id')!='demo_slam_body' or initial.get('navigation_ground_truth_used')is not False):
            raise ValueError('invalid_initial_canonical_SLAM_origin')
        scenario=self.profile['original_scenario'];world=[p for n in ('exploration','return_origin','navigation_f1_f3')for p in scenario[n]]
        expected=[origin.tolist()]+[(origin+np.asarray(p)).tolist()for p in world]
        if len(world)!=46 or a.get('registered_route_camera_init_xyz')!=expected or a.get('registered_route_points_sha256')!=digest(expected):
            raise ValueError('original46_given_axis_geometry_changed')
        self.anchor_sha=ah;self.registration_sha=rh
        return expected,self.profile['registered_route_fence'],dict(anchor_sha256=ah,registration_sha256=rh,
            route_points_sha256=digest(expected),frame_id='camera_init',navigation_ground_truth_used=False,
            IMU_world_orientation_used=False,registration_source=SOURCE)
