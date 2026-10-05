#!/usr/bin/env python3
"""Read-only sensor/Teacher observation shadow audit; never switches a policy.

Reference physics comes from recorded policy telemetry, only for comparison.
Missing IMU, SLAM, joints or clouds remain missing and UNVERIFIED.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
TOPICS = {'imu': '/livox/imu', 'slam': '/demo/slam/body_odom',
          'joints': '/demo/control/measured_joint_states', 'cloud': '/cloud_registered_full',
          'clock':'/clock','camera':'/demo/camera','raw_lidar':'/demo/teacher/raw_lidar','overview':'/demo/teacher/overview'}


def load(path):
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def digest(path):
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def clean(value):
    import numpy as np
    if isinstance(value, bool):
        return value
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def rotation(q):
    """World-from-local matrix, ROS xyzw; reject absent/invalid quaternions."""
    import numpy as np
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .98 <= float(q @ q) <= 1.02:
        raise ValueError('Invalid or unavailable orientation')
    x, y, z, w = q / np.linalg.norm(q)
    return np.asarray([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def causal(history, stamp, max_age):
    """Do not select a future sensor sample to improve retrospective errors."""
    stamp_ns=round(stamp*1e9);max_age_ns=round(max_age*1e9)
    eligible = [sample for sample in history if 0 <= stamp_ns-sample['stamp_ns'] <= max_age_ns]
    return max(eligible, key=lambda sample: sample['stamp_ns']) if eligible else None


def observed_scan(points, position, body_rotation, grid, radius=.075):
    """Highest OBSERVED point in each column, not a certified +20 m ray hit."""
    import numpy as np
    yaw = math.atan2(body_rotation[1, 0], body_rotation[0, 0])
    cy, sy = math.cos(yaw), math.sin(yaw)
    yaw_rotation = np.asarray([[cy, -sy], [sy, cy]])
    centers = grid @ yaw_rotation.T + position[:2]
    local_xy = (points[:, :2] - position[:2]) @ yaw_rotation
    points = points[(abs(local_xy[:, 0]) <= .8 + radius) & (abs(local_xy[:, 1]) <= .5 + radius)]
    hits = np.full(len(grid), np.nan)
    for index, center in enumerate(centers):
        inside = np.sum((points[:, :2] - center) ** 2, axis=1) <= radius ** 2
        if inside.any():
            hits[index] = points[inside, 2].max()
    values = np.clip(position[2] - hits - .5, -1., 1.)
    return {'values': values, 'valid_mask': np.isfinite(hits), 'observed_hit_z': hits,
            'covered_columns': int(np.isfinite(hits).sum()), 'grid_points': len(grid),
            'observed_overhead_columns': int(np.sum(hits > position[2])),
            'semantics': 'highest observed local-cloud column; overhead occlusion and unobserved surfaces are unresolved',
            'eligible_for_policy_replacement': False}


def phase_diagnostic(run,output):
    """Explain native PreUpdate vs ROS PostUpdate using actual archived samples.

    The original same-clock live comparison is preserved. This separate audit
    only pairs an earlier sample; it never retimes data or changes an actor.
    """
    import numpy as np
    source=run/'sensor_shadow/sensor_samples.jsonl'
    telemetry=run/'telemetry.jsonl';asset=load(run/'asset_manifest.json')
    dt=float(asset['physics_step_s']);dt_ns=round(dt*1e9)
    samples={'imu':{},'joints':{}}
    for line in source.open():
        row=json.loads(line)
        if row['source']in samples:samples[row['source']][row['stamp_ns']]=row
    offsets=[0,-dt_ns];metrics={};references=[json.loads(line)for line in telemetry.open()]
    for offset in offsets:
        errors={'q':[],'qd':[],'gyro_body':[],'gravity_body':[]};paired=[]
        for ref in references:
            clock_ns=round(ref['world_sim_time']*1e9);stamp_ns=clock_ns+offset
            imu=samples['imu'].get(stamp_ns);joints=samples['joints'].get(stamp_ns)
            if joints:
                errors['q'].append(np.asarray(joints['q'])-ref['q']);errors['qd'].append(np.asarray(joints['qd'])-ref['qd'])
            if imu:
                errors['gyro_body'].append(np.asarray(imu['gyro_body'])-ref['body_ang_vel'])
                w,x,y,z=ref['quaternion_wxyz'];gravity=rotation([x,y,z,w]).T@np.asarray([0.,0.,-1.])
                errors['gravity_body'].append(np.asarray(imu['gravity_body'])-gravity)
            if imu and joints:paired.append({'reference_world_stamp_ns':clock_ns,'sensor_stamp_ns':stamp_ns,'imu_frame':imu['frame'],'joints_frame':joints['frame']})
        metrics[str(offset)]={'offset_s':offset*1e-9,'paired_rows':len(paired),'future_samples':0,
             'metrics':{key:{'samples':len(values),'raw_rmse':np.sqrt(np.mean(np.asarray(values)**2,axis=0)).tolist(),
                             'raw_abs_error_max':np.max(np.abs(values),axis=0).tolist()}for key,values in errors.items()if values}}
        with(output/f'pairs_offset_{offset}_ns.jsonl').open('w')as stream:
            for pair in paired:stream.write(json.dumps(pair)+'\n')
    summary={'status':'phase_diagnostic_only_unverified_sensor_policy','policy_changed':False,'navigation_validation':'unverified',
        'reference_run':str(run),'references':len(references),'physics_step_s':dt,
        'source_semantics':'Teacher native PreUpdate(world_t) reads prior Physics results; JointStatePublisher and IMU PostUpdate use world_t header. Comparing prior-step ROS stamp world_t-dt is causal.',
        'clock_offsets':metrics,'original_live_comparison_preserved':str(run/'sensor_shadow/summary.json'),
        'missing_slam_and_cloud':'unverified; raw sensor-frame LiDAR cannot provide SLAM body velocity or registered local height scan',
        'input_hashes':{str(p):digest(p)for p in [source,telemetry,run/'asset_manifest.json',run/'actuator.jsonl']},'script_sha256':digest(Path(__file__))}
    (output/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False)+'\n')
    print(json.dumps(clean(summary),indent=2,allow_nan=False))


def make_node(rclpy, args, run, output, contract):
    import numpy as np
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState, PointCloud2, Image
    from rosgraph_msgs.msg import Clock as ClockMessage
    from nav_msgs.msg import Odometry
    from sensor_msgs_py import point_cloud2

    sys.path.insert(0, str(ROOT / 'policy'))
    from observation import SCAN_GRID_XY, TerrainHeightMap
    scenario = load(args.scenario)
    sensor_contract=load(run/'sensor_contract.json')
    config = sensor_contract.get('imu',{}).get('orientation_reference') or scenario.get('sensors', {}).get('imu', {}).get('orientation_reference', {})
    try:
        if sensor_contract and (config.get('localization')!='CUSTOM' or config.get('custom_parent_frame')!='world'):
            raise ValueError('Only explicit CUSTOM/world IMU orientation is established')
        body_imu = rotation(config['body_imu_quaternion'])
        world_reference = rotation(config['world_quaternion'])
    except (KeyError, ValueError):
        body_imu = world_reference = None
    asset = load(run / 'asset_manifest.json')
    try:
        com_offset = np.asarray([float(v) for v in asset['base_inertial_pose'].split()[:3]])
        if com_offset.shape != (3,) or not np.isfinite(com_offset).all():
            com_offset = None
    except (KeyError, ValueError, AttributeError):
        com_offset = None
    try:
        terrain = TerrainHeightMap.from_sdf(run / 'world.sdf')
    except (OSError, ValueError):
        terrain = None

    class Shadow(Node):
        def __init__(self):
            super().__init__('teacher_sensor_shadow_audit',enable_rosout=False,start_parameter_services=False)
            self.histories = {key: deque(maxlen=1200 if key != 'cloud' else 24) for key in TOPICS}
            self.counts, self.invalid = Counter(), Counter()
            self.periods = {key: {'sim': [], 'wall': []} for key in TOPICS}
            self.errors, self.raw_errors, self.gaps = {}, {}, {key: [] for key in TOPICS}
            self.frames={key:Counter()for key in TOPICS}
            self.matched = Counter()
            self.references = self.reference_failures = 0
            self.last_reference = -math.inf
            self.previous_reference_time = None
            self.previous_raw_action = None
            self.started = time.monotonic()
            self.pending = deque()
            self.partial = b''
            self.reference_file = run / 'telemetry.jsonl'
            self.reference_offset=0
            self.records = (output / 'shadow.jsonl').open('w', buffering=1)
            self.sensor_records = (output / 'sensor_samples.jsonl').open('w', buffering=1)
            self.graph_records=(output/'publisher_graph.jsonl').open('w',buffering=1)
            self.last_graph_wall=-math.inf;self.last_valid_graph=None
            self.subscribers = [self.create_subscription(Imu, TOPICS['imu'], self.imu, qos_profile_sensor_data),
                self.create_subscription(Odometry, TOPICS['slam'], self.slam, qos_profile_sensor_data),
                self.create_subscription(JointState, TOPICS['joints'], self.joints, qos_profile_sensor_data),
                self.create_subscription(PointCloud2, TOPICS['cloud'], self.cloud, qos_profile_sensor_data),
                self.create_subscription(ClockMessage,TOPICS['clock'],self.clock_message,qos_profile_sensor_data),
                self.create_subscription(Image,TOPICS['camera'],self.camera,qos_profile_sensor_data),
                self.create_subscription(PointCloud2,TOPICS['raw_lidar'],self.raw_lidar,qos_profile_sensor_data),
                self.create_subscription(Image,TOPICS['overview'],self.overview,qos_profile_sensor_data)]
            if any(sub.topic_name != topic for sub, topic in zip(self.subscribers, TOPICS.values())):
                raise RuntimeError('Shadow input remapping is forbidden; diagnostic truth is read only from recorded policy telemetry')
            self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.tick()
            (output/'ready').write_text(json.dumps({'pid':os.getpid(),'subscribed':TOPICS,
                'application_publishers':self.application_publishers(),'readonly':True})+'\n')

        def application_publishers(self):
            return [p.topic_name for p in self.publishers if p.topic_name not in ['/rosout','/parameter_events']]

        def snapshot_graph(self,source):
            before=time.monotonic();wall_ns=time.time_ns();graph={}
            for key,topic in TOPICS.items():
                try:graph[key]=[{'node_name':p.node_name,'node_namespace':p.node_namespace,'type':p.topic_type}for p in self.get_publishers_info_by_topic(topic)]
                except Exception as error:graph[key]={'unavailable':str(error)}
            snapshot={'source':source,'capture_started_monotonic_wall':before,'capture_finished_monotonic_wall':time.monotonic(),
                      'capture_wall_unix_ns':wall_ns,'last_reference_world_sim_time':self.last_reference if math.isfinite(self.last_reference)else None,
                      'publishers':graph,'valid':all(isinstance(v,list)for v in graph.values())}
            self.graph_records.write(json.dumps(snapshot,allow_nan=False)+'\n');self.last_graph_wall=before
            if snapshot['valid']:self.last_valid_graph=snapshot

        def add(self, key, message, **values):
            self.add_sample(key,message.header.stamp.sec*1000000000+message.header.stamp.nanosec,message.header.frame_id,**values)

        def add_sample(self,key,stamp_ns,frame,**values):
            stamp=stamp_ns*1e-9
            if stamp_ns < 0:
                self.invalid[key] += 1
                return
            now = time.monotonic()
            if self.histories[key]:
                previous = self.histories[key][-1]
                if stamp_ns <= previous['stamp_ns']:
                    self.invalid[key] += 1
                    return
                self.periods[key]['sim'].append(stamp - previous['t'])
                self.periods[key]['wall'].append(now - previous['received_monotonic_wall'])
            self.counts[key] += 1
            self.frames[key][frame]+=1
            sample={'t':stamp,'stamp_ns':stamp_ns,'received_monotonic_wall':now,'frame':frame,**values}
            self.histories[key].append(sample)
            # Actual sensor values remain reviewable independently of truth diagnostics.
            archived={k:v for k,v in sample.items()if k!='points'}
            self.sensor_records.write(json.dumps(clean({'source':key,'topic':TOPICS[key],**archived}),allow_nan=False)+'\n')
            required=('imu','joints','clock','raw_lidar','camera','overview')
            if not(output/'actual_sensor_ready.json').exists()and all(self.counts[k]>0 for k in required):
                (output/'actual_sensor_ready.json').write_text(json.dumps({'source':'actual received ROS sensor callbacks','counts':dict(self.counts),
                    'first_stamps_ns':{k:self.histories[k][0]['stamp_ns']for k in required},'frames':{k:dict(self.frames[k])for k in required},'readonly':True})+'\n')

        def clock_message(self,message):
            self.add_sample('clock',message.clock.sec*1000000000+message.clock.nanosec,'world_simulation_clock')

        def camera(self,message):
            self.add('camera',message,width=message.width,height=message.height,encoding=message.encoding,
                     data_bytes=len(message.data),data_sha256=hashlib.sha256(message.data).hexdigest())

        def overview(self,message):
            self.add('overview',message,width=message.width,height=message.height,encoding=message.encoding,
                     data_bytes=len(message.data),data_sha256=hashlib.sha256(message.data).hexdigest())

        def raw_lidar(self,message):
            try:
                points=point_cloud2.read_points_numpy(message,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
                points=points[np.isfinite(points).all(axis=1)]
            except (ValueError,TypeError,AssertionError):
                self.invalid['raw_lidar']+=1
                return
            if not self.counts['raw_lidar']:
                np.savez_compressed(output/'raw_lidar_first.npz',xyz=points,stamp_ns=message.header.stamp.sec*1000000000+message.header.stamp.nanosec,
                                    frame=message.header.frame_id)
            self.add('raw_lidar',message,point_count=len(points),width=message.width,height=message.height,
                     xyz_min=points.min(axis=0)if len(points)else None,xyz_max=points.max(axis=0)if len(points)else None,
                     data_bytes=len(message.data),data_sha256=hashlib.sha256(message.data).hexdigest(),
                     scope='actual raw sensor-frame LiDAR; no SLAM registration or policy height substitution')

        def imu(self, message):
            gyro = np.asarray([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
            acceleration = np.asarray([message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z])
            if not np.isfinite(gyro).all() or not np.isfinite(acceleration).all() or body_imu is None:
                self.invalid['imu'] += 1
                return
            q = message.orientation
            gravity = None
            if message.orientation_covariance[0] != -1:
                try:
                    world_body = world_reference @ rotation([q.x, q.y, q.z, q.w]) @ body_imu.T
                    gravity = world_body.T @ np.asarray([0., 0., -1.])
                except ValueError:
                    pass
            self.add('imu', message, gyro_body=body_imu @ gyro, gravity_body=gravity,
                     gyro_imu=gyro,acceleration_imu=acceleration,quaternion_xyzw=[q.x,q.y,q.z,q.w],
                     angular_velocity_covariance=list(message.angular_velocity_covariance),orientation_covariance=list(message.orientation_covariance),
                     orientation_source='Gazebo IMU CUSTOM reference; not a hardware attitude-estimator validation')

        def slam(self, message):
            if message.header.frame_id != 'camera_init' or message.child_frame_id != 'demo_slam_body':
                self.invalid['slam'] += 1
                return
            p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
            try:
                r = rotation([q.x, q.y, q.z, q.w])
            except ValueError:
                self.invalid['slam'] += 1
                return
            position, velocity = np.asarray([p.x, p.y, p.z]), np.asarray([v.x, v.y, v.z])
            if not np.isfinite(position).all() or not np.isfinite(velocity).all():
                self.invalid['slam'] += 1
                return
            velocity_valid = all(message.twist.covariance[7*i] < 1e5 for i in range(3))
            self.add('slam', message, position=position, rotation=r, velocity_origin_body=velocity if velocity_valid else None)

        def joints(self, message):
            if len(set(message.name)) != len(message.name):
                self.invalid['joints'] += 1
                return
            indices = {name: i for i, name in enumerate(message.name)}
            try:
                order = [indices[name] for name in contract['gazebo_joint_names']]
                q = np.asarray([message.position[i] for i in order])
                qd = np.asarray([message.velocity[i] for i in order])
                if not np.isfinite(q).all() or not np.isfinite(qd).all():
                    raise ValueError('Joint samples are nonfinite')
            except (KeyError, IndexError, ValueError):
                self.invalid['joints'] += 1
                return
            self.add('joints', message, q=q, qd=qd,
                     message_names=list(message.name),message_position=list(message.position),message_velocity=list(message.velocity),message_effort=list(message.effort),
                     ordered_names=contract['gazebo_joint_names'],
                     effort_scope='Not used as measured torque; Gazebo state effort provenance is not established')

        def cloud(self, message):
            if message.header.frame_id != 'camera_init':
                self.invalid['cloud'] += 1
                return
            try:
                points = point_cloud2.read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
                points = points[np.isfinite(points).all(axis=1)]
            except (ValueError, TypeError, AssertionError):
                self.invalid['cloud'] += 1
                return
            self.add('cloud', message, points=points, point_count=len(points))

        def queue(self, row):
            t = row.get('world_sim_time') if isinstance(row, dict) else None
            if not isinstance(t, (int, float)) or not math.isfinite(t) or t <= self.last_reference:
                if isinstance(row,dict)and 'world_sim_time'not in row:self.reference_failures+=1
                return
            self.last_reference = t
            self.pending.append((time.monotonic(), row))

        def read_reference(self):
            try:
                size = self.reference_file.stat().st_size
                if size < self.reference_offset:
                    self.reference_failures += 1
                    self.reference_offset, self.partial = 0, b''
                with self.reference_file.open('rb') as stream:
                    stream.seek(self.reference_offset)
                    chunk = stream.read(4 * 1024 * 1024)
                    self.reference_offset = stream.tell()
                lines = (self.partial + chunk).split(b'\n')
                self.partial = lines.pop()
                for line in lines:
                    if not line.strip():
                        continue
                    try:
                        self.queue(json.loads(line))
                    except ValueError:
                        self.reference_failures += 1
            except OSError:
                pass
            # Only the complete ordered telemetry stream is used. A newer
            # state.json must never skip intervening reference rows.

        def metric(self, name, candidate, reference, scale=1.):
            if candidate is None or reference is None:
                return None
            actual, baseline = np.asarray(candidate), np.asarray(reference)
            if actual.shape != baseline.shape or not np.isfinite(actual).all() or not np.isfinite(baseline).all():
                return None
            error = (np.clip(actual, -100, 100) - np.clip(baseline, -100, 100)) * scale
            self.errors.setdefault(name, []).append(error.tolist())
            self.raw_errors.setdefault(name,[]).append((actual-baseline).tolist())
            return {'candidate': actual, 'reference': baseline, 'raw_error':actual-baseline,'scale':scale,'scaled_error': error}

        def compare(self, reference):
            t = reference['world_sim_time']
            audit_wall = time.monotonic()
            samples = {key: causal(history,t,args.max_age)
                       for key, history in self.histories.items()}
            result = {'sim_time':reference['sim_time'],'world_sim_time':t,'reference_stamp_ns':round(t*1e9),'wall_elapsed_s': time.monotonic() - self.started,
                      'status': 'shadow_unverified', 'policy_changed': False,
                      'timestamp_matching':'latest actual ROS header stamp <= recorded world_sim_time; integer nanosecond matching; no future sample',
                      'sources_missing': [key for key, sample in samples.items() if sample is None],
                      'sample_ages_sim_s': {key: None if sample is None else t - sample['t'] for key, sample in samples.items()},
                      'sample_wall_ages_at_audit_s': {key: None if sample is None else audit_wall - sample['received_monotonic_wall'] for key, sample in samples.items()},
                      'sample_wall_fresh_at_audit':{key:sample is not None and audit_wall-sample['received_monotonic_wall']<=args.max_age for key,sample in samples.items()},
                      'matched_samples':{key:None if sample is None else {k:v for k,v in sample.items()if k!='points'}for key,sample in samples.items()},
                      'terms': {}, 'reference_source': 'recorded Teacher Gazebo privileged policy telemetry; diagnostic only'}
            for key, sample in samples.items():
                if sample is not None:
                    self.matched[key] += 1
                    self.gaps[key].append(t - sample['t'])
            imu, slam, joints, cloud = (samples[key] for key in ('imu', 'slam', 'joints', 'cloud'))
            gyro = imu['gyro_body'] if imu else None
            result['terms']['base_ang_vel'] = self.metric('base_ang_vel', gyro, reference.get('body_ang_vel'), .2)
            if slam and gyro is not None and com_offset is not None and slam['velocity_origin_body'] is not None:
                com_velocity = slam['velocity_origin_body'] + np.cross(gyro, com_offset)
                result['terms']['base_lin_vel'] = self.metric('base_lin_vel', com_velocity, reference.get('body_lin_vel'))
            else:
                result['terms']['base_lin_vel'] = None
                result['linear_velocity_missing_reason'] = 'Needs causal valid SLAM origin twist, IMU gyro, and actual Gazebo COM offset'
            quat = reference.get('quaternion_wxyz')
            try:
                baseline_gravity = rotation([quat[1], quat[2], quat[3], quat[0]]).T @ np.asarray([0., 0., -1.])
                result['terms']['projected_gravity_imu'] = self.metric('projected_gravity_imu', imu['gravity_body'] if imu else None, baseline_gravity)
                result['terms']['projected_gravity_slam'] = self.metric('projected_gravity_slam', slam['rotation'].T @ np.asarray([0., 0., -1.]) if slam else None, baseline_gravity)
            except (TypeError, IndexError, ValueError):
                result['terms']['projected_gravity_imu'] = result['terms']['projected_gravity_slam'] = None
            default_q = np.asarray(contract['default_joint_positions'])
            reference_q = np.asarray(reference['q']) - default_q if isinstance(reference.get('q'), list) and len(reference['q']) == 12 else None
            result['terms']['joint_pos_rel'] = self.metric('joint_pos_rel', joints['q'] - default_q if joints else None, reference_q)
            result['terms']['joint_vel_rel'] = self.metric('joint_vel_rel', joints['qd'] if joints else None, reference.get('qd'), .05)
            result['terms']['joint_effort'] = {'status': 'unverified', 'source': 'No physical torque measurement established; native applied_torque is a force command'}
            result['terms']['velocity_commands'] = {'status': 'internal_nonprivileged', 'recorded_value': reference.get('command')}
            previous_tick_known = self.previous_reference_time is not None and abs(t - self.previous_reference_time - .02) < .001
            result['terms']['last_action'] = {'status': 'internal_nonprivileged' if previous_tick_known else 'unverified_previous_tick_missing',
                                            'previous_tick_raw_action': self.previous_raw_action if previous_tick_known else None,
                                            'recorded_current_raw_action': reference.get('action'),
                                            'note': 'Current action is not last_action; consumer preserves the previous tick raw action, including stop transitions'}
            if slam and cloud:
                scan = observed_scan(cloud['points'], slam['position'], slam['rotation'], SCAN_GRID_XY)
                result['terms']['height_scan_observed_candidate'] = scan
                if terrain is not None and isinstance(reference.get('position'), list) and quat is not None:
                    ref_position = np.asarray(reference['position'])
                    ref_rotation = rotation([quat[1], quat[2], quat[3], quat[0]])
                    yaw = math.atan2(ref_rotation[1, 0], ref_rotation[0, 0])
                    yaw_rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
                    xy = SCAN_GRID_XY @ yaw_rotation.T + ref_position[:2]
                    hit_z, _ = terrain.raycast(np.column_stack((xy, np.full(187, ref_position[2] + 20.))))
                    true_scan = np.clip(ref_position[2] - hit_z - .5, -1., 1.)
                    mask = scan['valid_mask'] & np.isfinite(hit_z)
                    difference = scan['values'][mask] - true_scan[mask]
                    result['height_diagnostic'] = {'reference': 'static SDF +20m top rays; privileged diagnostic only',
                        'paired_columns': int(mask.sum()), 'reference_overhead_columns': int(np.sum(hit_z > ref_position[2])),
                        'observed_column_rmse': float(np.sqrt(np.mean(difference**2))) if len(difference) else None,
                        'complete_scan_equivalence': 'unverified; observed coverage does not prove the uppermost surface was visible'}
            else:
                result['terms']['height_scan_observed_candidate'] = None
            self.references += 1
            self.records.write(json.dumps(clean(result), ensure_ascii=False, allow_nan=False) + '\n')
            self.previous_reference_time, self.previous_raw_action = t, reference.get('action')

        def tick(self):
            self.read_reference()
            now = time.monotonic()
            if now-self.last_graph_wall>=1:self.snapshot_graph('normal_spin_tick')
            while self.pending and (now - self.pending[0][0] >= .1 or now - self.started >= args.duration):
                _, reference = self.pending.popleft()
                self.compare(reference)

        def finish(self):
            self.tick()
            while self.pending:
                _, row = self.pending.popleft()
                self.compare(row)
            sources = {}
            for key in TOPICS:
                gaps = self.gaps[key]
                sources[key] = {'topic': TOPICS[key], 'received': self.counts[key], 'invalid': self.invalid[key],
                    'frames':dict(self.frames[key]),
                    'causal_matched_rows': self.matched[key], 'status': 'shadow_data_available_unverified' if self.matched[key] else 'unverified_missing',
                    'received_period_s': {clock: {'p50': float(np.median(values)), 'p95': float(np.percentile(values, 95)), 'max': max(values)} if values else None
                                          for clock, values in self.periods[key].items()},
                    'pair_age_sim_s': {'p50': float(np.median(gaps)), 'p95': float(np.percentile(gaps, 95)), 'max': max(gaps)} if gaps else None}
            metrics = {name: {'samples': len(values), 'scaled_rmse': np.sqrt(np.mean(np.asarray(values)**2, axis=0)).tolist(),
                              'scaled_error_abs_max': np.max(np.abs(values), axis=0).tolist(),
                              'raw_rmse':np.sqrt(np.mean(np.asarray(self.raw_errors[name])**2,axis=0)).tolist(),
                              'raw_error_abs_max':np.max(np.abs(self.raw_errors[name]),axis=0).tolist()} for name, values in self.errors.items() if values}
            self.snapshot_graph('finish_while_context_available')
            graph=self.last_valid_graph['publishers']if self.last_valid_graph else {'status':'unverified_no_valid_graph_snapshot'}
            summary = {'status': 'shadow_unverified', 'policy_changed': False, 'navigation_validation': 'unverified',
                'real_robot': 'unverified', 'ros_domain': args.ros_domain, 'reference_run': str(run),
                'duration_wall_s': time.monotonic() - self.started, 'reference_rows': self.references,
                'reference_parse_failures': self.reference_failures, 'sources': sources, 'metrics': metrics,
                'ros_publisher_graph_at_finish':graph,'readonly_node_application_publishers':self.application_publishers(),
                'publisher_graph_snapshot':None if self.last_valid_graph is None else {k:v for k,v in self.last_valid_graph.items()if k!='publishers'},
                'publisher_graph_snapshot_age_wall_s':None if self.last_valid_graph is None else time.monotonic()-self.last_valid_graph['capture_finished_monotonic_wall'],
                'timestamp_matching':'causal world_sim_time to exact ROS header nanoseconds; delayed audit wall age logged separately, never replaces simulation stamp',
                'imu_orientation_reference':config,'imu_contract_source':'generated run/sensor_contract.json'if sensor_contract else 'legacy scenario fallback',
                'sensor_contract_sha256':digest(run/'sensor_contract.json'),
                'actual_base_com_offset_m': None if com_offset is None else com_offset.tolist(),
                'contract_sha256': digest(ROOT / 'policy/contract.json'), 'scenario_sha256': digest(args.scenario),
                'world_sha256': digest(run / 'world.sdf'), 'script_sha256': digest(Path(__file__)),
                'limitations': ['No actor execution or observation switch', 'Absent sources are not filled with zeros or truth',
                    'IMU simulated orientation is not a physical attitude-estimator validation',
                    'Applied native torque is a command, not a hardware sensor measurement',
                    'Local cloud uppermost observed points are not certified +20m uppermost ray intersections']}
            (output / 'summary.json').write_text(json.dumps(clean(summary), ensure_ascii=False, indent=2, allow_nan=False) + '\n')
            self.records.close()
            self.sensor_records.close()
            self.graph_records.close()
            print(json.dumps(clean(summary), ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return Shadow()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=ROOT / 'runs/latest')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--max-age', type=float, default=.3)
    parser.add_argument('--ros-domain', type=int, default=79)
    parser.add_argument('--scenario', type=Path, default=ROOT.parent / 'simulation/scenario.json')
    parser.add_argument('--prepare', action='store_true')
    parser.add_argument('--phase-diagnostic',action='store_true')
    args, ros_args = parser.parse_known_args()
    if args.prepare:
        print(json.dumps({'status': 'prepared_unverified', 'policy_changed': False, 'starts_ros': False,
                          'ros_domain': args.ros_domain, 'run': str(args.run.resolve()), 'sources': TOPICS,
                          'scope': 'Read-only shadow; missing camera-only IMU/SLAM/joints/cloud remain unverified'}, ensure_ascii=False, indent=2))
        return 0
    if args.output is None:
        parser.error('--output must name a new audit directory')
    if not 0 < args.duration <= 3600 or not 0 < args.max_age <= .3:
        parser.error('duration must be in (0,3600], max-age in (0,.3]')
    run = args.run.resolve()
    if not run.is_dir():
        parser.error('Reference run directory does not exist')
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    if args.phase_diagnostic:
        phase_diagnostic(run,output)
        return 0
    contract = load(ROOT / 'policy/contract.json')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node = make_node(rclpy, args, run, output, contract)
    stopped=False
    def requested_stop(signum,frame):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,requested_stop);signal.signal(signal.SIGTERM,requested_stop)
    try:
        while not stopped and rclpy.ok() and time.monotonic() - node.started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
