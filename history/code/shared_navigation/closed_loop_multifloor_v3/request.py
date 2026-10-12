#!/usr/bin/env python3
"""Freeze the finite out/return regions once from warmed measured SLAM."""
from __future__ import annotations
import argparse
from collections import deque
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time
import sys
import numpy as np
sys.path.append(str(Path(__file__).resolve().parent.parent))
from runtime_io import EvidenceWriter,latest_sensor_qos


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
def digest(raw):return hashlib.sha256(raw).hexdigest()


class SceneAxisRegistration:
    """Stationary, causal sensor attitude pairs for a single map-axis yaw."""
    def __init__(self,run,scope,writer):
        self.run=Path(run).resolve();self.scope=scope;self.writer=writer
        path=self.run/'sensor_contract.json';self._frozen(path)
        self.sensor_contract_sha256=digest(path.read_bytes());self.imu_config=json.loads(path.read_text())['imu']
        reference=self.imu_config.get('orientation_reference')or{}
        if (reference.get('localization')!='CUSTOM' or reference.get('custom_parent_frame')!='world' or
            not isinstance(reference.get('description'),str)or not reference['description'].strip()):
            raise ValueError('Scene axes require the actual sensor contract CUSTOM/world IMU orientation reference')
        snapshots=json.loads((self.run/'navigation_source_snapshots.json').read_text())
        self._frozen(self.run/'navigation_source_snapshots.json')
        sources=[(source,row)for source,row in snapshots.items()
                 if Path(source).name=='heading_alignment.py'and Path(source).parent.name=='slam']
        if len(sources)!=1:raise ValueError('No unique frozen original SLAM heading_alignment source')
        source,row=sources[0];snapshot=Path(row['snapshot']).resolve()
        if not snapshot.is_relative_to(self.run/'sources'):raise ValueError('Heading source archive is outside this run')
        self._frozen(Path(source));self._frozen(snapshot)
        if digest(snapshot.read_bytes())!=row['sha256']:raise ValueError('Heading source archive hash differs')
        spec=importlib.util.spec_from_file_location('teacher_registered_scene_heading',snapshot)
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.heading_source={'source':source,'snapshot':str(snapshot),'sha256':row['sha256']}
        self.reference=reference
        for name in ('world_quaternion','body_imu_quaternion'):
            q=np.asarray(reference.get(name),dtype=float)
            if q.shape!=(4,)or not np.isfinite(q).all()or not .98<=float(q@q)<=1.02:
                raise ValueError('Invalid actual IMU '+name)
        self.mount_rotation=self.module.quaternion_rotation(reference['body_imu_quaternion'])
        mount=self.module.Rotation.from_euler('xyz',self.imu_config['body_rpy']).as_matrix()
        if not np.allclose(self.mount_rotation,mount,atol=1e-10,rtol=0):
            raise ValueError('Actual sensor mount quaternion and body_rpy disagree')
        self.imu_rows=deque(maxlen=512);self.pairs=deque(maxlen=10);self.all_pairs=[]
        self.last_imu_stamp=None;self.last_pair_stamp=None;self.reset_count=0
        self.wait_reason='waiting_stationary_actual_sensor_pairs';self.receipt=None

    def _frozen(self,path):
        path=Path(path).resolve();expected=self.scope['references'].get(str(path))
        if expected is None or digest(path.read_bytes())!=expected:raise ValueError('Unfrozen registration source '+str(path))

    def reset(self,reason):
        if self.pairs:self.reset_count+=1
        self.pairs.clear();self.last_pair_stamp=None;self.wait_reason=reason

    def observe_imu(self,row):
        stamp=row['stamp_ns'];clock=row['callback_ros_clock_ns'];q=np.asarray(row['orientation_xyzw'],dtype=float)
        gyro=np.asarray(row['angular_velocity_sensor'],dtype=float)
        if (type(stamp)is not int or type(clock)is not int or not 0<=stamp<=clock or clock-stamp>300_000_000 or
            self.last_imu_stamp is not None and stamp<=self.last_imu_stamp or row['frame_id']!=self.imu_config['frame'] or
            not math.isfinite(row['orientation_covariance_0'])or row['orientation_covariance_0']<0 or q.shape!=(4,)or gyro.shape!=(3,)or
            not np.isfinite(q).all()or not np.isfinite(gyro).all()or not .98<=float(q@q)<=1.02):return False
        body=self.mount_rotation@gyro
        original=dict(row);original['angular_velocity_body']=body.tolist()
        self.imu_rows.append(original);self.last_imu_stamp=stamp
        return True

    def observe_pose(self,row,healthy,wall_ns):
        if self.receipt is not None:return False
        if not healthy:self.reset('sensor_gate_not_ready');return False
        stamp=row['stamp_ns'];clock=row['callback_ros_clock_ns'];q=np.asarray(row['quaternion_xyzw'],dtype=float)
        velocity=np.asarray(row['body_velocity'],dtype=float)
        if (not 0<=stamp<=clock or clock-stamp>300_000_000 or q.shape!=(4,)or velocity.shape!=(3,)or
            not np.isfinite(q).all()or not np.isfinite(velocity).all()or not .98<=float(q@q)<=1.02 or
            np.linalg.norm(velocity)>.03 or not 0<=wall_ns-row['received_wall_ns']<=300_000_000):
            self.reset('initial_SLAM_source_not_causal_fresh_stationary');return False
        imu=next((value for value in reversed(self.imu_rows)if 0<=stamp-value['stamp_ns']<=20_000_000),None)
        if (imu is None or not 0<=wall_ns-imu['received_wall_ns']<=300_000_000 or
            np.linalg.norm(imu['angular_velocity_body'])>.05):
            self.reset('no_causal_fresh_stationary_IMU_pair');return False
        if self.last_pair_stamp is not None:
            if stamp<=self.last_pair_stamp:return False
            if stamp-self.last_pair_stamp>300_000_000:self.reset('initial_header_pair_gap_exceeds300ms')
        pair=dict(sequence=len(self.all_pairs)+1,initialization_reset_count=self.reset_count,
            slam_stamp_ns=int(stamp),imu_stamp_ns=int(imu['stamp_ns']),pair_gap_ns=int(stamp-imu['stamp_ns']),
            slam_stamp=stamp/1e9,imu_stamp=imu['stamp_ns']/1e9,
            slam_body_quaternion=row['quaternion_xyzw'],imu_quaternion=imu['orientation_xyzw'],
            slam_source=dict(row),IMU_source=dict(imu),navigation_ground_truth_used=False)
        self.pairs.append(pair);self.all_pairs.append(pair);self.last_pair_stamp=stamp
        self.writer.append(self.run/'nav_registration_pairs.jsonl',pair)
        self.wait_reason='collecting_stationary_sensor_heading_pairs'
        return True

    def try_freeze(self,clock_ns,wall_ns):
        if self.receipt is not None:return self.receipt
        if len(self.pairs)<10 or self.pairs[-1]['slam_stamp_ns']-self.pairs[0]['slam_stamp_ns']<800_000_000:return None
        last=self.pairs[-1]
        if (not 0<=clock_ns-last['slam_stamp_ns']<=300_000_000 or
            not 0<=clock_ns-last['imu_stamp_ns']<=300_000_000 or
            not 0<=wall_ns-last['slam_source']['received_wall_ns']<=300_000_000 or
            not 0<=wall_ns-last['IMU_source']['received_wall_ns']<=300_000_000):
            self.reset('registration_last_pair_expired');return None
        try:
            heading=self.module.calibrate_scene_heading(list(self.pairs),
                imu_reference_world_quaternion=self.reference['world_quaternion'],
                body_imu_quaternion=self.reference['body_imu_quaternion'],
                reference_description=self.reference['description'],max_stamp_difference_s=.02,
                min_samples=10,min_span_s=.8,max_spread_rad=math.radians(1),max_gravity_residual_rad=math.radians(3))
        except ValueError as error:
            self.reset('heading_calibration_rejected: '+str(error));return None
        raw=''.join(json.dumps(row,allow_nan=False,ensure_ascii=False)+'\n'for row in self.all_pairs).encode()
        self.receipt=dict(schema='teacher_scene_axis_registration/v1',status='frozen',heading_receipt=heading,
            exact_paired_sample_records=list(self.pairs),paired_samples_canonical_sha256=digest(canonical(list(self.pairs)).encode()),
            paired_sources_file='nav_registration_pairs.jsonl',paired_sources_expected_file_sha256=digest(raw),
            paired_sources_expected_records=len(self.all_pairs),sensor_contract_sha256=self.sensor_contract_sha256,
            heading_source=self.heading_source,orientation_reference=dict(self.reference),
            frozen_ros_clock_ns=int(clock_ns),frozen_monotonic_wall_ns=int(wall_ns),
            stopped_initialization={'body_velocity_norm_max_mps':.03,'IMU_body_gyro_norm_max_radps':.05},
            pairing_causal=True,maximum_pair_gap_ns=20_000_000,minimum_samples=10,minimum_span_ns=800_000_000,
            heading_spread_limit_rad=math.radians(1),gravity_residual_limit_rad=math.radians(3),
            ground_truth_navigation_used=False,
            meaning='scene axes from actual sensor attitudes; known-map prior registration, not automatic point-cloud map registration')
        return self.receipt


class StartupDeadline:
    """Simulation-time startup budget plus finite wall cap for absent clocks."""
    def __init__(self,profile,wall):
        self.wall_start=wall;self.sim_start=None;self.last_sim=None;self.profile=profile;self.failed=None
    def observe(self,sim_ns,wall):
        if self.last_sim is not None and sim_ns<self.last_sim:self.failed='Actual ROS simulation clock moved backward'
        self.last_sim=sim_ns
        if sim_ns>0 and self.sim_start is None:self.sim_start=sim_ns
        if wall-self.wall_start>=self.profile['request_startup_timeout_wall_s']:
            self.failed='Sensor SLAM startup wall safety deadline exceeded'
        if self.sim_start is not None and (sim_ns-self.sim_start)/1e9>=self.profile['request_startup_timeout_sim_s']:
            self.failed='Sensor SLAM startup simulation deadline exceeded'
        return self.failed


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String,Bool
    from pid_scope import verify_scope
    from sensor_gate import atomic,TOPIC
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();receipt=verify_scope(run/'navigation_scope.json');profile=receipt['profile']
    scope=json.loads((run/'navigation_scope.json').read_text())
    if (run/'navigation_request.json').exists()or(run/'navigation_anchor.json').exists():
        raise RuntimeError('Choose a new run; a previously anchored route cannot be reused')
    rclpy.init(args=ros)
    class Request(Node):
        def __init__(self):
            super().__init__('teacher_navigation_request')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.pose=None;self.pose_wall=None;self.pose_stamp=None;self.health={};self.health_wall=None
            self.request=None;self.anchor=None;self.started_wall=time.monotonic();self.last_emit=-math.inf;self.failed=False
            self.deadline=StartupDeadline(profile,self.started_wall);self.evidence=EvidenceWriter()
            self.registration=SceneAxisRegistration(run,scope,self.evidence)if profile.get('route_world_points')is not None else None
            self.status_pub=self.create_publisher(String,'/demo/teacher/navigation/request_status',10)
            self.pub=self.create_publisher(String,'/demo/navigation/request',10)
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.anchor_pub=self.create_publisher(String,'/demo/teacher/navigation/anchor',latched)
            self.stop_pub=self.create_publisher(Bool,'/demo/navigation/stop',10)
            self.create_subscription(Odometry,'/demo/slam/body_odom',self.odom,latest_sensor_qos())
            if self.registration:self.create_subscription(Imu,'/livox/imu',self.imu,latest_sensor_qos())
            self.create_subscription(String,TOPIC,self.sensor,1)
            self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def sensor(self,msg):
            try:data=json.loads(msg.data)
            except (ValueError,TypeError):return
            if (data.get('ground_truth_navigation_used')is not False or data.get('mode')!=profile['controller_kind']
                or not -.05<=time.monotonic()-float(data.get('monotonic_wall',-math.inf))<.3):return
            self.health=data;self.health_wall=time.monotonic()
            if self.registration and data.get('ready')is not True:self.registration.reset('sensor_gate_not_ready')
        def imu(self,msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            now=self.get_clock().now().nanoseconds;q=msg.orientation;v=msg.angular_velocity
            self.registration.observe_imu(dict(stamp_ns=int(stamp),callback_ros_clock_ns=int(now),
                received_wall_ns=time.monotonic_ns(),frame_id=msg.header.frame_id,
                orientation_xyzw=[q.x,q.y,q.z,q.w],angular_velocity_sensor=[v.x,v.y,v.z],
                orientation_covariance_0=float(msg.orientation_covariance[0]),source_topic='/livox/imu'))
        def odom(self,msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            now=self.get_clock().now().nanoseconds;p=msg.pose.pose.position;q=msg.pose.pose.orientation
            if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body'
                or self.pose_stamp is not None and stamp<=self.pose_stamp
                or not -.05e9<=now-stamp<.3e9
                or not all(math.isfinite(v)for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                or not .98<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.02):return
            if self.registration and stamp>now:return
            self.pose=msg;self.pose_wall=time.monotonic();self.pose_stamp=stamp
            if self.registration:
                v=msg.twist.twist.linear;w=msg.twist.twist.angular;wall_ns=time.monotonic_ns()
                healthy=(self.health_wall is not None and wall_ns/1e9-self.health_wall<.3 and self.health.get('ready')is True
                    and 0<=wall_ns/1e9-float(self.health.get('monotonic_wall',-math.inf))<.3)
                self.registration.observe_pose(dict(stamp_ns=int(stamp),callback_ros_clock_ns=int(now),
                    received_wall_ns=wall_ns,frame_id=msg.header.frame_id,child_frame_id=msg.child_frame_id,
                    position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],
                    body_velocity=[v.x,v.y,v.z],body_angular_velocity=[w.x,w.y,w.z],source_topic='/demo/slam/body_odom'),
                    healthy,time.monotonic_ns())
        def freeze(self):
            p=self.pose.pose.pose.position;q=self.pose.pose.pose.orientation
            # Anchor is measured body yaw in camera_init, not simulator world
            # yaw/position, GPS, IMU yaw injection or a truth-based SE3 alignment.
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            origin=[p.x,p.y,p.z]
            registration=None
            if self.registration:
                registration=self.registration.try_freeze(self.get_clock().now().nanoseconds,time.monotonic_ns())
                if registration is None:return
                source=registration['exact_paired_sample_records'][-1]['slam_source']
                origin=list(source['position']);yaw=registration['heading_receipt']['yaw_camera_init_from_world']
            self.anchor={'schema':1,'experiment':profile['experiment'],'run_dir':str(run),
                'origin':origin,'yaw':yaw,'pose_stamp_ns':self.pose_stamp if registration is None else source['stamp_ns'],'frame_id':'camera_init',
                'source':'/demo/slam/body_odom','ground_truth_navigation_used':False,
                'frozen_once':True,'monotonic_wall':time.monotonic()}
            if registration is not None:
                atomic(run/'navigation_scene_axis_registration.json',registration)
                self.anchor.update(scene_axis_registration=registration,
                    scene_axis_registration_file='navigation_scene_axis_registration.json',
                    scene_axis_registration_file_sha256=digest((run/'navigation_scene_axis_registration.json').read_bytes()),
                    paired_sources_file=registration['paired_sources_file'],
                    paired_sources_expected_file_sha256=registration['paired_sources_expected_file_sha256'],
                    initial_body_quaternion_xyzw=source['quaternion_xyzw'],
                    body_yaw_camera_init=math.atan2(2*(source['quaternion_xyzw'][3]*source['quaternion_xyzw'][2]+source['quaternion_xyzw'][0]*source['quaternion_xyzw'][1]),
                        1-2*(source['quaternion_xyzw'][1]**2+source['quaternion_xyzw'][2]**2)))
                from route import bind_registered_route
                bind_registered_route(self.anchor,profile)
            arrival={'type':'disc_prism','radius_m':profile['arrival_radius_m'],
                'height_half_span_m':profile['arrival_height_half_span_m'],'dwell_sim_s':profile['dwell_sim_s'],
                'control_band':{'type':'disc_prism','radius_m':profile['arrival_control_radius_m'],
                               'height_half_span_m':profile['arrival_height_half_span_m']}}
            from route import build_request
            self.request=build_request(run.name,self.anchor,profile,arrival)
            atomic(run/'navigation_anchor.json',self.anchor);atomic(run/'navigation_request.json',self.request)
            self.anchor_pub.publish(String(data=json.dumps(self.anchor)))
        def tick(self):
            now=time.monotonic();sim_ns=self.get_clock().now().nanoseconds
            if self.evidence.error:self.failed=True
            if self.request is None and self.deadline.observe(sim_ns,now):self.failed=True
            ready=(self.pose_wall is not None and now-self.pose_wall<.3 and self.health_wall is not None
                and now-self.health_wall<.3 and self.health.get('ready')is True
                and self.pose_stamp is not None and -.05e9<=sim_ns-self.pose_stamp<.3e9
                and -.05<=now-float(self.health.get('monotonic_wall',-math.inf))<.3)
            if not self.failed and self.request is None and ready:self.freeze()
            if not self.failed and self.request is not None and now-self.last_emit>=1.:
                # Immutable idempotent resends handle DDS discovery. Anchor is
                # never re-estimated from later motion or simulator state.
                self.anchor_pub.publish(String(data=json.dumps(self.anchor)))
                self.pub.publish(String(data=json.dumps(self.request)))
                self.last_emit=now
            if self.failed:self.stop_pub.publish(Bool(data=True))
            state='failed'if self.failed else'route_frozen'if self.request else'waiting_actual_sensor_slam'
            status={'state':state,'ready':ready,'monotonic_wall':now,'ros_sim_time_ns':sim_ns,
                'startup_first_sim_ns':self.deadline.sim_start,'startup_failure':self.deadline.failed,
                'startup_wall_elapsed_s':now-self.started_wall,
                'navigation_validation':'unverified','request_id':None if self.request is None else self.request['request_id'],
                'source':'SLAM pose and warmed real sensor chain','ground_truth_navigation_used':False}
            if self.registration:
                status.update(scene_registration_wait_reason=self.registration.wait_reason,
                    scene_registration_pair_count=len(self.registration.pairs),scene_registration_total_pairs=len(self.registration.all_pairs),
                    scene_registration_initialization_resets=self.registration.reset_count,
                    scene_registration_is_map_prior=True,known_spawn_xy_uncertainty_m=.15)
            self.status_pub.publish(String(data=json.dumps(status)))
            self.evidence.append(run/'navigation_request_history.jsonl',status)
    node=Request()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.stop_pub.publish(Bool(data=True))
        try:
            node.evidence.close()
            if node.registration:
                path=run/'nav_registration_pairs.jsonl';expected=node.registration.receipt
                actual=digest(path.read_bytes())if path.exists()else None
                record={'schema':'teacher_scene_axis_registration_writer/v1',
                    'status':'drained'if expected is None or actual==expected['paired_sources_expected_file_sha256']else'failed',
                    'expected_records':len(node.registration.all_pairs),'actual_file_sha256':actual,
                    'expected_file_sha256':None if expected is None else expected['paired_sources_expected_file_sha256'],
                    'registration_frozen':expected is not None,'ground_truth_navigation_used':False}
                with (run/'navigation_registration_writer_receipt.json').open('x')as out:out.write(canonical(record)+'\n')
                if record['status']=='failed':raise RuntimeError('Actual registration pair archive does not match its frozen reference')
        finally:
            node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
