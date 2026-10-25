#!/usr/bin/env python3
"""Actual SLAM/IMU/SCAN coordinator for the original46 Teacher simulation.

This replaces request.py's single-request ROS node, not its measured sensor
registration. No Twist/joint command or robot simulator pose is published.
Heavy receipt verification runs off the ROS executor; its results must bind
actual records/files before the pure Mission46 advances.
"""
from __future__ import annotations
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.append(str(HERE.parent))
from mission46 import TeacherMission46, ACTIVE_STAGES, MissionDeadlineExceeded
from mission46_profile import ROUTE_COUNTS, validate_profile, canonical_sha
from mission46_runtime_evidence import (FileTail, native_ns, checked_json, seal, json_bytes,
    verify_initialization, verify_rgb, verify_terrain, verify_dynamic, verify_parking,
    native_projection, status_projection)
from request import SceneAxisRegistration, StartupDeadline
from runtime_io import EvidenceWriter, latest_sensor_qos


def atomic_exact(path, data):
    path=Path(path);raw=json_bytes(data);tmp=path.with_name(path.name+'.mission46.tmp')
    tmp.write_bytes(raw);tmp.replace(path)
    return hashlib.sha256(raw).hexdigest()


def required_runtime_files():
    return [HERE/name for name in ('mission46.py','mission46_profile.py','mission46_contract.py',
        'mission46_runtime.py','mission46_runtime_evidence.py','mission46_obstacle_runtime.py')]


def load_interfaces(run, scope):
    data, source=checked_json(run,'mission46_runtime_interfaces.json')
    if scope['references'].get(source['file'])!=source['sha256']:
        raise ValueError('Teacher46 interface config is not frozen by actual scope')
    interfaces=data.get('interfaces',data)
    for record in interfaces.values():
        p=Path(record['module']).resolve();h=hashlib.sha256(p.read_bytes()).hexdigest()
        if h!=record['implementation_sha256'] or scope['references'].get(str(p))!=h:
            raise ValueError('Declared runtime interface is not the actual frozen implementation')
    return interfaces


def build_node(run, scope, profile):
    import rclpy
    import numpy as np
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,HistoryPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String,Bool
    from std_srvs.srv import Trigger
    from sensor_gate import TOPIC

    class Coordinator(Node):
        def __init__(self):
            # Existing bridge checks this exact sole anchor publisher name.
            super().__init__('teacher_navigation_request')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            for path in required_runtime_files():
                if scope['references'].get(str(path.resolve()))!=hashlib.sha256(path.read_bytes()).hexdigest():
                    raise ValueError('Mission46 runtime source is not frozen: '+str(path))
            self.mission=TeacherMission46(profile['original_scenario'])
            self.mission.bind_runtime_interfaces(load_interfaces(run,scope))
            self.writer=EvidenceWriter();self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='teacher-mission46-audit')
            self.jobs={};self.last_errors={};self.started_wall=time.monotonic()
            self.deadline=StartupDeadline(profile,self.started_wall)
            self.registration=SceneAxisRegistration(run,scope,self.writer)
            self.pose=None;self.pose_row=None;self.origin_row=None;self.last_pose_stamp=None
            self.health={};self.health_wall=-math.inf;self.nav={};self.nav_wall=-math.inf
            self.bridge={};self.bridge_wall=-math.inf;self.map_status={};self.obstacle={};self.obstacle_wall=-math.inf
            self.pose_rows=deque(maxlen=3000);self.imu_rows=deque(maxlen=4000)
            self.status_rows=deque(maxlen=1000);self.native_records=deque(maxlen=1000)
            self.guard_records=deque(maxlen=2000);self.obstacle_records=[]
            self.telemetry=FileTail(run/'telemetry.jsonl',project=native_projection);self.guard_tail=FileTail(run/'navigation_guard_history.jsonl')
            self.obstacle_calls=FileTail(run/'mission46_obstacle_calls.jsonl')
            self.init_try_sim=-math.inf;self.dynamic_try_sim=-math.inf;self.parking_origin=None
            self.anchor=None;self.request=None;self.request_ready=False;self.last_emit=-math.inf
            self.save_future=None;self.save_requested_epoch=None;self.save_request_id=None
            self.control_sequence=0;self.status_sequence=0;self.sticky_failure=None
            self.started=False;self.end_written=False
            self.pub=self.create_publisher(String,'/demo/navigation/request',10)
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.anchor_pub=self.create_publisher(String,'/demo/teacher/navigation/anchor',latched)
            self.stop_pub=self.create_publisher(Bool,'/demo/navigation/stop',10)
            self.obstacle_pub=self.create_publisher(Bool,'/demo/obstacle/enable',10)
            self.state_pub=self.create_publisher(String,'/demo/mission/state',10)
            self.mission_pub=self.create_publisher(String,'/demo/teacher/mission46/state',10)
            self.control_pub=self.create_publisher(String,'/demo/teacher/mission46/control',10)
            self.status_pub=self.create_publisher(String,'/demo/teacher/navigation/request_status',10)
            self.terrain_pub=self.create_publisher(String,'/demo/teacher/mission46/terrain_request',10)
            self.map_client=self.create_client(Trigger,'/demo/slam/save_map')
            self.create_subscription(Odometry,'/demo/slam/body_odom',self.on_pose,latest_sensor_qos())
            # Initialization proof needs real raw samples, not a single latest
            # value relabelled as a complete stationary history.
            imu_qos=QoSProfile(depth=4000,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Imu,'/livox/imu',self.on_imu,imu_qos)
            self.create_subscription(String,TOPIC,self.on_health,1)
            self.create_subscription(String,'/demo/navigation/status',self.on_nav,10)
            self.create_subscription(String,'/demo/control/safety',self.on_bridge,1)
            self.create_subscription(String,'/demo/slam/map_status',self.on_map,1)
            self.create_subscription(String,'/demo/obstacle/state',self.on_obstacle,10)
            self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

        def clock_ns(self):return int(self.get_clock().now().nanoseconds)
        def fresh_health(self,now):
            h=self.health
            return (h.get('ready')is True and h.get('ground_truth_navigation_used')is False
                    and 0<=now-self.health_wall<.3 and 0<=now-float(h.get('monotonic_wall',-math.inf))<.3)
        def fresh_pose(self,now,ns):
            return self.pose_row is not None and 0<=now-self.pose_row['received_wall_ns']/1e9<.3 and 0<=ns-self.pose_row['stamp_ns']<300_000_000
        def observation(self):
            if self.pose_row is None:return None
            r=self.pose_row
            return dict(source='actual_slam_body',frame_id='camera_init',ground_truth_used=False,
                position=list(r['position']),stamp_ns=r['stamp_ns'],received_wall_s=r['received_wall_ns']/1e9)
        def json_message(self,msg):
            data=json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('Expected actual ROS status object')
            return data
        def on_health(self,msg):
            try:
                d=self.json_message(msg);now=time.monotonic()
                if d.get('mode')!='teacher' or d.get('ground_truth_navigation_used')is not False or not 0<=now-float(d['monotonic_wall'])<.3:return
                self.health=d;self.health_wall=now
                if d.get('ready')is not True:self.registration.reset('sensor_gate_not_ready')
            except (ValueError,TypeError,KeyError):return
        def on_bridge(self,msg):
            try:
                d=self.json_message(msg);now=time.monotonic()
                if d.get('mode')!='teacher' or d.get('source')!='scan_slam' or not 0<=now-float(d['monotonic_wall'])<.3:return
                self.bridge=d;self.bridge_wall=now
                if d.get('state')=='failed':self.fail('Actual execution bridge failed: '+str(d.get('reason')))
            except (ValueError,TypeError,KeyError):return
        def on_map(self,msg):
            try:
                d=self.json_message(msg)
                if d.get('run_id')==run.name:self.map_status=d
            except (ValueError,TypeError):return
        def on_obstacle(self,msg):
            try:
                d=self.json_message(msg)
                if d.get('run_id')!=run.name:return
                self.obstacle=d;self.obstacle_wall=time.monotonic()
                if d.get('failed_updates',0)>0:self.fail('Actual moving-obstacle service failed')
            except (ValueError,TypeError):return
        def on_imu(self,msg):
            now_ns=self.clock_ns();wall_ns=time.monotonic_ns();stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            q=msg.orientation;w=msg.angular_velocity;a=msg.linear_acceleration
            row=dict(stamp_ns=int(stamp),callback_ros_clock_ns=now_ns,received_wall_ns=wall_ns,
                frame_id=msg.header.frame_id,orientation_xyzw=[q.x,q.y,q.z,q.w],
                angular_velocity_sensor=[w.x,w.y,w.z],linear_acceleration_sensor=[a.x,a.y,a.z],
                orientation_covariance_0=float(msg.orientation_covariance[0]),source_topic='/livox/imu')
            if self.registration.observe_imu(row):
                row['angular_velocity_body']=(self.registration.mount_rotation@np.asarray(row['angular_velocity_sensor'])).tolist()
                if all(math.isfinite(x) for x in row['linear_acceleration_sensor']):self.imu_rows.append(row)
        def on_pose(self,msg):
            now_ns=self.clock_ns();wall_ns=time.monotonic_ns();stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist.linear;w=msg.twist.twist.angular
            values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z]
            if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body'
                    or not 0<=now_ns-stamp<=300_000_000 or self.last_pose_stamp is not None and stamp<=self.last_pose_stamp
                    or not all(math.isfinite(x) for x in values) or not .98<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.02):return
            imu=next((r for r in reversed(self.imu_rows) if 0<=stamp-r['stamp_ns']<=20_000_000),None)
            row=dict(stamp_ns=int(stamp),callback_ros_clock_ns=now_ns,received_wall_ns=wall_ns,
                source_received_wall_ns=wall_ns,received_sim_ns=now_ns,
                frame_id=msg.header.frame_id,child_frame_id=msg.child_frame_id,
                position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],
                body_velocity=[v.x,v.y,v.z],body_angular_velocity=[w.x,w.y,w.z],source_topic='/demo/slam/body_odom',
                body_gyro=None if imu is None else imu['angular_velocity_body'],navigation_ground_truth_used=False)
            self.pose=msg;self.pose_row=row;self.last_pose_stamp=int(stamp);self.pose_rows.append(row)
            self.registration.observe_pose(row,self.fresh_health(wall_ns/1e9),wall_ns)
            self.writer.append(run/'mission46_actual_slam_poses.jsonl',row)
        def on_nav(self,msg):
            try:
                d=self.json_message(msg)
                if d.get('navigation_ground_truth_used')is not False or d.get('controller_kind')!='teacher':return
                wall=time.monotonic();ns=self.clock_ns();self.nav=d;self.nav_wall=wall
                row=dict(data=status_projection(d),actual_full_status_canonical_sha256=canonical_sha(d),
                         received_wall_ns=round(wall*1e9),received_sim_ns=ns,
                         actual_slam_pose=copy.deepcopy(self.pose_row))
                self.status_rows.append(row);self.writer.append(run/'mission46_actual_navigation_statuses.jsonl',row)
                if self.mission.original.stage=='navigating' and d.get('request_id')==self.mission.original.current_request and self.pose_row:
                    obstacle=copy.deepcopy(self.obstacle)
                    if (obstacle.get('run_id')==run.name and obstacle.get('request_id')==d['request_id']
                            and 0<=wall-self.obstacle_wall<.3):
                        self.mission.observe_obstacle(held=bool(d.get('obstacle_hold')),
                            actual_speed_mps=float(np.linalg.norm(self.pose_row['body_velocity'][:2])),obstacle=obstacle)
            except (ValueError,TypeError,KeyError) as e:self.last_errors['nav_callback']=str(e)
        def fail(self,reason):
            if self.sticky_failure is not None or self.mission.completed:return
            self.sticky_failure=reason
            if self.started:self.apply(self.mission._fail(time.monotonic(),reason))
        def submit(self,key,operation):
            if key not in self.jobs:self.jobs[key]=self.pool.submit(operation)
        def initialization(self,now,ns):
            if self.mission.initialization is not None or 'initialization' in self.jobs:return
            if not self.fresh_health(now) or not self.fresh_pose(now,ns):return
            registration=self.registration.try_freeze(ns,time.monotonic_ns())
            if registration is None:return
            if self.mission.original.heading_alignment is None:
                reg_sha=atomic_exact(run/'navigation_scene_axis_registration.json',registration)
                heading=copy.deepcopy(registration['heading_receipt'])
                heading.update(source='actual_slam_and_imu',source_evidence_sha256=reg_sha,ground_truth_used=False)
                self.mission.set_heading_alignment(heading)
            if ns/1e9-self.init_try_sim<.5:return
            self.init_try_sim=ns/1e9
            cutoff=ns-3_200_000_000
            # Callback records are append-only immutable dictionaries. Snapshot
            # references, not their large serialized histories, on the owner.
            poses=tuple(r for r in self.pose_rows if r['stamp_ns']>=cutoff)
            imus=tuple(r for r in self.imu_rows if r['stamp_ns']>=cutoff)
            native=tuple(r for r in self.native_records if native_ns(r['data'])>=cutoff)
            health=copy.deepcopy(self.health);reg=copy.deepcopy(registration)
            self.submit('initialization',lambda:verify_initialization(run,scope,poses,imus,native,reg,health))
        def freeze_anchor(self):
            if self.anchor is not None:return
            origin=list(self.mission.original.origin);heading=self.mission.original.heading_alignment
            reg=self.registration.receipt;yaw=heading['yaw_camera_init_from_world'];c,s=math.cos(yaw),math.sin(yaw)
            points=[origin]+[[origin[0]+c*p[0]-s*p[1],origin[1]+s*p[0]+c*p[1],origin[2]+p[2]]
                for name in ROUTE_COUNTS for p in profile['original_scenario'][name]]
            source=self.origin_row or self.pose_row
            self.anchor=dict(schema=1,experiment=profile['experiment'],run_dir=str(run),origin=origin,yaw=yaw,
                pose_stamp_ns=source['stamp_ns'],frame_id='camera_init',source='/demo/slam/body_odom',
                ground_truth_navigation_used=False,frozen_once=True,monotonic_wall=time.monotonic(),
                scene_axis_registration=reg,scene_axis_registration_file='navigation_scene_axis_registration.json',
                scene_axis_registration_file_sha256=hashlib.sha256((run/'navigation_scene_axis_registration.json').read_bytes()).hexdigest(),
                registered_route_camera_init_xyz=points,registered_route_points_sha256=canonical_sha(points),
                registered_route_reference='original46 centers with single actualSLAM/IMU yaw and initial measured origin',
                original_region_count=46,initial_actual_slam_source=copy.deepcopy(source))
            atomic_exact(run/'navigation_anchor.json',self.anchor)
            if self.context.ok():self.anchor_pub.publish(String(data=json.dumps(self.anchor,allow_nan=False)))
        def route(self,action):
            self.freeze_anchor()
            request={k:copy.deepcopy(action[k]) for k in ('schema_version','request_id','frame_id','goals')}
            directory=run/'mission46_requests';directory.mkdir(exist_ok=True)
            name=request['request_id'].split(':')[-2]+'_'+request['request_id'].split(':')[-1]+'.json'
            file=directory/name;raw=json_bytes(request)
            if file.exists() and file.read_bytes()!=raw:raise ValueError('Request ID attempted a geometry rewrite')
            if not file.exists():file.write_bytes(raw)
            atomic_exact(run/'navigation_request.json',request)
            self.writer.append(run/'mission46_requests.jsonl',dict(request=request,sha256=hashlib.sha256(raw).hexdigest(),
                mission_stage=self.mission.original.stage,original_goals_hash=self.mission.original.current_goals_sha256,
                monotonic_wall=time.monotonic(),ros_sim_ns=self.clock_ns()))
            self.request=request;self.request_ready=True;self.last_emit=-math.inf
        def apply(self,actions):
            for action in actions:
                self.writer.append(run/'mission46_actions.jsonl',dict(action=action,monotonic_wall=time.monotonic(),sim_ns=self.clock_ns()))
                kind=action['kind']
                if kind=='route':self.route(action)
                elif kind=='stop_navigation' and self.context.ok():self.stop_pub.publish(Bool(data=True))
                elif kind=='obstacle' and self.context.ok():self.obstacle_pub.publish(Bool(data=action['enabled']))
                elif kind=='save_map':
                    if self.save_future is None:
                        if not self.map_client.service_is_ready():raise ValueError('Actual RGB save service is unavailable')
                        self.save_requested_epoch=time.time();self.save_request_id=self.mission.original.current_request
                        self.save_future=self.map_client.call_async(Trigger.Request())
                elif kind=='terrain_switch':
                    atomic_exact(run/'mission46_terrain_request.json',action)
                    if self.context.ok():self.terrain_pub.publish(String(data=json.dumps(action,allow_nan=False)))
                elif kind=='final_active_hold' and self.parking_origin is None:
                    self.parking_origin=copy.deepcopy(self.pose_row)
                # hold/release are reflected in the independent periodic file;
                # only root controller publishes the actual velocity.
        def complete_jobs(self,now,ns):
            for key,future in list(self.jobs.items()):
                if not future.done():continue
                del self.jobs[key]
                try:
                    value=future.result();self.last_errors.pop(key,None)
                    if key=='initialization':
                        self.mission.accept_initialization(value);atomic_exact(run/'mission46_initialization_receipt.json',value)
                    elif key=='RGB':
                        atomic_exact(run/'mission46_rgb_save_receipt.json',value)
                        self.apply(self.mission.map_saved(value,wall_s=now,sim_ns=ns))
                    elif key=='terrain':
                        self.apply(self.mission.accept_terrain(value,wall_s=now,sim_ns=ns))
                    elif key=='dynamic':
                        self.mission.accept_dynamic(value);atomic_exact(run/'mission46_dynamic_receipt.json',value)
                    elif key=='parking':
                        atomic_exact(run/'mission46_final_parking_receipt.json',value)
                        self.apply(self.mission.parking_complete(value,wall_s=now,sim_ns=ns,observation=self.observation()))
                except Exception as e:
                    self.last_errors[key]=type(e).__name__+': '+str(e)
                    self.writer.append(run/'mission46_failures.jsonl',dict(phase=self.mission.original.stage,operation=key,
                        error=self.last_errors[key],monotonic_wall=now,sim_ns=ns))
                    if key in ('RGB','terrain','parking'):self.fail(self.last_errors[key])
        def periodic_control(self,now,ns):
            active=self.mission.original.stage in {'exploring','returning','navigating'}
            hold=(not active or not self.request_ready or self.mission.terrain_pending is not None or
                  not self.fresh_health(now) or not self.fresh_pose(now,ns) or self.sticky_failure is not None)
            reason='fresh_original46_route_release'
            if self.mission.pending_final is not None:hold=True;reason='final_first5s_active_hold'
            elif self.mission.terrain_pending is not None:reason='terrain_ack_pending'
            elif hold:reason='initialization_stage_transition_or_source_protection'
            self.control_sequence+=1
            data=dict(schema='teacher_mission46_control/v1',run_id=run.name,sequence=self.control_sequence,
                current_request=self.mission.original.current_request,sim_ns=ns,monotonic_wall=now,
                hold=hold,reason=reason,continuous_teacher_required=True,navigation_ground_truth_used=False)
            self.writer.atomic(run/'mission46_control.json',data)
            self.writer.append(run/'mission46_control_history.jsonl',data)
            if self.context.ok():self.control_pub.publish(String(data=json.dumps(data,allow_nan=False)))
        def publish_state(self,now,ns):
            data=self.mission.snapshot();self.status_sequence+=1
            data.update(sequence=self.status_sequence,monotonic_wall=now,sim_ns=ns,
                navigation=copy.deepcopy(self.nav),last_runtime_errors=dict(self.last_errors),
                actual_source_ready=self.fresh_health(now) and self.fresh_pose(now,ns),
                initialization_verified=self.mission.initialization is not None,
                current_rgb_save_verified=self.mission.rgb_receipt is not None,
                original_dynamic_verified=self.mission.dynamic_receipt is not None,
                final_parking_verified=self.mission.parking_receipt is not None,
                terminal_receipt_binding=None if self.mission.parking_receipt is None else {
                    key:self.mission.parking_receipt[key] for key in ('source_evidence_file','source_evidence_sha256')},
                source='actual_SLAM_IMU_SCAN_Teacher46_coordinator',
                raw_source_offsets=dict(telemetry=self.telemetry.offset,guard=self.guard_tail.offset,obstacle=self.obstacle_calls.offset))
            if self.sticky_failure is not None:data.update(stage='failed',message=self.sticky_failure,functional_sequence_completed=False)
            self.writer.atomic(run/'mission46_status.json',data)
            self.writer.append(run/'mission46_status_history.jsonl',data)
            if self.context.ok():
                self.state_pub.publish(String(data=json.dumps(data,allow_nan=False)))
                self.mission_pub.publish(String(data=json.dumps(data,allow_nan=False)))
                self.status_pub.publish(String(data=json.dumps(dict(state=data['stage'],ready=data['actual_source_ready'],
                    monotonic_wall=now,ros_sim_time_ns=ns,request_id=data['current_request'],
                    navigation_validation='unverified',mission46=True),allow_nan=False)))
        def tick(self):
            now=time.monotonic();ns=self.clock_ns()
            try:
                if not self.started and ns>0:
                    self.started=True;self.apply(self.mission.start(run.name,wall_s=now,sim_ns=ns))
                if not self.started:return
                if self.writer.error:raise RuntimeError(self.writer.error)
                self.native_records.extend(self.telemetry.poll());self.guard_records.extend(self.guard_tail.poll())
                self.obstacle_records.extend(self.obstacle_calls.poll())
                if self.native_records and self.native_records[-1]['data'].get('fault')is not None:
                    self.fail('Actual native Teacher fault: '+str(self.native_records[-1]['data']['fault']))
                if self.mission.original.stage=='waiting_sensors':
                    if self.deadline.observe(ns,now):self.fail(self.deadline.failed)
                    self.initialization(now,ns)
                self.complete_jobs(now,ns)
                previous=copy.deepcopy(self.mission.original.origin)
                self.apply(self.mission.tick(wall_s=now,sim_ns=ns,sensors_ok=self.fresh_health(now),
                    observation=self.observation(),map_points=self.map_status.get('point_count',0)))
                if self.mission.original.origin!=previous and self.pose_row:self.origin_row=copy.deepcopy(self.pose_row)
                stage=self.mission.original.stage
                if stage in {'exploring','returning','navigating'} and self.nav.get('request_id')==self.mission.original.current_request:
                    if self.fresh_pose(now,ns) and 0<=now-self.nav_wall<.3:
                        self.apply(self.mission.terrain_intent(self.nav,wall_s=now,sim_ns=ns,observation=self.observation()))
                        if self.mission.terrain_pending is not None and 'terrain' not in self.jobs and (run/'mission46_terrain_ack.json').exists():
                            ack=json.loads((run/'mission46_terrain_ack.json').read_text())
                            if ack.get('request_id')==self.mission.original.current_request:
                                pending=copy.deepcopy(self.mission.terrain_pending)
                                self.submit('terrain',lambda:verify_terrain(run,scope,ack,pending,ns))
                        if (stage=='navigating' and self.mission.dynamic_receipt is None
                                and self.obstacle.get('visible_phase')=='clear'
                                and self.nav.get('aligned_obstacle_resumes',0)>0
                                and ns/1e9-self.dynamic_try_sim>=1.):
                            self.dynamic_try_sim=ns/1e9
                            rid=self.mission.original.current_request
                            sr=tuple(r for r in self.status_rows if r['data'].get('request_id')==rid)
                            first=min((r['received_sim_ns'] for r in sr),default=ns)
                            pr=tuple(r for r in self.pose_rows if r['stamp_ns']>=first-350_000_000)
                            gr=tuple(r for r in self.guard_records if r['data'].get('request_id')==rid)
                            ob=tuple(r for r in self.obstacle_records if r['data'].get('request_id')==rid)
                            self.submit('dynamic',lambda:verify_dynamic(run,rid,sr,pr,gr,ob))
                        if self.nav.get('state')=='succeeded' and stage=='navigating' and self.mission.dynamic_receipt is None:
                            # A verifier still pending is not a pass; wait until
                            # it finishes, then fail below if the real proof is absent.
                            if 'dynamic' not in self.jobs:self.fail('Original final navigation lacks actual dynamic obstacle evidence')
                        elif self.nav.get('state') in ('succeeded','failed'):
                            self.apply(self.mission.navigation_result(self.nav,wall_s=now,sim_ns=ns,observation=self.observation()))
                if self.save_future is not None and self.save_future.done():
                    future=self.save_future;self.save_future=None;response=future.result()
                    result=dict(success=bool(response.success),message=response.message)
                    rid=self.save_request_id;epoch=self.save_requested_epoch
                    self.submit('RGB',lambda:verify_rgb(run,rid,result,epoch))
                if self.mission.pending_final and 'parking' not in self.jobs:
                    start=self.mission.pending_final['parking_start_sim_ns']
                    if ns>=start+5_200_000_000:
                        end=start+5_200_000_000
                        poses=tuple(r for r in self.pose_rows if start<=r['stamp_ns']<=end)
                        native=tuple(r for r in self.native_records if start<=native_ns(r['data'])<=end)
                        statuses=tuple(r for r in self.status_rows if start<=r['received_sim_ns']<=end);origin=copy.deepcopy(self.parking_origin)
                        rid=self.mission.original.current_request
                        self.submit('parking',lambda:verify_parking(run,rid,start,poses,native,statuses,origin))
                if self.request and self.mission.original.stage in {'exploring','returning','navigating'} and now-self.last_emit>=1.:
                    self.anchor_pub.publish(String(data=json.dumps(self.anchor,allow_nan=False)))
                    self.pub.publish(String(data=json.dumps(self.request,allow_nan=False)));self.last_emit=now
                if self.mission.original.stage in {'completed','failed','stopped'}:
                    self.obstacle_pub.publish(Bool(data=False))
                    if self.mission.original.stage!='completed':self.stop_pub.publish(Bool(data=True))
                self.periodic_control(now,ns);self.publish_state(now,ns)
            except Exception as e:
                self.fail(type(e).__name__+': '+str(e))
                self.periodic_control(now,ns);self.publish_state(now,ns)

        def close(self):
            try:
                if self.started and self.mission.original.stage in ACTIVE_STAGES:
                    self.fail('Mission46 coordinator terminated before whole mission completion')
                if self.started:self.publish_state(time.monotonic(),self.clock_ns())
            finally:
                try:self.pool.shutdown(wait=True,cancel_futures=True)
                finally:self.writer.close()

    return Coordinator()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve()
    from pid_scope import verify_scope
    scope=json.loads((run/'navigation_scope.json').read_text());profile=verify_scope(run/'navigation_scope.json')['profile']
    validate_profile(profile)
    if any((run/n).exists() for n in ('navigation_anchor.json','navigation_request.json','mission46_status.json')):
        raise RuntimeError('Mission46 requires a new actual run, not reused registration or data')
    import rclpy
    rclpy.init(args=ros);node=build_node(run,scope,profile)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        try:node.close()
        finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
