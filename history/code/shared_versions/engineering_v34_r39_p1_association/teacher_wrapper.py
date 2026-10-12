#!/usr/bin/env python3
"""Teacher-specific finite wrapper around existing measured-pose Go2 SCAN.

The shared controller consumes SLAM, actual registered clouds and checked SCAN
trajectories. This wrapper narrows scope, fixes duplicate-stamp acceptance and
requires measured stopping before obstacle recovery. It owns no joint actuator.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];DEMO=ROOT.parent;NAV=HERE.parent
sys.path.insert(0,str(DEMO))
sys.path.append(str(NAV))
sys.path.insert(0,str(DEMO/'navigation'))
spec=importlib.util.spec_from_file_location('teacher_measured_scan_base',HERE/'shared_controller.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
from pid_scope import verify_scope
from status_acceptance_receipt import StatusAcceptanceReceipt
from control_core import rotation_xyzw,body_tilt
from runtime_io import EvidenceWriter,latest_sensor_qos
from compact_archives import make_evidence_writer
from transition_gate import TeacherHeadingGate
from guard_audit import NativeGuardAudit,result_json
from std_msgs.msg import String

NATIVE_STEERING_GUARD=base.steering_obstacle_ahead


class RecordedPublisher:
    def __init__(self,publisher,run,node):self.publisher=publisher;self.run=run;self.node=node
    def publish(self,msg):
        data=json.loads(msg.data)
        data.update(mode=self.node.profile['controller_kind'],scope=self.node.profile['experiment'],navigation_ground_truth_used=False,
            policy_observations=self.node.profile['policy_observations'],
            navigation_validation='unverified',acceptance_is_global=False)
        data['known_scene_localization']=copy.deepcopy(getattr(self.node,'known_scene_inputs',None))
        data['known_scene_runtime_status']=copy.deepcopy(getattr(self.node,'known_scene_status',None))
        data['known_scene_scan_epoch']=copy.deepcopy(getattr(self.node,'known_scene_scan_epoch',None))
        data['controller_kind']=self.node.profile['controller_kind']
        data['outer_velocity_controller']='Actual SLAM header-event geometric path/yaw PD and COM speed PI; causal IMU and actual Teacher input ACK; native SCAN collision guard'
        data['cascade_parking']=copy.deepcopy(getattr(self.node,'cascade_last_row',None))
        data['path_admission']=dict(hold=getattr(self.node,'path_admission_hold',False),
            replan_pending=getattr(self.node,'path_admission_replan_pending',False),
            decision_sequence=getattr(self.node,'path_admission_sequence',0),
            last_decision=copy.deepcopy(getattr(self.node,'path_admission_last_decision',None)),
            active_reference_stamp=getattr(self.node,'active_reference_stamp',None),
            pending_reference_stamp=self.node.trajectory_association.reference_stamp,
            occupancy_or_terrain_support_certified=False)
        data['teacher_transition']=self.node.heading_gate.evidence(self.node.get_clock().now().nanoseconds/1e9,time.monotonic())
        if self.node.profile.get('arrival_stop_policy')=='after_measured_dwell':
            data['motion_limits']['pure_turn_rate_rad_s']=self.node.heading_gate.max_turn_rate
            data['arrival_stop_policy']='after_measured_dwell'
            data['arrival_stop_policy_source']='Actual raw SLAM original control-region ArrivalWindow; no tracking-pose arrival'
        data=self.node.status_acceptance_receipt.project(data)
        self.publisher.publish(String(data=json.dumps(data,ensure_ascii=False,allow_nan=False)))
        self.node.evidence.append(self.run/'navigation_status.jsonl',{**data,'monotonic_wall':time.monotonic(),
            'ros_sim_time':self.node.get_clock().now().nanoseconds/1e9})
        self.node.evidence.atomic(self.run/'navigation_status.json',data)


class TeacherNavigation(base.Navigation):
    def __init__(self,run):
        self.run=run;receipt=verify_scope(run/'navigation_scope.json');self.profile=receipt['profile']
        self.status_acceptance_receipt=StatusAcceptanceReceipt(run,receipt,enabled=(
            self.profile.get('mission46_required') is True
            and self.profile.get('continuous_route_contract',{}).get('enabled') is True))
        super().__init__()
        self.evidence=make_evidence_writer(self.run,self.profile);self.gyro_body=None;self.gyro_stamp=None;self.gyro_wall=None
        self.heading_gate=TeacherHeadingGate(self.profile)
        self.status_pub=RecordedPublisher(self.status_pub,run,self)
        self.bridge_updated=-math.inf
        self.guard_cloud=None;self.guard_cloud_receipt=None;self.pose_quaternion=None
        self.guard_sequence=0;self.pending_guard_row=None;self.trajectory_archive_reference=None
        self.last_actual_guard_clock_ns=None;self.guard_continuity_resets=0
        self.native_guard_audit=NativeGuardAudit(NATIVE_STEERING_GUARD)
        # This module instance belongs only to the independent Teacher process.
        # The shared camera-mode files and native geometry remain unchanged.
        # Native guard is called through an explicit subclass method; no globals are mutated.
    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        if topic in ('/demo/slam/body_odom','/cloud_registered_full','/livox/imu'):
            qos_profile=latest_sensor_qos()
        if topic=='/demo/control/safety':qos_profile=1
        return super().create_subscription(msg_type,topic,callback,qos_profile,*args,**kwargs)
    def declare_parameter(self,name,value=None,*args,**kwargs):
        if name=='scenario_path':value=str(self.run/'navigation_scenario.json')
        if name=='max_speed':value=self.profile['max_speed_mps']
        if name in ('pose_timeout','cloud_timeout'):value=self.profile['pose_cloud_timeout_s']
        return super().declare_parameter(name,value,*args,**kwargs)
    def on_bridge_safety(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        if (data.get('mode')!=self.profile['controller_kind'] or data.get('source')!='scan_slam'
            or data.get('state')not in ('hold','ready','failed')):return
        try:
            if not -.05<=time.monotonic()-float(data['monotonic_wall'])<.3:return
        except (ValueError,TypeError,KeyError):return
        self.bridge_updated=time.monotonic();self.bridge_safety=data
        if self.state=='running'and self.apply_tilt_guard(self.bridge_updated,self.get_clock().now().nanoseconds/1e9):
            self.reset_region_arrival('protected')
            # Do not feed bridge hold -> repeated zero -> bridge status back
            # into another zero. A moving command still stops immediately.
            if any(self.command):self.publish_command()
            if self.state=='failed':self.publish_status()
    def on_imu(self,msg):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9;q=msg.orientation
        accepted=self.raw_imu.update([q.x,q.y,q.z,q.w],self.stamp(msg)/1e9,now,ros_now,
                                     msg.orientation_covariance[0]>=0)
        if accepted:
            self.counts['imu']+=1;v=msg.angular_velocity
            gyro=self.imu_body_rotation@np.array([v.x,v.y,v.z])
            if np.isfinite(gyro).all():
                self.gyro_body=gyro.tolist();self.gyro_stamp=self.stamp(msg)/1e9;self.gyro_wall=now
        if self.state=='running'and self.apply_tilt_guard(now,ros_now):
            self.reset_region_arrival('protected')
            # Protection still stops a moving robot in this very callback.
            # Already-zero held commands use the unchanged 20Hz timer heartbeat.
            if any(self.command):self.publish_command()
            if self.state=='failed':self.publish_status()
    def apply_tilt_guard(self,now,ros_now):
        # A communication hold is not measured tilt and must not reset a
        # parked Teacher's heading/path every time a fresh cloud is delayed.
        # Physical IMU/SLAM tilt protections retain the original thresholds.
        slam_tilt=body_tilt(self.rotation);fresh=self.raw_imu.fresh(now,ros_now)
        sources=[('SLAM',slam_tilt)]+([('raw_imu',self.raw_imu.tilt)]if fresh else [])
        self.max_tilt=max(self.max_tilt,*(tilt for _,tilt in sources))
        severe=[name for name,tilt in sources if tilt>=.50]
        if self.bridge_safety.get('state')=='failed':severe.append('execution_bridge')
        if self.evidence.error:severe.append('evidence_writer')
        if severe:
            self.state='failed';self.tilt_source='+'.join(severe);self.message=self.tilt_source+'保护已锁定，停车'
            return True
        high=[name for name,tilt in sources if tilt>=.30]
        if high:
            if not self.tilt_hold:self.tilt_stops+=1
            self.tilt_hold=True;self.tilt_source='+'.join(high);self.tilt_clear_since=None
        if not fresh:
            self.message='原始IMU超时，停车';self.tilt_clear_since=None;return True
        if self.tilt_hold:
            if all(tilt<.18 for _,tilt in sources)and now-self.pose_updated<self.pose_timeout:
                if self.tilt_clear_since is None:self.tilt_clear_since=ros_now
                if ros_now-self.tilt_clear_since>=.8:
                    self.tilt_hold=False;self.tilt_source=None;self.heading_gate.reset()
                    self.samples=None;self.last_reference=-math.inf
            else:self.tilt_clear_since=None
            self.message='实际倾角保护，停车等待稳定';return True
        if now-self.bridge_updated>.30 or self.bridge_safety.get('state')!='ready':
            self.message='Teacher 执行命令门或 SLAM 传感器预热未就绪，停车'
            return True
        self.tilt_source=None;return False
    def on_odom(self,msg):
        stamp=self.stamp(msg);ns=self.get_clock().now().nanoseconds
        received=time.monotonic();q=msg.pose.pose.orientation
        v=msg.twist.twist.linear;w=msg.twist.twist.angular;p=msg.pose.pose.position
        if (msg.header.frame_id!='camera_init' or msg.child_frame_id!='demo_slam_body' or stamp<=self.pose_stamp
            or not -.05e9<=ns-stamp<self.pose_timeout*1e9
            or not .98<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.02
            or not all(math.isfinite(x)for x in (p.x,p.y,p.z,v.x,v.y,v.z,w.x,w.y,w.z))):return
        super().on_odom(msg)
        if self.pose_stamp==stamp:self.pose_quaternion=[q.x,q.y,q.z,q.w]
        if self.gyro_body is not None and received-self.gyro_wall<.3:
            self.heading_gate.observe(stamp/1e9,received,[v.x,v.y,v.z],w.z,self.gyro_body,self.gyro_stamp,not any(self.command))
        row={'stamp_ns':stamp,'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
            'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w],
            'body_velocity':[v.x,v.y,v.z],'body_angular_velocity':[w.x,w.y,w.z],
            'received_monotonic_wall':received,'callback_ros_clock_ns':ns,'sim_age_at_callback_s':(ns-stamp)/1e9,
            'body_gyro':self.gyro_body,'gyro_stamp_s':self.gyro_stamp,'zero_velocity_command':not any(self.command),
            'measured_stop_evidence':self.heading_gate.evidence(ns/1e9,received)}
        self.evidence.append(self.run/'navigation_slam_poses.jsonl',row)
    def on_cloud(self,msg):
        stamp=self.stamp(msg);ns=self.get_clock().now().nanoseconds;received=time.monotonic()
        if stamp<=self.cloud_stamp:
            self.note_cloud_decision('rejected_duplicate_or_backward_header',decision_ros_clock_ns=ns,
                previous_accepted_stamp_ns=int(self.cloud_stamp));return
        if not -.05e9<=ns-stamp<self.cloud_timeout*1e9:
            self.note_cloud_decision('rejected_header_age',decision_ros_clock_ns=ns,
                actual_header_age_ns=ns-stamp,timeout_s=self.cloud_timeout);return
        super().on_cloud(msg)
        if self.cloud_stamp==stamp and self.cloud_input_context is not None:
            c=self.cloud_input_context
            row={'stamp_ns':stamp,'frame_id':msg.header.frame_id,'input_points':int(msg.width*msg.height),
                'filtered_points':c['filtered_points'],'self_filtered_points':c['self_filtered_points'],
                'nearest_slam_pose_stamp_ns':c['filtering_body_stamp_ns'],
                'received_monotonic_wall':received,'decoded_monotonic_wall':time.monotonic(),
                'callback_ros_clock_ns':ns,'sim_age_at_callback_s':(ns-stamp)/1e9,
                'filtered_xyz_float64_sha256':hashlib.sha256(np.asarray(self.cloud,dtype='<f8').tobytes()).hexdigest()}
            self.guard_cloud=self.cloud
            self.guard_cloud_receipt={**row,'filtering_body_pose':c['filtering_body_pose'].tolist(),
                'filtering_body_rotation':c['filtering_body_rotation'].tolist()}
            self.evidence.append(self.run/'navigation_cloud_history.jsonl',row)
    def note_cloud_decision(self,reason,**details):
        # A read-only hook into the actual rejection/acceptance branch. It never
        # retries a message, decodes another cloud, or changes a freshness gate.
        row=getattr(self,'pending_cloud_callback_row',None)
        if row is not None:row.update(reason=reason,decision_details=details)
    def accept_spline(self,msg,metadata):
        before=self.replans;super().accept_spline(msg,metadata)
        if self.replans==before or self.samples is None:return
        directory=self.run/'navigation_trajectories'
        array_path=directory/f'{self.replans:06d}_trajectory_{self.active_trajectory_id}.npz'
        coefficients=np.array([[p.x,p.y,p.z]for p in msg.pos_pts]);knots=np.array(msg.knots);samples=self.samples.copy()
        document={'trajectory_id':self.active_trajectory_id,'order':int(msg.order),'metadata':metadata,
            'reference_stamp':self.active_reference_stamp,'request_id':self.request_id,
            'waypoint_index':self.waypoint_index,'accepted_slam_stamp_ns':self.pose_stamp,
            'array_file':array_path.name,
            'source':'actual SCAN committed B-spline and associated metadata; sensor SLAM feedback'}
        self.trajectory_archive_reference=str(array_path.relative_to(self.run))
        def archive():
            directory.mkdir(exist_ok=True);np.savez_compressed(array_path,coefficients=coefficients,knots=knots,samples=samples)
            document['array_sha256']=hashlib.sha256(array_path.read_bytes()).hexdigest()
            array_path.with_suffix('.json').write_text(json.dumps(document,allow_nan=False,ensure_ascii=False)+'\n')
        self.evidence.enqueue(archive)
    def on_request(self,msg):
        try:
            requested=json.loads(msg.data);frozen=json.loads((self.run/'navigation_request.json').read_text())
            if requested!=frozen:raise ValueError('Only this run frozen SLAM-relative route is eligible')
        except (OSError,ValueError,TypeError)as error:
            self.last_rejected=str(error);self.publish_status();return
        super().on_request(msg)
    def measured_region_arrival(self):
        inside,arrived=super().measured_region_arrival()
        if self.profile.get('arrival_stop_policy')=='after_measured_dwell':
            # The original raw-SLAM ArrivalWindow, protection flags, integer
            # stamps, observation gap and dwell remain unchanged. Continue only
            # the checked SCAN route until that same window confirms arrival;
            # all native path-end/obstacle/tilt/stale stops still run below.
            return inside and arrived,arrived
        return inside,arrived
    def control(self,now):
        # Shared recovery already requires one clear second and a new checked
        # trajectory; additionally hold it until measured body motion is small.
        if self.evidence.error:
            self.state='failed';self.message=self.evidence.error;self.publish_command();return
        if self.profile.get('clear_guard_gap_max_sim_s')is not None and self.obstacle_hold:
            ns=self.get_clock().now().nanoseconds
            if not self.guard_continuity(ns,now):
                # Keep the existing physical protections active even when an
                # extra actual-source freshness check holds this callback.
                self.apply_tilt_guard(now,ns/1e9);self.reset_region_arrival('stale')
                self.stale_since=self.stale_since or now
                if now-self.stale_since>8.:
                    self.state='failed';self.message='实际SLAM/点云guard输入连续失联超过8秒，停车'
                self.publish_command();return
            if (self.obstacle_clear_since is not None and ns/1e9-self.obstacle_clear_since>=1.
                    and not now-self.last_obstacle_check>.10):
                # A cached clear result cannot finish the original one-second
                # gate. Wait for the next already scheduled native evaluation;
                # do not run another geometry check or refresh input stamps.
                protected=self.apply_tilt_guard(now,ns/1e9)
                stale=(now-self.pose_updated>self.pose_timeout or now-self.cloud_updated>self.cloud_timeout
                       or not self.raw_imu.fresh(now,ns/1e9))
                if protected or stale:
                    self.reset_region_arrival('protected'if protected else 'stale')
                    self.obstacle_clear_since=None
                self.publish_command();return
        if self.obstacle_hold and not self.heading_gate.stopped(self.get_clock().now().nanoseconds/1e9,now):
            self.obstacle_clear_since=None
        self.pending_guard_row=None
        try:super().control(now)
        finally:
            if self.pending_guard_row is not None:
                row=self.pending_guard_row;after_ns=self.get_clock().now().nanoseconds
                row.update(obstacle_hold_after=bool(self.obstacle_hold),clear_start_after_s=self.obstacle_clear_since,
                    clear_elapsed_after_s=None if self.obstacle_clear_since is None else after_ns/1e9-self.obstacle_clear_since,
                    after_control_ros_clock_ns=after_ns,command_after_control=list(self.command),
                    zero_requested_after_control=not any(self.command),command_ros_sim_time_s=self.last_command_time,
                    state_after_control=self.state,obstacle_resumes_after=self.obstacle_resumes,
                    command_source='actual sensor SLAM/SCAN controller; no mover or simulator pose navigation input',
                    evidence_queue_error=self.evidence.error)
                self.evidence.append(self.run/'navigation_guard_history.jsonl',row)
                self.pending_guard_row=None
        if (self.state=='running' and not getattr(self,'reference_consistency_hold',False)
                and self.heading_gate.phase in ('pre_turn','settle')):
            self.message='Teacher 等待实际SLAM机身速度和IMU角速确认停车，保持零速度'

    def record_native_guard(self,cloud,pose,checked_target,steering_direction,route):
        ns=self.get_clock().now().nanoseconds;received=time.monotonic()
        previous_guard_ns=self.last_actual_guard_clock_ns
        if self.profile.get('clear_guard_gap_max_sim_s')is not None:
            if previous_guard_ns is not None and ns<=previous_guard_ns:
                self.state='failed';self.message='实际native guard时钟重复或倒退，停车'
                raise RuntimeError(self.message)
            if self.obstacle_hold and not self.guard_continuity(ns,received):
                raise RuntimeError('Actual native guard source ceased being fresh before evaluation')
        if (cloud is not self.guard_cloud or self.guard_cloud_receipt is None
                or self.guard_cloud_receipt['stamp_ns']!=self.cloud_stamp or self.pose_quaternion is None):
            raise RuntimeError('Actual native guard lacks an exact accepted cloud/SLAM input receipt')
        result=self.native_guard_audit(cloud,pose,checked_target,steering_direction,route)
        self.last_actual_guard_clock_ns=ns
        self.guard_sequence+=1;c=self.guard_cloud_receipt
        self.pending_guard_row={'schema':'teacher_native_steering_guard/v1','sequence':self.guard_sequence,
            'compute_ros_clock_ns':ns,'compute_sim_time_s':ns/1e9,'compute_monotonic_wall':received,
            'request_id':self.request_id,'waypoint_index':self.waypoint_index,
            'trajectory_id':self.active_trajectory_id,'reference_stamp':self.active_reference_stamp,
            'trajectory_archive_file':self.trajectory_archive_reference,
            'control_pose_stamp_ns':int(self.pose_stamp),'control_pose':np.asarray(pose).tolist(),
            'control_quaternion':list(self.pose_quaternion),'control_rotation':self.rotation.tolist(),
            'tracking_pose':self.tracking_pose.tolist(),'control_pose_received_monotonic_wall':self.pose_updated,
            'cloud_header_stamp_ns':int(c['stamp_ns']),'cloud_frame_id':c['frame_id'],
            'cloud_received_monotonic_wall':c['received_monotonic_wall'],
            'cloud_callback_ros_clock_ns':c['callback_ros_clock_ns'],
            'cloud_filtering_body_stamp_ns':c['nearest_slam_pose_stamp_ns'],
            'cloud_filtering_body_pose':c['filtering_body_pose'],'cloud_filtering_body_rotation':c['filtering_body_rotation'],
            'filtered_xyz_float64_sha256':c['filtered_xyz_float64_sha256'],
            'filtered_points':c['filtered_points'],'self_filtered_points':c['self_filtered_points'],
            'checked_target':np.asarray(checked_target).tolist(),'steering_direction':np.asarray(steering_direction).tolist(),
            'route':np.asarray(route).tolist(),'route_float64_sha256':hashlib.sha256(np.asarray(route,dtype='<f8').tobytes()).hexdigest(),
            'goal':self.waypoints[self.waypoint_index].tolist(),
            'goal_corridor_result':result_json(self.native_guard_audit.calls[0][1]),
            'motion_corridor_result':result_json(self.native_guard_audit.calls[1][1]),
            'motion_corridor_target':self.native_guard_audit.calls[1][0].tolist(),
            'union_result':result_json(result),'obstacle_hold_before':bool(self.obstacle_hold),
            'clear_start_before_s':self.obstacle_clear_since,'clear_elapsed_before_s':None if self.obstacle_clear_since is None else ns/1e9-self.obstacle_clear_since,
            'native_geometry_source':'unchanged control_core.steering_obstacle_ahead code object and original two obstacle_ahead evaluations'}
        if self.profile.get('clear_guard_gap_max_sim_s')is not None:
            self.pending_guard_row.update(previous_actual_guard_clock_ns=previous_guard_ns,
                clear_guard_gap_max_sim_s=self.profile['clear_guard_gap_max_sim_s'],
                clear_guard_continuity_resets=self.guard_continuity_resets,
                clear_release_requires_this_callback_actual_guard=True)
        return result

    def guard_continuity(self,ns,wall):
        """V44 only: original clear timer cannot span missing actual checks."""
        maximum_ns=round(self.profile['clear_guard_gap_max_sim_s']*1e9)
        previous=self.last_actual_guard_clock_ns
        if previous is not None and ns<previous:
            self.state='failed';self.message='实际native guard时钟倒退，停车';self.obstacle_clear_since=None
            return False
        reset=previous is None or ns-previous>maximum_ns
        fresh_ns=round(self.profile['pose_cloud_timeout_s']*1e9)
        source_fresh=all(stamp>=0 and -50_000_000<=ns-stamp<fresh_ns for stamp in (self.pose_stamp,self.cloud_stamp))
        if reset or not source_fresh:
            before=self.obstacle_clear_since;self.obstacle_clear_since=None
            if before is not None:
                self.guard_continuity_resets+=1
                self.evidence.append(self.run/'navigation_guard_continuity_history.jsonl',{
                    'schema':'teacher_actual_guard_continuity/v1','compute_ros_clock_ns':ns,
                    'compute_monotonic_wall':wall,'previous_actual_guard_clock_ns':previous,
                    'actual_guard_gap_ns':None if previous is None else ns-previous,
                    'pose_stamp_ns':self.pose_stamp,'cloud_stamp_ns':self.cloud_stamp,
                    'clear_start_before_s':before,'clear_start_after_s':None,
                    'reason':'actual_source_stale'if not source_fresh else 'actual_guard_gap',
                    'extra_geometry_computed':False})
        if not source_fresh:self.message='实际SLAM/点云header超时，停车等待fresh guard输入'
        return source_fresh


def main():
    import rclpy
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();os.environ['DEMO_RUN_DIR']=str(run)
    rclpy.init(args=ros);node=TeacherNavigation(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publish_command()
        node.event_archive.close()
        try:node.evidence.close()
        finally:
            if node.profile.get('dynamic_fixture'):
                (run/'navigation_guard_writer_receipt.json').write_text(json.dumps({
                    'schema':'teacher_native_steering_guard_writer/v1','expected_records':node.guard_sequence,
                    'status':'failed'if node.evidence.error else 'drained',
                    'queue_error':node.evidence.error,'silent_record_loss_permitted':False,
                    'clear_guard_continuity_resets':node.guard_continuity_resets,
                    'source':'actual dynamic controller shutdown after bounded evidence queue drain',
                    'navigation_verified':False},allow_nan=False,ensure_ascii=False)+'\n')
            node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
