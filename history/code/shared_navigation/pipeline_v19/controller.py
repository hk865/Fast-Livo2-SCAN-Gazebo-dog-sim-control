#!/usr/bin/env python3
"""Real SLAM/IMU geometric cascade, protected by original SCAN cloud guards."""
import argparse,copy,hashlib,json,math,os,time
from collections import deque
from pathlib import Path
import numpy as np
from teacher_wrapper import TeacherNavigation
from pid_core import slew
from cascade_core import Controller
from control_core import follow_trajectory
from clock_hold import ControlClockHold

class ResetBookkeeping:
    """Legacy archive reset counter only; cascade_core owns all control maths."""
    def __init__(self):
        self.resets=0
        self.reset('startup')
    def reset(self,reason):
        self.resets+=1
        self.reset_reason=reason


class PIDNavigation(TeacherNavigation):
    def __init__(self,run):
        super().__init__(run);self.pid=ResetBookkeeping()  # no legacy controller mathematics
        self.pid_velocity=None;self.pid_pose_stamp=None;self.pid_row=None
        self.pid_records=0;self.pid_guard_needs_evaluation=False;self.pid_prepared_output=None
        self.cloud_callback_records=0;self.pending_cloud_callback_row=None
        self.imu_history=deque(maxlen=512);self.feedback=None;self.paired_imu=None
        self.cascade=None;self.cascade_key=None;self.cascade_last_row=None
        self.path_receipt=None;self.cascade_path_id=None;self.final_bundle=None
        self.imu_records=0;self.cloud_archive_records=0
        self.segment_references={}
        self.control_clock_hold=ControlClockHold()
        # PUBLICATION_OBSERVER_BEGIN init_counter
        self.publication_records=0
        # PUBLICATION_OBSERVER_END init_counter
    def on_request(self,msg):
        previous=self.request_id
        super().on_request(msg)
        if self.request_id!=previous and hasattr(self,'cascade'):
            # Each stage has its own measured path and PI state. Source/IMU
            # histories and the immutable original registration remain valid.
            self.cascade=None;self.cascade_key=None;self.cascade_last_row=None
            self.cascade_path_id=None;self.path_receipt=None;self.final_bundle=None
            self.pid_row=None;self.pid_prepared_output=None
            self.publish_command()
    def on_imu(self,msg):
        before=self.gyro_stamp;wall=time.monotonic_ns();clock=self.get_clock().now().nanoseconds
        super().on_imu(msg);stamp=int(self.stamp(msg));q=msg.orientation;v=msg.angular_velocity
        if self.gyro_stamp is not None and round(self.gyro_stamp*1e9)==stamp and self.gyro_stamp!=before and stamp<=clock:
            row=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='body',
                angular_velocity_body=list(self.gyro_body),source_frame_id=msg.header.frame_id,
                gyro_source_xyz=[v.x,v.y,v.z],orientation_xyzw=[q.x,q.y,q.z,q.w],
                orientation_covariance=list(msg.orientation_covariance),
                imu_to_body_rotation=self.imu_body_rotation.tolist(),callback_ros_clock_ns=clock,
                source_topic='/livox/imu',navigation_ground_truth_used=False)
            self.imu_history.append(row);self.imu_records+=1
            self.evidence.append(self.run/'navigation_imu_history.jsonl',{**row,'sequence':self.imu_records})
    def on_odom(self,msg):
        stamp=int(self.stamp(msg));clock=self.get_clock().now().nanoseconds;wall=time.monotonic_ns()
        if stamp>clock:return
        paired=next((r for r in reversed(self.imu_history)if 0<=stamp-r['stamp_ns']<=20_000_000),None)
        latest=(self.gyro_body,self.gyro_stamp,self.gyro_wall)
        if paired:
            self.gyro_body=paired['angular_velocity_body'];self.gyro_stamp=paired['stamp_ns']/1e9
            self.gyro_wall=paired['received_wall_ns']/1e9
        else:self.gyro_body=None
        before=self.pose_stamp
        try:super().on_odom(msg)
        finally:self.gyro_body,self.gyro_stamp,self.gyro_wall=latest
        if self.pose_stamp>before:
            q=msg.pose.pose.orientation;v=msg.twist.twist.linear
            self.feedback=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='camera_init',
                position_world_xyz=self.pose.tolist(),quaternion_wxyz=[q.w,q.x,q.y,q.z],
                origin_velocity_body=[v.x,v.y,v.z],callback_ros_clock_ns=clock)
            self.paired_imu=copy.deepcopy(paired)
            self.pid_velocity=self.rotation@np.array([v.x,v.y,v.z]);self.pid_pose_stamp=stamp
            self.evidence.append(self.run/'navigation_feedback_history.jsonl',dict(
                **self.feedback,paired_imu=copy.deepcopy(paired),navigation_ground_truth_used=False))
    def on_cloud(self,msg):
        wall=time.monotonic();clock=self.get_clock().now().nanoseconds;before=self.cloud_stamp
        self.cloud_callback_records+=1
        self.pending_cloud_callback_row=dict(schema='pid_actual_cloud_callback/v1',sequence=self.cloud_callback_records,
            producer_stamp_ns=int(self.stamp(msg)),frame_id=msg.header.frame_id,received_monotonic_wall=wall,
            received_ros_clock_ns=clock,source_topic='/cloud_registered_full',
            input_width=int(msg.width),input_height=int(msg.height),previous_accepted_stamp_ns=int(before),
            reason=None,navigation_ground_truth_used=False)
        try:
            super().on_cloud(msg)
            if self.cloud_stamp>before:self.archive_cloud(msg)
        finally:
            row=self.pending_cloud_callback_row;end=time.monotonic()
            row.update(finished_monotonic_wall=end,callback_duration_wall_s=end-wall,
                finished_ros_clock_ns=self.get_clock().now().nanoseconds,
                accepted=row['reason']=='accepted',accepted_stamp_ns=int(self.cloud_stamp),
                accepted_cloud_received_monotonic_wall=self.cloud_updated if math.isfinite(self.cloud_updated)else None)
            if row['reason']is None:row['reason']='callback_returned_without_branch_receipt'
            self.evidence.append(self.run/'navigation_cloud_callbacks.jsonl',row);self.pending_cloud_callback_row=None
    def archive_cloud(self,msg):
        self.cloud_archive_records+=1;seq=self.cloud_archive_records
        xyz=np.array(self.guard_cloud,dtype='<f8',copy=True);raw=bytes(msg.data)
        directory=self.run/'navigation_cloud_arrays';name=f'{seq:06d}_{self.cloud_stamp}'
        array=directory/(name+'.npy');payload=directory/(name+'.bin')
        record=dict(schema='actual_registered_cloud_xyz_archive/v1',sequence=seq,
            stamp_ns=int(self.cloud_stamp),frame_id=msg.header.frame_id,
            received_monotonic_wall=self.guard_cloud_receipt['received_monotonic_wall'],
            array_file=str(array.relative_to(self.run)),raw_payload_file=str(payload.relative_to(self.run)),
            filtered_xyz_float64_sha256=hashlib.sha256(xyz.tobytes()).hexdigest(),
            raw_payload_sha256=hashlib.sha256(raw).hexdigest(),dtype='<f8',shape=list(xyz.shape),
            fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count)for f in msg.fields],
            width=int(msg.width),height=int(msg.height),point_step=int(msg.point_step),row_step=int(msg.row_step),
            is_bigendian=bool(msg.is_bigendian),is_dense=bool(msg.is_dense),
            filtering_body_stamp_ns=self.guard_cloud_receipt['nearest_slam_pose_stamp_ns'],
            filtering_body_pose=self.guard_cloud_receipt['filtering_body_pose'],
            filtering_body_rotation=self.guard_cloud_receipt['filtering_body_rotation'],
            navigation_ground_truth_used=False)
        self.guard_cloud_receipt.update(cloud_array_file=record['array_file'],
            cloud_raw_payload_file=record['raw_payload_file'],cloud_archive_sequence=seq)
        def archive():
            directory.mkdir(exist_ok=True);np.save(array,xyz,allow_pickle=False);payload.write_bytes(raw)
            document={**record,'array_sha256':hashlib.sha256(array.read_bytes()).hexdigest()}
            with (self.run/'navigation_cloud_xyz.jsonl').open('a')as out:out.write(json.dumps(document,allow_nan=False)+'\n')
        self.evidence.enqueue(archive)
    def accept_spline(self,msg,metadata):
        if self.cascade is not None and self.cascade_key==(self.request_id,self.waypoint_index) and self.cascade.capture_stamp is not None:
            self.last_spline_rejected='固定终点捕获阶段保留已检查SCAN路径，实际点云guard继续工作'
            return
        before=self.replans;wall=time.monotonic_ns();super().accept_spline(msg,metadata)
        if self.replans>before and self.samples is not None:
            indices=[0]
            for i in range(1,len(self.samples)):
                if np.linalg.norm(self.samples[i,:2]-self.samples[indices[-1],:2])>=1e-5:indices.append(i)
            if len(indices)<2:
                self.samples=None;self.last_reference=-math.inf
                self.last_spline_rejected='实际SCAN未提供非退化水平路径，停车等待新规划';return
            points=self.samples[indices]
            self.path_receipt=dict(points_xyz=points.tolist(),
                source_sample_indices=indices,source_samples_count=len(self.samples),
                source_samples_float64_sha256=hashlib.sha256(np.asarray(self.samples,dtype='<f8').tobytes()).hexdigest(),
                path_id=str(self.request_id)+':'+str(self.waypoint_index)+':'+str(self.replans)+':'+str(self.active_trajectory_id),
                stamp_ns=int(self.trajectory_association.reference_stamp[0])*1_000_000_000+int(self.trajectory_association.reference_stamp[1]),
                received_wall_ns=wall,frame_id='camera_init',array_file=self.trajectory_archive_reference,
                request_id=self.request_id,waypoint_index=self.waypoint_index,
                trajectory_id=self.active_trajectory_id,metadata=copy.deepcopy(metadata))
            self.path_receipt['points_float64_sha256']=hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()).hexdigest()
            self.evidence.append(self.run/'cascade_path_history.jsonl',copy.deepcopy(self.path_receipt))
    def follow_checked_trajectory(self,*args,**kwargs):
        kwargs['tracking_pose']=self.pose
        result=follow_trajectory(*args,**kwargs)
        if not kwargs.get('return_steering'):return result
        self.ensure_cascade()
        # Preview the same immutable SCAN projection without committing source
        # progress or integrating a timer duplicate. Both heading owners use it.
        preview=copy.deepcopy(self.cascade)
        segment,point,tangent,normal,cross,grade,nearest_s=preview._project(self.pose)
        remaining=float(preview.cumulative[-1]-preview.progress)
        yaw=math.atan2(tangent[1],tangent[0])
        if np.linalg.norm(preview.path[-1]-preview.goal)<=.2 and remaining<=.15:
            yaw=preview.goal_yaw
        velocity,rate,target,steering=result
        original=copy.deepcopy(steering)
        steering=dict(steering,heading=yaw,error=math.atan2(math.sin(yaw-math.atan2(self.rotation[1,0],self.rotation[0,0])),
            math.cos(yaw-math.atan2(self.rotation[1,0],self.rotation[0,0]))),
            heading_reference='same actual SCAN nearest directed 3D projection as cascade',
            original_lookahead_steering=original,
            cascade_projection=dict(path_id=preview.path_id,path_sha256=preview.path_sha,segment=segment,
                point=point.tolist(),tangent=tangent.tolist(),nearest_s=nearest_s,remaining=remaining,
                actual_pose_stamp_ns=int(self.pose_stamp),navigation_ground_truth_used=False))
        self.evidence.append(self.run/'navigation_heading_projection.jsonl',dict(
            request_id=self.request_id,waypoint_index=self.waypoint_index,
            source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds,
            steering=copy.deepcopy(steering),feedback=copy.deepcopy(self.feedback),
            navigation_ground_truth_used=False))
        return velocity,rate,target,steering
    def measured_region_arrival(self):
        key=(self.request_id,self.waypoint_index)
        if key not in self.segment_references:
            if self.segment_start is None or not np.array_equal(self.segment_start,self.pose):
                raise RuntimeError('Actual segment activation lacks its original SLAM pose')
            self.segment_references[key]=dict(segment_start=self.segment_start.tolist(),
                source_pose_stamp_ns=int(self.segment_start_pose_stamp_ns),control_stamp_ns=int(self.segment_started_ros_ns),
                feedback=copy.deepcopy(self.feedback),request_id=self.request_id,waypoint_index=self.waypoint_index)
            self.evidence.append(self.run/'navigation_segment_activation.jsonl',dict(
                **self.segment_references[key],navigation_ground_truth_used=False))
        return super().measured_region_arrival()
    def ensure_cascade(self):
        key=(self.request_id,self.waypoint_index)
        if self.cascade_key!=key:
            goal=self.waypoints[self.waypoint_index];delta=goal[:2]-self.segment_start[:2]
            if np.linalg.norm(delta)<1e-6:raise ValueError('Frozen goal heading requires a nondegenerate actual SLAM segment')
            yaw=math.atan2(delta[1],delta[0])
            self.cascade=Controller(self.profile['cascade'],goal.tolist(),yaw,':'.join(map(str,key)))
            self.cascade_key=key;self.cascade_path_id=None
            self.evidence.append(self.run/'navigation_fixed_goal_references.jsonl',dict(
                request_id=self.request_id,waypoint_index=self.waypoint_index,
                segment_start=self.segment_start.tolist(),goal=goal.tolist(),heading_rad=yaw,
                source_pose_stamp_ns=self.segment_references[key]['source_pose_stamp_ns'],
                control_stamp_ns=self.segment_references[key]['control_stamp_ns'],
                goals_definition_sha256=self.goals_definition_sha256,fixed_goal=self.cascade.goal_record,
                fixed_goal_sha256=self.cascade.goal_sha,navigation_ground_truth_used=False))
        if (self.cascade.capture_stamp is None and self.path_receipt and
            (self.path_receipt['request_id'],self.path_receipt['waypoint_index'])==key and
            self.path_receipt['path_id']!=self.cascade_path_id):
            r=self.path_receipt
            self.cascade.set_path(r['points_xyz'],r['path_id'],r['stamp_ns'],r['received_wall_ns'],r['frame_id'])
            self.cascade_path_id=r['path_id']
            self.evidence.append(self.run/'navigation_cascade_paths.jsonl',dict(
                **self.cascade.path_record,path_sha256=self.cascade.path_sha,
                trajectory_id=r['trajectory_id'],array_file=r['array_file'],
                source_sample_indices=r['source_sample_indices'],source_samples_count=r['source_samples_count'],
                source_samples_float64_sha256=r['source_samples_float64_sha256'],
                points_float64_sha256=r['points_float64_sha256'],navigation_ground_truth_used=False))
    def get_ack(self):
        if not self.feedback:return None
        try:
            d=json.loads((self.run/'closed_loop_executor_ack.json').read_text())
            return next((r for r in reversed(d['entries'])if r['stamp_ns']<=self.feedback['stamp_ns']),None)
        except (OSError,KeyError,ValueError,TypeError):return None
    def select_pid_velocity(self,velocity,yaw_rate,target,allow_translation,mode_override=None):
        ns=self.get_clock().now().nanoseconds;wall=time.monotonic_ns();self.ensure_cascade()
        mode=mode_override or ('hold'if self.obstacle_hold or self.heading_gate.phase in ('pre_turn','settle')
            else 'drive'if allow_translation else 'turn')
        heading_reference=copy.deepcopy(self.steering);gate_heading=self.heading_gate.heading
        ack=self.get_ack();guard=None
        if self.feedback is None or self.paired_imu is None or ack is None:guard='actual_causal_feedback_or_ack_missing'
        cmd,diagnostic=self.cascade.update(self.feedback,self.paired_imu,ack,ns,wall,guard_reason=guard,mode_override=mode)
        if diagnostic.get('failure_latched'):
            self.state='failed';self.message='串级真实输入保护锁定：'+str(diagnostic['failure_latched'])
        self.cascade_last_row=diagnostic;self.pid_guard_needs_evaluation=(bool(diagnostic.get('controller_updated'))
            or self.control_clock_hold.guard_required)
        desired=cmd.copy()
        cmd=(np.zeros(3)if diagnostic['mode']in ('protect','recovering','pre_turn','settle','path_end_hold')
            else slew(self.command,desired,ns/1e9-self.last_command_time,[.6,.6,.8]))
        self.pid_prepared_output=cmd.copy();world_direction=self.rotation[:2,:2]@cmd[:2]
        if np.linalg.norm(world_direction)>1e-9:self.steering['direction']=world_direction.tolist()
        self.pid_records+=1
        self.pid_row=dict(schema='teacher_closed_loop_cascade_tick/v1',sequence=self.pid_records,
            cascade_sequence=self.pid_records,control_pose_stamp_ns=int(self.pose_stamp),source_pose_stamp_ns=int(self.pose_stamp),
            paired_imu_stamp_ns=None if self.paired_imu is None else self.paired_imu['stamp_ns'],
            control_stamp_ns=ns,compute_ros_clock_ns=ns,compute_monotonic_wall=wall/1e9,
            feedback=copy.deepcopy(self.feedback),imu=copy.deepcopy(self.paired_imu),ack=copy.deepcopy(ack),
            heading_gate_reference=dict(phase=self.heading_gate.phase,locked_heading=gate_heading,steering=heading_reference),
            path_receipt=None if self.path_receipt is None else {k:copy.deepcopy(v) for k,v in self.path_receipt.items()if k!='points_xyz'},cascade=diagnostic,
            control_pose=self.pose.tolist(),control_quaternion=self.pose_quaternion,control_rotation=self.rotation.tolist(),
            request_id=self.request_id,waypoint_index=self.waypoint_index,trajectory_id=self.active_trajectory_id,
            trajectory_archive_file=self.trajectory_archive_reference,mode=diagnostic['mode'],
            checked_target=np.asarray(target).tolist(),goal=self.waypoints[self.waypoint_index].tolist(),
            actual_pid_world_direction=world_direction.tolist(),steering_direction=self.steering['direction'],
            desired_body_command=desired.tolist(),prepared_after_slew_command=cmd.tolist(),
            navigation_ground_truth_used=False)
        return cmd[:2],float(cmd[2])
    def evaluate_motion_guard(self,*args):
        result=self.record_native_guard(*args)
        if self.pending_guard_row is not None:
            self.pending_guard_row.update(cloud_array_file=self.guard_cloud_receipt['cloud_array_file'],
                cloud_raw_payload_file=self.guard_cloud_receipt['cloud_raw_payload_file'],
                cloud_archive_sequence=self.guard_cloud_receipt['cloud_archive_sequence'])
            if self.pid_row:self.pending_guard_row.update(pid_sequence=self.pid_row['sequence'],
                cascade_sequence=self.pid_row['sequence'],pid_compute_ros_clock_ns=self.pid_row['compute_ros_clock_ns'],
                pid_control_pose_stamp_ns=self.pid_row['control_pose_stamp_ns'])
        self.pid_guard_needs_evaluation=False;self.control_clock_hold.guard_completed();return result
    def control(self,now):
        # The wall timer can run twice while real ROS /clock is unchanged.
        # Before any PI, heading, arrival or geometry operation, emit exact
        # zero and leave original source receipts untouched. On clock advance,
        # a new actual geometry check is mandatory before any nonzero command.
        if self.control_clock_hold.before_control(self,self.get_clock().now().nanoseconds,round(now*1e9)):
            return
        if self.profile.get('mission46_required'):
            from mission46_guard import mission_hold
            hold,reason=mission_hold(self.run,self.request_id,self.region_arrivals,
                self.goals_definition_sha256,self.get_clock().now().nanoseconds,now)
            if hold:
                self.message='Teacher46 transition: '+reason
                # A reached phase endpoint retains the existing feedback hold.
                # A connector pause is a zero VELOCITY request to the live
                # Teacher, never a zero action or a second joint controller.
                if self.state=='succeeded' and self.final_bundle is not None:
                    return self.control_parking(now)
                self.apply_tilt_guard(now,self.get_clock().now().nanoseconds/1e9)
                self.publish_command();return
        if self.state=='succeeded' and self.final_bundle is not None:return self.control_parking(now)
        if self.state=='running'and self.samples is not None and self.waypoint_index==len(self.waypoints)-1 and self.segment_start is not None:
            self.ensure_cascade()
            self.final_bundle=dict(index=self.waypoint_index,start=self.segment_start.copy(),
                trajectory_id=self.active_trajectory_id,array=self.trajectory_archive_reference,
                path_receipt=copy.deepcopy(self.path_receipt),reference_stamp=self.trajectory_association.reference_stamp)
        return super().control(now)
    def control_parking(self,now):
        ns=self.get_clock().now().nanoseconds
        bundle=self.final_bundle;index=self.waypoint_index;old_reference=self.trajectory_association.reference_stamp
        self.waypoint_index=bundle['index'];self.trajectory_association.reference_stamp=bundle['reference_stamp']
        self.active_trajectory_id=bundle['trajectory_id'];self.trajectory_archive_reference=bundle['array']
        self.path_receipt=bundle['path_receipt'];self.segment_start=bundle['start']
        goal=self.waypoints[self.waypoint_index]
        self.steering=dict(direction=(goal-self.pose)[:2].tolist(),heading=self.cascade.goal_yaw,
            heading_error=0.,exhausted=False)
        try:
            stale=(now-self.pose_updated>.3 or now-self.cloud_updated>.3 or self.paired_imu is None)
            if self.apply_tilt_guard(now,ns/1e9)or stale or self.obstacle_hold:
                self.select_pid_velocity(np.zeros(2),0.,goal,False,mode_override='hold')
                self.publish_command();return
            velocity,yawrate=self.select_pid_velocity(np.zeros(2),0.,goal,True,mode_override='capture')
            result=self.evaluate_motion_guard(self.cloud,self.pose,goal,self.steering['direction'],np.vstack([bundle['start'],goal]))
            if result[0]:
                self.message='终点实际点云保护停车'
                self.publish_command()  # Stop immediately and archive the guarded candidate.
                self.select_pid_velocity(np.zeros(2),0.,goal,False,mode_override='hold')
                self.publish_command()
            else:self.message='实际SLAM固定终点主动捕获/位置朝向保持';self.publish_command(velocity,yawrate)
            if self.pending_guard_row:
                self.pending_guard_row.update(command_after_control=list(self.command),zero_requested_after_control=not any(self.command),
                    state_after_control=self.state,after_control_ros_clock_ns=self.get_clock().now().nanoseconds,
                    obstacle_hold_after=self.obstacle_hold,evidence_queue_error=self.evidence.error)
                self.evidence.append(self.run/'navigation_guard_history.jsonl',self.pending_guard_row);self.pending_guard_row=None
        finally:
            self.waypoint_index=index;self.trajectory_association.reference_stamp=old_reference
    def publish_command(self,velocity=None,yaw_rate=0.):
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool
        # PUBLICATION_OBSERVER_BEGIN entry_snapshot
        from publication_ledger import PublicationLedger
        publication_before=PublicationLedger.begin(self,velocity is None)
        # PUBLICATION_OBSERVER_END entry_snapshot
        msg=Twist();ns=self.get_clock().now().nanoseconds
        self.last_publication_ros_clock_ns=ns
        if velocity is not None:
            output=np.r_[velocity,yaw_rate]
            if self.pid_prepared_output is None or not np.allclose(output,self.pid_prepared_output,rtol=0,atol=1e-12):
                raise RuntimeError('Cascade command changed after native guard')
            msg.linear.x,msg.linear.y,msg.angular.z=map(float,output)
        elif hasattr(self,'pid'):self.pid.reset('exact_zero_stop_or_protection')
        self.command=[msg.linear.x,msg.linear.y,msg.angular.z];self.last_command_time=ns/1e9
        row=getattr(self,'pid_row',None)
        if row is not None:
            row.update(publish_ros_clock_ns=ns,command_after_slew=list(self.command),state=self.state,
                obstacle_hold=self.obstacle_hold,alignment_hold=self.alignment_hold,stopped=velocity is None,
                reset_count_after_publish=self.pid.resets,queue_error=self.evidence.error)
            f=row['feedback'];i=row['imu']
            source=dict(schema='actual_SLAM_SCAN_cascade_command_source/v1',cascade_sequence=row['sequence'],
                source_pose_stamp_ns=row['source_pose_stamp_ns'],paired_imu_stamp_ns=row['paired_imu_stamp_ns'],
                control_stamp_ns=row['control_stamp_ns'],command_after_slew=list(self.command),navigation_ground_truth_used=False,
                source_pose_received_wall_ns=None if f is None else f['received_wall_ns'],
                source_imu_received_wall_ns=None if i is None else i['received_wall_ns'],
                source_pose_received_monotonic_wall=None if f is None else f['received_wall_ns']/1e9,
                paired_imu_received_monotonic_wall=None if i is None else i['received_wall_ns']/1e9,
                request_id=row['request_id'],trajectory_id=row['trajectory_id'],
                waypoint_index=row['waypoint_index'],
                goal_sha256=row['cascade']['fixed_goal_sha256'],path_sha256=row['cascade']['path_sha256'],
                cascade_mode=row['cascade']['mode'],
                parking_hold_declared_pose_stamp_ns=row['cascade']['parking_hold_declared_pose_stamp_ns'])
            tmp=self.run/'cascade_command_source.tmp';tmp.write_text(json.dumps(source,allow_nan=False)+'\n')
            tmp.replace(self.run/'cascade_command_source.json')
            self.evidence.append(self.run/'navigation_pid_history.jsonl',row);self.pid_row=None
        # PUBLICATION_OBSERVER_BEGIN actual_publish_start
        publication_started_wall_ns=PublicationLedger.publication_started()
        # PUBLICATION_OBSERVER_END actual_publish_start
        self.cmd_pub.publish(msg);self.counts['commands']+=1
        # PUBLICATION_OBSERVER_BEGIN actual_publish_completed
        PublicationLedger.completed(self,publication_before,ns,publication_started_wall_ns)
        # PUBLICATION_OBSERVER_END actual_publish_completed
        self.freeze_pub.publish(Bool(data=bool(self.state!='running'or self.obstacle_hold or np.linalg.norm(self.command[:2])<.015)))

def main():
    import rclpy
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();os.environ['DEMO_RUN_DIR']=str(run)
    rclpy.init(args=ros);node=PIDNavigation(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publish_command()
        try:node.event_archive.close();node.evidence.close()
        finally:
            (run/'navigation_pid_writer_receipt.json').write_text(json.dumps(dict(schema=1,
                expected_pid_records=node.pid_records,expected_guard_records=node.guard_sequence,
                expected_imu_records=node.imu_records,expected_cloud_archive_records=node.cloud_archive_records,
                expected_cloud_callback_records=node.cloud_callback_records,queue_error=node.evidence.error,
                expected_control_clock_hold_records=node.control_clock_hold.records,
                # PUBLICATION_OBSERVER_BEGIN writer_expected_count
                expected_command_publication_records=node.publication_records,
                final_commands_published_count=node.counts['commands'],
                # PUBLICATION_OBSERVER_END writer_expected_count
                status='failed'if node.evidence.error else 'drained',navigation_verified=False))+'\n')
            node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
