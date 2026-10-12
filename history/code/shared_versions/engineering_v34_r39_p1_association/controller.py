#!/usr/bin/env python3
"""Real SLAM/IMU geometric cascade, protected by original SCAN cloud guards."""
import argparse,copy,hashlib,json,math,os,time
from collections import deque
from pathlib import Path
import numpy as np
from teacher_wrapper import TeacherNavigation
from pid_core import slew
from cascade_core import Controller
from control_core import follow_trajectory, trajectory_has_progress
from scipy.interpolate import BSpline
from path_admission import FrozenFence, candidate, payload, replayable, certify_curve, remaining_certificate
from known_scene_route_registration import KnownSceneFrozenFence
from clock_hold import ControlClockHold
from global_route_reference import (contract as global_reference_contract, SegmentGuides,
    StableProjectionController, direction_check, admission_direction_check)

class ResetBookkeeping:
    """Legacy archive reset counter only; cascade_core owns all control maths."""
    def __init__(self):
        self.resets=0
        self.reset('startup')
    def reset(self,reason):
        self.resets+=1
        self.reset_reason=reason


from navigation.goal_regions import ArrivalWindow
from continuous_route import (configuration as continuous_configuration, RouteProgress,
    CausalHandoffProjectionController)


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
        self.global_reference_settings=global_reference_contract(self.profile)
        self.global_route_guides=SegmentGuides()
        self.continuous_settings=continuous_configuration(self.profile)
        self.continuous_route=None;self.coverage_replan_index=None
        self.scan_reference_coverage_index=None
        self.reference_consistency_hold=False;self.reference_consistency_check=None
        self.registered_cloud_archive_mode=self.profile.get('engineering_recording',{}).get('registered_cloud_archive','legacy_all')
        if self.registered_cloud_archive_mode not in ('legacy_all','disabled'):
            raise ValueError('Unsupported registered_cloud_archive mode; bounded_event is not yet implemented')
        self.reference_replan_pending=False
        self.path_admission_fence=KnownSceneFrozenFence(self.run,self.profile)
        self.path_admission_active=None;self.path_admission_hold=False;self.path_admission_replan_pending=False
        self.path_admission_sequence=0;self.path_admission_last_decision=None
        self.control_clock_hold=ControlClockHold()
        self.corridor_observer=None
        self.corridor_last_certificate=None
        self.corridor_last_request_wall=-math.inf
        self.corridor_snapshot_count=0
        corridor_config=self.profile.get('corridor',{'mode':'off'})
        if corridor_config.get('mode') not in ('off','shadow') or corridor_config.get('retain_valid_plan',False):
            raise ValueError('Unvalidated corridor control authority is not enabled in this revision')
        if corridor_config.get('mode')=='shadow':
            from corridor_runtime import CorridorObserver
            from nav_msgs.msg import Path as PathMsg
            from std_msgs.msg import String
            self.corridor_observer=CorridorObserver(dict(
                horizon_m=1.2,max_half_width_m=.30,width_step_m=.04,
                max_speed_mps=float(self.profile['max_speed_mps']),brake_decel_mps2=.15,
                latency_s=.3,stop_margin_m=.10,body_radius_m=.25,body_offset_m=.18,
                body_below_m=.12,body_above_m=.12,position_uncertainty_m=.08,
                yaw_half_range_rad=math.pi,map_max_age_ns=300_000_000,
                cell_max_age_ns=300_000_000,support_max_age_ns=300_000_000,
                body_ground_height_m=.32,support_height_tolerance_m=.10,max_support_slope_rad=.35))
            self.corridor_request_pub=self.create_publisher(PathMsg,'/demo/teacher/corridor/request',1)
            self.corridor_snapshot_sub=self.create_subscription(String,'/demo/teacher/corridor/snapshot',
                self.on_corridor_snapshot,1)
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
            self.reference_replan_pending=False
            self.path_admission_active=None;self.path_admission_hold=False;self.path_admission_replan_pending=False
            self.pid_row=None;self.pid_prepared_output=None
            self.reference_consistency_hold=False;self.reference_consistency_check=None
            self.publish_command()
            self.continuous_route=(RouteProgress(self.request_id,self.goals,
                self.goals_definition_sha256,self.continuous_settings)
                if self.continuous_settings["enabled"] else None)
            self.coverage_replan_index=None
    def on_corridor_snapshot(self,msg):
        from corridor_runtime import decode_snapshot,sha_text
        wall=time.monotonic_ns();clock=self.get_clock().now().nanoseconds
        if self.corridor_observer is None:return
        try:
            snapshot=decode_snapshot(msg.data)
            self.corridor_snapshot_count+=1
            if self.cascade is None or self.feedback is None or self.path_receipt is None:return
            reference=self.cascade.preview_reference(self.pose)
            # The route identity disambiguates overlapping floor paths; the
            # actual surface still has to be established by measured points.
            layer_id=str(self.request_id)+':'+str(self.waypoint_index)
            path=dict(points_xyz=self.cascade.path.tolist(),path_id=self.cascade.path_id,
                path_sha256=self.cascade.path_sha,frame_id='camera_init',layer_id=layer_id,
                stamp_ns=self.cascade.path_record['stamp_ns'])
            state=dict(position_world_xyz=self.pose.tolist(),
                yaw_rad=math.atan2(self.rotation[1,0],self.rotation[0,0]),
                speed_mps=float(np.linalg.norm(self.feedback['origin_velocity_body'][:2])),
                progress_m=reference['nearest_s_m'],stamp_ns=int(self.pose_stamp))
            digest=sha_text(msg.data)
            if self.corridor_observer.submit(snapshot,path,state,clock,wall,digest):
                import gzip
                raw=msg.data.encode('utf-8')
                relative=Path('corridor_snapshots')/(str(self.corridor_snapshot_count).zfill(6)+'.json.gz')
                def archive():
                    target=self.run/relative;target.parent.mkdir(exist_ok=True)
                    target.write_bytes(gzip.compress(raw,compresslevel=1,mtime=0))
                self.evidence.enqueue(archive)
                self.evidence.append(self.run/'corridor_inputs.jsonl',dict(snapshot_file=str(relative),
                    snapshot_sha256=digest,path=path,actual_state=state,limits=self.corridor_observer.limits,
                    received_wall_ns=wall,compute_request_clock_ns=clock,
                    navigation_ground_truth_used=False,control_authority=False))
        except Exception as error:
            self.evidence.append(self.run/'corridor_input_rejections.jsonl',dict(
                reason=type(error).__name__+':'+str(error),received_wall_ns=wall,
                clock_ns=clock,control_authority=False))
    def poll_corridor(self,now):
        try:return self._poll_corridor(now)
        except Exception as error:
            self.evidence.append(self.run/'corridor_input_rejections.jsonl',dict(
                reason='shadow_poll:'+type(error).__name__+':'+str(error),
                received_wall_ns=time.monotonic_ns(),control_authority=False))
    def _poll_corridor(self,now):
        if self.corridor_observer is None:return
        result=self.corridor_observer.poll()
        if result is not None:
            result['recorded_clock_ns']=self.get_clock().now().nanoseconds
            result['active_path_id_at_receipt']=None if self.cascade is None else self.cascade.path_id
            self.corridor_last_certificate=result
            self.evidence.append(self.run/'corridor_certificates.jsonl',result)
        if (self.state!='running' or self.cascade is None or self.cascade.path is None or
            now-self.corridor_last_request_wall<float(self.profile['corridor']['snapshot_request_period_s'])):
            return
        from nav_msgs.msg import Path as PathMsg
        from geometry_msgs.msg import PoseStamped
        from spatial_reference import point_at
        reference=self.cascade.preview_reference(self.pose)
        start=reference['nearest_s_m'];end=min(start+1.2,float(self.cascade.cumulative[-1]))
        msg=PathMsg();msg.header.frame_id='camera_init';msg.header.stamp=self.get_clock().now().to_msg()
        for point in [self.pose,*[point_at(self.cascade.path,self.cascade.cumulative,s)
                                 for s in np.linspace(start,end,13)]]:
            pose=PoseStamped();pose.header=msg.header
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,point)
            pose.pose.orientation.w=1.;msg.poses.append(pose)
        self.corridor_request_pub.publish(msg)
        self.corridor_last_request_wall=now
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
        if getattr(self,'registered_cloud_archive_mode','legacy_all')=='disabled':
            # Keep the accepted live geometry and exact native guard receipt.
            # Do not allocate an additional point copy or write payload files.
            self.guard_cloud_receipt.update(cloud_array_file=None,cloud_raw_payload_file=None,
                cloud_archive_sequence=None,cloud_archive_collected=False)
            self.evidence.append(self.run/'navigation_cloud_xyz.jsonl',dict(
                schema='actual_registered_cloud_archive_disabled/v1',sequence=seq,
                stamp_ns=int(self.cloud_stamp),frame_id=msg.header.frame_id,
                filtered_xyz_float64_sha256=self.guard_cloud_receipt['filtered_xyz_float64_sha256'],
                filtered_points=int(len(self.guard_cloud)),input_payload_bytes=len(msg.data),
                array_file=None,raw_payload_file=None,archive_mode='disabled',
                cloud_archive_collected=False,cloud_replay_verified=False,
                historical_full_cloud_evidence_pass=False,live_guard_input_preserved=True,
                navigation_ground_truth_used=False))
            return
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
    def admission_identity(self):
        return dict(request_id=self.request_id,waypoint_index=self.waypoint_index,
            goal_xyz=self.waypoints[self.waypoint_index].tolist())
    def path_execution_eligibility(self):
        """Per-condition original live-source/identity evidence, without TTL refresh."""
        active=self.path_admission_active;now=time.monotonic();ns=self.get_clock().now().nanoseconds
        def fresh_wall(value):
            return isinstance(value,(int,float)) and math.isfinite(value) and 0<=now-value<=.3
        conditions=dict(active_present=active is not None,
            same_request_waypoint_goal=active is not None and active['identity']==self.admission_identity(),
            same_active_trajectory=active is not None and active['trajectory_id']==self.active_trajectory_id,
            same_active_reference=active is not None and tuple(active['active_reference_stamp'])==tuple(self.active_reference_stamp or ()),
            samples_present=self.samples is not None,state_running=self.state=='running',
            pose_source_fresh=fresh_wall(self.pose_updated),cloud_source_fresh=fresh_wall(self.cloud_updated),
            raw_imu_source_fresh=self.raw_imu.fresh(now,ns/1e9),
            bridge_source_fresh=fresh_wall(self.bridge_updated),bridge_ready=self.bridge_safety.get('state')=='ready',
            obstacle_not_held=not self.obstacle_hold,tilt_not_held=not self.tilt_hold)
        return dict(conditions=conditions,samples_present=conditions['samples_present'],
            clock_ns=ns,checked_monotonic_wall=now,actual_source_times_refreshed=False)
    def retain_admitted_path(self,points,config):
        # New planner reference stamps are pending identities, not new receipts
        # for the path being executed. Progress is projected spatially only.
        eligibility=self.path_execution_eligibility()
        if not all(eligibility['conditions'].values()):
            return dict(allowed=False,reason='old_path_not_eligible_stopped',**eligibility)
        proof=remaining_certificate(self.path_admission_active,self.samples,self.pose,points,config)
        return dict(proof,eligibility=eligibility,conditions=eligibility['conditions'],samples_present=True,
            retention_action='kept_checked_old_path' if proof['allowed'] else 'stopped_old_path_not_certified')
    def capture_replacement_eligibility(self,retention):
        """Recover only the recorded exhaustion of this immutable same-goal path.

        A missing path, source loss, protection, or failed old proof by itself
        cannot create this authorization. New-curve proof and base acceptance
        remain separate necessary gates; this evidence never commands motion.
        """
        marker=getattr(self,'execution_path_exhaustion',None);active=self.path_admission_active
        receipt=self.path_receipt or {};eligibility=self.path_execution_eligibility()
        conditions={k:v for k,v in eligibility['conditions'].items() if k!='samples_present'}
        conditions.update(samples_absent=self.samples is None,
            capture_same_goal=self.cascade is not None and self.cascade_key==(self.request_id,self.waypoint_index)
                and self.cascade.capture_stamp is not None,
            cascade_not_protected=self.cascade is not None and not self.cascade.protected,
            cascade_failure_not_latched=self.cascade is not None and self.cascade.failure_latched is None,
            exhaustion_receipt_present=isinstance(marker,dict))
        if isinstance(marker,dict):
            conditions.update(exhaustion_internal_schema=marker.get('schema')=='internal_checked_path_exhaustion/v1'
                    and marker.get('source')=='actual_shared_control_exhausted_clear_branch'
                    and marker.get('navigation_ground_truth_used') is False,
                exhaustion_allowed_reason=marker.get('reason') in ('steering_exhausted','low_command_exhausted_path'),
                exhaustion_same_request_waypoint_goal=dict(request_id=marker.get('request_id'),
                    waypoint_index=marker.get('waypoint_index'),goal_xyz=marker.get('goal_xyz'))==self.admission_identity(),
                exhaustion_same_active_id=marker.get('active_trajectory_id')==self.active_trajectory_id,
                exhaustion_same_active_reference=marker.get('active_reference_stamp')==list(self.active_reference_stamp or ()),
                exhaustion_matches_accepted_samples=marker.get('previous_samples_sha256')==receipt.get('source_samples_float64_sha256')
                    and marker.get('previous_samples_count')==receipt.get('source_samples_count')
                    and type(marker.get('previous_samples_count')) is int and 2<=marker['previous_samples_count']<=2000,
                exhaustion_matches_accepted_receipt=(receipt.get('request_id'),receipt.get('waypoint_index'),receipt.get('trajectory_id'))
                    ==(self.request_id,self.waypoint_index,self.active_trajectory_id),
                exhaustion_causal_stamps=type(marker.get('source_pose_stamp_ns')) is int
                    and type(marker.get('control_stamp_ns')) is int
                    and 0<=marker['source_pose_stamp_ns']<=marker['control_stamp_ns']<=eligibility['clock_ns'])
        else:
            conditions['exhaustion_internal_schema']=False
        allowed=all(conditions.values())
        return dict(allowed=allowed,reason='recorded_exhausted_capture_can_rebind' if allowed else 'capture_replacement_not_eligible_stopped',
            conditions=conditions,samples_present=self.samples is not None,exhaustion_receipt=copy.deepcopy(marker),
            source_eligibility=eligibility,old_retention_reason=retention.get('reason'),actual_source_times_refreshed=False)
    def clear_committed_exhausted_capture(self):
        # Only called after all existing execution gates committed a new path.
        # Keep the fixed goal, its original activation/90s deadline, velocity PI,
        # filters, feedback clocks and command slew state. A new capture must
        # obtain fresh original dwell; old parking proof is not transferable.
        cascade=self.cascade
        old={key:copy.deepcopy(getattr(cascade,key)) for key in
            ('capture_stamp','capture_wall','capture_clock','arrival','capture_dwell','hold_stamp','hold_wall','hold_clock')}
        for key in old:setattr(cascade,key,None)
        cascade.hold_integral[:]=0.
        self.execution_path_exhaustion=None
        self.pid_guard_needs_evaluation=True
        return dict(cleared_capture=old,fixed_goal_and_deadline_preserved=True,
            velocity_PI_filters_and_slew_preserved=True,fresh_capture_dwell_required=True)
    def archive_admission(self,value,metadata,decision,started):
        self.path_admission_sequence+=1;sequence=self.path_admission_sequence
        decision=dict(decision,schema='SCAN_path_execution_admission/v1',sequence=sequence,
            request_id=self.request_id,waypoint_index=self.waypoint_index,
            association_payload_id=None if value is None else value.get('traj_id'),
            active_trajectory_id=self.active_trajectory_id,
            active_reference_stamp=self.active_reference_stamp,
            admission_replan_pending=self.path_admission_replan_pending,
            pending_reference_stamp=self.trajectory_association.reference_stamp,
            control_stamp_ns=self.get_clock().now().nanoseconds,source_pose_stamp_ns=int(self.pose_stamp),
            callback_elapsed_wall_ms=(time.monotonic()-started)*1000,
            navigation_ground_truth_used=False,occupancy_or_terrain_support_certified=False)
        self.path_admission_last_decision=copy.deepcopy(decision)
        document=replayable(dict(decision=decision,payload=value,metadata=copy.deepcopy(metadata)))
        relative=Path('path_admission_candidates')/(str(sequence).zfill(6)+'_trajectory_'+str(decision['association_payload_id'])+'.json')
        def archive():
            target=self.run/relative;target.parent.mkdir(exist_ok=True)
            with target.open('x')as out:out.write(json.dumps(document,allow_nan=False,ensure_ascii=False)+'\n')
        self.evidence.enqueue(archive)
        self.evidence.append(self.run/'path_admission_decisions.jsonl',dict(document['decision'],candidate_file=str(relative)))
    def accept_spline(self,msg,metadata):
        # Admission is a transaction BEFORE base acceptance, publication, PI
        # path replacement and the accepted trajectory archive. Association's
        # payload high-water is deliberately not rolled back on a rejection.
        started=time.monotonic();value=None;binding=None;points=None;config=None
        try:
            if len(msg.pos_pts)>512 or len(msg.knots)>516:raise ValueError('candidate_input_count_budget')
            value=payload(msg)
            value=candidate(msg,metadata,self.trajectory_association.reference_stamp,
                self.waypoints[self.waypoint_index] if self.continuous_route is None else self.trajectory_association.goal)
            if (self.continuous_route is not None and
                    self.scan_reference_coverage_index != self.continuous_route.next_index):
                raise ValueError('candidate_ordered_coverage_epoch_mismatch')
            if (self.continuous_route is not None and not np.array_equal(
                    self.trajectory_association.goal,self.waypoints[self.scan_reference_coverage_index])):
                raise ValueError('candidate_ordered_reference_goal_mismatch')
            points,config,binding=self.path_admission_fence.load()
            proof=certify_curve(value,points,config)
            if proof['allowed'] and self.continuous_route is not None:
                index=self.scan_reference_coverage_index
                prefix=self.continuous_route.points[index:index+2]
                ordered_proof=certify_curve(value,prefix,config)
                proof=dict(proof,ordered_current_leg=ordered_proof,
                    ordered_coverage_index=index,local_body_goal=list(self.trajectory_association.goal))
                if not ordered_proof['allowed']:
                    proof=dict(proof,allowed=False,reason='outside_ordered_current_leg')
            if proof['allowed']:
                begin,end=value['knots'][3],value['knots'][-4]
                check=BSpline(value['knots'],value['pos_pts'],3)(np.linspace(begin,end,min(2000,max(2,int((end-begin)/.08)+1))))
                if not np.isfinite(check).all():proof=dict(allowed=False,reason='nonfinite_execution_samples')
                elif self.pose is not None and np.linalg.norm(check[0]-self.pose)>1.2:
                    proof=dict(allowed=False,reason='execution_start_far_from_actual_pose')
                elif (not trajectory_has_progress(check,self.pose,self.waypoints[self.waypoint_index],self.arrival_radius)
                        or not np.any(np.linalg.norm(check[1:,:2]-check[0,:2],axis=1)>=1e-5)):
                    proof=dict(allowed=False,reason='execution_path_degenerate')
            if proof['allowed'] and self.global_reference_settings['enabled']:
                if self.segment_start is None:raise ValueError('direction_check_requires_original_segment_activation')
                delta=self.waypoints[self.waypoint_index][:2]-self.segment_start[:2]
                check_direction=admission_direction_check(check,self.pose,self.waypoints[self.waypoint_index],
                    self.fixed_terminal_heading(self.waypoint_index,self.segment_start),self.profile['cascade'],
                    self.profile['teacher_transition']['drive_heading_max_rad'],self.global_reference_settings)
                proof=dict(proof,direction_coherence=check_direction)
                if not check_direction['allowed']:
                    proof=dict(proof,allowed=False,reason='execution_direction_incoherent')
            if time.monotonic()-started>.100:proof=dict(allowed=False,reason='admission_callback_budget_exhausted')
        except (OSError,KeyError,ValueError,TypeError,OverflowError,IndexError,FloatingPointError)as error:
            proof=dict(allowed=False,reason='admission_input_or_anchor:'+type(error).__name__+':'+str(error))
        capture=(self.cascade is not None and self.cascade_key==(self.request_id,self.waypoint_index)
            and self.cascade.capture_stamp is not None)
        if self.cascade is not None and self.cascade.capture_stamp is not None and not capture:
            proof=dict(allowed=False,reason='capture_goal_identity_mismatch')
        retain=None;recovery=None
        if capture and proof['allowed']:
            retain=self.retain_admitted_path(points,config)
            if not retain['allowed']:recovery=self.capture_replacement_eligibility(retain)
        if time.monotonic()-started>.100:
            proof=dict(allowed=False,reason='admission_callback_budget_exhausted');recovery=None
        if not proof['allowed'] or capture and (recovery is None or not recovery['allowed']):
            # Only a geometrically outside candidate (or an existing terminal
            # capture) may keep an independently certified old same-goal path.
            # Invalid anchor/numbers and exhausted proofs always stop/replan.
            retain=(retain if retain is not None else self.retain_admitted_path(points,config) if points is not None
                and (proof.get('reason')=='outside_fence' or proof['allowed'] and capture)
                else dict(allowed=False,reason='fail_closed_rejection_requires_stop',samples_present=self.samples is not None,
                    conditions=self.path_execution_eligibility()['conditions']))
            if time.monotonic()-started>.100:retain=dict(retain,allowed=False,reason='retention_callback_budget_exhausted',samples_present=self.samples is not None)
            self.path_admission_hold=not retain['allowed']
            self.last_spline_rejected='V28 SCAN整条路径接纳拒绝：'+str(proof.get('reason'))
            self.path_admission_replan_pending=True
            if retain['allowed']:
                self.pid_guard_needs_evaluation=True
            else:
                self.message=self.last_spline_rejected+'；停车等待重规划'
                self.publish_command()
            self.archive_admission(value,metadata,dict(admitted=False,
                reason=('terminal_capture_kept_checked_path' if retain['allowed'] else 'terminal_capture_stopped_no_eligible_path')
                    if capture and proof['allowed'] else proof.get('reason'),
                proof=proof,anchor_binding=binding,old_path_retention=retain,capture_replacement=recovery,
                exact_zero_requested=not retain['allowed']),started)
            return
        previous_execution=(dict(samples=self.samples,active_trajectory_id=self.active_trajectory_id,
            active_reference_stamp=self.active_reference_stamp,planning_start_state=copy.deepcopy(self.planning_start_state),
            execution_path_exhaustion=copy.deepcopy(getattr(self,'execution_path_exhaustion',None)))
            if recovery is not None and recovery['allowed'] else None)
        before=self.replans;wall=time.monotonic_ns();super().accept_spline(msg,metadata)
        if self.replans>before and self.samples is not None:
            capture_clear=(self.clear_committed_exhausted_capture() if recovery is not None and recovery['allowed'] else None)
            self.path_admission_active=dict(payload=copy.deepcopy(value),metadata=copy.deepcopy(metadata),
                identity=self.admission_identity(),trajectory_id=self.active_trajectory_id,
                active_reference_stamp=tuple(self.active_reference_stamp),proof=copy.deepcopy(proof),anchor_binding=binding)
            self.path_admission_hold=False;self.path_admission_replan_pending=False;self.reference_replan_pending=False
            self.reference_consistency_hold=False;self.reference_consistency_check=None
            indices=[0]
            for i in range(1,len(self.samples)):
                if np.linalg.norm(self.samples[i,:2]-self.samples[indices[-1],:2])>=1e-5:indices.append(i)
            points=self.samples[indices]
            self.path_receipt=dict(points_xyz=points.tolist(),
                source_sample_indices=indices,source_samples_count=len(self.samples),
                source_samples_float64_sha256=hashlib.sha256(np.asarray(self.samples,dtype='<f8').tobytes()).hexdigest(),
                path_id=str(self.request_id)+':'+str(self.waypoint_index)+':'+str(self.replans)+':'+str(self.active_trajectory_id),
                stamp_ns=int(self.active_reference_stamp[0])*1_000_000_000+int(self.active_reference_stamp[1]),
                received_wall_ns=wall,frame_id='camera_init',array_file=self.trajectory_archive_reference,
                request_id=self.request_id,waypoint_index=self.waypoint_index,
                trajectory_id=self.active_trajectory_id,metadata=copy.deepcopy(metadata))
            self.path_receipt['points_float64_sha256']=hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()).hexdigest()
            self.evidence.append(self.run/'cascade_path_history.jsonl',copy.deepcopy(self.path_receipt))
            self.archive_admission(value,metadata,dict(admitted=True,reason='exhausted_capture_rebound_after_all_execution_gates' if capture_clear is not None else 'continuous_fence_and_existing_execution_gates_passed',
                proof=proof,anchor_binding=binding,capture_replacement=recovery,capture_clear=capture_clear),started)
        else:
            # Existing execution guards still own final admission. Their
            # refusal never becomes an admitted path in this separate ledger.
            if previous_execution is not None:
                for key,old in previous_execution.items():setattr(self,key,old)
            self.path_admission_hold=True;self.path_admission_replan_pending=True;self.publish_command()
            self.archive_admission(value,metadata,dict(admitted=False,reason='existing_execution_gate_rejected',proof=proof,
                anchor_binding=binding,capture_replacement=recovery,lower_gate_execution_identity_restored=previous_execution is not None,exact_zero_requested=True),started)
    def follow_checked_trajectory(self,*args,**kwargs):
        kwargs['tracking_pose']=self.pose
        result=follow_trajectory(*args,**kwargs)
        if not kwargs.get('return_steering'):return result
        self.ensure_cascade()
        # The gate and PD/PI use one spatial reference, including its short-tail
        # rule. Preview neither advances progress nor integrates a held sample.
        reference=self.cascade.preview_reference(self.pose)
        yaw=reference['heading_rad']
        if getattr(self,'global_reference_settings',{'enabled':False})['enabled']:
            self.reference_consistency_check=direction_check(self.cascade,reference,
                self.profile['teacher_transition']['drive_heading_max_rad'],self.global_reference_settings)
            self.reference_consistency_hold=not self.reference_consistency_check['allowed']
            if self.reference_consistency_hold:self.reference_replan_pending=True
        velocity,rate,target,steering=result
        original=copy.deepcopy(steering)
        steering=dict(steering,heading=yaw,error=math.atan2(math.sin(yaw-math.atan2(self.rotation[1,0],self.rotation[0,0])),
            math.cos(yaw-math.atan2(self.rotation[1,0],self.rotation[0,0]))),
            heading_reference='same actual SCAN finite-arc spatial reference as cascade',
            original_lookahead_steering=original,
            cascade_projection=dict(path_id=self.cascade.path_id,path_sha256=self.cascade.path_sha,
                reference=copy.deepcopy(reference),
                actual_pose_stamp_ns=int(self.pose_stamp),navigation_ground_truth_used=False),
            direction_coherence=copy.deepcopy(getattr(self,'reference_consistency_check',None)))
        self.evidence.append(self.run/'navigation_heading_projection.jsonl',dict(
            request_id=self.request_id,waypoint_index=self.waypoint_index,
            source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds,
            steering=copy.deepcopy(steering),feedback=copy.deepcopy(self.feedback),
            navigation_ground_truth_used=False))
        return velocity,rate,target,steering
    def continuous_before_region(self,now,clock_ns):
        ledger=self.continuous_route
        if ledger is None:return
        if ledger.points is None:
            ledger.activate(self.region_raw_pose,int(self.pose_stamp),clock_ns)
            self.waypoint_index=ledger.target_index
            self.region_arrival=ArrivalWindow(self.goals[self.waypoint_index])
        receipt=ledger.observe(self.region_raw_pose,int(self.pose_stamp),clock_ns,
            protected=self.obstacle_hold or self.tilt_hold or self.bridge_safety.get("state")!="ready")
        if receipt is not None:
            self.region_arrivals.append(copy.deepcopy(receipt))
            self.evidence.append(self.run/"navigation_region_passes.jsonl",receipt)
            self.path_admission_replan_pending=True
            self.last_reference=-math.inf
        if ledger.missed_pass_requires_replan() and self.coverage_replan_index!=ledger.next_index:
            self.coverage_replan_index=ledger.next_index
            self.evidence.append(self.run/"navigation_coverage_replans.jsonl",dict(
                ledger.status(),reason="ordinary_region_not_observed_replan_through_original_volume",
                source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=clock_ns,
                old_trajectory_id=self.active_trajectory_id,safety_gates_preserved=True))
            self.samples=None;self.execution_path_exhaustion=None
            self.trajectory_association.reset();self.planning_start_state=None
            self.active_trajectory_id=None;self.active_reference_stamp=None
            self.last_reference=-math.inf
            return True
        return False
    def continuous_record_hard(self,receipt,clock_ns):
        if self.continuous_route is not None:
            self.continuous_route.accept_hard_receipt(receipt,clock_ns)
    def continuous_release_target(self):
        ledger=self.continuous_route
        if ledger is None or self.state!="running" or ledger.points is None:return
        target=ledger.release_hard_target()
        if target!=self.waypoint_index and ledger.next_index<=target:
            self.waypoint_index=target
            self.region_arrival=ArrivalWindow(self.goals[target])
            self.region_arrival.last_stamp=int(self.pose_stamp)
            self.region_arrival_evidence=None
    def execution_guard_route(self,start,goal):
        ledger=self.continuous_route
        return ledger.guard_route() if ledger is not None and ledger.points is not None else np.vstack([start,goal])
    def fixed_terminal_heading(self,index,start):
        ledger=self.continuous_route
        if ledger is not None and ledger.points is not None:return ledger.terminal_heading(index)
        delta=self.waypoints[index][:2]-start[:2]
        if np.linalg.norm(delta)<1e-6:raise ValueError("Frozen goal heading requires nondegenerate actual SLAM segment")
        return math.atan2(delta[1],delta[0])
    def planner_goal(self):
        ledger=self.continuous_route
        return (self.waypoints[ledger.next_index] if ledger is not None and ledger.points is not None
            else super().planner_goal())
    def request_plan(self):
        ledger=self.continuous_route
        index=None if ledger is None else ledger.next_index
        before=self.reference_requests
        result=super().request_plan()
        if self.reference_requests>before:self.scan_reference_coverage_index=index
        return result
    def planner_reference_points(self,goal):
        if self.continuous_route is not None:
            points=self.continuous_route.reference_points(end_index=self.continuous_route.next_index)
            self.evidence.append(self.run/"navigation_global_reference_requests.jsonl",dict(
                self.continuous_route.status(),guide_points_xyz=copy.deepcopy(points),guide_count=len(points),
                local_body_goal=list(map(float,goal)),local_endpoint_coverage_index=self.continuous_route.next_index,
                fixed_hard_target_index=self.waypoint_index,
                frame_id="camera_init",guide_only_not_arrival=True,collision_or_support_certified=False,
                source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds))
            return points
        if not self.global_reference_settings['enabled']:return [goal]
        if self.segment_start is None:raise ValueError('Global planning reference lacks original measured segment activation')
        points,receipt=self.global_route_guides.remaining((self.request_id,self.waypoint_index),
            self.segment_start,goal,self.pose,self.global_reference_settings)
        self.evidence.append(self.run/'navigation_global_reference_requests.jsonl',dict(receipt,
            source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds,
            goals_definition_sha256=self.goals_definition_sha256))
        return points
    def record_segment_activation(self):
        key=(self.request_id,self.waypoint_index)
        if key not in self.segment_references:
            if self.segment_start is None or not np.array_equal(self.segment_start,self.pose):
                raise RuntimeError('Actual segment activation lacks its original SLAM pose')
            self.segment_references[key]=dict(segment_start=self.segment_start.tolist(),
                source_pose_stamp_ns=int(self.segment_start_pose_stamp_ns),control_stamp_ns=int(self.segment_started_ros_ns),
                feedback=copy.deepcopy(self.feedback),request_id=self.request_id,waypoint_index=self.waypoint_index)
            self.evidence.append(self.run/'navigation_segment_activation.jsonl',dict(
                **self.segment_references[key],navigation_ground_truth_used=False))
    def measured_region_arrival(self):
        self.record_segment_activation()
        if self.continuous_route is not None and self.continuous_route.next_index!=self.waypoint_index:
            self.reset_region_arrival("ordered_pass_coverage_pending")
            return False,False
        return super().measured_region_arrival()
    def ensure_cascade(self):
        key=(self.request_id,self.waypoint_index)
        if self.cascade_key!=key:
            self.reference_replan_pending=False
            goal=self.waypoints[self.waypoint_index]
            yaw=self.fixed_terminal_heading(self.waypoint_index,self.segment_start)
            core=(CausalHandoffProjectionController if self.continuous_route is not None else
                StableProjectionController if getattr(self,"global_reference_settings",{"enabled":False})["enabled"] else Controller)
            self.cascade=core(self.profile['cascade'],goal.tolist(),yaw,':'.join(map(str,key)))
            self.reference_consistency_hold=False;self.reference_consistency_check=None
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
            self.cascade.set_path(r['points_xyz'],r['path_id'],r['stamp_ns'],r['received_wall_ns'],r['frame_id'],
                position=self.pose)
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
        mode=mode_override or ('hold'if self.obstacle_hold or getattr(self,'reference_consistency_hold',False) or self.heading_gate.phase in ('pre_turn','settle')
            else 'drive'if allow_translation else 'turn')
        heading_reference=copy.deepcopy(self.steering);gate_heading=self.heading_gate.heading
        ack=self.get_ack();guard=None
        if self.feedback is None or self.paired_imu is None or ack is None:guard='actual_causal_feedback_or_ack_missing'
        cmd,diagnostic=self.cascade.update(self.feedback,self.paired_imu,ack,ns,wall,guard_reason=guard,mode_override=mode,
            external_turn_heading=gate_heading if mode=='turn' and self.heading_gate.phase=='align' else None)
        if diagnostic.get('failure_latched'):
            self.state='failed';self.message='串级真实输入保护锁定：'+str(diagnostic['failure_latched'])
        self.cascade_last_row=diagnostic;self.pid_guard_needs_evaluation=(bool(diagnostic.get('controller_updated'))
            or self.control_clock_hold.guard_required)
        self.reference_replan_pending=(getattr(self,'reference_consistency_hold',False)
            or diagnostic['mode']=='reference_constraint_hold')
        desired=cmd.copy()
        cmd=(np.zeros(3)if diagnostic['mode']in ('protect','recovering','pre_turn','settle','path_end_hold','reference_constraint_hold')
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
        try:
            return self._control_candidate(now)
        except Exception as error:
            # A reference/handoff error must stop this tick, not escape the
            # executor and rely only on the 300 ms downstream watchdog.
            self.state='failed'
            self.message='V33控制参考异常，停车：'+type(error).__name__+': '+str(error)
            self.reference_replan_pending=False
            self.publish_command()
            self.evidence.append(self.run/'navigation_controller_errors.jsonl',dict(
                error_type=type(error).__name__,error=str(error),command_after_stop=list(self.command),
                control_stamp_ns=self.get_clock().now().nanoseconds,
                source_pose_stamp_ns=int(self.pose_stamp),navigation_ground_truth_used=False))
    def _control_candidate(self,now):
        self.poll_corridor(now)
        # The wall timer can run twice while real ROS /clock is unchanged.
        # Before any PI, heading, arrival or geometry operation, emit exact
        # zero and leave original source receipts untouched. On clock advance,
        # a new actual geometry check is mandatory before any nonzero command.
        if self.control_clock_hold.before_control(self,self.get_clock().now().nanoseconds,round(now*1e9)):
            return
        self.request_admission_replan_if_due(now)
        if self.path_admission_hold:
            return self.control_admission_hold(now)
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
        self.continuous_release_target()
        if self.state=='succeeded' and self.final_bundle is not None:return self.control_parking(now)
        if self.state=='running'and self.samples is not None and self.waypoint_index==len(self.waypoints)-1 and self.segment_start is not None:
            self.ensure_cascade()
            self.final_bundle=dict(index=self.waypoint_index,start=self.segment_start.copy(),
                trajectory_id=self.active_trajectory_id,array=self.trajectory_archive_reference,
                path_receipt=copy.deepcopy(self.path_receipt),reference_stamp=tuple(self.active_reference_stamp))
        return super().control(now)
    def request_admission_replan_if_due(self,now):
        pending=(self.reference_replan_pending or self.path_admission_replan_pending or self.path_admission_hold)
        # A previous goal's pending replan must wait for inherited control to
        # activate the next segment from a fresh, unprotected actual SLAM pose.
        if (pending and self.state=='running' and now-self.last_reference>3.
            and (not getattr(self,'global_reference_settings',{'enabled':False})['enabled']
                or self.segment_start is not None)):
            self.evidence.append(self.run/'reference_replan_events.jsonl',dict(
                reason='path_admission_rejection' if self.path_admission_replan_pending or self.path_admission_hold else 'reference_constraint_hold',
                admission_reason=None if self.path_admission_last_decision is None else self.path_admission_last_decision['reason'],
                request_id=self.request_id,waypoint_index=self.waypoint_index,trajectory_id=self.active_trajectory_id,
                source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds,
                navigation_ground_truth_used=False))
            before=self.reference_requests;self.request_plan()
            if self.reference_requests>before:
                self.reference_replan_pending=False;self.path_admission_replan_pending=False
    def control_admission_hold(self,now):
        # Keep the inherited deadlines, measured IMU/bridge protection and
        # source-loss failure active while no execution-admitted path exists.
        # Holding does not tick the cascade, heading or arrival integrators.
        ns=self.get_clock().now().nanoseconds;ros_now=ns/1e9
        if self.state!='running':self.publish_command();return
        if (self.segment_start is not None and not self.goals[self.waypoint_index].legacy
                and self.segment_deadline_exceeded(ns)):
            self.reset_region_arrival('failed')
            self.state,self.message='failed','当前航点超过仿真时间限制，停车'
            self.publish_command();return
        protected=self.apply_tilt_guard(now,ros_now)
        if self.state=='failed':
            self.reset_region_arrival('failed');self.publish_command();return
        stale=(now-self.pose_updated>self.pose_timeout or now-self.cloud_updated>self.cloud_timeout
            or not self.raw_imu.fresh(now,ros_now))
        if stale:
            self.reset_region_arrival('stale');self.stale_since=self.stale_since or now
            self.message='SLAM、点云或原始 IMU 超时，停车等待传感器'
            if now-self.stale_since>8.:self.state,self.message='failed','SLAM、点云或原始 IMU 连续失联超过 8 秒'
        elif self.state=='failed':self.reset_region_arrival('failed')
        elif protected:self.reset_region_arrival('protected')
        else:
            self.stale_since=None;self.reset_region_arrival('path_admission_hold')
            if self.segment_start is None:
                self.segment_start=self.pose.copy();self.segment_started_ros=ros_now
                self.segment_started_ros_ns=ns;self.segment_start_pose_stamp_ns=int(self.pose_stamp)
                self.record_segment_activation()
            # Path admission must not starve the existing obstacle-clear
            # observation gate. This checks the original native geometry at
            # exact zero; it cannot release path admission or arrival dwell.
            if (getattr(self,'global_reference_settings',{'enabled':False})['enabled']
                    and self.obstacle_hold and (now-self.last_obstacle_check>.10 or self.pid_guard_needs_evaluation)):
                self.publish_command()
                goal=self.waypoints[self.waypoint_index]
                target=np.asarray((self.steering or {}).get('target',goal),dtype=float)
                direction=np.asarray((self.steering or {}).get('direction',(goal-self.pose)[:2]),dtype=float)
                result=self.evaluate_motion_guard(self.cloud,self.pose,target,direction,self.execution_guard_route(self.segment_start,goal))
                self.obstacle_result=result;self.last_obstacle_check=now
                if result[0] or not self.heading_gate.stopped(ros_now,now):
                    self.obstacle_clear_since=None
                else:
                    self.obstacle_clear_since=self.obstacle_clear_since or ros_now
                    if ros_now-self.obstacle_clear_since>=1.:
                        self.obstacle_hold=False;self.obstacle_resumes+=1
                        self.obstacle_stopped_duration=ros_now-(ros_now if self.obstacle_hold_started_ros is None else self.obstacle_hold_started_ros)
                        self.obstacle_resume_pending=False;self.heading_gate.reset()
                        self.samples=None;self.execution_path_exhaustion=None
                        # Retain the admission hold. Only a newly associated,
                        # execution-admitted curve can authorize future motion.
                if self.pending_guard_row is not None:
                    row=self.pending_guard_row
                    row.update(obstacle_hold_after=bool(self.obstacle_hold),clear_start_after_s=self.obstacle_clear_since,
                        after_control_ros_clock_ns=ns,command_after_control=list(self.command),
                        zero_requested_after_control=True,state_after_control=self.state,
                        admission_hold_observation_only=True,path_admission_hold_after=self.path_admission_hold,
                        obstacle_resumes_after=self.obstacle_resumes,evidence_queue_error=self.evidence.error)
                    self.evidence.append(self.run/'navigation_guard_history.jsonl',row);self.pending_guard_row=None
        self.publish_command()
    def control_parking(self,now):
        ns=self.get_clock().now().nanoseconds
        bundle=self.final_bundle;index=self.waypoint_index
        self.waypoint_index=bundle['index'];self.active_reference_stamp=bundle['reference_stamp']
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
            result=self.evaluate_motion_guard(self.cloud,self.pose,goal,self.steering['direction'],self.execution_guard_route(bundle['start'],goal))
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
            self.waypoint_index=index
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
    from known_scene_inputs import KnownSceneInputMixin
    class IntegratedNavigation(KnownSceneInputMixin,PIDNavigation):pass
    rclpy.init(args=ros);node=IntegratedNavigation(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publish_command()
        try:
            if node.corridor_observer is not None:
                remaining=node.corridor_observer.close()
                if remaining is not None:node.evidence.append(run/'corridor_certificates.jsonl',remaining)
                node.evidence.append(run/'corridor_worker_receipt.jsonl',dict(
                    submitted=node.corridor_observer.submitted,completed=node.corridor_observer.completed,
                    busy_diagnostic_snapshots=node.corridor_observer.busy_snapshots,
                    control_authority=False,navigation_ground_truth_used=False))
        except Exception as error:
            node.evidence.append(run/'corridor_input_rejections.jsonl',dict(
                reason='shadow_close:'+type(error).__name__+':'+str(error),control_authority=False))
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
