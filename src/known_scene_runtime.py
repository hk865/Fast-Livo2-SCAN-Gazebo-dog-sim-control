#!/usr/bin/env python3
"""Navigation-only map correction transactions; original FAST-LIVO stays untouched.
No truth subscriptions, commands, TF, map deletion, or frontend state resets.
"""
from integration_limits import KNOWN_BUNDLE_FRESHNESS_NS, KNOWN_BUNDLE_FRESHNESS_S
import argparse,copy,hashlib,json,sys,time
from collections import OrderedDict,deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from known_scene_matcher import SceneReference,KnownSceneMatcher,CorrectionState,transform_distance,checked_transform,policy_for_profile
from known_scene_adapter import ExactSourceJoin,MatcherWorker,validate_runtime_witness,stamp_ns,odometry_transform,pointcloud_xyz
from known_scene_stale_recovery import recovery_proposal
from point_provenance import decode,geometry_bundle,message_sha
from delivery_diagnostics import DeliveryDiagnostics,delivery_span
from r74_corrected_delivery_witness import (create as _corrected_create,
    producer_metadata as _corrected_metadata, flush as _corrected_flush)
from diagnostic_observation import enabled as diagnostic_enabled,record as diagnostic_record

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def atomic(p,x):
    q=p.with_suffix('.known.tmp');q.write_text(json.dumps(x,allow_nan=False)+'\n');q.replace(p)

def proposal_epoch_valid(result,generation,pending_target,transition):
    if not transition:return result.get('parent_generation')==generation
    if pending_target is None:return False
    distance,angle=transform_distance(np.asarray(result['C_world_odom']),pending_target)
    return distance<.08 and angle<.05

def proposal_body_effect(result,applied):
    """Measure a correction on its captured body, independently of world origin.

    The worker preserves the exact body pose that produced C. Never use a latest
    or nearest pose to decide a transaction on an older matching result.
    """
    source=result.get('source_ns')
    if type(source) is not int or source<=0 or result.get('T_odom_body_source_ns')!=source:
        raise ValueError('Correction trigger requires the actual same-source body pose')
    raw=checked_transform(result['T_odom_body'])
    candidate=checked_transform(result['C_world_odom'])
    matched=checked_transform(result['T_world_body'])
    current=checked_transform(applied)
    proposed=candidate@raw
    if not np.allclose(proposed,matched,rtol=0,atol=1e-7):
        raise ValueError('Correction body pose and candidate association inconsistent')
    before=current@raw
    translation=float(np.linalg.norm(proposed[:3,3]-before[:3,3]))
    angle=float(Rotation.from_matrix(proposed[:3,:3]@before[:3,:3].T).magnitude())
    return dict(schema='known_scene_same_source_body_correction_effect/v1',source_ns=source,
        translation_m=translation,rotation_rad=angle,
        translation_trigger_m=.04,rotation_trigger_rad=.025,
        requires_transaction=translation>.04 or angle>.025,
        origin_translation_diagnostic_m=float(np.linalg.norm(candidate[:3,3]-current[:3,3])),
        before_world_body_position=before[:3,3].tolist(),after_world_body_position=proposed[:3,3].tolist(),
        pose_association='exact_worker_captured_body_stamp',robot_truth_pose_used=False)

def transaction_convergence(applied,target,captured_body,source_ns):
    """Require convergence at the exact body captured when this target froze."""
    if type(source_ns) is not int or source_ns<=0:
        raise ValueError('Transaction release requires its frozen body source stamp')
    current=checked_transform(applied);goal=checked_transform(target)
    body=checked_transform(captured_body)
    origin,angle=transform_distance(current,goal)
    before=current@body;after=goal@body
    residual=float(np.linalg.norm(after[:3,3]-before[:3,3]))
    return dict(schema='known_scene_same_source_transaction_convergence/v1',
        frozen_body_source_ns=source_ns,body_residual_m=residual,origin_residual_m=origin,
        rotation_residual_rad=angle,body_limit_m=.005,original_origin_limit_m=.005,
        original_rotation_limit_rad=.003,
        converged=residual<.005 and origin<.005 and angle<.003,
        robot_truth_pose_used=False,pose_association='frozen_transaction_worker_body')

def corrected_odom(msg,c):
    out=copy.deepcopy(msg);t=c@odometry_transform(msg);q=Rotation.from_matrix(t[:3,:3]).as_quat()
    out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,t[:3,3])
    out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
    # Child/body twist is physical and unchanged; no derivative of correction C.
    cov=np.asarray(out.pose.covariance).reshape(6,6);a=np.zeros((6,6));a[:3,:3]=a[3:,3:]=c[:3,:3]
    out.pose.covariance=(a@cov@a.T).reshape(-1).tolist()
    return out

def corrected_cloud(msg,c):
    out=copy.deepcopy(msg);payload=bytearray(msg.data);endian='>' if msg.is_bigendian else '<'
    fields={f.name:f for f in msg.fields};views=[]
    for n in ('x','y','z'):
        f=fields[n]
        if f.datatype!=7 or f.count!=1:raise ValueError('Registered XYZ must remain float32')
        views.append(np.ndarray((msg.height,msg.width),dtype=endian+'f4',buffer=payload,
            offset=f.offset,strides=(msg.row_step,msg.point_step)))
    xyz=np.stack(views,-1);changed=xyz@c[:3,:3].T+c[:3,3]
    for k,v in enumerate(views):v[:]=changed[...,k]
    out.data=bytes(payload)
    return out

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock as ClockMessage
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True)
    args,ros=ap.parse_known_args();run=args.run.resolve();here=Path(__file__).resolve().parent
    profile=json.loads((run/'navigation_scope.json').read_text())['profile']
    sys.path.append(str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'src'))
    from runtime_io import EvidenceWriter
    from startup_trace import CallbackTrace
    from owned_ros_lifecycle import OwnedRosLifecycle
    from rclpy.signals import SignalHandlerOptions
    import r119_metadata_async
    r119_metadata_async.configure(run)
    lifecycle=OwnedRosLifecycle(rclpy,ros,SignalHandlerOptions.NO)
    class Runtime(Node):
        def __init__(self):
            super().__init__('demo_slam_odom_adapter')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            scenario=json.loads((run/'navigation_scenario.json').read_text())
            origin=np.asarray(scenario['slam_origin_in_world'],dtype=float)
            self.nav_world=np.eye(4);self.nav_world[:3,3]=-origin
            baseline=np.eye(4);baseline[:3,3]=origin
            self.correction=CorrectionState(initial_c=baseline)
            reference_path=run/'known_scene_reference.json'
            reference=SceneReference.load(reference_path,sha(reference_path),sha(run/'world.sdf'))
            self.matcher=KnownSceneMatcher(reference,policy_for_profile(profile))
            from collision_minimal_capture import CollisionObservedP95SourceFrames as P95SourceFrames
            self.diagnostic_capture=P95SourceFrames(run)
            self.worker=MatcherWorker(self.matcher,self.diagnostic_capture)
            from first_identity_capture import FirstIdentityCapture
            self.positive_capture=FirstIdentityCapture(run)
            self.join=None;self.slots=OrderedDict();self.last_published=-1;self.last_submit=-1
            self.initialized=False;self.transition=False;self.pending_target=None;self.last_result=None
            self.pending_body=None;self.pending_source_ns=None;self.startup_results=deque(maxlen=16)
            self.pending_recovery_count=0;self.last_complete_body=None
            self.transaction_count=0;self.bundle_count=0;self.error=None;self.loaded_witness=False;self.stop_since_ns=None
            self.callback_writer=EvidenceWriter()
            from match_query_capture import create as create_query_capture
            self.query_capture=create_query_capture(run,self.matcher,self.callback_writer)
            self.matcher.query_capture=self.worker.query_capture=self.query_capture
            self.delivery_diagnostics=DeliveryDiagnostics(run,self.callback_writer)
            self._r74_corrected_delivery_witness=_corrected_create(run,profile,'producer')
            self.delivery_diagnostics.corrected_delivery_witness=self._r74_corrected_delivery_witness
            self.callback_trace=CallbackTrace(run,'canonical',self.callback_writer)
            self.history=(run/'known_scene_matches.jsonl').open('x',buffering=1)
            self.bundle_history=(run/'known_scene_bundles.jsonl').open('x',buffering=1)
            self.decision_history=(run/'known_scene_correction_decisions.jsonl').open('x',buffering=1)
            self.release_history=(run/'known_scene_transaction_releases.jsonl').open('x',buffering=1)
            self.pubs={k:self.create_publisher(Odometry,'/demo/slam/'+k+'_odom',10) for k in ('body','lidar')}
            self.cloud_pub=self.create_publisher(PointCloud2,'/cloud_registered_full',qos_profile_sensor_data)
            self.bundle_pub=self.create_publisher(String,'/demo/navigation/localization_bundle',10)
            self.status_pub=self.create_publisher(String,'/demo/navigation/localization_status',10)
            self._r119_metadata_async_actual=r119_metadata_async.verify(self,run)
            for role,typ,topic in [('raw',PointCloud2,'/demo/teacher/raw_lidar'),
                ('adapted',PointCloud2,'/demo/slam/lidar_filtered'),('registered',PointCloud2,'/demo/slam/raw_registered_full'),
                ('body',Odometry,'/demo/slam/raw_body_odom'),('lidar',Odometry,'/demo/slam/raw_lidar_odom'),
                ('mapper',Odometry,'/aft_mapped_to_init')]:
                self.create_subscription(typ,topic,lambda m,k=role:self.receive(k,m),qos_profile_sensor_data)
            for role,topic,schema in [('source_identity','/demo/slam/raw_point_identity','raw_filtered_point_identity/v1'),
                ('point_identity','/demo/slam/point_identity','post_lio_source_point_commit/v1'),
                ('odom_identity','/demo/slam/odom_identity','mapper_body_lidar_commit/v1')]:
                self.create_subscription(String,topic,lambda m,k=role,s=schema:self.receive(k,self.delivery_diagnostics.call('decode',-1,decode,m.data,s)),10)
            # Data-only event observer; original matcher and publishers unchanged.
            self.create_subscription(String,'/demo/teacher/mission46/state',
                lambda m:self.delivery_diagnostics.mission(self,m.data,int(self.get_clock().now().nanoseconds),self.diagnostic_capture.observe_mission),10)
            self.create_subscription(String,'/demo/obstacle/state',
                lambda m:self.diagnostic_capture.observe_obstacle(m.data,int(self.get_clock().now().nanoseconds)),10)
            for role,typ,topic in [('imu_source',Imu,'/livox/imu'),
                    ('imu_relay',Imu,'/demo/teacher/slam/imu'),('clock',ClockMessage,'/clock')]:
                self.create_subscription(typ,topic,lambda m,k=role:self.diagnostic_capture.observe_aux(
                    k,m,int(self.get_clock().now().nanoseconds)),qos_profile_sensor_data)
            self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def load_witness(self):
            p=run/'known_scene_runtime_witness.json'
            if not p.exists():return
            witness=json.loads(p.read_text());status=validate_runtime_witness(witness,run.name)
            atomic(run/'known_scene_witness_validation.json',status)
            if not status['valid']:raise RuntimeError('Loaded producer/stage witness rejected: '+str(status['reasons']))
            self.join=ExactSourceJoin(status);self.loaded_witness=True
            self.diagnostic_capture.configure_witness(status,self.join._check)
        def idle_or_stopped(self,body,initial=False):
            p=run/'navigation_status.json'
            try:status=json.loads(p.read_text())
            except (OSError,ValueError):return False
            if initial:return status.get('state')=='idle' and self.bundle_count==0
            v=body.twist.twist.linear;w=body.twist.twist.angular
            command=run/'navigation_command.json'
            try:d=json.loads(command.read_text())
            except (OSError,ValueError):return False
            values=d.get('command_body',d.get('command',None))
            # Owner validates the actual bridge-file schema in integration tests.
            if values is None:
                values=[d.get('vx'),d.get('vy'),d.get('wz')]
            stable=(d.get('schema_version')==1 and d.get('mode')=='scan_slam' and d.get('source')=='scan_slam'
                and d.get('observation_source',{}).get('navigation_ground_truth_used') is False
                and 0<=time.monotonic()-float(d.get('monotonic_wall',0))<.3
                and all(abs(float(x))<1e-10 for x in d.get('requested',[1,1,1]))
                and isinstance(values,(list,tuple)) and len(values)==3
                and all(isinstance(x,(int,float)) and abs(x)<1e-10 for x in values)
                and np.linalg.norm([v.x,v.y,v.z])<.08 and abs(w.z)<.1)
            if not stable:self.stop_since_ns=None;return False
            ns=stamp_ns(body)
            if self.stop_since_ns is None:self.stop_since_ns=ns
            return ns-self.stop_since_ns>=100_000_000
        @delivery_span('receive')
        def receive(self,role,msg):
            ns=stamp_ns(msg);clock=self.get_clock().now().nanoseconds
            self.delivery_diagnostics.receive_clock(role,ns,clock)
            if self.join is not None and role in (*self.join.ROLES,*self.join.META_ROLES,*self.join.AUX_ROLES):
                accepted=self.delivery_diagnostics.call('join_put',ns,self.join.put,role,msg)
                if accepted:self.delivery_diagnostics.call('observe_source',ns,self.diagnostic_capture.observe_source,self.join,ns,int(clock))
            if ns<=self.last_published:return
            slot=self.slots.setdefault(ns,{});slot[role]=(msg,time.monotonic())
            while len(self.slots)>12:self.delivery_diagnostics.evict(self.slots)
            if all(k in slot for k in ('body','lidar','registered','raw','adapted','mapper','source_identity','point_identity','odom_identity')):
                self.publish_bundle(ns,slot,clock)
        def trace_bundle(self,ns,slot,clock,decision):
            measured_after_gate=time.monotonic()
            self.callback_trace.record('actual_canonical_bundle_publication',clock,source_ns=int(ns),
                actual_header_age_ns=int(clock-ns),initialized=self.initialized,original_decision=decision,
                original_role_source_ns={role:stamp_ns(pair[0]) for role,pair in slot.items()},
                original_role_received_wall_s={role:pair[1] for role,pair in slot.items()},
                diagnostic_role_wall_age_after_gate_s={role:measured_after_gate-pair[1] for role,pair in slot.items()},
                age_metrics_sampled_after_unchanged_original_gate=True)
            self.delivery_diagnostics.gate(ns,clock,decision)
        @delivery_span('publish_bundle')
        def publish_bundle(self,ns,slot,clock):
            if not self.initialized or not -50_000_000<=clock-ns<KNOWN_BUNDLE_FRESHNESS_NS:
                self.trace_bundle(ns,slot,clock,'original_initialized_or_header_veto')
                return
            if any(time.monotonic()-x[1]>=KNOWN_BUNDLE_FRESHNESS_S for x in slot.values()):
                self.trace_bundle(ns,slot,clock,'original_wall_age_veto')
                return
            self.trace_bundle(ns,slot,clock,'original_header_and_wall_gates_passed')
            try:
                if self.join is None or slot['odom_identity'][0].get('adapter_pid')!=self.join.witness.get('adapter_producer_pid'):
                    raise ValueError('actual adapter owner differs')
                parent=self.delivery_diagnostics.call('geometry_bundle',ns,geometry_bundle,slot,run.name,
                    self.join.witness.get('mapper_producer_pid') if self.join is not None else None,
                    self.join.witness.get('source_producer_pid') if self.join is not None else None)
            except (ValueError,KeyError,TypeError,AssertionError) as error:
                self.trace_bundle(ns,slot,clock,'explicit_source_commit_veto:'+str(error))
                return
            self.positive_capture.observe(slot,parent,clock)
            if self.transition and not self.idle_or_stopped(slot['body'][0]):
                # Freeze C until exact-zero navigation command and measured stop.
                saved=self.correction.target.copy();self.correction.target=self.correction.applied.copy()
                snapshot=self.correction.tick(ns);self.correction.target=saved
            else:snapshot=self.correction.tick(ns)
            if self.transition and not snapshot['correction_stale']:
                convergence=transaction_convergence(self.correction.applied,self.pending_target,
                    self.pending_body,self.pending_source_ns)
                if convergence['converged'] and self.idle_or_stopped(slot['body'][0]):
                    self.correction.target=self.correction.applied.copy()
                    self.transition=False;self.transaction_count+=1
                    self.release_history.write(json.dumps(dict(convergence=convergence,
                        released_source_ns=ns,generation=snapshot['generation'],
                        C_world_odom=self.correction.applied.tolist(),
                        target_C_world_odom=self.pending_target.tolist(),
                        frozen_T_odom_body=self.pending_body.tolist(),
                        measured_zero_stop_verified=True),allow_nan=False)+'\n')
            c=self.nav_world@np.asarray(snapshot['C_world_odom'])
            metadata=dict(schema='known_scene_navigation_bundle/v1',run_id=run.name,source_ns=ns,
                generation=snapshot['generation'],C_world_odom=snapshot['C_world_odom'],
                T_navigation_world=self.nav_world.tolist(),match_source_ns=snapshot['match_source_ns'],
                correction_stale=snapshot['correction_stale'],correction_transition=self.transition,
                transaction_count=self.transaction_count,control_allowed=not self.transition and not snapshot['correction_stale'],
                exact_body_lidar_registered_stamp=True,known_scene_map_assisted_navigation=True,
                robot_truth_pose_used=False,frontend_state_reset=False,frontend_velocity_or_bias_repaired=False)
            # Observe correction TTL without manufacturing an accepted match or
            # refreshing its source stamp. The applied transform remains the
            # last original correction; source/identity gates above still own it.
            if snapshot['correction_stale'] and not self.transition and diagnostic_enabled(profile):
                metadata['diagnostic_business_gate_continued']=diagnostic_record(run,profile,
                    'known_scene_correction_age','original_control_allowed_false_due_to_correction_stale',
                    dict(source_ns=ns,generation=snapshot['generation'],match_source_ns=snapshot['match_source_ns'],
                        correction_stale=True,original_control_allowed=metadata['control_allowed'],
                        correction_transition=False,source_commit_verified=True,
                        original_raw_header_and_wall_freshness_passed=True,
                        rejected_match_adopted=False,match_source_stamp_refreshed=False))
            body=self.delivery_diagnostics.call('corrected_odom',ns,corrected_odom,slot['body'][0],c);lidar=self.delivery_diagnostics.call('corrected_odom',ns,corrected_odom,slot['lidar'][0],c)
            cloud=self.delivery_diagnostics.call('corrected_cloud',ns,corrected_cloud,slot['registered'][0],c)
            metadata.update(source_commit=parent,corrected_body_message_sha256=self.delivery_diagnostics.call('message_sha',ns,message_sha,body),
                corrected_lidar_message_sha256=self.delivery_diagnostics.call('message_sha',ns,message_sha,lidar),corrected_cloud_message_sha256=self.delivery_diagnostics.call('message_sha',ns,message_sha,cloud))
            _corrected_metadata(self,ns,metadata)
            self.delivery_diagnostics.call('publish_metadata',ns,self.bundle_pub.publish,String(data=json.dumps(metadata,allow_nan=False)))
            self.delivery_diagnostics.call('publish_body',ns,self.pubs['body'].publish,body);self.delivery_diagnostics.call('publish_lidar',ns,self.pubs['lidar'].publish,lidar);self.delivery_diagnostics.call('publish_cloud',ns,self.cloud_pub.publish,cloud)
            self.last_published=ns;self.bundle_count+=1
            # One small exact published-body witness; no retained cloud payload.
            self.last_complete_body=(slot['body'][0],time.monotonic(),ns)
            self.delivery_diagnostics.call('bundle_history',ns,self.bundle_history.write,json.dumps(metadata,allow_nan=False)+'\n')
            self.diagnostic_capture.observe_metadata(metadata,int(clock))
            self.delivery_diagnostics.cleanup(ns,self.slots)
            for key in list(self.slots):
                if key<=ns:del self.slots[key]
        @delivery_span('tick')
        def tick(self):
            if not self.loaded_witness:self.load_witness()
            clock=self.get_clock().now().nanoseconds
            result=self.worker.poll()
            if result is not None:
                self.last_result=result;self.history.write(json.dumps(result,allow_nan=False)+'\n')
                if result.get('accepted') is not True and diagnostic_enabled(profile):
                    diagnostic_record(run,profile,'known_scene_match_acceptance',str(result.get('reason')),
                        dict(source_ns=result.get('source_ns'),accepted=result.get('accepted'),
                            evidence_verified=result.get('evidence_verified'),
                            candidate_rejections=[c.get('rejections',[])for c in result.get('candidates',[])],
                            rejected_match_adopted=False,last_match_source_stamp_refreshed=False))
                if not self.initialized:self.startup_results.append(copy.deepcopy(result))
                if result.get('accepted') and result.get('evidence_verified'):
                    candidate=np.asarray(result['C_world_odom'])
                    parent_matches=result.get('parent_generation')==self.correction.generation
                    continuation=self.transition and self.pending_target is not None and transform_distance(candidate,self.pending_target)[0]<.08 and transform_distance(candidate,self.pending_target)[1]<.05
                    recovery=None
                    epoch_valid=proposal_epoch_valid(result,self.correction.generation,self.pending_target,self.transition)
                    if not epoch_valid and self.transition and self.last_complete_body is not None:
                        body,received,complete_ns=self.last_complete_body
                        try:
                            if json.loads((run/'navigation_status.json').read_text()).get('state')!='running':
                                raise ValueError('Recovery cannot restart an ended navigation mission')
                            payload=(run/'scan_epoch_ready.json').read_bytes()
                            if len(payload)>65536:raise ValueError('SCAN receipt exceeds64KiB')
                            scan=json.loads(payload)
                            recovery=recovery_proposal(result,self.correction,self.pending_target,
                                self.pending_body,self.pending_source_ns,self.pending_recovery_count,clock,
                                complete_ns,received,time.monotonic(),scan,run.name,
                                self.join.witness.get('witness_sha256') if self.join is not None else None)
                        except (OSError,ValueError,TypeError):recovery=None
                        if recovery is not None:
                            if not self.idle_or_stopped(body):recovery=None
                            else:recovery['physical_zero_stop100ms_verified']=True
                    if not epoch_valid and recovery is None:
                        self.history.write(json.dumps(dict(event='stale_parent_proposal_not_adopted',source_ns=result['source_ns'],parent_generation=result.get('parent_generation'),applied_generation=self.correction.generation))+'\n')
                        result=None
                    latest=self.slots.get(max(self.slots),{}) if self.slots else {}
                    if result is not None and not self.initialized:
                        body=latest.get('body',(None,))[0]
                        if body is not None and self.idle_or_stopped(body,initial=True):
                            self.correction.propose(result);self.correction.applied=candidate.copy()
                            self.correction.generation+=1;self.initialized=True;self.transaction_count+=1
                            atomic(run/'known_scene_initialization_frame.json',dict(
                                schema='known_scene_canonical_initialization_frame/v1',run_id=run.name,
                                generation=self.correction.generation,source_ns=result['source_ns'],
                                C_world_odom=candidate.tolist(),T_navigation_world=self.nav_world.tolist(),
                                reference_sha256=sha(run/'known_scene_reference.json'),world_sha256=sha(run/'world.sdf'),
                                runtime_witness_sha256=sha(run/'known_scene_runtime_witness.json'),
                                actual_startup_results=list(self.startup_results),
                                IMU_world_orientation_used=False,robot_truth_pose_used=False,
                                frontend_state_reset=False,initial_commit_navigation_idle_verified=True))
                    elif result is not None:
                        if recovery is not None:
                            # Record an explicit ABORT, never a release. Transition
                            # remains true throughout this single owner callback.
                            self.history.write(json.dumps(recovery,allow_nan=False)+'\n')
                        self.correction.last_match_ns=result['source_ns']
                        effect=proposal_body_effect(result,self.correction.applied)
                        try:nav_state=json.loads((run/'navigation_status.json').read_text()).get('state')
                        except (OSError,ValueError):nav_state=None
                        # Keep a single frame through the bounded terminal active-hold proof.
                        start=(not self.transition and nav_state=='running' and effect['requires_transaction']) or recovery is not None
                        self.decision_history.write(json.dumps(dict(effect=effect,generation=self.correction.generation,
                            navigation_state=nav_state,transition_already_active=self.transition,
                            transaction_started=start,expired_transaction_recovery=recovery is not None,
                            original_stop_and_rate_limits_preserved=True),allow_nan=False)+'\n')
                        if start:
                            self.correction.target=candidate.copy();self.pending_target=candidate.copy()
                            self.pending_body=checked_transform(result['T_odom_body']);self.pending_source_ns=result['source_ns']
                            self.pending_recovery_count=self.pending_recovery_count+1 if recovery is not None else 0
                            self.stop_since_ns=None;self.transition=True
            if self.join is not None and clock-self.last_submit>=1_000_000_000:
                self.worker.submit_join(self.join,clock,self.correction.applied.copy(),parent_generation=self.correction.generation);self.last_submit=clock
            status=dict(schema='known_scene_navigation_status/v1',run_id=run.name,initialized=self.initialized,
                generation=self.correction.generation,correction_transition=self.transition,
                transaction_count=self.transaction_count,bundles_published=self.bundle_count,
                last_match_source_ns=self.correction.last_match_ns,last_result_reason=None if self.last_result is None else self.last_result.get('reason'),
                known_scene_map_assisted_navigation=True,independent_sensor_map=False,robot_truth_pose_used=False,
                monotonic_wall=time.monotonic(),ros_clock_ns=clock,error=self.error)
            self.status_pub.publish(String(data=json.dumps(status,allow_nan=False)))
            self.delivery_diagnostics.call('status_write',clock,atomic,run/'known_scene_status.json',status)
        def close(self):
            try:
                positive_capture=self.positive_capture.close()
                clean=self.worker.close()
                query_capture=(None if self.query_capture is None else self.query_capture.close(clean))
                self.diagnostic_capture.close(worker_drained=clean)
                self.delivery_diagnostics.close()
                self.callback_writer.close()
                self.history.close();self.bundle_history.close();self.decision_history.close();self.release_history.close()
                atomic(run/'known_scene_exit.json',dict(positive_capture=positive_capture,worker_drained=clean,bundles=self.bundle_count,
                    transactions=self.transaction_count,initialized=self.initialized,simulation_only=True,
                    match_query_capture=query_capture,
                    delivery_diagnostics=self.delivery_diagnostics.receipt()))
            finally:
                _corrected_flush(getattr(self,'_r74_corrected_delivery_witness',None))
    node=None
    try:
        node=Runtime();lifecycle.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:lifecycle.close(node)
if __name__=='__main__':main()
