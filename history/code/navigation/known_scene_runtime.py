#!/usr/bin/env python3
"""Navigation-only map correction transactions; original FAST-LIVO stays untouched.
No truth subscriptions, commands, TF, map deletion, or frontend state resets.
"""
import argparse,copy,hashlib,json,sys,time
from collections import OrderedDict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from known_scene_matcher import SceneReference,KnownSceneMatcher,CorrectionState,transform_distance,checked_transform
from known_scene_adapter import ExactSourceJoin,MatcherWorker,validate_runtime_witness,stamp_ns,odometry_transform,pointcloud_xyz

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
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True)
    args,ros=ap.parse_known_args();run=args.run.resolve();here=Path(__file__).resolve().parent
    rclpy.init(args=ros)
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
            self.matcher=KnownSceneMatcher(reference)
            from known_scene_diagnostic_capture import BoundedSourceFrames
            self.diagnostic_capture=BoundedSourceFrames(run)
            self.worker=MatcherWorker(self.matcher,self.diagnostic_capture)
            self.join=None;self.slots=OrderedDict();self.last_published=-1;self.last_submit=-1
            self.initialized=False;self.transition=False;self.pending_target=None;self.last_result=None
            self.transaction_count=0;self.bundle_count=0;self.error=None;self.loaded_witness=False;self.stop_since_ns=None
            self.history=(run/'known_scene_matches.jsonl').open('x',buffering=1)
            self.bundle_history=(run/'known_scene_bundles.jsonl').open('x',buffering=1)
            self.decision_history=(run/'known_scene_correction_decisions.jsonl').open('x',buffering=1)
            self.pubs={k:self.create_publisher(Odometry,'/demo/slam/'+k+'_odom',10) for k in ('body','lidar')}
            self.cloud_pub=self.create_publisher(PointCloud2,'/cloud_registered_full',qos_profile_sensor_data)
            self.bundle_pub=self.create_publisher(String,'/demo/navigation/localization_bundle',10)
            self.status_pub=self.create_publisher(String,'/demo/navigation/localization_status',10)
            for role,typ,topic in [('raw',PointCloud2,'/demo/teacher/raw_lidar'),
                ('adapted',PointCloud2,'/demo/slam/lidar_filtered'),('registered',PointCloud2,'/demo/slam/raw_registered_full'),
                ('body',Odometry,'/demo/slam/raw_body_odom'),('lidar',Odometry,'/demo/slam/raw_lidar_odom'),
                ('mapper',Odometry,'/aft_mapped_to_init')]:
                self.create_subscription(typ,topic,lambda m,k=role:self.receive(k,m),qos_profile_sensor_data)
            self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def load_witness(self):
            p=run/'known_scene_runtime_witness.json'
            if not p.exists():return
            witness=json.loads(p.read_text());status=validate_runtime_witness(witness,run.name)
            atomic(run/'known_scene_witness_validation.json',status)
            if not status['valid']:raise RuntimeError('Loaded producer/stage witness rejected: '+str(status['reasons']))
            self.join=ExactSourceJoin(status);self.loaded_witness=True
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
        def receive(self,role,msg):
            ns=stamp_ns(msg);clock=self.get_clock().now().nanoseconds
            if self.join is not None and role in self.join.ROLES:self.join.put(role,msg)
            if role not in ('body','lidar','registered') or ns<=self.last_published:return
            slot=self.slots.setdefault(ns,{});slot[role]=(msg,time.monotonic())
            while len(self.slots)>12:self.slots.popitem(last=False)
            if all(k in slot for k in ('body','lidar','registered')):
                self.publish_bundle(ns,slot,clock)
        def publish_bundle(self,ns,slot,clock):
            if not self.initialized or not -50_000_000<=clock-ns<300_000_000:return
            if any(time.monotonic()-x[1]>=.3 for x in slot.values()):return
            if self.transition and not self.idle_or_stopped(slot['body'][0]):
                # Freeze C until exact-zero navigation command and measured stop.
                saved=self.correction.target.copy();self.correction.target=self.correction.applied.copy()
                snapshot=self.correction.tick(ns);self.correction.target=saved
            else:snapshot=self.correction.tick(ns)
            if self.transition and transform_distance(self.correction.applied,self.pending_target)[0]<.005 \
                    and transform_distance(self.correction.applied,self.pending_target)[1]<.003:
                self.correction.target=self.correction.applied.copy()
                self.transition=False;self.transaction_count+=1
            c=self.nav_world@np.asarray(snapshot['C_world_odom'])
            metadata=dict(schema='known_scene_navigation_bundle/v1',run_id=run.name,source_ns=ns,
                generation=snapshot['generation'],C_world_odom=snapshot['C_world_odom'],
                T_navigation_world=self.nav_world.tolist(),match_source_ns=snapshot['match_source_ns'],
                correction_stale=snapshot['correction_stale'],correction_transition=self.transition,
                transaction_count=self.transaction_count,control_allowed=not self.transition and not snapshot['correction_stale'],
                exact_body_lidar_registered_stamp=True,known_scene_map_assisted_navigation=True,
                robot_truth_pose_used=False,frontend_state_reset=False,frontend_velocity_or_bias_repaired=False)
            body=corrected_odom(slot['body'][0],c);lidar=corrected_odom(slot['lidar'][0],c)
            cloud=corrected_cloud(slot['registered'][0],c)
            self.bundle_pub.publish(String(data=json.dumps(metadata,allow_nan=False)))
            self.pubs['body'].publish(body);self.pubs['lidar'].publish(lidar);self.cloud_pub.publish(cloud)
            self.last_published=ns;self.bundle_count+=1
            self.bundle_history.write(json.dumps(metadata,allow_nan=False)+'\n')
            for key in list(self.slots):
                if key<=ns:del self.slots[key]
        def tick(self):
            if not self.loaded_witness:self.load_witness()
            clock=self.get_clock().now().nanoseconds
            result=self.worker.poll()
            if result is not None:
                self.last_result=result;self.history.write(json.dumps(result,allow_nan=False)+'\n')
                if result.get('accepted') and result.get('evidence_verified'):
                    candidate=np.asarray(result['C_world_odom'])
                    parent_matches=result.get('parent_generation')==self.correction.generation
                    continuation=self.transition and self.pending_target is not None and transform_distance(candidate,self.pending_target)[0]<.08 and transform_distance(candidate,self.pending_target)[1]<.05
                    if not proposal_epoch_valid(result,self.correction.generation,self.pending_target,self.transition):
                        self.history.write(json.dumps(dict(event='stale_parent_proposal_not_adopted',source_ns=result['source_ns'],parent_generation=result.get('parent_generation'),applied_generation=self.correction.generation))+'\n')
                        result=None
                    latest=self.slots.get(max(self.slots),{}) if self.slots else {}
                    if result is not None and not self.initialized:
                        body=latest.get('body',(None,))[0]
                        if body is not None and self.idle_or_stopped(body,initial=True):
                            self.correction.propose(result);self.correction.applied=candidate.copy()
                            self.correction.generation+=1;self.initialized=True;self.transaction_count+=1
                    elif result is not None:
                        self.correction.last_match_ns=result['source_ns']
                        effect=proposal_body_effect(result,self.correction.applied)
                        try:nav_state=json.loads((run/'navigation_status.json').read_text()).get('state')
                        except (OSError,ValueError):nav_state=None
                        # Keep a single frame through the bounded terminal active-hold proof.
                        start=not self.transition and nav_state=='running' and effect['requires_transaction']
                        self.decision_history.write(json.dumps(dict(effect=effect,generation=self.correction.generation,
                            navigation_state=nav_state,transition_already_active=self.transition,
                            transaction_started=start,original_stop_and_rate_limits_preserved=True),allow_nan=False)+'\n')
                        if start:
                            self.correction.target=candidate.copy();self.pending_target=candidate.copy();self.stop_since_ns=None;self.transition=True
            if self.join is not None and clock-self.last_submit>=1_000_000_000:
                self.worker.submit_join(self.join,clock,self.correction.applied.copy(),parent_generation=self.correction.generation);self.last_submit=clock
            status=dict(schema='known_scene_navigation_status/v1',run_id=run.name,initialized=self.initialized,
                generation=self.correction.generation,correction_transition=self.transition,
                transaction_count=self.transaction_count,bundles_published=self.bundle_count,
                last_match_source_ns=self.correction.last_match_ns,last_result_reason=None if self.last_result is None else self.last_result.get('reason'),
                known_scene_map_assisted_navigation=True,independent_sensor_map=False,robot_truth_pose_used=False,
                monotonic_wall=time.monotonic(),ros_clock_ns=clock,error=self.error)
            self.status_pub.publish(String(data=json.dumps(status,allow_nan=False)))
            atomic(run/'known_scene_status.json',status)
        def close(self):
            clean=self.worker.close()
            if clean:self.diagnostic_capture.close()
            else:atomic(run/'known_scene_diagnostic_capture_receipt.json',dict(schema='known_scene_diagnostic8MiB_receipt/v1',
                status='NONFINAL_WORKER_NOT_DRAINED',final=False,worker_drained=False,
                maximum_groups=2,maximum_bytes=8*1024*1024,navigation_verified=False))
            self.history.close();self.bundle_history.close();self.decision_history.close()
            atomic(run/'known_scene_exit.json',dict(worker_drained=clean,bundles=self.bundle_count,
                transactions=self.transaction_count,initialized=self.initialized,simulation_only=True))
    node=Runtime()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
