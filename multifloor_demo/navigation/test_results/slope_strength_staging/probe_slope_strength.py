#!/usr/bin/env python3
"""Actual single-goal slope strength component; truth only evaluates results.

One actual sensor-derived calibration and real NAV world-axis [3,0,.3] on
spawn [4,2,.5]. Dynamic box disabled. No body velocity or robot pose publication,
fabricated index, direct unchecked waypoint steering, or GT feedback.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts

ROOT=Path(os.environ['DEMO_TEST_ROOT']).resolve()
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'simulation'))
from probe_slope_navigation import SlopeProbe
from slam.heading_alignment import calibrate_scene_heading, scene_waypoints_in_slam
from held_command_contract import audit_held_commands

OFFSETS=[[3.,0.,.3]]


def bounded_truth(t,times,positions,max_gap=.15):
    """Independent evaluation only: reject extrapolation and missing brackets."""
    index=int(np.searchsorted(times,t))
    if index<len(times) and abs(float(times[index])-t)<1e-9:
        return positions[index].copy(),(index,index)
    if index==0 or index==len(times):return None
    lo,hi=index-1,index
    gap=float(times[hi]-times[lo])
    if gap<=0 or gap>max_gap:return None
    alpha=(t-times[lo])/gap
    return positions[lo]+alpha*(positions[hi]-positions[lo]),(lo,hi)


class SlopeStrengthProbe(SlopeProbe):
    def __init__(self,output):
        super().__init__(output)
        self.request_id='slope-strength-physical-'+str(time.time_ns())
        self.run_id=self.request_id
        self.goals=None;self.nav_confirmations=[];self.obstacle_states=[]
        self.command_evidence=[];self.foot_contacts={};self.last_confirmed_index=0
        self.event_file=(output.parent/'dynamic_events.jsonl').open('w')
        self.mission_pub=self.create_publisher(String,'/demo/mission/state',10)
        self.enable_pub=self.create_publisher(Bool,'/demo/obstacle/enable',10)
        self.create_subscription(String,'/demo/obstacle/state',self.on_obstacle,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist,'/demo/cmd_vel',lambda m:self.on_command('requested',m),10)
        self.create_subscription(Twist,'/demo/control/safe_cmd_vel',lambda m:self.on_command('safe',m),10)
        for leg in ('lf','rf','lh','rh'):
            self.create_subscription(Contacts,'/demo/contacts/'+leg+'_foot',
                lambda m,leg=leg:self.on_foot(leg,m),qos_profile_sensor_data)

    def event(self,kind,data):
        self.event_file.write(json.dumps(dict(sim=self.now(),kind=kind,data=data))+'\n')
        self.event_file.flush()

    def on_nav(self,msg):
        super().on_nav(msg)
        if self.nav.get('request_id')!=self.request_id:return
        self.event('actual_navigation_status',self.nav)
        index=self.nav.get('waypoint_index')
        if type(index) is int and self.last_confirmed_index<index<=len(OFFSETS):
            for i in range(self.last_confirmed_index,index):
                self.nav_confirmations.append(dict(goal_index=i,actual_nav_index=index,
                    received_sim=self.now(),actual_nav_state=self.nav.get('state'),
                    raw_nav_pose=self.nav.get('pose'),actual_message=self.nav.get('message')))
            self.last_confirmed_index=index

    def on_obstacle(self,msg):
        row=dict(received_sim=self.now(),state=json.loads(msg.data))
        self.obstacle_states.append(row);self.event('actual_collidable_box',row['state'])

    def on_command(self,source,msg):
        if self.origin is None:return
        self.command_evidence.append(dict(sim=self.now(),source=source,
            command=[msg.linear.x,msg.linear.y,msg.angular.z],
            nav_request_id=self.nav.get('request_id'),nav_index=self.nav.get('waypoint_index'),
            obstacle_hold=self.nav.get('obstacle_hold'),resumes=self.nav.get('obstacle_resumes',0),
            replans=self.nav.get('replans',0),reference_stamp=self.nav.get('trajectory_reference_stamp'),
            accepted_trajectory_id=self.nav.get('accepted_trajectory_id')))

    def on_foot(self,leg,msg):
        if self.origin is None:return
        data=self.foot_contacts.setdefault(leg,dict(messages=0,nonempty=0,pairs=[],violations=[]))
        data['messages']+=1;data['nonempty']+=bool(msg.contacts)
        for c in msg.contacts:
            pair=[c.collision1.name,c.collision2.name]
            if pair not in data['pairs']:data['pairs'].append(pair)
            supports=('floor_1','floor_2','floor_3','ramp_12','ramp_23')
            if not any(any(s in name for s in supports) for name in pair):
                if pair not in data['violations']:data['violations'].append(pair)

    def tick(self):
        if self.origin is None:
            if not (len(self.pairs)>=10 and self.bridge.get('state')=='ready'
                and self.map.get('slam_healthy') and self.map.get('camera_healthy')
                and self.request_pub.get_subscription_count() and self.enable_pub.get_subscription_count()
                and self.poses):return
            try:
                alignment=calibrate_scene_heading(list(self.pairs),
                    imu_reference_world_quaternion=self.reference['world_quaternion'],
                    body_imu_quaternion=self.reference['body_imu_quaternion'],
                    reference_description=self.reference['description'])
            except ValueError as exc:
                self.last_calibration_error=str(exc);return
            self.alignment=alignment;self.origin=self.poses[-1]['p'].copy()
            self.origin_stamp=self.poses[-1]['stamp']
            self.goals=scene_waypoints_in_slam(OFFSETS,self.origin,alignment['yaw_camera_init_from_world'])
            self.baseline_holds=self.bridge.get('holds',0)
            self.command_publisher_names=[i.node_name for i in self.get_publishers_info_by_topic('/demo/cmd_vel')]
            self.request_pub.publish(String(data=json.dumps(dict(request_id=self.request_id,
                frame_id='camera_init',waypoints=self.goals))))
            self.event('one_time_actual_sensor_calibration',dict(origin=self.origin,origin_stamp=self.origin_stamp,
                heading_alignment=self.alignment,waypoints=self.goals))
        self.contact_publisher_seen |= self.count_publishers('/demo/body_contacts')>0
        if self.now()-self.origin_stamp>105:
            self.failure='component cleanup bound105sim; original per-waypoint NAV90sim remains unchanged'
        # Echo only the observed production NAV state. SceneTrigger itself checks
        # exact current_request, running state, index1, and immutable calibration.
        stage='navigating' if self.nav.get('request_id')==self.request_id and self.nav.get('state')=='running' else 'preparing_navigation'
        self.mission_pub.publish(String(data=json.dumps(dict(run_id=self.run_id,stage=stage,
            current_request=self.request_id,origin=self.origin,heading_alignment=self.alignment,
            navigation=self.nav))))
        self.enable_pub.publish(Bool(data=False))

    def result(self):
        checks={};joint=[[] for _ in OFFSETS];world_goals=None;fixed_SE3=None;precision=[];unpaired_truth=0
        post_resume_progress=[]
        active=[s['status'] for s in self.statuses if s['status'].get('request_id')==self.request_id]
        if self.origin is not None and self.truth and self.goals:
            times=np.array([r['stamp'] for r in self.truth]);tp=np.array([r['p'] for r in self.truth])
            at=self.origin_stamp
            initial_pair=bounded_truth(at,times,tp)
            if initial_pair is None:
                self.failure=self.failure or 'initial SE3 has no bounded independent truth pairing'
                return dict(scope=__doc__,passed=False,failure=self.failure,checks={},
                    missing_acceptance=['bounded_initial_truth_pair'],poses=self.poses,truth=self.truth,statuses=self.statuses)
            initial_truth,(lo,hi)=initial_pair
            quaternions=np.array([r['q'] for r in self.truth])
            initial_truth_r=(Rotation.from_quat(quaternions[lo]).as_matrix() if lo==hi else
                Slerp(times[[lo,hi]],Rotation.from_quat(quaternions[[lo,hi]]))([at]).as_matrix()[0])
            initial_slam=next(p for p in self.poses if p['stamp']==at)
            rotation=initial_truth_r@Rotation.from_quat(initial_slam['q']).as_matrix().T
            origin=np.array(initial_slam['p']);translation=initial_truth-rotation@origin
            fixed_SE3=dict(stamp=at,rotation_world_from_slam=rotation.tolist(),translation=translation.tolist(),
                scope='independent evaluation only; never used by calibration, request, mission or NAV')
            world_goals=initial_truth+np.array(OFFSETS)
            earliest=at
            for row in self.poses:
                t=row['stamp']
                if t<at or t>times[-1]:continue
                match=bounded_truth(t,times,tp)
                if match is None:
                    unpaired_truth+=1;continue
                actual=match[0]
                precision.append(float(np.linalg.norm(rotation@np.array(row['p'])+translation-actual)))
            for i,goal in enumerate(self.goals):
                confirmations=[c for c in self.nav_confirmations if c['goal_index']==i]
                limit=confirmations[0]['received_sim'] if confirmations else math.inf
                for row in self.poses:
                    t=row['stamp']
                    if t<earliest or t>times[-1] or t>limit:continue
                    seen=[s for s in self.statuses if s['sim']<=t
                          and s['status'].get('request_id')==self.request_id]
                    if not seen or seen[-1]['status'].get('waypoint_index')!=i:
                        continue
                    match=bounded_truth(t,times,tp)
                    if match is None:continue
                    actual=match[0]
                    raw_error=float(np.linalg.norm(np.array(row['p'])-goal))
                    actual_error=float(np.linalg.norm(actual-world_goals[i]))
                    # Same immutable initial SE3 is also recorded and checked.
                    rigid_goal=rotation@np.array(goal)+translation
                    rigid_error=float(np.linalg.norm(actual-rigid_goal))
                    if max(raw_error,actual_error,rigid_error)<=.30:
                        joint[i].append(dict(stamp=t,slam_error=raw_error,truth_error=actual_error,
                            fixed_SE3_goal_truth_error=rigid_error,
                            observed_NAV_index=seen[-1]['status']['waypoint_index'],
                            observed_NAV_status_sim=seen[-1]['sim']))
                if confirmations:earliest=confirmations[0]['received_sim']
            resume_commands=[c for c in self.command_evidence if c['source']=='safe'
                and c['nav_request_id']==self.request_id and c['resumes']>=1 and c['command'][0]>.015]
            if resume_commands:
                begin=resume_commands[0]['sim']
                slam_before=[p for p in self.poses if p['stamp']<=begin and begin-p['stamp']<=.15]
                actual_begin=bounded_truth(begin,times,tp)
                if slam_before and actual_begin is not None:
                    start=np.array(slam_before[-1]['p']);target=np.array(self.goals[-1])
                    unit=target[:2]-start[:2];unit/=max(float(np.linalg.norm(unit)),1e-9)
                    for p in self.poses:
                        if p['stamp']<=begin or (self.complete is not None and p['stamp']>self.complete):
                            continue
                        matched=bounded_truth(p['stamp'],times,tp)
                        if matched is None:continue
                        slam_forward=float((np.array(p['p'])[:2]-start[:2])@unit)
                        truth_forward=float(matched[0][0]-actual_begin[0][0]) # Scene's second leg is eastward.
                        if slam_forward>=.025 and truth_forward>=.025:
                            post_resume_progress.append(dict(begin_sim=begin,end_stamp=p['stamp'],
                                slam_forward_m=slam_forward,independent_truth_forward_m=truth_forward,
                                minimum_progress_m=.025));break
        held=[s for s in active if s.get('obstacle_hold')]
        held_zero=[c for c in self.command_evidence if c['source']=='safe'
            and c['nav_request_id']==self.request_id and c['obstacle_hold'] and c['command']==[0.,0.,0.]]
        requested_zero=[c for c in self.command_evidence if c['source']=='requested'
            and c['nav_request_id']==self.request_id and c['obstacle_hold'] and c['command']==[0.,0.,0.]]
        held_commands=[c for c in self.command_evidence if c['nav_request_id']==self.request_id and c['obstacle_hold']]
        resumed=[c for c in self.command_evidence if c['source']=='safe' and c['nav_request_id']==self.request_id
            and c['resumes']>=1 and c['command'][0]>.015]
        reference_after=any(s.get('obstacle_resumes',0)>=1 and s.get('accepted_trajectory_id') is not None
            and s.get('reference_requests',0)>=2 and s.get('replans',0)>held[0].get('replans',0)
            and s.get('trajectory_reference_stamp')!=held[0].get('trajectory_reference_stamp') for s in active) if held else False
        box=[s['state'] for s in self.obstacle_states if s['state'].get('mission_run_id')==self.run_id]
        phases={s.get('visible_phase') for s in box}
        native=[]
        for path in self.output.parent.glob('navigation_events/*.json'):
            entry=json.loads(path.read_text())
            if not (entry.get('edge_request_id')==entry.get('request_id')==self.request_id
                    and entry.get('edge_waypoint_index')==entry.get('waypoint_index')==1
                    and entry.get('guard_result',[False])[0] is True
                    and entry.get('zero_command')==[0.,0.,0.]):continue
            cloud_path=path.with_name(entry['cloud_file'])
            if cloud_path.is_file() and hashlib.sha256(cloud_path.read_bytes()).hexdigest()==entry['cloud_sha256']:
                native.append(dict(json_file=str(path),event=entry))
        self.curve_file.flush()
        metadata=[json.loads(line) for line in (self.output.parent/'trajectory_payloads.jsonl').open()]
        held_timing=audit_held_commands(native,self.statuses,metadata,self.command_evidence,
            self.request_id,self.goals or [])
        raw=[r for r in self.raw if self.origin_stamp is not None and r['stamp']>=self.origin_stamp]
        max_tilt=max((r['tilt'] for r in raw),default=None)
        holds=max(0,self.bridge.get('holds',0)-(self.baseline_holds or 0))
        checks.update(actual_NAV_succeeded=self.complete is not None and self.failure is None,
            actual_single_NAV_arrival=len(self.nav_confirmations)==len(OFFSETS),
            same_initial_SE3_joint_arrival=all(joint),
            actual_SCAN_accepted_steering=any(s.get('steering') and s.get('accepted_trajectory_id') is not None for s in active),
            actual_test_walk_yaw_cap_008=bool(active) and all(s.get('test_only_motion_override',{}).get('active_walk_yaw_cap')==.08 for s in active),
            dynamic_box_remains_disabled=not any(s.get('active') for s in box),
            no_unexpected_cloud_obstacle_hold=not held,
            body_contacts_absent=not self.contacts,actual_contact_publisher_present=self.contact_publisher_seen,
            feet_ground_only=all(self.foot_contacts.get(leg,{}).get('nonempty',0)>0
                and not self.foot_contacts[leg]['violations'] for leg in ('lf','rf','lh','rh')),
            tilt_envelope_kept=max_tilt is not None and max_tilt<.30 and holds==0,
            sole_body_command_publisher=self.command_publisher_names==['demo_navigation'])
        return dict(scope=__doc__,passed=bool(all(checks.values())),checks=checks,failure=self.failure,
            missing_acceptance=[k for k,v in checks.items() if not v],complete_sim=self.complete,
            origin=self.origin,origin_stamp=self.origin_stamp,goals=self.goals,world_axis_offsets=OFFSETS,initial_spawn_world=[4,2,.5],
            initial_fixed_SE3_evaluation_only=fixed_SE3,
            independent_truth_pairing_max_gap_s=.15,unpaired_truth_pose_rows=unpaired_truth,
            actual_post_resume_progress=post_resume_progress,
            truth_goals_evaluation_only=None if world_goals is None else world_goals.tolist(),
            initial_SE3_precision=dict(rmse=float(np.sqrt(np.mean(np.square(precision)))) if precision else None,
                max=max(precision,default=None)),joint_arrivals=joint,nav_confirmations=self.nav_confirmations,
            heading_alignment=self.alignment,native_obstacle_events=native,held_command_timing_audit=held_timing,
            observer_hold_labels_only=dict(nonzero_under_last_4hz_hold_label=[c for c in held_commands
                if c['command']!=[0.,0.,0.]],scope='diagnostic only; immutable native/reference timing defines actual held intervals'),
            foot_contacts=self.foot_contacts,
            active_max_imu_tilt=max_tilt,active_bridge_holds=holds,body_contacts=self.contacts,
            unchanged_limits=dict(nav_each_waypoint_sim_s=90,nav_raw_arrival_m=.22,joint_arrival_m=.30,
                obstacle_timing_sim_s=[8,20,8],component_cleanup_bound_sim_s=105),
            poses=self.poses,truth=self.truth,statuses=self.statuses,raw_imu=self.raw,
            obstacle_states=self.obstacle_states,command_evidence=self.command_evidence)


def main():
    output=Path(sys.argv[1]);output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=SlopeStrengthProbe(output);result=None
    try:
        while rclpy.ok() and node.failure is None and node.complete is None and time.monotonic()-node.started_wall<420:
            rclpy.spin_once(node,timeout_sec=.05)
        if node.failure is None and node.complete is None:node.failure='bounded component wall timeout'
        node.stop_pub.publish(Bool(data=True))
        end=time.monotonic()+3
        while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        result=node.result();output.write_text(json.dumps(result,separators=(',',':'))+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ('poses','truth','statuses','raw_imu','obstacle_states','command_evidence')},indent=2))
    except (KeyboardInterrupt,ExternalShutdownException):
        node.failure=node.failure or 'component interrupted; no success inferred'
        result=node.result();output.write_text(json.dumps(result,separators=(',',':'))+'\n')
    finally:
        if rclpy.ok():node.stop_pub.publish(Bool(data=True));node.enable_pub.publish(Bool(data=False))
        for stream in (node.curve_file,node.path_file,node.cloud_count_file,node.event_file):stream.close()
        node.destroy_node();rclpy.try_shutdown()
    return 0 if result and result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
