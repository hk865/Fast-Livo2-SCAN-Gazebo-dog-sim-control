#!/usr/bin/env python3
"""Actual first four Full18 canonical v2 regions, one sensor calibration.

Excluded feedback A/B component; unchanged NAV exclusively publishes motion.
Fresh default spawn [0,0,.30], box disabled, production caps and guards intact.
Truth only evaluates original native receipt windows under one initial SE3.
"""
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts

ROOT=Path(os.environ['DEMO_TEST_ROOT']).resolve()
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'simulation'))
spec=importlib.util.spec_from_file_location('production_first4_parent',ROOT/'simulation/probe_slope_navigation.py')
parent=importlib.util.module_from_spec(spec);spec.loader.exec_module(parent)
from slam.heading_alignment import calibrate_scene_heading
from mission.route_regions import transformed_route_goals,validate_route_regions
from navigation.goal_regions import definitions_sha256
from first_four_region_contract import evaluate_regions

def native_stamp(msg):return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
def serialize(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)
def encoded(value):return json.dumps(value,default=serialize,allow_nan=False,separators=(',',':'))


class FirstFourRegionProbe(parent.SlopeProbe):
    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        if topic=='/demo/test/joint_stop_safety':topic='/demo/control/joint_stop_safety'
        if topic=='/demo/ground_truth':qos_profile=QoSProfile(depth=2000,reliability=ReliabilityPolicy.BEST_EFFORT)
        return super().create_subscription(msg_type,topic,callback,qos_profile,*args,**kwargs)

    def __init__(self,output):
        mode=os.environ.get('DEMO_TEST_BODY_FEEDBACK_ENABLED')
        if mode not in ('0','1'):raise ValueError('DEMO_TEST_BODY_FEEDBACK_ENABLED must be 0 or 1')
        super().__init__(output)
        self.request_id='first-four-regions-'+str(time.time_ns());self.run_id=self.request_id
        self.scenario_bytes=(ROOT/'simulation/scenario.json').read_bytes()
        self.scenario=json.loads(self.scenario_bytes);validate_route_regions(self.scenario)
        if self.scenario.get('goal_region_profile',{}).get('name')!='three_platform_body_arrival_v2':
            raise ValueError('Current preregistered Full18 v2 scenario required')
        if len(self.scenario['route_goals']['exploration'][:4])!=4:raise ValueError('four canonical goals required')
        self.expected_feedback_enabled=mode=='1';self.feedback_health_established=False
        self.feedback_health_mode_errors=[];self.bridge_events=[];self.adapter_events=[]
        self.goals=();self.goal_hash=None;self.origin_stamp_ns=None;self.terminal_stamp_ns=None
        self.fourth_segment_entry_stamp_ns=None;self.fourth_segment_previous_receipt_stamp_ns=None
        self.command_evidence=[];self.foot_contacts={};self.zero_return_observed=False
        self.event_file=(output.parent/'first_four_events.jsonl').open('w',buffering=1024*1024)
        self.mission_pub=self.create_publisher(String,'/demo/mission/state',10)
        self.enable_pub=self.create_publisher(Bool,'/demo/obstacle/enable',10)
        for source,topic in [('requested','/demo/cmd_vel'),('safe','/demo/control/safe_cmd_vel'),
                             ('actuator','/demo/control/actuator_cmd_vel')]:
            self.create_subscription(Twist,topic,lambda m,s=source:self.on_command(s,m),10)
        for leg in ('lf','rf','lh','rh'):
            self.create_subscription(Contacts,'/demo/contacts/'+leg+'_foot',
                lambda m,leg=leg:self.on_foot(leg,m),qos_profile_sensor_data)

    def now_ns(self):return self.get_clock().now().nanoseconds
    def event(self,kind,value):
        self.event_file.write(encoded(dict(received_stamp_ns=self.now_ns(),wall=time.monotonic(),kind=kind,data=value))+'\n')

    def abort(self,reason):
        if self.failure is None:self.failure=reason
        if self.terminal_stamp_ns is None:self.terminal_stamp_ns=self.now_ns()
        self.stop_pub.publish(Bool(data=True))

    def on_slam(self,msg):
        super().on_slam(msg)
        self.poses[-1]['stamp_ns']=native_stamp(msg)

    def on_truth(self,msg):
        super().on_truth(msg)
        self.truth[-1]['stamp_ns']=native_stamp(msg)

    def on_imu(self,msg):
        super().on_imu(msg)
        self.raw[-1]['stamp_ns']=native_stamp(msg)

    def on_nav(self,msg):
        self.nav=json.loads(msg.data);ns=self.now_ns()
        self.statuses.append(dict(sim=ns/1e9,received_stamp_ns=ns,status=self.nav))
        if self.nav.get('request_id')!=self.request_id:return
        self.event('actual_navigation_status',self.nav)
        if self.nav.get('goals_definition_sha256')!=self.goal_hash or self.nav.get('goals_definitions')!=[g.definition() for g in self.goals]:
            self.abort('actual NAV goal declaration/hash mismatch');return
        index=self.nav.get('waypoint_index')
        if index==3 and self.fourth_segment_entry_stamp_ns is None:
            self.fourth_segment_entry_stamp_ns=ns
            receipts=self.nav.get('region_arrivals',[])
            self.fourth_segment_previous_receipt_stamp_ns=receipts[2].get('stamp_ns') if len(receipts)>=3 else None
        if self.nav.get('state')=='failed':
            self.abort('actual NAV failed: '+str(self.nav.get('message')))
        elif self.nav.get('state')=='succeeded':
            receipts=self.nav.get('region_arrivals')
            if index!=4 or self.nav.get('total')!=4 or not isinstance(receipts,list) or len(receipts)!=4:
                self.abort('NAV succeeded without actual fourth native receipt');return
            self.complete=ns/1e9
            if self.terminal_stamp_ns is None:self.terminal_stamp_ns=ns
            self.stop_pub.publish(Bool(data=True))

    def on_bridge(self,msg):
        self.bridge=json.loads(msg.data);row=dict(sim=self.now(),received_stamp_ns=self.now_ns(),safety=self.bridge)
        self.bridge_events.append(row)
        feedback=self.bridge.get('body_feedback')
        valid=(self.bridge.get('body_feedback_required') is True and isinstance(feedback,dict)
            and type(feedback.get('enabled')) is bool and feedback['enabled']==self.expected_feedback_enabled)
        if valid:self.feedback_health_established=True
        elif self.feedback_health_established or self.origin is not None or feedback is not None:
            self.feedback_health_mode_errors.append(row);self.abort('required body feedback mode missing or mismatched')
        if self.bridge.get('state')=='failed':self.abort('aggregate execution safety failed: '+str(self.bridge.get('reason')))
        self.event('aggregate_execution_safety',self.bridge)

    def on_adapter(self,msg):
        data=json.loads(msg.data);self.adapter_events.append(dict(sim=self.now(),received_stamp_ns=self.now_ns(),status=data))
        if data.get('failed'):self.abort('actual modern joint adapter failed: '+str(data.get('reason')))
        self.event('actual_modern_adapter_status',data)

    def on_contact(self,msg):
        if self.origin is not None and msg.contacts:
            self.contacts.append(dict(stamp_ns=native_stamp(msg),received_stamp_ns=self.now_ns(),count=len(msg.contacts)))
            self.abort('physical body contact')

    def on_foot(self,leg,msg):
        if self.origin is None:return
        data=self.foot_contacts.setdefault(leg,dict(messages=0,nonempty=0,pairs=[],violations=[]))
        data['messages']+=1;data['nonempty']+=bool(msg.contacts)
        for c in msg.contacts:
            pair=[c.collision1.name,c.collision2.name]
            if pair not in data['pairs']:data['pairs'].append(pair)
            if not any('floor_1' in name for name in pair):
                if pair not in data['violations']:data['violations'].append(pair)

    def on_command(self,source,msg):
        if self.origin is None:return
        self.command_evidence.append(dict(source=source,received_stamp_ns=self.now_ns(),wall=time.monotonic(),
            value=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]))

    def tick(self):
        if self.failure is not None:return
        self.enable_pub.publish(Bool(data=False))
        if self.origin is None:
            if not (len(self.pairs)>=10 and self.bridge.get('state')=='ready' and self.feedback_health_established
                    and self.map.get('slam_healthy') and self.map.get('camera_healthy') and self.poses
                    and self.request_pub.get_subscription_count() and self.enable_pub.get_subscription_count()):return
            try:
                alignment=calibrate_scene_heading(list(self.pairs),
                    imu_reference_world_quaternion=self.reference['world_quaternion'],
                    body_imu_quaternion=self.reference['body_imu_quaternion'],reference_description=self.reference['description'])
            except ValueError as exc:self.last_calibration_error=str(exc);return
            self.alignment=alignment;self.origin=self.poses[-1]['p'].copy()
            self.origin_stamp=self.poses[-1]['stamp'];self.origin_stamp_ns=self.poses[-1]['stamp_ns']
            self.goals=transformed_route_goals(self.scenario,'exploration',self.origin,self.alignment)[:4]
            self.goal_hash=definitions_sha256(self.goals);self.baseline_holds=self.bridge.get('holds',0)
            self.command_publisher_names=[x.node_name for x in self.get_publishers_info_by_topic('/demo/cmd_vel')]
            payload=dict(schema_version=2,request_id=self.request_id,frame_id='camera_init',goals=[g.definition() for g in self.goals])
            self.request_pub.publish(String(data=encoded(payload)))
            self.event('one_time_actual_sensor_calibration',dict(origin=self.origin,origin_stamp_ns=self.origin_stamp_ns,
                heading_alignment=self.alignment,request=payload,goals_definition_sha256=self.goal_hash))
        self.contact_publisher_seen|=self.count_publishers('/demo/body_contacts')>0
        if self.now_ns()-self.origin_stamp_ns>400_000_000_000:
            self.abort('component cleanup bound400sim; actual per-goal90s remains unchanged');return
        self.mission_pub.publish(String(data=encoded(dict(run_id=self.run_id,
            stage='exploring' if self.nav.get('request_id')==self.request_id and self.nav.get('state')=='running' else 'preparing_navigation',
            current_request=self.request_id,origin=self.origin,heading_alignment=self.alignment,navigation=self.nav))))

    def tail_ready(self):
        if self.terminal_stamp_ns is None or not self.adapter_events:return False
        adapter=self.adapter_events[-1]
        if adapter['received_stamp_ns']<self.terminal_stamp_ns or adapter['status'].get('state')!='idle' or adapter['status'].get('failed') or adapter['status'].get('nominal_calibrated') is not True:return False
        for source in ('requested','safe','actuator'):
            matching=[r for r in self.command_evidence if r['source']==source and r['received_stamp_ns']>=self.terminal_stamp_ns]
            if not matching or matching[-1]['value']!=[0.]*6:return False
        return True

    def result(self):
        end=self.terminal_stamp_ns
        evaluation=evaluate_regions(self.goals,self.request_id,self.statuses,self.poses,self.truth,self.origin_stamp_ns,end) if self.goals else dict(passed=False,checks={},regions=[],errors=['actual request/calibration never occurred'])
        active=[x['status'] for x in self.statuses if x['status'].get('request_id')==self.request_id]
        health=[r for r in self.bridge_events if self.origin_stamp_ns is not None and self.origin_stamp_ns<=r['received_stamp_ns']<=(end or self.now_ns())]
        raw=[r for r in self.raw if self.origin_stamp_ns is not None and self.origin_stamp_ns<=r['stamp_ns']<=(end or self.now_ns())]
        max_tilt=max((r['tilt'] for r in raw),default=None)
        holds=max((r['safety'].get('holds',0)-(self.baseline_holds or 0) for r in health),default=0)
        checks=dict(evaluation.get('checks',{}))
        checks.update(no_actual_failure=self.failure is None,actual_fourth_receipt_completed=self.complete is not None,
            body_feedback_required_mode=bool(health) and not self.feedback_health_mode_errors and all(r['safety'].get('body_feedback_required') is True
                and isinstance(r['safety'].get('body_feedback'),dict) and type(r['safety']['body_feedback'].get('enabled')) is bool
                and r['safety']['body_feedback']['enabled']==self.expected_feedback_enabled for r in health),
            aggregate_execution_safety_not_failed=bool(health) and not any(r['safety'].get('state')=='failed' for r in self.bridge_events),
            actual_production_limits=bool(active) and all(s.get('motion_limits')==dict(walk_yaw_rate_rad_s=.08,pure_turn_rate_rad_s=.12,forward_speed_m_s=.12) for s in active),
            modern_joint_adapter=bool(self.adapter_events) and all(r['status'].get('test_only') is False for r in self.adapter_events),
            actual_SCAN_used=bool(active) and any(s.get('accepted_trajectory_id') is not None and s.get('steering') for s in active),
            no_unexpected_obstacle_hold=not any(s.get('obstacle_hold') for s in active),
            no_body_contacts=not self.contacts,body_contact_publisher_seen=self.contact_publisher_seen,
            four_feet_ground_only=all(self.foot_contacts.get(leg,{}).get('nonempty',0)>0 and not self.foot_contacts[leg]['violations'] for leg in ('lf','rf','lh','rh')),
            active_tilt_component_envelope=max_tilt is not None and max_tilt<.30 and holds==0,
            sole_NAV_body_command_publisher=self.command_publisher_names==['demo_navigation'],
            actual_safe_zero_and_nominal_return=self.zero_return_observed)
        return dict(scope=__doc__,passed=evaluation['passed'] and all(checks.values()),checks=checks,
            failure=self.failure,missing_acceptance=[k for k,v in checks.items() if not v],request_id=self.request_id,
            selected_body_feedback_enabled=self.expected_feedback_enabled,feedback_health_mode_errors=self.feedback_health_mode_errors,
            initial_spawn_world=[0,0,.30],origin=self.origin,origin_stamp=self.origin_stamp,origin_stamp_ns=self.origin_stamp_ns,
            terminal_sim=None if end is None else end/1e9,terminal_stamp_ns=end,complete_sim=self.complete,
            fourth_segment_entry_stamp_ns=self.fourth_segment_entry_stamp_ns,
            fourth_segment_previous_receipt_stamp_ns=self.fourth_segment_previous_receipt_stamp_ns,
            goals_definitions=[g.definition() for g in self.goals],goals_definition_sha256=self.goal_hash,
            scenario_sha256=hashlib.sha256(self.scenario_bytes).hexdigest(),heading_alignment=self.alignment,
            region_evaluation=evaluation,active_max_imu_tilt=max_tilt,active_bridge_holds=holds,
            foot_contacts=self.foot_contacts,body_contacts=self.contacts,
            unchanged_contract=dict(per_goal_timeout_sim_s=90,raw_inner_dwell_sim_s=.4,max_raw_gap_sim_s=.2,
                independent_GT_outer='same preregistered outer region',truth_bracket_max_s=.15,
                imu_hold_rad=.30,imu_fail_rad=.50,component_cleanup_bound_sim_s=400),
            poses=self.poses,truth=self.truth,statuses=self.statuses,raw_imu=self.raw,
            command_evidence=self.command_evidence,aggregate_execution_safety_events=self.bridge_events,
            modern_adapter_statuses=self.adapter_events)


def main():
    output=Path(sys.argv[1]);output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=FirstFourRegionProbe(output);result=None
    try:
        while rclpy.ok() and node.failure is None and node.complete is None and time.monotonic()-node.started_wall<560:
            rclpy.spin_once(node,timeout_sec=.05)
        if node.failure is None and node.complete is None:node.abort('bounded component wall timeout')
        node.stop_pub.publish(Bool(data=True));node.enable_pub.publish(Bool(data=False))
        until=time.monotonic()+3
        while rclpy.ok() and time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
            if node.tail_ready():node.zero_return_observed=True;break
        if node.complete is not None and not node.zero_return_observed:node.failure=node.failure or 'bounded post-fourth zero/nominal return not observed'
        result=node.result();output.write_text(encoded(result)+'\n')
        print(encoded({k:v for k,v in result.items() if k not in {'poses','truth','statuses','raw_imu','command_evidence','aggregate_execution_safety_events','modern_adapter_statuses'}}))
    except (KeyboardInterrupt,ExternalShutdownException):
        node.failure=node.failure or 'component interrupted; no physical success inferred'
        if node.terminal_stamp_ns is None:node.terminal_stamp_ns=node.now_ns()
        result=node.result();output.write_text(encoded(result)+'\n')
    finally:
        try:
            if rclpy.ok():
                try:node.stop_pub.publish(Bool(data=True));node.enable_pub.publish(Bool(data=False))
                except _ros.RCLError:
                    if rclpy.ok():raise
        finally:
            for stream in (node.curve_file,node.path_file,node.cloud_count_file,node.event_file):stream.close()
            node.destroy_node();rclpy.try_shutdown()
    return 0 if result and result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
