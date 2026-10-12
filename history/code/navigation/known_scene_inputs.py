"""Atomic pose/cloud/metadata ingestion and controlled map-frame handoff."""
import json,time
from collections import OrderedDict
from std_msgs.msg import String
from known_scene_adapter import stamp_ns

class KnownSceneInputMixin:
    def __init__(self,*args,**kwargs):
        self.known_scene_enabled=True;self.known_scene_inputs=None
        self.known_scene_status=None;self.known_scene_transition=False
        self.known_scene_pending=OrderedDict();self.known_scene_callbacks={}
        self.known_scene_reference_generation=None;self.known_scene_scan_epoch=None
        super().__init__(*args,**kwargs)
        self.create_subscription(String,'/demo/navigation/localization_bundle',self.on_localization_bundle,10)
        self.create_subscription(String,'/demo/navigation/localization_status',self.on_localization_status,10)
        self.create_subscription(String,'/demo/navigation/scan_epoch_ready',self.on_scan_epoch_ready,10)

    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        role={'/demo/slam/body_odom':'body','/cloud_registered_full':'cloud'}.get(topic)
        if role is not None:
            self.known_scene_callbacks[role]=callback
            callback=lambda msg,k=role:self._known_receive(k,stamp_ns(msg),msg)
        return super().create_subscription(msg_type,topic,callback,qos_profile,*args,**kwargs)

    def on_localization_bundle(self,msg):
        try:d=json.loads(msg.data)
        except (TypeError,ValueError):return
        if (d.get('schema')!='known_scene_navigation_bundle/v1' or d.get('run_id')!=self.run.name
                or type(d.get('source_ns')) is not int or type(d.get('generation')) is not int
                or d.get('robot_truth_pose_used') is not False or not d.get('exact_body_lidar_registered_stamp')):return
        self._known_receive('metadata',d['source_ns'],d)

    def _known_receive(self,role,ns,value):
        if ns<=self.pose_stamp:return
        slot=self.known_scene_pending.setdefault(ns,{})
        slot[role]=value
        while len(self.known_scene_pending)>24:self.known_scene_pending.popitem(last=False)
        if all(k in slot for k in ('body','cloud','metadata')):
            self.known_scene_inputs={**slot['metadata'],'received_monotonic_wall':time.monotonic()}
            # Single executor callback: no control timer between the two inputs.
            self.known_scene_callbacks['body'](slot['body'])
            self.known_scene_callbacks['cloud'](slot['cloud'])
            self.evidence.append(self.run/'known_scene_controller_inputs.jsonl',dict(
                source_ns=ns,generation=slot['metadata']['generation'],
                accepted_pose_stamp_ns=int(self.pose_stamp),accepted_cloud_stamp_ns=int(self.cloud_stamp),
                control_allowed=slot['metadata']['control_allowed'],
                fixed_reference_frame='known scene world minus declared navigation origin',
                navigation_ground_truth_used=False))
            for key in list(self.known_scene_pending):
                if key<=ns:del self.known_scene_pending[key]

    def on_localization_status(self,msg):
        try:d=json.loads(msg.data)
        except (TypeError,ValueError):return
        if d.get('schema')!='known_scene_navigation_status/v1' or d.get('run_id')!=self.run.name:return
        self.known_scene_status={**d,'received_monotonic_wall':time.monotonic()}
        transition=d.get('correction_transition') is True
        if transition and not self.known_scene_transition:
            self.publish_command()
            self.samples=None;self.path_admission_active=None;self.active_trajectory_id=None
            self.cascade=None;self.cascade_key=None;self.cascade_last_row=None
            self.path_receipt=None;self.cascade_path_id=None;self.final_bundle=None
            self.execution_path_exhaustion=None;self.pid_prepared_output=None
            self.active_reference_stamp=None;self.trajectory_association.reset()
            self.planning_start_state=None;self.last_reference=-float('inf')
            self.reset_region_arrival('known_scene_correction_transaction')
            self.evidence.append(self.run/'known_scene_path_transactions.jsonl',dict(
                event='hold_and_invalidate_prior_SCAN_path',generation=d['generation'],
                source_pose_stamp_ns=int(self.pose_stamp),navigation_ground_truth_used=False))
        if not transition and self.known_scene_transition:
            self.path_admission_hold=False;self.path_admission_replan_pending=False
            self.samples=None;self.last_reference=-float('inf')
            self.evidence.append(self.run/'known_scene_path_transactions.jsonl',dict(
                event='commit_same_frame_inputs_then_replan',generation=d['generation'],
                ordered_region_receipts_preserved=len(self.region_arrivals),navigation_ground_truth_used=False))
        self.known_scene_transition=transition

    def on_scan_epoch_ready(self,msg):
        try:d=json.loads(msg.data)
        except (TypeError,ValueError):return
        if d.get('schema')=='known_scene_SCAN_epoch_ready/v1' and d.get('run_id')==self.run.name:
            self.known_scene_scan_epoch={**d,'received_monotonic_wall':time.monotonic()}

    def _known_generation_ready(self):
        d=self.known_scene_inputs;s=self.known_scene_status;e=self.known_scene_scan_epoch;now=time.monotonic()
        return bool(d is not None and s is not None and e is not None
            and all(0<=now-v['received_monotonic_wall']<.3 for v in (d,s,e))
            and d.get('control_allowed') and not s.get('correction_transition') and s.get('initialized')
            and e.get('ready') and d['generation']==s['generation']==e['generation']
            and self.pose_stamp==self.cloud_stamp==d['source_ns'])

    def _control_candidate(self,now):
        if not self._known_generation_ready():
            if self.control_clock_hold.before_control(self,self.get_clock().now().nanoseconds,round(now*1e9)):
                return
            # Frame holds cannot bypass original deadlines or raw-source/tilt protection.
            if self.state=='running':
                ns=self.get_clock().now().nanoseconds
                if (self.segment_start is not None and not self.goals[self.waypoint_index].legacy
                        and self.segment_deadline_exceeded(ns)):
                    self.state,self.message='failed','定位等待期间原区域超过仿真时间限制，停车'
                    self.reset_region_arrival('failed');self.publish_command();return
                self.apply_tilt_guard(now,ns/1e9)
                stale=(now-self.pose_updated>self.pose_timeout or now-self.cloud_updated>self.cloud_timeout
                    or not self.raw_imu.fresh(now,ns/1e9))
                if stale:
                    self.stale_since=self.stale_since or now
                    if now-self.stale_since>8.:
                        self.state,self.message='failed','定位等待期间原始传感器连续失联超过8秒，停车'
                else:self.stale_since=None
                if self.state=='failed':
                    self.reset_region_arrival('failed');self.publish_command();return
            self.publish_command();self.reset_region_arrival('known_scene_fresh_transaction_hold')
            if self.state!='failed':
                self.message='等待同一定位版本的新鲜位姿、点云或停车修正事务'
            return
        return super()._control_candidate(now)

    def request_plan(self):
        if not self._known_generation_ready():return
        generation=None if self.known_scene_inputs is None else self.known_scene_inputs['generation']
        before=self.reference_requests
        result=super().request_plan()
        if self.reference_requests>before:self.known_scene_reference_generation=generation
        return result

    def accept_spline(self,msg,metadata):
        if (not self._known_generation_ready()
                or self.known_scene_reference_generation!=self.known_scene_inputs['generation']):
            self.trajectory_association.reject('known_scene_epoch_not_ready_or_old_request_generation')
            return False
        return super().accept_spline(msg,metadata)
