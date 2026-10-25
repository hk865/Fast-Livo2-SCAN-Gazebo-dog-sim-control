"""Finite no-node tests of real CDR/hash joining and control ownership."""
import itertools
import json
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2
from known_scene_inputs import KnownSceneInputMixin
from point_provenance import message_sha, verify_corrected
from r86_prepared_feedback import (ros_mailbox, receipt_ns, check_negative_parking,
    SCHEMA, FRAMES, ROLES)
from ordinary_parking import OrdinaryParking


def packet(ns=1_040_000_000, commit=1, generation=1, bad_hash=False):
    body=Odometry();body.header.frame_id='camera_init';body.child_frame_id='demo_slam_body'
    body.header.stamp.sec,body.header.stamp.nanosec=divmod(ns,10**9)
    body.pose.pose.orientation.w=1.
    cloud=point_cloud2.create_cloud_xyz32(body.header,[(1.,.4,.1),(1.1,.4,.1)])
    metadata=dict(schema='known_scene_navigation_bundle/v1',run_id='fixture',source_ns=ns,
        generation=generation,robot_truth_pose_used=False,exact_body_lidar_registered_stamp=True,
        control_allowed=True,correction_transition=False,correction_stale=False,
        source_commit=dict(explicit_message_commit_token=True,producer_epoch='a'*32,
                           commit=commit,stamp_ns=ns),
        corrected_body_message_sha256=message_sha(body),
        corrected_cloud_message_sha256='bad' if bad_hash else message_sha(cloud))
    return dict(body=body,cloud=cloud,metadata=metadata)


def feed(box,data,order=ROLES,arrival=1_000_000):
    for i,role in enumerate(order):
        box.receive(role,data['metadata']['source_ns'],data[role],arrival+i)


class Base:
    def __init__(self,prepared=True):
        self.profile={'r86_prepared_feedback':dict(enabled=prepared,schema=SCHEMA)}
        self.run=Path('/fixture');self.pose_stamp=self.cloud_stamp=-1
        self.pose_updated=self.cloud_updated=-float('inf')
        self.clock=NS(nanoseconds=1_000_000_000)
        self.command=[0.,0.,0.];self.request_id='request1';self.waypoint_index=0
        self.state='running';self.feedback=None;self.cascade=None
        self.last_status=-float('inf');self.statuses=[]
        self.subscriptions={};self.observed=[];self.published=[];self.rows=[]
        self.evidence=NS(append=lambda path,row:self.rows.append(row))
        self.reject_body=self.reject_cloud=False
        self.subscribe_slam_body(Odometry,'body',self.on_odom,1)
        self.subscribe_registered_cloud(type(packet()['cloud']),'cloud',self.on_cloud,1)
    def get_clock(self):return NS(now=lambda:self.clock)
    def create_subscription(self,msg_type,topic,callback,qos,**kwargs):
        self.subscriptions[topic]=(callback,kwargs.get('callback_group'))
        return callback
    def subscribe_slam_body(self,*a,**k):return self.create_subscription(*a,**k)
    subscribe_registered_cloud=subscribe_slam_body
    def on_odom(self,msg):
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if self.reject_body or not 0<=self.clock.nanoseconds-ns<300_000_000:return
        self.pose_stamp=ns;self.pose_updated=receipt_ns(self,'body',time.monotonic_ns())/1e9
        self.feedback=dict(stamp_ns=ns,received_wall_ns=receipt_ns(self,'body',0))
        self.observed.append((ns,getattr(self,'_r86_positive_stop_allowed',True)))
    def on_cloud(self,msg):
        if self.reject_cloud:return
        self.cloud_stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.cloud_updated=receipt_ns(self,'cloud',time.monotonic_ns())/1e9
    def tick(self):pass
    def publish_status(self):self.statuses.append(self.state)
    def publish_command(self,velocity=None,yaw_rate=0.,**kwargs):
        self.command=[0.,0.,0.] if velocity is None else [*velocity,yaw_rate]
        self.published.append(tuple(self.command))
    def destroy_node(self):pass


class Node(KnownSceneInputMixin,Base):pass


class MailboxTests(unittest.TestCase):
    def test_all_six_orders_immutable_exact_identity(self):
        for order in itertools.permutations(ROLES):
            box=ros_mailbox(verify_corrected);data=packet()
            feed(box,data,order)
            data['body'].pose.pose.position.x=99.
            data['metadata']['generation']=999
            batch=box.take_batch();self.assertEqual(len(batch),1)
            body=box.decode('body',batch[0].body);metadata=box.decode('metadata',batch[0].metadata)
            self.assertEqual(body.pose.pose.position.x,0.)
            self.assertEqual(metadata['generation'],1)
            self.assertEqual(batch[0].token,('a'*32,1))
            self.assertEqual(box.snapshot()['inflight_frames'],1)
            box.release_batch(batch)
            self.assertEqual(box.snapshot()['retained_bytes'],0)

    def test_bad_hash_epoch_replay_and_duplicate_receipt(self):
        box=ros_mailbox(verify_corrected)
        feed(box,packet(bad_hash=True));self.assertFalse(box.take_batch())
        good=packet();feed(box,good,arrival=10)
        feed(box,good,arrival=999)
        batch=box.take_batch();bundle=batch[0];self.assertEqual(bundle.arrivals_ns,(10,11,12));box.release_batch(batch)
        other=packet(1_050_000_000,2);other['metadata']['source_commit']['producer_epoch']='b'*32
        feed(box,other);self.assertFalse(box.take_batch())

    def test_capacity_latches_and_good_frame_cannot_clear(self):
        box=ros_mailbox(verify_corrected)
        for i in range(FRAMES+2):feed(box,packet(1_000_000_000+i*1000,i+1))
        self.assertEqual(box.fault_reason(),'complete_bundle_overflow')
        self.assertEqual(len(box.take_batch()),FRAMES)
        feed(box,packet(2_000_000_000,100))
        self.assertFalse(box.take_batch())
        self.assertEqual(box.snapshot()['highwater_frames'],FRAMES)

    def test_verify_outside_lock_and_fixed_batch(self):
        entered=threading.Event();release=threading.Event()
        def blocked(*args):
            entered.set();self.assertTrue(release.wait(2));return verify_corrected(*args)
        box=ros_mailbox(blocked);thread=threading.Thread(target=feed,args=(box,packet()))
        thread.start();self.assertTrue(entered.wait(2))
        self.assertTrue(box.lock.acquire(timeout=.1));box.lock.release()
        self.assertFalse(box.take_batch())
        release.set();thread.join(2);self.assertFalse(thread.is_alive())
        first=box.take_batch();self.assertEqual(len(first),1)
        feed(box,packet(1_050_000_000,2))
        self.assertEqual(len(first),1);self.assertEqual(len(box.take_batch()),1)

    def test_scope_is_pinned_before_encode_and_zero_stamp_is_admitted(self):
        box=ros_mailbox(verify_corrected);box.set_scope(b'old')
        original=box.encode
        def switching(role,value):
            box.set_scope(b'new');return original(role,value)
        box.encode=switching
        feed(box,packet(0,1))
        bundle=box.take_batch()[0]
        self.assertEqual(bundle.source_ns,0)
        self.assertEqual(bundle.scopes,(b'old',b'new',b'new'))

    def test_fault_short_circuits_encoding_and_inflight_is_budgeted(self):
        box=ros_mailbox(verify_corrected);feed(box,packet())
        batch=box.take_batch();size=batch[0].size
        self.assertEqual(box.snapshot()['retained_bytes'],size)
        feed(box,packet(1_050_000_000,2))
        self.assertGreater(box.snapshot()['retained_bytes'],size)
        box.fail('injected')
        with patch.object(box,'encode',side_effect=AssertionError('must not encode')):
            feed(box,packet(1_060_000_000,3))
        box.release_batch(batch)


class IntegrationTests(unittest.TestCase):
    def node(self):
        node=Node();node.clock.nanoseconds=1_100_000_000;return node
    def test_independent_intake_and_control_only_activation(self):
        node=self.node();group=node._r86_intake_group
        self.assertIs(node.subscriptions['body'][1],group)
        self.assertIs(node.subscriptions['cloud'][1],group)
        self.assertIs(node.subscriptions['/demo/navigation/localization_bundle'][1],group)
        self.assertIsNone(node.subscriptions['/demo/navigation/localization_status'][1])
        feed(node._r86_prepared_mailbox,packet())
        self.assertEqual(node.pose_stamp,-1);self.assertIsNone(node.known_scene_inputs)
        node.tick()
        self.assertEqual(node.pose_stamp,node.cloud_stamp)
        self.assertEqual(node.known_scene_inputs['source_ns'],node.pose_stamp)
        self.assertEqual(node.feedback['received_wall_ns'],1_000_000)
        self.assertEqual(node.cloud_updated,1_000_001/1e9)

    def test_body_reject_and_partial_cloud_reject_fail_closed(self):
        for role in ('body','cloud'):
            node=self.node();setattr(node,'reject_'+role,True)
            feed(node._r86_prepared_mailbox,packet());node.tick()
            self.assertEqual(node.state,'failed');self.assertIsNone(node.known_scene_inputs)
            self.assertIn('failed',node.statuses)
            self.assertEqual(node.published[-1],(0.,0.,0.))
            node.publish_command([.1,0.],0.)
            self.assertEqual(node.published[-1],(0.,0.,0.))
            if role=='body':self.assertEqual(node.cloud_stamp,-1)

    def test_initial_zero_stamp_matches_production_minus_one_sentinel(self):
        node=self.node();node.clock.nanoseconds=10_000_000
        feed(node._r86_prepared_mailbox,packet(0,1))
        node.tick()
        self.assertEqual(node.state,'running');self.assertEqual(node.pose_stamp,0)
        self.assertIsNone(node._r86_prepared_mailbox.fault_reason())
        self.assertEqual(node.known_scene_inputs['source_ns'],0)
        self.assertEqual(node._r86_prepared_mailbox.snapshot()['retained_bytes'],0)
        node.clock.nanoseconds=1_100_000_000
        feed(node._r86_prepared_mailbox,packet(1_040_000_000,2));node.tick()
        self.assertEqual(node.pose_stamp,1_040_000_000)

    def test_scope_change_disallows_positive_stop_but_keeps_pose(self):
        node=self.node();feed(node._r86_prepared_mailbox,packet())
        node.request_id='new-request';node.tick()
        self.assertEqual(node.observed,[(1_040_000_000,False)])
        self.assertEqual(node.pose_stamp,1_040_000_000)

    def test_all_poses_and_generations_committed_once_transition_preserved(self):
        node=self.node();node.known_scene_transition=True
        for i in range(3):feed(node._r86_prepared_mailbox,packet(1_040_000_000+i*10_000_000,i+1,i+1))
        node.tick();node.tick()
        self.assertEqual([p[0] for p in node.observed],[1_040_000_000,1_050_000_000,1_060_000_000])
        self.assertEqual(node.known_scene_inputs['generation'],3)
        self.assertTrue(node.known_scene_transition)

    def test_disabled_retains_original_immediate_dispatch(self):
        node=Node(False);node.clock.nanoseconds=1_100_000_000
        data=packet()
        for role in ('body','cloud'):node.subscriptions[role][0](data[role])
        node.on_localization_bundle(String(data=json.dumps(data['metadata'])))
        self.assertEqual(node.pose_stamp,1_040_000_000)
        self.assertIsNone(node._r86_prepared_mailbox)

    def test_new_generation_never_compares_pose_to_old_parking_frame(self):
        node=self.node();node._last_authorized_hold_frame=(1,None,None)
        node.cascade=NS()
        feed(node._r86_prepared_mailbox,packet(generation=2))
        with patch('known_scene_inputs._r86_negative_parking',side_effect=AssertionError('old frame')):
            node.tick()
        self.assertEqual(node.state,'running')
        self.assertEqual(node.observed,[(1_040_000_000,False)])


class ParkingTests(unittest.TestCase):
    def make(self,ready):
        ordinary=OrdinaryParking();ordinary.begin(1_000_000_000,10_000_000_000)
        ordinary.dwell=1_000_000_000
        if ready:
            ordinary.ready_stamp=1_000_000_000;ordinary.origin=np.zeros(3);ordinary.origin_yaw=0.
        core=NS(ordinary_parking=ordinary,goal=np.zeros(3),goal_yaw=0.,goal_z_tolerance=.1,_protect=lambda *a,**k:None)
        node=NS(cascade=core,feedback=dict(stamp_ns=1_100_000_000,position_world_xyz=[0.,0.,0.],
            origin_velocity_body=[0.,0.,0.],quaternion_wxyz=[1.,0.,0.,0.]),
            paired_imu=dict(stamp_ns=1_100_000_000,angular_velocity_body=[0.,0.,0.]),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_200_000_000)))
        return node,ordinary
    def test_good_bad_good_retains_bad_and_never_grants_ready(self):
        node,ordinary=self.make(True);check_negative_parking(node)
        node.feedback['origin_velocity_body']=[.2,0.,0.]
        with self.assertRaisesRegex(ValueError,'actual_motion'):check_negative_parking(node)
        node.feedback['origin_velocity_body']=[0.,0.,0.];check_negative_parking(node)
        self.assertIn('actual_motion',ordinary.failure)
        node,ordinary=self.make(False);node.feedback['origin_velocity_body']=[.2,0.,0.]
        check_negative_parking(node);self.assertIsNone(ordinary.dwell)
        node.feedback['origin_velocity_body']=[0.,0.,0.];check_negative_parking(node)
        self.assertIsNone(ordinary.dwell);self.assertIsNone(ordinary.ready_stamp)

    def test_intermediate_z_loss_clears_original_capture_dwell(self):
        node,ordinary=self.make(False)
        node.feedback['position_world_xyz']=[0.,0.,.2]
        check_negative_parking(node)
        self.assertIsNone(ordinary.dwell)
        node.feedback['position_world_xyz']=[0.,0.,0.]
        check_negative_parking(node)
        self.assertIsNone(ordinary.dwell)

    def test_real_heading_gate_refuses_old_scope_brake_and_stop_credit(self):
        from transition_gate import TeacherHeadingGate
        from collections import deque
        gate=TeacherHeadingGate.__new__(TeacherHeadingGate)
        gate.last_stamp=None;gate.config={'stop_pose_gap_max_s':.2,'stop_planar_speed_mps':.1,
            'stop_slam_yaw_rate_radps':.1,'stop_body_gyro_radps':.1,'gyro_pose_pair_max_s':.02}
        gate.phase='settle';gate.settle_control_stage='controlled_braking'
        gate.brake_history=deque([(1.,1.)]);gate.heading=0.
        gate.handoff_event={'brake_entry_control_s':0.};gate.history=deque([(1.,1.)])
        gate.quiet_target_enabled=False;gate.active_braking_enabled=False
        gate._post_entry_brake_heading_band=lambda:.1;gate._motion_ready_for_zero=lambda *a:True
        gate.stop_window=NS(observe=lambda *a:None)
        result=gate.observe(1.1,1.1,[0.,0.,0.],0.,[0.,0.,0.],1.1,True,
                            position=[0.,0.,0.],quaternion=[0.,0.,0.,1.],positive_qualification_allowed=False)
        self.assertFalse(result);self.assertFalse(gate.brake_history);self.assertFalse(gate.history)
        self.assertEqual(gate.latest_motion['stamp_s'],1.1)


if __name__=='__main__':unittest.main(verbosity=2)
