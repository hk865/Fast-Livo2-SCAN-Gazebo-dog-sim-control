#!/usr/bin/env python3
"""STAGING, disabled by default. Body-pose feedback and pre-adapter stop gate.

Never reads SLAM, GT, robot world position or a navigation heading/goal.
Not launch-ready until its isolated command wiring/stop ACK is reviewed/tested.
"""
import argparse,collections,json,math,os,pathlib,sys,time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Imu,JointState
from ros_gz_interfaces.msg import Contacts
from geometry_msgs.msg import Twist,Pose
from std_msgs.msg import String
from feedback_core import Config,Feedback

SIM = pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(SIM))
from joint_stop_core import JointGeometry

class Kinematics(JointGeometry):
    def measured(self,q,qd):
        if not self.valid(q) or np.asarray(qd).shape!=(12,) or not np.isfinite(qd).all():raise ValueError('invalid actual joint state')
        feet=[];speeds=[]
        for i,leg in enumerate(('lf','rf','lh','rh')):
            pos=np.zeros(3);R=np.eye(3);origins=[];axes=[]
            for j,suffix in enumerate(('hip','upper_leg','lower_leg','foot')):
                element=self.joints[f'{leg}_{suffix}_joint'];origin=element.find('origin')
                pos+=R@np.fromstring(origin.attrib['xyz'],sep=' ')
                R=R@Rotation.from_euler('xyz',np.fromstring(origin.attrib['rpy'],sep=' ')).as_matrix()
                if j<3:
                    axis=np.fromstring(element.find('axis').attrib['xyz'],sep=' ')
                    origins.append(pos.copy());axes.append(R@axis)
                    R=R@Rotation.from_rotvec(axis*q[i*3+j]).as_matrix()
            J=np.stack([np.cross(axis,pos-o) for axis,o in zip(axes,origins)],axis=1)
            feet.append(pos.copy());speeds.append(J@qd[i*3:i*3+3])
        return np.array(feet),np.array(speeds)

def values(m):return np.array([m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z])
def twist(v):
    m=Twist();m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z=map(float,v);return m
def stamp_ns(m):return int(m.header.stamp.sec)*1_000_000_000+int(m.header.stamp.nanosec)
def stamp(m):return stamp_ns(m)/1_000_000_000

def measured_contact_mask(history,joint_stamp,wall):
    """Only contact observations at/before the paired JointState instant."""
    mask=[]
    for leg in ('lf','rf','lh','rh'):
        item=next((row for row in reversed(history.get(leg,())) if row['stamp']<=joint_stamp+1e-9),None)
        mask.append(item is not None and item['nonempty']
            and -.000000001<=joint_stamp-item['stamp']<=.040001 and wall-item['wall']<=.10)
    return np.array(mask)

def newest_complete_joint(joints,imus):
    """Use an actual completed sensor-time pair, never a future joint sample."""
    if not imus:return None,None
    latest=imus[-1]['stamp_ns']
    for joint in reversed(joints):
        if joint['stamp_ns']>latest:continue
        if latest-joint['stamp_ns']>30_000_000:break
        imu=min(imus,key=lambda row:abs(row['stamp_ns']-joint['stamp_ns']))
        if abs(imu['stamp_ns']-joint['stamp_ns'])<=5_000_000:return joint,imu
    return None,None

class Stabilizer(Node):
    def __init__(self,a):
        super().__init__('demo_staging_body_stabilizer')
        self.f=Feedback(Config(enabled=a.enable));self.started=time.monotonic()
        self.geometry=Kinematics((SIM/'generated/go2.urdf').read_text(),.30,0.)
        scenario=json.loads((SIM/'scenario.json').read_text())['sensors']['imu']['orientation_reference']
        self.R_BI=Rotation.from_quat(scenario['body_imu_quaternion']).as_matrix()
        self.R_WR=Rotation.from_quat(scenario['world_quaternion']).as_matrix()
        self.imu_history=collections.deque();self.joint_history=collections.deque();self.imu=None;self.joints=None
        self.contacts={};self.request=np.zeros(6);self.command_wall=None;self.actual=np.zeros(6);self.actual_wall=None
        self.adapter={};self.adapter_wall=None;self.last_status=0.;self.last_support_joint_stamp=None
        self.gate_ready=False;self.state='boot';self.published_commands=0
        self.last_published_pose=np.zeros(2);self.neutral_wait_wall=None
        self.input_predicates={}
        self.log=pathlib.Path(a.output).open('w',buffering=1024*1024)
        self.cmd_pub=self.create_publisher(Twist,'/demo/test/body_stabilizer/safe_cmd_vel',10)
        self.pose_pub=self.create_publisher(Pose,'/demo/test/body_stabilizer/body_pose',1)
        self.status_pub=self.create_publisher(String,'/demo/test/body_stabilizer/status',10)
        self.create_subscription(Imu,'/livox/imu',self.on_imu,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(JointState,getattr(a,'joint_topic','/joint_states'),self.on_joints,qos_profile_sensor_data)
        self.create_subscription(Twist,'/demo/control/safe_cmd_vel',self.on_command,10)
        adapter_namespace='/demo/test' if a.legacy_adapter_topics else '/demo/control'
        self.create_subscription(Twist,adapter_namespace+'/actuator_cmd_vel',self.on_actual,10)
        self.create_subscription(String,adapter_namespace+'/joint_stop_safety',self.on_adapter,10)
        for leg in ('lf','rf','lh','rh'):
            self.create_subscription(Contacts,f'/demo/contacts/{leg}_foot',lambda m,l=leg:self.on_contacts(l,m),
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        # Wall watchdog; reference/filter progress always uses actual IMU time.
        self.create_timer(.01,self.tick)
    def emit_pose(self,q):
        m=Pose();r=Rotation.from_euler('xyz',[q[0],q[1],0]).as_quat()
        m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w=map(float,r)
        self.pose_pub.publish(m);self.last_published_pose=np.array(q).copy()
    def zero_then_neutral(self):
        # Cross-topic publication order is NOT a delivery-order guarantee.
        # Require an actual adapter stop-state acknowledgement before changing
        # native posture, so its q0 is already frozen. Body Twist is zero now.
        self.cmd_pub.publish(Twist())
        neutral_safe=(not np.any(abs(self.last_published_pose)>1e-12)
            or self.adapter.get('state') in ('wait_native_zero','returning','idle','failed'))
        if neutral_safe:
            waited=self.neutral_wait_wall is not None
            self.emit_pose(np.zeros(2));self.neutral_wait_wall=None
            if waited:
                self.log.write(json.dumps(dict(event='identity_after_adapter_stop_ack',
                    wall_monotonic=time.monotonic(),adapter_state=self.adapter.get('state'),
                    imu_stamp=None if self.imu is None else self.imu['stamp']))+'\n')
        else:
            self.neutral_wait_wall=self.neutral_wait_wall or time.monotonic()
            if time.monotonic()-self.neutral_wait_wall>.12:
                self.f.fail('adapter stop acknowledgement missing; keep bounded last posture, body Twist zero')
                self.gate_ready=False;self.state='failed'
    def fault(self,reason):
        self.f.fail(reason);self.gate_ready=False;self.state='failed';self.zero_then_neutral()
    def on_command(self,m):
        v=values(m);self.command_wall=time.monotonic()
        if not np.isfinite(v).all():self.fault('nonfinite upstream safe command');return
        self.request=v.copy()
        if not np.any(abs(v)>1e-8):self.zero_then_neutral();return
        if not self.f.failed and (not self.f.c.enabled or self.gate_ready):self.cmd_pub.publish(m)
        else:self.zero_then_neutral()
    def on_actual(self,m):
        v=values(m)
        if not np.isfinite(v).all():self.fault('nonfinite actual actuator command');return
        self.actual=v;self.actual_wall=time.monotonic()
    def on_adapter(self,m):
        try:
            data=json.loads(m.data)
            if (not isinstance(data,dict) or data.get('state') not in
                    ('calibrating','idle','walk','wait_native_zero','returning','failed')
                    or type(data.get('failed')) is not bool
                    or type(data.get('nominal_calibrated')) is not bool):
                raise ValueError('unexpected adapter schema')
            self.adapter=data;self.adapter_wall=time.monotonic()
        except (ValueError,TypeError):self.fault('invalid adapter status');return
        if self.adapter.get('failed'):self.fault('joint-reference adapter failed')
    def on_imu(self,m):
        ns=stamp_ns(m);t=ns/1_000_000_000;q=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w]
        w=np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
        if not np.isfinite([t,*q,*w]).all() or abs(np.linalg.norm(q)-1)>.01:
            self.fault('invalid raw IMU');return
        if self.imu is not None and ns<=self.imu['stamp_ns']:return
        R=self.R_WR@Rotation.from_quat(q).as_matrix()@self.R_BI.T
        self.imu=dict(stamp=t,stamp_ns=ns,wall=time.monotonic(),rotation=R,gyro=self.R_BI@w)
        self.imu_history.append(self.imu)
        while self.imu_history and ns-self.imu_history[0]['stamp_ns']>1_000_000_000:self.imu_history.popleft()
    def on_joints(self,m):
        try:
            if len(set(m.name))!=len(m.name):raise ValueError('duplicate joint names')
            ids=[m.name.index(n) for n in self.geometry.names]
            q=np.array([m.position[i] for i in ids]);qd=np.array([m.velocity[i] for i in ids])
            feet,vf=self.geometry.measured(q,qd)
            ns=stamp_ns(m);t=ns/1_000_000_000
            if not math.isfinite(t):raise ValueError('nonfinite joint timestamp')
            if self.joints is not None and ns<=self.joints['stamp_ns']:return
            self.joints=dict(stamp=t,stamp_ns=ns,wall=time.monotonic(),feet=feet,velocities=vf)
            self.joint_history.append(self.joints)
            while self.joint_history and ns-self.joint_history[0]['stamp_ns']>250_000_000:self.joint_history.popleft()
        except (ValueError,IndexError,TypeError) as e:
            if self.f.established:self.fault('invalid measured joints: '+str(e))
    def on_contacts(self,leg,m):
        # Fresh actual contact events only; no synthesized gait-phase contacts.
        allowed={'floor_1','floor_2','floor_3','ramp_12','ramp_23'}
        foot=f'go2::{leg}_lower_leg_link::{leg}_lower_leg_link_fixed_joint_lump__{leg}_foot_link_collision_1'
        for c in m.contacts:
            pair=(c.collision1.name,c.collision2.name)
            if foot not in pair or (pair[1] if pair[0]==foot else pair[0]).split('::')[0] not in allowed:
                self.fault('foot contact is not a scene support surface');return
        t=stamp(m)
        if not math.isfinite(t):self.fault('invalid foot contact timestamp');return
        history=self.contacts.setdefault(leg,collections.deque())
        if history and t<=history[-1]['stamp']:return
        history.append(dict(stamp=t,wall=time.monotonic(),nonempty=bool(m.contacts)))
        while history and t-history[0]['stamp']>.25:history.popleft()
    def tick(self):
        now=time.monotonic()
        if not self.f.c.enabled:
            self.state='failed' if self.f.failed else 'disabled';self.emit_pose(np.zeros(2))
            if self.f.failed or self.command_wall is None or now-self.command_wall>.25:self.cmd_pub.publish(Twist())
            self.report(now);return
        if self.imu is None or self.joints is None:
            self.zero_then_neutral()
            if now-self.started>30:self.fault('feedback startup timeout')
            self.report(now);return
        t=self.imu['stamp'];j,matched=newest_complete_joint(self.joint_history,self.imu_history)
        checks=dict(imu_wall_fresh=now-self.imu['wall']<=.10,
            complete_joint_imu_pair=j is not None,
            joint_wall_fresh=j is not None and now-j['wall']<=.10,
            joint_sim_fresh=j is not None and 0<=self.imu['stamp_ns']-j['stamp_ns']<=30_000_000,
            actuator_wall_fresh=self.actual_wall is not None and now-self.actual_wall<=.10,
            adapter_wall_fresh=self.adapter_wall is not None and now-self.adapter_wall<=.10,
            nominal_calibrated=bool(self.adapter.get('nominal_calibrated')),
            adapter_not_failed=not bool(self.adapter.get('failed')))
        fresh=all(checks.values())
        self.input_predicates=dict(checks=checks,latest_imu_stamp=t,latest_joint_stamp=self.joints['stamp'],
            selected_joint_stamp=None if j is None else j['stamp'],
            selected_imu_stamp=None if matched is None else matched['stamp'],
            imu_wall_age=now-self.imu['wall'],joint_wall_age=None if j is None else now-j['wall'],
            latest_imu_stamp_ns=self.imu['stamp_ns'],latest_joint_stamp_ns=self.joints['stamp_ns'],
            selected_joint_stamp_ns=None if j is None else j['stamp_ns'],
            selected_imu_stamp_ns=None if matched is None else matched['stamp_ns'],
            latest_joint_age_ns=self.imu['stamp_ns']-self.joints['stamp_ns'],
            selected_joint_age_ns=None if j is None else self.imu['stamp_ns']-j['stamp_ns'],
            matched_pair_gap_ns=None if j is None else abs(matched['stamp_ns']-j['stamp_ns']),
            latest_joint_age=(self.imu['stamp_ns']-self.joints['stamp_ns'])/1e9,
            selected_joint_age=None if j is None else (self.imu['stamp_ns']-j['stamp_ns'])/1e9,
            actuator_wall_age=None if self.actual_wall is None else now-self.actual_wall,
            adapter_wall_age=None if self.adapter_wall is None else now-self.adapter_wall)
        if fresh and self.last_support_joint_stamp!=j['stamp']:
            mask=measured_contact_mask(self.contacts,j['stamp'],now)
            self.f.observe_support(j['stamp'],matched['rotation'],j['feet'],j['velocities'],mask,matched['gyro'],stamp_ns=j['stamp_ns'])
            self.last_support_joint_stamp=j['stamp']
        requested_fresh=self.command_wall is not None and now-self.command_wall<=.25
        moving=(requested_fresh and np.any(abs(self.request)>1e-8)
            and self.neutral_wait_wall is None
            and self.actual_wall is not None and now-self.actual_wall<=.10
            and np.any(abs(self.actual)>1e-8) and self.adapter.get('state')=='walk')
        correction,self.state=self.f.step(t,self.imu['rotation'],self.imu['gyro'],actual_moving=bool(moving),inputs_fresh=fresh,stamp_ns=self.imu['stamp_ns'])
        self.gate_ready=self.state in ('active','neutral_for_stop')
        if self.f.failed or not self.gate_ready or not requested_fresh or not np.any(abs(self.request)>1e-8):self.zero_then_neutral()
        else:
            self.cmd_pub.publish(twist(self.request))
            if moving:self.emit_pose(correction)
            elif self.adapter.get('state') in ('wait_native_zero','returning','idle'):
                self.emit_pose(np.zeros(2));self.neutral_wait_wall=None
        self.report(now)
        self.log.write(json.dumps(dict(event='input_freshness_tick',wall_monotonic=now,
            predicates=self.input_predicates,state=self.state,failed=bool(self.f.failed),
            support_stamp=self.f.support_stamp,plane_stamp=self.f.plane_stamp,
            support_stamp_ns=self.f.support_stamp_ns,plane_stamp_ns=self.f.plane_stamp_ns),allow_nan=False)+'\n')
    def report(self,now):
        if now-self.last_status<.05:return
        self.last_status=now
        r=dict(state=self.state,enabled=self.f.c.enabled,failed=bool(self.f.failed),reason=self.f.failed,
            imu_stamp=None if self.imu is None else self.imu['stamp'],body_pose_roll_pitch=self.f.output.tolist(),
            actual_published_body_pose_roll_pitch=self.last_published_pose.tolist(),neutral_ack_pending=self.neutral_wait_wall is not None,
            diagnostics=self.f.diagnostic,input_predicates=self.input_predicates,
            test_only=True,reference='measured support terrain, never world robot pose')
        self.status_pub.publish(String(data=json.dumps(r)))
        try:self.log.write(json.dumps(r,allow_nan=False)+'\n')
        except Exception:self.fault('feedback diagnostic writer failed')

def shutdown_node(n):
    try:
        if rclpy.ok():
            n.request=np.zeros(6);n.zero_then_neutral();until=time.monotonic()+.15
            while rclpy.ok() and n.neutral_wait_wall is not None and time.monotonic()<until:
                rclpy.spin_once(n,timeout_sec=.005);n.zero_then_neutral()
    except (_ros.RCLError,rclpy.executors.ExternalShutdownException):
        # Context may close after ok() and before a final publish/spin call.
        # Live-context errors remain failures of this required node.
        if rclpy.ok():raise
    finally:
        n.log.close();n.destroy_node();rclpy.try_shutdown()

def main():
    p=argparse.ArgumentParser();p.add_argument('--enable',action='store_true');p.add_argument('--output',required=True)
    p.add_argument('--legacy-adapter-topics',action='store_true',help='Historical test adapter /demo/test topics; default is modern /demo/control')
    p.add_argument('--joint-topic',default='/joint_states',help='Actual joint sensor input; timestamps and freshness rules are unchanged')
    a=p.parse_args()
    rclpy.init();n=Stabilizer(a)
    try:rclpy.spin(n)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    except RuntimeError:
        if rclpy.ok():raise
    finally:shutdown_node(n)
if __name__=='__main__':main()
