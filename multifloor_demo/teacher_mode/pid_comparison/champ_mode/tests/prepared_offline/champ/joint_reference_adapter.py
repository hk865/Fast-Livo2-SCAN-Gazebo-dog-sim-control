#!/usr/bin/env python3
"""Immediate body zero plus continuous return of joint references.

Enabled by default in the production simulation launch.
No ground truth or pose input; no old nonzero velocity is retained at zero.
"""
import copy,json,os,pathlib,time
import numpy as np
import yaml
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from joint_stop_core import JointGeometry,StopReturn
from execution_safety import adapter_settings

ROOT=pathlib.Path(__file__).resolve().parent

class Adapter(Node):
    def __init__(self):
        super().__init__('demo_joint_reference_adapter')
        self.settings=adapter_settings()
        gait=yaml.safe_load(pathlib.Path(os.environ.get('DEMO_TEST_GAIT_CONFIG',ROOT/'config/gait.yaml')).read_text())['/**']['ros__parameters']['gait']
        urdf=pathlib.Path(os.environ.get('DEMO_TEST_ROBOT_URDF',ROOT/'generated/go2.urdf')).read_text()
        self.geometry=JointGeometry(urdf,gait['nominal_height'],gait['com_x_translation'])
        self.sim=0.;self.started=time.monotonic();self.state='calibrating';self.reason='boot';self.failure_latched=False
        self.clock_seen=False;self.last_clock_wall=None
        self.nominal_sample_sim=None;self.nominal_sample_wall=None;self.nominal_samples=0
        self.request=Twist();self.last_cmd=None;self.last_raw=None;self.nominal=None;self.candidate=None;self.candidate_since=None
        self.last_output=None;self.last_template=None;self.transition=None;self.wait_started=None;self.zero_start=None
        self.counters=dict(raw=0,filtered=0,stops=0,queued_nonzero=0,ignored_queued_gait=0)
        self.log=pathlib.Path(os.environ['DEMO_RUN_DIR'],'joint_stop_adapter.jsonl').open('w',buffering=1024*1024)
        self.cmd_pub=self.create_publisher(Twist,self.settings['actuator_topic'],10)
        self.joint_pub=self.create_publisher(JointTrajectory,'/joint_group_effort_controller/joint_trajectory',10)
        self.status_pub=self.create_publisher(String,self.settings['status_topic'],10)
        self.create_subscription(Twist,'/demo/control/safe_cmd_vel',self.command,10)
        self.create_subscription(JointTrajectory,self.settings['raw_topic'],self.raw,100)
        self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
        self.create_timer(.02,self.watchdog)

    def log_row(self,kind,**fields):
        try:
            self.log.write(json.dumps(dict(kind=kind,sim=self.sim,wall=time.monotonic()-self.started,
                state=self.state,**fields),separators=(',',':'),allow_nan=False)+'\n')
        except Exception:
            # Diagnostic provenance failure must not leave an old walking command active.
            self.failure_latched=True;self.state='failed';self.reason='adapter evidence writer failed';self.request=Twist()
            self.cmd_pub.publish(Twist())

    @staticmethod
    def nonzero(msg):return max(abs(msg.linear.x),abs(msg.linear.y),abs(msg.linear.z),abs(msg.angular.x),abs(msg.angular.y),abs(msg.angular.z))>1e-8

    def emit_command(self,msg):
        if self.failure_latched:msg=Twist()
        self.cmd_pub.publish(msg)
        self.log_row('actual_champ_command',value=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z])

    def clock(self,msg):
        stamp=msg.clock.sec+msg.clock.nanosec*1e-9
        if stamp<self.sim-1e-9:self.fail('clock moved backwards')
        self.sim=stamp;self.clock_seen=True;self.last_clock_wall=time.monotonic()

    def fail(self,reason):
        first=not self.failure_latched
        self.failure_latched=True;self.state='failed';self.reason=reason;self.request=Twist();self.emit_command(Twist())
        if first:self.log_row('failure',reason=reason)

    def command(self,msg):
        values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        if not np.all(np.isfinite(values)):self.fail('nonfinite body command');return
        self.last_cmd=time.monotonic();self.request=copy.deepcopy(msg)
        self.log_row('requested_safe',value=values)
        if self.state=='failed':self.emit_command(Twist());return
        if not self.nonzero(msg):
            self.emit_command(Twist()) # Immediate, regardless of reference-return progress.
            if self.state=='walk':self.begin_stop('safe zero edge')
        elif self.state in ('walk','idle'):
            self.state='walk';self.emit_command(msg)
        else:
            self.counters['queued_nonzero']+=1;self.emit_command(Twist())

    def begin_stop(self,reason):
        if self.last_output is None or self.nominal is None:self.fail('stop without calibrated reference');return
        self.state='wait_native_zero';self.reason=reason;self.wait_started=time.monotonic();self.zero_start=self.sim
        self.transition=None;self.stop_origin=self.last_output.copy();self.counters['stops']+=1
        self.log_row('zero_edge',q0=self.stop_origin.tolist(),nominal=self.nominal.tolist())

    def emit(self,msg,q,mode,**evidence):
        if not self.geometry.valid(q):self.fail('output position violates URDF');return
        out=msg if mode=='passthrough' else copy.deepcopy(msg)
        if mode!='passthrough':
            out.points[0].positions=q.tolist();out.points[0].velocities=[];out.points[0].accelerations=[];out.points[0].effort=[]
        self.joint_pub.publish(out);self.last_output=q.copy();self.last_template=copy.deepcopy(out)
        self.counters['filtered']+=1;self.log_row('filtered_target',positions=q.tolist(),mode=mode,**evidence)

    def raw(self,msg):
        self.last_raw=time.monotonic();self.counters['raw']+=1
        if list(msg.joint_names)!=self.geometry.names or len(msg.points)!=1:
            self.fail('unexpected joint trajectory schema');return
        point=msg.points[0]
        if (msg.header.stamp.sec!=0 or msg.header.stamp.nanosec!=0 or msg.header.frame_id
                or point.time_from_start.sec!=0 or point.time_from_start.nanosec!=16666666
                or point.velocities or point.accelerations or point.effort):
            self.fail('native trajectory violates header/position-only/horizon contract');return
        q=np.array(msg.points[0].positions,dtype=float)
        if not self.geometry.valid(q):self.fail('raw target outside URDF or nonfinite');return
        self.log_row('raw_target',positions=q.tolist(),header_stamp=[msg.header.stamp.sec,msg.header.stamp.nanosec],
            horizon=[msg.points[0].time_from_start.sec,msg.points[0].time_from_start.nanosec])
        if self.failure_latched:
            if self.last_output is not None:self.emit(msg,self.last_output,'failed_hold')
            return
        if self.state=='failed':
            if self.last_output is not None:self.emit(msg,self.last_output,'failed_hold')
            return
        if self.state=='calibrating':
            if (not self.clock_seen or self.last_clock_wall is None
                    or self.last_raw-self.last_clock_wall>.10 or not self.geometry.is_nominal(q)):
                self.candidate_since=None;self.nominal_sample_sim=None;self.nominal_sample_wall=None;self.nominal_samples=0
                return
            contiguous=(self.candidate_since is not None and self.candidate is not None
                and np.max(np.abs(q-self.candidate))<=1e-8
                and self.nominal_sample_sim is not None and 0<=self.sim-self.nominal_sample_sim<=.030001
                and self.nominal_sample_wall is not None and self.last_raw-self.nominal_sample_wall<=.10)
            if not contiguous:
                self.candidate=q.copy();self.candidate_since=self.sim;self.nominal_samples=0
            self.nominal_samples+=1;self.nominal_sample_sim=self.sim;self.nominal_sample_wall=self.last_raw
            self.emit(msg,q,'native_nominal_boot')
            if self.failure_latched:return
            if self.sim-self.candidate_since>=.5 and self.nominal_samples>=50:
                self.nominal=q.copy();self.state='idle';self.log_row('nominal_frozen',positions=q.tolist(),feet=self.geometry.feet(q).tolist(),
                    first_sample_sim=self.candidate_since,last_sample_sim=self.sim,samples=self.nominal_samples)
            return
        if self.state=='walk':self.emit(msg,q,'passthrough');return
        if self.state=='wait_native_zero':
            if np.max(np.abs(q-self.nominal))<1e-8:
                self.transition=StopReturn(self.stop_origin,self.nominal,self.geometry,self.sim)
                self.state='returning';self.log_row('native_zero_ack',duration=self.transition.duration)
            else:
                self.counters['ignored_queued_gait']+=1;self.emit(msg,self.stop_origin,'wait_native_zero');return
        if self.state=='returning':
            if np.max(np.abs(q-self.nominal))>1e-8:self.fail('native gait moved during body-zero return');return
            try:out,velocity,acceleration,done=self.transition.evaluate(self.sim)
            except ValueError as exc:self.fail(str(exc));return
            self.emit(msg,out,'stop_return',planned_velocity=velocity.tolist(),planned_acceleration=acceleration.tolist(),
                elapsed=self.sim-self.transition.start,duration=self.transition.duration)
            if self.failure_latched:return
            if done:
                self.state='idle';self.log_row('return_complete',duration=self.sim-self.zero_start)
                if self.nonzero(self.request) and self.last_cmd and time.monotonic()-self.last_cmd<=.25:
                    self.state='walk';self.emit_command(self.request);self.log_row('queued_motion_released')
            return
        # IDLE is an actual native nominal hold, never a retained moving-foot target.
        self.emit(msg,self.nominal,'native_nominal_idle')

    def watchdog(self):
        now=time.monotonic()
        if self.state!='failed':
            if self.last_cmd and now-self.last_cmd>.25:
                self.request=Twist();self.emit_command(Twist())
                if self.state=='walk':self.begin_stop('safe command watchdog')
            if self.state=='wait_native_zero' and now-self.wait_started>.25:self.fail('native zero ack timeout')
            if self.state in ('walk','returning') and self.last_raw and now-self.last_raw>.25:self.fail('raw joint target timeout')
        if self.state not in ('walk',):self.emit_command(Twist())
        state=dict(state=self.state,failed=self.state=='failed',reason=self.reason,sim=self.sim,counters=self.counters,
            nominal_calibrated=self.nominal is not None,test_only=self.settings['legacy_test_topics'])
        self.status_pub.publish(String(data=json.dumps(state)))

def main():
    rclpy.init();node=Adapter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    except _ros.RCLError:
        # SIGINT can invalidate the context between executor wait-set calls.
        # Errors while the context is live must still fail this required node.
        if rclpy.ok():raise
    finally:
        try:
            if rclpy.ok():node.emit_command(Twist())
        except _ros.RCLError:
            # The context can close after the above check but before publish.
            if rclpy.ok():raise
        finally:
            node.log_row('shutdown');node.log.close();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
