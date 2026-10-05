#!/usr/bin/env python3
"""Actual reviewed SLAM command envelope -> unique CHAMP Twist publisher.

No actor, ground-truth navigation, joint force publisher or base servo exists.
Native state is used only for independent safety/measurement, as in Teacher.
"""
import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent
TEACHER=HERE.parents[1]
JOINT_NAMES=[f'{leg}_{part}_joint'for leg in ('lf','rf','lh','rh')for part in ('hip','upper_leg','lower_leg')]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def atomic(path,data):
    temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(json.dumps(data,allow_nan=False)+'\n');temporary.replace(path)
def consumer():
    """Load only the exact frozen reader function; never import or load an actor."""
    path=TEACHER/'policy/worker.py';tree=ast.parse(path.read_text())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='navigation_command')
    module=ast.Module(body=[function],type_ignores=[])
    context={'json':json,'hashlib':hashlib,'time':time,'np':np}
    exec(compile(module,str(path),'exec'),context)
    return context['navigation_command'],hashlib.sha256(ast.dump(function,include_attributes=False).encode()).hexdigest()

def rpy(q):
    w,x,y,z=q
    return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(float(np.clip(2*(w*y-z*x),-1,1))),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--command-file',type=Path,required=True)
    p.add_argument('--acceptance',type=Path,required=True);p.add_argument('--duration',type=float,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();stage=run/'champ';contract=json.loads((run/'champ_contract.json').read_text())
    if contract.get('schema')!='champ_execution_contract/v1'or contract.get('run')!=str(run):raise RuntimeError('Incorrect CHAMP executor contract')
    if float(contract['duration_s'])!=args.duration:raise RuntimeError('CHAMP duration differs from prepared native watchdog')
    if sha(run/'world.sdf')!=contract['world_sha256']:raise RuntimeError('CHAMP physical world changed')
    for path,digest in contract['artifacts'].items():
        if sha(path)!=digest:raise RuntimeError('CHAMP prepared artifact changed: '+path)
    for path,digest in contract['executor_source_sha256'].items():
        if sha(path)!=digest:raise RuntimeError('CHAMP executor source changed: '+path)
    sys.path.insert(0,str(TEACHER/'navigation'));from bridge import check_acceptance
    acceptance=check_acceptance(args.acceptance);read_command,function_sha=consumer()
    sys.path.insert(0,str(TEACHER/'policy'));from observation import TerrainHeightMap
    safety=TerrainHeightMap.from_sdf(run/'world.sdf')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Twist
    from trajectory_msgs.msg import JointTrajectory
    from control_msgs.msg import JointTrajectoryControllerState
    from std_msgs.msg import String
    rclpy.init(args=ros)
    class Reader(Node):
        def __init__(self):
            super().__init__('champ_reviewed_command_reader')
            self.clock_ns=None;self.clock_wall=None;self.last_tick_ns=None;self.last_wall_tick=0.;self.last_dt=None
            self.cmd=np.zeros(3);self.sequence={};self.failure=None;self.done=False;self.finished=False;self.last_health=None
            self.adapter=None;self.adapter_wall=None;self.backend=None;self.backend_wall=None;self.backend_stamp=None
            self.started_wall=time.monotonic();self.graph_wall=None;self.graph=None;self.latest_native=None;self.native_wall=None
            self.target_log=(stage/'joint_target_events.jsonl').open('w',buffering=1024*1024)
            self.telemetry=(run/'telemetry.jsonl').open('w',buffering=1024*1024)
            self.healthlog=(stage/'execution_health_history.jsonl').open('w',buffering=1024*1024)
            self.pub=self.create_publisher(Twist,'/demo/control/safe_cmd_vel',10)
            self.healthpub=self.create_publisher(String,'/demo/champ/execution_health',10)
            self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
            self.create_subscription(String,'/demo/control/joint_stop_safety',self.adapter_status,10)
            self.create_subscription(JointTrajectoryControllerState,'/joint_group_effort_controller/controller_state',self.controller,qos_profile_sensor_data)
            self.create_subscription(JointTrajectory,'/joint_group_effort_controller/joint_trajectory',self.target,100)
            self.create_timer(.005,self.tick)
            (run/'worker_ready').write_text('CHAMP command reader ready; backend still requires actual graph/calibration/feedback\n')
        def clock(self,msg):
            stamp=msg.clock.sec*10**9+msg.clock.nanosec
            if self.clock_ns is not None and stamp<self.clock_ns:self.failure=self.failure or 'clock moved backward'
            if self.clock_ns is None or stamp>self.clock_ns:self.clock_ns=stamp;self.clock_wall=time.monotonic()
        def adapter_status(self,msg):
            try:
                data=json.loads(msg.data)
                if data.get('state')not in ('calibrating','idle','walk','wait_native_zero','returning','failed'):raise ValueError()
                if data.get('failed')is True:self.failure=self.failure or 'original joint adapter failed: '+str(data.get('reason'))
                self.adapter=data;self.adapter_wall=time.monotonic()
            except (ValueError,TypeError):self.failure=self.failure or 'invalid adapter health'
        def controller(self,msg):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            positions=list(msg.feedback.positions);velocities=list(msg.feedback.velocities)
            if list(msg.joint_names)!=JOINT_NAMES or len(positions)!=12 or len(velocities)!=12 or not np.isfinite(positions+velocities).all():
                self.failure=self.failure or 'invalid actual controller feedback';return
            if self.backend_stamp is not None and stamp<self.backend_stamp:self.failure=self.failure or 'backend stamp moved backward'
            self.backend={'stamp_ns':stamp,'q':positions,'qd':velocities,'source':'actual JointTrajectoryControllerState.feedback'}
            self.backend_stamp=stamp;self.backend_wall=time.monotonic()
        def target(self,msg):
            self.target_log.write(json.dumps({'received_clock_ns':self.clock_ns,'received_monotonic_wall':time.monotonic(),'source_topic':'/joint_group_effort_controller/joint_trajectory',
                'original_header_stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,'original_frame':msg.header.frame_id,'joint_names':list(msg.joint_names),
                'points':[{'positions':list(v.positions),'velocities':list(v.velocities),'effort':list(v.effort),'time_from_start_ns':v.time_from_start.sec*10**9+v.time_from_start.nanosec}for v in msg.points]},allow_nan=False)+'\n')
        def publish(self,value):
            msg=Twist();msg.linear.x=float(value[0]);msg.linear.y=float(value[1]);msg.angular.z=float(value[2]);self.pub.publish(msg)
        def health(self,now):
            reasons=[];t=0. if self.clock_ns is None else self.clock_ns*1e-9
            if self.clock_wall is None or now-self.clock_wall>.3:reasons.append('Actual ROS clock absent/stale')
            if self.adapter_wall is None or now-self.adapter_wall>.3 or not self.adapter.get('nominal_calibrated'):reasons.append('Original adapter not calibrated/fresh')
            if self.backend_wall is None or now-self.backend_wall>.3 or not -.05<=t-self.backend_stamp*1e-9<=.3:reasons.append('Actual controller feedback absent/stale')
            if self.graph_wall is None or now-self.graph_wall>=.1:
                expected={'/demo/control/safe_cmd_vel':'champ_reviewed_command_reader','/demo/control/joint_reference/raw':'quadruped_controller_node',
                    '/joint_group_effort_controller/joint_trajectory':'demo_joint_reference_adapter','/demo/control/joint_stop_safety':'demo_joint_reference_adapter',
                    '/joint_group_effort_controller/controller_state':'joint_group_effort_controller'}
                self.graph={}
                for topic,node in expected.items():
                    infos=self.get_publishers_info_by_topic(topic);names=[v.node_name for v in infos]
                    self.graph[topic]={'names':names,'passed':names==[node]}
                self.graph_wall=now
            if not self.graph or not all(v['passed']for v in self.graph.values()):reasons.append('CHAMP publisher graph is not unique expected chain')
            if self.failure:reasons.append(self.failure)
            return {'schema':1,'mode':'champ','source':'scan_slam','state':'failed'if self.failure else'hold'if reasons else'ready','ready':not reasons,
                'reason':'; '.join(reasons)if reasons else'Actual CHAMP trajectory controller, adapter nominal and publisher graph ready',
                'ros_sim_time_ns':self.clock_ns,'monotonic_wall':now,'publisher_graph':self.graph,'adapter':self.adapter,'controller_feedback':self.backend,
                'ground_truth_navigation_used':False,'body_servo':False,'actor_started':False}
        def tick(self):
            if self.done:return
            now=time.monotonic();ns=self.clock_ns
            advanced=ns is not None and (self.last_tick_ns is None or ns-self.last_tick_ns>=20_000_000)
            if not advanced and now-self.last_wall_tick<.02:return
            self.last_wall_tick=now
            t=0. if ns is None else ns*1e-9
            health=self.health(now);self.last_health=health;self.healthpub.publish(String(data=json.dumps(health,allow_nan=False)))
            self.healthlog.write(json.dumps(health,allow_nan=False)+'\n');atomic(stage/'execution_health.json',health)
            request,expired,reason=read_command(args.command_file,t,acceptance['sha256'],self.sequence)
            if t<3:request=np.zeros(3);reason='CHAMP zero-command initialization'
            if not health['ready']:request=np.zeros(3);reason=health['reason']
            # Consume exactly the same outer route until its duration, then explicitly record extra parking.
            if t>=args.duration:request=np.zeros(3);reason='Reviewed eight-second parking after navigation duration'
            try:
                state=json.loads((stage/'native_state.json').read_text())
                if state.get('state_phase')!='PostUpdate'or not -.05<=t-float(state['world_sim_time'])<=.3:raise ValueError('native state stale/phase')
                self.latest_native=state;self.native_wall=now
                position=state['position'];angles=rpy(state['quaternion_wxyz']);ground=float(safety.height(np.array([position[:2]]),ray_start_z=position[2])[0]);clearance=position[2]-ground
                if t>1.5 and (max(abs(angles[0]),abs(angles[1]))>.8 or clearance<.15):self.failure=self.failure or 'fallen_or_low_clearance'
                if t>1.5 and state['contacts'][0]>0:self.failure=self.failure or 'body_contact'
            except (OSError,ValueError,KeyError,TypeError):
                state=None;angles=None;clearance=None
                if t>3:self.failure=self.failure or 'native safety state absent/stale'
            dt=.02 if self.last_tick_ns is None or ns is None else min(.02,max(0.,(ns-self.last_tick_ns)*1e-9))
            if advanced:self.cmd+=np.clip(request-self.cmd,-np.array([.6,.6,.8])*dt,np.array([.6,.6,.8])*dt);self.last_tick_ns=ns
            if self.failure or self.clock_wall is None or now-self.clock_wall>.3:self.cmd=np.zeros(3)
            self.publish(self.cmd)
            if advanced:
                measured=None if state is None else[state['body_lin_vel_com'][0],state['body_lin_vel_com'][1],state['body_ang_vel'][2]]
                row={'sim_time':t,'world_sim_time':t,'state_physics_world_time':None if state is None else state['world_sim_time'],'state_time_offset_s':0,'state_phase':'PostUpdate',
                    'controller_kind':'champ','command':self.cmd.tolist(),'requested':request.tolist(),'command_expired':bool(expired),'command_reason':reason,'navigation_envelope':dict(self.sequence),
                    'command_age_sim_s':self.sequence.get('sim_age_s'),'command_age_wall_s':self.sequence.get('wall_age_s'),'state':'champ_tracking'if any(self.cmd)else'champ_zero_velocity_native_nominal',
                    'fault':self.failure,'measured':measured,'rpy':angles,'body_clearance':clearance,'actual_execution_health':health,'actor_started':False,
                    'action':None,'q_target':None,'joint_target_evidence':str(stage/'joint_target_events.jsonl'),'native_state':state}
                if state:row.update({k:state[k]for k in ('position','quaternion_wxyz','q','qd')},body_lin_vel=state['body_lin_vel_com'],body_ang_vel=state['body_ang_vel'],applied_torque=state['tau'],contacts=dict(zip(['body','FR','FL','RR','RL'],state['contacts'])))
                self.telemetry.write(json.dumps(row,allow_nan=False)+'\n')
            if t>=args.duration+8 or (self.failure and t>=2):self.finished=True;self.done=True
        def finish(self):
            self.publish(np.zeros(3))
            for stream in (self.target_log,self.telemetry,self.healthlog):stream.flush();stream.close()
            metadata={'schema':1,'controller_kind':'champ','actor_device':None,'actor_started':False,'inference_threads':0,'cpu_only':True,'test_duration_s':args.duration+8,'navigation_duration_s':args.duration,
                'completed':self.finished and self.failure is None,'fault':self.failure,'last_ros_sim_ns':self.clock_ns,'consumer_function_ast_sha256':function_sha,
                'command_reader_sha256':sha(__file__),'accepted_scope':acceptance,'last_execution_health':self.last_health,'physical_state_phase':'PostUpdate','state_time_offset_s':0,
                'torque_source':'native JointForceCmd component; not measured motor torque','startup_semantics':contract['initialization'],'final_parking_s':8,'truth_navigation_used':False,
                'no_second_force_writer':True,'body_stabilizer':False,'body_servo':False,'raw_recorders_closed':True}
            atomic(run/'policy_metadata.json',metadata);atomic(stage/'monitor_done.json',metadata)
    node=Reader();exit_code=0
    try:
        while rclpy.ok()and not node.done:rclpy.spin_once(node,timeout_sec=.01)
        if node.failure:exit_code=2
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):node.failure=node.failure or 'CHAMP monitor interrupted';exit_code=130
    finally:
        try:node.finish()
        finally:node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(exit_code)
if __name__=='__main__':main()
