#!/usr/bin/env python3
"""Independent Teacher PID speed commands behind actual SCAN/sensor safeguards."""
import argparse,json,math,os,time
from pathlib import Path
import numpy as np
from teacher_wrapper import TeacherNavigation,base
from pid_core import HeaderPID,slew
from control_core import follow_trajectory


class PIDNavigation(TeacherNavigation):
    def __init__(self,run):
        super().__init__(run);self.pid=HeaderPID(self.profile);self.pid_velocity=None
        self.execution_health=None;self.execution_health_wall=None
        self.cloud_callback_records=0;self.pending_cloud_callback_row=None
        self.pid_pose_stamp=None;self.pid_row=None;self.pid_records=0;self.pid_guard_needs_evaluation=False;self.pid_prepared_output=None
        if self.profile['controller_kind']=='champ':
            from std_msgs.msg import String
            self.create_subscription(String,'/demo/champ/execution_health',self.on_execution_health,1)
    def on_execution_health(self,msg):
        try:
            d=json.loads(msg.data);ns=self.get_clock().now().nanoseconds;wall=time.monotonic()
            if(d.get('schema')!=1 or d.get('mode')!='champ'or d.get('source')!='scan_slam'
                or d.get('ground_truth_navigation_used')is not False or d.get('actor_started')is not False
                or d.get('state')not in ('hold','ready','failed')or not -.05<=wall-float(d['monotonic_wall'])<.3
                or not -50_000_000<=ns-int(d['ros_sim_time_ns'])<300_000_000):return
            self.execution_health=d;self.execution_health_wall=wall
            if self.state=='running'and self.apply_tilt_guard(wall,ns/1e9)and any(self.command):self.publish_command()
        except (ValueError,TypeError,KeyError):return
    def on_odom(self,msg):
        before=self.pose_stamp;super().on_odom(msg)
        if self.pose_stamp>before:
            v=msg.twist.twist.linear;self.pid_velocity=self.rotation@np.array([v.x,v.y,v.z])
            self.pid_pose_stamp=self.pose_stamp
    def on_cloud(self,msg):
        wall=time.monotonic();start_ns=self.get_clock().now().nanoseconds
        self.cloud_callback_records+=1
        self.pending_cloud_callback_row={'schema':'pid_actual_cloud_callback/v1','sequence':self.cloud_callback_records,
            'producer_stamp_ns':int(self.stamp(msg)),'frame_id':msg.header.frame_id,
            'received_monotonic_wall':wall,'received_ros_clock_ns':start_ns,
            'source_topic':'/cloud_registered_full','input_width':int(msg.width),'input_height':int(msg.height),
            'previous_accepted_stamp_ns':int(self.cloud_stamp),'reason':None,'navigation_ground_truth_used':False}
        try:super().on_cloud(msg)
        except Exception as error:
            self.note_cloud_decision('callback_exception',error=type(error).__name__+': '+str(error));raise
        finally:
            row=self.pending_cloud_callback_row;end=time.monotonic()
            row.update(finished_monotonic_wall=end,callback_duration_wall_s=end-wall,
                duration_scope='entry through original callback processing, before asynchronous receipt enqueue',
                finished_ros_clock_ns=self.get_clock().now().nanoseconds,
                accepted=row['reason']=='accepted',accepted_stamp_ns=int(self.cloud_stamp),
                accepted_cloud_received_monotonic_wall=self.cloud_updated if math.isfinite(self.cloud_updated)else None)
            if row['reason']is None:row['reason']='callback_returned_without_branch_receipt'
            self.evidence.append(self.run/'navigation_cloud_callbacks.jsonl',row);self.pending_cloud_callback_row=None
    def follow_checked_trajectory(self,*args,**kwargs):
        # The PID uses raw measured SLAM XY. Neither its arrival nor feedback
        # is the predictive tracking pose used by the historical CHAMP follower.
        kwargs['tracking_pose']=self.pose
        return follow_trajectory(*args,**kwargs)
    def select_pid_velocity(self,velocity,yaw_rate,target,allow_translation):
        ns=self.get_clock().now().nanoseconds;now=time.monotonic()
        mode=('hold'if self.obstacle_hold or self.heading_gate.phase in ('pre_turn','settle')else
              'drive'if allow_translation else'turn')
        fresh=(self.pid_velocity is not None and self.gyro_body is not None
            and self.gyro_wall is not None and now-self.gyro_wall<.3
            and -50_000_000<=ns-round(self.gyro_stamp*1e9)<300_000_000
            and self.pid_pose_stamp==self.pose_stamp)
        if not fresh:mode='hold'
        yaw=math.atan2(self.rotation[1,0],self.rotation[0,0]);heading=self.steering['heading']
        if mode=='turn'and self.heading_gate.heading is not None:heading=self.heading_gate.heading
        key=(self.request_id,self.waypoint_index,mode)
        if fresh:
            cmd,diagnostic=self.pid.update(int(self.pose_stamp),self.pose,yaw,self.pid_velocity,self.gyro_body[2],target,heading,mode,key,ns)
        else:
            self.pid.reset('IMU_or_SLAM_source_not_fresh');cmd=np.zeros(3);diagnostic={'updated':False,'reason':'source_not_fresh'}
        self.pid_guard_needs_evaluation=bool(diagnostic.get('updated'))
        if self.steering['exhausted']:self.pid.reset('checked_path_exhausted');cmd=np.zeros(3)
        desired=cmd.copy()
        cmd=(np.zeros(3)if mode=='hold'or self.steering['exhausted']else
             slew(self.command,desired,ns/1e9-self.last_command_time,self.profile['pid']['slew_acceleration']))
        self.pid_prepared_output=cmd.copy()
        # Guard the actual PID motion, including lateral motion, together with
        # the original collision-checked target corridor.
        c,s=math.cos(yaw),math.sin(yaw);r=np.array([[c,-s],[s,c]])
        world_direction=r@cmd[:2]
        if np.linalg.norm(world_direction)>1e-9:self.steering['direction']=world_direction.tolist()
        self.pid_records+=1;self.pid_row={'schema':'teacher_sensor_slam_pid_tick/v1','sequence':self.pid_records,
            'control_pose_stamp_ns':int(self.pose_stamp),'compute_ros_clock_ns':ns,'compute_monotonic_wall':now,
            'control_pose':self.pose.tolist(),'control_quaternion':self.pose_quaternion,'control_rotation':self.rotation.tolist(),
            'measured_world_velocity':None if self.pid_velocity is None else self.pid_velocity.tolist(),
            'imu_gyro_body':self.gyro_body,'imu_stamp_ns':None if self.gyro_stamp is None else round(self.gyro_stamp*1e9),
            'heading':heading,'yaw':yaw,'checked_target':np.asarray(target).tolist(),'goal':self.waypoints[self.waypoint_index].tolist(),
            'actual_pid_world_direction':world_direction.tolist(),'steering_direction':self.steering['direction'],
            'request_id':self.request_id,'waypoint_index':self.waypoint_index,'trajectory_id':self.active_trajectory_id,
            'trajectory_archive_file':self.trajectory_archive_reference,'mode':mode,'pid':diagnostic,
            'desired_body_command':desired.tolist(),'prepared_after_slew_command':cmd.tolist(),'navigation_ground_truth_used':False}
        return cmd[:2],float(cmd[2])
    def evaluate_motion_guard(self,*args):
        result=self.record_native_guard(*args)
        if self.pending_guard_row is not None and self.pid_row is not None:
            self.pending_guard_row.update(pid_sequence=self.pid_row['sequence'],
                pid_compute_ros_clock_ns=self.pid_row['compute_ros_clock_ns'],
                pid_control_pose_stamp_ns=self.pid_row['control_pose_stamp_ns'])
        self.pid_guard_needs_evaluation=False;return result
    def apply_tilt_guard(self,now,ros_now):
        protected=super().apply_tilt_guard(now,ros_now)
        if self.profile['controller_kind']=='champ':
            d=getattr(self,'execution_health',None)
            ready=(d is not None and self.execution_health_wall is not None and now-self.execution_health_wall<.3
                and -.05<=now-float(d['monotonic_wall'])<.3 and -.05<=ros_now-int(d['ros_sim_time_ns'])/1e9<.3
                and d.get('state')=='ready'and d.get('ready')is True)
            if not ready:protected=True;self.message='CHAMP 实际执行健康未就绪，保持零速度'
            if d is not None and d.get('state')=='failed':self.state='failed'
        if protected and hasattr(self,'pid'):self.pid.reset('existing_tilt_or_execution_hold')
        return protected
    def publish_command(self,velocity=None,yaw_rate=0.):
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool
        msg=Twist();ns=self.get_clock().now().nanoseconds;sim=ns/1e9
        if velocity is not None:
            output=np.r_[velocity,yaw_rate]
            if self.pid_prepared_output is None or not np.allclose(output,self.pid_prepared_output,rtol=0,atol=1e-12):raise RuntimeError('PID output changed after motion guard')
            msg.linear.x,msg.linear.y,msg.angular.z=map(float,output)
        elif hasattr(self,'pid'):self.pid.reset('exact_zero_stop_or_protection')
        self.command=[msg.linear.x,msg.linear.y,msg.angular.z];self.last_command_time=sim
        self.cmd_pub.publish(msg);self.counts['commands']+=1
        self.freeze_pub.publish(Bool(data=bool(self.state!='running'or self.obstacle_hold or np.linalg.norm(self.command[:2])<.015)))
        if getattr(self,'pid_row',None)is not None:
            row=self.pid_row;row.update(publish_ros_clock_ns=ns,command_after_slew=list(self.command),
                state=self.state,obstacle_hold=self.obstacle_hold,alignment_hold=self.alignment_hold,
                stopped=velocity is None,reset_count_after_publish=self.pid.resets,queue_error=self.evidence.error)
            self.evidence.append(self.run/'navigation_pid_history.jsonl',row);self.pid_row=None


def main():
    import rclpy
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args,ros=parser.parse_known_args();run=args.run.resolve();os.environ['DEMO_RUN_DIR']=str(run)
    rclpy.init(args=ros);node=PIDNavigation(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publish_command()
        try:node.event_archive.close();node.evidence.close()
        finally:
            (run/'navigation_pid_writer_receipt.json').write_text(json.dumps({'schema':1,'expected_pid_records':node.pid_records,
                'expected_guard_records':node.guard_sequence,'queue_error':node.evidence.error,
                'expected_cloud_callback_records':node.cloud_callback_records,
                'status':'failed'if node.evidence.error else'drained','navigation_verified':False})+'\n')
            node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
