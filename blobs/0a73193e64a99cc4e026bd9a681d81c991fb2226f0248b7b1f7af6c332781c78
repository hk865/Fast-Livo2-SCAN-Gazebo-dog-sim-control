#!/usr/bin/env python3
"""Own one frozen SCAN child; recreate its grid on navigation-frame transactions.

Only this supervisor's saved child PID is signalled. Physics, sensors, frontend
SLAM and Actor remain live. Fresh exact-stamp inputs and a new planner process
are required before an epoch-ready acknowledgment permits navigation.
"""
import argparse,hashlib,json,os,signal,subprocess,time
from collections import OrderedDict
from pathlib import Path

REMAPPINGS={'body_pose':'/demo/navigation/scan_body_odom','sensor_pose':'/demo/slam/lidar_odom',
    'cloud':'/demo/navigation/cloud','initial_path':'/demo/navigation/scan_reference',
    'planning/bspline':'/demo/navigation/bspline','planning/trajectory_metadata':'/demo/navigation/trajectory_metadata',
    'planning/go2_execution_frozen':'/demo/navigation/execution_frozen','grid_map/occupancy':'/demo/navigation/occupancy'}
def identity(pid):
    try:
        p=Path('/proc')/str(pid);raw=(p/'stat').read_text();f=raw[raw.rfind(')')+2:].split()
        return dict(pid=pid,ppid=int(f[1]),pgid=int(f[2]),start_ticks=int(f[19]),exe=os.readlink(p/'exe'))
    except (OSError,ValueError):return None

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--planner',type=Path,required=True);ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--params',type=Path,required=True);a,ros=ap.parse_known_args()
    run=a.run.resolve();binary=a.planner.resolve();digest=hashlib.sha256(binary.read_bytes()).hexdigest()
    rclpy.init(args=ros)
    class Supervisor(Node):
        def __init__(self):
            super().__init__('teacher_scan_epoch_supervisor')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.child=None;self.saved=None;self.epoch=0;self.ready=False;self.status=None
            self.slots=OrderedDict();self.confirmed=[];self.restart_clock=0;self.restarts=0
            self.log=(run/'scan_epoch_history.jsonl').open('x',buffering=1)
            self.pub=self.create_publisher(String,'/demo/navigation/scan_epoch_ready',10)
            self.create_subscription(String,'/demo/navigation/localization_status',self.on_status,10)
            self.create_subscription(String,'/demo/navigation/localization_bundle',lambda m:self.receive('metadata',m),10)
            self.create_subscription(Odometry,'/demo/navigation/scan_body_odom',lambda m:self.receive('odom',m),qos_profile_sensor_data)
            self.create_subscription(PointCloud2,'/demo/navigation/cloud',lambda m:self.receive('cloud',m),qos_profile_sensor_data)
            self.start_child();self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def record(self,kind,**data):
            self.log.write(json.dumps(dict(event=kind,monotonic_wall=time.monotonic(),
                ros_clock_ns=self.get_clock().now().nanoseconds,epoch=self.epoch,**data),allow_nan=False)+'\n')
        def start_child(self):
            if self.child is not None and self.child.poll() is None:raise RuntimeError('Second SCAN writer forbidden')
            command=[str(binary),'--ros-args','-r','__node:=scan_planner_node','--params-file',str(a.config),
                '--params-file',str(a.params)]
            for name,topic in REMAPPINGS.items():command+=['-r',name+':='+topic]
            self.child=subprocess.Popen(command)
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                saved=identity(self.child.pid)
                if saved and Path(saved['exe']).resolve()==binary:self.saved=saved;break
                if self.child.poll() is not None:raise RuntimeError('SCAN exited during owned bootstrap')
                time.sleep(.01)
            else:raise RuntimeError('Cannot bind owned SCAN executable identity')
            self.restart_clock=self.get_clock().now().nanoseconds;self.slots.clear();self.confirmed=[]
            self.ready=False;self.record('new_planner_with_empty_grid',identity=self.saved,binary_sha256=digest)
        def stop_child(self,reason):
            if self.child is None:return
            for sig,grace in ((signal.SIGINT,3),(signal.SIGTERM,2),(signal.SIGKILL,2)):
                if self.child.poll() is not None:break
                current=identity(self.child.pid)
                if (current is None or current['start_ticks']!=self.saved['start_ticks']
                        or current['ppid']!=os.getpid() or Path(current['exe']).resolve()!=binary):
                    raise RuntimeError('Refuse signalling an unowned planner PID')
                self.child.send_signal(sig)
                try:self.child.wait(timeout=grace)
                except subprocess.TimeoutExpired:continue
            if self.child.poll() is None:raise RuntimeError('Owned SCAN child failed to stop')
            self.record('owned_planner_stopped',reason=reason,exit_code=self.child.returncode,identity=self.saved)
        def on_status(self,msg):
            d=json.loads(msg.data)
            if d.get('run_id')==run.name:self.status={**d,'received_wall':time.monotonic()}
        def receive(self,role,msg):
            if role=='metadata':
                value=json.loads(msg.data)
                if value.get('run_id')!=run.name:return
                ns=value['source_ns']
                if value['generation']!=self.epoch or value['correction_transition']:self.ready=False
            else:ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec;value=None
            if ns<self.restart_clock:return
            slot=self.slots.setdefault(ns,{})
            slot[role]=(value,time.monotonic())
            while len(self.slots)>16:self.slots.popitem(last=False)
            if all(k in slot for k in ('metadata','odom','cloud')):
                d=slot['metadata'][0];clock=self.get_clock().now().nanoseconds
                if (d['generation']==self.epoch and not d['correction_transition'] and not d['correction_stale']
                        and -50_000_000<=clock-ns<300_000_000 and all(time.monotonic()-v[1]<.3 for v in slot.values())):
                    if not self.confirmed or ns>self.confirmed[-1]:self.confirmed=(self.confirmed+[ns])[-4:]
        def publish(self):
            d=dict(schema='known_scene_SCAN_epoch_ready/v1',run_id=run.name,generation=self.epoch,
                ready=self.ready,planner_identity=self.saved,planner_binary_sha256=digest,
                clean_empty_grid_at_generation_start=True,confirmed_source_stamps=self.confirmed,
                monotonic_wall=time.monotonic(),ros_clock_ns=self.get_clock().now().nanoseconds,
                restarts=self.restarts,robot_truth_pose_used=False)
            if rclpy.ok():self.pub.publish(String(data=json.dumps(d,allow_nan=False)))
            q=run/'scan_epoch_ready.tmp';q.write_text(json.dumps(d)+'\n');q.replace(run/'scan_epoch_ready.json')
        def tick(self):
            if self.child.poll() is not None:raise RuntimeError('Required SCAN child exited outside an owned epoch transaction')
            s=self.status
            if s is None or time.monotonic()-s['received_wall']>=.3 or s['correction_transition'] or not s['initialized']:
                self.ready=False;self.publish();return
            if s['generation']!=self.epoch:
                self.ready=False;self.publish();self.stop_child('completed_navigation_frame_transaction')
                self.epoch=s['generation'];self.restarts+=1;self.start_child()
            # Both inputs have reached the newly created planner subscriptions.
            graph=all(any(i.node_name=='scan_planner_node' for i in self.get_subscriptions_info_by_topic(topic))
                for topic in ('/demo/navigation/scan_body_odom','/demo/navigation/cloud'))
            clock=self.get_clock().now().nanoseconds
            self.ready=graph and len(self.confirmed)>=2 and 0<=clock-self.confirmed[-1]<300_000_000
            self.publish()
        def close(self):
            self.ready=False;self.publish();self.stop_child('owner_shutdown');self.log.close()
    node=Supervisor()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
