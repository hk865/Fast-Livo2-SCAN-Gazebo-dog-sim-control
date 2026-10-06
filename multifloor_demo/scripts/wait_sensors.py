#!/usr/bin/env python3
"""Start SLAM only after physical sensors and a continuous static IMU window."""
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import String
from sensor_startup_gate import StaticImuWindow

def main():
    root=Path(__file__).resolve().parents[1]
    scenario=json.loads((root/'simulation/scenario.json').read_text())
    reference=scenario['sensors']['imu']['orientation_reference']['world_quaternion']
    gate=StaticImuWindow(reference)
    rclpy.init();node=Node('demo_initial_sensor_gate')
    seen={'sim':0.,'imu':0,'camera':0,'imu_at':0.,'camera_at':0.}
    def clock(msg):seen['sim']=msg.clock.sec+msg.clock.nanosec*1e-9
    def imu(msg):
        wall=time.monotonic();seen['imu']+=1;seen['imu_at']=wall
        q,w,a=msg.orientation,msg.angular_velocity,msg.linear_acceleration
        gate.update(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,wall,
                    (q.x,q.y,q.z,q.w),(w.x,w.y,w.z),(a.x,a.y,a.z),msg.orientation_covariance[0]!=-1)
    def camera(msg):seen['camera']+=1;seen['camera_at']=time.monotonic()
    def safety(msg):
        try:gate.update_bridge(json.loads(msg.data),time.monotonic())
        except (ValueError,TypeError):gate.reset('invalid_bridge_status')
    subs=[node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data),
          node.create_subscription(Imu,'/livox/imu',imu,qos_profile_sensor_data),
          node.create_subscription(Image,'/camera/image_color',camera,qos_profile_sensor_data),
          node.create_subscription(String,'/demo/control/safety',safety,10)]
    start=time.monotonic();passed=False
    try:
        while rclpy.ok() and time.monotonic()-start<150:
            rclpy.spin_once(node,timeout_sec=.05)
            now=time.monotonic()
            if seen['sim']>=10 and seen['imu']>=300 and seen['camera']>=10 and now-seen['camera_at']<3:
                if gate.ready(seen['sim'],now):passed=True;break
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):gate.reason='interrupted'
    finally:
        if not passed and time.monotonic()-start>=150:
            gate.reason='timeout_before_static_initialization: '+gate.reason
        report=dict(passed=passed,observed=seen,wall_elapsed=time.monotonic()-start,**gate.report())
        directory=os.environ.get('DEMO_RUN_DIR')
        if directory:
            Path(directory).mkdir(parents=True,exist_ok=True)
            (Path(directory)/'startup_gate.json').write_text(json.dumps(report,indent=2)+'\n')
        print('Sensor startup gate: '+json.dumps(report),flush=True)
        node.destroy_node();rclpy.try_shutdown()
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
