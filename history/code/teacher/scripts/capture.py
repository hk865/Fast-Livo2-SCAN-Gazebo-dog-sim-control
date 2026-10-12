#!/usr/bin/env python3
"""Archive actual Gazebo RGB, with sensor timestamp. No rendered substitute."""
import argparse, io, json
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PIL
def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    rclpy.init();node=rclpy.create_node('teacher_real_rgb_archive');count=0
    def callback(msg):
        nonlocal count
        if msg.encoding not in ('rgb8','bgr8','rgba8'):return
        fmt='RGBA'if msg.encoding=='rgba8'else 'RGB';raw='BGR'if msg.encoding=='bgr8'else fmt
        im=PIL.frombytes(fmt,(msg.width,msg.height),bytes(msg.data),'raw',raw,msg.step).convert('RGB')
        tmp=a.run/'frame.tmp';im.save(tmp,format='JPEG',quality=86);tmp.replace(a.run/'frame.jpg')
        if count%5==0:
            (a.run/'frames').mkdir(exist_ok=True);im.save(a.run/'frames'/f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}.jpg',quality=86)
        (a.run/'frame_source.json').write_text(json.dumps({'source':'Actual Gazebo /demo/camera via ROS bridge','stamp_sec':msg.header.stamp.sec,'stamp_nsec':msg.header.stamp.nanosec,'count':count,'width':msg.width,'height':msg.height}))
        count+=1
    node.create_subscription(Image,'/demo/camera',callback,qos_profile_sensor_data)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
