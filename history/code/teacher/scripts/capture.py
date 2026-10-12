#!/usr/bin/env python3
"""Archive actual Gazebo RGB, with sensor timestamp. No rendered substitute."""
import argparse, io, json
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from PIL import Image as PIL
def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--topic',default='/demo/camera');p.add_argument('--archive-stride',type=int,default=5);a=p.parse_args()
    if a.archive_stride<1:p.error('--archive-stride must be positive')
    rclpy.init();node=rclpy.create_node('teacher_real_rgb_archive');count=0
    info_counts={}
    def camera_info(msg,kind,topic):
        directory=a.run/'camera_info';directory.mkdir(exist_ok=True)
        info_counts[kind]=info_counts.get(kind,0)+1
        record={'source':'actual Gazebo CameraInfo via ros_gz_bridge','topic':topic,'camera':kind,
                'stamp_sec':msg.header.stamp.sec,'stamp_nsec':msg.header.stamp.nanosec,'frame':msg.header.frame_id,
                'width':msg.width,'height':msg.height,'distortion_model':msg.distortion_model,
                'd':list(msg.d),'k':list(msg.k),'r':list(msg.r),'p':list(msg.p),
                'binning_x':msg.binning_x,'binning_y':msg.binning_y,'count':info_counts[kind]}
        with (directory/(kind+'.jsonl')).open('a')as stream:stream.write(json.dumps(record)+'\n')
        temp=directory/(kind+'.tmp');temp.write_text(json.dumps(record,indent=2)+'\n');temp.replace(directory/(kind+'.json'))
    for kind,topic in [('vehicle','/demo/camera_info'),('overview','/demo/teacher/overview_info')]:
        node.create_subscription(CameraInfo,topic,lambda msg,kind=kind,topic=topic:camera_info(msg,kind,topic),qos_profile_sensor_data)
    def callback(msg):
        nonlocal count
        if msg.encoding not in ('rgb8','bgr8','rgba8'):return
        fmt='RGBA'if msg.encoding=='rgba8'else 'RGB';raw='BGR'if msg.encoding=='bgr8'else fmt
        im=PIL.frombytes(fmt,(msg.width,msg.height),bytes(msg.data),'raw',raw,msg.step).convert('RGB')
        tmp=a.run/'frame.tmp';im.save(tmp,format='JPEG',quality=86);tmp.replace(a.run/'frame.jpg')
        if count%a.archive_stride==0:
            (a.run/'frames').mkdir(exist_ok=True);im.save(a.run/'frames'/f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}.jpg',quality=86)
        (a.run/'frame_source.json').write_text(json.dumps({'source':f'Actual Gazebo {a.topic} via ROS bridge','topic':a.topic,'frame':msg.header.frame_id,'stamp_sec':msg.header.stamp.sec,'stamp_nsec':msg.header.stamp.nanosec,'count':count,'width':msg.width,'height':msg.height,'archive_stride':a.archive_stride}))
        count+=1
    node.create_subscription(Image,a.topic,callback,qos_profile_sensor_data)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
