#!/usr/bin/env python3
"""Read-only measured-cloud diagnosis; never publishes a control or sensor message."""
import json,time,datetime
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rcl_interfaces.srv import GetParameters
from control_core import rotation_xyzw
archive=Path(__file__).parent/'diagnostics'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
archive.mkdir(parents=True,exist_ok=True)
rclpy.init()
n=Node('demo_readonly_cloud_diagnostic')
clouds={}; poses={}
def cloud_cb(topic,m):
    p=read_points_numpy(m,field_names=('x','y','z'),skip_nans=True)
    clouds[topic]=np.asarray(p).reshape(-1,3)
for topic in ['/cloud_registered_full','/demo/navigation/cloud','/demo/navigation/occupancy','/grid_map/occupancy_inflate']:
    n.create_subscription(PointCloud2,topic,lambda m,t=topic:cloud_cb(t,m),qos_profile_sensor_data)
n.create_subscription(Odometry,'/demo/slam/body_odom',lambda m:poses.update(body=m),qos_profile_sensor_data)
client=n.create_client(GetParameters,'/scan_planner_node/get_parameters')
names=['grid_map.obstacles_inflation_z_down','grid_map.obstacles_inflation_z_up','grid_map.body_height','grid_map.resolution','grid_map.ground_height']
future=client.call_async(GetParameters.Request(names=names))
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
result={'topics':{},'parameters':{},'pose':None}
if future.done() and future.result():
    result['parameters']={key:{'double':val.double_value,'integer':val.integer_value} for key,val in zip(names,future.result().values)}
if 'body' in poses:
    p=poses['body'].pose.pose.position;q=poses['body'].pose.pose.orientation
    pose=np.array([p.x,p.y,p.z]);rot=rotation_xyzw([q.x,q.y,q.z,q.w]);result['pose']=pose.tolist()
    result['rotation_world_body']=rot.tolist()
    for topic,points in clouds.items():
        local=(points-pose)@rot
        near=local[np.linalg.norm(local[:,:2],axis=1)<.75]
        result['topics'][topic]={'points':len(points),'near_count':len(near),
            'near_z_range':None if not len(near) else [float(near[:,2].min()),float(near[:,2].max())],
            'near_above_floor':near[near[:,2]>-.2].tolist()[:12],
            'body_box_returns':int(((np.abs(local[:,0])<.30)&(np.abs(local[:,1])<.25)&(local[:,2]>-.32)&(local[:,2]<.2)).sum())}
        np.save(Path(__file__).with_name(topic.strip('/').replace('/','_')+'.npy'),local)
        np.save(archive/(topic.strip('/').replace('/','_')+'.npy'),local)
Path(__file__).with_name('live_cloud_diagnostic.json').write_text(json.dumps(result,indent=2))
(archive/'diagnostic.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
n.destroy_node();rclpy.shutdown()
