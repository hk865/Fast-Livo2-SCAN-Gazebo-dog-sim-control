#!/usr/bin/env python3
import math
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from heading_alignment import scene_heading_sample,calibrate_scene_heading,scene_waypoints_in_slam,yaw_rotation

IDENTITY=[0,0,0,1]
def q(degrees):return Rotation.from_euler('z',degrees,degrees=True).as_quat().tolist()
def pairs(imu=5,slam=0):
    return [{'imu_stamp':20+i*.1+.005,'slam_stamp':20+i*.1,'imu_quaternion':q(imu),
             'slam_body_quaternion':q(slam)} for i in range(15)]
def estimate(data):return calibrate_scene_heading(data,imu_reference_world_quaternion=IDENTITY,
    body_imu_quaternion=IDENTITY,reference_description='test fixture explicit CUSTOM/world reference')


class HeadingTests(unittest.TestCase):
    def test_five_degree_heading_corrects_fifteen_metre_route_sign(self):
        result=estimate(pairs());theta=result['yaw_camera_init_from_world']
        self.assertAlmostEqual(theta,math.radians(-5))
        p=scene_waypoints_in_slam([[15,0,2.4]],[.2,-.1,.01],theta)[0]
        world=yaw_rotation(math.radians(5))@(np.array(p)-[.2,-.1,.01])
        np.testing.assert_allclose(world,[15,0,2.4],atol=1e-12)
        self.assertAlmostEqual(p[2],2.41)

    def test_nonidentity_imu_mount_is_removed(self):
        result=scene_heading_sample(q(95),q(0),imu_reference_world_quaternion=IDENTITY,body_imu_quaternion=q(90))
        self.assertAlmostEqual(result['yaw_camera_init_from_world'],math.radians(-5))

    def test_boot_relative_orientation_requires_known_reference(self):
        result=scene_heading_sample(q(10),q(7),imu_reference_world_quaternion=q(30),body_imu_quaternion=IDENTITY)
        self.assertAlmostEqual(result['yaw_camera_init_from_world'],math.radians(-33))

    def test_wrapped_yaw_does_not_create_358_degree_correction(self):
        result=scene_heading_sample(q(179),q(-179),imu_reference_world_quaternion=IDENTITY,body_imu_quaternion=IDENTITY)
        self.assertAlmostEqual(result['yaw_camera_init_from_world'],math.radians(2))

    def test_body_tilt_cancels_before_yaw_projection(self):
        rwb=Rotation.from_euler('xyz',[8,-7,5],degrees=True).as_matrix()
        rcb=yaw_rotation(math.radians(-5))@rwb
        result=scene_heading_sample(Rotation.from_matrix(rwb).as_quat(),Rotation.from_matrix(rcb).as_quat(),
            imu_reference_world_quaternion=IDENTITY,body_imu_quaternion=IDENTITY)
        self.assertAlmostEqual(result['yaw_camera_init_from_world'],math.radians(-5))
        self.assertAlmostEqual(result['gravity_axis_residual_rad'],0,places=12)

    def test_rejects_stale_pair_and_reversed_clock(self):
        data=pairs();data[-1]['imu_stamp']+=.1
        with self.assertRaisesRegex(ValueError,'pairing tolerance'):estimate(data)
        data=pairs();data[5]=data[4]
        with self.assertRaisesRegex(ValueError,'must increase'):estimate(data)

    def test_rejects_unknown_reference_invalid_quaternion_and_inconsistent_heading(self):
        with self.assertRaisesRegex(ValueError,'reference assumption'):
            calibrate_scene_heading(pairs(),imu_reference_world_quaternion=IDENTITY,body_imu_quaternion=IDENTITY,reference_description='')
        data=pairs();data[0]['imu_quaternion']=[0,0,0,0]
        with self.assertRaisesRegex(ValueError,'nonzero'):estimate(data)
        data=pairs();data[-1]['imu_quaternion']=q(9)
        with self.assertRaisesRegex(ValueError,'not consistent'):estimate(data)

    def test_rotation_is_fixed_when_later_robot_heading_changes(self):
        result=estimate(pairs());theta=result['yaw_camera_init_from_world']
        p0=scene_waypoints_in_slam([[2,3,1.2]],[0,0,0],theta)
        # A later 90 degree robot turn is not an input to waypoint mapping.
        self.assertEqual(p0,scene_waypoints_in_slam([[2,3,1.2]],[0,0,0],theta))
        after=estimate(pairs(imu=95,slam=90))
        self.assertAlmostEqual(after['yaw_camera_init_from_world'],theta)


if __name__=='__main__':unittest.main()
