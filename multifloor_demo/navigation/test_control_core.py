#!/usr/bin/env python3
"""Behavior tests: invalid frames, feedback motion, ramp ground and obstacles."""
import math
import unittest
import numpy as np
from control_core import (rotation_xyzw, validate_request, follow_trajectory,
                          obstacle_ahead, remove_go2_self_returns, body_tilt, limit_acceleration, HeadingGate,
                          TrackingPositionFilter, trajectory_has_progress, MAX_TURN_RATE, MAX_WALK_YAW_RATE,
                          RawImuTilt)
from control_core import steering_obstacle_ahead


class NavigationGeometryTests(unittest.TestCase):
    def test_terminal_heading_uses_fixed_scale_checked_tangent(self):
        path=np.array([[x,0.,.1*x] for x in np.linspace(0,1,51)])
        pose=np.array([.9,.1,.09])
        _,_,target,steer=follow_trajectory(pose,np.eye(3),path,path[-1],gate_translation=False,return_steering=True)
        self.assertAlmostEqual(steer['heading'],math.atan2(-.1,.8))
        self.assertLess(abs(steer['heading']),.20)
        np.testing.assert_equal(target,path[-1])

    def test_fixed_scale_real_path_turn_and_body_yaw_still_stop(self):
        for path,rotation in [(np.array([[0.,0.,0.],[0.,1.,0.]]),np.eye(3)),
                              (np.array([[0.,0.,0.],[1.,0.,0.]]),rotation_xyzw([0,0,math.sin(.3),math.cos(.3)]))]:
            _,_,_,steer=follow_trajectory(np.zeros(3),rotation,path,path[-1],return_steering=True)
            yaw=math.atan2(rotation[1,0],rotation[0,0]);gate=HeadingGate();gate.phase='drive'
            self.assertFalse(gate.update(1.,yaw,steer['heading'])[0])
            self.assertEqual(gate.phase,'pre_turn')

    def test_real_cross_track_error_retains_persistent_and_severe_gates(self):
        path=np.array([[0.,0.,0.],[2.,0.,0.]])
        for cross,severe in [(.3,False),(.6,True)]:
            pose=np.array([.5,cross,0.]);_,_,_,steer=follow_trajectory(pose,np.eye(3),path,path[-1],gate_translation=False,return_steering=True)
            gate=HeadingGate();gate.phase='drive'
            self.assertEqual(gate.update(1.,0.,steer['heading'])[0],not severe)
            if not severe:self.assertFalse(gate.update(1.51,0.,steer['heading'])[0])
            self.assertEqual(gate.phase,'pre_turn')

    def test_exhausted_checked_path_does_not_drive_virtual_extension(self):
        path=np.array([[0.,0.,0.],[1.,0.,0.]])
        velocity,yaw,_,steer=follow_trajectory(np.array([1.1,.3,0.]),np.eye(3),path,path[-1],gate_translation=False,return_steering=True)
        self.assertTrue(steer['exhausted']);np.testing.assert_equal(velocity,np.zeros(2));self.assertEqual(yaw,0.)

    def test_obstacle_union_preserves_original_and_actual_motion_corridors(self):
        pose=np.array([.9,.4,0.]);target=np.array([1.,0.,0.]);direction=np.array([.8,-.4]);route=np.array([[0.,0.,0.],[2.,0.,0.]])
        for ray in [direction,target[:2]-pose[:2]]:
            center=pose[:2]+ray/np.linalg.norm(ray)*.8
            points=np.array([[center[0]+dx,center[1]+dy,.1] for dx,dy in [(-.01,0),(0,0),(.01,0)]])
            self.assertTrue(steering_obstacle_ahead(points,pose,target,direction,route)[0])
        self.assertFalse(steering_obstacle_ahead(np.array([[1.5,1.,.1]]*3),pose,target,direction,route)[0])

    def test_aligned_obstacle_resume_requires_stop_and_fresh_direction(self):
        for seconds,heading,allowed in [(.9,0.,False),(1.,.19,True),(1.,.21,False),(1.,math.pi/2,False)]:
            gate=HeadingGate()
            self.assertEqual(gate.resume_after_stop(0.,heading,seconds),allowed)
            self.assertEqual(gate.phase,'drive' if allowed else 'pre_turn')

    def test_raw_imu_fixed_extrinsic_recovers_body_tilt(self):
        body = rotation_xyzw([math.sin(.2),0.,0.,math.cos(.2)])
        extrinsic = rotation_xyzw([0.,math.sin(.3),0.,math.cos(.3)])
        # q(body roll .4 × fixed IMU pitch .6), independently composed.
        q = [math.sin(.2)*math.cos(.3), math.cos(.2)*math.sin(.3),
             math.sin(.2)*math.sin(.3), math.cos(.2)*math.cos(.3)]
        monitor = RawImuTilt(extrinsic)
        self.assertTrue(monitor.update(q,10.,1.,10.))
        self.assertAlmostEqual(monitor.tilt,body_tilt(body))

    def test_raw_imu_repeat_and_invalid_cannot_clear_or_refresh(self):
        monitor = RawImuTilt(np.eye(3))
        self.assertTrue(monitor.update([math.sin(.2),0,0,math.cos(.2)],10.,1.,10.))
        for q,t,available in [([0,0,0,1],10.,True),([0,0,0,1],9.9,True),
                              ([0,0,0,0],10.1,True),([float('nan'),0,0,1],10.1,True),
                              ([0,0,0,1],10.1,False)]:
            self.assertFalse(monitor.update(q,t,1.1,10.1,available))
        self.assertAlmostEqual(monitor.tilt,.4)
        self.assertFalse(monitor.fresh(1.3,10.3))

    def test_raw_imu_stale_backlog_and_future_clock_are_rejected(self):
        monitor = RawImuTilt(np.eye(3))
        self.assertFalse(monitor.update([0,0,0,1],9.,1.,10.))
        self.assertFalse(monitor.update([0,0,0,1],10.2,1.,10.))
        self.assertTrue(monitor.update([0,0,0,1],10.05,1.,10.))
        self.assertTrue(monitor.fresh(1.1,10.1))
        self.assertFalse(monitor.fresh(1.1,10.4))

    def test_request_rejects_untransformed_world_and_nonfinite(self):
        for request in [dict(request_id='r', frame_id='world', waypoints=[[1,2,3]]),
                        dict(request_id='r', frame_id='camera_init', waypoints=[[1,2,float('nan')]]),
                        dict(request_id='r', frame_id='camera_init', waypoints=[])]:
            with self.assertRaises(ValueError):
                validate_request(request)

    def test_standard_body_velocity_rotation(self):
        rot = rotation_xyzw([0, 0, math.sin(math.pi/4), math.cos(math.pi/4)])
        np.testing.assert_allclose(rot @ [0.2, 0, 0], [0, 0.2, 0], atol=1e-10)

    def test_controller_uses_measured_pose_for_lookahead(self):
        path = np.array([[x, 0, 0] for x in np.linspace(0, 4, 81)])
        velocity, yaw, target = follow_trajectory(np.array([2.,0.,0.]), np.eye(3), path, path[-1])
        self.assertGreater(target[0], 2.)
        self.assertLess(target[0], 2.95)
        self.assertGreater(velocity[0], 0)
        self.assertLessEqual(np.linalg.norm(velocity), 0.12)

    def test_heading_alignment_precedes_translation(self):
        path = np.array([[0, 0, 0], [0, 1, 0]])
        velocity, yaw, _ = follow_trajectory(np.zeros(3), np.eye(3), path, path[-1])
        np.testing.assert_equal(velocity, np.zeros(2))
        self.assertGreater(yaw, 0)

    def test_goal_reached_does_not_continue_along_old_spline(self):
        path = np.array([[0, 0, 0], [2, 0, 0]])
        velocity, _, _ = follow_trajectory(np.array([1.,0,0]), np.eye(3), path, np.array([1.,0,0]))
        np.testing.assert_equal(velocity, np.zeros(2))

    def test_ramp_support_is_not_obstacle(self):
        route = np.array([[0, 0, 0], [4, 0, 0.4]])
        ground = np.array([[x, y, x*0.1-0.3] for x in np.arange(.35,.94,.05)
                           for y in np.arange(-.3,.31,.05)])
        blocked, _, _ = obstacle_ahead(ground, np.zeros(3), np.array([1.,0,.1]), route)
        self.assertFalse(blocked)

    def test_real_returns_in_swept_corridor_stop(self):
        points = np.array([[.65,y,z] for y in [-.1,0,.1] for z in [0.,.1,.2]])
        blocked, clearance, count = obstacle_ahead(points, np.zeros(3), np.array([1.,0,0]),
                                                   np.array([[0,0,0],[2,0,0]]))
        self.assertTrue(blocked)
        self.assertAlmostEqual(clearance, .65)
        self.assertEqual(count, 9)

    def test_side_wall_is_not_in_swept_corridor(self):
        points = np.array([[x,.8,.1] for x in np.arange(.3,1,.01)])
        self.assertFalse(obstacle_ahead(points, np.zeros(3), np.array([1.,0,0]),
                                       np.array([[0,0,0],[2,0,0]]))[0])

    def test_self_filter_preserves_outside_obstacles_and_uses_body_rotation(self):
        local = np.array([[-.2,.15,.04],[-.38,-.18,-.18], [.65,0,.1], [0,.5,.1], [1,0,-.3]])
        rot = rotation_xyzw([0,0,math.sin(.6),math.cos(.6)])
        pose = np.array([3.,4.,1.2])
        world = local @ rot.T+pose
        filtered, count = remove_go2_self_returns(world,pose,rot)
        self.assertEqual(count,2)
        np.testing.assert_allclose(filtered,world[2:])

    def test_translation_has_no_lateral_motion_and_turns_first(self):
        path=np.array([[0.,0.,0.],[1.,.3,0.]])
        velocity,yaw,_=follow_trajectory(np.zeros(3),np.eye(3),path,path[-1])
        np.testing.assert_equal(velocity,np.zeros(2))
        self.assertLessEqual(abs(yaw),MAX_TURN_RATE)
        path[-1,1]=.1
        velocity,yaw,_=follow_trajectory(np.zeros(3),np.eye(3),path,path[-1])
        self.assertGreater(velocity[0],0)
        self.assertEqual(velocity[1],0)
        self.assertLessEqual(abs(yaw),MAX_WALK_YAW_RATE)

    def test_acceleration_limited_but_turn_alignment_stops_translation(self):
        command=limit_acceleration([0.,0.,0.],[.12,0,.12],.05)
        np.testing.assert_allclose(command,[.0075,0,.0125])
        command=limit_acceleration([.12,0.,0.],[0.,0.,.12],.05)
        self.assertEqual(command[0],0.)

    def test_tilt_uses_slam_orientation(self):
        rot=rotation_xyzw([math.sin(.2),0,0,math.cos(.2)])
        self.assertAlmostEqual(body_tilt(rot),.4)

    def test_heading_is_latched_during_turn_despite_new_scan_path(self):
        gate=HeadingGate()
        self.assertEqual(gate.update(0.,0.,-1.57),(False,0.))
        move,yaw=gate.update(1.1,0.,-1.57)
        self.assertFalse(move);self.assertLess(yaw,0)
        move,yaw=gate.update(1.2,-1.,-.8)
        self.assertFalse(move);self.assertLess(yaw,0)
        self.assertEqual(gate.heading,-1.57)

    def test_settle_is_uninterrupted_and_allows_physical_residual_yaw(self):
        gate=HeadingGate()
        gate.update(0.,0.,-1.57)
        gate.update(1.1,0.,-1.57)
        self.assertEqual(gate.update(2.,-1.50,-1.57),(False,0.))
        self.assertEqual(gate.phase,'settle')
        self.assertEqual(gate.update(2.5,-1.70,-1.57),(False,0.))
        move,yaw=gate.update(3.1,-1.70,-1.57)
        self.assertTrue(move);self.assertEqual(gate.phase,'drive')

    def test_large_heading_change_after_settle_requires_a_new_turn(self):
        gate=HeadingGate();gate.update(0.,0.,0.);gate.update(1.1,0.,0.)
        self.assertEqual(gate.update(2.2,0.,1.),(False,0.))
        self.assertEqual(gate.phase,'pre_turn')
        move,yaw=gate.update(3.3,0.,1.)
        self.assertFalse(move);self.assertEqual(gate.phase,'align')
        self.assertEqual(gate.heading,1.)

    def test_drive_to_turn_has_full_zero_command_braking_window(self):
        gate=HeadingGate();gate.update(0.,0.,0.);gate.update(1.1,0.,0.)
        self.assertTrue(gate.update(2.2,0.,0.)[0])
        self.assertEqual(gate.update(2.3,0.,1.),(False,0.))
        self.assertEqual(gate.phase,'pre_turn')
        self.assertEqual(gate.update(3.2,0.,1.),(False,0.))
        self.assertIsNone(gate.heading)
        move,yaw=gate.update(3.4,0.,1.)
        self.assertFalse(move);self.assertGreater(yaw,0)

    def test_gait_heading_excursion_does_not_restart_a_stride(self):
        gate=HeadingGate();gate.update(0.,0.,0.);gate.update(1.1,0.,0.)
        self.assertTrue(gate.update(2.2,0.,0.)[0])
        self.assertTrue(gate.update(2.3,0.,.30)[0])
        self.assertTrue(gate.update(2.7,0.,.30)[0])
        self.assertTrue(gate.update(2.75,0.,.10)[0])
        self.assertEqual(gate.phase,'drive')
        self.assertIsNone(gate.outside_since)

    def test_persistent_heading_error_stops_after_half_a_sim_second(self):
        gate=HeadingGate();gate.update(0.,0.,0.);gate.update(1.1,0.,0.)
        gate.update(2.2,0.,0.)
        self.assertTrue(gate.update(2.3,0.,.30)[0])
        self.assertEqual(gate.update(2.81,0.,.30),(False,0.))
        self.assertEqual(gate.phase,'pre_turn')

    def test_large_route_direction_change_stops_immediately(self):
        gate=HeadingGate();gate.update(0.,0.,0.);gate.update(1.1,0.,0.)
        gate.update(2.2,0.,0.)
        self.assertEqual(gate.update(2.3,0.,.56),(False,0.))

    def test_measured_tracking_filter_rejects_repeats_and_preserves_raw_pose(self):
        filt=TrackingPositionFilter()
        start=np.zeros(3);sway=np.array([.08,0,0])
        filt.update(start,1.)
        smoothed=filt.update(sway,1.1)
        self.assertLess(smoothed[0],.018)
        np.testing.assert_equal(sway,[.08,0,0])
        np.testing.assert_equal(filt.update([9,9,9],1.1),smoothed)
        np.testing.assert_equal(filt.update([1,2,3],.1),[1,2,3])

    def test_filtered_route_direction_does_not_bypass_raw_goal_distance(self):
        path=np.array([[0.,0.,0.],[1.,0.,0.]])
        velocity,_,_=follow_trajectory(np.array([1.,0,0]),np.eye(3),path,path[-1],
                                      tracking_pose=np.array([.9,0,0]),gate_translation=False)
        np.testing.assert_equal(velocity,np.zeros(2))
        # Translation permission belongs to HeadingGate in the live controller.
        velocity,_,_=follow_trajectory(np.array([.7,0,0]),np.eye(3),path,path[-1],
                                      tracking_pose=np.array([.7,-.1,0]),gate_translation=False)
        self.assertGreater(velocity[0],0.)

    def test_tracking_filter_same_sensor_time_has_same_output_at_any_wall_rate(self):
        filters=[TrackingPositionFilter(),TrackingPositionFilter()]
        positions=[np.array([x,0,0]) for x in [0,.08,0,.02]]
        stamps=[1.,1.1,1.2,1.3]
        results=[[f.update(p,t) for p,t in zip(positions,stamps)] for f in filters]
        np.testing.assert_equal(results[0],results[1])

    def test_scan_parking_spline_is_rejected_before_actual_arrival(self):
        stop=np.repeat([[.9,-.15,0.]],20,axis=0)
        self.assertFalse(trajectory_has_progress(stop,np.array([.9,-.15,0.]),np.zeros(3)))
        self.assertTrue(trajectory_has_progress(stop,np.array([.1,0.,0.]),np.zeros(3)))

    def test_short_valid_scan_trajectory_is_not_misclassified_as_parking(self):
        samples=np.array([[x,0.,0.] for x in np.linspace(.3,0.,20)])
        self.assertTrue(trajectory_has_progress(samples,np.array([.3,0.,0.]),np.zeros(3)))
        samples=np.array([[x,0.,0.] for x in np.linspace(.3,.29,20)])
        self.assertFalse(trajectory_has_progress(samples,np.array([.3,0.,0.]),np.zeros(3)))

    def test_turn_rate_stays_within_measured_slam_ab_range(self):
        gate=HeadingGate();gate.update(0.,0.,math.pi)
        move,yaw=gate.update(1.1,0.,math.pi)
        self.assertFalse(move)
        self.assertEqual(abs(yaw),.12)
        _,yaw,_=follow_trajectory(np.zeros(3),np.eye(3),
                                  np.array([[0.,0.,0.],[0.,1.,0.]]),np.array([0.,1.,0.]))
        self.assertEqual(abs(yaw),.12)


if __name__ == '__main__':
    unittest.main(verbosity=2)
