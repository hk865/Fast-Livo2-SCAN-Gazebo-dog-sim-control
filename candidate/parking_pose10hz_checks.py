"""Finite motion/data-policy checks, not simulation or historical PASS repair."""
import json,math,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace as NS
import numpy as np
from parking_pose10hz import PoseGrid,CaptureGrid,validate_capture,CONFIG
from ordinary_parking import OrdinaryParking

def row(t,x=0.,yaw=0.,speed=0.):
    ns=round(t*1e9)
    return dict(stamp_ns=ns,position=[x,0.,0.],quaternion_xyzw=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],
        body_velocity=[speed,0.,0.],source_topic='/demo/slam/body_odom',navigation_ground_truth_used=False,
        received_sim_ns=ns,received_wall_ns=ns,source_received_wall_ns=ns)

class PoseParkingChecks(unittest.TestCase):
    def owner(self):
        owner=OrdinaryParking();owner.configure_pose10hz(CONFIG)
        owner.capture_goal=[0.,0.,0.];owner.capture_heading=0.;owner.begin(0,20_000_000_000)
        return owner
    def commit(self,owner,source):
        from r86_prepared_feedback import check_negative_parking
        q=source['quaternion_xyzw'];clock=source['stamp_ns']
        feedback=dict(stamp_ns=clock,position_world_xyz=source['position'],quaternion_wxyz=q[-1:]+q[:-1],
            origin_velocity_body=source['body_velocity'],received_wall_ns=source['received_wall_ns'],
            callback_ros_clock_ns=source['received_sim_ns'],frame_id='camera_init')
        core=NS(ordinary_parking=owner,goal=np.zeros(3),goal_yaw=0.,goal_z_tolerance=.1,
            ordinary_zero_event=None,_protect=lambda *a,**k:None)
        node=NS(cascade=core,feedback=feedback,paired_imu=dict(stamp_ns=clock,angular_velocity_body=[0.,0.,99.]),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock)))
        check_negative_parking(node)
    def window(self,coordinate=lambda k:0.):
        grid=PoseGrid(0,row(0.))
        for k in range(1,51):grid.observe(row(k*.1,coordinate(k)))
        return grid
    def test_high_speed_never_vetoes_capture_or_window(self):
        capture=CaptureGrid()
        for k in range(7):capture.observe(row(k*.1,speed=99.),[0.,0.,0.],0.)
        self.assertTrue(capture.ready)
        self.assertTrue(validate_capture(capture.evidence(),[0.,0.,0.],0.,capture.last['source']))
        grid=PoseGrid(0,row(0.,speed=99.))
        for k in range(1,51):grid.observe(row(k*.1,speed=99.))
        self.assertTrue(grid.evidence()['passed'])
    def test_slow_cumulative_drift_fails_fixed_anchor(self):
        grid=self.window(lambda k:k*.0012)
        self.assertLess(max(s['adjacent_xy_m']or 0 for s in grid.samples),.002)
        self.assertFalse(grid.evidence()['passed']);self.assertIsNotNone(grid.motion_failure)
    def test_sampled_excursion_then_return_is_not_endpoint_pass(self):
        grid=self.window(lambda k:.06 if k==20 else 0.)
        self.assertEqual(grid.samples[-1]['xy_from_anchor_m'],0.)
        self.assertFalse(grid.evidence()['passed'])
    def test_unsampled_high_frequency_excursion_is_diagnostic_only(self):
        grid=PoseGrid(0,row(0.))
        for k in range(1,501):
            t=k*.01
            grid.observe(row(t,.08 if k%10==5 else 0.,speed=2.))
        self.assertTrue(grid.evidence()['passed'])
    def test_no_duplicate_or_missing_pose_fabricates_complete_window(self):
        grid=PoseGrid(0,row(0.));grid.observe(row(.1));grid.observe(row(.1));grid.observe(row(.4))
        for k in range(5,51):grid.observe(row(k*.1))
        self.assertTrue(grid.evidence()['complete']);self.assertFalse(grid.evidence()['data_available'])
        self.assertIsNone(grid.motion_failure)
    def test_causal_selection_no_interpolation(self):
        grid=PoseGrid(0,row(0.));grid.observe(row(.09,.01));grid.observe(row(.11,.02))
        selected=grid.samples[1]
        self.assertEqual(selected['grid_ns'],100_000_000)
        self.assertEqual(selected['source_stamp_ns'],90_000_000)
        self.assertEqual(selected['position'][0],.01)
    def test_yaw_wrap_and_fixed_anchor(self):
        grid=PoseGrid(0,row(0.,yaw=math.pi-.02))
        for k in range(1,51):grid.observe(row(k*.1,yaw=-math.pi+.02))
        self.assertTrue(grid.evidence()['passed'])
        self.assertAlmostEqual(grid.evidence()['max_abs_yaw_from_anchor_rad'],.04)
        grid=self.window();grid.observe(row(5.1))
        self.assertEqual(len(grid.samples),51)
    def test_online_owner_uses_same_window_and_never_reanchors(self):
        owner=OrdinaryParking();owner.configure_pose10hz(CONFIG);owner.capture_goal=[0.,0.,0.];owner.capture_heading=0.;owner.begin(0,20_000_000_000)
        for k in range(7):
            p=row(k*.1,speed=99.)
            owner.update(p['stamp_ns'],p['stamp_ns'],p['position'],0.,[99.,0.,0.],99.,99.,0.,0.,sensor_pose=p)
        self.assertEqual(owner.stage,'handoff_zero')
        proof=dict(qualification_clock_ns=600_000_000,qualification_origin=owner.capture_grid.last['source'])
        owner.start_pose10hz_window(proof)
        with self.assertRaises(ValueError):owner.start_pose10hz_window(proof)
        for k in range(7,57):
            p=row(k*.1,.06 if k==30 else 0.,speed=99.)
            if owner.failure:break
            owner.update(p['stamp_ns'],p['stamp_ns'],p['position'],0.,[99.,0.,0.],99.,99.,0.,0.,sensor_pose=p)
        self.assertEqual(owner.failure,'parking_pose10hz_fixed_anchor_exceeded')
    def test_real_R86_hook_owns_samples_but_never_grants_handoff(self):
        owner=self.owner()
        for k in range(7):self.commit(owner,row(k*.1,speed=99.))
        self.assertTrue(owner.capture_grid.ready)
        self.assertIsNone(owner.ready_stamp);self.assertIsNone(owner.first_actor_zero_ack)
        self.assertEqual(owner.stage,'capture')
        p=row(.6,speed=99.)
        owner.update(p['stamp_ns'],p['stamp_ns'],p['position'],0.,[99.,0.,0.],99.,99.,0.,0.,sensor_pose=p)
        self.assertEqual(owner.stage,'handoff_zero')
        self.assertEqual(len(owner.capture_grid.grid.samples),7)
    def test_batched_R86_sources_match_audit_grid_and_do_not_skip_intermediate_source(self):
        owner=self.owner();origin=row(0.);self.commit(owner,origin)
        owner.start_pose10hz_window(dict(qualification_clock_ns=0,qualification_origin=origin))
        sources=[row(.06),row(.09,.06),row(.12)]
        for source in sources:self.commit(owner,source)
        self.assertEqual(owner.failure,'parking_pose10hz_fixed_anchor_exceeded')
        replay=PoseGrid(0,origin)
        for source in sources:replay.observe(source)
        self.assertEqual(owner.pose_grid.samples,replay.samples)
        self.assertEqual(owner.pose_grid.samples[1]['source_stamp_ns'],90_000_000)
    def test_R86_unsampled_raw_excursion_and_high_speed_are_not_motion_vetoes(self):
        owner=self.owner();origin=row(0.);self.commit(owner,origin)
        owner.start_pose10hz_window(dict(qualification_clock_ns=0,qualification_origin=origin))
        for p in [row(.05,.4,speed=99.),row(.1,speed=99.),row(.2,speed=99.)]:self.commit(owner,p)
        self.assertIsNone(owner.failure);self.assertIsNone(owner.pose_grid.motion_failure)
    def test_preT0_protection_clears_capture_dwell_but_never_resets_T0(self):
        owner=self.owner()
        for k in range(7):self.commit(owner,row(k*.1))
        self.assertTrue(owner.capture_grid.ready);owner.guard('fixture_source_fault')
        self.assertIsNone(owner.capture_grid.grid)
        for k in range(7,13):self.commit(owner,row(k*.1))
        self.assertFalse(owner.capture_grid.ready)
        self.commit(owner,row(1.3));self.assertTrue(owner.capture_grid.ready)
        owner.start_pose10hz_window(dict(qualification_clock_ns=1_300_000_000,qualification_origin=row(1.3)))
        window=owner.pose_grid;owner.guard('fixture_source_fault')
        self.assertIs(owner.pose_grid,window)
    def test_committed_prefix_closes_before_concurrent_tail_and_keeps_strict_join(self):
        from parking_pose10hz import committed_poses
        poses=[row(k*.1)for k in range(51)]
        late=row(5.25) # Outside the auditor's already-frozen callback snapshot.
        def commit(p):
            return dict(source_ns=p['stamp_ns'],accepted_pose_stamp_ns=p['stamp_ns'],
                accepted_cloud_stamp_ns=p['stamp_ns'],control_allowed=True,
                navigation_ground_truth_used=False,source_receipts_refreshed=False,
                parking_pose10hz_source_intake=True)
        def feedback(p):
            q=p['quaternion_xyzw']
            return dict(stamp_ns=p['stamp_ns'],position_world_xyz=p['position'],
                quaternion_wxyz=q[-1:]+q[:-1],origin_velocity_body=p['body_velocity'],
                frame_id='camera_init',callback_ros_clock_ns=p['received_sim_ns'],
                received_wall_ns=p['received_wall_ns'])
        with tempfile.TemporaryDirectory()as td:
            cp=Path(td,'known_scene_controller_inputs.jsonl');fp=Path(td,'navigation_feedback_history.jsonl')
            def write(commits,feedbacks):
                cp.write_text(''.join(json.dumps(c)+'\n'for c in commits))
                fp.write_text(''.join(json.dumps(f)+'\n'for f in feedbacks))
            cs=[commit(p)for p in poses];fs=[feedback(p)for p in poses]
            write(cs+[commit(late)],fs+[feedback(late)])
            selected,proof=committed_poses(td,poses,0,5_300_000_000,window_end_ns=5_000_000_000)
            self.assertEqual(len(selected),51);self.assertEqual(proof['closing_source_ns'],5_000_000_000)
            with self.assertRaisesRegex(ValueError,'absent from original SLAM'):
                committed_poses(td,poses[:-1],0,5_300_000_000,window_end_ns=5_000_000_000)
            altered=[dict(p)for p in poses];altered[20]=dict(altered[20],position=[.001,0.,0.])
            with self.assertRaisesRegex(ValueError,'pose differs'):
                committed_poses(td,altered,0,5_300_000_000,window_end_ns=5_000_000_000)
            write(cs,fs[:-1])
            with self.assertRaisesRegex(ValueError,'feedback unavailable'):
                committed_poses(td,poses,0,5_300_000_000,window_end_ns=5_000_000_000)
            write(cs[:-1],fs)
            with self.assertRaisesRegex(ValueError,'closing source unavailable'):
                committed_poses(td,poses,0,5_300_000_000,window_end_ns=5_000_000_000)
            write(cs[:21]+[cs[20]]+cs[21:],fs)
            with self.assertRaisesRegex(ValueError,'identity/order'):
                committed_poses(td,poses,0,5_300_000_000,window_end_ns=5_000_000_000)

    def test_final_audit_has_no_native_motion_or_count_gate(self):
        import mission46_runtime_evidence as verifier
        poses=[row(k*.1,speed=99.)for k in range(51)]
        event={'synthetic_fixture_only':True}
        statuses=[dict(received_sim_ns=round(k*.1e9),data=dict(state='succeeded',request_id='fixture',command=[0.,0.,0.],
            cascade_parking=dict(mode='ordinary_zero_hold',ordinary_terminal_zero_event=event,ordinary_parking={}),
            execution_bridge_safety=dict(state='ready')))for k in range(51)]
        # One native record intentionally cannot meet the old 200-record gate;
        # its large velocity is finite and is not an independent physical fault.
        native=dict(state_physics_world_time=.5,world_sim_time=.505,rpy=[0.,0.,0.],body_lin_vel=[99.,0.,0.],body_ang_vel=[0.,0.,99.],
            qd=[0.]*12,applied_torque=[0.]*12,q_target=[0.]*12,body_clearance=.3,contacts={'body':0.},
            fault=None,actor_inferred_this_frame=True,command=[0.,0.,0.],requested=[0.,0.,0.],command_expired=False)
        with tempfile.TemporaryDirectory()as td:
            Path(td,'navigation_profile.json').write_text('{}')
            Path(td,'known_scene_controller_inputs.jsonl').write_text(''.join(json.dumps(dict(source_ns=p['stamp_ns'],
                accepted_pose_stamp_ns=p['stamp_ns'],accepted_cloud_stamp_ns=p['stamp_ns'],control_allowed=True,
                navigation_ground_truth_used=False,source_receipts_refreshed=False,parking_pose10hz_source_intake=True))+'\n'for p in poses))
            Path(td,'navigation_feedback_history.jsonl').write_text(''.join(json.dumps(dict(stamp_ns=p['stamp_ns'],
                position_world_xyz=p['position'],quaternion_wxyz=p['quaternion_xyzw'][-1:]+p['quaternion_xyzw'][:-1],
                origin_velocity_body=p['body_velocity'],frame_id='camera_init',callback_ros_clock_ns=p['received_sim_ns'],
                received_wall_ns=p['received_wall_ns']))+'\n'for p in poses))
            # A subscriber-only pose cannot alter the accepted controller stream.
            poses.append(row(.55,.2,speed=99.))
            with patch.object(verifier,'verify_line'),patch.object(verifier,'validate_zero_observation'),\
                 patch.object(verifier,'seal',side_effect=lambda r,n,p:p),patch.object(verifier,'receipt',side_effect=lambda *a,**k:k):
                result=verifier.verify_pose10hz_parking(td,'fixture',0,poses,[dict(data=native)],statuses,poses[0],dict(event=event),{},'fixture.json')
                self.assertEqual(result['metrics']['slam_pose10hz_xy_drift_max_m'],0.)
                bad=dict(native,fault='fixture_physical_fault')
                with self.assertRaisesRegex(ValueError,'Independent native physical'):
                    verifier.verify_pose10hz_parking(td,'fixture',0,poses,[dict(data=bad)],statuses,poses[0],dict(event=event),{},'fixture.json')

if __name__=='__main__':unittest.main()
