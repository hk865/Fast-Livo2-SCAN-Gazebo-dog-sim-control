"""Meaningful geometry/input counterexamples for an excluded hypothesis only."""
import math
import unittest
from turn_target_candidate import BoundedTurnReference,angle

KEY=('run:exploration','goal3',11,(111,792000000))

class CandidateTests(unittest.TestCase):
    def test_real_ninety_degree_turn_cannot_false_settle(self):
        c=BoundedTurnReference(KEY,math.pi/2,0)
        for i in range(1,20):
            self.assertFalse(c.update(i*100_000_000,math.pi/2,0,KEY)['can_start_original_settle'])
        self.assertTrue(c.update(2_000_000_000,math.pi/2,math.pi/2-.09,KEY)['can_start_original_settle'])

    def test_alternating_sway_does_not_accumulate_correction(self):
        c=BoundedTurnReference(KEY,0,0)
        for i in range(1,101):
            r=c.update(i*50_000_000,.12 if i%2 else -.12,-.5,KEY)
            self.assertEqual(r['reference_offset'],0)
            self.assertFalse(r['can_start_original_settle'])

    def test_sustained_change_is_rate_and_total_bounded(self):
        c=BoundedTurnReference(KEY,1,0)
        previous=1.
        for i in range(1,71):
            r=c.update(i*100_000_000,.72,0,KEY)
            self.assertLessEqual(abs(angle(r['reference_heading']-previous)),.006+1e-12)
            self.assertLessEqual(abs(r['reference_offset']),.30+1e-12)
            previous=r['reference_heading']
        self.assertAlmostEqual(previous,.72)

    def test_full_l_turn_requires_replan(self):
        c=BoundedTurnReference(KEY,0,0)
        r=c.update(100_000_000,math.pi/2,0,KEY)
        self.assertTrue(r['needs_fresh_plan']);self.assertTrue(r['caller_must_hold'])
        self.assertFalse(r['can_start_original_settle']);self.assertEqual(c.reference,0)

    def test_new_trajectory_same_goal_cannot_relabel_old_reference(self):
        c=BoundedTurnReference(KEY,0,0)
        changed=(*KEY[:2],12,KEY[3])
        r=c.update(100_000_000,.1,0,changed)
        self.assertTrue(r['needs_fresh_plan']);self.assertFalse(r['can_start_original_settle'])

    def test_new_goal_same_center_identity_rejected(self):
        c=BoundedTurnReference(KEY,0,0)
        r=c.update(100_000_000,0,0,(KEY[0],'differentgoal',KEY[2],KEY[3]))
        self.assertTrue(r['caller_must_hold']);self.assertFalse(r['can_start_original_settle'])

    def test_repeated_native_stamp_cannot_complete_or_accumulate(self):
        c=BoundedTurnReference(KEY,0,0)
        a=c.update(100_000_000,.1,-.5,KEY)
        for i in range(20):
            r=c.update(100_000_000,0,0,KEY)
            self.assertFalse(r['can_start_original_settle'])
            self.assertEqual(r['reference_heading'],a['reference_heading'])

    def test_observation_gap_resets_coherence_and_holds(self):
        c=BoundedTurnReference(KEY,0,0)
        c.update(100_000_000,.1,-.5,KEY)
        r=c.update(400_000_001,.1,0,KEY)
        self.assertTrue(r['caller_must_hold']);self.assertFalse(r['can_start_original_settle'])
        self.assertEqual(c.reference,0)

    def test_side_obstacle_protected_cannot_settle_or_update(self):
        c=BoundedTurnReference(KEY,0,0)
        for i in range(1,12):
            r=c.update(i*100_000_000,.15,0,KEY,protected=True)
            self.assertTrue(r['caller_must_hold']);self.assertFalse(r['can_start_original_settle'])
            self.assertEqual(c.reference,0)

    def test_nonfinite_heading_and_noninteger_stamp_hold(self):
        c=BoundedTurnReference(KEY,0,0)
        for t,h,y in [(100_000_000,math.nan,0),(100_000_000,0,math.inf),(1.0,0,0)]:
            self.assertTrue(c.update(t,h,y,KEY)['caller_must_hold'])

    def test_pi_wrap_short_direction_correction(self):
        c=BoundedTurnReference(KEY,math.pi-.04,0)
        for i in range(1,30):
            r=c.update(i*100_000_000,-math.pi+.04,0,KEY)
            self.assertLessEqual(abs(r['reference_offset']),.08+1e-12)
        self.assertAlmostEqual(angle(c.reference-(-math.pi+.04)),0)

    def test_current_path_error_cannot_bypass_original_drive_gate(self):
        c=BoundedTurnReference(KEY,0,0)
        r=c.update(100_000_000,.25,0,KEY)
        self.assertFalse(r['can_start_original_settle'])

if __name__=='__main__':unittest.main(verbosity=2)
