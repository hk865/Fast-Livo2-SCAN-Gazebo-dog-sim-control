"""Meaningful edge negatives for the test-only held interval evaluator."""
import copy
import unittest
from held_command_contract import audit_held_commands


class HeldTests(unittest.TestCase):
    def setUp(self):
        self.events=[dict(event=dict(event_id='event_000001',request_id='actual',edge_request_id='actual',
            edge_waypoint_index=1,waypoint_index=1,obstacle_edge_stamp=10.,zero_command_stamp=10.,
            zero_command=[0.,0.,0.],guard_result=[True,.7,8],reference_stamp=[9,0]))]
        self.statuses=[dict(sim=20.3,status=dict(request_id='actual',waypoint_index=1,state='running',
            obstacle_stops=1,obstacle_resumes=1,obstacle_hold=False,trajectory_reference_stamp=[20,0],
            accepted_trajectory_id=3,steering=dict(stamp=20.29,trajectory_id=3)))]
        self.metadata=[dict(received_sim=20.02,metadata=dict(reference_stamp=[20,0],body_goal=[2,2,0],trajectory=dict(traj_id=3)))]
        self.commands=[dict(sim=10.01,source='requested',command=[0.,0.,0.],nav_request_id='actual',nav_index=1),
            dict(sim=10.03,source='safe',command=[0.,0.,0.],nav_request_id='actual',nav_index=1),
            dict(sim=20.1,source='safe',command=[.12,0.,0.],nav_request_id='actual',nav_index=1,obstacle_hold=True)]

    def result(self):
        return audit_held_commands(self.events,self.statuses,self.metadata,self.commands,'actual',[[0,2,0],[2,2,0]])

    def test_real_executed_reference_excludes_late_4hz_hold_label(self):
        self.assertTrue(self.result()['passed'])
        self.assertEqual(self.result()['intervals'][0]['end'],20.)

    def test_metadata_only_unexecuted_reference_does_not_end_hold(self):
        self.statuses[0]['status']['steering']=None
        self.assertFalse(self.result()['passed'])

    def test_unaccepted_new_reference_does_not_end_hold(self):
        self.statuses[0]['status']['accepted_trajectory_id']=None
        self.assertFalse(self.result()['passed'])

    def test_old_goal_must_not_end_hold(self):
        self.metadata[0]['metadata']['body_goal']=[0,2,0]
        self.assertFalse(self.result()['passed'])

    def test_other_request_must_not_end_hold(self):
        self.statuses[0]['status']['request_id']='stale'
        self.assertFalse(self.result()['passed'])

    def test_true_nonzero_during_held_must_fail(self):
        self.commands[1]['command']=[0.,0.,-.04]
        self.assertFalse(self.result()['passed'])

    def test_reference_from_old_hold_must_not_end_hold(self):
        self.statuses[0]['status']['trajectory_reference_stamp']=[9,0]
        self.assertFalse(self.result()['passed'])

    def test_missing_actual_resume_counter_does_not_end_hold(self):
        self.statuses[0]['status']['obstacle_resumes']=0
        self.assertFalse(self.result()['passed'])

    def test_missing_native_zero_is_not_safety_pass(self):
        self.events[0]['event']['zero_command']=[0,0,-.04]
        self.assertFalse(self.result()['passed'])

    def test_missing_command_source_must_fail(self):
        self.commands=[c for c in self.commands if c['source']!='requested']
        self.assertFalse(self.result()['passed'])

    def test_reblocked_interval_nonzero_still_fails(self):
        second=copy.deepcopy(self.events[0]);e=second['event']
        e.update(event_id='event_000002',obstacle_edge_stamp=20.05,zero_command_stamp=20.05,reference_stamp=[20,0])
        self.events.append(second)
        # The first resume can be observed after the immediate second hold.
        self.statuses[0]['status'].update(obstacle_hold=True,obstacle_stops=2)
        s=copy.deepcopy(self.statuses[0]);s.update(sim=22.)
        s['status'].update(obstacle_hold=False,obstacle_resumes=2,trajectory_reference_stamp=[21,0],accepted_trajectory_id=4)
        s['status']['steering'].update(stamp=21.9,trajectory_id=4);self.statuses.append(s)
        self.metadata.append(dict(received_sim=21.02,metadata=dict(reference_stamp=[21,0],body_goal=[2,2,0],trajectory=dict(traj_id=4))))
        self.commands.append(dict(sim=20.06,source='requested',command=[0.,0.,0.],nav_request_id='actual',nav_index=1))
        self.assertTrue(self.result()['intervals'][0]['passed'])
        self.assertFalse(self.result()['intervals'][1]['passed'])
        self.assertFalse(self.result()['passed'])


if __name__=='__main__':
    unittest.main()
