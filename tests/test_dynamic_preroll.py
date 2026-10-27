import copy,unittest
from dynamic_stop_event import select_event_preroll,SCHEMA

class DynamicPrerollTests(unittest.TestCase):
    def fixture(self):
        times=[1365000000,1430000000,1485000000,1525000000,1580000000,1615000000,1670000000,1720000000]
        rows=[]
        for sequence,t in enumerate(times,1):
            rows.append(dict(data=dict(request_id='request',waypoint_index=0,sequence=sequence,
                compute_ros_clock_ns=t,cloud_header_stamp_ns=t-50000000,filtered_xyz_float64_sha256='a'*64,
                trajectory_id=2,reference_stamp=[1,0],union_result=dict(blocked=sequence==len(times)),
                obstacle_hold_before=False,obstacle_hold_after=sequence==len(times),
                obstacle_stops_before=0,obstacle_stops_after=int(sequence==len(times)),
                previous_actual_guard_clock_ns=times[sequence-2]if sequence>1 else 1300000000)))
        last=rows[-1]['data']
        event=dict(schema=SCHEMA,request_id='request',waypoint_index=0,sequence=len(rows),clock_ns=times[-1],
            cloud_source_ns=last['cloud_header_stamp_ns'],cloud_sha256='a'*64,trajectory_id=2,reference_stamp=[1,0],stop_count=1)
        held=[dict(data=dict(dynamic_stop_event=event,request_id='request',waypoint_index=0,obstacle_stops=1),received_sim_ns=times[-1]+10000000)]
        return held,rows
    def test_real_predecessor_covers_sampling_boundary(self):
        held,rows=self.fixture()
        selected,trigger,event=select_event_preroll('request',held,rows,lambda r:None)
        self.assertEqual(selected[0]['data']['compute_ros_clock_ns'],1365000000)
        self.assertEqual(selected[-1]['data']['compute_ros_clock_ns']-selected[0]['data']['compute_ros_clock_ns'],355000000)
    def test_missing_causal_prefix_does_not_become_coverage(self):
        held,rows=self.fixture()
        with self.assertRaises(ValueError):select_event_preroll('request',held,rows[1:],lambda r:None)
    def test_missing_guard_cannot_be_bridged(self):
        held,rows=self.fixture();del rows[3]
        with self.assertRaises(ValueError):select_event_preroll('request',held,rows,lambda r:None)
if __name__=='__main__':unittest.main()
