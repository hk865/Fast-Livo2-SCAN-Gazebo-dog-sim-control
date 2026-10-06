import json
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from sensor_startup_gate import StaticImuWindow

READY={'state':'ready','requested':[0,0,0],'safe':[0,0,0]}

class StartupGateTests(unittest.TestCase):
    def feed(self,gate,start,end,gyro=(0,0,0),acc=(0,0,9.81),quaternion=(0,0,0,1),bridge=READY):
        accepted=None
        for i in range(round(start*100),round(end*100)+1):
            t=i*.01;gate.update_bridge(bridge,t);gate.update(t,t,quaternion,gyro,acc)
            if gate.ready(t,t):accepted=accepted or t
        return accepted

    def test_warm_ten_then_three_continuous_seconds(self):
        g=StaticImuWindow();self.assertIsNone(self.feed(g,0,12.99))
        self.assertEqual(self.feed(g,13,13),13)

    def test_recovery_rotation_restarts_window(self):
        g=StaticImuWindow();self.feed(g,10,12)
        self.feed(g,12.01,12.2,gyro=(.04,0,0))
        self.assertIsNone(self.feed(g,12.21,15.20))
        self.assertEqual(self.feed(g,15.21,15.21),15.21)

    def test_acceleration_spread_blocks_even_zero_mean(self):
        g=StaticImuWindow()
        for i in range(1000,1401):
            t=i*.01;g.update_bridge(READY,t);g.update(t,t,(0,0,0,1),(0,0,0),(0,0,9.81+(1 if i%2 else -1)))
        self.assertFalse(g.ready(14,14));self.assertEqual(g.reason,'acceleration_variation')

    def test_constant_linear_acceleration_not_gravity(self):
        g=StaticImuWindow();self.assertIsNone(self.feed(g,10,14,acc=(.7,0,9.81)))
        self.assertEqual(g.reason,'acceleration_orientation_disagree')

    def test_bridge_recovery_and_commanded_motion_reset(self):
        for bad in [dict(READY,state='hold'),dict(READY,state='failed'),dict(READY,requested=[.01,0,0]),{'state':'ready'}]:
            g=StaticImuWindow();self.feed(g,10,12);self.feed(g,12.01,12.1,bridge=bad)
            self.assertIsNone(self.feed(g,12.11,15.10));self.assertEqual(self.feed(g,15.11,15.11),15.11)

    def test_missing_or_old_imu_restarts_window(self):
        g=StaticImuWindow();self.feed(g,10,12)
        g.update_bridge(READY,12.1);g.update(12.1,12.1,(0,0,0,1),(0,0,0),(0,0,9.81))
        self.assertEqual(g.reason,'imu_time_gap_or_nonincreasing_stamp')
        self.assertIsNone(self.feed(g,12.11,15.10))
        self.assertFalse(g.ready(15.5,15.5))

    def test_missing_orientation_never_passes(self):
        g=StaticImuWindow();self.feed(g,10,12)
        g.update_bridge(READY,12.01);g.update(12.01,12.01,(0,0,0,1),(0,0,0),(0,0,9.81),False)
        self.assertEqual(g.reason,'invalid_imu');self.assertFalse(g.samples)

    def test_quaternion_sign_is_same_pose(self):
        g=StaticImuWindow();self.feed(g,10,11.5)
        self.assertEqual(self.feed(g,11.51,13.1,quaternion=(0,0,0,-1)),13)

    def test_orientation_change_blocks_static_gyro_report(self):
        g=StaticImuWindow();self.feed(g,10,12)
        q=(0,0,math.sin(.02/2),math.cos(.02/2))
        self.assertIsNone(self.feed(g,12.01,15,quaternion=q))

    def test_actual_recorded_still_segments_are_not_starved(self):
        from scipy.spatial.transform import Rotation
        reports={}
        for name in ('baseline','four','loaded'):
            path=ROOT/'simulation/test_results'/f'stability_run9_{name}.json'
            data=json.loads(path.read_text());g=StaticImuWindow();states=iter(data['safety']);state=next(states,None);accepted=None
            for row in data['raw_imu']:
                if row['stamp']>20:break
                while state is not None and state['wall']<=row['wall']:
                    g.update_bridge(state,state['wall']);state=next(states,None)
                q=Rotation.from_euler('xyz',row['rpy']).as_quat()
                g.update(row['stamp'],row['wall'],q,row['gyro'],row['acceleration'])
                if g.ready(row['stamp'],row['wall']):accepted=row['stamp'];break
            self.assertIsNotNone(accepted,name)
            self.assertGreaterEqual(accepted,13)
            reports[name]={'first_pass_stamp':accepted,**g.report()}
        (ROOT/'test_results/startup_gate_recorded_replay.json').write_text(json.dumps(reports,indent=2)+'\n')

if __name__=='__main__':unittest.main()
