from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/validation'))
from parking_grid import causal_grid
class ObserverGrid(unittest.TestCase):
    def origin(self):return dict(stamp_ns=1_000_000_000,position=[0,0,0],quaternion_xyzw=[0,0,0,1])
    def test_actual_distinct_headers(self):
        poses=[dict(t=1+n*.1,p=[0,0,0],q=[0,0,0,1]) for n in range(51)]
        rows,errors=causal_grid(poses,1_000_000_000,self.origin())
        self.assertEqual(len(rows),51);self.assertEqual(errors,[])
    def test_reused_low_rate_header_fails(self):
        poses=[dict(t=1+n*.2,p=[0,0,0],q=[0,0,0,1]) for n in range(26)]
        _,errors=causal_grid(poses,1_000_000_000,self.origin())
        self.assertIn('reused_or_reversed_pose_header',errors)
if __name__=='__main__':unittest.main()
