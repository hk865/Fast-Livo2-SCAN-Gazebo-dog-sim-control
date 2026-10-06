import importlib.util
import math
from pathlib import Path
import unittest

SPEC=importlib.util.spec_from_file_location('plot_ab',Path(__file__).with_name('plot_curvature_ab.py'))
m=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(m)

class PlotTests(unittest.TestCase):
    def test_missing_is_null_not_zero(self):
        result=m.metric([None,float('nan'),float('inf')])
        self.assertEqual(result['n'],0)
        self.assertIsNone(result['rms']);self.assertIsNone(result['peak_abs'])

    def test_only_fresh_drive_statistics(self):
        rows=[dict(fresh_math=True,mode='drive',phase='drive',inner_heading_error_rad=.1),
              dict(fresh_math=False,mode='drive',phase='drive',inner_heading_error_rad=10),
              dict(fresh_math=True,mode='turn',phase='align',inner_heading_error_rad=20)]
        result=m.summarize_rows(rows)
        self.assertEqual(result['fresh_math_drive_rows'],1)
        self.assertEqual(result['inner_heading_error_rad']['mean'],.1)
        self.assertEqual(result['curvature_FF_raw_radps']['n'],0)

    def test_body_z_error_is_not_euler_yaw(self):
        row=dict(fresh_math=True,mode='drive',phase='drive',reference_COM_vx_vy_wz=[.2,0,.3],
                 measured_SLAM_COM_velocity=[.1,0,99],measured_body_omega=[1,2,.1],
                 measured_Euler_yawrate=15)
        result=m.summarize_rows([row])
        self.assertAlmostEqual(result['raw_IMU_body_z_rate_error_radps']['mean'],.2)
        self.assertAlmostEqual(result['planar_COM_speed_error_mps']['mean'],.1)

    def test_missing_drive_and_long_gaps_break_lines(self):
        rows=[dict(clock_ns=1_000_000_000,fresh_math=True,mode='drive',phase='drive',y=1),
              dict(clock_ns=1_100_000_000,fresh_math=False,mode='drive',phase='drive',y=999),
              dict(clock_ns=5_000_000_000,fresh_math=True,mode='drive',phase='drive',y=2)]
        x,y=m.points(rows,lambda r:r['y'],mask=m.drive)
        self.assertEqual(len(x),4)
        self.assertTrue(math.isnan(y[1]));self.assertTrue(math.isnan(x[2]));self.assertEqual(y[-1],2)

if __name__=='__main__':unittest.main()
