"""Small reader-only checks; no runtime logs, ROS or simulator are opened."""
import copy
import math
from pathlib import Path
import unittest

from evaluate_full46_v23 import heading_reference,load_adapter,OLD,OLD_SHA,sha

class FakeAudit:
    def __init__(self,rows):self.values=rows
    def snapshot(self,*args):return Path('/unused/source')
    def rows(self,*args):return iter(self.values)
    def ck(self,value,**details):return dict(passed=value,**details)

def record(mode='drive'):
    # The raw nearest segment points x, while the finite-arc reference points y.
    spatial=dict(tangent_xy=[0.,1.],normal_xy=[-1.,0.],cross_ref_m=.02,
        path_id='p',path_sha256='a'*64,heading_rad=math.pi/2,heading_source='finite_horizontal_arc')
    cascade=dict(controller_updated=True,mode=mode,spatial_reference=spatial,
        local_horizontal_tangent=[1.,0.],control_horizontal_tangent=[0.,1.],
        control_horizontal_normal=[-1.,0.],control_error_cross_m=.02,path_id='p',path_sha256='a'*64,
        reference_yaw_rad=math.pi/2,endpoint_is_fixed_goal=False,remaining_horizontal_arc_m=1.,
        fixed_goal=dict(heading_rad=.3))
    if mode=='turn':
        cascade.update(reference_yaw_rad=.8,reference_yaw_source='external_heading_gate_locked_reference',
            external_turn_heading_rad=.8,ungated_reference_yaw_rad=math.pi/2)
    if mode in ('capture','active_hold'):cascade['reference_yaw_rad']=.3
    return dict(sequence=1,cascade=cascade,heading_gate_reference=dict(phase='align'if mode=='turn'else'drive',locked_heading=.8))

class AdapterTests(unittest.TestCase):
    def test_import_uses_V20_and_keeps_frozen_old_bytes(self):
        audit=load_adapter()
        self.assertEqual(audit.V19.name,'corridor_tracking_v23_curvature_full46')
        self.assertEqual(sha(OLD),OLD_SHA)

    def check_rows(self,rows):return heading_reference(FakeAudit(rows),Path('/unused/run'),{})

    def test_finite_arc_lock_and_capture_join(self):
        result=self.check_rows([record(mode)for mode in ('drive','turn','capture','active_hold')])
        self.assertTrue(result['passed'])
        self.assertEqual(result['actual_align_turn_updates'],1)

    def test_wrong_raw_tangent_effective_heading_is_rejected(self):
        rows=[record(),record('turn')];rows[0]['cascade']['reference_yaw_rad']=0.
        self.assertFalse(self.check_rows(rows)['passed'])

    def test_mismatched_normal_or_lock_or_nonfinite_is_rejected(self):
        for field,value in (('control_horizontal_normal',[0.,1.]),('external_turn_heading_rad',.9),('reference_yaw_rad',float('nan'))):
            row=record('turn');row['cascade'][field]=value
            self.assertFalse(self.check_rows([row])['passed'])

    def test_missing_turn_coverage_stays_unverified(self):
        self.assertIsNone(self.check_rows([record()])['passed'])

if __name__=='__main__':unittest.main()
