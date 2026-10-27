from pathlib import Path
import copy,sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/validation'))
from parking_status import check_window
class ParkingStatusWindow(unittest.TestCase):
    def rows(self):
        d=dict(request_id='run:goal',state='succeeded',command=[0.,0.,0.],cascade_parking=dict(mode='ordinary_zero_hold',protection_active=False,failure_latched=None,ordinary_parking={}),execution_bridge_safety=dict(state='ready'),known_scene_localization=dict(control_allowed=True,correction_stale=False,correction_transition=False,generation=1),known_scene_runtime_status=dict(initialized=True,correction_transition=False,generation=1),known_scene_scan_epoch=dict(ready=True,generation=1))
        return[dict(t=1.025+i*.1,data=copy.deepcopy(d))for i in range(50)]
    def check(self,rows):return check_window(rows,1_000_000_000,6_000_000_000,'run:goal')
    def test_preT0_deceleration_not_carried_into_hold(self):
        rows=self.rows();prior=copy.deepcopy(rows[0]);prior['t']=.87;prior['data']['command']=[.01,0.,0.];prior['data']['cascade_parking']['mode']='controlled_deceleration'
        self.assertTrue(self.check([prior,*rows])['passed'])
    def test_first_status_more_than_300ms_late_fails(self):self.assertFalse(self.check(self.rows()[3:])['passed'])
    def test_middle_gap_more_than_300ms_fails(self):self.assertFalse(self.check(self.rows()[:10]+self.rows()[14:])['passed'])
    def test_nonzero_inside_fixed_window_fails(self):
        rows=self.rows();rows[10]['data']['command']=[.01,0.,0.];self.assertFalse(self.check(rows)['passed'])
    def test_missing_or_changed_coordinate_source_fails(self):
        rows=self.rows();rows[10]['data']['known_scene_scan_epoch']['generation']=2;self.assertFalse(self.check(rows)['passed'])
if __name__=='__main__':unittest.main()
