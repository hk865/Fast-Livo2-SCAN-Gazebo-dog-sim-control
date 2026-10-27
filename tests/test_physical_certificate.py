from pathlib import Path
import copy,sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from r85_physical_path_gate import physical_allowed,digest
class PhysicalCertificateTests(unittest.TestCase):
    def setUp(self):
        self.payload=dict(frame_id='camera_init',order=3,pos_pts=[[0.,0.,.32],[.2,0.,.32],[.4,0.,.32],[.6,0.,.32]],knots=[0.,0.,0.,0.,1.,1.,1.,1.])
        self.context=dict(T_navigation_world=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-.3],[0.,0.,0.,1.]])
        self.proof=dict(schema='r85_continuous_scan_physical_certificate/v1',allowed=True,status='CERTIFIED_STATIC_SPACE_ONLY',payload_sha256=digest(self.payload),T_navigation_world=self.context['T_navigation_world'],budgets=dict(max_patches=4096,max_depth=18,wall_budget_s=2.))
    def test_certified_geometry_has_no_code_or_world_file_pin(self):self.assertTrue(physical_allowed(self.proof,self.payload,self.context))
    def test_failed_geometry_remains_rejected(self):
        self.proof['allowed']=False;self.assertFalse(physical_allowed(self.proof,self.payload,self.context))
    def test_unknown_result_is_not_a_certificate(self):
        self.proof['status']='UNKNOWN_REJECTED';self.assertFalse(physical_allowed(self.proof,self.payload,self.context))
    def test_another_curve_cannot_reuse_certificate(self):
        self.payload['pos_pts'][2][0]=9.;self.assertFalse(physical_allowed(self.proof,self.payload,self.context))
    def test_changed_transform_is_rejected(self):
        self.proof['T_navigation_world']=[];self.assertFalse(physical_allowed(self.proof,self.payload,self.context))
    def test_geometry_budget_is_not_relaxed(self):
        self.proof['budgets']['max_depth']=19;self.assertFalse(physical_allowed(self.proof,self.payload,self.context))
if __name__=='__main__':unittest.main()
