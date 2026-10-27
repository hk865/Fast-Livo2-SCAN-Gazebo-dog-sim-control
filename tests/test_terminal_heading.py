from project_config import loads as asset_loads
from pathlib import Path
import copy,json,math,sys,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from project_config import read
from terminal_heading_contract import derive

class TerminalHeadingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.typed=asset_loads((ROOT/'assets/geometry/route.json').read_text())
        self.typed['source_route_file']='/intentionally-unavailable/historical/raw.json'
        self.typed['source_route_sha256']='obsolete-history-is-not-a-runtime-input'
        self.profile=read();self.profile['r85_task'].pop('typed_route_source_sha256',None)
    def document(self):
        p=Path(self.temp.name)/'route.json';p.write_text(json.dumps(self.typed))
        return dict(schema='r85_pct_scan_global_reference/v1',source_route_file=str(p),frame_id='camera_init',reference_scope='full_route',navigation_ground_truth_used=False,T_navigation_world=self.typed['T_navigation_world'],physical_sources=self.typed['physical_sources'],ground_points=self.typed['ground_points'],body_points=self.typed['body_points'],physical_body_ground_offset_m=.32)
    def test_heading_without_historical_raw_source_or_profile_pin(self):
        heading,proof=derive(self.profile,self.document())
        self.assertAlmostEqual(abs(heading),math.pi)
        self.assertEqual(proof['direction_window_arc_m'],.8)
        self.assertNotIn('raw_source_file',proof)
    def test_wrong_current_frame_is_rejected(self):
        d=self.document();d['frame_id']='world'
        with self.assertRaises(ValueError):derive(self.profile,d)
    def test_wrong_transform_is_rejected(self):
        d=self.document();d['T_navigation_world']=[]
        with self.assertRaises(ValueError):derive(self.profile,d)
    def test_body_ground_geometry_cannot_change(self):
        self.typed['body_points'][50][2]+=.05
        with self.assertRaises(ValueError):derive(self.profile,self.document())
    def test_connector_endpoint_must_still_match(self):
        self.typed['endpoint_support_connectors'][0]['planned_body_nav_xyz']=[999.,0.,0.]
        with self.assertRaises(ValueError):derive(self.profile,self.document())

if __name__=='__main__':unittest.main()
