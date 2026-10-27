import copy,json,unittest
from pathlib import Path
from clear_guard_window import ClearGuardWindow
from safety_configuration import parking,clock,settle
from dynamic_scene import configuration,target_at
from diagnostic_observation import strict_formal_enabled
import r85_dynamic_obstacle_verifier as dynamic
import student_lower_loop_ack as ack

ROOT=Path(__file__).resolve().parents[1]
class CurrentRuntimeTests(unittest.TestCase):
    def setUp(self):self.profile=json.loads((ROOT/'config/navigation.json').read_text())
    def test_descriptions_are_not_release_gates(self):
        p=self.profile
        for key in ('control_clock_contract','ordinary_terminal_parking_contract','navigation_settle_statistics_contract'):
            p[key].pop('schema',None);p[key]['description']='current configuration'
        p['ordinary_terminal_parking_contract']['pose10hz'].pop('schema',None)
        clock(p['control_clock_contract']);parking(p['ordinary_terminal_parking_contract']);settle(p['navigation_settle_statistics_contract'])
        p['r85_formal_regression'].pop('schema',None)
        self.assertTrue(strict_formal_enabled(p))
    def test_five_second_and_freshness_limits_remain(self):
        for key,value in [('duration_ns',6_000_000_000),('source_age_ns',400_000_000),('capture_xy_m',.05)]:
            p=copy.deepcopy(self.profile['ordinary_terminal_parking_contract']);p[key]=value
            with self.assertRaises(ValueError):parking(p)
    def test_diagnostic_cannot_gain_formal_pass(self):
        self.profile['diagnostic_observe_only']['enabled']=True
        with self.assertRaises(ValueError):strict_formal_enabled(self.profile)
    def test_actual_clear_needs_ten_observations(self):
        w=ClearGuardWindow()
        for i in range(9):self.assertFalse(w.observe(1_000_000_000+i*140_000_000,False,True))
        self.assertTrue(w.observe(2_260_000_000,False,True))
    def test_cached_clear_cannot_count(self):
        w=ClearGuardWindow()
        for i in range(20):self.assertFalse(w.observe(i*100_000_000,False,False))
        self.assertEqual(len(w.clocks),0)
    def test_new_blockage_and_source_gap_reset_clear(self):
        w=ClearGuardWindow()
        for i in range(9):w.observe(i*140_000_000,False,True)
        self.assertFalse(w.observe(1_260_000_000,True,True));self.assertEqual(len(w.clocks),0)
        w.observe(1_400_000_000,False,True)
        self.assertFalse(w.observe(1_800_000_000,False,True));self.assertEqual(len(w.clocks),1)
    def test_geometry_helpers_import_from_current_project(self):
        modules,locations=dynamic._modules()
        self.assertEqual(set(modules),{'core','audit','stop','payload'})
        for m in modules.values():self.assertTrue(Path(m.__file__).resolve().is_relative_to(ROOT/'src'))
    def test_scene_uses_current_settings(self):
        scene=configuration(self.profile)
        self.assertEqual(target_at(0.,scene),([17.,5.7,1.8],'entering'))
        self.assertEqual(target_at(10.,scene),([14.9,5.7,1.8],'blocking'))
        self.assertEqual(target_at(40.,scene),([17.,5.7,1.8],'clear'))
        self.profile['dynamic_scene']['encounter_route_point_index']=12
        self.assertEqual(configuration(self.profile)['encounter_route_point_index'],12)
    def test_invalid_scene_is_rejected(self):
        self.profile['dynamic_scene']['obstacle_size_m']=[1.,-1.,1.]
        with self.assertRaises(ValueError):configuration(self.profile)
    def current_guard(self):
        route=[[0.,0.,0.],[1.,0.,0.]]
        doc=dict(body_points=route)
        segment=dict(coordinate_chain=[dict(cut=1_000_000_000,reference=doc,reference_source={},edge=dict(boundary_wall=1.))])
        guard=dict(compute_ros_clock_ns=1_200_000_000,control_pose_stamp_ns=1_100_000_000,
            cloud_header_stamp_ns=1_100_000_000,cloud_filtering_body_stamp_ns=1_100_000_000,
            compute_monotonic_wall=1.2,route=copy.deepcopy(route))
        return guard,segment
    def test_current_actual_route_needs_no_audit_route_version(self):
        guard,segment=self.current_guard()
        doc,_=dynamic._coordinate_guard_reference(guard,segment,{}, {})
        self.assertEqual(doc['body_points'],guard['route'])
    def test_wrong_actual_route_still_rejected(self):
        guard,segment=self.current_guard();guard['route'][1][0]=2.
        with self.assertRaises(ValueError):dynamic._coordinate_guard_reference(guard,segment,{}, {})
    def test_old_coordinate_source_still_rejected(self):
        guard,segment=self.current_guard();guard['control_pose_stamp_ns']=999_999_999
        with self.assertRaises(ValueError):dynamic._coordinate_guard_reference(guard,segment,{}, {})
    def test_optional_capture_has_no_source_or_authority_registration(self):
        import tempfile
        from p95_capture import P95SourceFrames
        with tempfile.TemporaryDirectory()as directory:
            cfg=copy.deepcopy(self.profile['source_capture_settings']);cfg['run_id']=Path(directory).name
            cfg['schema']='descriptive_metadata'
            capture=P95SourceFrames(directory,contract=cfg,start_writer=False)
            self.assertEqual(capture.bindings,[])
if __name__=='__main__':unittest.main()
