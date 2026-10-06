"""Finite V20 wiring/route/gate checks. No ROS, GPU, policy or simulator."""
from pathlib import Path
import ast
import copy
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import corridor_preflight
import prefix_contract
import route
import scan_workspace
import slam_workspace
from mission46_profile import validate_profile as validate_full_profile
from mission.route_regions import transformed_route_goals


class WiringChecks(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((HERE/'profiles/corridor_staged_original46_prefix9.json').read_text())

    def test_inherited_SLAM_actual_source_build_binary_identity(self):
        d = slam_workspace.verify_slam_workspace()
        self.assertFalse(d['CPP_changed']);self.assertFalse(d['build_copied'])
        self.assertFalse(d['runtime_acceptance_inherited'])
        self.assertEqual(slam_workspace.SLAM_WORKSPACE.resolve(), (HERE.parent/'pipeline_v19/slam_ws').resolve())

    def test_prefix_exact_original_transformed_regions_axes_dwell_timeouts(self):
        for origin, yaw in (([0.,0.,.3],0.),([.017,-.033,.321],.42),([2.4,-1.2,.29],-.87)):
            anchor=dict(source='/demo/slam/body_odom',ground_truth_navigation_used=False,
                origin=origin,scene_axis_registration=dict(heading_receipt=dict(yaw_camera_init_from_world=yaw)))
            route.bind_registered_route(anchor,self.profile)
            got=route.build_request('run',anchor,self.profile,{'unused_compatibility_arrival':True})
            expected=[g.definition()for g in transformed_route_goals(self.profile['original_scenario'],
                'exploration',origin,anchor['scene_axis_registration']['heading_receipt'])[:9]]
            self.assertEqual(got['goals'],expected)
            self.assertEqual(len(got['goals']),9)
            self.assertTrue(all(g['timeout_sim_s']==90. and g['arrival']['dwell_sim_s']==.4 for g in got['goals']))
            self.assertEqual(got['goals'][-1]['goal_id'],'exploration:8')
        tree=ast.parse((HERE/'pid_scope.py').read_text())
        assignments=[n for n in ast.walk(tree)if isinstance(n,ast.Assign)and
            any(isinstance(t,ast.Name)and t.id=='scenario'for t in n.targets)]
        self.assertEqual(len(assignments),1)
        self.assertIn("profile.get('original46_prefix_regions') is not None",ast.unparse(assignments[0].value))

    def test_prefix_rejects_geometry_source_or_cutoff_success_changes(self):
        for key,value in (('spawn',[0,0,.5,0]),('original46_prefix_regions',8),('duration_s',1500),
                          ('expected_region_count',46),('navigation_ground_truth_used',True),('pose_cloud_timeout_s',.6)):
            p=copy.deepcopy(self.profile);p[key]=value
            with self.assertRaises(ValueError):prefix_contract.validate_prefix(p)
        p=copy.deepcopy(self.profile);p['route_world_points'][8]['xyz'][0]+=.01
        with self.assertRaises(ValueError):prefix_contract.validate_prefix(p)
        p=copy.deepcopy(self.profile);p['bounded_prefix_contract']['clock_cutoff_is_success']=True
        with self.assertRaises(ValueError):prefix_contract.validate_prefix(p)

    def test_full_original46_same_geometry_spawn_region_definitions(self):
        p=json.loads((HERE/'profiles/pipeline_staged_original46.json').read_text())
        validate_full_profile(p)
        old=json.loads((HERE.parent/'pipeline_v19/profiles/pipeline_staged_original46.json').read_text())
        self.assertEqual(p['original_scenario'],old['original_scenario'])
        self.assertEqual(p['spawn'],old['spawn'])
        self.assertEqual(p['teacher_transition'],old['teacher_transition'])
        self.assertEqual(p['control_clock_contract'],old['control_clock_contract'])
        self.assertEqual(p['cascade']['command_limits'],old['cascade']['command_limits'])

    def test_all_candidate_profiles_keep_protected_rates_and_CPU(self):
        for file in (HERE/'profiles').glob('*.json'):
            p=json.loads(file.read_text())
            self.assertEqual(p['controller_selector'],'corridor_tracking_v20')
            self.assertEqual(p['pose_cloud_timeout_s'],.3)
            self.assertIs(p['navigation_ground_truth_used'],False)
            self.assertEqual(p['lio_jacobian_parallelism']['threads'],4)
            self.assertEqual(p['vio_patch_parallel']['threads'],1)
            self.assertEqual(p['software_multicore']['teacher_cpu_threads'],1)
            for path in p.get('mission46_required_source_files',[]):
                if '/navigation/' in path and '/teacher_mode/' in path:self.assertIn('/corridor_tracking_v20/',path)

    def test_worker_dual_TTL_and_ordered_context_drain_source_unchanged(self):
        for name in ('worker.py','pipeline_lifecycle.py','clock_hold.py','publication_ledger.py',
                     'sensor_gate.py','bridge.py'):
            self.assertEqual((HERE/name).read_bytes(),(HERE.parent/'pipeline_v19'/name).read_bytes())

    def test_baseline_SCAN_contract_is_fixed_and_unknown_selection_rejected(self):
        workspace,_,d=scan_workspace.scan_contract(self.profile)
        self.assertEqual(workspace,(HERE.parents[2]/'navigation/ros2_ws').resolve())
        self.assertTrue(d['actual_loaded_binary_witness_required'])
        with self.assertRaises(RuntimeError):scan_workspace.scan_contract({'scan_workspace_selector':'/tmp/arbitrary'})

    def test_missing_new_gate_cannot_inherit_existing_V19_pass(self):
        with tempfile.TemporaryDirectory()as t:
            Path(t,'PIPELINE_V19_PREFLIGHT.json').write_bytes((HERE/'HISTORICAL_INHERITED_V19_PREFLIGHT.json').read_bytes())
            with self.assertRaisesRegex(RuntimeError,'historical V19 PASS'):
                corridor_preflight.verify_preflight(t,self.profile)

    def test_copied_V19_gate_wrong_identity_is_rejected(self):
        with tempfile.TemporaryDirectory()as t:
            Path(t,'CORRIDOR_V20_PREFLIGHT.json').write_bytes((HERE/'HISTORICAL_INHERITED_V19_PREFLIGHT.json').read_bytes())
            with self.assertRaises(RuntimeError):corridor_preflight.verify_preflight(t,self.profile)

    def test_empty_bindings_cannot_authorize_a_claimed_new_gate(self):
        with tempfile.TemporaryDirectory()as t:
            claimed=dict(schema='corridor_tracking_v20_preflight/v1',status='PASS_LIMITED_NEW_EXPERIMENT',
                candidate_root=str(Path(t).resolve()),allowed=True,actual_navigation_verified=False,
                historical_pass_inherited=False,checks={k:True for k in corridor_preflight.REQUIRED},
                control_review={'path':str(Path(t)/'review.json'),'sha256':'0'*64},
                finite_receipts=[{'path':str(Path(t)/'finite.json'),'sha256':'0'*64}],verified_inputs_sha256={})
            Path(t,'CORRIDOR_V20_PREFLIGHT.json').write_text(json.dumps(claimed))
            with self.assertRaisesRegex(RuntimeError,'mandatory candidate'):
                corridor_preflight.verify_preflight(t,self.profile)

    def test_optimized_python_cannot_disable_new_gate_validation(self):
        p=subprocess.run([sys.executable,'-O','-B','-c','import corridor_preflight'],cwd=HERE,
            capture_output=True,text=True)
        self.assertNotEqual(p.returncode,0)
        self.assertIn('forbids optimized Python',p.stderr)

    def test_actual_binary_witnesses_execute_before_owned_physics_start(self):
        tree=ast.parse((HERE/'run.py').read_text())
        execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='execute')
        text=ast.unparse(execute)
        self.assertLess(text.index('verify_loaded_slam('),text.index("start('gazebo'"))
        self.assertLess(text.index('verify_loaded_scan('),text.index("start('gazebo'"))
        self.assertIn('actual_executable_sha256',(HERE/'run.py').read_text())


if __name__=='__main__':unittest.main()
