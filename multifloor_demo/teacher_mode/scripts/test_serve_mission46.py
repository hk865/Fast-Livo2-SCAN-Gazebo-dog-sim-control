"""Small read-only selection/HTTP/tail tests; no ROS, sockets or simulation."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('teacher_dashboard_mission46_test', HERE/'serve.py')
serve = importlib.util.module_from_spec(spec)
spec.loader.exec_module(serve)


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.old_root = serve.ROOT
        serve.ROOT = Path(self.temp.name)
        self.runs = serve.ROOT/'runs'
        self.run = self.runs/'selected_mission'
        self.run.mkdir(parents=True)
        self.rid = self.run.name+':exploration:1'
        self.write('navigation_scope.json', {'profile': {'mission46_required': True, 'pipeline': {
            'schema': 'pipeline_v19_contract/v1', 'mode': 'staged', 'image_copy_opt': 0}}})
        goals=[dict(goal_id='exploration:'+str(i),center=[0,0,0],arrival=dict(dwell_sim_s=.4))for i in range(1,19)]
        goal_hash=hashlib.sha256(json.dumps(goals,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
        nav=dict(request_id=self.rid,frame_id='camera_init',mode='teacher',controller_kind='teacher',
            navigation_ground_truth_used=False,goals_definitions=goals,goals_definition_sha256=goal_hash,
            region_arrivals=[dict(request_id=self.rid,goal_id='exploration:'+str(i),goals_definition_sha256=goal_hash,
                stamp_ns=500000000,start_stamp_ns=0,dwell_ns=500000000,max_observation_gap_ns=50000000,
                region_inside=True,control_region_inside=True,protected=False,reason='arrived',raw_position=[0,0,0])for i in range(1,18)])
        self.write('navigation_request.json',dict(schema_version=2,request_id=self.rid,frame_id='camera_init',goals=goals))
        self.write('navigation_status.json',nav)
        self.state = {'schema': 'teacher_original_mission46_state/v1', 'run_id': self.run.name,
            'expected_region_count': 46, 'navigation_ground_truth_used': False,
            'stage': 'exploring', 'current_request': self.rid, 'completed_stages': [],
            'current_goals_sha256':goal_hash,'functional_sequence_completed': False,'navigation':nav}
        self.write('mission46_status.json', self.state)

    def tearDown(self):
        serve.ROOT = self.old_root
        self.temp.cleanup()

    def write(self, name, value):
        p = self.run/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(value)+'\n')
        return p

    def test_partial_phase_is_17_of_18_not_full_mission(self):
        view = serve.mission46_view(self.run)
        self.assertEqual(view['recorded_arrivals'], 17)
        self.assertEqual([p['expected'] for p in view['phases']], [18,14,14])
        self.assertFalse(view['functional_sequence_completed'])
        self.assertEqual(view['independent_status'], 'unverified')
        self.state.update(stage='failed',current_request=None)
        self.write('mission46_status.json',self.state)
        self.assertEqual(serve.mission46_view(self.run)['recorded_arrivals'],17)

    def test_completed_runtime_and_old_pass_never_certify_46(self):
        self.state.update(stage='completed', completed_stages=['exploring','returning','navigating'],
                          functional_sequence_completed=True)
        self.write('mission46_status.json', self.state)
        self.write('summary_closed_loop_cascade_independent.json', {
            'schema': 'independent_actual_SLAM_SCAN_cascade_navigation/v1',
            'run': str(self.run), 'scope': {'navigation_ground_truth_used': False},
            'status': 'passed', 'checks': {'old_check': {'status': 'passed','passed': True}}})
        self.write('mission46_final_evaluation.json', {'run': '/foreign/V18', 'status': 'passed'})
        view = serve.mission46_view(self.run)
        self.assertTrue(view['functional_sequence_completed'])
        self.assertEqual(view['recorded_arrivals'], 46)
        self.assertFalse(view['navigation_is_certified'])
        self.assertEqual(view['final_evaluations'][0]['status'], 'unverified')
        legacy = serve.closed_loop_receipt_view(self.run)
        self.assertEqual(legacy['status'], 'unverified')
        self.assertNotIn('common', legacy)

    def final_receipt(self):
        for name in serve.MISSION46_FINAL_SOURCES:
            if not (self.run/name).exists():
                self.write(name, {'source': name})
        checks={name:dict(status='unverified',passed=None)for name in serve.MISSION46_FINAL_CHECKS}
        checks['whole_mission_terminal']=dict(status='failed',passed=False,reason='actual region timeout')
        return dict(schema=serve.MISSION46_FINAL_SCHEMA,run=str(self.run.resolve()),run_id=self.run.name,
            simulation_only=True,navigation_ground_truth_used=False,real_robot_verified=False,
            status='failed',passed=False,checks=checks,
            source_bindings={name:hashlib.sha256((self.run/name).read_bytes()).hexdigest()
                for name in serve.MISSION46_FINAL_SOURCES})

    def test_final_failure_requires_actual_failed_check_and_every_source(self):
        raw=self.final_receipt();path=self.write('summary_mission46_independent.json',raw)
        view=serve.mission46_view(self.run)
        final=view['selected_independent_failure']
        self.assertEqual(view['independent_status'],'failed')
        self.assertTrue(final['failure_evidence_verified'])
        self.assertFalse(final['formal_acceptance']);self.assertFalse(final['formal_pass_allowed'])
        self.assertFalse(view['navigation_is_certified'])
        self.assertEqual(final['sha256'],hashlib.sha256(path.read_bytes()).hexdigest())
        self.assertEqual(serve.closed_loop_receipt_view(self.run)['status'],'failed')
        (self.run/'navigation_anchor.json').write_text('{"changed":true}\n')
        self.assertEqual(serve.mission46_view(self.run)['independent_status'],'unverified')

    def test_final_missing_foreign_unsupported_or_empty_bindings_are_unverified(self):
        raw=self.final_receipt()
        candidates=[]
        for key,value in [('run','/foreign'),('run_id','foreign'),('schema','unsupported'),
                          ('source_bindings',{}),('checks',{}),('simulation_only',None)]:
            row=dict(raw);row[key]=value;candidates.append(row)
        row=dict(raw);row['source_bindings']=dict(raw['source_bindings']);row['source_bindings'].pop('worker_result.json');candidates.append(row)
        for row in candidates:
            with self.subTest(key=row):
                self.write('summary_mission46_independent.json',row)
                final=serve.mission46_view(self.run)['final_evaluations'][0]
                self.assertEqual(final['status'],'unverified')
                self.assertFalse(final['failure_evidence_verified'])

    def test_final_failure_with_no_failed_check_or_inconsistent_check_is_unverified(self):
        for change in ('no_failure','inconsistent','missing_check'):
            row=self.final_receipt()
            if change=='no_failure':row['checks']['whole_mission_terminal']=dict(status='passed',passed=True)
            if change=='inconsistent':row['checks']['whole_mission_terminal']=dict(status='failed',passed=True)
            if change=='missing_check':row['checks'].pop('full_200Hz_native_actuator_trace_replay')
            self.write('summary_mission46_independent.json',row)
            self.assertEqual(serve.mission46_view(self.run)['independent_status'],'unverified')

    def test_future_pass_and_completed_runtime_never_upgrade_final_pass(self):
        self.state.update(stage='completed',functional_sequence_completed=True)
        self.write('mission46_status.json',self.state)
        row=self.final_receipt();row.update(status='passed',passed=True)
        row['checks']={name:dict(status='passed',passed=True)for name in serve.MISSION46_FINAL_CHECKS}
        self.write('summary_mission46_independent.json',row)
        view=serve.mission46_view(self.run)
        self.assertTrue(view['functional_sequence_completed'])
        self.assertEqual(view['independent_status'],'unverified')
        self.assertFalse(view['navigation_is_certified'])
        self.assertEqual(serve.closed_loop_receipt_view(self.run)['status'],'unverified')

    def test_final_foreign_extra_binding_cannot_escape_selected_run(self):
        row=self.final_receipt();foreign=serve.ROOT/'foreign_final.json';foreign.write_text('{}\n')
        row['source_bindings']['../foreign_final.json']=hashlib.sha256(foreign.read_bytes()).hexdigest()
        self.write('summary_mission46_independent.json',row)
        self.assertEqual(serve.mission46_view(self.run)['independent_status'],'unverified')

    def test_final_hash_cache_reads_static_bound_bytes_only_once(self):
        row=self.final_receipt();self.write('summary_mission46_independent.json',row)
        serve.SOURCE_DIGEST_CACHE.clear()
        binary_reads=[];original=Path.open
        def opened(path,*args,**kwargs):
            if (args and args[0]=='rb')or kwargs.get('mode')=='rb':binary_reads.append(str(path))
            return original(path,*args,**kwargs)
        with mock.patch.object(Path,'open',opened):
            self.assertEqual(serve.mission46_view(self.run)['independent_status'],'failed')
            count=len(binary_reads)
            self.assertGreaterEqual(count,len(row['source_bindings'])+1)
            self.assertEqual(serve.mission46_view(self.run)['independent_status'],'failed')
            self.assertEqual(len(binary_reads),count)

    def test_foreign_or_missing_source_state_is_not_selected(self):
        for key,value in [('run_id','foreign_run'),('navigation_ground_truth_used',None)]:
            candidate = dict(self.state);candidate[key] = value
            self.write('mission46_status.json', candidate)
            view = serve.mission46_view(self.run)
            self.assertFalse(view['state_valid_for_selected_run'])
            self.assertEqual(view['recorded_arrivals'], 0)

    def test_foreign_arrivals_and_duplicate_ids_are_not_counted(self):
        arrivals = self.state['navigation']['region_arrivals']
        arrivals.extend([dict(arrivals[0]),dict(request_id='foreign',goal_id='exploration:18')])
        self.write('mission46_status.json', self.state)
        self.write('navigation_status.json',self.state['navigation'])
        self.assertEqual(serve.mission46_view(self.run)['recorded_arrivals'],17)

    def test_control_requires_run_schema_source_and_boolean_hold(self):
        control = dict(schema='teacher_mission46_control/v1',run_id=self.run.name,
            navigation_ground_truth_used=False,hold=True,sim_ns=1000,reason='test')
        self.write('mission46_control.json',control)
        self.assertTrue(serve.mission46_view(self.run)['control_valid_for_selected_run'])
        control['hold']=1;self.write('mission46_control.json',control)
        self.assertFalse(serve.mission46_view(self.run)['control_valid_for_selected_run'])

    def test_runtime_receipt_hash_is_bound_but_not_formal_acceptance(self):
        name,schema,checks=serve.MISSION46_RUNTIME_RECEIPTS['initialization']
        evidence=self.write('initialization_evidence.json',{'actual':True})
        receipt=dict(schema=schema,run_id=self.run.name,navigation_ground_truth_used=False,
            passed=True,binding_verified=True,checks={x:dict(status='passed',passed=True)for x in checks},
            source_evidence_file=str(evidence),source_evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
        self.write(name,receipt)
        view=serve.mission46_view(self.run)['runtime_receipts']['initialization']
        self.assertTrue(view['source_binding_valid']);self.assertFalse(view['formal_mission_acceptance'])
        evidence.write_text('{"tampered":true}\n')
        self.assertFalse(serve.mission46_view(self.run)['runtime_receipts']['initialization']['source_binding_valid'])

    def test_foreign_evidence_cannot_supply_runtime_pass(self):
        name,schema,checks=serve.MISSION46_RUNTIME_RECEIPTS['initialization']
        evidence=serve.ROOT/'foreign_evidence.json';evidence.write_text('{}\n')
        self.write(name,dict(schema=schema,run_id=self.run.name,navigation_ground_truth_used=False,
            passed=True,binding_verified=True,checks={x:dict(status='passed',passed=True)for x in checks},
            source_evidence_file=str(evidence),source_evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest()))
        self.assertFalse(serve.mission46_view(self.run)['runtime_receipts']['initialization']['source_binding_valid'])

    def test_http_logic_selects_only_named_run_without_server(self):
        handler=object.__new__(serve.handler_for(serve.Dashboard(self.runs)))
        handler.path='/api/run?run='+self.run.name
        captured=[]
        handler.send=lambda data,content_type=None,status=200,headers=None:captured.append((status,data))
        handler.do_GET()
        code,payload=captured[0];data=json.loads(payload)
        self.assertEqual(code,200);self.assertEqual(data['run_id'],self.run.name)
        self.assertEqual(data['mission46']['recorded_arrivals'],17)
        self.assertEqual(data['closed_loop_result']['status'],'unverified')

    def test_v19_atomic_state_is_latest_exact_sample(self):
        self.write('telemetry.jsonl',dict(sim_time=1.,command=[0,0,0],body_lin_vel=[0,0,0],body_ang_vel=[0,0,0]))
        self.write('state.json',dict(sim_time=20.,command=[.1,0,0],body_lin_vel=[.08,0,0],body_ang_vel=[0,0,0]))
        native=serve.Dashboard(self.runs).snapshot(self.run.name)['telemetry']
        self.assertEqual(native['latest']['t'],20.)
        self.assertEqual(native['archive_latest_t'],1.)
        self.assertIn('atomic state.json',native['latest_source'])

    def test_recent_window_skips_only_display_and_completes_partial_line(self):
        p=self.run/'telemetry.jsonl'
        rows=[json.dumps(dict(sim_time=float(i),command=[0,0,0],position=[i,0,0]))+'\n'for i in range(20)]
        p.write_text(''.join(rows)+'{"sim_time":20')
        original=hashlib.sha256(p.read_bytes()).hexdigest()
        reader=serve.Telemetry();reader.CHUNK_BYTES=250
        view=reader.read(p,latest_window=True)
        self.assertEqual(view['latest']['t'],19.)
        self.assertGreater(view['display_skipped_bytes'],0)
        self.assertEqual(view['invalid_records'],0)
        self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),original)
        with p.open('a')as f:f.write(',"command":[0,0,0]}\n')
        view=reader.read(p,latest_window=True)
        self.assertEqual(view['latest']['t'],20.)
        self.assertEqual(view['invalid_records'],0)

    def test_old_archive_mode_retains_backfill_behavior(self):
        p=self.run/'telemetry.jsonl'
        p.write_text(''.join(json.dumps(dict(sim_time=float(i)))+'\n'for i in range(20)))
        reader=serve.Telemetry();reader.CHUNK_BYTES=100
        view=reader.read(p)
        self.assertTrue(view['loading']);self.assertEqual(view['display_skipped_bytes'],0)
        self.assertLess(view['latest']['t'],19.)

    def test_page_addition_has_no_ros_and_valid_javascript(self):
        web=serve.ROOT/'web';web.mkdir();(web/'index.html').write_text('<html><body>original RGB/SCAN markup</body></html>')
        page=serve.dashboard_page().decode()
        self.assertIn('original RGB/SCAN markup',page)
        self.assertIn('mission46Panel',page)
        self.assertIn('stage completed',serve.MISSION46_PAGE_SCRIPT.replace('阶段 completed','stage completed'))
        node=shutil.which('node')
        if node:
            p=subprocess.run([node,'--check'],input=serve.MISSION46_PAGE_SCRIPT.removeprefix('\n<script>\n').removesuffix('\n</script>\n'),text=True,capture_output=True)
            self.assertEqual(p.returncode,0,p.stderr)


if __name__=='__main__':unittest.main()
