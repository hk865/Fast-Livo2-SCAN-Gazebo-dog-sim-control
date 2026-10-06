"""Synthetic observer/lifecycle tests only: no actual-run reads."""
import copy
import gzip
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import audit_curvature_prefix9 as w

LEGACY,AUDIT=w.load()

def write(path,value):path.write_text(json.dumps(value)+'\n')
def row(t=10,phase='drive',mode='drive',updated=True):
    stamp=round(t*1e9)
    return dict(control_stamp_ns=stamp,waypoint_index=8,request_id='r',trajectory_id=1,
        control_pose=[1.,0.,.3],goal=[2.,0.,.3],command_after_slew=[.1,0.,.2],
        feedback=dict(stamp_ns=stamp,origin_velocity_body=[.08,.01,0.]),
        heading_gate_reference=dict(phase=phase,steering=dict(heading_error=.25,error=.99)),
        cascade=dict(mode=mode,controller_updated=updated,feedback_pose_stamp_ns=stamp,path_id='path',error_yaw_rad=.1,
            spatial_reference=dict(curvature_per_m=1.,curvature_rate_per_m2=.3,curvature_valid=True,heading_source='finite_horizontal_arc'),
            yaw_PD=dict(curvature_FF=.12,P=.13,D=.01),curvature_speed_supervisor=dict(enabled=True,requested_speed_mps=.2,limited_speed_mps=.1,reference_budget_feasible=True),
            integral_dt_s=.03,header_dt_s=.03,command_saturated=[False,False,True]))
def ref(n=1):return dict(file='/fixture.jsonl',line=n,offset=n*100,length=100,sha256='a'*64)

class ObserverChecks(unittest.TestCase):
    def test_real_ff_budget_and_heading_key_no_row_mutation(self):
        c=w.Collector(LEGACY);r=row();before=copy.deepcopy(r)
        c.observe(Path('/r/navigation_pid_history.jsonl'),r,ref())
        self.assertEqual(r,before);x=c.pid[0]
        self.assertEqual(x['outer_heading_error_rad'],.25);self.assertEqual(x['outer_heading_error_source_key'],'heading_error')
        self.assertEqual(x['curvature_FF_raw_radps'],.12)
        self.assertEqual(c.summarize_rows(c.pid)['limited_speed_rows'],1)

    def test_stale_math_not_reused_as_new_ff_and_fixed_tail_not_failed(self):
        c=w.Collector(LEGACY);r=row(updated=False);c.observe(Path('/r/navigation_pid_history.jsonl'),r,ref())
        self.assertIsNone(c.pid[0]['curvature_FF_raw_radps']);self.assertEqual(c.pid[0]['speed_supervisor'],{})
        r=row(mode='active_hold');r['cascade']['spatial_reference'].update(curvature_valid=False,heading_source='fixed_goal_tail')
        c.observe(Path('/r/navigation_pid_history.jsonl'),r,ref(2));s=c.summarize_rows(c.pid)
        self.assertEqual(s['curvature_unavailable_drive_or_constraint_rows'],0);self.assertEqual(s['fixed_goal_tail_math_rows'],1)

    def test_ninth_activation_and_arrival_exclude_stale_start_and_parking_reset(self):
        c=w.Collector(LEGACY)
        for i,(t,p)in enumerate(((9.,'pre_turn'),(10.,'drive'),(10.1,'drive'),(10.2,'pre_turn'),(11.,'drive'),(11.1,'pre_turn'))):
            c.observe(Path('/r/navigation_pid_history.jsonl'),row(t,p),ref(i+1))
        c.activations=[dict(data=dict(waypoint_index=8,control_stamp_ns=10_000_000_000),source=ref())]
        c.receipts['exploration:8']=dict(stamp_ns=11_000_000_000)
        result=c.ninth('/r');self.assertEqual(result['elapsed_s'],1.)
        self.assertEqual(result['post_arrival_rows_excluded'],1)
        self.assertEqual(result['before_arrival_or_last_observation']['drive_exit_count'],1)

    def test_missing_activation_remains_uncomparable(self):
        c=w.Collector(LEGACY);c.observe(Path('/r/navigation_pid_history.jsonl'),row(),ref())
        self.assertFalse(c.ninth('/r')['comparable'])

class WrapperChecks(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.run=self.root/'run';self.run.mkdir();self.out=self.root/'output'
        write(self.run/'runtime_manifest.json',dict(all_owned_and_children_clean=True,owned_processes=[dict(pid=99999999,start_ticks=1)]))
        write(self.run/'run_result.json',dict(done=True));write(self.run/'navigation_profile.json',dict(cascade=dict(spatial_reference=dict(curvature_feedforward_enabled=False,curvature_speed_limit_enabled=False))))
        self.pid=row();write(self.run/'navigation_pid_history.jsonl',self.pid)
        write(self.run/'navigation_slam_poses.jsonl',dict(stamp_ns=10_000_000_000,position=[1.,0.,.3],quaternion=[0.,0.,0.,1.],body_velocity=[.1,0.,0.]))
        write(self.run/'navigation_status.jsonl',dict(ros_sim_time=10.,waypoint_index=8,region_arrivals=[]))
        write(self.run/'telemetry.jsonl',dict(state_physics_world_time=10.,body_lin_vel=[.1,0.,0.],body_ang_vel=[0.,0.,.1]))
        write(self.run/'actuator.jsonl',dict(kind='actuator_contract',base_com_offset_body_m=[.05,0.,0.]))
        write(self.run/'navigation_segment_activation.jsonl',dict(waypoint_index=8,control_stamp_ns=10_000_000_000))
    def tearDown(self):self.tmp.cleanup()
    def fake(self,run,baseline=None):
        self.assertIsNone(baseline);s=AUDIT.Sources()
        for name in ('navigation_slam_poses.jsonl','navigation_status.jsonl','navigation_pid_history.jsonl','telemetry.jsonl','actuator.jsonl'):
            values=list(s.rows(run/name))
            if name=='navigation_pid_history.jsonl':self.assertEqual(values[0][0],self.pid)
        return dict(schema='independent_original46_prefix9_actual_run/v1',run=str(run),run_id=run.name,
            prefix9_limited_pass=False,result_status='not_passed',checks=dict(synthetic_acceptance=False),source_bindings=list(s.bindings.values()))
    def invoke(self,fake=None):
        with mock.patch.object(w,'load',return_value=(LEGACY,AUDIT)),mock.patch.object(AUDIT,'audit',side_effect=fake or self.fake):
            return w.run_once(self.run,'OFF',self.out)
    def test_original_sources_single_pass_hashes_and_failure_preserved(self):
        original=AUDIT.Sources;result=self.invoke();self.assertIs(AUDIT.Sources,original)
        self.assertFalse(result['prefix9_limited_pass']);receipt=json.loads(Path(result['paths']['receipt']).read_text())
        self.assertTrue(all(v==1 for v in receipt['full_JSONL_read_counts'].values()));self.assertEqual(len(receipt['full_JSONL_read_counts']),6)
        formal=json.loads(Path(result['paths']['report']).read_text());self.assertEqual(formal['checks'],dict(synthetic_acceptance=False))
        compact=json.loads(gzip.decompress(Path(result['paths']['series']).read_bytes()))
        binding=next(x for x in compact['source_bindings']if x['file'].endswith('navigation_pid_history.jsonl'))
        self.assertEqual(binding['sha256'],w.sha(self.run/'navigation_pid_history.jsonl'))
        self.assertEqual(compact['pid'][0]['source']['sha256'],hashlib.sha256((self.run/'navigation_pid_history.jsonl').read_bytes()).hexdigest())
    def test_exception_preserves_partial_compact_and_receipt(self):
        def failed(run,baseline=None):
            list(AUDIT.Sources().rows(run/'navigation_pid_history.jsonl'));raise ValueError('synthetic stop')
        result=self.invoke(failed);self.assertIn('synthetic stop',result['error'])
        self.assertFalse(Path(result['paths']['report']).exists());self.assertTrue(Path(result['paths']['series']).exists())
        d=json.loads(Path(result['paths']['diagnostic']).read_text());self.assertFalse(d['diagnostic_complete'])
        self.assertFalse(d['original_acceptance']['prefix9_limited_pass'])
    def test_live_process_refused_even_if_clean_claim(self):
        raw=Path('/proc/self/stat').read_text();start=int(raw[raw.rfind(')')+2:].split()[19])
        write(self.run/'runtime_manifest.json',dict(all_owned_and_children_clean=True,owned_processes=[dict(pid=os.getpid(),start_ticks=start)]))
        with self.assertRaisesRegex(RuntimeError,'stopped'):self.invoke()
        self.assertFalse(self.out.exists())
    def test_missing_final_files_refused_without_raw_read(self):
        (self.run/'run_result.json').unlink()
        with self.assertRaisesRegex(RuntimeError,'live-run'):self.invoke()
    def test_condition_mismatch_refused(self):
        p=self.run/'navigation_profile.json';d=json.loads(p.read_text());d['cascade']['spatial_reference']['curvature_feedforward_enabled']=True;write(p,d)
        with self.assertRaisesRegex(ValueError,'profile flags'):self.invoke()
    def test_existing_output_and_run_output_refused(self):
        self.invoke()
        with self.assertRaisesRegex(ValueError,'overwrite'):self.invoke()
        with self.assertRaisesRegex(ValueError,'archive'):w.run_once(self.run,'OFF',self.run/'outputs')
    def test_duplicate_full_read_refused_and_receipted(self):
        def duplicate(run,baseline=None):
            s=AUDIT.Sources();list(s.rows(run/'navigation_pid_history.jsonl'));list(s.rows(run/'navigation_pid_history.jsonl'))
        self.assertIn('Repeated full JSONL',self.invoke(duplicate)['error'])

if __name__=='__main__':unittest.main(verbosity=2)
