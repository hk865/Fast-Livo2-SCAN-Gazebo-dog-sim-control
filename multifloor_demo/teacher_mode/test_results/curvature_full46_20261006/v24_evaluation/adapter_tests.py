"""Small reader-only checks; no runtime logs, ROS or simulator are opened."""
import copy
import math
from pathlib import Path
import unittest

from evaluate_full46_v24 import heading_reference,load_adapter,OLD,OLD_SHA,sha

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
        self.assertEqual(audit.V19.name,'corridor_tracking_v24_recovery_replan')
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


import ast
import json
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import evaluate_full46_v24 as adapter

class V24SourceAndScopeTests(unittest.TestCase):
    def fixture(self,d):
        top=Path(d)/'teacher';candidate=top/'navigation/corridor_tracking_v24_recovery_replan'
        run=Path(d)/'closed-run';candidate.mkdir(parents=True)
        pins={};index={};manifest={};refs={}
        for name in ('shared_controller.py','replan_policy.py'):
            source=candidate/name;source.write_text('fixture '+name);pin=sha(source);pins[name]=pin
            relative=source.relative_to(top);snapshot=run/'sources'/relative
            snapshot.parent.mkdir(parents=True,exist_ok=True);snapshot.write_bytes(source.read_bytes())
            index[str(source)]={'snapshot':str(snapshot),'sha256':pin}
            refs[str(source)]=refs[str(snapshot)]=pin
            manifest[str(source)]=manifest[str(relative)]=pin
        profile={'controller_selector':'v24_recovery_replan_full46','profile_version':'corridor_tracking_v24_recovery_replan'}
        data={'navigation_source_snapshots.json':index,'source_manifest.json':manifest,'navigation_profile.json':profile}
        audit=SimpleNamespace(read=lambda p:data[Path(p).name])
        return top,candidate,run,pins,index,manifest,refs,audit,profile

    def test_original_checks_and_threshold_expressions_AST_unchanged(self):
        old=ast.parse(adapter.FROZEN_PARENT_ADAPTER.read_text());new=ast.parse(Path(adapter.__file__).read_text())
        self.assertEqual(sha(adapter.FROZEN_PARENT_ADAPTER),adapter.FROZEN_PARENT_ADAPTER_SHA)
        self.assertEqual(sha(adapter.FROZEN_VERSION_ADAPTER),adapter.FROZEN_VERSION_ADAPTER_SHA)
        self.assertEqual(sha(adapter.OLD),adapter.OLD_SHA)
        class Normalize(ast.NodeTransformer):
            def visit_Constant(self,node):
                if isinstance(node.value,str):node.value=node.value.replace('V24 source selection','V23 source selection').replace('this V24 full46','this V23 full46').replace('unverified. V24 recorded','unverified. V23 recorded')
                return node
        old=Normalize().visit(old);new=Normalize().visit(new)
        additions={'FROZEN_PARENT_ADAPTER','FROZEN_PARENT_ADAPTER_SHA','REPAIR_PINS'}
        new.body=[n for n in new.body if not(isinstance(n,ast.Assign)and any(isinstance(x,ast.Name)and x.id in additions for x in n.targets))and not(isinstance(n,ast.FunctionDef)and n.name=='repair_source_closure')]
        def assignments(tree):return {n.targets[0].id:n for n in tree.body if isinstance(n,ast.Assign)and isinstance(n.targets[0],ast.Name)}
        oa=assignments(old);na=assignments(new)
        for name in ('TOP','V20'):na[name].value=copy.deepcopy(oa[name].value)
        load=next(n for n in new.body if isinstance(n,ast.FunctionDef)and n.name=='load_adapter')
        self.assertIn('FROZEN_PARENT_ADAPTER',ast.unparse(load.body[0]));load.body.pop(0)
        closure=next(n for n in load.body if isinstance(n,ast.FunctionDef)and n.name=='source_closure')
        closure.body=[n for n in closure.body if not(isinstance(n,ast.Assign)and isinstance(n.targets[0],ast.Name)and n.targets[0].id=='repair_sources')]
        update=next(n.value for n in closure.body if isinstance(n,ast.Expr)and isinstance(n.value,ast.Call)and isinstance(n.value.func,ast.Attribute)and n.value.func.attr=='update')
        update.keywords=[k for k in update.keywords if k.arg!='recovery_replan_exact_source_closure']
        self.assertEqual(ast.dump(old,include_attributes=False),ast.dump(new,include_attributes=False))

    def test_real_prepare_small_sources_have_exact_repair_closure(self):
        run=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_173302_closed_loop_cascade_clock_hold_v24_recovery_replan_prepare_09f7')
        audit=SimpleNamespace(read=lambda p:json.loads(Path(p).read_text()))
        rows=adapter.repair_source_closure(audit,run,audit.read(run/'navigation_scope.json'))
        self.assertEqual({Path(r['source']).name:r['sha256']for r in rows},adapter.REPAIR_PINS)

    def test_exact_source_fixture_passes(self):
        with tempfile.TemporaryDirectory()as d:
            top,candidate,run,pins,index,manifest,refs,audit,profile=self.fixture(d)
            with patch.object(adapter,'TOP',top),patch.object(adapter,'V20',candidate),patch.object(adapter,'REPAIR_PINS',pins):
                self.assertEqual(len(adapter.repair_source_closure(audit,run,{'references':refs})),2)

    def test_foreign_same_basename_cannot_replace_exact_V24(self):
        with tempfile.TemporaryDirectory()as d:
            top,candidate,run,pins,index,manifest,refs,audit,profile=self.fixture(d)
            key=str(candidate/'shared_controller.py');index[str(candidate.parent/'corridor_tracking_v23_curvature_full46/shared_controller.py')]=index.pop(key)
            with patch.object(adapter,'TOP',top),patch.object(adapter,'V20',candidate),patch.object(adapter,'REPAIR_PINS',pins):
                with self.assertRaises(ValueError):adapter.repair_source_closure(audit,run,{'references':refs})

    def test_missing_wrong_digest_ref_or_either_manifest_key_rejected(self):
        for kind in ('missing','digest','source_ref','snapshot_ref','manifest_relative','manifest_absolute'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory()as d:
                top,candidate,run,pins,index,manifest,refs,audit,profile=self.fixture(d)
                source=candidate/'replan_policy.py';key=str(source);row=index[key]
                if kind=='missing':index.pop(key)
                elif kind=='digest':row['sha256']='0'*64
                elif kind=='source_ref':refs.pop(key)
                elif kind=='snapshot_ref':refs.pop(row['snapshot'])
                elif kind=='manifest_relative':manifest.pop(str(source.relative_to(top)))
                elif kind=='manifest_absolute':manifest.pop(key)
                with patch.object(adapter,'TOP',top),patch.object(adapter,'V20',candidate),patch.object(adapter,'REPAIR_PINS',pins):
                    with self.assertRaises(ValueError):adapter.repair_source_closure(audit,run,{'references':refs})

    def test_noncanonical_missing_symlink_or_modified_snapshot_rejected(self):
        for kind in ('foreign','missing','symlink','modified','live_changed'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory()as d:
                top,candidate,run,pins,index,manifest,refs,audit,profile=self.fixture(d)
                source=candidate/'replan_policy.py';row=index[str(source)];snap=Path(row['snapshot'])
                if kind=='foreign':
                    other=run/'other.py';other.write_bytes(source.read_bytes());row['snapshot']=str(other)
                elif kind=='missing':snap.unlink()
                elif kind=='symlink':snap.unlink();snap.symlink_to(source)
                elif kind=='modified':snap.write_text('changed')
                elif kind=='live_changed':source.write_text('changed')
                with patch.object(adapter,'TOP',top),patch.object(adapter,'V20',candidate),patch.object(adapter,'REPAIR_PINS',pins):
                    with self.assertRaises(ValueError):adapter.repair_source_closure(audit,run,{'references':refs})

    def test_candidate_selector_or_version_mismatch_rejected(self):
        for name in ('controller_selector','profile_version'):
            with self.subTest(name=name),tempfile.TemporaryDirectory()as d:
                top,candidate,run,pins,index,manifest,refs,audit,profile=self.fixture(d);profile[name]='parent_or_foreign'
                with patch.object(adapter,'TOP',top),patch.object(adapter,'V20',candidate),patch.object(adapter,'REPAIR_PINS',pins):
                    with self.assertRaises(ValueError):adapter.repair_source_closure(audit,run,{'references':refs})

    def test_absent_full_replays_stay_unverified_and_other_failure_stays_failed(self):
        # Twenty synthetic base slots exercise only aggregation. The AST test
        # proves the production base check identities and logic are unchanged.
        for fail in (False,True):
            with self.subTest(fail=fail),tempfile.TemporaryDirectory()as d:
                run=Path(d)/'closed';run.mkdir()
                for n in ('run_result.json','runtime_manifest.json'):(run/n).write_text('{}')
                checks={f'base_{i}':{'status':'passed','passed':True}for i in range(18)}
                for n in adapter.UNIMPLEMENTED_FULL_REPLAYS:checks[n]={'status':'unverified','passed':None}
                if fail:checks['base_0']={'status':'failed','passed':False}
                fake=SimpleNamespace(evaluate=lambda run:dict(checks=copy.deepcopy(checks),limitations=['placeholder']),read=lambda p:{},wrapped=lambda f:dict(status='passed',passed=True,result=f()))
                with patch.object(adapter,'load_adapter',return_value=fake),patch.object(adapter,'scan_scope',return_value={}),patch.object(adapter,'phase_proof_order',return_value={}):out=adapter.evaluate(run)
                self.assertEqual(len(out['checks']),22);self.assertEqual(out['status'],'failed'if fail else'unverified')
                self.assertFalse(out['passed']);self.assertIs(out['functional46_limited_pass'],not fail)
                self.assertTrue(all(out['checks'][n]['passed']is None for n in adapter.UNIMPLEMENTED_FULL_REPLAYS))

if __name__=='__main__':unittest.main(verbosity=2)
