"""Synthetic-only checks for the additive limited-prefix result panel."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('prefix9_dashboard',HERE.parents[2]/'scripts/serve.py')
viewer=importlib.util.module_from_spec(spec);spec.loader.exec_module(viewer)


def write(path,data):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(data,sort_keys=True)+'\n')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


class Prefix9PanelTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.root_patch=patch.object(viewer,'ROOT',self.root);self.root_patch.start()
        viewer.PREFIX9_VIEW_CACHE.clear();viewer.SOURCE_DIGEST_CACHE.clear()
        self.run=self.root/'runs/synthetic_prefix';self.run.mkdir(parents=True)
        for name in viewer.PREFIX9_RUN_SOURCES|{'worker_result.json'}:write(self.run/name,{})
        write(self.run/'navigation_profile.json',{'original46_prefix_regions':9,'controller_selector':'corridor_tracking_v20'})
        self.report_path=self.root/'test_results/prefix_report.json'
        self.index_path=self.root/viewer.PREFIX9_REPORT_INDEX
        receipts=[{'goal_id':f'exploration:{i}','passed':True,'checks':{'raw_dwell':True}}for i in range(9)]
        self.raw={'schema':viewer.PREFIX9_REPORT_SCHEMA,'run_id':self.run.name,'run':str(self.run),
            'prefix9_limited_pass':True,'result_status':'limited_pass',
            'checks':{key:True for key in viewer.PREFIX9_REQUIRED_CHECKS},
            'full46_pass':False,'whole_200hz_physical_acceptance':False,'corridor_control_verified':False,
            'original_regions':{'passed':True,'actual_receipt_count':9,'raw_validated_count':9,'receipts':receipts},
            'first5s_parking':{'passed':True,'checks':{'fixed_first':True},'clock_interval_ns':[10,5_000_000_010],
                'metrics':{'slam_xy_drift_max_m':.0065478451805498624,'slam_yaw_drift_max_rad':.00366,
                    'native_planar_speed_max_mps':.0197,'native_wz_max_radps':.00312,'native_yaw_dot_max_radps':.00302}},
            'source_bindings':[{'file':str(p),'sha256':digest(p)}for p in sorted(self.run.iterdir())]}
        self.save()
    def tearDown(self):
        self.root_patch.stop();self.temp.cleanup()
    def save(self):
        write(self.report_path,self.raw)
        write(self.index_path,{'schema':'teacher_prefix9_report_index/v1','reports':{self.run.name:{
            'run_directory':str(self.run),'report_file':str(self.report_path),'report_sha256':digest(self.report_path)}}})
    def view(self):return viewer.prefix9_view(self.run)
    def test_complete_consistent_report_is_only_limited_pass(self):
        r=self.view();self.assertTrue(r['valid_for_selected_run']);self.assertTrue(r['limited_prefix9_pass'])
        self.assertEqual(r['original_regions_verified'],9);self.assertEqual(r['status'],'passed_limited')
        self.assertAlmostEqual(r['parking_metrics']['slam_xy_drift_max_m'],.0065478451805498624)
        self.assertFalse(r['full46_pass']);self.assertFalse(r['whole_200hz_physical_acceptance'])
        self.assertFalse(r['corridor_control_verified']);self.assertEqual(viewer.mission46_view(self.run),{})
        self.assertEqual(viewer.closed_loop_receipt_view(self.run),{})
    def test_foreign_run_cannot_reuse_pass_even_with_rebound_report_hash(self):
        self.raw['run_id']='other';self.save();self.assertFalse(self.view()['valid_for_selected_run'])
    def test_changed_report_hash_is_rejected(self):
        self.report_path.write_text(self.report_path.read_text()+' ')
        r=self.view();self.assertFalse(r['limited_prefix9_pass']);self.assertIn('bytes changed',r['validation_errors'][0])
    def test_source_changed_same_size_and_mtime_invalidates_cache(self):
        self.assertTrue(self.view()['limited_prefix9_pass'])
        path=self.run/'telemetry.jsonl';before=path.stat();path.write_text('[]\n')
        os.utime(path,ns=(before.st_atime_ns,before.st_mtime_ns))
        r=self.view();self.assertFalse(r['valid_for_selected_run']);self.assertIn('source bytes changed',r['validation_errors'][0])
    def test_cached_refresh_stats_but_does_not_rehash_sources(self):
        self.assertTrue(self.view()['limited_prefix9_pass'])
        with patch.object(viewer,'source_file_sha256',side_effect=AssertionError('unexpected rehash')):
            self.assertTrue(self.view()['limited_prefix9_pass'])
    def test_missing_selected_raw_source_binding_is_rejected(self):
        self.raw['source_bindings']=[b for b in self.raw['source_bindings']if not b['file'].endswith('telemetry.jsonl')]
        self.save();self.assertFalse(self.view()['valid_for_selected_run'])
    def test_scope_escalation_is_rejected(self):
        for key in ('full46_pass','whole_200hz_physical_acceptance','corridor_control_verified'):
            self.raw[key]=True;self.save();self.assertFalse(self.view()['valid_for_selected_run']);self.raw[key]=False
    def test_count_without_nine_raw_receipts_is_rejected(self):
        self.raw['original_regions']['receipts'].pop();self.save();self.assertFalse(self.view()['valid_for_selected_run'])
    def test_non_fixed_or_failed_parking_is_rejected(self):
        self.raw['first5s_parking']['clock_interval_ns'][1]-=1;self.save()
        self.assertFalse(self.view()['valid_for_selected_run'])
    def test_parking_metric_cannot_exceed_original_limit(self):
        self.raw['first5s_parking']['metrics']['slam_xy_drift_max_m']=.05001;self.save()
        self.assertFalse(self.view()['valid_for_selected_run'])
    def test_changed_check_cannot_keep_pass_flag(self):
        self.raw['checks']['execution_contract']=False;self.save()
        self.assertFalse(self.view()['valid_for_selected_run'])
    def test_foreign_source_cannot_be_hashed_as_selected_evidence(self):
        foreign=self.root/'other.txt';foreign.write_text('foreign')
        self.raw['source_bindings'].append({'file':str(foreign),'sha256':digest(foreign)})
        self.save();self.assertFalse(self.view()['valid_for_selected_run'])
    def test_malformed_index_is_unverified_without_server_exception(self):
        for value in ([],{'reports':[]}):
            write(self.index_path,value)
            self.assertFalse(self.view()['valid_for_selected_run'])
    def test_malformed_profile_has_no_prefix_panel(self):
        write(self.run/'navigation_profile.json',[])
        self.assertEqual(self.view(),{})
    def test_run_switch_to_nonprefix_hides_panel(self):
        self.assertTrue(self.view()['limited_prefix9_pass'])
        write(self.run/'navigation_profile.json',{'mission46_required':True})
        self.assertEqual(self.view(),{})
    def test_script_is_syntax_valid_and_does_not_replace_old_acceptance(self):
        script=viewer.PREFIX9_PAGE_SCRIPT.replace('<script>','').replace('</script>','')
        self.assertNotIn("$('acceptanceStatus')",script)
        self.assertNotIn('showTests(',script)
        if shutil.which('node'):
            checked=subprocess.run(['node','--check'],input=script,text=True,capture_output=True)
            self.assertEqual(checked.returncode,0,checked.stderr)
    def test_renderer_formats_limited_metric_and_hides_on_run_switch(self):
        if not shutil.which('node'):self.skipTest('Node is not installed')
        script=viewer.PREFIX9_PAGE_SCRIPT.replace('<script>','').replace('</script>','')
        setup="""
const elements={};
class Element {
 constructor(){this.children=[];this.textContent='';this.hidden=false;}
 append(...items){this.children.push(...items);}
 replaceChildren(){this.children=[];this.textContent='';}
 before(item){elements[item.id]=item;}
}
globalThis.document={createElement:()=>new Element()};
for(const id of ['legacyPanel','raw','acceptanceStatus'])elements[id]=new Element();
elements.raw.textContent='{}';elements.acceptanceStatus.textContent='Existing full46 UNVERIFIED';
globalThis.$=id=>elements[id];globalThis.show=()=>{};
globalThis.fmt=(value,n)=>Number(value).toFixed(n);
"""
        verify="""
const allText=e=>e.textContent+' '+e.children.map(allText).join(' ');
show({prefix9_result:RESULT});
if(!allText(elements.prefix9Panel).includes('0.00655 m'))throw Error('metric missing');
if(!allText(elements.prefix9Panel).includes('有限通过'))throw Error('limited result missing');
if(elements.acceptanceStatus.textContent!=='Existing full46 UNVERIFIED')throw Error('global acceptance overwritten');
show({prefix9_result:{}});
if(!elements.prefix9Panel.hidden)throw Error('foreign run retained prefix panel');
""".replace('RESULT',json.dumps(self.view()))
        rendered=subprocess.run(['node'],input=setup+script+verify,text=True,capture_output=True)
        self.assertEqual(rendered.returncode,0,rendered.stderr)


if __name__=='__main__':unittest.main(verbosity=2)
