#!/usr/bin/env python3
"""Pure negative fixtures and read-only 36-run index audit; no package writes."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
SCRIPT=ROOT/'scripts/update_package_manifest.py'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write(path,data):Path(path).write_text(json.dumps(data,indent=2)+'\n')


def module():
    spec=importlib.util.spec_from_file_location('_readonly_package_index_audit',SCRIPT)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def fixtures(m):
    results=[];real_sha=m.FROZEN_SHA
    with tempfile.TemporaryDirectory(prefix='pure_fixtures_',dir=HERE) as tmp:
        root=Path(tmp);runs=root/'runs';runs.mkdir();checkpoint=root/'PURE_SYNTHETIC_MODEL_FIXTURE.bin'
        checkpoint.write_bytes(b'Pure fixture only; not a Teacher model or runtime')
        m.FROZEN_SHA=sha(checkpoint)
        def make(name):
            r=runs/name;r.mkdir();native=r/'sources/native';native.mkdir(parents=True)
            binary=native/'libteacher_actuator.so';binary.write_bytes(b'PURE_NATIVE_METADATA_FIXTURE')
            world=r/'world.sdf';world.write_text('<sdf version="1.9"><world name="PURE_FIXTURE_ONLY"/></sdf>')
            write(r/'source_manifest.json',{
                'native':{'path':str(binary),'sha256':sha(binary)},
                'input_world':{'path':str(world),'sha256':sha(world),'generated_input':True}})
            write(r/'runtime_manifest.json',{'owned_processes':[{'role':'worker','pid':101,'returncode':0},{'role':'gazebo','pid':102,'returncode':0}],
                'exclusive_writer':'teacher_sim::TeacherActuator','source_manifest_sha256':sha(r/'source_manifest.json'),
                'native_plugin_sha256':sha(binary),'model_sha256':m.FROZEN_SHA})
            write(r/'policy_manifest.json',{'checkpoint':str(checkpoint),'checkpoint_sha256':m.FROZEN_SHA})
            rows=[{'kind':'actuator_contract','writer':'teacher_sim::TeacherActuator sole JointForceCmd writer'},
                  {'kind':'model_plugin_ownership','passed':True,'teacher_writers':1,'plugins':[{'name':'teacher_sim::TeacherActuator','filename':'libteacher_actuator.so'}]},
                  {'kind':'physics_step','fault':0}]
            (r/'actuator.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows));(r/'telemetry.jsonl').write_text('{"PURE_FIXTURE_ONLY":true}\n')
            return r
        def test(name,run,expected):
            row=next(x for x in m.run_index(root,[]) if x['run_id']==run.name)
            assert row['classification']==expected,(name,row)
            assert row['counted_as_complete_pass'] is False
            results.append({'name':name,'passed':True,'classification':row['classification'],'basis':row['classification_basis']})
        empty=runs/'empty_prepare';empty.mkdir();test('empty_prepare',empty,'configuration_or_offline_preparation')
        valid=make('valid_self_prepare');test('valid_matched_self_sources_no_runtime_run_field',valid,'actual_gazebo_runtime_recorded')
        copied=runs/'offline_prepare_copied';shutil.copytree(valid,copied)
        test('copied_other_run_absolute_sources_rejected',copied,'configuration_or_offline_preparation')
        tampered=make('tampered_prepare');runtime=json.loads((tampered/'runtime_manifest.json').read_text());runtime['source_manifest_sha256']='0'*64;write(tampered/'runtime_manifest.json',runtime)
        test('tampered_source_manifest_SHA_rejected',tampered,'configuration_or_offline_preparation')
        graph=make('missing_graph_prepare');runtime=json.loads((graph/'runtime_manifest.json').read_text());runtime['owned_processes']=runtime['owned_processes'][:1];write(graph/'runtime_manifest.json',runtime)
        test('no_owned_native_worker_gazebo_graph',graph,'configuration_or_offline_preparation')
        plugin=make('no_native_owner_prepare');data=[json.loads(x) for x in (plugin/'actuator.jsonl').read_text().splitlines()];data[1]['teacher_writers']=2;data[1]['passed']=False;(plugin/'actuator.jsonl').write_text(''.join(json.dumps(r)+'\n'for r in data))
        test('multiple_native_writers_rejected',plugin,'configuration_or_offline_preparation')
        failed=make('failed_actual_prepare');runtime=json.loads((failed/'runtime_manifest.json').read_text());runtime['owned_processes'][1]['returncode']=-15;runtime['error']='PURE_FAILURE_FIXTURE';write(failed/'runtime_manifest.json',runtime)
        test('nonzero_failed_runtime_is_still_actual_evidence_never_PASS',failed,'actual_gazebo_runtime_recorded')
        partial=make('partial_actual_prepare');runtime=json.loads((partial/'runtime_manifest.json').read_text());runtime['owned_processes'][1]['returncode']=None;write(partial/'runtime_manifest.json',runtime)
        test('unfinished_runtime_is_still_actual_evidence_never_PASS',partial,'actual_gazebo_runtime_recorded')
        binary=make('tampered_native_prepare');(binary/'sources/native/libteacher_actuator.so').write_bytes(b'TAMPERED_FIXTURE')
        test('native_binary_tampering_rejected',binary,'configuration_or_offline_preparation')
        model=make('wrong_model_prepare');runtime=json.loads((model/'runtime_manifest.json').read_text());runtime['model_sha256']='1'*64;write(model/'runtime_manifest.json',runtime)
        test('model_identity_mismatch_rejected',model,'configuration_or_offline_preparation')
    m.FROZEN_SHA=real_sha
    return results


def main():
    ast.parse(SCRIPT.read_text());m=module();tests=fixtures(m)
    campaign=ROOT/'test_results/curvature_campaign_20261005'
    paths=[campaign/'phase_radius_v3_results.jsonl',campaign/'phase_boundary_v3_results.jsonl']
    records=[json.loads(line) for p in paths for line in p.read_text().splitlines() if line]
    assert len(records)==36 and len({r['run']for r in records})==36
    index={r['run_id']:r for r in m.run_index(ROOT,[])}
    actual=[]
    for original in records:
        run=Path(original['run']).resolve();row=index[run.name]
        assert row['classification']=='actual_gazebo_runtime_recorded',row
        assert row['classification_basis']['verified'] is True and row['counted_as_complete_pass'] is False
        actual.append({'run':str(run),'original_result_status':original['status'],'classification':row['classification'],
            'counted_as_complete_pass':row['counted_as_complete_pass'],'classification_basis':row['classification_basis'],
            'runtime_manifest_sha256':sha(run/'runtime_manifest.json'),'source_manifest_sha256':sha(run/'source_manifest.json')})
    result={'schema':'teacher_package_index_classification_audit/v1','status':'passed','script_sha256':sha(SCRIPT),
        'audit_script_sha256':sha(__file__),'pure_fixture_tests':tests,'pure_fixture_test_count':len(tests),
        'actual_plant_runs_checked':36,'actual_classification_counts':{'actual_gazebo_runtime_recorded':36},
        'actual_runs':actual,'original_registry_hashes':{str(p):sha(p)for p in paths},
        'package_manifest_written':False,'current_status_written':False,'source_or_actual_receipts_changed':False,
        'counted_as_complete_pass_for_every_case':False,'simulation_or_process_operation_performed':False,
        'meaning':'Inventory classification only. A failed/nonzero actual runtime remains actual evidence; no overall PASS promotion.'}
    out=HERE/'audit.json'
    with out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'status':'passed','pure_tests':len(tests),'actual36':'actual_gazebo_runtime_recorded','audit':str(out),'sha256':sha(out),'script_sha256':sha(SCRIPT)}))


if __name__=='__main__':main()
