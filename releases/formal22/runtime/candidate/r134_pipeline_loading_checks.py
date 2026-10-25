"""Finite actual-loader checks under the real legacy pipeline cache; no runtime."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import sys
import tempfile

sys.dont_write_bytecode=True
import owned_guardian as guardian


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    here=Path(__file__).resolve().parent;source=here/'pipeline_lifecycle.py'
    old=Path('/opt/experiment/history/2026-10-07/task/go2_engineering_v33/R83_PARKING_HYSTERESIS_STAGE/pipeline_lifecycle.py')
    spec=importlib.util.spec_from_file_location('pipeline_lifecycle',old)
    legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
    previous=sys.modules.get('pipeline_lifecycle');previous_path=list(sys.path)
    checks={};detail={}
    try:
        sys.modules['pipeline_lifecycle']=legacy;sys.path.insert(0,str(old.parent))
        try:exec('from pipeline_lifecycle import reuse_normal_stop',{})
        except ImportError:checks['actual_legacy_cache_reproduces_D11_missing_function']=True
        else:checks['actual_legacy_cache_reproduces_D11_missing_function']=False
        with tempfile.TemporaryDirectory(prefix='r134_pipeline_loading_') as temporary:
            run=Path(temporary);manifest={str(source):hashlib.sha256(source.read_bytes()).hexdigest()}
            raw=json.dumps(manifest).encode();(run/'source_manifest.json').write_bytes(raw)
            plan=dict(source_manifest_sha256=hashlib.sha256(raw).hexdigest())
            loaded,witness=guardian.load_pipeline_lifecycle(run,plan)
            checks['verified_sibling_loaded_despite_real_legacy_cache_and_path']=(
                loaded is not legacy and loaded.__file__==str(source) and
                witness['actual_source_sha256']==manifest[str(source)] and witness['verified'] is True and
                all(Path(v)==source for v in witness['actual_function_code'].values()))
            checks['legacy_module_cache_not_silently_replaced']=sys.modules['pipeline_lifecycle'] is legacy
            try:guardian.load_pipeline_lifecycle(run,dict(source_manifest_sha256='0'*64))
            except RuntimeError:checks['changed_frozen_manifest_rejected_before_ready']=True
            else:checks['changed_frozen_manifest_rejected_before_ready']=False
            bad=json.dumps({str(source):'0'*64}).encode();(run/'source_manifest.json').write_bytes(bad)
            try:guardian.load_pipeline_lifecycle(run,dict(source_manifest_sha256=hashlib.sha256(bad).hexdigest()))
            except RuntimeError:checks['changed_sibling_source_binding_rejected_before_ready']=True
            else:checks['changed_sibling_source_binding_rejected_before_ready']=False
            detail=witness
    finally:
        sys.path[:]=previous_path
        if previous is None:sys.modules.pop('pipeline_lifecycle',None)
        else:sys.modules['pipeline_lifecycle']=previous
    report=dict(schema='r134_guardian_pipeline_loading_checks/v1',passed=all(checks.values()),checks=checks,
        actual_loader_witness=detail,legacy_pipeline=dict(path=str(old),sha256=hashlib.sha256(old.read_bytes()).hexdigest()),
        no_admission_runtime_ROS_model_process_signals_or_CRC=True)
    with a.output.open('x')as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps(dict(passed=report['passed'],checks=checks),indent=2))
    if not report['passed']:raise SystemExit(2)


if __name__=='__main__':main()
