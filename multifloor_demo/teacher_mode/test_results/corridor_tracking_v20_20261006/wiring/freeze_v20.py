"""Freeze only new reviewed V20 finite evidence; no ROS or simulation launch."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys

if not __debug__:
    raise RuntimeError('Do not disable V20 finite checks')
ROOT=Path(__file__).resolve().parents[3]
HERE=ROOT/'navigation/corridor_tracking_v20'
sys.path.insert(0,str(HERE))
from corridor_preflight import REQUIRED,verify_preflight
from slam_workspace import evidence_files as slam_evidence
from scan_workspace import evidence_files as scan_evidence


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--control-review',type=Path,required=True)
    p.add_argument('--finite-receipt',type=Path,action='append',required=True)
    p.add_argument('--extra-evidence',type=Path,action='append',default=[])
    args=p.parse_args()
    target=HERE/'CORRIDOR_V20_PREFLIGHT.json'
    if target.exists():raise RuntimeError('Frozen V20 gate already exists; use an explicit new revision')
    review_path=args.control_review.resolve();review=json.loads(review_path.read_text())
    if (review.get('schema')!='corridor_tracking_v20_control_review/v1' or review.get('reviewed')is not True
        or review.get('actual_navigation_verified')is not False
        or review.get('original_geometry_timeout_dwell_or_safety_changed')is not False
        or review.get('new_reference_is_legacy_controller_byte_identity')is not False):
        raise RuntimeError('V20 reference correction has no explicit limited reviewed source contract')
    pins=review.get('reviewed_source_sha256',{})
    if not {'controller.py','cascade_core.py','spatial_reference.py'}.issubset(pins):
        raise RuntimeError('New reviewed production control/reference pins missing')
    for name,digest in pins.items():
        path=(HERE/name).resolve()
        if not path.is_relative_to(HERE)or sha(path)!=digest:raise RuntimeError('Reviewed source changed: '+name)
    receipts=[];extra=[]
    for path in args.finite_receipt:
        path=path.resolve();d=json.loads(path.read_text())
        if (d.get('passed')is not True or d.get('actual_navigation_verified')is not False
            or d.get('runtime_authorized')is not False):
            raise RuntimeError('Only new finite passing receipts with no runtime outcome may feed this gate')
        source_bindings=d.get('source_bindings')
        if not isinstance(source_bindings,dict)or not source_bindings:
            raise RuntimeError('New finite receipt must name the actual tested source bytes')
        for name,digest in source_bindings.items():
            if sha(name)!=digest:raise RuntimeError('Finite-tested source changed: '+name)
            extra.append(Path(name))
        output=d.get('output')
        if isinstance(output,dict):
            if sha(output['path'])!=output['sha256']:raise RuntimeError('Finite log changed')
            extra.append(Path(output['path']))
        receipts.append(dict(path=str(path),sha256=sha(path)))
    result=subprocess.run([sys.executable,'-B','-m','unittest','wiring_tests','-v'],cwd=HERE,
        capture_output=True,text=True)
    log=Path(__file__).parent/'FINAL_WIRING_REVIEW.log';log.write_text(result.stdout+result.stderr)
    if result.returncode:raise RuntimeError('Fresh final V20 wiring/profile review failed')
    profiles=list((HERE/'profiles').glob('*.json'))
    files=list(HERE.glob('*.py'))+profiles+[HERE/'CORRIDOR_V20_CONTRACT.json',
        *slam_evidence(),review_path,log,Path(__file__).resolve(),*extra,*args.extra_evidence]
    for profile in profiles:files+=scan_evidence(json.loads(profile.read_text()))
    files += [Path(r['path'])for r in receipts]
    for path in HERE.glob('*.py'):compile(path.read_bytes(),str(path),'exec')
    bindings={str(Path(x).resolve()):sha(x)for x in files}
    gate=dict(schema='corridor_tracking_v20_preflight/v1',status='PASS_LIMITED_NEW_EXPERIMENT',
        candidate_root=str(HERE),allowed=True,actual_navigation_verified=False,historical_pass_inherited=False,
        checks={key:True for key in sorted(REQUIRED)},control_review=dict(path=str(review_path),sha256=sha(review_path)),
        finite_receipts=receipts,verified_inputs_sha256=bindings,
        actual_loaded_binary_witness_required=True,
        limits=['V20 new spatial controller/reference finite checks only; actual run has not been verified',
                'V19 core numerical/packet evidence is inherited only for the unchanged exact SLAM binary',
                'First9 profile requires original9 arrivals and first5s active hold; cutoff is not success',
                'Old V9 whole-controller identity failure remains historical; new references are explicitly changed'])
    with target.open('x')as out:json.dump(gate,out,indent=2);out.write('\n')
    for path in profiles:verify_preflight(HERE,json.loads(path.read_text()))
    print(json.dumps(dict(gate=str(target),sha256=sha(target),actual_navigation_verified=False)))


if __name__=='__main__':main()
