#!/usr/bin/env python3
"""Freeze only new V21 limited evidence after independent scalar/control review."""
import argparse,hashlib,json,sys
from pathlib import Path

if not __debug__:
    raise RuntimeError('gate freeze forbids optimized Python')
HERE=Path(__file__).resolve().parents[3]/'navigation/corridor_tracking_v21_curvature'
sys.path.insert(0,str(HERE))
from corridor_preflight import REQUIRED,verify_preflight
from slam_workspace import evidence_files as slam_evidence_files
from scan_workspace import evidence_files as scan_evidence_files

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--review',required=True,type=Path)
    ap.add_argument('--scalar-receipt',required=True,type=Path);ap.add_argument('--wiring-receipt',required=True,type=Path)
    a=ap.parse_args();target=HERE/'CURVATURE_V21_PREFLIGHT.json'
    if target.exists():raise RuntimeError('refuse replacing frozen V21 gate')
    profile=json.loads((HERE/'profiles/curvature_original46_prefix9_on.json').read_text())
    scalar=json.loads(a.scalar_receipt.read_text());wire=json.loads(a.wiring_receipt.read_text())
    if scalar.get('passed') is not True or wire.get('passed') is not True:
        raise RuntimeError('new scalar and wiring finite receipts must actually pass')
    paths=list(HERE.glob('*.py'))+list((HERE/'profiles').glob('*.json'))+list(HERE.glob('*.json'))
    paths+=slam_evidence_files()+scan_evidence_files(profile)+[a.review,a.scalar_receipt,a.wiring_receipt]
    # Include the exact evidence recursively declared by the freshly produced finite receipts.
    for receipt in (scalar,wire):
        bindings=receipt.get('source_bindings',{})
        items=bindings.items() if isinstance(bindings,dict) else [(r['path'],r['sha256'])for r in bindings]
        for path,digest in items:
            if sha(path)!=digest:raise RuntimeError('finite source/input changed '+path)
            paths.append(Path(path))
    control=json.loads(a.review.read_text())
    for field in ('finite_receipt','failure_reproduction','patch_scope'):
        row=control[field]
        if sha(row['path'])!=row['sha256']:raise RuntimeError('review dependency changed '+field)
        paths.append(Path(row['path']))
    if scalar.get('schema')!='curvature_v21_serialization_finite_repair/v1' or scalar['execution']['exit_code']!=0:
        raise RuntimeError('wrong new finite repair evidence')
    d=dict(schema='curvature_v21_serialization_preflight/v1',status='PASS_LIMITED_NEW_EXPERIMENT',
        candidate_root=str(HERE.resolve()),allowed=True,actual_navigation_verified=False,historical_pass_inherited=False,
        allowed_profiles=['curvature_original46_prefix9_on.json'],checks={k:True for k in sorted(REQUIRED)},
        control_review=dict(path=str(a.review.resolve()),sha256=sha(a.review)),
        finite_receipts=[dict(path=str(p.resolve()),sha256=sha(p))for p in [a.scalar_receipt,a.wiring_receipt]],
        actual_loaded_binary_witness_required=True,
        verified_inputs_sha256={str(p.resolve()):sha(p)for p in sorted(set(paths))},
        limitations=['Bounded original9 ON retest only, not an actual motion/navigation pass',
            'V20 cc82 numpy.bool_ crash remains an immutable failed experiment',
            'V19 estimator and protected SCAN identities inherited explicitly; no new C++ build or performance claim',
            'Original9 dwell plus first5s active hold must be independently evaluated; clock cutoff is not success'])
    target.write_text(json.dumps(d,indent=2)+'\n')
    verify_preflight(HERE,profile)
    print(json.dumps(dict(gate=str(target),sha256=sha(target),bindings=len(d['verified_inputs_sha256']),
                         profile='curvature_original46_prefix9_on'),indent=2))
if __name__=='__main__':main()
