#!/usr/bin/env python3
"""Freeze only new V24 limited evidence after independent scalar/control review."""
import argparse,hashlib,json,sys
from pathlib import Path

if not __debug__:
    raise RuntimeError('gate freeze forbids optimized Python')
HERE=Path(__file__).resolve().parents[3]/'navigation/corridor_tracking_v24_recovery_replan'
sys.path.insert(0,str(HERE))
from corridor_preflight import REQUIRED,verify_preflight
from geometry_archive import geometry_helper_files
from mission46_profile import required_source_files
from full46_launch_contract import baseline_profile_path
from slam_workspace import evidence_files as slam_evidence_files
from scan_workspace import evidence_files as scan_evidence_files

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--review',required=True,type=Path)
    ap.add_argument('--scalar-receipt',required=True,type=Path);ap.add_argument('--wiring-receipt',required=True,type=Path)
    ap.add_argument('--full46-review',required=True,type=Path)
    a=ap.parse_args();target=HERE/'RECOVERY_REPLAN_V24_PREFLIGHT.json'
    if target.exists():raise RuntimeError('refuse replacing frozen V24 gate')
    profile=json.loads((HERE/'profiles/curvature_original46_on.json').read_text())
    scalar=json.loads(a.scalar_receipt.read_text());wire=json.loads(a.wiring_receipt.read_text())
    if scalar.get('passed') is not True or wire.get('passed') is not True or wire.get('schema') != 'recovery_replan_v24_finite/v1' or wire.get('tests') != 61 or wire.get('execution', {}).get('exit_code') != 0:
        raise RuntimeError('new scalar and wiring finite receipts must actually pass')
    paths=list(HERE.glob('*.py'))+list((HERE/'profiles').glob('*.json'))+list(HERE.glob('*.json'))
    paths+=slam_evidence_files()+scan_evidence_files(profile)+geometry_helper_files(HERE)+required_source_files()+[baseline_profile_path(HERE),a.review,a.full46_review,a.scalar_receipt,a.wiring_receipt,
        HERE.parent/'corridor_tracking_v21_curvature/PUBLICATION_LEDGER_PREFLIGHT.json']
    # Include the exact evidence recursively declared by the freshly produced finite receipts.
    for receipt in (scalar,wire,json.loads(a.full46_review.read_text())):
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
    d=dict(schema='recovery_replan_v24_full46_preflight/v1',status='PASS_LIMITED_NEW_EXPERIMENT',
        candidate_root=str(HERE.resolve()),allowed=True,actual_navigation_verified=False,historical_pass_inherited=False,
        allowed_profiles=['curvature_original46_on.json'],checks={k:True for k in sorted(REQUIRED)},
        control_review=dict(path=str(a.review.resolve()),sha256=sha(a.review)),
        full46_review=dict(path=str(a.full46_review.resolve()),sha256=sha(a.full46_review)),
        finite_receipts=[dict(path=str(p.resolve()),sha256=sha(p))for p in [a.scalar_receipt,a.wiring_receipt]],
        actual_loaded_binary_witness_required=True,
        verified_inputs_sha256={str(p.resolve()):sha(p)for p in sorted(set(paths))},
        limitations=['Full original46 curvature ON prospective attempt only, not an actual motion/navigation pass',
            'V20 cc82 numpy.bool_ crash and V21 dc92 incomplete archive remain immutable failed experiments',
            'Five geometry helpers are direct gate inputs and canonical source snapshots; no old manifest is amended',
            'V19 estimator and protected SCAN identities inherited explicitly; no new C++ build or performance claim',
            'Protected/recovery exact-zero no longer falsely clears a geometrically valid SCAN path; V23 failure remains unchanged',
            'Original46 initialization, 18+14+14 arrival dwells, RGB, dynamic stop/resume, layers and first5s parking must be independently verified; cutoff is not success'])
    target.write_text(json.dumps(d,indent=2)+'\n')
    verify_preflight(HERE,profile)
    print(json.dumps(dict(gate=str(target),sha256=sha(target),bindings=len(d['verified_inputs_sha256']),
                         profile='curvature_original46_on'),indent=2))
if __name__=='__main__':main()
