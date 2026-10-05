#!/usr/bin/env python3
"""Freeze finite plant sweep plans; no ROS context or simulator is started."""
import argparse
import hashlib
import json
from pathlib import Path
import itertools
HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[1]


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def build(speed, yaw, repetition=1):
    p = json.loads((HERE/'protocol.json').read_text())
    if speed not in p['speeds_mps'] or yaw not in p['yaw_rates_radps'] or type(repetition) is not int or repetition < 1:
        raise ValueError('Case outside prospective speed/rate grid')
    # This actual-run policy archive has the same unchanged frozen Teacher as
    # calibration; new curvature sources and native binary are pinned below.
    baseline = ROOT/'runs/20261005_000700_truth_pid_flat_line_cascade_pi_25hz_5bb4/sources/policy'
    sources = {str(HERE/name): sha(HERE/name) for name in ('plant_worker.py','prepare_plan.py','run.py','evaluate_radius.py','protocol.json','README.md','pure_tests.py')}
    for name in ('worker.py','observation.py','contract.json'):
        sources[str(baseline/name)] = sha(baseline/name)
    for path in [ROOT/'simulation/prepare.py', ROOT/'simulation/teacher_actuator.cpp', ROOT/'simulation/build/libteacher_actuator.so', ROOT/'scripts/capture.py',
                 ROOT.parent/'simulation/generated/go2_converted.sdf', ROOT.parent/'simulation/generated/three_floors.sdf']:
        sources[str(path)] = sha(path)
    return {'schema': 'teacher_continuous_turn_plant_plan/v1', 'speed_mps': float(speed), 'yaw_rate_radps': float(yaw),
            'command_radius_m': float(speed/abs(yaw)), 'repetition': repetition, 'timing': p['timing'], 'fixture': p['fixture'],
            'model_sha256': p['model_sha256'], 'protocol_sha256': sha(HERE/'protocol.json'), 'source_hashes': sources,
            'policy_source_directory': str(baseline), 'uses_navigation_truth': False, 'counts_as_SLAM_navigation': False,
            'acceptance_scope': p['scope']}


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--speed',type=float); ap.add_argument('--yaw',type=float)
    ap.add_argument('--repetition',type=int,default=1); ap.add_argument('--all',action='store_true')
    ap.add_argument('--output-dir',type=Path,default=HERE/'plans'); a=ap.parse_args()
    p=json.loads((HERE/'protocol.json').read_text()); cases=list(itertools.product(p['speeds_mps'],p['yaw_rates_radps'])) if a.all else [(a.speed,a.yaw)]
    if any(v is None or w is None for v,w in cases): ap.error('Specify --all or both --speed and --yaw')
    if not a.output_dir.resolve().is_relative_to(HERE): ap.error('Derived plans must remain in this new directory')
    a.output_dir.mkdir(parents=True,exist_ok=True)
    for v,w in cases:
        plan=build(v,w,a.repetition); name=f"v{v:.1f}_w{'p' if w>0 else 'n'}{abs(w):.1f}_r{a.repetition}.json"
        dest=a.output_dir/name
        if dest.exists(): raise FileExistsError('Never overwrite frozen plan '+str(dest))
        dest.write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n'); print(json.dumps({'plan':str(dest.resolve()),'sha256':sha(dest)}))


if __name__=='__main__': main()
