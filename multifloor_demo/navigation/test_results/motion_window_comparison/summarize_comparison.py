#!/usr/bin/env python3
"""Summarize existing passive outputs; do not fit control or use GT feedback."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from compare_windows import ROOT, OBSERVER, NS, stats

out=Path(sys.argv[1]).resolve()
report=json.loads((out/'result.json').read_text())
all_rows={}
summary=dict(scope=__doc__,windows={},common_available_walk=None,common_valid_walk=None,
             original_observer_source_sha256=report['observer_source_sha256'],
             body_omega_and_world_heading_rate_separate=True,controller_output_generated=False)
for label in report['windows']:
    records=[json.loads(line) for line in (out/f'snapshots_{label}.jsonl').open()]
    all_rows[label]={x['slam']['stamp_ns']:x for x in records}
    c=Counter();rejected_gt=Counter();quality=[];opposite_quality=[]
    for x in records:
        s=x['slam'];g=x['GT_evaluation_only'];cq=x['command_window_quality']
        if not s.get('available') or s.get('execution_mode')!='walk':continue
        c['available_walk']+=1
        rs=s['fit_residuals']
        c['position_residual_over_gate']+=int(rs['position_rms_m']>.02)
        c['angular_or_heading_residual_over_gate']+=int(max(rs['rotation_rms_rad'],rs['heading_rms_rad'])>.04)
        if not s['valid'] and g and g.get('available'):
            rg=g['fit_residuals']
            rejected_gt['same_window_pair']+=1
            rejected_gt['independent_GT_also_over_same_fit_gates']+=int(rg['position_rms_m']>.02 or max(rg['rotation_rms_rad'],rg['heading_rms_rad'])>.04)
        if s['valid'] and cq:
            latest=cq['latest_applied_command'];quality.append([max(0,.12-latest[0]),max(0,.08-abs(latest[2]))])
            c['valid_latest_forward_ge_118']+=int(latest[0]>=.118)
            c['valid_latest_yaw_ge_078']+=int(abs(latest[2])>=.078)
        if x['classifications']['opposite_body_GT_confirmed_consistent_command_sign']:
            latest=cq['latest_applied_command']
            c['consistent_opposite_latest_yaw_ge_078']+=int(abs(latest[2])>=.078)
            opposite_quality.append([max(0,.12-latest[0]),max(0,.08-abs(latest[2]))])
    summary['windows'][label]=dict(counts=dict(c),fit_rejected_independent_GT=dict(rejected_gt),
        valid_latest_cap_headroom_vx_yaw=stats(quality),consistent_opposite_latest_cap_headroom_vx_yaw=stats(opposite_quality))

for valid,field in ((False,'common_available_walk'),(True,'common_valid_walk')):
    common=set.intersection(*[{t for t,x in records.items() if x['slam'].get('available') and x['slam'].get('execution_mode')=='walk' and (not valid or x['slam']['valid'])} for records in all_rows.values()])
    values={}
    for label,records in all_rows.items():
        samples=[]
        for t in sorted(common):
            s=records[t]['slam'];g=records[t]['GT_evaluation_only']
            if not g or not g.get('available'):continue
            samples.append([s['window_body_twist']['linear'][0]-g['window_body_twist']['linear'][0],
                s['window_body_twist']['angular'][2]-g['window_body_twist']['angular'][2],
                s['heading_rate_world']-g['heading_rate_world']])
        values[label]=stats(samples)
    summary[field]=dict(count=len(common),SLAM_minus_GT_vx_bodyomega_worldheading=values,
                       caution='Common endpoints only; windows still measure different historical intervals. No plant-gain comparison.')

episodes=json.loads((out/'episodes_0.6s.json').read_text())
long=[e for e in episodes['opposite_body_GT_confirmed_consistent_command_sign']['episodes'] if e['at_least_one_nominal_gait_cycle']]
evidence=[]
for e in long:
    same=[x for t,x in all_rows['0.6s'].items() if e['start_stamp_ns']<=t<=e['last_stamp_ns']]
    evidence.append(dict(episode=e,rows=same))
summary['complete_nominal_cycle_opposite_evidence']=evidence

# Record the nominal-cycle assumption and distinguish archive/config from a
# current external CHAMP header that was never in the run's source tarball.
header=ROOT.parent/'go2_sim_control/src/unitree_go2_ros2/champ/include/champ/leg_controller/phase_generator.h'
runtime=json.loads((ROOT/'runs'/report['run']/'runtime_manifest.json').read_text())
cmdlib=Path(runtime['artifacts']['champ_base_controller']['path'])
summary['nominal_cycle_provenance']=dict(archived_stance_duration_s=.35,
    phase_generator_current_source_path=str(header),phase_generator_current_source_sha256=hashlib.sha256(header.read_bytes()).hexdigest(),
    source_snippet='float swing_phase_period = 0.25f * SECONDS_TO_MICROS;',
    header_captured_in_original_source_tar=False,
    full15_CHAMP_library_sha256=runtime['artifacts']['champ_base_controller']['sha256'],
    current_CHAMP_library_sha256=hashlib.sha256(cmdlib.read_bytes()).hexdigest(),
    current_runtime_matches_full15=hashlib.sha256(cmdlib.read_bytes()).hexdigest()==runtime['artifacts']['champ_base_controller']['sha256'],
    contact_phase_measured=False,nominal_period_only=True)
(out/'analysis_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
print(json.dumps({label:dict(x['counts'],GT_residual_match=x['fit_rejected_independent_GT']) for label,x in summary['windows'].items()},indent=2))
