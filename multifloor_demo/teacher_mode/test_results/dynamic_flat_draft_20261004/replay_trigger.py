#!/usr/bin/env python3
"""Read-only causal replay of actual V4 records, not a dynamic test."""
from bisect import bisect_right
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'navigation/dynamic'))
from dynamic_obstacle import Trigger
protocol=json.loads((ROOT/'navigation/dynamic/protocol.json').read_text())
campaign=json.loads((ROOT/'test_results/navigation_v4_independent_campaign/navigation_v4_independent_campaign.json').read_text())
rows=[]
for item in campaign['runs']:
    run=Path(item['run_dir']);anchor=json.loads((run/'navigation_anchor.json').read_text())
    poses=[json.loads(x)for x in(run/'navigation_slam_poses.jsonl').read_text().splitlines()];pw=[x['received_monotonic_wall']for x in poses]
    nav=[json.loads(x)for x in(run/'navigation_status.jsonl').read_text().splitlines()];nw=[x['monotonic_wall']for x in nav]
    trigger=Trigger(protocol);first=None;missed=None
    for line in(run/'telemetry.jsonl').read_text().splitlines():
        t=json.loads(line);wall=t['navigation_envelope']['read_monotonic_wall'];sim=t['world_sim_time']
        pi=bisect_right(pw,wall)-1;ni=bisect_right(nw,wall)-1
        if pi<0 or ni<0:continue
        if trigger.observe(poses[pi],t,nav[ni],anchor,sim,wall):
            first={'sim_s':sim,'slam_along_m':trigger.along,'body_forward_mps':poses[pi]['body_velocity'][0],
                'requested_forward_mps':t['requested'][0],'pose_samples':trigger.samples,'pose_span_s':trigger.last-trigger.first};break
        if trigger.missed_window and missed is None:missed={'sim_s':sim,'along_m':trigger.along}
    rows.append({'run_id':run.name,'first_eligible_trigger':first,'missed_window_before_trigger':missed})
result={'schema':1,'status':'offline_replayed','scope':'Original V4 records only; mover never started; not proof of dynamic success',
        'protocol':protocol,'actual_run_causal_trigger_results':rows,'all_three_have_early_trigger':all(x['first_eligible_trigger']for x in rows),
        'starts_ros':False,'starts_simulation':False}
out=Path(__file__).with_name('trigger_replay_receipt.json');out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');print(json.dumps(result,indent=2))
