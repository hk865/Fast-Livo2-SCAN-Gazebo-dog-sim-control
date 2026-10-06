#!/usr/bin/env python3
"""Descriptive, source-bound SLAM-only replay summaries; no speed PASS gate."""
import argparse
import csv
import json
import math
import struct
from pathlib import Path
from read_stages import distribution, sha

NAMES = ['Process2', 'undistort', 'deskew_points', 'LIO_residual_query',
         'LIO_StateEst', 'map_update', 'VIO_processFrame', 'output_conversion_publish',
         'owner_loop_spin_or_commit', 'old_lidar_callback', 'imu_callback',
         'image_callback', 'old_callback_preprocess', 'sync_packages',
         'handleLIO', 'handleVIO', 'map_build']


def require(value, reason):
    if not value: raise ValueError(reason)


def boundary(run):
    d = run/'fastlivo_diagnostics'
    stats = json.loads((d/'writer_stats.json').read_text())
    require(stats['schema'] == 'fastlivo_diagnostics/v1' and stats['final']
            and stats['attempted'] == stats['written'] and stats['dropped'] == 0
            and not stats['writer_io_failed'], 'Diagnostic writer incomplete')
    totals = [[0]*5 for _ in NAMES]
    windows = [[] for _ in NAMES]
    count = 0
    with (d/'records.bin').open('rb') as f, (d/'index.csv').open() as idx:
        require(f.read(16) == b'FLIVODIAG0001LE\0', 'Foreign binary magic')
        for row in csv.DictReader(idx):
            if int(row['kind']) != 300: continue
            require(int(row['n_values']) == 90, 'Foreign kind300 payload')
            f.seek(int(row['offset']))
            hdr = struct.unpack('<7Q', f.read(56))
            require(hdr == tuple(int(row[k]) for k in ('kind','sequence','stamp_ns','stage','iteration','level','n_values')), 'Index/body mismatch')
            val = struct.unpack('<90d', f.read(720))
            require(val[0] == 1 and val[4] == 17 and all(math.isfinite(x) and x >= 0 and x == int(x) for x in val), 'Foreign boundary version/noninteger')
            require(val[2] <= val[3], 'Boundary window backwards')
            count += 1
            for i in range(17):
                c = list(map(int, val[5+i*5:10+i*5]))
                require(c[4] == 0, 'Invalid producer clock sample')
                totals[i] = [a+b for a,b in zip(totals[i], c)]
                if c[0]: windows[i].append([c[j]/c[0]/1e6 for j in (1,2,3)])
    require(count > 0, 'No boundary windows')
    result = {}
    for name, total, window in zip(NAMES, totals, windows):
        result[name] = {'calls_in_completed_windows':total[0], 'invalid':total[4],
            'mean_wall_ms_per_call':total[1]/total[0]/1e6 if total[0] else None,
            'mean_owner_thread_CPU_ms_per_call':total[2]/total[0]/1e6 if total[0] else None,
            'mean_process_CPU_ms_per_call':total[3]/total[0]/1e6 if total[0] else None,
            'window_mean_wall_ms_per_call':distribution([x[0] for x in window]),
            'window_mean_owner_thread_CPU_ms_per_call':distribution([x[1] for x in window])}
    return {'producer':'kind300/v1,17-boundary sums in completed ~1s owner windows',
        'rows':count,'boundaries':result,
        'tail_after_last_completed_window_not_flushed':True,
        'parent_child':'query within StateEst within handleLIO; Process2 contains undistort/deskew; handleVIO contains processFrame; do not add nested intervals',
        'stage8':'serial spin_some includes inline admission/decode; staged owner commit excludes decoder work, so this is a moved-work interval, not whole-chain speed',
        'stage12':'predecode now occurs outside old callback; absent interval is N/A, not zero compute',
        'process_CPU':'CPU consumed by every process thread during a boundary, may include overlapping decoder/logger work; not exclusive stage cost',
        'wall_minus_CPU_is_communication':False,
        'source_sha256':{str(d/p):sha(d/p) for p in ('index.csv','writer_stats.json')}}


def summarize(run):
    run = run.resolve()
    plan = json.loads((run/'PLAN.json').read_text())
    runtime = json.loads((run/'RUNTIME.json').read_text())
    stage = json.loads((run/'STAGE_RECEIPT.json').read_text())
    headers = json.loads((run/'INPUT_HEADER_MATCH.json').read_text())
    require(plan['schema']=='V19_standalone_SLAM_replay_plan/v1' and Path(plan['out']).resolve()==run, 'Foreign run plan')
    require(runtime['error'] is None and runtime['all_owned_processes_clean'], 'Runtime failed')
    for p, h in stage['input_sha256'].items(): require(sha(p)==h, 'Changed producer stage input')
    poses = [json.loads(x) for x in (run/'shadow_poses.jsonl').read_text().splitlines()]
    receipt = json.loads((run/'OBSERVER_RECEIPT.json').read_text())
    require(len(poses)==receipt['pose_count'] and [p['sequence'] for p in poses]==list(range(1,len(poses)+1)), 'Observer missing rows')
    stamps = [p['source_ns'] for p in poses]
    gaps = [(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]
    ages = [(p['latest_observed_clock_ns']-p['source_ns'])/1e6 for p in poses if p['latest_observed_clock_ns'] is not None]
    samples = json.loads((run/'PROCESS_SAMPLES.json').read_text())
    by_thread = {}
    for sample in samples['samples']:
        for tid, r in sample['owned_roles']['mapping']['threads'].items():
            by_thread.setdefault(tid, []).append((sample['wall_ns'],r))
    cpu = {}
    for tid, values in by_thread.items():
        first,last=values[0],values[-1]
        ticks=(last[1]['utime_ticks']+last[1]['stime_ticks'])-(first[1]['utime_ticks']+first[1]['stime_ticks'])
        seconds=(last[0]-first[0])/1e9
        cpu[tid]={'name':last[1]['name'],'sampled_CPU_seconds':ticks/samples['CLK_TCK'],
            'sampled_wall_seconds':seconds,'one_CPU_utilization_percent':100*ticks/samples['CLK_TCK']/seconds if seconds else None,
            'observed_CPU_ids':sorted({r['processor'] for _,r in values}),
            'voluntary_context_switch_delta':last[1]['voluntary_ctxt_switches']-first[1]['voluntary_ctxt_switches'],
            'involuntary_context_switch_delta':last[1]['nonvoluntary_ctxt_switches']-first[1]['nonvoluntary_ctxt_switches']}
    return {'schema':'V19_descriptive_SLAM_only_replay_summary/v1','run':str(run),
        'mode':plan['mode'],'rate':plan['rate'],'stage_status':stage['status'],
        'headers':headers,'strict_final_counters':stage['checks']['whole_lifecycle_no_cancel_reject_missing']['final_counters'],
        'ingress_metrics':stage['metrics'],'boundary':boundary(run),
        'odometry':{'pose_count':len(poses),'first_source_ns':stamps[0] if stamps else None,
            'last_source_ns':stamps[-1] if stamps else None,'source_Hz':(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else None,
            'backward_transitions':sum(x<0 for x in gaps),'source_gap_ms':distribution(gaps),
            'latest_observer_clock_minus_pose_header_ms':distribution(ages),
            'negative_observer_age_count':sum(x<0 for x in ages),
            'age_limitation':'Includes observer DDS scheduling; latest received observer clock is not an estimator clock snapshot nor truth pose. Negative values are preserved.'},
        'sampled_mapper_threads':cpu,'same_estimator_output_or_timer_order':'UNVERIFIED',
        'general_statistical_speed_PASS':False,'actual_navigation_or_Sim2Sim_PASS':False,
        'source_sha256':{str(run/p):sha(run/p) for p in ('PLAN.json','RUNTIME.json','LOADED_BINARY.json','STAGE_RECEIPT.json','INPUT_HEADER_MATCH.json','OBSERVER_RECEIPT.json','PROCESS_SAMPLES.json','shadow_poses.jsonl')},
        'reader_sha256':sha(Path(__file__))}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    with args.out.open('x') as f:json.dump(summarize(args.run),f,indent=2,allow_nan=False);f.write('\n')


if __name__=='__main__':main()
