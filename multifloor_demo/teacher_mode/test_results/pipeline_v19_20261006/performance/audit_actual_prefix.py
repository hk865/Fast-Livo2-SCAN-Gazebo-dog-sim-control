#!/usr/bin/env python3
"""Limited read-only V19 actual prefix timing/lifecycle audit, no navigation gate."""
import argparse
import json
import re
from pathlib import Path
from read_stages import analyze,check,distribution,read_csv,read_summary,COLUMNS,LIFE_COLUMNS,sha

CORE_SHA='f06e68d0dcbcab9f840077c372d0cda2e128f9935c4fe968987784e2124e00d8'


def audit(run):
    run=run.resolve();s=read_summary(run/'fastlivo_debug/pipeline_v19_summary.json')
    rows=read_csv(run/'fastlivo_debug/pipeline_v19_events.csv',COLUMNS)
    life=read_csv(run/'fastlivo_debug/pipeline_v19_lifecycle.csv',LIFE_COLUMNS,('event',))
    stage=analyze(rows,life,s)
    loaded=json.loads((run/'slam_loaded_binary.json').read_text())
    runtime=json.loads((run/'run_result.json').read_text())
    stop=json.loads((run/'pipeline_normal_stop.json').read_text())
    manifest=json.loads((run/'runtime_manifest.json').read_text())
    profile=json.loads((run/'navigation_profile.json').read_text())
    poses=[json.loads(x)for x in(run/'navigation_slam_poses.jsonl').read_text().splitlines()]
    snapshots=json.loads((run/'navigation_source_snapshots.json').read_text())
    checks=stage['checks'].copy();source_rows=[]
    for old,row in snapshots.items():
        if old.endswith(('/LIVMapper.cpp','/runner.cpp','/staged_pipeline.h','/stack.launch.py','/controller.py','/run.py')):
            snap=Path(row['snapshot']).resolve();within=snap.is_relative_to(run/'sources')
            digest=sha(snap)if within and snap.is_file()else None
            source_rows.append({'source':old,'snapshot':str(snap),'within_run_sources':within,'actual_sha256':digest,'expected_sha256':row['sha256'],'matched':digest==row['sha256']})
    checks['same_actual_run_frozen_core_and_source_witness']=check(
        Path(loaded['run']).resolve()==run and loaded['verified'] and loaded['before_physics']
        and loaded['actual_core_sha256']==CORE_SHA and loaded['expected']['core']['sha256']==CORE_SHA
        and len(loaded['loaded_core_paths'])==1 and loaded['loaded_core_paths'][0]==loaded['expected']['core']['path']
        and source_rows and all(x['matched']for x in source_rows)
        and any(x['source'].endswith('/LIVMapper.cpp')for x in source_rows),
        source_snapshots=source_rows,live_runtime_library_not_rehashed=True)
    checks['runtime_and_normal_context_drain_stop_original']=check(
        Path(runtime['run']).resolve()==run and runtime['runtime_error']is None
        and Path(stop['run']).resolve()==run and stop['error']is None
        and stop['signal']==10 and stop['signal_sent'] and stop['normal_completed']
        and stop['summary']==s and stop['summary_sha256']==sha(run/'fastlivo_debug/pipeline_v19_summary.json')
        and stop['gate_errors']==[] and stop['sent_monotonic_wall']<=life[1]['wall_ns']/1e9
        and life[-1]['wall_ns']/1e9<=stop['ended_monotonic_wall'])
    stamps=[x['stamp_ns']for x in poses];ages=[];age_consistency=True
    for x in poses:
        age=(x['callback_ros_clock_ns']-x['stamp_ns'])/1e9
        ages.append(age*1e3);age_consistency=age_consistency and abs(age-x['sim_age_at_callback_s'])<=1e-9
    checks['actual_SLAM_pose_clock_and_original_300ms_age']=check(
        len(poses)>0 and all(b>a for a,b in zip(stamps,stamps[1:]))
        and age_consistency and all(0<=x<=300 for x in ages)
        and {x['frame_id']for x in poses}=={'camera_init'}
        and {x['child_frame_id']for x in poses}=={'demo_slam_body'},
        pose_count=len(poses),original_clock_age_checked=True,
        complete_navigation_source_chain_gate_checked_here=False)
    checks['navigation_source_declarations_privilege_and_TTL_unchanged']=check(
        Path(manifest['run']).resolve()==run and manifest['simulation_only'] and not manifest['real_robot']
        and manifest['navigation_ground_truth_used']is False and manifest['actor_privileged_dimensions']==232
        and profile['pose_cloud_timeout_s']==.3 and profile['cascade']['feedback_ttl_sim_and_wall_s']==.3
        and profile['cascade']['navigation_ground_truth_used']is False,
        declarations_are_not_complete_source_or_actuator_proof=True)
    sync=[]
    for line in(run/'navigation_stack.log').open():
        if '[DEMO_SYNC]'in line:
            m=re.search(r'camera=([\d.]+).*imu_newest=([\d.]+).*complete=(\d)',line)
            if m:sync.append((float(m[1]),float(m[2]),int(m[3])))
    lack=[(a-b)*1e3 for a,b,c in sync if not c]
    metrics={kind:{name:stage['metrics'][kind][name]for name in ('decode_wall','decode_thread_CPU','ready_marker_to_owner_pop_wall','receipt_to_commit_wall','source_header_rate')}for kind in ('imu','lidar','image')}
    sources=['run_result.json','runtime_manifest.json','navigation_profile.json','slam_loaded_binary.json',
             'navigation_source_snapshots.json','pipeline_normal_stop.json','navigation_slam_poses.jsonl',
             'navigation_stack.log','fastlivo_debug/pipeline_v19_summary.json',
             'fastlivo_debug/pipeline_v19_lifecycle.csv','fastlivo_debug/pipeline_v19_events.csv']
    return {'schema':'V19_actual_prefix_independent_timing_lifecycle/v1','run':str(run),
        'status':'PASS_LIMITED_TIMING_LIFECYCLE'if all(c['passed']for c in checks.values())else'FAILED',
        'checks':checks,'metrics':metrics,'actual_pose':{'count':len(poses),'first_stamp_ns':stamps[0]if stamps else None,'last_stamp_ns':stamps[-1]if stamps else None,
            'source_header_Hz':(len(stamps)-1)*1e9/(stamps[-1]-stamps[0])if len(stamps)>1 else None,
            'callback_clock_minus_header_ms':distribution(ages),'source_gap_ms':distribution([(b-a)/1e6 for a,b in zip(stamps,stamps[1:])])},
        'logged_sync':{'rows':len(sync),'complete0':len(lack),'target_exceeds_owner_latest_IMU_more_than_1us':sum(x>.001 for x in lack),'maximum_uncovered_interval_ms':max(lack,default=None),
            'interpretation':'Only current LIO group target coverage in owner buffer, not packet-loss count or alternating VIO phase. Nanosecond floating threshold cases separated.'},
        'actual_navigation_full32_or46_PASS':False,'Sim2Sim_PASS':False,'real_robot_tested':False,
        'source_sha256':{str(run/name):sha(run/name)for name in sources},'reader_sha256':sha(Path(__file__)),
        'original_navigation_and_publication_guards_evaluated_by_separate_original_gate':True}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();value=audit(args.run)
    with args.out.open('x')as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'status':value['status'],'out':str(args.out)}))


if __name__=='__main__':main()
