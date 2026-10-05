#!/usr/bin/env python3
"""Retrospective source/signal audit; preserve an explicit run evidence folder."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path): return json.loads(path.read_bytes())
def rows(path): return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]
def identity(value): return tuple(json.dumps(value[k], sort_keys=True) for k in ['pid','starttime_ticks','uid','cmdline'])
def snapshot_equal(a,b): return all(a[k]==b[k] for k in ['sha256','mtime_ns','size_bytes'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fixture',type=Path)
    args=parser.parse_args(); fixture=args.fixture.resolve()
    receipt=read(fixture/'receipt.json'); events=rows(fixture/'events.jsonl'); protocol=read(fixture/'protocol.json'); run=Path(receipt['run'])
    archive=run/'ttl_fixture_evidence'; archive.mkdir(exist_ok=False)
    manifest={}
    for name in ['protocol.json','executed_fixture.py','events.jsonl','receipt.json']:
        shutil.copyfile(fixture/name,archive/name)
        manifest[name]={'source':str(fixture/name),'sha256':sha(fixture/name),'archived_sha256':sha(archive/name)}
    (archive/'copy_sha256_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    trigger=next(e for e in events if e['kind']=='trigger')
    stop=next(e for e in events if e['kind']=='signal_after' and e['signal']=='SIGSTOP')
    resume=next(e for e in events if e['kind']=='signal_after' and e['signal']=='SIGCONT')
    start=receipt['hold_start_after_confirmed_stop_clock_ns']; end=receipt['scheduled_resume_clock_ns']
    target=receipt['target']; frozen=receipt['original_frozen_envelope']; envelope=frozen['original_envelope']
    raw=(json.dumps(envelope,ensure_ascii=False,allow_nan=False,indent=2)+'\n').encode()
    (archive/'frozen_original_navigation_command.json').write_bytes(raw)
    poses=[e['raw_slam_row'] for e in events if e['kind']=='actual_slam_motion']
    original=rows(run/'navigation_slam_poses.jsonl'); indexed={p['stamp_ns']:p for p in original}
    tr=[p for p in poses if trigger['motion_start_ns']<=p['stamp_ns']<=trigger['motion_last_ns']]
    paused=[p for p in poses if start<=p['stamp_ns']<=end]
    clock=[e for e in events if e['kind']=='actual_clock' and start<=e['ros_sim_time_ns']<=end]
    hold_files=[e for e in events if e['kind']=='original_file_envelope' and e['phase']=='holding']
    post=[e for e in events if e['kind']=='original_file_envelope' and e['phase']=='resumed']
    fresh=next((e for e in post if e['original_envelope'].get('healthy') and e['original_envelope']['command'][0]>=.08),None)
    signal_events=[e for e in events if e['kind'] in ['signal_before','signal_after']]
    original_commands=rows(run/'navigation_command_history.jsonl')
    original_envelopes={v['sequence']:v for v in original_commands}
    source_terms=['sequence','monotonic_wall','sim_time','stamp','healthy','command','requested','reason']
    observed=[e['original_envelope'] for e in events if e['kind'] in ['original_file_envelope','frozen_original_envelope']]
    envelope_source_match=all(v['sequence'] in original_envelopes and all(v.get(k)==original_envelopes[v['sequence']].get(k) for k in source_terms) for v in observed)
    runtime=read(run/'runtime_manifest.json')
    checks={
        'protocol_and_executed_source_sha':sha(archive/'protocol.json')==receipt['protocol_sha256'] and sha(archive/'executed_fixture.py')==receipt['fixture_sha256']==protocol['fixture_sha256'],
        'events_sha':sha(archive/'events.jsonl')==receipt['events_sha256'],
        'all_evidence_copies_identical':all(v['sha256']==v['archived_sha256'] for v in manifest.values()),
        'fixture_completed_without_error':receipt['status']=='injection_completed' and receipt['error'] is None and not receipt['logging_errors'],
        'only_exact_bridge_argv_environment':str(run.parent.parent/'navigation/bridge.py') in target['cmdline'] and str(run/'navigation_scope.json') in target['cmdline'] and str(run/'navigation_command.json') in target['cmdline'] and target['environment']=={'ROS_DOMAIN_ID':'79','GZ_PARTITION':'teacher_'+run.name,'DEMO_RUN_DIR':str(run)},
        'all_signal_records_same_pid_starttime_uid_cmdline':all(identity(e['target'])==identity(target) for e in signal_events),
        'actual_SIGSTOP_T_then_SIGCONT_live':stop['target']['state']=='T' and resume['target']['state'] not in ['T','t','Z','X','x'],
        'at_least_ten_actual_clock_seconds_after_stop':end-start>=10000000000 and receipt['actual_scheduled_hold_sim_ns']==end-start,
        'mandatory_same_process_finally_recovery':receipt['recovery']['restored_same_process'] and identity(receipt['recovery']['before'])==identity(target)==identity(receipt['recovery']['after']) and receipt['recovery']['after']['state'] not in ['T','t','Z','X','x'],
        'all_raw_slam_observations_match_original_source':all(indexed.get(p['stamp_ns'])==p for p in poses),
        'trigger_actual_forward_continuous_slam':len(tr)>=3 and tr[-1]['stamp_ns']-tr[0]['stamp_ns']>=299999999 and all(p['body_velocity'][0]>=.03 for p in tr) and max(b['stamp_ns']-a['stamp_ns'] for a,b in zip(tr,tr[1:]))<=200000001,
        'trigger_actual_healthy_forward_file':trigger['command_snapshot']['original_envelope']['healthy'] is True and trigger['command_snapshot']['original_envelope']['command'][0]>=.08,
        'original_envelope_bytes_reconstruct_exactly':hashlib.sha256(raw).hexdigest()==frozen['sha256'] and len(raw)==frozen['size_bytes'],
        'hold_file_snapshots_unchanged':all(snapshot_equal(e,frozen) for e in hold_files),
        'original_envelope_timestamps_not_synthesized':envelope_source_match,
        'actual_slam_and_clock_continue_during_dropout':len(paused)>=90 and len(clock)>=100 and paused[-1]['stamp_ns']-paused[0]['stamp_ns']>=9.8e9,
        'post_resume_first_envelope_zero_unhealthy':bool(post) and post[0]['original_envelope']['healthy'] is False and post[0]['original_envelope']['command']==[0,0,0],
        'same_original_envelope_after_resume_not_accepted_as_fresh':bool(fresh) and fresh['original_envelope']['sim_time']>end/1e9 and fresh['original_envelope']['monotonic_wall']>envelope['monotonic_wall'],
        'completed_run_owned_processes_exit_cleanly':all(p['returncode']==0 for p in runtime['owned_processes']) and runtime['error'] is None,
        'no_command_or_joint_publication_by_fixture':receipt['commands_or_joint_targets_written'] is False and receipt['truth_navigation_used'] is False,
    }
    result={'schema_version':1,'scope':'Independent actual dropout/source/signal audit only; physical stop/recovery checked separately','status':'verified' if all(checks.values()) else 'failed','checks':checks,
            'run':str(run),'target_pid':target['pid'],'target_starttime_ticks':target['starttime_ticks'],'uid':target['uid'],
            'hold_actual_sensor_gate_clock_s':[start/1e9,end/1e9],'hold_duration_sim_s':(end-start)/1e9,'trigger_original_slam_stamps_s':[tr[0]['stamp_ns']/1e9,tr[-1]['stamp_ns']/1e9],
            'original_frozen_file':{'sha256':frozen['sha256'],'mtime_ns':frozen['mtime_ns'],'bytes':frozen['size_bytes'],'sequence':envelope['sequence'],'sim_time_s':envelope['sim_time'],'monotonic_wall':envelope['monotonic_wall'],'stamp':envelope['stamp']},
            'clock_envelope_phase_difference_s':envelope['sim_time']-start/1e9,
            'during_pause':{'raw_slam_samples':len(paused),'actual_clock_samples':len(clock),'maximum_slam_stamp_gap_s':max(b['stamp_ns']-a['stamp_ns'] for a,b in zip(paused,paused[1:]))/1e9,'maximum_gate_clock_gap_s':max(b['ros_sim_time_ns']-a['ros_sim_time_ns'] for a,b in zip(clock,clock[1:]))/1e9,'logged_file_snapshot_changes':sum(not snapshot_equal(e,frozen) for e in hold_files)},
            'first_post_resume_original_envelope':post[0]['original_envelope'] if post else None,'first_post_resume_healthy_forward_original_envelope':fresh['original_envelope'] if fresh else None,
            'run_owned_exit_codes':{p['role']:p['returncode'] for p in runtime['owned_processes']},
            'source_hashes':{name:sha(run/name) for name in ['navigation_slam_poses.jsonl','navigation_command_history.jsonl','navigation_sensor_gate_history.jsonl','runtime_manifest.json','navigation_scope.json']},
            'evidence_copy_manifest_sha256':sha(archive/'copy_sha256_manifest.json'),'analyzer_sha256':sha(Path(__file__)),
            'limits':['The producer source uses a sensor-gate clock lagging the file envelope45ms; sourceTTL must use original7.475s, not STOPgate7.43s',
                      'StateT and same kernel identity record actual suspension/recovery; no other live signal was sent by this audit',
                      'Filebyte/mtime identity is enforced by the frozen helper everypoll; logs store changes, not duplicate every unchanged read',
                      'Sourced SLAMvelocity is not independent physical stop truth; no physical/nav pass derived here']}
    out=archive/'independent_fixture_audit.json';out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    shutil.copyfile(__file__,archive/'executed_independent_audit.py')
    print(json.dumps({'report':str(out),'status':result['status'],'failed_checks':[k for k,v in checks.items() if not v]}))


if __name__=='__main__':main()
