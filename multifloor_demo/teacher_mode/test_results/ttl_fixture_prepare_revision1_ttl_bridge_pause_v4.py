#!/usr/bin/env python3
"""One future run: pause its own command-file bridge, then reliably resume it.

No ROS, actor, truth telemetry, command writes or process-group signals.
The protocol is frozen before execution; physical stopping is audited elsewhere.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
RULES = {'label': 'slam_scan_ttl_drop_v4', 'wait_wall_s': 180., 'hold_sim_ns': 10000000000,
         'clock_stall_wall_s': 60., 'motion_dwell_ns': 300000000, 'pose_gap_max_ns': 200000000,
         'forward_body_velocity_mps': .03, 'healthy_forward_command_mps': .08,
         'trigger_source_age_wall_s': .3, 'trigger_source_age_sim_s': .3,
         'clock_phase_tolerance_s': .05, 'post_resume_sim_ns': 3000000000,
         'post_resume_max_wall_s': 30., 'poll_wall_s': .02}


def sha(raw): return hashlib.sha256(raw).hexdigest()
def read(path): return json.loads(path.read_bytes())
def atomic(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def proc(pid):
    path = Path('/proc') / str(pid)
    tail = (path/'stat').read_text().rsplit(')', 1)[1].split()
    argv = [s.decode() for s in (path/'cmdline').read_bytes().split(b'\0') if s]
    env = dict(s.split(b'=', 1) for s in (path/'environ').read_bytes().split(b'\0') if b'=' in s)
    return {'pid': pid, 'ppid': int(tail[1]), 'state': tail[0], 'starttime_ticks': int(tail[19]),
            'uid': path.stat().st_uid, 'cmdline': argv,
            'environment': {k: env.get(k.encode(), b'').decode() for k in ['ROS_DOMAIN_ID','GZ_PARTITION','DEMO_RUN_DIR']}}


def same(a, b):
    return all(a[k] == b[k] for k in ['pid', 'starttime_ticks', 'uid', 'cmdline'])


def bridge_args(argv, run):
    script = str(ROOT/'navigation/bridge.py')
    return script in argv and any(argv[i:i+2] == ['--acceptance', str(run/'navigation_scope.json')] for i in range(len(argv))) and any(argv[i:i+2] == ['--command-file', str(run/'navigation_command.json')] for i in range(len(argv)))


def unique_bridge(run):
    candidates = []
    for p in Path('/proc').iterdir():
        if not p.name.isdecimal(): continue
        try:
            item = proc(int(p.name))
            if bridge_args(item['cmdline'], run): candidates.append(item)
        except (OSError, ValueError, UnicodeError): continue
    if len(candidates) != 1: raise RuntimeError(f'Expected exactly one run-owned navigation bridge; got {len(candidates)}')
    item = candidates[0]
    if item['uid'] != os.geteuid() or item['state'] in ['T','t','Z']: raise RuntimeError('Bridge uid/state is ineligible')
    if item['environment'] != {'ROS_DOMAIN_ID':'79','GZ_PARTITION':'teacher_'+run.name,'DEMO_RUN_DIR':str(run)}:
        raise RuntimeError('Bridge run/ROS/partition environment differs')
    parent = item['ppid']
    for _ in range(16):
        a = proc(parent)
        if str(ROOT/'navigation/stack.launch.py') in a['cmdline'] and 'run_dir:='+str(run) in a['cmdline']:
            item['navstack_ancestor'] = {k:a[k] for k in ['pid','starttime_ticks','cmdline']}
            return item
        parent = a['ppid']
        if parent <= 1: break
    raise RuntimeError('Bridge has no matching run-owned navstack ancestor')


def signal_same(original, fd, sig, reader=proc, sender=signal.pidfd_send_signal):
    current = reader(original['pid'])
    if not same(original, current): raise RuntimeError('PID identity changed; refuse signal')
    sender(fd, sig, None, 0)  # pidfd pins the original process even across PID reuse.
    return current


class PoseTail:
    def __init__(self): self.offset=0; self.partial=b''; self.first=None; self.last=None; self.latest=None
    def consume(self, path):
        if not path.exists(): return []
        with path.open('rb') as stream:
            stream.seek(self.offset); block=stream.read(1024*1024); self.offset=stream.tell()
        parts=(self.partial+block).split(b'\n'); self.partial=parts.pop()
        if len(self.partial)>1024*1024: raise RuntimeError('SLAM record exceeds input bound')
        return [json.loads(p) for p in parts if p.strip()]
    def observe(self, row):
        stamp=row.get('stamp_ns'); v=row.get('body_velocity'); self.latest=row
        valid=isinstance(stamp,int) and isinstance(v,list) and len(v)==3 and all(isinstance(x,(float,int)) and math.isfinite(x) for x in v)
        valid=valid and row.get('frame_id')=='camera_init' and row.get('child_frame_id')=='demo_slam_body' and v[0]>=RULES['forward_body_velocity_mps']
        if not valid or self.last is not None and (stamp<=self.last or stamp-self.last>RULES['pose_gap_max_ns']+1):
            self.first=None; self.last=None
        if valid:
            if self.first is None: self.first=stamp
            self.last=stamp
        return valid
    def moving(self): return self.first is not None and self.last-self.first>=RULES['motion_dwell_ns']-1


def file_snapshot(path):
    raw=path.read_bytes(); stat=path.stat(); data=json.loads(raw)
    return {'sha256':sha(raw),'mtime_ns':stat.st_mtime_ns,'size_bytes':len(raw),'original_envelope':data}


def trigger(tail, gate, envelope, now, run):
    if not tail.moving() or gate.get('ready') is not True: return False
    ns=gate.get('ros_sim_time_ns'); pose=tail.latest
    if not isinstance(ns,int) or not 0<=now-gate.get('monotonic_wall',-math.inf)<=.3: return False
    if not 0<=now-pose.get('received_monotonic_wall',-math.inf)<=.3: return False
    if not -.05<=(ns-pose['stamp_ns'])/1e9<=.3: return False
    command=envelope.get('command',[])
    return (envelope.get('healthy') is True and envelope.get('source')=='scan_slam' and envelope.get('mode')=='scan_slam'
            and envelope.get('acceptance',{}).get('run_dir')==str(run) and envelope.get('observation_source',{}).get('navigation_ground_truth_used') is False
            and len(command)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in command) and command[0]>=.08
            and 0<=now-envelope.get('monotonic_wall',-math.inf)<=.3 and -.05<=ns/1e9-envelope.get('sim_time',-math.inf)<=.3)


def self_test():
    tail=PoseTail()
    base={'frame_id':'camera_init','child_frame_id':'demo_slam_body','body_velocity':[.04,0,0]}
    for ns in [1000000000,1100000000,1200000000]: tail.observe({**base,'stamp_ns':ns})
    assert not tail.moving()
    tail.observe({**base,'stamp_ns':1300000000}); assert tail.moving()
    tail.observe({**base,'stamp_ns':1700000000}); assert not tail.moving()
    tail.observe({**base,'stamp_ns':1800000000,'body_velocity':[0,.2,0]}); assert not tail.moving()
    run=Path('/safe/new-run'); argv=['python3',str(ROOT/'navigation/bridge.py'),'--acceptance',str(run/'navigation_scope.json'),'--command-file',str(run/'navigation_command.json')]
    assert bridge_args(argv,run) and not bridge_args(argv,Path('/safe/other-run'))
    a={'pid':123,'starttime_ticks':456,'uid':1000,'cmdline':argv}; signals=[]
    signal_same(a,99,signal.SIGSTOP,reader=lambda _:a,sender=lambda fd,sig,info,flags:signals.append(sig))
    signal_same(a,99,signal.SIGCONT,reader=lambda _:a,sender=lambda fd,sig,info,flags:signals.append(sig))
    assert signals==[signal.SIGSTOP,signal.SIGCONT]
    try: signal_same(a,99,signal.SIGCONT,reader=lambda _:{**a,'starttime_ticks':457},sender=lambda *args:signals.append('unsafe'))
    except RuntimeError: pass
    else: raise AssertionError('Reused PID was not rejected')
    assert 'unsafe' not in signals
    for ns in [1000000000,1100000000,1200000000,1300000000]: tail.observe({**base,'stamp_ns':ns,'received_monotonic_wall':8.99})
    gate={'ros_sim_time_ns':1300000000,'monotonic_wall':8.99,'ready':True}
    envelope={'healthy':True,'source':'scan_slam','mode':'scan_slam','acceptance':{'run_dir':str(run)},
              'observation_source':{'navigation_ground_truth_used':False},'command':[.1,0,0],'monotonic_wall':8.99,'sim_time':1.3}
    assert trigger(tail,gate,envelope,9.,run)
    assert not trigger(tail,gate,{**envelope,'command':[.07,0,0]},9.,run)
    assert not trigger(tail,gate,{**envelope,'healthy':False},9.,run)
    assert not trigger(tail,gate,envelope,10.,run)
    print(json.dumps({'self_test':'passed','live_signals_sent':0,'checks':['stamp dwell/gaps','forward velocity only','healthy forward/fresh trigger','exact run argv','mock STOP/CONT','PID reuse refusal']}))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label',default=RULES['label'])
    parser.add_argument('--output',type=Path)
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args()
    if args.self_test: self_test(); return 0
    protocol_path=Path(__file__).with_name('ttl_bridge_pause_protocol_v4.json'); protocol_raw=protocol_path.read_bytes(); protocol=json.loads(protocol_raw)
    if protocol['rules']!=RULES or protocol['fixture_sha256']!=sha(Path(__file__).read_bytes()) or args.label!=RULES['label']: raise RuntimeError('Frozen protocol/script/label mismatch')
    if args.output is None: parser.error('--output is required: a new teacher_mode/test_results directory')
    output=args.output.resolve()
    if ROOT/'test_results' not in output.parents: raise RuntimeError('Output must be a new teacher_mode/test_results subdirectory')
    output.mkdir(parents=True,exist_ok=False); (output/'protocol.json').write_bytes(protocol_raw); (output/'executed_fixture.py').write_bytes(Path(__file__).read_bytes())
    events=(output/'events.jsonl').open('x',buffering=1); memory=[]; logging_errors=[]
    def event(kind,**fields):
        row={'kind':kind,'monotonic_wall':time.monotonic(),**fields}; memory.append(row)
        try: events.write(json.dumps(row,allow_nan=False)+'\n')
        except OSError as error: logging_errors.append(str(error))
    def interrupted(sig,frame): raise RuntimeError('Fixture interrupted by signal '+str(sig))
    watched=[signal.SIGINT,signal.SIGTERM,signal.SIGHUP]
    previous={s:signal.signal(s,interrupted) for s in watched}
    started=time.monotonic(); existing={p.name for p in (ROOT/'runs').iterdir()}; pattern=re.compile(r'^\d{8}_\d{6}_navigation_'+re.escape(args.label)+r'_r\d+_[a-f0-9]{4}$')
    run=None; target=None; fd=None; stopped=False; restored=False; frozen=None; clock_ns=None; hold_start_ns=None; resume_ns=None; last_progress=started; error=None; phase='waiting'; tail=PoseTail(); last_command_hash=None
    event('armed',label=args.label,protocol_sha256=sha(protocol_raw),existing_runs_excluded=len(existing),wait_wall_s=180)
    try:
        while True:
            now=time.monotonic()
            if phase=='waiting' and now-started>180: raise RuntimeError('180s wall arming/trigger wait expired')
            if logging_errors: raise RuntimeError('Fixture logging failed: '+logging_errors[-1])
            if run is None:
                found=[p.resolve() for p in (ROOT/'runs').iterdir() if p.name not in existing and pattern.fullmatch(p.name) and p.is_dir() and not p.is_symlink()]
                if len(found)>1: raise RuntimeError('Multiple newly created label runs; refuse ambiguity')
                if not found: time.sleep(.02); continue
                run=found[0]; event('selected_new_run',run=str(run))
            gate_path=run/'navigation_sensor_gate.json'; command_path=run/'navigation_command.json'
            if not gate_path.exists() or not command_path.exists():
                if phase!='waiting': raise RuntimeError('Actual input file disappeared during pause/recovery')
                if (run/'worker_result.json').exists(): raise RuntimeError('Run ended before actual trigger inputs')
                time.sleep(.02); continue
            gate=read(gate_path); ns=gate.get('ros_sim_time_ns')
            if not isinstance(ns,int): raise RuntimeError('Actual /clock-derived sensor gate stamp invalid')
            if clock_ns is not None and ns<clock_ns: raise RuntimeError('Actual ROS clock reversed')
            if ns!=clock_ns:
                clock_ns=ns; last_progress=now; event('actual_clock',ros_sim_time_ns=ns,source=str(gate_path),gate_monotonic_wall=gate.get('monotonic_wall'),ready=gate.get('ready'))
            for row in tail.consume(run/'navigation_slam_poses.jsonl'):
                tail.observe(row); event('actual_slam_motion',raw_slam_row=row)
            snapshot=file_snapshot(command_path); envelope=snapshot['original_envelope']
            if snapshot['sha256']!=last_command_hash:
                event('original_file_envelope',phase=phase,**snapshot); last_command_hash=snapshot['sha256']
            if phase=='waiting' and trigger(tail,gate,envelope,now,run):
                if read(run/'policy_manifest.json').get('test')!='navigation': raise RuntimeError('Target is not a navigation simulation')
                scope=read(run/'navigation_scope.json')
                if scope.get('run_dir')!=str(run) or scope.get('navigation_ground_truth_used') is not False: raise RuntimeError('Navigation scope differs')
                target=unique_bridge(run); fd=os.pidfd_open(target['pid'],0)
                if not same(target,proc(target['pid'])): raise RuntimeError('Identity changed after pidfd open')
                event('trigger',clock_ns=ns,motion_start_ns=tail.first,motion_last_ns=tail.last,trigger_slam_row=tail.latest,command_snapshot=snapshot,target=target)
                event('signal_before',signal='SIGSTOP',target=proc(target['pid']))
                stopped=True  # Set before syscall: finally resumes even if interrupted immediately after it.
                signal_same(target,fd,signal.SIGSTOP)
                until=time.monotonic()+2
                while proc(target['pid'])['state'] not in ['T','t']:
                    if time.monotonic()>until: raise RuntimeError('SIGSTOP did not produce stopped state')
                    time.sleep(.01)
                event('signal_after',signal='SIGSTOP',target=proc(target['pid']))
                frozen=file_snapshot(command_path); event('frozen_original_envelope',**frozen)
                hold_start_ns=read(gate_path).get('ros_sim_time_ns')
                if not isinstance(hold_start_ns,int) or hold_start_ns<ns: raise RuntimeError('Post-SIGSTOP actual clock is invalid')
                event('hold_start_after_confirmed_stop',clock_ns=hold_start_ns,required_resume_clock_ns=hold_start_ns+10000000000)
                phase='holding'; last_progress=time.monotonic()
            elif phase=='holding':
                if not same(target,proc(target['pid'])) or proc(target['pid'])['state'] not in ['T','t']: raise RuntimeError('Paused process identity/state changed')
                if any(snapshot[k]!=frozen[k] for k in ['sha256','mtime_ns','size_bytes']): raise RuntimeError('Command envelope changed while its sole bridge was paused')
                if ns-hold_start_ns>=10000000000:
                    event('signal_before',signal='SIGCONT',clock_ns=ns,target=proc(target['pid']))
                    signal_same(target,fd,signal.SIGCONT); restored=True; phase='resumed'; resume_ns=ns; resume_wall=time.monotonic()
                    event('signal_after',signal='SIGCONT',clock_ns=ns,target=proc(target['pid']))
                elif now-last_progress>=60: raise RuntimeError('Actual /clock stalled60wall seconds while paused')
            elif phase=='resumed':
                if proc(target['pid'])['state'] in ['T','t']: raise RuntimeError('Bridge remains stopped after SIGCONT')
                if ns-resume_ns>=3000000000 or now-resume_wall>=30:
                    event('post_resume_observation_end',clock_ns=ns,elapsed_sim_ns=ns-resume_ns,latest_original_envelope=snapshot); break
            if (run/'worker_result.json').exists() and phase!='resumed': raise RuntimeError('Run ended before pause/resume completed')
            if (run/'worker_result.json').exists() and phase=='resumed': event('run_ended_after_resume',clock_ns=ns); break
            time.sleep(.02)
    except BaseException as exc:
        error=f'{type(exc).__name__}: {exc}'; event('fixture_failed',phase=phase,error=error)
    finally:
        # Block further interrupts only for this short mandatory recovery/receipt.
        blocked=signal.pthread_sigmask(signal.SIG_BLOCK,watched)
        recovery={'needed':stopped,'restored_same_process':False}
        if stopped and target is not None:
            try:
                before=signal_same(target,fd,signal.SIGCONT); end=time.monotonic()+2
                while proc(target['pid'])['state'] in ['T','t'] and time.monotonic()<end: time.sleep(.01)
                after=proc(target['pid']); recovery.update(before=before,after=after,restored_same_process=same(target,after) and after['state'] not in ['T','t'])
            except BaseException as exc: recovery['error']=f'{type(exc).__name__}: {exc}'
            event('mandatory_finally_SIGCONT',recovery=recovery)
        if fd is not None: os.close(fd)
        completed=error is None and restored and recovery['restored_same_process']
        receipt={'schema_version':1,'status':'injection_completed' if completed else 'failed','scope':'Command-producer dropout only; physical stop/recovery/navigation remain independently unverified',
                 'protocol_sha256':sha(protocol_raw),'fixture_sha256':sha(Path(__file__).read_bytes()),'run':str(run) if run else None,
                 'target':target,'planned_hold_sim_ns':10000000000,'last_clock_ns':clock_ns,'phase':phase,'error':error,'recovery':recovery,
                 'hold_start_after_confirmed_stop_clock_ns':hold_start_ns,'scheduled_resume_clock_ns':resume_ns,
                 'actual_scheduled_hold_sim_ns':resume_ns-hold_start_ns if resume_ns is not None and hold_start_ns is not None else None,
                 'original_frozen_envelope':frozen,'signals_target_only_one_exact_pid':True,'commands_or_joint_targets_written':False,'truth_navigation_used':False,
                 'logging_errors':logging_errors,'events':len(memory),'events_sha256':sha((output/'events.jsonl').read_bytes())}
        atomic(output/'receipt.json',receipt); events.close()
        for sig,handler in previous.items(): signal.signal(sig,handler)
        signal.pthread_sigmask(signal.SIG_SETMASK,blocked)
    print(json.dumps({'receipt':str(output/'receipt.json'),'status':receipt['status'],'run':receipt['run'],'recovery':recovery['restored_same_process']}))
    return 0 if completed else 1


if __name__=='__main__': sys.exit(main())
