#!/usr/bin/env python3
"""Read-only drive-window diagnosis; writes new evidence, changes no criterion."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for c in iter(lambda:f.read(1<<20),b''): h.update(c)
    return h.hexdigest()

def rows(p):
    with p.open() as f: return [json.loads(x) for x in f if x.strip()]

def run_diagnostic(run):
    script=ROOT/'scripts/analyze_pid_navigation_v1_receipt_fix.py'
    spec=importlib.util.spec_from_file_location('pid_diagnostic_helpers',script)
    helper=importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    p=json.loads((run/'pid_navigation_protocol.json').read_text())
    pid=rows(run/'navigation_pid_history.jsonl'); ex=rows(run/'telemetry.jsonl')
    commands=rows(run/'navigation_command_history.jsonl')
    byseq={x['sequence']:x for x in commands}
    with np.load(run/'pid_navigation_independent_arrays.corrected_v1.npz') as z:
        a={k:z[k] for k in z.files}
    nt=a['native_world_time_s']; pt=np.array([x['compute_ros_clock_ns']/1e9 for x in pid])
    ni,nv=helper.causal_index(pt,nt,.3)
    et=a['execution_world_time_s']; ei,ev=helper.causal_index(et,nt,.020001)
    wp=np.array([pid[i]['waypoint_index'] for i in ni])
    dm=a['drive_mask']; rot=helper.rotation(a['native_quaternion_wxyz'])
    native_cmd=a['execution_command'][ei]
    worldcmd=np.einsum('nij,nj->ni',rot,native_cmd)
    windows=[]; groups=[]
    for leg in sorted(set(wp[dm])):
        vertices=a['planned_world_polyline']; d=vertices[leg+1,:2]-vertices[leg,:2]
        d=d/np.linalg.norm(d); mask=dm&(wp==leg)
        v=a['native_world_COM_velocity'][:,:2]@d; c=worldcmd[:,:2]@d
        for i in np.flatnonzero(mask):
            j=np.searchsorted(nt,nt[i]-p['route']['rolling_window_s'])
            if nt[i]-nt[j]<p['route']['rolling_window_s']-p['physical']['native_sample_gap_max_s'] or not mask[j:i+1].all(): continue
            mean=float(v[j:i+1].mean())
            if mean>p['route']['minimum_projected_world_forward_window_mean_mps']: continue
            row={'waypoint_index':int(leg),'start_world_s':float(nt[j]),'end_world_s':float(nt[i]),
                 'actual_PID_mode':pid[ni[i]]['mode'],'native_samples':int(i-j+1),
                 'physical_projected_COM_mean_mps':mean,'actor_command_world_projected_mean_mps':float(c[j:i+1].mean())}
            windows.append(row)
            if not groups or leg!=groups[-1]['waypoint_index'] or row['end_world_s']-groups[-1]['last_failing_window_end_world_s']>p['physical']['native_sample_gap_max_s']:
                groups.append({'waypoint_index':int(leg),'first_window_start_world_s':row['start_world_s'],
                               'first_failing_window_end_world_s':row['end_world_s'],
                               'last_failing_window_end_world_s':row['end_world_s'],'failing_windows':1})
            else: groups[-1]['last_failing_window_end_world_s']=row['end_world_s']; groups[-1]['failing_windows']+=1
    traces=[]
    for k,g in enumerate(groups):
        lo=g['first_window_start_world_s']; hi=g['last_failing_window_end_world_s']; selected=[]
        for r in ex:
            t=r['world_sim_time']
            if not lo<=t<=hi: continue
            ix,valid=helper.causal_index(pt,np.array([t]),.3)
            pi=pid[ix[0]] if valid[0] else None
            source=byseq.get(r.get('navigation_envelope',{}).get('sequence'))
            selected.append(r)
            traces.append({'group':k,'world_sim_time':t,'policy_sim_time':r['sim_time'],
                'physics_state_world_time':r.get('state_physics_world_time'),
                'waypoint_index':pi.get('waypoint_index') if pi else None,'actual_PID_mode':pi.get('mode') if pi else None,
                'PID_sequence':pi.get('sequence') if pi else None,'raw_SLAM_header_ns':pi.get('control_pose_stamp_ns') if pi else None,
                'PID_desired_body_command':pi.get('desired_body_command') if pi else None,
                'PID_prepared_after_slew_command':pi.get('prepared_after_slew_command') if pi else None,
                'PID_after_slew_command':pi.get('command_after_slew') if pi else None,
                'actual_source_envelope_sequence':r.get('navigation_envelope',{}).get('sequence'),
                'source_raw_requested_body_command':source.get('requested') if source else None,
                'source_actual_command':source.get('command') if source else None,
                'source_healthy':source.get('healthy') if source else None,'source_reason':source.get('reason') if source else None,
                'worker_requested_body_command':r.get('requested'),'actor_body_command':r.get('command'),
                'actor_inferred_this_frame':r.get('actor_inferred_this_frame'),'command_expired':r.get('command_expired'),
                'consumer_reason':r.get('command_reason'),'physical_body_COM_velocity':r.get('body_lin_vel'),
                'physical_body_gyro':r.get('body_ang_vel'),'position':r.get('position'),
                'PID_raw_SLAM_goal_error_m':float(np.linalg.norm(np.asarray(pi['goal'])[:2]-np.asarray(pi['control_pose'])[:2])) if pi else None})
        reasons={}
        for r in selected:
            if r.get('command_expired'): reasons[r.get('command_reason','missing')]=reasons.get(r.get('command_reason','missing'),0)+1
        g['policy_rows']=len(selected); g['protected_zero_rows']=sum(bool(r.get('command_expired')) for r in selected)
        g['protected_reasons']=reasons
        g['interpretation']='Diagnostic only: first movement or resumed movement, approach or protection must be distinguished from these actual rows; no causal policy-failure attribution from speed alone.'
    archive=ROOT/'test_results/pid_navigation_analyzer_erratum_v1'/run.name
    archive.mkdir(parents=True,exist_ok=True)
    originals={'analyze_pid_navigation.py':ROOT/'scripts/analyze_pid_navigation.py',
               'summary_pid_navigation_independent.original.json':run/'summary_pid_navigation_independent.json',
               'independent_freeze_v1.json':ROOT/'tests/pid_navigation/independent_freeze_v1.json'}
    originals_sha={}
    for name,path in originals.items():
        dest=archive/name
        if dest.exists() and sha(dest)!=sha(path): raise ValueError('Existing original archive bytes differ')
        if not dest.exists(): shutil.copyfile(path,dest)
        originals_sha[name]=sha(dest)
    (run/'pid_forward_failed_windows.jsonl').write_text(''.join(json.dumps(x,allow_nan=False)+'\n' for x in windows))
    (run/'pid_forward_failure_trace_50hz.jsonl').write_text(''.join(json.dumps(x,allow_nan=False)+'\n' for x in traces))
    result={'schema':'pid_drive_failure_diagnostic/v1','run':str(run),'status':'diagnostic_only',
        'acceptance_changed':False,'original_receipts_unchanged':True,'original_archive':str(archive),
        'original_archive_sha256':originals_sha,'original_analyzer_sha256':sha(ROOT/'scripts/analyze_pid_navigation.py'),
        'corrected_analyzer_sha256':sha(script),'corrected_summary_sha256':sha(run/'summary_pid_navigation_independent.corrected_v1.json'),
        'diagnostic_script_sha256':sha(__file__),'failure_windows':len(windows),'groups':groups,
        'trace_rows':len(traces),'input_hashes':{n:sha(run/n) for n in ['pid_navigation_protocol.json','navigation_pid_history.jsonl','telemetry.jsonl','navigation_command_history.jsonl','pid_navigation_independent_arrays.corrected_v1.npz']},
        'export_hashes':{n:sha(run/n) for n in ['pid_forward_failed_windows.jsonl','pid_forward_failure_trace_50hz.jsonl']},
        'no_runtime_or_prospective_protocol_modification':True}
    (run/'pid_drive_failure_diagnostic.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('run',type=Path)
    run_diagnostic(parser.parse_args().run.resolve())
