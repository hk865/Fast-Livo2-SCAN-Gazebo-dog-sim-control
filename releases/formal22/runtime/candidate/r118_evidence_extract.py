"""Read only CLOSED bounded evidence. No ROS imports, mutation or payload replay.

python r118_evidence_extract.py --run ABS_RUN --cpp ABS_CPP_DIR --output NEW_JSON
The caller must first complete normal shutdown and actual-source/load checks.
Retained-window distributions never claim full-run coverage.
"""
import argparse
from collections import defaultdict, Counter
import hashlib
import json
from pathlib import Path

KEYS=('kind','planner_pid','epoch','generation','revision','attempt_id','traj_id','start_sec','start_nsec')
BINDINGS={}


def read(path,limit=8*1024*1024):
    path=Path(path);before=path.stat()
    if before.st_size>limit:raise ValueError('bounded input limit: '+str(path))
    raw=path.read_bytes();after=path.stat()
    if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):
        raise ValueError('input still changing: '+str(path))
    BINDINGS[str(path.resolve())]=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
    return json.loads(raw)


def rows(data,*names):
    out={}
    for name in names:
        for values in data.get(name,[]):
            row=dict(zip(data['columns'],values))
            seq=row['seq']
            if seq in out and out[seq]!=row:raise ValueError('conflicting retained sequence')
            out[seq]=row
    return sorted(out.values(),key=lambda r:r['seq'])


def stats(values):
    values=sorted(values)
    if not values:return dict(n=0,coverage='no matched retained intervals; not zero latency')
    def q(p):return values[int((len(values)-1)*p)]
    return dict(n=len(values),min_ns=values[0],p50_ns=q(.5),p95_ns=q(.95),p99_ns=q(.99),max_ns=values[-1],
                quantile='lower rank on retained matched rows; not a complete-run quantile')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--cpp',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run=args.run.resolve()
    report=dict(schema='r118_closed_evidence_extract/v1',run=str(run),navigation_PASS=False,
        inference_limits=['publish to take is DDS/executor/callback-group composite, not pure network',
          'take occurs after callback group acquisition; previous wait is unobserved',
          'retained windows are selected/censored; aggregate histograms are separately scoped',
          'callback return is not business acceptance or Actor execution',
          'source/header SIM stamps are never subtracted from MONOTONIC',
          'C++ cached odom age is a cache proxy, not proof of source generation age'])
    cpp=[]; cpp_files=[]
    if args.cpp:
        for path in sorted(args.cpp.glob('r107_scan_transport_cpp_*.json')):
            d=read(path,512*1024)
            if d['run_id']!=run.name:raise ValueError('C++ run mismatch')
            cpp_files.append(dict(path=str(path),identity={k:d.get(k)for k in ('pid','start_ticks','boot_id','epoch')},
                                  recorded=d['recorded'],evicted=d['evicted'],errors=d['errors']))
            for v in d['rows']:
                r=dict(zip(d['columns'],v));r.update(planner_pid=d['pid'],boot_id=d['boot_id'])
                cpp.append(r)
    report['cpp_files']=cpp_files
    python=[]
    for path in sorted(run.glob('r107_scan_transport_python_*.json')):
        d=read(path)
        if d['run_id']!=run.name:raise ValueError('Python run mismatch')
        rr=rows(d,'rows','anomaly_rows');events=defaultdict(dict);reasons=Counter();joins=[]
        pub=defaultdict(list)
        for r in cpp:
            if r['kind']in(1,3):pub[tuple(r[k]for k in KEYS)].append(r)
        for r in rr:
            if r['event_index']>0:events[r['event_index']][r['stage']]=r
        app=[];business=[];commits=[];takecb=[]
        for event,stages in events.items():
            if 1 in stages and 6 in stages:app.append(stages[6]['mono_ns']-stages[1]['original_receive_mono_ns'])
            if 6 in stages and 7 in stages:
                a,b=stages[6],stages[7]
                business.append(b['mono_ns']-a['mono_ns'])
            if 11 in stages and 15 in stages:commits.append(stages[15]['mono_ns']-stages[11]['mono_ns'])
            semantic=stages.get(7)
            if semantic is None:reasons['no_retained_consume_return']+=1;continue
            if semantic['kind']not in(1,3)or semantic['identity_status']!=2:
                reasons['partial_or_unresolved_identity']+=1;continue
            matches=pub[tuple(semantic[k]for k in KEYS)]
            if len(matches)!=1:reasons['publication_missing_or_nonunique']+=1;continue
            c=matches[0];receive=stages.get(1)
            if receive is None:reasons['receive_not_retained']+=1;continue
            if c['boot_id']!=d['identity']['boot_id']:reasons['boot_mismatch']+=1;continue
            fields=('take_seq','take_before_mono','take_after_mono','callback_enter_mono')
            if any(semantic[k]<0 or semantic[k]!=receive[k]for k in fields):
                reasons['event_take_link_missing_or_mismatch']+=1;continue
            if c['publish_returned']!=1:reasons['publish_not_returned']+=1;continue
            before,after=receive['take_before_mono'],receive['take_after_mono']
            if not (c['begin_mono_ns']<=c['end_mono_ns'] and c['begin_mono_ns']<=before<=after<=receive['callback_enter_mono']<=receive['mono_ns']):
                reasons['clock_or_order_mismatch']+=1;continue
            interval=dict(event_index=event,identity=list(tuple(semantic[k]for k in KEYS)),
                publish_begin_to_take_before_ns=before-c['begin_mono_ns'],
                publish_return_to_take_before_ns=before-c['end_mono_ns'],
                take_call_ns=after-before,
                take_to_callback_ns=receive['callback_enter_mono']-after,
                callback_to_receive_ns=receive['mono_ns']-receive['callback_enter_mono'],
                receive_to_consume_ns=None if 6 not in stages else stages[6]['mono_ns']-receive['original_receive_mono_ns'])
            interval['producer_cache_scalar_proxies']={k:c.get(k) for k in (
                'cached_odom_at_publish_SIM_ns','publish_ROS_now_ns','attempt_begin_MONO_ns',
                'cached_odom_at_attempt_SIM_ns')}
            if all(type(c.get(k))is int for k in ('publish_ROS_now_ns','cached_odom_at_publish_SIM_ns')):
                interval['cached_odom_age_at_publish_SIM_ns']=c['publish_ROS_now_ns']-c['cached_odom_at_publish_SIM_ns']
            interval['publish_return_overlaps_take']=c['end_mono_ns']>before
            joins.append(interval)
            takecb.append(interval['take_to_callback_ns'])
        python.append(dict(path=str(path),identity=d['identity'],recorded=d['recorded'],evicted=d['evicted'],
            errors=d['errors'],attempted=d['attempted'],retained_unique_rows=len(rr),
            strict_joins=joins,unmatched=dict(reasons),duplicate_retained_publication_keys=sum(len(v)>1 for v in pub.values()),
            full_run_distributions=d.get('distributions'),stage_counts=d['stage_counts'],
            retained_receive_to_consume=stats(app),retained_business=stats(business),retained_commit=stats(commits),
            retained_take_to_callback=stats(takecb),anomaly_trigger=d.get('anomaly_trigger'),
            internal_mark_cost={k:d.get(k)for k in ('cost_wall_ns','cost_cpu_ns','cost_wall_max_ns')}))
    report['python_scan']=python
    feedback=[]
    for path in sorted(run.glob('r118_feedback_evidence_[0-9]*.json')):
        d=read(path);rr=rows(d,'rows','window_rows','external_window_rows');by_source=defaultdict(dict)
        for r in rr:by_source[r['source_ns']][r['stage']]=r
        chains=[]
        for source,stages in by_source.items():
            if source<0:continue
            row=dict(source_ns=source,retained_stages=sorted(stages))
            if 3 in stages and 4 in stages:row['ready_residence_ns']=stages[4]['mono_ns']-stages[3]['mono_ns']
            if 5 in stages and 6 in stages:
                a,b=stages[5],stages[6];row['commit_wall_ns']=b['mono_ns']-a['mono_ns']
                row['commit_thread_cpu_ns']=b['thread_cpu_ns']-a['thread_cpu_ns'] if a['tid']==b['tid'] else None
                row['accepted_body_cloud_same_source']=b['outcome']==1
            if any(k in stages for k in (5,6,7,8)):chains.append(row)
        feedback.append(dict(path=str(path),identity=d['identity'],recorded=d['recorded'],evicted=d['evicted'],
            dropped_lock_contention=d['dropped_lock_contention'],errors=d['errors'],retained_unique_rows=len(rr),
            full_run_distributions=d['distributions'],stage_counts=d['stage_counts'],
            trigger=d['trigger'],external_trigger=d.get('external_trigger'),chains=chains,
            token_namespaces=d['token_namespaces'],namespace_overflow=d['namespace_overflow'],
            buffer_bytes=d['buffer_bytes'],cost_wall_ns=d['cost_wall_ns'],
            cost_thread_cpu_ns=d['cost_thread_cpu_ns'],cost_wall_max_ns=d['cost_wall_max_ns']))
    report['feedback']=feedback
    worker_file=run/'r93_worker_timeline.json'
    if worker_file.exists():
        d=read(worker_file);report['child_worker']=dict(identity=d['identity'],sequence=d['sequence'],
            evicted=d['evicted'],observation_errors=d['observation_errors'],
            full_finished_local_job_phase_totals=d.get('completed_phase_totals'),
            retained_jobs=d.get('completed'),limitations=d.get('limitations'))
    exit_file=run/'r88_async_physical_exit.json'
    if exit_file.exists():
        d=read(exit_file);report['parent_exit']={k:v for k,v in d.items()if k not in ('r107_scan_transport',)}
    report['still_missing']=['DDS arrival/kernel transport separation',
        'exact SLAM source generation/publish MONOTONIC for same corrected source identity',
        'spline full scoped identity before association; unconsumed String source identity',
        'SCAN original commit callback return to first legal control-use full owner/job chain',
        'bounded windows may not include requested source; absence is censored coverage']
    report['input_bindings']=BINDINGS
    with args.output.open('x')as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(output=str(args.output),scan_files=len(python),feedback_files=len(feedback),
                         cpp_files=len(cpp_files),strict_joins=sum(len(x['strict_joins'])for x in python))))

if __name__=='__main__':main()
