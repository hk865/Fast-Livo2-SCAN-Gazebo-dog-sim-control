#!/usr/bin/env python3
"""Archived V17 smoke only; read-only, no ROS, process signals or launch."""
from pathlib import Path
import argparse,bisect,collections,csv,hashlib,json,math,statistics,struct
HERE=Path(__file__).resolve().parent
TEACHER=HERE.parents[2]
V17=TEACHER/'navigation/ingress_pipeline_v17'
CRITERION=HERE.parent/'evaluation/PROSPECTIVE_V17_60S_SMOKE_CRITERIA.json'
CRITERION_SHA='85a39bc1adc538e468207fd785348bf32630c66772ebadc985f9fcf096342867'
TIMING_SHA='709d4cf1e906cb869b82411c5a399ac0e85597645e438bd513518404b1cbbb96'
CPP_SHA='39195ef48be4a72a01d988aba8c75d2b3fadb8c71c296d4cb8b6a9999427f727'
COLUMNS=['sequence','kind','source_ns','receipt_wall','decoded_wall','pop_wall','commit_end_wall','receive_tid','owner_tid','decode_cpu_begin_ns','decode_cpu_end_ns','pending_after_pop','bytes_after_pop']
INTS=set(COLUMNS)-{'receipt_wall','decoded_wall','pop_wall','commit_end_wall'}
class Missing(Exception):pass
def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def read_json(path):
    if not Path(path).is_file():raise Missing('Missing '+str(path))
    return json.loads(Path(path).read_text())
def ck(value,**details):return dict(status='unverified'if value is None else'passed'if value else'failed',passed=value,**details)
def overall(checks):
    if any(x['status']=='failed'for x in checks.values()):return'failed'
    return'unverified'if any(x['status']!='passed'for x in checks.values())else'passed'
def distribution(values):
    if not values:return {'count':0,'status':'unavailable','reason':'No observations; missing is not zero'}
    a=sorted(values);pos=(len(a)-1)*.95;i=int(pos)
    return dict(count=len(a),minimum=a[0],median=statistics.median(a),mean=statistics.mean(a),p95=a[i]+(a[min(i+1,len(a)-1)]-a[i])*(pos-i),maximum=a[-1])
def parse_trace(path):
    if not path.is_file():raise Missing('Missing committed ingress trace')
    with path.open()as f:
        r=csv.DictReader(f)
        if r.fieldnames!=COLUMNS:raise ValueError('Foreign or incomplete CSV schema')
        rows=[]
        for row in r:
            if None in row or any(row.get(k)is None for k in COLUMNS):raise ValueError('Truncated CSV row')
            x={}
            for k in COLUMNS:
                if k in INTS:
                    if not row[k].isdigit():raise ValueError('Non-unsigned integer '+k)
                    x[k]=int(row[k])
                else:
                    x[k]=float(row[k])
                    if not math.isfinite(x[k]):raise ValueError('Nonfinite '+k)
            rows.append(x)
    return rows
def parse_summary(path):
    if not path.is_file():raise Missing('No final ingress summary; do not infer cleanup zero')
    d={}
    for line in path.read_text().splitlines():
        if '='not in line:raise ValueError('Bad summary row')
        k,v=line.split('=',1)
        if k in d:raise ValueError('Duplicate summary key')
        d[k]=v
    keys={'schema','accepted','delivered','committed','canceled','rejected','peak_pending','peak_bytes','failure'}
    if set(d)!=keys or d['schema']!='ordered_ingress_v17':raise ValueError('Incomplete or foreign final summary')
    for k in keys-{'schema','failure'}:
        if not d[k].isdigit():raise ValueError('Bad summary integer')
        d[k]=int(d[k])
    return d
def indexed_diagnostics(run):
    binary=run/'fastlivo_diagnostics/records.bin';index=run/'fastlivo_diagnostics/index.csv'
    if not binary.is_file()or not index.is_file():raise Missing('Original raw diagnostic binary/index missing')
    data=collections.defaultdict(list);count=0;offset=16
    with binary.open('rb')as f,index.open()as ix:
        if f.read(16)!=b'FLIVODIAG0001LE\0':raise ValueError('Foreign diagnostic magic')
        for row in csv.DictReader(ix):
            h=tuple(int(row[k])for k in ['kind','sequence','stamp_ns','stage','iteration','level','n_values'])
            if int(row['offset'])!=offset or not 0<=h[-1]<=4194304:raise ValueError('Diagnostic offset/size invalid')
            f.seek(offset);raw=f.read(56)
            if len(raw)!=56 or struct.unpack('<7Q',raw)!=h:raise ValueError('Original index/header mismatch')
            if h[0]in(1,2,3,10,11,12,13,300):
                n=min(h[-1],90 if h[0]==300 else 14);body=f.read(n*8)
                if len(body)!=n*8:raise ValueError('Truncated selected diagnostic')
                values=struct.unpack('<'+str(n)+'d',body)
                if not all(math.isfinite(v)for v in values):raise ValueError('Nonfinite selected diagnostic')
                data[h[0]].append({'header':h,'values':values,'offset':offset,'record_index':count})
            offset+=56+h[-1]*8;count+=1
    if offset!=binary.stat().st_size:raise ValueError('Final indexed binary size mismatch')
    return data,dict(records=count,file_bytes=offset)
def integrity(rows,summary,raw):
    checks={};n=len(rows)
    checks['final_lifecycle_no_loss']=ck(n>0 and summary['accepted']==summary['delivered']==summary['committed']==n and summary['canceled']==summary['rejected']==0 and summary['failure']=='',trace_committed=n,summary=summary,cleanup_exemption_applied=False,missing_count=max(0,summary['accepted']-n))
    checks['ready_FIFO_contiguous_original_order']=ck([r['sequence']for r in rows]==list(range(1,n+1)) and all(r['kind']in(0,1,2,3)for r in rows))
    rxtids={r['receive_tid']for r in rows};ownertids={r['owner_tid']for r in rows}
    checks['single_distinct_RX_and_owner_TID']=ck(len(rxtids)==len(ownertids)==1 and rxtids!=ownertids and all(x>0 for x in rxtids|ownertids),receiver_tids=sorted(rxtids),owner_tids=sorted(ownertids))
    clocks=all(0<=r['receipt_wall']<=r['decoded_wall']<=r['pop_wall']<=r['commit_end_wall']and r['decode_cpu_begin_ns']<=r['decode_cpu_end_ns']for r in rows)
    clocks=clocks and all(b['receipt_wall']>=a['decoded_wall']and b['pop_wall']>=a['commit_end_wall']and b['decode_cpu_begin_ns']>=a['decode_cpu_end_ns']for a,b in zip(rows,rows[1:]))
    checks['receive_decode_pop_commit_causality']=ck(clocks)
    bound=all(0<=r['pending_after_pop']<=511 and 0<=r['bytes_after_pop']<67108864 and ((r['pending_after_pop']==0)==(r['bytes_after_pop']==0))for r in rows)
    peak_consistent=bool(rows)and summary['peak_pending']>=max(r['pending_after_pop']for r in rows)+1 and summary['peak_bytes']>max(r['bytes_after_pop']for r in rows)
    checks['ready_queue_count_and_accounted_byte_bounds']=ck(bound and peak_consistent and summary['peak_pending']<=512 and summary['peak_bytes']<=67108864,byte_accounting_scope='Ready queue; CSV same-lock post-pop snapshot. Per-packet byte sizes not recorded, so no invented exact enqueue byte reconstruction',limits=[512,67108864])
    checks['original_timer_metadata']=ck(all(r['source_ns']==0 and r['decoded_wall']==r['receipt_wall']and r['decode_cpu_begin_ns']==r['decode_cpu_end_ns']for r in rows if r['kind']==0))
    sensors=[r for r in rows if r['kind']in(1,2,3)]
    actual=sorted([r for k in(1,2,3)for r in raw[k]],key=lambda r:r['record_index'])
    expected=[(r['kind'],r['source_ns'],struct.pack('<d',r['receipt_wall']))for r in sensors]
    observed=[(r['header'][0],r['header'][2],struct.pack('<d',r['values'][0]))for r in actual]
    checks['every_committed_sensor_original_integer_header_receipt_order_exact']=ck(expected==observed and all(any(r['kind']==k for r in rows)for k in(1,2,3)),committed_sensor_counts=dict(collections.Counter(r['kind']for r in sensors)),raw_counts={str(k):len(raw[k])for k in(1,2,3)},receipt_rule='IEEE754 exact roundtrip from precision17 CSV; header integer exact; filtered global order exact',sensor_payload_limit='RAW RGB/LiDAR records contain metadata, not complete original image/point bytes; runtime body bit identity is not claimed')
    return checks
def metrics(rows,raw):
    kinds={}
    for k in(0,1,2,3):
        a=[r for r in rows if r['kind']==k]
        kinds[str(k)]={'count':len(a),'receive_to_decoded_wall_ms':distribution([(r['decoded_wall']-r['receipt_wall'])*1000 for r in a]),'decoded_to_pop_including_push_and_FIFO_wall_ms':distribution([(r['pop_wall']-r['decoded_wall'])*1000 for r in a]),'pop_to_commit_end_wall_ms':distribution([(r['commit_end_wall']-r['pop_wall'])*1000 for r in a]),'receive_to_commit_end_wall_ms':distribution([(r['commit_end_wall']-r['receipt_wall'])*1000 for r in a]),'decode_RX_thread_CPU_ms':distribution([(r['decode_cpu_end_ns']-r['decode_cpu_begin_ns'])*1e-6 for r in a])}
        if len(a)>1:
            kinds[str(k)].update(received_wall_hz=(len(a)-1)/(a[-1]['receipt_wall']-a[0]['receipt_wall']),committed_wall_hz=(len(a)-1)/(a[-1]['commit_end_wall']-a[0]['commit_end_wall']))
            if k!=0:kinds[str(k)]['source_header_hz']=(len(a)-1)*1e9/(a[-1]['source_ns']-a[0]['source_ns'])if a[-1]['source_ns']>a[0]['source_ns']else None
    phases={}
    for label,stage,kind in[('LIO',2,13),('VIO',1,12)]:
        attempted=[r for r in raw[kind]if r['header'][3]==stage]
        completed=[r for r in attempted if r['values'][0]==1]
        started=[r for r in raw[10]if r['header'][3]==stage]
        m={'successful_completed_records':len(completed),'skipped_completed_records':sum(r['values'][0]==0 for r in attempted),'started_records':len(started),'completed_wall_hz':None,'completed_source_hz':None,'completion_rule':'Only original diag_stage flag1; flag0 skipped snapshots are not processed updates'}
        if len(completed)>1:
            wall=[r['values'][1]for r in completed];source=[r['header'][2]*1e-9 for r in completed]
            if wall[-1]>wall[0]:m['completed_wall_hz']=(len(wall)-1)/(wall[-1]-wall[0])
            if source[-1]>source[0]:m['completed_source_hz']=(len(source)-1)/(source[-1]-source[0])
        for k in(1,2,3):
            sources=sorted(raw[k],key=lambda r:r['values'][0]);walls=[r['values'][0]for r in sources];ages=[]
            for s in started:
                i=bisect.bisect_right(walls,s['values'][0])-1
                if i>=0:ages.append((sources[i]['header'][2]-s['header'][2])*1e-9)
            m[f'received_kind{k}_header_minus_processing_target_s']=distribution(ages)
        phases[label]=m
    boundary=collections.defaultdict(list)
    for r in raw[300]:
        v=r['values']
        if len(v)!=90 or v[0]!=1 or v[4]!=17:raise ValueError('Foreign kind300 timing layout')
        for stage in range(17):
            calls,wall,cpu,process,invalid=v[5+stage*5:10+stage*5]
            if min(calls,wall,cpu,process,invalid)<0:raise ValueError('Bad boundary counter')
            boundary[str(stage)].append(dict(calls=calls,wall_ns=wall,owner_thread_CPU_ns=cpu,all_process_threads_CPU_ns=process,invalid=invalid))
    labels={8:'owner FIFO commit batch, not old spin_some',9:'owner cloud commit without preprocessing',11:'owner image commit without decoding',12:'N/A_ACTIVE_PREPROCESS_MOVED: empty deferred-error branch; not RX preprocessing timing'}
    return {'per_kind':kinds,'actual_processed_updates_and_received_source_heads':phases,'kind300':{'observations':dict(boundary),'active_labels':labels,'interpretation':'Nested intervals overlap and are not summed; process CPU contains concurrent RX/writer/other threads and is not exclusive owner CPU. Wall minus CPU is not identified communication.'},'source_age_limits':'receive-to-commit wall ages are exact; header-minus-processing-target is latest causally received source head, not current /clock age. No sender transmit clocks or exact enqueue clock; no invented IPC transit/pure FIFO span. Original controller dual TTL is independently read.'}
def scope_checks(run,rows):
    plan=read_json(run/'runtime_plan.json');runtime=read_json(run/'runtime_manifest.json');scope=read_json(run/'navigation_scope.json');snaps=read_json(run/'navigation_source_snapshots.json');loaded=read_json(run/'slam_loaded_binary.json');env=read_json(run/'navigation_stack_effective_environment.json')
    if any(str(Path(d.get(k,'')).resolve())!=str(run)for d,k in[(plan,'run'),(runtime,'run'),(scope,'run_dir')]):raise ValueError('Foreign run identity')
    refs=scope.get('references',{});source=V17/'slam_ws/src/fast_livo2_core/src/LIVMapper.cpp';cpp=snaps.get(str(source),{})
    archive=Path(cpp.get('snapshot','')).resolve();bound=archive.is_relative_to(run/'sources')and archive.is_file()and sha(archive)==CPP_SHA==cpp.get('sha256')==refs.get(str(source))
    binary=plan.get('slam_binary_contract',{});expected_prefix=V17/'slam_ws/install';bc=True
    for key,actualsha,actualpath in [('core','actual_core_sha256','loaded_core_paths'),('executable','actual_executable_sha256','actual_executable_path')]:
        contract=binary.get(key,{});path=Path(contract.get('path','')).resolve();bc=bc and path.is_relative_to(expected_prefix)and path.is_file()and sha(path)==contract.get('sha256')==loaded.get(actualsha)
        actual=loaded.get(actualpath)
        bc=bc and ([str(path)]==actual if key=='core'else str(path)==actual)
    bc=bc and loaded.get('verified')is True and runtime.get('slam_loaded_binary_verified')is True and runtime.get('slam_loaded_binary_receipt_sha256')==sha(run/'slam_loaded_binary.json')
    p=plan.get('profile',{});identity=loaded.get('mapping_identity',{});owner={r['owner_tid']for r in rows}
    source_ok=bound and bc and plan.get('scope_sha256')==sha(run/'navigation_scope.json')and plan.get('source_manifest_sha256')==sha(run/'source_manifest.json')and runtime.get('runtime_plan_sha256')==sha(run/'runtime_plan.json')
    controlled=p.get('navigation_ground_truth_used')is False and p.get('cascade',{}).get('navigation_ground_truth_used')is False and runtime.get('truth_navigation_used')is False and runtime.get('navigation_ground_truth_used')is False
    roles=runtime.get('owned_processes',[]);rx=next(iter({r['receive_tid']for r in rows}),None)
    checks={'actual_frozen_V17_source_and_loaded_binary_bindings':ck(source_ok,source_snapshot=str(archive),core_sha256=loaded.get('actual_core_sha256')),'actual_environment_pipeline_and_owner_role':ck(env.get('FASTLIVO_INGRESS_PIPELINE')=='1'and env.get('DEMO_RUN_DIR')==str(run)and owner=={identity.get('pid')}and rx!=identity.get('pid'),mapping_identity=identity,actual_environment=env),'actual_navigation_no_truth_unchanged_CPU_owner_scope':ck(controlled and plan.get('actor_device')=='cpu'and plan.get('actor_threads')==1 and plan.get('exclusive_writer')=='teacher_sim::TeacherActuator'and p.get('pose_cloud_timeout_s')==.3 and len([r for r in roles if r.get('role')=='worker'])==1),'declared_60s_scope_same_sampling':ck(plan.get('duration_s')==60 and p.get('sensor_sampling',{}).get('lidar_hz')==30 and p.get('sensor_sampling',{}).get('camera_hz')==30)}
    try:
        import yaml
        config=run/'navigation_fastlivo.yaml'
        if not config.is_file():raise Missing('Frozen actual SLAM configuration missing')
        params=yaml.safe_load(config.read_text())['/**']['ros__parameters']
        gps=params['common']['use_gps']
        checks['actual_GPS_and_decode_exception_source_semantics']=ck(source_ok and sha(config)==refs.get(str(config))and gps is False and params['preprocess']['lidar_type']==0,actual_GPS_parameter=gps,proof='Explicit frozen config GPSfalse, actual loaded same-source startup guard rejects GPStrue in pipeline mode; original owner deferred-error location bound by frozen source and pre-test17-case semantic receipt. No exception is invented or ignored.',config_sha256=sha(config))
    except ImportError:checks['actual_GPS_and_decode_exception_source_semantics']=ck(None,reason='PyYAML unavailable; no guessed GPSfalse')
    except (Missing,KeyError,TypeError,ValueError)as e:checks['actual_GPS_and_decode_exception_source_semantics']=ck(None,reason=str(e))
    return checks,runtime
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);ap.add_argument('--output',required=True);args=ap.parse_args();run=Path(args.run).resolve();checks={};inputs={}
    if sha(CRITERION)!=CRITERION_SHA:raise ValueError('Prospective smoke criterion bytes changed')
    criterion=read_json(CRITERION)
    for name,digest in criterion['pre_test_references_sha256'].items():
        if sha(name)!=digest:raise ValueError('Prospective reference changed '+name)
    inputs[str(CRITERION)]=CRITERION_SHA
    def attempt(key,fn):
        try:result=fn();checks.update(result)if isinstance(result,dict)and'status'not in result else checks.update({key:result});return result
        except Missing as e:checks[key]=ck(None,reason=str(e))
        except (ValueError,KeyError,TypeError,OSError,struct.error)as e:checks[key]=ck(False,reason=type(e).__name__+': '+str(e))
    trace=run/'fastlivo_debug/ingress_pipeline.csv';summary_path=run/'fastlivo_debug/ingress_pipeline_summary.txt'
    measurements={};runtime={}
    try:
        rows=parse_trace(trace);summary=parse_summary(summary_path);raw,counts=indexed_diagnostics(run)
        checks.update(integrity(rows,summary,raw));measurements=metrics(rows,raw)
        writer=read_json(run/'fastlivo_diagnostics/writer_stats.json')
        checks['original_writer_final_complete_and_drained']=ck(writer.get('final')is True and writer.get('attempted')==writer.get('written')==counts['records']and writer.get('dropped')==0 and writer.get('writer_io_failed')is False and writer.get('file_bytes')==counts['file_bytes'],writer=writer)
        attempt('actual_source_scope_bindings',lambda:scope_checks(run,rows)[0])
        for path in [trace,summary_path,run/'fastlivo_diagnostics/index.csv',run/'fastlivo_diagnostics/records.bin',run/'fastlivo_diagnostics/writer_stats.json',run/'runtime_plan.json',run/'runtime_manifest.json',run/'navigation_scope.json',run/'navigation_source_snapshots.json',run/'slam_loaded_binary.json',run/'navigation_stack_effective_environment.json']:
            if path.is_file():inputs[str(path)]=sha(path)
    except Missing as e:checks['required_original_pipeline_inputs']=ck(None,reason=str(e))
    except (ValueError,KeyError,TypeError,OSError,struct.error)as e:checks['required_original_pipeline_inputs']=ck(False,reason=type(e).__name__+': '+str(e))
    checks['original_native_motion_and_300ms_publication_guard_receipts']=ck(None,reason='Must be separately joined to this run original common/publication receipts; this reader does not invent native/TTL PASS or excuse cleanup')
    if'actual_GPS_and_decode_exception_source_semantics'not in checks:
        checks['actual_GPS_and_decode_exception_source_semantics']=ck(None,reason='Frozen config/source startup guard binding unavailable; missing is not false GPS')
    report={'schema':'independent_actual_V17_ingress_smoke_pipeline_audit/v1','run':str(run),'status':overall(checks),'pipeline_integrity_status':overall({k:v for k,v in checks.items()if k not in('original_native_motion_and_300ms_publication_guard_receipts','actual_GPS_and_decode_exception_source_semantics')}),'criteria_sha256':CRITERION_SHA,'timing_semantics_sha256':TIMING_SHA,'checks':checks,'measurements':measurements,'verified_input_source_sha256':inputs,'reader_sha256':sha(__file__),'navigation_pass_claimed':False,'limitations':criterion['non_claims'],'canceled_events_exempted':False,'full_original_32_route_remains_independently_unverified':True}
    output=Path(args.output).resolve()
    if not output.is_relative_to(HERE):raise ValueError('This reader owns only pipeline_runtime_audit output paths')
    with output.open('x')as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'status':report['status'],'pipeline_integrity_status':report['pipeline_integrity_status'],'run':str(run)}))
if __name__=='__main__':main()
