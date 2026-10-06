"""Read-only Full19 original receipt/GT/native input and one CDR metadata audit."""
from pathlib import Path
from decimal import Decimal
import collections, hashlib, importlib.util, json, math, re, struct, sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
RUN = ROOT / 'runs/20261002_084329_14d9cc'
OUT = ROOT / 'slam/test_results'

def load(p): return json.loads(p.read_text())
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def metric(q):
    x,y,z,w=q; n=x*x+y*y+z*z+w*w
    roll=math.atan2(2*(w*x+y*z),n-2*(x*x+y*y))
    pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x)/n)))
    return max(abs(roll),abs(pitch)),roll,pitch
def write(name,obj):
    p=OUT/name;p.write_text(json.dumps(obj,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps({'report':str(p),'sha256':sha(p)},ensure_ascii=False),flush=True)
    return p

def core_audit():
    spec=importlib.util.spec_from_file_location('full19_readonly_evaluator',ROOT/'scripts/evaluate_run.py')
    e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)
    mission=load(RUN/'mission.json'); scenario=load(RUN/'scenario.json')
    source=load(RUN/'source_manifest.json'); runtime=load(RUN/'runtime_manifest.json')
    runtimeproof=load(RUN/'full_control_runtime_evidence.json'); official=load(RUN/'acceptance.json')
    bad_sources=[n for n,h in source['sha256'].items() if sha(ROOT/n)!=h]
    samples,errors=e.read_poses(RUN/'pose_audit.jsonl')
    trajectory,R,T,truth=e.compare_trajectory(samples)
    region=e.ordered_region_evidence(RUN,scenario,mission,samples,R,T,truth)
    active=[p for p in samples['slam'] if p['stage'] in e.MOVING|{'saving_map'}]
    start=min(p['stamp_ns'] for p in active); end=max(p['stamp_ns'] for p in active)
    bridge=load(RUN/'bridge_control_timing.json')
    safety_end=int(Decimal(str(bridge['first_bridge_failure']['imu_stamp']))*1000000000)
    matched,gtp=e.time_matched_positions(active,truth)
    st=[p['stamp'] for p in matched]
    grot=Slerp([p['stamp'] for p in truth],Rotation.from_quat([p['q'] for p in truth]))(st)
    est=Rotation.from_matrix(R)*Rotation.from_quat([p['q'] for p in matched])
    residual=(grot.inv()*est).magnitude()
    attitude={'active_matched_samples':len(matched),'RMSE_rad':float(np.sqrt(np.mean(residual**2))),
        'max_rad':float(residual.max()),'last_same_stamp':[{'stamp_ns':p['stamp_ns'],
        'SLAM_world_Euler_rad':metric(est[i].as_quat())[0],'GT_world_Euler_rad':metric(grot[i].as_quat())[0],
        'orientation_residual_rad':float(residual[i])} for i,p in enumerate(matched) if p['stamp']>=853.8]}
    sensorcounts=collections.Counter(); rawstamps=set(); rawgap=[]; active_rawgap=[]
    first={}; crosses=[]; peak={'value':-1}; previous=None; previous_wall=None; max_wallgap={}
    firstcamera=None; lastcamera=None; camera_gaps=[]; camera_prev=None
    with (RUN/'sensor_audit.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line); kind=row['source'];sensorcounts[kind]+=1
            ns=int((Decimal(str(row['stamp']))*1000000000).to_integral_value())
            if kind=='camera':
                firstcamera=ns if firstcamera is None else firstcamera;lastcamera=ns
                if camera_prev is not None and ns-camera_prev>200000000:camera_gaps.append([camera_prev,ns])
                camera_prev=ns
            if kind!='imu':continue
            rawstamps.add(ns); value,roll,pitch=metric(row['quaternion']); is_active=start<=ns<=safety_end
            if previous is not None and ns-previous!=1000000:
                item={'previous_stamp_ns':previous,'next_stamp_ns':ns,'gap_ns':ns-previous}
                rawgap.append(item)
                if start<=previous<=safety_end and start<=ns<=safety_end:active_rawgap.append(item)
            if previous_wall is not None and ns>=10000000000:
                gap=row['wall_monotonic']-previous_wall
                if gap>max_wallgap.get('gap_s',-1):max_wallgap=dict(gap_s=gap,previous_stamp_ns=previous,next_stamp_ns=ns)
            previous=ns;previous_wall=row['wall_monotonic']
            if is_active and value>peak['value']:peak={'value':value,'stamp_ns':ns,'roll':roll,'pitch':pitch,'row':row}
            for limit in [.2,.3,.5]:
                if is_active and value>=limit and str(limit) not in first:
                    first[str(limit)]={'stamp_ns':ns,'Euler_metric_rad':value,'roll_rad':roll,'pitch_rad':pitch,'row':row}
            if is_active and value>=.30 and (not crosses or ns-crosses[-1]['last_stamp_ns']>1000000):
                crosses.append({'first_stamp_ns':ns,'last_stamp_ns':ns,'peak':value})
            elif is_active and value>=.30:
                crosses[-1]['last_stamp_ns']=ns;crosses[-1]['peak']=max(crosses[-1]['peak'],value)
    sync=[];slices=[]
    with (RUN/'stack.log').open() as stream:
        for line in stream:
            if '[DEMO_SYNC]' in line:sync.append(dict(re.findall(r'(camera|imu_newest|imu_last_used|imu_count|complete)=(-?[\d.]+)',line)))
            if '[DEMO_LIDAR_SLICE]' in line:slices.append(dict(re.findall(r'(camera|previous|points|offset_min_ms|offset_max_ms|pending)=(-?[\d.]+)',line)))
    base=int(Decimal(sync[0]['camera'])*1000000000)
    native=[];nulls=0
    with (RUN/'fastlivo_debug/imu.txt').open('rb') as stream:
        for line in stream:
            nulls+=line.count(b'\x00')
            if line.strip():native.append(int(Decimal(line.split()[0].decode())*1000000000)+base)
    grid=[int(round(n/1000000))*1000000 for n in native];nsset=set(grid)
    lo,hi=min(grid),max(grid)
    missing=sorted(set(range(lo,hi+1,1000000))-nsset)
    used_unobserved=sorted(nsset-rawstamps)
    observed_unused=sorted(n for n in rawstamps if lo<=n<=hi and n not in nsset)
    active_sync=[s for s in sync if start<=int(Decimal(s['camera'])*1000000000)<=end]
    syncbad=[s for s in active_sync if s['complete']!='1' or Decimal(s['imu_last_used'])<Decimal(s['camera'])-Decimal('0.000000002')]
    slicebad=[s for s in slices if s['pending']!='0' or s['offset_min_ms']!=s['offset_max_ms'] or abs(float(s['offset_min_ms'])-1000*(float(s['camera'])-float(s['previous'])))>2e-5]
    meta=load(RUN/'map_metadata.json')
    cloud=np.fromfile(RUN/meta['binary_filename'],dtype=np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,)),('pad','u1')]))
    packed=cloud['rgb'].astype(np.uint32); colors=(packed[:,0]<<16)|(packed[:,1]<<8)|packed[:,2]
    stages={k:{'required':v['required_waypoints'],'reached_ordered':v['reached_waypoints'],
        'stage_passed':v['passed'],'definition_errors':v['definition_errors'],
        'arrived_original_windows':[p for p in v['waypoints'] if 'navigation_receipt' in p]} for k,v in region['stages'].items()}
    checks={'official_original_FAIL_preserved':official['passed'] is False,
        'source329_declared_currentbytes_exact':len(source['sha256'])==329 and not bad_sources,
        'original_all18_exploration_rawinner_GTouter_windows':stages['exploring']['reached_ordered']==18 and stages['exploring']['stage_passed'],
        'original_first_return_rawinner_GTouter_window':stages['returning']['reached_ordered']==1,
        'active_GT_all_bounded_paired':region['truth_full_pose_coverage']['passed'],
        'preregistered_v2_declaration_exact':region['preregistered_declaration']['passed'],
        'ownedclean_runtime16_alltrue':runtimeproof['passed'] is True and len(runtimeproof['checks'])==16 and all(runtimeproof['checks'].values()),
        'map_actual_binary_RGB_matches_metadata':len(cloud)==meta['point_count']==meta['rgb_points'] and np.isfinite(cloud['xyz']).all().item(),
        'map_current_RGB_no_reference_or_GT':meta['ground_truth_used'] is False and meta['reference_map_loaded'] is False and len(np.unique(colors))>=8,
        'map_not_capacity_truncated':meta['capacity_rejections']==0 and meta['capacity_points']==8000000}
    result={'scope':'Independent immutable Full19 failure/prefix evidence. Original 46-goal contract is not complete.',
        'checks':checks,'necessary_prefix_evidence_ready':all(checks.values()),'run_id':RUN.name,
        'mission_events':mission['events'],'terminal_message':mission['message'],
        'active_pose_interval_ns':[start,end],'single_initial_SE3':trajectory,'region_evidence':region,
        'original_safety_sensor_interval_ns':[start,safety_end],
        'ordered_prefix':stages,'orientation_same_initial_SE3':attitude,'pose_parse_errors':errors,
        'official_acceptance':{'passed':official['passed'],'sha256':sha(RUN/'acceptance.json'),'failed_checks':official['failed_checks']},
        'source329':{'manifest_sha256':sha(RUN/'source_manifest.json'),'bad_sources':bad_sources},
        'actual_runtime':runtimeproof,'original_safety_metric':'max(abs(normalized Euler roll),abs(normalized Euler pitch)); not total body-axis angle',
        'raw_IMU':{'source_counts':dict(sensorcounts),'count':sensorcounts['imu'],'unique_headers':len(rawstamps),
        'stamp_encoding_note':'Official sensor audit stores ROS stamp as float seconds; converted to nearest integer nanosecond, not truncated or interpolated. Original exact sec/nsec is retained separately in CDR metadata where available.',
        'header_range_ns':[min(rawstamps),max(rawstamps)],'unexpected_1ms_gaps':rawgap,'active_unexpected_1ms_gaps':active_rawgap,
        'max_post10_wall_gap':max_wallgap,'active_peak':peak,'active_first_crossings':first,'active030_episodes':crosses},
        'executed_first_failure_RAM':bridge['first_bridge_failure'],
        'FAST_native_IMU':{'rows':len(native),'unique_1ms_quantized_headstamps':len(nsset),'span_ns':[lo,hi],
        'origin_from_first_camera_ns':base,'quantization_max_ns':max(abs(a-b) for a,b in zip(native,grid)),
        'null_bytes':nulls,'missing_native_1ms_grid':missing,'official_observed_but_native_unconsumed':observed_unused,
        'native_unpaired_official':used_unobserved,'active_missing_native':[n for n in missing if start<=n<=end]},
        'generic_current_frame':{'active_sync_records':len(active_sync),'sync_bad':syncbad,'slice_records':len(slices),'slice_bad':slicebad},
        'RGB_observation':{'camera_header_range_ns':[firstcamera,lastcamera],'camera_gaps_above200ms':camera_gaps,
        'camera_observations':sensorcounts['camera'],'observed_RGB_samples':meta['observed_rgb_samples']},
        'actual_map':{'metadata':meta,'decoded_points':len(cloud),'finite_xyz':bool(np.isfinite(cloud['xyz']).all()),
        'unique_RGB_triplets':int(len(np.unique(colors))),'PCD_save_requested':meta['save']['requested'],
        'PCD_files':[p.name for p in RUN.glob('*.pcd')]},
        'sampling_diagnostics_not_repaired':{'official_active_all_1ms':not active_rawgap,
        'native_active_all_1ms':not any(start<=n<=end for n in missing)},
        'input_sha256':{n:sha(RUN/n) for n in ['source_manifest.json','runtime_manifest.json','scenario.json','mission.json','pose_audit.jsonl',
        'navigation_audit.jsonl','sensor_audit.jsonl','fastlivo_debug/imu.txt','stack.log','map_metadata.json','full_control_runtime_evidence.json']},
        'limits':['Original FAIL and original arrival windows are immutable. No later window rescues a missing goal.',
        'One initial SE3 is evaluation-only; no GT/camera/world correction is applied to control or observed map.',
        'No save was requested; live RGB observed map is not an official saved PCD or a completed full demo.',
        'IMU input/observer omissions remain explicit; agreement does not fill samples or identify a unique physical cause.']}
    write('full19_independent_prefix_core.json',result)
    print(json.dumps({'original_prefix':{k:v['reached_ordered'] for k,v in stages.items()},'coverage':region['truth_full_pose_coverage'],
        'raw_gaps':len(rawgap),'native_missing':len(missing),'first_crossings':{k:v['stamp_ns'] for k,v in first.items()},'checks':checks},ensure_ascii=False)[:3500],flush=True)
    return result

def metadata_audit(core):
    start,end=core['active_pose_interval_ns']; protected_end=int(Decimal(str(core['executed_first_failure_RAM']['imu_stamp']))*1000000000)
    footer=load(RUN/'actuator/observer_result.json'); path=RUN/'actuator/actuator_suffix.cdrlog'
    counts=collections.Counter(); seqerrors=[]; errors=[]; losses=[]; previous_seq=None; firstseq=None; payloadbytes=0
    stats={}; shortstats={}; short_start=820000000000;short_end=856300000000
    indexpath=OUT/'full19_shared_820_856p3_CDR_metadata_index.jsonl'
    indexcount=0; expected={'clock':1000000,'imu':1000000,'measured_joint':1000000,'legacy_joint':4000000,'jtc':4000000}
    def accumulate(table,k,t,seq):
        s=table.setdefault(k,dict(count=0,first_ns=None,last_ns=None,minimum_ns=None,maximum_ns=None,max_gap_ns=None,
            duplicate_or_backward=0,unexpected_fixed_gaps=[]))
        if s['count']:
            delta=t-s['last_ns'];s['max_gap_ns']=max(delta,s['max_gap_ns'] if s['max_gap_ns'] is not None else delta)
            s['duplicate_or_backward']+=delta<=0
            if k in expected and delta!=expected[k]:s['unexpected_fixed_gaps'].append(dict(previous_ns=s['last_ns'],next_ns=t,gap_ns=delta,sequence=seq))
        else:s['first_ns']=t
        s['count']+=1;s['last_ns']=t;s['minimum_ns']=t if s['minimum_ns'] is None else min(t,s['minimum_ns']);s['maximum_ns']=t if s['maximum_ns'] is None else max(t,s['maximum_ns'])
    with path.open('rb') as f,indexpath.open('w') as index:
        if f.read(16)!=b'ACTUATORSUFFIX1\n':raise ValueError('capture magic')
        while True:
            offset=f.tell();sizes=f.read(8)
            if not sizes:break
            if len(sizes)!=8:raise ValueError('truncated sizes')
            hn,cn=struct.unpack('<II',sizes);hdata=f.read(hn)
            if len(hdata)!=hn or hn>20000:raise ValueError('invalid metadata')
            h=json.loads(hdata); payload_offset=f.tell();f.seek(cn,1);payloadbytes+=cn
            k=h['kind'];seq=h['sequence'];counts[k]+=1
            if firstseq is None:firstseq=seq
            if previous_seq is not None and seq!=previous_seq+1:seqerrors.append([previous_seq,seq])
            previous_seq=seq
            header=h.get('header_stamp_ns'); clock=h['clock_ns'];time_ns=header if header is not None and header>0 else clock
            if header is not None and h.get('header_sec') is not None and h.get('header_nanosec') is not None and (h['header_sec']*1000000000+h['header_nanosec'])!=header:
                errors.append({'sequence':seq,'error':'metadata sec/nsec mismatch'})
            if start<=time_ns<=protected_end:accumulate(stats,k,time_ns,seq)
            if short_start<=time_ns<=short_end:
                accumulate(shortstats,k,time_ns,seq)
                if k in ['imu','measured_joint','legacy_joint','jtc','raw_target','target','actual_command','adapter','feedback_pose','foot_lf','foot_rf','foot_lh','foot_rh','truth','slam','safety','clock','middleware_lost']:
                    index.write(json.dumps(dict(record_offset=offset,payload_offset=payload_offset,payload_size=cn,metadata=h),separators=(',',':'))+'\n');indexcount+=1
            if k=='middleware_lost':losses.append(dict(h,within_active=start<=clock<=protected_end,after_protected_terminal=clock>protected_end))
        actual_end=f.tell()
    retained=[firstseq,None if previous_seq is None else previous_seq+1]
    report={'scope':'Unique retained CDR metadata scan only; no original payload decoded or repaired.',
        'path':str(path),'file_bytes':path.stat().st_size,'metadata_records':sum(counts.values()),'counts':dict(counts),
        'retained_sequence':retained,'sequence_errors':seqerrors,'metadata_schema_errors':errors,
        'payload_bytes_counted':payloadbytes,'matches_footer':sum(counts.values())==footer['retained_events'] and retained==footer['retained_sequence'] and payloadbytes==footer['payload_bytes'],
        'footer':footer,'active_coverage_scope_ns':[start,protected_end],'active_retained_streams':stats,
        'short_window_ns':[short_start,short_end],'short_window_streams':shortstats,
        'retained_middleware_lost_callbacks':losses,'loss_callback_record_counts':dict(collections.Counter(x.get('source') for x in losses)),
        'shared_short_window_index':{'path':str(indexpath),'sha256':sha(indexpath),'records':indexcount},
        'cannot_claim_full_active_CDR_coverage':footer['overwritten_events']>0,
        'prefix_overwritten_events':footer['overwritten_events'],
        'limits':['Overwritten prefix payload and loss callback records cannot be restored. Footer total losses do not identify omitted Header timestamps.',
        'Contact messages are sparse observations; missing contact records are not invented empty-contact or airborne measurements.',
        'Headerless command/Pose times use latest recorder clock association, not exact producer or CHAMP callback timestamps.',
        'JTC/JSB message continuity does not prove every native controller trigger, hardware write, or applied joint torque.']}
    write('full19_unique_CDR_metadata_scan.json',report)
    print(json.dumps({'retained':report['metadata_records'],'overwrite':footer['overwritten_events'],'losses':losses,
        'fixed_window':{k:shortstats.get(k) for k in expected},'shared_index':report['shared_short_window_index']},ensure_ascii=False)[:4000],flush=True)
    return report

if __name__=='__main__':
    c=load(OUT/'full19_independent_prefix_core.json') if '--metadata-only' in sys.argv else core_audit()
    if '--metadata' in sys.argv:metadata_audit(c)
