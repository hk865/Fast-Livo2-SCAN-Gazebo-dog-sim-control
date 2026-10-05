#!/usr/bin/env python3
"""Read-only actual first4 A/B audit. No ROS nodes or control publishers."""
from pathlib import Path
from decimal import Decimal
import bisect,collections,hashlib,importlib.util,json,math,re,struct,sys,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())
def metric(q):
    x,y,z,w=q;n=x*x+y*y+z*z+w*w
    roll=math.atan2(2*(w*x+y*z),n-2*(x*x+y*y))
    pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x)/n)))
    return max(abs(roll),abs(pitch)),roll,pitch
def actual_stamp(m):return m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
def peak(values):return max((abs(v) for v in values),default=0.)

def audit_cdr(run,start,end):
    path=run/'actuator/actuator_suffix.cdrlog';footer=load(run/'actuator/observer_result.json')
    stamps=collections.defaultdict(list);clock_stamps=collections.defaultdict(list)
    window_stamps=collections.defaultdict(list);payload_checks=collections.Counter();peaks=collections.defaultdict(float)
    counts=collections.Counter();classes={};prev_seq=None;first_seq=None;last_seq=None;seq_errors=[];errors=[];interval_lost=[]
    jtc_joints=set();measured_joints=set();effort_joint_nonzero=0;jtc_peak_record={}
    unavailable=collections.Counter();joint_peaks={}
    urdf=ET.parse(run/'staging/go2_measured.urdf').getroot()
    limits={j.attrib['name']:{k:float(v) for k,v in j.find('limit').attrib.items()} for j in urdf.findall('joint') if j.attrib['type']=='revolute'}
    window_metrics=['reference.positions','reference.velocities','reference.accelerations','feedback.positions','feedback.velocities','error.positions','error.velocities','output.effort']
    with path.open('rb') as f:
        if f.read(len(b'ACTUATORSUFFIX1\n'))!=b'ACTUATORSUFFIX1\n':raise ValueError('invalid capture magic')
        while True:
            sizes=f.read(8)
            if not sizes:break
            if len(sizes)!=8:raise ValueError('truncated record length')
            hn,cn=struct.unpack('<II',sizes);hdata=f.read(hn);cdr=f.read(cn)
            if len(hdata)!=hn or len(cdr)!=cn:raise ValueError('truncated original record')
            h=json.loads(hdata);k=h['kind'];seq=h['sequence'];counts[k]+=1
            if first_seq is None:first_seq=seq
            if prev_seq is not None and seq!=prev_seq+1:seq_errors.append([prev_seq,seq])
            prev_seq=last_seq=seq;ns=h.get('header_stamp_ns');clock_stamps[k].append(h['clock_ns'])
            if ns is not None:stamps[k].append(ns)
            time_ns=ns if ns is not None and ns>0 else h['clock_ns']
            if k=='middleware_lost' and start<=h['clock_ns']<=end:interval_lost.append(h)
            if not start<=time_ns<=end:continue
            window_stamps[k].append(time_ns)
            if k not in ('imu','measured_joint','legacy_joint','jtc','foot_lf','foot_rf','foot_lh','foot_rh','truth','slam'):continue
            if h['type'] not in classes:classes[h['type']]=get_message(h['type'])
            m=deserialize_message(cdr,classes[h['type']]);payload_checks[k]+=1
            if actual_stamp(m)!=ns:errors.append(dict(kind=k,sequence=seq,error='actual CDR Header differs from retained metadata'))
            if k=='imu':
                q=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w]
                if not np.isfinite(q).all() or np.linalg.norm(q)==0:errors.append(dict(kind=k,sequence=seq,error='invalid actual quaternion'))
                peaks['raw_guard_Euler_max_rad']=max(peaks['raw_guard_Euler_max_rad'],metric(q)[0])
            elif k in ('measured_joint','legacy_joint'):
                names=tuple(m.name)
                if len(names)!=12 or len(set(names))!=12:errors.append(dict(kind=k,sequence=seq,error='wrong actual 12 names'))
                (measured_joints if k=='measured_joint' else jtc_joints).add(names)
                for field in ('position','velocity','effort'):
                    values=getattr(m,field)
                    if k=='legacy_joint' and field=='effort' and not np.isfinite(values).all():
                        # JSB legally reports NaN for unavailable hardware effort.
                        # Preserve that absence, never convert it into measured zero.
                        unavailable[k+'.effort_nonfinite_messages']+=1
                    elif values and (len(values)!=12 or not np.isfinite(values).all()):errors.append(dict(kind=k,sequence=seq,error='invalid '+field))
                    finite=[v for v in values if math.isfinite(v)]
                    if finite:peaks[k+'.'+field]=max(peaks[k+'.'+field],peak(finite))
                if k=='measured_joint' and any(v!=0 for v in m.effort):effort_joint_nonzero+=1
            elif k=='jtc':
                names=tuple(m.joint_names);jtc_joints.add(names)
                if len(names)!=12 or len(set(names))!=12:errors.append(dict(kind=k,sequence=seq,error='wrong JTC actual names'))
                for dotted in window_metrics:
                    section,field=dotted.split('.');values=getattr(getattr(m,section),field)
                    if values and (len(values)!=12 or not np.isfinite(values).all()):errors.append(dict(kind=k,sequence=seq,error='invalid JTC '+dotted))
                    value=peak(values)
                    if value>peaks['jtc.'+dotted]:
                        peaks['jtc.'+dotted]=value;jtc_peak_record[dotted]=dict(stamp_ns=ns,max_abs=value,values=list(values))
                for index,name in enumerate(names):
                    item=joint_peaks.setdefault(name,dict(URDF=limits[name],output_command_effort_abs_max=0.,reference_velocity_abs_max=0.,actual_feedback_velocity_abs_max=0.,position_error_abs_max=0.,velocity_error_abs_max=0.,command_output_above_URDF_effort_rows=0,first_command_output_above_URDF_effort_stamp_ns=None,actual_velocity_at_URDF_limit_rows=0))
                    for key,section,field in [('output_command_effort_abs_max','output','effort'),('reference_velocity_abs_max','reference','velocities'),('actual_feedback_velocity_abs_max','feedback','velocities'),('position_error_abs_max','error','positions'),('velocity_error_abs_max','error','velocities')]:
                        data=getattr(getattr(m,section),field)
                        if len(data)==12 and abs(data[index])>item[key]:item[key]=abs(data[index]);item[key+'_stamp_ns']=ns
                    if abs(m.output.effort[index])>limits[name]['effort']:
                        item['command_output_above_URDF_effort_rows']+=1
                        if item['first_command_output_above_URDF_effort_stamp_ns'] is None:item['first_command_output_above_URDF_effort_stamp_ns']=ns
                    if abs(m.feedback.velocities[index])>=limits[name]['velocity']*(1-1e-8):item['actual_velocity_at_URDF_limit_rows']+=1
    streams={}
    for kind in ['clock','imu','measured_joint','legacy_joint','jtc','truth','slam','foot_lf','foot_rf','foot_lh','foot_rh']:
        ns=stamps.get(kind,[]);win=window_stamps.get(kind,[])
        expected=1_000_000 if kind in ('clock','imu','measured_joint') else 4_000_000 if kind in ('legacy_joint','jtc') else None
        gaps=[b-a for a,b in zip(win,win[1:])]
        streams[kind]=dict(retained_header_first_ns=min(ns,default=None),retained_header_last_ns=max(ns,default=None),
            native_headers_bracket_interval=bool(ns) and min(ns)<=start and max(ns)>=end,
            interval_count=len(win),decoded_original_messages=payload_checks[kind],
            duplicate_or_backward_in_interval=sum(v<=0 for v in gaps),max_interval_header_gap_ns=max(gaps,default=None),
            expected_period_ns=expected,unexpected_fixed_period_gaps=[v for v in gaps if expected is not None and v!=expected][:30],
            middleware_lost_events=footer['middleware_lost_events'].get(kind,0),
            middleware_lost_callbacks_in_this_interval=sum(e.get('source')==kind for e in interval_lost),
            sparse_contact_observations=kind.startswith('foot_'))
    fixed_ok=all(streams[k]['native_headers_bracket_interval'] and streams[k]['interval_count']>0
        and not streams[k]['unexpected_fixed_period_gaps'] and not streams[k]['duplicate_or_backward_in_interval']
        and streams[k]['middleware_lost_callbacks_in_this_interval']==0 for k in ('clock','imu','measured_joint','legacy_joint','jtc'))
    return dict(interval_ns=[start,end],all_fixed_period_sensor_JTC_headers_complete=fixed_ok,streams=streams,
        retained_events=sum(counts.values()),retained_sequence=[first_seq,None if last_seq is None else last_seq+1],
        interval_middleware_lost_callback_records=interval_lost,
        sequence_errors=seq_errors,metadata_matches_footer=sum(counts.values())==footer['retained_events'] and [first_seq,last_seq+1]==footer['retained_sequence'],
        full_original_payload_errors=errors,decoded_original_window_messages=dict(payload_checks),joint_name_order_sets=dict(measured=[list(x) for x in measured_joints],legacy_and_JTC=[list(x) for x in jtc_joints]),
        absolute_peaks=peaks,JTC_peak_records=jtc_peak_record,measured_joint_nonzero_effort_messages=effort_joint_nonzero,
        unavailable_legacy_effort=dict(unavailable),JTC_name_mapped_peaks_and_actual_model_limits=joint_peaks,
        footer=footer,limits=['CDR retains all original fields; selected original payloads were decoded without altering messages.',
          'Sparse contact header gaps are not invented empty-contact records or proof of a missed publisher frame.',
          'Header continuity is observed state-publication evidence, not a native ControllerInterface::trigger_update capture.',
          'JTC output.effort is commanded effort; actual JointState effort zero is not proof of zero applied torque.',
          'Legacy JSB effort NaN is unavailable measurement, preserved as unavailable rather than an invalid payload or zero torque.',
          'Output command exceeding URDF effort does not establish actual applied force, actuator clipping or its causal role.',
          'Headerless command and Pose use recorder latest /clock association, not an invented exact simulation publication stamp.'])

def audit_native(run,result):
    sync=[];slices=[]
    with (run/'stack.log').open() as f:
        for line in f:
            if '[DEMO_SYNC]' in line:sync.append(dict(re.findall(r'(camera|imu_newest|imu_last_used|imu_count|complete)=(-?[\d.]+)',line)))
            if '[DEMO_LIDAR_SLICE]' in line:slices.append(dict(re.findall(r'(camera|previous|points|offset_min_ms|offset_max_ms|pending)=(-?[\d.]+)',line)))
    raw=(run/'fastlivo_debug/imu.txt').read_bytes();base=int(Decimal(sync[0]['camera'])*1_000_000_000)
    times=[int(Decimal(line.split()[0].decode())*1_000_000_000)+base for line in raw.splitlines() if line.strip()]
    grid=[round(ns/1_000_000)*1_000_000 for ns in times];s=set(grid)
    low,high=min(grid),max(grid);expected=set(range(low,high+1,1_000_000));official={x['stamp_ns'] for x in result['raw_imu']}
    start,end=result['origin_stamp_ns'],result['terminal_stamp_ns']
    active_sync=[x for x in sync if start<=int(Decimal(x['camera'])*1_000_000_000)<=end]
    bad=[x for x in active_sync if x['complete']!='1' or Decimal(x['imu_last_used'])<Decimal(x['camera'])-Decimal('0.000000002')]
    bad_slice=[x for x in slices if x['pending']!='0' or x['offset_min_ms']!=x['offset_max_ms'] or abs(float(x['offset_min_ms'])-1000*(float(x['camera'])-float(x['previous'])))>2e-5]
    return dict(native_samples=len(times),unique_1ms_native_samples=len(s),span_ns=[low,high],missing_native_1ms=sorted(expected-s),null_bytes=raw.count(b'\0'),
        raw_observed_but_native_unconsumed_in_native_span=sorted(t for t in official if low<=t<=high and t not in s),
        native_not_in_probe_raw_count=len(s-official),native_clock_origin_first_camera_ns=base,log_float_quantization_max_ns=max(abs(a-b) for a,b in zip(times,grid)),
        active_sync_count=len(active_sync),active_sync_bad=bad,generic_slice_count=len(slices),generic_slice_bad=bad_slice,
        limits='Only native integrator log head timestamps recovered by documented first LiDAR origin; <=2ns log representation quantized for 1ms input coverage, never state interpolation or correction.')

def audit_case(name,enabled):
    run=ROOT/'simulation/test_results'/name;result=load(run/'first_four_result.json')
    spec=importlib.util.spec_from_file_location('archived_'+name,run/'staging/first_four_region_contract.py');contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)
    evaluation=contract.evaluate_regions(tuple(parse_goal(g) for g in result['goals_definitions']),result['request_id'],result['statuses'],result['poses'],result['truth'],result['origin_stamp_ns'],result['terminal_stamp_ns'])
    transform=evaluation['initial_fixed_SE3_evaluation_only'];R=np.array(transform['rotation_world_from_slam']);T=np.array(transform['translation'])
    attitude=[]
    for p in result['poses']:
        if not result['origin_stamp_ns']<=p['stamp_ns']<=result['terminal_stamp_ns']:continue
        matched=contract.bounded_pair(p['stamp_ns'],result['truth'])
        if matched is None:continue
        actual=Rotation.from_matrix(matched[1]);estimated=Rotation.from_matrix(R)*Rotation.from_quat(p['q'])
        attitude.append(dict(stamp_ns=p['stamp_ns'],orientation_error_rad=float((actual.inv()*estimated).magnitude()),GT_Euler_guard_metric_rad=metric(actual.as_quat())[0],aligned_SLAM_Euler_guard_metric_rad=metric(estimated.as_quat())[0]))
    raw=[r for r in result['raw_imu'] if result['origin_stamp_ns']<=r['stamp_ns']<=result['terminal_stamp_ns']]
    guard={str(level):next((dict(stamp_ns=r['stamp_ns'],metric_rad=metric(r['quaternion'])[0],roll_pitch_rad=list(metric(r['quaternion'])[1:])) for r in raw if metric(r['quaternion'])[0]>=level),None) for level in (.30,.50)}
    for value in guard.values():
        if value is not None:
            actual=contract.bounded_pair(value['stamp_ns'],result['truth'])
            value['same_stamp_GT_world_Euler_rad']=None if actual is None else metric(Rotation.from_matrix(actual[1]).as_quat())[0]
    interval_start=result.get('fourth_segment_previous_receipt_stamp_ns')
    if interval_start is None:interval_start=result['origin_stamp_ns']
    cdr=audit_cdr(run,interval_start,result['terminal_stamp_ns'])
    out=dict(name=name,body_feedback_enabled=enabled,original_passed=result['passed'],original_failure=result['failure'],missing_acceptance=result['missing_acceptance'],independent_original_regions=evaluation,
        attitude=dict(rmse_rad=float(np.sqrt(np.mean([x['orientation_error_rad']**2 for x in attitude]))),max_rad=max(x['orientation_error_rad'] for x in attitude),last_twelve=attitude[-12:]),
        original_Euler_guard_first_crossings=guard,active_raw_Euler_max=max(metric(x['quaternion'])[0] for x in raw),native_input=audit_native(run,result),actual_CDR=cdr,
        cleanup=load(run/'process_cleanup.json'),manifest=load(run/'first4_manifest.json'),original_result_sha256=sha(run/'first_four_result.json'),
        limits=['Original A PASS and B FAIL retained. Formal Full18 FAIL remains unmodified.','No GT/SLAM state feeds back to the body pose; one initial SE3 only for independent evaluation.',
          'A does not reproduce the original Full18 failure. Fresh A/B physics initial microscopic states are not identical; this pair does not uniquely identify the cause.'])
    print(json.dumps(dict(case=name,region_passed=evaluation['passed'],precision=evaluation['precision'],raw_max=out['active_raw_Euler_max'],cdr_headers_complete=cdr['all_fixed_period_sensor_JTC_headers_complete'],cdr_payload_errors=len(cdr['full_original_payload_errors']),native_missing=len(out['native_input']['missing_native_1ms']))),flush=True)
    return out

if __name__=='__main__':
    a=audit_case('20261002_feedback_first4_a_disabled',False)
    b=audit_case('20261002_feedback_first4_b_enabled',True)
    report=dict(verdict='Current enabled posture feedback rejected for adoption: A original first4 passed, B actually tilted past .50 and completed 0/4.',A=a,B=b,
        only_enable_preregistered=b['manifest'].get('only_enable_comparison'),all_actual_mapped_library_paths_SHA_identical=b['cleanup'].get('A_B_entire_prestop_mapped_libraries_identical'),
        original_formal_Full18_acceptance_sha256=sha(ROOT/'runs/20261002_001559_882ddb/acceptance.json'),
        input_sha256={name:{s:sha(ROOT/'simulation/test_results'/name/s) for s in ['first_four_result.json','actuator/observer_result.json','actuator/actuator_suffix.cdrlog','process_cleanup.json','first4_manifest.json','fastlivo_debug/imu.txt','stack.log']} for name in [a['name'],b['name']]})
    dest=ROOT/'slam/test_results/oct2_feedback_first4_AB_actual_independent.json';dest.write_text(json.dumps(report,indent=2)+'\n');print(str(dest),flush=True)
