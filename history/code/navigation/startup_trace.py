"""Bounded real-input diagnostics. Never grants initialization or navigation."""
from pathlib import Path
import hashlib
import json
import math
import threading
import time
import numpy as np

def safe(value):
    if isinstance(value,dict):return {str(k):safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [safe(v) for v in value]
    if isinstance(value,np.ndarray):return safe(value.tolist())
    if isinstance(value,(float,np.floating)):return float(value) if math.isfinite(value) else None
    if isinstance(value,np.integer):return int(value)
    if isinstance(value,np.bool_):return bool(value)
    return value

def encoded(value):
    return (json.dumps(safe(value),sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()

def allocated_for(length):return ((length+4095)//4096)*4096

class CallbackTrace:
    """Two processes use disjoint131072B caps; original async writer drains."""
    def __init__(self,run,role,writer):
        if role not in ('mission','canonical'):raise ValueError('Unknown callback trace role')
        self.path=Path(run)/('R33_'+role+'_callback_decisions.jsonl')
        self.writer=writer;self.bytes=0;self.maximum=131072;self.closed=False
    def record(self,kind,clock_ns,**fields):
        if self.closed:return
        data=dict(schema='R33_actual_callback_decision/v1',kind=kind,clock_ns=int(clock_ns),
                  actual_monotonic_wall_ns=time.monotonic_ns(),**fields)
        if clock_ns>20000000000:
            data=dict(schema='R33_actual_callback_decision/v1',kind='trace_window_closed',clock_ns=int(clock_ns))
            self.closed=True
        data=safe(data)
        # Match the original EvidenceWriter encoding, not a smaller estimate.
        length=len((json.dumps(data,allow_nan=False,ensure_ascii=False)+'\n').encode())
        if allocated_for(self.bytes+length)>self.maximum-4096:
            data=dict(schema='R33_actual_callback_decision/v1',kind='trace_cap_reached',clock_ns=int(clock_ns))
            length=len((json.dumps(data,allow_nan=False,ensure_ascii=False)+'\n').encode());self.closed=True
        if allocated_for(self.bytes+length)>self.maximum:raise RuntimeError('Callback trace budget accounting failed')
        self.bytes+=length;self.writer.append(self.path,data)

def predicate_metrics(config,poses,imus,native,registration,health):
    """Report each original predicate; original verifier makes every decision."""
    from mission46_runtime_evidence import native_ns,native_safe
    cfg=config['/**']['ros__parameters']['imu']
    metrics=dict(original_thresholds=dict(imu_required=int(cfg['imu_int_frame']),
        imu_span_ns=round(float(cfg.get('init_min_span',2.9))*1e9),
        imu_gap_limit_ns=round(float(cfg.get('init_max_sample_gap',.03))*1e9),
        gyro_limit=float(cfg.get('init_max_gyro_norm',.03)),
        acceleration_norm_error_limit=float(cfg.get('init_max_acc_norm_error',1.2)),
        acceleration_axis_std_limit=float(cfg.get('init_max_acc_axis_std',.4)),
        pose_span_ns=2900000000,pose_gap_limit_ns=200000000,pose_speed_limit=.03,
        pose_displacement_limit=.08,native_span_ns=2900000000,native_gap_limit_ns=30000000),
        counts=dict(imu=len(imus),pose=len(poses),native=len(native)),predicates={})
    checks=metrics['predicates'];checks['imu_count']=len(imus)>=int(cfg['imu_int_frame'])
    checks['pose_count']=len(poses)>=10;checks['native_count']=len(native)>=150
    checks['stationary_estimator_gate_enabled']=cfg.get('stationary_initialization_en') is True
    if imus:
        stamps=np.array([r['stamp_ns'] for r in imus],dtype=np.int64)
        gyro=np.asarray([r['angular_velocity_sensor'] for r in imus],dtype=float)
        acc=np.asarray([r['linear_acceleration_sensor'] for r in imus],dtype=float)
        span=int(stamps[-1]-stamps[0]);gap=int(max(np.diff(stamps))) if len(stamps)>1 else None
        metrics['imu']=dict(first_ns=int(stamps[0]),last_ns=int(stamps[-1]),span_ns=span,max_gap_ns=gap,
            max_gyro_norm=float(max(np.linalg.norm(gyro,axis=1))),
            max_acceleration_norm_error=float(max(abs(np.linalg.norm(acc,axis=1)-9.81))),
            max_acceleration_axis_std=float(max(np.std(acc,axis=0))))
        checks.update(imu_span=span>=round(float(cfg.get('init_min_span',2.9))*1e9),
            imu_strictly_ordered=bool(np.all(np.diff(stamps)>0)),
            imu_max_gap=gap is not None and gap<=round(float(cfg.get('init_max_sample_gap',.03))*1e9),
            gyro_finite=bool(np.isfinite(gyro).all()),acceleration_finite=bool(np.isfinite(acc).all()),
            gyro_norm=metrics['imu']['max_gyro_norm']<=float(cfg.get('init_max_gyro_norm',.03)),
            acceleration_norm=metrics['imu']['max_acceleration_norm_error']<=float(cfg.get('init_max_acc_norm_error',1.2)),
            acceleration_axis_std=metrics['imu']['max_acceleration_axis_std']<=float(cfg.get('init_max_acc_axis_std',.4)))
    if poses:
        stamps=[r['stamp_ns'] for r in poses];span=stamps[-1]-stamps[0]
        gap=int(max(np.diff(stamps))) if len(stamps)>1 else None
        speed=float(max(np.linalg.norm(r['body_velocity']) for r in poses))
        displacement=float(np.max(np.linalg.norm(np.array([r['position'] for r in poses])-poses[0]['position'],axis=1)))
        metrics['pose']=dict(first_ns=stamps[0],last_ns=stamps[-1],span_ns=span,max_gap_ns=gap,
                             max_speed=speed,max_displacement=displacement)
        checks.update(pose_span=span>=2900000000,pose_max_gap=gap is not None and gap<=200000000,
                      pose_speed=speed<=.03,pose_displacement=displacement<=.08)
    if native:
        stamps=[native_ns(r['data']) for r in native];span=stamps[-1]-stamps[0]
        gap=int(max(np.diff(stamps))) if len(stamps)>1 else None
        unsafe=[i for i,r in enumerate(native) if not native_safe(r['data'],True)]
        metrics['native']=dict(first_ns=stamps[0],last_ns=stamps[-1],span_ns=span,max_gap_ns=gap,
                              unsafe_indices=unsafe)
        checks.update(native_span=span>=2900000000,native_max_gap=gap is not None and gap<=30000000,
                      native_original_safe=not unsafe)
    checks.update(sensor_health_ready=health.get('ready') is True,
        sensor_health_no_truth=health.get('ground_truth_navigation_used') is False,
        registration_no_truth=registration.get('ground_truth_navigation_used') is False,
        registration_frozen=registration.get('status')=='frozen')
    metrics['diagnostic_only']=True;metrics['thresholds_modified']=False
    return safe(metrics)

class InitializationTrace:
    """The actual captured tuple is submitted unchanged to the original verifier."""
    def __init__(self,run):
        self.run=Path(run);self.path=self.run/'R33_INITIALIZATION_JOBS.jsonl'
        self.maximum=1572864;self.bytes=0;self.snapshots=0;self.lock=threading.Lock()
    def append(self,data,reserve=0,exact=False):
        raw=((json.dumps(data,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
             if exact else encoded(data))
        if allocated_for(self.bytes+len(raw))>self.maximum-reserve:return None
        offset=self.bytes
        with self.path.open('ab') as f:f.write(raw)
        self.bytes+=len(raw)
        return dict(file=str(self.path),offset=offset,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
    def verify(self,scope,poses,imus,native,registration,health,token,submit_clock_ns,submit_wall_ns):
        from mission46_runtime_evidence import verify_initialization,json_bytes
        import yaml
        with self.lock:
            started=time.monotonic_ns()
            config=yaml.safe_load((self.run/'navigation_fastlivo.yaml').read_text())
            try:metrics=predicate_metrics(config,poses,imus,native,registration,health)
            except Exception as error:metrics=dict(diagnostic_only=True,metrics_error=type(error).__name__+':'+str(error))
            references=[]
            for row in native:
                item={k:v for k,v in row.items() if k!='data'}
                try:item['actual_projection_sha256']=hashlib.sha256(json_bytes(row['data'])).hexdigest()
                except (ValueError,TypeError):
                    item.update(actual_projection_sha256=None,projection_not_canonical_JSON=True)
                references.append(item)
            snapshot=dict(schema='R33_actual_initialization_submit_snapshot/v1',job_token=token,
                actual_submit_clock_ns=submit_clock_ns,actual_submit_wall_ns=submit_wall_ns,
                actual_worker_started_wall_ns=started,actual_pose_tuple=poses,actual_IMU_tuple=imus,
                actual_native_tuple_exact_source_references=references,
                actual_native_tuple_data_recoverable_from_exact_source_line_and_projection_fields=True,
                actual_registration=registration,actual_sensor_health=health,actual_config=config,
                scope_file_sha256=hashlib.sha256((self.run/'navigation_scope.json').read_bytes()).hexdigest(),
                config_file_sha256=hashlib.sha256((self.run/'navigation_fastlivo.yaml').read_bytes()).hexdigest(),
                predicates=metrics,original_tuple_reconstructed_or_changed=False,admission_by_diagnostic=False)
            saved=None;snapshot_error=None
            if self.snapshots<4:
                try:saved=self.append(snapshot,reserve=65536,exact=True)
                except (ValueError,TypeError) as error:snapshot_error=type(error).__name__+':'+str(error)
            if saved:self.snapshots+=1
            result=dict(schema='R33_actual_initialization_result/v1',job_token=token,
                        actual_submit_clock_ns=submit_clock_ns,actual_submit_wall_ns=submit_wall_ns,
                        snapshot=saved,snapshot_complete=saved is not None,predicates=metrics,
                        snapshot_error=snapshot_error,
                        original_tuple_reconstructed_or_changed=False,admission_by_diagnostic=False)
            try:
                value=verify_initialization(self.run,scope,poses,imus,native,registration,health)
                result.update(original_verifier_passed=True,original_receipt=value)
                return value
            except Exception as error:
                result.update(original_verifier_passed=False,original_exception=type(error).__name__+':'+str(error))
                raise
            finally:
                result['actual_worker_completed_wall_ns']=time.monotonic_ns()
                self.append(result,reserve=32768)
                raw=encoded(result);tmp=self.run/'R33_INITIALIZATION_RESULT.tmp'
                if allocated_for(self.bytes)+2*allocated_for(len(raw))>self.maximum:
                    raise RuntimeError('Initialization trace complete result exceeds reserved cap')
                tmp.write_bytes(raw);tmp.replace(self.run/'R33_INITIALIZATION_RESULT.json')

def read_live_json(path):
    """Concurrent partial writes are pending evidence, never a positive result."""
    try:return json.loads(Path(path).read_text())
    except (OSError, ValueError):return None

def actual_scan_association(run,rid,admitted_only=False):
    """Recheck the original controller's real archived association transaction."""
    directory=Path(run)/'path_admission_candidates'
    if not directory.is_dir():return None
    for p in sorted(directory.glob('*.json'))[:64]:
        document=read_live_json(p)
        if not isinstance(document,dict):continue
        decision=document.get('decision',{});metadata=document.get('metadata',{});payload=document.get('payload')
        if not isinstance(decision,dict) or not isinstance(metadata,dict) or not isinstance(payload,dict):continue
        nonce=decision.get('pending_reference_stamp')
        if (decision.get('request_id')!=rid or decision.get('navigation_ground_truth_used') is not False
            or not isinstance(nonce,(list,tuple)) or len(nonce)!=2
            or any(type(n)is not int for n in nonce) or nonce[0]<0 or not 0<=nonce[1]<1000000000
            or metadata.get('schema')!=1 or metadata.get('reference_stamp')!=list(nonce)
            or metadata.get('trajectory')!=payload
            or decision.get('association_payload_id')!=payload.get('traj_id')):continue
        if admitted_only and decision.get('admitted') is not True:continue
        return dict(candidate_file=str(p),candidate_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                    request_id=rid,reference_stamp=list(nonce),trajectory_id=payload.get('traj_id'),
                    complete_payload_matches_metadata=True,path_admitted=decision.get('admitted') is True,
                    actual_original_admission_reason=decision.get('reason'),
                    evidence_source='actual controller association transaction archive; no synthetic ACK')
    return None

def startup_stop_decision(run,clock_ns,wall_elapsed,run_allocated_bytes):
    from first_goal_stop import first_goal_stop_decision
    return first_goal_stop_decision(run,clock_ns,wall_elapsed,run_allocated_bytes)
