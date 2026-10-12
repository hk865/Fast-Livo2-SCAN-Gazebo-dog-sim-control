"""Bounded background-only decode/archive with explicit evidence failures."""
from __future__ import annotations
from collections import Counter, deque
import json
from pathlib import Path
import queue
import threading
import time
import numpy as np
from cloud import GravityCalibration, decode_cloud, sha


def atomic_json(path, data):
    path = Path(path); tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n'); tmp.replace(path)


def temporal_admission(row, pose):
    reasons = []
    stamp, clock = row['original_stamp_ns'], row.get('received_ros_clock_ns')
    if row.get('frame_id') != 'camera_init': reasons.append('non_camera_init_cloud')
    if clock is None or not -50_000_000 <= clock-stamp < 300_000_000: reasons.append('cloud_header_not_fresh_300ms')
    clock_age = row.get('received_clock_wall_age_s')
    if clock_age is None or not 0 <= clock_age < .3: reasons.append('actual_received_clock_not_fresh_300ms')
    if pose is None: reasons.append('missing_actual_slam_body_pose')
    else:
        if abs(stamp-pose['original_stamp_ns']) > 150_000_000: reasons.append('no_actual_body_pose_within_150ms')
        if not 0 <= row['received_monotonic_wall']-pose['received_monotonic_wall'] < .3:
            reasons.append('body_pose_wall_not_fresh_300ms')
        if pose.get('frame_id') != 'camera_init' or pose.get('child_frame_id') != 'demo_slam_body':
            reasons.append('unexpected_actual_slam_body_frames')
    return reasons


class CaptureWriter:
    def __init__(self, output, *, capacity=512, max_queue_bytes=64*1024**2, max_archive_bytes=8*1024**3,
                 max_frame_bytes=32*1024**2):
        self.output=Path(output); self.output.mkdir(exist_ok=False,parents=True); (self.output/'clouds').mkdir()
        self.queue=queue.Queue(capacity); self.max_queue_bytes=max_queue_bytes
        self.max_archive_bytes=max_archive_bytes; self.max_frame_bytes=max_frame_bytes
        self.lock=threading.Lock(); self.queue_bytes=0; self.peak_queue_bytes=0
        self.error=None; self.errors=[]; self.closed=False; self.submitted=0; self.processed=0
        self.counts=Counter(); self.clouds=[]; self.archive_bytes=0; self.histories={n:deque(maxlen=256) for n in ('body_odom','imu_slam_odom')}
        self.gravity=GravityCalibration(); self.anchor=None; self.last_cloud_stamp=None
        self.streams={}; self.thread=threading.Thread(target=self._run,name='actual-ramp-cloud-archive',daemon=True); self.thread.start()

    def fail(self, reason):
        with self.lock:
            self.error=self.error or reason; self.errors.append(reason)

    def append(self, row, raw=None):
        # Called from lightweight ROS callbacks; no decoding/hash/disk/JSON encoding.
        size=(len(raw) if raw is not None else 0)+2048
        with self.lock:
            if self.closed or self.error: raise RuntimeError(self.error or 'capture writer closed')
            if raw is not None and len(raw)>self.max_frame_bytes:
                self.error='cloud payload exceeds frozen per-frame limit'; self.errors.append(self.error); raise RuntimeError(self.error)
            if self.queue_bytes+size>self.max_queue_bytes:
                self.error='capture byte queue overflow; evidence incomplete'; self.errors.append(self.error); raise RuntimeError(self.error)
            self.queue_bytes+=size; self.peak_queue_bytes=max(self.peak_queue_bytes,self.queue_bytes)
        try: self.queue.put_nowait((row,raw,size))
        except queue.Full:
            with self.lock: self.queue_bytes-=size
            self.fail('capture record queue overflow; evidence incomplete'); raise RuntimeError(self.error)
        self.submitted+=1

    def nearest(self, name, stamp):
        history=self.histories[name]
        return min(history,key=lambda r:abs(r['original_stamp_ns']-stamp)) if history else None

    def latest_source_past(self,name,stamp):
        valid=[r for r in self.histories[name]if r['original_stamp_ns']<=stamp]
        return max(valid,key=lambda r:r['original_stamp_ns'])if valid else None

    def _line(self, name, row):
        if name not in self.streams: self.streams[name]=(self.output/(name+'.jsonl')).open('x')
        self.streams[name].write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')

    def process(self, row, raw):
        stream=row['stream']; self.counts[stream]+=1
        row=dict(row); row['processed_monotonic_wall']=time.monotonic()
        row['background_queue_delay_wall_s']=row['processed_monotonic_wall']-row['received_monotonic_wall']
        if stream in self.histories:
            values=row['position']+row['quaternion']+row.get('body_linear_velocity',[])
            values+=row.get('pose_covariance',[])+row.get('twist_covariance',[])
            history=self.histories[stream];child='demo_slam_body' if stream=='body_odom' else 'aft_mapped'
            accepted=(np.isfinite(values).all() and row['frame_id']=='camera_init' and row['child_frame_id']==child
                and .98<=sum(v*v for v in row['quaternion'])<=1.02
                and (not history or row['original_stamp_ns']>history[-1]['original_stamp_ns']))
            row['pose_admitted']=bool(accepted)
            if accepted:history.append(row)
            else:row['pose_rejection_reason']='invalid_frame_values_quaternion_or_nonincreasing_stamp'
        elif stream=='imu':
            ip=self.latest_source_past('imu_slam_odom',row['original_stamp_ns']); bp=self.latest_source_past('body_odom',row['original_stamp_ns'])
            clock=row.get('received_ros_clock_ns');ca=row.get('received_clock_wall_age_s')
            fresh=(clock is not None and -50_000_000<=clock-row['original_stamp_ns']<300_000_000
                and ca is not None and 0<=ca<.3)
            if ip is not None and bp is not None and self.gravity.result is None and fresh:
                row['gravity_pairing']={'slam_imu_pose_stamp_ns':ip['original_stamp_ns'],'body_pose_stamp_ns':bp['original_stamp_ns'],
                    'slam_imu_quaternion':ip['quaternion'],'slam_imu_pose_received_wall':ip['received_monotonic_wall'],
                    'body_pose_received_wall':bp['received_monotonic_wall'],'body_linear_velocity':bp['body_linear_velocity']}
                if self.gravity.observe(row,ip,bp) is not None: atomic_json(self.output/'gravity.json',self.gravity.result)
        elif stream=='full_cloud':
            decode_error=None
            try:fields,xyz,indices=decode_cloud(row,raw)
            except ValueError as exc:
                decode_error=str(exc);fields=np.empty(0,dtype=np.uint8);xyz=np.empty((0,3),dtype=np.float64);indices=np.empty(0,dtype=np.int64)
            body=self.nearest('body_odom',row['original_stamp_ns'])
            reasons=temporal_admission(row,body)
            if decode_error is not None:reasons.append('invalid_PointCloud2_field_layout_or_empty_payload: '+decode_error)
            if not len(xyz):reasons.append('no_finite_actual_XYZ')
            if self.last_cloud_stamp is not None and row['original_stamp_ns']<=self.last_cloud_stamp:
                reasons.append('cloud_header_not_strictly_increasing')
            self.last_cloud_stamp=max(row['original_stamp_ns'],self.last_cloud_stamp or row['original_stamp_ns'])
            if self.gravity.result is None: reasons.append('actual_gravity_not_initialized')
            payload_size=len(raw)+fields.nbytes+xyz.nbytes+indices.nbytes
            if self.archive_bytes+payload_size>self.max_archive_bytes: raise RuntimeError('capture archive byte limit; no silent truncation')
            filename=f"clouds/{self.counts[stream]:06d}_{row['original_stamp_ns']}.npz"
            destination=self.output/filename
            np.savez(destination,raw=np.frombuffer(raw,dtype=np.uint8),fields=fields,xyz=xyz,finite_source_indices=indices)
            self.archive_bytes+=destination.stat().st_size
            row.update(archive_file=filename,archive_sha256=sha(destination.read_bytes()),raw_payload_sha256=sha(raw),
                raw_payload_bytes=len(raw),decoded_xyz_dtype=xyz.dtype.str,decoded_xyz_shape=list(xyz.shape),
                decoded_xyz_sha256=sha(xyz.tobytes()),original_structured_dtype=fields.dtype.descr,
                finite_points=len(xyz),nonfinite_points=len(fields)-len(xyz),decode_error=decode_error,
                body_pose=body,geometry_admitted=not reasons,geometry_rejection_reasons=reasons,
                gravity_receipt=self.gravity.result,source_topic='/cloud_registered_full',
                color_cloud_used=False,ground_truth_used=False)
            if not reasons and self.anchor is None: self.anchor=body
            self.clouds.append(row)
        self._line(stream,row)

    def _run(self):
        try:
            while True:
                item=self.queue.get()
                try:
                    if item is None: break
                    row,raw,size=item
                    if not self.error:
                        try: self.process(row,raw); self.processed+=1
                        except Exception as exc: self.fail(f'archive processing failed: {type(exc).__name__}: {exc}')
                finally:
                    if item is not None:
                        with self.lock: self.queue_bytes-=item[2]
                    self.queue.task_done()
        finally:
            for stream in self.streams.values():
                try: stream.flush(); stream.close()
                except Exception as exc: self.fail(f'archive close failed: {type(exc).__name__}: {exc}')

    def close(self):
        self.closed=True
        try: self.queue.put(None,timeout=2.)
        except queue.Full: self.fail('capture queue could not close')
        self.thread.join(timeout=10.)
        if self.thread.is_alive(): self.fail('capture writer did not drain within 10 wall seconds')
        if self.processed!=self.submitted: self.fail('capture submitted/processed mismatch')
        if self.error: raise RuntimeError(self.error)


def finish(writer, base, *, failure=None, stop_reason='duration_complete'):
    """Always attempt drain and failed manifest, including earlier writer errors."""
    errors=[] if failure is None else [str(failure)]
    try: writer.close()
    except Exception as exc: errors.append(str(exc))
    errors.extend(writer.errors)
    manifest=dict(base,status='failed' if errors else 'observation_complete' if stop_reason=='duration_complete' else 'interrupted',
        stop_reason=stop_reason,errors=list(dict.fromkeys(errors)),counts=dict(writer.counts),
        submitted=writer.submitted,processed=writer.processed,writer_drained=not writer.thread.is_alive(),
        peak_queue_bytes=writer.peak_queue_bytes,archive_bytes=writer.archive_bytes,
        admitted_clouds=sum(r['geometry_admitted'] for r in writer.clouds),recorded_clouds=len(writer.clouds),
        body_anchor=writer.anchor,gravity=writer.gravity.result,clouds=writer.clouds,
        navigation_verified=False,control_outputs=0,ground_truth_used=False)
    if not manifest['admitted_clouds']: manifest['registration_readiness']='unverified_no_admitted_clouds'
    else: manifest['registration_readiness']='offline_analysis_required'
    try: atomic_json(writer.output/'capture_manifest.json',manifest)
    except Exception as exc: errors.append(f'manifest write failed: {exc}')
    if errors: raise RuntimeError('; '.join(dict.fromkeys(errors)))
    return manifest
