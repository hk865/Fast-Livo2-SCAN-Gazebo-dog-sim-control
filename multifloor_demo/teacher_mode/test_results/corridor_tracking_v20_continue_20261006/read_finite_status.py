#!/usr/bin/env python3
"""One bounded read of atomic navigation_status.json; no polling or acceptance."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path

MAX_BYTES = 2 * 1024 * 1024


def select(value, fields):
    value = value if isinstance(value, dict) else {}
    return {key: value.get(key) for key in fields}


def extract(run):
    run = Path(run).resolve()
    path = run/'navigation_status.json'
    # Atomic rename cannot cause a partial old/new mix within this opened fd.
    with path.open('rb') as stream:
        raw = stream.read(MAX_BYTES+1)
    if len(raw) > MAX_BYTES:
        raise ValueError('Status exceeds fixed 2 MiB read bound; no full-log fallback.')
    row = json.loads(raw)
    if not isinstance(row, dict): raise ValueError('Status is not an object')
    cascade = row.get('cascade_parking') or {}
    receipts = row.get('region_arrivals') or []
    data = dict(schema='v20_single_status_observation/v1',run=str(run),
        observed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        source=dict(file=str(path),bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()),
        progress=select(row,('request_id','state','waypoint_index','total','message',
            'goal_activated_ros_clock_ns','accepted_trajectory_id','replans')),
        observed_region_receipts_count=len(receipts),
        observed_region_goal_ids=[r.get('goal_id') for r in receipts],
        observed_latest_receipt=select(receipts[-1] if receipts else None,
            ('goal_id','waypoint_index','stamp_ns','start_stamp_ns','dwell_ns')),
        heading_gate=select(row.get('teacher_transition'),
            ('phase','measured_stop','stop_valid_pose_samples','stop_first_stamp_s','stop_last_stamp_s')),
        recorded_protection=select(row,('obstacle_hold','tilt_hold','tilt_source','alignment_hold',
            'pose_age','cloud_age','raw_imu_age','command','feedback_source')),
        execution_bridge=select(row.get('execution_bridge_safety'),('state','reason','message')),
        cascade=select(cascade,('mode','reason','compute_clock_ns','progress_m','path_id',
            'parking_capture_clock_ns','parking_hold_declared_clock_ns','parking_hold_declared_pose_stamp_ns',
            'parking_window_interrupted','protection_active','failure_latched')),
        reference=select(cascade.get('spatial_reference'),('heading_rad','cross_ref_m','curvature_per_m',
            'fallback','fallback_reason')),
        only_one_small_atomic_file_read=True,run_or_production_modified=False,
        prefix9_acceptance='unverified',raw_dwell_recomputed=False,first5s_parking_verified=False,
        whole_200hz_physics_verified=False,
        limits=['One instantaneous status is not route/parking acceptance.',
            'Receipt count is controller-observed progress until independently checked against raw SLAM.',
            'No reset frequency or sustained physical behavior is inferred from this snapshot.'])
    return data


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    data=extract(args.run)
    raw=json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n'
    if args.output:
        if args.output.resolve().is_relative_to(args.run.resolve()):
            parser.error('Output must be outside the run archive')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x') as stream:stream.write(raw)
    print(raw,end='')


if __name__=='__main__':main()
