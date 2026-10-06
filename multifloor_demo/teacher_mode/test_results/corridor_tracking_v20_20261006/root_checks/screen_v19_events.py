#!/usr/bin/env python3
"""Hash-verified static screen of 104 fixed V19 events through the V20 API.

This does not roll out the dynamics, replay PI, count future gate resets,
validate a corridor, or prove navigation. Archived inputs are read only.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

TEACHER = Path(__file__).resolve().parents[3]
V20 = TEACHER/'navigation'/'corridor_tracking_v20'
AUDIT = TEACHER/'test_results'/'pipeline_v19_20261006'/'evaluation'/'V19_9779_HEADING_RESET_PROJECTION_AUDIT.json'
AUDIT_SHA256 = '826bf321a5ff4464dfc41f22824158e4535bf8dceaf7895e351d9a39ba5e7a8d'
sys.path.insert(0,str(V20))
from cascade_core import Controller


def sha_bytes(value):return hashlib.sha256(value).hexdigest()
def sha_file(path):return sha_bytes(Path(path).read_bytes())
def array_sha(value):return sha_bytes(np.asarray(value,dtype='<f8').tobytes())


def require(condition, reason):
    if not condition:raise ValueError(reason)


def read_original_row(binding,run):
    path=Path(binding['source_file']).resolve()
    require(path.is_relative_to(run),'Source row leaves frozen run')
    with path.open('rb') as stream:
        if binding['source_offset']:
            stream.seek(binding['source_offset']-1)
            require(stream.read(1)==b'\n','Source offset is not a line boundary')
        stream.seek(binding['source_offset'])
        raw=stream.read(binding['source_length'])
    require(raw.endswith(b'\n') and raw.count(b'\n')==1,'Source length is not one JSONL row')
    require(sha_bytes(raw)==binding['source_line_sha256'],'Original source line SHA mismatch')
    row=json.loads(raw)
    require(row['sequence']==binding['source_line'],'Original row sequence/line mismatch')
    return row


def screen_event(event,run,config):
    previous=read_original_row(event['previous']['source'],run)
    current=read_original_row(event['current']['source'],run)
    require(previous['sequence']==event['previous']['sequence'],'Previous sequence changed')
    require(current['sequence']==event['current']['sequence'],'Current sequence changed')
    require(previous['heading_gate_reference']['phase']=='drive','Original previous phase changed')
    require(current['heading_gate_reference']['phase']=='pre_turn','Original current phase changed')
    require(current['trajectory_id']==event['current']['trajectory_id'],'Trajectory identity mismatch')
    require(np.array_equal(current['control_pose'],event['current']['pose']),'Original pose changed')
    pose=np.asarray(current['control_pose'],float)
    rotation=np.asarray(current['control_rotation'],float)
    yaw=math.atan2(rotation[1,0],rotation[0,0])
    require(abs(yaw-event['current']['actual_yaw'])<1e-12,'Original yaw mismatch')
    receipt=current['path_receipt']
    archive=Path(event['path_archive']).resolve()
    require(archive.is_relative_to(run),'Path archive leaves original run')
    require((run/receipt['array_file']).resolve()==archive,'Source row/path archive association mismatch')
    digest=sha_file(archive)
    require(digest==event['path_archive_sha256'],'Original NPZ SHA mismatch')
    sidecar=archive.with_suffix('.json')
    metadata=json.loads(sidecar.read_text())
    require(metadata['array_sha256']==digest,'Original archive sidecar SHA mismatch')
    require(metadata['trajectory_id']==current['trajectory_id'],'Original archive sidecar identity mismatch')
    with np.load(archive,allow_pickle=False) as payload:
        samples=np.asarray(payload['samples'],dtype='<f8')
    require(len(samples)==receipt['source_samples_count'],'Original sample count changed')
    require(array_sha(samples)==receipt['source_samples_float64_sha256'],'Original full sample bytes changed')
    indices=np.asarray(receipt['source_sample_indices'],int)
    require(len(indices)>1 and np.all(np.diff(indices)>0) and indices[0]>=0 and indices[-1]<len(samples),
        'Invalid original accepted sample indices')
    points=samples[indices]
    points_sha=array_sha(points)
    require(points_sha==receipt['points_float64_sha256']==event['frozen_path_points_sha256'],
        'Original accepted point bytes changed')
    fixed_goal=current['cascade']['fixed_goal']
    core=Controller(config,fixed_goal['position_world_xyz'],fixed_goal['heading_rad'],fixed_goal['goal_id'])
    core.set_path(points,receipt['path_id'],receipt['stamp_ns'],receipt['received_wall_ns'],receipt['frame_id'],position=pose)
    original_projection=current['heading_gate_reference']['steering']['cascade_projection']
    require(core.path_sha==original_projection['path_sha256'],'Immutable path record SHA mismatch')
    reference=core.preview_reference(pose)
    raw=reference['raw_projection']
    require(raw['segment']==original_projection['segment']==event['projection']['segment'],'Original projected segment changed')
    require(np.allclose(raw['point_xyz'],original_projection['point'],rtol=0,atol=1e-12),'Original projection point changed')
    require(np.allclose(raw['tangent_xy'],original_projection['tangent'],rtol=0,atol=1e-12),'Original tangent changed')
    require(abs(reference['nearest_s_m']-original_projection['nearest_s'])<1e-12,'Original arc projection changed')
    require(abs(reference['remaining_m']-original_projection['remaining'])<1e-12,'Original remaining distance changed')
    original_error=float(current['heading_gate_reference']['steering']['error'])
    require(abs(original_error-event['current']['actual_SCAN_yaw_error'])<1e-12,'Original heading error changed')
    error=math.atan2(math.sin(reference['heading_rad']-yaw),math.cos(reference['heading_rad']-yaw))
    return dict(sequence=current['sequence'],trajectory_id=current['trajectory_id'],sim_time=event['current']['sim_time'],
        previous_source=event['previous']['source'],current_source=event['current']['source'],
        path_archive=str(archive),path_archive_sha256=digest,path_sidecar_sha256=sha_file(sidecar),
        accepted_points_sha256=points_sha,source_rows_and_npz_and_projection_verified=True,
        original_heading_error_rad=original_error,heading_error_rad=error,absolute_error_rad=abs(error),
        over_original_0_2rad_threshold=bool(abs(error)>.2),fallback_reason=reference['fallback_reason'],
        curvature_valid=reference['curvature_valid'],reference=reference)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).with_name('V20_STATIC_104_EVENTS.json'))
    parser.add_argument('--profile',type=Path,default=V20/'profiles'/'pipeline_staged_original46.json')
    args=parser.parse_args()
    if not __debug__:raise RuntimeError('Hash-verification screen forbids optimized Python')
    if args.output.exists():raise FileExistsError('Refusing to overwrite an existing static receipt; choose a new --output')
    require(sha_file(AUDIT)==AUDIT_SHA256,'Frozen 104-event selection SHA changed')
    audit=json.loads(AUDIT.read_text());run=Path(audit['run']).resolve()
    require(len(audit['events'])==audit['drive_to_pre_turn_count']==104,'Frozen event selection changed')
    for name,digest in audit['source_bindings'].items():
        require(sha_file(name)==digest,'Original audit source binding changed: '+name)
    profile=json.loads(args.profile.read_text());config=profile['cascade']
    inputs={str(path):sha_file(path) for path in (AUDIT,args.profile.resolve(),V20/'cascade_core.py',V20/'spatial_reference.py',Path(__file__).resolve())}
    inputs.update(audit['source_bindings'])
    events=[];errors=[]
    for index,event in enumerate(audit['events']):
        try:events.append(screen_event(event,run,config))
        except (ValueError,KeyError,OSError,TypeError,IndexError) as error:
            errors.append(dict(event_index=index,sequence=event['current']['sequence'],error=str(error)))
    values=np.asarray([event['absolute_error_rad'] for event in events])
    fallback=collections.Counter(event['fallback_reason'] for event in events if event['fallback_reason'] is not None)
    report=dict(schema='corridor_tracking_v20_static_frozen_event_screen/v1',
        status='PASS_LIMITED_STATIC_SCREEN' if not errors and len(events)==104 else 'FAIL_SOURCE_OR_PROJECTION_VERIFICATION',
        scope='Only 104 originally selected V19 drive-to-pre_turn events. Original poses, accepted sampled paths and projections remain fixed. Production V20 preview_reference is called once on each immutable original input.',
        static_only=True,dynamic_rollout=False,PI_replayed=False,gate_state_machine_replayed=False,
        closed_loop_reset_count_estimated=False,corridor_or_obstacle_safety_verified=False,
        handoff_trajectory_history_replayed=False,actual_navigation_verified=False,
        profile=str(args.profile.resolve()),spatial_reference_config=config.get('spatial_reference',{}),
        source_bindings=inputs,original_event_count=104,verified_event_count=len(events),
        original_over_0_2rad=104,production_reference_over_0_2rad=int(np.sum(values>.2)),
        fallback_count=sum(fallback.values()),fallback_reasons=dict(fallback),
        curvature_invalid_count=sum(not event['curvature_valid'] for event in events),
        median_abs_error_rad=None if not len(values) else float(np.median(values)),
        p95_abs_error_rad=None if not len(values) else float(np.percentile(values,95)),
        max_abs_error_rad=None if not len(values) else float(values.max()),errors=errors,events=events)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps({key:report[key] for key in ('status','verified_event_count','production_reference_over_0_2rad',
        'fallback_count','fallback_reasons','curvature_invalid_count','median_abs_error_rad','p95_abs_error_rad','max_abs_error_rad','errors')},ensure_ascii=False))
    return 0 if not errors and len(events)==104 else 1


if __name__=='__main__':sys.exit(main())
