#!/usr/bin/env python3
"""Offline single-anchor frame registration; never changes navigation or gates."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import numpy as np

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()

def rows(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]

def diagnose(run):
    summary_path = run/'summary_pid_navigation_independent.json'
    before = summary_path.read_bytes()
    summary = json.loads(before)
    anchor = json.loads((run/'navigation_anchor.json').read_text())
    request = json.loads((run/'navigation_request.json').read_text())
    poses = rows(run/'navigation_slam_poses.jsonl')
    matches = [p for p in poses if p['stamp_ns'] == anchor['pose_stamp_ns']]
    if len(matches) != 1:
        raise ValueError('Anchor must have exactly one original integer-stamped raw SLAM pose')
    anchor_pose = matches[0]
    if anchor_pose['position'] != anchor['origin']:
        raise ValueError('Anchor origin differs from original raw SLAM')
    helper_path = Path(__file__).with_name('analyze_pid_navigation_v3.py')
    spec = importlib.util.spec_from_file_location('registration_helpers', helper_path)
    h = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(h)
    with np.load(run/'pid_navigation_independent_arrays.npz') as z:
        a = {key: z[key] for key in z.files}
    nt = a['native_world_time_s']
    anchor_time = anchor['pose_stamp_ns']/1e9
    ai = int(np.searchsorted(nt, anchor_time, side='right')-1)
    if ai < 0 or not 0 <= anchor_time-nt[ai] <= .005001:
        raise ValueError('No causal native anchor state within one physics step')
    native_rotation = h.rotation(a['native_quaternion_wxyz'])
    slam_anchor_rotation = h.rotation(np.asarray(anchor_pose['quaternion'])[[3,0,1,2]][None,:])[0]
    world_from_slam_rotation = native_rotation[ai]@slam_anchor_rotation.T
    world_from_slam_translation = a['native_position'][ai]-world_from_slam_rotation@np.asarray(anchor['origin'])
    raw_vertices = np.asarray([anchor['origin']]+[g['center'] for g in request['goals']], dtype=float)
    registered_vertices = raw_vertices@world_from_slam_rotation.T+world_from_slam_translation
    registered_slam = a['SLAM_position']@world_from_slam_rotation.T+world_from_slam_translation
    original_vertices = a['planned_world_polyline']
    case_vector = original_vertices[1,:2]-original_vertices[0,:2]
    requested_vector = registered_vertices[1,:2]-registered_vertices[0,:2]
    case_heading = math.atan2(case_vector[1],case_vector[0])
    requested_heading = math.atan2(requested_vector[1],requested_vector[0])
    requested_error = h.path_error(a['native_position'], registered_vertices)
    original_error = a['lateral_error_m']
    actual_phase = nt >= anchor_time
    drive = a['drive_mask']
    native_indices = np.searchsorted(nt, a['SLAM_stamp_ns']/1e9, side='right')-1
    valid = native_indices >= 0
    clipped = np.clip(native_indices,0,len(nt)-1)
    valid &= (a['SLAM_stamp_ns']/1e9-nt[clipped] >= 0)
    valid &= (a['SLAM_stamp_ns']/1e9-nt[clipped] <= .005001)
    alignment_error = np.linalg.norm(registered_slam[valid]-a['native_position'][clipped[valid]],axis=1)
    yaw = h.rpy(a['native_quaternion_wxyz'])[:,2]
    projection = (a['native_position'][:,:2]-registered_vertices[0,:2])@requested_vector/(requested_vector@requested_vector)
    k = int(np.argmax(original_error))
    result = dict(schema='pid_single_anchor_offline_route_registration/v1', run=str(run),
        purpose='Distinguish originally issued SLAM route geometry from fixed physical-case reference; diagnosis only.',
        original_formal_status=summary['status'], formal_acceptance_reference_changed=False,
        original_protocol_thresholds_changed=False, registration_used_for_navigation=False,
        registration_method='One immutable full rigid transform from original integer raw SLAM anchor pose and causal native body-origin pose; never refitted.',
        anchor=dict(original_stamp_ns=anchor['pose_stamp_ns'], frame_id=anchor_pose['frame_id'],
            original_slam_position=anchor_pose['position'], original_slam_quaternion_xyzw=anchor_pose['quaternion'],
            native_world_time_s=float(nt[ai]), native_time_gap_s=float(anchor_time-nt[ai]),
            native_position=a['native_position'][ai].tolist(),
            native_quaternion_wxyz=a['native_quaternion_wxyz'][ai].tolist(),
            native_yaw_rad=float(yaw[ai]), slam_yaw_rad=anchor['yaw']),
        world_from_slam_rotation=world_from_slam_rotation.tolist(),
        world_from_slam_translation=world_from_slam_translation.tolist(),
        raw_frozen_request_polyline=raw_vertices.tolist(),
        offline_projected_original_request_polyline=registered_vertices.tolist(),
        unchanged_physical_case_polyline=original_vertices.tolist(),
        geometry=dict(case_world_heading_rad=case_heading,
            projected_request_world_heading_rad=requested_heading,
            initial_heading_disagreement_rad=float(h.wrap(requested_heading-case_heading)),
            frozen_frame_yaw_offset_rad=math.atan2(world_from_slam_rotation[1,0],world_from_slam_rotation[0,0]),
            original_goal_world_lateral_offset_from_case_m=float(registered_vertices[1,1]-original_vertices[1,1]),
            original_goal_world_XY_distance_from_case_goal_m=float(np.linalg.norm(registered_vertices[1,:2]-original_vertices[1,:2])),
            originally_issued_route_coincides_with_physical_case_axis=False,
            equality_is_diagnostic_not_acceptance_gate=True),
        original_fixed_case=dict(maximum_bounded_route_distance_m=float(np.max(original_error)),
            active_drive_RMS_m=float(np.sqrt(np.mean(original_error[drive]**2))),
            maximum_drive_heading_rad=summary['checks']['physical_route_tracking_and_actual_motion']['maximum_drive_heading_error_rad'],
            largest_error_world_time_s=float(nt[k]), largest_error_native_position=a['native_position'][k].tolist()),
        relative_to_original_issued_route_diagnostic=dict(
            maximum_bounded_route_distance_since_anchor_m=float(np.max(requested_error[actual_phase])),
            active_drive_bounded_RMS_m=float(np.sqrt(np.mean(requested_error[drive]**2))),
            maximum_drive_heading_to_projected_reference_rad=float(np.max(abs(h.wrap(yaw[drive]-requested_heading)))),
            maximum_projected_along_route_fraction=float(np.max(projection[actual_phase])),
            does_not_reclassify_original_failed_case=True),
        offline_Slam_to_native_alignment=dict(actual_integer_stamp_pairs=int(valid.sum()),
            maximum_3D_origin_position_error_m=float(np.max(alignment_error)),
            RMS_3D_origin_position_error_m=float(np.sqrt(np.mean(alignment_error**2))),
            no_navigation_truth_substitution=True),
        input_sha256={p.name:sha(p) for p in [summary_path,run/'navigation_anchor.json',run/'navigation_request.json',
            run/'navigation_slam_poses.jsonl',run/'pid_navigation_independent_arrays.npz']},
        script_sha256=sha(__file__), frozen_analyzer_sha256=sha(helper_path),
        limitation='Registration explains a reference mismatch but does not authorize post-run goal rotation or establish a real-map centerline. Formal failure remains.')
    result['geometry']['originally_issued_route_coincides_with_physical_case_axis'] = bool(abs(h.wrap(requested_heading-case_heading)) < 1e-9 and np.max(abs(registered_vertices[:,:2]-original_vertices[:,:2])) < 1e-9)
    output = run/'pid_route_registration_diagnostic.json'
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3,1,figsize=(13,12),constrained_layout=True)
    fig.suptitle(f'Actual original-request registration | original strict case remains {summary["status"].upper()}\n{run.name}',fontsize=12)
    axes[0].plot(a['native_position'][:,0],a['native_position'][:,1],label='Actual native body origin (offline)',color='#315b99')
    axes[0].plot(original_vertices[:,0],original_vertices[:,1],'--',label='Unchanged physical-case reference',color='#b53e3e')
    axes[0].plot(registered_vertices[:,0],registered_vertices[:,1],'--',label='Original issued SLAM request; offline one-anchor transform',color='#57944b')
    axes[0].plot(registered_slam[:,0],registered_slam[:,1],alpha=.7,lw=.8,label='Actual raw SLAM; same immutable offline transform',color='#b58b39')
    axes[0].set_aspect('equal',adjustable='datalim')
    axes[0].set_xlabel('Gazebo world x (m); analysis frame only')
    axes[0].set_ylabel('world y (m)')
    axes[0].legend(fontsize=9)
    axes[1].plot(nt, np.where(drive,yaw,np.nan),label='Native yaw only during actual PID drive',color='#315b99')
    axes[1].axhline(case_heading,ls='--',label='Unchanged physical-case heading',color='#b53e3e')
    axes[1].axhline(requested_heading,ls='--',label='Projected original issued SLAM leg heading',color='#57944b')
    axes[1].set_ylabel('yaw (rad)')
    axes[1].legend(fontsize=9)
    axes[2].plot(nt,original_error,label='Distance to unchanged physical case; formal gate',color='#b53e3e')
    axes[2].plot(nt,requested_error,label='Distance to original issued route; extra diagnostic',color='#57944b')
    axes[2].set_ylabel('bounded XY polyline distance (m)')
    axes[2].legend(fontsize=9)
    for axis in axes[1:]:
        axis.set_xlim(0,min(float(nt[-1]),65))
        axis.set_xlabel('actual native world simulation time (s)')
    for axis in axes:
        axis.grid(alpha=.2)
    figure = run/'pid_route_registration_diagnostic.png'
    fig.savefig(figure,dpi=150,bbox_inches='tight')
    plt.close(fig)
    proof = dict(schema='pid_route_registration_actual_figure_provenance/v1',figure=figure.name,
        figure_sha256=sha(figure),diagnostic_sha256=sha(output),input_sha256=result['input_sha256'],
        plot_script_sha256=sha(__file__),synthetic_data=False,transform_fit_count=1,
        transform_is_offline_diagnostic_only=True,formal_gate_changed=False,
        visual_QA='pending actual pixel inspection')
    (run/'pid_route_registration_figure_manifest.json').write_text(json.dumps(proof,indent=2)+'\n')
    if summary_path.read_bytes()!=before:
        raise RuntimeError('Original formal summary changed during diagnosis')
    print(json.dumps(dict(output=str(output),geometry=result['geometry'],
        issued_route_diagnostic=result['relative_to_original_issued_route_diagnostic'])))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    diagnose(parser.parse_args().run.resolve())
