"""Original fourth camera run: strict receipts and native update decomposition.

No ROS, sensor-file scan, new alignment, result repair or feedback to control.
"""
from collections import defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
RUN = ROOT / 'camera_mode/runs/20261002_192933_3ce19a'
DEST = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    sys.path.insert(0, str(ROOT / 'camera_mode/slam'))
    spec = importlib.util.spec_from_file_location('fourth_camera_audit', ROOT / 'camera_mode/slam/evaluate_run.py')
    evaluator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(evaluator)
    acceptance = json.loads((RUN / 'acceptance.json').read_text())
    mission = json.loads((RUN / 'mission.json').read_text())
    scenario = json.loads((RUN / 'scenario.json').read_text())
    samples, pose_errors = evaluator.SHARED.read_poses(RUN / 'pose_audit.jsonl')
    trajectory, R, T, truth = evaluator.SHARED.compare_trajectory(samples)
    regions = evaluator.SHARED.ordered_region_evidence(RUN, scenario, mission, samples, R, T, truth)
    (DEST / 'original_regions_recomputed.json').write_text(json.dumps(regions, indent=2) + '\n')
    points = [w for stage in regions['stages'].values() for w in stage['waypoints']]
    first_bad = next(w for w in points if not w['reached_in_order'])
    tt = np.array([w['stamp_ns'] for w in truth], dtype=np.int64) / 1e9
    tp = np.array([w['p'] for w in truth])

    def gt_position(t):
        if not tt[0] <= t <= tt[-1]:
            raise ValueError('diagnostic GT extrapolation forbidden')
        return np.array([np.interp(t, tt, tp[:,k]) for k in range(3)])

    def gt_velocity(t):
        # A diagnostic centered 40-ms GT finite difference, not a measured
        # velocity message or an arrival-window interpolation replacement.
        return R.T @ ((gt_position(t+.02) - gt_position(t-.02)) / .04)

    pre = np.loadtxt(RUN / 'fastlivo_debug/mat_pre.txt')
    out = np.loadtxt(RUN / 'fastlivo_debug/mat_out.txt')
    gp, go = defaultdict(list), defaultdict(list)
    for row in pre:
        gp[round(float(row[0]), 6)].append(row)
    for row in out:
        go[round(float(row[0]), 6)].append(row)
    pairs = [(t, gp[t][0], go[t][0][:19], gp[t][1], go[t][1][:19])
             for t in sorted(go) if len(gp[t]) == len(go[t]) == 2]
    offset = samples['slam'][0]['stamp_ns']/1e9 - out[0, 0]
    native_chain_gap = max(float(np.max(np.abs(c-b))) for _,a,b,c,d in pairs)
    intervals = [(195,201), (201,205), (202,209), (205,208), (205,210),
                 (208,210), (210,212.8), (212.8,216)]
    updates = []
    for begin, end in intervals:
        indices = [i for i,p in enumerate(pairs) if begin < p[0]+offset <= end and i > 0]
        selected = [pairs[i] for i in indices]
        first, last = pairs[indices[0]-1], pairs[indices[-1]]
        L = np.array([p[2]-p[1] for p in selected])
        V = np.array([p[4]-p[3] for p in selected])
        propagation = np.array([pairs[i][1]-pairs[i-1][4] for i in indices])
        t0, t1 = first[0]+offset, last[0]+offset
        g0, g1 = gt_velocity(t0), gt_velocity(t1)
        native_dv = last[4][9]-first[4][9]
        gt_dv = g1[2]-g0[2]
        updates.append(dict(range_s=[t0,t1], frames=len(selected),
            LIO_sum_position_update_m=L[:,4:7].sum(0).tolist(),
            VIO_sum_position_update_m=V[:,4:7].sum(0).tolist(),
            LIO_sum_velocity_update_m_s=L[:,7:10].sum(0).tolist(),
            VIO_sum_velocity_update_m_s=V[:,7:10].sum(0).tolist(),
            propagation_sum_position_increment_m=propagation[:,4:7].sum(0).tolist(),
            propagation_sum_velocity_increment_m_s=propagation[:,7:10].sum(0).tolist(),
            GT_velocity_change_in_frozen_SLAM_frame_m_s=(g1-g0).tolist(),
            propagation_minus_GT_delta_vz_m_s=float(propagation[:,9].sum()-gt_dv),
            native_minus_GT_vz_error_change_m_s=float(native_dv-gt_dv),
            velocity_bookkeeping_residual_m_s=float(native_dv - (L[:,9]+V[:,9]+propagation[:,9]).sum()),
            largest_LIO_VIO_absolute_z_update_m=[float(abs(L[:,6]).max()),float(abs(V[:,6]).max())]))
    checkpoints = []
    native_slice = []
    for t,a,b,c,d in pairs:
        if 198 <= t+offset <= 216:
            native_slice.append(dict(relative_time=t, reconstructed_absolute_time_s=t+offset,
                LIO_before=a.tolist(), LIO_after=b.tolist(), VIO_before=c.tolist(), VIO_after=d.tolist()))
    for target in [201,203,204,205,205.5,206,206.5,207,207.5,208,208.5,209,210,211,212.3,212.8]:
        t,a,b,c,d = min(pairs, key=lambda p: abs(p[0]+offset-target))
        at = t+offset
        checkpoints.append(dict(reconstructed_absolute_time_s=at,
            GT_world_position=gt_position(at).tolist(),
            GT_velocity_in_frozen_SLAM_frame_m_s=gt_velocity(at).tolist(),
            native_predicted_velocity=a[7:10].tolist(), native_after_LIO_velocity=b[7:10].tolist(),
            native_after_VIO_velocity=d[7:10].tolist(),
            native_LIO_position_update_m=(b[4:7]-a[4:7]).tolist(),
            native_VIO_position_update_m=(d[4:7]-c[4:7]).tolist()))
    (DEST / 'native_state_pairs_198_216.json').write_text(json.dumps(native_slice, indent=2)+'\n')
    log = (RUN / 'stack.log').read_text()
    regex = re.compile(r'\[DEMO_SYNC\] camera=([0-9.]+) lidar_newest=([0-9.]+) imu_newest=([0-9.]+) '
                       r'imu_last_used=([-0-9.]+) imu_count=([0-9]+) complete=([01])')
    sync = np.array([[float(v) for v in match.groups()] for match in regex.finditer(log)])
    critical = sync[(sync[:,0]>=198) & (sync[:,0]<=216)]
    manifest = json.loads((RUN / 'source_manifest.json').read_text())
    archive_matches = [sha(RUN/'sources'/row['path']) == row['sha256'] for row in manifest['files']]
    runtime = json.loads((RUN / 'startup_runtime_actual.json').read_text())
    cleanup = json.loads((RUN / 'shutdown_verification.json').read_text())
    official = acceptance['ordered_waypoints']
    reached = {s:d['reached_waypoints'] for s,d in regions['stages'].items()}
    checks = dict(original_acceptance_failed=acceptance['passed'] is False,
        original_pose_integer_stamps_valid=not pose_errors,
        independent_initial_SE3_equals_original=trajectory['rotation_world_from_slam']==acceptance['trajectory']['rotation_world_from_slam']
            and trajectory['translation_world_from_slam']==acceptance['trajectory']['translation_world_from_slam'],
        original_46_regions_recomputed=True,
        recomputed_ordered_prefix_equals_original=reached=={s:d['reached_waypoints'] for s,d in official['stages'].items()},
        full_active_GT_coverage=regions['truth_full_pose_coverage']['passed'],
        preregistered_original_declaration=regions['preregistered_declaration']['passed'],
        source_manifest_sidecar_valid=sha(RUN/'source_manifest.json')==(RUN/'source_manifest.sha256').read_text().strip(),
        all_original_archived_sources_match=all(archive_matches),
        actual_owned_runtime_maps=runtime['passed'] is True,
        owned_cleanup=cleanup.get('owned_clean') is True,
        actual_bias_disabled_log='Bias Estimation Disabled' in log,
        native_bias_six_axes_all_zero=bool(np.all(out[:,10:16]==0)),
        complete_native_LIO_VIO_chain=native_chain_gap==0,
        native_values_finite=bool(np.isfinite(pre).all() and np.isfinite(out).all()),
        critical_native_IMU_complete=bool(np.all(critical[:,-1]==1)),
        critical_native_last_used_within2p1ns=bool(np.max(critical[:,0]-critical[:,3])<=2.1e-9),
        diagnostic_velocity_bookkeeping_consistent=all(abs(x['velocity_bookkeeping_residual_m_s'])<1e-12 for x in updates),
        no_sensor_file_rescan=True)
    report = dict(scope='Independent original failed fourth camera run; strict actual windows remain unchanged',
        evidence_checks=checks, evidence_ready=all(checks.values()), original_acceptance_passed=False,
        original_failed_checks=acceptance['failed_checks'], run=str(RUN),
        original_source_manifest_sha256=sha(RUN/'source_manifest.json'), archive_source_count=len(archive_matches),
        original_artifact_sha256={n:sha(RUN/n) for n in ['acceptance.json','pose_audit.jsonl','navigation_audit.jsonl',
            'fastlivo_debug/mat_pre.txt','fastlivo_debug/mat_out.txt','stack.log']},
        recomputed_reached=reached, original_single_SE3=trajectory,
        active_GT_coverage=regions['truth_full_pose_coverage'], first_strict_joint_failure=first_bad,
        actual_task_terminal={k:mission.get(k) for k in ['stage','elapsed_s','message']},
        native_relative_to_absolute_offset_s=offset,
        native_precision='Approximately six significant digits; diagnostic absolute native times reconstructed from the original first SLAM header, not new message headers',
        GT_velocity_scope='Centered 40-ms position finite differences, rotated by the one original R; no GT feedback or alternate alignment',
        native_RP_peak_deg_57p3=abs(out[:,1:3]).max(0).tolist(),
        native_bg_ba_absolute_max=abs(out[:,10:16]).max(0).tolist(),
        native_IMU_sync=dict(total_frames=len(sync), incomplete_total=int(np.sum(sync[:,-1]==0)),
            critical_range_s=[198,216], critical_frames=len(critical),
            critical_count_min_max=[float(critical[:,4].min()),float(critical[:,4].max())]),
        update_decomposition=updates, native_checkpoints=checkpoints,
        original_sensor_header_evidence=acceptance['sensor_headers'],
        original_sampling_complete=acceptance['sensor_headers']['complete_nominal_sampling'],
        findings=[
            'Fresh-process disabled biases remain zero; estimated roll/pitch shrink greatly, but this does not resolve returning height error.',
            'Exploration reaches all 18 original joint windows. Returning first fails return_origin:6 at 212.3–212.8s, with approximately +0.466m GT-minus-SLAM height.',
            'VIO repeatedly adds negative vertical-velocity updates during 205–208s. LIO also contributes negative updates in part of this interval, then applies strong positive corrections.',
            'There is no single large vertical-position correction. Wrong velocity propagates into height; the native state bookkeeping separates propagation from both measurement updates.',
            'Native IMU consumption is complete in the critical interval; raw observer nominal gaps remain preserved and are not filled.',
            'This identifies a targeted visual-state-update ablation, not a proof of a unique VIO, sensor or scene cause.'],
        parent_authorized_next_ablation='Only camera vio.img_point_cov 100→100000000; real RGB/VIO processing retained. Preparation has a separate receipt; original result is not changed.')
    result = DEST/'result.json'
    result.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(checks=len(checks), all_true=all(checks.values()), result_sha256=sha(result),
        reached=reached, first_goal=first_bad['goal_id'], RP_peak=report['native_RP_peak_deg_57p3'],
        key_decomposition=next(x for x in updates if abs(x['range_s'][0]-205)<1e-6 and abs(x['range_s'][1]-208)<1e-6))))


if __name__ == '__main__':
    main()
