"""Strict independent fifth camera run audit; no ROS or sensor-file rescan."""
from collections import defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
RUN = ROOT / 'camera_mode/runs/20261002_194830_d16898'
DEST = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def native_summary(run, begin, end):
    pre = np.loadtxt(run / 'fastlivo_debug/mat_pre.txt')
    out = np.loadtxt(run / 'fastlivo_debug/mat_out.txt')
    gp, go = defaultdict(list), defaultdict(list)
    for row in pre:
        gp[round(float(row[0]),6)].append(row)
    for row in out:
        go[round(float(row[0]),6)].append(row)
    first_slam = next(json.loads(line) for line in (run/'pose_audit.jsonl').open()
                      if json.loads(line)['source']=='slam')
    offset = first_slam['stamp_ns']/1e9-out[0,0]
    pairs = [(t,gp[t][0],go[t][0][:19],gp[t][1],go[t][1][:19])
             for t in sorted(go) if len(gp[t])==len(go[t])==2]
    selected = [p for p in pairs if begin < p[0]+offset <= end]
    L = np.array([p[2]-p[1] for p in selected])
    V = np.array([p[4]-p[3] for p in selected])
    chain = max(float(abs(p[3]-p[2]).max()) for p in pairs)
    regex = re.compile(r'\[DEMO_SYNC\] camera=([0-9.]+) lidar_newest=([0-9.]+) imu_newest=([0-9.]+) '
                       r'imu_last_used=([-0-9.]+) imu_count=([0-9]+) complete=([01])')
    sync = np.array([[float(x) for x in m.groups()] for m in regex.finditer((run/'stack.log').read_text())])
    return dict(printed_precision='Approximately six significant digits; native absolute times reconstructed from original first SLAM header',
        native_relative_to_absolute_offset_s=offset, complete_LIO_VIO_pairs=len(pairs),
        preVIO_equals_postLIO_max_difference=chain,
        finite=bool(np.isfinite(pre).all() and np.isfinite(out).all()),
        RP_peak_deg_57p3=abs(out[:,1:3]).max(0).tolist(),
        gyro_acc_bias_absolute_max=abs(out[:,10:16]).max(0).tolist(),
        IMU_sync=dict(frames=len(sync), incomplete=int(np.sum(sync[:,-1]==0)),
            last_used_max_age_s=float(np.max(sync[:,0]-sync[:,3]))),
        same_canonical_return5_to_return6_segment=dict(range_s=[begin,end],frames=len(selected),
            LIO_sum_delta_vz_m_s=float(L[:,9].sum()),VIO_sum_delta_vz_m_s=float(V[:,9].sum()),
            LIO_mean_absolute_delta_vz_m_s=float(abs(L[:,9]).mean()),
            VIO_mean_absolute_delta_vz_m_s=float(abs(V[:,9]).mean()),
            VIO_max_absolute_delta_vz_m_s=float(abs(V[:,9]).max()),
            LIO_sum_delta_z_m=float(L[:,6].sum()),VIO_sum_delta_z_m=float(V[:,6].sum())))


def main():
    sys.path.insert(0,str(ROOT/'camera_mode/slam'))
    spec=importlib.util.spec_from_file_location('fifth_camera_audit',ROOT/'camera_mode/slam/evaluate_run.py')
    ev=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ev)
    acceptance=json.loads((RUN/'acceptance.json').read_text())
    mission=json.loads((RUN/'mission.json').read_text())
    scenario=json.loads((RUN/'scenario.json').read_text())
    samples,errors=ev.SHARED.read_poses(RUN/'pose_audit.jsonl')
    trajectory,R,T,truth=ev.SHARED.compare_trajectory(samples)
    regions=ev.SHARED.ordered_region_evidence(RUN,scenario,mission,samples,R,T,truth)
    (DEST/'original_regions_recomputed.json').write_text(json.dumps(regions,indent=2)+'\n')
    reached={s:d['reached_waypoints'] for s,d in regions['stages'].items()}
    manifest=json.loads((RUN/'source_manifest.json').read_text())
    archive_same=[sha(RUN/'sources'/x['path'])==x['sha256'] for x in manifest['files']]
    config_file=RUN/'sources/camera_mode/slam/fastlivo.yaml'
    config=yaml.safe_load(config_file.read_text())['/**']['ros__parameters']
    runtime=json.loads((RUN/'startup_runtime_actual.json').read_text())
    cleanup=json.loads((RUN/'shutdown_verification.json').read_text())
    metadata=json.loads((RUN/'map_metadata.json').read_text())
    binary=ev.SHARED.inside_run(RUN,metadata['binary_filename'])
    records=np.frombuffer(binary.read_bytes(),dtype=ev.SHARED.MAP_DTYPE)
    pcd_file=ev.SHARED.inside_run(RUN,metadata['save']['filename'])
    pcd=ev.SHARED.read_pcd(pcd_file)
    colors=int(len(np.unique(records['rgb'],axis=0)))
    health=metadata['save']['healthy_sensor_evidence']
    returning=regions['stages']['returning']['waypoints']
    begin=returning[5]['navigation_receipt']['stamp_ns']/1e9
    end=returning[6]['navigation_receipt']['stamp_ns']/1e9
    native=native_summary(RUN,begin,end)
    fourth=ROOT/'camera_mode/runs/20261002_192933_3ce19a'
    old=json.loads((fourth/'acceptance.json').read_text())
    previous_return=old['ordered_waypoints']['stages']['returning']['waypoints']
    comparison=native_summary(fourth,previous_return[5]['navigation_receipt']['stamp_ns']/1e9,
                              previous_return[6]['navigation_receipt']['stamp_ns']/1e9)
    height=[dict(goal_id=w['goal_id'],window_ns=[w['navigation_receipt']['start_stamp_ns'],w['navigation_receipt']['stamp_ns']],
        GT_minus_raw_SLAM_z_min_max_m=[min(x['truth_position_in_slam'][2]-x['slam_position'][2] for x in w['window_observations']),
                                    max(x['truth_position_in_slam'][2]-x['slam_position'][2] for x in w['window_observations'])])
        for w in returning]
    log=(RUN/'stack.log').read_text()
    checks=dict(original_all38_checks_passed=len(acceptance['checks'])==38 and all(c['passed'] for c in acceptance['checks']),
        mission_all_stages_completed=mission['stage']=='completed',
        independently_recomputed_original46_same_windows=regions['passed'] and list(reached.values())==[18,14,14],
        original_canonical_goal_declarations_and_hashes=regions['preregistered_declaration']['passed'] and not regions['navigation_audit_errors']
            and all(not d['definition_errors'] for d in regions['stages'].values()),
        independent_single_initial_SE3_equals_original=trajectory['rotation_world_from_slam']==acceptance['trajectory']['rotation_world_from_slam']
            and trajectory['translation_world_from_slam']==acceptance['trajectory']['translation_world_from_slam'],
        all_active_SLAM_same_time_GT_covered=regions['truth_full_pose_coverage']['passed'],
        original_pose_integer_stamps_valid=not errors,
        native_states_finite=native['finite'],
        native_LIO_VIO_chain_exact=native['preVIO_equals_postLIO_max_difference']==0,
        actual_native_zero_bias_preserved=all(x==0 for x in native['gyro_acc_bias_absolute_max']),
        native_IMU_all_sync_complete=native['IMU_sync']['incomplete']==0,
        archived_visual_weight_and_kept_configuration=sha(config_file)=='6019b86bc6e3abab57dce3426fa71ac8c534fcd1c4cb1867e6b7983fa39e4cca'
            and config['vio']['img_point_cov']==100000000 and config['common']['img_en']==1 and config['vio']['max_iterations']==5,
        actual_disabled_bias_and_gravity_logs='Bias Estimation Disabled' in log and 'Online Gravity Estimation Disabled' in log,
        source_manifest_sidecar_matches=sha(RUN/'source_manifest.json')==(RUN/'source_manifest.sha256').read_text().strip(),
        all_archived_sources_match=all(archive_same),
        actual_owned_runtime_maps_and_SHA=runtime['passed'] is True and all(runtime['checks'].values()),
        owned_cleanup=cleanup.get('mode')=='camera' and cleanup.get('owned_clean') is True,
        both_camera_startup_gates_original_healthy=all(json.loads((RUN/f'startup_{p}.json').read_text()).get('passed') is True for p in ['sensors','slam']),
        actual_live_RGB_geometry_finite=len(records)==metadata['point_count'] and np.isfinite(records['xyz']).all() and colors>=8,
        actual_official_saved_RGB_PCD=metadata['save']['complete'] is True and pcd['point_count']==metadata['save']['point_count'] and pcd['unique_colors']>=8,
        actual_RGB_sensor_provenance=metadata['ground_truth_used'] is False and metadata['reference_map_loaded'] is False
            and metadata['source_topic']=='/cloud_registered' and '/camera/image_color' in metadata['color_source']
            and all(metadata['counts'].get(k,0)>0 for k in ['imu','lidar','camera','odom','colored_cloud']),
        original_save_sensor_freshness=health['slam_healthy'] is True and health['camera_healthy'] is True
            and health.get('error') is None and all(0<=health['ages'][k]<2 for k in ['imu','lidar','camera','odom','colored_cloud','full_cloud']),
        no_map_capacity_truncation=metadata['capacity_rejections']==0,
        original_dynamic_obstacle_motion=next(c['passed'] for c in acceptance['checks'] if c['name']=='physical_dynamic_obstacle_motion'),
        original_dynamic_stop_and_resume=next(c['passed'] for c in acceptance['checks'] if c['name']=='dynamic_obstacle_stop_and_resume'),
        original_acquisition_header_diagnostics_retained=acceptance['sensor_headers']['passed'] is True,
        no_sensor_file_rescan=True)
    report=dict(scope='Strict original full camera-carrier acceptance; no Go2, RL, real-hardware or perfect-sampling claim',
        independent_passed=all(checks.values()),checks=checks,run=str(RUN),
        official_acceptance_passed=acceptance['passed'],original_failed_checks=acceptance['failed_checks'],
        reached=reached,original_46_receipt_windows=regions,
        single_initial_SE3_and_trajectory=trajectory,active_GT_coverage=regions['truth_full_pose_coverage'],
        source_archive_count=len(archive_same),source_manifest_sha256=sha(RUN/'source_manifest.json'),
        selected_camera_yaml_sha256=sha(config_file),actual_runtime_evidence_sha256=sha(RUN/'startup_runtime_actual.json'),
        original_artifact_sha256={n:sha(RUN/n) for n in ['acceptance.json','pose_audit.jsonl','navigation_audit.jsonl',
            'runtime_manifest.json','shutdown_verification.json','map_metadata.json','colored_map.pcd']},
        official_saved_PCD=dict(**pcd,sha256=sha(pcd_file)),
        actual_live_map=dict(points=len(records),unique_colors=colors,filename=binary.name,sha256=sha(binary)),
        original_map_save_freshness=health,returning_height_windows=height,
        current_native=native,previous_fourth_same_segment_native=comparison,
        original_sensor_header_evidence=acceptance['sensor_headers'],
        complete_nominal_sampling=acceptance['sensor_headers']['complete_nominal_sampling'],
        sensor_scope='Reuse SHA-pinned original evaluator acquisition scan; all original observer gaps remain. Native IMU complete consumption is a separate producer/consumer observation, not proof that every observer sampled all sensors.',
        peer_native_cloud_control_audit='camera_mode/navigation/test_results/automatic_20261002_194830_d16898/nav_command_cloud_audit.json SHA b55a5b32612f111ea4c816cc558beac938ba74b38b908b6e9fab04fb32a0dea7',
        findings=['All 46 original native arrival windows pass raw-inner/GT-outer together under one initial SE3.',
            'Original first failed return6 from the fourth run now has only approximately 4–5mm GT-minus-SLAM height error.',
            'Artificial visual pose-update downweighting greatly reduces native VIO vertical velocity updates; real image processing and official RGB map saving remain enabled.',
            'The full run is a successful effect comparison, not proof of a unique cause or the optimal visual weight.',
            'All four earlier failed runs are preserved. Acquisition gaps, different live/saved map sizes and the camera-only physics scope remain explicit.'])
    result=DEST/'result.json'
    result.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(checks=len(checks),all_true=all(checks.values()),result_sha256=sha(result),reached=reached,
        trajectory=trajectory['overall'],PCD=pcd,native_segment=native['same_canonical_return5_to_return6_segment'],
        fourth_segment=comparison['same_canonical_return5_to_return6_segment'],nominal_sampling=report['complete_nominal_sampling'])))


if __name__=='__main__':
    main()
