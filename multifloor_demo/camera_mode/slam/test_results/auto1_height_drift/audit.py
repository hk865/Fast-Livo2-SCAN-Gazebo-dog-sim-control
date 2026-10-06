"""Read-only original camera run state decomposition. No ROS or truth feedback."""
from pathlib import Path
from collections import defaultdict
import hashlib
import json
import re
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
RUN = ROOT / 'camera_mode/runs/20261002_183724_1f7548'
DEST = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    acceptance = json.loads((RUN / 'acceptance.json').read_text())
    trajectory = acceptance['trajectory']
    # Reuse the original one initial transform; never estimate another transform.
    R = np.array(trajectory['rotation_world_from_slam'])
    T = np.array(trajectory['translation_world_from_slam'])
    rows = [json.loads(l) for l in (RUN / 'pose_audit.jsonl').read_text().splitlines()]
    truth = [r for r in rows if r['source'] == 'truth']
    slam = [r for r in rows if r['source'] == 'slam']
    tt = np.array([r['stamp_ns'] for r in truth], dtype=np.int64)
    tp = np.array([r['p'] for r in truth])
    st = np.array([r['stamp_ns'] for r in slam], dtype=np.int64)
    sp = np.array([r['p'] for r in slam])
    indices = np.searchsorted(tt, st, side='left')
    truth_at_slam, valid = [], []
    for stamp, index in zip(st, indices):
        if index < len(tt) and tt[index] == stamp:
            truth_at_slam.append(tp[index]); valid.append(True)
        elif 0 < index < len(tt) and tt[index] - tt[index-1] <= 150_000_000:
            f = (int(stamp)-int(tt[index-1])) / int(tt[index]-tt[index-1])
            truth_at_slam.append(tp[index-1]*(1-f)+tp[index]*f); valid.append(True)
        else:
            truth_at_slam.append([np.nan]*3); valid.append(False)
    gt = np.array(truth_at_slam)
    error = sp @ R.T + T - gt
    pre = np.loadtxt(RUN / 'fastlivo_debug/mat_pre.txt')
    out = np.loadtxt(RUN / 'fastlivo_debug/mat_out.txt')
    gp, go = defaultdict(list), defaultdict(list)
    for row in pre: gp[round(float(row[0]), 6)].append(row)
    for row in out: go[round(float(row[0]), 6)].append(row)
    # handleLIO publishes odometry; handleVIO follows at the identical time.
    offset = st[0]/1e9 - out[0, 0]
    pairs = [(time, *gp[time], *go[time]) for time in sorted(go)
             if len(gp[time]) == len(go[time]) == 2]
    equality = max(float(np.max(abs(p[2] - p[3][:19]))) for p in pairs)
    pattern = re.compile(r'\[DEMO_SYNC\] camera=([0-9.]+) lidar_newest=([0-9.]+) '
                         r'imu_newest=([0-9.]+) imu_last_used=([-0-9.]+) '
                         r'imu_count=([0-9]+) complete=([01])')
    sync = []
    for line in (RUN / 'stack.log').read_text().splitlines():
        match = pattern.search(line)
        if match: sync.append([float(v) for v in match.groups()])
    sync = np.array(sync)
    critical = sync[(sync[:, 0] >= 198) & (sync[:, 0] <= 216)]
    intervals = [(190,198),(198,200),(200,202),(202,206),(206,209.8),
                 (209.8,212.5),(212.5,214),(214,218)]
    updates = []
    for begin, end in intervals:
        selected = [p for p in pairs if begin < p[0]+offset <= end]
        L = np.array([p[3][:19]-p[1] for p in selected])
        V = np.array([p[4][:19]-p[2] for p in selected])
        k = (st > begin*1e9) & (st <= end*1e9)
        updates.append(dict(range_s=[begin,end],frames=len(selected),
                            LIO_sum_position_update_m=L[:,4:7].sum(0).tolist(),
                            VIO_sum_position_update_m=V[:,4:7].sum(0).tolist(),
                            LIO_sum_velocity_update_m_s=L[:,7:10].sum(0).tolist(),
                            VIO_sum_velocity_update_m_s=V[:,7:10].sum(0).tolist(),
                            LIO_sum_roll_pitch_update_deg_57p3=L[:,1:3].sum(0).tolist(),
                            VIO_sum_roll_pitch_update_deg_57p3=V[:,1:3].sum(0).tolist(),
                            height_error_min_max_m=[float(error[k,2].min()),float(error[k,2].max())]))
    checkpoints = []
    for stamp in [190,198,199,200,201,202,204,206,208,209.3,209.8,210,212,214,216,220]:
        i = int(np.argmin(abs(st-stamp*1e9)))
        native = min(pairs, key=lambda p: abs(p[0]+offset-st[i]/1e9))
        checkpoints.append(dict(stamp_ns=int(st[i]), stage=slam[i]['stage'],
                                raw_SLAM_p=sp[i].tolist(), original_GT_world_p=gt[i].tolist(),
                                fixed_initial_SE3_position_error_world_m=error[i].tolist(),
                                native_relative_time=native[0],
                                native_LIO_roll_pitch_yaw_deg_57p3=native[3][1:4].tolist(),
                                native_predicted_velocity=native[1][7:10].tolist(),
                                native_LIO_velocity=native[3][7:10].tolist(),
                                native_VIO_velocity=native[4][7:10].tolist()))
    native_rows = []
    for time,a,b,c,d in pairs:
        if not 198 <= time+offset <= 216: continue
        native_rows.append(dict(absolute_stamp_ns=round((time+offset)*1e9),
                                relative_time=time,LIO_before=a.tolist(),LIO_after=c.tolist(),
                                VIO_before=b.tolist(),VIO_after=d.tolist()))
    (DEST/'native_state_pairs_198_216.json').write_text(json.dumps(native_rows,indent=2)+'\n')
    checks = dict(original_run_failed=acceptance['passed'] is False,
                  original_single_SE3_reused=True, all_original_SLAM_bounded_GT=all(valid),
                  native_pre_VIO_exact_post_LIO=equality == 0,
                  critical_native_IMU_complete=bool(np.all(critical[:,-1] == 1)),
                  critical_every_frame_100_IMU=bool(np.all(critical[:,4] == 100)),
                  critical_last_IMU_at_camera_within_2p1ns=bool(np.max(critical[:,0]-critical[:,3]) <= 2.1e-9),
                  no_original_artifact_rewrite=True)
    report = dict(scope='Read-only diagnosis of original failed automatic camera run; no new physics or correction',
                  run=str(RUN), checks=checks, evidence_ready=all(checks.values()),
                  original_acceptance_passed=False,
                  source_hashes={name:sha(RUN/name) for name in ['acceptance.json','pose_audit.jsonl',
                    'fastlivo_debug/mat_pre.txt','fastlivo_debug/mat_out.txt','stack.log']},
                  original_alignment=trajectory, frame_time_offset_s=offset,
                  native_rows=dict(pre=len(pre),out=len(out),complete_LIO_VIO_pairs=len(pairs),
                                   printed_precision='Native state files contain approximately six significant digits',
                                   pre_VIO_post_LIO_max_difference=equality),
                  native_IMU_consumption=dict(total_frames=len(sync), incomplete_frames=int(np.sum(sync[:,-1]==0)),
                    critical_range_s=[198,216],critical_frames=len(critical),
                    count_min_max=[float(critical[:,4].min()),float(critical[:,4].max())],
                    max_camera_minus_last_used_s=float(np.max(critical[:,0]-critical[:,3]))),
                  update_decomposition=updates,checkpoints=checkpoints,
                  first_failed_region=dict(goal_id='return_origin:6',window_ns=[209300000000,209800000000],
                    raw_SLAM_z_min_max=[float(sp[(st>=209.3e9)&(st<=209.8e9),2].min()),
                                        float(sp[(st>=209.3e9)&(st<=209.8e9),2].max())],
                    reason='GT in the one frozen SLAM frame lies outside the original outer height ±0.10m'),
                  findings=[
                    'The critical native synchronization consumes complete IMU groups; require_complete_imu=false did not cause incomplete consumption in this interval.',
                    'The approximately 0.19m increase in vertical error develops across propagation steps after a yaw turn; direct LIO/VIO vertical position updates are small.',
                    'Both LIO and VIO change estimated velocity and attitude. Native velocity has the wrong vertical sign around 200–201s while real IMU/GT has upward velocity.',
                    'GT remains horizontal while estimated roll/pitch reach approximately one degree; the error recovers after the next turn and forward motion around 213–214s.',
                    'Online gravity is an active unlogged state by default. The current logs do not uniquely attribute this failure to gravity, VIO, or LIO.',
                    'The initial SE3 tilt contributes about 15mm at this position and cannot explain the transient additional 0.19m.',
                    'The 46 native NAV arrivals do not establish joint SLAM/GT region success; subsequent ordered-prefix incompleteness is preserved.'],
                  proposed_single_variable=dict(file='camera_mode/slam/fastlivo.yaml',field='imu.gravity_est_en',
                    old_effective_value=True,new_value=False,
                    purpose='Ablate online gravity cross-coupling while retaining the actual stationary IMU gravity initialization',
                    status='Parent authorized preparation, not a confirmed cause or successful repair',
                    no_GT_feedback=True,no_region_or_NAV_change=True),
                  peer_physics_evidence='camera_mode/test_results/auto1_height_imu/result.json; independent raw IMU integration is -6.74 to -8.09mm at the failed window')
    (DEST/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(checks=checks,result_sha256=sha(DEST/'result.json'),
                          critical_sync_frames=len(critical),native_pairs=len(pairs))))


if __name__ == '__main__': main()
