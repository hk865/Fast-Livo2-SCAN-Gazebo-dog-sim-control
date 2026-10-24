# V32 bounded map-surface A/B validation

This isolated deployment continues the V28 actual-SLAM/SCAN original46 mission. The frozen CPU Teacher, native 50/200 Hz actuator, source freshness protections, control gains, path admission, route fence, region geometry and arrivals remain unchanged. No hardware robot or RL training is operated.

## Purpose and limits

Compare the same new V32 SLAM binary with `--surface-validity 0` (original plane acceptance) and `--surface-validity 1` (additional support-consistency validity). This is an experimental engineering gate, not a calibrated statistical test or a guaranteed height repair. The original minimum-eigenvalue gate and octree subdivision remain. A plane is additionally accepted only for N>=6, middle/maximum eigenvalue ratio>=0.05, and normalized projected residual energy/(N-3)<=4 with variance floor 1e-8 m². No floor labels, Gazebo truth, region number or height reset enter this gate.

The separate frozen V19 implementation and protected camera mode remain untouched. Binary/source identity is checked before physics against the actual executable and `/proc` mapped library. An independent source-bound preflight authorizes this bounded experiment only. It does not confer interface, motion, Sim2Sim, height or navigation success.

## Sources and evidence

Navigation uses actual FAST-LIVO2 body odometry, causal IMU gyro, observed registered clouds and accepted SCAN paths. Actor observations retain 232 privileged simulator dimensions, explicitly recorded. Native world pose is used only in offline evaluation. The prior d48 loop backend is not connected in this frontend A/B, so this experiment cannot claim backend benefit.

State, covariance, IMU, stage and timing records remain on. Dense H/R/matching records cover source seconds 209–215. Exact map-fit support point XYZ, full 3x3 covariance and source identity cover 0–15 and 209–215 within SLAM voxel-centre bounds x14–18, y-1–3, z0–1.5. These bounds only select evidence and never alter estimation. Support capture has a 64 MiB cap with explicit omission counters. PointCloud2 CDR bytes and IMU are separately captured in source windows [3,4.5], [10,12], [208.5,215], capped at 1 GiB. The adapted-input topic is before FAST-LIVO preprocessing; it is not labelled as preprocessed cloud.

Synchronous 50 Hz safety telemetry and command validation/acknowledgment remain. Duplicate complete command envelopes are moved into a bounded asynchronous evidence file, with sequence and raw SHA retained in each telemetry row. Height-source detail is windowed. Any overflow, missing support, incomplete window or writer failure remains an explicit evidence failure; it is not navigation success.

## Prospective evaluation

The 270-second profile retains all original46 regions but is a prefix, not a complete46 test. Keep every failure and termination reason. Compare fixed initial SE(3)-aligned SLAM height to native truth, first height-error event and stage budgets: propagation, LIO, VIO, map support/association, z-vz covariance, residuals and bias. Report peak height error and timing independently; do not change offline HEIGHT acceptance to improve the score.

Check physical contact/posture, source freshness and Teacher IPC faults first. A physical damping/drop failure or command starvation cannot prove or disprove the map repair. Only comparable trajectories reaching the fault region provide causal A/B evidence. Functional region progress alone cannot prove position/height accuracy. Candidate plane rejection alone cannot prove improved localization.

## Commands

```bash
cd /opt/experiment/workspace/multifloor_demo/teacher_mode
/usr/bin/python3 -B navigation/map_surface_v32/run.py --profile repair_prefix270 --surface-validity 0 --label V32_surface_baseline_r1 --domain 151
/usr/bin/python3 -B navigation/map_surface_v32/run.py --profile repair_prefix270 --surface-validity 1 --label V32_surface_candidate_r1 --domain 152
```

Use a fresh private domain and run identity for each experiment. The runner owns and cleans only its process groups, saves resource/storage receipts, and terminates at the finite mission/fault cutoff. The full original46 variant is prepared separately after prefix review; a cutoff is never success. Live browser: `http://127.0.0.1:8768/?run=<fresh_run_name>`.

## Region height and feasible-direction proposal

Terrain-aware arrival tolerance and map calibration are different controls. The original regional arrival and route thresholds stay frozen in this A/B. A subsequent navigation-only experiment may unify capture geometry with the existing ramp-normal arrival envelope; any wider tolerance needs explicit terrain/support uncertainty and must not hide wrong-floor ambiguity. Strong global correction must come from independently credible submap registration. Registered cloud and frontend pose share errors. A recent reachable direction may be chosen only within the same route-progress/layer window and fresh obstacle, swept-body, braking and support evidence. Unknown or stale space is rejected.
