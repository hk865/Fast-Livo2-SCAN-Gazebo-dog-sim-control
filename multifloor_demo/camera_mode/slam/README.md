# Camera-carrier sensor SLAM

This mode uses the original isolated FAST-LIVO2 binary, its calibrated IMU,
LiDAR and RGB extrinsics, the original odometry frame adapter and the RGB map
archive. `source_contract.json` pins those sources and both selected binaries.
The carrier origin is its center, initially 0.75 m above F1. The existing 46
relative route centers therefore place the center 0.75 m above each floor.
`demo_slam_body` identifies this center in camera mode; it is not a Go2 pose.

The LiDAR relay changes delivery QoS, keeping the original message, acquisition
stamp, fields and point payload. It removes zero points and never loads the Go2
body/leg envelope. IMU and RGB bridge publishers must offer reliable delivery
because the preserved FAST-LIVO2 subscriptions request it. The observation
nodes accept sensor-data delivery and do not feed truth into estimation.

The parent launch owns this sequence:

1. Start the real carrier sensors, camera safety bridge and passive recorder.
2. Run `health_gate.py --phase sensors --scenario SCENARIO --run-dir RUN`.
3. Include `launch.py` with `scenario:=SCENARIO` and `run_dir:=RUN`.
4. Run `health_gate.py --phase slam --scenario SCENARIO --run-dir RUN`.

Both gates retain the original 10-second warmup and three-second stationary
IMU/gravity thresholds. The first success publishes `sensors_ready`; only the
second success publishes `mode=camera, phase=slam, state=ready` on the reliable,
transient-local `/camera_demo/startup` topic. The bridge then permits motion.
There are no CM, joint, gait, body-feedback or foot prerequisites.

The parent records `mission.json`, `scenario.json`, `pose_audit.jsonl`,
`navigation_audit.jsonl`, `sensor_audit.jsonl`, the original map metadata and
immutable binary revisions. Pose observations retain original `stamp_ns` and
stage labels. Navigation preserves the original schema-2 receipt definitions,
hashes, indices and measured windows, with `mode=camera`.

Run `evaluate_run.py RUN` after owned-process cleanup to evaluate a full route.
It reuses the original region and RGB/PCD checks while reading this mode's
`source_manifest.json` (`files` entries with relative path and SHA), its plain
SHA sidecar and `sources/camera_mode/simulation/scenario.json`. It also requires
the two successful startup reports, `camera_slam_contract.json`, the camera
`runtime_manifest.json` artifact list and `shutdown_verification.json` with
`mode=camera, owned_clean=true`.

Arrival evidence uses the exact original NAV window, raw SLAM inside the inner
band and same-time truth inside the outer region, under one initialization
SE(3). Missing active truth fails; neither later windows nor extrapolation
repair it. Sensor observer gaps are reported separately without filling them.
Map acceptance requires this run's measured RGB cloud and an official saved
RGB PCD with fresh sensor evidence. Manual-only observations cannot constitute
completion of the 18+14+14 route.

Offline tests are in `tests/test_camera_slam.py`. The recorder's actual installed
ROS message-construction regression is `tests/test_record_sensors_messages.py`;
it creates no ROS context or node. Receipts live under `tests/test_results`.
These tests certify source/interface preparation, not real SLAM accuracy or
physical carrier motion. This mode provides no quadruped locomotion or RL
policy evidence.
