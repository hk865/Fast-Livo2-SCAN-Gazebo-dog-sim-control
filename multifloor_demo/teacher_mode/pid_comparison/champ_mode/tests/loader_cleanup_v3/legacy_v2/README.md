# Independent CHAMP comparison executor

This directory prepares CHAMP for the same newly generated Teacher physical world and the same actual SLAM/SCAN/PID outer controller. It has not completed an actual baseline run. Historical camera mode is a legless velocity carrier and is not CHAMP motion evidence.

The only force writer is `gz_ros2_control::GazeboSimROS2ControlPlugin`. Qualified CHAMP gait and IK produce joint references, and the original bounded joint-stop adapter sends them to the effort trajectory controller. No Teacher actor, body servo or body stabilizer is launched. The stop adapter is a declared auxiliary: zero body command returns toward the calibrated native nominal stance; it is not a frozen moving-foot reference.

Original CHAMP PID is retained: P=220.982919, I=.2, D=1, I-clamp=2.5. This differs from Teacher P=25/D=.5. The existing private ControllerManager declares 250 Hz, but the common physical step is .005s; actual update timing must be measured. Physical mass, collisions, joint limits/effort envelope, friction, spawn and all sensor geometry are copied exactly from the common run, changing only the actuation plugins. Original camera/demo/shared simulation/training files are read only and protected by hashes.

Preparation and parent-owned operation:

```bash
cmake -S multifloor_demo/teacher_mode/pid_comparison/champ_mode/native -B multifloor_demo/teacher_mode/pid_comparison/champ_mode/native/build -DCMAKE_BUILD_TYPE=Release
cmake --build multifloor_demo/teacher_mode/pid_comparison/champ_mode/native/build -j1
# First the parent generates the normal Teacher run/world, scenario and spawn.
python3 multifloor_demo/teacher_mode/pid_comparison/champ_mode/prepare.py --run RUN --duration 180
# Then the parent creates the PID scope with controller_kind=champ.
ros2 launch multifloor_demo/teacher_mode/pid_comparison/champ_mode/baseline.launch.py run_dir:=RUN
python3 multifloor_demo/teacher_mode/pid_comparison/champ_mode/command_file_reader.py --run RUN --command-file RUN/navigation_command.json --acceptance RUN/navigation_scope.json --duration 180
```

The parent alone starts Gazebo and actual sensor SLAM. Prepend the directory of `champ_contract.json.controller_manager_library.path` to **only Gazebo's** `LD_LIBRARY_PATH`. Set `CHAMP_ACTUATOR_LOG=RUN/actuator.jsonl` (existing `TEACHER_ACTUATOR_LOG` is accepted). The native observer uses its absolute per-run library path. Do not launch the Teacher worker/plugin, another robot-state publisher, old simulator launch, old body stabilizer or camera carrier in this run.

The command reader writes `worker_ready` after its ROS subscriptions and publishers exist; it does not imply actuator readiness. `/demo/champ/execution_health` is ready only after actual trajectory-controller feedback, original adapter nominal calibration and unique publisher chain checks. Source command timestamps are never refreshed. The exact existing Teacher command-envelope function is extracted without importing the actor: source/mode `scan_slam`, reviewed acceptance hash, monotonic sequence, finite bounded command, wall and simulation ages in [-.05,.3] seconds. Missing/stale/unhealthy commands become zero velocity commands; CHAMP continues native zero stance and its bounded stop transition. Native body state is used only for independent safety/measurement, never outer route tracking.

CHAMP initialization differs: URDF initial positions (0,.9,-1.8 per leg), legacy hold-joints P=100 before spawner readiness, and original adapter .5s contiguous/>=50 raw samples. Actual startup may fail; the flat-short test must establish this. Native observer has no force/pose/reset APIs and records PostUpdate state at offset 0 seconds; Teacher PreUpdate measurements are offset -.005s. `tau` is the commanded `JointForceCmd` component after physics, not a measured motor torque. `tau_available` preserves missing data. Joint targets are separately recorded with original ROS header (normally zero for native CHAMP) and actual receive-clock stamp in `champ/joint_target_events.jsonl`; no target is invented inside native physics rows.

For lifecycle safety the reader consumes the same outer route until D seconds, then holds a zero body command for eight additional seconds and writes `champ/monitor_done.json`. The passive observer requests stop on the next physics step; its D+10 watchdog fails closed if the monitor is absent. This extra tail is a declared lifecycle difference and is excluded from comparative route/fixed arrival-parking metrics. Safety failure writes failure evidence, zeros the body command, stops the own world and exits 2. It does not claim Teacher fault-damping equivalence.

Logs: `actuator.jsonl` (native pose/COM velocity/q/qd/commanded force/real contacts), `telemetry.jsonl` (50Hz command envelopes and source ages), `joint_stop_adapter.jsonl`, `champ/joint_target_events.jsonl`, `champ/execution_health_history.jsonl`, `champ/spawner_exit.json`, `policy_metadata.json`, and the per-run `champ_contract.json`. Actual selected process/DSO maps and all owned-child cleanup still need the parent's runtime audit. Offline checks are in `tests/offline_report.json`; they are interface/preparation evidence only.
