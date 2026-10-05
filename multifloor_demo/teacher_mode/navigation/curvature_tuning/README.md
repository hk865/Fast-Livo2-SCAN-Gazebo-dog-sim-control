# Teacher continuous-turn plant sweep

This new experiment measures continuous moving turns of the frozen Teacher on
a generated, unobstructed 60 × 60 m plane. It does not certify PID circle/S
tracking, SLAM navigation, the original multifloor map, slopes, or real hardware.
No training parameters or robot inertial/joint/actuator data are changed. The
existing isolated asset preparation is reused; only the world's static terrain
and spawn pose are replaced in each new run directory.

The prospective grid is forward body speeds 0.3, 0.5, 1.0 m/s and signed body
yaw rates ±0.2, ±0.4, ±0.6, ±0.8 rad/s. The first sweep contains 24 separate
tests. Boundary repeatability requires three fixed repetitions of the selected
boundary cases after the root operator reads the first sweep; one passed test
does not establish a global minimum turning radius.

Each 33 s test requests Teacher zero velocity for 0–3 s, linearly ramps forward
and yaw requests for 3–6 s, holds them for 6–26 s, and requests zero for 26–33 s.
The unchanged policy applies its original [0.6, 0.6, 0.8] command slew per second
at 50 simulated Hz. Teacher inference continues during parking. Joint targets
are neither captured nor replaced by zero action. The CPU Actor, observation
builder, bootstrap, native DCMotor speed/torque curve and exclusive 200 Hz
actuator are archived and hashed. Privileged Gazebo observations are explicit.

The fixed parking window is 28–33 s; it is never selected from a favorable
portion of the log. All 200 Hz native torque, joint speed, roll/pitch, clearance,
body contact and source coverage are checked. Native PreUpdate state is tagged
with the original world clock minus dt. Base-origin velocity is independently
reconstructed from COM body velocity minus angular velocity cross the recorded
COM offset, then checked against the separately recorded native origin velocity.

Cruise must have forward and yaw mean absolute errors within 25% of request.
Every complete 1 s rolling forward window must achieve at least 70% of the
requested forward speed. Instantaneous gait velocity is still saved; a zero or
in-place turn cannot pass the forward gate. A least-squares circle is fitted to
the complete 20 s base-origin XY arc: at least 60° observed arc, radial RMSE
≤0.05 m, and CV ≤0.2 across four fixed five-second subarc radius fits. Each
subarc must cover at least 0.25 rad. These are prospective gates, not filters
used to choose a favorable arc. Heading slip is an additional diagnostic.

The command radius v/|w| is a reference. The geometric fitted radius and the
mean measured speed/rate ratio are separate outputs. Only the smallest passed
tested fitted radius for a given speed and sign can be reported; failure or an
untested lower rate/radius remains visible. No instantaneous v/w sample can be
used as evidence of minimum radius.

Prepare plans without launching ROS or simulation:

```bash
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/curvature_tuning/pure_tests.py
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/curvature_tuning/prepare_plan.py --all
```

The root operator first reviews the plan and prepares an immutable run:

```bash
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/curvature_tuning/run.py --plan /ABSOLUTE/PLAN.json --prepare-only
```

Only the root operator launches an actual run by invoking the same runner
without `--prepare-only`. This creates another fresh run directory; a prepared
or historical run is not overwritten. `--camera` enables a real hardware/Ogre2
observer camera at 2 Hz and 80° FOV. It adds the direct ROS image bridge and
recorder, no SLAM or navigation nodes. The default bare run owns just its CPU
worker and Gazebo process. All actual owned processes must exit zero for a pass.
Resources, environment, exact native binary, model, policy, world, protocol,
commands, inference arrays and run sources are archived. No unrelated process
is stopped or signaled.

Outputs are `summary_radius_independent.json`, `radius_evidence_arrays.npz`,
`plant_commands.jsonl`, `telemetry.jsonl`, `actuator.jsonl`, and the ordinary
worker/ownership/runtime/source/fixture receipts. Missing source or physical
evidence cannot pass. The independent evaluator writes only new plant results;
it does not run or modify any controller.

Future continuous curves need a separately frozen controller: curvature
feed-forward `w_ref = kappa * v`, continuous spatial arc-length references,
and measured feasible speed/curvature limits. Existing polygon waypoint
stop/turn behavior, or successful in-place yaw, is not continuous-curve tracking.
