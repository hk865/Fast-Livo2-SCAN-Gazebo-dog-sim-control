# Excluded native classical-PID profile contract

`actual_20261002_043959/result.json` passes 19 named checks with the actual
installed JTC 4.40.1, controller interface 4.45.2 and control toolbox library.
The sole ROS 77 process exited 0 and its owned group is clean. All 269 frozen
production source hashes remained unchanged. No Gazebo, robot command, GT input,
or production change was used.

The actual GetParameters response for all twelve joints and each PID object's
public getter confirm P 220.982919, I 0.2, D 3.360637, integral limits ±2.5,
velocity feedforward scale 0, `interpolate_from_desired_state=true`, and closed
loop effort PID. Initial command-interface effort is zero through ordinary
activation; the fixture never writes the controller's private state.

Only one twelve-joint batch executes; this is not a repeated P/D fit. The two
integer event grids remain distinct: 441 actual position-only trajectory
messages at 5 ms, with a new RT shared pointer confirmed for every DDS callback,
and 550 actual synchronous `trigger_update` returns at 4 ms. At common event
times the new trajectory callback completes first. Each message has header 0
and the original 16,666,666 ns endpoint horizon. These are controlled simulation
timestamps, not a wall-rate claim.

The stream covers repeated nominal targets, a changed target, a 375 ms quintic
return anchored to the producer's last target, nominal idle and rapid restart.
All twelve measured interface positions/velocities remain the prescribed
snapshots. A nonzero preset velocity with fixed position is not a physically
consistent Go2 trajectory. All stages retain reference effort feedforward 0,
and the actual PID plus feedforward reconstructs the command-interface output
within 4.44e-16. The largest observed output magnitude is 4.0506654856.

This bare JTC test does not instantiate ControllerManager or ResourceManager,
exercise the Go2 URDF total command clamp, confirm actual Adapter return ACKs,
or measure applied torque or physical stability. The separately reviewed CM/RM
contract supplies the total-limit mechanism; subsequent physical profile tests
are still required. The production controller and every historical failed
physical run remain unchanged.

Two preparation/analysis failures are retained:

- `actual_20261002_043942/prelaunch_failure.json`: the first archival executable
  copy lacked execute permission. Failure occurred before starting any ROS
  process; the runner now preserves executable mode with `copy2`.
- `actual_20261002_043959/result_before_mapping_correction.json`: all motion
  contract checks passed, but an extra analysis assertion incorrectly expected
  a hardware-interface DSO. The memory LoanedInterfaces are header-defined;
  actual maps contain the three JTC/interface/toolbox libraries. The corrected
  analysis records their real SHA values and the installed interface header
  SHAs. `analysis_used.py` identifies the successful analyzer; the original
  archived `analyze.py` and initial failed result remain intact. No native test
  data or runtime code changed for this correction.

Reproduction (exclusive ROS 77, no concurrent physical suite):

```bash
source /opt/ros/jazzy/setup.bash
cmake -S multifloor_demo/navigation/test_results/jtc_classic_profile_native \
  -B multifloor_demo/navigation/test_results/jtc_classic_profile_native/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build multifloor_demo/navigation/test_results/jtc_classic_profile_native/build --parallel 2
python3 multifloor_demo/navigation/test_results/jtc_classic_profile_native/run_fixture.py
python3 multifloor_demo/navigation/test_results/jtc_classic_profile_native/analyze.py
```

`prepare.py` creates the excluded source from the SHA-checked previous fixture;
it is a provenance recipe, not a production build dependency. The final
receipt pins the copied source, YAML, executable, loaded libraries and actual
input/output stream hashes.
