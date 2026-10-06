# Native repeated-reference response

This is an excluded diagnostic of the installed JTC 4.40.1 effort interface. It
does not run Gazebo, publish robot motion, use truth, change production gains,
or adopt `interpolate_from_desired_state=true`.

The native run `actual_20261002_034521` has six owned processes, all exit 0 and
clean. All 52 named checks pass; the 269 collected source hashes are unchanged.
Source `0ee625ee…`, executable `6bdd073f…`, result `95a1a976…` identify the exact
observed run. The preliminary archived `analyze.py` had a syntax error; it was
not used. `analysis_used.py` is the exact successful analyzer identified in the
result. Original native records and execution receipt remain unchanged.

The two integer event grids are separate: a new position-only header-zero
reference every 5,000,000 ns and installed synchronous JTC `trigger_update`
every 4,000,000 ns. At a tie, the real DDS subscription must receive a different
RT shared pointer before the update. Every batch observes 241 new callbacks and
300 successful returned 4 ms periods over 1.2 s. These are controlled sim-time
events; the fixture does not assert a real-time 200 Hz wall schedule.

There are 35 independent interface snapshots: position error ±.005, ±.01,
±.02 and zero, crossed with measured velocity ±.1, ±.2 and zero. They are split
across three 12-joint batches. Each batch has normal fresh activation, measured
positions/velocities held at the declared values, and initial command effort
zero. A constant position with independently prescribed nonzero velocity is a
controlled interface snapshot, not a physically consistent robot trajectory.

Actual PID error terms, weighted integral, PID output, interpolated reference
effort, and total command output are logged. The analyzer verifies
`output = reference_effort + PID`, then separates the observed linear carry of
previous output's integral contribution. It fits the last 200 updates per case
to `command = P_effective * position_error - D_effective * measured_velocity + bias`.
All five update phases in the 20 ms common cycle are retained.

| Setting | Effective P, integral carry removed | Effective D |
|---|---:|---:|
| Original false | 220.982919 | 3.360637 |
| True, original configured P100/D1 | 100.000000 | 1.000000 |

Original false varies by phase: P 203.35–234.44 and D 2.87–3.73. These differ
from a provisional source-only estimate of P 250–350/D 4.17. They describe this
reference cadence/horizon and prescribed snapshots, not a general robot plant
or a reason to select new gains. Mean linear-fit residual is below 4e−14. In
the true batches, actual reference effort is zero throughout activation and
updates; that fact is limited to the zero-initial-command, fixed position-only
inputs here. It does not establish zero FF for every physical stop/restart.

This fixture does not invoke ResourceManager command limits. Its maximum
original-false total command is 5.4365, below the actual Go2 joint limits. The
separate CM/RM native contract verifies the clamp of total PID plus FF before
hardware write. Root's current candidate keeps false/P100/I.2/D1 and enables
only that CM command limit; this diagnostic introduces no second variable.

Reproduce under the ROS environment, with domain 77 reserved:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S multifloor_demo/navigation/test_results/jtc_effective_response_native \
  -B multifloor_demo/navigation/test_results/jtc_effective_response_native/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build multifloor_demo/navigation/test_results/jtc_effective_response_native/build -j2
python3 multifloor_demo/navigation/test_results/jtc_effective_response_native/run_fixture.py
python3 multifloor_demo/navigation/test_results/jtc_effective_response_native/analyze.py
```
