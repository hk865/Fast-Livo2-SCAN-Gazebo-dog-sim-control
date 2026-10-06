# Excluded pure-turn drift interruption

The single selected candidate is `displacement_015_02`: cumulative horizontal
displacement from the original raw SLAM pose at the start of a pure turn reaches
**0.15 m for 0.2 simulation seconds and at least three distinct raw stamps**.
Original pose gaps must be at most 0.2 s and pose age at most 0.25 s. It requires
the existing `align` phase, a current committed SCAN identity, exactly zero
planned vx/vy and a nonzero angular command. Protected, stale, nonturn, duplicate
and discontinuous observations cannot finish the window. A checked source or
goal change resets the observation; no additional source-change STOP is enabled.

The entry is `nav_drift_controller.py`, with `turn_drift.py` beside it.
`DEMO_TEST_ROOT` points to the frozen `multifloor_demo` root. The launch explicitly
passes `--ros-args -p scenario_path:=<root>/simulation/scenario.json`.
The entry imports the frozen original Navigation and its dependencies using the
explicit root, so it can be copied to a deeper run/staging directory. An archived
import check passed with no ROS initialization or node.

At the event the original exact-zero publisher runs first. The old samples and
trajectory association are discarded, region dwell resets, and nonzero commands
are suppressed. Original control continues every tick, including the original
90 s deadline, IMU/SLAM tilt, sensor watchdog and terminal handling. A new Bridge
status callback must confirm zero requested/safe input and a fresh post-STOP
Adapter IDLE/nominal state with an increased actual stop counter. Old IDLE, a
cached callback, a repeated clock, missing ACK, or RETURNING cannot advance the
handoff. **One second of consecutive fresh actual IDLE evidence** must elapse.
Only then can the original planner request a newer immutable reference for the
unchanged goal. A matched full metadata/B-spline payload and the original
Navigation start/progress checks must succeed before the pending latch clears.
The original preturn/align/settle gate then runs. Absent SCAN responses retry
after the original three wall seconds, with a repeated real idle handshake;
the original goal deadline remains in force.

`Bridge.safe` is the command sent to the Adapter, not a newly observed actuator
publication. During WALK its unchanged protocol forwards that command. Actual
zero dwell uses the Adapter's fresh IDLE invariant: the original Adapter emits
only exact zero while IDLE, and the increased stop counter plus later Adapter
sim time proves this is the new STOP. Native actual Adapter publications in the
physical test must independently corroborate all intervals. The existing
Adapter sim field is a float; its conversion is explicitly diagnosed and never
used as raw SLAM time. Raw SLAM Header stamps remain integers.

The selected-profile and actual-method test entry is:

```
python3 multifloor_demo/navigation/test_results/turn_drift_staging/test_candidate.py
```

It passed 27 checks, including exact threshold/time/sample boundaries, strict
pure rotation, stale/protected samples, old/cached IDLE, actual idle dwell,
same/old reference, full payload mismatch, real production degenerate rejection,
original timeout and unchanged preturn protection. Middleware is replaced by
test boundaries; the production request identity, progress validator and
deadline/control method are executed as actual source methods. This is an
interface result, not a physical PASS.

The offline replay command is `python3 .../replay_015.py`. It uses original native
raw pose, the original full metadata B-splines and unchanged steering filter.
430 A, 100 B and 349 Full18 captured controller geometries were reconstructed
with zero numerical difference. Each original turn is analyzed independently:

| Original run | First event per original qualifying turn (sim s) |
|---|---|
| A disabled | 28.6, 66.0, 94.7, 115.7 |
| B enabled | 26.6, 41.0 |
| Full18 | 28.4, 50.6, 76.0, 117.2 |

Full18 fourth measurement event 117.2 s precedes raw tilt 0.20 at 119.295 by
2.095 s. There is no captured original control sample using exactly 117.2;
the first later captured control using a qualifying pose is 117.439/pose117.4,
which precedes that tilt onset by 1.856 s. These are original-data measurement
and captured-callback opportunities, not a new STOP publication or a simulated
stable outcome. After an interruption the recorded future is counterfactual.

The original 0.25 m/0.4 s AND and displacement-only definitions and reports are
retained. Both give Full18 fourth 119.5 s, after the 0.20 tilt onset. No physical
result, old failure, region, gain, gait, guard, or production source was changed.
The test's body-feedback mode remains disabled; it does not mix the Kd0 trial.
