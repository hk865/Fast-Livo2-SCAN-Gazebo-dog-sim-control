# Classic-PID first-four NAV audit

The run is now terminal and owned clean. It remains FAIL: the unchanged sensor
startup gate timed out after 150 wall seconds with `angular_motion`. Actual IMU
and camera messages continued, but no initial calibration, request, goals,
SLAM body poses, active motion or region receipts were produced. The actual
control-parameter gate was not reached; missing readback is not a passed
physical-profile check. `startup_terminal_audit.json` records this boundary,
the original failure and complete input hashes. No CDR decoding was needed for
this NAV conclusion. The planned active-route audit below is inapplicable to
this run and was not reported as passed.

Physical owner: simulation agent, ROS 76.
Run: `simulation/test_results/20261002_classic_pd_first4_candidate`.
Start: UTC 2026-10-01 20:44:27.203024; owned PG 2858489.
Manifest: 9bbb prefix. Production 269 sources, candidate NAV f592194d and
turn-drift helper 101912f6 remain frozen. No additional heading, body-feedback,
gait or navigation-control change is part of this profile test.

No audit of large streams starts before the physical owner explicitly confirms
terminal and owned cleanup. The plan is:

1. Read `process_cleanup.json`, actual parameter gate and original
   `first_four_result.json` summary. Preserve the original PASS or FAIL.
2. Re-evaluate the four canonical v2 goals using the exact archived
   `first_four_region_contract.py`, original native receipt windows, original
   raw SLAM stamps and one initial SE3. Check inner raw membership, outer GT
   membership, fresh ordered 0.4 s windows and gaps no larger than 0.2 s.
   Missing or invalid original windows stay failed; no later replacement window.
3. Bind each drift trigger to native zero emission and Adapter ACK/return,
   fresh idle and incremented stop counter, one second of zero, new SCAN
   reference and complete metadata. Reconstruct the archived full Bspline and
   match the actually published accepted path before the original heading gate
   resumes. Keep the original per-goal 90 s deadline and control geometry.
4. Segment the native Adapter's emitted commands into walk, pure turn and zero.
   Evaluate SLAM forward displacement and yaw, with GT only as an independent
   fixed-SE3 check using bounded native interpolation. Record command saturation
   duration, pure-turn retreat and any protected/pending interval without
   substituting upstream safe commands for native emission evidence.
5. Record source/input hashes and actual terminal/tilt context. Joint/CDR
   mechanics are reviewed by the simulation owner; this NAV audit will not
   infer applied force from JTC output or independently rerun SCAN collision
   optimization from occupancy.

The native profile contract is independently recorded under
`navigation/test_results/jtc_classic_profile_native`. It is a bare installed-JTC
interface test, not physical stability evidence. The new profile changes
multiple joint-control settings together; any observed physical difference is
a profile result, not a unique causal attribution to a single parameter.
