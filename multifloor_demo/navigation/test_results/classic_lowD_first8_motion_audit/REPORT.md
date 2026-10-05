# Low-D first8: original FAIL retained

The actual run `20261002_classic_lowD_first8_candidate` reached seven original
regions. Its eighth goal exhausted the original 90 simulation seconds without
an arrival receipt. The physical processes exited and were cleaned up before
this offline analysis. No source, archived result or original receipt changed.

| Eighth goal, native emitted-command classification | Simulation seconds | Independent GT body-forward displacement |
| --- | ---: | ---: |
| Forward walk | 57.673 | +3.7995 m |
| Pure turn | 12.583 | −1.0302 m |
| Exact zero | 19.878 | −0.0134 m |

The bounds are the original seventh receipt at 355.800 through the actual
terminal at 445.945. A boundary-crossing 11 ms native command interval is
disclosed and excluded from this goal grouping. The emitted forward command
integrates to 6.6104 m, averaging 0.11462 m/s; observed GT forward speed averages
0.06588 m/s. Forward emission is at least 0.119 m/s for 41.043 seconds, 71.17%
of walking time. Increasing a saturated forward gain is unsupported.

There are eight native pure-turn intervals in goal8 and three drift
interruptions; the whole route has nineteen. All nineteen original handoffs
pass the recorded exact-zero, post-stop ACK/return/idle, new-reference/full
metadata, reconstructed accepted path and original ≥1 s preturn checks. No
deadline, region or dwell resets into a longer allowance.

Goal8 has twelve complete SCAN metadata records, trajectory IDs 44–55, all for
the original goal. After the three STOP handoffs, IDs 45/46/50 use the frozen
planning-reference boundary and initial 0.8 m tangent agrees with the goal
bearing. Automatic replans 47 and 49 have initial tangents respectively 0.2228
and 0.3134 rad away from their instantaneous goal bearings. These are current
goal curves with measured filtered start velocity, not stale goal identities.
Actual turn response also varies: 360.462–361.936 emits positive yaw integral
0.1511 rad but measured GT yaw changes −0.0121 rad; 369.644–371.142 emits +0.1539
but GT changes −0.0018. Other turn intervals mostly follow the emitted sign.
This descriptive observation does not identify a constant plant gain.

The closest original raw sample is 441.3 s: along-slope goal error −0.48163 m.
At the last original raw sample, 445.9 s, it is −0.57903 m, lateral +0.06453 and
normal −0.01117 m. The required inner half-extents remain [0.25, 0.20, 0.07].
This failure is materially short of the along-slope region, rather than a
millimetre boundary disagreement or a large SLAM error.

The driver poses have a genuine 442.8→443.1 s observation gap. No interpolation
crosses that 0.3 s bracket. SLAM movement omits thirteen native command
intervals totalling 0.326 s; their exact times appear in the JSON. GT has full
bounded support and both original streams cover the terminal without
extrapolation. Independent CDR has the omitted pose messages, but those are
not substituted into this driver audit or its original FAIL.

For a candidate estimator, five original raw samples spanning 0.4 s are fitted
in world coordinates and projected through the last matched body rotation.
Among 87 valid goal8 turn windows, body-forward velocity has median −0.09470
m/s and 5th percentile −0.17985. Position residual P95 is 0.01040 m. Independent
GT validation has velocity error absolute P95 0.00588 m/s. This demonstrates
usable measured translation; it does not make gait oscillation sensor noise.
The fit has approximately 0.2 simulation seconds of effective lag. Original
status pose age P95 is 0.097 s, maximum 0.127 s; it is a cached watchdog measure,
not an independent callback latency measurement.

`lowD_first8_motion.json` gives full segment/handoff provenance;
`goal8_response.json` preserves actual steering snapshots, all twelve metadata
diagnostics and velocity quality counts. Native Adapter emissions are velocity
requests, not observations of CHAMP reception or applied force. GT is used
only for independent evaluation under the original single initial SE3.
