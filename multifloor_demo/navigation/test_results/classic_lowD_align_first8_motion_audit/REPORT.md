# ALIGN compensation canonical first8: original FAIL retained

The run reaches six original region windows. It fails at the seventh goal when raw IMU tilt reaches .5002074218 rad at 338.414 sim s. The eighth goal is never entered. Seventh-goal timing is 261.800→338.414 s (76.614 s), so this is a safety failure before the unchanged 90 s deadline. Owned process group 3581507 exits0 and all source/runtime/profile hashes are unchanged. The post-terminal tilt of 3.1395 is excluded from the active result.

## Failure onset by actual phase and command

| Event | Raw IMU time | Supported NAV phase | Last native Adapter emission before event | Next native zero log time |
|---|---:|---|---|---:|
| First .20 rad | 323.755 | DRIVE, matching traj39 status bracket | vx+.08970 / yaw−.004254 | 324.691 |
| First .30 rad hold | 326.775 | ALIGN, traj39 with compensation | vx+.06750 / yaw+.08 | 326.776 |
| Second hold | 337.683 | ALIGN, traj42 with compensation | vx+.06542 / yaw+.08 | 337.684 |
| First .50 rad fail | 338.414 | protected ALIGN/zero | exactly zero | already zero |

Log sim times are cached floating clocks, not command Headers or measured DDS latency. These are actual Adapter publications. They are not CHAMP receive traces or proof that body velocity follows the request. The first .20 onset is **ordinary DRIVE** after the previous ALIGN/SETTLE chain, not evidence that ALIGN compensation starts the entire disturbance.

325.728→326.775 s starts at the actual traj39 ALIGN arm. Native integrals are vx+.016005 m and yaw+.076347 rad, while raw SLAM body-forward/yaw are −.152524 m/−.132242 rad; independent same-initial-SE3 GT gives −.151458 m/−.133522 rad. The body retreats and turns against the positive yaw correction in this short interval. Path heading−.0484 and locked heading−.05394 differ by only .0055 rad at 326.481; the positive correction corresponds to real body heading error+.3788. This is not evidence of an old-goal spline or a wrong sign in target geometry.

At 326.4 raw pose, the reported fit RMS is .010161 m, body-vx estimate−.244202 m/s and requested helper vx is capped+.10 m/s. The candidate/production slew stages emit only+.0675 before the hold. The reported signed body corridor is clear under the same traj39 source. No corridor event is logged. The .15 m drift gate has one qualifying observation at 326.6 and two at 326.7; the .30 IMU hold precedes the third raw observation needed for .2 s persistence. Existing protection stops execution rather than bypassing that gate.

336.8→337.683 s has native integrals vx+.017761 m/yaw+.073202 rad but GT body-forward−.073298 m/yaw−.046145 rad. After second hold, 337.7→338.3 s is all-native-zero while GT still moves −.040751 m forward. There is no raw SLAM pose beyond338.3 in the original driver: the last114 ms to the raw IMU failure is not extrapolated.

## Whole seventh-goal supported phase evidence

| Supported phase | Time (sim s) | GT body-forward (m) |
|---|---:|---:|
| ALIGN with compensation | 11.916 | −.263812 |
| ALIGN pure rotation | 4.988 | −.462734 |
| DRIVE translation | 35.733 | +2.745314 |
| PRE_TURN | 10.920 | retained separately in JSON |
| SETTLE | 3.141 | retained separately in JSON |
| Phase uncertain | 9.713 | retained separately in JSON |

The .4 s raw-SLAM fit is a delayed motion estimate, not a full contact-aware plant controller. Caps and quality checks remain active, yet this trial fails physically. The successful first4 trial does not establish uphill safety or exact motion tracking.

## Seven STOP handoffs

Cause counts are **displacement3 / source_identity2 / fit_residual2 / signed_corridor0**. Every actual STOP is followed by original exact-zero publication, incremented stop counter and native ACK/return, fresh idle, new reference, full matching SCAN metadata/published path and the original pre-turn gate. All seven native handoff checks pass; the original timeout and region dwell contracts are never reset or widened. No SCAN collision optimizer is rerun offline.

The first unsafe DRIVE restart also has a measured joint-target jump reported independently by the simulation owner at323.373 s. That execution-layer evidence deserves investigation, but neither its timestamp nor the above correlations alone prove a unique mechanical cause. The NAV sources and original failed result remain unchanged.

## Artifacts and limits

`hold_window.json` contains original first threshold samples, bracketed NAV status, reported fit/corridor/emit context, native command and stop-return file lines, and bounded body projections. `motion.json` verifies seven handoffs; `phase_motion.json` keeps exact supported phase totals and explicitly records missing goal8. `final_receipt.json` hashes these products, source files and original inputs. No CDR was decoded, no ROS node started, and no command or gain changed during this audit. A first unsupported338.4 projection attempt is preserved in `first_hold_window_no_extrapolation_failure.json`; the corrected audit clips to actual common338.3 support rather than filling the tail.
