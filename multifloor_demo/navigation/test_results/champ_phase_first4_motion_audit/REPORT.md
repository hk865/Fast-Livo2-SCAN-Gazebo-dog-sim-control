# Private CHAMP phase first4: NAV motion and handoff

The original component result remains **20/20 true, four original region windows passed**, with raw tilt peak .1796998 rad, zero tilt holds and zero body contacts. This audit verifies its existing NAV/command/movement evidence. It does not establish successful slope, eight-goal, obstacle or full-demo operation, or accurate actuation tracking.

The original four native arrival windows are52.1–52.5,81.4–81.9,111.2–111.7 and131.8–132.3 s. Their original400/500/500/500 ms dwells are retained. Origin is24 s; receipt-to-receipt durations are28.5,29.4,29.8 and20.6 s, all within the unchanged declared90 s bounds. No window, region, safety threshold or deadline is moved. Successful examples do not newly exercise the timeout failure branch.

## Actual response remains different from the command

Phase support uses consecutive same-source ALIGN arm/observe callbacks and matching bounded adjacent statuses for other phases. Nonzero ALIGN compensation is kept distinct from ordinary DRIVE. Command intervals use the exact Adapter emission edges; SLAM and GT motion use the original single initial SE3 and bounded pose brackets. GT never enters control.

| Supported phase and emitted mode | Duration | Emitted vx integral | SLAM body-forward displacement | GT body-forward displacement |
|---|---:|---:|---:|---:|
| ALIGN with compensation |44.887 s|+1.44220 m|−.10623 m|−.08473 m|
| ALIGN pure rotation |8.816 s|0|−.15019 m|−.14590 m|
| Ordinary DRIVE with translation |35.263 s|+4.00714 m|+3.14621 m|+3.14301 m|

Ordinary DRIVE's measured GT forward speed averages .08913 m/s across those supported intervals. During mixed ALIGN, emitted yaw integral is+1.40936 rad, while SLAM/GT yaw increments are+2.70749/+2.76183 rad. These are grouped integrals across varying commands and physical histories, not an identified fixed plant gain. In particular, positive net longitudinal compensation input still corresponds to a slightly negative net body-forward movement in that ALIGN group.

Native emissions respect the original .12 m/s overall vx cap, zero vy and .08 rad/s mixed-motion yaw cap; pure rotation also stays within .12 rad/s. All411 recorded live ALIGN configuration observations agree with the archived configuration. The three NAV source files are byte-identical to the previous ALIGN first4; gains, geometry/arrival and protection logic are unchanged.

## The one STOP is an identity event

At64.121 s the guard stops for `foreign_identity`: the incoming trajectory is6 and the helper retains5, under the same goal/reference. Counts are **source-identity STOP1, displacement STOP0, corridor STOP0**. This must not be presented as successful .15 m drift intervention.

The original handshake evidence is:

- Native Adapter zero64.140 s; native zero ACK64.144 s.
- Return completes to idle64.489 s, followed by the original fresh zero/idle dwell.
- New reference65.568 s; matching checked path accepted65.572 s.
- First native nonzero command66.636 s, after the original ≥1 s pre-turn delay.
- All224 native emissions before resume are exact zero. Full metadata reconstructs the published accepted path with zero numeric error and matches the original goal.

This verifies source association and the emitted checked path; it does not rerun SCAN's collision optimization from occupancy. Adapter emission is publisher evidence, not the exact CHAMP callback time or applied joint force.

## Comparison and evidence boundaries

The previous unchanged-ALIGN first4 had supported ALIGN duration57.320 s and GT forward drift−.46946 m; this phase run has53.724 s and−.23063 m. Previous/current raw tilt peaks are .14969/.17970 rad. DRIVE averages are .09254/.08913 m/s. Both are different fresh physical starts with different trajectory histories: these descriptive differences do not show a general causal stability or command-response improvement.

Unsupported/conflicting phase boundaries total **7.316 s**, retained as uncertain. Original terminal is132.408 s, but common native SLAM/GT pose support ends at132.3 s; the final **108 ms** motion tail is not extrapolated. The original terminal and acceptance windows remain unchanged. Peer SLAM review separately preserves one active CDR sample loss at45.704 s; this audit does not replace missing samples or repeat the930029-record CDR decode. Peer simulation owns the targeted raw/JTC restart sample extraction.

Files in this directory:

- `motion.json`: exact emitted-command mode integrals, original receipts and complete STOP handshake.
- `phase_motion.json`: same-source supported ALIGN/DRIVE analysis and descriptive previous-ALIGN comparison. Its historical key `original_lowD4` refers to **20261002_classic_lowD_align_first4_candidate**, not a non-compensated NAV baseline.
- `final_receipt.json`:15 audit checks, source/configuration/input SHA receipts and explicit limits. SHA `04ca9460ee7434b82854e8698277179659800bc84972e41cb4f4641055a64997`.

The existing audit scripts were copied unchanged. They read owned-clean JSONL/JSON evidence only. No ROS, physical process, candidate source modification, new gain or production adoption occurred.
