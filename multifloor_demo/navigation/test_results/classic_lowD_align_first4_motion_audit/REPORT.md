# ALIGN longitudinal SLAM compensation: actual first4 audit

The original component passes all 20 checks and its four original region windows. The source-identical candidate was cleaned before this audit. The three frozen NAV source hashes and all raw input hashes are in `final_receipt.json`. This is first4 evidence, not full-route success or a claim that general command tracking is repaired.

## Observed motion by phase

ALIGN is supported by successive original guard arm/observe callbacks under the same reference, goal and trajectory, with raw gaps at most .2 s and callback gaps at most .25 s. DRIVE/PRE/SETTLE use matching adjacent actual statuses within .35 s. Uncertain boundary intervals are retained separately. Actual Adapter velocity emissions split these windows; `vx != 0` alone never identifies ordinary DRIVE.

| Supported group | Time (sim s) | Raw SLAM body-forward (m) | Independent GT body-forward (m) |
|---|---:|---:|---:|
| ALIGN with longitudinal compensation | 50.538 | -.123127 | -.101568 |
| ALIGN without translation | 6.677 | -.365210 | -.367897 |
| Ordinary DRIVE translation | 34.666 | +3.198777 | +3.208136 |

The supported compensated ALIGN intervals emit 1.433922 m of integrated longitudinal command and 1.775520 rad of yaw command. Measured body-forward remains slightly negative: the command integral is not displacement. Raw SLAM and independent GT agree on that distinction. Supported ALIGN emitted vx spans -.021184 to +.100000 m/s, vy is zero and mixed yaw is at most .08 rad/s. The actual Kp=.8/Kd=.4, .4 s fit, .10 m/s cap and quality limits were read back from the node.

## Comparison with the previous lowD first4

These are separate fresh starts, not a paired deterministic plant experiment. Supported ALIGN total GT body-forward is -.469465 m versus -1.669058 m in the previous trial. ALIGN takes longer, 57.320 versus 31.507 sim s; PRE_TURN decreases to 5.664 from 20.881 s. Ordinary DRIVE forward speed is .092544 versus .093437 m/s, so this does not show improved ordinary DRIVE speed tracking. Guard stops decrease from eight to one and raw tilt peak is .149693 versus .282319 rad on these two trials, without a hold in either trial.

## The only STOP is a source-identity protection

At raw stamp 93.200 s the active context changes from traj9 to traj10 under the same goal/reference. The helper rejects the foreign identity. Exact zero is published at callback time 93.223 s. Counts are **source_identity_stop=1, displacement_stop=0, signed_corridor_stop=0**. It is not evidence that the .15 m displacement gate intervened or prevented instability in this trial.

Actual native zero begins at 93.242 s, ACK at 93.244 s, return completes at 93.549 s. Fresh bridge/idle evidence starts the zero window at 93.601 s; the stop counter increases 7→8. New reference is 94.642 s, fully matched checked traj11 is accepted at 94.646 s, and first native motion follows at 95.714 s after the original pre-turn interval. All 222 native emissions from the first actual zero to resume are exactly zero. The accepted published path reconstructs from full metadata with zero error. Original goal deadlines, region definitions and raw dwell windows are untouched. This does not independently rerun the SCAN collision optimizer.

## Evidence boundaries

The candidate has 7.301 s of uncertain phase support and the baseline 12.413 s. The candidate terminal suffix of 19 ms and baseline suffix of 20 ms are outside common pose support and are not extrapolated. Native Adapter publish evidence is not a CHAMP receive trace or measured applied torque. Thirty milliseconds of PRE translation and 14/15 ms of SETTLE rotation/translation lie at sampled status-to-Adapter command boundaries; these labels alone do not prove nonzero NAV publication during a zero phase.

The independent original-window review is SHA `343299398e51586fe8d2e69f19a2a6b44d984340b65169451374e801bf49bfc2`. It retains probe IMU losses at 38.003/56.403, native losses at 83.205/.206, and the CDR IMU/three contact sample losses at 88.411 as original missing diagnostics. No missing raw data is backfilled and no previous FAIL is changed.

## Reproduce the read-only calculation

Run `audit_motion.py <run> <fresh-output.json>` and `audit_phase.py <run> <previous-lowD4-run> <fresh-output.json>`. Both consume existing original files, use one original initial SE3 for independent GT evaluation and bounded interpolation (.2 s raw SLAM/.15 s GT), and never start ROS or send commands. Output paths must be new. `phase_motion.json` and `motion.json` remain the original audit products; the finalized cause classification is in `final_receipt.json`.
