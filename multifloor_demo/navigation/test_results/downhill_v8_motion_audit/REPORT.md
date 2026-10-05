# v8 downhill command and movement audit

The original component remains **FAIL, 1/2 regions**. Its first native arrival receipt is 80.9–81.4 s; the safety failure is `imu_tilt_at_least_0.50` at 117.389 s. This report does not replace its result or arrival windows.

Pure turning still produces translation. Supported `ALIGN_ROTATION` covers 8.641 s with zero longitudinal/lateral command; its summed body-forward displacement is −0.08587 m in SLAM and −0.09080 m in GT. These are grouped signed displacements, not one continuous turn. A concrete 19.287–19.787 s segment emits positive yaw integral +0.02189 rad while SLAM/GT retreat by 0.01822/0.02192 m; later turns also have lateral drift. This does not establish a universal response sign or a contact mechanism.

Mixed yaw response can oppose the emitted command within continuous supported windows. For ordinary positive-vx DRIVE at 91.601–92.967 s, yaw command integral is −0.10110 rad while SLAM/GT yaw changes are +0.07426/+0.07304 rad. For compensated ALIGN at 96.486–98.328 s, the corresponding values are −0.14736 versus +0.20717/+0.22085 rad. Five supported mixed segments lasting at least 0.4 s have this opposite-sign observation in both measurements. Command magnitude varies within these segments; they are not constant-input plant identification.

ALIGN compensation is separate from ordinary DRIVE. The supported 40.115 s of `ALIGN_TRANSLATION` has emitted vx integral +0.88177 m but summed SLAM/GT body-forward displacement −0.33633/−0.30458 m. Supported DRIVE translation has 24.287 s, command integral +2.82195 m and SLAM/GT forward displacement +2.58823/+2.58811 m. The grouped numbers contain different episodes and command signs; they do not prove tracking was corrected.

All five guard handoffs are recorded: one identity stop, two fit-quality stops and two displacement stops. Each has native zero emission, zero ACK, completed return, fresh idle with an increased stop counter, at least one simulated second of the required zero window, a new current-goal reference, reconstructed accepted path and at least one second of the original pre-turn gate before motion resumes. The guard-stop/native-zero times are 36.424/36.443, 43.842/43.863, 98.328/98.350, 103.724/103.738 and 111.856/111.876 s. All native emissions between each first zero and its resume are zero.

All 12 nonempty accepted paths reproduce their nearby full SCAN metadata control points/knots to at most 1e−12. Their trajectory IDs increase, original requested centers match the two immutable goal definitions, and native association rejection count is zero. This proves the recorded association and path payload, not an independent occupancy/collision optimization replay. NAV `Path.header.stamp` is the path publication clock; it is not the spline execution `start_time`. An initial audit that incorrectly equated those fields is preserved as `first_attempt_path_header_assumption.json`; the corrected result uses full point reconstruction and nearby receipt timestamps.

The first raw tilt ≥0.30 occurs at 107.995 s. The latest cached NAV status is ALIGN, with `[+0.06846, 0, −0.08]`; it is not an exact transition trace. The final actual nonzero Adapter emission is `[+0.05511, 0, −0.08]` at 116.919 s, followed by the zero edge at 116.936, ACK at 116.941 and return completion at 117.281 s. The ≥0.50 failure is later at 117.389 s while the Adapter reports idle and safe/requested commands are zero. These temporal facts alone do not attribute the failure to the stop or the envelope.

Movement is evaluated over 18.0–117.299999999 s using the original single initial SE3. SLAM/GT maximum interpolation brackets are 0.100000001/0.02 s; no pose brackets fail the original 0.2/0.15 s diagnostic limits. Phase transitions/support conflicts leave 8.538 s explicitly `UNCERTAIN`. The final 89.000001 ms lacks common pose support and is not extrapolated. Adapter emissions use cached floating clock logs and are not direct CHAMP receive, actual applied velocity or joint force. No CDR, joint/contact payload scan, ROS node or new physical run was used.

Reproduce after owned cleanup, using a fresh output filename:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B multifloor_demo/navigation/test_results/downhill_v8_motion_audit/audit_motion.py multifloor_demo/simulation/test_results/20261002_downhill_disabled_envelope_v8 /absolute/path/to/a/new_motion_result.json
```

`motion_handoff_result_v2.json` contains all continuous same-yaw-sign segments and complete handoff timings. `final_receipt.json` records 11/11 audit checks and the immutable original FAIL result SHA; audit completion is not a physical component PASS.
