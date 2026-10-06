# CM force-limit A/B navigation audit

Both original physical results remain failed: A (CM flag false) reached six of
eight regions and timed out at index 6; B (flag true) reached five and timed out
at index 5. Neither entered index 7. A active maximum raw tilt is 0.296512662;
B is 0.251978372. Neither had an active tilt hold, body contact, navigation
obstacle stop, degenerate trajectory or association rejection.

`light_audit.json` and `b_light_audit.json` retain the exact original native
arrival windows. Their hash, fresh ordered raw observations, inner membership,
0.4 s dwell and maximum 0.2 s gap were verified. Reported fixed-SE3 GT coordinates
were checked against the original outer geometry; original truth-source pairing
is independently reviewed by the SLAM owner. The first light audit read only a
bounded result prefix, status suffix and small guard/trajectory records while B
was running. It did not decode CDR or native command/Joint data.

After both groups were confirmed terminal and owned clean, `audit_runtime.py`
checked all 16 A and 17 B interruptions through native exact-zero emission,
ACK and return completion, fresh idle, new reference/full metadata, original
published path reconstruction and subsequent movement after the original
one-second preturn interval. All passed; no goals or deadlines were modified.
This revalidates identity and the full published spline, not collision solving
again from occupancy.

The failed-goal interval runs from the preceding original receipt to the
actual NAV terminal and includes goal-transition scheduling:

| Quantity | A, failed index 6 | B, failed index 5 |
|---|---:|---:|
| Walk time | 58.059 s | 31.117 s |
| Turn time | 14.249 s | 33.646 s |
| Zero-command time | 17.968 s | 25.476 s |
| SLAM local forward during walk | +3.069 m | +2.384 m |
| SLAM local forward during turn | -0.993 m | -1.530 m |
| SLAM local forward during zero | +0.007 m | +0.046 m |
| Closest raw distance to failed center | 1.244 m | 1.059 m |

B walk commands emitted a yaw integral of -0.280923 rad, while SLAM observed
+0.945148 rad and fixed-SE3 GT evaluation observed +0.954716 rad. Actual motion
still differs from requested motion; these totals alone do not prove a joint
or clamp cause. A and B failed at different goals and cannot be treated as the
same-segment causal comparison.

`a_runtime_audit.json` and `b_runtime_audit.json` preserve the complete command,
result, trajectory, guard, configuration and source SHAs. Command times are the
native Adapter's cached float sim clock, not a Twist header or actual CHAMP
receive timestamp. Publisher evidence is not measured applied force. Position
and orientation boundaries use bounded native brackets and Slerp without
extrapolation; GT is evaluation only, with the original single initial SE3.
JTC/contact mechanisms are outside this NAV audit and reviewed separately.
