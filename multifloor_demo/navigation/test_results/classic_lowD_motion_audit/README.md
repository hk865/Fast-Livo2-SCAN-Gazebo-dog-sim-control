# Low-D first4 terminal audit preparation

No ROS, physics, controller execution or CDR decoder. Prepared from the existing reviewed native-command/SLAM/GT handoff audit. Future data are not read until process_cleanup says the owned process group is clean. The original first_four result and all native receipt windows are retained, regardless of PASS/FAIL.

Run only after cleanup: `python3 multifloor_demo/navigation/test_results/classic_lowD_motion_audit/audit_motion.py /absolute/run /absolute/new_audit.json`. Existing output is refused.

The audit groups native Adapter-emitted commands into walk/turn/zero and computes command integrals, bounded raw-SLAM and independent original-initial-SE3 GT body-forward/net-yaw response. It records saturation duration, sign-group walk yaw and each continuous mode. Original-region evaluator data are attached without replacing receipt windows. Drift-stop handoffs retain exact native zero/ACK/return/idle/new reference/full metadata and reconstructed accepted path checks. Per-goal boundaries use prior original receipt times and explicitly disclose tiny command intervals straddling those boundaries.

Adapter emission is not evidence of actual actuator receive or applied force. No command-interface effort is treated as torque. The audit does not independently rerun collision optimization or infer contact dynamics. Same-sign aggregate yaw is not a constant-input plant model. Source269 and f592/1019 remain frozen.
