# V20 source and evidence checkpoint

Branch: `codex/v20-spatial-corridor-shadow`. Parent/main baseline: `5237b8865d1f788b88f86fe21b72852db3bedb03`. This branch contains the implemented V20 finite-arc reference and shadow corridor exporter, compact evidence from the interrupted 7/9 run, and the next trajectory-selection design. The proposed supervisor is **not implemented**. No active corridor control is enabled.

See [experiment report](../multifloor_demo/teacher_mode/test_results/corridor_tracking_v20_20261006/README.md), [trajectory-selection plan](../multifloor_demo/teacher_mode/test_results/corridor_tracking_v20_20261006/TRAJECTORY_SELECTION_PLAN.md), and [exact export inventory](../provenance/V20_SOURCE_EVIDENCE_EXPORT.json). The original raw data stays on the original machine and is not included here. Compact reports cannot substitute for raw replay or independent reacceptance.

## Dependencies and execution boundary

The V20 SLAM workspace reuses the original V19 pipeline build. Its source is already exported in main; no new SLAM source is introduced here. The V20 SCAN planner is from `multifloor_demo/teacher_mode/navigation/corridor_tracking_v20/scan_ws/src/`, not from a differently named shared planner directory. The additional `scan_planner_msgs` and `traj_utils` underlay sources are already present under `scan_multifloor/ros2_ws/src/planner/`. Their presence does not mean the V20 overlay has been rebuilt in this clone.

The original full-mission dependency files `multifloor_demo/simulation/scenario.json` and `obstacle_trigger.py` are included. Main's portable model/asset resolution changes in shared `teacher_mode/simulation/prepare.py` and `policy/worker.py` are preserved. V20's own runner still contains original host paths and its contracts bind original binaries. Commands in the copied candidate README and old receipts are **historical original-machine commands**, not supported portable launch instructions.

No V20 portable prepare/execute entry or fresh host gate has been implemented. Do not bypass the existing portable V19 runtime block or transplant historical PASS files to authorize a new binary. Runtime integration requires a fresh local rebuild, source/loader/numeric/queue checks and new simulation evidence. Nothing in this branch claims a portable runtime, completed prefix9, original46, Sim2Sim or hardware pass.

The source export excludes `build`, `install`, caches, model weights and raw run streams. It adds `scan_ws/test_snapshot_transport.py` as fixture source, not as a previously executed portable test. Existing finite test commands and original receipts are retained; the export audit verifies file identities and Python syntax without launching ROS, Gazebo, a model, or a build.

## Rollback

The original working project was not switched or reset. The separate export repository holds this branch. To review the baseline from a clean checkout:

```bash
git status --short
git switch main
```

Keep any local uncommitted work before switching; do not use a destructive reset. Return with `git switch codex/v20-spatial-corridor-shadow`. Main remains at the recorded baseline for this delivery. Runtime rollback is a separate deliberate choice of original V19 runner/profile in the original workspace; changing a branch does not change a running controller or certify V19's previously failed original46 task.
