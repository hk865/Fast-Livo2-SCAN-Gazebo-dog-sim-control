# V25 terrain event repair (simulation only)

This independent candidate keeps frozen V24 controller, curvature switches, gains, 300 ms source limits, unique CPU Teacher executor, 187 height rays and original 18+14+14 mission geometry. It changes only `SwitchingTerrainProvider._record`: accept an incoming literal `navigation_ground_truth_used=False`, reject any other explicit value, then preserve the payload and assign authoritative event sequence, local recording time and the false navigation flag.

V24 R2's successful terrain ray check was followed by a duplicate-key `TypeError` in the event logger, which latched protective zero velocity. The failed V24 run is preserved. Obstacle-event callers were audited; their actual payloads do not duplicate reserved recording fields, so the obstacle implementation is unchanged.

Fresh finite checks cover the actual terrain-provider constructor and three production `check_switch` transitions using the frozen SDF geometry and synthetic original source receipts. The exact worker request prefix through its terrain override is replayed with a declared fresh-command fixture. This is not full native IPC, inference or navigation validation. There are 63 fresh finite checks; the candidate also requires a new independent review and source-bound gate before preparation. It explicitly inherits frozen V19 SLAM and protected SCAN binaries and verifies actual loaded binaries before Gazebo.

```bash
python3 -B run.py --profile curvature_original46_on --label v25_prepare --prepare-only --run-storage-root /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs
```

Actual launch is performed by the root task after resource and run review; no physical simulation has been launched by this candidate's implementation task. Startup/storage/safety/arrival gates are unchanged. No retraining or real robot operation is included.
