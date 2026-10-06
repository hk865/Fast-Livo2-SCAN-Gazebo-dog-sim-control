# V24 full46 evaluator adaptation

This reader preserves the frozen V23 adapter and original full46 evaluator checks. The 22 formal gates, all thresholds, original 18+14+14 regions, deadlines, first fixed five-second parking window and cleanup checks remain unchanged. Missing complete 200 Hz/native, PI/publication and all SCAN geometry replays remain UNVERIFIED.

Only candidate selection/provenance changes. The existing source-closure gate additionally requires the exact V24 shared_controller.py and replan_policy.py paths, pinned hashes, canonical archived bytes, source/snapshot references and both manifest aliases. Same-name V23 files cannot substitute. No runtime Controller is replayed.

13 finite reader tests passed, including AST scope equality and source-closure negative cases. The small prepare09f7 source metadata was read; active run raw logs were not read. An initial test-generation cwd mistake failed before creating tests; the corrected 13-test execution is recorded in FINITE_READER_REVIEW.json.

Run only after the owner confirms all writers stopped:

```bash
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/curvature_full46_20261006/v24_evaluation/evaluate_full46_v24.py --run /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_173509_closed_loop_cascade_clock_hold_curvature_on_V24_full46_r1_27bd --output /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/curvature_full46_20261006/v24_evaluation/27bd_FULL46_ACTUAL_EVALUATION.json
```

Output must be new and outside the immutable run. This command retains the original evaluator's file/hash behavior; it does not make a claim that every large file is read once. Source closure hashes archived source/local references, and the original evaluator reads its required SLAM/status/native/PID/CSV streams. Do not run repeatedly or concurrently with writers.

For source-only readiness: `python3 -B evaluate_full46_v24.py --check-adapter`. The report carries this adapter's SHA; FINITE_READER_REVIEW.json pins the adapter, tests, all parent readers and current control/repair source bytes.
