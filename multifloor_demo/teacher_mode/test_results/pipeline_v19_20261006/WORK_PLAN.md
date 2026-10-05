# V19 original work plan — 2026-10-06

Final outcome: implementation and tests complete; original46 r3 FAILED 8/46. This file retains the original plan; see [final report](README.md). New V19 raw is retained locally.

User requested: after cleanup, complete multithreaded pipeline/throughput optimization, then attempt original full46-region task. Only simulation, frozen CPU Teacher, no RL/training or passed camera changes. Throughput bound is longest independent stage; IMU/LIO/VIO share state and remain sequential owner stages. Never promise one-frame max(LIO,VIO) without an algorithm proof.

Cleanup DONE: 300941 raw files / 377442220572 bytes (351.520 GiB); remote code/evidence/cleanup commit 027724b. New portable source build/150 finite checks/V18 prepare-only now completed and final tools push underway. Frozen checkpoint SHA bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34 unchanged. All old full raw replay unavailable; keep historical failures and compact summaries. No active Gazebo/SLAM/training processes at task restart; own old replay servers stopped because their raw was deleted. New tests must use new run IDs.

New candidate: navigation/pipeline_v19 seeded from original V18 source, excluding builds/install/logs. Seed metadata retained. Any copied V18 PASS is historical and cannot authorize V19. New preflight required.

Roles:
- root: Python launch/run/stop handshake/preflight, build integration, actual controlled experiments, observer/browser, Git export, final reports.
- lidar_lio_multicore_build: V19 slam_ws/src CPP/CMake + test_results/pipeline_v19_20261006/core. Combine V18 math with staged RX/cloud decoder/image decoder/ordered sole estimator; full inflight capacity512/64MiB; normal stop context valid SIGUSR1 request/drain/close then shutdown; strict failures identity logged. No physics.
- lidar_rate_sync_audit: performance/ reader and same-input A/B/capture/replay design; no runtime edits or physics.
- lidar_experiment_evaluator: PROSPECTIVE_EVALUATION.md + evaluation/, new mission46*.py pure adapter modules only; original46 geometry/phase/dynamic preserved. No physics.

Execution order:
1. Independent lifecycle/queue/production semantics/finite arithmetic/FP/source-binding preflight. Modes serial, RXdecode, staged must be explicit and bound. Do not weaken300ms, drops, cancellation or controller ownership.
2. Capture fresh short actual sensor input if needed; compare matched input throughput, stage wall/threadCPU, tail latency, age, counts. ROSbag bytes match does not imply cross-topic admission order match. Actual30Hz sustained output does not prove peak capacity. wall-threadCPU is not pure communication.
3. New60s lifecycle/physical prefix; scoped ramp/32 validation if needed; preserve failures. Stop only owned processes (PID starttick/PGID/exe).
4. Full46 is original scenario: exploration18 + return14 + save actual RGB map + navigation14, original dynamic obstacle and original regions. It is not V1832 with more points. Root must integrate Teacher-only mission adapter, bidirectional floor/ramp Actor height semantics and actual start-origin check before attempt. Original start roof/height mismatch failure preserved; do not silently move spawn/relax geometry.
5. Full46 bounded1500sim seconds, prefer natural completion, ~121.36m route. Plan up to120GiB/run with>=150GiB available +30GiB reserve; log source/contacts/commands/safety needed for independent evaluation, bounded detail115–118s. Do not regrow unbounded logs.
6. New code/results copied to authorized GitHub repo git@github.com:hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control.git with relative portable controls and new validation. Never inherit original physical PASS into exported rebuild.

Original V17 actual efde failure: accepted31615/committed31614/canceled1. Mechanism stop_ingress cancel+join then queue.close clears pending; last missing type/header/receipt unknown. Fix before new actual; original FAIL stays.

V18 actual prior: 32/32 two12m UP ramps and5s parking passed once; full46/dynamic with newest config not verified; strict complete Isaac/Gazebo Sim2Sim remains not passed. Preserve all distinctions.
