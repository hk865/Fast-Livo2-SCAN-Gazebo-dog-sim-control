# Frozen V9/V10 prefix results

These are separate single 210 s simulations with unchanged Teacher CPU single-thread inference, command gains, 20 Hz wall timer and 300 ms source freshness gates. They are not statistical estimates of an optimum and do not constitute complete 32-region navigation or full ramp/contact acceptance. Gazebo state is used only offline.

| Item | V9 64 lines / LiDAR20 / RGB20 | V10 64 lines / LiDAR30 / RGB30 / LIO2 |
|---|---:|---:|
| Run suffix | `195524…2a1a` | `200025…70fc` |
| Actor samples / native duration | 10501 / 210.005 s | 10501 / 210.005 s |
| Actual raw LiDAR / RGB source rate | 20.00095 Hz each | 30.30433 Hz each |
| LIO optimizer records | 4128, through210 s | 6238, through209.455 s |
| VIO optimizer records | 4122 | 6231, through209.420 s |
| Whole phase wall throughput | 19.809 Hz | 28.018 Hz |
| Actual accepted navigation poses | 4128, no>300 ms source gap | 3291, eight>300 ms gaps; largest43.495 s |
| Controller fresh math updates | 3194 | 638, last source71.945 s |
| Original measured region arrivals | 25/32 | 4/32 |
| Physical-phase navigation failure | None | First failed82.26 s |
| Actor CPU forward p95 | 0.403 ms | 0.460 ms |
| Native body contact/fault samples | 0 / 0 | 0 / 0 |
| All accepted-source height error | max71.26 mm | max8.01 mm, accepted source epochs only |
| Original131–134 s fault window | max26.60 mm | No sufficiently timely navigation source; N/A |
| Original independent source / native physical safety | PASS | PASS |
| Original guard geometry/freshness | PASS | FAIL:14 stale/future source joins |
| Original speed/heading/route metric | PASS | FAIL:speedMAE0.058707 m/s |
| Full32-region gate | FAIL:finite prefix | FAIL:stale-source stop |
| Final5 s active parking / complete nonflat contacts | Unverified | Unverified |
| Clock-hold source/integrity/reset/resume, PI replay | PASS | PASS |
| Exact publication-slew replay | PASS | Missing publisher chronology, retained FAIL |

V9 has15 duplicate-clock holds in physical execution and four wall-stalled holds during cleanup. Its independent PI maximum error is1.69e-15; the publication-slew maximum error including holds is1.73e-14. Cleanup-only stale input does not replace the successful running-prefix status.

V10 has two physical duplicate-clock holds, not a backward-clock failure. One was already protected because original SLAM/cloud/paired sources were stale. Its actual independent PI replay has638 updates with maximum error1.67e-16. All frozen model/core and prospective criterion sources bind correctly. Its failure message was later overwritten by the wrapper's waiting-for-stop text, so that text alone is not a cause diagnosis.

The V10 exact-slew receipt does not include independent times for asynchronous IMU/bridge `publish_command()` zeros outside PID/clock-hold records. There are99 replay discrepancies; every one has an actual independently archived requested-zero topic receipt between the preceding math/hold event and the next math event. This explains why replaying only PID/hold commands uses an incomplete publication history. A topic receive clock cannot be substituted for the producer's actual publication anchor. The original failed receipt is preserved; these diagnostics do not assert that the slew formula is incorrect or upgrade acceptance. Future V11 adds a producer publication ledger under a separate prospective criterion before physics starts.

The complete LIO/VIO source headers in V10 do not prove timely navigation output. Source-cadence processing approaches30.30 Hz, while the whole wall pipeline averages28.018 Hz and navigation rejects stale sources. Before the detail window, median LIO state solve is22.681 ms and VIO solve3.008 ms. These end immediately after the state/visual solve and exclude map updates, colored-cloud/image/path publication and dispatch/wait. They cannot be used to claim a complete33 ms pipeline budget. The existing two-thread LIO trial did not solve the complete pipeline issue; this single run does not isolate a unique CPU/GPU/logger cause.

The V10 final writer records262688 attempted/written, zero dropped records and47,901,621,560 bytes. The **actual** detail window is115–165 s. V9/V10 short-window profiles were merely declared and were not executed with a3 s detail window because their launcher hardcoded the original window. Future V11 reads the actual profile window and removes unused diagnostic allocations outside it. Outside-window kind100 and VIO kind201 detailed data will be recorded as not collected, not zero constraints or zero optimizer work.

Formal original and hold-aware receipts are appended in each run. Selective analyses and exact source hashes are under `comparison_v9_20/` and `comparison_v10_30_mp2/`. V10 `pipeline_clock_and_zero_evidence.json` contains the99 actual topic-zero joins and phase/source clock age brackets. No whole large diagnostic-binary scans were run during the next physics trial.
