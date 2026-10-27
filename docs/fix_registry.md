# Fixes and boundaries

| Issue | Current treatment | Evidence / boundary |
| --- | --- | --- |
| DDS shared-memory heartbeat | Keep the actual two-sided handshake fix at both ends. | Source patch and regression fixtures in `third_party/fastdds/`; actual project-local library loading is observed separately. Historical failures have several causes. |
| Causal inputs | Actual SLAM frame pairing and the functional IMU FIFO remain; 20ms causal pairing and 300ms freshness stay. | The migration accidentally disabled the FIFO; run 18 exposed repeated zero holds. Restored before run 19. |
| Daily runtime coupled to experiments | Replace the experiment owner with current mission/reference publication; remove version/source approvals, audit inputs, archive barriers and experiment protocol ledgers from normal control. | Normal entry and all twelve node imports are checked separately. Independent tools perform development measurements. |
| Parking status coverage | Publish zero-hold status every 100ms wall time instead of 250ms. | Preserve original 300ms coverage limit. Earlier coverage failures remain recorded failures. |
| Parking source audit | Remove raw callback/journal equality and historical source approval as daily requirements. | Actual measured position/yaw, current-source timing and zero control remain; explicit tests use the original pose-only 10Hz fixed window. |
| Incorrect observer velocity veto | Remove an accidentally added test-only speed veto that was absent from the existing pose rule. | Run 19's original FAIL is retained; the correction changes no controller or numerical position/yaw threshold. |
| Dynamic clear release | Both branches require ten actual evaluations over at least one second, with the existing 300ms maximum gap. | Run 19 observed thirteen clear samples over 1.055s, one actual stop/recovery. Final-layout results are recorded separately. |
| Multiple stops / raw replay | Test every real stop counter, including a single event; do not infer completion from only one token. | An earlier five-stop failed run still lacks later raw clouds and cannot be retrospectively declared a full replay PASS. Raw cloud capture is not a daily prerequisite. |
| CPU resource aborts | Preserve separate unresolved classification. | No claim of root-cause closure; their data can be removed only through its explicit cleanup batch. |

Reports preserve conclusions and limits. Deleted raw data cannot be reconstructed from summaries. No newly proposed permanent deletion has been executed in this phase.
