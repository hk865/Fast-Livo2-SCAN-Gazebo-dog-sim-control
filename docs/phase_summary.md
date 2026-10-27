# Navigation phase record

The current task is one F1-to-F3 simulation route with a real moving obstacle, measured stop/recovery, current RGB map saving, and fixed five-second terminal parking. It is not the old 46-region mission.

Current changes retain the private two-sided DDS shared-memory heartbeat fix, causal frame pairing and corrected-input handling, dynamic evidence collection, and the current Student45/GRU controller. Failures have different causes; the phase is not attributed wholesale to DDS.

Historical source/binary hashes, prior successful-run identities, approval files and build receipts are no longer startup or acceptance authorities. Current request IDs, planner epochs and coordinate generations still prevent cross-run or stale-frame mixing. Data CRCs and source-packet fingerprints still establish actual recorded data consistency.

Parking status publication has additional margin during zero hold. The parking audit uses actual causal SLAM callbacks without interpolation or resetting the first qualification time. Limits remain 5s, 10Hz, 5cm position drift, 0.1rad yaw drift and 300ms source age.

Dynamic clear release now requires at least ten actual native geometry observations over at least one second in both normal and path-admission hold branches. The pre-roll includes the real causal predecessor of the 300ms boundary; minimum covered duration and maximum source gap remain 300ms. Inline full-route geometry is compared with the actual current reference instead of requiring an audit route-version label.

The final complete regression on 2026-10-27 at 12:37–12:44 UTC passed, with normal exit code 0. It advanced 34.90956m through F1 to F3, passed one actual obstacle stop/release/recovery, saved the current RGB map, and passed the first fixed five-second parking window. Maximum parking drift was 0.027778m and 0.012715rad. All owned children exited, all three locks were released, and 20,415 worker samples passed CRC verification. Four process-loading snapshots found no foreign project paths. This is one complete simulation PASS, not a repeated-run reliability or real-robot result. Exact metrics are in `validation_summary.json`.

There are 51 focused tests. A completed failed run contained five true stop/release events, exposing a verifier that expected one stop and an evidence buffer that sealed after the first cycle. The current implementation checks each actual stop independently and captures each complete window under one shared 48MiB storage/64MiB memory budget. Float32 storage is used only when conversion back to float64 reproduces every actual source value exactly. No cloud is synthesized and no numerical predicate was relaxed. Timing and measured-stop checks passed for all five old events; missing later raw clouds prevent claiming a full retrospective PASS for that run. The successful new regression had one actual stop cycle.

Earlier assembly failures included imports through an obsolete mission module into a retired `simulation` helper. Current telemetry and parking/RGB readers now use the project-local `runtime_evidence` module; an actual child-environment function call and the full regression verified the repaired import chain. Other earlier complete-route attempts passed parking but remained overall failures or a cooperative stop. Their recorded outcomes are unchanged.

After the full run, template cleanup removed obsolete actuation extensions already stripped by the generator and replaced private mesh paths with project-local references. Component comparison produced an identical physics world byte for byte; all frame/joint geometry was identical except 19 mesh filename references. Native binaries were not rebuilt for this migration.

The original main at 990f1c98 and business changes beginning at 06de9bb are retained in normal Git history; they are provenance references, not runtime gates. Historical results from the older 7b954872 line must not be merged into the latest business result counts. Two earlier CPU-resource protection aborts remain unresolved and must not be described as fixed. Their raw data may only be removed after explicit cleanup scope confirmation.

For closed bugs, preserve root cause, fix description/diff history, metric summaries and validation outcomes. Raw investigation data is proposed for removal after exact-scope confirmation; this report does not promise future replay or audit from deleted data. Required local models, robot geometry and current runtime dependencies are protected separately.
