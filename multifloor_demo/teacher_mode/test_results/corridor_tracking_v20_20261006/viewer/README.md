# V20 read-only dashboard storage repair

The dashboard now accepts exactly one additional external recording root: `/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs`. Existing root ownership, exact 0700 root mode, directory identity, run-name pattern, canonical local alias, and escape rejection remain in force. No parent directory or arbitrary external directory is allowed.

V20 adds a read-only shadow corridor panel. It reads at most the final 256 KiB of `corridor_certificates.jsonl`, ignores unfinished trailing records, and requires the selected run path prefix, expected schema, explicit shadow-only state, no control authority, and no navigation ground truth. This is display provenance, not an independent recomputation of the corridor evidence. It never grants navigation acceptance or activates corridor control.

Validation: 28 tests passed (18 existing dashboard tests plus 10 storage/shadow tests). These cover permitted roots/aliases, wrong names, missing aliases, foreign ownership, wrong permissions, root and child symlinks, nested escapes, bounded partial-line handling, foreign or authoritative corridor records, and historical V19 display behavior. HTTP checks confirm the new V20 run and both archived actual RGB files are available. The prior V19 run remains FAILED at 8/46; the V20 record does not establish prefix or full-mission success.

Only the dashboard process was replaced after checking its exact command, owner and working directory. No Gazebo, controller, training, or execution process was touched. The old PID was 1931048; the replacement PID is 2683057. Server log: `server.log`. Exact process identity, source hashes and HTTP observations: `RECEIPT.json`. Test output: `TESTS.log`.

Browser inspection remains the parent agent's responsibility. HTTP observations are a snapshot of the selected recording, not a claim that its simulation is still running.
