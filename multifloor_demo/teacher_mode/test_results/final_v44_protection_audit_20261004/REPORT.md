# Post-V44 protection, ownership and source audit

Run `20261004_102555_navigation_slam_scan_dynamic_flat_v44_dwell_r1_1536`: 18 read-only audit checks passed. This audit assigns no physical parking, dynamic obstacle or navigation acceptance result. The pre-frozen independent evaluator remains responsible for those criteria.

The V44 freeze receipt has authorized SHA256 `714e75459d5a6087cd147498c955013f2d7d7a16899669b1f1046bd04681314f`. All32 current frozen runtime hashes match. All29 runtime source files actually present in the run's original sources archive also match. Three entries were not archived there: the obstacle observer binary, native actuator binary and global acceptance file. They match the current freeze/protection baseline, but are explicitly not counted as archived executed binary copies.

The checkpoint, frozen env/agent training archive, original camera_mode generated assets, original Demo physics assets, native actuator and original global acceptance retain their protection hashes. Actor inference remained CPU with one Torch thread. The actual native initialization receipts confirm one sole Teacher joint-force writer, kp25/kd0.5, physics step0.005s and decimation4; the run manifest disables CHAMP and other joint controllers. Navigation truth is not used.

All6 expected self-owned processes returned0. All15 children listed as started in the navigation launch log have matching clean exits, with no died/missing/unexpected clean records. No recorded PID remains and no current process argv refers to this completed run. The worker completed without a fault. The actual guard writer drained394 records, with exact sequences1..394 and no queue error.

Resource snapshot at Beijing10:30:49: loadavg2.33/2.43/1.91, GPU943MiB with21% utilization; no training candidate process was found. This script issued no signal, training launch or training-file write. Matching hashes and a process snapshot do not prove every historical process action; the explicit scope is the present protection evidence and root-owned process receipts.

Original global acceptance is preserved: interface passed; motion and Sim2Sim failed; navigation and real robot unverified. The local finite V44 scope must be reported separately from broader deployment readiness.

Evidence: `audit.json` SHA256 `d4fa891c89f2729d481d8d6931e443c8f6728b00a8b831187d43c92a76fee664`. No runtime, original log, README, current_status or package inventory was modified. All files written by this audit are inside this new independent directory.
