# Independent V44 arrival and clear continuity review

Frozen version reviewed: `navigation_v44_measured_dwell_actual_guard_freeze.json`, SHA256 `714e75459d5a6087cd147498c955013f2d7d7a16899669b1f1046bd04681314f`. All 36 runtime/shared-source hashes match at the recorded audit time.

45 independent offline checks passed. The harness executes the actual shared arrival/control and Teacher override/continuity/guard methods in synthetic accepted sensor contexts; native geometry call sites are mocked. 600 old-profile combinations retained identical arrival results and receipts. Only the new dwell profile continues checked SCAN until the original actual raw-SLAM dwell passes. The 0.17 m radius, 0.6 s dwell, 0.2 s arrival gap, 90 s goal deadline and 0.3 s source/command TTL remain unchanged. Duplicate/backward stamps and protective states cannot create arrival or advance a goal; a next goal cannot reuse the previous raw stamp.

The new clear branch prevents an elapsed cached tick from releasing without the original scheduled native guard in that callback. An actual-guard gap over 0.3 s resets the existing clear timer, fresh source headers are required, and repeated/backward actual guard clocks fail closed before geometry. The inherited 1 s clear span is retained. These flow checks add no geometry computations, no truth inputs, no live signals and no runtime writes.

This is offline semantic/source validation, not an actual navigation, physical parking, full-sensor deployment or Sim2Sim pass. The actual frozen V44 run must still meet the pre-frozen independent physical and navigation criteria.

Earlier audit attempts are preserved: v1's two false checks were incorrect audit expectations (the additional root-authorized clear gap scope, and comparing the shared source to the Teacher archived path); v2 failed before any receipt because its offline namespace omitted Python time. Neither failure was a runtime regression. Revision3 corrects only the independent harness, using the same frozen source bytes.
