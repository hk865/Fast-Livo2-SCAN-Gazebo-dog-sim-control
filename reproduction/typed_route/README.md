# Relocated typed global static certificate

`typed_r85_global_reference.json` is a fresh static recertification of the exact original prepared337×3 ground/body arrays. It is not the old `allowed` certificate renamed. Current `candidate/r85_geometry_admission.py.check_typed_polyline` was called exactly once with new-package geometry inputs:1672 subsegments, maximum spacing0.025000000000000355m, zero physical failures,6.684042903s wall/6.683879376s process CPU. The existing non-gating observation diagnostics were disabled; every original physical support/body/collision/terrain limit was evaluated.

`physical_route_admission.json` has `allowed=true`, `ready_for_control=false`, `navigation_PASS=false`; actual runtime freshness/dynamic/ACK/control checks remain pending. No route search, policy/math/threshold change, Node or simulator occurred.

- All four route arrays, task endpoints, physical offset.32 and T_navigation_world are exactly original.
- All43 reference boxes and original metadata geometry/grid/face/transform numeric fields are unchanged; limits remain original b07c… bytes.
- Missing canonical frame was copied exactly from the original metadata's SHA189b… binding (246479B).
- Only authorized identity constants changed: new dependency helper REFERENCE_SHA matches the same-geometry published reference; candidate LEGACY_SHA matches the relocated helper. Every function/class AST is unchanged by these two pin updates; CONFIG_SHA remains original.
- Metadata inputs.reference/world/canonical_frame now bind actual new-package paths/SHA/bytes. Original/public/relocated hashes are in SOURCE_IDENTITY_CLOSURE.json and LINEAGE_BEFORE_CALL.json.
- Historical source_task/raw failure provenance remains historical, not an executable old-worktree dependency or a rewritten old PASS. Two source_task file strings were already public aliases; their numerical/other identity fields were not changed.

The first preflight assertion stopped before any business call because it compared those two historical source paths byte-for-byte. PREFLIGHT_PATH_NORMALIZATION_NOTE.json preserves the distinction. CALL_INTENT.json guards against a second business invocation. Parent PROFILE/prepare must use this new typed/certificate and separately perform fresh loading/43-box equivalence/runtime acceptance.
