# Private CHAMP phase canonical first8 — NAV motion and handoff audit

The original eight-region component passed, with active raw tilt 0.236467361 rad, no hold/body contact, and an owned-clean exit. All 16 checks below are true. This does not establish full demo success, perfect command tracking, or general physical stability.

All eight original predeclared inner raw-SLAM/outer GT receipt windows remain unchanged, dwell ≥0.4 sim seconds, each goal timeout 90 sim seconds. Goal-wise conservative origin/prior receipt→receipt durations are [32.6, 27.3, 30.5, 39.5, 58.5, 41.9, 60.5, 45.5] seconds. Full independent native-window/GT/header review is owned by SLAM agent (report 32311a50); no later window or missing source record is substituted here.

The 3 guard stops separate into displacement 1, foreign_identity 2, corridor 0. The displacement stop occurred at raw60.7/callback60.724, goal index1, raw planar movement0.161639m with original .15/.2/min3 gate. The other stops were identity protections at136.124/index3 and198.923/index4. All three native zero→ACK→idle→fresh reference→checked path→original ≥1s pre-turn chains pass, with 675 exact zero emissions through their resumed motion. All45 nonempty accepted paths reconstruct exactly from45 full metadata messages, whose original body centers match the declared route. This verifies recorded association and sampled-path equality; no separate raw Bspline capture or occupancy/collision recomputation is claimed.

## Last uphill goal (index7)

Diagnostic interval308.8→354.3s is45.5s; actual terminal354.401 leaves101ms without native SLAM support, excluded from movement totals. Original goal8 receipt353.8→354.3 is retained. Supported phases: DRIVE38.815s, ALIGN1.836s, PRE1.147s, SETTLE0.898s, and UNCERTAIN2.804s. No guard interruption occurs in goal8.

Ordinary supported DRIVE translation: actual-emitted vx integral4.6073m versus measured SLAM body-forward2.9405m / independently aligned GT2.9267m over38.815s. GT forward speed0.07540m/s; mean emitted vx0.11870m/s. Emitted yaw integral-0.28573rad versus SLAM-0.05429/GT-0.09906. Requested motion is not exact body motion.

Supported ALIGN mixed compensation: 1.425s, emitted vx integral+0.04420m, but SLAM forward-0.04729m /GT-0.04698m. Emitted yaw+0.11400 versus SLAM+0.23039/GT+0.23806rad. Pure rotation0.400s still has GT forward-0.01320m. Arrival passed with these remaining execution mismatches; do not relabel ALIGN vx!=0 as ordinary walking or call the compensation a precise velocity loop.

Whole-run supported ALIGN mixed97.292s still produces GT forward−.39843m; pure rotation23.105s GT−.57338m. Phase-supported actual caps: ALIGN vx≤0.100000000, mixed yaw≤0.080000000, pure yaw≤0.120000000; global vx≤0.119999998, vy0. PD config remains Kp.8/Kd.4/noI/.4s fit/.1cap, no GT/foot-observer input.

## Evidence boundaries and replay

ALIGN support requires consecutive original arm/observe callbacks under the same context/raw gap≤.2s/callback gap≤.25s. DRIVE/PRE/SETTLE support uses adjacent matching original status brackets≤.35s. Whole-run UNCERTAIN24.639s remains explicit. Native command timestamps are logged cached sim-clock floats, not message Headers; these are publisher emissions, not direct CHAMP receives or applied force. Position/orientation interpolation is bounded without extrapolation: original SLAM gap≤.2s, GT≤.15s, one original initial SE3 for evaluation only. Sampled phase/command edges can straddle statuses; the small PRE/SETTLE translation boundary fragments are recorded, not evidence of a live branch issuing forbidden compensation. No million-message CDR redecoding, ROS, new physical run, model fit, or counterfactual trajectory was used.

Replay from repository root:

    python3 multifloor_demo/navigation/test_results/champ_phase_first8_motion_audit/audit_motion.py multifloor_demo/simulation/test_results/20261002_champ_phase_first8_candidate /tmp/new_phase8_motion.json
    python3 multifloor_demo/navigation/test_results/champ_phase_first8_motion_audit/audit_phase.py multifloor_demo/simulation/test_results/20261002_champ_phase_first8_candidate /tmp/new_phase8_phase.json

Outputs refuse to overwrite existing audits. New source/diagnostic receipts are excluded; no production source or original result changed.
