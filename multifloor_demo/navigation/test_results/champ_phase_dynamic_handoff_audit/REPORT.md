# Private CHAMP phase dynamic two-goal audit

Original component30/30 passed, two original receipt windows, active raw peak0.216827271, zero holds/body contact, owned group4171804 cleaned. New independent NAV checks17/17 true. This component is not full demo acceptance, perfect actuator tracking or a general stability proof. Original result, thresholds and historical FAIL remain unchanged.

## Exact obstacle event and recovery

At74.979s native event_000001 committed the exact guard-used self-filtered14681×3 cloud; NPZ SHA55c9a947c38d8c205a1212d81d138af48115ec44ac9a0af1a02a91e23918b4ee. Cloud/odom/filter-body stamp all74900000000ns, camera_init, actual raw pose/rotation and current request/index1/second center match. Native JSON SHA28f52212b965e205dbabadfaefdfb330ac6803af158253390cc5095dc489d383. Replaying original guard on those exact inputs produces(True,0.5697343653541573,84) identically. No later recorder cloud, semantic obstacle label or GT feeds this calculation.

Before that event, positive-body-x ALIGN signed-corridor protection had already stopped at73.032s (63 actual points, clearance.930126m): Adapter zero edge73.045→native ACK73.046→idle73.351. The base obstacle hold starts while already idle. Thus no new Adapter edge is invented and no claim is made that event74.979 first stopped a moving body.

Held interval74.979→new reference97.758 has3426 actual requested/safe samples all exact zero. Its endpoint is validated by a genuinely accepted and subsequently steering-used current-goal trajectory, not only a new timestamp or stale 4Hz hold label. Independently2060 native Adapter emissions remain exact zero through accepted path97.797. Continuous idle before the new reference is24.407s. Current reference97.758→full metadata97.795→checked sampled path97.797→first native nonzero97.820, trajectory13 and original same second goal; old trajectory10/reference74.462 does not authorise resume.

The original HeadingGate aligned-resume rule is used: stopped≥1s and fresh path heading error≤.20. Status98.010 observes aligned_obstacle_resumes1, DRIVE, error−.03613794. It intentionally resumes without a new full pre-turn interval; report does not claim an additional1s wait after this new path. True turns still retain original pre/align/settle. The earlier corridor handoff's zero interval spans the later obstacle hold, so its first resumed motion uses the newer obstacle-clear reference rather than the earlier candidate path.

Guard interventions3 = displacement2 (27.627,33.125, first goal) + signed_corridor1(73.032, second). Original three zero→ACK→idle≥1s→fresh path handoffs all pass. All14 nonempty recorded paths reconstruct exactly from complete metadata and match original centers; no occupancy-map collision rerun or separate raw Bspline recapture is claimed. Source matching retains the live checked-planning/identity contract.

## Motion and boundaries

Original native two sphere windows: [(66.599999999, 67.0), (148.599999999, 149.0)], inner radius.22/outer.30, dwell≥.4/gap≤.2, each deadline90. Conservative origin/prior-receipt→end durations[46.5, 82.0]. Single original SE3 is reused for all GT evaluation, never control.

Supported ordinary DRIVE translation40.942s emits vx integral4.72875m; actual SLAM/GT body-forward4.08334/4.09097m. Supported ALIGN mixed33.091s emits+1.09212m but SLAM/GT move−.36271/−.34366m; supported pure rotation8.777s adds GT−.26447m. Feedback commands do not make the physical response exact. Whole phase-boundary uncertainty30.748s includes the long obstacle zero wait; it is not reassigned to ordinary DRIVE. Active native mixed yaw≤.08, vx≤.12, vy0; PD/guard config unchanged.

Observation ends149.0 before terminal149.057;57ms remains excluded, not extrapolated. Native command sim fields are cached-clock publisher logs, not actual receive/Header or applied-force measurements. SLAM independently owns two-window GT/header/physical box-pose audit and raw/native sampling diagnostics; no missing row is filled here. Only JSONL plus one exact NPZ were read, no million-CDR duplicate decoding or new ROS/physics.

Replay (repository root, fresh output path):

    python3 multifloor_demo/navigation/test_results/champ_phase_dynamic_handoff_audit/audit_cloud_hold.py multifloor_demo/simulation/test_results/20261002_champ_phase_dynamic_candidate /tmp/new_dynamic_cloud_held.json
    python3 multifloor_demo/navigation/test_results/champ_phase_dynamic_handoff_audit/audit_motion.py multifloor_demo/simulation/test_results/20261002_champ_phase_dynamic_candidate /tmp/new_dynamic_motion.json

The scripts refuse to overwrite audit outputs. All source changes in this work are excluded audit files only.
