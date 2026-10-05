# Low-D first4 actual terminal audit

Actual run 20261002_classic_lowD_first4_candidate: original20/20 checks, four immutable windows and healthy cleanup. No source or physical result was modified.

Original region windows: 54.1–54.5, 78.6–79.0, 103.9–104.4, 133.6–134.0 sim seconds. Raw inner / same original initial-SE3 GT outer membership remain in the original evaluator. Each region step takes <37sim s, below its unchanged90sim deadline.

All eight recorded drift stops pass native zero/ACK/nominal return/fresh idle/new reference/complete metadata/published checked-path reconstruction and ≥1sim original pre-turn before motion. Native metadata goal and trajectory identities agree. Collision optimization itself is not recomputed from an occupancy map.

| Native emission mode | Duration s | Emitted vx integral m | Body-forward SLAM / GT m | GT net yaw rad |
|---|---:|---:|---:|---:|
| walk | 49.803 | 5.7118 | 4.5299 / 4.5426 | -0.1417 |
| turn | 31.585 | 0.0000 | -1.7144 / -1.6988 | 2.7075 |
| zero | 34.912 | 0.0000 | 0.0226 / 0.0135 | -0.1621 |

Walk GT forward speed is .0912m/s versus emitted mean vx .1147m/s; response remains imperfect. Same-sign walk-yaw grouped integrals are negative −1.0433 emitted / −2.3238 GT and positive +1.0914 emitted / +2.4963 GT, not a constant-input gain or tracking guarantee. One walk35.652–45.770 has near-zero emitted yaw integral+.005 but GTyaw−.291rad; physical heading drift remains.

The earlier original-profile NAV-only first4 component had descriptive walk mean .0839m/s and pure-turn body retreat−1.0458m, versus .0912 and−1.6988m now. Its task duration was98.35sim s versus116.32 now, and guard counts4 versus8. New positive-vx performance is somewhat higher, while turn retreat and total time have not improved. This is a complete-profile, independently initialized comparison, not a sole-D causal estimate; old PASS/FAILs remain.

The first audit attempt correctly refused to extrapolate last SLAM134.300 to terminal134.320. Its original code/error are retained. The final motion report explicitly covers18.000–134.300 and discloses20ms zero-command tail without a SLAM pose; no pose was synthesized beyond support, and original region/terminal evidence was not changed. GT brackets are≤.15s and SLAM≤.2s; bounded Slerp/position interpolation only.

Report: lowD_first4_motion.json. All native per-mode segments, per-goal intervals, eight handoffs, source hashes and original receipts are saved. Adapter-emitted command is not actual receive or applied torque. No JTC effort, contact dynamics, total-clamp causality, full8/46 route success or new adoption is claimed.
