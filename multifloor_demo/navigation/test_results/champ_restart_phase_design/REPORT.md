# CHAMP first-swing phase design, offline only

The smallest candidate removes only the two assignments that suppress RF/LH's initial swing signal in `phase_generator.h:122–123`. It retains LF/RH's initial stance mask, all phase clocks, the .35 s stance/.25 s swing/.60 s stride, Bezier control points, IK, zero reset and subsequent trot pairing. The patch is `candidate.patch`; it has **two deleted lines**, with no added algorithm or parameters. Production source and installed CHAMP artifacts remain unchanged.

The original half-stride offset gives RF/LH `leg_clock = elapsed − .30`. Their swing starts when `leg_clock > −.25`, i.e. **elapsed > .05 s**, and finishes at .30 s. The original first-cycle mask nevertheless forces their swing signal to zero until LF stance reaches .5 at .175 s. Releasing it then enables swing phase **.5** and approximately **69.036 mm** of vertical lift at once. Deleting only those two assignments lets the original swing start at its phase-zero boundary and progress through the complete .25 s interval. Once elapsed reaches .175 s, both implementations set `has_swung` in the same branch and their state/output matches exactly in the tested subsequent strides.

This is **not a whole-trajectory C0 guarantee**. At exactly .05 s the strict source boundary gives both phase signals zero, so `TrajectoryPlanner` retains its previous nominal foot. The next positive swing sample approaches the unchanged Bezier endpoint `x = −step_length/2`, `z = 0`, producing a horizontal endpoint jump. LF/RH still begin their stance at .175 s with x=0 and z=−5 mm. These residual boundaries are kept separate from the removed 69 mm midpoint-lift discontinuity.

The native harness calls the **installed original** `BodyController → LegController/PhaseGenerator → TrajectoryPlanner → Kinematics::inverse` headers directly. Candidate compilation overrides only its private copied phase header. Link origins come from the actual run's measured URDF, mapped explicitly LF/RF/LH/RH. Initial nominal IK matches the archived raw reference within 1 µrad. No ROS, DDS, Gazebo, production package build or controller was started.

## Actual-input source replay

The input comes from original Adapter emissions beginning at323.191 s: initial vx=.0075 m/s, yaw=.01198539 rad/s, followed by the recorded changes. Emission time is used for a ZOH input reconstruction. Internal CHAMP callbacks and phase were not recorded: these deterministic simulation-time grids **are not a replay of an observed native callback schedule**. The production CHAMP timer is5 ms wall time, while phase is based on ROS simulation time.

| Source sampling grid | Original max joint step, first300 ms | Candidate max joint step | Original max foot step | Candidate max foot step | First RF/LH swing, old → candidate |
|---|---:|---:|---:|---:|---:|
| 1 ms | .416427 rad | .033340 rad | 69.048 mm | 5.000 mm | 175 → 51 ms |
| 4 ms | .417331 rad | .034794 rad | 69.205 mm | 5.631 mm | 176 → 52 ms |
| 5 ms | .416427 rad | .043482 rad | 69.048 mm | 7.018 mm | 175 → 55 ms |

The5 ms candidate maximum occurs at280 ms during the ordinary Bezier descent, not at mask release. At all sampled times ≥175 ms through1200 ms, original and candidate **phase signals, foot positions and joint targets are exactly equal**. At5 ms this is206 paired rows. The original42-check receipt remains immutable; the one-scenario5 ms supplement has7 independent checks.

## Deliberate counterexamples and unchanged boundaries

- Full straight vx=.12 m/s: step_length=.042 m. At the first1 ms swing sample (.051 s), the candidate still jumps horizontally21.366 mm, with max joint step .068783 rad. This disproves a claim that deleting the swing mask makes all first-foot coordinates continuous. At .05 s exactly, the previous nominal foot is retained.
- Pure yaw ±.12 rad/s: step_length is about10.077 mm. The same start-phase fix applies even when vx=0; it does not depend on a forward-speed envelope. Opposite leg step directions and subsequent diagonal pairing are unchanged.
- Tiny nonzero vx=1 µm/s: the original midpoint lift remains about69 mm despite the almost-zero step. The candidate removes that midpoint jump, but the unchanged5 mm LF/RH stance-depth edge remains. No arbitrary small-speed threshold is introduced.
- Any large-time zero→restart in the tested stream resets all phase signals/`has_swung`, restores nominal IK during zero and reanchors the next cycle. The candidate changes none of the zero branch. Stopping during a swing still returns nominal according to the original code; this proposal is not a stop-transition smoothing change.
- First boot at absolute100 ms starts at phase0 through the original `!has_started` branch. A deliberately early zero→restart at absolute200 ms carries elapsed200 ms because `has_started` is not reset and time is still less than a stride. Both profiles therefore begin at RF swing .6. This is an existing, separate early-clock limitation; no patch is proposed here. It is not the actual323 s restart cause.

## Why this candidate is smaller than alternatives

Keeping the complete first-cycle mask and remapping swing to phase0 at175 ms either compresses a .25 s swing into the remaining125 ms or shifts first touchdown away from the existing300 ms offset. Either choice changes first-cycle timing and diagonal overlap. Resetting only `has_started` would reanchor elapsed but would retain the175 ms midpoint release. Deleting the entire first-cycle mask would additionally expose LF/RH's own initial stance endpoint near `+step/2`, an unnecessary second behavioral change. A downstream joint-target envelope changes the output time response but leaves the identified phase boundary in place. Those designs are not combined with this two-line candidate.

The candidate's mathematical promise is narrow: RF/LH's first swing uses the original phase-zero onset rather than being revealed at phase .5, while output after the original175 ms release remains identical. Contact support, actual joint response, tilt and route safety still require a fresh controlled physical comparison before adoption. The original canonical8 failure and all source/runtime archives remain unchanged.

## Provenance and reproduction

`../champ_restart_phase_source_audit/source_receipt.json` contains original source=installed header hashes, actual selected executable/library hashes, compiler dependency/flags and the original runtime manifest. Installed phase source SHA is `c204d0ec060ceb72ed8bbd11c092a8f73d10869146fdd382222ef1fb8fab3f32`. Private candidate phase SHA is `9d0c0040221a328a78134649dd31c5cb59aa5383551325d35fda76d5eff95400`; patch SHA is `fab8b5321a1eebfb3bb9b9c329d1f19f4a68d2dcc36dbd4f5bc606318c4c82e7`.

Original runtime node SHA is `634e7e3caef6c07eb833a71b7696f57dd0beacee8fc0744bc5609194d4953f62`; runtime `libquadruped_controller.so` SHA is `aee71be83925e129c969382b099c6cc600bb76631e30fd7c9acec2b562934e13`. These artifacts were checked against the original run; the harness does not replace them.

From the repository root, the reproducible offline commands are:

```bash
python3 multifloor_demo/navigation/test_results/champ_restart_phase_design/run_replay.py
python3 multifloor_demo/navigation/test_results/champ_restart_phase_design/supplement_5ms.py
```

The first command compiles only the two local harness binaries and records compiler arguments/dependencies. The second reproduces only the5 ms supplement. `result.json` SHA is `d3823a3d471ea2a39904cc7d18b5c686986c0282cf99681f888241b06d21d0a3`; `supplement_5ms_result.json` SHA is `ac2bc7524ec3133755fd93e4fe0874d9ff47977cae58681c9598e5d952bc9f5c`. The first local harness build had a formatting-function name clash with `std::array`; its failed build log is retained as `build_original_first_attempt.log`. No source algorithm changed to resolve that harness-only compilation issue.
