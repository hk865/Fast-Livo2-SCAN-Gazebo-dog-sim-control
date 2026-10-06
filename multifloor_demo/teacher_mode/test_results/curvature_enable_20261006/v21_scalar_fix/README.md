# V21 curvature scalar repair — finite evidence only

The frozen V20 defect is independently reproduced using the original production numerical profile and real `Controller` update code with mocked fresh sensor/clock fixtures. After a curvature sign change, `reference_budget_feasible` is `numpy.bool_(False)`. Its truth is false, but `value is False` does not match. V20 consequently stays in `drive`, produces a nonzero command, and updates translation PI; the same PID row also fails the actual `EvidenceWriter.append` JSON serialization. This is a control predicate defect as well as a logging defect. These fixtures are not a replay of the failed physical run.

V21 changes only the numerical boundary of `spatial_reference.speed_budget`: inputs and numeric outputs become native Python `float`; predicates become native Python `bool`. The equations, gains, limits, tolerances and cross-frame behavior are unchanged. `controller.py` and `cascade_core.py` are byte-identical to frozen V20. The spatial source increases by 629 bytes. Runner/profile/gate wiring is separately owned and evidenced in `../v21_wiring/`.

The same production fixture now enters `reference_constraint_hold`, sends exactly `[0, 0, 0]`, preserves the integrator, and serializes successfully. The actual outer wrapper also emits exact zero despite a prior nonzero command. No catch-and-discard logging workaround was added.

The executed command was:

```bash
cd multifloor_demo/teacher_mode/navigation/corridor_tracking_v21_curvature
python3 -B -m unittest curvature_scalar_tests spatial_reference_tests controller_reference_tests pure_tests -v
```

All 46 tests passed (7 new scalar regressions and 39 inherited spatial/controller regressions), including 270 finite old/new numerical comparisons. The earlier curvature admission tests did not exercise the NumPy scalar boundary reached when production limits are active; the frozen earlier evidence is preserved and does not certify this candidate.

Evidence:

- `V20_NUMPY_BOOL_FAILURE_REPRODUCTION.json`: exact old/new source bindings, direct types, identity predicates, production commands and PI deltas.
- `FINITE_REPAIR_RECEIPT.json`: actual command, 46 test IDs, source bindings and finite scope.
- `CONTROL_REVIEW.json`: new limited review for the V21 preflight; actual navigation remains unverified.
- `NUMERICAL_PATCH_SCOPE.json` and `SPATIAL_REFERENCE_SCALAR_REPAIR.diff`: numerical source scope and before/after hashes.
- `COPY_PROVENANCE.json`: initial plain-source copy, with no inherited certification or build/workspace copying.

The new source-bound gate and exact ON profile must be closed separately before a root-owned rerun. The original route, dwell, parking, source and physical acceptance checks remain required. This repair does not certify plant response, full 46-region completion, or native 200 Hz physics. No ROS/Gazebo run was started for this review.
