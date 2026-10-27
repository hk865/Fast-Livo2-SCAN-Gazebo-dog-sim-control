# Isolated R73 simulation comparison

This local branch is a diagnostic comparison, with no inherited R83/R85 or fixed-five-second parking PASS.

The base is commit `6ab2c64087c85b804d6413f6048f93f56d8c4e4d`. The original source and configuration are recoverable from tag `r73-before-simplification-6ab2c640` and the sibling task's pre-change bundle/archive.

Use the task entry `../start_one_minimal_comparison.py` from the task directory, with its actual final profile and a fresh, explicit one-round ledger/valid bounded time window. The entry requires `GO2_R73_MINIMAL_SIMULATION=1` and the exact simulation-only profile opt-in. It uses the existing Gazebo scene, SLAM/SCAN binaries and unchanged fixed Student export; it never drives hardware.

The minimum flow remains actual corrected localization, the same typed global route, actual local SCAN, bounded velocity control and actual Student45/GRU execution. Repeated wall-age and readiness-business vetoes are observations. Optional prezero/active-handoff/taper/fade/retention/XY-stop modes are off. Geometric heading alignment can release to drive; this does not certify old stop/quiet qualification. Actual endpoint control-band dwell invokes the existing diagnostic stop path, with formal acceptance false.

Finite/interpretable inputs, coordinate and generation consistency, source simulation ages, causal IMU, actual actuator binding, serious physical faults, resource limits and owned cleanup remain. Original timestamps and ACKs are never refreshed or fabricated. The normal mode is unchanged by the minimal-mode branches.

The full patch disposition and run result are in the sibling task directory, outside this source repository. No remote push is part of this experiment.
