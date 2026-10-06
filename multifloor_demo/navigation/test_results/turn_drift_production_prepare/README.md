# Future clean entry, not adopted

`drift_controller.py` and `turn_drift.py` are prepared for the standard final
`multifloor_demo/navigation/` folder. Their bootstrap uses that folder's own
path, without `DEMO_TEST_ROOT`. The original frozen f592/1019 files and all
269 collected production sources remain unchanged.

The only changed executable method is `drift_context`: legacy goals return no
context, so a legacy pure turn cannot arm the additional stop/replan guard.
Legacy point arrival and deadlines keep the original base behavior. All v2
stop/idle/identity/progress/heading/deadline snapshots match the tested f592
candidate. Its original legacy pending-arrival failure negative is retained.

The 27 selected-profile/integration tests and nine scope/equivalence tests
pass. A fresh import from a deeply nested standard archive passes eight
checks; it does not initialize a ROS context, create a node, or launch a
process. Local dependencies and the default scenario resolve within that
archive. Ten copied files match their input bytes.

`guard_configuration.json` declares the exact actual selected `.15 m/.2 s/
3 raw observations` profile and source hashes. It is a provenance declaration,
not a new runtime configuration input. Root owns final stack selection and
runtime hashing. See `preparation.json` and `controller_preparation.patch` for
complete receipt and minimal diff. Physical slope safety, arrival performance,
dynamic interaction, and the full demo remain separate acceptance questions.

```bash
python3 multifloor_demo/navigation/test_results/turn_drift_production_prepare/test_prepared.py
python3 multifloor_demo/navigation/test_results/turn_drift_production_prepare/test_legacy_scope.py
source /opt/ros/jazzy/setup.bash
source scan_multifloor/ros2_ws/install/setup.bash
source multifloor_demo/navigation/ros2_ws/install/local_setup.bash
python3 multifloor_demo/navigation/test_results/turn_drift_production_prepare/test_archive_import.py
python3 multifloor_demo/navigation/test_results/turn_drift_production_prepare/finalize_receipt.py
```
