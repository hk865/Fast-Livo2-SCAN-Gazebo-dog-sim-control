# Future integration checklist — read-only, not adopted

Source-only review against the owned-clean canonical first8 `20261002_champ_phase_first8_candidate`. This checklist changes no production file, launch, binary, controller gain, previous FAIL, or acceptance window. The component reached eight regions; full demo remains unverified.

## Standard NAV entry

Copy prepared `align_controller.py`, `turn_drift.py`, `align_translation.py` to `navigation/` only after Root adoption. Their SHA prefixes are 945a5acb/e11ed0f5/7d9fa373. All class/function ASTs match the tested 4e770555/e13e4ebb/7d9fa373 except `NavigationDrift.drift_context`, which additionally returns None for legacy goals. All schema2 control methods are unchanged; the PD helper is byte-identical. This legacy exclusion preserves old legacy behavior and avoids using the experimental pending guard on old arrival timers.

Execute `python3 <ROOT>/navigation/align_controller.py --ros-args -p scenario_path:=<ROOT>/simulation/scenario.json` in the usual frozen ROS/Demo SLAM/NAV environment. Set `DEMO_TEST_ALIGN_TRANSLATION_ENABLED=1` explicitly (the current prepared constructor accepts only 0 or 1), `DEMO_RUN_DIR`, and modern `DEMO_JOINT_STOP_ADAPTER=1`. Prepared ROOT is `Path(__file__).resolve().parents[1]`; it is correct only at the intended navigation path or an archive retaining that tree. Running the current deep prepare path directly is not a valid standard import test. Helper imports resolve beside navigation; do not add a hidden dependency on test_results.

## Minimum tested roles to retain

| Role | Current tested implementation | Difference from current production |
|---|---|---|
| NAV PD plus drift/identity handoff | Three sources above, schema2 | Production still base controller.py; root selects new entry |
| Modern command and stop acknowledgement | Production Adapter and bridge, modern actuator/raw-reference topics | Adapter input is relayed safe topic in tested stack |
| Disabled body relay and required health | body_stabilizer_node.py (no --enable), bridge_with_feedback.py + feedback_core.py; timed_feedback_bridge.py if timing diagnostics selected | Exact finite safe command pass-through, identity body_pose and required state watchdog; removing these or enabling balance is a different topology |
| Pose remap | gait `/body_pose` -> `/demo/test/body_stabilizer/body_pose`; Adapter safe input -> `/demo/test/body_stabilizer/safe_cmd_vel` | Production currently direct safe input and default body_pose |
| Measured joint source | prepare_joint_sensor generated Gz JointStatePublisher plus separate parameter bridge to `/demo/control/measured_joint_states` | Actual measured joints, separate from JSB; explicit names mapping, physical Header |
| Joint controller profile | interpolate_from_desired_state=true, P220.982919/I.2/D1/i_clamp2.5, FF0, CM enforce_command_limits=true, update250Hz | Production old false/P100/D1/default CM false; selected profile requires actual BOOL/gains service readback |
| Selected gait | CHAMP header patch removes only first RF/LH swing mask; selected node2527d4c9/DSO60ae6c05 | Current production ament selects original gait; no other gait/stride/swing/envelope change |
| Startup gate | spawners exit0 -> services ready -> actual64 parameters -> owned Gz five libraries -> selected gait maps -> original静稳 -> SLAM/SCAN/NAV | Current stack starts wait_sensors without these explicit parameter/loaded-library gates |
| Ownership and exit | Required node exits shut stack; recorders subscribe before motion; zero/ACK/idle chain preserved | Keep shutdown and archival semantics; task driver is not production Mission |

Disabled relay is not active balancing. It still provides status/liveness, validates messages and sends zero when failed or upstream stale; `FeedbackHealth` reports required=true even when enabled=false. Its health deadline is .30 wall once established. The bridge combines this health with original IMU/Adapter guards. Keep exact publish/subscriber/remap topology or validate a separately declared replacement. No GT/ideal body pose/foot observer feeds NAV.

Selected gait is loaded by absolute executable plus gait-child-only LD_LIBRARY_PATH. Preserve global AMENT and Gz-only CM isolation. Private CHAMP source consists only of champ (header-only) and champ_base, official LICENSE/provenance, the precise two-line patch and reproducible local build. The node SHA is 2527d4c9e64b7562a57e6ba8e9559c98b3828eae9463af16df7e78880138df64; DSO SHA is 60ae6c054c143ec7751297906cbd9f9ff0b3a6d9eb48084ad7dc8f6deeb9d1f8. Selected header SHA9d0c0040221a328a78134649dd31c5cb59aa5383551325d35fda76d5eff95400. Baseline gait binaries are retained as references, not selected artifacts.

Actual measured joint plugin is installed `libgz-sim8-joint-state-publisher-system.so.8.11.0`, SHA a0981fa8f63d974d2d7cf49b44cf9cc077495f8676a713e358fda6fdf99640b6. Its separate bridge is GZ /demo/physical_joint_states -> ROS /demo/control/measured_joint_states, SENSOR_DATA queue5, non-lazy. Do not replace actual q with nominal/FK values.

## Source and runtime provenance changes Root must make

`mission/artifacts.py.snapshot_sources` excludes test_results/build/install. Current private CHAMP source and staged relay/gates will therefore be absent until copied to normal collected paths. Put adopted two-package sources, LICENSE and origin/patch provenance into a Demo isolated workspace; standard future build must not depend on an excluded preparation script. New production source count/fingerprint must be fresh, not called the old269 freeze.

`snapshot_runtime` currently resolves champ_base node and libquadruped_controller.so through ament; that resolves original libraries despite a privately selected launch. Resolve the same selected gait executable/library as launch, hash both and include selection/child loader scope. Keep CM selected-path logic separate. Include measured plugin, actual generated URDF/private YAML normalization, profile and mode declarations. Actual startup/prestop /proc executable/maps receipts verify launch selection beyond a manifest claim.

Full Mission, scene goals/regions and owned scene timing remain Root production components. Component probes, copied runners, and large CDR suffix recorders are evidence tools, not replacements for Mission. Do not change historical results. Architecture/map-cache/README HTTP candidates are Root-owned and intentionally outside this checklist.
