This is a prepared adoption recipe, not permission to modify the frozen production stack or proof of improved physical control. Root decides adoption after the sole simulation owner's actual native-period and startup comparisons.

Suggested production workspace: `multifloor_demo/simulation/ros2_control_ws`. It owns one package, `controller_manager`, copied from the immutable official4.45.2 archive. `prepare_overlay.py` checks archive and baseline CPP SHA, applies the exact reviewed two-expression patch, checks candidate CPP SHA, preserves source license headers, copies the upstream Apache license into the snapshot-recognized `package/LICENSE`, and writes patch provenance. Any existing package license must match the official archive exactly. It requires an explicit workspace and refuses to overwrite a package lacking its own provenance marker. Package version remains4.45.2; the separate patch SHA distinguishes the local change.

After root authorization, build with the staged script:

```bash
bash multifloor_demo/navigation/test_results/cm_time_contract_staging/build_overlay.sh \
  "$PWD/multifloor_demo/simulation/ros2_control_ws"
```

No `apt` replacement is needed. The minimal runtime selection adds only this directory to the Gazebo `ExecuteProcess` environment:

```text
LD_LIBRARY_PATH=/absolute/ros2_control_ws/install/controller_manager/lib:<existing child library path>
```

After the one-time source copy, the production workspace is self-contained. Root's normal `run.sh build` runs the same colcon arguments directly there and does not invoke or depend on scripts or an archive under `test_results`.

This selects the DSO used by the installed Gazebo plugin. Keep installed JTC, JSB, controller-interface, control-toolbox, and hardware dependencies. Do not launch the overlay's `ros2_control_node` in addition to the existing embedded manager. Root's chosen integration will resolve the selected CM path through `simulation/controller_runtime.py` and apply it only to the Gazebo child. `run.sh build` builds CM, while `serve` does not source the CM overlay globally or replace the installed spawners. The rebuilt library SHA may differ because debug/source paths differ; hash the actual output, not an expected cached DSO value.

Source snapshot and runtime provenance should include the new workspace's package source, patch manifest, version/license, compiler/dependency receipt, and actual loaded CM/interface/PID/JTC/JSB/Gazebo paths and SHA. Record `use_sim_time=true`, caller input clock typeROS, and actual synchronous `is_async=false` for JTC/JSB. The loaded process maps and native trigger observations must agree with the selected library. The original system CM remains available for comparison and rollback by removing the child's library-path prefix; every comparison uses a fresh world.

The controlled fixture proves that the candidate passes monotonic physical time and correct periods despite a paused ROS clock. It does not prevent publisher/bridge stalls, change sensor freshness, guarantee all controllers' async behavior, or support mid-run Gazebo world reset. Diagnostics that explicitly use ROS node time retain that time. Original failure runs and safety/arrival/collision contracts remain unchanged.
