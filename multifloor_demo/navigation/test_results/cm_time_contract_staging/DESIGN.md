The candidate changes two expressions in the isolated ControllerManager4.45.2 source. With the existing `use_sim_time_` flag true, `update(time, period)` uses caller-supplied time for its current timestamp, last-cycle difference, and synchronous `trigger_update` timestamp. The false branch retains the original trigger-clock and node-clock expressions. Public headers, controller-rate gating, first-cycle period, gains, PID, interpolation, hardware state reads, effort handling, and safety remain unchanged. The supplied time must have the same ROS clock type as Gazebo's physical `_info.simTime`; it is not GT pose or a synthetic navigation correction.

Only this package is built against installed Jazzy dependencies under this excluded directory. No system package, project checkout, or production workspace is overwritten. The two public headers match the installed copies byte for byte;66 strong CM namespace functions remain exported, and dynamic relocation checking reports no missing symbols. This is concrete compatibility evidence, not a universal ABI certificate. Compiler-generated weak/template exports differ and their original broad comparison is retained separately.

The controlled fixture calls the actual CM shared library and installed synchronous ControllerInterface. Its controller has no command or state interfaces and records the actual callback arguments. A local ROS clock stays at16.456s while caller input time advances by4ms. The baseline delivers repeated time and zero period; the candidate delivers monotonic physical time and4ms period. The same executable also checks lower-rate gating under250Hz CM calls (requested100Hz, actually normalized by CM to83Hz), unchanged non-sim behavior, ROS clock reset with monotonic physical time, negative physical input propagation, and repeated caller input. Both libraries satisfy all12 corresponding checks. The first fixture attempt's generic convenience initializer used a SYSTEM-clock last-cycle value and failed before these checks; it is preserved. The final fixture initializes the clock type like the real `load_controller` path using `add_controller(spec)`; it does not invoke the plugin loader.

The candidate does not reject every zero period. Artificially repeating the physical caller timestamp still yields zero, as explicitly tested. In real Gazebo, a stopped physical simulation does not pass its update-period gate and thus does not call CM again. Mid-run physical world reset remains unsupported: the original Gazebo plugin's last physical update time is not reset by this change, and a direct backward input is propagated. The test probe rejects that negative period; this is not a new CM guard. Each physical comparison must spawn a fresh world. Asynchronous controllers and other ROS-based diagnostic headers are outside this contract.

Experimental loading is per Gazebo process: prepend `install_candidate/controller_manager/lib` to that process's `LD_LIBRARY_PATH`. Verify actual `/proc/<owned_gz_pid>/maps` library paths and SHA for CM, interface, PID, JTC, and Gazebo before interpreting results. Production remains on the installed library. The experiment must retain the native trigger observer for actual JTC time/period, actual reference/feedback/error/output, measured joint stream, clock/IMU transport timing, zero body commands, and all existing physical protection. No physical improvement is inferred from the controlled fixture or old-run replay.

Build used:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log_candidate build --base-paths candidate/controller_manager \
  --build-base build_candidate --install-base install_candidate \
  --packages-select controller_manager --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake -S fixture -B build_fixture \
  -DCMAKE_PREFIX_PATH="$PWD/install_candidate/controller_manager" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build_fixture --parallel 1
```

The fixture logs and selected actual library paths are in `controlled_fixture_v2_result.json`. Baseline and candidate must be selected through their separate process library environments; do not launch it in a domain concurrently occupied by another test.
