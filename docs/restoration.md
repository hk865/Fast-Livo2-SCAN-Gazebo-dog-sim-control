# Runtime and restoration

The local project has a complete working installation. Its sole entry is `start.py`, its editable configuration is `config/navigation.json`, and generated data is under `runs/`. Runtime does not need an old checkout, an approved commit, a source/binary hash list, or a historical successful-run directory.

## Retained local layout

| Material | Project location |
| --- | --- |
| Python navigation, control, Actor, parking and observation code | `src/` |
| Current Student45/GRU model | `models/student_model_4999_deterministic_jit.pt` |
| Route, scene geometry, robot and sensor meshes | `assets/` |
| FAST-LIVO2 application source | `native/slam/src/` |
| SCAN planner and its four support packages | `native/scan/src/` |
| Vikit and ROS message/package source | `native/sdk/src/` |
| Existing SLAM installation | `.local/runtime/native/ws/install/` |
| Existing SCAN installation | `.local/runtime/protocol_sender/protocol_sender/scan_ws/install/` |
| Local SDK installation | `.local/runtime/sdk/` |
| Patched DDS library | `.local/private_dds/install/lib/` |
| Native actuator source / existing library | `simulation/teacher_actuator.cpp` / `simulation/build/libteacher_actuator.so` |
| Actual DDS writer inspector source / existing helper | `src/r119_writer_qos.cpp` / `src/r119_writer_qos.so` |

The migrated native binaries are the existing working programs. They were not rebuilt or replaced by a different algorithm. Old build-directory names may remain in diagnostic strings inside a binary; those strings are not library lookup paths or files loaded at runtime. Four live process-map snapshots during the complete regression found project code/libraries only inside this project.

## Shared machine dependencies

Ubuntu 24.04 / Linux x86-64, ROS 2 Jazzy (including rclpy, rmw_fastrtps, ros_gz_bridge and message packages), Gazebo Harmonic, normal ROS/C++ shared libraries, and a CPU-capable PyTorch Python environment remain external. ROS development packages, Eigen, PCL, OpenCV, Boost, Sophus, fmt, OpenSSL and nlohmann_json are build dependencies. Livox driver development uses its upstream SDK. These are shared environment installations, not navigation-history cleanup candidates.

Actor Python is selected with `ISAAC_SIM_PATH` or the conventional `~/isaacsim` installation. Only its Python/PyTorch runtime is used for the CPU Actor; copying the full shared Isaac installation into this project is unnecessary. No authentication or private credentials are part of this project.

## Rebuilding boundary

Application CMake/package sources and the fixed DDS patch are included. A clean rebuild from this public snapshot has **not** been executed in this migration. Preserve the current local installations until a replacement has independently passed the same full regression. Rebuilding creates new binaries and needs fresh validation; successful relocation is not a claim that a clean compiler run reproduced them.

The expected ROS build order is SDK packages, FAST-LIVO2, then SCAN. Use separate build directories and these install destinations:

```text
native/sdk/src   -> .local/runtime/sdk
native/slam/src  -> .local/runtime/native/ws/install
native/scan/src  -> .local/runtime/protocol_sender/protocol_sender/scan_ws/install
```

Use `colcon build --base-paths <source>` with explicit `--build-base` and `--install-base`, source the appropriate ROS/SDK underlay, and keep the package-isolated SDK layout. The Livox package needs its ROS2 CMake option. The actuator links the Gazebo Harmonic development libraries; the writer inspector links the Jazzy rcl/rmw headers and the local patched DDS ABI. Do not substitute an old project's setup file as an underlay.

`build_support/private_dds/build_private_dds.py --help` describes the separate DDS build. Provide official local Fast DDS/Asio source archives identified in `third_party/fastdds/SOURCES.json`, choose fresh work/install directories, and execute the included heartbeat regression tests. Build input integrity checks are ordinary build reproducibility checks; they are not runtime source/version approval gates. The simulator must load the patched DDS at both communication ends.

## Public asset boundary

Unitree body/leg meshes match the official Unitree ROS mesh files, including the upstream `base.dae` saved here as `trunk.dae`. VLP16 meshes match Toyota Research Institute's velodyne_simulator; `d455.stl` matches Intel RealSense. Their notices are retained in `LICENSES/`.

The local `assets/meshes/L1_4D_Lidar.dae` (1,925,390 bytes) has no established public redistribution source/license in this review. It is protected locally and excluded from this new public snapshot. A fresh clone requires that retained local asset as well as compiled dependencies. No geometry placeholder is substituted for the validated local asset.

The current exported model has identical tensor/archive content after removing path-bearing debug metadata. A 128-frame recurrent replay produced bitwise-identical actions and hidden state; the complete regression used that export.

## Evidence and rollback

Git history and `phase_summary.md` retain decisions, fixes and outcomes. `validation_summary.json` contains concise measured results with private host paths omitted. Closed-bug raw diagnostics are proposed for deletion only after the exact cleanup manifest is approved. Reports cannot recreate deleted point clouds, bags or full failed runs.

The original checkouts are still untouched. Until cleanup is explicitly approved, rollback means using their existing entry/environment; no current project file must be moved or disabled to prove independence. After historical cleanup, normal Git restoration plus the protected local asset/install set is the retained recovery path. Earlier unresolved CPU-resource abort data is listed separately and is not described as a repaired bug.
