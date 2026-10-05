# Portability changes and boundaries

The source/assets export retains its original relative project layout. `provenance/SOURCE_EXPORT_INITIAL.json` pins the bytes copied before these changes. Only the new repository was edited; the original workspace, accepted camera mode, frozen model and training task were left untouched.

Implemented changes:

- `portable_common.py` locates an external SHA-verified Frozen Teacher and repository-local simulation meshes. Unitree meshes use the exported description package; the D455 mesh and Apache-2.0 license are included under `tools/assets/`. Generated SDF/URDF URI fields are resolved within this repository.
- V18/V17 run, scope, launch and storage helpers use this repository's private `.local` underlay/SCAN build, relocated small historical acceptance evidence and an explicit owned storage root. The policy worker's checkpoint locator uses `TEACHER_REPO_ROOT`/`TEACHER_MODEL_CHECKPOINT`; its observation, network and command mathematics are unchanged.
- A fresh portable transport receipt binds the local ROS middleware libraries and the owned XML configuration. It does not inherit an old transport-delivery PASS.
- Livox SDK2/driver source is vendored with original commit/file provenance. `build_local.py` rebuilds it privately along with vikit, SCAN, the actuator and V12/V18/V17. It removes ambient project search paths, verifies actual copied inputs and pins each source/build artifact.
- `prepare_offline.py` performs source/assets-only generation without ROS, Gazebo or Torch. `local_preflight.py` builds new fixtures against private rebuilt libraries; no historical result is treated as a new binary's PASS. Runtime readers require complete mandatory source, binary, build, finite-output and FP/team/loader bindings and the exact frozen profile.

The first source-build attempt failed because `traj_utils` was initially placed before its `bspline_opt` dependency. The retry moves that source into the SCAN workspace so colcon orders the dependency correctly. The original failure logs and subsequent successful build logs remain in `.local/`. A subsequent source-copy provenance check identified vikit's CMake-generated `.so` inside its source tree; generated `.so/.o/.a/.pyc` outputs are now explicitly excluded from copied **source** comparisons, and privately installed libraries are independently hashed.

Validated on this local Ubuntu 24.04/Jazzy host: no-ROS source/assets generation, external model digest/module-spec checks, fresh private source builds, fresh finite arithmetic equivalence, FP/team/private-library observations, and a new V18 prepare-only plan. Exact numbers and hashes are in `PORTABILITY_VALIDATION.json`. Generated host-specific `.local` PASS receipts are not committed; another checkout must produce its own receipts.

Not validated by this export: another machine, complete historical estimator replay, new statistical throughput or performance, Gazebo motion/navigation under the newly rebuilt binaries, true stairs, or physical robot deployment. V17's new local queue semantic gate remains blocked. The historical LIO aggregation mismatch and small/medium performance regressions recorded in the experiment reports remain unchanged. Nothing here upgrades those failures.
