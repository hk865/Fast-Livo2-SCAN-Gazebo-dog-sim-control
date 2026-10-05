# Portable local rebuild and experiment entry

This directory targets **Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic** on the same machine class as the recorded experiment. It does not promise cross-platform execution. Original experimental acceptance is historical evidence; it cannot authorize a newly built library.

The Frozen Teacher weights stay outside Git. Supply `model_1000.pt` with SHA256 `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`. The actor uses the supplied Python interpreter with PyTorch and NumPy on CPU; its original single-thread configuration, 50 Hz policy, 300 ms source deadlines, joint mapping and sole actuator remain unchanged. No hardware interface is provided by these tools.

Run from the repository root. A shell variable named `TEACHER_MODEL_FILE` below is an example for your external checkpoint location:

```bash
export TEACHER_MODEL_FILE=/absolute/path/to/model_1000.pt
export TEACHER_CPU_INTERPRETER=/absolute/path/to/python3
python3 -B tools/check_env.py --model "$TEACHER_MODEL_FILE" --cpu-python "$TEACHER_CPU_INTERPRETER"
python3 -B tools/prepare_offline.py --variant v18 \
  --profile combined_lio4_vio1_l64_r30_c30_600 \
  --output /var/tmp/go2_source_preview_new --model "$TEACHER_MODEL_FILE"
python3 -B tools/build_local.py
python3 -B tools/local_preflight.py
python3 -B tools/run_local.py --variant v18 \
  --profile combined_lio4_vio1_l64_r30_c30_600 \
  --model "$TEACHER_MODEL_FILE" --cpu-python "$TEACHER_CPU_INTERPRETER" \
  --label local_prepare --domain 86
```

The last command defaults to **prepare only**. It freezes a new experiment plan and reports `prepared_unverified`; it starts no ROS nodes, Gazebo or actor. An explicit `--execute` requests a real local simulation after the fresh finite gate. Use an unused ROS domain and ensure no other controller owns that simulation. `--storage-root /var/tmp/your_owned_private_directory` selects an owned, non-symlink, mode-0700 run root; the complete run is created there with a project alias. Existing runs and aliases are never overwritten. A full recording can consume several GiB.

`build_local.py` compiles the vendored Livox SDK/driver, private vikit/SCAN dependencies, native actuator and V12/V18/V17 sources. It uses at most two compiler jobs and a load limit of two. Project underlay/search variables are cleared before stock ROS and the private underlay are sourced. Its copied build inputs must exactly match the exported source, excluding documented generated libraries/object files and Python caches. A stale copy is rejected rather than silently compiled. Remove/recreate only the corresponding **new repository `.local` build directory** deliberately before retrying; these tools never delete previous workspace artifacts automatically.

The V19 source export adds the bounded staged input pipeline: one short receiver, dedicated cloud/image decoders, ordered owner commit, and context-valid normal drain. Its source can be rebuilt explicitly with `python3 -B tools/build_local.py --variants v19`; the build fixes LIO to four threads and VIO patch processing to one, matching V18. The default build list remains V12/V18/V17. This new repository's V19 build has **not been run yet**. `V19_SOURCE_EXPORT.json` records the exact copied source bytes, not a portable acceptance. `run_local.py --variant v19` refuses both prepare and execute with `PORTABLE_V19_RUNTIME_BLOCKED` until fresh portable queue lifecycle and production packet tests, numeric/loader checks and a new local runtime gate exist. The original machine's core receipt and loaded-library hashes cannot authorize a clone. Do not use the directly copied V19 runner to bypass this boundary.

Install system dependencies using the package inventory emitted by `check_env.py`: ROS Jazzy desktop, ros-gz, PCL ROS, cv_bridge, Sophus, Fast DDS RMW; colcon, SciPy/NumPy, YAML and Pillow; Eigen, OpenCV, PCL, fmt, Boost thread, OpenSSL and APR development packages. `PASS_ENV_ONLY` checks the specified OS, paths, model digest and Python module availability. The `dpkg-query` output is an inventory, not a complete dependency guarantee; the source build is the dependency test. Run `check_env.py` again when changing the model or interpreter.

`local_preflight.py` freshly compares V12 with V18 and V17 using the preserved synthetic estimator and patch fixtures. It verifies byte equality of state/covariance, component outputs and original diagnostics after removing only wall-time fields; it also observes floating-point controls, actual OpenMP teams and private library loading. This is a finite component gate, not a replay of the complete historical estimator trajectory or a navigation acceptance. V18 is explicitly an experimental LIO-4/VIO-1 candidate. V17 remains **`BLOCKED_QUEUE_SEMANTICS_REQUIRED`** in this portable export: source/assets, build and finite arithmetic checks are available, but its independent receive/decode/commit semantic gate has not been rerun by these tools, so `run_local.py` refuses V17 runtime.

All generated builds, fresh host-specific receipts, fixture outputs and new runs remain outside the source export (`.local/` and the ignored run/build directories). Missing or altered required source, binary or numeric evidence fails closed. Python `-O` / `PYTHONOPTIMIZE` is rejected at validation entry points. The historical JSON files remain readable and are not copied into a new local PASS.

See [PORTABILITY_CHANGES.md](PORTABILITY_CHANGES.md) and [PORTABILITY_VALIDATION.json](PORTABILITY_VALIDATION.json) for the exact tested scope and retained limitations. No new actual Gazebo navigation pass is claimed by the portable build checks.
