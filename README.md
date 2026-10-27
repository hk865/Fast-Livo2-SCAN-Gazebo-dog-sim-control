# Fast-LIVO2 / SCAN / Go2 navigation

This is the single current simulation project. Start from this directory:

```bash
python3 -B start.py --check
python3 -B start.py --prepare-only
python3 -B start.py
```

Edit `config/navigation.json`. `--check` checks files and basic configuration; it does not certify a complete navigation run. `--prepare-only` creates fresh inputs without starting ROS, Gazebo, or an Actor.

The runtime uses project-local `src/`, `models/`, `assets/`, and `.local/` native installations. Application C++ and SDK source is under `native/`; the DDS fix and rebuild tooling are under `third_party/fastdds/` and `build_support/private_dds/`. Runs are created only under `runs/`. Historical workspaces, successful-run directories, source/binary SHA lists, and historical approval contracts are not launch inputs.

The shared environment supplies Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, system C++/ROS libraries, and a Python installation containing PyTorch. The existing simulator Python is located using `ISAAC_SIM_PATH` (or the conventional `~/isaacsim` installation); Actor inference runs on CPU. These shared packages are not project cleanup targets.

Real protections remain: actual SLAM/cloud/IMU freshness, native response deadlines, resource limits and exclusive process locks, physical route geometry, dynamic stop/recovery, actual zero Actor input ACK, and immutable five-second parking using causal 10Hz position/yaw samples. See `docs/phase_summary.md` for scope and verification status.

The complete F1→F3 simulation regression passed on 2026-10-27, including dynamic stop/recovery, RGB saving, fixed five-second parking, CRC and process cleanup. See [the phase record](docs/phase_summary.md) and [measured results](docs/validation_summary.json).

Compiled installations and raw runs are local artifacts. [Restoration and dependencies](docs/restoration.md) describe the required layout and the boundary of what was tested. Third-party notices are in `LICENSES/`. One local visual asset, `L1_4D_Lidar.dae`, is retained locally and excluded from this new public snapshot because its redistribution source/license has not been established. A fresh clone needs that retained asset and compiled dependencies before running. The current local project is complete; Git source publication is not a tested binary distribution.
