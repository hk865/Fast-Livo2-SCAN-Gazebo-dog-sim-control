# Fast-LIVO2 / SCAN / Go2 navigation

One simulation project, one configuration, one normal entry:

```bash
python3 -B start.py --check
python3 -B start.py --prepare-only
python3 -B start.py
```

Edit `config/navigation.json`, including its `mission` goal settings. Generated files and logs go under `runs/`. Ctrl+C stops this entry's own processes and releases its three exclusive simulation locks.

The daily chain is configuration → current world/sensors → SLAM and map localization → SCAN → velocity controller → CPU Student Actor → Gazebo actuator. It does not import the experiment launcher, acceptance owner or verifier. Historical runs, approved versions, source/model/binary hash lists, frozen contracts, source-audit journals and test PASS files are not startup inputs. Ordinary command and status files exchange current control state; they are not historical approvals.

Actual protections remain: finite inputs and valid frames, causal IMU pairing, fresh SLAM/cloud/IMU and planner generations, obstacle and tilt stops, command limits, native response deadlines, and owned-process cleanup. `--resource-check` enables a developer CPU/RAM precheck.

Development tests are explicit and separate:

```bash
python3 -B -m unittest discover -s tests -v
python3 -B tools/validation/run_navigation_check.py --output /tmp/navigation-development-check
```

The second command explicitly runs the normal entry, moves the test obstacle, observes SLAM/control, and sends SIGINT to its own entry after measurement. It changes the test scene and lifecycle; normal startup does not depend on it. Its fixed five-second parking measurement uses the existing 10Hz position/yaw rule and the first actual zero-input qualification time; it is never called by normal startup. Use a new output directory for each test. See [phase results](docs/phase_summary.md).

Application and SDK sources are in `native/`, actuator sources in `simulation/`, and the two-sided DDS heartbeat fix in `third_party/fastdds/`. Current compiled installations are project-local under `.local/`; ROS 2 Jazzy, Gazebo Harmonic and the CPU-capable PyTorch environment are shared machine dependencies. They are not historical project directories. [Restoration details](docs/restoration.md) describe the build and distribution boundary.

This local installation includes the required model and meshes. The proposed public content excludes the Student weight binary and one retained local mesh, `L1_4D_Lidar.dae`, because redistribution rights have not been established. Compiled installations and raw runs are also excluded. A fresh source clone therefore needs those locally retained assets and a build; it is not a tested binary distribution. See [the model notice](models/MODEL_NOTICE.md). Third-party notices are in `LICENSES/`.
