# Installed JTC interpolation contract

This excluded fixture calls the installed Jazzy JointTrajectoryController 4.40.1 and ControllerInterface 4.45.2. It initializes actual effort command and position/velocity state interfaces for all twelve joints, activates the controller, delivers position-only trajectories through its actual ROS subscription, and invokes synchronous `trigger_update` with controlled 4 ms input periods. It does not run Gazebo or publish robot velocity, pose or ground truth.

The sole parameter difference is `interpolate_from_desired_state=false/true`. Both actual GetParameters services return BOOL, `open_loop_control=false`, and the original effort PID stays enabled. Parameters are copied from the frozen production `simulation/config/ros_control.yaml`; no gain or safety parameter changes. All 269 producer inputs in each trial have header zero and a 16,666,666 ns horizon. A new shared message pointer is required for every callback, including identical repeated nominal targets.

The final evidence is `actual_20261002_024858/result.json` (31 checks pass). Both processes exit zero with owned groups clean, all 269 production source hashes match before and after, and all 293 executed updates per trial return a 4 ms period. Activation precedes any update, so its returned-period marker is -1 and is not counted as an observed update.

Constant measured drift, nominal return, idle and rapid restart use identical prescribed measured position/velocity inputs in both trials. All producer targets, including the quintic .3 s return anchored on the producer's previous target, are identical. The intervening moving-target case uses commanded effort to advance a declared toy actuator state; those measured streams naturally differ between trials. The earlier `actual_20261002_024611` trial is retained as an output-dependent return illustration and is not the same-measured-input return proof.

The false branch constructs the new interpolation before-point from measured position/velocity and reads its effort from the previous command interface. The true branch uses the previous commanded-next reference/time. The current measured-state PID remains active in both. The independently reconstructed commanded effort equals reference effort feedforward plus the original PID within 9e-16. A commanded effort is not an independently measured applied torque.

In this controlled input, the repeated-target reference velocity peak changes from 2.700 to .552 rad/s and the stop-return peak from 3.119 to .108 rad/s. These observations do not establish Go2 stability, explain the missing first8 JTC trace around 193 s, or prove actual Adapter ACK/return behavior. The actual physics comparison must retain the original guards, regions and deadlines.

After coordinating exclusive ROS domain 77, the local fixture can be reproduced with:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S multifloor_demo/navigation/test_results/jtc_desired_native_contract \
  -B multifloor_demo/navigation/test_results/jtc_desired_native_contract/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build multifloor_demo/navigation/test_results/jtc_desired_native_contract/build -j2
python3 multifloor_demo/navigation/test_results/jtc_desired_native_contract/run_fixture.py
JTC_CONTRACT_RUN="$(cat multifloor_demo/navigation/test_results/jtc_desired_native_contract/latest_attempt.txt)"
cp multifloor_demo/navigation/test_results/jtc_desired_native_contract/{fixture.cpp,parameters.yaml,CMakeLists.txt,run_fixture.py} "$JTC_CONTRACT_RUN/"
cp multifloor_demo/navigation/test_results/jtc_desired_native_contract/build/jtc_desired_native_contract "$JTC_CONTRACT_RUN/"
python3 multifloor_demo/navigation/test_results/jtc_desired_native_contract/analyze.py
```

Production source/runtime stays frozen during the authorized physical A/B. Re-running this fixture is unnecessary for an unchanged binary and inputs.
