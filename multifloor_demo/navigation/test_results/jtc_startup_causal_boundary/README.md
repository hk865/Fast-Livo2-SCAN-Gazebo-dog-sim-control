This audit changes no production source, parameter, gain, command, or original result. The two original runs share exactly the same frozen 12-joint nominal target and all body velocity commands through17sim seconds are zero. Full16 actual body height begins dropping around16.0s; its largest adapter target wall gap starts at16.437s, after that change. The adapter log proves producer-side publication, not JTC consumption or applied torque. Run15 successfully initialized but later failed its independent region acceptance; it is not a successful complete demo.

Installed JTC4.40.1 recognizes the configured legacy integral clamp. Effort-only control uses actual joint position and velocity feedback. Zero-header position-only targets are interpolated from the current measured state over16.666666ms. Thus equal endpoint positions do not imply equal desired velocity. Previous command effort is also interpolated toward endpoint zero and added to the PID result. [Versioned JTC source](https://raw.githubusercontent.com/ros-controls/ros2_controllers/4.40.1/joint_trajectory_controller/src/joint_trajectory_controller.cpp), [trajectory source](https://raw.githubusercontent.com/ros-controls/ros2_controllers/4.40.1/joint_trajectory_controller/src/trajectory.cpp).

Gazebo1.2.19 supplies physical simulation time to ControllerManager, but installed CM4.45.2 reselects the ROS trigger clock for the synchronous controller call. If ROS clock delivery stalls while physical updates continue, the controller can receive repeated timestamps and zero periods. PID4.11.0 returns its previous command at zero period. [Gazebo plugin](https://raw.githubusercontent.com/ros-controls/gz_ros2_control/1.2.19/gz_ros2_control/src/gz_ros2_control_plugin.cpp), [CM source](https://raw.githubusercontent.com/ros-controls/ros2_control/4.45.2/controller_manager/src/controller_manager.cpp), [PID source](https://raw.githubusercontent.com/ros-controls/control_toolbox/4.11.0/control_toolbox/src/pid.cpp).

The separate passive startup probe records84 successive publications with the same16.456s JTC header. Independent forward replay initializes one integral state per joint once, then predicts997 actual output steps:914 positive-period and83 zero-period steps. Maximum residuals are3.55e−15 and8.88e−16 respectively. This supports the mechanism in this probe; it does not supply missing JTC records for the original full16 failure, establish the clock stall's transport cause, or prove that a controller change fixes the physical demo.

Reproduce without ROS, Gazebo, GT feedback, or command publication:

```bash
python3 multifloor_demo/navigation/test_results/jtc_startup_causal_boundary/audit_startup_targets.py
python3 multifloor_demo/navigation/test_results/jtc_startup_causal_boundary/replay_jtc_output.py
```

`startup_target_comparison.json` records exact producer arrays and bounded actual truth examples used only for evaluation. `jtc_output_forward_replay.json` records the passive input SHA and each predicted step. `causal_boundary_review.json` records cached official version-tag sources and the remaining uncertainty. The cached upstream files retain their original Apache license headers.
