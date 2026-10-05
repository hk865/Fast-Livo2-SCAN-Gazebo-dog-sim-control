# 5 / 10 cm 持续上台阶重测

本次单项结果：**passed**。5/10 cm主测各3独立进程轮，另有2轮北侧影像补充；全部8轮保留，补录不替换主测。

| 高度 | 主测功能通过 | 四脚上台确认 s | 上台后继续 m | 实际world前进速度 m/s | 26–30秒停车漂移 m |
|---|---:|---:|---:|---:|---:|
| 5 cm | 3/3 | 9.055 | 2.906660 | 0.233850 | 0.000514 |
| 10 cm | 3/3 | 8.440 | 3.085850 | 0.237510 | 0.003651 |

八轮均为固定名义条件的确定性仿真；重复不代表随机地形/扰动鲁棒性。实际登台与后续运动用四脚接触点XYZ、法线、足端几何、持续实际进度和停车验证，旧world机身高度比未达不能等同于登不上。

冻结Teacher不重新训练，CPU推理。速度命令在3–22秒持续前进，随后闭环停车至30秒。实际相机帧、原始telemetry、原生接触收据与哈希逐轮保存。

测试平台前沿x=7 m、宽1.4 m及高度0.05/0.10 m保留；台面由原x=7..9延长至x=7..13，以让上台后继续行走而不在测试末尾下台。它是新fixture，不能当成原2 m台面相同条件。是低台阶登上台面，不是完整障碍跨越或真实楼梯。

本次功能复核使用预先冻结的足部上台支持、持续运动、安全与26–30秒停车判据。world body height gain仅为诊断。新增continue测试的summary.json由兼容旧评估器生成，旧时窗18–30秒含18–22秒前进、5cm_continue旧高度分支误按10cm，因此不适用于本轮能力判定；错误兼容记录原样保留。原42轮历史判据与summary另列，历史高度比未达不等于实际无法登台。原起点场景、Sim2Sim与导航结论保持原记录。

| 运行 | 分组 | 单项interface | 功能复核 | 兼容旧summary（不适用） | 冻结收据核对 | 实际画面数 |
|---|---|---|---|---|---|---:|
| 20261003_232320_step05_continue_functional_retest_r1_5ba8 | 主测 | passed | passed | failed | True | 261 |
| 20261003_232357_step10_continue_functional_retest_r1_2a2e | 主测 | passed | passed | failed | True | 260 |
| 20261003_232434_step05_continue_functional_retest_r2_d03c | 主测 | passed | passed | failed | True | 251 |
| 20261003_232511_step10_continue_functional_retest_r2_da05 | 主测 | passed | passed | failed | True | 254 |
| 20261003_232549_step05_continue_functional_retest_r3_9038 | 主测 | passed | passed | failed | True | 257 |
| 20261003_232626_step10_continue_functional_retest_r3_4512 | 主测 | passed | passed | failed | True | 268 |
| 20261003_233510_step05_continue_visible_retest_r1_028d | 影像补充 | passed | passed | failed | True | 256 |
| 20261003_233547_step10_continue_visible_retest_r1_7da7 | 影像补充 | passed | passed | failed | True | 242 |

## 真实影像的可见范围

首六轮南侧相机在越台前沿时被landmark_2遮挡，不能用这组影像宣称直接看到了cross。两轮额外北侧相机能清楚看到台前接近、越前沿及台上早段继续；21秒后前身部分离开左画幅，因此不声称完整30秒全身可见，完整进度与停车用实际遥测确认。

[逐图实际可见性检查](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/step_retest_visual_review_20261003.json)；[相机补录等价性](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/step_camera_supplement_equivalence.json)：两高度补录与相应主轮实际247观测、12动作最大差均为0，只改诊断相机视角。

## 执行源码与安全收据

主6轮runner实际缓存执行同一首版evaluator；中途磁盘文件变化使后5轮摘要里的evaluator SHA误指另版。原摘要和足部数组逐字节保留；追加首版独立复现、来源纠正，checks/metrics/protocol/全部数组精确一致。报告核对实际源码、原始/复现SHA及内容相等，不掩盖元数据错误。北侧2轮各自导入归档首版，SHA正确。

8轮追加200Hz安全审计覆盖0.1–30秒，每轮5981样本；最大roll/pitch 0.113963 rad，无机身接触、缺失接触或故障锁存；5cm最小支持间隙0.266428m、10cm 0.234574m。原功能摘要不被补充审计覆盖。


## 20261003_232320_step05_continue_functional_retest_r1_5ba8

功能逐项收据：

功能读取 `summary_functional.reproduced.json`；原摘要与纠正收据均保留。200Hz补充安全：**passed**。

- fixture: `{"passed": true, "front_x_m": 7.0, "back_x_m": 13.0, "tread_y_limits_m": [-1.4, 0.0], "top_z_m": 0.05, "collision": "teacher_low_step::link::collision"}`
- complete_continuous_runtime: `{"passed": true, "duration_s": 30.0, "policy_samples": 1501, "physics_samples": 6001, "native_termination": true, "worker_fault": null, "runner_error": null, "main_exit_codes": [0, 0]}`
- continuous_cpu_teacher: `{"passed": true, "target_action_max_error_rad": 0.0, "bootstrap_excluded_s": 0.1, "legacy_interface": "passed"}`
- command_profile: `{"passed": true, "forward_interval_s": [3, 22], "stop_interval_s": [26, 30], "forward_samples": 950, "stop_samples": 201}`
- FR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1", "radius_m": 0.02, "chain": ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"], "entry_time_s": 6.78, "entry_center_world_m": [7.004414659916565, -0.7382224671274388, 0.06952085463599822], "tread_contact_pair_samples": 4340, "verified_top_support_samples": 4329, "max_continuous_tread_support_s": 0.6749999999999989, "support_interval_s": [8.950000000000001, 9.625], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.6749999999999989, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 6.890000000000001, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- FL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1", "radius_m": 0.02, "chain": ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"], "entry_time_s": 7.025, "entry_center_world_m": [7.000115443496173, -0.6043869709543904, 0.07940989035505669], "tread_contact_pair_samples": 4429, "verified_top_support_samples": 4317, "max_continuous_tread_support_s": 0.7850000000000001, "support_interval_s": [8.58, 9.365], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7850000000000001, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 7.21, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1", "radius_m": 0.02, "chain": ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"], "entry_time_s": 8.505, "entry_center_world_m": [7.006919959721385, -0.8067367143830572, 0.0773448658101325], "tread_contact_pair_samples": 4183, "verified_top_support_samples": 4050, "max_continuous_tread_support_s": 0.7449999999999992, "support_interval_s": [8.63, 9.375], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7449999999999992, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.73, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1", "radius_m": 0.02, "chain": ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"], "entry_time_s": 8.790000000000001, "entry_center_world_m": [7.001789606600672, -0.5411518862106246, 0.07452337367240569], "tread_contact_pair_samples": 4118, "verified_top_support_samples": 3985, "max_continuous_tread_support_s": 1.004999999999999, "support_interval_s": [11.120000000000001, 12.125], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 1.004999999999999, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 9.055, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- four_feet_simultaneous_top_stop_support: `{"passed": true, "continuous_span_s": 4.0, "interval_s": [26.0, 30.0], "final_sample_all_four_on_top": true}`
- continued_teacher_forward_on_tread: `{"passed": true, "edge_plus_half_m_first_time_s": 9.57, "all_four_feet_active_ascent_confirmed_time_s": 9.055, "continued_x_progress_m": 2.906660280597184, "late_interval_s": [9.57, 22], "late_forward_velocity_mean_mps": 0.23385035797775625, "late_positive_velocity_fraction": 1.0, "on_platform_body_samples_fraction": 1.0, "continued_duration_s": 12.424999999999997, "rolling_velocity_windows": 2387, "rolling_window_s": 0.5, "maximum_no_foot_top_support_s": 0.0, "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"}`
- safety: `{"passed": true, "max_abs_roll_pitch_rad": 0.039924376876884285, "min_support_clearance_m": 0.2664275660576187, "body_contact_samples": 0, "unknown_contact_samples": 0, "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"}`
- teacher_zero_command_stop: `{"passed": true, "window_s": [26, 30], "translation_drift_m": 0.0005139875341777531, "yaw_drift_rad": 0.004163867110098141, "velocity_rms": [0.0003298905402229285, 0.00015423779773844176, 0.001164953219387489], "samples": 201, "strategy": "continuous Teacher at zero velocity command"}`

原始功能指标：

```json
{
  "continued_motion": {
    "edge_plus_half_m_first_time_s": 9.57,
    "all_four_feet_active_ascent_confirmed_time_s": 9.055,
    "continued_x_progress_m": 2.906660280597184,
    "late_interval_s": [
      9.57,
      22
    ],
    "late_forward_velocity_mean_mps": 0.23385035797775625,
    "late_positive_velocity_fraction": 1.0,
    "on_platform_body_samples_fraction": 1.0,
    "continued_duration_s": 12.424999999999997,
    "rolling_velocity_windows": 2387,
    "rolling_window_s": 0.5,
    "maximum_no_foot_top_support_s": 0.0,
    "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"
  },
  "safety": {
    "max_abs_roll_pitch_rad": 0.039924376876884285,
    "min_support_clearance_m": 0.2664275660576187,
    "body_contact_samples": 0,
    "unknown_contact_samples": 0,
    "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"
  },
  "stop": {
    "window_s": [
      26,
      30
    ],
    "translation_drift_m": 0.0005139875341777531,
    "yaw_drift_rad": 0.004163867110098141,
    "velocity_rms": [
      0.0003298905402229285,
      0.00015423779773844176,
      0.001164953219387489
    ],
    "samples": 201,
    "strategy": "continuous Teacher at zero velocity command"
  },
  "feet": {
    "FR": {
      "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rf_hip_joint",
        "rf_upper_leg_joint",
        "rf_lower_leg_joint"
      ],
      "entry_time_s": 6.78,
      "entry_center_world_m": [
        7.004414659916565,
        -0.7382224671274388,
        0.06952085463599822
      ],
      "tread_contact_pair_samples": 4340,
      "verified_top_support_samples": 4329,
      "max_continuous_tread_support_s": 0.6749999999999989,
      "support_interval_s": [
        8.950000000000001,
        9.625
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.6749999999999989,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 6.890000000000001,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "FL": {
      "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lf_hip_joint",
        "lf_upper_leg_joint",
        "lf_lower_leg_joint"
      ],
      "entry_time_s": 7.025,
      "entry_center_world_m": [
        7.000115443496173,
        -0.6043869709543904,
        0.07940989035505669
      ],
      "tread_contact_pair_samples": 4429,
      "verified_top_support_samples": 4317,
      "max_continuous_tread_support_s": 0.7850000000000001,
      "support_interval_s": [
        8.58,
        9.365
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7850000000000001,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 7.21,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RR": {
      "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rh_hip_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint"
      ],
      "entry_time_s": 8.505,
      "entry_center_world_m": [
        7.006919959721385,
        -0.8067367143830572,
        0.0773448658101325
      ],
      "tread_contact_pair_samples": 4183,
      "verified_top_support_samples": 4050,
      "max_continuous_tread_support_s": 0.7449999999999992,
      "support_interval_s": [
        8.63,
        9.375
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7449999999999992,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.73,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RL": {
      "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lh_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint"
      ],
      "entry_time_s": 8.790000000000001,
      "entry_center_world_m": [
        7.001789606600672,
        -0.5411518862106246,
        0.07452337367240569
      ],
      "tread_contact_pair_samples": 4118,
      "verified_top_support_samples": 3985,
      "max_continuous_tread_support_s": 1.004999999999999,
      "support_interval_s": [
        11.120000000000001,
        12.125
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 1.004999999999999,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 9.055,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    }
  },
  "body_world_height_gain_diagnostic_m": 0.023222602943720194,
  "body_world_height_gain_used_for_pass": false,
  "command_timing": {
    "duration_s": 30.0,
    "forward_end_s": 22,
    "stop_evaluation_start_s": 26
  }
}
```

新增continue轮兼容旧summary原记录（时窗/高度规则不适用，不据此判断能否登台）：

```json
[
  {
    "name": "step05_continue",
    "status": "failed",
    "reason": "Teacher stand/stop translation drift; Teacher stand/stop residual velocity; Stand/stop window contains nonzero velocity command; Low step tread was not reached with settled measured height gain",
    "metrics": {
      "duration_s": 30.0,
      "samples": 1501,
      "max_abs_roll_pitch_rad": 0.039924376876884285,
      "min_body_clearance_m": 0.2664275660576187,
      "body_contact_samples": 0,
      "unknown_contact_samples": 0,
      "max_applied_torque_Nm": 15.560171297267097,
      "position_delta_m": [
        4.434752001298786,
        0.5224141317262949,
        -0.06101994310748343
      ],
      "total_yaw_delta_rad": 0.28072649777593733,
      "assisted_phase_counts": {
        "support_capture": 0,
        "support_hold": 0
      },
      "tracking": [
        {
          "window_s": [
            5,
            10
          ],
          "command_mean": [
            0.2999999999999986,
            0.0,
            0.0
          ],
          "measured_mean": [
            0.24261920471407955,
            0.0024459310863514014,
            0.012240824085237856
          ],
          "rmse": [
            0.0708432601809192,
            0.04822173660672922,
            0.05448839916815815
          ]
        }
      ],
      "stop_teacher": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9157221750002436,
        "yaw_drift_rad": 0.057663338266871755,
        "velocity_rms": [
          0.13284170352079835,
          0.03111444570329373,
          0.020549059081416316
        ]
      },
      "stop_assisted": {
        "samples": 0
      },
      "stop": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9157221750002436,
        "yaw_drift_rad": 0.057663338266871755,
        "velocity_rms": [
          0.13284170352079835,
          0.03111444570329373,
          0.020549059081416316
        ]
      },
      "terrain": {
        "description": "Low-step ascent onto box tread; not complete obstacle traversal",
        "start_median_position_m": [
          6.001747980796979,
          -0.6969356765924537,
          0.315415261441494
        ],
        "end_median_position_m": [
          10.434649953994573,
          -0.17761997385525144,
          0.33876157997109835
        ],
        "height_gain_m": 0.023346318529604337,
        "native_end_support_foot_groups": [
          1,
          2,
          3,
          4
        ],
        "native_end_all_four_support_samples": 201,
        "native_end_samples": 201,
        "end_inside_tread": true,
        "step_height_m": 0.1,
        "minimum_height_gain_m": 0.06
      }
    }
  }
]
```

[实际MP4回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232320_step05_continue_functional_retest_r1_5ba8/frame_replay.mp4)

[实际GIF回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232320_step05_continue_functional_retest_r1_5ba8/frame_replay.gif)

![实际运动、接触、间隙与轨迹](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232320_step05_continue_functional_retest_r1_5ba8/step_functional.png)

逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232320_step05_continue_functional_retest_r1_5ba8/step_retest_evidence_manifest.json)。

回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。

## 20261003_232357_step10_continue_functional_retest_r1_2a2e

功能逐项收据：

功能读取 `summary_functional.reproduced.json`；原摘要与纠正收据均保留。200Hz补充安全：**passed**。

- fixture: `{"passed": true, "front_x_m": 7.0, "back_x_m": 13.0, "tread_y_limits_m": [-1.4, 0.0], "top_z_m": 0.1, "collision": "teacher_low_step::link::collision"}`
- complete_continuous_runtime: `{"passed": true, "duration_s": 30.0, "policy_samples": 1501, "physics_samples": 6001, "native_termination": true, "worker_fault": null, "runner_error": null, "main_exit_codes": [0, 0]}`
- continuous_cpu_teacher: `{"passed": true, "target_action_max_error_rad": 0.0, "bootstrap_excluded_s": 0.1, "legacy_interface": "passed"}`
- command_profile: `{"passed": true, "forward_interval_s": [3, 22], "stop_interval_s": [26, 30], "forward_samples": 950, "stop_samples": 201}`
- FR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1", "radius_m": 0.02, "chain": ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"], "entry_time_s": 6.765000000000001, "entry_center_world_m": [7.003484987502451, -0.7450499710442733, 0.12903616469988402], "tread_contact_pair_samples": 4338, "verified_top_support_samples": 4334, "max_continuous_tread_support_s": 0.6999999999999993, "support_interval_s": [20.09, 20.79], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.6999999999999993, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 6.92, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- FL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1", "radius_m": 0.02, "chain": ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"], "entry_time_s": 7.025, "entry_center_world_m": [7.007303448211745, -0.5935669941775383, 0.12922486244031764], "tread_contact_pair_samples": 4278, "verified_top_support_samples": 4271, "max_continuous_tread_support_s": 0.7300000000000004, "support_interval_s": [19.755, 20.485], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7300000000000004, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 7.29, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1", "radius_m": 0.02, "chain": ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"], "entry_time_s": 7.970000000000001, "entry_center_world_m": [7.005007280949083, -0.809137581179769, 0.14221377080706618], "tread_contact_pair_samples": 4078, "verified_top_support_samples": 4059, "max_continuous_tread_support_s": 0.7149999999999999, "support_interval_s": [19.75, 20.465], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7149999999999999, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.175, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1", "radius_m": 0.02, "chain": ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"], "entry_time_s": 8.21, "entry_center_world_m": [7.005604451037178, -0.5415882254946017, 0.13585076746649244], "tread_contact_pair_samples": 4073, "verified_top_support_samples": 4055, "max_continuous_tread_support_s": 0.634999999999998, "support_interval_s": [17.95, 18.584999999999997], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.634999999999998, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.44, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- four_feet_simultaneous_top_stop_support: `{"passed": true, "continuous_span_s": 4.0, "interval_s": [26.0, 30.0], "final_sample_all_four_on_top": true}`
- continued_teacher_forward_on_tread: `{"passed": true, "edge_plus_half_m_first_time_s": 9.005, "all_four_feet_active_ascent_confirmed_time_s": 8.44, "continued_x_progress_m": 3.085849879133712, "late_interval_s": [9.005, 22], "late_forward_velocity_mean_mps": 0.2375096390912059, "late_positive_velocity_fraction": 1.0, "on_platform_body_samples_fraction": 1.0, "continued_duration_s": 12.989999999999997, "rolling_velocity_windows": 2500, "rolling_window_s": 0.5, "maximum_no_foot_top_support_s": 0.0, "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"}`
- safety: `{"passed": true, "max_abs_roll_pitch_rad": 0.05935923857839813, "min_support_clearance_m": 0.23457368181977484, "body_contact_samples": 0, "unknown_contact_samples": 0, "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"}`
- teacher_zero_command_stop: `{"passed": true, "window_s": [26, 30], "translation_drift_m": 0.0036514907168625813, "yaw_drift_rad": 0.017750208646398362, "velocity_rms": [0.0011941515323118579, 0.0009719387709058877, 0.005857364830571079], "samples": 201, "strategy": "continuous Teacher at zero velocity command"}`

原始功能指标：

```json
{
  "continued_motion": {
    "edge_plus_half_m_first_time_s": 9.005,
    "all_four_feet_active_ascent_confirmed_time_s": 8.44,
    "continued_x_progress_m": 3.085849879133712,
    "late_interval_s": [
      9.005,
      22
    ],
    "late_forward_velocity_mean_mps": 0.2375096390912059,
    "late_positive_velocity_fraction": 1.0,
    "on_platform_body_samples_fraction": 1.0,
    "continued_duration_s": 12.989999999999997,
    "rolling_velocity_windows": 2500,
    "rolling_window_s": 0.5,
    "maximum_no_foot_top_support_s": 0.0,
    "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"
  },
  "safety": {
    "max_abs_roll_pitch_rad": 0.05935923857839813,
    "min_support_clearance_m": 0.23457368181977484,
    "body_contact_samples": 0,
    "unknown_contact_samples": 0,
    "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"
  },
  "stop": {
    "window_s": [
      26,
      30
    ],
    "translation_drift_m": 0.0036514907168625813,
    "yaw_drift_rad": 0.017750208646398362,
    "velocity_rms": [
      0.0011941515323118579,
      0.0009719387709058877,
      0.005857364830571079
    ],
    "samples": 201,
    "strategy": "continuous Teacher at zero velocity command"
  },
  "feet": {
    "FR": {
      "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rf_hip_joint",
        "rf_upper_leg_joint",
        "rf_lower_leg_joint"
      ],
      "entry_time_s": 6.765000000000001,
      "entry_center_world_m": [
        7.003484987502451,
        -0.7450499710442733,
        0.12903616469988402
      ],
      "tread_contact_pair_samples": 4338,
      "verified_top_support_samples": 4334,
      "max_continuous_tread_support_s": 0.6999999999999993,
      "support_interval_s": [
        20.09,
        20.79
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.6999999999999993,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 6.92,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "FL": {
      "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lf_hip_joint",
        "lf_upper_leg_joint",
        "lf_lower_leg_joint"
      ],
      "entry_time_s": 7.025,
      "entry_center_world_m": [
        7.007303448211745,
        -0.5935669941775383,
        0.12922486244031764
      ],
      "tread_contact_pair_samples": 4278,
      "verified_top_support_samples": 4271,
      "max_continuous_tread_support_s": 0.7300000000000004,
      "support_interval_s": [
        19.755,
        20.485
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7300000000000004,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 7.29,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RR": {
      "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rh_hip_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint"
      ],
      "entry_time_s": 7.970000000000001,
      "entry_center_world_m": [
        7.005007280949083,
        -0.809137581179769,
        0.14221377080706618
      ],
      "tread_contact_pair_samples": 4078,
      "verified_top_support_samples": 4059,
      "max_continuous_tread_support_s": 0.7149999999999999,
      "support_interval_s": [
        19.75,
        20.465
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7149999999999999,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.175,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RL": {
      "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lh_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint"
      ],
      "entry_time_s": 8.21,
      "entry_center_world_m": [
        7.005604451037178,
        -0.5415882254946017,
        0.13585076746649244
      ],
      "tread_contact_pair_samples": 4073,
      "verified_top_support_samples": 4055,
      "max_continuous_tread_support_s": 0.634999999999998,
      "support_interval_s": [
        17.95,
        18.584999999999997
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.634999999999998,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.44,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    }
  },
  "body_world_height_gain_diagnostic_m": 0.06489389797146,
  "body_world_height_gain_used_for_pass": false,
  "command_timing": {
    "duration_s": 30.0,
    "forward_end_s": 22,
    "stop_evaluation_start_s": 26
  }
}
```

新增continue轮兼容旧summary原记录（时窗/高度规则不适用，不据此判断能否登台）：

```json
[
  {
    "name": "step10_continue",
    "status": "failed",
    "reason": "Teacher stand/stop translation drift; Teacher stand/stop residual velocity; Stand/stop window contains nonzero velocity command",
    "metrics": {
      "duration_s": 30.0,
      "samples": 1501,
      "max_abs_roll_pitch_rad": 0.05935923857839813,
      "min_body_clearance_m": 0.23457368181977484,
      "body_contact_samples": 0,
      "unknown_contact_samples": 0,
      "max_applied_torque_Nm": 16.557874052988854,
      "position_delta_m": [
        4.600319170076096,
        0.45062194640502795,
        -0.019556714086178983
      ],
      "total_yaw_delta_rad": 0.06197269232487406,
      "assisted_phase_counts": {
        "support_capture": 0,
        "support_hold": 0
      },
      "tracking": [
        {
          "window_s": [
            5,
            10
          ],
          "command_mean": [
            0.2999999999999986,
            0.0,
            0.0
          ],
          "measured_mean": [
            0.2712102100487885,
            -0.006769432697175983,
            0.008260296603306225
          ],
          "rmse": [
            0.05108766742230146,
            0.03771411094896639,
            0.05894133545227834
          ]
        }
      ],
      "stop_teacher": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9100046238557103,
        "yaw_drift_rad": 0.10450069501188627,
        "velocity_rms": [
          0.1328191570785257,
          0.028512743291096966,
          0.030063657617788484
        ]
      },
      "stop_assisted": {
        "samples": 0
      },
      "stop": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9100046238557103,
        "yaw_drift_rad": 0.10450069501188627,
        "velocity_rms": [
          0.1328191570785257,
          0.028512743291096966,
          0.030063657617788484
        ]
      },
      "terrain": {
        "description": "Low-step ascent onto box tread; not complete obstacle traversal",
        "start_median_position_m": [
          6.001747980796979,
          -0.6969356765924537,
          0.315415261441494
        ],
        "end_median_position_m": [
          10.600284280274217,
          -0.24941706897288976,
          0.3799977573705121
        ],
        "height_gain_m": 0.0645824959290181,
        "native_end_support_foot_groups": [
          1,
          2,
          3,
          4
        ],
        "native_end_all_four_support_samples": 201,
        "native_end_samples": 201,
        "end_inside_tread": true,
        "step_height_m": 0.1,
        "minimum_height_gain_m": 0.06
      }
    }
  }
]
```

[实际MP4回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232357_step10_continue_functional_retest_r1_2a2e/frame_replay.mp4)

[实际GIF回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232357_step10_continue_functional_retest_r1_2a2e/frame_replay.gif)

![实际运动、接触、间隙与轨迹](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232357_step10_continue_functional_retest_r1_2a2e/step_functional.png)

逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232357_step10_continue_functional_retest_r1_2a2e/step_retest_evidence_manifest.json)。

回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。

## 20261003_232434_step05_continue_functional_retest_r2_d03c

功能逐项收据：

功能读取 `summary_functional.reproduced.json`；原摘要与纠正收据均保留。200Hz补充安全：**passed**。

- fixture: `{"passed": true, "front_x_m": 7.0, "back_x_m": 13.0, "tread_y_limits_m": [-1.4, 0.0], "top_z_m": 0.05, "collision": "teacher_low_step::link::collision"}`
- complete_continuous_runtime: `{"passed": true, "duration_s": 30.0, "policy_samples": 1501, "physics_samples": 6001, "native_termination": true, "worker_fault": null, "runner_error": null, "main_exit_codes": [0, 0]}`
- continuous_cpu_teacher: `{"passed": true, "target_action_max_error_rad": 0.0, "bootstrap_excluded_s": 0.1, "legacy_interface": "passed"}`
- command_profile: `{"passed": true, "forward_interval_s": [3, 22], "stop_interval_s": [26, 30], "forward_samples": 950, "stop_samples": 201}`
- FR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1", "radius_m": 0.02, "chain": ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"], "entry_time_s": 6.78, "entry_center_world_m": [7.004414659916565, -0.7382224671274388, 0.06952085463599822], "tread_contact_pair_samples": 4340, "verified_top_support_samples": 4329, "max_continuous_tread_support_s": 0.6749999999999989, "support_interval_s": [8.950000000000001, 9.625], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.6749999999999989, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 6.890000000000001, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- FL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1", "radius_m": 0.02, "chain": ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"], "entry_time_s": 7.025, "entry_center_world_m": [7.000115443496173, -0.6043869709543904, 0.07940989035505669], "tread_contact_pair_samples": 4429, "verified_top_support_samples": 4317, "max_continuous_tread_support_s": 0.7850000000000001, "support_interval_s": [8.58, 9.365], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7850000000000001, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 7.21, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1", "radius_m": 0.02, "chain": ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"], "entry_time_s": 8.505, "entry_center_world_m": [7.006919959721385, -0.8067367143830572, 0.0773448658101325], "tread_contact_pair_samples": 4183, "verified_top_support_samples": 4050, "max_continuous_tread_support_s": 0.7449999999999992, "support_interval_s": [8.63, 9.375], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7449999999999992, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.73, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1", "radius_m": 0.02, "chain": ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"], "entry_time_s": 8.790000000000001, "entry_center_world_m": [7.001789606600672, -0.5411518862106246, 0.07452337367240569], "tread_contact_pair_samples": 4118, "verified_top_support_samples": 3985, "max_continuous_tread_support_s": 1.004999999999999, "support_interval_s": [11.120000000000001, 12.125], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 1.004999999999999, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 9.055, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- four_feet_simultaneous_top_stop_support: `{"passed": true, "continuous_span_s": 4.0, "interval_s": [26.0, 30.0], "final_sample_all_four_on_top": true}`
- continued_teacher_forward_on_tread: `{"passed": true, "edge_plus_half_m_first_time_s": 9.57, "all_four_feet_active_ascent_confirmed_time_s": 9.055, "continued_x_progress_m": 2.906660280597184, "late_interval_s": [9.57, 22], "late_forward_velocity_mean_mps": 0.23385035797775625, "late_positive_velocity_fraction": 1.0, "on_platform_body_samples_fraction": 1.0, "continued_duration_s": 12.424999999999997, "rolling_velocity_windows": 2387, "rolling_window_s": 0.5, "maximum_no_foot_top_support_s": 0.0, "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"}`
- safety: `{"passed": true, "max_abs_roll_pitch_rad": 0.039924376876884285, "min_support_clearance_m": 0.2664275660576187, "body_contact_samples": 0, "unknown_contact_samples": 0, "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"}`
- teacher_zero_command_stop: `{"passed": true, "window_s": [26, 30], "translation_drift_m": 0.0005139875341777531, "yaw_drift_rad": 0.004163867110098141, "velocity_rms": [0.0003298905402229285, 0.00015423779773844176, 0.001164953219387489], "samples": 201, "strategy": "continuous Teacher at zero velocity command"}`

原始功能指标：

```json
{
  "continued_motion": {
    "edge_plus_half_m_first_time_s": 9.57,
    "all_four_feet_active_ascent_confirmed_time_s": 9.055,
    "continued_x_progress_m": 2.906660280597184,
    "late_interval_s": [
      9.57,
      22
    ],
    "late_forward_velocity_mean_mps": 0.23385035797775625,
    "late_positive_velocity_fraction": 1.0,
    "on_platform_body_samples_fraction": 1.0,
    "continued_duration_s": 12.424999999999997,
    "rolling_velocity_windows": 2387,
    "rolling_window_s": 0.5,
    "maximum_no_foot_top_support_s": 0.0,
    "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"
  },
  "safety": {
    "max_abs_roll_pitch_rad": 0.039924376876884285,
    "min_support_clearance_m": 0.2664275660576187,
    "body_contact_samples": 0,
    "unknown_contact_samples": 0,
    "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"
  },
  "stop": {
    "window_s": [
      26,
      30
    ],
    "translation_drift_m": 0.0005139875341777531,
    "yaw_drift_rad": 0.004163867110098141,
    "velocity_rms": [
      0.0003298905402229285,
      0.00015423779773844176,
      0.001164953219387489
    ],
    "samples": 201,
    "strategy": "continuous Teacher at zero velocity command"
  },
  "feet": {
    "FR": {
      "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rf_hip_joint",
        "rf_upper_leg_joint",
        "rf_lower_leg_joint"
      ],
      "entry_time_s": 6.78,
      "entry_center_world_m": [
        7.004414659916565,
        -0.7382224671274388,
        0.06952085463599822
      ],
      "tread_contact_pair_samples": 4340,
      "verified_top_support_samples": 4329,
      "max_continuous_tread_support_s": 0.6749999999999989,
      "support_interval_s": [
        8.950000000000001,
        9.625
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.6749999999999989,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 6.890000000000001,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "FL": {
      "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lf_hip_joint",
        "lf_upper_leg_joint",
        "lf_lower_leg_joint"
      ],
      "entry_time_s": 7.025,
      "entry_center_world_m": [
        7.000115443496173,
        -0.6043869709543904,
        0.07940989035505669
      ],
      "tread_contact_pair_samples": 4429,
      "verified_top_support_samples": 4317,
      "max_continuous_tread_support_s": 0.7850000000000001,
      "support_interval_s": [
        8.58,
        9.365
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7850000000000001,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 7.21,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RR": {
      "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rh_hip_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint"
      ],
      "entry_time_s": 8.505,
      "entry_center_world_m": [
        7.006919959721385,
        -0.8067367143830572,
        0.0773448658101325
      ],
      "tread_contact_pair_samples": 4183,
      "verified_top_support_samples": 4050,
      "max_continuous_tread_support_s": 0.7449999999999992,
      "support_interval_s": [
        8.63,
        9.375
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7449999999999992,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.73,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RL": {
      "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lh_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint"
      ],
      "entry_time_s": 8.790000000000001,
      "entry_center_world_m": [
        7.001789606600672,
        -0.5411518862106246,
        0.07452337367240569
      ],
      "tread_contact_pair_samples": 4118,
      "verified_top_support_samples": 3985,
      "max_continuous_tread_support_s": 1.004999999999999,
      "support_interval_s": [
        11.120000000000001,
        12.125
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 1.004999999999999,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 9.055,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    }
  },
  "body_world_height_gain_diagnostic_m": 0.023222602943720194,
  "body_world_height_gain_used_for_pass": false,
  "command_timing": {
    "duration_s": 30.0,
    "forward_end_s": 22,
    "stop_evaluation_start_s": 26
  }
}
```

新增continue轮兼容旧summary原记录（时窗/高度规则不适用，不据此判断能否登台）：

```json
[
  {
    "name": "step05_continue",
    "status": "failed",
    "reason": "Teacher stand/stop translation drift; Teacher stand/stop residual velocity; Stand/stop window contains nonzero velocity command; Low step tread was not reached with settled measured height gain",
    "metrics": {
      "duration_s": 30.0,
      "samples": 1501,
      "max_abs_roll_pitch_rad": 0.039924376876884285,
      "min_body_clearance_m": 0.2664275660576187,
      "body_contact_samples": 0,
      "unknown_contact_samples": 0,
      "max_applied_torque_Nm": 15.560171297267097,
      "position_delta_m": [
        4.434752001298786,
        0.5224141317262949,
        -0.06101994310748343
      ],
      "total_yaw_delta_rad": 0.28072649777593733,
      "assisted_phase_counts": {
        "support_capture": 0,
        "support_hold": 0
      },
      "tracking": [
        {
          "window_s": [
            5,
            10
          ],
          "command_mean": [
            0.2999999999999986,
            0.0,
            0.0
          ],
          "measured_mean": [
            0.24261920471407955,
            0.0024459310863514014,
            0.012240824085237856
          ],
          "rmse": [
            0.0708432601809192,
            0.04822173660672922,
            0.05448839916815815
          ]
        }
      ],
      "stop_teacher": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9157221750002436,
        "yaw_drift_rad": 0.057663338266871755,
        "velocity_rms": [
          0.13284170352079835,
          0.03111444570329373,
          0.020549059081416316
        ]
      },
      "stop_assisted": {
        "samples": 0
      },
      "stop": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9157221750002436,
        "yaw_drift_rad": 0.057663338266871755,
        "velocity_rms": [
          0.13284170352079835,
          0.03111444570329373,
          0.020549059081416316
        ]
      },
      "terrain": {
        "description": "Low-step ascent onto box tread; not complete obstacle traversal",
        "start_median_position_m": [
          6.001747980796979,
          -0.6969356765924537,
          0.315415261441494
        ],
        "end_median_position_m": [
          10.434649953994573,
          -0.17761997385525144,
          0.33876157997109835
        ],
        "height_gain_m": 0.023346318529604337,
        "native_end_support_foot_groups": [
          1,
          2,
          3,
          4
        ],
        "native_end_all_four_support_samples": 201,
        "native_end_samples": 201,
        "end_inside_tread": true,
        "step_height_m": 0.1,
        "minimum_height_gain_m": 0.06
      }
    }
  }
]
```

[实际MP4回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232434_step05_continue_functional_retest_r2_d03c/frame_replay.mp4)

[实际GIF回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232434_step05_continue_functional_retest_r2_d03c/frame_replay.gif)

![实际运动、接触、间隙与轨迹](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232434_step05_continue_functional_retest_r2_d03c/step_functional.png)

逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232434_step05_continue_functional_retest_r2_d03c/step_retest_evidence_manifest.json)。

回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。

## 20261003_232511_step10_continue_functional_retest_r2_da05

功能逐项收据：

功能读取 `summary_functional.reproduced.json`；原摘要与纠正收据均保留。200Hz补充安全：**passed**。

- fixture: `{"passed": true, "front_x_m": 7.0, "back_x_m": 13.0, "tread_y_limits_m": [-1.4, 0.0], "top_z_m": 0.1, "collision": "teacher_low_step::link::collision"}`
- complete_continuous_runtime: `{"passed": true, "duration_s": 30.0, "policy_samples": 1501, "physics_samples": 6001, "native_termination": true, "worker_fault": null, "runner_error": null, "main_exit_codes": [0, 0]}`
- continuous_cpu_teacher: `{"passed": true, "target_action_max_error_rad": 0.0, "bootstrap_excluded_s": 0.1, "legacy_interface": "passed"}`
- command_profile: `{"passed": true, "forward_interval_s": [3, 22], "stop_interval_s": [26, 30], "forward_samples": 950, "stop_samples": 201}`
- FR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1", "radius_m": 0.02, "chain": ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"], "entry_time_s": 6.765000000000001, "entry_center_world_m": [7.003484987502451, -0.7450499710442733, 0.12903616469988402], "tread_contact_pair_samples": 4338, "verified_top_support_samples": 4334, "max_continuous_tread_support_s": 0.6999999999999993, "support_interval_s": [20.09, 20.79], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.6999999999999993, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 6.92, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- FL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1", "radius_m": 0.02, "chain": ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"], "entry_time_s": 7.025, "entry_center_world_m": [7.007303448211745, -0.5935669941775383, 0.12922486244031764], "tread_contact_pair_samples": 4278, "verified_top_support_samples": 4271, "max_continuous_tread_support_s": 0.7300000000000004, "support_interval_s": [19.755, 20.485], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7300000000000004, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 7.29, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1", "radius_m": 0.02, "chain": ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"], "entry_time_s": 7.970000000000001, "entry_center_world_m": [7.005007280949083, -0.809137581179769, 0.14221377080706618], "tread_contact_pair_samples": 4078, "verified_top_support_samples": 4059, "max_continuous_tread_support_s": 0.7149999999999999, "support_interval_s": [19.75, 20.465], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7149999999999999, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.175, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1", "radius_m": 0.02, "chain": ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"], "entry_time_s": 8.21, "entry_center_world_m": [7.005604451037178, -0.5415882254946017, 0.13585076746649244], "tread_contact_pair_samples": 4073, "verified_top_support_samples": 4055, "max_continuous_tread_support_s": 0.634999999999998, "support_interval_s": [17.95, 18.584999999999997], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.634999999999998, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.44, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- four_feet_simultaneous_top_stop_support: `{"passed": true, "continuous_span_s": 4.0, "interval_s": [26.0, 30.0], "final_sample_all_four_on_top": true}`
- continued_teacher_forward_on_tread: `{"passed": true, "edge_plus_half_m_first_time_s": 9.005, "all_four_feet_active_ascent_confirmed_time_s": 8.44, "continued_x_progress_m": 3.085849879133712, "late_interval_s": [9.005, 22], "late_forward_velocity_mean_mps": 0.2375096390912059, "late_positive_velocity_fraction": 1.0, "on_platform_body_samples_fraction": 1.0, "continued_duration_s": 12.989999999999997, "rolling_velocity_windows": 2500, "rolling_window_s": 0.5, "maximum_no_foot_top_support_s": 0.0, "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"}`
- safety: `{"passed": true, "max_abs_roll_pitch_rad": 0.05935923857839813, "min_support_clearance_m": 0.23457368181977484, "body_contact_samples": 0, "unknown_contact_samples": 0, "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"}`
- teacher_zero_command_stop: `{"passed": true, "window_s": [26, 30], "translation_drift_m": 0.0036514907168625813, "yaw_drift_rad": 0.017750208646398362, "velocity_rms": [0.0011941515323118579, 0.0009719387709058877, 0.005857364830571079], "samples": 201, "strategy": "continuous Teacher at zero velocity command"}`

原始功能指标：

```json
{
  "continued_motion": {
    "edge_plus_half_m_first_time_s": 9.005,
    "all_four_feet_active_ascent_confirmed_time_s": 8.44,
    "continued_x_progress_m": 3.085849879133712,
    "late_interval_s": [
      9.005,
      22
    ],
    "late_forward_velocity_mean_mps": 0.2375096390912059,
    "late_positive_velocity_fraction": 1.0,
    "on_platform_body_samples_fraction": 1.0,
    "continued_duration_s": 12.989999999999997,
    "rolling_velocity_windows": 2500,
    "rolling_window_s": 0.5,
    "maximum_no_foot_top_support_s": 0.0,
    "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"
  },
  "safety": {
    "max_abs_roll_pitch_rad": 0.05935923857839813,
    "min_support_clearance_m": 0.23457368181977484,
    "body_contact_samples": 0,
    "unknown_contact_samples": 0,
    "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"
  },
  "stop": {
    "window_s": [
      26,
      30
    ],
    "translation_drift_m": 0.0036514907168625813,
    "yaw_drift_rad": 0.017750208646398362,
    "velocity_rms": [
      0.0011941515323118579,
      0.0009719387709058877,
      0.005857364830571079
    ],
    "samples": 201,
    "strategy": "continuous Teacher at zero velocity command"
  },
  "feet": {
    "FR": {
      "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rf_hip_joint",
        "rf_upper_leg_joint",
        "rf_lower_leg_joint"
      ],
      "entry_time_s": 6.765000000000001,
      "entry_center_world_m": [
        7.003484987502451,
        -0.7450499710442733,
        0.12903616469988402
      ],
      "tread_contact_pair_samples": 4338,
      "verified_top_support_samples": 4334,
      "max_continuous_tread_support_s": 0.6999999999999993,
      "support_interval_s": [
        20.09,
        20.79
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.6999999999999993,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 6.92,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "FL": {
      "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lf_hip_joint",
        "lf_upper_leg_joint",
        "lf_lower_leg_joint"
      ],
      "entry_time_s": 7.025,
      "entry_center_world_m": [
        7.007303448211745,
        -0.5935669941775383,
        0.12922486244031764
      ],
      "tread_contact_pair_samples": 4278,
      "verified_top_support_samples": 4271,
      "max_continuous_tread_support_s": 0.7300000000000004,
      "support_interval_s": [
        19.755,
        20.485
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7300000000000004,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 7.29,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RR": {
      "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rh_hip_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint"
      ],
      "entry_time_s": 7.970000000000001,
      "entry_center_world_m": [
        7.005007280949083,
        -0.809137581179769,
        0.14221377080706618
      ],
      "tread_contact_pair_samples": 4078,
      "verified_top_support_samples": 4059,
      "max_continuous_tread_support_s": 0.7149999999999999,
      "support_interval_s": [
        19.75,
        20.465
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7149999999999999,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.175,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RL": {
      "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lh_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint"
      ],
      "entry_time_s": 8.21,
      "entry_center_world_m": [
        7.005604451037178,
        -0.5415882254946017,
        0.13585076746649244
      ],
      "tread_contact_pair_samples": 4073,
      "verified_top_support_samples": 4055,
      "max_continuous_tread_support_s": 0.634999999999998,
      "support_interval_s": [
        17.95,
        18.584999999999997
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.634999999999998,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.44,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    }
  },
  "body_world_height_gain_diagnostic_m": 0.06489389797146,
  "body_world_height_gain_used_for_pass": false,
  "command_timing": {
    "duration_s": 30.0,
    "forward_end_s": 22,
    "stop_evaluation_start_s": 26
  }
}
```

新增continue轮兼容旧summary原记录（时窗/高度规则不适用，不据此判断能否登台）：

```json
[
  {
    "name": "step10_continue",
    "status": "failed",
    "reason": "Teacher stand/stop translation drift; Teacher stand/stop residual velocity; Stand/stop window contains nonzero velocity command",
    "metrics": {
      "duration_s": 30.0,
      "samples": 1501,
      "max_abs_roll_pitch_rad": 0.05935923857839813,
      "min_body_clearance_m": 0.23457368181977484,
      "body_contact_samples": 0,
      "unknown_contact_samples": 0,
      "max_applied_torque_Nm": 16.557874052988854,
      "position_delta_m": [
        4.600319170076096,
        0.45062194640502795,
        -0.019556714086178983
      ],
      "total_yaw_delta_rad": 0.06197269232487406,
      "assisted_phase_counts": {
        "support_capture": 0,
        "support_hold": 0
      },
      "tracking": [
        {
          "window_s": [
            5,
            10
          ],
          "command_mean": [
            0.2999999999999986,
            0.0,
            0.0
          ],
          "measured_mean": [
            0.2712102100487885,
            -0.006769432697175983,
            0.008260296603306225
          ],
          "rmse": [
            0.05108766742230146,
            0.03771411094896639,
            0.05894133545227834
          ]
        }
      ],
      "stop_teacher": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9100046238557103,
        "yaw_drift_rad": 0.10450069501188627,
        "velocity_rms": [
          0.1328191570785257,
          0.028512743291096966,
          0.030063657617788484
        ]
      },
      "stop_assisted": {
        "samples": 0
      },
      "stop": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9100046238557103,
        "yaw_drift_rad": 0.10450069501188627,
        "velocity_rms": [
          0.1328191570785257,
          0.028512743291096966,
          0.030063657617788484
        ]
      },
      "terrain": {
        "description": "Low-step ascent onto box tread; not complete obstacle traversal",
        "start_median_position_m": [
          6.001747980796979,
          -0.6969356765924537,
          0.315415261441494
        ],
        "end_median_position_m": [
          10.600284280274217,
          -0.24941706897288976,
          0.3799977573705121
        ],
        "height_gain_m": 0.0645824959290181,
        "native_end_support_foot_groups": [
          1,
          2,
          3,
          4
        ],
        "native_end_all_four_support_samples": 201,
        "native_end_samples": 201,
        "end_inside_tread": true,
        "step_height_m": 0.1,
        "minimum_height_gain_m": 0.06
      }
    }
  }
]
```

[实际MP4回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232511_step10_continue_functional_retest_r2_da05/frame_replay.mp4)

[实际GIF回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232511_step10_continue_functional_retest_r2_da05/frame_replay.gif)

![实际运动、接触、间隙与轨迹](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232511_step10_continue_functional_retest_r2_da05/step_functional.png)

逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232511_step10_continue_functional_retest_r2_da05/step_retest_evidence_manifest.json)。

回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。

## 20261003_232549_step05_continue_functional_retest_r3_9038

功能逐项收据：

功能读取 `summary_functional.reproduced.json`；原摘要与纠正收据均保留。200Hz补充安全：**passed**。

- fixture: `{"passed": true, "front_x_m": 7.0, "back_x_m": 13.0, "tread_y_limits_m": [-1.4, 0.0], "top_z_m": 0.05, "collision": "teacher_low_step::link::collision"}`
- complete_continuous_runtime: `{"passed": true, "duration_s": 30.0, "policy_samples": 1501, "physics_samples": 6001, "native_termination": true, "worker_fault": null, "runner_error": null, "main_exit_codes": [0, 0]}`
- continuous_cpu_teacher: `{"passed": true, "target_action_max_error_rad": 0.0, "bootstrap_excluded_s": 0.1, "legacy_interface": "passed"}`
- command_profile: `{"passed": true, "forward_interval_s": [3, 22], "stop_interval_s": [26, 30], "forward_samples": 950, "stop_samples": 201}`
- FR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1", "radius_m": 0.02, "chain": ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"], "entry_time_s": 6.78, "entry_center_world_m": [7.004414659916565, -0.7382224671274388, 0.06952085463599822], "tread_contact_pair_samples": 4340, "verified_top_support_samples": 4329, "max_continuous_tread_support_s": 0.6749999999999989, "support_interval_s": [8.950000000000001, 9.625], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.6749999999999989, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 6.890000000000001, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- FL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1", "radius_m": 0.02, "chain": ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"], "entry_time_s": 7.025, "entry_center_world_m": [7.000115443496173, -0.6043869709543904, 0.07940989035505669], "tread_contact_pair_samples": 4429, "verified_top_support_samples": 4317, "max_continuous_tread_support_s": 0.7850000000000001, "support_interval_s": [8.58, 9.365], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7850000000000001, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 7.21, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1", "radius_m": 0.02, "chain": ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"], "entry_time_s": 8.505, "entry_center_world_m": [7.006919959721385, -0.8067367143830572, 0.0773448658101325], "tread_contact_pair_samples": 4183, "verified_top_support_samples": 4050, "max_continuous_tread_support_s": 0.7449999999999992, "support_interval_s": [8.63, 9.375], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7449999999999992, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.73, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1", "radius_m": 0.02, "chain": ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"], "entry_time_s": 8.790000000000001, "entry_center_world_m": [7.001789606600672, -0.5411518862106246, 0.07452337367240569], "tread_contact_pair_samples": 4118, "verified_top_support_samples": 3985, "max_continuous_tread_support_s": 1.004999999999999, "support_interval_s": [11.120000000000001, 12.125], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 1.004999999999999, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 9.055, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- four_feet_simultaneous_top_stop_support: `{"passed": true, "continuous_span_s": 4.0, "interval_s": [26.0, 30.0], "final_sample_all_four_on_top": true}`
- continued_teacher_forward_on_tread: `{"passed": true, "edge_plus_half_m_first_time_s": 9.57, "all_four_feet_active_ascent_confirmed_time_s": 9.055, "continued_x_progress_m": 2.906660280597184, "late_interval_s": [9.57, 22], "late_forward_velocity_mean_mps": 0.23385035797775625, "late_positive_velocity_fraction": 1.0, "on_platform_body_samples_fraction": 1.0, "continued_duration_s": 12.424999999999997, "rolling_velocity_windows": 2387, "rolling_window_s": 0.5, "maximum_no_foot_top_support_s": 0.0, "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"}`
- safety: `{"passed": true, "max_abs_roll_pitch_rad": 0.039924376876884285, "min_support_clearance_m": 0.2664275660576187, "body_contact_samples": 0, "unknown_contact_samples": 0, "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"}`
- teacher_zero_command_stop: `{"passed": true, "window_s": [26, 30], "translation_drift_m": 0.0005139875341777531, "yaw_drift_rad": 0.004163867110098141, "velocity_rms": [0.0003298905402229285, 0.00015423779773844176, 0.001164953219387489], "samples": 201, "strategy": "continuous Teacher at zero velocity command"}`

原始功能指标：

```json
{
  "continued_motion": {
    "edge_plus_half_m_first_time_s": 9.57,
    "all_four_feet_active_ascent_confirmed_time_s": 9.055,
    "continued_x_progress_m": 2.906660280597184,
    "late_interval_s": [
      9.57,
      22
    ],
    "late_forward_velocity_mean_mps": 0.23385035797775625,
    "late_positive_velocity_fraction": 1.0,
    "on_platform_body_samples_fraction": 1.0,
    "continued_duration_s": 12.424999999999997,
    "rolling_velocity_windows": 2387,
    "rolling_window_s": 0.5,
    "maximum_no_foot_top_support_s": 0.0,
    "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"
  },
  "safety": {
    "max_abs_roll_pitch_rad": 0.039924376876884285,
    "min_support_clearance_m": 0.2664275660576187,
    "body_contact_samples": 0,
    "unknown_contact_samples": 0,
    "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"
  },
  "stop": {
    "window_s": [
      26,
      30
    ],
    "translation_drift_m": 0.0005139875341777531,
    "yaw_drift_rad": 0.004163867110098141,
    "velocity_rms": [
      0.0003298905402229285,
      0.00015423779773844176,
      0.001164953219387489
    ],
    "samples": 201,
    "strategy": "continuous Teacher at zero velocity command"
  },
  "feet": {
    "FR": {
      "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rf_hip_joint",
        "rf_upper_leg_joint",
        "rf_lower_leg_joint"
      ],
      "entry_time_s": 6.78,
      "entry_center_world_m": [
        7.004414659916565,
        -0.7382224671274388,
        0.06952085463599822
      ],
      "tread_contact_pair_samples": 4340,
      "verified_top_support_samples": 4329,
      "max_continuous_tread_support_s": 0.6749999999999989,
      "support_interval_s": [
        8.950000000000001,
        9.625
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.6749999999999989,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 6.890000000000001,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "FL": {
      "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lf_hip_joint",
        "lf_upper_leg_joint",
        "lf_lower_leg_joint"
      ],
      "entry_time_s": 7.025,
      "entry_center_world_m": [
        7.000115443496173,
        -0.6043869709543904,
        0.07940989035505669
      ],
      "tread_contact_pair_samples": 4429,
      "verified_top_support_samples": 4317,
      "max_continuous_tread_support_s": 0.7850000000000001,
      "support_interval_s": [
        8.58,
        9.365
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7850000000000001,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 7.21,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RR": {
      "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rh_hip_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint"
      ],
      "entry_time_s": 8.505,
      "entry_center_world_m": [
        7.006919959721385,
        -0.8067367143830572,
        0.0773448658101325
      ],
      "tread_contact_pair_samples": 4183,
      "verified_top_support_samples": 4050,
      "max_continuous_tread_support_s": 0.7449999999999992,
      "support_interval_s": [
        8.63,
        9.375
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7449999999999992,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.73,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RL": {
      "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lh_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint"
      ],
      "entry_time_s": 8.790000000000001,
      "entry_center_world_m": [
        7.001789606600672,
        -0.5411518862106246,
        0.07452337367240569
      ],
      "tread_contact_pair_samples": 4118,
      "verified_top_support_samples": 3985,
      "max_continuous_tread_support_s": 1.004999999999999,
      "support_interval_s": [
        11.120000000000001,
        12.125
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 1.004999999999999,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 9.055,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    }
  },
  "body_world_height_gain_diagnostic_m": 0.023222602943720194,
  "body_world_height_gain_used_for_pass": false,
  "command_timing": {
    "duration_s": 30.0,
    "forward_end_s": 22,
    "stop_evaluation_start_s": 26
  }
}
```

新增continue轮兼容旧summary原记录（时窗/高度规则不适用，不据此判断能否登台）：

```json
[
  {
    "name": "step05_continue",
    "status": "failed",
    "reason": "Teacher stand/stop translation drift; Teacher stand/stop residual velocity; Stand/stop window contains nonzero velocity command; Low step tread was not reached with settled measured height gain",
    "metrics": {
      "duration_s": 30.0,
      "samples": 1501,
      "max_abs_roll_pitch_rad": 0.039924376876884285,
      "min_body_clearance_m": 0.2664275660576187,
      "body_contact_samples": 0,
      "unknown_contact_samples": 0,
      "max_applied_torque_Nm": 15.560171297267097,
      "position_delta_m": [
        4.434752001298786,
        0.5224141317262949,
        -0.06101994310748343
      ],
      "total_yaw_delta_rad": 0.28072649777593733,
      "assisted_phase_counts": {
        "support_capture": 0,
        "support_hold": 0
      },
      "tracking": [
        {
          "window_s": [
            5,
            10
          ],
          "command_mean": [
            0.2999999999999986,
            0.0,
            0.0
          ],
          "measured_mean": [
            0.24261920471407955,
            0.0024459310863514014,
            0.012240824085237856
          ],
          "rmse": [
            0.0708432601809192,
            0.04822173660672922,
            0.05448839916815815
          ]
        }
      ],
      "stop_teacher": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9157221750002436,
        "yaw_drift_rad": 0.057663338266871755,
        "velocity_rms": [
          0.13284170352079835,
          0.03111444570329373,
          0.020549059081416316
        ]
      },
      "stop_assisted": {
        "samples": 0
      },
      "stop": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9157221750002436,
        "yaw_drift_rad": 0.057663338266871755,
        "velocity_rms": [
          0.13284170352079835,
          0.03111444570329373,
          0.020549059081416316
        ]
      },
      "terrain": {
        "description": "Low-step ascent onto box tread; not complete obstacle traversal",
        "start_median_position_m": [
          6.001747980796979,
          -0.6969356765924537,
          0.315415261441494
        ],
        "end_median_position_m": [
          10.434649953994573,
          -0.17761997385525144,
          0.33876157997109835
        ],
        "height_gain_m": 0.023346318529604337,
        "native_end_support_foot_groups": [
          1,
          2,
          3,
          4
        ],
        "native_end_all_four_support_samples": 201,
        "native_end_samples": 201,
        "end_inside_tread": true,
        "step_height_m": 0.1,
        "minimum_height_gain_m": 0.06
      }
    }
  }
]
```

[实际MP4回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232549_step05_continue_functional_retest_r3_9038/frame_replay.mp4)

[实际GIF回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232549_step05_continue_functional_retest_r3_9038/frame_replay.gif)

![实际运动、接触、间隙与轨迹](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232549_step05_continue_functional_retest_r3_9038/step_functional.png)

逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232549_step05_continue_functional_retest_r3_9038/step_retest_evidence_manifest.json)。

回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。

## 20261003_232626_step10_continue_functional_retest_r3_4512

功能逐项收据：

功能读取 `summary_functional.reproduced.json`；原摘要与纠正收据均保留。200Hz补充安全：**passed**。

- fixture: `{"passed": true, "front_x_m": 7.0, "back_x_m": 13.0, "tread_y_limits_m": [-1.4, 0.0], "top_z_m": 0.1, "collision": "teacher_low_step::link::collision"}`
- complete_continuous_runtime: `{"passed": true, "duration_s": 30.0, "policy_samples": 1501, "physics_samples": 6001, "native_termination": true, "worker_fault": null, "runner_error": null, "main_exit_codes": [0, 0]}`
- continuous_cpu_teacher: `{"passed": true, "target_action_max_error_rad": 0.0, "bootstrap_excluded_s": 0.1, "legacy_interface": "passed"}`
- command_profile: `{"passed": true, "forward_interval_s": [3, 22], "stop_interval_s": [26, 30], "forward_samples": 950, "stop_samples": 201}`
- FR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1", "radius_m": 0.02, "chain": ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"], "entry_time_s": 6.765000000000001, "entry_center_world_m": [7.003484987502451, -0.7450499710442733, 0.12903616469988402], "tread_contact_pair_samples": 4338, "verified_top_support_samples": 4334, "max_continuous_tread_support_s": 0.6999999999999993, "support_interval_s": [20.09, 20.79], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.6999999999999993, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 6.92, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- FL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1", "radius_m": 0.02, "chain": ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"], "entry_time_s": 7.025, "entry_center_world_m": [7.007303448211745, -0.5935669941775383, 0.12922486244031764], "tread_contact_pair_samples": 4278, "verified_top_support_samples": 4271, "max_continuous_tread_support_s": 0.7300000000000004, "support_interval_s": [19.755, 20.485], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7300000000000004, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 7.29, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1", "radius_m": 0.02, "chain": ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"], "entry_time_s": 7.970000000000001, "entry_center_world_m": [7.005007280949083, -0.809137581179769, 0.14221377080706618], "tread_contact_pair_samples": 4078, "verified_top_support_samples": 4059, "max_continuous_tread_support_s": 0.7149999999999999, "support_interval_s": [19.75, 20.465], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7149999999999999, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.175, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1", "radius_m": 0.02, "chain": ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"], "entry_time_s": 8.21, "entry_center_world_m": [7.005604451037178, -0.5415882254946017, 0.13585076746649244], "tread_contact_pair_samples": 4073, "verified_top_support_samples": 4055, "max_continuous_tread_support_s": 0.634999999999998, "support_interval_s": [17.95, 18.584999999999997], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.634999999999998, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.44, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- four_feet_simultaneous_top_stop_support: `{"passed": true, "continuous_span_s": 4.0, "interval_s": [26.0, 30.0], "final_sample_all_four_on_top": true}`
- continued_teacher_forward_on_tread: `{"passed": true, "edge_plus_half_m_first_time_s": 9.005, "all_four_feet_active_ascent_confirmed_time_s": 8.44, "continued_x_progress_m": 3.085849879133712, "late_interval_s": [9.005, 22], "late_forward_velocity_mean_mps": 0.2375096390912059, "late_positive_velocity_fraction": 1.0, "on_platform_body_samples_fraction": 1.0, "continued_duration_s": 12.989999999999997, "rolling_velocity_windows": 2500, "rolling_window_s": 0.5, "maximum_no_foot_top_support_s": 0.0, "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"}`
- safety: `{"passed": true, "max_abs_roll_pitch_rad": 0.05935923857839813, "min_support_clearance_m": 0.23457368181977484, "body_contact_samples": 0, "unknown_contact_samples": 0, "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"}`
- teacher_zero_command_stop: `{"passed": true, "window_s": [26, 30], "translation_drift_m": 0.0036514907168625813, "yaw_drift_rad": 0.017750208646398362, "velocity_rms": [0.0011941515323118579, 0.0009719387709058877, 0.005857364830571079], "samples": 201, "strategy": "continuous Teacher at zero velocity command"}`

原始功能指标：

```json
{
  "continued_motion": {
    "edge_plus_half_m_first_time_s": 9.005,
    "all_four_feet_active_ascent_confirmed_time_s": 8.44,
    "continued_x_progress_m": 3.085849879133712,
    "late_interval_s": [
      9.005,
      22
    ],
    "late_forward_velocity_mean_mps": 0.2375096390912059,
    "late_positive_velocity_fraction": 1.0,
    "on_platform_body_samples_fraction": 1.0,
    "continued_duration_s": 12.989999999999997,
    "rolling_velocity_windows": 2500,
    "rolling_window_s": 0.5,
    "maximum_no_foot_top_support_s": 0.0,
    "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"
  },
  "safety": {
    "max_abs_roll_pitch_rad": 0.05935923857839813,
    "min_support_clearance_m": 0.23457368181977484,
    "body_contact_samples": 0,
    "unknown_contact_samples": 0,
    "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"
  },
  "stop": {
    "window_s": [
      26,
      30
    ],
    "translation_drift_m": 0.0036514907168625813,
    "yaw_drift_rad": 0.017750208646398362,
    "velocity_rms": [
      0.0011941515323118579,
      0.0009719387709058877,
      0.005857364830571079
    ],
    "samples": 201,
    "strategy": "continuous Teacher at zero velocity command"
  },
  "feet": {
    "FR": {
      "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rf_hip_joint",
        "rf_upper_leg_joint",
        "rf_lower_leg_joint"
      ],
      "entry_time_s": 6.765000000000001,
      "entry_center_world_m": [
        7.003484987502451,
        -0.7450499710442733,
        0.12903616469988402
      ],
      "tread_contact_pair_samples": 4338,
      "verified_top_support_samples": 4334,
      "max_continuous_tread_support_s": 0.6999999999999993,
      "support_interval_s": [
        20.09,
        20.79
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.6999999999999993,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 6.92,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "FL": {
      "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lf_hip_joint",
        "lf_upper_leg_joint",
        "lf_lower_leg_joint"
      ],
      "entry_time_s": 7.025,
      "entry_center_world_m": [
        7.007303448211745,
        -0.5935669941775383,
        0.12922486244031764
      ],
      "tread_contact_pair_samples": 4278,
      "verified_top_support_samples": 4271,
      "max_continuous_tread_support_s": 0.7300000000000004,
      "support_interval_s": [
        19.755,
        20.485
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7300000000000004,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 7.29,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RR": {
      "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rh_hip_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint"
      ],
      "entry_time_s": 7.970000000000001,
      "entry_center_world_m": [
        7.005007280949083,
        -0.809137581179769,
        0.14221377080706618
      ],
      "tread_contact_pair_samples": 4078,
      "verified_top_support_samples": 4059,
      "max_continuous_tread_support_s": 0.7149999999999999,
      "support_interval_s": [
        19.75,
        20.465
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7149999999999999,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.175,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RL": {
      "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lh_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint"
      ],
      "entry_time_s": 8.21,
      "entry_center_world_m": [
        7.005604451037178,
        -0.5415882254946017,
        0.13585076746649244
      ],
      "tread_contact_pair_samples": 4073,
      "verified_top_support_samples": 4055,
      "max_continuous_tread_support_s": 0.634999999999998,
      "support_interval_s": [
        17.95,
        18.584999999999997
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.634999999999998,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.44,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    }
  },
  "body_world_height_gain_diagnostic_m": 0.06489389797146,
  "body_world_height_gain_used_for_pass": false,
  "command_timing": {
    "duration_s": 30.0,
    "forward_end_s": 22,
    "stop_evaluation_start_s": 26
  }
}
```

新增continue轮兼容旧summary原记录（时窗/高度规则不适用，不据此判断能否登台）：

```json
[
  {
    "name": "step10_continue",
    "status": "failed",
    "reason": "Teacher stand/stop translation drift; Teacher stand/stop residual velocity; Stand/stop window contains nonzero velocity command",
    "metrics": {
      "duration_s": 30.0,
      "samples": 1501,
      "max_abs_roll_pitch_rad": 0.05935923857839813,
      "min_body_clearance_m": 0.23457368181977484,
      "body_contact_samples": 0,
      "unknown_contact_samples": 0,
      "max_applied_torque_Nm": 16.557874052988854,
      "position_delta_m": [
        4.600319170076096,
        0.45062194640502795,
        -0.019556714086178983
      ],
      "total_yaw_delta_rad": 0.06197269232487406,
      "assisted_phase_counts": {
        "support_capture": 0,
        "support_hold": 0
      },
      "tracking": [
        {
          "window_s": [
            5,
            10
          ],
          "command_mean": [
            0.2999999999999986,
            0.0,
            0.0
          ],
          "measured_mean": [
            0.2712102100487885,
            -0.006769432697175983,
            0.008260296603306225
          ],
          "rmse": [
            0.05108766742230146,
            0.03771411094896639,
            0.05894133545227834
          ]
        }
      ],
      "stop_teacher": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9100046238557103,
        "yaw_drift_rad": 0.10450069501188627,
        "velocity_rms": [
          0.1328191570785257,
          0.028512743291096966,
          0.030063657617788484
        ]
      },
      "stop_assisted": {
        "samples": 0
      },
      "stop": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9100046238557103,
        "yaw_drift_rad": 0.10450069501188627,
        "velocity_rms": [
          0.1328191570785257,
          0.028512743291096966,
          0.030063657617788484
        ]
      },
      "terrain": {
        "description": "Low-step ascent onto box tread; not complete obstacle traversal",
        "start_median_position_m": [
          6.001747980796979,
          -0.6969356765924537,
          0.315415261441494
        ],
        "end_median_position_m": [
          10.600284280274217,
          -0.24941706897288976,
          0.3799977573705121
        ],
        "height_gain_m": 0.0645824959290181,
        "native_end_support_foot_groups": [
          1,
          2,
          3,
          4
        ],
        "native_end_all_four_support_samples": 201,
        "native_end_samples": 201,
        "end_inside_tread": true,
        "step_height_m": 0.1,
        "minimum_height_gain_m": 0.06
      }
    }
  }
]
```

[实际MP4回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232626_step10_continue_functional_retest_r3_4512/frame_replay.mp4)

[实际GIF回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232626_step10_continue_functional_retest_r3_4512/frame_replay.gif)

![实际运动、接触、间隙与轨迹](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232626_step10_continue_functional_retest_r3_4512/step_functional.png)

逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_232626_step10_continue_functional_retest_r3_4512/step_retest_evidence_manifest.json)。

回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。

## 20261003_233510_step05_continue_visible_retest_r1_028d

功能逐项收据：

功能读取 `summary_functional.json`；原摘要与纠正收据均保留。200Hz补充安全：**passed**。

- fixture: `{"passed": true, "front_x_m": 7.0, "back_x_m": 13.0, "tread_y_limits_m": [-1.4, 0.0], "top_z_m": 0.05, "collision": "teacher_low_step::link::collision"}`
- complete_continuous_runtime: `{"passed": true, "duration_s": 30.0, "policy_samples": 1501, "physics_samples": 6001, "native_termination": true, "worker_fault": null, "runner_error": null, "main_exit_codes": [0, 0]}`
- continuous_cpu_teacher: `{"passed": true, "target_action_max_error_rad": 0.0, "bootstrap_excluded_s": 0.1, "legacy_interface": "passed"}`
- command_profile: `{"passed": true, "forward_interval_s": [3, 22], "stop_interval_s": [26, 30], "forward_samples": 950, "stop_samples": 201}`
- FR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1", "radius_m": 0.02, "chain": ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"], "entry_time_s": 6.78, "entry_center_world_m": [7.004414659916565, -0.7382224671274388, 0.06952085463599822], "tread_contact_pair_samples": 4340, "verified_top_support_samples": 4329, "max_continuous_tread_support_s": 0.6749999999999989, "support_interval_s": [8.950000000000001, 9.625], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.6749999999999989, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 6.890000000000001, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- FL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1", "radius_m": 0.02, "chain": ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"], "entry_time_s": 7.025, "entry_center_world_m": [7.000115443496173, -0.6043869709543904, 0.07940989035505669], "tread_contact_pair_samples": 4429, "verified_top_support_samples": 4317, "max_continuous_tread_support_s": 0.7850000000000001, "support_interval_s": [8.58, 9.365], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7850000000000001, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 7.21, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1", "radius_m": 0.02, "chain": ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"], "entry_time_s": 8.505, "entry_center_world_m": [7.006919959721385, -0.8067367143830572, 0.0773448658101325], "tread_contact_pair_samples": 4183, "verified_top_support_samples": 4050, "max_continuous_tread_support_s": 0.7449999999999992, "support_interval_s": [8.63, 9.375], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7449999999999992, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.73, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1", "radius_m": 0.02, "chain": ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"], "entry_time_s": 8.790000000000001, "entry_center_world_m": [7.001789606600672, -0.5411518862106246, 0.07452337367240569], "tread_contact_pair_samples": 4118, "verified_top_support_samples": 3985, "max_continuous_tread_support_s": 1.004999999999999, "support_interval_s": [11.120000000000001, 12.125], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 1.004999999999999, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 9.055, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- four_feet_simultaneous_top_stop_support: `{"passed": true, "continuous_span_s": 4.0, "interval_s": [26.0, 30.0], "final_sample_all_four_on_top": true}`
- continued_teacher_forward_on_tread: `{"passed": true, "edge_plus_half_m_first_time_s": 9.57, "all_four_feet_active_ascent_confirmed_time_s": 9.055, "continued_x_progress_m": 2.906660280597184, "late_interval_s": [9.57, 22], "late_forward_velocity_mean_mps": 0.23385035797775625, "late_positive_velocity_fraction": 1.0, "on_platform_body_samples_fraction": 1.0, "continued_duration_s": 12.424999999999997, "rolling_velocity_windows": 2387, "rolling_window_s": 0.5, "maximum_no_foot_top_support_s": 0.0, "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"}`
- safety: `{"passed": true, "max_abs_roll_pitch_rad": 0.039924376876884285, "min_support_clearance_m": 0.2664275660576187, "body_contact_samples": 0, "unknown_contact_samples": 0, "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"}`
- teacher_zero_command_stop: `{"passed": true, "window_s": [26, 30], "translation_drift_m": 0.0005139875341777531, "yaw_drift_rad": 0.004163867110098141, "velocity_rms": [0.0003298905402229285, 0.00015423779773844176, 0.001164953219387489], "samples": 201, "strategy": "continuous Teacher at zero velocity command"}`

原始功能指标：

```json
{
  "continued_motion": {
    "edge_plus_half_m_first_time_s": 9.57,
    "all_four_feet_active_ascent_confirmed_time_s": 9.055,
    "continued_x_progress_m": 2.906660280597184,
    "late_interval_s": [
      9.57,
      22
    ],
    "late_forward_velocity_mean_mps": 0.23385035797775625,
    "late_positive_velocity_fraction": 1.0,
    "on_platform_body_samples_fraction": 1.0,
    "continued_duration_s": 12.424999999999997,
    "rolling_velocity_windows": 2387,
    "rolling_window_s": 0.5,
    "maximum_no_foot_top_support_s": 0.0,
    "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"
  },
  "safety": {
    "max_abs_roll_pitch_rad": 0.039924376876884285,
    "min_support_clearance_m": 0.2664275660576187,
    "body_contact_samples": 0,
    "unknown_contact_samples": 0,
    "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"
  },
  "stop": {
    "window_s": [
      26,
      30
    ],
    "translation_drift_m": 0.0005139875341777531,
    "yaw_drift_rad": 0.004163867110098141,
    "velocity_rms": [
      0.0003298905402229285,
      0.00015423779773844176,
      0.001164953219387489
    ],
    "samples": 201,
    "strategy": "continuous Teacher at zero velocity command"
  },
  "feet": {
    "FR": {
      "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rf_hip_joint",
        "rf_upper_leg_joint",
        "rf_lower_leg_joint"
      ],
      "entry_time_s": 6.78,
      "entry_center_world_m": [
        7.004414659916565,
        -0.7382224671274388,
        0.06952085463599822
      ],
      "tread_contact_pair_samples": 4340,
      "verified_top_support_samples": 4329,
      "max_continuous_tread_support_s": 0.6749999999999989,
      "support_interval_s": [
        8.950000000000001,
        9.625
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.6749999999999989,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 6.890000000000001,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "FL": {
      "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lf_hip_joint",
        "lf_upper_leg_joint",
        "lf_lower_leg_joint"
      ],
      "entry_time_s": 7.025,
      "entry_center_world_m": [
        7.000115443496173,
        -0.6043869709543904,
        0.07940989035505669
      ],
      "tread_contact_pair_samples": 4429,
      "verified_top_support_samples": 4317,
      "max_continuous_tread_support_s": 0.7850000000000001,
      "support_interval_s": [
        8.58,
        9.365
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7850000000000001,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 7.21,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RR": {
      "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rh_hip_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint"
      ],
      "entry_time_s": 8.505,
      "entry_center_world_m": [
        7.006919959721385,
        -0.8067367143830572,
        0.0773448658101325
      ],
      "tread_contact_pair_samples": 4183,
      "verified_top_support_samples": 4050,
      "max_continuous_tread_support_s": 0.7449999999999992,
      "support_interval_s": [
        8.63,
        9.375
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7449999999999992,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.73,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RL": {
      "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lh_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint"
      ],
      "entry_time_s": 8.790000000000001,
      "entry_center_world_m": [
        7.001789606600672,
        -0.5411518862106246,
        0.07452337367240569
      ],
      "tread_contact_pair_samples": 4118,
      "verified_top_support_samples": 3985,
      "max_continuous_tread_support_s": 1.004999999999999,
      "support_interval_s": [
        11.120000000000001,
        12.125
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 1.004999999999999,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 9.055,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    }
  },
  "body_world_height_gain_diagnostic_m": 0.023222602943720194,
  "body_world_height_gain_used_for_pass": false,
  "command_timing": {
    "duration_s": 30.0,
    "forward_end_s": 22,
    "stop_evaluation_start_s": 26
  }
}
```

新增continue轮兼容旧summary原记录（时窗/高度规则不适用，不据此判断能否登台）：

```json
[
  {
    "name": "step05_continue",
    "status": "failed",
    "reason": "Teacher stand/stop translation drift; Teacher stand/stop residual velocity; Stand/stop window contains nonzero velocity command; Low step tread was not reached with settled measured height gain",
    "metrics": {
      "duration_s": 30.0,
      "samples": 1501,
      "max_abs_roll_pitch_rad": 0.039924376876884285,
      "min_body_clearance_m": 0.2664275660576187,
      "body_contact_samples": 0,
      "unknown_contact_samples": 0,
      "max_applied_torque_Nm": 15.560171297267097,
      "position_delta_m": [
        4.434752001298786,
        0.5224141317262949,
        -0.06101994310748343
      ],
      "total_yaw_delta_rad": 0.28072649777593733,
      "assisted_phase_counts": {
        "support_capture": 0,
        "support_hold": 0
      },
      "tracking": [
        {
          "window_s": [
            5,
            10
          ],
          "command_mean": [
            0.2999999999999986,
            0.0,
            0.0
          ],
          "measured_mean": [
            0.24261920471407955,
            0.0024459310863514014,
            0.012240824085237856
          ],
          "rmse": [
            0.0708432601809192,
            0.04822173660672922,
            0.05448839916815815
          ]
        }
      ],
      "stop_teacher": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9157221750002436,
        "yaw_drift_rad": 0.057663338266871755,
        "velocity_rms": [
          0.13284170352079835,
          0.03111444570329373,
          0.020549059081416316
        ]
      },
      "stop_assisted": {
        "samples": 0
      },
      "stop": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9157221750002436,
        "yaw_drift_rad": 0.057663338266871755,
        "velocity_rms": [
          0.13284170352079835,
          0.03111444570329373,
          0.020549059081416316
        ]
      },
      "terrain": {
        "description": "Low-step ascent onto box tread; not complete obstacle traversal",
        "start_median_position_m": [
          6.001747980796979,
          -0.6969356765924537,
          0.315415261441494
        ],
        "end_median_position_m": [
          10.434649953994573,
          -0.17761997385525144,
          0.33876157997109835
        ],
        "height_gain_m": 0.023346318529604337,
        "native_end_support_foot_groups": [
          1,
          2,
          3,
          4
        ],
        "native_end_all_four_support_samples": 201,
        "native_end_samples": 201,
        "end_inside_tread": true,
        "step_height_m": 0.1,
        "minimum_height_gain_m": 0.06
      }
    }
  }
]
```

[实际MP4回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_233510_step05_continue_visible_retest_r1_028d/frame_replay.mp4)

[实际GIF回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_233510_step05_continue_visible_retest_r1_028d/frame_replay.gif)

![实际运动、接触、间隙与轨迹](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_233510_step05_continue_visible_retest_r1_028d/step_functional.png)

逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_233510_step05_continue_visible_retest_r1_028d/step_retest_evidence_manifest.json)。

回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。

## 20261003_233547_step10_continue_visible_retest_r1_7da7

功能逐项收据：

功能读取 `summary_functional.json`；原摘要与纠正收据均保留。200Hz补充安全：**passed**。

- fixture: `{"passed": true, "front_x_m": 7.0, "back_x_m": 13.0, "tread_y_limits_m": [-1.4, 0.0], "top_z_m": 0.1, "collision": "teacher_low_step::link::collision"}`
- complete_continuous_runtime: `{"passed": true, "duration_s": 30.0, "policy_samples": 1501, "physics_samples": 6001, "native_termination": true, "worker_fault": null, "runner_error": null, "main_exit_codes": [0, 0]}`
- continuous_cpu_teacher: `{"passed": true, "target_action_max_error_rad": 0.0, "bootstrap_excluded_s": 0.1, "legacy_interface": "passed"}`
- command_profile: `{"passed": true, "forward_interval_s": [3, 22], "stop_interval_s": [26, 30], "forward_samples": 950, "stop_samples": 201}`
- FR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1", "radius_m": 0.02, "chain": ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"], "entry_time_s": 6.765000000000001, "entry_center_world_m": [7.003484987502451, -0.7450499710442733, 0.12903616469988402], "tread_contact_pair_samples": 4338, "verified_top_support_samples": 4334, "max_continuous_tread_support_s": 0.6999999999999993, "support_interval_s": [20.09, 20.79], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.6999999999999993, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 6.92, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- FL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1", "radius_m": 0.02, "chain": ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"], "entry_time_s": 7.025, "entry_center_world_m": [7.007303448211745, -0.5935669941775383, 0.12922486244031764], "tread_contact_pair_samples": 4278, "verified_top_support_samples": 4271, "max_continuous_tread_support_s": 0.7300000000000004, "support_interval_s": [19.755, 20.485], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7300000000000004, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 7.29, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- FL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RR_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1", "radius_m": 0.02, "chain": ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"], "entry_time_s": 7.970000000000001, "entry_center_world_m": [7.005007280949083, -0.809137581179769, 0.14221377080706618], "tread_contact_pair_samples": 4078, "verified_top_support_samples": 4059, "max_continuous_tread_support_s": 0.7149999999999999, "support_interval_s": [19.75, 20.465], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.7149999999999999, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.175, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RR_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- RL_front_entry_and_real_tread_support: `{"passed": true, "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1", "radius_m": 0.02, "chain": ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"], "entry_time_s": 8.21, "entry_center_world_m": [7.005604451037178, -0.5415882254946017, 0.13585076746649244], "tread_contact_pair_samples": 4073, "verified_top_support_samples": 4055, "max_continuous_tread_support_s": 0.634999999999998, "support_interval_s": [17.95, 18.584999999999997], "top_position_with_missing_normals_samples": 0, "position_and_collider_geometry_fallback_span_s": 0.634999999999998, "tread_pair_missing_position_samples": 0, "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support", "first_active_support_confirmed_time_s": 8.44, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_stopped_on_real_tread: `{"passed": true, "stop_tread_support_samples": 801, "max_continuous_stop_tread_support_s": 4.0, "stop_support_interval_s": [26.0, 30.0]}`
- RL_contact_geometry_complete: `{"passed": true, "missing_positions": 0, "missing_normals": 0, "raw_wrench_not_used_as_inferred_load": true}`
- four_feet_simultaneous_top_stop_support: `{"passed": true, "continuous_span_s": 4.0, "interval_s": [26.0, 30.0], "final_sample_all_four_on_top": true}`
- continued_teacher_forward_on_tread: `{"passed": true, "edge_plus_half_m_first_time_s": 9.005, "all_four_feet_active_ascent_confirmed_time_s": 8.44, "continued_x_progress_m": 3.085849879133712, "late_interval_s": [9.005, 22], "late_forward_velocity_mean_mps": 0.2375096390912059, "late_positive_velocity_fraction": 1.0, "on_platform_body_samples_fraction": 1.0, "continued_duration_s": 12.989999999999997, "rolling_velocity_windows": 2500, "rolling_window_s": 0.5, "maximum_no_foot_top_support_s": 0.0, "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"}`
- safety: `{"passed": true, "max_abs_roll_pitch_rad": 0.05935923857839813, "min_support_clearance_m": 0.23457368181977484, "body_contact_samples": 0, "unknown_contact_samples": 0, "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"}`
- teacher_zero_command_stop: `{"passed": true, "window_s": [26, 30], "translation_drift_m": 0.0036514907168625813, "yaw_drift_rad": 0.017750208646398362, "velocity_rms": [0.0011941515323118579, 0.0009719387709058877, 0.005857364830571079], "samples": 201, "strategy": "continuous Teacher at zero velocity command"}`

原始功能指标：

```json
{
  "continued_motion": {
    "edge_plus_half_m_first_time_s": 9.005,
    "all_four_feet_active_ascent_confirmed_time_s": 8.44,
    "continued_x_progress_m": 3.085849879133712,
    "late_interval_s": [
      9.005,
      22
    ],
    "late_forward_velocity_mean_mps": 0.2375096390912059,
    "late_positive_velocity_fraction": 1.0,
    "on_platform_body_samples_fraction": 1.0,
    "continued_duration_s": 12.989999999999997,
    "rolling_velocity_windows": 2500,
    "rolling_window_s": 0.5,
    "maximum_no_foot_top_support_s": 0.0,
    "measured_position_source": "native Gazebo state, diagnostic simulation evaluation only"
  },
  "safety": {
    "max_abs_roll_pitch_rad": 0.05935923857839813,
    "min_support_clearance_m": 0.23457368181977484,
    "body_contact_samples": 0,
    "unknown_contact_samples": 0,
    "clearance_definition": "base-link origin world z minus actual collision surface directly below base; safety only"
  },
  "stop": {
    "window_s": [
      26,
      30
    ],
    "translation_drift_m": 0.0036514907168625813,
    "yaw_drift_rad": 0.017750208646398362,
    "velocity_rms": [
      0.0011941515323118579,
      0.0009719387709058877,
      0.005857364830571079
    ],
    "samples": 201,
    "strategy": "continuous Teacher at zero velocity command"
  },
  "feet": {
    "FR": {
      "collision": "go2::rf_lower_leg_link::rf_lower_leg_link_fixed_joint_lump__rf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rf_hip_joint",
        "rf_upper_leg_joint",
        "rf_lower_leg_joint"
      ],
      "entry_time_s": 6.765000000000001,
      "entry_center_world_m": [
        7.003484987502451,
        -0.7450499710442733,
        0.12903616469988402
      ],
      "tread_contact_pair_samples": 4338,
      "verified_top_support_samples": 4334,
      "max_continuous_tread_support_s": 0.6999999999999993,
      "support_interval_s": [
        20.09,
        20.79
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.6999999999999993,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 6.92,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "FL": {
      "collision": "go2::lf_lower_leg_link::lf_lower_leg_link_fixed_joint_lump__lf_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lf_hip_joint",
        "lf_upper_leg_joint",
        "lf_lower_leg_joint"
      ],
      "entry_time_s": 7.025,
      "entry_center_world_m": [
        7.007303448211745,
        -0.5935669941775383,
        0.12922486244031764
      ],
      "tread_contact_pair_samples": 4278,
      "verified_top_support_samples": 4271,
      "max_continuous_tread_support_s": 0.7300000000000004,
      "support_interval_s": [
        19.755,
        20.485
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7300000000000004,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 7.29,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RR": {
      "collision": "go2::rh_lower_leg_link::rh_lower_leg_link_fixed_joint_lump__rh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "rh_hip_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint"
      ],
      "entry_time_s": 7.970000000000001,
      "entry_center_world_m": [
        7.005007280949083,
        -0.809137581179769,
        0.14221377080706618
      ],
      "tread_contact_pair_samples": 4078,
      "verified_top_support_samples": 4059,
      "max_continuous_tread_support_s": 0.7149999999999999,
      "support_interval_s": [
        19.75,
        20.465
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.7149999999999999,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.175,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    },
    "RL": {
      "collision": "go2::lh_lower_leg_link::lh_lower_leg_link_fixed_joint_lump__lh_foot_link_collision_1",
      "radius_m": 0.02,
      "chain": [
        "lh_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint"
      ],
      "entry_time_s": 8.21,
      "entry_center_world_m": [
        7.005604451037178,
        -0.5415882254946017,
        0.13585076746649244
      ],
      "tread_contact_pair_samples": 4073,
      "verified_top_support_samples": 4055,
      "max_continuous_tread_support_s": 0.634999999999998,
      "support_interval_s": [
        17.95,
        18.584999999999997
      ],
      "top_position_with_missing_normals_samples": 0,
      "position_and_collider_geometry_fallback_span_s": 0.634999999999998,
      "tread_pair_missing_position_samples": 0,
      "contact_normal_source": "actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support",
      "first_active_support_confirmed_time_s": 8.44,
      "stop_tread_support_samples": 801,
      "max_continuous_stop_tread_support_s": 4.0,
      "stop_support_interval_s": [
        26.0,
        30.0
      ]
    }
  },
  "body_world_height_gain_diagnostic_m": 0.06489389797146,
  "body_world_height_gain_used_for_pass": false,
  "command_timing": {
    "duration_s": 30.0,
    "forward_end_s": 22,
    "stop_evaluation_start_s": 26
  }
}
```

新增continue轮兼容旧summary原记录（时窗/高度规则不适用，不据此判断能否登台）：

```json
[
  {
    "name": "step10_continue",
    "status": "failed",
    "reason": "Teacher stand/stop translation drift; Teacher stand/stop residual velocity; Stand/stop window contains nonzero velocity command",
    "metrics": {
      "duration_s": 30.0,
      "samples": 1501,
      "max_abs_roll_pitch_rad": 0.05935923857839813,
      "min_body_clearance_m": 0.23457368181977484,
      "body_contact_samples": 0,
      "unknown_contact_samples": 0,
      "max_applied_torque_Nm": 16.557874052988854,
      "position_delta_m": [
        4.600319170076096,
        0.45062194640502795,
        -0.019556714086178983
      ],
      "total_yaw_delta_rad": 0.06197269232487406,
      "assisted_phase_counts": {
        "support_capture": 0,
        "support_hold": 0
      },
      "tracking": [
        {
          "window_s": [
            5,
            10
          ],
          "command_mean": [
            0.2999999999999986,
            0.0,
            0.0
          ],
          "measured_mean": [
            0.2712102100487885,
            -0.006769432697175983,
            0.008260296603306225
          ],
          "rmse": [
            0.05108766742230146,
            0.03771411094896639,
            0.05894133545227834
          ]
        }
      ],
      "stop_teacher": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9100046238557103,
        "yaw_drift_rad": 0.10450069501188627,
        "velocity_rms": [
          0.1328191570785257,
          0.028512743291096966,
          0.030063657617788484
        ]
      },
      "stop_assisted": {
        "samples": 0
      },
      "stop": {
        "samples": 601,
        "window_s": [
          18.0,
          30.0
        ],
        "translation_drift_m": 0.9100046238557103,
        "yaw_drift_rad": 0.10450069501188627,
        "velocity_rms": [
          0.1328191570785257,
          0.028512743291096966,
          0.030063657617788484
        ]
      },
      "terrain": {
        "description": "Low-step ascent onto box tread; not complete obstacle traversal",
        "start_median_position_m": [
          6.001747980796979,
          -0.6969356765924537,
          0.315415261441494
        ],
        "end_median_position_m": [
          10.600284280274217,
          -0.24941706897288976,
          0.3799977573705121
        ],
        "height_gain_m": 0.0645824959290181,
        "native_end_support_foot_groups": [
          1,
          2,
          3,
          4
        ],
        "native_end_all_four_support_samples": 201,
        "native_end_samples": 201,
        "end_inside_tread": true,
        "step_height_m": 0.1,
        "minimum_height_gain_m": 0.06
      }
    }
  }
]
```

[实际MP4回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_233547_step10_continue_visible_retest_r1_7da7/frame_replay.mp4)

[实际GIF回放](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_233547_step10_continue_visible_retest_r1_7da7/frame_replay.gif)

![实际运动、接触、间隙与轨迹](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_233547_step10_continue_visible_retest_r1_7da7/step_functional.png)

逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_233547_step10_continue_visible_retest_r1_7da7/step_retest_evidence_manifest.json)。

回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。

## 旧42轮中的六次台阶结果继续保留

旧5cm/10cm每类3轮的原summary按原数值判据保留。历史接触和继续进度另有只读审计，旧数据缺顶面接触XYZ/normal，不能追认本轮功能通过。

| 旧正式运行 | 原interface / motion | 原失败原因 |
|---|---|---|
| [20261003_194006_step05_r1_7b46](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_194006_step05_r1_7b46/summary.json) | passed / failed | Low step tread was not reached with settled measured height gain |
| [20261003_194529_step05_r2_6f0b](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_194529_step05_r2_6f0b/summary.json) | passed / failed | Low step tread was not reached with settled measured height gain |
| [20261003_195052_step05_r3_b4ba](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_195052_step05_r3_b4ba/summary.json) | passed / failed | Low step tread was not reached with settled measured height gain |
| [20261003_194035_step10_r1_b04c](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_194035_step10_r1_b04c/summary.json) | passed / passed |  |
| [20261003_194558_step10_r2_8974](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_194558_step10_r2_8974/summary.json) | passed / passed |  |
| [20261003_195120_step10_r3_18d2](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261003_195120_step10_r3_18d2/summary.json) | passed / passed |  |

历史功能只读审计：[原记录](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/step_historical_functional_review_20261003.json)。

原全局验收levels：`{'interface': 'passed', 'motion': 'failed', 'sim2sim': 'failed', 'navigation': 'unverified', 'real_robot': 'unverified'}`；本脚本没有写入原acceptance.json、旧REPORT或summary。导航未执行，真机未验证。

功能协议SHA256：`8b7a43c74b21c063f5b56984e2b63256ee9bb0db88fda5c2f59944bb8b70f431`；模型SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。
