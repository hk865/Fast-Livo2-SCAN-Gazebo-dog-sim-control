# fast_livo2_ros

FAST-LIVO2 的 ROS2 启动/封装层，负责加载配置、实例化 `LIVMapper` 并启动 SLAM 节点。

## 代码架构

### 依赖关系

```
fast_livo2_ros (可执行文件)
├── fast_livo2_core   # 核心 SLAM 算法库
├── vikit_common      # 相机模型
├── vikit_ros         # 相机参数加载
└── image_transport   # 图像传输
```

### 文件结构

| 文件 | 功能 |
|------|------|
| [src/main.cpp](src/main.cpp) | **唯一入口**：初始化 ROS2 节点 → 创建 `LIVMapper` → 订阅传感器 → `mapper.run()` 主循环 |
| [launch/mapping_aviz.launch.py](launch/mapping_aviz.launch.py) | 启动文件：加载 avia.yaml + camera_pinhole.yaml 参数 → 启动 `fastlivo_mapping` 节点 |
| [config/avia.yaml](config/avia.yaml) | LiDAR + IMU 参数配置 |
| [config/camera_pinhole.yaml](config/camera_pinhole.yaml) | 相机内参配置（pinhole 模型） |
| [rviz/fast_livo2.rviz](rviz/fast_livo2.rviz) | RViz2 可视化配置 |
| [scripts/decompress_images.py](scripts/decompress_images.py) | 图像解压脚本（ROS bag 回放用） |

### main.cpp 核心逻辑

```cpp
main():
  rclcpp::init()
  LIVMapper mapper(nh, "laserMapping")          // 构造主控类
  mapper.initializeSubscribersAndPublishers()    // 注册 Sub/Pub
  mapper.run(nh)                                // 主循环 (sync + LIO + VIO)
  rclcpp::shutdown()
```

**注意**：所有核心算法逻辑都在 `fast_livo2_core::LIVMapper` 中，`fast_livo2_ros` 只负责 ROS2 生命周期管理。

## 数据流分析

### 启动流程

```
ros2 launch fast_livo2_ros mapping_aviz.launch.py use_rviz:=True

启动时执行的节点:
┌──────────────────────────────┐
│ parameter_blackboard         │  ← 加载 camera_pinhole.yaml 作为全局参数
│ (demo_nodes_cpp)             │
└──────────────────────────────┘
┌──────────────────────────────┐
│ image_transport republish    │  ← 解压压缩图像 (compressed → raw)
│ (如果 bag 中是压缩图像)       │
└──────────────────────────────┘
┌──────────────────────────────┐
│ fastlivo_mapping             │  ← 加载 avia.yaml + camera params
│ /laserMapping                │  ← LIVMapper.run() 主循环
└──────────────────────────────┘
┌──────────────────────────────┐
│ rviz2 (可选)                 │  ← 加载 fast_livo2.rviz
└──────────────────────────────┘
```

### 参数加载

`LIVMapper::readParameters()` 在构造函数中读取所有参数（通过 ROS2 declare_parameter/get_parameter）。关键配置项分类：

| 命名空间 | 关键参数 | 说明 |
|----------|---------|------|
| `common` | `lid_topic, imu_topic, img_topic, img_en, lidar_en` | 传感器话题和启用开关 |
| `preprocess` | `lidar_type, blind, point_filter_num, filter_size_surf` | 预处理参数 |
| `imu` | `imu_en, acc_cov, gyr_cov, gravity_est_en` | IMU 启用和协方差 |
| `vio` | `max_iterations, patch_size, normal_en, exposure_estimate_en` | VIO 迭代和 patch 参数 |
| `lio` | `voxel_size, max_layer, sigma_num, beam_err, dept_err` | LIO 体素参数 |
| `extrin_calib` | `extrinsic_T, extrinsic_R, Pcl, Rcl` | 传感器外参 |
| `time_offset` | `imu_time_offset, img_time_offset` | 时间同步偏移 |

### 启动命令

```bash
# 真机/数据集回放
ros2 launch fast_livo2_ros mapping_aviz.launch.py use_rviz:=True

# 仿真环境 (在 go2_slam_bringup 或 go2_simulation 中自动启动)
# fast_livo2_ros 作为子节点被 IncludeLaunchDescription 调用
```
