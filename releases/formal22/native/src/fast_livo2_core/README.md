# fast_livo2_core

FAST-LIVO2 核心算法库，实现 LiDAR-Inertial-Visual Odometry (LIVO) 的底层算法，包括 ESIKF 状态估计、体素地图构建、视觉光度误差优化、IMU 预积分等。

## 代码架构

### 依赖关系

```
fast_livo2_core (共享库)
├── vikit_common     # 相机模型、数学工具
├── vikit_ros        # ROS2 相机参数加载
├── Sophus           # SE(3) 李群
├── Eigen / OpenCV / PCL
└── livox_ros_driver2  # Livox LiDAR 驱动
```

### 类与模块

| 文件 | 类/结构体 | 功能 |
|------|----------|------|
| [LIVMapper.h](include/fast_livo2_core/core/LIVMapper.h) | `LIVMapper` | **主控类**，协调 LIO/VIO/LIVO 模式的状态估计。管理传感器同步、EKF 更新、可视化发布、GPS 融合。 |
| [common_lib.h](include/fast_livo2_core/core/common_lib.h) | `StatesGroup` | **19 维状态向量**：rotation(3) + position(3) + inv_expo(1) + velocity(3) + gyro_bias(3) + accel_bias(3) + gravity(3)。支持 SO(3) 流形上的加/减法运算。 |
| | `LidarMeasureGroup` | 存储一帧 LiDAR 数据 + 时间区间的 IMU/图像测量对。 |
| | `pointWithVar` | 带协方差的 3D 点，记录 body/IMU/world 三坐标系坐标。 |
| | `MeasureGroup` | 一对 VIO 时间测量：包含 IMU 队列 + 一帧图像。 |
| | `Pose6D` | LiDAR 点去畸变用的预积分位姿快照。 |
| [vio.h](include/fast_livo2_core/core/vio.h) | `VIOManager` | **直接法视觉里程计管理器**。核心功能：<br>- `processFrame()`: 处理新图像帧，执行视觉稀疏建图和 EKF 更新<br>- `retrieveFromVisualSparseMap()`: 从 voxel map 中检索可见的 VisualPoint<br>- `generateVisualMapPoints()`: 在新区域生成视觉地图点<br>- `computeJacobianAndUpdateEKF()`: 计算 Photometric Jacobian 并更新 EKF<br>- `updateState()`/`updateStateInverse()`: Inverse Compositional / Forward Compositional 光度对齐<br>- `getWarpMatrixAffine()`: 计算仿射变换矩阵用于 patch 投影 |
| | `VOXEL_POINTS` | 对 VoxelCell 存储所有 VisualPoint 的哈希表进行包装。 |
| | `SubSparseMap` | 当前帧可见的稀疏视觉子地图缓存。 |
| [voxel_map.h](include/fast_livo2_core/core/voxel_map.h) | `VoxelMapManager` | **八叉体素地图管理器**。核心功能：<br>- `UpdateVoxelMap()`: 用新 LiDAR 点云增量更新体素地图<br>- `BuildResidualListOMP()`: OpenMP 并行为每个 LiDAR 点构建 point-to-plane 残差<br>- `StateEstimation()`: 基于 body_var + 平面方程的 ESIKF 更新<br>- `mapSliding()`: 局部地图滑动窗口机制 |
| | `VoxelOctoTree` | 八叉树叶节点，内部存储 `VoxelPlane`（均值+方差+法向量+平面方程）。支持递归细分直到平面拟合或达到最大层数。 |
| | `VoxelPlane` | 拟合的点云平面：中心、法向量、协方差、特征值等。 |
| | `PointToPlane` | point-to-plane 残差项：LiDAR 点在世界坐标系到拟合平面的距离。 |
| | `VoxelMapConfig` | 体素地图配置（max_layer, voxel_size, sigma_num, sliding_thresh 等）。 |
| [IMU_Processing.h](include/fast_livo2_core/core/IMU_Processing.h) | `ImuProcess` | **IMU 预积分与点云去畸变**。核心功能：<br>- `Process2()`: IMU 初始化 + 计算预积分位姿 + 对点云去畸变<br>- `UndistortPcl()`: 用预积分位姿插值去畸变 LiDAR 点云<br>- `IMU_init()`: 静止时校零偏（b_a, b_g），初始化重力方向 |
| [preprocess.h](include/fast_livo2_core/core/preprocess.h) | `Preprocess` | **LiDAR 点云预处理**。支持多品牌激光雷达（Avia, Velodyne, Ouster, Hesai, RoboSense, L515）。核心功能：<br>- `avia_handler()`: 解析 Livox CustomMsg → PointCloudXYZI<br>- `give_feature()`: 极坐标邻域差分法提取 planar 特征<br>- `plane_judge()`: 判断点是否属于平面<br>- `edge_jump_judge()`: 检测边缘跳变点 |
| | `orgtype` | 原始 LiDAR 点的结构描述：range, angle, edge jump 类型等。 |
| [frame.h](include/fast_livo2_core/core/frame.h) | `Frame` | **图像帧**。存储图像、提取的 Features、当前帧位姿（`T_f_w_`）及 IMU 先验位姿（`T_f_w_prior_`）。提供 w2c / w2f / f2c 坐标系转换。 |
| | `frame_utils` | 构建图像金字塔 `createImgPyramid()`。 |
| [feature.h](include/fast_livo2_core/core/feature.h) | `Feature` | 图像 patch 特征。关联到其 3D `VisualPoint` 的引用，存储像素坐标、bearing vector、梯度方向、patche 数据等。类型分为 CORNER / EDGELET。 |
| [visual_point.h](include/fast_livo2_core/core/visual_point.h) | `VisualPoint` | 视觉 3D 地图点。存储世界坐标、法向量、所有观测（Feature 列表）、收敛标志。支持 `findMinScoreFeature()`（选最佳观测视角）、`getCloseViewObs()`（找闭合视角）。 |
| [so3_math.h](include/fast_livo2_core/utils/so3_math.h) | 模板函数 | `Exp()` / `Log()` SO(3) 指数映射/对数映射，`RotMtoEuler()` 旋转矩阵转欧拉角，`SKEW_SYM_MATRX` 反对称矩阵宏。 |
| [types.h](include/fast_livo2_core/utils/types.h) | 类型别名 | `PointCloudXYZI`, `V3D`, `M3D`, `MD(a,b)`, `VD(a)` 等 Eigen/PCL 简写。 |
| [utils.h](include/fast_livo2_core/utils/utils.h) | 工具函数 | `stamp2Sec()` / `sec2Stamp()` 时间戳转换，`createTransformStamped()` / `createQuaternionMsgFromRPY()` TF 消息构造。 |

### 算法核心流程 (LIVMapper::run)

```
loop:
  1. sync_packages()          → 同步 LiDAR + IMU + Image 数据包
  2. processImu() / gravityAlignment()  → IMU 预积分 + 重力对齐
  3. handleFirstFrame()       → 首帧初始化（静置校准、map 初始化）
  4. stateEstimationAndMapping() → 核心状态估计：
      ├── handleLIO()  → VoxelMap 构建 + Point-to-Plane EKF 更新
      ├── handleVIO()  → 视觉稀疏建图 + Photometric EKF 更新
      └── LIVO 模式    → LIO 和 VIO 串行执行
  5. publish_odometry / publish_path → 发布里程计和路径
  6. publish_frame_world / voxel_map → 发布点云和体素地图可视化
```

### 状态维度

19 维 ESIKF 状态向量：
- `rot_end` (SO(3), 3dof): 末端位姿旋转
- `pos_end` (R^3): 末端位姿平移
- `inv_expo_time` (R^1): 曝光时间倒数
- `vel_end` (R^3): 速度
- `bias_g` (R^3): 陀螺仪零偏
- `bias_a` (R^3): 加速度计零偏
- `gravity` (R^3): 重力加速度

## 数据流分析

### Node 内部数据流

```
┌──────────────────────────────────────────────────────────────┐
│                    fastlivo_mapping Node                     │
│                                                              │
│  ┌─────────┐   ┌──────────┐   ┌──────────┐                  │
│  │ sub_pcl │   │ sub_imu  │   │ sub_img  │  ← ROS2 Sub     │
│  └────┬────┘   └────┬─────┘   └────┬─────┘                  │
│       │              │              │                         │
│  ┌────▼────┐    ┌───▼──────┐   ┌───▼──────┐                 │
│  │Preprocess│   │  IMU     │   │ sync_    │                 │
│  │(去噪/提  │   │ buffer+  │   │ packages │                 │
│  │ 特征)    │   │ 预积分   │   │(时间对齐)│                 │
│  └────┬─────┘   └───┬──────┘   └───┬──────┘                 │
│       │              │              │                         │
│       │   ┌──────────▼──┐          │                         │
│       │   │ImuProcess   │          │                         │
│       │   │(去畸变)     │          │                         │
│       │   └──────┬──────┘          │                         │
│       │          │                 │                         │
│  ┌────▼──────────▼─────────────────▼─────┐                   │
│  │          LIVMapper.run()              │                   │
│  │                                       │                   │
│  │  ┌──────────────────────┐            │                   │
│  │  │ handleLIO()          │            │                   │
│  │  │ ├ UpdateVoxelMap()   │  (LiDAR)   │                   │
│  │  │ ├ BuildResidualList()│            │                   │
│  │  │ └ eKOM update        │            │                   │
│  │  └──────────────────────┘            │                   │
│  │                                       │                   │
│  │  ┌──────────────────────┐            │                   │
│  │  │ handleVIO()          │            │                   │
│  │  │ ├ processFrame()     │  (Visual)  │                   │
│  │  │ ├ updateState()      │            │                   │
│  │  │ └ computeJacobian()  │            │                   │
│  │  └──────────────────────┘            │                   │
│  │                                       │                   │
│  │  StatesGroup ←── ESIKF Fusion ←──────┘                   │
│  └──────────────────────────────────────┘                   │
│                                                              │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────┐      │
│  │publish   │ │publish   │ │publish   │ │publish     │      │
│  │odometry  │ │path      │ │clouds    │ │voxel_map   │      │
│  └──────────┘ └──────────┘ └──────────┘ └────────────┘      │
└──────────────────────────────────────────────────────────────┘
```

### 输入 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` 或 `livox_ros_driver2/CustomMsg` | LiDAR 点云 (Velodyne/ouster/hesai 通用) |
| `/livox/imu` | `sensor_msgs/Imu` | IMU 数据 (200Hz) |
| `/camera/image_color` | `sensor_msgs/Image` | 相机图像 (30Hz) |
| `/ground_truth/odom` (可选) | `nav_msgs/Odometry` | GPS/真值融合输入 |

### 输出 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 里程计 (世界系位姿) |
| `/path` | `nav_msgs/Path` | 累计轨迹 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 注册后的全分辨率点云 (world 系) |
| `/cloud_undistort` | `sensor_msgs/PointCloud2` | 去畸变后点云 (body 系) |
| `/rgb_image` | `sensor_msgs/Image` | 带特征可视化标注的图像 |
| `/effective_points` | `sensor_msgs/PointCloud2` | 有效匹配点 (残差可视化) |
| `/voxel_map` | `visualization_msgs/MarkerArray` | 体素平面地图 (RViz 可视化) |
| `/imu_propagate` | `nav_msgs/Odometry` | IMU 传播里程计 (高频输出) |

### TF 树

```
camera_init (世界原点)                 odom (仿真里程计世界)
    │                                      │
    │ ← LIVMapper 发布 odometry             │ ← go2_tf_broadcaster
    ▼                                      ▼
aft_mapped (动态)                       go2/base
    │                                      │
    │ ← LIVMapper 发布                      │ ← robot_state_publisher
    ▼                                      ▼
  livox (LiDAR 传感器系)             base / FR_hip / ... (URDF)
    │
    │ ← fast_livo2_ros static_tf (Pcl 外参)
    ▼
  camera (相机传感器系)
```

**关键帧关系：**
- `camera_init` → `aft_mapped`: 里程计坐标，随时间增长，由 LIVMapper 每帧发布
- `aft_mapped` → `livox`: 恒等变换（LiDAR 与地图对齐后的位姿）
- `livox` → `camera`: 固定外参变换（由 `Pcl` / `Rcl` 配置）
- 仿真环境下额外有 `odom` → `go2/base` (Gazebo 里程计) 及 `go2/base` → `livox` 的桥接 TF

### VIO 内部消息流

```
图像帧到达
    │
    ▼
Frame 对象创建 (T_f_w_ = EKF 先验)
    │
    ▼
VIOManager::processFrame()
    ├── 新建 VisualPoint → 插入 voxel_map
    ├── 从 voxel_map 检索可见 3D 点 → SubSparseMap
    └── computeJacobianAndUpdateEKF()
            ├── 对每个 VisualPoint:
            │   ├── 仿射 Warp patch (getWarpMatrixAffine)
            │   ├── Photometric error 计算
            │   ├── Jacobian (投影+光度) 计算
            │   └── 堆叠 H, b (Hessian+残差)
            └── ESIKF 迭代更新 state
```

### LIO 内部消息流

```
LiDAR 点云到达
    │
    ▼
Preprocess::process()
    ├── 解析 CustomMsg/PointCloud2 → PointCloudXYZI
    └── 提取 planar 特征 (可选)
    │
    ▼
VoxelMapManager::UpdateVoxelMap()
    ├── TransformLidar(): 点云 body→world
    └── 逐点 Insert 到 VoxelOctoTree（自适应八叉树细分）
    │
    ▼
VoxelMapManager::BuildResidualListOMP()
    ├── 对每个 LiDAR 点 find_correspond()
    ├── 对每个匹配到的体素平面计算 point-to-plane distance
    └── 堆叠 H, b
    │
    ▼
ESIKF ← 残差融合更新 StatesGroup
```
