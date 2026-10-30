# Building模块与数据流

`start.py → src/bringup/building.py`负责配置生成、进程生命周期和完整收尾；新轮只写独立runs目录。

| 模块 | 主要实现 | 职责 |
| --- | --- | --- |
| 仿真/策略 | `src/simulation`、`src/control/policy` | Gazebo原生关节力写入、Student45/GRU推理、原速度PID与接触保护 |
| 传感器/前端 | `src/localization/building_sensors.py`、`frontend/fast_livo2_core` | 原源戳LiDAR过滤、视觉惯性/激光ESIKF、连续局部odom |
| 快照桥 | `frontend/fast_livo2_core/src/backend/snapshot_bridge.cpp` | 成功更新后的自有快照，最多4096点，SPSC队列4、spool16 |
| 后端 | `src/localization/backend` | PCD空间索引、局部粗到精匹配、验证、时序准入、关键帧/NPY子图与历史图 |
| 修正所有者 | `src/localization/building_runtime.py` | 同戳pose/cloud使用同一次捕获的修正，唯一map→odom出口 |
| 全局参考 | `src/planning/pct` | 原生3D A*与高度连通支持面，Building多层85.70m参考 |
| 局部规划 | `src/planning/scan` | SCAN曲线及局部占据地图，3D相邻参考/进度约束，避免跨层跳段 |
| 跟踪/停车 | `src/control/truth_navigation.py`、`parking` | SLAM或显式truth输入、曲线跟踪、原停车窗口和保护 |
| 独立验证 | `tools/validation` | 退出状态、接触、关节力、停车与GT离线误差、1Hz CPU/RSS |

地图坐标到Gazebo世界固定平移约`[-.000002,-.000001,-.1]m`。多层出生先验为`[-5.2633744,6.6,.32]m`、RPY零；其它路线按自身起点生成先验。该值是配置先验，不是在线GT测量。当前外参`T_body_lidar`平移`[.2,0,.1177]m`，原点云为LiDAR测量末时刻坐标。

`T_AB`把B坐标映入A。地图配准得到`T_map_lidar`后，`T_map_body=T_map_lidar·inverse(T_body_lidar)`，修正`C_map_odom=T_map_body·inverse(T_odom_body)`。同一修正作用到匹配时间戳的位姿及注册云；后台不持有视觉裸指针，不改前端ESIKF状态。

重计算使用CPU NumPy/SciPy及Eigen原生图求解器。关键帧最短.5s，平移.8m/旋转.2rad或最长2s触发；缺测时不保证2s。自动回环每8关键帧检查，至少分离20帧与实际odom行程5m，两次历史几何一致确认才加边。图最多80活动位姿、8迭代，旧位姿固定；当前驻留历史上限2000关键帧，不声称无限历史或紧耦合滑窗。

无地图时的历史图修正提案与已知地图使用同一结果通道，但无地图真实导航尚未验收。一般地图失跟后撤销localized/停行并重获定位的仲裁未完成；传感器300ms断流保护不等于一般失跟恢复。

Dockerfile/build_support提供公开依赖编译路径。build/install不进Git；runs、原始逐帧日志和录像不进Git。Student自训部署权重已提供，L1仍为本地补充资产，许可边界见根README；不得修改碰撞几何后继承原PASS。
