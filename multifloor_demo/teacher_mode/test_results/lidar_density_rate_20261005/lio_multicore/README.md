# 既有 LIO 残差循环两线程候选

独立 V10 工作区已构建，通过有限合成数值和线程观测检查，尚未做本候选的实际 Gazebo 吞吐或导航验收。此处不把构建成功、单个合成测试或线程存在称为完整数值等价或 30 Hz 可用。

## 修改与安全复核

`navigation/lidar_sampling_v10/slam_ws/src/` 从冻结 V7 普通复制 41 个真实文件，未使用符号链接或硬链接。唯一源差异是 `fast_livo2_core/CMakeLists.txt` 对 `src/voxel_map.cpp` 设置 `COMPILE_DEFINITIONS "MP_EN;MP_PROC_NUM=2"`，没有修改任何 C++。OpenMP 缺失时构建直接失败。保留原 Release `-O3 -funroll-loops`，未加 `fast-math` 或 `march=native`。独立两包采用 sequential / `-j2`，65 秒完成，负载快照保留；没有启动 ROS/Gazebo 或更改旧工作区、模型、camera_mode、训练进程。

`BuildResidualListOMP` 每个 i 只写 `pv_list[i]`、对应 `diagnostic_query_rows[i]` 及局部候选，地图和递归 plane/tree 只读。`vector<bool>` 位写入和 `all_ptpl_list[i]` 都在原 mutex 内。循环结束仍按 i 串行合并；求解与地图更新仍位于原串行阶段，因此未引入并行浮点归约或改变对应排序。VIO 原 `float error` 归约段不启用 MP 宏。

逐项检查了实际 `compile_commands.json`：9 个 core translation units 中仅 voxel_map.cpp 启用两个宏，VIO 及其余 8 个未启用。符号表新增的项目 OMP 段仅为 `BuildResidualListOMP`，没有 VIO OMP 段。已有 Eigen GEMM OMP 模板在新旧库均存在。原 `omp_set_num_threads(2)` 会改变当前主线程后续 OpenMP 默认上限，因此本候选严格保证的是“既有 LIO 显式残差循环两线程，VIO 显式归约循环串行”；不能据此保证所有第三方矩阵/图像操作永久只用一线程。实际调度、Eigen/第三方运行范围需要继续观测。

## 已执行的数值检查

复用原 V7 合成 729 点 fixture，从原已冻结源码普通复制，分别链接冻结 V7 / 新 V10 core，并各运行诊断 logger off/on，共 4 次，3 次实际 LIO 迭代：

- 四份 state25 + cov361 的 386 个 double 输出逐字节相同。
- 8 条日志的 header、残差/查询、迭代/矩阵/状态全部逐字节相同；仅排除了 kind100 的 4 个真实 wall 计时 double（offset9–12）。
- 测试专用 LD_PRELOAD GOMP 观察记录：V10 每次残差循环实际 team=2，两个不同 tid；V7 fixture 无 GOMP 调用。观察版本输出与无观察输出也逐字节相同。此 preload 不进入实际运行。

这是合成几何与真实安装二进制的有限检查。原实际记录未包含从起始建立的完整地图，所以本轮没有用缺失地图拼接结果伪称完整真实输入重放。真实闭环数值、吞吐、导航结果仍未验证。

证据：[构建与实际 flags](build_receipt.json)、[复制与源码 SHA](source_copy_receipt.json)、[有限数值比较](fixture_comparison.json)、[实际 team 观测](observed_team_receipt.json)、[启动源复制](top_copy_receipt.json)。库 SHA256：`5c45dc9d9dc98214bd8e8a85397af468803b19937dd8f24ac5c42e8c02a9902a`。

## 独立候选启动

V10 从冻结 V9 复制启动与 clock hold 逻辑，四处 workspace 引用指向新 V10 安装库。`controller.py`、`clock_hold.py`、`cascade_core.py`、执行桥、Teacher、共享运动/安全数学等保持 V9 原字节。各 profile 只追加 software_multicore / version / prospective 文本元数据，同名 profile 的采样、数值配置、保护、路线、期限和详细日志窗口与 V9 相同。`LIO_MULTICORE_CONTRACT.json`、`multicore_preflight.json` 已纳入每次实际 source scope，binary loader 仍核对 `/proc/exe/maps` 及 SHA。原 V9 clock hold criteria/preflight/amendment 与 V7 diagnostic preflight 原字节保留，表示复用的历史执行逻辑证据；新库有限数值证明另列，未伪装为早已完成 V10 实测。

以下命令由根任务在当前唯一仿真退出后运行，不能和其它机器人控制器共用执行器：

```bash
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v10/run.py --profile l64_r30_c30_210 --label lidar64_30hz_rgb30_lio2_prepare --prepare-only
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v10/run.py --profile l64_r30_c30_210 --label lidar64_30hz_rgb30_lio2_r1 --domain 88
# 若对比诊断记录负载，必须与同名 V9 profile 对照，不混用 50 s / 3 s 窗口：
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v10/run.py --profile l64_r30_c30_detail3s_210 --label lidar64_30hz_rgb30_lio2_detail3s_r1 --domain 88
```

Teacher 仍 CPU 1 线程，50 Hz；物理 PD 仍 200 Hz，控制器 wall tick 仍 20 Hz。实际 SLAM/IMU/点云和 SCAN 用于导航，Gazebo 真值仅离线验收；原 300 ms 有效期和唯一 Teacher 执行器保持。
