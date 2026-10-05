# 64线/30 Hz实验只读评估审计

所有结果只在实际进程退出、`worker_result.json`生成、诊断writer `final=true`且排空后形成最终结论。运行期间索引监看只能提供预览，不能提供完整性或导航通过。

## 保持的正式判据

`test_results/closed_loop_navigation_20261005/acceptance_audit/evaluate_closed_loop.py`与`evaluate_ramp.py`可直接复用。它们按实际原始消息时间戳与本次冻结`sensor_contract.json`核验，不假定32线或10 Hz。正式区域到达、原始SCAN曲线回放、300 ms双时钟TTL、IMU因果关联、200 Hz物理采样、唯一执行器、原先路线与安全阈值均不放宽。有限210 s前缀即使穿过原故障窗，也不能替代600 s/32个区域/完整双坡道/停车验证。

正式common包含`exclusive_Teacher_CPU_identity`，本轮三个传感器对照继续CPU单线程。独立GPU推理基准不能变更common判据，也不能被当作GPU已实际导航通过。

正式收据使用本run预先冻结的common源；可追加`sensor_experiment`收据后逐字节复制为浏览器识别的canonical收据，已有canonical文件必须校验相等，不能覆盖不同旧结论。

## 原只读分析器适用性

- `verify_diagnostics.py --run NEW_RUN`会从本run的`navigation_source_snapshots.json`取三个冻结schema并核对SHA，可复用。115–165 s详细窗与32 MiB单record上限来自未改V7记录器，不是新传感器验收门槛。
- `analyze_lio.py --records NEW_RUN/fastlivo_diagnostics/records.bin --schema FROZEN_LIO_SCHEMA --out OWN_OUTPUT`支持所有实际iteration/header，不依赖10 Hz；必须显式指定新run冻结schema，不能将跨run的plane ID看成同一实体。
- `analyze_vio_diagnostics.py`重建依赖实际`dt`、H、协方差，未要求camera10 Hz。比较应报告实际VIO更新数量/频率，不能单独从lidar配置推断滤波器频率。
- 原`physical_height_analysis.py`硬编码无故障时`ft=156.4`、特定窗口和图x轴。它适合旧失败复现，不能作为新无故障210 s结果的主统计。本目录`analyze_sensor_experiment.py`支持实际首guard时间、没有guard时末5 s，以及原131–134 s共同窗口；全时段图与原故障窗图都保存。
- 原`analyze_runtime_health.py`数值计算依赖实际header可复用，但文本将运行限定为210 s，应在600 s验证时另写准确说明。共享GPU利用率不是Teacher独占利用率，top-level PID CPU不是整个SLAM/Gazebo进程树。

## 新脚本的来源与边界

`analyze_sensor_experiment.py`分别记录：冻结SDF/契约配置的光束数、实际FAST-LIVO PointCloud2有效输入点数、详细窗口预处理数、首iteration降采样数与有效约束数；消息callbacks、LIO、VIO、SLAM发布、controller ticker与真正数学更新频率分别呈现。30 Hz雷达可能因RGB10 Hz同步仍只有10 Hz LIO，这必须实际记录。

native `t-dt`、原始位姿、关节/接触只做离线物理核验；本次冻结yaw注册和末次静态SLAM/native配对只用于离线相对高度与面几何分类，不回写导航。

面质量：详细窗口每个整数仿真秒选取第一个首iteration记录；使用本run literal SDF `wall_18.5`真实竖墙、实际native机身姿态/实测lidar外参把`point_lidar`映射到场景，统计距竖墙6 cm内被实际接受的点所用法向误差、旧面mid/max谱、近水平假面数量。6 cm、60°、mid/max0.05均为诊断分类，绝不是更改导航或匹配门槛；点是估计器已去畸变后的点，保留扫描运动与估计去畸变的不确定性。它不是原始点云重放，也不是修复轨迹。

记录器没有初始化0–10 s map-event明细，不能声称看到了完整初始面创建重放。kind101实际接受的匹配面含历史谱/创建/最后更新元数据；kind103在115–165 s内可记录实际创建/refit/freeze事件。跨run只比较统计与场景几何，不比较literal plane ID。

## 操作

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/evaluation/analyze_sensor_experiment.py \
  --run multifloor_demo/teacher_mode/runs/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf \
  --run multifloor_demo/teacher_mode/runs/NEW_RUN \
  --output multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/evaluation/comparison
```

该脚本不导入ROS/Gazebo/policy，不发送控制命令，不停止进程，不修改输入run，不提供新的正式PASS门槛。
