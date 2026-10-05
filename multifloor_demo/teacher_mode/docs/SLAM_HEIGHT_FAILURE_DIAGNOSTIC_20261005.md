# SLAM 二层高度失效：实际 V7 诊断重跑

本轮完成了 **210 秒实际 Gazebo / Teacher / FAST-LIVO2 / SCAN 闭环重跑**。证据支持的主要故障链是：**初始远墙的单扫描线簇被拟合成近水平面，地图按点数停止更新；靠近墙后，真实竖墙点仍使用旧错误法向，LIO 向下修正位置和垂直速度；视觉相关方向约束变弱，负垂直速度积分使估计高度快速下降。** 本轮机身实际只下降毫米量级，没有跌倒。已有证据比“PID反馈太慢”或“GPU竞争”更直接地指向前端地图几何和匹配质量。

实际运行：[20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf](../runs/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf/)。[浏览器真实画面与 SLAM/SCAN 路线](http://127.0.0.1:8768/?run=20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf)显示两路存档 Gazebo RGB、原始相机 K/D、真实传感器轨迹及独立 **FAILED** 收据。存档不声称实时。

## 运行边界和首个故障

- 使用独立 [V7](../navigation/closed_loop_multifloor_v7/README.md) 源码、构建与安装目录；只增加有界异步诊断。未改原 core/ros、原安装库、V1–V6、camera_mode 或另一任务的训练/评估。冻结 Teacher 保持 CPU 单线程、50 Hz，唯一关节执行器及 200 Hz 基础 PD 不变。
- V6 原控制器、路线、场景和判据保持；仅把本次生命周期从 600 秒缩为 210 秒，覆盖失效前后。路径高度门仍 300 mm、来源超时仍 300 ms。没有降低门槛，未用 action=0 停车。
- 导航命令、路线反馈和区域到达来自实际 SLAM/IMU/点云及 SCAN；Gazebo native 位姿只用于离线物理/几何核对。Actor 仍使用 **232 维特权输入＋15 维命令/上一动作**；此前 IMU/关节 30 维替换通过不等于与本轮联合通过。
- 第一条执行桥保护为 **clock 133.13 s，SLAM source 133.099999999 s**：路径高度误差 **303.483 mm**，水平误差 **5.145 mm**，处于路线第16段。此前看到的156.465 s是后续 failed 状态，不能作为首次故障时间。
- 正式共同收据为 [diagnostic_prefix_r1](../runs/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf/summary_closed_loop_cascade_independent.diagnostic_prefix_r1.json)：19项中15通过、2失败、2未验证，**16/32 区域到达，整体 FAILED**。页面标准名收据是同字节展示副本，另存关联证明；不覆盖历史收据。210秒完整退出不代表600秒完整任务通过。

## 实际机身与估计高度分开核对

统一窗口使用源时间 **(131,134] s**。native 200 Hz记录600条，所有真值分析均离线。SLAM发布 body odometry 在 LIO 之后、VIO 之前；内部 IMU 状态则记录到 VIO 之后，不能把两个端点混用。

|来源与量|本窗口实际结果|含义|
|---|---:|---|
|native 机身世界 Z 净变化|−3.668 mm|没有实际下沉30 cm|
|native Z 最大跨度|11.840 mm|正常步态起伏量级|
|native roll/pitch 最大绝对值|0.920° / 1.786°|未见该时段剧烈倾覆|
|body contact / actuator fault|0 / 0|没有机身碰地或执行故障|
|发布 SLAM body Z 净变化|−330.971 mm|导航看到的高度下降|
|内部 IMU 状态 Z 净变化|−336.545 mm|下表积分重建的状态来源|

[物理对照完整数据](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_physical/summary.json)、[200 Hz native轨迹](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_physical/native_trace_200hz.csv)、[SLAM轨迹](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_physical/slam_body_trace.csv)。

![实际机身与估计高度、速度、姿态](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_physical/actual_height_velocity_attitude.png)

## 高度下降如何进入状态

从保存的每一步原始 IMU、旋转、bias、重力、P 和 LIO/VIO 更新重建，同一窗口的内部状态分担为：

|状态更新来源|累计 ΔZ|
|---|---:|
|IMU传播中已有垂直速度的积分|−323.481 mm|
|IMU传播中当步加速度的 `0.5*a*dt²`|−0.0247 mm|
|LIO位置修正|−79.444 mm|
|VIO位置修正|+66.405 mm|
|合计|−336.545 mm|

LIO向下的速度修正经正常的 ESIKF 位置—速度协方差耦合进入 `vz`。一度估计 `vz≈−0.399 m/s`，其间不断积分；随后速度回到近零也不会自动追回已丢失的高度。表中当步加速度项小，**不等于排除所有此前加速度、姿态、bias、初始化或相互耦合的影响**，也不能据此认定IMU无用。它明确了本窗口直接高度变化的来源。

原始 IMU 源时间200 Hz、5 ms间隔，没有倒序或重复。该窗口加速度模均值约9.826 m/s²，陀螺峰值约0.472 rad/s，结合native姿态未发现能独立解释30 cm下沉的异常冲击。规范分析为 [actual_r1_vio_v2](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_vio_v2/summary.json)，[窗口明细](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_vio_v2/failure_window_131_134.json)。初版 `actual_r1_vio/` 的重建不通过源于分析器对原 Exp 死区和协方差阶段的假设错误，已保留并纠正；它不是运行算法损坏的证据。

## 具体几何错误：竖墙用了水平面法向

独立坐标审计解析本次实际 `world.sdf`、sensor/extrinsic 和 native 姿态；LIO 的 `camera_init` 经过冻结yaw与初始化配对平移的**离线**对齐，未进入控制器。10个可疑面的具体实体均为 `wall_18.5` 的室内面 **world x=18.4 m**。132 s实际 LiDAR接受点转到world后落在x≈18.386–18.423 m，位于墙的有限y/z范围内；这些点不是台阶、坡面或地板。

plane871的历史法向为 `[-0.0529,-0.00146,0.9986]`，接近水平面的z法向。当前接受的9点重新做PCA得到 `[0.9970,-0.0318,-0.0699]`，接近真实竖墙的x法向，二者绝对点积仅 **0.1225**。plane872等具有同类冲突。132.0 s首迭代3369条接受约束中，546条法向近水平；其中209条独立核对属于真实东墙，净向下 `Δvz=−0.030867 m/s`，占近水平组净向下影响约 **98.02%**。

成因有相连的源码与采样证据：

1. 实际LiDAR为32×480、竖直−30°至15°、10 Hz，范围噪声stddev=8 mm。初始距东墙约17 m，相邻扫描环高度间距约43 cm，接近0.5 m叶格边长；叶格可能仅含单个扫描环。近线簇不能可靠确定二维面的法向。
2. `voxel_map.cpp::init_plane`仅检查最小特征值小于0.005 m²，没有启用二维切向支撑条件。历史 `λmid/λmax≈0.0037–0.0173`，第二切向方向很弱；并非数学严格rank1，但法向不代表可靠表面方向。
3. 关键面出生约3.8 s，经过8次更新，最后refit在5.7–6.0 s；达到50点上限后停止更新。**不是一出生立即冻结。** 132 s近墙束间距约6.1 cm，本已有多环墙面信息，旧模型却不能吸收。
4. 大的旧plane协方差使错误方向点仍通过3σ门。871中位candidate sigma≈0.006048 m²，其中plane项≈0.006010，query项仅≈0.0000387；因此不能归因为当前位姿协方差过大。通过后的权重仍足以累计产生偏置。

解析射线与独立8 mm噪声示例解释了“单环最小特征向量可转向z”的机制；它们不是原随机噪声重放，也不是修复A/B结果。[独立实体/采样报告](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_plane_geometry_audit.md)、[JSON完整证据](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_plane_geometry_audit.json)、[LIO逐迭代报告](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_lio/LIO_MATCHING_FAULT_REPORT.md)。

![历史模型与当前真实点簇法向冲突](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_lio/frozen_plane_current_geometry.png)

## 限定重算确认贡献，未宣称在线修复

固定原P、state、vec、实际H/残差/权重，仅移除有明确历史/当前法向冲突且当前二维支撑足够的组，重新求解M、K、G，得到：

|实际首迭代|移除点数|原 Δvz|重算 Δvz|原 Δpz|重算 Δpz|
|---|---:|---:|---:|---:|---:|
|132.0 s|119/3369|−0.030664 m/s|−0.011839 m/s|−8.797 mm|−3.601 mm|
|132.3 s|153/3619|−0.023715 m/s|+0.002667 m/s|−7.599 mm|−0.322 mm|

这是实际约束的单迭代反事实，不是将某些贡献简单相减。原解重建误差≤1.4e−16。它支持错误墙面组的实质作用，尚未重建新地图、重跑后续传播或完成新闭环；各帧不能拼接为“修复后轨迹”。事后阈值也不能直接用于生产：真实地面扫描弧同样可能有很小的mid/max。[完整限定重算](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_lio/single_iteration_counterfactual.json)。

## 视觉、初始化、时序和资源分别判断

VIO没有完全失效：有效视觉点中位数从125–130 s的44.5降到131–134 s的24，仍在向上纠偏。局部高度—俯仰—前后位置的耦合方向明显变弱；132.4–132.5 s高度条件信息低于此前。LIO同时仍有数千约束和强信息，**不是完全失去高度可观测性**；错误几何也可产生数值满秩且很强的信息。

实际配置 `normal_en=true`，LIO的获胜plane法向经`pv.normal`→`VisualPoint.normal_`→homography进入VIO patch warp，存在潜在放大路径。但 `raycast_en=false`，`updateReferencePatch()`在冻结流程中没有调用，且现有日志没有trackID—planeID链；不能把全部视觉弱化、深度拒绝或旧track法向归于当帧假面。

因此本轮更具体的“初始化问题”是**早期稀疏墙面建图质量与冻结**，而非已证实狗刚启动运动造成IMU严重初始化失败。延后运动可改善初态，但不能代替地图持续吸收不同视角、判断真实二维支撑的机制。

本次详细窗口估计阶段最大18.804 ms，SLAM/点云回调源仿真龄最大30 ms；控制器读取位姿墙龄最大172.04 ms、配对IMU墙龄199.78 ms，均在300 ms内。原IMU两个>300 ms墙钟间隔仅在0.01–0.02 s启动期；3.7 s后最大89.516 ms。CPU单次推理p95≈0.329 ms。GPU峰值1146 MiB/40%，20核机器load1峰值4.56、可用RAM至少47.52 GiB。**低负载仍能复现故障，资源竞争不是这次失败的必要条件**；未做相同条件禁用logger对照，不能报logger独占开销或声称GPU从无影响。[运行健康数据](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_runtime_health.json)、[准确首次保护与启动间隔补充](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_runtime_addendum.json)。

另发现fallback neighbor把无量纲 `p_world/voxel_size` 与米制center/quarter_length比较的确定量纲缺陷。132.0 s实际采用fallback仅2点，直接Δvz很小；可能遗漏更好的补偿关联，需独立A/B。它不是当前主要墙面误约束的唯一解释，日志中的大量尝试次数含不同迭代重复，不能称独立点数。

## 下一轮修复优先级和验收

优先在独立V8处理 **平面二维支撑/法向可信度、近线模型延迟成熟、冻结模型遇到可靠新视角的有界重估**，并保留正常地面和坡道约束。先用采样/噪声和正常面证据制定规则，不盲用本报告事后mid/max阈值，也不硬钳制机身高度、关闭正常p-v耦合或放宽导航保护。neighbor量纲修复另立单变量对照，便于区分贡献。

每个候选仍用同路线、初始化、controller和原门：记录地图模型健康、实际接受/拒绝组、状态分担与完整退出，先确认210 s前缀不再失效，再跑完整600 s/32区域、双坡道接触几何、首5 s停车及后续动态障碍。基线旧失败永久保留。此时才判断视觉管理是否还需增强及后端回环/位姿图优化的净收益。

core负责模型质量、关联与健康指标；ROS层负责参数、诊断传输和odom/map契约，导航层消费连续局部odom并处理健康状态。后端回环解决累计漂移与全局一致性，不能替代进入当前滤波器的错误面约束；本轮没有用真值导航或移植后端来掩盖失效。

## 完整性、哈希与复现

新core/ros两个包构建通过。日志完整159757/159757条、0丢失、无IO失败，原始二进制6912621864 bytes；LIO8885次迭代、IMU41280个传播对、VIO14319次实际解重建通过。[日志完整性](../navigation/closed_loop_multifloor_v7/analysis/actual_r1_integrity.json)、[冻结源清单](../runs/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf/source_manifest.json)、[实际加载库证明](../runs/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf/slam_loaded_binary.json)。合成fixture logger开/关状态和P逐字节一致，不等于实际全运行时序逐字节一致。

|对象|SHA256|
|---|---|
|冻结 model_1000.pt|bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34|
|实际诊断 records.bin|f11b6d9be79a7cc6fd78e34d820bb8993be581e9eecbccb9a087d4cf964288a5|
|诊断 index.csv|0cc817d4c0f66e18e4cafe5c4900d17bf4cc2dbc5c5ecd617625a48f239edbc7|
|实际加载新 libfast_livo2_core.so|4f0f0727171c8c7185073f5757a3ef80aeb6dffa70bff61217260426a0b58107|
|实际加载新 fastlivo_mapping|defed90c382bc7b81059493ef871ead4b2c5f4db7ad3fac22a0c52fbd03b6997|

从项目根执行下列命令。复算输出使用新目录，避免覆盖本轮派生结果；运行命令会另建run。

```bash
task_v7=multifloor_demo/teacher_mode/navigation/closed_loop_multifloor_v7
task_run=multifloor_demo/teacher_mode/runs/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf
python3 -B "$task_v7/analysis/verify_diagnostics.py" --run "$task_run" --output /tmp/go2_v7_integrity_repeat.json
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B "$task_v7/analysis/analyze_lio.py" --records "$task_run/fastlivo_diagnostics/records.bin" --out /tmp/go2_v7_lio_repeat
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B "$task_v7/analysis/analyze_vio_diagnostics.py" "$task_run" --output /tmp/go2_v7_vio_repeat --window 131:134
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B "$task_v7/analysis/physical_height_analysis.py" "$task_run" --output /tmp/go2_v7_physical_repeat
python3 -B "$task_v7/run.py" --profile multifloor_diagnostic210 --label height_diag_baseline_repeat --domain 87
```

**验收仍分层**：接口通过；已有运动及5/10 cm功能登台通过，本轮210 s物理安全通过；完整Sim2Sim未通过，新controller的匹配Isaac闭环对照未验证；有限真实SLAM/SCAN及单次完整12 m坡道已有通过，完整32区域多层导航本轮仍失败；真机未验证。三层连接是坡道，不能称真实楼梯通过。旧5 cm机身升高比例失败不能再表述为登不上台阶。
