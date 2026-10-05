# Teacher 串级控制、实际 SLAM/SCAN 与多层坡道验证

本报告接续 2026-10-05 的真值控制器标定。导航反馈改用实际传感器 SLAM 位姿、因果 IMU 陀螺与实际 SCAN 路径；Gazebo 真值只用于离线物理验收。Teacher 仍有 232 维特权状态/高度输入，另 15 维为命令及上一动作；本批没有将此前独立通过的 IMU/关节 30 维替换与导航联测。没有重训、操作实体机器人、修改 camera_mode 或停止其他 RL 任务。

## 已取得的结果与保留的失败

| 测试/版本 | 结果 | 实际证据与边界 |
|---|---|---|
| 串级实际 SLAM/SCAN 平地 3 轮 | 2 通过，1 失败 | 54a1、5011 的 18 项共同门通过；a9a2 首个固定停车窗 XY 漂移 60.75 mm，超过 50 mm，不能选后续稳定窗覆盖 |
| V1/V2 长目标坡道 | 失败 | 首先修正场景轴注册；V2 55f6 暴露父 HeadingGate 与局部路径朝向不同，Teacher 停在坡中；另有 0186 未启动 Actor 的构造参数失败 |
| V3 同朝向坡道 | 失败 | 2d63 解除了朝向冲突，但 SCAN 的长程曲线路径偏离事前场景路线管道 0.450151 m，超过 0.45 m；控制器没有证明已完成 12 m |
| V4 1 m 分段单坡道 | **通过，单次** | 4791 实走完整 ramp12、14/14 原 SLAM 区域、12 个接触分段、四足有序坡面/出口支撑、正式首 5 s 停车通过；正式 ramp 18/18 与 heading 8/8 |
| V4 连续三层双坡道 | **失败** | d42e 完成 16/32 区域及下层完整 12 m；connector_y 的高度差 −216.49 mm，原 ±100 mm 门拒绝到达；上层坡道与最终停车未完成 |
| V5 在线重力/偏置估计 | **失败** | b182 仍完成 16/32，connector_y 高差 −222.60 mm；429.33 s socket 故障后发生阻尼塌落，完整运行、安全和导航失败均保留 |
| V6 视觉协方差 1000 对照 | **失败，正式审计完成** | 9cc6 在原 SLAM 头 131.7 s 高度管道误差 306.505 mm，超过原 300 mm 管道保护并停车；实际完成600s/30001次推理、worker无fault、全部自有角色/子进程清理；正式共同/ramp FAILED、heading 8/8 PASS，运行完成不提升导航失败 |

上表的三层连接均是 **12 m 坡道**，不是实体楼梯。真值控制器此前的 35 次运动通过、旧开环失效和本批实际 SLAM/SCAN 结果是不同作用域；新 Gazebo 闭环没有做对应 Isaac 闭环重放，因此不提升完整 Sim2Sim 或真机结论。动态障碍的历史有限合同已有通过记录，本批新串级多层动态恢复尚未测试。

### 4791 单坡道正式数值

COM 入坡 13.110 s，越过坡尾并进入出口余量 98.555 s，四足接触逐脚与 12 个 1 m 区段均通过。最大 COM 横向偏离 32.87 mm，最长无足支撑 10 ms，出口机身原点净高至少 0.296888 m。路线误差 max 0.135718 m / RMS 0.031454 m，朝向 max 0.102358 rad，COM 速度 MAE 0.042167 m/s。

停车使用**第一次实际被执行器消费的固定 5 秒窗** 112.465–117.465 s，1001 条 native 与 50 条 SLAM：XY 漂移 4.5475 mm、yaw 0.00264278 rad、机身原点速度峰值 0.0125513 m/s。初筛用 source 时刻得到的 3.6 mm 不替代这个正式结果。

## 控制与来源

- `cascade_core.py`：有向最近路径投影与单调进度，横向/朝向 PD、COM 速度/角速度 PI；输入实际 SLAM 及因果 IMU，位姿头约 10 Hz。20 Hz 发布心跳不是新反馈，Actor 50 Hz 也不是定位 50 Hz。当前没有 IMU 位置插值形成新位置反馈。
- `controller.py` 与 `transition_gate.py`：路线捕获、停车过渡、朝向统一、实际障碍/SCAN 检查。停车继续运行 Actor 并允许有界纠偏命令，不能将 action=0 解释为停车。
- `bridge.py` / `worker.py` / `sensor_gate.py`：唯一速度输入桥与源收据，双时钟 300 ms 新鲜度、因果 gyro 配对 ≤20 ms、异常锁定；Actor/native socket 完整帧另有 200 ms 墙钟门，两门不混淆。
- `route.py` / `route_fence.py`：原 SLAM5 已知静态地图几何，加初始 spawn XY 先验（±0.15 m）及实际 SLAM/IMU 配对注册；不是自动全局点云地图配准。高程为初始实测机身基准加地图层高，未使用导航真值，也未用错误当前 SLAM Z 改目标。
- `terrain_provider.py`：Actor 的下层/上层特权高度扫描，在实际 SLAM connector_mid 3D 到达后才申请切换，并核对 187 射线公共平台一致性。它不改变真实 Gazebo 地面、SLAM 激光或导航位姿。

固定策略 SHA256 为 `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`，CPU 单线程。关节 PD 200 Hz、策略 50 Hz、物理步长 0.005 s；训练关节顺序显式映射，kp=25、kd=0.5、动作比例 0.25，力矩/速度原限幅不变。

## 事前判据及独立复核

共同门保持路线 max ≤0.20 m / RMS ≤0.08 m、朝向 ≤0.20 rad，COM 速度 MAE ≤max(0.05 m/s, 0.25×参考速度)；原 3D 到达半径 0.22 m、控制半径 0.17 m、高度 ±0.10 m、连续 0.6 s、每目标 90 s。已知路线管道横向 0.45 m / 高度 0.30 m；单坡道和全三层均不因失败放宽。

安全门包括 roll/pitch ≤0.65 rad、机身净高 ≥0.18 m、机身接触 0、关节力矩 ≤23.50001 Nm、关节速度 ≤30.001 rad/s。停车首 5 s XY ≤0.05 m、yaw ≤0.1 rad、原点速度 ≤0.08 m/s、实际 yawrate/body-wz ≤0.1 rad/s。

`evaluate_closed_loop.py` SHA `074b468581de568264f558e38f82ce25e809094843c6f86e2816130babf8099e`、`evaluate_ramp.py` SHA `d7024636a84e59dfcc3d5e94b8cc79cedfb1b6cec85cdac2babad23ab2e17b89`、heading source verifier SHA `c93ff2df891bdf8784b487d5016b450c7c5ff72186e75233239b9ba8c4bd3ac2` 均冻结，按原文件追加收据。运行完成与模型加载不等于通过。

V5 heading 原严格来源门另有两行 locked/current 相差 0.000618147 rad，超过来源一致性 1e-8 门，虽远小于运动门也保持 FAIL；不能据此归因 22 cm 高度误差。V4 heading 8/8 来源通过。ramp 评估器对 provider 包装状态的全字典比较另产生 UNVER：原 74 字段全部一致，但 observer 多两个时间元数据。独立附录证明来源一致，原 UNVER/FAILED 收据没有改写或升级。

## SLAM 高度故障与本轮对照

d42e 的 130–140 s 高度下降集中发生在二层平台转向/前进阶段，物理机身 Z 基本不变；末端误差严格分解约 −203.49 mm 定位误差、−18.95 mm 支撑净高变化、+5.95 mm 固定坐标倾斜。LIO 更新也注入向下速度，不能定为纯 IMU 积分故障。

初始化接受 605 样本/3.02 s，但接受窗口仍慢转 2.1°，是低运动而非完全静止。旧视觉协方差 1e8 使每像素信息比默认 100 弱 1e6 倍；已有局部灰度 patch/参考观测管理，彩色点云发布不是 color ICP，也未发现全局回环调用链。V5 flags 与 V6 视觉权重都是单独配置对照，不能预设唯一根因。详见[初始化与匹配审计](SLAM_INITIALIZATION_MATCHING_AUDIT_20261005.md)、[更新阶段原曲线](../test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/DIAGNOSTIC.md)、[两轮真实点云](../test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/compare_b182/final/REPORT.md)。

真实点云局部地面仍有约 6000 点、5.5 mm 拟合残差，点云与已知层高能观测法向漂移；该纯离线观察器尚未融合上线，不能据其拟合结果宣称原 LIO 内部关联正确或完整 6DoF 可观。下一步所需证据是实际使用的平面对应、拒绝原因、残差、信息矩阵、状态协方差与各阶段增量；回环/关键帧全局后端须另行验证。

## 运行、查看与复核

在 SLAM 项目根目录运行会创建新的独立 run。只启动自己的 ROS domain 86 / GZ partition，不连接真机，不停止 RL。运行器记录资源与所有进程，退出只清理其自有 5 角色/11 子进程。

```bash
# 已通过的单坡道配置；另跑仍须独立验收，单次旧通过不保证复跑通过
python3 -B multifloor_demo/teacher_mode/navigation/closed_loop_multifloor_v4/run.py --profile ramp12_segmented_turn06 --label ramp12_repeat --domain 86

# 最新仅恢复图像协方差的完整路线实验；当前已有失败，不是推荐部署配置
python3 -B multifloor_demo/teacher_mode/navigation/closed_loop_multifloor_v6/run.py --profile multifloor12_23_segmented_turn06 --label visual_cov1000_repeat --domain 86

# 每个新run自己的归档验收器；保留原冻结合同，不覆盖已有收据
python3 -B multifloor_demo/teacher_mode/runs/新run/sources/acceptance/evaluate_closed_loop.py --run multifloor_demo/teacher_mode/runs/新run
python3 -B multifloor_demo/teacher_mode/runs/新run/sources/acceptance/evaluate_ramp.py --run multifloor_demo/teacher_mode/runs/新run
```

验收器实际归档路径以该 run 的 `sources/` 和 `ramp_acceptance_source_manifest.json` 为准；禁止覆盖已生成的收据。可查看[单坡道正式通过](http://127.0.0.1:8768/?run=20261005_145244_closed_loop_cascade_segmented_ramp12_r4_4791)、[V4 多层失败](http://127.0.0.1:8768/?run=20261005_145640_closed_loop_cascade_actual_multifloor_r1_d42e)、[视觉权重实际对照](http://127.0.0.1:8768/?run=20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6)。两路画面都是实际 Gazebo 图像；D/K 与原始时戳单独展示。

逐轮 `runtime_manifest.json`、`navigation_scope.json`、`source_manifest.json`、模型清单、原 telemetry/native、源 SLAM/IMU/cloud/SCAN 数组及独立收据在 run 目录。报告全量包清单需在本轮所有物理/审计/文档写入停止后更新，旧包清单保留历史字节。

## V6 最终独立收据与资源

V6共同/ramp正式FAILED，heading来源8/8 PASS（实际76条align locked，最大差9.4189e-12rad）。全200Hz原生安全、实际source/SCAN/PI/guard/TTL、CPU/完整运行、路线门通过；原3D全区域和最终停车没有通过。安全minclear0.275687m、RPmax0.113975rad、bodycontact/fault均0；路线max0.141601m/RMS0.034814m/heading0.121823rad，COM速度MAE0.038538m/s。lower12完整接触七专项通过，上层未通过。详细 [V6独立失败附录](../test_results/closed_loop_navigation_20261005/online_SLAM_AB/V6_FAILURE_APPENDIX.md)。

共同SHA `b875631e4fb33701429623e8da072960ce9861b1e873839dc43aaf2239d38583`；ramp SHA `8384975078c81d3f202fcdf9f580c1f144939b2caddeb345b5c128ff1426c2d6`；heading SHA `ab4e5fc106dad5c143e6e7174d9fc188e7752b502181d7113ca650985b4a1f04`。附录SHA `18ea41a8301d059c37290dfa8f86bf2c8a95875018831232e36275e703355cef`。

V6开始/结束GPU占用5285/4362MiB（总16303）、util62/60%，20核load1为8.07/8.11，MemAvailable约46/47GB；原600仿真秒实际约615.9墙钟秒。Actor CPU单线程，每run保留forward_ms分布。其他017任务持续运行，无信号/参数修改，不能将该高度跳变预设为GPU独占竞争。V5的socket故障只有IO合并code3，forward快不足以排除日志/调度整帧延迟，其200ms故障原件保留。

包级来源/收据清单为 `test_results/closed_loop_navigation_20261005/campaign_evidence.json`；全包文件清单为 `PACKAGE_MANIFEST.json`。这些清单是归档身份，不能代替失败的导航验收。
