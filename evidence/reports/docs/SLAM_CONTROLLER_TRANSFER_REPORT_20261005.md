# Teacher 控制器向实际 SLAM 的迁移记录（2026-10-05）

实际 SLAM V5 的 6m 单程三次、6m＋6m 往返三次均通过独立验收，共 132/132 门。这里通过的是当前平地、相对机身固定路线、0.3m/s 参考速度的仿真迁移范围。控制所用定位和到达判定来自实际 SLAM 原始消息，未使用 Gazebo 真值导航；Actor 仍有 232/247 维特权观测，尚未完成全传感器部署。

本记录只整理已完成的 [六次汇总](../test_results/slam_transfer_v3_20261005/v5_aggregate/REPORT.md) 和原收据，没有重新运行试验、重算验收或修改旧结果。参数选择与真值校准另见 [TRUTH_PID_CALIBRATION_REPORT_20261005.md](TRUTH_PID_CALIBRATION_REPORT_20261005.md)。

## 实际结果和门限

| 路线 / run 尾号 | 实际 COM 均速 m/s | 路线最大 / RMS m | 行驶朝向峰值 rad | 固定 5s 停车 XY / yaw |
|---|---:|---:|---:|---:|
| 6m 单程 / 8dda | 0.296490 | 0.025518 / 0.009610 | 0.054990 | 0.014063m / 0.080430rad |
| 6m 单程 / ad47 | 0.296561 | 0.019609 / 0.008787 | 0.054990 | 0.009438m / 0.083804rad |
| 6m 单程 / 8bf5 | 0.296715 | 0.025664 / 0.009130 | 0.055208 | 0.007801m / 0.069798rad |
| 6m＋6m 往返 / e372 | 0.293541 | 0.027575 / 0.008600 | 0.066220 | 0.008896m / 0.045306rad |
| 6m＋6m 往返 / 63d3 | 0.293474 | 0.035993 / 0.008410 | 0.069005 | 0.006762m / 0.059709rad |
| 6m＋6m 往返 / 2e3f | 0.293695 | 0.035678 / 0.009464 | 0.070729 | 0.003627m / 0.006272rad |

六次均保持事前门限：路线最大/RMS≤0.20/0.08m，行驶朝向≤0.2rad，速度 MAE≤max(0.05m/s, 0.25×参考速度)，末目标≤0.15m并以不同原始 SLAM 消息头连续 dwell 0.6s，固定首个合格停车窗 5s 的 XY/yaw 漂移≤0.05m/0.1rad。安全门为 RP≤0.65rad、离地≥0.18m、机身接触和 fault 为0、力矩施力命令≤23.50001Nm、qd≤30.001rad/s。

全组 RP 峰值 0.113963rad、最低离地 0.275679m、最大力矩施力命令 14.623962Nm、最大 qd 13.716193rad/s，无机身接触或 fault。力矩字段是送入 physics 的施力命令，不能称为测得电机力矩。每个固定停车窗均有原生 1001 帧、Teacher 251 帧、实际 SLAM 50 个不同原始头；六个主进程和六个 SLAM 子进程均正常退出。150 项原输入哈希匹配，旧收据保持原样。

## 25Hz 校准向实际 10Hz 新源的迁移

所选结构为 `cascade_pi`，选定校准频率为 25Hz。迁移使用同一冻结 core、gains、滤波、限幅和 command slew，显式接受实际 `/demo/slam/body_odom` 新消息约 10Hz：只有新原始 pose 触发数学更新，dt 来自源时间差；Teacher 仍以 50Hz 推理，native PD 为 200Hz。不能把 heartbeat、缓存控制行或停车/初始化行算成 25Hz 实际 SLAM 更新。

本轮实际数学更新率六次均约 10Hz。部署需明确传入 `--allow-source-rate-reduction`；未传入时，runner 拒绝将所选 25Hz profile直接迁移到 10Hz SLAM。这六次验证覆盖了当前场景和速度下的显式降率，不保证其他速度、延迟或路线形状同样通过。

## 定位、路线和来源边界

路线在 `camera_init` 中由首次合格的实际 SLAM 机身原点与 yaw 一次锚定，随后固定为前方 6m，或前方 6m后返回初始原点。命令、原始 pose、因果 gyro 和实际 Teacher 读包逐项关联；gyro 不晚于所配 SLAM pose，控制所用 pose 不晚于 Teacher native 状态时钟。原传感头和接收墙钟不刷新，未把重复 pose 计为新反馈或到达 dwell。

六次合格包没有未来反馈或 TTL 违规。SLAM/IMU 仿真龄峰值均为 0.21s，墙龄峰值分别为 0.189026s / 0.212106s。producer `/clock` 与 native 的已声明相位容差仍为 50ms，并单独报告；它不是允许使用未来 SLAM pose 的理由。首次合格读包后的命令拒绝为0；初始化预热拒绝保留在原记录中。

原场景绝对中心线及旧地图尚未注册，这一 body-relative 路线不能当作旧地图导航完成。离线 native 与 SLAM 的一次对齐只用于运动误差评估，不产生控制命令、不选择目标、不调整路线。Actor 输入仍包含 232 维 Gazebo 特权状态和高度扫描，另 15 维为速度命令3和上一动作12；高层使用实际 SLAM/IMU不代表 Actor 已完成真实输入替换。

## V4、V5和历史失败

V4 存在读取时钟竞态：consumer 提前取墙钟，而 producer 可能在随后读取原子 JSON 前发布新版本，导致该版本的收到墙钟看起来晚于 consumer 的旧读取时刻。V5 只改为实际读完该版本后计算墙龄，并保存 `read_begin`、`read_completed`、`read_span`；worker记录该实际读包时刻。原始头、接收墙钟、controller、Teacher、physics、runner和参数均不变。双 TTL 仍为 300ms，producer-clock容差仍为50ms，没有放宽新鲜度门限。

六次 V5 独立通过与七次 V4 原成功/失败收据分别保存，不合并统计或回填 V4 的停车失败。启动失败 2e0e、6a9f、4b68 也保持原失败状态：早期 Node context 冲突及布尔/序列化问题属于接口诊断，不能计为运动通过。

实际 ROS 元数据已纠正：保留的七次 V4 和六次 V5 共 121301 条原始 IMU 记录均为 `orientation_available=true`、`orientation_covariance` 九项全0。故这些实际试验没有验证“姿态 unavailable 但 gyro 有效”分支；该分支目前仅有纯离线测试。V2只保存拒绝记录，没有原始 IMU，不能用静态 SDF 的 −1 推断当时实际 ROS 姿态不可用。V2 的比较表达式可能产生 `numpy.bool_`，与 `is not True` 身份检查不兼容；V3 又出现 NumPy 布尔不可序列化。类型处理缺陷有源码和报错支持，但缺少 V2 raw 时不能认定唯一根因。详情见 [IMU 来源说明](../test_results/slam_transfer_v3_20261005/v5_aggregate/README_IMU_SOURCE.md)。

## 操作命令与结果页面

以下为当前 runner 的显式执行接口，命令会新建 run并启动仿真；本次文档整理没有执行它们。三种距离均使用同一冻结所选 profile，实际相对 route 由 `--distance` / `--roundtrip` 生成，并在新 run 的 `slam_binding.json`、`slam_execution.json` 中归档。不要把 profile 中旧真值校准路线字段当成本轮实际反馈来源。

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode

SLAM_TRANSFER_PROFILE=/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/truth_pid_campaign_20261004/selected_controller_v3_deployment.json
SLAM_TRANSFER_CORE=/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_000700_truth_pid_flat_line_cascade_pi_25hz_5bb4/sources/truth/core.py

# 1m 安全诊断；历史 V4 51d2 已完成，采用当前 V5 源会产生新试验。
python3 -B navigation/truth_tuning_slam/run.py \
  --profile "$SLAM_TRANSFER_PROFILE" --profile-sha256 0e85b8f1afb2db5c52d4fb784bafe39fc002cabdb79f3b11c0f9abf78bcee527 \
  --controller "$SLAM_TRANSFER_CORE" --controller-sha256 1f7f74df251833cd416cbaf308f59aa80747aa407f7f35e3b85f7b01c9a026e8 \
  --distance 1 --duration 80 --post-completion 7 --allow-source-rate-reduction --label safety1m_mapped10_readtime_v5

# 6m 单程，与本轮 V5 单程验收范围相同。
python3 -B navigation/truth_tuning_slam/run.py \
  --profile "$SLAM_TRANSFER_PROFILE" --profile-sha256 0e85b8f1afb2db5c52d4fb784bafe39fc002cabdb79f3b11c0f9abf78bcee527 \
  --controller "$SLAM_TRANSFER_CORE" --controller-sha256 1f7f74df251833cd416cbaf308f59aa80747aa407f7f35e3b85f7b01c9a026e8 \
  --distance 6 --duration 180 --post-completion 7 --allow-source-rate-reduction --label flat6m_mapped10_readtime_v5

# 6m＋6m 往返，与本轮 V5 往返验收范围相同。
python3 -B navigation/truth_tuning_slam/run.py \
  --profile "$SLAM_TRANSFER_PROFILE" --profile-sha256 0e85b8f1afb2db5c52d4fb784bafe39fc002cabdb79f3b11c0f9abf78bcee527 \
  --controller "$SLAM_TRANSFER_CORE" --controller-sha256 1f7f74df251833cd416cbaf308f59aa80747aa407f7f35e3b85f7b01c9a026e8 \
  --distance 6 --roundtrip --duration 180 --post-completion 7 --allow-source-rate-reduction --label roundtrip6m_mapped10_readtime_v5
```

`--prepare-only` 可追加到同一命令，只创建独立准备产物。ROS domain默认83、RTF默认1，路线达到独立完成条件后继续7s停车。运行请求允许执行不是运行成功收据；结论以新增 `summary_slam_transfer_independent.json` 为准。

浏览器只读结果页：[实际 SLAM / Teacher 结果（8770）](http://127.0.0.1:8770/)。该页展示真实 SLAM 路线、原始来源、实际相机与 native COM 诊断；与 [真值校准页（8769）](http://127.0.0.1:8769/) 分开。若页面服务未启动，不影响已保存的 [实际轨迹](../test_results/slam_transfer_v3_20261005/v5_aggregate/actual_slam_trajectories.png)、[速度](../test_results/slam_transfer_v3_20261005/v5_aggregate/actual_com_velocity.png)和[来源新鲜度](../test_results/slam_transfer_v3_20261005/v5_aggregate/actual_source_freshness.png)图。

## 冻结来源与未验证范围

以下 SHA 来自本轮已完成汇总/执行归档，不是新的源码冻结或新的验收：

| 来源 | SHA256 |
|---|---|
| model_1000.pt | `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34` |
| 冻结 controller core | `1f7f74df251833cd416cbaf308f59aa80747aa407f7f35e3b85f7b01c9a026e8` |
| selected_controller_v3_deployment.json | `0e85b8f1afb2db5c52d4fb784bafe39fc002cabdb79f3b11c0f9abf78bcee527` |
| V5 adapter.py | `2d332f24440b6f275962274c50e49cbc612662ad79e0a702b8f257b4ed5bc8a2` |
| V5 worker.py | `beef949e1e1e86be442f539a123dac44a4325fb56565e702c9c43fa54bd38584` |
| SLAM transfer run.py（V4/V5相同） | `35bdafdc1904b09d7e9c2dee8e8a0d47030bd4350150ae4fcb770de0a147d996` |
| 独立 SLAM 验收器 | `65df78481f7e5e30dc7b1a09bb9c9aab8fa5fa351d38b602c2d4ce018693d231` |
| 原共同 native验收器 / protocol | `7e386d004e2d1187753609b73f6b9f419c417aa35c1a30147d28385ffcd9fb8a` / `e2308b7b7585f933081cf741bd8670e950607cea495c3f4eca293e33ae7796a8` |
| native libteacher_actuator.so | `3a0ca63fb63c9e20c07639e2faf4d7509fba29a2bd5e987b8875dd50534a2ec2` |
| 本轮 aggregate.json | `8accee196d2709a193af174c425d829ac1dde49e155dd851faa2eac33426d149` |
| 汇总最终 manifest.json | `bf321c89b41ad1c6179303009de253449f8d70938968b26aa74e4c7c758a86c8` |

每次运行的完整源/输入哈希和原收据可从 [aggregate.json](../test_results/slam_transfer_v3_20261005/v5_aggregate/aggregate.json) 追溯。已通过结论仅属于上述实际 SLAM 固定平地路线迁移。这个新控制器尚未验证 SCAN路径规划、实时云避障、多层路线、连续曲线 SLAM、旧地图绝对路线、新控制律的 Isaac 对照或真机；不替代历史控制链各自的已封存结果，不宣称通用 Sim2Sim通过。
