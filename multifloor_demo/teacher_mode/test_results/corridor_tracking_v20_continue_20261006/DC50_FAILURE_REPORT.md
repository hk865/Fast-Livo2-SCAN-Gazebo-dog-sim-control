# dc50：原完整 46 区失败，第 10 区路线越界

运行 `20261006_140607_closed_loop_cascade_clock_hold_v20_original46_r1_dc50` 的[独立正式报告](20261006_140607_closed_loop_cascade_clock_hold_v20_original46_r1_dc50_FULL46_ACTUAL_EVALUATION.json)保留 **FAILED**：完成探索 9/18 区、总体 9/46，第 10 区未到达。10 项通过、4 项失败、8 项未验证。回原点、再次导航和最终首次五秒停车均未完成；正常退出及资源清理不替代导航通过。

## 越界有实际物理对应

| 时点 | SLAM 原路线横向误差 | 原生机身原点对原场景路线的横向偏离 |
|---|---:|---:|
| 首次保护使用的 SLAM header：187.575 s | 0.4501175 m | 0.4717252 m（native 187.565 s） |
| 最后 SLAM header：194.670 s | 0.4629687 m | 0.4863361 m |

原门限始终是 **0.45 m**。首次 failed 输入回执在 187.62 s 被 Actor 读取，要求零速度。上述第一组时刻相差 10 ms，不宣称严格同刻；原生位置也明显在同一侧越界，不支持将这次失败解释为纯 SLAM 假报警。停止后仍有站立运动，整个尾段 SLAM 最大横向误差为 0.4914747 m（192.49 s），不能用最终值当作全程峰值。

首次越界附近，native RPY 为 `[-0.003153, 0.021936, 0.330829] rad`，按冻结场景轴注册换算后的 SLAM 与 native yaw 差约 **0.000516 rad**。native COM 速度修正到机身原点后，世界速度为 `[0.027987, 0.071680, -0.039745] m/s`，仍向路线外侧运动。该快照无 fault、无机身接触，四足接触均有记录。全运行 9735 个 50 Hz 原生快照的有限安全检查通过；这不是全部 200 Hz 物理步骤验收。

坐标对齐使用本 run 的 `rotation_camera_init_from_world`、原始 SLAM anchor 和固定场景 spawn XY，未拟合真值轨迹。归档 `teacher_actuator.cpp` 证明 `position` 已是机身原点，不能再减 COM 偏移；仅速度按 `v_origin = v_COM - omega × COM_offset` 修正。场景路线高度与实际机身高度的基准不同，本报告不宣称直接三维位置残差。真值仅用于事后诊断，不参与导航。

## 可以确定的路径与转向事实

实际接收的 path 256 在原采样点上距实验路线中心线最远 **0.429365 m**；随后 path 257 的 121 个原采样点中 **29 个超过原实验 route fence**，最大 **0.481395 m**。已逐文件验证 NPZ SHA 和原 samples 浮点字节 SHA。这说明候选路径与实验任务路线边界存在不一致；**不是证明 costmap 碰撞，也没有重放 SCAN 碰撞检查或连续足支撑安全性**。

第 10 区的 PID 记录含 205 次路径变更、6 次从 `drive` 退出到转向阶段；6 次均在同一条记录观察到路径变更。最后一次发生于 187.28 s，记录 heading error 为 −0.58718 rad。记录关联不能独立证明换路导致越界，更不能排除 Teacher 跟踪、速度环、状态估计或其他因素共同影响。

181.6 s 后的 154 个实际 `drive` 更新中，相对当前路径的控制横向误差最大绝对值仅 0.03755 m，但参考世界横向速度均值为 +0.06525 m/s。SLAM 与 native COM XY 速度差的 RMS 为 `[0.01242, 0.01475] m/s`；native 与当时参考速度差的 RMS 为 `[0.06592, 0.05466] m/s`。当前路径局部跟踪误差较小，不保证留在原实验路线边界内。下一步应独立测试安全候选轨迹选择与实际执行路径监视；本轮没有修改控制器或降低任何门。

## 证据与调用记录

- [详细诊断 JSON](DC50_REGION10_FAILURE_DIAGNOSIS.json)：具体时戳、姿态、速度、路径 SHA、逐行 offset/length/SHA 和未验证范围。
- [170–190 s 精简数值窗口](DC50_REGION10_NUMERIC_WINDOW.json.gz)：400,913 字节，可导出绘图；完整 25 MB compact 留在本机。
- [实际审计 provenance](DC50_AUDIT_PROVENANCE.json)：绑定正式报告、执行包装器 SHA、完整 V20 归档路径及实际使用的源 SHA。源选择已逐条核对，未宣称另有合成 selector 单元测试。
- [首次评价器中断记录](DC50_FIRST_INTERRUPTED_AUDIT.json)：旧 basename 查询遇到 V19/V20 同名源文件而中断。新包装器按完整 V20 归档路径和冻结 SHA 解歧义，不改原数值门。首遍未落 compact，需要一次重读；最终完整读取约 21 秒，每个大 JSONL 一遍。后续只读 compact 和四个小型路径 NPZ，没有再扫大 raw。

本次实际调用为 `python3 audit_dc50_once.py --run /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_140607_closed_loop_cascade_clock_hold_v20_original46_r1_dc50`。包装器在报告或 compact 已存在时拒绝覆盖；此命令是历史调用记录，不应重复生成或覆盖现有报告。未来复评必须指定另一个已结束的新 run，并生成其独立文件。仅运行 `evaluate_full46_v20.py` 不包含本次完整路径解歧义及 compact 捕获。
