# 独立实际 SLAM 固定路线迁移验收

这是离线、只读分析器，不启动 ROS/仿真，不发布速度、关节或信号。仅为指定新 run 新增独立收据，不改变旧总结、原始日志或失败结果。

```bash
python3 navigation/truth_tuning_design/test_evaluate_slam_transfer.py
python3 navigation/truth_tuning_design/evaluate_slam_transfer.py \
  --run /absolute/new/run \
  --native-helper /absolute/archived/truth_tuning/evaluate.py \
  --protocol /absolute/archived/truth_tuning/protocol.json
```

输出 `summary_slam_transfer_independent.json`。既有收据拒绝覆盖；如确实需要独立追加诊断，可显式使用 `--receipt-suffix label`。退出码 0 只表示本独立范围的全部验收门通过；失败或缺证据返回 2。默认 helper/protocol 为现有冻结 v2，任何版本都必须匹配分析器内冻结 SHA。

原始 SLAM/IMU 接收日志、实际 producer JSON、每个 Teacher read attempt 原 UTF-8/bytes SHA、真实 core 更新和原生物理记录共同构成证据。重复 heartbeat 允许继续读取既有包，但不能刷新旧源接收墙钟、计算一次新的控制更新或延长到达 dwell。gyro 必须在 pose 之前，pose/control feedback 必须在 Teacher 实际读时钟之前。原 producer `/clock` 合同允许 50ms 未来相位容差，与严格因果 SLAM feedback 分开报告；不偷偷放宽 300ms 双 TTL。

保持 frozen v2 数值门：路线最大/RMS 0.20/0.08m，实际行驶朝向 0.2rad，稳定 COM 速度平均绝对误差 `max(0.05,0.25*reference)`，原始 SLAM 末目标 0.15m、连续源头 dwell 0.6s、5s 停车位移/偏航 0.05m/0.1rad。原生 200Hz 每步检查 RP≤0.65、clearance≥0.18、机身碰撞0、力矩命令≤23.50001、qd≤30.001。Actor 外部命令 slew 保持 0.6/0.6/0.8 每秒。

源到达/停车触发由 SLAM 原头决定。Gazebo pose 只在其后用于离线执行测量：在首个真实 SLAM anchor 时刻做一次因果 yaw/translation 对齐，不从后续真值拟合或优化路线，不用真值选择目标/停车安静窗。这只能验证已锚定的机身相对路线；原场景绝对中心线注册另列 `unverified`。

选中 CAS25 的数学、gains 与 command limits 保持，实际 mapped-SLAM 新源上限10Hz必须显式授权降率，记录实际更新率。不能称 SLAM25 已实现。Actor 仍保留232维特权观测，3维速度命令与12维上一动作为已知输入；新高层 SLAM/IMU 不等于 Actor 的全部传感替换。

要求六个主自有进程及 SLAM launch 六个子进程原日志均干净退出，记录队列 drain 且原始文件条数一致。缺证据为 `unverified`，明确矛盾/超门为 `failed`；任何结果 score 恒为 null。通过也不会授予 SCAN、实时避障、多楼层、通用 Sim2Sim 或真机通过。
