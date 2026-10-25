# published repro11 独立实际验收：未通过

本轮任务在 389.015 SIM 失败，高水位 34.913044m。正式失败原因是 `actual_navigation_failed:普通停车失败锁定：ordinary_zero_hold_actual_motion_or_drift_exceeded`；停车、动态最终验收与 full F3 均为 false，original46 也为 false。沿用原19项标准读取本轮现成结果，不继承旧 formal22 PASS。

首次 T0=388.790，固定窗口应至 393.790，未重置。实际 Actor 零输入 388.730 → native ACK 388.750 → 资格建立 388.790 的因果片段已成立。但控制器失败后停车 audit 未运行、没有完整固定5秒证据，不能认证窗口速度、姿态、漂移或保护状态。具体失败触发量由 root 分析，本审查不重复。

动态仅有业务片段：真实 stop 211.510、resume 236.805、恢复 0.791567m、clear/business completion 240.890。状态为 completed_pending_verification；task.dynamic_result=None、独立最终物理验证/owner capsule未形成，因此不是 dynamic PASS。RGB终验 required=true、verified=false、result=None，也不等于已证明没有RGB数据。

收尾与加载已证：六项进程/域/锁检查、91项加载、Actor原CRC回执 19582 样本、pipeline正常drain 174583 包均通过，closure errors=[]。ENTRY_EXIT rc=1，wall=473.623s，无外层强停。worker自然rc0、无信号、completed=true，但原worker_result和CRC收据仍记录 `EOFError: Gazebo actuator disconnected`；所以不能把CRC有效或closure errors0写成worker无fault。该EOF原因不在本次复核中追查。

实际使用新包 native f5b/e0 和新SCAN已由原加载收据验证。工具navigation_PASS=false是非授予型字段；本轮未通过的结论来自真实task/parking结果，而非工具占位。未重算几何、CRC、全source或运行新仿真，未改工作树/原回执。剩余控制问题由父对话处理，不擅自调参数或重跑。
