# 最终 CM 下的现代执行组件入口

两个 driver 是 excluded 验证资产，未启动节点或物理。runner/launch/完整实际传感器记录由 simulation owner 统一运行；实际生产 NAV 是唯一 Twist 发布器，driver 只发送任务、stop Bool 和真实任务状态/箱子 enable。

- `probe_slope_motion.py`：spawn `[4,2,.5]`，一次实际 SLAM/IMU 校准后的世界轴相对 `[3,0,.3]`，箱子始终关闭。
- `probe_dynamic_motion.py`：spawn `[0,0,.30]`，相对 `[0,2,0]→[2,2,0]`，只将实际匹配 request/index/state 原样回显；生产 SceneTrigger 自然触发箱子 8+20+8s，没有强制 trigger。
- `held_command_contract.py`：原已修精确 native zero 边沿及同目标实际已执行新 reference 的 held-command 合同；4Hz status 标签仅诊断，不能把迟到旧标签当违规或只靠新 reference 未执行解除 hold。

调用约定为 `DEMO_TEST_ROOT=/absolute/multifloor_demo`，`argv[1]` 为结果文件，`DEMO_RUN_DIR` 与结果目录统一由 owner 设置。生产父类 `simulation/probe_slope_navigation.py` 明确从绝对 root 导入，避免与本目录入口同名导入；runner 必须逐字归档 driver、held helper、父类及全部 source/runtime SHA。

parent 的旧 adapter subscription 在同一个 `create_subscription` 调用中映射到 `/demo/control/joint_stop_safety`，没有重复旧话题订阅。GT BEST_EFFORT depth2000，其他输入 QoS 不变。任何 aggregate bridge failed 或 adapter failed（包括 origin 前）立即发送 NAV stop，不等待90s；没有把 hold 当 failed。

两项均保持 raw .22m/.4s、每点90sim、独立同刻 .30m、一次初始 SE3、GT≤.15s真实括号且无外推。origin→实际 terminal 内缺真值尾端也计 coverage failure，只有明确 terminal 之后的清理记录不参与任务精度。动态仍要求真实 native guard 云提交、held期间双源零速、fresh same-goal executed SCAN、恢复后 SLAM+GT真实前进≥.025m、箱子实际确认位置变化≥1m；不只靠命令或阶段标志通过。

没有 strength wrapper、没有新控制反馈。状态验证实际生产 .12vx/.08walkyaw/.12pureturn，原始 tilt/heading/碰撞/区域未改。这些单目标/两目标 legacy component 不等于46段区域完整 Demo。

`python3 .../cm_motion_driver_staging/test_driver_contracts.py` 的10项实际 AST 方法反例通过，验证现代topic/GT队列、origin前fail即时stop、正常hold、真值缺尾/缺括号与清理边界、没有Twist publisher/strengthoverride。该结果只证明接口/验收方法，不是物理 PASS。
