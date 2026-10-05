# V4.4 离线准备：真实到达停留和真实障碍清除

本版仍是未验证的新动态实验。冻结文件 `../navigation_v44_measured_dwell_actual_guard_freeze.json` 的 SHA256 为 `714e75459d5a6087cd147498c955013f2d7d7a16899669b1f1046bd04681314f`。本目录和四个 `runs/20261004_navigation_v44_offline_*` 目录仅作离线状态检查、配置生成和 LaunchDescription 构造；没有 ROS 初始化、节点、Gazebo 或训练。

前一实际 V43 `2f70` 没有完成出程。机器人在原 `.17m` 控制到达边缘反复零速，原连续 `.6s` 停留最长仅 `.5s`，90秒期限正常判失败。原程序还将清除开始 `25.69s` 跨过 `.55s` 没有实际检查的间隔，缓存 tick 在 `26.69s` 释放并请求新 SCAN；严格实际检查直到 `27.69s` 才满足连续1秒。原停车 `.033278m>.03m`、这次提前恢复和0个区域均保留为失败，没有追认。

新导航 scope 是 `finite_flat_dynamic_stop_resume_dwell_v1`，继承前版 `.20rad/s` 转向上限，仅两处行为改变：

- 只有新 profile `arrival_stop_policy=after_measured_dwell` 将原真实 SLAM 到达结果返回为 `(inside and arrived, arrived)`。第一次进入边界不会立即把 Teacher 置为零命令；继续沿原碰撞检查过的 SCAN 轨迹，直到原到达窗口真正满足后停车并切换目标。
- 只有新 profile `clear_guard_gap_max_sim_s=.3` 在实际原定障碍检查间隔超过300ms或原点云/SLAM header不新鲜时重置原清除计时。缓存 tick 不能完成清除，等待下一次原定实际几何检查确认满1秒后才释放；不额外运行几何检查。实际 guard 时钟重复或倒退会保护停车。

原 `.17m` 控制区域、`.6s` 连续到达、**200ms到达位姿最大间隔**、90秒目标期限、300ms源/命令超时、1秒连续清除/300ms最大检查间隔、实体阻挡10秒、5秒停车及全部停车阈值不改。原 flat `.12`、旧 dynamic `.12` 和 V43 dynamic `.2` profile 文件字节不改；原 checkpoint、247输入、关节映射、PD、物理和控制权不改。策略仍保留特权 COM/高度扫描，导航仍只读真实传感器 SLAM/SCAN，不能称完整感知替换或多楼层导航通过。

新增实际时序旁注是 `navigation_guard_continuity_history.jsonl`；只记录真实 source/clock 导致的计时重置，明确不是障碍计算。每次实际计算仍完整写 `navigation_guard_history.jsonl`，清除释放必须出现在该次真正 guard 记录的 `obstacle_hold_before=true`、`obstacle_hold_after=false`、`clear_release_requires_this_callback_actual_guard=true`，并由原XYZ、过滤姿态、buffer hash和原两走廊返回值独立复算。字段冻结于 `navigation/dynamic/guard_continuity_schema.json`，原 guard schema 不覆盖。writer 收据的 expected_records 必须等于完整实际 guard 行序列；continuity_resets 必须等于重置旁注行数，无错误或静默丢失。

144项检查重新通过：24项真实 controller 到达/目标切换/路径末端/保护方法（使用离线假状态），16项实际 scheduled-guard/cached-tick 方法（复刻 `.55s` 间隔、1秒缓存不释放、满1.05秒实际检查后释放）、28项原 gate/几何回归、37项边界、9项物理保护方法、30项 fixture 检查。原 gate 6000步状态和命令完全相同、原几何800例结果及类型完全相同；原两次几何计算不增加。四种 scope/launch 构造通过，默认平地11个 action、三种动态各19个；没有启动 LaunchService。

由 root 编排一次新 run，命令为：

```bash
python3 scripts/run_test.py navigation --navigation-dynamic-dwell \
  --navigation-duration 240 --label slam_scan_dynamic_dwell_v44 --repeat 1
```

runner 对配置加 `--dynamic --dynamic-turn20 --dynamic-dwell`；新实体 fixture 仍以原物理 experiment v1、三轮真实有限平地通过为依据。必须先冻结独立 candidate4，再实际验证。总体通过仍要求真实障碍/停车/撤离、连续真实清除和新SCAN、复走、两个原SLAM区域在原期限内到达、终点实际停车、六个自有进程退出0和全部已启动子进程正常结束。离线检查通过不能代替这些实际结果，全局历史验收不升级。
