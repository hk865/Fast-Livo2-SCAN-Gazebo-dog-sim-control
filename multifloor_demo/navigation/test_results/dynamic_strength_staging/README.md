# 待物理验证的单变量 NAV 候选

本目录在 `test_results` 中，不改生产源码或默认值，也未启动第二次仿真。当前实际两点组件失败证据保持在 `../20261001_dynamic_two_goal_actual`。

候选仅将 **行走混合偏航上限** 从 0.04 改为 0.08 rad/s；增益仍为 0.5，纯转仍为 0.12 rad/s，0.20 持续 0.5 秒 / 0.55 即停、原始倾角保护、实际 SCAN、点云保护、每点 90 秒、0.22 / 0.4 秒到达与独立 0.30 米验收均不变。是否执行必须先依据物理 owner 的实际固定输入对照；此目录本身不声称 0.08 安全或有效。

`controller_strength_wrapper.py` 导入冻结的实际生产 controller，运行时仅覆盖同一生产 `control_core.MAX_WALK_YAW_RATE`。原生产文件 SHA 必须严格匹配，否则拒绝启动。该值同时供真实 `follow_trajectory` 和 `HeadingGate` 使用。导航仍是唯一的机身指令发布者，GT 仅用于结果验收。

实际执行时，runner 逐字复制并记录五个测试入口、壳脚本及 SHA；额外保存 `navigation_strength_override.json`，分开列出生产源码 SHA、测试运行时 cap 和全部保留的保护。测试副本 stack 只将根目录解析改成已传入的 `DEMO_TEST_ROOT`，并将原 controller 入口替换为这个 wrapper，其余启动顺序和传感器/SLAM/SCAN/记录器不变。

域 73 必须先由物理 owner 明确清场交接。物理对照证明候选后可执行以下独立 A/B（当前仅保存命令，不执行）：

```sh
bash multifloor_demo/navigation/test_results/dynamic_strength_staging/run_dynamic_component.sh --joint-stop-adapter --walk-yaw-cap .04 --output-dir "$PWD/multifloor_demo/navigation/test_results/dynamic_strength_a_004"
bash multifloor_demo/navigation/test_results/dynamic_strength_staging/run_dynamic_component.sh --joint-stop-adapter --walk-yaw-cap .08 --baseline-dir "$PWD/multifloor_demo/navigation/test_results/dynamic_strength_a_004" --output-dir "$PWD/multifloor_demo/navigation/test_results/dynamic_strength_b_008"
```

两次请求仍为真实 SLAM 冻结校准后的 `[0,2,0] → [2,2,0]` 世界轴相对目标；真实任务状态关联后，箱子自然执行 8+20+8 秒。失败原样保留、清理全部所属进程，不能把停止/恢复检查通过当作两点到达成功。

B 启动前将逐项比较 A/B 的完整 source_manifest、所有 runtime 库/二进制 SHA、测试入口逐字 SHA、相同 adapter 与 A 已清理确认。只有 cap 值不同；任一比较失败就在启动前报错并保留差异证据。每条实际 NAV 状态都额外记录 `test_only_motion_override`，指明本次 cap 和生产默认 0.04。
