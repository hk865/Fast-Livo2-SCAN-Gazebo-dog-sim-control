# 保留活动轨迹的安全链审计（2026-10-06）

状态：只读 V20 源码审计；**没有改生产代码、编译或启动仿真**。V20 原样复测由主任务执行。这里给下一独立候选 A/B 的最小准入，不给旧路径保持授权。

## 结论

不能只在 Python 端拒绝新样条并继续旧 `samples`。**SCAN 的碰撞回调跟随规划器最新 `local_data_`，并不知道执行侧是否接受它；拒绝新候选后，旧执行路径可能失去 SCAN 持续碰撞监测。** 当前实时点云 guard 仍会工作，但覆盖范围、输入和判据不同，不能作为等价替代。

最小 A 先做无行为变化的候选 prepare/commit 边界和原因记录。B 的保持/选择滞回，必须等“已提交执行路径”的独立监测、失效通知及新鲜反馈链完成有限验收后才启用。

## 1. 具体源码证据

以下 `V20/` 指 `multifloor_demo/teacher_mode/navigation/corridor_tracking_v20/`。

| 位置 | 实际行为与影响 |
|---|---|
| `scan_ws/src/plan_manage/src/planner_manager.cpp:307,317,497` | 成功 replan 与 EmergencyStop 均调用 `updateTrajInfo()`，先替换 `local_data_.position_traj_/start_time_/duration_`、增加 ID，之后才发布候选。 |
| `scan_ws/src/plan_manage/src/scan_replan_fsm.cpp:787` | 50 ms wall timer 的 `checkCollisionCallback()` 只取 `planner_manager_->local_data_`。没有执行侧 commit/active-path 回执输入。进入 WAIT_TARGET 或起始时间无效时直接返回。 |
| 同文件 `:800–824` | 碰撞检查使用名义 `t_cur=now-start_time`；早段只查剩余轨迹的前 2/3。发现占据后尝试新规划、切换 emergency/replan；最新局部轨迹又会替换。保持旧路径若只保 Python 数组，无法保留该监视对象。 |
| 同文件 `:437` | `execution_frozen` 仅移动最新 local trajectory 的 start_time；它不是“执行轨迹 ID 选择器”，不能用这个 Bool 修复旧路径失联。 |
| `V20/shared_controller.py:434,516,649` | 活动样条被替换后执行侧 follows 新 samples；实时 guard 检查当前 checked target/实际 steering 两个方向，route 用 segment-start 到 goal 的高度插值。 |
| `multifloor_demo/navigation/control_core.py:286,329` | 点云 guard 仅查前向 0.30–0.95 m、半宽 .42 m、至少三点和高度筛选。空/未观测区本身不能建立“整条剩余旧路自由”；它不等于 SCAN 地图剩余轨迹检查。 |
| `scan_ws/src/plan_env/include/plan_env/grid_map.h:392–411` | SCAN 检查双圆柱 inflated occupancy；outside map 返回 -1，在原 if 中被视为占据。地图内部 buffer 为 0 不等于新走廊的三态、逐 free-cell 新鲜和测得支撑证明。 |
| `V20/cascade_core.py:162` | 同目标的 `set_path()` 已保留 PI/filters/command，并按当前真实 3D SLAM pose重投影。无需再次把积分清零归因于换路；需要修复执行选择与监视选择的一致性。 |

当前 capture 已会拒绝换路（`V20/controller.py:215`），也应明确记录执行/规划监视 ID 是否分离；这不能作为扩大保持到全路径阶段的安全证据。本轮审计没有重跑这些边界。

## 2. 最小步骤 A：只整理接收事务，暂不保持拒绝后的旧路

在新候选抽出 `prepare_candidate(msg, metadata)`，完成现有 association、尺寸/有限性、采样、原目标来源、水平非退化等检查，生成 immutable candidate；准备失败不能留下 samples/path receipt/cascade ID 的半更新。owner 一次 commit 更新 outer samples、执行 ID、path receipt、cascade path；保持当前接受策略及安全 zero 分支，单独记录收到、准备、提交、拒绝。

保留 `trajectory_contract.py` 的原 reference stamp/body goal/完整 payload/递增 ID核验。把“已关联”高水位与“已执行提交”序号分别记账。SCAN 新 overlay 可只增加明确的 `new_reference / periodic_replan / collision_invalidated / emergency_stop / planner_failed` 原因、原路径 ID及递增 safety-event ID，不改 search/optimizer 数学。**真实 emergency/旧路失效必须零命令；不能因 stop 样条退化而按普通质量候选忽略。**

A 不宣称连续旧路监视已解决，不启 `retain_valid_plan`。先验收事务完整性和原因路径，再进入 B。

## 3. 最小步骤 B：活动执行路径监视与切换握手

建议在新 SCAN overlay 增加独立 immutable `ExecutedPlan`，与候选 `local_data_` 分离。planner 的优化/发布仍可替换 local_data，但不得覆盖/删除仍在执行的监视缓存。

1. 执行侧发送 proposed/committed plan 回执：run/request epoch、原 goal index/definition hash、reference stamp、traj ID、完整 payload及 hash、path hash、frame/layer、commit sequence。SCAN 只登记它自己实际发出的同内容候选；同 ID不同内容、未知候选、外任务及外楼层全部拒绝。
2. 候选预登记获得绑定反馈后，owner 原子提交；原路径监视保留到新执行 commit 被确认。握手不确定时保守监测 old/new 的并集，任一路失效先停车；断 ACK/缓存缺失时不能继续保持。不能写一个无记录的短暂“零监视”窗口。
3. 每次检查显式选 `ExecutedPlan`，**独立于 WAIT_TARGET、新候选是否成功、规划时间是否结束**。通过当前真实 SLAM pose 的 3D投影确定剩余前缀，保留 floor/progress约束；不能沿旧名义时间假定机器狗已走过某个障碍。停止/漂移/冻结也必须检查当前 footprint和刹停前缀，检查范围不能短于原 SCAN 有效监测范围。若点数/计算预算不足则 unavailable并停车，不截断后报 clear。
4. 检查发布绑定状态与不可丢失的 invalidation事件：所检查执行 commit/hash、地图 revision/source stamp、实际 pose stamp、检查时刻、范围、occupied/outside/unavailable原因、safety-event序号。事件对象是路径身份，不能给一个可能已变更的“当前路径”无身份 stop/clear。
5. tracker 使用本机 receipt +原 ROS source stamps核反馈；pose/cloud/IMU/command 原 300 ms sim+wall保护全部保留，监视反馈也必须有事前新鲜期限、序号及 ACK。过期、地图/帧/层不符、明确碰撞/紧急事件均零命令并重规划。旧 clear不得覆盖较新的 invalidation。
6. 原实时 steering/target cloud guard、tilt、heading、bridge、executor ACK、source watchdog、原障碍清除后等待/再规划以及终点 capture/parking仍独立有效。supervisor只能在所有必要条件满足时保持普通旧路径，不能用滞回覆盖任一保护。

这个 monitor 首先只延续原 SCAN占据碰撞范围，不能把 inflated-buffer clear升级为完整三态占据/地形/支撑认证。正式 corridor控制仍另需新鲜 measured support及逐格来源；r1 **0/400 certified**不能用于 B 的运动授权。机身与坡道地面扫掠相交也不能靠删除 ground voxels解决。

## 4. 有限准入及实际复测

新独立有限测试至少覆盖：旧 A 被执行、新 B 已替换 local_data但被 supervisor拒绝，障碍仅位于 A；planner进入 WAIT_TARGET后 A仍受监测；B clear但A occupied必须停车；长停/Teacher漂移时原名义时间已过而A前缀仍检查；collision→replan成功、emergency退化stop、异步commit/失效乱序、同ID不同payload、goal/floor/epoch切换、snapshot旧clear、feedback失联、缓存溢出/预算不足、owner提交故障无半更新。每例核实际监视对象和zero分支，不只检查journal文字。

还须证明新增双监视无并发 map race（当前 SCAN为SingleThreadedExecutor）、检查时延不会饿死原50 ms安全回调/源反馈，并保持原源码保护核验。传输单元测试不能替代这条生产对象选择链。

只有新有限源码/库/配置绑定门通过后，由主任务做原前九区 A/B对照：同spawn/实际registration/原目标区域/90 s timeout/.4 s dwell/300 ms保护，记录 old/new executed/monitored IDs、监视空窗、普通换路数、方向变化、PI/filter连续性、全保护事件和九区后的五秒主动停车。GPU中止独立保留；不能把减重规划、7/9或未碰撞当PASS。先基线复测完成，再决定 B 的实际试验。

本次仅新增这个审计文件，未改 V20 frozen目录或上一版方案。源码级建议与准入尚未实施；仓库分支上传、后续源码修改和实际仿真由主任务统一安排。
