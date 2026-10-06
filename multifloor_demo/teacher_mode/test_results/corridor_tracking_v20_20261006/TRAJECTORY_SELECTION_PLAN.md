# 轨迹选择与受控切换方案（2026-10-06）

状态：**只读源码审计与下一候选设计，未实现、未构建、未运行**。现有 V20 及 r1 证据保持冻结。本方案优化 SCAN 候选轨迹的选择和切换，不改变 PID 增益、Teacher、原区域到达条件或保护期限；与跨主机部署拆分无关。

## 1. 本轮证据与问题边界

实际运行 `20261006_130405_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r1_39ea` 留下 7/9 个原区域回执，随后因 `Less than audited GPU headroom` 中止，不能据此判定第八区控制失败，也没有完成九区或末尾五秒停车。GPU 资源中止与下列轨迹/诊断问题分别记录。

独立走廊时序审计记录 400 输入、400 结果：241 unavailable、159 blocked、**0 certified**；49/400 返回时活动路径已变化。输入 cloud 的原始源龄中位 70 ms、最大 205.000001 ms，但计算 wall 时间中位 378.664 ms、p95 473.022 ms，另有队列等待中位 28.303 ms；372/400 返回时该 cloud 已超过 300 ms。wall 时间包括调度/GIL等，不是纯计算 CPU 时间，也不能直接当通信耗时。

地图总体新鲜并不代表每个扫掠体素新鲜：严格逐 free-cell 300 ms 条件累计产生 3091 次 interval 拒绝；支撑模型大量因凸包覆盖不足、点数不足等拒绝。坡道诊断还观察到扫掠 AABB 与测得地面占据格相交。这只能要求复核机身/地面参考和体素几何，不能直接把这些占据格删掉或当作误报。**当前 shadow 结果不能授权“走廊已安全”或旧计划继续运动。**

来源：[独立时序审计](evaluation/V20_39EA_CORRIDOR_RECEIPT_CLOCKS.json)、[九区部分运行验收](evaluation/V20_39EA_PREFIX9_ACTUAL_EVALUATION.json)、[坡道占据格诊断](evaluation/V20_39EA_OCCUPIED_PREFIX_DIAGNOSTIC.json)。

## 2. 当前源码的插入点

| 边界 | 现有行为 | 下一候选修改位置 |
|---|---|---|
| `multifloor_demo/navigation/trajectory_contract.py:40,64,83` | 每次 reference 重置有界关联缓存；核 reference stamp、原 body goal、完整样条 payload、递增 traj id。它证明来源，不证明碰撞自由。 | 保留这些核验；匹配后交给 supervisor，不直接表示执行接受。分别记录“已关联”和“已执行提交”高水位。 |
| `V20/shared_controller.py:434` | 解码/采样后立即替换 `samples/active_trajectory_id/planning_start_state`、增加 replans；退化样条会清空活动路径并发零。 | 抽出不修改活动状态的 `prepare_candidate()`；保留显式紧急停车语义，不能把真正 stop 当普通劣质候选忽略。 |
| `V20/controller.py:215` | 先调用上述父入口，随后筛水平采样点、写 path receipt；capture 已有“保留旧路径”例外。 | supervisor 放在父入口副作用之前；候选完全构造并核验后，owner 一次提交样条、收据与 cascade。 |
| `V20/controller.py:274` / `cascade_core.py:162` | 仅 request/goal 改变才新建 cascade；同目标换路以当前真实 3D SLAM pose 重投影，已保留 PI/滤波/命令。 | 复用这个 handoff，不重复 reset；将延迟的 `ensure_cascade()` 换路纳入同一次提交，避免 outer 旧路径、inner 新路径或反向组合。 |
| `V20/shared_controller.py:482,516` | 新 reference 有不可变 stamp；无路径/路径耗尽/低速停滞会再请求；当前 IMU、姿态、点云、障碍、执行 ACK、到达检查独立生效。 | 区分“请求备选”与“活动路径无效”；请求备选本身不清空活动路径，不改变已有保护分支。 |

这里的 `V20/` 指 `multifloor_demo/teacher_mode/navigation/corridor_tracking_v20/`。旧 path 的生成 stamp 本来就不是 300 ms 路径 TTL（`cascade_core.py:163`）；保留路径仍须持续核对新鲜源和当前保护，不能延长 pose/cloud/IMU/command 的期限。

## 3. Supervisor：先准备、再选择、最后原子提交

新增独立模块，不直接发布速度。它拥有不可变 `ActivePlan`、最多一个 `PendingCandidate` 和决策 journal；ROS owner 是唯一可提交者，worker 只返回结果。

1. **候选准备。** 从已关联 payload 构造样条、采样、有限弧方向和完整收据，不修改 `samples`、PI、heading gate 或活动 ID。身份至少包含 run/request epoch、goal index/goal-definition hash、reference stamp、trajectory id/完整 payload hash、frame、路线 layer、source/plan sequence。候选的 adjusted goal 不可替换原到达区域。
2. **旧计划可继续的有限条件。** 同 request/goal/layer，活动路径仍有可执行前缀，实时 source/tilt/cloud/native guard/ACK/bridge 均通过，且没有原路径碰撞、明确 emergency 或耗尽证据。满足时，普通重规划失败、重复/退化的非紧急候选可以被拒绝，原路径和投影保持；新规划请求不能反复触发 `pre_turn`。这只继承原 SCAN+实时 guard 的功能范围，不能升级成完整占据/支撑认证。
3. **选择与滞回。** 相同或近似等价几何不换路；普通候选须改善任务前进/方向连续性/可执行长度并支付切换代价。评分权重、改善门和最短保持条件在新候选测试前冻结，不从一次成功结果反推。碰撞/emergency/源失效优先于评分，立即零命令并重规划；不能靠滞回等待危险旧路。
4. **提交再核验。** 在一次 owner tick 中重新核 epoch、当前原目标、frame/layer、活动 generation、最新真实 pose、保护状态及异步结果有效期限。旧/新路径同 XY、不同楼层不允许复用。失败无副作用；成功一次更新 `samples`、执行 ID、path receipt、cascade path、outer finite-arc reference 和 journal，再从真实当前 3D pose 映射 progress。保持 velocity PI、filters、slew anchor、ACK状态；只有真新路径才处理 heading 参考，原 0.2 rad 运行方向门仍生效。
5. **过期处理。** 返回结果若 epoch/path/hash/层不符，记 obsolete，不挂到当前路径。即使 ID 相同，也须核完整内容。请求时的 `now_ns` 不可作为返回时的现在；`valid_until_ns`、逐格/支撑源龄必须在提交时再核。新目标、stop、失败、clock reset 和 capture 均使 pending 失效，不保留跨 epoch 结果。

生产保护 zero、主动停车与 native 故障 damping 保持原用途；不把 `action=0` 当停车，不用规划器估计时间或 Gazebo 真值代替原 SLAM 区域到达。

## 4. SCAN 重规划原因与诊断优化分开提交

`scan_ws/src/plan_manage/src/scan_replan_fsm.cpp` 已有 `TRIG/FSM/SAFETY` 状态来源，但 `publishTrajectoryMetadata():920` 未给执行侧输出具体 replan reason。下一独立 overlay 可只加元数据 hook，不改 planner/search/optimizer 数学：

- reference callback `:377`：新原目标/reference；
- `EXEC_TRAJ :629–691`：常规距离阈值、时间路径结束、序列换目标，分别标记；其中规划器“时间结束”不等于机器狗到达；
- `checkCollisionCallback :787`：碰撞触发、旧路径无效、emergency 与碰撞后重规划成功；
- `callReboundReplan :834` / `callEmergencyStop :883`：generation reason、真实起点/速度源、原 reference、完整 payload，并保留旧字段。

执行侧 journal 应区分 `candidate_received/rejected/retained/committed/obsolete/active_invalidated`，保存 old/new IDs、原目标 hash、决策原因、当时 SLAM/IMU/cloud/ACK stamps、heading/progress跳变量。紧急事件用独立递增事件身份关联原路径，不可只凭一个可以迟到的候选 stop 样条。

走廊先继续 shadow。为消除 379 ms 计算与 300 ms有效期冲突，优先减少重复的局部支撑查询/哈希/完整地图遍历，按同一 immutable snapshot revision 缓存确定性索引和局部 patch；查询限于实际刹停前缀，但仍覆盖完整 footprint。缓存不得刷新 cell/source age。向量化/C++移植须逐结果等价、边界负例和 wall/threadCPU 对照；单 worker、有界任务保持，不通过增加排队掩盖过期。

支撑覆盖、逐 cell 新鲜度和机身扫掠参考属于单独模型实验：复核测得平面及实际机身外形，再定义可证明的 yaw/姿态包络；不凭当前 full-π AABB保守性直接减包络，不把稀疏点插成已观察地面，不把 unknown 设 free。若改变 cell-age 规则，必须另立有动态障碍失效检测的地图寿命协议，**不改现有 300 ms 输入/命令保护**；本计划不授权放宽该规则。

## 5. 分支、准入与验收

推荐三个可回滚提交/实验因子：**A 元数据原因 + prepare/commit边界；B 活动轨迹保持/选择滞回；C 走廊计算与测量模型优化**。新独立候选复用冻结 V19 SLAM；SCAN 元数据另建 overlay，V20旧源码、gate、r1/raw/report一律不改。仓库分支和上传由主任务统一执行。

先完成纯函数及抽取生产入口测试：乱序 metadata/同 ID不同 payload、goal/epoch/floor切换、候选计算时旧路变化、紧急停车、candidate失败不变更任何活动状态、一次提交无混合路径、handoff保留 PI/filter/限幅、反复普通 replan 不触发新 `pre_turn`、所有原 TTL/zero/capture/parking分支。再使用 r1 记录重放决策，明确它只能验证相同旧输入，不能预测新闭环。

实际准入需要新有限收据与源码/二进制/配置哈希，不继承旧 PASS。首先同原九区 profile、spawn、registration、90 s目标期限、0.4 s区域 dwell、300 ms双时钟期限、CPU Teacher与唯一执行器，比较普通换路数、无效化原因、heading gate复位、方向/速度/横向误差、源龄/计算龄/obsolete率。必须九区原几何实际到达且末尾五秒主动停车独立通过，才能扩大原46及多层。资源中止、超时、无证书、停车缺失分别保留；不以7/9、减少换路或 shadow结果代替导航/Sim2Sim通过。

走廊取得控制权前额外要求：每个用于运动的结果在提交时仍新鲜、与实际 active path/frame/layer绑定、所有 footprint free-cell与测得支撑条件通过且覆盖刹停距离；blocked/unavailable/unknown/过期只能保留诊断或停车，不能写 PASS。

## 审计来源固定

- `controller.py` SHA256 `2af343e23e60a3c9a2e9757ea1e60c8bdbb342b792f91bc5c5dcb315fb2a6a5d`
- `cascade_core.py` SHA256 `3c1a3cb00993fe213b622b0df2bdaf511cb3fcbbd2728f0fc998a87c155f4e82`
- 独立时序报告 SHA256 `90767801f0dd34b7c766a6a6cc55b7fa6ffb606e79f506777a7f7e1b981ab5ee`
- 独立部分九区验收 SHA256 `f7984218d670ec0ed13ca464530aa29edfa91a6a891bb3e6546a3736e12f82a4`
- 坡道占据格诊断 SHA256 `1c89d5309e8de5bcae0424f5ac3be0e6891593a0d225c43f63ad92fbcea88c83`
- supervisor身份/atomic handoff/保留保护边界已由空间参考审计 agent只读交叉复核；以上均不是新实现或新测试收据。
