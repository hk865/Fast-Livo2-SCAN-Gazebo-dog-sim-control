# V25 e8de：SCAN 停车路径与高度接口只读审计

本审计未运行仿真、未修改任何源码/参数/门，也未重读 PID、status、导航栈原始日志或原始云。实际证据来自原 22 门报告、关闭后的小 worker/runtime/config，以及 B agent 一次读取生成的 `E8DE_LAST3_SCAN_GEOMETRY.json`、`E8DE_ALL_TRAJECTORY_METADATA.json`、`e8de_SCAN_diagnostic_lines.json.gz`。源码路径以下均相对 SLAM 项目根。

## 实际结果与证据边界

- 原 22 门报告总体 **FAILED**：探索完成 11/18 区，全任务 11/46；第 12 区原 90 s 期限到期。前 11 区原 3D 区域、连续 dwell、期限独立验证成功，SLAM 最大间隔 35 ms。
- 原来源、唯一 CPU Teacher、实际加载 SLAM/SCAN、native 物理安全、HeadingReference 和 pipeline 正常 drain 门通过。runtime 无错误、owned cleanup 正常；它们不能提升导航失败结论。
- Actor provider 仍 `initial`，`switched=false`、`failure=null`、`completed_transitions=[]`。本轮未触发地形切换，V25 事件修复仅有有限生产路径验证，尚无实际三次切换通过证据。
- 真正加载的是 protected baseline SCAN，exe SHA `5a5e3745314d7252bc69dbdf41118305f988c2252e8148e2956b11114b687332`。当前 baseline CMakeCache 指向下列源码目录；本审计静态阅读这些源码，没有新增重建或机器码逐条等价验证。baseline 合同直接绑定的是运行 exe/config/setup 四项。
- 最后 234/235/236 三份**归档的有效轨迹**仍是前进样条，水平跨度约 2.43/2.46/2.47 m。236 接受于 217.67 s。三个 metadata 的 `body_goal == adjusted_body_goal == [16.0772831253,6.8390626561,1.2034672280]`，所以这三份没有把第 12 目标改走。B 记录之后有 34 个退化样条被拒，但它们在归档前已返回，不能拿这三份 NPZ 当停车样条的实测几何。

## 原地样条的源码因果链

1. `multifloor_demo/navigation/ros2_ws/src/plan_manage/src/planner_manager.cpp:317–325` 的 `EmergencyStop` 明确创建 6 个完全相同的 `stop_pos` 控制点；它生成停车轨迹，不是到达判定。
2. `scan_replan_fsm.cpp:729–738`：连续失败达到配置 `max_replan_fail_count=50`，进入 `EMERGENCY_STOP`。`:696–718` 执行停车/等待新目标；`:883–915` 发布该样条和关联 metadata。
3. `scan_replan_fsm.cpp:787–827`：对当前轨迹占据检查非零，先尝试重新规划；失败且障碍距当前轨迹时间不足 `emergency_time=1 s`，也进入紧停。检查只跳过 `WAIT_TARGET`/无轨迹时间，因此还能检查 `GEN_NEW_TRAJ`、`REPLAN_TRAJ`、`EMERGENCY_STOP` 保留的旧/停车轨迹。这解释了源码允许“当前位置仍占据→恢复生成→再次安全紧停”的循环，但不证明哪一个体素造成实际碰撞。
4. `shared_controller.py:451–460` 使用 `trajectory_has_progress` 拒绝尚离目标较远的停车样条，清空 samples/active ID、发精确零并清显示路径。原 `multifloor_demo/navigation/control_core.py:188–199` 要求整条三维几何弧长至少 .05 m、跨度至少 .025 m。这是合法停车处理，不能为了继续前进忽略 SCAN 紧停。

B 的匹配日志 compact 内有 34 条 `Replan failed 50 times`、594 条 `Obstacle discovered; emergency stop in 0.000s`、25 条 `A-star failed; aborting optimization`，末尾反复 `GEN_NEW_TRAJ→EMERGENCY_STOP→GEN_NEW_TRAJ`，最后等待新目标。计数属于 B 的匹配提取范围，不把未匹配日志当缺失或零。actual 紧停机制已有日志支持；碰撞对象/地图层/高度根因仍未由体素快照证明。

B 的同时间固定初始 SE3 诊断显示：217–230 s native base 世界高度稳定约 1.50 m（转换至 camera_init 约 1.20 m），SLAM z 从 217–218 s 中位 1.2229 m 降至 225–230 s 中位 1.0072 m；20 ms 插值 bracket，最近 native 样本不超过 10 ms，没有再次减 COM 偏移。220.31 s 起退化计数增加，之后 457 条 running status 命令全零、`obstacle_hold=false`。这支持“高度估计偏离和 SCAN 紧停在同一段发生”，尚不证明高度误差直接产生了具体占据体素。原日志 wall timestamp 不直接当 sim clock；另存的 `SCAN_LOG_CLOCK_JOIN.json` 仅是已存在 compact 与观察器本地双时钟的近似时间关联，不改变这个因果限制。

## 高度、帧、过滤和目标接口

| 项目 | 实际接线/源码 | 结论 |
|---|---|---|
| 目标高度约定 | `shared_controller.py:488–497` 发 body goal 时减 .4；`scan_replan_fsm.cpp:364–379` 加 `grid_map.body_height=.4` | 两处精确抵消；不能把 .4 与狗约 .3 的真实站高之差直接当作目标偏差。后三份原 body goal 已实测一致。 |
| 位姿/速度 | `shared_controller.py:226–255` 使用真实 SLAM body pose，将 child-frame 速度旋转为 SCAN 需要的 world twist；`multifloor_demo/slam/odom_adapter.py:56–85` 给 body/lidar 明确外参位姿 | 没有真值导航或用目标 z 覆盖测量 z。 |
| 云与射线原点 | `stack.launch.py:43–45` 的 body、sensor、cloud 分别来自 SCAN body odom、SLAM lidar odom、registered cloud；`grid_map.cpp:870–970` 在 `cloud_is_world=true` 下不再变换已配准点 | 整体 frame 是 camera_init，`need_extrinsic=false`，未发现重复外参变换。cloud callback 使用最近 sensor pose，而非同 header 精确关联；这是可改善的接口可观测性，尚无本轮时序失配致错证据。 |
| 地面过滤 | `shared_controller.py:273–304` 只做近邻 SLAM 姿态下自体过滤；`control_core.py:77–90` 是机身/腿包络；`grid_map.cpp:934–964` 不分割地面 | SCAN 保留真实地板/坡道表面作为占据 hit；没有根据 floor ID 删除点。不能说“配置 ground_height=0 就过滤了第二层”。 |
| ground_height/地图窗口 | `grid_map.cpp:88,114` 只是初始地图原点/边界；`:350–444` 在 x/y/z 滑动；窗口 10×10×5 m | 不是固定在一层的高度筛选。是否当前目标越界，需要实际 map version/boundary，静态源码不能确认。 |
| 体素与碰撞体 | `stack.launch.py:35–39` 配置 resolution=.08、垂直 inflation ±.12、双圆柱 radius=.25、offset=.18；`grid_map.cpp:234–250` 使用 ceil；`grid_map.h:372–389` 查前/后圆柱 | ±.12 实际为 ±2 个体素偏移，即 .16 m，另有体素宽度/相位；不能当连续精确 ±12 cm。地面点或历史高度不一致若进入这一带，可以触发占据，但本轮没有 offending voxel 的来源证据。 |

规划本身是受限的 2.5D：`planner_manager.cpp:10–38,251` 按 XY 弧长设置线性 z；`path_searching/src/dyn_a_star.cpp:169–179,227–236` 只搜索 XY 邻居，z 由起终点插值；`bspline_opt/src/bspline_optimizer.cpp:1169,1197` 将 z 梯度置零。它既不能自动重估支撑面高度，也不能自由升降绕开一个错误高度的占据层。Teacher 导航指令只含 vx/vy/wz，亦不能在平平台凭速度 PI 消除 SLAM 的绝对 z 偏差。应区分支撑面/SLAM高度问题和二维跟踪问题。

两处目标改动属于需要观测的独立分支：`scan_replan_fsm.cpp:306–344` 占据的全局终点会沿全局曲线后退，更新 `end_pt_`；`:997–1045` 占据的局部 horizon 目标会寻找前/后 free 点。原 metadata `:930–938` 已有原/调整目标，后三条没有调整；停车后未归档样条不能证明这些分支也完全未触发。内部调整目标绝不是原目标区到达。

## 最小后续步骤，未实施

先在**新候选**补每次规划失败/紧停的有界证据：FSM 前后状态、明确原因、source cloud/pose header、地图版本/边界、start/local/original/adjusted goal、前后圆柱查询体素索引与占据值、hit 来源/支撑面相对 body 高度、样条是否 EmergencyStop。保持 .3 s 输入期限、紧停和原 90 s/3D dwell 原门。现有 metadata 无紧停类型/失败原因，退化样条又在归档前返回，补这一接口比先改 PID 或盲目缩小 inflation 更能区分原因。

然后用同一有限真实输入检查：是否旧 SLAM 高度层残留形成机身附近 ghost 占据、是否 floor 支撑面被当障碍、是否局部目标实际越界、是否固定插值 z 导致不可行。确认坐标/支撑面失配后再单独修接口或表面估计，保留障碍和未知空间语义；禁止用真值纠正导航 z、删掉地面/碰撞点、忽略停车样条或延长期限来获得通过。本文不授权参数或算法改动。

配置数值由当前 run 的冻结 `stack.launch.py` 快照和 installed planner.yaml 合并读取，三个 controller/launch 源逐字节等于归档，source/install YAML 也相同；没有另行将配置声明当作运行时 parameter dump。来源 SHA 与机器可读限制见 `SCAN_STOP_SOURCE_AUDIT.json`。
