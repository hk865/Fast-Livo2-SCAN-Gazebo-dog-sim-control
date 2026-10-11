# 10 · 胶水代码与门禁:修改建议(含推导过程与复核方法)

> 仓库: `Fast-Livo2-SCAN-Gazebo-dog-sim-control` @ `study/go2-core-20261010` (`2331b93`)
> 范围: **集成层(胶水/适配)与运动门禁**的改造建议。不含算法本身的数学改动。
>
> ⚠️ **本文与前 9 份文档的性质不同**:`00`–`09` 是**结构描述**(读代码即可验证);本文是**判断与建议**(含归纳、口径选择、价值判断)。因此本文必须额外交代**推导过程**,否则读者只能"信"或"不信"。

---

## 0. 本文怎么读 / 怎么复核

```
§1 出发点          —— 这三个结论是为了回答什么问题才产生的
§2 口径定义        —— 数字怎么数出来的(不定义口径的数字不可复现)
§3 推导过程与结论  —— 每节:出发点 → 方法 → 原始数据 → 结论
§4 修改建议        —— 每条附「依据 §3.x 推导」
§5 复核清单        —— 结论 → 可直接执行的检索命令
§6 盲区与不确定性  —— 本方法会漏掉什么、哪些结论是判断而非事实
```

**怎么判断本文是否可信**:

1. **§2 的口径是否合理** —— 如果你不认同某个口径,该口径下的所有数字都要重算,但不影响其它口径的结论。
2. **§5 的命令是否真能复现 §3 的数字** —— 这是最硬的检验。
3. **§6 的盲区是否覆盖了你关心的场景** —— 如果没覆盖,结论对该场景不成立。

---

## 1. 出发点:三个原始问题

本文所有内容都来自这三问,没有第四问:

| 问 | 原话 | 对应的困惑 |
|---|---|---|
| **Q1** | "哪些模块是属于打补丁的?哪些是把他们串起来打补丁的东西?" | 已知 SLAM / SCAN / Go2 是成熟方案,想知道**多出来的那部分有多大、是什么** |
| **Q2** | "哪些门禁可能会导致他不再运动、运动时效?把这些门禁的列表给我列一下" | 实际运行中机器人**莫名停住**(本轮 FAILED),想知道**都有哪些地方能让它停** |
| **Q3** | "哪些代码不优美?或者说应该被修改为可读性更强、比较规范的模块连接与适配代码?" | 想改造,但**不知道从哪下手、哪些值得动** |

**Q1 决定了 §3.1 的方法**(按上游来源分类 + 体积对比)。
**Q2 决定了 §3.2 的方法**(穷举"命令变零"的代码路径)。
**Q3 决定了 §3.3–§3.5 的方法**(找反复出现的结构性问题,而不是逐个文件评价风格)。

> **一个前提说明**:Q1 里"controller 是成熟方案"这个前提**部分不成立**。本仓库有两层控制器:底层 RL 运动策略(成熟产物)与**导航层控制器**(自研,`controller.py`+`shared_controller.py`+`cascade_core.py` ≈151 KB)。SCAN-Planner 自带的 `closed_loop_controller.cpp`(188 行)编译安装了但**本轮从不启动**。所以"补丁"的范围比 Q1 预设的更大。

---

## 2. 口径定义

**不定义口径的数字不可复现** —— 本文初版就有这个问题:同一份代码里 `300_000_000` 全仓有 54 处、`0.35` 有 49 处,而我写的是"4 处""3 处",因为我心里有过滤器却没写出来。以下是本文使用的全部口径。

### 口径 A:上游件 vs 本项目件(用于 §3.1)

- **判定依据**:目录归属 + 仓库 `README.md:43` 声明的三个外部来源(FAST-LIVO2、SCAN-Planner、Go2 simulation)。
- **不依据**代码风格或修改量 —— 上游件也有大量本地修改。
- **单位**:KB(bytes/1024 四舍五入)、文件数。

### 口径 B:"运动门禁"(用于 §3.2)

一条门禁 = **代码中一个能导致以下任一后果的判据(谓词)**:

```
publish_command()        无参调用 → cmd 三字段保持 0
command = [0,0,0]        bridge 改写成零
Emit Stop / Latch       原生插件切断执行机构
stop 请求                通知别的进程停
```

**排除规则**(不纳入计数):
- 只写日志/证据、不改命令的模块(自述 `observer only` 且经核实)
- 起点初始化阶段的等待(尚未开始运动)
- 同一判据的多个调用点**只算一条**(例如 `publish_command()` 在 `if/else` 两分支各出现一次)

> ⚠️ **口径 B 是"判据数",不是"调用点数"。** 两者差距很大:`publish_command()` 无参调用点有 **46 处**(口径 C),而判据只有几十条。**引用数字时必须说明是哪个口径。**

### 口径 C:`publish_command()` 无参调用点(用于 §3.2 的辅助数字)

- 检索式:`publish_command\(\s*\)`(即不带 `velocity` 参数)
- 范围:`ENG/**/*.py`
- **注意**:这**不是**门禁数。它是"改造工作量"的度量 —— 加一个 `report` 模式要改的处数。

### 口径 D:"同语义常量副本"(用于 §3.4)

一个"副本"必须**同时**满足:
1. **同一个物理量**(同一个语义,不是同一个数值)
2. **独立的字面量或独立来源**(不是对同一常量的引用)
3. **在运行时被读取**(声明但无读取者单列为"死配置")

> ⚠️ 这条口径最容易出错。例如 `0.35` 全仓 49 处命中,但其中 **39 处是区域半径 `radius_m`**,与射线残差门**语义无关**。所以必须先按语义筛,再数副本。

### 口径 E:"死代码 / 死配置"(用于 §3.5)

- **死配置**:在 profile / 契约 JSON 中被声明,但全仓**无任何读取点**
- **死代码**:有完整定义(类/函数),但**无实例化点或无调用点**
- **死可执行文件**:被 `CMakeLists.txt` 编译并安装,但**不在任何 launch 文件中**

---

## 3. 推导过程与结论

### 3.1 规模对比 → 包裹层大于被包裹的算法

**出发点**:Q1。

**方法**:按口径 A 分类,统计各类的 KB 与文件数。

**原始数据**(复核命令见 §5.H1):

| 组 | KB | 文件数 |
|---|---:|---:|
| FAST-LIVO2 core(`multifloor_demo/slam/ros2_ws/src/fast_livo2_core`) | 410 | 34 |
| SCAN 后端(`multifloor_demo/navigation/ros2_ws/src`) | 405 | 44 |
| Vikit(`slam5_navigation/ros2_ws/src`) | 181 | 47 |
| Go2 描述/仿真(`go2_sim_control`) | 56 | 19 |
| **上游合计** | **1052** | **144** |
| `ENG/`(本项目核心,含 `profiles/`、`policy_candidate/`) | **1216** | **84** |

**结论**:本项目自研层(`ENG/` 单目录)体积 **1216 KB > 四个上游件合计 1052 KB**。

**这一步是事实,不是判断。** 判断在下一条。

**进一步的分类**(这一步含判断,依据是**文件 docstring 自述**):

按"它为什么存在"把 `ENG/` 及其它自研件分为四类:

| 类 | 判定依据 | 代表 |
|---|---|---|
| A 串接胶水 | 存在原因是**三套成熟件的坐标系/时间基/QoS/进程边界不一致** | `odom_adapter`、`self_echo_filter`、`sensor_relay`、`trajectory_contract`、`bridge`+socket |
| B 架构级补丁 | 存在原因是**成熟 SLAM 单独跑会漂** | `known_scene_*`(8 文件) |
| C 故障驱动加固 | **docstring 直接写着事故形状** | `p95_capture`、`storage_recovery`、`clock_hold`、`*_shutdown` |
| D 工程治理 | 存在原因是**需要证明"这次跑的是被授权的那份代码"** | `run.py`、`scoped_profile`、`*_CONTRACT.json` |

**C 类的判定方法值得单独说明**:不靠猜,靠 docstring 的自述语言。例如:

- `storage_recovery.py:1` — "One explicitly reviewed recovery epoch; **historical storage is never refunded**" → 磁盘爆过
- `known_scene_stale_recovery.py:1` — "fail-closed proposal for explicitly aborting an **expired frozen transaction**" → 就是本轮这个故障
- `guardian_archive_shutdown.py:1` — "Observe worker EOF/archive close after owned physics stop, **never signal**" → 归档被截断过
- `clock_hold.py:1` — "A duplicate clock **can only emit exact zero**" → 重复时钟导致重复积分

**规律**(可复核):带 `_capture` / `_evidence` / `_recovery` / `_guard` / `_shutdown` / `_ledger` / `_trace` / `_witness` 后缀的,几乎都是事后加的。复核命令见 §5.H2。

---

### 3.2 门禁枚举 → 46 个调用点 / 15 条 bridge 判据 / 12 处原生 latch

**出发点**:Q2。

**方法**(三步):

1. **找"命令变零"的位置** —— 检索 `publish_command()`、`command=[0,0,0]`、`Twist()`、`stop_requested`、`Emit Stop`、`Latch(`
2. **反查每个位置的触发谓词** —— 逐处 `read` 上下文,提取条件
3. **按执行顺序分层** —— 任务 → 定位授权 → 时钟 → 控制器 → 级联 → IPC/执行

**原始数据**(全部实测,复核命令见 §5.H3):

| 口径 | 数量 | 明细 |
|---|---:|---|
| B 门禁判据 | **约 64 条**(见下表分层) | 含并集/intersection 的合并,见下方说明 |
| C `publish_command()` 无参调用点 | **46** | `shared_controller.py` 21、`controller.py` 14、`teacher_wrapper.py` 6、`known_scene_inputs.py` 4、`clock_hold.py` 1 |
| bridge `reasons.append` | **15** | `bridge.py`,覆盖 19 个失败条件(5 个是 `self.failed=` 赋值,经 `:226` 汇总) |
| 原生 `Latch()` 调用点 | **12** | 8 个不同故障码,见 §3.2.4 |

> ⚠️ **订正**:本文初版写"70 条门禁"与"19 条 reason",前者口径未定义、后者把 5 处 `self.failed=` 赋值与 15 处 `append` 混在一起重复计数。**以本节口径为准。**

#### 3.2.1 第 1 层:任务层(后果:通知别的进程停)

**出发点**:命令链的最上游是任务层,它不直接发速度,但能请求停机。

| # | 门 | 触发条件 | 位置 |
|---|---|---|---|
| 1 | 区域进度死线 | 单区 90 s 未通过(生效值是 `Goal.timeout_sim_s`) | `continuous_route.py:93-95` → `known_scene_inputs.py:99-102`、`controller.py:748-752` |
| 2 | 阶段超时 | `stage_timeout_s` 1500 s | `state_machine.py:174` |
| 3 | 全局仿真上限 | `SIMULATION_MAX_S` 1500 s | `mission46.py:115-118` |
| 4 | 启动超时 | `StartupDeadline` | `request.py:137-149` |
| 5 | 任务 hold | `mission46_control.json` / 地形 ACK 时间戳不新鲜 | `mission46_guard.py:11-43` |
| 6 | 任务停 | `/demo/navigation/stop` | `mission46_runtime.py:291` → `shared_controller.py:375-383` |
| 7 | 地形切换 | 切换期间 Actor 侧强制零命令 | `worker.py:172-186` |
| 8 | 紧急 owner-loss | 看门狗判定 owner 丢失 | `owned_guardian.py:143-211` |

#### 3.2.2 第 2–5 层:定位授权 / 时钟 / 控制器 / 级联

**出发点**:本轮 FAILED 的直接链条在**第 2 层**(定位授权),但排查时必须知道全貌。

| # | 层 | 门 | 触发条件 | 位置 |
|---|---|---|---|---|
| 9 | 定位 | **三件套全等门** | 三输入各 `0 ≤ now−received < 0.3 s` **且** generation 三方相等 **且** `pose_stamp == cloud_stamp == source_ns` | `known_scene_inputs.py:84-90` → `:113` |
| 10 | 定位 | `control_allowed=false` | `transition or correction_stale`(TTL 3 s) | `known_scene_runtime.py:229`,`:234` 下发 |
| 11 | 定位 | 事务进行中 | 修正效应 >0.04 m / 0.025 rad 触发 `transition` | `known_scene_runtime.py:45-46` |
| 12 | 定位 | 未初始化 | `initialized=false` | `known_scene_runtime.py` |
| 13 | 定位 | SCAN 栅格重建期 | 定位事务触发重建,`ready=False` | `scan_epoch_supervisor.py:110,112,118` |
| 14 | 定位 | 等待帧内超时 | 等待期间照跑死线与 8 s 检查,**直接 failed 不可恢复** | `known_scene_inputs.py:99-112` |
| 15 | 定位 | **修正事务开始 → 立即停车并作废旧路径** | `transition` 由假变真(`on_localization_status`),**不咨询 #9 全等门** | `known_scene_inputs.py:59`(零命令)、`:60-65`(同时作废 samples/cascade/路径/参考) |
| 16 | 时钟 | 重复时钟 | 同 tick ROS clock 与上次相同 | `clock_hold.py:46-64` → `:163` |
| 17 | 时钟 | 时钟倒退 | 检测到倒退 → 直接 failed | `clock_hold.py:133-135` |
| 18 | 时钟 | 时钟停滞 | ≥300 ms 无推进(`TTL_NS`) | `clock_hold.py:15-16,59-62` |
| 19 | 时钟 | 连续失联 8 s | | `clock_hold.py:148-152` |
| 20 | 时钟 | 证据写失败致命化 | `evidence.error` 被放大成致命 | `clock_hold.py:130-135` |
| 21 | 控制 | 状态非 running | `state != 'running'` | `controller.py:747` |
| 22 | 控制 | 段死线超时 | → `failed` | `controller.py:748-752` |
| 23 | 控制 | **倾角保护** | ≥0.50 rad → failed;≥0.30 → hold;恢复需 <0.18 且 0.8 s | `controller.py:753-755`、`shared_controller.py:186-226`、`teacher_wrapper.py:135-154` |
| 24 | 控制 | **原始 IMU 缺失/不新鲜** | `raw_imu.fresh()` 为假 | `control_core.py:28,56-58` |
| 25 | 控制 | 传感器超时 | pose/cloud > `pose_cloud_timeout_s`(0.3 s) | `shared_controller.py:756-762` |
| 26 | 控制 | **连续失联 8 s** | 同上持续 8 s → `failed` | `shared_controller.py:761` |
| 27 | 控制 | 桥安全 hold/failed | `/demo/control/safety` | `teacher_wrapper.py:155-157` |
| 28 | 控制 | **接纳 hold** | `path_admission` 证不出曲线包含性 | `controller.py:416-437,467-474,800` |
| 29 | 控制 | 轨迹配对失败 | `trajectory_contract` 返回 None → `samples=None` → 零 + 3 s 重规划 | `trajectory_contract.py:83-95` → `shared_controller.py:672-678` |
| 30 | 控制 | **障碍 hold** | 点走廊 ∪ 0.8 m 转向走廊有障碍;恢复需连续 1.0 s 无阻塞 **且**实测停车 | `shared_controller.py:742-805` |
| 31 | 控制 | 到达 / pre_turn / settle | 到点与航向转换序列 | `shared_controller.py:634,788-791` |
| 32 | 控制 | 路径耗尽 / 参考不一致 | | `shared_controller.py:691-698,784-787` |
| 33 | 控制 | 无参考 | `samples is None` | `shared_controller.py:583,594,601,607,616,619` |
| 34 | 控制 | 早期返回哨 | 若干发布前短路径 | `controller.py:95,432,472,689,717,814,819,821,886` |
| 35 | 级联 | `_protect` 保护位 | | `cascade_core.py:271-287` |
| 36 | 级联 | 失败锁存 | | `cascade_core.py:403-408` |
| 37 | 级联 | `reference_constraint_hold` | | `cascade_core.py:562-565` |
| 38 | 级联 | `path_end_hold` | | `cascade_core.py:572-573` |
| 39 | 级联 | 级联主保护分支 | | `cascade_core.py:400-425` |

#### 3.2.3 第 6 层:bridge(15 条 `reasons.append` → `command=[0,0,0]`)

**方法**:`bridge.py` 的 `snapshot()` 收集 `reasons`(列表),末尾 `:286-292` 一次性折叠:

```python
healthy = not reasons
command = clamp(requested) if healthy else [0., 0., 0.]
stop_requested = not healthy or not any(command)
```

所以**每一条 reason 都是一条独立门禁**,只是共用同一个出口。`TIMEOUT = .300` 定义在 `:30`。

| # | reason | 行 |
|---|---|---|
| 40 | Invalid simulation clock | `:171` |
| 41 | Simulation clock moved backward | `:173` |
| 42 | SCAN command nonfinite | `:204` |
| 43 | SCAN command requests unsupported z/roll/pitch | `:208` |
| 44 | `self.failed` 通用汇总 | `:226` |
| 45 | Cascade provenance rejected | `:236` |
| 46 | CHAMP executor feedback 不健康/不新鲜 | `:243` |
| 47 | CHAMP executor failed | `:244` |
| 48 | sensor gate / SLAM anchor 发布链不唯一 | `:246` |
| 49 | SLAM 未完成健康预热 | `:249` |
| 50 | 等待不可变 SLAM 路线 anchor | `:251` |
| 51 | 实测位姿离开路线围栏 → failed | `:269-270` |
| 52 | ROS 发布者图检查过期 | `:272` |
| 53 | SLAM 源必须恰好一个 `demo_slam_odom_adapter` | `:274` |
| 54 | 命令源必须恰好一个 `demo_navigation` | `:276` |
| 55 | `self.pose_error` | `:278` |
| 56 | 等待两个连续有效 SLAM 位姿 | `:280` |
| 57 | `self.command_error` | `:282` |
| 58 | 任一传感器 age 缺失/过期/不一致 | `:285` |

> 上表 19 行 vs 15 个 `append`:5 个 `self.failed=` 赋值(`:171,173,204,208,269`)经 `:226` 汇总输出,所以**条件 19 个、出口 15 个**。

#### 3.2.4 第 6 层:原生插件(12 处 `Latch()` → 8 个故障码)

**方法**:检索 `Latch(`。`:336` 是定义,**其余 12 处是调用点**。

| 故障码 | 原因 | 行 |
|---:|---|---|
| 1 | socket_creation_failed | `:348` |
| 1 | socket_connect_failed | `:355` |
| 3 | socket_timeout_disconnect_or_partial_frame | `:386` |
| 4 | nonfinite_or_invalid_inference_response | `:394` |
| 5 | inference_requested_fault_damping | `:396` |
| 6 | missing_or_invalid_native_physics_state | `:199` |
| 6 | missing_native_contact_component | `:204` |
| 6 | joint_velocity_outside_DCMotor_envelope | `:238` |
| 7 | physics_timestep_or_clock_contract_violation | `:175` |
| 8 | multiple_joint_actuator_plugins | `:169` |
| 9 | actuator_evidence_write_failed | `:309` |
| 10 | diagnostic_native_body_or_nonterrain_foot_collision | `:208` |

**后果与其他层不同**:`Latch()` 置 `fault` 后,`:232`/`:239` 把力矩切成 `−faultDamping·qd`(阻尼),`:337` 停止交换。**这是物理切断,不是"发零命令"。**

#### 3.2.5 结果形态分类(改造时后果不同)

| 形态 | 含义 | 例子 |
|---|---|---|
| **A** | 精确零命令但**不改写状态** | #9 → `known_scene_inputs.py:113` |
| **B** | 零命令 **+ 状态置 `failed`** | #22、#26 |
| **C** | **不推进轨迹/不积分** | #16–#20(时钟)、#31(pre_turn) |
| **D** | **改写成零但控制逻辑继续跑** | #40–#58(bridge) |
| **E** | **直接切断执行机构** | §3.2.4 全部 |
| **F** | **请求别的进程停** | #1–#8 |

---

### 3.3 三个跨文件耦合点(含本文两处错误结论的订正)

**出发点**:Q2 的延伸 —— 用户的目标是"把门禁改成只上报",所以必须找出**跨门的结构性问题**(改单门无效的那些)。

**方法**:不新增检索,而是**检查 §3.2 的门之间是否有共享状态或并行路径**。

> ⚠️ **本节初版的三个结论里有两个是错的。** 订正记录如下 —— 保留它,因为它就是 §6.7 的由来:
>
> | 初版结论 | 复核结果 | 依据 |
> |---|---|---|
> | 陷阱 1:`bridge.py:292` 构成自锁回环,零命令被持久化为 stop 状态 | **不成立** | `stop_requested` 从未参与 `reasons`/`healthy` 构造,不存在反馈路径 |
> | 陷阱 2:`execution_frozen` 是第二套停车,不同步改会冻死规划侧 | **不成立** | 规划侧用它做"参考点跟随",不是冻结规划 |
> | 陷阱 3:300 ms fail-closed 分散 4 处且字面量不一致 | **成立** | grep 实测 |
>
> 两处错误的原因相同:**把"看到两个东西语义相关"当成了"推断了因果关系"**。这正是 §0 要求"每个结论附推导、附复核命令"的原因。

#### 耦合点 1(订正后):`bridge.py:292` 把两种语义混进一个字段

**初版写法(错误)**:称其为"自锁回环":

```python
stop_requested = not healthy or not any(command)
```

**初版推导**:零命令 → `stop_requested=True` → `worker.py:165` 置零 → 下帧又 `not any(command)` → 永久停。

**复核为什么错**:

1. `healthy = not reasons`(`bridge.py:286`),而 `reasons` 的全部来源(`:226`、`:236`、`:243-244`、`:246`、`:249`、`:251`、`:269-270`、`:272`、`:274`、`:276`、`:278`、`:280`、`:282`、`:285`)**没有一条依赖 `stop_requested`,也没有一条依赖"命令是否为零"**。
2. `stop_requested` 只被三处消费:`:465` 发布 Bool、`:502` 写日志、`worker.py:165` 与 `policy_candidate/worker.py:138` 的**逐帧**判断。
3. 没有任何一处把它**存为状态**。下一帧由 `reasons` 重新计算。**不存在自锁。**

**订正后的真实结论(性质不同,影响也不同)**:

`stop_requested` 把两种不同的事情压进了同一个字段:

| 语义 | 条件 | 是否真的是"被请求停车" |
|---|---|---|
| 门控不健康 | `not healthy` | 是 |
| 命令恰好为零 | `not any(command)` | **否** —— 正常停车、保护 hold、到点驻留都会这样 |

**后果是归因错误,不是控制错误**:

- `/demo/teacher/navigation_bridge/stop`(`:465`)与命令信封里的 `stop_requested` 会把**正常停车标成"被请求停车"**
- `worker.py:165` / `policy_candidate/worker.py:138` 会走 `'SLAM/SCAN requested controlled stop'` 分支 —— **日志上的原因与实际不符**

**仍然建议拆成两个字段**,但理由是**归因正确性**,不是"防自锁"。这改变了 §4 表格第 1 行的性质:不是"修控制 bug",而是"修可观测性"。

#### 耦合点 2(订正后):`execution_frozen` 是"参考点跟随"耦合,改造时必须一起考虑

**初版写法(错误)**:称它是"与零命令并行的第二套停车路径",会"把规划侧冻死"。

**实际链路**(行号已核实):

```
controller.py:874     execution_frozen = state!='running' or obstacle_hold or |v[:2]|<.015
                      ↓ /demo/navigation/execution_frozen (shared_controller.py:126)
scan_epoch_supervisor.py:12-15  重映射 planning/go2_execution_frozen → /demo/navigation/execution_frozen
                      ↓
scan_replan_fsm.cpp:84-85       订阅 planning/go2_execution_frozen
scan_replan_fsm.cpp:434-435     go2_execution_frozen_ = msg->data; if (frozen) measured_start_.stop()
scan_replan_fsm.cpp:444-449     冻结期间每 tick 执行 info->start_time_ += dt
scan_replan_fsm.cpp:221,414,776,782   用于重规划起点的速度估计
```

**订正后的真实结论**:`execution_frozen` 在规划侧的作用有两条——

1. **把重规划起点的实测速度归零**(`measured_start_.stop()`,`:435`)
2. **冻结期间让轨迹的 `start_time_` 跟着墙钟走**(`:444-449`),净效果是**参考点停在原处,不随墙钟向前跑**

也就是:**它实现的是"机器人不动时,参考点也别动"** —— 这是一个**正确的耦合**,不是陷阱。

**但改造时它确实是必须一起考虑的耦合点**:一旦让控制器"只上报、不停车",会出现两种失配——

| 失配 | 后果 |
|---|---|
| 机器人真在动,但 `:874` 仍报 frozen | 参考点停在原处 → 控制器持续追一个落后的参考 |
| 机器人被 hold,但 `:874` 报 not frozen | 参考点随墙钟跑远 → 恢复瞬间位置误差很大 |

所以正确做法不是"同步改 `:874`",而是**保证 `:874` 的语义与"机器人是否真的在动"始终一致**。

#### 耦合点 3(成立):300 ms fail-closed 分散在 4 个文件,而且字面量**不一致**

**发现路径**:这是本文初版**出错**的地方,值得作为方法学案例说明。

初版写"四处独立的 300 ms"是**凭印象**。实测:

| 文件 | `300_000_000` 命中 | 实际时效表达 |
|---|---:|---|
| `bridge.py` | 4(`:102,105,241,418`) | `300_000_000` |
| `worker.py` | 1(`:158`) | `300_000_000` |
| `policy_candidate/worker.py` | **0** | **`.3`**(`:134`,以及 `:84`、`:97`) |
| `cascade_core.py` | 5(`:309,411,412,518,525`) | `300_000_000` |

**结论**:口径 D 在此**必须放宽** —— 同一语义(命令/源时效 300 ms)在不同文件里用了不同字面量(`300_000_000` vs `.3`)。

**这使问题比初版描述的更严重**:统一 grep **找不到全部副本**。

---

### 3.4 同语义常量副本 → 6 组

**出发点**:Q3(为什么"改了一处不生效")。

**方法**:按口径 D,先按语义筛,再数字面量。

**原始数据与结论**:

| 语义 | 字面量处数 | 文件数 | 关键位置 | 备注 |
|---|---:|---:|---|---|
| 3 s 修正 TTL | **7** | **5** | `known_scene_matcher.py:419`(**唯一生效**)、`event_aware_source_capture.py:19`、`known_scene_route_registration.py:139`、`known_scene_stale_recovery.py:16,38,48`、`known_scene_adapter.py:120` | `adapter:120` 的 `maximum_source_age_ns` **是另一个语义**(源槽龄,不是匹配龄) |
| 300 ms 命令/源时效 | ≥10 | 4 | 见 §3.3 耦合点 3 | 字面量不统一(`300_000_000` / `.3`),grep 无法统一检索 |
| 0.35 m 射线残差门 | **3** 处声明 | 3 | `known_scene_matcher.py:53`、`event_aware_source_capture.py:19`、`EVENT_CAPTURE_CONTRACT_TEMPLATE.json:21` | 其余 39 处 `0.35` 命中是**区域半径**,语义无关 |
| 雷达外参 | 3 | 3 | `known_scene_adapter.py:121`、`known_scene_matcher.py:303`、`scenario.json:308` | 代码 2 份 + 场景 1 份;`worker` 调 `match` 时**不转发 mount** |
| 90 s 区域死线 | ≥3 | 3 | `route_regions.py:56`(**唯一生效**)、`profiles/engineering_nav46.json:7`、`:3693`(**无读取者**) | 两处声明是死的 |
| 0.3 s 新鲜度 | 3 | 2 | `known_scene_runtime.py:200,203`(**硬编码**)、`known_scene_inputs.py:87`(**硬编码**)、`profiles/...json:8` | 代码里硬编码,**不读 profile** |

> ⚠️ **订正**:初版写"3 s ×5"是把"5 个文件"当成了"5 份"。按口径 D,实际是 **7 处字面量、5 个文件**,且其中 1 处语义不同。**有效的副本是 6 处,跨 4 个文件。**

---

### 3.5 死代码 / 死配置清单

**出发点**:Q3 的延伸 —— 未接线的东西会误导判断。

**方法**:按口径 E,逐个反查调用点/读取点。

**原始数据**(实测):

| 项 | 定义处 | 实例化/读取点 | 判定 |
|---|---:|---:|---|
| `pid_core.HeaderPID` | 1(`pid_core.py:18`) | **0** | 死代码(完整 PID,**从未运行**) |
| `region_progress_deadline_s` | 3 | **0** | 死配置 |
| `whole_session_budget_bytes` | 1 | **0** | 死配置 |
| `mission46_source_timeout_s` | 2 | **0** | 死配置 |
| `sensor_gate.py:48` `warmup_ns` | 1 | **0** | 只赋值不读 |

**这一类里 `HeaderPID` 的危害最具体**:它导致了一次**实际误判** —— 在讨论"加 PID 跟随命令"时,它看起来就是那个 PID,但 `controller.py:8` 只导入了同文件的 `slew`,`HeaderPID` 全仓零实例化。

**其它未接线项**(判定依据是"编译/安装但不在任何 launch 文件中",非口径 E):

- `open_loop_controller`、`go2_kinematic_sim`、`go2_gait_publisher` —— `plan_manage/CMakeLists.txt:40-52` 编译安装,`ENG/stack.launch.py` 不含
- 三套手写 bridge yaml(`gz_main_bridge.yaml`、`gz_perception_bridge.yaml`、`simulation/config/bridge.yaml`)—— 生效的是运行时生成的 `run/sensor_bridge.yaml`
- `DataDisp.a..e`、`time_search_/time_optimize_/time_adjust_` —— 只声明不写入
- `GradientDescentOptimizer`、`BsplineOptimizeTraj` 等 —— 只声明无定义

---

## 4. 修改建议(每条附依据)

| # | 建议 | 依据 | 风险 |
|---|---|---|---|
| 1 | 拆 `bridge.py:292`:`stop_requested` 与 `not any(command)` 分两个字段 | §3.3 耦合点 1 | 低(**改进归因,不改控制**) |
| 2 | **把 §3.2 的门禁如实搬进声明式门禁表,行为不变** | §3.2 全节 | 低 |
| 3 | 加 `Mode.ENFORCE \| REPORT` 开关,默认 ENFORCE | §3.2(46 个调用点) | 低 |
| 4 | 让 `controller.py:874` 的语义与"机器人是否真的在动"保持一致 | §3.3 耦合点 2 | 中 |
| 5 | 300 ms 四处收敛到 `contracts/limits.py` | §3.3 耦合点 3 | 中 |
| 6 | 常量唯一真源(6 组副本) | §3.4 | 低 |
| 7 | 显式端口替代话题劫持 | 见 [00 号文档 §6.1](00-overview-dataflow.md) | 中 |
| 8 | 显式适配器替代 monkey-patch | 见 `known_scene_inputs.py:18-23`、`teacher_wrapper.py:88-92`、`ENG/worker.py:41-43` | 高 |
| 9 | 控制律与取证分离 | `cascade_core.py:314` 增益写在函体内 | 高 |
| 10 | 删除或接线死代码 | §3.5 | 低 |

### 安全分档(判断,依据见下)

| 档 | 门(#) | 判定依据 |
|---|---|---|
| 🔴 **物理安全锁** | 23、24、25、26、30、47,以及 §3.2.4 全部 | **后果是"运动会导致实际损伤"** —— 倾角超限会翻、障碍不看会撞、原生 latch 会切断力矩 |
| 🟡 **逻辑/工程门**(可先改只上报) | 9–22、28、29、31–39、45、48–58 | **后果是"不确定自己在哪/证据不完整"**,但本体状态是好的 |
| 🟠 **流程/任务门**(需单独决策) | 1–8、51 | **静默 = 任务永不失败**;#51 静默 = 允许跑到注册路线之外 |

> 这个分档是**判断**,不是事实。判定依据只有一条:**静默后机器人的物理行为是否会变危险**。如果你不同意某条的归类,改档即可,不影响其余结论。

---

## 5. 复核清单(可直接执行的命令)

以下命令均在**仓库根目录**下执行。
`$ENG = multifloor_demo\teacher_mode\navigation\engineering_v34_r44_active_p95_capture_candidate`

### H1 —— 验证 §3.1 的体积对比

```powershell
function Sz($p){ $f=Get-ChildItem $p -Recurse -File; [pscustomobject]@{KB=[math]::Round((($f|Measure-Object Length -Sum).Sum)/1KB,0);Files=$f.Count} }
Sz 'multifloor_demo\slam\ros2_ws\src\fast_livo2_core'   # 期望 410 KB / 34
Sz 'multifloor_demo\navigation\ros2_ws\src'              # 期望 405 KB / 44
Sz 'slam5_navigation\ros2_ws\src'                        # 期望 181 KB / 47
Sz 'go2_sim_control'                                     # 期望  56 KB / 19
Sz $ENG                                                  # 期望 1216 KB / 84
```

### H2 —— 验证 §3.1 的"故障驱动"命名规律

```powershell
Get-ChildItem $ENG -Filter *.py |
  Where-Object { $_.Name -match '_capture|_evidence|_recovery|_guard|_shutdown|_ledger|_trace|_witness|_audit' } |
  ForEach-Object { $_.Name }
# 期望 15 个文件命中
```

### H3 —— 验证 §3.2 的三个计数

```powershell
# 口径 C:46 个无参 publish_command 调用点
Get-ChildItem $ENG -Recurse -Filter *.py |
  ForEach-Object { [pscustomobject]@{
      File=$_.Name
      N=(Select-String -LiteralPath $_.FullName -Pattern 'publish_command\(\s*\)' -AllMatches).Matches.Count } } |
  Where-Object N -gt 0 | Sort-Object N -Descending
# 期望: shared_controller 21 / controller 14 / teacher_wrapper 6 / known_scene_inputs 4 / clock_hold 1  → 46

# bridge reasons.append = 15
(Select-String "$ENG\bridge.py" -Pattern 'reasons\.append').Count

# 原生 Latch( = 13 处(含 1 处定义 → 12 个调用点)
(Select-String 'multifloor_demo\teacher_mode\simulation\native\current\teacher_actuator.cpp' -Pattern 'Latch\(').Count
```

### H4 —— 验证 §3.3 的三个陷阱

```powershell
# 耦合点 1:stop_requested 的两个语义
# 期望:stop_requested 只出现在 bridge.py:292(定义)、:465、:502,以及两个 worker 的消费点
#       关键是要确认它【不参与 reasons 构造】—— 即它不出现在 :224-285 之间
Select-String "$ENG\bridge.py" -Pattern 'stop_requested'
Select-String "$ENG\bridge.py" -Pattern 'reasons\.append'      # 期望 15,且无一含 stop_requested
Select-String "$ENG\worker.py" -Pattern 'stop_requested'
Select-String "$ENG\policy_candidate\worker.py" -Pattern 'stop_requested'

# 耦合点 2:execution_frozen 在规划侧的真实作用(必须读到消费端才算核实)
Select-String "$ENG\controller.py" -Pattern 'execution_frozen'
Select-String "$ENG\shared_controller.py" -Pattern 'execution_frozen'
Select-String "$ENG\scan_epoch_supervisor.py" -Pattern 'execution_frozen'
Select-String 'multifloor_demo\navigation\ros2_ws\src\plan_manage\src\scan_replan_fsm.cpp' -Pattern 'execution_frozen|measured_start_'
# 关键两行: :435 measured_start_.stop()  与  :448-449 info->start_time_ += dt(参考点跟随)

# 耦合点 3:300 ms 字面量不统一(关键:注意 policy_candidate 命中 0)
foreach($f in 'bridge.py','worker.py','policy_candidate\worker.py','cascade_core.py'){
  "{0,-28} {1}" -f $f,(Select-String "$ENG\$f" -Pattern '300_000_000').Count }
Select-String "$ENG\policy_candidate\worker.py" -Pattern 'wall_age|sim_age'
```

### H5 —— 验证 §3.4 的常量副本

```powershell
# 3 s TTL:期望 7 处 / 5 文件
Get-ChildItem $ENG -Recurse -File | Select-String -Pattern '3_000_000_000' |
  ForEach-Object { "{0}:{1}" -f $_.Filename,$_.LineNumber }

# 0.35:期望 49 处命中,但只有 3 处是"残差门声明",其余是区域半径
(Get-ChildItem $ENG -Recurse -File | Select-String -Pattern '0\.35\b').Count
Get-ChildItem $ENG -Recurse -File | Select-String -Pattern 'ray_abs_p95|allray_abs_p95' |
  ForEach-Object { "{0}:{1}" -f $_.Filename,$_.LineNumber }

# 外参:期望 3 处 / 3 文件
Get-ChildItem $ENG,'multifloor_demo\simulation' -Recurse -File | Select-String -Pattern '\.1177' |
  ForEach-Object { "{0}:{1}" -f $_.Filename,$_.LineNumber }
```

### H6 —— 验证 §3.5 的死代码

```powershell
# HeaderPID:期望 定义 1 处 / 实例化 0 处
Get-ChildItem $ENG -Recurse -File | Select-String -Pattern 'HeaderPID' | ForEach-Object { "{0}:{1}" -f $_.Filename,$_.LineNumber }
Select-String "$ENG\controller.py" -Pattern '^from pid_core'    # 期望只有 slew

# 无读取者的配置:期望只在 profile + 契约里出现
foreach($k in 'region_progress_deadline_s','whole_session_budget_bytes','mission46_source_timeout_s'){
  "{0,-32} {1}" -f $k,(Get-ChildItem $ENG -Recurse -File | Select-String -Pattern $k).Count }
```

---

## 6. 盲区与不确定性

**§6 存在的意义**:上面每条结论都有适用范围。以下是本方法的已知边界。

### 6.1 方法是**静态**的 —— 无法区分"可达"与"不可达"

本文所有门禁都来自**读代码**。如果某条门禁在当前配置下**永远不会触发**(例如被上游条件挡住),它仍然会被列进来。**本文没有做可达性分析。**

**影响**:§3.2 的计数可能**高估**了实际生效的门禁数。

### 6.2 计数口径依赖,换口径数字就变

同一份代码:

| 口径 | 数字 |
|---|---|
| 门禁判据 | ≈64 |
| `publish_command()` 无参调用点 | 46 |
| bridge `reasons.append` | 15(覆盖 19 个条件) |
| 原生 `Latch()` 调用点 | 12 |

**本文初版混用了这些口径,产生过"70 条""44 个""19 条"三个错误数字。** 引用时必须带口径。

### 6.3 §3.1 的四类划分含判断

"A 胶水 / B 补丁 / C 故障驱动 / D 治理"的边界不是客观的。判定依据只有两条:**上游来源声明**(README)与**docstring 自述**。若某文件的 docstring 是后补的,分类会错。

### 6.4 §4 的安全分档是**判断**,不是事实

"物理安全锁 / 逻辑门 / 流程门"的判定依据只有一条:静默后物理行为是否变危险。**这是价值判断。** 例如 #25(传感器超时)—— 静默它意味着"用旧位姿继续走",有人会视为逻辑门,也有人会视为安全锁。**换档不影响其余结论。**

### 6.5 未覆盖:运行时行为

本文**没有运行过任何东西**(快照本身也跑不起来 —— 缺 `R33_SHORT_STARTUP_CONTRACT.json`、`STORAGE_SESSION.json`、两个 workspace 的 `build/install`、策略权重等,清单见 [09 号文档 §9](09-runtime-orchestration-and-archive.md))。

所以:

- **"哪个门禁在本轮实际触发过"** —— 本文**不知道**,只能从 `docs/C44_LOG_EXCERPT.log:24` 读出定位授权链这一个。
- **"改了之后会不会引入新问题"** —— 本文**无法回答**,§4 的风险列只是基于代码结构的估计。

### 6.6 未覆盖:跨文件的运行时耦合

§3.3 的耦合点全部是**读代码推断**出来的,没有运行时验证。可能存在其它同类耦合没被发现。**判定"改单门无效"的唯一可靠方法是实际跑一次** —— 这也是 §4 建议先做"行为不变的门禁表搬迁"的原因。

### 6.7 本文自身出过的错(方法学记录)

这一节记录本文的**实际错误**,因为它是"为什么 §0 要求附推导与复核命令"的最好证据。

**本文件被推送后,按"补推导"的要求复核,发现 §3.3 的三个结论里有 2 个是错的:**

| 错误结论 | 错在哪 | 性质 |
|---|---|---|
| 陷阱 1:`bridge.py:292` 自锁回环 | 从未检查 `stop_requested` 是否参与 `reasons` 构造。实测:不参与,无反馈路径 | **虚构了因果关系** |
| 陷阱 2:`execution_frozen` 冻死规划侧 | 从未读 `scan_replan_fsm.cpp` 的消费端,只看到"有独立链路"就断言是"第二套停车" | **在没读消费端的情况下断言后果** |

**共同错误模式**:**观察到"语义相关"就断言"存在因果",把推断当成了事实。**

**另外,通过实测还订正了初版 3 个数字错误**:`publish_command()` 调用点 44→**46**;bridge reason 19→**15 个出口 / 19 个条件**(初版重复计数 `self.failed=` 赋值);3 s TTL "5 份"→**7 处字面量 / 5 文件**(初版把"文件数"当成了"份数")。

**这对读者的实际意义**:

1. §3.1、§3.2、§3.4、§3.5 的数字都经过 grep 实测,可以信,但**请按 §5 自己跑一遍**。
2. §3.3 现在只剩**耦合点 3 是实测成立的**;耦合点 1、2 是订正后的定性描述,**仍属读代码推断**,置信度低于前三节。
3. §4 的"风险"列是**结构估计**,不是实测。
4. 凡本文出现"会""必然""导致"这类**因果断言**,请先确认它后面跟了 `文件:行号` 与可复核的推导 —— **没有的,就是本文没验证的。**

---

## 7. 一句话判断(以及它为什么是判断)

> 这套代码不是"写得差",是"没有在正确的时候回头重构"。

**支撑它的事实**:§3.1 的体积对比、§3.2 的 46 个调用点、§3.4 的 6 组常量副本 —— 这些可复核。

**属于判断的部分**:
- "作者懂行" —— 依据是 §3.1 提到的纯函数核隔离与 fail-closed 纪律(可复核),但"懂行"本身是评价
- "应该抽象门禁" —— 这是**方案选择**,不是唯一解。也可以选择继续加 `if`,代价是 §4 表格里的可读性与归因成本
- "选门禁表作为第一刀" —— 依据是"风险最低 + 收益最大"的权衡,如果优先级不同,结论会变

**如果只能改一件事**,我仍然选门禁表:它同时解决可读性、`report` 模式、事故归因(依据见 §4 表格第 2 行),并且 §4 的第 1、3、4、5 步都依赖它。

> **但这个"仍然"的置信度已经下降**:§3.3 的两处错误说明,**未经运行时验证的结构性断言不可靠**。如果 §4 第 2 步(行为不变的门禁表搬迁)能先跑起来,它会顺带把本文其余的结构性推断一起证伪或证实 —— 那比继续读代码更有价值。
