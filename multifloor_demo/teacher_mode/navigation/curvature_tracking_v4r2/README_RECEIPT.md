# 分阶段接近与主动终点保持独立验收 V4

这是新的真值反馈仿真实验停车语义。旧共同验收与 v1 曲率收据原样保留；其“实际速度命令持续为零”停车门不会被改成通过。本收据允许有界的小非零速度修正，不能用于宣称原零命令停车、真实 SLAM/SCAN、整套多层导航或真机部署通过。Teacher 仍含 232 个特权观测维度，247 个输入；不执行推理重算。

CLI（根任务独占实际运行，验收只读数据）：

```sh
PYTHONDONTWRITEBYTECODE=1 python3 navigation/curvature_tracking_v4/active_hold_receipt.py --run /absolute/teacher_mode/runs/NEW_RUN
PYTHONDONTWRITEBYTECODE=1 python3 navigation/curvature_tracking_v4/test_active_hold_receipt.py
```

只新增 `summary_active_hold_independent.json` 与 `active_hold_independent_arrays.npz`。已有任一输出时拒绝覆盖。schema 是 `independent_truth_teacher_active_endpoint_hold/v4`；顶层包含 `status/checks/metrics/ancestors/verified_input_source_sha256/analyzer_sha256`，score 永远 null。任何安全或驱动失败仍 failed，缺失证据 unverified。profile 中 `active_parking.schema` 必须为 `fixed_endpoint_active_parking/v2`，不把旧候选自动解释成新分阶段控制。

事前固定条件：

- 原共同 13 门中除 `fixed_final_goal_parking` 外的 **12 门全部通过**，并用原归档 evaluator 只读重算；原 parking 状态与 overall status 写入祖先链保留。
- 原 v1 core 的完整 21,098 字节前缀 SHA `82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d` 必须不变；路径、速度、反馈频率、驱动增益、限幅、时限、COM、spawn 与原 profile 逐字段不变。
- 按原 200Hz 原生数据独立重算同一七类动态曲线门：连续曲线不停止转向、完整顺序路径、所有完整 1 秒前进窗口、带符号转向段、路径最大 .20m/RMS .08m/heading .20rad、实际 COM 速度误差 `max(.05,.25*vref)`、原 κv 前馈几何与 .64rad/s 预算。
- 停车目标是事前 `StaticPath.final_xyz` 与 `heading0` 的规范 JSON SHA，不是初次到达实测位置。保留 profile 原 target 的精确 JSON/hash，独立解析几何只作数值绑定：endpoint 最大差 ≤1e-12m，wrapped heading 差 ≤1e-12rad，path 参数 SHA 精确匹配；避免不同 NumPy 版本的约1e-15m四舍五入重新选择 target/hash。测得 arrival 只作诊断；target 与首 capture/hold 声明时间后续不得改变。
- capture 进入半径 .025m/yaw .035rad；保持半径 .03m/yaw .045rad，显式迟滞。实际 origin 平面速度 <.03m/s、Euler yawdot <.06rad/s，按严格递增 fresh feedback 时间驻留 .6s，重复或 held 行不得延长。此门属于控制采样驻留，不假称采样间所有 native 点也满足。首次 capture 独立积分清零一次。
- **第一声明的 `parking_hold_declared_state_time_s` 即唯一停车窗起点，连续 5 秒**，必须有完整 1,001 个 200Hz 原生样本；不得等待 actual command 归零，不得挑后面安静窗口。XY 漂移 ≤.05m、解包 yaw 漂移 ≤.1rad，沿用原数值；停车首次声明比 v1 dwell 更严格且可更晚，不能将更晚时刻的改善直接归因于闭环。
- 新增更严停车峰值门：origin 平面速度 ≤.08m/s、Euler yawdot ≤.1rad/s、body wz ≤.1rad/s；后两量分别记录，不混称。原 truth 共同验收没有峰值门。全原生安全门仍 RP ≤.65rad、clearance ≥.18m、机身接触零、指令扭矩 ≤23.50001Nm、qd ≤30.001rad/s。
- V4 capture 接近阶段：position P=.8、D=.2，reference normXY ≤.05m/s，requested body caps [.15,.07,.10]、XY norm ≤.15。首次声明为 active_hold 的**同一帧**即切换 weak hold：P=.18、D=.2、reference normXY ≤.025m/s、requested caps [.06,.04,.10]、XY norm ≤.06。两阶段 yaw P=.65、D=.18，Euler yawdot reference ≤.07rad/s。限幅 metadata 逐行必须匹配原 mode；PI 仅首次 capture 入口清零，转 hold 不再次清零。
- 实际 Teacher 始终按 [.6,.6,.8]/s 与 .02s 原 slew。独立重建对应阶段的实际命令、原生测量 outer PD、原滤波和连续停车内层 antiwindup PI；held 行不额外积分。实际 command 在阶段切换时可因原 slew 暂保留 capture 量，不冒称立刻满足新 requested caps，也不补裁剪日志。
- 因果相位保持原 native `t-dt` (.005s)。第一次声明 state 是控制计算前 .005s 的缓存状态，其原因果参考可仍 capture；不得把未来 active_hold 命令反向赋到该状态。计算后的窗口全程须是 active_hold 参考。
- 双模拟/墙时 TTL .3s 不变，fresh 反馈严格递增且10/25/50Hz由数学更新计算；Teacher50Hz、native200Hz，CPU 单线程、唯一 `teacher_sim::TeacherActuator` 写执行器。指令扭矩是 feed-to-physics，不是测得电机扭矩。

运行需归档原 references 到 `sources/references/{v1_core.py,v1_curve_receipt.py}`，原 evaluator/protocol 及新 active receipt/test/README、core/worker/run/profiles、policy/native 放入 `source_manifest.json`。profile 的 prospective `frozen_source_hashes`、runtime manifest、模型 SHA、所有祖先输入和数组 SHA 全核对，采用执行归档而非后来的 live 源。v1 helper SHA `14adc95fc60bd65f4b9a8c0dd456c4af680c8315a191016a7531e328582aeb28` 只提供纯测量/几何辅助，不调用其硬编码旧 core 身份的整体验收。StaticPath 数学与控制器共享，实际测量重建独立，不称几何公式完全独立。

旧 tight S 25Hz 原件停车 yaw .11452169698071524rad >.1，固定窗口 37.905..42.905s，XY .004315032312596484m；该原失败永久保留。V2 启动因跨 NumPy 末位 target 比较失败、V3 capture 未在180s内达精度的原失败也保留；V4 只改变接近阶段，严格 capture/hold 驻留、首窗漂移与全原生安全数值不放宽。新收据必须看实际结果，不能据设计宣称主动保持已解决漂移，也不能据停车得分宣称反馈频率全局最优。
