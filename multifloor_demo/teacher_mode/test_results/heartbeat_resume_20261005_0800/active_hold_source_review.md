# 新 active hold 控制器只读源码审查

结论：在本次读取的 core.py / profiles.py 中，未发现会绕过300ms保护、移动停车目标、用缓存行凑到达停留，或通过修改旧零命令停车结论制造通过的实质阻断。此为源码检查 READY，尚不代表主动停车实际通过。未写src、未运行仿真/ROS/模型/控制器，不修改旧收据；只写本文件。

## 阅读身份与稳定性

- 初读UTC 2026-10-05T00:15:14.423722Z；复读UTC 00:16:42.673989Z；最终写报告UTC 2026-10-05T00:18:14.793842+00:00。前两次读取的源码SHA和mtime相同。其后final检查观察到core修改；下方单列最终差异核对，不能把整个审查区间称为无修改。
- core.py：`b76ca6af38752a6fbd1baa6a1ef9fee7a13e06170368e43ffc229a0b27a07136`，mtime_ns `1791159454339729114`。
- profiles.py：`b50e111ab09022776ff064600b5b5ed257a2f0cc0abbeb943e45c05a5d79d202`，mtime_ns `1791159086414473471`。
- 仅做ast.parse语法读取及字节/哈希比对，没有执行Controller更新。
- v1驱动前缀21098字节与原curvature_tracking/core.py逐字节相同，SHA `82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d`。新增逻辑位于core.py第350行以后。

## 关键控制与证据检查

|项目|源码结论与依据|
|---|---|
|固定目标|core.py:360–365/384–389生成静态路径最终endpoint+heading0及hash，拒绝配置目标不一致；捕获姿态仅诊断，从不替代_target。profiles.py:19–38明确prospective fixed endpoint。|
|来源时相|native world s[0]与有效物理stamp s[0]−.005分开保存；capture elapsed/world/state与hold declared elapsed/world/state各自独立，core.py:433–436/546/483。没有把elapsed/world与物理戳混为一个字段。|
|进入capture|仅首次父控制器goal_dwell且无fault时触发，core.py:542–559；原到达门还是进度length−.15、终点.12/.14、实际低速度/朝向门。captured arrival_pose只是诊断。|
|声明hold|core.py:474–486只在fresh更新中检查固定目标XY .025/.03、yaw .035/.045、原点实际速度<.03、Euler yawrate<.06；停留起点及结束用严格递增的native有效stamp，需.6s。held tick不调用该累计逻辑。|
|300ms保护|core.py:567–576先计算旧last_feedback/last_wall的gap，然后再承认fresh；仿真或墙clock超龄fault锁存、三个积分清零并请求零。检查每个hold tick执行，并未先刷新旧时间来吞掉迟到。dt允许约10ns数值容差，但未放宽实际300ms协议。|
|失去目标|core.py:461–464偏离固定目标>.3m或>.6rad时锁parking_target_lost，命令清零；不会跟随新位置重设目标。后续eligible变false不重选更安静停车窗，hold_stamp保持首个声明。|
|积分与滤波|capture入口hold_integral清零一次，core.py:549；drive与parking积分分离，旧drive积分不被误当holding积分。父控制器已对capture帧更新滤波，因此entry_reset避免第二次滤波，core.py:467–473。之后仅fresh dt更新。|
|反积分饱和|core.py:502–511先按逐轴+平面范数限幅阻止同向积分，再按最后Teacher变化率估计阻止slew追不上的积分，最后重算raw/limited。输出上限 [.06,.04,.10]、平面范数≤.06；只输出body速度命令。|
|切换与slew|core.py:543/552/556恢复父更新前的applied估计，再对主动输出恰做一次50Hz步长slew，避免在capture同tick先向zero再向active算两次slew。Worker仍为真实执行端，估计不是执行测量。|
|停车律|位置PD .18/.20、yawPD .65/.18，XY/yaw参考限幅 .025m/s/.07rad/s；弱body速度输出上限另审。保持COM lever-arm/Euler换算并声明desired omega_xy=0假设，core.py:496–498/529。|
|profiles来源|from_v1校核父profile实际SHA、schema、原path hash；深复制所有驾驶字段，新profile明确v2 active_hold、pending freeze、actual_active_parking_verified=false，保留旧source refs。没有把旧FAILED自动提升PASS。|
|新旧门区分|profiles.py:37–38及core.py:442明确新active停车命令允许非零，不称旧全零停车协议通过；目标/窗/模型与来源仍需新事前冻结及实际验收。|

## 非阻断的实现边界与实际验收注意

1. metadata `entry_integral_reset`表示新的parking hold_integral在入口reset；原drive外/内积分保留但capture后不再参与。不可把它解释为所有滤波和积分状态全部清空。首次capture使用当前header_dt，对新holding积分求一次增量，属于事前明确的离散实现，并非实际跨整个上一interval已有holding闭环的证据。
2. core只在update被调用时检查wall gap；后台失联、IPC、唯一执行器与原生安全仍依赖原worker/native机制。源码只读审查不能证明系统在完全无callback时已经实际停止。
3. `completed_t`在第一次严格hold声明处锁存，runner必须从该首次时刻完整保存固定首5s物理窗。不能以mode=active_hold、源码自报eligible或后续安静区间代替200Hz实际XY/yaw/velocity、力矩/qd/接触与持续Teacher验收。
4. active body命令允许非零是新合同，不是放宽旧零命令试验；新试验需验证实际Teacher接收与记录after-slew，估计slew不可直接作为施加动作证据。
5. 固定原点平面及Gazebo反馈仍是真值标定，不是SLAM导航；COM参考omega_xy=0假设与姿态安全边界须原样保留，不外推坡道/真机。
6. 根runner/profile/protocol/source冻结、实际唯一写入者、完整owned进程退出和独立验收器未在本2源任务中重新验证，必须由实际新run自己的归档提供。此次未运行纯内存Controller仿真或旧模型推理，不借旧physical pass推断active_hold效果。

READY：本报告写入完成后停止writers。没有提出需要改动驾驶前缀或放宽旧门限的源码修正；建议root按完整新冻结做单个pilot实际验证，再根据第一固定停车窗及捕获阶段原始证据决定确认重复。

## 最终冻结前的变动追加核对

最终复核UTC 2026-10-05T00:19:13.510092+00:00：core SHA `b76ca6af38752a6fbd1baa6a1ef9fee7a13e06170368e43ffc229a0b27a07136`。Sibling确认只追加固定数值/source契约校验后停止写入。实际只读删除该8行新增校验块后的完整文件SHA精确恢复初读 `2e1718e86cb6e14f479b50c2b3639d0acbc4b0ffc5f72c8e572bf0e77e059fcf`，故变动范围已字节确认：core.py:415–422新增fixed_contract一致性拒绝检查（5s、XY .05/yaw .1、速度 .08/.1、TTL .3、slew [.6,.6,.8]、Teacher持续推理、非action=0、非SLAM/真机及非旧zero-command门）。它只严格拒绝被改变的契约，无控制输出/计时/积分实现变化。profiles最终仍 `e55854c6e7f103bf1c8872554cd99682a4dd59fd937bf67e70a292e1093771ef`。未执行源码逻辑，未运行仿真/模型。READY，报告写入后所有writers停止。

### profiles最终新增身体角速度限额字段

UTC 2026-10-05T00:21:12.777211+00:00只读确认profiles SHA变为 `e55854c6e7f103bf1c8872554cd99682a4dd59fd937bf67e70a292e1093771ef`：仅原parking_contract第31行追加 `maximum_native_body_wz_radps=.1`。去除这一新增字段后的完整文件SHA恢复初读b50e111a…；它只追加实际native body wz限额声明，不修改fixed目标、控制输出、PI、时间戳或旧驾驶profile。core当前SHA `b76ca6af38752a6fbd1baa6a1ef9fee7a13e06170368e43ffc229a0b27a07136`。新的body-wz物理门还应由独立actual evaluator核验；本源码不把声明当测得通过。最终READY，review唯一写入文件停止。
