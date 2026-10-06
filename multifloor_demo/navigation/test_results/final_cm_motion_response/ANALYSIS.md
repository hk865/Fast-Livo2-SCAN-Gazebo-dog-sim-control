# 新 CM 三组实际运动的被动速度响应检查

仅离线读取已完成的四转、短坡、两点动态组件；没有 ROS init、节点、物理启动、指令生成或生产代码修改。三组原组件分别 8/8、15/15、22/22 通过，不等于完整 Demo。Full17 随后的真实启动失败仍保留。

比较调用 **完全相同且未修改**的 `MeasuredMotionObserver` 与 Full15 分析函数。窗口 .4/.6/.8/1.2 秒；位置 fit RMS .02 m、旋转/世界 heading fit RMS .04 rad、位姿 gap≤.15 秒、command gap≤.10 秒、状态/运动模式/目标及轨迹交接拒绝规则均相同。持续反号需 command 与测得 body ωz 绝对值都>.02、整窗指令符号一致、相同 context 的连续合格窗口之间 gap≤.15 秒，并持续至少名义步态周期 .60 秒。`.25 swing+.35 stance` 是原比较的名义周期口径，并非本报告从实际接触推断的周期。

新 body/GT 数据来自每组 CDR 原始 Odometry，直接读取整数 Header sec/nsec；GT 只在每组原先一次初始 SE3 中评估，插值 bracket≤.15 秒、不外推，active SLAM 全配对。四转按首个真实 SLAM pose 与同刻 bounded GT 构造一次初始 SE3，未重拟合。actual CHAMP command 使用与 Full15 相同的 Adapter 日志，Twist 无 Header，因此仍明确是缓存收到的 `/clock` float 重构 ns，并非 native command Header。NAV 轨迹 context 来自稀疏实际 status；四转 context 是真实固定 heading segment，未伪称 SCAN 规划路线。

| .6 秒窗口 | 四转 | 短坡 | 动态两点 |
|---|---:|---:|---:|
| active SLAM snapshots | 659 | 452 | 966 |
| 合格 walk 窗 | 39 | 56 | 106 |
| 整体 fit 拒绝 / 模式或轨迹交接拒绝 | 383 / 118 | 325 / 54 | 376 / 208 |
| 同号 command 且 SLAM+GT 确认反 body ωz 的窗 | 10 | 9 | 17 |
| 最长连续同 context 反号 | .20 s | .20 s | .30 s |
| 持续≥.60 秒的反号 episode | 0 | 0 | 0 |
| 合格 walk 平均实际 vx（SLAM） | .0853 m/s | .0694 m/s | .0872 m/s |
| 同窗平均 actual command vx | .1180 m/s | .1193 m/s | .1169 m/s |
| 当前最新 vx 正向 cap 余量均值 | .00034 m/s | .00084 m/s | .00062 m/s |
| 当前最新同方向 yaw cap 余量均值 | .0404 rad/s | .0395 rad/s | .0488 rad/s |
| 同窗 SLAM−GT abs P95：vx / body ωz | .00361 / .00136 | .00545 / .00111 | .00439 / .00162 |

Full15 相同定义曾有一个 .6 秒窗口构成的连续 .7 秒反号 episode。新三组短测试没有同定义的跨周期证据，**不能据此宣称持续反向已消除**：这些窗口大量因真实身体非线性摆动或交接而拒绝，拒绝窗会切断 episode；四转、坡、动态与 Full15 的路线、起点、长度也不同，不能作同输入 plant gain 或 CM 因果比较。

四组窗口的合格 walk 数分别：四转 93/39/24/14，短坡 277/56/18/2，动态 281/106/47/19（.4/.6/.8/1.2）。较长窗口虽跨越名义步态周期，也明显降低有效覆盖。`.6` 窗估计自身约 .30～.34 秒滞后；.8/.1.2 更慢。该离线比较沿用理想收包年龄，不能替代实时输入 transport/freshness 证明。

原地转向的零平移命令仍不等于机身零平移：四转与动态的合格 `.6` turn 窗平均 body vx 为 −.0403、−.0482 m/s，独立 GT 同窗吻合。短坡没有合格 turn 窗，不能填零当成没有后退。行走 command 接近 .12 cap，正向余量通常不到 .001 m/s，直接加 vx P 增益不能补足约 .03～.05 m/s 的差；纯转后退也不能通过只在 walk 期加速度反馈解决。

当前生产已经用 **实测 SLAM 位置和航向**投影 SCAN 路线、纠偏和判断到达；本次只是有限窗口 body 速度/角速度诊断，未接入新的执行反馈。body ωz 与 world heading rate 分开统计，不用 no-slip 假设。未改 observer 中 `observed_motion_ready` 的旧 `driving` 枚举（真实 adapter 为 `walk`），其 `feedback_eligible` 仍固定 false；将来接入任何控制前须另定义真实状态/时间/质量合同。

建议下一步先运行同源只读实时观察器，记录 native body Header、实际 applied command、有效窗口/交接/接触相位、原安全状态与收包年龄；只有重现跨周期且方向一致、可信、仍有 yaw cap 余量的偏差，才考虑另行隔离验证原 `.12/.08` caps 内的低频有界修正。无需根据这三组短测试立即增加增益，不能以 GT 拟合控制输出。三组原通过与 Full17 原失败均不重写。

`comparison_summary.json` 包含四窗口全部原始 counts、拒绝原因、独立 GT 对照、饱和余量和每组一次 SE3；`comparison_*/snapshots_*.jsonl`、`episodes_*.json` 保存实际逐窗数据。`native_inputs_*/normalization_receipt.json` 保存原 CDR、actual command、source/runtime/result 的 SHA；比较报告另保存所有规范化输入 SHA。`final_receipt.json` 汇总输出 SHA。复现用 source ROS 环境后离线执行本目录 `compare_components.py fourturn_v2|slope|dynamic`（输出目录须不存在）及 `summarize.py`，不会创建节点。
