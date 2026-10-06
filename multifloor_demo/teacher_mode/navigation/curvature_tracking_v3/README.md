# 主动停车 v3：跨 NumPy 数值一致性修复

V2先导在Actor初始化前拒绝配置，Gazebo未启动；原失败run `20261005_082431_truth_pid_S_tight_active_hold_weak_p1_curvature_cascade_pi_active_hold_25hz_c0b1`保留。system NumPy1.26.4与CPU Kit NumPy2.5.2的解析终点最大差1.7763568394002505e-15m。V3仍用事前冻结的target与exact canonical SHA，只把重新计算的端点一致性检查改为1e-12m/1e-12rad数值容差；不改变任务误差门或控制增益，不按实测pose重设目标。其余停车契约与v2相同，V3另冻结、实际重跑。

# 冻结 Teacher 曲线后的主动停车候选

截至源码冻结前，本版本尚未实际运行，不代表停车或导航通过。后续实际结果须查当前主README与独立运行收据。只修改停车控制；原 14 次曲线、tightS25 的 yaw=0.114522rad 固定停车失败以及所有旧源码/收据不覆盖。本目录与 `curvature_tracking/`、相机 Demo、训练任务独立。

`core.py` 前 21098 字节与冻结 v1 完全一致，SHA256 为 `82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d`。新 Controller 子类仅在旧完整路线进度/0.12m/低速度首次到达候选后接管停车；入/出圆的曲率连接、drive 路径/速度/姿态控制数学与所有增益不变。纯检查对两种曲线和 10/25/50Hz逐帧比对接管前速度命令与积分，不把合成状态检查当物理通过。

停车目标永远是事前 `StaticPath.final_xyz` 与 `heading0`，不是随实测漂移移动的锚。初次到达的真实 pose 仅保存诊断。目标 SHA 使用 `parameter_hash({'position_world_xyz': finalXYZ, 'heading0_rad': heading0, 'path_parameters_sha256': pathSHA})`，canonical JSON 为 sort_keys=True、separators=(',',':')、allow_nan=False。

首次进入 `capture` 清零独立停车 PI 积分，保留旧测量滤波并且同一帧不重复滤波。弱世界位置 P/D=0.18/0.20、朝向 P/D=0.65/0.18，位置/角度 deadband 为 3mm/0.005rad；参考平移范数≤0.025m/s、Euler yaw 参考≤0.07rad/s。再按当前姿态与 COM 偏移转换到 Teacher body COM 速度/omega_z，沿用旧内环 PI=.6/.5、.4/.3、.3/.2 和滤波 .2/.1s。参考 COM 杠杆仍假设期望 omega_xy=0，但实测原点转换使用完整 omega，Euler 转换使用实测 omega_y。

最终请求上限为 `[0.06,0.04,0.10]`，平移范数另限 0.06m/s；反积分饱和同时处理轴/范数限幅和下游 Teacher 变化率 `[0.6,0.6,0.8]/s`。积分仅在新鲜反馈更新，held 帧保持积分但 Teacher 每20ms仍推进命令 slew。50Hz CPU 单线程 Actor、200Hz关节 PD、唯一执行器与冻结模型不变，没有 action=0、冻结关节目标或机身伺服。

`capture` 必须到固定终点≤0.025m、朝向≤0.035rad、实际原点平移速度<0.03m/s、Euler yawdot<0.06rad/s，以真实 fresh feedback_time 连续0.6s才进入 `active_hold`；已开始 capture dwell 的位置/朝向保持半径为0.03m/0.045rad。仅这个首次声明时刻启动固定五秒物理验收窗，不选后续安静窗口。active_hold 持续闭环，并且 completed 后仍执行300ms仿真/墙钟来源保护；迟到的新鲜帧在重置墙钟前判过期并锁定零速度。目标距>0.3m或朝向误差>0.6rad锁定失败。

新的停车协议允许小幅非零 Teacher 速度输入，因此不能声称满足旧“真实速度命令全部为零”的门。独立新验收保留首窗 XY漂移≤0.05m、yaw漂移≤0.1rad，另要求全部 native 速度峰值≤0.08m/s、Euler yawrate≤0.1rad/s及body omega_z≤0.1rad/s。实际轨迹/朝向/安全/到达标准不放宽；active_hold 的固定终点误差与初窗漂移分别记录。旧 common/curve 总状态不回填，新 active_hold 收据单独表述。

最小预登记为原 tight S(L6,A1、0.3m/s、25Hz、相同 spawn/物理) 一个先导及三次确认重复；同一候选不边跑边改，先导失败仍保存。`profiles.minimal_tight25_campaign(original_v1_profile, expectedSHA)` 只返回四个新定义，drive 字段完全保留；root 随后冻结本目录/执行器/模型/分析器，并归档旧 reference。固定初态重复不代表随机扰动鲁棒性；未来真实 SLAM/多层和真机仍未验证。

核心 API 与旧版相同：`Controller(profile).update(native64, elapsed)` 返回 Teacher body vx/vy/wz 与 mode。root 负责新 runner/worker/protocol/独立验收，需显式处理 `capture`/`active_hold`；原 v1 CORE_SHA 身份不接受此版本。row 新增 `parking_target_*`、`parking_capture_*`、`parking_hold_declared_*` 三套固定来源；`entry_integral_reset` 仅首次 capture 帧为 true。停车 fresh 行的 `reference_xy` 表示固定终点，原曲率几何/κv门只检查 unchanged drive，不能把 parking 参考误判为弧长参考。

离线检查（不会启动 Teacher 或仿真）：

```bash
python3 -B navigation/curvature_tracking_v2/pure_tests.py
```
