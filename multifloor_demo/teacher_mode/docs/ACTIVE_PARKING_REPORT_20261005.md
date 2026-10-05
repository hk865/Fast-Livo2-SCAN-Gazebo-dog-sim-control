# 2026-10-05：tight S25 固定终点主动停车实测

新停车控制在真实 Gazebo 中完成先导及三次确认，4/4 独立收据通过。行驶使用原曲线控制律；新控制只接管完成曲线后的靠近与保持。反馈为 Gazebo 真值，场景是独立 60×60 m 平面，不计入真实 SLAM/SCAN、多层导航或 Sim2Sim 通过。

本次接续已有标定与圆圈/S测试，没有重复此前36轮转弯包络、14轮曲线、35轮留出运动或6轮真实SLAM迁移。单次08点任务 `go2-teacher-08` 已由实际工具与本地配置确认 PAUSED；未改另一任务 `go2-017`。Teacher始终CPU单线程，没有重新训练、操作真机或修改camera_mode。

## 为什么修改停车

原 tightS25 的行驶通过，但零速度停车首5秒偏航漂移0.114521697 rad，超过0.1 rad门。该阶段绕过速度PI，不能由此得出“25Hz反馈太慢”。原失败不回填。

新弱控制V3实际跑满180秒，仍没有进入保持。最后终点残差约66 mm，Teacher前向输入接近0.06 m/s上限，而原点实际速度接近零。这支持当前低命令捕获包络响应不足的判断；没有证明所有情况下存在固定死区，也没有证明GPU或反馈频率是唯一原因。V4提高靠近段位置增益与速度包络，进入保持后恢复弱控制；未逐项分离这两个改变的因果贡献。

本次初始资源审计为20逻辑核、整体CPU约14%、可用内存约51 GiB、GPU显存1126/16303 MiB、利用率6%。未见匹配的RL训练/评估进程，也未向其他进程发信号。资源空闲时仍能重现V3靠近失败，不能把旧问题全部归因于训练占GPU。详见本轮[资源审计](../test_results/heartbeat_resume_20261005_0800/host_audit.json)。

## 控制与冻结判据

路线保持原解析反向转弯/侧移返回S：L=6 m、A=1 rad、前后直线各2 m，总弧长10 m，参考0.3 m/s。固定终点为 `[9.11737464413959, -1.7493495212337745e-16, 0.32]`，目标yaw=0；不是实际到达时的姿态，也不随漂移移动。

|阶段|位置P/D|世界XY参考模长上限|Teacher body命令轴上限 vx/vy/wz|
|---|---|---|---|
|靠近 capture|0.8 / 0.2|0.05 m/s|0.15 / 0.07 m/s / 0.10 rad/s|
|保持 active_hold|0.18 / 0.2|0.025 m/s|0.06 / 0.04 m/s / 0.10 rad/s|

朝向PD=0.65/0.18、yaw参考上限0.07 rad/s；内环COM速度PI分别为 vx=0.6/0.5、vy=0.4/0.3、wz=0.3/0.2。位置误差为固定终点的世界XY向量，行驶阶段另用几何路径投影、切向/横向误差与因果弧长进度，并非按预设时间追逐理论点。body COM与原点速度及Euler yawrate均显式转换。

新鲜反馈25Hz，Teacher50Hz，关节PD200Hz。积分只在新鲜反馈更新；保持旧帧不能延长驻留。停车积分只在首次capture清零，切入hold不清零；滤波状态保留。限幅、范数限制、反积分饱和和最终变化率 `[0.6,0.6,0.8]/s` 均记录与重放核对。300 ms仿真/墙钟双龄保护在completed后仍有效，唯一执行器仍为TeacherActuator。

捕获入门：固定目标XY≤25 mm、yaw≤0.035 rad、原点实际速度<0.03 m/s、Euler yawrate<0.06 rad/s；以新鲜反馈连续0.6 s确认，期间XY保持门30 mm、yaw保持门0.045 rad。采样证据不等于两反馈样本之间的连续真值证明。

正式停车从首次声明hold的原生状态时刻开始，固定5秒，包含1001个200Hz物理样本；不选后续安静窗口。XY漂移≤0.05 m、yaw漂移≤0.1 rad；另要求平移速度峰值≤0.08 m/s、body wz与Euler yawrate峰值各≤0.1 rad/s。新合同允许小幅非零Teacher速度命令，Teacher持续推理；没有action=0停车、冻结关节目标或机身伺服。原共同收据overall FAILED保留，其中旧零命令停车门UNVERIFIED；新主动停车收据单列。

## 全部实际结果与失败

|批次|实际情况|结论|
|---|---|---|
|V2启动检查|NumPy版本终点重算差约1.8e-15 m，严格哈希拒绝，未启动Gazebo|启动未验证，原日志保留|
|V3弱捕获|1次180 s真实运行，未声明hold；3次确认未运行|失败，停车窗口未验证|
|V4原批|4次真实运行：相机先导4788通过；c842/6f9d/3765缺事前冻结的可选观察器源码归档|1通过、3未验证，不补旧档|
|V4r2|只修runner始终归档capture.py；控制器、模型、物理及参数与V4完全相同；事前绑定4788，再实际跑518e/5627/6767|先导＋3新确认，共4/4通过|

本轮共8次实际物理运行，另1次无物理启动失败；不是8次通过。无相机确认明确记录观察器未执行，不能声称这些运行有RGB。V2→V3只为跨NumPy重算加入1e-12数值一致性容差，仍精确绑定事前目标字节和SHA；没有放宽物理任务门。

四次接受运行使用同一初态、名义参数，结果重合：

|指标|原件结果|
|---|---|
|每轮新独立检查|22/22通过，共88/88|
|capture→首次hold|37.285→51.405 s，耗时14.12 s|
|首次正式窗|51.405–56.405 s，1001原生样本|
|XY最大漂移|0.004448267 m（4.45 mm）|
|yaw最大漂移|0.002408736 rad（约0.138°）|
|原点平移速度峰值|0.010276815 m/s|
|body wz / Euler yawrate峰值|0.000776517 / 0.000794754 rad/s|
|行驶路径RMS / 速度MAE|0.008318478 m / 0.014471827 m/s|
|行驶朝向误差峰值|0.031546585 rad|
|保持声明附近终点XY残差|约0.02441 m，漂移与绝对终点误差分别记录|
|最大roll/pitch / 最小机身间隙|0.113963424 rad / 0.275679013 m|
|施加ForceCmd / qd最大绝对值|18.469220 Nm / 13.445767 rad/s|
|机身碰撞 / 执行器或TTL故障|0 / 0|

ForceCmd是仿真施加命令，不是实体电机测量力矩。最后约0.10 m靠近用14.12秒仍偏慢；相同初态四次通过不能称随机扰动鲁棒、最快或全局最优。原V1首窗比新hold更早，驻留/保持语义也不同，不能将两者当作同窗同合同的直接改善倍数。另保留事前指定capture+[0.6,5.6]诊断窗，不用它替代验收。

所有原200Hz轨迹、50Hz命令/动作、接触/姿态/力矩、配置/源码快照及失败保留。两套分析的六张图已人工QA，310个唯一原输入复核无变化。详见[接受组曲线与原件报告](../test_results/heartbeat_resume_20261005_0800/analysis/analysis_v4r2_confirm3/REPORT.md)、[原V4缺档组](../test_results/heartbeat_resume_20261005_0800/analysis/analysis_v4_final/REPORT.md)、[V3失败](../test_results/heartbeat_resume_20261005_0800/analysis/analysis_v3_failure/REPORT.md)。

## 来源与各层结论

高层新控制用Gazebo原生真值；Actor为232维特权状态/几何高度＋15维已知命令/上一动作。实际IMU/q/qd的30维替换有独立证据，但未与本轮联测。新高度扫描仍是静态几何187点，没有宣称局部点云替代完成。

|层级|当前结论|
|---|---|
|接口|通过，冻结Teacher、名称映射、CPU推理、唯一执行器与保护|
|运动控制|既有真值35/35运动、曲线drive14/14与新tightS25主动停车4/4分别通过；原曲线零命令停车13/14与各失败保留|
|Sim2Sim|整体未通过；新曲线/主动停车控制律未做匹配Isaac闭环对照|
|导航集成|旧有限真实SLAM/SCAN及实际SLAM固定6m路线有通过；新曲线主动停车实际SLAM/SCAN、旧地图注册与多层尚未验证|
|真机|未验证，本任务不操作真机|

新真值PID已经有完整ramp12/23上下坡通过，原开环12 m坡道失败保留；不能把后者当作新PID结果，也不能用前者宣称真实SLAM多层成功。三层是坡道连接，不是楼梯。5/10 cm持续登台各3次功能通过；旧机身高度比例失败不表示登不上。

## 哈希、展示与操作

模型SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。

V4/V4r2数学控制器SHA256：`9eca389f54277986656c0ccda3da42f01d9cbc25ed15d99b57cc9a1e51087384`；原drive前21098字节SHA256：`82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d`。

V4r2 runner SHA256：`70ced9ed76c4e333467d6c6b4c6c12faf145177b2ca3d0da4dc83c76883d379b`；事前计划SHA256：`851ae5a756dc0f7e5883c50de1f1ddd12e08d4612d6bb9f75ce60e2f39d102ee`；接受组aggregate SHA256：`9b8ec5d7f2e1639470870cf6c1c5c3f9ce2d0dac1b763173a26763a22524a1c3`。各轮23个冻结输入、配置及收据哈希详见[计划](../test_results/heartbeat_resume_20261005_0800/active_hold_plan_v4r2.json)、[登记](../test_results/heartbeat_resume_20261005_0800/active_hold_results_v4r2.jsonl)及各run source_manifest。

[新相机先导与曲线](http://127.0.0.1:8769/?run=20261005_084153_truth_pid_S_tight_active_hold_capture_p2_curvature_cascade_pi_active_hold_25hz_4788)显示独立主动停车验收及原common状态；43秒有实际两路Gazebo RGB、原始时间与CameraInfo，D均为五个0、640×480。宽视角透视不能据此称镜头畸变未矫正。另[实际SLAM往返页](http://127.0.0.1:8770/?run=20261005_014940_SLAM_fixed_route_v3_roundtrip6m_mapped10_readtime_v5_r3_2e3f)使用真实camera_init路线，保留旧控制版本标识；两页均只读。

从teacher_mode目录复现会创建新run，已有验收器拒绝覆盖收据：

```bash
python3 -B navigation/curvature_tracking_v4r2/run.py --profile test_results/heartbeat_resume_20261005_0800/profiles_v4r2/00_confirmation_tightS25_r1.json --camera
python3 -B runs/新run目录/sources/truth/active_hold_receipt.py --run runs/新run目录
python3 -B navigation/truth_tuning_design/dashboard.py --port 8769
python3 -B navigation/truth_tuning_design/slam_transfer_dashboard.py --port 8770
```

源码复审、纯检查与资源审计先于实际测试完成：控制器13项、独立收据30项在system NumPy1.26.4与CPU Kit NumPy2.5.2均通过。它们没有替代真实物理收据。当前包清单由全部writers结束后生成，旧PACKAGE_MANIFEST精确归档；历史acceptance SHA保持 `0250c047a09ada1b29ddaaa6f7251b3fbb57ad091f75f9ff0273f48ef08ba76c`。
