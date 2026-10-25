# 整理目录 published_repro_11：全路线到达，严格停车未通过

本轮是用户授权的同机整理目录复现，不继承原 formal22 成功。实际业务实现 `e4b120d2e52d24fe441bc61ae543ac020cc966ca`，部署 `ecb62adb4e55513e6a11f23f23803838b19e09f6`。未调控制参数或改业务策略；R99 OFF。启动前保留量122,205,626,368B，加本轮10GB及另留10GB低于170GB；运行2400秒、收尾300秒边界不变。

## 实际结果

- run：`20261025T073601Z_closed_loop_R85_published_repro_11_same_formal22_fb3d`。07:36:18 UTC受控启动，prepare52项及实际加载91项通过。任务13.865 SIM接纳，389.015 SIM失败；路线高水位34.913044228m，已进入F3终点区域。
- 动态业务片段：stop211.510 → resume236.805，恢复0.791567m，clear240.890。状态`completed_pending_verification`；最终独立dynamic verifier未形成，不能标dynamic PASS。
- 首次不可重置T0=388.790 SIM，固定结束应为393.790 SIM；Controller在388.965 SIM锁定`ordinary_zero_hold_actual_motion_or_drift_exceeded`，task在389.015获知。audit未执行、完整5秒未完成。停车/fullF3/RGB终验/original46均不授PASS；没有重选窗口。
- 真实Actor零输入388.730 → native ACK388.750 →资格388.790已成立。零输入前slew命令前向0.086109m/s；零指令不等于物理已持续静止。

## 直接触发证据与边界

在线guard使用的SLAM样本stamp388.910、接收clock388.960，平面速度0.080310030m/s，超过原0.08m/s界。相对T0位置漂移0.007920926m、偏航0.000580970rad，配对IMU body wz=-0.005256642rad/s，Euler yawrate=-0.004474398rad/s；这个样本只跨越速度条件。

同期native物理遥测（审计用途，不作导航输入）在T0至failure间8样本，最大平面速度0.084244623m/s、最大|body wz|0.017503333rad/s，观测位置XY漂移0.009810370m；零命令仍在实际送入Actor，未过期。故不能简单认定SLAM误报。当前证据确定“零命令后短时物理速度与估计速度越过停车界”，还不能唯一归因于策略步态、内层PID记忆、减速交接或定位误差。本次仅打包复现，不以放宽阈值、重选T0或调参消除失败。

## 加载、归档与退出

本目录新构建native ELF f5b82d9383a3ae5680f1a57d1e33c650bd4aa8dc459e8be8a28d727b9fcc2ca8，core e0f0a80a81f94e0b2263b1bf5569b0fc942adbc97f160352bfe2f99f52a811bd，新SCAN ELF 3a44129cb18695437d2131316b656933d611297b6e3743c355582116b8cd2eeb；实际91项加载通过，未回落旧顶层构建。仍共享明确固定的SDK、规划库、actuator、模型及管理入口，不是完全自包含或新机器/Docker实测。

ENTRY_EXIT rc1、墙钟473.623126秒，无外层强停/其他训练进程信号。6项进程/私有域/锁退出检查通过。Actor19582样本CRC通过，pipeline174583包正常drain。worker自然rc0且completed=true，但原worker fault为`EOFError: Gazebo actuator disconnected`，不能把CRC有效或closure.errors=[]写为无fault。该EOF不作为本轮停车失败原因的已证结论。

收尾快照保留124,215,541,760B（十进制124.216GB）；预算加下一轮10GB及另留10GB为144,215,541,760B<170GB，仅为资源事实，不是另开一轮许可。本阶段不再自动重跑，不永久删除。

原536c9d3部署仍是独立的成功记录，未改写。整理目录完整验收未完成；若继续处理停车控制问题，需与本次“打包路径/依赖修正”明确区分。

## 后续只读核查追加

Controller首次失败388.965，task获知389.015；两者区分。ACK的previous_slew_command不是最后实际Actor输入，后者388.710为(0.09810854,0.01230069,0.00477446)。T0上层/Actor/PID命令全零，SLAM0.007266109m/s、native0.020722347m/s，随后速度升高跨界。146个后续零输入帧不等于完整5秒PASS；零速度目标也不等于零关节输出。见PARKING_TIMELINE_CLARIFICATION.zh-CN.md（公开结果目录）及保留的原始日志索引。
