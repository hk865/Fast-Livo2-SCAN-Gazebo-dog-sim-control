# CHAMP dd50 力矩记录时序审计与 V4 修复边界

dd50 的 37,604 个 PostUpdate 物理帧中，tau 全为零；其中 37,599 帧仅证明 JointForceCmd 组件存在。力矩测量与实际 23.5 Nm 限幅均为 **UNVERIFIED**，旧路线 fence 失败不变。

直接原因已由版本匹配的源代码及已安装 ros2_control ELF 调用核实：控制插件在 PreUpdate 写命令；Physics UpdatePhysics 将其送入 SetForce，步进之后 UpdateSim 清空命令缓冲；原只读 observer 最后在 PostUpdate 读取清零值。准确原始源码及行号见 `dd50_force_timing_audit.json` / `source_evidence/`。网页阅读器压缩空行后的行号不同，报告使用下载原文件行号。

V4 仅修复 CHAMP 独立记录与失败停车保护。Observer XML `gz:system_priority=1` 及 ConfigurePriority 返回1；原 control 插件默认0。GZ8 优先级小者先执行，所有 PreUpdate 结束后才进入 Physics Update。缓存采用 UpdateInfo.iterations、simTime 与 dt 纳秒严格同一步配对。PostUpdate q/qd/姿态/接触相位仍为0偏移。tau 表示 command_feed_to_physics，不冒称测得 motor torque；缺采样使用 null/available0，原 PostUpdate 缓冲另存诊断字段。

声明 23.5Nm / 30rad/s 配置与原 CM enforce 开关有证据，但硬件 write 本身没有新增 clamp，也没有 Teacher 的 DC 速度力矩曲线。V4 原始值不截断；每200Hz物理帧在 .1s 初始化窗后锁存超限，50Hz reader 收到后零速度停车并失败退出，3s后缺有效同一步力矩证据也失败。底层 gait/IK/PD、默认姿态、声明限幅、场景、相机、Teacher 与导航控制均未改。下一轮实际采样可能证明超限并保留失败，离线通过不授实际力矩 pass。

V3源/DSO及旧报告完整拷贝至 `legacy_v3/`；dd50当时已有874文件的SHA封存且复核未改。新编译缓存与读源安全测试、SDF解析、ELF接口与只读API、场景几何完全一致、保护资产等50项离线检查通过。没有启动ROS/仿真/训练或发送信号。
