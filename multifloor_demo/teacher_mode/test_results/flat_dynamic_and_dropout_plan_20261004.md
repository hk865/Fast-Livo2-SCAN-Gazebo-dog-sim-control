# 平地动态障碍与实际命令源中断：待实施的独立验证

此文件是新实验设计，不是通过收据，没有修改任何运行中world、导航源码或scope。前提是本次V4及两次同设置重复均完整独立finite route通过；保留所有失败/expired事件。全局原42协议、Sim2Sim、完整坡道和46区域不升级。

## 真实动态障碍

现Teacher world名 `teacher_demo`，已带 `UserCommands`、`SceneBroadcaster`、Contact。原资产存在可碰撞红色 `moving_obstacle`（static box 0.5×0.5×1.2m，原pose[1,4,0.6]）。旧 `simulation/obstacle_controller.py` 调用 `/world/multifloor_demo/set_pose`；camera_mode独立版本调用 `/world/camera_demo/set_pose`，都只发 `entity.name=moving_obstacle, type=MODEL` 的 `ros_gz_interfaces/srv/SetEntityPose`，不设置robot。两个原控制器依赖旧mission触发和错误场景位置，**不能直接启动在Teacher平地一米路线**。

最小新入口：新run预先把同一原box的初始pose改为[7.15,-1.6,0.6]（仅新world，改后保存world SHA与资产收据）；Robot仍[6,-0.7,0.4,0]，原一米SLAM相对目标/围栏/到达与90sim单目标期限不变。按已知场景坐标预定box位置，不使用实时真值机器人位姿校准目标。Actor静态ray解析原本明确排除moving_obstacle；不要把动态box临时写进特权高度网格。实际raw LiDAR、RGB、SLAM注册云仍会看到真实碰撞box。

独立simulation-owned mover只准调用 `/world/teacher_demo/set_pose`，只准实体moving_obstacle。唯一ROS桥的接口已由旧实际Demo代码验证可用：

```bash
# 以下只在新run中由拥有该run的编排器启动；本方案未执行。
ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 GZ_PARTITION="teacher_$RUN_ID" \
  ros2 run ros_gz_bridge parameter_bridge \
  '/world/teacher_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose'
```

触发从**实际SLAM相对已冻结anchor**与实际NAV状态/accepted非零指令判断：waypoint0、running，along≥0.10m、实际SLAM平面速度≥0.03m/s，且实际非零forward至少1sim秒。无trigger则保留未验证，不按墙秒强行生成成功事件。独立mover用实际`/clock`控制10Hz位姿请求，以0.25m/s从y−1.6进入y−0.7，确认实际模型到位后保持10sim秒，再沿原路撤回−1.6。x=7.15,z=.6保持；仅一次可重复冻结程序，不追随机器人、不发cmd_vel、stop、safety、fakecloud或SLAM位姿。初始box侧面距路线0.65m，进入时首先进入0.42m保守LiDAR走廊；前面x6.9在机器人推进初段的0.95m障碍范围内，需实际记录物理余量，不能把这一估算当无碰撞保证。

服务success仅是命令回执。必须另录Gazebo实际moving_obstacle pose与RGB；优先检查本世界SceneBroadcaster实际pose topic是否包含静态移动model。若缺少，可只在新world中为box加原部署已有PosePublisher同类独立诊断（模型pose，不能桥为导航odom）。未采到实际pose不能把service请求位置假冒实际位置。每次请求/响应、实际pose/采集戳、sourcehash都进入单独 `obstacle_motion_history.jsonl`，采集方不得把真值位置提供给NAV。

准入需新增明确`finite_flat_dynamic_stop_resume`独立profile/scope条款，引用所有完整V4 repeat通过收据，冻结新world/mover/bridge/observer/协议SHA；现profile含`dynamic_obstacle_success_claim` excluded，不能静默套旧scope发布动态通过。scope只许可试验，不写全局passed。Root在当前重复结束后才能改相关接口、冻结并执行；建议新run240sim秒，goal90不放宽。

验收必须同时具备：真实box进入/保持/撤离；至少一个真实LiDAR障碍stop edge，其异步native obstacle event保存精确注册XYZ及同刻SLAM姿态；独立重算原guard结果；命令在检测后一个20Hz控制周期内归零（consumer另记录50Hz实际读入）；Teacher继续执行而非action0/关节冻结；零命令下实际原生COM平面速≤.03m/s、|yawrate|≤.05rad/s持续至少1sim秒且停车窗口xy漂移≤.03m、yaw≤.10rad。保持至少5sim秒；原生200Hz没有body/box接触、跌倒、fault、越界。撤离后的真实云走廊清空连续1秒+实测停车+**新的真实SCAN payload关联**后才恢复，至少再实测前进.20m并完成两个SLAM区域。只能称停车等待/重新规划恢复，不称横向绕过动态障碍。缺失或绕行导致未触发停车就如实未验证该功能。

## 实际ROS命令发布器中断≥10sim秒

单独新flat run，无移动box。只暂停本run的 `teacher_mode/navigation/controller.py --run RUN` 子PID（节点demo_navigation、唯一 `/demo/cmd_vel` publisher），不暂停Gazebo、SLAM、ROS桥、Teacher、训练，也不写假stop flag。PID需以运行器owned navigation_stack祖先、cmdline精确run路径、/proc启动时间共同确认；记录只向该PID发SIGSTOP，10**实际sim秒**后SIGCONT，finally必恢复并交还父编排器处理。不要对process group发SIGSTOP，避免同时冻结传感器。未发现准确owned PID则拒绝执行。

触发同样要求真实SLAM沿程≥.10m/速度≥.03m/s及accepted非零forward至少1sim秒。只读observer使用`/clock`累计暂停时长，异常时钟/observer退出需finally SIGCONT。此时bridge持续输出健康拒绝原因command age≥300ms，worker实际consumer应读到expired并继续Teacher停车。不能从仅raw ROS指令归零或file过期推断物理已停。

验收：独立日志证明真实publisher进程停止≥10sim秒且cmd消息停止（不是fakeflag）；最后有效运动command后300ms + 一个50Hz策略周期内actor requested cmd变0；保留实际source/readwall/sim age。≥1秒实测停车窗口+≥5秒保持，native200Hz安全/姿态/关节/接触无fault；恢复后需真实新pose/clock/cloud/IMU与唯一源健康，旧滞留cmd不获准；重新实际forward≥.20m并最终两个SLAM区域到达。基于原始pose/header与mono wall审查恢复，任何源超时/错误不偷偷truth补全。该收据只解锁此严格TTL物理停车/恢复子项，旧短expired39事件仍unverified，不倒推旧记录通过。

下一步root需在当前repeat自然结束后实现只作用新run的mover/producer-pause编排、独立准入条款与新证据采集，然后离线核scope，再实际执行；此方案中的新增入口尚未实现或运行。
