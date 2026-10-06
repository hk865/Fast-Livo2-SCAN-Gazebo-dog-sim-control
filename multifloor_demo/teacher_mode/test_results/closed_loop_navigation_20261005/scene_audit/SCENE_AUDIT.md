# Teacher真实SLAM/SCAN多层闭环场景审计

本次只读审计支持立即建立独立的闭环坡道模式。历史开环失败应保留，用来诊断，不能成为用户新授权闭环任务的前置通过条件。现有场景包含两条12m、宽2m、抬升1.2m的坡道；它们不是楼梯。新控制器还没有完成这一真实SLAM/SCAN多层路线，本文不赋予新的通过结论。

审计于2026-10-05北京时间12:35:44取得当前资源快照。没有启动ROS/Gazebo/Actor，没有给其他进程发信号，没有修改原源码、模型、相机Demo、训练或历史run。原始输入在审计前后SHA256一致，见本目录scene_audit.json和input_hashes.json。

## 已有实际证据与复用边界

- 实际SLAM/IMU固定平地6m单程及6m+6m往返各3轮，6份summary_slam_transfer_independent.json均为passed；反馈来自约10Hz的新SLAM header，Teacher依然50Hz，native PD为200Hz。这证明冻结控制器可以使用真实SLAM估计完成限定平地路线，尚未包含SCAN规划或原场景中心线注册。
- 真实SLAM/SCAN平地有限路线、TTL停车恢复和V4.5a动态箱完整运行已有各自通过收据。它们的独立范围不能扩成多层通过。
- 旧相机Demo具备场景轴向注册和14点楼层路线先验；其载体、初始高度、历史数值变换和较宽到达窗不能直接当Teacher证据。
- 真值反馈控制器已完成完整坡道运动，与真实SLAM/SCAN多层闭环是不同证据。Actor当前仍使用232维native特权状态/局部高度输入，另15维命令和上一动作；高层导航接SLAM并不自动实现Actor传感器替换。

实际平地证据例：[6m第3轮](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_014633_SLAM_fixed_route_v3_flat6m_mapped10_readtime_v5_r3_8bf5/summary_slam_transfer_independent.json)、[往返第3轮](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_014940_SLAM_fixed_route_v3_roundtrip6m_mapped10_readtime_v5_r3_2e3f/summary_slam_transfer_independent.json)。完整6份路径和SHA保存在scene_audit.json。

## 原场景与路线坐标

来源为原始[three_floors.sdf](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/simulation/generated/three_floors.sdf)。下表是离线地图先验，不是实时导航定位输入。

|区域|世界坐标下已知范围/中心线|踏面高度|
|---|---|---|
|floor_1|x=-4..19，y=-4..11|0m|
|ramp_12|x=2..14，中心y=2，宽2m|0→1.2m，沿+x上行|
|floor_2|x=14..18，y=0..9|1.2m|
|ramp_23|x=14..2，中心y=7，宽2m|1.2→2.4m，沿−x上行|
|floor_3|x=-3..2，y=0..9|2.4m|

坡角约5.7106°。原相机场景路线为(0,2,0)→(2,2,0)→(5,2,.3)→(8,2,.6)→(11,2,.9)→(14,2,1.2)→(16,2,1.2)→(16,7,1.2)→(14,7,1.2)→(11,7,1.5)→(8,7,1.8)→(5,7,2.1)→(2,7,2.4)→(0,7,2.4)。来源[scenario.json](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/camera_mode/simulation/scenario.json)。

这些z值是相对楼层增量，Teacher公开导航目标应保持SLAM基座原点高度语义。不能复制相机载体.75m高度，也不能把地面z直接当基座目标。原SCAN REFERENCE_PATH会加grid_map.body_height=.4，而共享controller的request_plan先减.4；新适配必须保持这对抵消或明确统一改变接口，避免坡道路径垂直偏移。

## 可执行的真实传感器注册方案

现有[heading_alignment.py](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/heading_alignment.py)使用真实IMU与SLAM姿态，不订阅native真值，也不修改SLAM位姿：

R_world_body = R_world_IMU-reference · R_IMU-reference_IMU · R_body_IMUᵀ；
R_camera_init_world = R_camera_init_body · R_world_bodyᵀ。

初始化静止阶段取得10个不同原始SLAM header、时间跨度至少.8s、与真实IMU姿态差不超过20ms；方向离散不超过1°、重力轴残差不超过3°后，冻结一次yaw变换。每对原始stamp、四元数、外参及参考系假设都必须保存，后续不得随机器人yaw更新这个变换。

此仿真IMU具有明确的SDF CUSTOM/world参考：[Teacher实际sensor_contract](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_014633_SLAM_fixed_route_v3_flat6m_mapped10_readtime_v5_r3_8bf5/sensor_contract.json)为CUSTOM、parent_frame=world、world_rpy=[0,0,0]、body_IMU旋转恒等。对应实际adapter_IMU_inputs.jsonl共7068条，7068条orientation_available=true，不是拿缺失姿态的单位四元数填补。

旧相机实际记录使用10对16.7..17.6s数据，轴向变换yaw=.000190044rad，最大配对差约1ns、方向离散.000078451rad，重力残差.001353097rad，见[mission.json](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/camera_mode/runs/20261002_190238_5466b5/mission.json)。这些旧数值禁止搬到新Teacher run；可复用的是算法和证据格式。CUSTOM/world方向是本次仿真传感器契约，并不能证明真机IMU天然拥有全球航向。

平移不能仅靠IMU/SLAM确定场景绝对原点。最小闭环方案是显式声明已知初始场景放置为地图先验，以本轮稳定真实SLAM初始基座原点为camera_init原点，将场景路径减去声明的场景anchor，再按新冻结yaw旋转。初始化沉降/漂移造成的先验误差要保留和量测，不能用实时native pose补平移。更完整方案是用真实注册点云地标/边沿拟合求地图到SLAM变换，独立登记不确定度和失败条件。

因此本轮可以采用“已知地图先验＋真实传感器轴向注册＋SLAM运行反馈”的明确scope。不能称其为纯点云自动全局定位；也不需要把完整坡道自动建图先验的尚未通过作为开展闭环控制的障碍。

## 点云支持层的实际状态

stand2071的[ramp registration原收据](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/ramp_registration_actual_20261004/stand2071/registration/registration.json)仅使用本次实际registered cloud/IMU/SLAM，ground_truth_used=false。它累计55,557点，10个候选支持平面，ramp_candidates为空；status=unverified、full_route_eligible=false、full_route=null，原因是current_support_layer_not_observed和no_supported_ramp_plane_observed。没有发导航请求或控制机器人。

当前geometry.py按实际SLAM基座下方.15..65m、周围.45m内实际点选择支持面，并拒绝高度近似的歧义层；完整候选还要求坡两侧和上下平台接缝实测。它是几何支持代理，不是脚接触/载荷量测。这份结果不能被升级为完整坡道已注册。地图点云的XYZ范围跨多层也不等于连通关系已确认。

最小多层闭环应将“当前已注册的活动路线段/支持层先验”与“实际SLAM高度及点云支持面的相符性”分别记录。先验route_surface_id可以定义floor_1→ramp_12→floor_2→ramp_23→floor_3的状态，实际SLAM原始位姿和点云负责确认转换。不能使用实时Gazebo z来切换层，不能每次取最高水平板，也不能把路由标签当作实际脚支撑通过。

Teacher局部高度扫描现有lower12/upper23两个显式固定特权provider：[lower12](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/tests/terrain_targets/lower12.json)、[upper23](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/tests/terrain_targets/upper23.json)。lower12仅floor_1/ramp_12/floor_2，避免floor_3顶板被当脚下地形；upper23仅floor_2/ramp_23/floor_3。这只改变Actor观测目标集合，不能删物理碰撞/SCAN障碍。连续多层若需要provider转换，依据必须来自SLAM已注册路线状态，逐条保存provider ID/转换源stamp；高度值本身仍是仿真特权源，未验证真实局部点云替代。

## SLAM、SCAN与命令执行接口

可复用的[SLAM-only launch](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/slam.launch.py)只启动6个自有子进程：传感器保真relay、self-echo filter、相机参数节点、隔离fastlivo_mapping、body/lidar odom adapter、map archive。它不发布速度、不启动CHAMP。

实际反馈为/demo/slam/body_odom（frame=camera_init，child=demo_slam_body），IMU为/livox/imu，规划云为/cloud_registered_full。需要保持传感器外参、body原点到COM的ω×offset转换、整数stamp因果配对；10Hz新SLAM header不能被20/50Hz心跳包装成25Hzfresh估计。IMU传播是未来单独来源契约，当前不因开启参数就算验证。

现有[SCAN stack](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/stack.launch.py)包含上述SLAM和5个导航子进程。SCAN接收actual SLAM lidar/body pose和完整registered cloud；body twist转世界语义后进入SCAN。发布checked bspline及trajectory metadata，必须精确匹配request/goal/reference stamp，不能把旧轨迹用到新楼层目标。

现有[scoped_profile.py](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/scoped_profile.py)限flat及spawn[6,-.7,.4,0]；即使slam_only也在早返回前检查这一条件。它还保留旧30轮运动basis。正确办法是新独立多层scope和配置生成器，冻结明确的本次授权、profile/world/传感器/控制器来源；旧scope不修改、不借旧pass升级新scope，也不把旧failed gate当本次授权的禁止条件。

原[obstacle_ahead](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/navigation/control_core.py:286)依据活动路线预期基座高度，排除低于该高度.12m的支持点；原checked target与实际移动方向两个走廊取union。多层XY重叠时，如果输入整条路线所有段，最近XY段可能选择错误高度；新适配须采用正确的活动注册段/支持层，不能仅用最高/最低楼层启发式。SCAN本身是3D占据/碰撞规划，不提供完整可行走支持层证明。

唯一CPU Teacher worker/native执行器保留原200Hz PD、50Hz Actor和DCMotor限制。高层模块只拥有一个速度命令出口。300ms源/命令freshness、原始header到达窗、连续点云clear证据和保护转换保持。原始SLAM区域到达应独立于预测tracking pose；active endpoint hold必须首先服从stale/倾角/点云障碍保护，不能靠残留PI继续运动。native真值只用于离线路线误差、姿态、脚接触、clearance、q/qd/JointForceCmd验收。

## 建议root串行实际顺序

1. 新scope平地短路线先验证本次真实SLAM/SCAN、唯一命令出口、注册符号/高度、停车和退出。已有平地通过不能替代本轮来源完整性，但不必重复开环能力矩阵。
2. ramp12闭环全段：低端入口x≈1.2,y=2，经过x2→14中心线，再到平台x≈14.8。完整12m、两端实际脚接触、SLAM区域到达和停车分别验收；未到平台则停车未验证，途中安全失败如实保留。
3. floor2平台沿x≈16从y2到y7连接，在相同真实SLAM frame中转向并进入ramp23。节点/圆角需要考虑已测Teacher转弯半径和足迹膨胀，不能原地折线通过冒称连续曲线通过。
4. ramp23沿−x从x≈14.8,y=7经过14→2，至floor3 x≈1.2。随后可以在同源通过基础上登记两坡连续路线；上下行仍分别报告，不称楼梯。

新protocol应预先冻结source到达窗/时限、路线偏离（保留已适用.45m门）、RP≤.65、相对踏面clearance≥.18、200Hz力矩/qd限制和退出完整性。每段按实际SLAM命令、速度和原始source stamp验收，不能只看终点pose或模型加载成功。连续路线需要独立新provenance和sourcechain，不归并成全局Sim2Sim/真机通过。

## 当前主机与安全启动建议

当前20核CPU总忙约7.96%，1/5/15分钟负载1.52/1.96/2.08；可用内存约50.72GiB、磁盘剩余约266.58GiB。RTX5070Ti实际使用1160/16303MiB、GPU8%，compute列表仅ToDesk 376MiB。选定进程快照没有看到活动Gazebo/Teacher/SLAM/SCAN/RL进程；这只是当时观测，不能承诺其他任务保持空闲。

8768/8769/8770均仅在127.0.0.1监听，本次只读GET均200；对应PID分别563213/735456/563216。viewer不是控制进程。资源和HTTP原始回执见host_audit.json。没有停止这些viewer或任何RL相关任务。

root可串行启动一个新独立Gazebo batch：CPU Actor单线程、独有GZ partition/socket和ROS domain、64MiB显式DDS loopback配置、原硬件Ogre2图形上下文。启动前再次采样资源，不继承其他任务domain/partition，所有group清理只针对本run保存的PID身份。首次闭环不需要额外并行物理测试；图/大点云离线分析留到实际运行结束，避免抢占反馈链路。

本文仅完成接口与场景审计。没有新的闭环多层实际通过、没有自动完整坡道地图通过、没有真机部署结论。
