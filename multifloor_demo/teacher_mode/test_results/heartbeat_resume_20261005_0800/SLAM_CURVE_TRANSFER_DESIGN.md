# 解析曲线接真实 SLAM/IMU：只读接口设计

当前仅完成源码审计，未启动 ROS、Teacher、推理或仿真；不计曲线 SLAM 接入通过。旧真实 SLAM V5 的六米直线/往返通过只属于旧折线控制器。此次只新增本说明与 JSON，所有冻结 v1/v2、训练、camera_mode 和原 run 不改。

## 不能直接把旧 core 路径换成曲线 core

|位置|具体问题|新接口要求|
|---|---|---|
|truth_tuning_slam/adapter.py:125、294|只替换 route_world_xyz 或赋 controller.route；StaticPath 实际读 profile.path，Python 可悄悄接收无效新属性|一次实际 SLAM anchor 后构造全新固定解析参数/profile/path/endpoint/heading/target SHA|
|adapter.py:246|protect 访问 curve 没有的 turn_phase，恢复仍是旧 pre_turn/stop_since 语义|独立 drive/capture/active_hold 保护，停车积分和 fresh dwell 必须明确处理|
|adapter.py:319–323|按 header gap 估算缺失50Hz帧，并强制 frame 选中；异步读/拒绝时估计 slew 不等于实际 Actor 命令|只按实际新头做10Hz数学更新，反积分使用实际50Hz执行命令来源/因果确认|
|adapter.py:415|fresh 同时给 producer 与 source 50ms future tolerance，而最终验收严格要求源不得超前|消费者分别审 producer clock 与 pose/control/gyro 的严格过去因果|
|run.py:99|仅准备直线/往返；固定 spawn 和整场景不能自动适用完整圆/S|新的局部解析路线与安全平地场景；不能套用原旧地图注册标签|
|evaluate_slam_transfer.py:209、582、603|到达只认旧 goal_dwell/parking；距离是采样折线；停车要求真实 Actor cmd=0|新解析有序进度、capture/active_hold 来源驻留、固定首停车窗与单独主动保持验收|

原源码绝对路径、行号和逐文件 SHA 完整保存于旁边 JSON。新迁移应放独立目录，不修改已通过的旧 SLAM 六轮版本。

## 一次真实 SLAM anchor 如何变换

只使用 `/demo/slam/body_odom` 的 `camera_init → demo_slam_body` 原数据。两个不同且新鲜、速度有效的 SLAM 头、因果 IMU 配对和 publisher 图健康后，选择第一个实际使用的头作 anchor，原 stamp/pose/quaternion/接收墙钟保留。用 anchor 的水平 yaw 定义 R_A，位置为 p_A。

事前明确路线属于初始机身水平局部坐标。固定局部解析路径 p_local(s)，变换 `p_map(s)=p_A+R_A·(p_local(s)−p_local(0))`，heading 加 anchor yaw。参数中的 origin 改为 p_A，heading0 改为 anchor yaw+local heading0，半径、S长度/幅度、曲率符号、entry/exit 不变。减去局部起点避免把旧0.32m模拟高度再次加到实测身体高度。

停车终点是同一个变换后的事前 endpoint，heading 是 anchor yaw+local heading0；初次到达 pose 仍只作诊断，不能重设目标。构造 anchored profile 时一次保存父 profile/path SHA、原 SLAM anchor、变换矩阵、派生 path/target/effective-profile SHA，之后所有字段固定。原 frozen profile 文件不改，新增 anchored artifact 的只创建/哈希契约明确。重启/恢复不重新 anchor，不把漂移参考移动到当前机器人处。

这只验证初始机身相对的 SLAM 曲线；不是把路径注册到旧 SLAM5 地图，更不是多层导航。StaticPath 是水平平地，不支持任意带倾斜的 SE3 路线或多层曲面；后者需另立几何与区域/楼层契约。平面测试场景还须有实际 SLAM 可用的静态特征；原60×60空平面相机只有地面/天空，不能因 Gazebo 曲线通过而推断视觉/LiDAR定位可观测。

## 10Hz 真测量与50Hz执行分开

现有 SLAM odom 的线速度由相邻 body pose 有限差分再转入当前 child body frame（slam/odom_adapter.py:79–84，geometry.py:25–38），是区间均值，不是瞬时物理 COM 速度。保留两个原头和dt便于诊断噪声/滞后。IMU gyro 用已归档 R_body_imu 转 body，必须过去配对 gyro_stamp≤pose_stamp、gap≤20ms；没有有效 IMU orientation 时姿态来自原 SLAM body quaternion。

高层 ABI 仅使用：s0=SLAM原stamp秒+.005，抵消继承 core 的5ms约定；反馈时间仍是原SLAM stamp。s1:4为实测 body-origin位置，s4:8为其四元数 wxyz，s11:14为实际body gyro；s8:11=原SLAM origin_body velocity+omega×归档COM偏移。实际 Euler yawdot=(sin roll·omega_y+cos roll·omega_z)/cos pitch。其余零占位只用于高层控制器，不能把这个 synthetic64 喂给247维 Actor；Actor仍是原生特权反馈，未与旧30维观测替换联测。

新的 source profile 明确实际数学频率10Hz，父25Hz标定仅留作来源。只有不同且实际使用的原SLAM头更新 P/D/PI/filter/capture dwell，重复 heartbeat 不积分。Teacher 每20ms仿真时间仍读有来源的 held velocity 并执行原slew；关节PD仍5ms。现有 `.02s STEADY` 定时器是墙钟 heartbeat，当 real_time_factor≠1时不等于50simHz。

实际 Actor command/read/rejection ledger 或受审核的因果 ack 必须反馈给 slew antiwindup；按两帧SLAM间隔推测执行了几次Teacher帧不能证明实际执行。ack仅是控制器已知命令，不是Gazebo真值位姿。模型、关节映射、CPU单线程、唯一执行器维持原契约。

## 双TTL、停车与独立验收

producer、原pose/gyro及最后实际controller input均保留仿真/墙钟300ms期限；消费者对pose/control≤native read clock与gyro≤pose严格过去检查，producer的±50ms时钟同步容差单独标明。clock停顿/回退、源失联、图冲突与未完成记录均保护停车。捕获驻留跨缺头>0.2s须清除；原frame/integral字段硬改不能代替清楚的暂停/恢复接口。

主动保持保留固定目标和最初declared hold窗口，不能故障后选择后续安静窗。新的源到达由SLAM/IMU原头重算完整路径和capture .025/.03m、yaw .035/.045rad、低速、0.6s新鲜驻留；不能沿用只认goal_dwell/parking的旧验证器。native真值只在源链先通过后用于一次因果anchor的离线测量，永不用于生成命令/anchor/区域到达。解析有序圆圈/S、连续速度、朝向和安全门维持；停车对实际SLAM来源和200Hz原生状态分别记录首5s XY≤.05m/yaw≤.1rad，保留新active hold速度峰值门，不要求真实Teacher命令全零。旧全零门/旧失败收据不可升级。

先完成主动停车候选实测，再做新binding的离线变换/帧/TTL/恢复/执行ack负例，最后在真实10Hz SLAM下从短平地和gentle S先导/重复推进。未通过的来源、运动或停车如实保留；SCAN、多层、动态障碍和新控制律Isaac对照另立验证。

## 本轮跨解释器几何阻断的独立复核

同一冻结v2 core、同一 tight S path SHA 下，仅计算 StaticPath：system NumPy1.26.4 endpoint x=9.11737464413959、y=−1.7493495212337745e−16；CPU Kit NumPy2.5.2 x=9.117374644139588、y=+1.249000902703301e−16。最大绝对差1.7764e−15m，路径参数 SHA 相同，但派生 target canonical SHA 不同。未加载Actor/torch或启动仿真。

因此 v3 使用事前冻结profile target的精确字节/哈希作为身份，并独立数值比较解析endpoint≤1e−10m、exact path SHA、wrapped heading≤1e−12rad合理且必要。这不是放宽物理运动门，也不能以测量到达位置替换终点。原v2初始化 FAILED /无physics保留，不能回填成运动通过。两解释器输出、版本与 SHA见 JSON。
