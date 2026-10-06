# 2026-10-05 Teacher 真值速度／路径控制标定

本轮冻结 Teacher、CPU单线程、50Hz推理和200Hz关节PD不变。实际Gazebo测试选择固定空间路径／朝向PD外环与机身COM速度PI内环，25Hz新鲜真值反馈。它是本次有限候选矩阵中最佳参数，尚非全球最优，也不能据此宣布普遍优于CHAMP。

## 控制结构与误差

路线位置为机身link原点；横向误差是固定有向路径的带符号法向投影，沿路径剩余距离用于减速，终点欧氏距离用于到达。朝向误差为wrap(路径切线方向−当前yaw)。没有按时间生成一个理论位置点。COM速度内环使用实际COM偏置和omega×偏置，把原点速度转换为COM速度；Euler yawdot与机身omega_z分别处理，坡道也保持坐标一致。

|控制项|冻结参数|
|---|---|
|横向路径PD|Kp=0.8，Kd=0.2，Ki=0|
|朝向PD|Kp=1.3，Kd=0.15，Ki=0|
|前向COM速度PI|Kp=0.6，Ki=0.5|
|侧向COM速度PI|Kp=0.4，Ki=0.3|
|机身角速度PI|Kp=0.3，Ki=0.2|
|测量低通|外环0.2s，内环0.1s|
|反馈／Teacher／关节PD|25／50／200Hz|
|机身速度命令限幅|vx±1.0、vy±0.35m/s；wz±0.8rad/s|
|命令变化率|[0.6,0.6,0.8]每秒|
|失联保护|仿真时钟与墙钟均300ms|
|到达／停车|入口0.12m、保持0.14m，低速度/低角速度连续0.6s；随后Teacher零速度指令7s|

积分在保持旧反馈、初始化、停转过渡和停车阶段冻结；限幅及执行端变化率尚未跟上的同向误差阻止积分。停车始终运行Teacher，绝非action=0或冻结关节目标。

## 预登记结构、频率与增益比较

A2结构×10/25/50Hz共9次，B单因素增益9次，C前三候选各3次，共27次全部通过原冻结判据。旧A1的边界到达抖动及runner提前清理失败全部保留；v3仅增加12/14cm到达滞回、非驱动积分冻结及Gazebo自然退出后的worker落盘等待。

0.3m/s命令下，旧位置控制和路径PD实际约0.24m/s；串级PI实际约0.296m/s。CAS10/25/50Hz速度均值仅差0.000115m/s。25Hz分数51.061，高于10Hz的38.242及50Hz的39.051，主要来自停车漂移差异；这没有证明PID反馈必须是模型频率的多倍。三次重复数组相同，反映固定初态、seed和无噪声的确定性，不是随机扰动鲁棒性。

完整原曲线、积分/变化率复算、各候选和原件哈希：[独立A2/B/C分析](../navigation/truth_tuning_design/campaign_ABC_analysis_final.md)。

## 冻结后留出场景实际验证

D/D2共35次全部通过共同运动判据及适用的真实接触几何判据。旧v2验收器对坡道/台阶的“未实现接触适用性”原始失败不改写；新增只读组合收据核对其12项共同运动检查全部通过，再要求独立几何收据全部通过，没有放宽数值门限。

|场景|独立实际次数|结果与实际速度|
|---|---:|---|
|90度折线路径10/25/50Hz|1/3/1|5/5通过；25Hz空间RMS约13mm|
|6m去程＋180度转向回程|3|3/3通过，稳定实际约0.292m/s|
|0.5／0.7／1.0m/s平直线|各3|9/9通过，实际约0.497／0.699／0.972m/s|
|完整ramp12上／下坡|各3|6/6通过，实际约0.296／0.294m/s|
|原失败fixture的完整ramp23上／下坡|各3|6/6通过，实际约0.296／0.295m/s|
|5／10cm持续登台|各3|6/6通过，实际四脚顶面支撑、继续前进与停车|

坡道要求12m全部接触分段、各脚有序入口／坡面／出口支撑、出口四脚连续支撑及实际停车。ramp23上坡路径RMS约5.9mm、下坡约5.6mm。三层连接是坡道，不是楼梯。

汇总全部重复、范围与原件：[35次运动汇总](../test_results/truth_pid_campaign_20261004/actual_motion_overview/actual_motion_overview.json)。

## 对旧坡道失败的解释

原固定速度开环在ramp23约4.8m处失稳，旧CPU Isaac同命令前缀也有横向漂移／低机身。新冻结串级控制在相同Gazebo完整坡面、模型、执行器及特权观测下，上下坡各3次完整通过。这支持速度偏差、朝向与横向漂移的闭环纠正是主要可改善因素，不能把GPU竞争当唯一原因，也不能把Teacher原始命令跟踪看作已经充分准确。

新控制器尚未在Isaac执行同一路线/控制律对照；故完整跨仿真等价仍未通过。这里使用真值路径反馈和Actor特权高度，不计真实SLAM多楼层导航。质量、摩擦、缓存时序和训练观测差异仍在原报告保留。

5cm旧“机身高度增量比例”数值失败保留，功能判定以持续登顶和真实四脚接触为准；不能据旧比例断言Teacher登不上。5/10cm本轮再各3次功能通过。

## 真实相机与资源

D折线旁观相机位于不透明墙外，原灰色影像保留。E仅将独立旁观相机移到场景内，额外32.36s运动实测通过，真实旁观285帧、机载274帧归档，两路CameraInfo D均为5个0。实际K/尺寸一致；墙遮挡和宽角透视不是漏做镜头畸变矫正。原camera_mode未修改。时序与像素证据：[实际相机补充收据](../test_results/truth_pid_campaign_20261004/observer_placement_completed_receipt_v2.json)。

各run存resources_before/after，Teacher为CPU单线程且CUDA不可见，其他训练/评估未被停止或修改。资源竞争不作为运动失败的自动解释。

## 固定来源与操作命令

模型SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。

冻结core：`1f7f74df251833cd416cbaf308f59aa80747aa407f7f35e3b85f7b01c9a026e8`；协议：`e2308b7b7585f933081cf741bd8670e950607cea495c3f4eca293e33ae7796a8`；共同验收器：`7e386d004e2d1187753609b73f6b9f419c417aa35c1a30147d28385ffcd9fb8a`；部署参数：`0e85b8f1afb2db5c52d4fb784bafe39fc002cabdb79f3b11c0f9abf78bcee527`。实际run归档每份源码、观测/动作、200Hz执行力矩/接触和配置SHA。

从`multifloor_demo/teacher_mode`执行（只在本任务未运行其他物理仿真时）：

```bash
python3 -B navigation/truth_tuning/run.py --profile test_results/truth_pid_campaign_20261004/heldout_ramp23_up_25hz_r1.json --camera
python3 -B navigation/truth_tuning_design/compose_motion_receipt.py --run runs/本次新目录
python3 -B navigation/truth_tuning_design/dashboard.py --port 8769
```

真实SLAM迁移另见[新迁移报告](SLAM_CONTROLLER_TRANSFER_REPORT_20261005.md)。随后已完成36次实测转弯包络及14次独立连续曲线测试；其行驶14/14通过、含停车13/14通过，tightS25原停车失败保留。来源及范围见[曲线报告](CURVATURE_ENVELOPE_REPORT_20261005.md)，不把上述折线停转能力当作圆圈/S通过。
