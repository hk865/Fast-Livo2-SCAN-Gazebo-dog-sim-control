# 实际 SLAM / SCAN 串级 Teacher：独立验收

本目录的 `evaluate_closed_loop.py` 只离线读取新运行的原始记录，不启动 ROS、Gazebo、Teacher 或训练，也不发送任何控制命令。输出是新的 `summary_closed_loop_cascade_independent.json`；文件已存在时拒绝覆盖，可显式使用 `--receipt-suffix` 保存新实现版本的追加收据。旧共同验收、旧曲线、旧零速度停车收据不会被替换或升级。

本轮数字在首轮物理执行前由根代理确认，并冻结在新 `navigation_scope.json.prospective_gates`。首轮实际录制发生在验收实现最后适配完成之前：验收代码身份由追加收据的 `evaluator_sha256` 给出，不冒称这个实现版本已经事前归档。后续运行可事前归档本脚本和本说明。数值门不根据首轮结果调整。

## 来源和相位

* 高层反馈只用原 `/demo/slam/body_odom`：`camera_init / demo_slam_body` 的机身原点位姿和原点速度；机身角速度来自原 `/livox/imu`，按已归档安装外参旋转。配对 IMU 时间戳必须不晚于 SLAM，差不超过 20 ms。反馈位姿不能晚于计算/实际 Teacher 读取的原生时钟。源仿真龄和墙龄上限仍为 300 ms。
* 生产者 `/clock` 与接收相位的已有 50 ms 顺序容差独立记录；不据此允许未来 SLAM 位姿参与反馈。真实物理状态是 native PreUpdate 的 `t-dt`，`dt=0.005s`；Actor仍 50 Hz。原反馈入口与原 wrapper 内记录墙钟是两个真实采样点：分别验 freshness、原数据与原 receipt 不刷新，报告相位差。
* `cascade.controller_updated` 才是新的数学反馈更新。重复头、20/50 Hz heartbeat、停车 held 行均不能充当新源或延长到达 dwell。控制律使用原点速度；COM速度由实际源 `origin + omega × base_com_offset` 构造。执行 ack只确认上一真实 Teacher 速度输入，不携带真值位置或姿态。
* SCAN保留原提交 spline系数、knots、samples。送入新core的几何只能是原samples按事前明确的连续重复点索引去重；逐点与原样本、原SHA核对。局部投影独立重建受界且单调的弧进度。原始 PointCloud2字节、字段、端序、行padding、过滤位姿和过滤XYZ均逐消息核对，并重放原native两走廊并集。共享几何代码不是完全独立的算法，原数据、时间关联和PI/PD计算分别重建。

## 事前门与停车语义

| 项目 | 门限 |
|---|---|
| 平地实际路线 | 最大XY距离 ≤ .20 m，活动RMS ≤ .08 m，drive航向误差 ≤ .20 rad |
| 稳定移动速度 | 真实native COM沿已固定航段的速度 MAE ≤ max(.05 m/s, .25×参考均值)；参考来自原新cascade，不把Teacher请求当目标速度 |
| 3D区域 | 原请求控制半径 .17 m，外判定半径 .22 m，高度半跨度 .10 m；原SLAM distinct header连续 .6s、间隙 ≤ .2s、每目标原激活后90s；保护不能累计到达 |
| 200Hz安全 | roll/pitch绝对值 ≤ .65 rad、全碰撞支撑离地 ≥ .18 m、机身接触/故障0、command-feed力矩 ≤ 23.50001 Nm、joint速度 ≤ 30.001 rad/s、全足无支持最长 ≤ .3s |
| 主动捕获 | 固定请求终点与冻结航段yaw；XY/yaw入窗 .025m/.035rad，保持迟滞 .03m/.045rad；真实源原点速度 < .03m/s、Euler yawdot < .06rad/s，distinct source连续 .6s |
| 新主动保持 | 首次实际Teacher消费原 `active_hold` 声明对应的缓存物理样本开始，固定5s；native与原SLAM漂移各 ≤ .05m/.10rad；原点速度峰 ≤ .08m/s、Euler yawdot与bodyωz峰各 ≤ .10rad/s |
| 控制执行 | 原50Hz Actor/200Hz native/CPU单线程，唯一TeacherActuator，不重设机身、不冻结关节、不用action=0停车；原两层速度slew保持 .6/.6/.8每秒，原source双300ms TTL |

主动停车允许有受限的非零小速度修正，**与旧全零速度停车是不同契约**。捕获阶段 P=.8/D=.2、XY参考范数 ≤ .05，请求轴限 [.15,.07,.10]；保持 P=.18/D=.2、XY参考范数 ≤ .025、请求轴限 [.06,.04,.10]。yaw P=.65/D=.18、参考角速度限 .07。两阶段的COM PI与实际执行ack防积分饱和独立重建，capture入口仅一次清零积分、capture→hold不清零，声明帧按保持参数计算。

首窗口不得延后到静止或安静片段。源声明、首次真实文件消费、worker后续观察成功status的时刻分别记录；它们并非同一个采样点。缺原头、原XYZ、实际ack、完整首5秒、全部退出或writer drain，则未验证；真实越界则失败，候选score始终null。

运行期状态只取最后实际Actor读取之前的原status发布；后续shutdown状态作为完整诊断保留，不能替代真实运行结果。运行期任何failed仍失败，退出/故障/原native数据另独立核验，不仅靠一条succeeded状态。

## 保护、动态和多层边界

stale/clock/graph/tilt、实际云障碍和无效/耗尽检查路径优先于drive/capture/active_hold，必须请求高层速度0并继续Teacher执行。终点保持也必须检查新的实际云与原native走廊，不能向障碍持续修正。区域和路径不能在保护状态前进。

本脚本的本轮整体status只针对该有界平地 SLAM/SCAN 闭环。旧SLAM V5固定路线6/6、旧V45a动态停车/恢复以及真值主动保持，均不能相加授予本新控制器动态/坡道通过。

* 新动态专项仍需实际阻挡→零命令停车→实际guard连续clear1s、gap≤.3s、original pose/cloud鲜活→新checked SCAN→真实恢复运动的独立记录。此脚本只验当前实际guard与保护，专项动态结果保持未验证。若动态安全停车也要非零主动保持，需要另一事前风险与停车契约，不直接套用终点主动保持。
* 非平地整体结果强制未验证，直至新统一追加几何收据证明原200Hz脚球/真实contact XYZ+normal、每足入口/坡面/出口连续支撑及完整12m分段覆盖。短前缀或只达到目标不能替代完整坡道。三层靠坡道连接，不称真实楼梯。
* 当前body-relative一次锚定路线没有注册原场景中心线。离线固定一次SE3只用于评估，绝不反馈命令/目标。可另建立由实际SLAM/IMU一次heading注册加已知地图初始放置prior的仿真已知地图合同，但不能称纯点云自动全球注册；原stand局部扫描未验证全坡道地图。
* Actor247维仍为232维特权输入与15维已知命令/上一动作。外层导航ground_truth=false，不代表Actor已转成纯SLAM/IMU/点云输入。全多层、真实曲线SLAM新链、泛化Sim2Sim、真机均保持未验证。

## 操作

```bash
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/acceptance_audit/evaluate_closed_loop.py --self-test
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/acceptance_audit/evaluate_closed_loop.py --run /absolute/path/to/new_run
# 显式新追加版本，绝不覆盖原收据：
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/acceptance_audit/evaluate_closed_loop.py --run /absolute/path/to/new_run --receipt-suffix revision2
```

本脚本不导入ROS，不运行任何模型；只写指定新run的一个追加JSON。`--no-write`只读打印。归档模块读取关闭pyc写入，未写旧sources、旧summary或全局验收。
