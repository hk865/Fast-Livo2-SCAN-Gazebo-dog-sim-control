# Tight S25 新主动捕获与保持：原件诊断对照

登记完整：本次4轮；原验收1 PASSED + 0 FAILED + 3 UNVERIFIED。
原V1零命令停车FAILED、V3弱捕获FAILED全部保留。新合同允许有界非零Teacher命令：不能追认旧零命令停车，也不证明SLAM、Sim2Sim或全局最优。

|原件|原收据|捕获→首次hold s|正式5s XY/yaw|正式峰值 planar/Euler|缺源归档|
|---|---|---:|---|---|---|
|V1 zero 0b78|failed|0.6199999999999974|0.004315m / 0.114522rad|0.009330m/s / 0.028497rad/s|README.md, capture.py|
|V3 weak 7f55|failed|None|未声明hold，未验证|未验证|无|
|V4 pilot 1 4788|passed|14.119999999999997|0.004448m / 0.002409rad|0.010277m/s / 0.000795rad/s|无|
|V4 confirmation 1 c842|unverified|14.119999999999997|0.004448m / 0.002409rad|0.010277m/s / 0.000795rad/s|capture.py|
|V4 confirmation 2 6f9d|unverified|14.119999999999997|0.004448m / 0.002409rad|0.010277m/s / 0.000795rad/s|capture.py|
|V4 confirmation 3 3765|unverified|14.119999999999997|0.004448m / 0.002409rad|0.010277m/s / 0.000795rad/s|capture.py|

## 停车语义与相位

V4捕获改为位置P=.8、D=.2、世界XY参考模长上限.05m/s、body命令上限[.15,.07,.1]。保持仍用弱P=.18、D=.2与原[.06,.04,.1]上限；25mm入、30mm保持、fresh .6s驻留、300ms保护和正式5s漂移/峰值标准保持。模型、PD、动作语义未改变。
目标是事前固定曲线终点，不是到达时的测量姿态。capture是运动阶段，只有首次真实active_hold声明才开启正式窗。capture+[.6,5.6]仅诊断，完整数值见aggregate；不得选更晚安静窗。
首个正式native state关联时间不晚于state的缓存capture行，保留5ms相位；新active_hold命令计算在约5ms后，不回绑定到更早状态。
新V4缺可选capture.py归档的三条原件保持UNVERIFIED。当前源码或其他run的副本不能替代运行时缺失归档；图中即使物理曲线重合也不能升级验收。

## 限定结论

旧弱捕获末段vx请求约.06而原点实际vx近零、目标残差约66mm；新捕获包络使实际接近固定目标并声明hold。该配置对照支持当前捕获阶段的改进，未区分位置增益和速度包络各自的因果贡献，也不证明通用Teacher死区或GPU原因。
既有common中仅适用原零速度停车的UNVERIFIED原样保留；独立主动保持收据是不同明示合同。重复是相同冻结场景的有限证据，不能扩展到任意S曲率、SLAM闭环或实体机器人。

## 实际科研图

![固定目标与捕获延迟](01_endpoint_capture_lag_actual.png)

![请求、Teacher输入和实际响应](02_requested_teacher_actual_response.png)

![正式窗口和峰值](03_formal_window_limits_and_delay.png)

使用全部原生200Hz与Teacher 50Hz样本，无插值、滤波、合成或模型运行。旧线条到记录终点即结束。原始输入和归档源SHA、现有独立NPZ交叉核对、逐门原状态均在aggregate.json。
