# V20 空间参考与走廊观测候选

独立于 V19、camera_mode 和强化学习工程。冻结 Teacher 使用 CPU 单线程；实际 SLAM/IMU/点云反馈导航，Gazebo 真值只允许离线验收。Actor 的仿真特权观测尚未完成替换。SLAM 复用 V19 已构建流水线与 LIO4/VIO1，启动前重新核验实际加载的二进制。

## 控制改动

- 原路径三维最近投影和水平弧长进度继续保留。参考方向使用默认 0.2 m 有限弧长，朝向门、横向 PD 与速度前馈使用同一方向/法向；记录原横向误差及新的控制误差。
- 新路径通过实际 SLAM 位姿重新投影，保留 PI、滤波和执行命令，不伪造规划初速度。朝向锁在停转期间仍传给内环。
- 弦偏差过大或弦抵消回退原线段，不能据平滑方向宣称切角安全。实际运动仍通过原点云守护。
- 曲率前馈、角速度/角加速度余量限速分别可配置，几何基线默认关闭。曲率无解时明确零速度请求、冻结积分并请求新规划。这里的参考变化率约束不等于已经识别 Teacher 的物理带宽。
- 原区域、dwell、90 s 目标期限、300 ms 输入时效、真实停稳、倾角/障碍保护和唯一执行器保持。不用零 action 停车。

## 走廊实现与当前权限

隔离 `scan_ws` 从真实注册 LiDAR 的 raycast 地图导出有限三态体素快照，包含 FREE/UNKNOWN/OCCUPIED、逐体素观测时效及实测表面点；不存在障碍命中不等于 FREE。只读导出不改变 SCAN 求解。

`corridor.py` 检查机器人扫掠体积、左右宽度、停止前段，以及有限实测平面支撑（点数、凸包覆盖、观测空洞、残差、坡度、高度层）。结果绑定路径/地图/帧/时间/参数。该模型不证明足接触、摩擦、动力学或未观测地形。

首轮只启用 **shadow**：独立单 worker、最多一个任务，不积压诊断。快照、输入和证书均归档。不可用或拒绝原因如实保留。当前代码明确拒绝 `retain_valid_plan=true` 或主动走廊控制，尚未用走廊放宽控制或压制 SCAN 重规划；要先用实际数据验证该边界，再启用后续监督策略。

## 操作

在项目根目录；确认本任务没有重复仿真，不停止其他任务的训练。新源码需新有限检查与哈希门，旧 V19 PASS 不能授权本候选。

```bash
python3 -B multifloor_demo/teacher_mode/navigation/corridor_tracking_v20/run.py --profile exporter_shadow_original46_prefix9 --label v20_prefix9_shadow --domain 90 --prepare-only
python3 -B multifloor_demo/teacher_mode/navigation/corridor_tracking_v20/run.py --profile exporter_shadow_original46_prefix9 --label v20_prefix9_shadow --domain 90
```

前缀是原探索任务前 9 区，保留对应原始区域定义；不含全部 46 区、返回/RGB保存/后14区和完整动态障碍任务。达到9区、停止窗口及失败原因分别验收，启动或计时结束不记通过。完整任务使用 `pipeline_staged_original46`，需明确其选定的 SCAN workspace 与控制开关。

新 run 在 teacher_mode/runs，浏览器 `http://127.0.0.1:8768/?run=<run_id>` 显示归档/实际 Gazebo 画面和 SLAM/SCAN 路线。结果见 `../../test_results/corridor_tracking_v20_20261006/`；未取得新的物理结果前，不继承 V19 或历史曲线的导航通过结论。三层通过坡道连接，不称真实楼梯通过。真机未验证。
