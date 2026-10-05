# 本地 Teacher 接手审计与分工

日期：2026-10-03。范围仅仿真，冻结 `model_1000.pt`；不训练、不操作实体、不替换已通过的 camera_mode。所有新增文件位于 `multifloor_demo/teacher_mode`。

## 审计结论与实施边界

原 Go2 启动链为 CHAMP → IK → 关节轨迹控制器 / PID，不包含 Teacher 推理。原 `simulation.launch.py` 不适合与新力矩执行器同时启动。原相机服务在 8767 / ROS域78，本模式在 8768 / ROS域79；每轮 Gazebo transport 分区独立。本轮运行器只管理自己创建的子进程组，不按程序名批量停止进程。另一个强化学习训练 PID 3603641 和相机服务 PID 3574217 保持运行。

冻结训练归档而非通用部署配置是契约来源：`params/env.yaml`、`params/agent.yaml`、归档代码和当前相同 IsaacLab 源码。`policy/contract.json` 保存文件哈希、提交、字段区间及源码定位。网络键为 `actor_state_dict/mlp.*`，有12维 Gaussian mean、无观测归一化；旧 `model_state_dict/actor.*` 导出约定不适用。本模式直接 PyTorch CPU 推理，之前 ONNX 产物只作为接口历史证据。

执行链选为 Gazebo 原生插件的200 Hz关节力矩 PD/DCMotor + 同步50 Hz Unix socket Actor。删除复制模型的全部旧插件，再仅加入 Teacher 力矩写入者。读状态、相机、IMU和关节状态发布器只旁路读取。没有通过基座外力、速度伺服或连续姿态重置辅助运动。

观测初期使用真实物理仿真状态、应用后的力矩和静态碰撞几何射线；明确标为特权输入。COM速度与关节逐名映射必须先核对；高度扫描网格187点及 +20 m 射线起点必须保留训练语义。多楼层楼板产生上方击中是必须记录的场景分布差异，不能改成脚下地面后仍称原语义匹配。停车采用速度命令平滑归零并持续执行 Teacher，不将 action=0 或冻结姿态作为正常停车。

## 模块分工与交付

| 模块 | 负责内容 | 文件 / 证据 |
|---|---|---|
| training_audit | 冻结模型与归档语义；Isaac原物理CPU匹配命令 / 地形对照 | `policy/contract.json`、`policy/observation.py`、`scripts/isaac_cpu_reference.py`、`runs/isaac_*` |
| gazebo_audit | 唯一关节写入者、真实接触、200/50Hz周期、DC限幅；异常测试及严格验收；只读传感器核查 | `simulation/teacher_actuator.cpp`、`scripts/evaluate.py`、native audit、sensor shadow |
| deployment_audit | 独立浏览器、导航边界门禁、观测替换评估、结果汇总及曲线 | `web/`、`scripts/serve.py`、`navigation/bridge.py`、`docs/OBSERVATION_REPLACEMENT.md`、aggregate / plots |
| root | 独立资产、CPU Actor、命令/停车状态、实际Gazebo闭环测试、报告和操作说明 | `simulation/prepare.py`、`policy/worker.py`、`scripts/run_test.py`、`tests/protocol.json`、README / report |

## 预登记的验证顺序

1. 模型哈希、247维观测、12动作、名称顺序、真实力矩限幅、唯一执行权和故障锁定接口检查。
2. 固定名义参数下逐命令站立、前后左右、正负转向、行走停车、切换、命令TTL、上/下10%坡、5/10 cm低台阶登台。每项3次独立进程重复，所有失败保留。
3. 与 Isaac CPU 相同命令时序、幅值、初始化和真实同几何碰撞对照；分开报告记录输入回放一致性、即时物理快照一致性及运动表现。
4. 在当前原导航起点另做场景适用性测试；所有基础运动及 Sim2Sim 门均通过后才启动 SLAM/SCAN。路线位置只用 SLAM，真值仅离线诊断。不得把坡道当楼梯，也不得把原相机 Demo 成功继承为 Teacher 导航成功。

数值判据见运动前保存的 `tests/protocol.json` 和每轮 `sources/tests/protocol.json`。5 cm台阶升高不足、原起点失稳和严格快照语义偏差均须保留，不能为解锁导航而降低判据。
