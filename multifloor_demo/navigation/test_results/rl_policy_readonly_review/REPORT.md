本地确有经过真实 Isaac Lab 评估的 Go2 策略，可以节省网络、观测契约及导出部分的工作；目前没有一份证据能证明它已经通过本 Demo 的 Gazebo 接入门。此次只读文本、JSON、YAML、XML及二进制哈希，未反序列化、加载或运行任何 policy，未启动 ROS/物理，也未写入旁边仓库。适用目录及祖先未发现 AGENTS.md；已读仓库 README 和各实验的范围说明。

| 候选 | 已有事实 | 当前判断 |
|---|---|---|
| `002/visualization_old_model49999_fixed_physics` | README 明确是无效 001 checkpoint 在修正物理下的诊断回放，不是 002 模型 | 排除作为合格控制器；不能把该 ONNX 叫修正后的 002 |
| 官方 003 / `model_36900` | 修正物理和课程；20秒100/100、30秒92/100，线/yaw RMSE .06943/.04153；所选 checkpoint 哈希吻合 | 最明确的纠正后官方 flat 基线，但长时95%门失败，所选 run 目录未见导出 |
| STAGE3-FLAT003 / `model_1998` | 固定命令30秒99/100；4秒切换79/80通过；1秒切换跟踪失败，action幅值待处理；现有导出及外部data、TorchScript实际哈希全部匹配 validation | 最直接的“现有已导出”接口原型候选，仍不具备 Gazebo 部署资格 |
| STAGE6 rough | 早期工程/容量/导出门通过不代表收敛；最新004B no Teacher selected，30秒64/100 | 不作为本 Demo 坡道控制器；privileged Teacher 也不能直接部署 |

STAGE3 导出位于 `logs/rsl_rl/rl_unitree_go2_flat_multidirection/2026-08-21_17-01-56_stage3_go2_flat_multidirection_continuous_hold4to8s_seed42/exported/`。`policy.onnx` SHA 为 `ebfb033b361e315594d63662bd6259853bdc4caf0e496791048cc32a93251a4a`，其 `policy.onnx.data` 必须一并保留，SHA 为 `c4f01fe3519214c26614bc4358d10e879c51b6a31aa3d1a332a7ee1c89b92772`。checkpoint SHA `a6cc5b7d6bb369af125b6e41eefac4bee2d30d611be53b7eec6af8f0521f1ed5`。已有 golden 最大误差 `2.384e-7`；本次只核产物身份，未重跑推理。

两个 flat actor 都是 stateless 45→12：body gyro×.2、body gravity、body速度命令、q−default_q、qdot×.05、上次 raw action；无 critic/GT/height/contact输入，actor obs normalization=false。输出为 `default_q+.25*a`，不是默认限制到±1的 action。STAGE3 顺序是 **FR/FL/RR/RL，每腿 hip/thigh/calf**；官方003是 **FL/FR/RL/RR hips，随后同序 thighs，再 calves**。不可拿通用 go2.yaml 的顺序套官方003。映射到 Demo 关节时轴符号一致，必须按名字重新排；STAGE3→Demo adapter LF/RF/LH/RH序的索引为 `[3,4,5,0,1,2,9,10,11,6,7,8]`。

训练 physics 5ms，decimation4，policy20ms/50Hz。两者关节 P/D 为25/.5；STAGE3实际为 DCMotor 全12关节23.5Nm、30rad/s，官方003为 UnitreeActuator 的速度相关扭矩包络（13.5rad/s拐点、30rad/s空载，同向20.2Nm/反向23.4Nm）。**STAGE3 deploy.yaml 却列通用 URDF 的 hip/thigh23.7、calf45.43Nm，与实际训练/评估执行器不一致。** 网络导出 golden PASS 不验证这项动力学契约。当前 Demo 的 P221/I.2/D1、4ms JTC 和 CHAMP停止名义姿态不能隐式沿用；也不能把网络 action 当力矩。

Go2 XML/URDF 与 Demo 的12个关节轴、髋间距及 .213m 链长可按名字对应。主要不匹配为后腿上关节范围、膝关节扭矩/速度限额（RL URDF45.43Nm/15.70rad/s，Demo35.55/20.06）、参考站姿（RL hips±.1、后thigh1.0，Demo hip≈0/upper.78946/lower−1.57893）、被动阻尼/armature、固定脚/转子/传感器质量惯性布局。文件质量总和分别为 RL URDF16.087kg、MuJoCo XML15.206408kg、Demo16.512kg；这些不是同一个实际训练 plant 的质量声明。训练随机质量范围也不能证明当前接触模型兼容。

现有 Go2 MuJoCo `pd_hold_1s` 的 status=passed 仅指有限值/模型映射接口；`policy_quality=not_evaluated`，高度保持条件其实 false（最低/初始=.49688）。没有完整 policy 的 MuJoCo/Gazebo闭环稳定 PASS。STAGE3移动分布最低.15m/s、纯yaw最低.2rad/s，本 Demo .12/.08低速与频繁zero/ALIGN、小纵向补偿、坡道/下坡、障碍等待均无对应现成验收。

若 Root 后续选 RL 备选，最小可复用方案是把身份明确的 STAGE3 ONNX 作为隔离 low-level backend 原型，保留 NAV/SCAN/SLAM区域及原IMU保护，在相同安全接口后取代 CHAMP参考生成；显式恢复45-D输入、20ms物理时钟、joint map、训练执行器及参考站姿。新的低层 profile 需要验证 exact-zero/ACK/归位、freshness、唯一命令拥有者、有限输出/关节限额，再用原低速、纯转、stop/restart、坡道与动态组件合同判定。当前不能称可直接接入或可稳定备用。若更重视纠正后的官方基线，应先取得003 model36900对应的可信导出与manifest；不复用旧001导出冒名。

全部来源哈希、关节/URDF逐项数据、候选接口及未过工程门记录在同目录 `result.json`。本报告未改变任何历史失败结论或选用现有政策。
