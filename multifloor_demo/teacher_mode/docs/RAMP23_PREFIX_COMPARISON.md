# 冻结 Teacher：12 m 坡道的实际运动前段对照

上行在 Gazebo 与名义 Isaac CPU 物理中均失败；下行在 Isaac 存活到30秒、Gazebo在25.48秒失败。**两个方向都没有完成12 m坡道、两端平台过渡与目的平台停车，不能标记完整坡道或整体Sim2Sim通过。** 所有数据来自实际物理运行，不是动画或仅模型加载。

| 方向 | Isaac CPU实际记录 | Gazebo实际记录 | 结论 |
|---|---|---|---|
| 上行 | 21.04 s后终止；最后pre-step记录21.02 s，世界X前进4.3924 m，侧漂0.6834 m，最低记录离地0.1744 m | 22.89 s原生机身LiDAR碰撞坡面，worker22.90 s报告；前进4.8197 m，侧漂0.7130 m，最低离地0.1547 m | 两环境上行前段均失败；不是完整坡道通过 |
| 下行 | 完成30 s前段；世界X前进5.0180 m，侧漂0.6831 m，最低离地0.1869 m | 25.48 s低离地安全拒绝；前进4.8070 m，侧漂0.7108 m，最低离地0.1459 m | Isaac前段存活不能外推到12 m通过或停车 |

表中前进距离投影到坡道方向世界X，区别于旧脚本4.4453/5.0643 m的XY位移模长。Gazebo使用开始运动前中值位置，Isaac同时保存spawn基准和开始运动前中值基准（上/下分别4.3887/5.0143 m），约毫米的初始静止差异不影响结论。

![实际坡道前段对照](../test_results/ramp23_prefix_comparison_20261004/ramp23_prefix_comparison.png)

## 对照条件与不等价之处

冻结CPU actor SHA256 `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`，单env、单推理线程、5 ms物理步长、decimation4、关闭观测噪声。0–0.1 s使用默认关节PD初始化；0.1–3 s Teacher零命令；3 s后机身vx=+0.3 m/s，命令slew为0.6/0.6/0.8。上行spawn `[14.8,7,1.6]`、yawπ；下行 `[1.2,7,2.8]`、yaw0。Isaac本次计划只取30秒，整个活动前段保持前进命令，没有安排停车段；Gazebo按独立完整坡道协议运行，实际安全终止发生在前段。

Isaac夹具导入完整静态SDF为真正物理三角mesh与RayCaster目标；其SDF SHA256 `a1a4f6cb78c600af8535f7874f43c04edee339f28ee0b2ec3fa58df3c0886d33` 与Gazebo上行场景一致。CPU回放所有Teacher动作最大误差上行3.10e−6、下行2.86e−6；实际247维中的指令与有效指令差≤1.24e−8，高度raw裁剪与实际policy扫描误差0。实际mesh命中点在**记录的hit XY**处与精确SDF几何误差上行≤4.56e−6 m、下行≤4.64e−6 m；这个检查没有宣称传感器缓存与每帧新位姿严格同步。

两环境动力学不同：Isaac训练机器人16.087 kg、机器人与地面名义摩擦系数1/1；Gazebo16.512 kg、0.7/1，惯性、附加碰撞体和接触求解器也不同。Gazebo首Teacher有效物理时刻约0.105 s，Isaac0.1 s。Isaac诊断显式关闭扫描式base-height提前终止，改为独立真实下方支撑离地检查，其他原生终止仍在；Gazebo完整验收要求离地≥0.18 m、无机身接触等。不能把不同终止门槛和资产称为完全相同物理。

Isaac上行最终终止发生于`env.step`之后，原始脚本只保存此前状态，没有保存最后post-step状态和各原生termination原因。最后记录base合力为0不证明终止时没有接触，也不能据此断言唯一跌倒原因。初始t0日志含旧derived velocity缓存，下一物理step刷新；t0执行的是明确零动作默认PD，未给Teacher。该缓存限制如实保留。

共同上行失稳表明需要审查策略对这一场景的限制，单凭本对照不足以排除适配或动力学问题。下行差异则值得进一步审查资产、接触与传感器时序。两例均不涉及SLAM导航、实机或重新训练。

## 保存的依据与复现

- [独立JSON对照及全部原日志哈希](../test_results/ramp23_prefix_comparison_20261004/comparison.json)，SHA256 `2cb02d0c82e30ba3a4f6170b0cb451e631c003ce7555ac4ed33eed8f9999d4ac`。
- [真实CPU执行收据](../test_results/full_ramp_isaac_prefix_20261004/execution_receipt.json)：exit0、Kit结束、CPU单env、当时训练进程未动及资源记录。
- [CPU原始运行](../runs/isaac_cpu_ramp23_prefix_20261004_a/results.json)，[Gazebo上行](../runs/20261004_010011_ramp_up_ramp23_full_first_r1_4c51/summary_full_ramp.json)，[Gazebo下行](../runs/20261004_010157_ramp_down_ramp23_full_first_r1_a13d/summary_full_ramp.json)。
- [冻结分析源码](../test_results/ramp23_prefix_comparison_20261004/executed_analysis_source.py)，SHA256 `20884126aa4dcb21657f8e5c62ff2dea5886363902e3a6796d81f8ecad52c30a`；[只读画图源码](../test_results/ramp23_prefix_comparison_20261004/executed_plot_source.py)和[图收据](../test_results/ramp23_prefix_comparison_20261004/plot_receipt.json)。原CPU Python缺matplotlib，分析JSON成功后用系统Python只读原日志补图，没有重跑物理。

离线分析使用`python scripts/compare_ramp23_prefix.py --prefix <CPU原目录> --gazebo-up <上行原目录> --gazebo-down <下行原目录> --output <新目录>`；需CPU torch和matplotlib环境。已存在报告可用系统`python3 scripts/plot_ramp23_prefix_comparison.py <comparison.json>`绘图，输出已存在时拒绝覆盖。此处不是重新运行Kit的命令。
