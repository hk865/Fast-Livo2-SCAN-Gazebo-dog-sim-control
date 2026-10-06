# V18 完整静态32区域三层坡道独立验收

原范围的完整仿真闭环通过，新闭源 metadata-v2 联合原 common19、publication26、ramp25 门。原三份 UNVERIFIED 收据保留；新读者只修正冻结 producer 历史行比原status payload多出的两个归档字段比较，完整payload唯一匹配、原因果顺序、原187ray和全部数值门仍保持。新来源闭合收据 SHA256：`1398f53ec33310c510944ba5100d62d67d205c7596250b972a89332bb57e5280`。

运行：`/var/tmp/go2_teacher_parallel_20261005/20261005_234102_closed_loop_cascade_clock_hold_combined_v18_full_r1_ab53`。600s 是上限，实际 Teacher 到 264.600s，native 到 264.605s；Teacher 13231 条、native 52921 条，native 200Hz 连续，无body contact/fault和物理期失败guard。runner和CPU sampler均退出0，所有owned cleanup确认。原真SLAM32/32三维区域到达，最后目标驻留截止249.085s。两条原12m坡道均完成COM入坡到出坡、四足顺序接触、12m全部分箱支撑、连续landing与正确物理楼层。坡道不称真实楼梯。

| 实际SLAM/控制来源 | 结果 |
|---|---:|
| SLAM original header 数 | 7907，重复/倒退0 |
| SLAM源header平均频率 | 30.29583Hz |
| SLAM源最大header间隔 | 65.0ms；>300ms 0 |
| pose sim age median/p95/max | 45.0/65.0/80.0ms |
| pose original receipt wall age median/p95/max | 18.30/36.98/136.09ms |
| paired IMU sim age median/p95/max | 50.0/65.0/85.0ms |
| 真数学控制行/平均sim频率 | 4634/17.97408Hz；ticker20Hzwall |

年龄统计取实际未受保护控制行；完整all-mode统计也未超过300ms。控制数学包括暂停/转向/停车语义，数学行之间的间隔不等于SLAM源丢帧。Teacher执行频率是仿真50Hz、CPU单线程。Actor推理p50/p95 0.320/0.410ms，整个worker进程CPU不能当模型推理CPU。

| 原离线实际路线门 | 结果 |
|---|---:|
| 最大横向路线距离 | 0.138906m（门0.2） |
| 活跃路线RMS | 0.037279m（门0.08） |
| 最大行走朝向误差 | 0.166577rad（门0.2） |
| 实际COM投影均速/参考均速 | 0.171186/0.191085m/s |
| 速度MAE | 0.038278m/s（按原门） |

路线max/RMS按一次原来源锚点做body-relative offline SE3；绝对原场景中心线注册仍UNVERIFIED，不能把上述数值宣称为绝对全局定位精度。Gazebo真值只在命令/真实传感器源审计之后用于离线几何验收，导航仍用实际SLAM/IMU/SCAN。

首个原来源声明的固定5s active hold 窗口为 257.345–262.345s，native1001/SLAM151条。native xy/yaw漂移 2.140mm/0.004682rad，峰速 0.018309m/s、峰wz 0.029003rad/s，全部原门通过。不是action=0停车。

本次只证明一条原静态32区域路线的仿真导航和两条坡道。Actor仍是232维特权输入+15维已知命令/上一动作；动态障碍停车恢复、完全真实传感器Actor、1m/s、GPU部署、真机部署均未由本轮验证。full冻结关闭kind300，边界耗时标N/A，不记0。V18是组合候选，无法把实际收益拆分归因单一模块。

[FULL_ORIGINAL_SCOPE_COVERAGE.json](FULL_ORIGINAL_SCOPE_COVERAGE.json)保存精确原门/指标与来源哈希；所有失败历史和原收据不改。
