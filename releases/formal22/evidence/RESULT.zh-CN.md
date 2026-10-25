# 第22轮：当前R85 F1→F3完整任务通过

本轮在选定的新质量/地形策略下完成完整F1→F3任务、真实动态障碍停止与恢复、首次T0起连续固定5秒停车、当前RGB地图检查。task_result为succeeded，passed/parking_pass/dynamic_pass/full_F3_PASS均true；original46_PASS仍false。不是短测或区域到达。只证明这一次实际运行通过，不代表旧5%策略、原车体导数限制、原46点任务或所有间歇故障均通过。

## 实施与实际加载

- 独立候选 `0f35662822925a68de9a659558cc5073845b5b87`，工作树 `worktrees/curve_support_grade`；部署 `536c9d3f4773ef43edef48992c598ccf2fc3f021`，工作树 `worktrees/runtime_compact_formal_22`。二者clean，原source HEAD仍45396505，无远程main/推送改动。
- 本轮相对21仅 `candidate/r85_geometry_admission.py` 业务变化，SHA `73804fb93fc450bfcf44b1cb0086bddda97bb5587c6b66e23187027427afd3b2`；部署另外四个行政文件精确匹配提案。原prepare48项、实际父端/worker/Actor/SCAN/native等加载91项通过。实际worker geometry SHA一致。
- 32源库存、原native571、SCAN独立构建、ASYNC relay、明确static_support_nonforeground/v1质量模型、status分类修复保持；R99 OFF、S1 OFF，速度/PID/模型、300ms/IMU因果检查、碰撞/倾角、停车阈值未变。未夹带新速度研究。
- Run：`/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261025T042130Z_closed_loop_R85_compact_formal_22_curve_grade_round27_74bc`。runner3641717/start17569885，outer3641362/start17569577。实际原startup审批执行，未被拒；旧18未执行的拒绝回执不改。

## 这次改哪里、为什么

`_curve_patch`复用原Bezier控制凸包、affine残差与支撑/SAT计算。仅整个连续段和原0.51m脚印完全落在同一有限面、原0.10m body/ground残差及净空/碰撞均满足时，用该面的真实最大坡度界认证地形，仍为0.35rad；不满足时回原typed cone。原XY精确恒定而Z变化的public及private子段拒绝保留，无新epsilon/阈值、采样放行、深度或时间预算放宽。旧body导数诊断保留，并明确实际依据。

21轮31个末端worker任务均完成消费，唯一false为typed ground cone；两条真实旧曲线重放发现同floor3真坡0、body高度残差约1.5mm，却把body高度收敛导数误作地形坡。新的同面语义可修这类错误，但**取消了该分支对body Z/XY导数的旧间接约束，不是动力学或旧保证等价**。静态证书不代替实际控制/执行验收。中间20f31的25项有限检查和最终0f3566的1个必要vertical-only增量分别保留，未谎称最终全suite复跑；独立源审12项通过，proposal17项及final绑定22项通过。

## 本轮结果

- 实际任务开始11.750sim秒，完成350.930sim秒，任务区间339.180sim秒；路线高水位34.907904m（路线投影进度，不冒实际累计路程）。world最终365.890sim秒，原runner自然返回0；包含启动/收尾的总墙钟456.125秒不能当纯运行RTF。
- 动态真实stop177.565sim秒（cloud177.510/traj97），resume203.625，恢复进度0.593909m至207.175，业务clear207.225。最终独立verification.allowed/passed均true；初始event_window的business_completion_only/physical_verified=false是历史业务快照，不覆盖最终物理验证。
- 停车首次T0=344.880，结束349.880，连续5秒且never reset；Actor输入344.810为精确零，native ACK344.830，资格344.880。原六项固定5秒核验全过。窗口最大平面速度0.073033m/s、native wz0.037483rad/s、SLAM XY漂移0.044373m、yaw漂移0.008698rad。
- 当前RGB保存检查通过：233102点/10578种颜色，容量拒绝0；来自本轮FAST-LIVO2真实输出，不是参考地图或导航真值。

## 实际触发与归因边界

完整179份唯一连续证书（分类/schema/payloadSHA/T/完整sourcebindings去重）：SCAN165份中29份触发33个patch，旧路径保留connector14份中12份触发12个patch，合计41份/45patch。不能把各内部旧coarse cone false等同旧整条曲线拒绝。末两条实际SCAN payload离线旧整certifier也通过，旧patch10/depth2及8/depth1→新各6/depth0；证明更浅细分，**不能把本次整轮PASS唯一归功该分支**。22最终floor3未见新依据触发；旧21的真实拒绝与新22通过必须分开。完整统计与两次旧回放见 `../../protocol_reduction/curve_support_grade/FORMAL22_EFFECT.json`，现场新worker与离线旧测时不同、含冷初始化，不计算净加速。

controller在440个现成采样、439.390s墙钟窗消耗407.880CPU秒，平均92.829%单核，RSS峰305684480B。这不是整机负载/等待或GIL、GC归因，也不是与不同运行窗口的速度收益对照。一次无队列/时序致命错误不证明所有尾延迟根治。

## 清理与边界

六项原closure均true：runner/自有组退出、退出回执、私有域清空、无匹配运行、三锁可用。Actor18295样本CRC通过，pipeline正常、无outer停止/强制停止或向其他进程发信号。原ENTRY_EXIT.navigation_PASS/closure.formal_navigation_PASS为工具固定不授业务PASS的字段，业务结论来自原r85_task_result及各验收回执；不改旧文件来统一标志。

收尾后原资源盘点保留 `120278765568` B（120.279GB），加下一轮10GB额度和10GB额外预留为 `140278765568` B，低于170GB；本轮未永久删除。当前无仿真，不追加成功后重跑。

## 本轮以外已实现/尚未宣称

- 前景质量改用现有robust fit、静态支持/退化/残差/覆盖/歧义/两帧确认，取消独立5% veto并明确nonforeground面误差；没有删除避障当前观测，不把拒绝一次地图修正等同丢弃连续里程计。该模型非旧5%保证等价，原失败保留。
- 原native状态协方差导出候选d62e282已独立实现、私有构建与实际ROS CDR/adapter映射检查，默认OFF；此次仍原native571，未物理加载、未标定、未用于控制或恢复。corrected_odom仍未融合地图修正不确定度。细节见 `../../native_pose_covariance/REPORT.zh-CN.md`。
- 未新增回环、位姿图或完整全局重定位；已知场景辅助仍是局部匹配。后续采纳/稳定性需保持上述精确范围，不将单轮PASS泛化为全局定位正确性或新旧策略安全等价。

## 最终独立复核

现成证据独立复核19/19通过，无阻断：`../../investigations/deployment/FORMAL22_ACCEPTANCE_INDEPENDENT_REVIEW_FINAL.json`，SHA `fcbb786d20aba7aeb968610e4a821974c1f859413b5ecb73535b83d423a02ce9`。核对停车原阈值、事件/Actor零输入/ACK/T0、动态stop/resume/恢复与验证时owner保存快照、最终task绑定，以及91实际加载、6closure/ActorCRC/正常pipeline。首次把可变收尾runtime_state当作验证时snapshot的错误假设回执原样保留；验证时5092B快照已在owner证据中完整保存且SHA一致，不补写原证据。复核未重跑suite/verifier/CRC/closure或启动新实验。
