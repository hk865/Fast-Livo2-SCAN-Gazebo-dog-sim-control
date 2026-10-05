# 下一次实际运行的匹配与可观性诊断插点

状态：**仅只读设计，尚未插桩、编译或运行**。当前 V6 只增大视觉信息权重仍出现二层高度偏差和执行桥保护；该结果不足以证明单一原因。当前及旧 run、源码、参数、验收全部保持原件。下一步应采集求解器实际使用的约束，先分辨弱约束、错误关联、状态协方差耦合和地图污染，停止凭原始点数或图像数推测有效融合。

本设计记录真实 SLAM 内部值，不将 Gazebo 真值送入 SLAM 或导航，不改变 PID、Teacher、观测生成、初始化门、300 ms 超时、围栏、区域到达门或停车门。真值只供另一个离线分析文件对照。诊断日志自身不能升级任何 FAIL 为 PASS。

## 最小实施集合与实际位置

以下行号来自本次检查的本地 FAST-LIVO2 源码。正式实施需另建仿真诊断版本，冻结修改源、编译产物及实际加载的可执行文件/库哈希；不能仅以当前仓库源码哈希冒充运行二进制来源。

| 插点 | 要保留的实际值 | 解决的问题 |
|---|---|---|
| [LIVMapper.cpp:426](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/LIVMapper.cpp:426)，`stateEstimationAndMapping` / `handleLIO` / `handleVIO` | LIO/VIO阶段上下文、绝对clock、原始消息stamp、传播前后/观测更新前后state及cov | 让新证据与原 `mat_pre/out`、原SLAM odom、真实导航命令同源对齐 |
| [voxel_map.cpp:411](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/voxel_map.cpp:411)，`StateEstimation`，每次 `BuildResidualListOMP` 后 | 每个iteration的下采样候选数、`effct_feat_num_`、实际选中 `ptpl_list_` | 原始约14k点不等于被采用的约束，更不等于高度信息 |
| [voxel_map.cpp:462](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/voxel_map.cpp:462) 和 [voxel_map.cpp:482](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/voxel_map.cpp:482)，权重/矩阵构建后、求solution前 | 实际 `R_inv`、`Hsub`、`meas_vec`、`Hsub_T_R_inv*Hsub`、`HTz`、完整19×19先验cov及迭代state | 重建真实垂直信息和向下速度修正的来源 |
| [voxel_map.cpp:489](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/voxel_map.cpp:489)，solution与cov更新 | solution19、更新前后state/cov、rematch/convergence/exit原因 | 区分位姿直接误修正与协方差把误差传给速度/偏置/重力 |
| [LIVMapper.cpp:583](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/LIVMapper.cpp:583)，地图写入 | 以最终估计pose重投影后的实际写入点、选定plane新旧参数/更新序号 | 检查首次高度偏差是否被回写进地图并造成后续自洽偏置 |
| [vio.cpp:371](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/vio.cpp:371)，`retrieveFromVisualSparseMap` | 实际候选、FOV/遮挡/法向/参考缺失/NCC/patch残差拒绝和accepted tracks | 区分没track、track被拒、track存在但信息弱 |
| [vio.cpp:1539](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/vio.cpp:1539) / [vio.cpp:1417](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/vio.cpp:1417)，实际选择的update分支 | 每个pyramid level/iteration的实际有效pixel数、H、z、`HᵀH/img_point_cov`、solution、接受/回滚 | 不能从启用VIO或调整img_point_cov就宣称视觉纠错有效 |

建议首轮只做这些诊断插点，不同时修改关联门、滑窗、鲁棒核、地图分辨率、状态更新、关键帧或回环算法。logger开关关闭时不得更改求解分支或状态；打开时也只从局部变量取快照。

## 同源时钟与阶段索引

由 `LIVMapper` 在分发前创建只读上下文传给logger，至少包含：`run_id`、`estimator_update_sequence`、`stage=LIO|VIO|LO`、`stage_sequence_within_stamp`、`last_lio_update_time_ns`、`first_lidar_time_ns`、原 `relative_debug_time_s`、实际scan start/end、实际image stamp、IMU first/last stamp/count/max gap、实际同步模式、monotonic开始/结束时间、输入和配置哈希。每个观测更新另有 `iteration`；VIO还有 `pyramid_level` 和 `inverse_composition_en`。

直接记录整数ns绝对stamp及原浮点时间，两者转换误差明示。不可继续依赖“点数非零是LIO、零是VIO”的推断，不硬编码d42e的+.01 s offset，不把事后receipt时间或控制器当前clock当匹配发生时间。当前VIO通过 `processFrame(..., img_time)` 获得的是相对时间；正式诊断需由caller显式传入原image绝对stamp及LIO/IMU上下文，不能自行猜测image_clock=odom_clock。

新日志应能与同一stamp的：IMU传播状态、LIO结果、VIO结果、发布odom、地图写入、NAV实际source stamp逐项配对；没有执行的阶段必须写明确 `skipped` 和原因，禁止缺行被解释为0修正或健康。

## LIO：实际匹配、权重与信息

1. `BuildResidualListOMP` 内每个原下采样点保留确定性input index。parallel线程只写自己的诊断slot，串行归并后按input index输出；不在OMP内竞争写文件，不改变原ptpl排序和胜出概率。记录是否主voxel/相邻voxel/递归leaf被搜索，以及最终是否匹配。最小拒绝计数是：无voxel、无平面、平面径向范围失败、sigma门失败、通过但未成为最大prob候选。要区分“所有候选都失败”和“某候选失败但另一个成功”，不能仅数return次数。

2. 选中约束逐条保存：input index、候选查询voxel坐标、选定plane的现有 `id_` / layer / voxel地址的稳定几何键、`point_b_`、`point_w_`、normal、center、d、radius、plane points/eigenvalues、plane covariance6×6、body covariance3×3、signed `dis_to_plane_`、对应实际H行6、`meas_vec`及`R_inv`。plane id/voxel metadata应从实际胜出候选中拷贝到独立diagnostic sidecar，不能从求解后位置重新关联另一plane。id不是跨run一致身份；plane出生/更新clock若目前没有，新增仅诊断sidecar时间，不参与算法。

3. 当前 `PointToPlane` 没有选定plane ID，且 `eigen_value_` / `is_valid_` 未见在现匹配路径赋值；**不要读取这些未初始化成员当作诊断值**。从实际胜出 `VoxelPlane` 读取已初始化字段并标注来源，或缺字段记null。

4. 关联sigma门与求解器权重不同，必须分别记录：`build_single_residual` 的候选sigma包括 `plane_var` 和 `pv.var`（后者含当前pose covariance），使用 `dis_abs < sigma_num*sqrt(sigma_candidate)`；StateEstimation的求解权重则是 `1/(.001+sigma_plane+nᵀvar_body_world n)`。二者不能共用一个sigma字段，也不应以raw cloud平面拟合方差替换。对有限性/正数失败只记实际情况，不改变原分支或加新的算法门。

5. 每个实际iteration在求解前保存：`M6=Hsub_T_R_inv*Hsub` 的全部36 doubles、`HTz6`、`P19_before`、`state_propagat`、`state_iter_before`、原solver构造的完整19维矩阵；求解后保存solution19、`state_iter_after`、`G19x6`或足以精确重建的值、`P19_after`、convergence flag/rematch count/exit reason。原state顺序严格为：rotation0:3、position3:6、exposure6、velocity7:10、gyro bias10:13、acc bias13:16、gravity16:19。尤其保留 `P[pz,vz] = P[5,9]` 及与ba/gravity的交叉项，而不是只记录cov对角。

6. 仅在离线分析计算 `eig(M6)`、translation block eig、normal方向分布、`Σ R_inv*nz²`、`HTz[pz]`、weighted residual RMS/MAD、信息矩阵秩/条件数、最弱特征方向。保存原始矩阵，不因对称化、截断特征值或regularization覆盖原值。rotation与translation单位混合，比较eigs必须同时给出state order和事前冻结的诊断尺度（例如参考lever_length=1 m），并保留未缩放结果；数值小不直接等价于物理高度不可观。

特别注意：平地上大量竖直normal通常对Z很强而对XY/yaw较弱。不能看总体condition number差就断言Z弱。相反，Z信息很强也不证明匹配正确：若错关联到已偏的plane，solver可以很有把握地给出错误Z。因此必须同时看selected plane身份、位置/高度、signed residual以及具体solution，必要时检验Z在其他自由度条件下的Schur信息/最弱子空间投影；不能只用一条总分。

保存normal原符号及signed residual。plane normal翻号会让残差翻号，直接平均signed residual可能抵消；离线可给出符号一致的辅助视图，但不得覆盖原n、d、残差或H。对于|nz|足够大的actual accepted planes，可离线计算查询点XY对应plane Z，按实际plane ID/age观察是否在同一平台位置选到了逐步偏移或另一层plane。

## VIO：从候选到实际像素信息

`processFrame` 的实际调用链是 retrieve → computeJacobianAndUpdateEKF → generateVisualMapPoints → updateVisualMapPoints。当前 `updateReferencePatch` 仅定义，没有在这个调用链执行；诊断应记录实际选择的引用，不把未调用函数当运行能力。

retrieval每帧记录：feat map点数、查询voxel数、候选points、FOV内/behind camera/grid选中数、null或obs为空、depth discontinuity拒绝（当前.5m）、normal未初始化、reference缺失、NCC启用flag/门/实际值、patch SSE与 `outlier_threshold*patch_size_total`、accepted `total_points`、新增/删除观测数。NCC disabled时记 `not_evaluated`，禁止填0并称全部拒绝。

每个accepted track保留：诊断稳定point ID（现VisualPoint若无ID则只在logger sidecar分配、不反馈）、实际reference frame ID/时间/pose、观测数量、当前像素坐标/搜索level、ref与当前patch hash、SSE/NCC、normal/3D点；actual frame/reference切换次数和age、当前图像覆盖/梯度能量可作辅助。不称这些direct patches为ORB feature或ORB keyframe。

实际update branch每个level/iteration，在误差接受判断前记录：`total_points`、实际 `n_meas`（不是total_points×64的猜值）、patch/pixel residual SSE及分位数、图像协方差R、exposure/增益开关、实际H_sub和z或其hash/数组、`HᵀH`、**`HᵀH/R`**、`Hᵀz/R`、P19_before、state_propagat/当前state、solution19、实际更新前后state/cov、误差是否非增/回滚、停止原因。exposure关闭的最后H列可能为0；要记录实际维度6/7，不能从配置猜完整rank。对于n_meas=0、NaN、空图像或total_points=0必须明确记skipped/invalid；不为记录方便改算法。

VIO当前R从1e8改1000只恢复同H的理论信息权重；如果accepted tracks为0或几何/光度H弱，增强R不会自动产生约束；若关联/相机标定有误，增强错误约束可能更糟。下一次应实际比较同方向Z信息、tracks和观测更新，而不是仅比较参数值或彩色地图画面。彩色点云只是投影显示，不能作为视觉匹配成功证据。

## 存储、资源与下一次判读

最小全程日志是每LIO/VIO实际iteration的聚合矩阵、state/cov/solution和拒绝计数。逐约束/逐track的大数组以binary double结构独立归档，采用事前固定时间窗（同路线建议115–165 s，涵盖已出现的二层跳变）与每个stage首/末实际iteration；其他iteration仍保留完整聚合M、HTz和P。固定窗不能因结果差而挑“好看”时段。若启动阶段变更导致未进入窗，明确本轮detail覆盖不足，不能声称重建了未保存H。

只有只读快照进入bounded writer queue，后台写盘；队列drop、最大深度、写入耗时、诊断CPU/内存/文件字节都归档。不能为日志堵塞放宽300ms；日志完整性不满足时标记诊断不足。保持原求解顺序和线程设置，不在实时回调做eigendecomposition或逐点JSON打印；6×6/19×19 raw矩阵记录已足够离线算eigs。配置/source/build flags/binary/schema哈希、数组shape/endian、source stamp和append-only索引必须可查。

最低离线验证：从saved H/R/residual重建M/HTz与实际logger矩阵一致；由P/state差复现solution及Δpz/Δvz；有效数与逐点accepted数一致；阶段stamp和iteration无错配；全零/单水平plane/正交planes/错误signed residual/不同prior交叉项的数学坏例能区分。logger关闭与打开的既有算法段应逐字节审计，先确认记录不改变数学再进行正式freeze；本设计没有实际完成这些检查。

下一轮判读顺序：

1. 在首次ΔZ/Δvz异常之前，effective constraints与Z方向信息是否已经下降；若下降，检查实际FOV、surface切换和候选拒绝。
2. 若Z信息强，actual selected planes/residual/HTz是否指向向下修正；检查关联和map更新是否使偏差持续自洽。
3. 若LIO pose修正不大但Δvz显著，重建完整cov交叉项和solution，区分在线bias/gravity估计及先验耦合。
4. VIO是否有足量实际accepted tracks和Z方向信息、是否提供相反修正、还是没特征/被拒/低rank/回滚。图像权重更大但没有这些证据，仍不能说融合生效。
5. 局部问题清楚之后才评估关键帧/全局回环：参考age/视角/覆盖骤变可支持改局部观测管理；同一已见区域返回仍有一致全局误差才为回环需求提供证据。当前单向二层connector首次出现的短时高度跳变，不能仅靠“加回环”解释或保证修复。ORB-SLAM3不是现core，需要新的坐标/尺度/时间/融合合同，不能与现局部patch管理混称。

本轮只写此提案，不修改任何编译源码，不运行节点或仿真。原初始化与匹配审计可交叉阅读 [SLAM_INITIALIZATION_MATCHING_AUDIT_20261005.md](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/docs/SLAM_INITIALIZATION_MATCHING_AUDIT_20261005.md) 和 [INIT_MATCHING_APPENDIX.md](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/INIT_MATCHING_APPENDIX.md)。
