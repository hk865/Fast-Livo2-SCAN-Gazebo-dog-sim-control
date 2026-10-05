# V15 同轮 LIO Jacobian/权重并行：构建、数值与性能报告

2026-10-05。仅本模块独立源码与组件测试，未运行Gazebo、Teacher或新的导航场景。V12/相机Demo/训练/历史run未修改。实际controller接口与7个数学/保护来源pin均保留原字节。

## 改动范围

独立普通副本 navigation/parallel_lio_v15；新slam_ws由V12真实src复制，没有复制build/install/log。core中只有voxel_map.cpp与对应header两个源文件改变；CMake保留V12对voxel_map.cpp的MP_EN/MP_PROC_NUM4，其他cpp尤其VIO仍没有MP_EN。原StateEstimation逐行loop机械提取成生产BuildJacobianRows，原表达式逐字节保留。新BEGIN/END块移除并原call还原，可重建原V12完整cpp/header；verify_source_equivalence.py已执行通过。

各i拥有Hsub row、Hsub_T_R_inv col、R/z及sigma/bodyworld槽；固定本轮state/extrinsic/ptpl，无共享浮点归约，隐式barrier后仍原Eigen产品、原索引残差汇总和完整19维prior。保留单canonical state writer，VIO/map commit/IMU时序不并发。

参数FASTLIVO_LIO_JACOBIAN_THREADS仅允许1/4，首次访问后缓存；profile及navigation-only环境冻结。256是accepted constraints行数门，不是原始点数门。small147点实际仅48constraints；test-only GOMP观测已证configured4时该row循环实际team1，原residual query仍4。真实6196constraints分别证实际team1/4，P核CPU0–3、FE_TONEAREST0/MXCSR control8064一致。观测preload的时序不用于性能。

## 构建与失败

独立Release构建最终成功；MAKEFLAGS=-j2 -l2并明确保留无colcon额外jobs的command.log，-O3/-funroll，未添加fastmath/native。首次相对输出路径、ROS setup配合nounset、header Eigen类型未限定导致编译失败分别保留。首次colcon忽略CMAKE_BUILD_PARALLEL_LEVEL而追加-j20的事实也保留；成功构建使用明确MAKEFLAGS，未声称峰值compiler数已独立采样。

## 数值范围

27个有限synthetic进程：729点完整StateEstimation及9patch未改VIO forward/inverse，三variant V12/V15T1/T4 ×诊断off/inside/outside。state/cov/solver输出及全部非wall诊断逐字节一致，所有writer无丢失/错误。LIO包含3个迭代aggregate与原first/final H/R/z详情。它不是实际完整SLAM轨迹重放。

三组实际原V12 kind101 ptpl输入：6196/6068/6339，严格关联同seq/stamp/iteration kind100与冻结extrinsic。H/R/z/sigma/body_world/HTz对旧实际记录逐字节一致；fresh V12逐字节原body参考与V15生产方法T1/T4输出互相逐字节一致，包含新计算HTH。实际记录HTH重建仍差2.98–4.47e-8；dense6×6目标、恢复原19×19block目标及OMP4诊断均未消除。原因未证，不能归于layout/核型，不能放宽bitwise门。historical_archive_aggregation_replay=NOT_BITEXACT_UNVERIFIED_CAUSE；没有完整地图/视觉cache checkpoint，historical_full_estimator_replay=UNVERIFIED。

384个正式进程中，同输入64次输出全部逐字节一致；大点数synthetic覆盖完整StateEstimation最终state/cov、有序完整residual、H/加权H/R/z/sigma/bodyworld、原Eigen HTH/HTz。真实输入仍只是row组件。直接移动原loop与原表达式证明、有限完整估计与大点数fresh比较应分别阅读，不能把来源范围合并成“历史全轨迹通过”。

## 性能

原48进程pilot每scene/variant只有4个独立batch，全部保留，不能把40次进程内reps当40个独立batch。新confirmatory每scene/variant32个独立进程，共384；每批10warmup+40valid，ABBA/BAAB对称，taskset P0–7、固定OMP places；记录每批wall/processCPU/threadCPU、频率/负载、输入与输出hash。统计是每variant/scene1280组内计时合并及32批聚合分别列，组内不是独立batch。

下表为row kernel，单位ms；它不是总SLAM处理链。

| 场景 | median V12 → V15 | p95 V12 → V15 | median改善 | p95不退化 |
|---|---:|---:|---:|---|
| actual_first_6196_rows | 0.131445 → 0.036250 | 0.135742 → 0.057787 | 72.42% | True |
| actual_maximum_6339_rows | 0.135333 → 0.037169 | 0.141258 → 0.058105 | 72.54% | True |
| actual_minimum_6068_rows | 0.129099 → 0.035665 | 0.134191 → 0.050926 | 72.37% | True |
| large24843 | 0.568126 → 0.140043 | 0.642884 → 0.225896 | 75.35% | True |
| medium2883 | 0.057993 → 0.018928 | 0.060667 → 0.023446 | 67.36% | True |
| small147 | 0.000969 → 0.001137 | 0.000982 → 0.001194 | -17.34% | False |

large24843完整synthetic StateEstimation median21.870→20.6352ms、p9524.47951→23.26256ms，仅约5.65%中位改善。medium2883 row正式p95改善，但完整StateEstimation p953.849232→4.362522ms仍退化；small48constraints固定串行回退，row额外约0.0002ms、完整stage p950.142712→0.162459ms退化。pilot medium rowp95退化保留，正式不覆盖旧失败。没有在测试途中改变256阈值，没有根据有利样本删除数据。暂不能以这些结果宣称整体导航加速或更高实时时序通过。

## 运行依赖与状态

候选profiles为4线程210s、同源1线程210s及4线程600s；原32目标、路线、TeacherCPU1、50Hz、physics0.005、300ms和停车保护保留。新外部run root固定/var/tmp/go2_teacher_parallel_20261005，其0700/UID/非symlink/alias防覆盖保持原逻辑；没有改旧封存root。

LIO_JACOBIAN_PREFLIGHT是新fail-closed gate，旧V12报告在inherited_v12，不能给候选binary放行。只有独立评估认可本有限组件合同并冻结source/library/fixture/criterion references后才允许新candidate prepare；实际运动/导航依然未验证。Root选择是否进行210s实际前缀及后续full。未操作真机，未训练Teacher。

主要证据：source_copy_receipt.json、source_strip_proof.json、verify_source_equivalence.py、build_receipt.json、finite_fixture_comparison.json、real_numeric_comparison.json、原失败.dense6x6_target_attempt、benchmark_results.json（pilot）、confirmatory_results.json、confirmatory_summary.json、small_serial_and_FP_observation/receipt.json、real_team_FP_observation/receipt.json、controller_source_equivalence.json。
