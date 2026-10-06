# V15：同轮 LIO Jacobian/权重逐行并行

这是从已冻结 `lidar_sampling_v12` 普通复制的新目录；没有复制其 build/install/log，没有改 V12、主项目或 camera_mode。新候选尚未实际运行 Gazebo。Teacher 保持 CPU 单线程、50 Hz、原模型；规范 SLAM state 仍单写入，LIO/VIO/map 提交次序与300 ms源保护不变。

仅机械提取原 `StateEstimation()` 的逐行构造为生产 `VoxelMapManager::BuildJacobianRows()`，同轮 state、外参与 ptpl 固定，每个 i 写自己的 H 行/加权H列/R/z/诊断槽。默认4线程，少于256行串行；原 Eigen 乘法、残差汇总、完整19维prior及状态更新没有改。VIO仍串行，原 residual query 仍4线程。`FASTLIVO_LIO_JACOBIAN_THREADS=1/4` 在进程中首次读取后固定，实际运行由 profile→launch plan→navigation-only环境冻结，不能在运行中热切换。

新 `LIO_JACOBIAN_PREFLIGHT.json` 预检完成前明确拒绝 prepare/actual。`inherited_v12/` 中旧构建报告只作历史依据；它们不能给新 binary 放行。独立有限组件通过也不等于 V15 的运动、导航或全历史 SLAM 重放通过。

证据位于 `../../test_results/parallel_pipeline_20261005/lio_v15/`：

- `source_copy_receipt.json`、`verify_source_equivalence.py`：独立普通副本，原循环表达式逐字节保留；删除新增method/config、还原call即可重建V12完整voxel cpp/header。
- `build_receipt.json`：独立 Release 构建，成功调用明确 `MAKEFLAGS=-j2 -l2`；仅voxel_map编译宏MP_EN/MP_PROC_NUM4，无fastmath/native。初次脚本与编译失败均保留。
- `finite_fixture_comparison.json`：729点完整synthetic StateEstimation，以及未变的9patch VIO forward/inverse，V12/V15线程1/4的27次运行状态/cov与非wall诊断字节一致。不能叫真实全轨迹。
- `real_numeric_comparison.json`：实际V12详细窗口中6196/6068/6339条ptpl的限定组件重放，H/R/z/sigma/bodyworld/HTz对历史记录字节一致，fresh V12参考/V15线程1/4输出互相字节一致。历史HTH重建仍差2.98–4.47e-8，原因未验证；失败保留，不能称历史同上下文完整重放已通过。
- `benchmark_results.json`：48进程pilot，仅每scene/variant4个独立进程；进程内40次计时不能冒充40独立batch。`confirmatory_results.json` 若存在则为每scene/variant32个独立进程的新正式测量。中小场景的不利尾延迟保留。
- `controller_source_equivalence.json`：controller/clock_hold/cascade/publication helper及原v2合同7个pins与V12逐字节相同。

三个候选 profile 与V12原同路线、原验收数值门一致：

- `l64_r30_c30_jacobian4_detail3s_210`：4线程，210 s有界前缀，有边界计时。
- `l64_r30_c30_jacobian1_detail3s_210`：同源1线程对照。
- `l64_r30_c30_jacobian4_600`：4线程、600 s上界、原32目标与双坡道，详细窗口仍115–118 s；不是已经通过的新full run。

实际启动由root完成。预检及源码冻结通过后才使用新私有storage；下面命令在未通过预检时应拒绝：

```bash
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/parallel_lio_v15/run.py --profile l64_r30_c30_jacobian4_detail3s_210 --label jacobian4_prefix --domain 86 --run-storage-root /var/tmp/go2_teacher_parallel_20261005 --prepare-only
```

移除 `--prepare-only` 才是实际启动。本候选没有执行该动作。外部root的UID、0700、非symlink与项目run alias防覆盖验证保持原逻辑；没有改动已封存的 `/var/tmp/go2_teacher_simulation_20261005`。

一轮kernel显著加速不能按相同倍数推断整条SLAM链；完整StateEstimation、VIO、通信和实际路线需要分别报告。三层连接仍是坡道，不称真实楼梯或真机部署通过。
