# V18：LIO4 + VIO1 独立计算组合

以冻结 V16 单线程视觉 patch 源码为底，合入 V15 按索引并行的 LIO Jacobian/权重构造。残差匹配4线程，LIO Jacobian4线程（有效约束少于256时串行），VIO patch1线程。保持原逐行公式、索引汇总、Eigen乘法、19维先验、单状态写入和地图更新。Teacher 为 CPU1线程；实际 SLAM/SCAN 来源、300ms保护、执行器唯一性、32区域路线与停车保护保持 V12 同字节。

本候选独立编译并新跑729点 StateEstimation 与41个 VIO 边界测例；状态/协方差和原非wall诊断逐字节一致。组件性能来自独立 V15/V16 有限基准，组合全链吞吐尚未实测。V15历史HTH重建未逐字节一致、small/medium全State尾延迟退化以及V16四线程性能失败均保留，不据此宣称仿真/导航已经通过。

启动前校验将核对本候选源码、实际编译设置、私有库、两套新数值收据、线程/加载观察与原控制器。只导航栈继承 FASTLIVO_LIO_JACOBIAN_THREADS=4 和 FASTLIVO_VIO_PATCH_THREADS=1；后者核对编译设置，不改变运行时算法。直接启动 SLAM 也要求二者与冻结profile一致。

```bash
/usr/bin/python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/combined_compute_v18/run.py --profile combined_lio4_vio1_l64_r30_c30_210 --label combined_prefix --domain 86 --run-storage-root /var/tmp/go2_teacher_parallel_20261005 --prepare-only
```

去掉 `--prepare-only` 才实际启动，实际测试由主任务执行。`combined_lio4_vio1_l64_r30_c30_600`为同32区域完整路线600s候选；210s仅前缀，不能当完整路线通过。两个profile诊断仅仿真115–118s，600s不新增逐点全程记录。外部run创建要求current UID、0700、非symlink根，并保留项目runs的唯一alias；不覆盖旧run。

证据目录：`../../test_results/parallel_pipeline_20261005/combined_v18/`。不修改camera_mode、旧V12或模型，不操作真机或训练。
