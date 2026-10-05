# V16 的数值边界与并行范围

本候选从独立冻结的 `navigation/lidar_sampling_v12/slam_ws/src` 普通复制。C++ 只修改 `vio.cpp` 的 `updateState` 和 `updateStateInverse` 中 patch `i` 循环，以及该翻译单元的 CMake 编译定义。LIO4、IMU 积分／去畸变、地图、控制器、导航判据与 Teacher 均继承原字节。全局 VIO `MP_EN` 浮点 reduction 没有开启。

每个 patch 保留原像素访问、Jacobian、曝光交叉项和 `float patch_error` 的累加顺序。H/z 的行区间为 `[i*patch_size_total, (i+1)*patch_size_total)`；`errors[i]`、独立 `float patch_errors[i]` 和 `int patch_measurements[i]` 各自只由一个 i 写入。离开 OpenMP 隐式 join 后，按原 i 顺序累加 error 和 n_meas，保留原 nullptr 跳过分支。inverse 原函数作用域的 pc/JdR/Jdt 移到 patch 作用域，避免共享临时变量。Rcw/Pcw、状态、相机、图像、参考缓存在循环中只读；先验、求解、接受／回滚、金字塔级别和参考缓存构造仍串行。

`team_partition_proof.json` 的 234 条记录来自 test-only GOMP 观察器，真实运行 team 为 1 或 4，不更改 requested team。候选源码 `schedule(static)` 与编译后 omp_get_num_threads/thread_num + 整数商／余数分区对应，给出每 team 的互斥覆盖区间。该区间是编译器分区证明，**没有把理论区间冒充逐 i 的运行日志**。观察器不进入生产 runner。CPU 耗时基准不 preload 观察器或任何 team override。

有限 fixture 输入包括 512×512 单通道合成图、明确针孔相机内参、3 层参考 patch、确定的世界点和 prior state/cov、参考曝光、search level、prepared inverse Jacobian，覆盖 forward/inverse、曝光开／关、level0/1/2、混合 search、nullptr、all-null、fresh reference cache、多级 update、零／小／较大输入、63/64/65 编译阈值与接受／回滚。输出逐字节比较 state/cov/G/H/errors/referenceH 与原 200/201/203 非壁钟诊断；这比只核对一次 Hessian 或状态加载更完整，但仍只是 synthetic 更新方法边界。

原 `precomputeReferencePatches` 先读 pt->ref_patch 后才检查 pt==nullptr；本轮没有改此原前置条件。inverse nullable 测试使用已构造的 reference cache，fresh-cache/multi-level inverse 输入非空。不能声称涵盖 fresh-cache+null 的不合法原输入。

现有实际日志没有完整保存原相机图、参考图／patch、visibility/submap 和所有特征关联。因此这些 fixture 不能称为原路线 `processFrame` 回放。kind201 Hessian、kind203 H/z 也不足以反推出原 RGB/patch 输入。实际完整 SLAM/SCAN 仍由 root 在独立 run 身份下验收。

最初 fixture 把 `cv::Mat image{512,512,CV_8UC1}` 解析为三个 int 的列表，造成 3×1/CV_32S 内存越界，在 baseline 尚未进入更新前失败。原错误源码、二进制和失败日志已保留；`opencv_allocation_probe.log` 独立证明构造类型，修正为显式 `cv::Mat(512,512,CV_8UC1)` 并检查形状。此失败不归因于 estimator，也没有删除。

`benchmark_provisional64.json` 是先行的每 scene/variant 8 独立进程 ×4 内部有效测量，仅作 pilot。正式 `benchmark_confirmatory32.json` 采用每 scene/variant 32 独立进程批、每批12预热+30内部有效，三个 variant 的任意两者投影均为 ABBA/BAAB。统计单位是进程批，960 次内部测量不是960个独立批。V12、同源 t1、t4 同用 P 核 0/2/4/6，并保存频率、loadavg、/proc/stat。t1 也发生编译／变量布局变化，所以 t4 相对 V12 的收益不能全部归因于线程。
