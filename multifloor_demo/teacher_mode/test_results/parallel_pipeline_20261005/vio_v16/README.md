# V16 独立 VIO patch 并行候选

当前状态：独立 1／4 线程 Release 构建、123个有限数值进程、768个正式性能进程批与25个启动门边界检查完成。选择 t1；t4 没有满足全部事前性能门槛，保留失败。本目录不包含 Gazebo 或导航通过证明。生产 preflight 为 `PASS_LIMITED_BUILD_NUMERIC_AND_BENCHMARK`，`actual_runtime_verified=false`。

- 基线：`navigation/lidar_sampling_v12`；普通源码副本：`navigation/parallel_vio_v16`，没有复制 build/install/log 或旧候选的构建 PASS。
- 数值：`numeric_comparison.json`，41 场景 ×3实现＝123 独立进程，state/cov/G/H/errors/referenceH 和原非壁钟诊断记录全部逐字节一致。观察到98个接受迭代、19个回滚迭代。覆盖范围与原 nullptr precompute 限制见 [SOURCE_AND_REPLAY_SCOPE.md](SOURCE_AND_REPLAY_SCOPE.md)。
- 真实 OpenMP team：`team_partition_proof.json`。1／4线程候选共有234个更新 region 的真实 team 观察；编译器静态商余数分区与互斥 H/z 行写入给出覆盖证明，不把该公式当作逐 i 运行追踪。
- 先行性能：`benchmark_provisional64.json`，每 scene/variant 8 独立进程、每进程4内部有效样本，只作 pilot。任何32样本表述均不是32独立进程批。
- 正式结果：`benchmark_confirmatory32.json`，forward/inverse ×32/128/512/1024 patch ×V12/t1/t4，每 scene/variant32独立进程批，每批12预热+30内部有效样本；六进程对称循环投影到任意两实现为 ABBA/BAAB。统计单位是进程批。所有768个最终输出逐字节一致。
- 原始动态加载证据：`loaded_library_witness.json`，统计结束后3个单次小fixture的 LD_DEBUG 调用初始化路径与下面库哈希一致；该观察器未用于任何统计进程。先前只读 `/proc` 采样未抓到仍在运行的fixture，空采样原件也保留，没有冒充成功。
- 启动门：`candidate_gate_tests.json`，25个内存叠加边界通过，包括外国候选、错误库、修改来源、缺失数值门、不足独立批／样本／预热、失败性能、真值导航、放宽TTL、错误控制器等拒绝；原候选源码及收据未被测试修改。

编译阈值冻结为64 patch，四线程候选在该值以下使用 team1；选择的 t1 在全部规模均串行。t1和t4私有库分别保存在 `candidate_V16_t1`、`candidate_V16_t4`，避免安装目录切换污染对照。当前工作区安装 t1；只有 `vio_patch1_l64_r30_c30_210/600` 与选定库一致。profile4 由启动门拒绝。

正式较大场景的独立批 median（ms）：

| 模式／patch | V12 | t1 | t4 | t1 相对V12改善 |
|---|---:|---:|---:|---:|
| forward／512 | 2.813 | 1.982 | 2.117 | 29.5% |
| forward／1024 | 7.875 | 5.632 | 5.424 | 28.5% |
| inverse／512 | 0.937 | 0.654 | 0.609 | 30.2% |
| inverse／1024 | 2.465 | 1.879 | 2.265 | 23.8% |

t1 四个较大场景均满足 median 至少改善10%和两种p95不退化，小32patch也改善并无尾部退化。t4 的 inverse512 内部样本p95的批间median比V12增加11.3%；inverse1024 median仅改善8.1%，低于10%门槛，且内部p95增加0.7%。不能把t4作为全场景性能通过配置。相对同源t1，t4在forward512慢6.8%、inverse1024慢20.5%；128patch部分median更快，仍不能宣称普遍增加线程更好。

t1同样比V12快，说明V12→t4的收益不能全归于四线程。候选包含临时变量作用域、独立计数和编译后的循环布局变化；具体编译优化归因未测，只给直接对照结论。这里测量的是完整 synthetic 更新函数及其求解部分，尚未测实际前端时序或整链。

独立库哈希：

| 实现 | SHA256 |
|---|---|
| V12 baseline | `8303ec94d33a976bc8ac0940e9d3d213120ce4a035ea22385e07f301c019efed` |
| V16 t1 | `e571a101cd7091601f55d49942c52563512823b8a6e2b17174548704bb6ed2c9` |
| V16 t4 | `b98984378f144521956ee359e405c7a4d566f880904303f797c1fc5f861c8318` |

复核命令（统计脚本只在 root 分配的无物理／构建独占窗口执行）：

```bash
python3 -B multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/vio_v16/compile_fixture.py
python3 -B multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/vio_v16/run_numeric_fixtures.py
python3 -B multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/vio_v16/benchmark_confirmatory.py
```

数值／统计脚本使用独立新输出目录，拒绝覆盖旧结果。已完成的数值脚本不能直接再运行到同目录；复核应新建独立 revision 路径。编译脚本依赖先保留 t4 私有库、再构建 t1，不能把后来工作区安装库当成 t4。

重建选定 t1 必须显式指定 `-DV16_VIO_PATCH_THREADS=1 -DV16_VIO_PATCH_MIN_POINTS=64`，并设 `MAKEFLAGS='-j2 -l2'`。原CMake候选默认4仍保留，不能省略此参数后把四线程库当作选定t1；启动门核对真实 CMakeCache 与库SHA并拒绝混用。

边界 fixture 的最初 OpenCV 构造错误完整保留：`fixture_initial_failure.json`、`vio_patch_fixture.initial_braces_failure.cpp`、`*/fixture.initial_braces_failure`、baseline `numeric_initial_braces_failure` 和 `opencv_allocation_probe.log`。修正后的123个进程没有删除或覆盖这项失败。

启动门测试最初使用独立模块实例，使模拟哈希补丁没有到达 runner 实际导入的模块；该测试工具错误保存在 `candidate_gate_initial_harness_failure.json` 与 `test_candidate_gate.initial_wrong_module.py`。修正测试的 canonical module 注入后25项全部通过；生产 gate 源码没有因此改变。

后续实际仿真由 root 单独启动、生成新 run 和实际库 witness。新固定外部存储根为 `/var/tmp/go2_teacher_parallel_20261005`，保留 current UID/0700/禁止 symlink-root/严格 alias 验证。Teacher 仍 CPU单线程，232维特权观测范围、300ms超时、唯一执行器、真实SLAM/SCAN导航、32区域与原停车／坡道判据保持。继承的 publication component preflight 仅证明未变的控制发布数学组件，不代表 V16 C++ 构建或实际导航已通过。
