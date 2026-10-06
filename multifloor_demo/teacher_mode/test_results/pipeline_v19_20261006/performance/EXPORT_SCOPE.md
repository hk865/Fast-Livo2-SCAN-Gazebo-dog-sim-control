# V19 精简导出范围与复现边界

本目录是2026-10-06原实验工作区 `pipeline_v19_20261006/{performance,core}` 的同结构精简副本。原脚本、配置、协议、报告与被选摘要保持逐字节原件，SHA和选择/排除清单在 [COMPACT_EXPORT_MANIFEST.json](COMPACT_EXPORT_MANIFEST.json)。此复制没有运行新路径上的物理、模型或有限数学检查，没有新PASS。

## Git 保留

- 分阶段23列/7阶段lifecycle只读reader、输入索引/比较、pose与sync比较脚本、事前协议和有限反例测试。
- 原生产packet/lifecycle的C++fixture与脚本、导航/相机fixture配置、核心有限收据及source/build哈希。
- 新鲜V18/V19算子数学fixture、FP/team/private-loader observer源码、100process回归脚本、最终atomic收据与原先版本。observer是test-only，从不进入production性能统计。
- 六轮SLAM-only的参数、执行计划、实际loader与退出收据、stage/输入头/派生耗时摘要；新实际60s前缀的只读时序收据。

## 未进入 Git、仍在本地保留

原始MCAP/完整CDR内容、每条捕获输入索引JSONL、传感器/odom/CPU原始JSONL、诊断records.bin/canonical.bin/output.bin、大型event/index CSV、实际日志/生成地图、编译产物与动态库均没有导出。清单列出本subtree中每个遗漏本地文件的路径、大小、原因；rawbag单独登记。原始大清理授权只涉及旧已清理数据，本轮刚采集83e9包和V19新数据**没有删除**。

只有摘要和SHA不能独立重审完整原运行。当前本地raw尚存，可以读取那些明确路径进行同一实验完整复核；另一台仅有Git的机器必须重新采集/运行新的输入和验证，不能从SHA恢复被省略数据。

## 能直接复用与需要新迁移的入口

纯reader接受 `--run` 或 `--events/--lifecycle/--summary/--out` 等输入路径，通常可在准备同schema新数据后复用。缺少raw时应报缺失，不能补0/伪造原通过。

`run_shadow_replay.py` 是历史实验受控入口：固定原核心库及fresh-math收据SHA，实验机ROS/依赖路径也明确写入。`core/run_production_checks.py`/`run_lifecycle_checks.py` 是原实验harness，仍保留历史工作区/构建和旧配置来源路径；它们不能被宣传为clone即可运行的通用工具。Git中的 `navigation_fixture.yaml`/`camera_fixture.yaml` 提供实际有限fixture配置，原旧V12 raw配置目录已经按此前授权删除，因此新路径复现必须指定现存配置并重新编译/验证。

算子harness `finite_math/atomic_final/run_fresh_math.py` 使用派生workspace和freshbuild flags，但基准/候选库的来源与原SHA仍强绑定。新repo重新构建会产生新的路径/二进制身份；必须由本地迁移工具或独立新harness重新冻结新加载库、参数、有限数学/FP/team和生命周期门，不能拷贝本收据授权运行。旧PASS只描述当时原工作区。

本次只导出performance/core。Gazebo/模型/控制/导航源码由主任务单独整理。这里未执行commit/push、未删除任何原数据；不代表Sim2Sim、完整46或真机通过。
