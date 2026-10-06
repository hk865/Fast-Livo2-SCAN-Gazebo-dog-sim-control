# 2026-10-06 两批旧 Teacher raw 清理与复核

本目录记录用户授权后由主任务执行的两批清理。第一批是旧 `83e9`、`9eaf`、`4fc5`，第二批是旧 `9779`、`39ea` 的指定 raw。源码、配置、报告、有限数值夹具和明确保留的分析输入不在删除范围。删除由主任务完成；独立复核工具没有删除操作。

| 批次 | 范围 | 数量 | 字节 | GiB |
|---|---|---:|---:|---:|
| 第一批旧 raw | `83e9/9eaf/4fc5` 的点云、完整图像、原生流、诊断和旧 bag 等 | 33,443 文件 | 33,460,466,802 逻辑字节 | 约31.16 |
| 第二批旧 raw | `9779/39ea` 的大块原生流、逐帧云、诊断二进制和图像归档 | 36,402 文件 | 40,101,105,705 逻辑字节 | 约37.35 |
| 两批旧 raw 合计 | 以上两批，精确逐文件清单互不重叠 | 69,845 文件 | 73,561,572,507 逻辑字节 | 约68.51 |
| pip HTTP 缓存 | 超过24小时的下载缓存；不含安装包、模型和实验数据 | 1,310 文件 | 9,464,946,102 字节 | 约8.81 |
| 本轮 raw 与缓存合计 | 不含更早的351.520 GiB批次 | 71,155 文件 | 83,026,518,609 字节 | 约77.32 |

以上 GiB 由确切逻辑字节除以2³⁰计算；分配空间和可用空间变化会受块大小及并发写入影响。第一批事前实际分配为33,530,408,960字节，第二批为40,170,799,104字节。这些数值不能与执行收据的逻辑字节混用。汇总原件是 [CLEANUP_TOTAL.json](CLEANUP_TOTAL.json)。

`FINAL_DELETE_CANDIDATES.jsonl` 是主任务实际执行的精确计划；`PURGE_RECEIPT.json` 与 `PURGE_EXECUTION_JOURNAL.jsonl.gz` 是执行收据。`PRIORITY_4FC5_RAW.jsonl` 是完整计划的子集，已经随完整计划执行，不能重复使用。最初 `INVENTORY_SUMMARY.json` 将普通目录上下文过度当成依赖而给出0文件，保留为历史草稿；其后 `FINAL_INVENTORY_SUMMARY.json` 依实际预检文件绑定语义修正，不覆盖旧记录。

独立 [CHECK.json](CHECK.json) 记录第一批后的时点：全部33,443目标不存在，335个保留路径在，211个当前gate小文件SHA相同，仅读约4.7 MB。第二批由 `PHASE2_DELETE_CANDIDATES.jsonl`、`PHASE2_PURGE_RECEIPT.json` 和压缩执行journal记录；[PHASE2_CHECK_AFTER.json](PHASE2_CHECK_AFTER.json) 核对全部36,402目标不存在、4,740个完整保留清单文件存在且大小相同，受保护子目录在。旧收据保持原字节，不更新其历史时点结论。

主任务的 `POST_PURGE_GATE_VERIFIED.json`、`PHASE2_POST_PURGE_GATE_VERIFIED.json` 分别记录两批删除后 exact frozen full46 源码/输入gate通过；该预检不代表导航通过。第二批后，`9779` 的PID/SLAM/status三主日志、segment activation、全1931份样条NPZ和1931份metadata、配置/源快照保留；`39ea` 的400份corridor快照、输入/证书/worker三日志、源快照和全部摘要保留，涵盖99/187/316三份代表原件。它们的小数据对照与corridor原输入重放仍可开展。

第一批复核曾观察 `9779/39ea` 的原生telemetry/actuator等存在；第二批随后依授权删除这些指定大块raw。因此不能将历史 `CHECK.json` 误读为当前它们仍存在。当前原9区 `401b` 与已结束的 `dc50` 的PID、SLAM、状态、telemetry、actuator、云目录和源快照均已做最后只读存在性复核并保留；没有读取其大raw。`camera_mode`、RL训练、Teacher模型、控制源码和所有其他运行不在删除范围。

pip 缓存执行收据在 [PIP_HTTP_CACHE_CLEANUP.json](../PIP_HTTP_CACHE_CLEANUP.json)。此收据记录1310项删除；独立复核没有重建不存在的缓存逐文件清单，也没有读取或修改已安装软件。

被删五次旧运行的完整原生/点云/图像/诊断回放与 `83e9` 原bag重放已不可用。`39ea` 的corridor原输入仍完整，但其单独原生运动验收不能再从已删telemetry/actuator复做。旧manifest、SHA和FAILED/PASS摘要保留当时字节，不证明对应raw仍存在，不能据此重新宣称独立验收完成。新运行须保留自己的原始证据并另行验收。

以上清理已经结束。不要重复执行任何清理计划，不再清理更多数据。重新做只读复核必须另立收据，不能覆盖旧检查或历史报告；`401b/dc50` 全部数据继续保留。
