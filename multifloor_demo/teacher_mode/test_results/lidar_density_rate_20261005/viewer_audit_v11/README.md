# V11 只读结果页契约扩展

仅修改 Teacher 的 `scripts/serve.py`；未修改 camera_mode、原 web 文件、控制器、仿真或任何原收据，未重启服务。

新增固定 schema `independent_actual_SLAM_SCAN_publication_ledger_navigation/v2`，固定判据 SHA256 `756ad4c3f78a68f42483c9d016a7caa49e2bcbfd22f41595de4ac1675acb480c`。保留 V9/V10 v1 的固定 SHA256 `9c2b5f2ff6a8f6525df662411ca51976ee3aa6196661f3b774b8a30bf4259f65` 与六个额外检查。V2 必须有原六项及第七项 `actual_control_publication_ledger_complete_and_original`，所有原共同检查（仅显式允许 PI bookkeeping 替换）保持原 payload。共同数值门、父判据、固定 V11 / V12 候选来源（七个执行源字节必须保持同一756ad协议）、七个执行前 producer/helper/evaluator 源的实际归档字节都核对。

任何外来 run、未知 schema、改判据、改来源、缺账本检查、nonpassing 检查、失败运行阶段或 21/32 等部分到达伪称 full PASS 都被拒绝。联合收据未完成时仍为未验证，不回退到旧共同 PASS。实际 V9 2a1a 的有效 FAILED 保持 22 通过 / 1 失败 / 2 未验证；其旧收据字节不变。

来源 SHA 缓存只在 `(dev,inode,size,mtime_ns,ctime_ns)` 不变时复用，读取前后复核 stat，改内容同大小/恢复 mtime 或替换 inode 均重新计算，缓存上限 256 个文件。缓存不替代独立数值回放，也不把 UI 通过当运动通过。

验证：原 v1 16 个边界测试、新 v2 26 个边界测试、外部存储 10 个边界测试、Python 内存编译与新增 JS Node 语法检查通过。测试均为合成显示完整性 fixture，不认证实际导航。实际 V11 4d77 的 v2 收据显示完整性有效且保持 FAILED：20 通过 / 4 失败 / 2 未验证，未更改原收据。浏览器核对由 root 重启服务后执行。

证据：`before_manifest.json`、`serve.before.py`、`serve.after.py`、`serve.diff`、`validation.json` 与测试日志。103 个保护引用中不包含此次修改文件；未对大运行日志重新整哈希。

## 受控外部运行目录

只允许 `/var/tmp/go2_teacher_simulation_20261005`：root 必须属于当前 UID、0700、非 symlink；run 子目录属于当前 UID、非 symlink，并符合时间戳+`closed_loop_cascade_`+四位 run id 的名称。项目 `runs/` 必须有同 basename 的准确 symlink；目录内容读取和收据身份始终使用真实 canonical path。任意外部目录、root 下非 run 文件、错误 basename alias、缺精确 alias、向外逃逸的 JSON/telemetry/media symlink 均拒绝。

固定756ad源契约仅允许 `lidar_sampling_v11` 与 `lidar_sampling_v12` 两个候选；七个 producer/helper/evaluator basename 必须唯一，来自正确候选目录，并匹配固定预先冻结 SHA。全部归档路径必须位于本 run/sources；V12 C++ 算法库区别继续由原 common frozen-scope/loaded-binary 门核验，不能靠 viewer source pin 宣称实际完整链通过。
