# V20 续测与旧数据清理：2026-10-06 下午

本轮在原机继续冻结 V20 实际仿真。原始前九区和首次固定五秒停车独立有限验收通过；随后实际尝试原完整46区，在第十区途中超过原路线边界而停车，不能宣称46区完成。按用户要求清理旧实验 raw 与过期 pip 缓存共 **77.32 GiB**，两次新运行全部原始证据保留。

## 实际运行结果

| 运行 | 结果 | 证据边界 |
|---|---|---|
| `133944…r2_d643` | 启动前磁盘余量保护拒绝 | 尚未启动机器人；当时不足原150 GiB记录余量。之后清理过期pip缓存，未降低余量门。 |
| `134714…r3_401b` | 原始9/9区域与首次固定5秒停车通过 | 真实SLAM到达、原几何/dwell、输入与唯一执行器及50 Hz物理快照独立核对；不覆盖全部200 Hz物理步骤。 |
| `140607…r1_dc50` | 完整46任务失败，到达9区 | 第十区途中SLAM首次偏离0.45012 m超过0.45 m边界后停车；末帧偏离0.46297 m。程序正常退出不是导航通过。 |

前九区使用 `exporter_shadow_original46_prefix9`，完整任务使用既有冻结 `pipeline_staged_original46`。两者使用相同V20有限弧长参考，但前者SCAN含只读快照导出器，后者使用保护的基线SCAN；不是完全相同栈的单变量A/B。原路线、到达区域、0.4秒dwell、90秒区域期限、300 ms新鲜度、唯一执行器均未放宽。

### 前九区的独立证据

[独立报告](20261006_134714_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r3_401b_PREFIX9_ACTUAL_EVALUATION.json)的全部有限检查通过。首次停车区间为181.315–186.315仿真秒：SLAM最大XY漂移 **6.55 mm**，yaw漂移0.00366 rad，原生平面速度最大0.01967 m/s。9424个原生50 Hz快照无Actor fault、机身接触或采样安全违反。

[第九区修订后的诊断对照](20261006_134714_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r3_401b_REGION9_COMPARISON_REVIEWED.json)使用实际区域激活journal，激活150.320秒，到达173.945秒，共 **23.625秒**；到达前 `drive→pre_turn` 复位0次。旧V19对应区域90.165秒超时，104次复位均伴随新路径。新运行到达后174.005秒进入终点停车时的一次复位单列，不能混入到达前计数。两次终点任务不同，不能用该对照断言严格因果关系。

旧未修订诊断文件保留原字节；它从带有前一区残留时钟的首个状态取激活时间，只影响诊断统计，原九区和停车验收未改。[压缩曲线](20261006_134714_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r3_401b_COMPACT_SERIES.json.gz)及[图表](viewer/V20_R3_PREFIX9_COMPARISON.png)可独立查看。

### 走廊与轨迹选择边界

本轮仍只运行shadow观测，没有启用旧路径保留或走廊控制。447份返回收据中认证0、unavailable 303、blocked 144；这些统计来自收据读取，不是完整纯函数重放。返回时pose源龄343/447超过300 ms；已知cloud时间的383项中372项超过300 ms。原始语义未改，unknown不会当作free，blocked也不等于发生碰撞。见[走廊收据审计](V20_R3_401B_CORRIDOR_RECEIPTS.json)。

[保留旧路径的安全审计](PLAN_RETENTION_SAFETY_AUDIT.md)发现：现有SCAN碰撞检查跟随其最新 `local_data_`，如果Tracker拒绝新候选而继续旧路径，SCAN不会自动继续认证那条正在执行的旧路径。下一步旧路径保留必须同时实现实际执行路径的独立监视、版本绑定、失效通知和新鲜有效期；不能直接打开 `retain_valid_plan`。

另对旧39ea的99/187/316三个快照做了单次离线profile，支撑凸包是主要耗时。局部候选重用同次调用内不可变数据并避免重复标量转换，三个完整返回字典完全一致，166个凸包边界样例一致。带profile的wall约421–475 ms降至288–319 ms；**仅三例且profile影响相对开销，不代表在线速度、满足300 ms或导航通过**。候选没有接入本轮控制。[离线报告](corridor_optimization/REPORT.json)。

## 清理与可复现范围

[清理报告](cleanup/README.md)及[精确总计](cleanup/CLEANUP_TOTAL.json)：旧raw 69,845文件、73,561,572,507字节（68.51 GiB）；pip过期HTTP缓存1,310文件、9,464,946,102字节（8.81 GiB）。合计77.32 GiB，与更早351.520 GiB批次分开计数。只删除逐文件计划中的旧数据，未修改RL训练、冻结模型、camera_mode或控制代码。

删除后两批目标均复核不存在，第二批4740个保留文件仍在。旧V19 9779保留PID/SLAM/status、区域激活和1931份样条及metadata；旧39ea保留全部400个走廊快照与三条收据流。旧原生/点云/图像完整重放不再可用。保留SHA只证明身份，不替代已删除内容。

401b与dc50的原始目录均保留在 `/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/`。远程分支只提交源码、精简曲线、报告和清理收据，不上传这些raw、模型或build/install。当前源码能用于准备新实验；原机含绝对路径和二进制SHA的gate不能作为clone启动授权。

## 模型、配置与操作

冻结Teacher SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。CPU单线程、50 Hz推理，原生关节PD 200 Hz。未训练新模型，也未操作真机。导航用真实SLAM/IMU/点云/SCAN，仿真真值仅离线诊断；Actor仍有232维特权输入和15维命令/上一动作输入。

- 401b runtime manifest：`1b0e452cb85b9c4a661793f1b20a8c523629abbfd00f3e4af640e32c89e88594`。
- 401b独立报告：`dc4894e4d19ca1eeaa61c0e2cec48d6b8880cef72b47d3a3df961db2e534cf13`。
- dc50 runtime manifest：`1bd8b6b884286c881df27d044cb3417aa3f28821852fb949d8eea501ca7e30bc`。
- dc50 profile：`0a70f3c6842cb56ead6f5fc9c2106d92fa08219a61297c889291fe190c4d7c25`。

事前判据分别保存在 `PROSPECTIVE_REPEAT_CRITERIA.json` 和 `PROSPECTIVE_FULL46_CRITERIA.json`，保留事前NOT_RUN状态；后验结果另立文件，不回填事前记录。

原机本轮实际使用的命令（完整任务未通过，不应当作推荐默认导航）：

```bash
python3 -B multifloor_demo/teacher_mode/navigation/corridor_tracking_v20/run.py \
  --profile exporter_shadow_original46_prefix9 --label v20_exporter_prefix9_r3 \
  --domain 89 --run-storage-root /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs
python3 -B multifloor_demo/teacher_mode/navigation/corridor_tracking_v20/run.py \
  --profile pipeline_staged_original46 --label v20_original46_r1 \
  --domain 89 --run-storage-root /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs
```

[九区真实画面与结果](http://127.0.0.1:8768/?run=20261006_134714_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r3_401b)、[完整任务失败回放](http://127.0.0.1:8768/?run=20261006_140607_closed_loop_cascade_clock_hold_v20_original46_r1_dc50)。界面显示归档画面，不声称当前正在仿真。前九区面板核对本run的294份源字节，独立显示有限通过；全46验收单列。

接口和已验基础运动通过；本轮前九区导航有限通过；完整46区失败，主动走廊控制未验证；新控制器完整跨引擎Sim2Sim未通过；真机未验证。三层连接始终是坡道。

## dc50 完整任务独立复核补充

[正式 full46 报告](20261006_140607_closed_loop_cascade_clock_hold_v20_original46_r1_dc50_FULL46_ACTUAL_EVALUATION.json)为 **FAILED**：10 项通过、4 项失败、8 项未验证；探索到达 9/18，总体 9/46。首次路线保护对应 SLAM header 187.575 s，横向误差 0.4501175 m；最近 native 187.565 s 的机身原点实际横向偏离为 0.4717252 m。末帧 SLAM 的约 0.463 m 是停止后的最终值，不是首次触发值或全程峰值。详见[简短失败报告](DC50_FAILURE_REPORT.md)及[数值诊断](DC50_REGION10_FAILURE_DIAGNOSIS.json)。

原接受 path 257 的 121 个原采样点中，29 个超实验路线 fence，最大 0.481395 m；这不是 costmap 碰撞证明。6 次记录中的 drive 退出都伴随同记录新路径，但关联不构成独立因果证明。本轮没有重放控制器、启用走廊控制或修改路线门。

实际复评使用 [audit_dc50_once.py](audit_dc50_once.py)，而非单独运行 adapter：[provenance](DC50_AUDIT_PROVENANCE.json)绑定最终报告、包装器 SHA 和完整 V20 归档路径选择。首次 reader 同名源查询中断及必要重读另立记录，未隐藏。包装器 `--run` 必须指向已结束的 run；现有同名输出拒绝覆盖，未来新 run 单独生成独立证据。导出使用 [170–190 s 精简数值](DC50_REGION10_NUMERIC_WINDOW.json.gz)（400,913 字节）；25 MB 完整 compact 留本机，不需要再次扫描大 raw。
