# V17 efde 实际60秒管线验收

**严格全生命周期 FAILED。** 原最终summary为 accepted 31615、delivered＝committed＝31614、canceled 1、rejected 0、failure 空。事前标准要求取消为0；没有套用清理豁免，没有修改运行源码、门限或实际日志。

实际 run：`/var/tmp/go2_teacher_parallel_20261005/20261005_234627_closed_loop_cascade_clock_hold_ingress_v17_smoke_r1_efde`。正式收据为上级 `V17_efde_RUNTIME_RECEIPT.json`；本目录 `CANCEL_AND_MEASURED_RUNTIME.json` 追加末尾证据与测量局限。事前reader准备清单、两次有限测试收据均原样保留。

运行期间的FIFO序号连续1..31614；RX TID1280635与owner1280423各唯一且不同，owner与实际加载mapping进程绑定。所有已提交 IMU12002、LiDAR1819、RGB1819 按原 RAW 全序精确匹配整数stamp和原接收receipt。ready峰值15包／4211680字节，低于512包／64MiB；writer最终41303条全部写入，dropped0、IO失败false。完整来源快照、实际库、配置与环境绑定通过。完整RGB/点云正文并未由RAW元数据逐字节重建，因此只声明原header/receipt/order的实际匹配。

| 输入 | source header实测Hz | 接收→提交 wall p50 / p95 / max（ms） | RX decode CPU p50 / p95（ms） |
|---|---:|---:|---:|
| IMU | 200.000 | 1.209 / 15.585 / 24.141 | 0.000319 / 0.000556 |
| LiDAR | 30.308 | 0.564 / 0.738 / 6.851 | 0.416 / 0.510 |
| RGB | 30.308 | 0.444 / 4.109 / 14.180 | 0.170 / 0.280 |

LIO成功1709次、初始化/跳过109次，成功更新壁钟30.034Hz；VIO成功1708次、跳过109次，成功更新壁钟30.030Hz。频率分母使用成功记录首尾时间，不能将初始化跳过记成有效更新。名义采样为30Hz；表中源header频率由实际整数stamp计算，没有把名义值填作测量值。

LIO开始时，最新因果已接收IMU header领先处理目标的p50／p95／最大值为15／20／25ms；LiDAR和RGB对应差为0。VIO开始时IMU为45／50／55ms，LiDAR和RGB为35／35／35ms。这些量是已接收源头与当前处理目标的相对差，并非当前ROS `/clock` age；不代替控制器原300ms双时钟保护。

末尾可确认：最后传感器为seq31524 IMU，source60.010s、commit_end458416.740019303；其后90条已提交事件均为timer。最后seq31614 timer在wall458417.098512574 pop时队列pending0／bytes0，commit_end458417.098516158。源码在退出时先cancel/join RX，再queue.close，close把当时pending记入canceled。可确认少提交的一条位于close时残余队列，但它没有CSV记录，**类型、source stamp、接收墙钟、精确close墙钟仍未验证**；不把它推定为timer，也不认为它因此可豁免。

kind300 stage8测量owner commit batch（含大量空轮询），stage9/11分别不含已移到RX的点云预处理/图像解码；stage12为N/A，不能称预处理免费。嵌套计时不可相加，process CPU含同时工作的RX／writer，wall减CPU不能直接归为IPC。与完整旧算法单轮运行没有匹配A/B，不能仅凭本次60秒更新频率宣称流水线加速量。

本reader把原native运动、发布账本与300ms保护委托给同run原验收，保留该项UNVERIFIED；60秒不声明32区域、完整坡道、最终停车或导航通过。全生命周期取消1条已使管线门失败，运行期间的低延迟事实单列保留。
