# 同包1.5x释放：容量单次探测

本轮是同一60s输入包的两个新进程回放。没有Gazebo/Teacher/执行器，也没有把速度倍率当机器人速度。采用同一最终V19库、copy0、P核mask及配置；原source stamp保持，native wall timer和DDS跨topic排序仍不同。

**全输入头/严格停机生命周期两者通过；运行中的位姿时效明显不同。** 该差异仅为每模式一次的有限容量实测，未满足32独立批次统计门，也不是完整固定全序前端数学回放。

|模式|提交/取消/拒绝|peak pending|位姿数|实际wall输出Hz|source header Hz|观察age median/p95/max(ms)|StateEst wall/CPU(ms)|
|---|---|---:|---:|---:|---:|---|---|
|serial|21737/0/0|1|1709|42.871|30.305|40.00/1891.00/2535.00|11.213/9.670|
|staged|26807/0/0|18|1709|45.113|30.305|35.00/45.00/50.00|9.250/8.127|

串行最终排完1709个位姿，但发生秒级观察端source lag，不能称1.5x实时稳定通过。流水线本轮观察age在15–50ms且全部排完，支持该输入负载下保留单owner、移动解码/RX能增加容量余量。没有外推到2x/3x、所有地形或整个实际控制链。

source header Hz按原acquisitionstamp仍30.305；实际wall输出约42.871/45.113Hz与1.5x释放速度分别列出。串行peak pending=1只统计inline charged-slot，不涵盖DDS深队列，所以不能说串行没有积压。stagedpeak18/20,656,153B为包括Raw/Decoding/Ready/Owner的reservations，未代表全RSS。

观察age=observer最后收到clock−poseheader，包含观察端DDS调度；它提供端到端时效证据，不能单独定位为通信/算法哪个独占原因。同包而不同全序会改变同步切片/地图/残差迭代，因此StateEst耗时差不可严格全部归因于流水线。query⊂StateEst⊂handleLIO，不能相加；processCPU含并发worker/logger，wall−CPU不作通信归因。

原300ms控制保护未放宽，本回放没有控制器。随后实际短闭环和原46任务需新验收，旧V18通过不替代V19。所有本次自有ROS进程均退出0且无残余；无新消息取消、容量拒绝或closed rejection。

来源：[CAPACITY_1P5X_COMPARISON.json](CAPACITY_1P5X_COMPARISON.json)，两轮 `replays/C1_serial_1p5x/`、`replays/D1_staged_1p5x/` 的原CSV/位姿和绑定收据。此JSON保存每5s source bin的age分布，1x四轮另见[REPORT.md](REPORT.md)。
