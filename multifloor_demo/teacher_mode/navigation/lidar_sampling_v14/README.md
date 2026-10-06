# V14：同冻结 V12 四线程库的自有进程 CPU 分组

V14 只新增启动期 CPU affinity 与其观察记录，明确复用 `../lidar_sampling_v12/slam_ws` 中已冻结的源、core 和 mapping 二进制，不另外构建 SLAM 库。LIO 仍四线程无锁残差循环，VIO 显式循环仍串行，Teacher CPU 单线程。core SHA256 为 `8303ec94d33a976bc8ac0940e9d3d213120ce4a035ea22385e07f301c019efed`。

七个控制生产/验收文件与冻结 V12 逐字节相同，继承 v2 criterion `756ad4c3f78a68f42483c9d016a7caa49e2bcbfd22f41595de4ac1675acb480c`。原 300 ms TTL、控制数学、唯一关节执行权、保护零命令 publication ledger、停车保护与所有数值门保持。真实 SLAM/IMU/SCAN 用于导航；Actor 的特权观测限制仍存在，真值只离线验收。

## 自有进程分组

硬件审计得到 P 核 0–7、E 核 8–19，无 SMT。仅本次新建进程通过 taskset 前缀使用以下不重叠分组：

| 进程 | CPU |
|---|---|
| Gazebo 及其子线程 | P0–3 |
| fastlivo_mapping 及其 OpenMP/DDS/logger 子线程 | P4–7 |
| CPU Teacher Actor | E18 |
| 导航 controller | E19 |
| 其它本次 bridge/capture/SCAN/传感器转发/地图归档进程 | E8–17 |

新建的 launch 父进程及普通子节点继承 communications 组；mapping 和 controller 在各自启动时覆盖为明确分组。`cpu_affinity_contract.json` 在首次准备时冻结拓扑和分组。启动期核对根进程 mask，此后每约 2 秒读取 saved-owned-process-groups 的每个 `/proc/<pid>/task/<tid>/status`，保存 `cpu_affinity_witness.jsonl`。活线程 mask 不符时通过现有仅自有进程清理路径失败。

五个新建无 ROS 测试子进程的实际 mask 已通过，测试 runner 的 mask 未改变。这只验证 prefix 接口；实际完整 Gazebo/多线程/路线成败由 run 独立验收。CPU masks 不能预留核心，也不能隔离其它任务、GPU、内存带宽、磁盘或异步 DDS；训练和全局调度设置保持原状态。

## Profile 和操作命令

在 `teacher_mode` 目录运行：

```bash
/usr/bin/python3 -B navigation/lidar_sampling_v14/run.py --profile l64_r30_c30_lockfree4_timing_affinity_detail3s_210 --label v14_affinity_preflight --run-storage-root=/var/tmp/go2_teacher_simulation_20261005 --prepare-only
/usr/bin/python3 -B navigation/lidar_sampling_v14/run.py --profile l64_r30_c30_lockfree4_timing_affinity_detail3s_210 --label v14_affinity_actual --run-storage-root=/var/tmp/go2_teacher_simulation_20261005 --domain 86
/usr/bin/python3 -B navigation/lidar_sampling_v14/run.py --profile l64_r30_c30_lockfree4_affinity_600 --label v14_affinity_full600 --run-storage-root=/var/tmp/go2_teacher_simulation_20261005 --domain 86
```

210 秒 profile 打开 kind300，保持实际 3 秒详细窗口 [115,118]。600 秒 profile 保留原 32 goals、门和相同 3 秒详细窗口，CPU affinity=true、kind300=false。所有启动文件/profile 在首次 prepare 前冻结；121 个 runtime refs 位于 `../../test_results/lidar_density_rate_20261005/owned_affinity/source_freeze_v14.json`。运行中的这些文件不得修改。

显式外部存储使用当前 UID 所有、0700、无 symlink 的固定 root；canonical run 从开始绑定 `/var/tmp` 路径，项目 runs 下仅建唯一 alias。已有 run/alias 不覆盖。私有 domain 被占用时拒绝启动。

## 完整通过的要求

所有 32 个区域必须按冻结顺序达成，真实 SCAN 路线和实际 SLAM/IMU 原始时间戳必须及时，指令发布链需完整，实际速度/方向/高度/姿态/接触/关节限幅/停车和 evidence writer 必须满足原 criterion。prefix 测试、prepare-only、前缀区域通过或运行无 fault 都不代表完整多层导航通过。三层由坡道相连；真实楼梯和真机部署未验证。V13 同 dt 变换缓存只是独立源码候选，未加入 V14。
