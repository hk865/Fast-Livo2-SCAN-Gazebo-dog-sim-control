# 现有记录能够复放什么

已导出 `real_lio_rows_v12_format16/manifest.json`，SHA256 `f559d510eb0e12e23ea3fb555cf356b4fba9a4479a2be428dfbe9ba03e35c229`。输入取自冻结 V12 210 s run 的实际 115–118 s 详细窗，按事前确定的原时间第一、最少、最多 constraints 选择 6196、6068、6339 行。保持 kind101 原行序和原 float64 字节，精确关联同 sequence/stamp/stage/iteration 的 kind100，使用实际导航配置 extrinsic_R/T。不是通用虚构数据，也没有用 Gazebo 真值修正数据。

NPZ 最小接口为 `rows[N,107]`、`prop_state[25]`、`before_state[25]`、`before_cov[19,19]`、`extR[3,3]`、`extT[3]`。另存原记录的 expected H/R_inv/meas/sigma/body-world covariance/HTH/HTz，以及整份 kind100/101 供独立对照。native binary magic 精确为 16 字节 `FLIVOJACROW0001\0`，之后 uint64 N 和 row-major float64 的 extR9/extT3/prop25/before25/cov361/rowsNx107。初版 header 长度错误的产物保留在另一个目录，禁止消费，见 `EXPORT_FORMAT_CORRECTION.md`。

V15 应调用候选实际编译的 `BuildJacobianRows`；V12 的原行循环保留逐字节参考。对照 ordered H、HTR、R_inv、meas、sigma/body-world 与原 Eigen 最后 HTH/HTz/更新时，使用相同 FP/compiler/core 类型。可以检查这一组真实约束对并行行构造的数值保持和有限耗时；它没有重新建立 voxel map，也没有重新进行所有 residual matching，因此不能称为完整 estimator replay。详细 kind101 只记录首/末迭代，不虚称中间每次 H 都完整。

VIO kinds201/203 保存已构造的 H、z 和 Hessian/HTz/更新、accept/rollback 决策，可校核同一矩阵的 solver algebra。kind202 只含部分 track/reference 元数据，缺少全部原图像字节、reference patch/warp 深度/visibility 和起始 visual submap；因此不能用这些记录重建原 updateState 全部像素 Jacobian，更不能声称完整 processFrame replay。V16 的生产组件 synthetic fixtures 必须单独标明这一有限范围。

现有 kind1/10/11/20 支持所观察范围内 IMU 离散传播步与状态/协方差核对；kind4 只在详细窗保存实际预处理 LiDAR 点。kind14 的有限 map-write 不是完整起始 voxel checkpoint，kind103 也不是全图索引的可恢复序列。不能从这些缺失数据推算完整 210 s 或全路线确定性 estimator 输入。

如果后续需要完整 estimator replay，最小新增接口可以二选一：

1. 从启动起不可变地保存所有原 PointCloud2 字节/fields/ring/逐点时间、Image 编码/step/全部 pixels、IMU exact 原时间戳及原 ROS callback dispatch 序号、原 timer 事件，并按原单 owner commit 序列送入实际 mapper；原初始化和全部配置必须同源。离线不启动 Gazebo，native 真值不参与估计。
2. 在明确窗口起点序列化可恢复的 IMU buffer/propagated state/covariance、完整 voxel/octree map、VIO reference/submap/图像金字塔/warp cache 和实际后续原输入事件；确认 restore 完整性后再 replay 该窗口。仅保存状态 25/协方差 361 不能替代地图和视觉历史。

V17 先采用单接收/解码 executor 到有界 FIFO、单 owner commit；最小 pure 验收必须包括接收序号、原 receipt、decode 完成、enqueue/dequeue/commit、timer、queue watermark/overflow、owner thread identity。对照同 raw bytes 的 decoded 数值与 V12，检查原全局 callback 顺序和保护门；队列排等待时间可以与 decode/compute 分开观察，但没有 DDS source transmit 边界时不能称为完整网络通信时延。
