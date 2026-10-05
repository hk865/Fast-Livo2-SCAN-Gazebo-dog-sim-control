# V17 修正后的委托异常与 header 拒绝测试

`v17_header_exception_fixture.cpp` 是待编译的测试草案，调用实际 `LIVMapper::receive_image`、`commit_ingress`、`img_cbk` 和 `getImageFromMsg`，没有复制 header 门。它尚未构建/执行，不能写 PASS。编译/运行必须等 root 确认纯性能窗口结束，并且候选 header 与新候选实际库匹配；不可将 V17 新类布局链接到旧 V12 库。

每个 case 单独进程，给 frozen navigation/camera 两个 ROS `--params-file`；只构造 LIVMapper，不调用订阅初始化、start/run/spin，也不启动 Gazebo/模型。指定当前 evaluation 内独立输出和 private ROS domain，避免向其它任务 topic 提交任何数据。生产 remote camera 参数 loader 可能有等待，整个 fixture 耗时不能作为 decoder 性能基准。

| case | decoder | 原 callback 预期 | direct 与 queued 应一致的状态 |
|---|---|---|---|
| duplicate_bad | 已确认编码非法抛出 | exact duplicate guard 返回 | 无异常，无 buffer/last 改动 |
| near_duplicate_bad | 非法抛出 | <1 ms guard 返回 | 无异常，无 buffer/last 改动 |
| backward_bad | 非法抛出 | 回退 guard 返回 | 无异常，无 buffer/last 改动 |
| under_20ms_bad | 非法抛出 | <20 ms guard 返回 | 无异常，无 buffer/last 改动 |
| fresh_bad | 非法抛出 | 原 decode 位置抛出 | 原异常类型/文字，buffer/last 无提交 |
| fresh_valid | 实际 bgr8 clone | 正常插入 | 2×2 原像素逐字节，last timestamp 一致，改变原 msg 后 clone 不变 |
| fresh_empty | 原 decoder 返回空 Mat | 原 callback 插入空 Mat | 保留这个原始行为，不在新队列擅自改变算法 |

queued 测试由独立 receiver thread 调用实际 receive method，主线程调用实际 commit；检查 accepted/delivered/committed 与失败位置，异常后 committing_ingress 指针清除。direct 测试调用原 callback 分支；fresh_bad 中原代码抛出时 mutex 仍锁定，fixture 仅为正常析构显式释放，不改生产的原异常行为。现有该错误传播会导致实际 estimator 退出，导航 watchdog/原门必须保持。

这个 fixture 只覆盖 decoder 异常延后和 header skip 语义，不能代替所有 source decoded 数据 bits、完整 callback 序列/timer/Estimator 结果或实际 FIFO 时效验收。cloud 的 decoder bit 比较应另以正确 PointCloud2 fields/stride/payload 的同输入驱动 V12 Preprocess 和 V17 receive，再对 points[x/y/z/intensity/curvature/normals]逐字节；如需测试 cloud exception 的原位置，使用明确 exception packet，避免故意构造可能触发 PCL 未定义内存读取的坏 raw buffer。

新观测记录应将 `receipt→decoded`、`decoded→pop`（包含 push/排队短段）、`pop→commit_end` 分别命名；没有 enqueue 精确时钟时不能进一步精确区分 push 与 FIFO。kind300 stage8 是 owner commit，stage12 为 active pipeline 模式下 N/A，不能用它的接近零来宣称 preprocess 加速。主线程与 decoder thread CPU 各自测量，不直接把所有剩余时间归为 communication。
