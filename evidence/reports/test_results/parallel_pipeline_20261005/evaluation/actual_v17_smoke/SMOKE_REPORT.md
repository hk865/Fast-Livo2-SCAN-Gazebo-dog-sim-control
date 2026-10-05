# V17 60s输入流水线 smoke：严格失败保留

严格FIFO/lifecycle门失败：accepted 31615，delivered/committed 31614，canceled 1，rejected 0。事前门要求零取消，因此不作正常清理豁免。最后已提交sequence31614连续；缺失候选sequence31615没有实际trace，kind/header/receipt/确切close时钟均UNVERIFIED，不能从timer尾缀猜取消包种类。

原common19和publication26单独验收完成。原16项有限前缀运动/来源/唯一执行器/双300ms时效/slew/native安全门和publication新增7项全部通过；两份总状态仍FAILED（60s只完成8/32，停车、完整两坡未验证）。原pipeline receipt里委托native门的UNVERIFIED保持原字节，新的原common/v2只提供独立证据，不回填该旧字段，也不抵消取消失败。

Teacher到60.000s/3001样本、faultnull，native12001条连续200Hz；最小clearance0.275687m，最大roll/pitch0.113975rad，bodycontact/fault0。一次body-relative offlineSE3路线max 0.135379m、RMS 0.027266m、headingmax 0.100481rad、speedMAE 0.040222m/s。绝对原场景中心线注册仍UNVERIFIED；真值不参与导航。

实际932次数学更新；median header dt65ms（header频率不能当50Hz Actor）。SLAM sim age median/p95/max50/65/75ms，original receipt wall age18.949/38.272/64.209ms，原双TTL门未改。FIFOpeak15events/4,211,680accountedbytes，在冻结512/64MiB上限内；单RX/owner TID、全局已提交顺序、整数source header与原receipt、decode/pop/commit原因果和原writer41303/41303、drop0已由独立pipeline收据证明。

运行时RGB/LiDAR完整body位相等未由metadata日志证明，纯语义17case只限有限生产入口；不称完整processFrame重放。本60s无115–118s详细窗口，未收集的数值细节为N/A。kind300的commit/preprocess改义不能直接用旧spin/preprocess耗时比较；main剩余或wall减CPU不能全归通信。一个60s smoke不证明性能收益或完整导航。

来源：pipeline原收据SHA`80efc11ed7c5537c7ef70493f58527db5b8f28ef20cdcfe3bcc2e2dd5df4af20`；[新增原门/严格失败联合覆盖](SMOKE_ORIGINAL_GUARD_AND_STRICT_PIPELINE_COVERAGE.json)。所有原失败与UNVERIFIED收据保留，未修改任何阈值、producer或原数据。Actor仍232维特权+15维已知输入；未验证动态障碍、全传感器Actor、GPU/1m/s和真机。
