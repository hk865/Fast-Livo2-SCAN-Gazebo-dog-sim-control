# formal22 本候选实际效果

实际task_result：350.930SIM `passed/parking_pass/dynamic_pass/full_F3_PASS=true`，`original46_PASS=false`。本报告只核本候选触发和有限反事实，不替父任务完整closure/标准声明。worker3643479实际startup geometry SHA `73804fb93fc450bfcf44b1cb0086bddda97bb5587c6b66e23187027427afd3b2` 吻合最终源码。R88 received=consumed563、submitted=completed183、fault/invalidation/discard均0。

|证书业务人口|唯一证书|使用新依据|重分类patch|
|---|---:|---:|---:|
|顶层SCAN候选曲线|165|29|33|
|嵌套旧路径保留connector|14|12|12|
|全部连续物理证书|179|41|45|

去重键为业务类别+schema+payload SHA+T_navigation_world+完整sources绑定。全部179次occurrence均唯一。顶层SCAN165中143物理true且最终admitted，18物理true但外部门未admit，4物理false。新依据全量last-face分布floor_1=7/ramp_12=14/floor_2=5/ramp_23=15；每证只保存最后一个触发patch，不能假称这是45patch逐面完整分布。SCAN触发分布epoch2/gen1=22、epoch3/gen31=7，first source48.250、last322.345；末段floor3未观察新依据使用。

父此前110/26是较早递归prefix，不能与final165/29顶层口径混用：目标payload481742…在第111行old_path_retention.r85_physical_retention.connector_physical_certificate，traj_id=None/source225.985/lastfacefloor_2。是同一连续certifier的retention connector证书，应单列，不是无关重复或改SHA。

仅对末两条实际SCAN触发payload回放58bdc24原整certifier（共2调用，没有全量回放）：

|epoch3/gen31轨迹/source|旧整证书|旧patch/depth|记录新patch/depth|真实新cert wall|真实worker whole wall/CPU|离线旧调用wall/CPU|
|---|---|---|---|---|---|---|
|27 /317.230|true|10/2|6/0|25.488ms|25.547/25.119ms|102.250/95.369ms|
|30 /322.345|true|8/1|6/0|24.810ms|24.881/24.881ms|30.388/30.389ms|

**两条均不能证明旧整证书拒绝→新证书通过。**旧coarse cone失败可以由原细分继续证明；这里只证明新依据改变了证明路径并减少这两例细分。离线旧调用和真实新worker不同窗口、冷/热与计时范围不同（首旧调用含首次SciPy导入），不据此计算净加速或控制收益；原workerCPU是整个compute，未单存几何函数CPU。

41次实际分支触发不是41次避免旧whole拒绝。21两条真实拒绝可被新代码修复的旧证据保留，但22不等于重放21；22最终PASS不能仅凭本报告唯一归因于本候选。新terrain/body语义不是旧导数锥安全等价，静态证不是动力学PASS。未改源/旧21/原25或vertical增量证据，无Node/仿真/额外suite。
