# 清理容量档：建议T2约30.75GB，尚未删除

本计划完整分类原284项大文件（56.836GB），并合入旧5项及整理复验10新增2项，合计291个文件。原284项均有保留/排除/候选结论；没有把首批C001冒充全部方案。清理候选全部按精确文件列出，未列出路径一律保留，不整目录删。

推荐T2，在保留formal22成功整轮、本次11失败整轮、当前停车核查/新0.1直走零速测试全部数据与依赖、代码/模型/配置/当前3313项scope引用的前提下，删所列历史逐帧大流。**报告可读，不等于原始流可重放。** 所有档当前delete_authorized=false、execution_ready=false、实际删除0字节。容量档是嵌套选择，不相加。

|档|精确文件数|条件满足时预计回收分配字节|主要损失/保留|
|---|---:|---:|---|
|T1|110|20,313,190,400（20.313GB）|较早已报告阶段大流+失败partial；保留后期和代表性对照原始流。|
|T2|170|30,750,932,992（30.751GB）|再加后期已报告阶段及已解决打包复验10的2个大流；仍保留B05/B10/D11/P12代表性原始流约5.096GB。建议此档。|
|T3|260|50,907,336,704（50.907GB）|再删代表性对照和早期原始大流；旧阶段完整逐帧/native重放损失更广，部分旧改动/候选SHA仍UNKNOWN。|
|T4|276|56,195,870,720（56.196GB）|再删所列唯一压缩恢复载体、派生表和视频；有些原件早已按旧授权删除，删除载体后不再保留其原始字节恢复能力。最高损失，不默认推荐。|

## 真正空间与当前检查

候选逐个全量SHA256、前后device/inode/size/mtime/nlink稳定，均nlink1非符号普通文件，已去重inode。预计值是当前st_blocks×512，未计目录块；无实际unlink/df前后差值，当前保证回收0字节，不能把逻辑大小当实际已释放。已知同UID161进程/6421FD未见候选持有；6个PID不可读，未绕过，且未排除不同UID、共享extent/快照、未来打开。任何获批执行都须即时再核身份、活动持有、依赖和文件系统；有冲突则跳过并报告，不能扩大清单。

保留/排除：原formal22的8项大流1.377GB以及该整轮其他全部文件；新11整轮和新直走零速测试的全部原始数据；7项共0.601GB的包/维护审计/manifest/未决依赖。Cupy wheel和历史zip也未当作“必可重建”随意删除。本批没有经过完整来源重建证明的构建产物删除组，构建/SDK/模型不动。其余所有未列文件，包括Actor npz、源码快照、profile、world、known_scene_reference、证书、结果/索引和小日志均保留。

## 来源、轮次和上传映射

- [run_reports.json](run_reports.json)复用原history.json的goal/change/progress/confirmed_issue/followup/implementation/deployment及现存小结果SHA，提供本批52个run的对应关系。已有38个主记录，另补打包10；旧轮缺少可证变更或后续因果时明确UNKNOWN，不从目录名编造。T3的这些更老原始流损失由用户单独决定。
- 原历史章节02–06及history.json/evidence/receipts.json已上传；本附录补run_reports、逐文件清单及archive/derived来源/损失。原formal22结果在main的releases/formal22/evidence，11结果在reproduction/results/published_repro_11。报告只保留结论、小结果和SHA，**没有上传actuator/telemetry/records.bin全量内容**。
- actuator删除会丢native关节/力矩/接触全序列；telemetry删除会丢命令/实速等全时序；native records删除会丢原始source/solver/timing包；PID/command/status/guard/dynamic流删除会丢逐步控制/保护/事件关联复算所需值。Actor小归档仍保留也不能重建这些完整流。
- [artifact_sources.json](artifact_sources.json)列压缩carrier、派生表、视频原路径及相邻报告SHA；未证明可完整再生，一律按不可恢复损失而非“安全缓存”报价。旧archive索引/restore收据继续保留，但删除gzip本体后不能靠索引恢复。

## 硬保护与复验保留

[hard_protection.json](hard_protection.json)列原formal22成功/新11失败整轮和3313个当前输入路径；未来新测试/新目录均不在本次精确候选中。整理准备或失败复验不是自动废件：代码/配置/错误/身份/加载/退出/几何失败证据保留，只有10标签2个大流在T2选列，损失完整时序已说明。T1/T2保留B05/B10/D11/P12的所有本批大流，T3以上才让用户选择放弃这些旧对照的逐帧重放。

本机精确批准文件使用真实路径、inode、mtime、SHA；公开版本仅只读路径投影，不可用来执行删除。容量数字和内容SHA一致。旧C001单文件建议已被本次整体容量计划取代，不再零碎请求确认。

本地精确批准清单SHA和每档文件数见[tier_index.json](tier_index.json)；公开文件路径是只读投影。
