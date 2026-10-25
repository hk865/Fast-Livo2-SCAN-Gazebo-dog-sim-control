# formal22冻结导出

本目录依据实际部署536c9d3及其1793项scope引用提炼，不取其它工作树“最新文件”。`SOURCE_EXPORT.json`逐文件记录原始SHA、公开SHA、角色和脱敏项，另列未上传的模型/ELF/构建输出。此公开导出不是原始二进制部署的同字节副本，也未从本目录重跑。

|目录|内容|
|---|---|
|`runtime/`|正式22 Git checkout的源码/配置，包括随冻结版本保留但未必启用的兼容模块和有限检查|
|`native/src/`|实际加载的FAST-LIVO2 native571源码；没有未加载的真实协方差候选|
|`scan/src/`|真实SCAN构建scope引用的源文件；其它共享源码/库来源在dependencies和manifest|
|`dependencies/`|实际scope需要的外部文本依赖与历史绑定；不是另一组已实测新候选|
|`deployment/`|正式22选择、PROFILE、原管理入口和已消费/过期的单轮窗口；不能直接用于再启动|
|`evidence/`|原任务/停车/动态/退出/加载/独立复核和有限检查的小型公开记录|
|`logs/`|明确first3+last3行的小日志片段，带原文件SHA和行号；不是完整回放或全部原件备份|
|`licenses/`|FAST-LIVO2与SCAN原有许可文本；各源文件版权/许可继续适用|

接受范围是当前R85单次F1→F3；`original46_PASS=false`。ENTRY_EXIT/closure工具的navigation_PASS=false是固定“不授业务PASS”字段，最终业务结果来自`evidence/run/r85_task_result.json`及各原核验回执，不修改旧标志来制造一致。

179份唯一连续证书中41份使用新依据/45个patch（SCAN29/165；旧路径保留connector12/14）。末两条SCAN旧整证书细分后也能通过，不能把41份都解释为避免失败，或把整轮PASS唯一归功这项改动。controller采样439.390s消耗407.880CPU秒、92.829%单核；不是GIL/GC归因或与旧轮严格性能对照。

库/模型按原SHA重建或自行合法提供，不能用相同文件名代替身份。实际Student导出SHA `d8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185`，native/SCAN ELF SHA在部署与原结果中。权重许可未重新授予，未上传权重。原仓库已有上游资源/许可继续保留；新机还需取得其依赖并重新审查构建/运行。公开路径使用`/opt/experiment/...`别名，不能误认为真实安装路径已经存在。
