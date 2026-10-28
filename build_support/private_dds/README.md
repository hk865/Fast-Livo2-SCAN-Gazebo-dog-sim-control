# Fast DDS双端heartbeat补丁构建

通常由[全量构建入口](../README.md)调用。底层`build_private_dds.py`接受已按`third_party/fastdds/SOURCES.json`下载的官方archives，在新工作目录应用生产补丁与SHM fixture，输出`install/dds`。

生产补丁同时覆盖SHM checker通知与listener空唤醒heartbeat刷新。全量构建随后执行5项reviewed_heartbeat测试，含旧listener负对照、停止listener总超时、正常数据与close。组件通过不等于导航通过，也不能将任意队列故障归因DDS。

上游配置中与本组件无关的嵌套Docker网络集成测试显式关闭；SHM目标仍完整构建/执行。不挂载Docker socket，不覆盖系统DDS。运行统一环境将本包库放在所有通信端搜索路径首位，实际加载证据见本轮验证记录。
