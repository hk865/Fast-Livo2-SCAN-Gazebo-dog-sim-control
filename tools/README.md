# 独立开发工具

[validation](validation/README.md)包含完整导航观察与停车/障碍检查。它显式启动正常入口并观察本次执行，普通`start.py`不导入它，也不依赖历史PASS文件。

工具会改变测试障碍/运行生命周期，不能把它当只读查看器或普通启动所需步骤。小规模回归位于根`tests/`；源码/消息/时序见[结构文档](../docs/source_architecture.zh-CN.md)。
