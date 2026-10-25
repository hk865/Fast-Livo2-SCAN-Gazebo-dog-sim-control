# B05 / R115 实施、部署与组件SHA释义

原17文件及history.csv/json的行内容无需更正，均保持原字节。此附页只澄清层次：

- B05实施：`cd19e89a4120d13daeb16d38f92d932d41e9a040`，精确来自原SPECIFICATION.implementation_commit。
- B05实际部署：`d6f84da81ced8738b8d7a7c3ff0f1a06639148ba`，原DEPLOYMENT_FREEZE、ENTRY_EXIT、ACTUAL_LOADING、POST_RUN_COMPACT四处一致。它是cd19的直接子提交，**不是仅组件候选SHA**。
- git两提交只有POLICY_SELECTION_CONTRACT.json、B05_RESOURCE_ADMIN、R73_SCAN_RUNTIME_CONTRACT.json和r85_resource_renewal.py四个行政绑定差异。R119 Python/XML/native cpp/so/buildreceipt五文件字节均未变，SHA逐项吻合实际r119 setup收据。不是把文档某时HEAD当业务候选。
- 实际metadata writer publish_mode=1（ASYNC），其他body/lidar/cloud/status仍0；这只是原B05已加载事实，不重新审其效果或宣称331ms根治。完整组件SHA及来源文件SHA见同名JSON。

R115两档共同实施`4bac533bcccc493f5a606942842fcf83e824ddda`；020部署`c57f85a4a2c9f53f9b899553c6a0f95fbf1492b8`、050部署`49c9cfe23b554923ba336498529bfbc201b21706`继续分列，依据旧round26对应分轮RESULT明确记录。020只完成native180短窗、050启动失败，不能合成完整速度/任务PASS。

本次仅上述小收据与git对象核对，无大raw/源码修改、构建、仿真或远程操作。此附录新增后总manifest更新，旧17文件SHA全相同。
