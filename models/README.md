# Student部署模型卡

本分支提供现有去debug部署版`student_model_4999_deterministic_jit.pt`（1,342,573字节），由发布者项目本地训练并导出。本次只补回既有文件，没有重训、重新导出或修改参数。来源、证据路径及对外许可状态见[MODEL_NOTICE.md](MODEL_NOTICE.md)。

| 项目 | 当前接口 |
| --- | --- |
| 运行格式/设备 | TorchScript，CPU推理 |
| 机器人输入 | `obs: [N,45]`，无高度扫描、真值线速度或全局位姿直接输入 |
| 记忆输入/输出 | 单层GRU256，`h_in/h_out: [1,N,256]` |
| 动作输出 | `actions: [N,12]`，腿序FR/FL/RR/RL，各hip/thigh/calf |
| 动作解释 | `q_target=q_default+0.25*action`；原执行器限制继续生效 |
| 控制周期 | 仿真50Hz；运行源码`src/control/policy/student.py` |
| 初始化 | 机器人/episode启动时隐状态清零，不在命令切换时清零 |
| 已验证用途 | 配置出生先验下的Gazebo Building导航；不代表真机部署或任意场景泛化 |

实际launch与预检查都读取根目录下此同名文件；Docker构建也将它纳入镜像。L1网格仍未包含，需按根README处理，不能将新检出称为全部资产齐备。

本模型是发布者项目的自训部署产物，不是宇树官方预训练Student。训练使用环境及冻结Teacher监督；发布授权不自动替代第三方依赖的各自许可，也不构成新的MIT/Apache等权重许可。
