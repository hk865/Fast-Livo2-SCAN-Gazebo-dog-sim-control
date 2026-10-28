# Student模型

`student_model_4999_deterministic_jit.pt`为当前必要资产，本发行已包含。输入为45维观测及`[1,1,256]`隐状态，输出12动作及新隐状态；动作缩放0.25后叠加默认关节角。模型由`src/control/policy/student.py`加载。

接口与来源见[通知](MODEL_NOTICE.md)、[控制模块](../src/control/README.md)和[技术报告](../docs/technical_report.zh-CN.md)。不会用模型hash作为历史启动批准。
