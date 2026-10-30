# 本地Student模型接口

当前控制源码需要本地文件`student_model_4999_deterministic_jit.pt`，输入45维观测及`[1,1,256]`隐状态，输出12动作及新隐状态。动作缩放.25后叠加默认关节角；由`src/control/policy/student.py`进行CPU推理。

公开分支不包含该文件：具体权属、Teacher父链与再分发条款尚未确认。历史导航使用的本机文件保留不动；取得使用授权后自行放入同名路径。它不是可用任意同维模型替换后继承验收的接口。更多边界见MODEL_NOTICE.md。
