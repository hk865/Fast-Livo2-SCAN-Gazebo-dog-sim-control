# 运行器中断清理审计

仅审计 `scripts/run_test.py` 的小范围信号改动，未修改运行器或导航源码。SIGTERM→143、SIGINT→130 的两个真实 Python mock 子进程探针通过：运行器收到信号后捕获 InterruptedError，屏蔽清理期间的重复信号，只向自己记录的两个 child process group 发SIGINT，两者正常退出，failed摘要与runtime error均写入；独立sentinel在运行器清理后仍存活，最后由探针自身结束。没有启动ROS/Gazebo、载入Teacher或触碰训练。具体PID、退出码与源SHA见 `receipt.json`。

实际 evaluator 已检查：`actual_run_end` 要求无runtime.error，因此中断不会被误判正常完成。mock evaluator只验证错误被传递、摘要已落盘，不冒充物理验收。

边界仍保留：prepare/source-copy阶段在runtime try外，此时尚未建立仿真子进程，但中断不保证run receipt；既有最后wait/resource/evaluator异常可能打断收据；异步信号落在Popen返回与变量赋值之间的窄窗口未由此确定性探针覆盖。这些并非本次新增handler改变的信号对象范围。
