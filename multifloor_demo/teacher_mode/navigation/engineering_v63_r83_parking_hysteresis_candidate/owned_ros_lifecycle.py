"""Signal only requests exit; ROS teardown follows the last callback."""
import signal

class OwnedRosLifecycle:
    def __init__(self,rclpy,args,signal_handler_options_no,signal_api=signal):
        self.ros=rclpy;self.signals=signal_api;self.stop_requested=False
        self.saved={sig:signal_api.getsignal(sig)for sig in (signal_api.SIGINT,signal_api.SIGTERM)}
        try:
            for sig in self.saved:signal_api.signal(sig,self.request_stop)
            rclpy.init(args=args,signal_handler_options=signal_handler_options_no)
        except BaseException:
            try:
                if self.ros.ok():self.ros.shutdown()
            finally:self.restore()
            raise
    def request_stop(self,signum,frame):self.stop_requested=True
    def spin(self,node):
        while not self.stop_requested and self.ros.ok():
            self.ros.spin_once(node,timeout_sec=.1)
    def restore(self):
        for sig,handler in self.saved.items():self.signals.signal(sig,handler)
    def close(self,node):
        try:
            if node is not None:node.close()
        finally:
            try:
                if node is not None:node.destroy_node()
            finally:
                try:
                    if self.ros.ok():self.ros.shutdown()
                finally:self.restore()
