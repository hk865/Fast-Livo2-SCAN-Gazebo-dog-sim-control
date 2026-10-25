"""One bounded real positive-commit archive. Evidence only; no control authority."""
from pathlib import Path
import hashlib,json,threading
from rclpy.serialization import serialize_message
MAX_B=16*1024*1024
class FirstIdentityCapture:
    def __init__(self,run):self.run=Path(run);self.thread=None;self.error=None
    def observe(self,slot,parent,clock):
        if self.thread is not None:return
        # These messages are immutable subscriber objects; only one bundle is kept.
        saved={k:v[0] for k,v in slot.items()}
        self.thread=threading.Thread(target=self._save,args=(saved,dict(parent),int(clock)),daemon=True)
        self.thread.start()
    def _save(self,saved,parent,clock):
        try:
            binary={k:bytes(serialize_message(saved[k])) for k in ('raw','adapted','registered','mapper','body','lidar')}
            metadata={k:saved[k] for k in ('source_identity','point_identity','odom_identity')}
            text=(json.dumps(dict(schema='actual_positive_point_commit_capture/v1',source_commit=parent,metadata=metadata,
                original_clock_ns=clock,run_id=self.run.name,formal_R6_complete_sampling=False,robot_truth_pose_used=False),allow_nan=False)+'\n').encode()
            if sum(map(len,binary.values()))+len(text)>MAX_B-128*1024:raise ValueError('one commit archive exceeds16MiB; no payload truncated')
            directory=self.run/'first_actual_point_commit';directory.mkdir()
            for k,data in binary.items():(directory/(k+'.cdr')).write_bytes(data)
            (directory/'COMMIT.json').write_bytes(text)
            bindings={str(p.name):dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())for p in directory.iterdir()if p.is_file()}
            logical=sum(p.stat().st_size for p in directory.iterdir()if p.is_file())
            allocated=sum(p.stat().st_blocks*512 for p in directory.iterdir())+directory.stat().st_blocks*512
            (directory/'RECEIPT.json').write_text(json.dumps(dict(complete=True,files=bindings,maximum_B=MAX_B,
                pre_receipt_logical_B=logical,pre_receipt_allocated_B=allocated,receipt_and_filesystem_reserve_B=128*1024,
                only_actual_messages=True,observational_capture=True,formal_R6_complete_sampling=False),allow_nan=False)+'\n')
        except BaseException as e:
            self.error=type(e).__name__+': '+str(e)
            (self.run/'first_actual_point_commit_error.json').write_text(json.dumps(dict(error=self.error,control_authority=False))+'\n')
    def close(self):
        if self.thread is not None:self.thread.join(timeout=2.)
        return dict(started=self.thread is not None,completed=self.thread is not None and not self.thread.is_alive()and self.error is None,error=self.error)
