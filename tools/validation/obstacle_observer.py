"""Read-only test observation of moving_obstacle only; no robot truth."""
import json,math,queue,subprocess,threading,time
import numpy as np
ENTITY="moving_obstacle"
POSE_TOPIC="/world/teacher_demo/pose/info"

def _stamp(header):
    stamp=header.get('stamp',{})
    sec=stamp.get('sec',0); nsec=stamp.get('nsec',stamp.get('nanosec',0))
    def integer(value):
        if type(value)is int:return value
        if isinstance(value,str) and value.isascii() and value.isdigit():return int(value)
        raise ValueError('invalid_native_stamp')
    sec,nsec=integer(sec),integer(nsec)
    if sec<0 or not 0<=nsec<1_000_000_000:raise ValueError('invalid_native_stamp')
    return sec*1_000_000_000+nsec

def obstacle_observations(message):
    """Parse only exact model names in native gz.msgs.Pose_V JSON."""
    stamp=_stamp(message.get('header',{})); out=[]
    for pose in message.get('pose',[]):
        if pose.get('name')!=ENTITY: continue
        p=pose.get('position',{}); xyz=[float(p.get(k,0.)) for k in ('x','y','z')]
        if stamp<=0 or not all(math.isfinite(v) for v in xyz): raise ValueError('invalid_native_obstacle_pose')
        orientation=pose.get('orientation',{})
        q=[float(orientation.get(k,0.)) for k in ('x','y','z','w')]
        if (not all(math.isfinite(v) for v in q) or np.linalg.norm(q[:3])>1e-6
                or abs(abs(q[3])-1.)>1e-6):raise ValueError('native_obstacle_axis_alignment_changed')
        out.append(dict(entity=ENTITY,entity_id=pose.get('id'),position_world_xyz=xyz,stamp_ns=stamp,
            topic=POSE_TOPIC,native_gz_pose_info=True,source_message_type='gz.msgs.Pose_V',
            raw_model_pose=pose,header=message.get('header',{})))
    return out

class NativeObstacleObserver:
    """Owned read-only CLI child. It never publishes or reads a robot GT pose."""
    def __init__(self,*,spool_path=None):
        self.error=None; self.stopping=False;self.spool_enabled=spool_path is not None
        if self.spool_enabled:
            from native_observation_spool import NativeObservationSpool
            self.queue=NativeObservationSpool(spool_path)
        else:self.queue=queue.Queue(512)
        try:
            self.process=subprocess.Popen(['gz','topic','-e','--json-output','-t',POSE_TOPIC],
                stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,bufsize=1)
        except Exception:
            if self.spool_enabled:self.queue.close()
            raise
        self.thread=threading.Thread(target=self._read,daemon=True,name='r85-obstacle-pose-info');self.thread.start()
    def _read(self):
        decoder=json.JSONDecoder(); buffer=''
        try:
            for line in self.process.stdout:
                if self.stopping:return
                buffer+=line
                if len(buffer)>4_194_304:raise ValueError('native_pose_info_message_exceeds_4MiB')
                while buffer.strip():
                    buffer=buffer.lstrip()
                    try: message,end=decoder.raw_decode(buffer)
                    except json.JSONDecodeError:break
                    buffer=buffer[end:]
                    for value in obstacle_observations(message):
                        self.queue.put_nowait((value,time.monotonic()))
            if not self.stopping:self.error='native_pose_info_observer_exited:'+str(self.process.poll())
        except Exception as error:self.error=type(error).__name__+':'+str(error)
    def close(self):
        self.stopping=True
        if self.process.poll()is None:
            self.process.terminate()
            try:self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2.)
        self.thread.join(timeout=2.)
        if self.spool_enabled:
            if self.thread.is_alive():
                if self.error is None:self.error='native_observation_spool_reader_shutdown_incomplete'
                raise RuntimeError(self.error)
            else:self.queue.close()
