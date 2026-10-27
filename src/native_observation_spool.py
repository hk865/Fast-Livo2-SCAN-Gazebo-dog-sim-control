"""Single reader-thread producer / single ROS-tick consumer, append-only evidence FIFO.
The file is retained on close, including any unread committed records or failed tail.
"""
import json
import math
import os
from pathlib import Path
import queue
import threading

MAX_SPOOL_BYTES=128*1024**2
MAX_RECORD_BYTES=4*1024**2
READ_CHUNK_BYTES=64*1024


def _reject_constant(value):raise ValueError('nonfinite_native_observation_spool_JSON:'+value)


def _finite_float(value):
    result=float(value)
    if not math.isfinite(result):raise ValueError('nonfinite_native_observation_spool_JSON')
    return result


class NativeObservationSpool:
    def __init__(self,path):
        self.path=Path(path)
        self._fd=os.open(self.path,os.O_RDWR|os.O_CREAT|os.O_EXCL|os.O_APPEND,0o600)
        self._lock=threading.Lock();self._committed=0;self._consumed=0
        self._loaded=0;self._buffer=bytearray();self._closed=False;self.error=None

    def _fail(self,error):
        if self.error is None:self.error=type(error).__name__+':'+str(error)
        return error

    def _open(self):
        if self._closed:raise RuntimeError('native_observation_spool_closed')
        if self.error is not None:raise RuntimeError(self.error)

    def put_nowait(self,item):
        try:
            if (type(item)is not tuple or len(item)!=2 or not isinstance(item[0],dict)
                    or type(item[1])not in (int,float) or not math.isfinite(item[1])):
                raise ValueError('invalid_native_observation_spool_item')
            raw=(json.dumps(item,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n').encode('utf-8')
            if len(raw)>MAX_RECORD_BYTES:raise ValueError('native_observation_spool_record_budget')
            with self._lock:
                self._open()
                if self._committed+len(raw)>MAX_SPOOL_BYTES:raise ValueError('native_observation_spool_total_budget')
                written=os.write(self._fd,raw)
                if written!=len(raw):raise OSError('native_observation_spool_short_write')
                self._committed+=written
        except Exception as error:
            raise self._fail(error)

    def get_nowait(self):
        try:
            with self._lock:
                self._open();committed=self._committed
            if self._consumed==committed:raise queue.Empty
            while b'\n'not in self._buffer:
                size=min(READ_CHUNK_BYTES,committed-self._loaded)
                if size<=0:raise ValueError('native_observation_spool_partial_committed_record')
                raw=os.pread(self._fd,size,self._loaded)
                if len(raw)!=size:raise OSError('native_observation_spool_partial_committed_read')
                self._loaded+=len(raw);self._buffer.extend(raw)
                if len(self._buffer)>MAX_RECORD_BYTES+READ_CHUNK_BYTES:
                    raise ValueError('native_observation_spool_record_budget')
            end=self._buffer.index(10)+1
            if end>MAX_RECORD_BYTES:raise ValueError('native_observation_spool_record_budget')
            item=json.loads(self._buffer[:end],parse_constant=_reject_constant,parse_float=_finite_float);del self._buffer[:end]
            if (not isinstance(item,list) or len(item)!=2 or not isinstance(item[0],dict)
                    or type(item[1])not in (int,float) or not math.isfinite(item[1])):
                raise ValueError('invalid_native_observation_spool_record')
            self._consumed+=end
            return item[0],item[1]
        except queue.Empty:raise
        except Exception as error:raise self._fail(error)

    def snapshot_state(self):
        with self._lock:
            return dict(path=str(self.path),committed_bytes=self._committed,
                consumed_bytes=self._consumed,unread_committed_bytes=self._committed-self._consumed,
                maximum_bytes=MAX_SPOOL_BYTES,closed=self._closed,error=self.error,
                consumer_at_committed_prefix=self._consumed==self._committed)

    def close(self):
        failure=None
        with self._lock:
            if not self._closed:
                try:os.fsync(self._fd)
                except Exception as error:failure=self._fail(error)
                try:os.close(self._fd)
                except Exception as error:
                    self._fail(error)
                    if failure is None:failure=error
                finally:self._closed=True
        if failure is not None:raise failure
        return self.snapshot_state()
