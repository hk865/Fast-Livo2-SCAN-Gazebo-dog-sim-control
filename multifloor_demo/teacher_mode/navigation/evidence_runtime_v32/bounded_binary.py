"""Nonblocking, byte-bounded optional evidence writer with explicit loss receipts.

Format: 16-byte magic, then repeated <uint64 metadata_size, uint64 body_size>,
UTF-8 JSON metadata and exact body bytes. A failed capture is never success.
"""
import collections
import hashlib
import json
from pathlib import Path
import struct
import threading
import time

MAGIC = b'GO2BOUNDED001LE\0'
assert len(MAGIC) == 16

class BoundedBinaryWriter:
    def __init__(self, path, *, maximum_file_bytes, maximum_queue_bytes=16*1024*1024,
                 maximum_queue_records=64):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.maximum_file_bytes=int(maximum_file_bytes)
        self.maximum_queue_bytes=int(maximum_queue_bytes)
        self.maximum_queue_records=int(maximum_queue_records)
        if min(self.maximum_file_bytes,self.maximum_queue_bytes,self.maximum_queue_records)<=0:
            raise ValueError('Positive recording bounds required')
        self.stream=self.path.open('xb');self.stream.write(MAGIC)
        self.condition=threading.Condition();self.queue=collections.deque()
        self.queued_bytes=0;self.stopping=False;self.closed=False
        self.stats=dict(schema='go2_bounded_binary_writer/v1',attempted=0,accepted=0,
            written=0,dropped=0,file_bytes=16,maximum_queue_bytes=0,
            maximum_queue_records=0,writer_io_failed=False,byte_limit_reached=False,
            maximum_file_bytes=self.maximum_file_bytes,queue_byte_limit=self.maximum_queue_bytes,
            queue_record_limit=self.maximum_queue_records,final=False,drained=False)
        self.thread=threading.Thread(target=self._loop,name='optional-evidence-writer',daemon=True)
        self.thread.start()

    def submit(self, metadata, body):
        if not isinstance(body,(bytes,str)):raise TypeError('Evidence body must be immutable bytes or str')
        # Reserve conservatively for UTF-8 and metadata without serializing large
        # command text on the actuator thread. Metadata must remain small.
        reserve=(len(body) if isinstance(body,bytes) else 4*len(body))+8192
        with self.condition:
            self.stats['attempted']+=1
            if (self.stopping or self.stats['writer_io_failed'] or
                len(self.queue)>=self.maximum_queue_records or
                self.queued_bytes+reserve>self.maximum_queue_bytes or
                self.stats['file_bytes']+self.queued_bytes+reserve>self.maximum_file_bytes):
                self.stats['dropped']+=1
                if self.stats['file_bytes']+self.queued_bytes+reserve>self.maximum_file_bytes:
                    self.stats['byte_limit_reached']=True
                return False
            self.queue.append((dict(metadata),body,reserve));self.queued_bytes+=reserve
            self.stats['accepted']+=1
            self.stats['maximum_queue_bytes']=max(self.stats['maximum_queue_bytes'],self.queued_bytes)
            self.stats['maximum_queue_records']=max(self.stats['maximum_queue_records'],len(self.queue))
            self.condition.notify()
            return True

    def _loop(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda:self.stopping or bool(self.queue),timeout=.2)
                    if not self.queue:
                        if self.stopping:break
                        continue
                    metadata,body,reserve=self.queue.popleft()
                    self.queued_bytes-=reserve
                if isinstance(body,str):body=body.encode('utf-8')
                metadata['body_bytes']=len(body)
                metadata['body_sha256']=hashlib.sha256(body).hexdigest()
                expected=metadata.get('expected_body_sha256')
                if expected is not None and expected!=metadata['body_sha256']:
                    raise ValueError('Queued original body SHA256 mismatch')
                meta=json.dumps(metadata,separators=(',',':'),sort_keys=True,allow_nan=False).encode()
                size=16+len(meta)+len(body)
                if len(meta)>8192 or self.stats['file_bytes']+size>self.maximum_file_bytes:
                    with self.condition:
                        self.stats['dropped']+=1;self.stats['byte_limit_reached']=True
                    continue
                self.stream.write(struct.pack('<QQ',len(meta),len(body)));self.stream.write(meta);self.stream.write(body)
                with self.condition:
                    self.stats['written']+=1;self.stats['file_bytes']+=size
        except Exception as error:
            with self.condition:
                self.stats['writer_io_failed']=True
                self.stats['writer_error']=type(error).__name__+': '+str(error)
        finally:
            try:self.stream.flush();self.stream.close()
            except OSError as error:
                self.stats['writer_io_failed']=True;self.stats['writer_error']=str(error)

    def snapshot(self):
        with self.condition:
            return {**self.stats,'queued_records':len(self.queue),'queued_bytes':self.queued_bytes,
                    'capture_complete':bool(self.stats['final'] and self.stats['drained'] and
                        not self.stats['writer_io_failed'] and self.stats['dropped']==0 and
                        self.stats['attempted']==self.stats['written'])}

    def close(self):
        if self.closed:return self.snapshot()
        with self.condition:self.stopping=True;self.condition.notify_all()
        self.thread.join(timeout=10)
        with self.condition:
            self.stats['drained']=not self.thread.is_alive() and not self.queue
            self.stats['final']=not self.thread.is_alive()
            self.closed=True
        result=self.snapshot();tmp=self.path.with_suffix(self.path.suffix+'.stats.tmp')
        tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        tmp.replace(self.path.with_suffix(self.path.suffix+'.stats.json'))
        return result

def read_records(path):
    """Streaming verifier/reader; no ROS required to inspect metadata and bytes."""
    with Path(path).open('rb') as f:
        if f.read(16)!=MAGIC:raise ValueError('Wrong bounded evidence magic')
        while True:
            header=f.read(16)
            if not header:break
            if len(header)!=16:raise ValueError('Truncated record header')
            m,n=struct.unpack('<QQ',header)
            if m>8192:raise ValueError('Invalid metadata length')
            metadata_bytes=f.read(m);body=f.read(n)
            if len(metadata_bytes)!=m or len(body)!=n:raise ValueError('Truncated record')
            metadata=json.loads(metadata_bytes)
            if metadata['body_bytes']!=n or hashlib.sha256(body).hexdigest()!=metadata['body_sha256']:
                raise ValueError('Evidence payload SHA mismatch')
            yield metadata,body
