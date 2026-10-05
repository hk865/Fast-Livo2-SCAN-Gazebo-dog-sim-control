"""Bounded actual decoded registered-cloud pre-roll; no ROS or control output."""
from collections import deque
import copy


class ActualCloudPreRoll:
    def __init__(self,span_sim_s=1.,max_frames=32,max_bytes=16*1024*1024):
        self.span_ns=round(span_sim_s*1e9);self.max_frames=max_frames;self.max_bytes=max_bytes
        self.entries=deque();self.bytes=0;self.last_stamp=None;self.error=None
        self.evicted_expired=0;self.duplicates=0;self.flushed_frames=0;self.peak_frames=0;self.peak_bytes=0
    def fail(self,message):
        self.error=message;raise RuntimeError(message)
    def add(self,row,xyz):
        stamp=row['stamp_ns']
        if type(stamp)is not int or stamp<0:self.fail('Invalid original registered-cloud integer stamp')
        if self.last_stamp is not None and stamp<self.last_stamp:
            self.fail('Registered-cloud pre-roll source stamp moved backward; refusing silent reorder')
        if self.entries and stamp==self.entries[-1][0]['stamp_ns']:
            if row['data_sha256']!=self.entries[-1][0]['data_sha256']:
                self.fail('Same original cloud stamp has different raw payload hash')
            self.duplicates+=1;return False
        self.last_stamp=stamp
        while self.entries and stamp-self.entries[0][0]['stamp_ns']>self.span_ns:
            _,old=self.entries.popleft();self.bytes-=old.nbytes;self.evicted_expired+=1
        if len(self.entries)>=self.max_frames or self.bytes+xyz.nbytes>self.max_bytes:
            self.fail('Actual cloud pre-roll capacity overflow inside retained source-time window')
        xyz.setflags(write=False)
        self.entries.append((copy.deepcopy(row),xyz));self.bytes+=xyz.nbytes
        self.peak_frames=max(self.peak_frames,len(self.entries));self.peak_bytes=max(self.peak_bytes,self.bytes)
        return True
    def drain(self):
        result=list(self.entries);self.flushed_frames+=len(result);self.entries.clear();self.bytes=0
        return result
    def receipt(self):
        return {'span_sim_s':self.span_ns/1e9,'max_frames':self.max_frames,'max_bytes':self.max_bytes,
            'cached_frames':len(self.entries),'cached_bytes':self.bytes,'peak_frames':self.peak_frames,
            'peak_bytes':self.peak_bytes,'expired_source_time_evictions':self.evicted_expired,
            'same_stamp_same_payload_duplicates':self.duplicates,'flushed_frames':self.flushed_frames,
            'error':self.error,'synthetic_or_refreshed_data_permitted':False}
