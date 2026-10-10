"""Bounded optional IPC history and wall-clock timing; no actuator decisions.

Live telemetry and causal ACK snapshots remain synchronous. Optional histories
can be omitted, with explicit final receipts. No receipt can certify native
parking: the native 200ms watchdog and controller source deadlines are unchanged.
"""
from __future__ import annotations
import collections
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import threading
import time


class BoundedHistory:
    """No producer-side disk I/O after construction; bounded bytes and records.

    Pending bounds include the in-flight write. No disk operation holds the
    condition lock. Stats distinguish attempted, admitted, written and omitted.
    A partial write/error is never counted as a successful record.
    """
    def __init__(self, path, *, maximum_file_bytes, maximum_queue_bytes=512*1024,
                 maximum_queue_records=256, maximum_record_bytes=32*1024):
        self.path=Path(path)
        self.maximum_file_bytes=int(maximum_file_bytes)
        self.maximum_queue_bytes=int(maximum_queue_bytes)
        self.maximum_queue_records=int(maximum_queue_records)
        self.maximum_record_bytes=int(maximum_record_bytes)
        if min(self.maximum_file_bytes,self.maximum_queue_bytes,
               self.maximum_queue_records,self.maximum_record_bytes)<=0:
            raise ValueError('Positive history bounds required')
        self.stream=self.path.open('xb')
        self.condition=threading.Condition();self.queue=collections.deque()
        self.pending_bytes=0;self.pending_records=0;self.stopping=False;self.closed=False
        self.stats=dict(schema='go2_optional_ipc_history/v1',attempted=0,admitted=0,
            written=0,omitted=0,omitted_at_admission=0,attempted_bytes=0,file_bytes=0,
            maximum_pending_bytes=0,maximum_pending_records=0,writer_io_failed=False,
            byte_limit_reached=False,oversize_records=0,queue_limit_reached=False,
            maximum_file_bytes=self.maximum_file_bytes,queue_byte_limit=self.maximum_queue_bytes,
            queue_record_limit=self.maximum_queue_records,record_byte_limit=self.maximum_record_bytes,
            final=False,drained=False,last_attempt_sha256=None,last_attempt_sequence=None)
        self.thread=threading.Thread(target=self._loop,name='optional-ipc-history',daemon=True)
        self.thread.start()

    def submit(self, text, *, sequence=None):
        if not isinstance(text,(str,bytes)):raise TypeError('History requires immutable text/bytes')
        body=text.encode('utf-8') if isinstance(text,str) else text
        digest=hashlib.sha256(body).hexdigest();size=len(body)
        with self.condition:
            s=self.stats;s['attempted']+=1;s['attempted_bytes']+=size
            s['last_attempt_sha256']=digest;s['last_attempt_sequence']=sequence
            oversize=size>self.maximum_record_bytes
            byte_limit=s['file_bytes']+self.pending_bytes+size>self.maximum_file_bytes
            queue_limit=(self.pending_records>=self.maximum_queue_records or
                         self.pending_bytes+size>self.maximum_queue_bytes)
            if self.stopping or s['writer_io_failed'] or oversize or byte_limit or queue_limit:
                s['omitted_at_admission']+=1
                s['oversize_records']+=int(oversize)
                s['byte_limit_reached']|=byte_limit;s['queue_limit_reached']|=queue_limit
                return False
            self.queue.append(body);self.pending_bytes+=size;self.pending_records+=1
            s['admitted']+=1
            s['maximum_pending_bytes']=max(s['maximum_pending_bytes'],self.pending_bytes)
            s['maximum_pending_records']=max(s['maximum_pending_records'],self.pending_records)
            self.condition.notify();return True

    def _loop(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda:self.stopping or bool(self.queue))
                    if not self.queue:
                        if self.stopping:break
                        continue
                    body=self.queue.popleft()
                count=self.stream.write(body)
                if count!=len(body):raise OSError('Partial optional history write')
                with self.condition:
                    self.stats['written']+=1;self.stats['file_bytes']+=len(body)
                    self.pending_bytes-=len(body);self.pending_records-=1
        except Exception as error:
            with self.condition:
                self.stats['writer_io_failed']=True
                self.stats['writer_error']=type(error).__name__+': '+str(error)
        finally:
            try:self.stream.flush();self.stream.close()
            except Exception as error:
                with self.condition:
                    self.stats['writer_io_failed']=True
                    self.stats['writer_error']=type(error).__name__+': '+str(error)

    def snapshot(self):
        with self.condition:
            s=dict(self.stats);s['omitted']=s['attempted']-s['written']
            s.update(pending_bytes=self.pending_bytes,pending_records=self.pending_records)
            s['capture_complete']=bool(s['final'] and s['drained'] and
                not s['writer_io_failed'] and s['omitted']==0)
            return s

    def close(self):
        if not self.closed:
            with self.condition:self.stopping=True;self.condition.notify_all()
            self.thread.join(timeout=5)
            with self.condition:
                self.stats['final']=not self.thread.is_alive()
                self.stats['drained']=(not self.thread.is_alive() and self.pending_records==0)
                self.closed=True
        result=self.snapshot()
        # Called after the actuator loop; final receipt I/O is never a frame stage.
        receipt=self.path.with_suffix(self.path.suffix+'.stats.json')
        tmp=receipt.with_suffix(receipt.suffix+'.tmp')
        tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');tmp.replace(receipt)
        return result


class HistorySink:
    def __init__(self, writer, timer, stage):
        self.writer=writer;self.timer=timer;self.stage=stage;self.closed=False
    def write(self, text):
        if self.closed:raise ValueError('Closed optional history')
        with self.timer.measure(self.stage):self.writer.submit(text,sequence=self.timer.sequence)
        return len(text)
    def flush(self):pass
    def close(self):self.closed=True


class TimedSink:
    """Wrap only synchronous telemetry; do not change its exact row sampling."""
    def __init__(self, stream, timer, stage):self.stream=stream;self.timer=timer;self.stage=stage
    def write(self, text):
        with self.timer.measure(self.stage):return self.stream.write(text)
    def __getattr__(self,name):return getattr(self.stream,name)


class FrameTiming:
    def __init__(self, writer):
        self.writer=writer;self.sequence=0;self.frame=None
        self.last_read_end=None;self.last_send_end=None;self.frame_errors=0

    def begin_receive(self):
        if self.frame is not None:self.finish('incomplete_previous_frame')
        now=time.monotonic_ns();self.sequence+=1
        self.frame=dict(schema='teacher_ipc_frame_wall_timing/v1',frame_sequence=self.sequence,
            receive_start_wall_ns=now,stages_ns={},stage_calls={},stage_errors={},
            stages_overlap=True,intervals=[],native_deadline_ms=200,source_deadline_ms=300,
            after_previous_send_to_receive_start_ns=None if self.last_send_end is None else now-self.last_send_end,
            native_parking_verified=False,recv_calls=0,recv_total_bytes=0,recv_events=[],recv_events_omitted=0,recv_event_limit=8)

    @contextmanager
    def measure(self,name):
        frame=self.frame
        if frame is None:yield;return
        start=time.monotonic_ns();error=False
        try:yield
        except BaseException:error=True;raise
        finally:
            end=time.monotonic_ns()
            frame['stages_ns'][name]=frame['stages_ns'].get(name,0)+end-start
            frame['stage_calls'][name]=frame['stage_calls'].get(name,0)+1
            if error:frame['stage_errors'][name]=frame['stage_errors'].get(name,0)+1
            frame['intervals'].append((start,end))

    def receive_event(self, requested, start, end, returned=None, error=None):
        frame=self.frame
        if frame is None:return
        frame['recv_calls']+=1
        if returned is not None:frame['recv_total_bytes']+=returned
        duration=end-start
        frame['stages_ns']['state_recv_wait']=frame['stages_ns'].get('state_recv_wait',0)+duration
        frame['stage_calls']['state_recv_wait']=frame['stage_calls'].get('state_recv_wait',0)+1
        if error is not None:frame['stage_errors']['state_recv_wait']=frame['stage_errors'].get('state_recv_wait',0)+1
        if len(frame['recv_events'])<frame['recv_event_limit']:
            frame['recv_events'].append(dict(requested_bytes=requested,returned_bytes=returned,
                start_wall_ns=start,end_wall_ns=end,error=None if error is None else type(error).__name__))
        else:frame['recv_events_omitted']+=1

    def received(self, clock_ns):
        now=time.monotonic_ns();self.frame['state_read_end_wall_ns']=now
        self.frame['native_clock_ns']=clock_ns
        self.frame['state_read_interval_ns']=None if self.last_read_end is None else now-self.last_read_end
        self.last_read_end=now

    def command_binding(self, attempt):
        if self.frame is None:return
        envelope=attempt.get('decoded_envelope') or {}
        self.frame.update(command_sequence=envelope.get('sequence'),
            original_command_bytes_sha256=attempt.get('raw_bytes_sha256'),
            command_validation_status=attempt.get('status'),command_validation_reason=attempt.get('reason'))

    def finish(self, outcome, error=None, *, response_sent=False):
        if self.frame is None:return
        frame=self.frame;self.frame=None;end=time.monotonic_ns()
        intervals=sorted(frame.pop('intervals'));covered=0;left=right=None
        for a,b in intervals:
            if left is None:left,right=a,b
            elif a<=right:right=max(right,b)
            else:covered+=right-left;left,right=a,b
        if left is not None:covered+=right-left
        frame.update(outcome=outcome,response_sent=response_sent,end_wall_ns=end,
            receive_start_to_end_ns=end-frame['receive_start_wall_ns'],
            after_read_to_end_ns=None if 'state_read_end_wall_ns' not in frame else end-frame['state_read_end_wall_ns'],
            measured_stage_union_ns=covered,
            unmeasured_ns=max(0,end-frame['receive_start_wall_ns']-covered),
            previous_send_to_end_ns=None if self.last_send_end is None else end-self.last_send_end)
        if error is not None:frame['error']=type(error).__name__+': '+str(error);self.frame_errors+=1
        if response_sent:self.last_send_end=end
        self.writer.submit(json.dumps(frame,separators=(',',':'),allow_nan=False)+'\n',sequence=self.sequence)


class TimedConnection:
    def __init__(self, conn, timer):self.conn=conn;self.timer=timer
    def recv(self, size, *args, **kwargs):
        start=time.monotonic_ns()
        try:data=self.conn.recv(size,*args,**kwargs)
        except BaseException as error:
            self.timer.receive_event(size,start,time.monotonic_ns(),error=error)
            raise
        self.timer.receive_event(size,start,time.monotonic_ns(),returned=len(data))
        return data

    def sendall(self, data, *args, **kwargs):
        try:
            with self.timer.measure('response_send'):result=self.conn.sendall(data,*args,**kwargs)
        except BaseException as error:
            self.timer.finish('response_send_failed',error);raise
        self.timer.finish('response_sent',response_sent=True)
        return result
    def __getattr__(self,name):return getattr(self.conn,name)
    def __enter__(self):self.conn.__enter__();return self
    def __exit__(self,*args):return self.conn.__exit__(*args)


class TimedSocketModule:
    """Module-local wrapper: does not monkeypatch global Python socket."""
    def __init__(self,module,timer):self.module=module;self.timer=timer
    def __getattr__(self,name):return getattr(self.module,name)
    def socket(self,*args,**kwargs):
        server=self.module.socket(*args,**kwargs);timer=self.timer
        class Server:
            def accept(self):
                conn,address=server.accept();return TimedConnection(conn,timer),address
            def __getattr__(self,name):return getattr(server,name)
        return Server()
