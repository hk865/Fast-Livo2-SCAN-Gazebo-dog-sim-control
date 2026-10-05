"""Bounded asynchronous native event evidence, never a control input."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import queue
import tempfile
import threading

import numpy as np

_UNSCOPED = object()


def json_value(value):
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.generic):
        return json_value(value.item())
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    return value


class EventArchive:
    """Queue immutable bundles after safe zero; all file work stays in worker.

    The controller retains its own read-only per-message cloud, so a later ROS
    callback cannot replace the buffer stored in an earlier guard bundle.
    Archive errors are diagnostic only and cannot cancel a protective stop.
    """
    def __init__(self, run_dir=None, capacity=2):
        configured = os.environ.get('DEMO_RUN_DIR') if run_dir is None else run_dir
        self.root = None if not configured else Path(configured).expanduser().absolute()/'navigation_events'
        self.pending = queue.Queue(maxsize=capacity)
        self.submitted = self.completed = self.dropped = 0
        self.last = {}
        self.closed = False
        self.thread = threading.Thread(target=self._worker, name='navigation-event-archive', daemon=True)
        self.thread.start()

    def enqueue(self, arrays, metadata):
        if self.closed:
            return None
        self.submitted += 1
        event_id = f'event_{self.submitted:06d}'
        try:
            self.pending.put_nowait((event_id, arrays, metadata))
        except queue.Full:
            self.dropped += 1
            self.last = dict(event_id=event_id, request_id=metadata.get('edge_request_id',metadata.get('request_id')),
                             error='event archive queue full; protective stop retained')
            return None
        return event_id

    def status(self, request_id=_UNSCOPED):
        last = self.last
        if request_id is not _UNSCOPED and last.get('request_id') != request_id:
            last = {}
        return dict(submitted=self.submitted, completed=self.completed, dropped=self.dropped,
                    pending=self.pending.qsize(), counter_scope='node lifetime', **last)

    def _write(self, event_id, arrays, metadata):
        if self.root is None:
            # Captured by this worker once; never fall back to a shared Log path.
            self.root = Path(tempfile.mkdtemp(prefix='demo-navigation-events-'))
        self.root.mkdir(parents=True, exist_ok=True)
        # Unique names also isolate a controller restarted in the same run.
        with tempfile.NamedTemporaryFile(prefix=event_id+'_', suffix='.tmp.npz',
                                         dir=self.root, delete=False) as stream:
            cloud_tmp = Path(stream.name)
        cloud_path = cloud_tmp.with_suffix('').with_suffix('.npz')
        json_path = cloud_path.with_suffix('.json')
        json_tmp = json_path.with_suffix('.tmp.json')
        try:
            np.savez_compressed(cloud_tmp, **arrays)
            document = json_value(dict(schema=1, scope='actual native guard input; diagnostics only',
                event_id=event_id, cloud_file=cloud_path.name,
                cloud_sha256=hashlib.sha256(cloud_tmp.read_bytes()).hexdigest(), **metadata))
            json_tmp.write_text(json.dumps(document, ensure_ascii=False, allow_nan=False)+'\n')
            cloud_tmp.replace(cloud_path)
            json_tmp.replace(json_path)  # JSON is the event commit marker.
            return str(json_path)
        finally:
            cloud_tmp.unlink(missing_ok=True)
            json_tmp.unlink(missing_ok=True)

    def _worker(self):
        while True:
            item = self.pending.get()
            try:
                if item is None:
                    return
                event_id, arrays, metadata = item
                try:
                    path = self._write(event_id, arrays, metadata)
                    self.completed += 1
                    self.last = dict(event_id=event_id, request_id=metadata.get('edge_request_id',metadata.get('request_id')),
                                     path=path, error=None)
                except Exception as exc:
                    self.last = dict(event_id=event_id, request_id=metadata.get('edge_request_id',metadata.get('request_id')),
                                     error=f'{type(exc).__name__}: {exc}')
            finally:
                self.pending.task_done()

    def close(self, timeout=2.):
        self.closed = True
        try:
            self.pending.put_nowait(None)
        except queue.Full:
            # Never block node shutdown; owned worker is a daemon.
            return False
        self.thread.join(timeout)
        return not self.thread.is_alive()
