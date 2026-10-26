#!/usr/bin/env python3
"""Single-run read-only R85 file dashboard. No ROS, control or pool discovery."""
import argparse
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlsplit


def vector(v, size):
    return (isinstance(v, list) and len(v) == size and
            all(type(x) in (int, float) and math.isfinite(x) for x in v))


class Journal:
    def __init__(self, path, capacity):
        self.path, self.capacity = path, capacity
        self.root = path.parent.resolve()
        self.offset, self.identity, self.rows = 0, None, deque(maxlen=capacity)

    def read(self):
        try:
            if self.path.is_symlink() or self.path.resolve().parent != self.root:
                return []
            st = self.path.stat()
            identity = (st.st_dev, st.st_ino)
            if identity != self.identity or st.st_size < self.offset:
                self.offset, self.rows, self.identity = 0, deque(maxlen=self.capacity), identity
            with self.path.open('rb') as stream:
                stream.seek(self.offset)
                while True:
                    raw = stream.readline(2 * 1024 * 1024)
                    if not raw or not raw.endswith(b'\n'):
                        break
                    self.offset = stream.tell()
                    try:
                        self.rows.append(json.loads(raw))
                    except (ValueError, UnicodeError):
                        continue
        except OSError:
            pass
        return list(self.rows)


class View:
    def __init__(self, run):
        if not run.is_absolute() or not run.is_dir() or run.is_symlink():
            raise ValueError('--run must name one absolute existing directory')
        self.run, self.lock = run.resolve(), threading.Lock()
        self.poses = Journal(self.file('navigation_slam_poses.jsonl'), 20000)
        self.paths = Journal(self.file('navigation_cascade_paths.jsonl'), 64)

    def file(self, name):
        p = self.run / name
        if not p.resolve().is_relative_to(self.run) or p.is_symlink():
            raise ValueError('Run file escapes selected directory')
        return p

    def read(self, name):
        try:
            value = json.loads(self.file(name).read_text())
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    def plan(self, nav):
        for row in reversed(self.paths.read()):
            if (row.get('trajectory_id') != nav.get('accepted_trajectory_id') or
                    row.get('frame_id') != 'camera_init' or row.get('navigation_ground_truth_used') is not False):
                continue
            points = row.get('points_xyz', [])
            if not points or not all(vector(p, 3) for p in points):
                return []
            # Actual accepted-controller journal; verify archived point bytes.
            import struct
            raw = b''.join(struct.pack('<3d', *p) for p in points)
            if hashlib.sha256(raw).hexdigest() != row.get('points_float64_sha256'):
                return []
            stride = max(1, math.ceil(len(points) / 2000))
            return points[::stride]
        return []

    def state(self):
        with self.lock:
            nav, result = self.read('navigation_status.json'), self.read('r85_task_result.json')
            task = self.read('r85_task_status.json')
            metadata = self.read('map_metadata.json')
            rows = [r for r in self.poses.read() if r.get('frame_id') == 'camera_init'
                    and vector(r.get('position'), 3) and vector(r.get('quaternion'), 4)]
            last = rows[-1] if rows else {}
            q = last.get('quaternion')
            yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2)) if q else None
            stage = ('completed' if result.get('passed') is True else 'failed' if result.get('passed') is False
                     else 'active' if nav or task else None)
            stride = max(1, math.ceil(len(rows) / 2000))
            age = time.monotonic() - last['received_monotonic_wall'] if last.get('received_monotonic_wall') is not None else None
            frame = self.file('vehicle_rgb/frame.jpg')
            frame_kind = 'vehicle_rgb' if frame.is_file() else 'overview'
            frame = frame if frame.is_file() else self.file('frame.jpg')
            frame_age = max(0, time.time()-frame.stat().st_mtime) if frame.is_file() else None
            return dict(run_id=self.run.name, stage=stage, navigation=nav, acceptance=result or None,
                        message=result.get('failure_reason') or task.get('failure_reason') or nav.get('message') or '等待本轮导航文件',
                        pose=last.get('position'), yaw=yaw, trace=[r['position'] for r in rows[::stride]],
                        trace_note='仅显示最后20000条SLAM记录，最多2000点；降采样不参与验收',
                        planned_path=self.plan(nav), current_goals=nav.get('goals_definitions', []),
                        map=metadata, counts=metadata.get('counts', {}), ages=metadata.get('ages', {}), task=task,
                        slam_wall_age_s=age if age is not None and age >= 0 else None,
                        elapsed_s=(last['callback_ros_clock_ns']/1e9 if 'callback_ros_clock_ns' in last else None),
                        camera=dict(available=frame.is_file(), age=frame_age, kind=frame_kind),
                        display_only=True, navigation_ground_truth_used=False)


def handler(view):
    class Handler(BaseHTTPRequestHandler):
        def send(self, body, mime='application/json', status=200):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path
            try:
                if path == '/':
                    return self.send(Path(__file__).with_name('index.html').read_bytes(), 'text/html; charset=utf-8')
                if path in ('/api/state', '/api/acceptance'):
                    value = view.state() if path.endswith('state') else view.read('r85_task_result.json') or None
                    return self.send(json.dumps(value, allow_nan=False).encode())
                if path == '/api/frame':
                    f = view.file('vehicle_rgb/frame.jpg')
                    return self.send((f if f.is_file() else view.file('frame.jpg')).read_bytes(), 'image/jpeg')
                if path.startswith('/api/map/'):
                    name = path.removeprefix('/api/map/')
                    if name != 'map_metadata.json' and not re.fullmatch(r'colored_map\.[0-9]{6}\.bin', name):
                        return self.send(b'{}', status=404)
                    data = view.file(name).read_bytes()
                    if name.endswith('.bin') and len(data) % 16:
                        return self.send(b'{}', status=503)
                    return self.send(data, 'application/octet-stream' if name.endswith('.bin') else 'application/json')
                return self.send(b'{}', status=404)
            except (OSError, ValueError, KeyError):
                return self.send(b'{"error":"Selected run data unavailable"}', status=503)

        def do_POST(self):
            self.send(b'{"error":"Read only"}', status=405)

        do_PUT = do_DELETE = do_PATCH = do_POST
    return Handler


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8769)
    args = parser.parse_args()
    ThreadingHTTPServer(('127.0.0.1', args.port), handler(View(args.run))).serve_forever()
