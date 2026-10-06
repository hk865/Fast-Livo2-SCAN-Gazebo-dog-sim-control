#!/usr/bin/env python3
"""Read-only local dashboard for recorded Teacher Gazebo tests.

No ROS publisher, simulation control, or training process is created here.
"""
from __future__ import annotations

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlsplit


ROOT = Path(__file__).resolve().parents[1]


def finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def vector(value, size=3):
    if isinstance(value, dict):
        if isinstance(value.get('linear'), dict):
            linear, angular = value['linear'], value.get('angular', {})
            value = [linear.get('x'), linear.get('y'), angular.get('z')]
        else:
            keys = ('x', 'y', 'z') if any(k in value for k in ('x', 'y', 'z')) else ('vx', 'vy', 'wz')
            value = [value.get(k) for k in keys]
    if not isinstance(value, (list, tuple)) or len(value) < size:
        return None
    values = [finite_number(v) for v in value[:size]]
    return values if all(v is not None for v in values) else None


def first(row, names):
    for name in names:
        value = row.get(name)
        if value is not None:
            return value
    return None


def normalize(row):
    """Keep only display fields; never synthesize missing measurements."""
    if not isinstance(row, dict):
        return None
    stamp = finite_number(first(row, ('sim_time', 'sim', 't', 'time', 'stamp', 'clock')))
    if stamp is None and finite_number(row.get('stamp_ns')) is not None:
        stamp = float(row['stamp_ns']) / 1e9
    if stamp is None:
        return None
    command = vector(first(row, ('command', 'cmd', 'cmd_vel', 'requested', 'requested_command')))
    measured = vector(first(row, ('measured', 'measured_velocity', 'velocity_body', 'actual_velocity')))
    if measured is None:
        linear = vector(first(row, ('base_lin_vel', 'body_lin_vel', 'linear_velocity_body')))
        angular = vector(first(row, ('base_ang_vel', 'body_ang_vel', 'angular_velocity_body')))
        if linear is not None and angular is not None:
            measured = [linear[0], linear[1], angular[2]]
    position = vector(first(row, ('position', 'p', 'base_position', 'truth_position', 'ground_truth_position')))
    if position is None and isinstance(row.get('pose'), dict):
        position = vector(row['pose'].get('position'))
    rpy = vector(first(row, ('rpy', 'roll_pitch_yaw', 'attitude')))
    if rpy is None:
        roll, pitch, yaw = (finite_number(row.get(k)) for k in ('roll', 'pitch', 'yaw'))
        if None not in (roll, pitch, yaw):
            rpy = [roll, pitch, yaw]
    return {
        't': stamp, 'command': command, 'measured': measured, 'position': position,
        'rpy': rpy, 'contacts': first(row, ('contacts', 'foot_contacts', 'contact')),
        'state': first(row, ('state', 'mode', 'controller_state')),
        'test': first(row, ('test', 'test_name', 'case', 'phase')),
        'reason': first(row, ('reason', 'failure', 'error', 'fault')),
    }


def read_json(path):
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {'data': value}
    except (OSError, ValueError):
        return {}


def safe_json(value):
    """Nonfinite diagnostic values stay missing instead of breaking the UI."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    return value


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.entries = {}

    def read(self, path):
        with self.lock:
            try:
                stat = path.stat()
            except OSError:
                return {'rows': [], 'trajectory': [], 'latest': None, 'records': 0,
                        'invalid_records': 0, 'age_s': None, 'loading': False}
            key = str(path)
            entry = self.entries.get(key)
            identity = (stat.st_dev, stat.st_ino)
            if entry is None or entry['identity'] != identity or stat.st_size < entry['offset']:
                entry = {'identity': identity, 'offset': 0, 'partial': b'',
                         'rows': deque(maxlen=6000), 'trajectory': [], 'records': 0,
                         'invalid_records': 0, 'latest': None}
                self.entries[key] = entry
                # Bound memory when the user browses many archived runs.
                while len(self.entries) > 6:
                    self.entries.pop(next(iter(self.entries)))
            try:
                with path.open('rb') as stream:
                    stream.seek(entry['offset'])
                    chunk = stream.read(8 * 1024 * 1024)
                    entry['offset'] = stream.tell()
            except OSError:
                chunk = b''
            lines = (entry['partial'] + chunk).split(b'\n')
            entry['partial'] = lines.pop()
            # A partially written line is completed on the next poll.
            if len(entry['partial']) > 8 * 1024 * 1024:
                entry['partial'] = b''
                entry['invalid_records'] += 1
            for line in lines:
                if not line.strip():
                    continue
                try:
                    row = normalize(json.loads(line))
                except (ValueError, UnicodeDecodeError):
                    row = None
                if row is None:
                    entry['invalid_records'] += 1
                    continue
                entry['records'] += 1
                entry['latest'] = row
                entry['rows'].append(row)
                position = row['position']
                if position is not None:
                    point = [position[0], position[1], row['t']]
                    path_rows = entry['trajectory']
                    if not path_rows or math.hypot(point[0] - path_rows[-1][0], point[1] - path_rows[-1][1]) >= .015:
                        path_rows.append(point)
                    if len(path_rows) > 4000:
                        entry['trajectory'] = path_rows[::2]
            rows = list(entry['rows'])
            stride = max(1, math.ceil(len(rows) / 2000))
            displayed = rows[::stride]
            if rows and (not displayed or displayed[-1] is not rows[-1]):
                displayed.append(rows[-1])
            return {'rows': displayed, 'trajectory': entry['trajectory'], 'latest': entry['latest'],
                    'records': entry['records'], 'invalid_records': entry['invalid_records'],
                'age_s': None if entry['offset'] < stat.st_size else max(0., time.time() - stat.st_mtime),
                    'loading': entry['offset'] < stat.st_size}


class Dashboard:
    def __init__(self, runs):
        self.runs = runs.resolve()
        self.telemetry = Telemetry()
        self.plan_cache = {}

    def navigation_view(self, directory):
        """Only measured SLAM and an archived actual SCAN payload are drawn."""
        poses = self.telemetry.read(directory / 'navigation_slam_poses.jsonl')
        request = read_json(directory / 'navigation_request.json')
        result = {'poses': poses, 'request': request,
                  'state': read_json(directory / 'navigation_status.json'),
                  'scan': {}, 'source': 'Actual sensor SLAM in camera_init; no Gazebo truth input'}
        files = sorted((directory / 'navigation_trajectories').glob('*_trajectory_*.json'))
        if not files:
            return result
        record = files[-1]
        try:
            key = (str(record), record.stat().st_mtime_ns)
            if key not in self.plan_cache:
                import numpy as np
                data = read_json(record)
                archive = self.inside(record.parent / data['array_file'])
                if archive is None or hashlib.sha256(archive.read_bytes()).hexdigest() != data['array_sha256']:
                    raise ValueError('Actual SCAN array hash mismatch')
                with np.load(archive, allow_pickle=False) as saved:
                    samples = saved['samples']
                    if samples.ndim != 2 or samples.shape[1] != 3 or not np.isfinite(samples).all():
                        raise ValueError('Actual SCAN samples malformed')
                    stride = max(1, math.ceil(len(samples) / 2000))
                    points = samples[::stride].tolist()
                self.plan_cache[key] = {'trajectory_id': data['trajectory_id'],
                    'waypoint_index': data['waypoint_index'], 'points': points,
                    'array_sha256': data['array_sha256'], 'source': data['source'],
                    'file': record.name, 'hash_verified': True}
                while len(self.plan_cache) > 6:
                    self.plan_cache.pop(next(iter(self.plan_cache)))
            result['scan'] = self.plan_cache[key]
        except (OSError, ValueError, KeyError, ImportError):
            result['scan'] = {'error': 'Actual SCAN payload not available or hash not verified'}
        return result

    def inside(self, path):
        try:
            resolved = path.resolve()
            resolved.relative_to(self.runs)
            return resolved
        except (ValueError, OSError):
            return None

    def directories(self):
        try:
            children = list(self.runs.iterdir())
        except OSError:
            return []
        result = []
        for child in children:
            resolved = self.inside(child)
            if child.name != 'latest' and resolved is not None and resolved.is_dir():
                try:
                    modified = child.stat().st_mtime
                    summary = read_json(child / 'summary.json')
                    state = read_json(child / 'state.json')
                    result.append({'id': child.name, 'modified_at': modified,
                                   'status': summary.get('status', state.get('state', state.get('status'))),
                                   'tests': len(summary.get('tests', [])) if isinstance(summary.get('tests'), (dict, list)) else 0})
                except OSError:
                    continue
        return sorted(result, key=lambda item: item['modified_at'], reverse=True)

    def resolve(self, run):
        if run == 'latest':
            latest = self.inside(self.runs / 'latest')
            if latest is not None and latest.is_dir():
                return latest
            directories = self.directories()
            return self.inside(self.runs / directories[0]['id']) if directories else None
        if not run or run in {'.', '..'} or '/' in run or '\\' in run:
            return None
        path = self.inside(self.runs / run)
        return path if path is not None and path.is_dir() else None

    @staticmethod
    def frame(directory):
        candidates = [p for p in (directory / 'frame.jpg', directory / 'frame.png') if p.is_file()]
        return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None

    def snapshot(self, run):
        directory = self.resolve(run)
        if directory is None:
            return {'available': False, 'run_id': None, 'state': {}, 'summary': {},
                    'telemetry': {'rows': [], 'trajectory': [], 'latest': None}, 'frame': {'available': False},
                    'acceptance': read_json(self.runs / 'acceptance.json')}
        state_path = directory / 'state.json'
        frame = self.frame(directory)
        vehicle_frame = self.frame(directory / 'vehicle_rgb')
        try:
            state_age = max(0., time.time() - state_path.stat().st_mtime)
        except OSError:
            state_age = None
        original_functional = read_json(directory / 'summary_functional.json')
        reproduced = read_json(directory / 'summary_functional.reproduced.json')
        correction = read_json(directory / 'provenance_correction.json')
        corrected = bool(reproduced and correction.get('status') == 'reproduced_identical'
            and correction.get('all_checks_identical') is True and correction.get('all_metrics_identical') is True
            and original_functional.get('checks') == reproduced.get('checks')
            and original_functional.get('metrics') == reproduced.get('metrics'))
        ramp_original = read_json(directory / 'summary_full_ramp.json')
        stop_addendum = read_json(directory / 'full_ramp_stop_status_addendum.json')
        ramp_display = ramp_original
        if ramp_original and stop_addendum and stop_addendum.get('original_summary_sha256') == hashlib.sha256((directory / 'summary_full_ramp.json').read_bytes()).hexdigest():
            ramp_display = {**ramp_original, 'checks': {**ramp_original.get('checks', {}),
                'teacher_stop_on_destination_landing': stop_addendum['teacher_stop_on_destination_landing']},
                'display_note': 'Parking status uses separately archived clarification; original full-ramp receipt retained'}
        return {'available': True, 'run_id': directory.name, 'selected': run,
                'state': read_json(state_path), 'state_age_s': state_age,
                'summary': read_json(directory / 'summary.json'),
                'step_functional': reproduced if corrected else original_functional,
                'step_functional_original': original_functional,
                'step_provenance': correction,
                'step_functional_source': 'summary_functional.reproduced.json' if corrected else 'summary_functional.json',
                'step_safety': read_json(directory / 'supplemental_native_safety.json'),
                'full_ramp': ramp_display,
                'full_ramp_original': ramp_original,
                'full_ramp_stop_addendum': stop_addendum,
                'navigation_summary': read_json(directory / 'summary_navigation_independent.json'),
                'targeted_ttl': read_json(directory / 'summary_ttl_dropout_independent.json'),
                'dynamic_obstacle': read_json(directory / 'summary_dynamic_obstacle_independent.json'),
                'runtime_processes': read_json(directory / 'runtime_manifest.json').get('owned_processes', []),
                'navigation': self.navigation_view(directory),
                'replay': {'available': (directory / 'frame_replay.mp4').is_file() or (directory / 'frame_replay.gif').is_file(),
                           'mp4': (directory / 'frame_replay.mp4').is_file(),
                           'gif': (directory / 'frame_replay.gif').is_file(),
                           'manifest': read_json(directory / 'frame_replay_manifest.json') or read_json(directory / 'step_retest_evidence_manifest.json'),
                           'scope': 'Archived actual Gazebo RGB in sensor timestamp order; no interpolated or synthesized frames'},
                'step_plot': (directory / 'step_functional.png').is_file(),
                'acceptance': read_json(self.runs / 'acceptance.json'),
                'current_validation': read_json(self.runs.parent / 'current_status.json'),
                'telemetry': self.telemetry.read(directory / 'telemetry.jsonl'),
                'frame': {'available': frame is not None,
                          'age_s': max(0., time.time() - frame.stat().st_mtime) if frame else None,
                          'filename': frame.name if frame else None},
                'camera_info': {kind: read_json(directory / 'camera_info' / (kind + '.json')) for kind in ('overview','vehicle')},
                'vehicle_frame': {'available': vehicle_frame is not None,
                                  'age_s': max(0., time.time() - vehicle_frame.stat().st_mtime) if vehicle_frame else None,
                                  'filename': vehicle_frame.name if vehicle_frame else None},
                'diagnostic_source': 'Gazebo simulation truth for motion diagnostics; not a navigation input',
                'read_only': True}


def handler_for(dashboard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            # Polling should not bury simulation diagnostics in HTTP logs.
            if len(args) < 2 or str(args[1]) not in {'200', '304'}:
                super().log_message(format, *args)

        def send(self, data, content_type='application/json; charset=utf-8', status=200, headers=None):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def json(self, value, status=200):
            data = json.dumps(safe_json(value), ensure_ascii=False, allow_nan=False, default=str).encode('utf-8')
            self.send(data, status=status)

        def media(self, path, content_type):
            if path is None or dashboard.inside(path) is None or not path.is_file():
                self.json({'error': 'No recorded media'}, 404)
                return
            try:
                size = path.stat().st_size
                start, end, status = 0, size - 1, 200
                headers = {'Accept-Ranges': 'bytes'}
                value = self.headers.get('Range')
                if value:
                    # One byte range is enough for read-only HTML video seeking.
                    import re
                    match = re.fullmatch(r'bytes=(\d*)-(\d*)', value.strip())
                    if not match or not any(match.groups()):
                        self.send(b'', content_type, 416, {'Content-Range': f'bytes */{size}'})
                        return
                    first, last = match.groups()
                    if first:
                        start = int(first); end = min(size - 1, int(last)) if last else size - 1
                    else:
                        start = max(0, size - int(last))
                    if start > end or start >= size:
                        self.send(b'', content_type, 416, {'Content-Range': f'bytes */{size}'})
                        return
                    status = 206; headers['Content-Range'] = f'bytes {start}-{end}/{size}'
                with path.open('rb') as stream:
                    stream.seek(start); payload = stream.read(end - start + 1)
                self.send(payload, content_type, status, headers)
            except OSError:
                self.json({'error': 'Recorded media changed while reading'}, 404)

        def do_GET(self):
            parsed = urlsplit(self.path)
            run = parse_qs(parsed.query).get('run', ['latest'])[0]
            if parsed.path in {'/', '/index.html'}:
                try:
                    self.send((ROOT / 'web/index.html').read_bytes(), 'text/html; charset=utf-8')
                except OSError:
                    self.json({'error': 'Dashboard page is unavailable'}, 404)
            elif parsed.path == '/api/runs':
                self.json({'runs': dashboard.directories(), 'read_only': True})
            elif parsed.path == '/api/run':
                self.json(dashboard.snapshot(run))
            elif parsed.path == '/api/acceptance':
                self.json(read_json(dashboard.runs / 'acceptance.json'))
            elif parsed.path == '/api/frame':
                directory = dashboard.resolve(run)
                camera = parse_qs(parsed.query).get('camera', ['overview'])[0]
                if camera not in ('overview','vehicle'):
                    self.json({'error': 'Unknown camera'}, 400)
                    return
                frame = dashboard.frame(directory / 'vehicle_rgb' if camera == 'vehicle' else directory) if directory else None
                if frame is None or dashboard.inside(frame) is None:
                    self.json({'error': 'No recorded Gazebo frame'}, 404)
                    return
                try:
                    self.send(frame.read_bytes(), 'image/png' if frame.suffix == '.png' else 'image/jpeg')
                except OSError:
                    self.json({'error': 'Frame changed while reading'}, 404)
            elif parsed.path == '/api/replay':
                directory = dashboard.resolve(run)
                kind = parse_qs(parsed.query).get('kind', ['mp4'])[0]
                if kind not in {'mp4', 'gif'}:
                    self.json({'error': 'Unsupported recorded media type'}, 400)
                    return
                self.media(directory / ('frame_replay.' + kind) if directory else None, 'video/mp4' if kind == 'mp4' else 'image/gif')
            elif parsed.path == '/api/step_plot':
                directory = dashboard.resolve(run)
                self.media(directory / 'step_functional.png' if directory else None, 'image/png')
            else:
                self.json({'error': 'Not found'}, 404)

        def do_POST(self):
            self.json({'error': 'This dashboard is read-only'}, 405)

        do_PUT = do_POST
        do_DELETE = do_POST
        do_PATCH = do_POST
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8768)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--runs', type=Path, default=ROOT / 'runs')
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), handler_for(Dashboard(args.runs)))
    print(f'Teacher read-only dashboard: http://{args.host}:{args.port}/', flush=True)
    print(f'Recordings: {args.runs.resolve()}', flush=True)
    try:
        server.serve_forever(poll_interval=.3)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
