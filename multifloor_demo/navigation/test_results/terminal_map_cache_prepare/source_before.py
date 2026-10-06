#!/usr/bin/env python3
"""Local browser gateway and feedback-driven mission coordinator."""
import argparse
from collections import Counter, deque
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import uuid

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Image, Imu, PointCloud2
from ros_gz_interfaces.msg import Contacts

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission.state_machine import Mission, ObstacleEvidence, ACTIVE_STAGES
from mission.route_regions import validate_route_regions
from mission.processes import finish_owned_process
from mission.artifacts import snapshot_sources, snapshot_runtime
from simulation.controller_runtime import controller_manager_library
from slam.heading_alignment import calibrate_scene_heading


class Coordinator(Node):
    def __init__(self):
        super().__init__('multifloor_mission_server')
        self.lock, self.action_lock = threading.RLock(), threading.Lock()
        self.mission = Mission()
        self.proc = None
        self.proc_log = None
        self.stopping = False
        self.run_dir = None
        self.epoch = time.monotonic()
        self.pose, self.yaw, self.truth = None, 0., None
        self.trace, self.truth_trace, self.path = [], [], []
        self.counts, self.last = Counter(), {}
        self.nav, self.map_status, self.obstacle = {}, {}, {}
        self.execution_safety = {}
        self.image_bytes, self.image_at = None, None
        self.obstacle_evidence = ObstacleEvidence()
        self.obstacle_history = []
        self.save_future = None
        self.last_write = 0.
        self.command = [0., 0., 0.]
        self.audit_stream = None
        self.command_stream = None
        self.navigation_stream = None
        self.body_contacts = []
        self.max_tilt = 0.
        self.body_contact_publisher_seen = False
        # Retain the same sensor-time span at either 100 or 1000 Hz. A fixed
        # sample count can discard the IMU pair before a delayed SLAM callback.
        self.imu_attitudes = deque()
        self.heading_pairs = deque(maxlen=15)
        self.heading_error = None
        self.evaluation_started = False
        self.acceptance = None
        self.request_pub = self.create_publisher(String, '/demo/navigation/request', 10)
        self.stop_pub = self.create_publisher(Bool, '/demo/navigation/stop', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/demo/obstacle/enable', 10)
        self.state_pub = self.create_publisher(String, '/demo/mission/state', 10)
        self.map_client = self.create_client(Trigger, '/demo/slam/save_map')
        truth_recording_qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT,
                                        history=HistoryPolicy.KEEP_LAST)
        self.subs = [
            self.create_subscription(Odometry, '/demo/slam/body_odom', self.on_pose, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/demo/ground_truth', self.on_truth, truth_recording_qos),
            self.create_subscription(Contacts, '/demo/body_contacts', self.on_contacts, qos_profile_sensor_data),
            self.create_subscription(String, '/demo/navigation/status', self.on_nav, 10),
            self.create_subscription(String, '/demo/control/safety', self.on_control_safety, 10),
            self.create_subscription(RosPath, '/demo/navigation/path', self.on_path, 10),
            self.create_subscription(String, '/demo/slam/map_status', self.on_map, 10),
            self.create_subscription(String, '/demo/obstacle/state', self.on_obstacle, 10),
            self.create_subscription(Twist, '/demo/cmd_vel', self.on_command, 10),
            self.create_subscription(PointCloud2, '/livox/lidar', lambda m: self.touch('lidar'), qos_profile_sensor_data),
            self.create_subscription(Imu, '/livox/imu', self.on_imu, qos_profile_sensor_data),
            self.create_subscription(Image, '/camera/image_color', self.on_image, qos_profile_sensor_data),
        ]
        self.timer = self.create_timer(.2, self.tick)

    def elapsed(self):
        if self.mission.stage == 'idle':
            return 0.
        if self.mission.stage in {'completed', 'failed', 'stopped'}:
            return self.mission.stage_started
        return time.monotonic()-self.epoch

    def touch(self, name):
        with self.lock:
            self.counts[name] += 1
            self.last[name] = time.monotonic()

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = [p.x, p.y, p.z]
        if not all(math.isfinite(x) for x in pose):
            return
        with self.lock:
            self.touch('slam_pose')
            self.pose = pose
            self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.audit_pose('slam', msg)
            if self.mission.stage == 'waiting_sensors' and self.mission.heading_alignment is None and self.imu_attitudes:
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                imu_stamp, imu_q = min(self.imu_attitudes, key=lambda row: abs(row[0]-stamp))
                if abs(stamp-imu_stamp) <= .02 and (not self.heading_pairs or stamp > self.heading_pairs[-1]['slam_stamp']):
                    self.heading_pairs.append({'slam_stamp': stamp, 'imu_stamp': imu_stamp,
                        'imu_quaternion': imu_q, 'slam_body_quaternion': [q.x,q.y,q.z,q.w]})
                reference = self.mission.scenario.get('sensors', {}).get('imu', {}).get('orientation_reference')
                if reference and len(self.heading_pairs) >= 10:
                    try:
                        self.mission.heading_alignment = calibrate_scene_heading(list(self.heading_pairs),
                            imu_reference_world_quaternion=reference['world_quaternion'],
                            body_imu_quaternion=reference['body_imu_quaternion'], reference_description=reference['description'])
                        self.mission.heading_alignment['attitude_pairs'] = list(self.heading_pairs)
                        self.heading_error = None
                    except (ValueError, KeyError) as exc:
                        self.heading_error = str(exc)
            if self.run_dir and (not self.trace or math.dist(pose, self.trace[-1]) > .05):
                self.trace.append(pose)

    def on_imu(self, msg):
        with self.lock:
            self.touch('imu')
            q = msg.orientation
            values = [q.x, q.y, q.z, q.w]
            if msg.orientation_covariance[0] != -1 and all(map(math.isfinite, values)) and sum(v*v for v in values) > .5:
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if not self.imu_attitudes or stamp > self.imu_attitudes[-1][0]:
                    self.imu_attitudes.append((stamp, values))
                    while self.imu_attitudes[0][0] < stamp-4.0:
                        self.imu_attitudes.popleft()

    def on_truth(self, msg):
        # Independent validation channel. Never published as navigation input.
        with self.lock:
            self.touch('ground_truth')
            p = msg.pose.pose.position
            self.truth = [p.x, p.y, p.z]
            self.audit_pose('truth', msg)
            q = msg.pose.pose.orientation
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
            if self.mission.stage in {'exploring', 'returning', 'navigating'}:
                self.max_tilt = max(self.max_tilt, abs(roll), abs(pitch))
                if max(abs(roll), abs(pitch)) > .75:
                    self.apply(self.mission.fail(self.elapsed(), '独立物理验收发现机身倾覆，停止任务。'))
            if self.run_dir and (not self.truth_trace or math.dist(self.truth, self.truth_trace[-1]) > .05):
                self.truth_trace.append(self.truth)

    def audit_pose(self, source, msg):
        if self.audit_stream:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            record = {'source': source, 'stamp': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                      'stamp_ns': int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec),
                      'p': [p.x, p.y, p.z], 'q': [q.x, q.y, q.z, q.w], 'stage': self.mission.stage}
            self.audit_stream.write(json.dumps(record)+'\n')

    def on_contacts(self, msg):
        with self.lock:
            self.touch('body_contact_sensor')
            if self.mission.stage not in {'exploring', 'returning', 'navigating'}:
                return
            relevant = [c for c in msg.contacts if not c.depths or max(c.depths) > .001]
            if relevant:
                self.body_contacts.append({'elapsed_s': self.elapsed(), 'pairs': [[c.collision1.name, c.collision2.name] for c in relevant]})
                self.apply(self.mission.fail(self.elapsed(), '独立接触传感器检测到机身碰撞，任务停止。'))

    def on_path(self, msg):
        with self.lock:
            self.path = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
            self.touch('path')

    def on_nav(self, msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.touch('navigation')
            if self.navigation_stream:
                self.navigation_stream.write(json.dumps({'stage': self.mission.stage,
                    'current_request': self.mission.current_request, 'elapsed_s': self.elapsed(),
                    'status': status}, ensure_ascii=False)+'\n')
            self.nav = status
            if self.mission.stage == 'navigating' and status.get('request_id') == self.mission.current_request:
                self.obstacle_evidence.observe(bool(status.get('obstacle_hold')),
                                               math.hypot(*self.command[:2]), self.obstacle)
            status['dynamic_obstacle_verified'] = self.obstacle_evidence.verified
            self.apply(self.mission.navigation_result(status, self.pose, self.elapsed()))

    def on_control_safety(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get('state') not in {'hold', 'ready', 'failed'}:
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.touch('control_safety')
            self.execution_safety = data
            if data['state'] == 'failed' and self.mission.stage in ACTIVE_STAGES:
                reason = str(data.get('reason', 'unknown_control_failure'))
                self.apply(self.mission.fail(self.elapsed(), '底层控制已锁定失败，停止任务：'+reason))

    def on_map(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            if self.run_dir and data.get('run_id') not in (None, self.mission.run_id):
                return
            self.map_status = data
            self.touch('map')

    def on_obstacle(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.obstacle = data
            self.touch('obstacle')
            if self.mission.stage == 'navigating' and data.get('active'):
                self.obstacle_history.append({**data, 'elapsed_s': self.elapsed()})

    def on_command(self, msg):
        with self.lock:
            self.command = [msg.linear.x, msg.linear.y, msg.angular.z]
            if self.command_stream:
                self.command_stream.write(json.dumps({'elapsed_s': self.elapsed(), 'stage': self.mission.stage,
                    'command': self.command, 'slam_pose': self.pose, 'slam_yaw': self.yaw,
                    'truth_pose': self.truth, 'waypoint_index': self.nav.get('waypoint_index'),
                    'alignment_hold': self.nav.get('alignment_hold'),
                    'obstacle_hold': self.nav.get('obstacle_hold')})+'\n')

    def on_image(self, msg):
        self.touch('camera')
        now = time.monotonic()
        if self.image_at and now-self.image_at < .25:
            return
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
        if channels is None or not msg.width or not msg.height:
            return
        try:
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            arr = rows[:, :msg.width*channels].reshape(msg.height, msg.width, channels)
            if msg.encoding in ('bgr8', 'bgra8'):
                arr = arr[:, :, [2, 1, 0]]
            elif channels == 4:
                arr = arr[:, :, :3]
            elif channels == 1:
                arr = arr[:, :, 0]
            image = PILImage.fromarray(arr)
            image.thumbnail((800, 600))
            out = io.BytesIO()
            image.save(out, 'JPEG', quality=78)
            with self.lock:
                self.image_bytes, self.image_at = out.getvalue(), now
        except (ValueError, TypeError):
            return

    def apply(self, actions):
        for action in actions:
            if action['kind'] == 'route':
                self.request_pub.publish(String(data=json.dumps({k: v for k, v in action.items() if k != 'kind'})))
            elif action['kind'] == 'stop_navigation':
                self.stop_pub.publish(Bool(data=True))
            elif action['kind'] == 'obstacle':
                self.obstacle_pub.publish(Bool(data=action['enabled']))
            elif action['kind'] == 'save_map':
                if self.map_client.service_is_ready():
                    self.save_future = self.map_client.call_async(Trigger.Request())
                else:
                    self.apply(self.mission.map_saved(False, 0, self.elapsed(), '地图保存服务未就绪。'))

    def tick(self):
        with self.lock:
            now = time.monotonic()
            if self.mission.stage in {'exploring', 'returning', 'navigating'}:
                self.body_contact_publisher_seen |= self.count_publishers('/demo/body_contacts') > 0
            ages = {k: now-v for k, v in self.last.items()}
            sensors_ok = all(ages.get(k, 1e9) < limit for k, limit in [('slam_pose', 3.), ('lidar', 3.), ('imu', 2.), ('camera', 4.), ('navigation', 5.)])
            if self.proc is not None and self.proc.poll() is not None and self.mission.stage in ACTIVE_STAGES:
                self.apply(self.mission.fail(self.elapsed(), f'仿真进程退出，退出码 {self.proc.returncode}。'))
            self.apply(self.mission.tick(self.elapsed(), sensors_ok, self.pose, self.map_status.get('point_count', 0)))
            if self.mission.stage == 'failed' and self.proc is not None and self.proc.poll() is None and not self.stopping:
                self.obstacle_pub.publish(Bool(data=False))
                self.stopping = True
                self.proc.send_signal(signal.SIGINT)
                threading.Thread(target=self.finish_stop, args=(self.proc,), daemon=True).start()
            if self.save_future is not None and self.save_future.done():
                future, self.save_future = self.save_future, None
                try:
                    result = future.result()
                    self.apply(self.mission.map_saved(result.success, self.map_status.get('point_count', 0), self.elapsed(), result.message))
                except Exception as exc:
                    self.apply(self.mission.map_saved(False, 0, self.elapsed(), str(exc)))
            if now-self.last_write > 1.:
                state = self.snapshot()
                self.state_pub.publish(String(data=json.dumps({k: v for k, v in state.items() if k not in ('trace', 'planned_path')}, ensure_ascii=False)))
                if self.run_dir:
                    tmp = self.run_dir/'mission.tmp'
                    tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2))
                    tmp.replace(self.run_dir/'mission.json')
                    if self.audit_stream:
                        self.audit_stream.flush()
                    if self.command_stream:
                        self.command_stream.flush()
                    if self.navigation_stream:
                        self.navigation_stream.flush()
                self.last_write = now
                if self.run_dir and self.mission.stage in {'completed', 'failed'} and not self.stopping and not self.evaluation_started:
                    self.evaluation_started = True
                    threading.Thread(target=self.evaluate_run, args=(self.run_dir,), daemon=True).start()

    def evaluate_run(self, run_dir):
        try:
            with (run_dir/'evaluation.log').open('w') as output:
                subprocess.run([sys.executable, str(ROOT/'scripts/evaluate_run.py'), str(run_dir)],
                    stdout=output, stderr=subprocess.STDOUT, timeout=120, check=False)
            result = json.loads((run_dir/'acceptance.json').read_text())
        except Exception as exc:
            result = {'passed': False, 'error': str(exc)}
        with self.lock:
            if self.run_dir == run_dir:
                self.acceptance = result

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            state = self.mission.snapshot()
            state.update({'process_running': self.proc is not None and self.proc.poll() is None,
                          'stopping': self.stopping, 'pose': self.pose, 'yaw': self.yaw,
                          'trace': self.trace, 'planned_path': self.path,
                          'navigation': dict(self.nav), 'map': dict(self.map_status),
                          'obstacle': dict(self.obstacle), 'execution_safety': dict(self.execution_safety),
                          'counts': dict(self.counts),
                          'ages': {k: round(now-v, 3) for k, v in self.last.items()},
                          'elapsed_s': round(self.elapsed(), 1), 'run_directory': str(self.run_dir) if self.run_dir else None,
                          'camera': {'available': self.image_bytes is not None, 'age': now-self.image_at if self.image_at else None},
                          'validation': {'ground_truth_samples': len(self.truth_trace), 'ground_truth_pose': self.truth,
                                         'obstacle_hold_events': self.obstacle_evidence.hold_events,
                                         'resumed_after_hold': self.obstacle_evidence.resumed,
                                         'obstacle_history': list(self.obstacle_history),
                                         'body_contact_events': list(self.body_contacts), 'max_abs_tilt_rad': self.max_tilt,
                                         'body_contact_publisher_present': self.body_contact_publisher_seen},
                          'simulation': self.mission.scenario.get('simulation', {}),
                          'heading_calibration_error': self.heading_error,
                          'acceptance': self.acceptance,
                          'evaluation_started': self.evaluation_started,
                          'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID', 72))})
            return state

    def start_run(self):
        with self.action_lock, self.lock:
            if self.stopping or (self.proc is not None and self.proc.poll() is None):
                return 409, {'error': '已有仿真运行，请先停止。'}
            scenario_path = ROOT/'simulation/scenario.json'
            launcher = ROOT/'scripts/run.sh'
            if not scenario_path.exists() or not launcher.exists():
                return 409, {'error': '仿真模块尚未准备就绪。'}
            scenario = json.loads(scenario_path.read_text())
            for name in ('exploration', 'return_origin', 'navigation_f1_f3'):
                points = scenario.get(name)
                if not isinstance(points, list) or not points or any(not isinstance(p, list) or len(p) != 3 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in p) for p in points):
                    return 400, {'error': f'场景缺少有效 {name} 航点。'}
            try:
                validate_route_regions(scenario)
            except (ValueError, TypeError, KeyError) as exc:
                return 400, {'error': '场景目标区域无效：'+str(exc)}
            try:
                controller_manager_library()
            except RuntimeError as exc:
                return 409, {'error': str(exc)}
            run_id = time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
            self.run_dir = ROOT/'runs'/run_id
            self.run_dir.mkdir(parents=True)
            snapshot_sources(ROOT, self.run_dir)
            snapshot_runtime(self.run_dir, control_timing_enabled=True)
            if self.audit_stream:
                self.audit_stream.close()
            if self.command_stream:
                self.command_stream.close()
            if self.navigation_stream:
                self.navigation_stream.close()
            self.audit_stream = (self.run_dir/'pose_audit.jsonl').open('w')
            self.command_stream = (self.run_dir/'control_audit.jsonl').open('w')
            self.navigation_stream = (self.run_dir/'navigation_audit.jsonl').open('w')
            (self.run_dir/'scenario.json').write_text(json.dumps(scenario, ensure_ascii=False, indent=2))
            self.epoch = time.monotonic()
            self.mission.start(run_id, scenario)
            self.pose, self.truth, self.image_bytes, self.image_at = None, None, None, None
            self.trace, self.truth_trace, self.path = [], [], []
            self.counts, self.last = Counter(), {}
            self.nav, self.map_status, self.obstacle = {}, {}, {}
            self.execution_safety = {}
            self.obstacle_evidence, self.obstacle_history = ObstacleEvidence(), []
            self.body_contacts, self.max_tilt = [], 0.
            self.body_contact_publisher_seen = False
            self.imu_attitudes.clear()
            self.heading_pairs.clear()
            self.heading_error = None
            self.acceptance, self.evaluation_started = None, False
            self.save_future = None
            self.command = [0., 0., 0.]
            env = dict(os.environ, DEMO_RUN_DIR=str(self.run_dir), DEMO_RUN_ID=run_id,
                       DEMO_CONTROL_TIMING_AUDIT='1')
            self.proc_log = (self.run_dir/'stack.log').open('w')
            self.proc = subprocess.Popen(['bash', str(launcher), 'stack'], cwd=ROOT, env=env,
                                         stdout=self.proc_log, stderr=subprocess.STDOUT, start_new_session=True)
            return 200, {'run_id': run_id, 'message': '完整演示已启动，等待传感器与 SLAM 就绪。'}

    def stop_run(self):
        with self.action_lock, self.lock:
            self.mission.stop(self.elapsed())
            self.stop_pub.publish(Bool(data=True))
            self.obstacle_pub.publish(Bool(data=False))
            if self.proc is not None and self.proc.poll() is None and not self.stopping:
                self.stopping = True
                proc = self.proc
                proc.send_signal(signal.SIGINT)
                threading.Thread(target=self.finish_stop, args=(proc,), daemon=True).start()
            return 200, {'message': '停止请求已发送，正在等待本次仿真退出。'}

    def finish_stop(self, proc):
        try:
            finish_owned_process(proc)
        except (OSError, ValueError, TimeoutError) as exc:
            self.get_logger().error(f'Owned simulation shutdown failed: {exc}')
            return
        with self.action_lock, self.lock:
            if self.proc is proc:
                self.stopping = False
                if self.proc_log:
                    self.proc_log.close()
                    self.proc_log = None


def handler(coordinator):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(ROOT), **kwargs)

        def end_headers(self):
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            super().end_headers()

        def content(self, status, data, mime):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def json(self, status, data):
            self.content(status, json.dumps(data, ensure_ascii=False, allow_nan=False).encode(), 'application/json; charset=utf-8')

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/state':
                return self.json(200, coordinator.snapshot())
            if path == '/api/acceptance':
                with coordinator.lock:
                    result = coordinator.acceptance
                return self.json(200 if result is not None else 404, result or {'error': '本次独立验收尚未结束。'})
            if path == '/api/camera.jpg':
                with coordinator.lock:
                    data, t = coordinator.image_bytes, coordinator.image_at
                if not data or not t or time.monotonic()-t > 4:
                    return self.json(404, {'error': '尚无新鲜相机画面。'})
                return self.content(200, data, 'image/jpeg')
            if path.startswith('/api/map/'):
                with coordinator.lock:
                    directory = coordinator.run_dir
                name = path.removeprefix('/api/map/')
                if directory is None or Path(name).name != name or not (name == 'map_metadata.json' or (name.startswith('colored_map') and name.endswith(('.bin', '.pcd')))):
                    return self.send_error(404)
                target = directory/name
                if not target.is_file():
                    return self.send_error(404)
                return self.content(200, target.read_bytes(), 'application/json' if name.endswith('.json') else 'application/octet-stream')
            if path == '/':
                self.path = '/web/index.html'
            target = Path(self.translate_path(self.path)).resolve()
            if not any(target.is_relative_to(ROOT/x) for x in ('web', 'docs')) or not target.is_file():
                return self.send_error(404)
            return super().do_GET()

        def do_POST(self):
            host = self.headers.get('Host', '')
            if host not in (f'localhost:{self.server.server_port}', f'127.0.0.1:{self.server.server_port}') or self.headers.get('Origin') not in (None, 'http://'+host):
                return self.json(403, {'error': '仅支持本地同源控制。'})
            if urlsplit(self.path).path != '/api/mission':
                return self.send_error(404)
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size < 1024:
                    raise ValueError('body length')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('object required')
            except ValueError:
                return self.json(400, {'error': '无效请求。'})
            if data.get('action') == 'start':
                status, result = coordinator.start_run()
            elif data.get('action') == 'stop':
                status, result = coordinator.stop_run()
            else:
                return self.json(400, {'error': '仅支持 start / stop。'})
            self.json(status, result)

        def log_message(self, fmt, *args):
            if len(args) > 1 and str(args[1]) not in ('200', '304'):
                super().log_message(fmt, *args)
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8767)
    args = parser.parse_args()
    rclpy.init()
    node = Coordinator()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler(node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'Full demo browser: http://127.0.0.1:{args.port}/', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        with node.lock:
            node.mission.stop(node.elapsed())
        if rclpy.ok():
            node.stop_run()
        elif node.proc is not None and node.proc.poll() is None:
            node.proc.send_signal(signal.SIGINT)
        if node.proc is not None and node.proc.poll() is None:
            node.finish_stop(node.proc)
        if node.run_dir:
            with node.lock:
                (node.run_dir/'mission.tmp').write_text(json.dumps(node.snapshot(), ensure_ascii=False, indent=2))
                (node.run_dir/'mission.tmp').replace(node.run_dir/'mission.json')
                for stream in (node.audit_stream, node.command_stream, node.navigation_stream):
                    if stream:
                        stream.flush()
                        stream.close()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
