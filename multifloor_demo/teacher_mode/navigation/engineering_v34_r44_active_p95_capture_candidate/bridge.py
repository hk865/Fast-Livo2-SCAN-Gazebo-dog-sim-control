#!/usr/bin/env python3
# Public learning snapshot 2026-10-10: privacy/path metadata or documentation links adjusted; control algorithms and numeric gates unchanged.
"""Gate reviewed SLAM/SCAN Twist commands into an atomic local JSON file.

This is a prepared integration boundary, not evidence of navigation success.
It never subscribes to Gazebo ground truth and never actuates joints itself.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
import sys
sys.path.append(str(Path(__file__).resolve().parent.parent))
from runtime_io import BackgroundCheck,LatestCommandWriter,latest_sensor_qos
from compact_archives import make_command_writer


ROOT = Path(__file__).resolve().parents[2]
SLAM_TOPIC = '/demo/slam/body_odom'
COMMAND_TOPIC = '/demo/cmd_vel'
STATUS_TOPIC = '/demo/teacher/navigation_bridge/status'
STOP_TOPIC = '/demo/teacher/navigation_bridge/stop'
TIMEOUT = .300
LIMITS = (.3, .2, .3)
FROZEN_SHA = ''


def passed(value):
    if value == 'passed':
        return True
    return isinstance(value, dict) and value.get('passed') is True and value.get('status', 'passed') == 'passed'


def check_acceptance(path):
    """Only an explicit aggregate motion AND Sim2Sim pass unlocks runtime."""
    try:
        raw = path.read_bytes()
        if len(raw) > 2 * 1024 * 1024:
            raise ValueError('Acceptance file exceeds the review limit')
        data = json.loads(raw)
    except (OSError, ValueError) as error:
        raise RuntimeError(f'Navigation refused: acceptance unavailable: {error}') from error
    levels = data.get('levels') if isinstance(data, dict) else None
    if isinstance(data, dict) and data.get('schema') == 'teacher_closed_loop_navigation_scope/v1':
        from pid_scope import verify_scope
        return verify_scope(path, raw)
    if not isinstance(levels, dict) or not all(passed(levels.get(k)) for k in ('motion', 'sim2sim')):
        raise RuntimeError('Navigation refused: runs/acceptance.json must explicitly pass both levels.motion and levels.sim2sim')
    if data.get('publication') != 'final':
        raise RuntimeError('Navigation refused: acceptance must be a final reviewed receipt')
    scene = data.get('additional_scene_gate', {})
    if scene.get('status') != 'passed':
        raise RuntimeError('Navigation refused: original scene gate is not passed')
    navigation = data.get('navigation', {})
    if navigation.get('allowed') is not True or navigation.get('truth_navigation_used') is not False:
        raise RuntimeError('Navigation refused: aggregate does not permit a SLAM-only navigation run')
    model_hash = data.get('checkpoint_sha256')
    if model_hash != FROZEN_SHA:
        raise RuntimeError('Navigation refused: acceptance refers to another checkpoint')
    return {'path': str(path.resolve()), 'sha256': hashlib.sha256(raw).hexdigest(),
            'levels': {key: levels[key] for key in ('motion', 'sim2sim')},
            'checkpoint_sha256': model_hash, 'navigation_is_verified': False}


def finite_values(values, length):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        return False
    return all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def command_limits(profile):
    """The reference speed is not the cascade's compensation command limit."""
    if profile is not None and isinstance(profile.get('cascade'), dict):
        limits = profile['cascade']['command_limits']
        if (not finite_values(limits, 3) or
            not all(0 < value <= maximum for value, maximum in zip(limits, (1.,.4,1.)))):
            raise ValueError('Invalid closed-loop Teacher command envelope')
        return tuple(limits)
    return LIMITS


def validate_controller_source(data, requested, clock_ns, wall_ns):
    """Check original source times without refreshing them on file reads."""
    if not isinstance(data, dict) or data.get('navigation_ground_truth_used') is not False:
        raise ValueError('No original actual-SLAM cascade provenance')
    if not finite_values(data.get('command_after_slew'), 3) or data['command_after_slew'] != list(requested):
        raise ValueError('Controller source does not exactly match received Twist')
    keys = ('source_pose_stamp_ns','paired_imu_stamp_ns','control_stamp_ns',
            'source_pose_received_wall_ns','source_imu_received_wall_ns','cascade_sequence')
    if any(type(data.get(key)) is not int or data[key] < 0 for key in keys) or data['cascade_sequence'] < 1:
        raise ValueError('Original cascade source timestamps/sequence are malformed')
    if type(clock_ns) is not int or type(wall_ns) is not int:
        raise ValueError('Source clock unavailable')
    pose, gyro, control = (data[key] for key in keys[:3])
    if not (0 <= clock_ns-pose <= 300_000_000 and 0 <= pose-gyro <= 20_000_000
            and pose <= control <= clock_ns+50_000_000):
        raise ValueError('Original cascade source age or causality rejected')
    if not all(0 <= wall_ns-data[key] <= 300_000_000 for key in keys[3:5]):
        raise ValueError('Original SLAM/IMU cascade wall source expired')
    # Preserve optional seconds aliases exactly, while refusing inconsistent
    # representations if the executor's reader uses those aliases.
    aliases = (('source_pose_received_monotonic_wall','source_pose_received_wall_ns'),
               ('paired_imu_received_monotonic_wall','source_imu_received_wall_ns'))
    for seconds, nanos in aliases:
        if seconds in data and (not finite_values([data[seconds]],1) or
                                abs(data[seconds]-data[nanos]/1e9)>1e-9):
            raise ValueError('Cascade wall-time aliases disagree')
    if data.get('healthy') is False:
        raise ValueError('Controller source explicitly unhealthy')
    return data


def read_controller_source(path, requested, clock_ns, wall_ns):
    """Return the unmodified decoded source and raw-file receipt on failure too."""
    data = None
    receipt = {'path': str(path.resolve()), 'sha256': None, 'read_begin_wall_ns': wall_ns,
               'read_wall_ns': None}
    try:
        raw = path.read_bytes()
        if len(raw) > 256*1024:
            raise ValueError('Cascade source exceeds 256 KiB limit')
        receipt['sha256'] = hashlib.sha256(raw).hexdigest()
        data = json.loads(raw)
        # A producer may publish between read begin and actual read. Evaluate
        # wall TTL after decoding, preserving begin/completion for audit, as
        # the already validated V5 command consumer does.
        receipt['read_wall_ns'] = time.monotonic_ns()
        validate_controller_source(data, requested, clock_ns, receipt['read_wall_ns'])
        return data, receipt, None
    except (OSError, ValueError, TypeError, KeyError) as error:
        return data, receipt, type(error).__name__+': '+str(error)


class CommandGate:
    """Wall time remains authoritative during paused or missing ROS clocks."""
    def __init__(self, profile=None):
        self.profile = profile
        self.limits = command_limits(profile)
        self.controller_source = None
        self.controller_source_receipt = None
        self.controller_source_error = None
        self.execution_health=None;self.execution_health_wall=None
        self.position = None
        self.sensor_health = None
        self.sensor_health_wall = None
        self.anchor = None
        self.scope_graph_ok = False
        self.clock_ns = None
        self.clock_wall = None
        self.pose_stamp_ns = None
        self.pose_wall = None
        self.pose_valid_samples = 0
        self.pose_error = 'SLAM pose not received'
        self.command_wall = None
        self.command_sim_ns = None
        self.requested = [0., 0., 0.]
        self.command_error = 'SCAN command not received'
        self.graph = {'slam': False, 'command': False, 'details': {}}
        self.graph_wall = None
        self.failed = None

    def clock(self, stamp_ns, now):
        if not isinstance(stamp_ns, int) or stamp_ns < 0:
            self.failed = 'Invalid simulation clock'
        elif self.clock_ns is not None and stamp_ns < self.clock_ns:
            self.failed = 'Simulation clock moved backward; restart the integration run'
        elif self.clock_ns is None or stamp_ns > self.clock_ns:
            self.clock_ns, self.clock_wall = stamp_ns, now

    def pose(self, stamp_ns, position, quaternion, frame, child_frame, now):
        error = None
        if frame != 'camera_init' or child_frame != 'demo_slam_body':
            error = 'Navigation pose must be camera_init / demo_slam_body'
        elif not finite_values(position, 3) or not finite_values(quaternion, 4):
            error = 'SLAM pose is nonfinite or malformed'
        elif not .98 <= sum(v * v for v in quaternion) <= 1.02:
            error = 'SLAM quaternion is not normalized'
        elif not isinstance(stamp_ns, int) or stamp_ns < 0:
            error = 'SLAM pose timestamp is invalid'
        elif self.pose_stamp_ns is not None and stamp_ns <= self.pose_stamp_ns:
            error = 'SLAM pose timestamps must strictly increase'
        elif self.clock_ns is not None and stamp_ns > self.clock_ns + 50_000_000:
            error = 'SLAM pose timestamp is ahead of simulation clock'
        if error:
            self.pose_error = error
            self.pose_valid_samples = 0
            return
        if self.pose_stamp_ns is not None and stamp_ns - self.pose_stamp_ns >= int(TIMEOUT * 1e9):
            self.pose_valid_samples = 0
        self.pose_stamp_ns, self.pose_wall = stamp_ns, now
        self.position = list(position)
        self.pose_valid_samples += 1
        self.pose_error = None

    def command(self, values, unsupported_axes, now, controller_source=None, source_receipt=None, source_error=None):
        if not finite_values(values, 3) or not finite_values(unsupported_axes, 3):
            self.failed = 'SCAN command contains a nonfinite value'
            self.command_error = self.failed
            return
        if any(abs(v) > 1e-9 for v in unsupported_axes):
            self.failed = 'SCAN command requests unsupported z/roll/pitch motion'
            self.command_error = self.failed
            return
        self.requested = list(values)
        self.controller_source = controller_source
        self.controller_source_receipt = source_receipt
        self.controller_source_error = source_error
        self.command_wall, self.command_sim_ns = now, self.clock_ns
        self.command_error = None

    def snapshot(self, now):
        ages = {'clock_wall_s': None if self.clock_wall is None else now - self.clock_wall,
                'pose_wall_s': None if self.pose_wall is None else now - self.pose_wall,
                'command_wall_s': None if self.command_wall is None else now - self.command_wall,
                'pose_sim_s': None if self.clock_ns is None or self.pose_stamp_ns is None else (self.clock_ns - self.pose_stamp_ns) / 1e9,
                'command_sim_s': None if self.clock_ns is None or self.command_sim_ns is None else (self.clock_ns - self.command_sim_ns) / 1e9}
        reasons = []
        if self.failed:
            reasons.append(self.failed)
        if self.profile is not None:
            if self.profile.get('cascade') is not None and any(self.requested):
                try:
                    if any(abs(value)>limit for value,limit in zip(self.requested,self.limits)):
                        raise ValueError('Cascade Twist exceeds its frozen command envelope')
                    if self.controller_source_error:
                        raise ValueError(self.controller_source_error)
                    validate_controller_source(self.controller_source,self.requested,self.clock_ns,round(now*1e9))
                except (ValueError, TypeError, KeyError) as error:
                    reasons.append('Cascade provenance rejected: '+str(error))
            if self.profile['controller_kind']=='champ':
                h=self.execution_health
                ready=(h is not None and self.execution_health_wall is not None
                    and now-self.execution_health_wall<TIMEOUT and -.05<=now-float(h['monotonic_wall'])<TIMEOUT
                    and self.clock_ns is not None and -50_000_000<=self.clock_ns-int(h['ros_sim_time_ns'])<300_000_000
                    and h.get('state')=='ready'and h.get('ready')is True)
                if not ready:reasons.append('Actual CHAMP executor calibration/controller feedback is not healthy and fresh')
                if h is not None and h.get('state')=='failed':self.failed='CHAMP executor failed';reasons.append(self.failed)
            if not self.scope_graph_ok:
                reasons.append('Finite experiment sensor gate / SLAM anchor publisher chain is not unique')
            if (self.sensor_health is None or self.sensor_health_wall is None
                    or now-self.sensor_health_wall >= TIMEOUT or self.sensor_health.get('ready') is not True):
                reasons.append('Actual vehicle sensor SLAM has not completed healthy warmup')
            if self.anchor is None:
                reasons.append('Waiting for immutable SLAM-derived route anchor')
            elif self.position is not None:
                a=self.anchor;dx=self.position[0]-a['origin'][0];dy=self.position[1]-a['origin'][1]
                c,s=math.cos(a['yaw']),math.sin(a['yaw']);along=c*dx+s*dy;lateral=-s*dx+c*dy
                fence=self.profile.get('fence')
                if self.profile.get('registered_route_fence'):
                    from route_fence import tube_evidence
                    config=self.profile['registered_route_fence']
                    try:
                        self.registered_fence=tube_evidence(self.position,a['registered_route_camera_init_xyz'],
                            config['horizontal_radius_m'],config['height_error_m'])
                        outside=not self.registered_fence['inside']
                    except (KeyError,ValueError,TypeError):outside=True
                else:
                    outside=(not fence['along_min_m']<=along<=fence['along_max_m']
                    or abs(lateral)>fence['lateral_abs_m']
                    or abs(self.position[2]-a['origin'][2])>fence['height_abs_m'])
                if outside:
                    self.failed='Measured SLAM body pose left finite route fence; restart after review'
                    reasons.append(self.failed)
        if self.graph_wall is None or now - self.graph_wall >= TIMEOUT:
            reasons.append('ROS publisher graph check is stale')
        if not self.graph['slam']:
            reasons.append('SLAM source must have exactly one demo_slam_odom_adapter publisher')
        if not self.graph['command']:
            reasons.append('Command source must have exactly one demo_navigation publisher')
        if self.pose_error:
            reasons.append(self.pose_error)
        if self.pose_valid_samples < 2:
            reasons.append('Waiting for two consecutive valid SLAM poses')
        if self.command_error:
            reasons.append(self.command_error)
        for name, age in ages.items():
            if age is None or age >= TIMEOUT or age < -.05:
                reasons.append(f'{name} is missing, stale, or inconsistent')
        healthy = not reasons
        command = [max(-limit, min(limit, value)) for value, limit in zip(self.requested, self.limits)] if healthy else [0., 0., 0.]
        if self.profile is not None and self.profile.get('cascade') is None:
            scoped_limits=[self.profile['max_speed_mps'],self.profile['max_lateral_speed_mps'],self.profile['max_yaw_rate_radps']]
            command=[max(-limit,min(limit,value))for value,limit in zip(command,scoped_limits)]
        return {'state': 'failed' if self.failed else 'ready' if healthy else 'hold',
                'healthy': healthy, 'stop_requested': not healthy or not any(command),
                'reason': '; '.join(dict.fromkeys(reasons)) if reasons else 'Fresh SLAM/SCAN inputs',
                'requested': self.requested.copy(), 'command': command, 'ages': ages,
                'slam_valid_samples': self.pose_valid_samples,
                'command_received_monotonic_wall': self.command_wall,
                'command_received_ros_sim_time_ns': self.command_sim_ns,
                'slam_stamp_ns': self.pose_stamp_ns, 'publisher_graph': self.graph,
                'controller_source': self.controller_source,
                'controller_source_receipt': self.controller_source_receipt,
                'registered_route_fence': getattr(self,'registered_fence',None),
                'controller_source_error': self.controller_source_error}


class CommandFile:
    def __init__(self, path):
        self.path = path.resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = (self.path.parent / (self.path.name + '.lock')).open('a')
        try:
            fcntl.flock(self.lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self.lock.close()
            raise RuntimeError('Another navigation bridge owns this command file') from error

    def write(self, value):
        payload = json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                             prefix='.' + self.path.name + '.', delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(payload)
                stream.flush()
                # Transient transport: close + atomic replace provide complete
                # old/new JSON reads. No power-loss durability is promised.
                # Original stamps remain unchanged; late delivery still expires.
            temporary.replace(self.path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def close(self):
        self.lock.close()


def make_node(rclpy, writer, acceptance_path, acceptance):
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String

    class Bridge(Node):
        def __init__(self):
            super().__init__('teacher_scan_slam_bridge')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
            self.gate = CommandGate(acceptance.get('profile') if acceptance.get('experimental') else None)
            self.sequence = 0
            self.acceptance_checked_wall = None
            self.last_status_wall=-math.inf;self.last_published_state=None
            history=Path(acceptance['run_dir'])/'navigation_command_history.jsonl'if acceptance.get('experimental')else None
            self.transport=make_command_writer(writer,history,acceptance)
            self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
            self.stop_pub = self.create_publisher(Bool, STOP_TOPIC, 10)
            self.safety_pub = self.create_publisher(String, '/demo/control/safety', 10)
            if acceptance.get('experimental'):
                self.create_subscription(String, '/demo/teacher/navigation/sensor_health', self.sensor_health, 1)
                latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(String, '/demo/teacher/navigation/anchor', self.anchor, latched)
                if acceptance['controller_kind']=='champ':
                    self.create_subscription(String,'/demo/champ/execution_health',self.execution_health,1)
            self.pose_sub = self.create_subscription(Odometry, SLAM_TOPIC, self.pose, latest_sensor_qos())
            self.command_sub = self.create_subscription(Twist, COMMAND_TOPIC, self.command, 1)
            if self.pose_sub.topic_name != SLAM_TOPIC or self.command_sub.topic_name != COMMAND_TOPIC:
                self.destroy_node()
                raise RuntimeError('Remapping SLAM/SCAN input topics is forbidden; ground truth cannot be a navigation source')
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.checker=BackgroundCheck(self.check_integrity)
            self.timer = self.create_timer(.02, self.tick, clock=self.wall_clock)

        def sync_clock(self):
            # rclpy's single ROS TimeSource subscription retains the latest
            # real /clock. Never stamp commands from a second lagging queue.
            self.gate.clock(self.get_clock().now().nanoseconds,time.monotonic())

        def pose(self, message):
            self.sync_clock()
            p, q = message.pose.pose.position, message.pose.pose.orientation
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            self.gate.pose(stamp, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
                           message.header.frame_id, message.child_frame_id, time.monotonic())
            if self.gate.pose_error:
                self.tick()

        def command(self, message):
            self.sync_clock();was_moving=any(self.gate.requested)
            values=[message.linear.x, message.linear.y, message.angular.z]
            source=source_receipt=source_error=None
            if any(values) and acceptance.get('experimental'):
                source,source_receipt,source_error=read_controller_source(
                    Path(acceptance['run_dir'])/'cascade_command_source.json',values,
                    self.gate.clock_ns,time.monotonic_ns())
            self.gate.command(values,[message.linear.z, message.angular.x, message.angular.y],
                              time.monotonic(),source,source_receipt,source_error)
            # 50Hz wall heartbeat writes ordinary commands. Stop/fault edges
            # enqueue an immediate zero without creating a callback feedback.
            if self.gate.failed or source_error or was_moving and not any(self.gate.requested):self.tick()

        def sensor_health(self, message):
            self.sync_clock()
            try:
                data=json.loads(message.data)
                now=time.monotonic()
                if (data.get('mode')!=acceptance['controller_kind'] or data.get('ground_truth_navigation_used')is not False
                    or not -.05<=now-float(data['monotonic_wall'])<TIMEOUT):return
                self.gate.sensor_health=data;self.gate.sensor_health_wall=now
            except (ValueError,TypeError,KeyError):return

        def execution_health(self,message):
            self.sync_clock();now=time.monotonic()
            try:
                d=json.loads(message.data)
                if(d.get('schema')!=1 or d.get('mode')!='champ'or d.get('source')!='scan_slam'
                    or d.get('ground_truth_navigation_used')is not False or d.get('actor_started')is not False
                    or d.get('state')not in ('hold','ready','failed')or not -.05<=now-float(d['monotonic_wall'])<TIMEOUT
                    or self.gate.clock_ns is None or not -50_000_000<=self.gate.clock_ns-int(d['ros_sim_time_ns'])<300_000_000):return
                self.gate.execution_health=d;self.gate.execution_health_wall=now
                if d['state']!='ready'and any(self.gate.requested):self.tick()
            except (ValueError,TypeError,KeyError):return

        def anchor(self, message):
            try:
                data=json.loads(message.data)
                if (data.get('schema')!=1 or data.get('frame_id')!='camera_init'
                    or data.get('source')!=SLAM_TOPIC or data.get('ground_truth_navigation_used')is not False
                    or data.get('run_dir')!=acceptance['run_dir'] or data.get('frozen_once')is not True
                    or not finite_values(data.get('origin'),3)
                    or not finite_values([data.get('yaw')],1)):return
                frozen=json.loads((Path(acceptance['run_dir'])/'navigation_anchor.json').read_text())
                if data!=frozen:raise ValueError('Anchor differs from frozen actual SLAM receipt')
                if self.gate.anchor is not None and self.gate.anchor!=data:
                    self.gate.failed='Attempt to change finite route SLAM anchor'
                    return
                self.gate.anchor=data
            except (OSError,ValueError,TypeError,KeyError):return

        def publishers(self, topic, allowed):
            infos = self.get_publishers_info_by_topic(topic)
            names = [item.node_namespace.rstrip('/') + '/' + item.node_name for item in infos]
            return len(infos) == 1 and infos[0].node_name == allowed and infos[0].node_namespace == '/', names

        def envelope(self, now):
            status = self.gate.snapshot(now)
            self.sequence += 1
            status.update({'schema_version': 1, 'sequence': self.sequence, 'mode': 'scan_slam', 'source': 'scan_slam',
                           'monotonic_wall': now, 'sim_time': None if self.gate.clock_ns is None else self.gate.clock_ns / 1e9,
                           'stamp': {'monotonic_wall': now, 'ros_sim_time_ns': self.gate.clock_ns,
                                     'ros_sim_time': None if self.gate.clock_ns is None else self.gate.clock_ns / 1e9},
                           'valid_for_wall_s': TIMEOUT, 'limits': {'vx_mps': self.gate.limits[0], 'vy_mps': self.gate.limits[1], 'wz_radps': self.gate.limits[2]},
                           'observation_source': {'navigation_pose': SLAM_TOPIC, 'navigation_frame': 'camera_init',
                               'navigation_ground_truth_used': False, 'velocity_request': COMMAND_TOPIC,
                               'policy_inputs': acceptance['profile']['policy_observations']},
                           'acceptance': acceptance, 'navigation_validation': 'unverified',
                           'health_scope': 'SLAM/SCAN input health only; this does not establish actuator health or physical stopping'})
            status['clock_source']='single rclpy ROS TimeSource /clock, KEEP_LAST depth=1'
            status['transport']={'type':'latest_pending_atomic_file','superseded':self.transport.superseded,
                                 'written':self.transport.written,'max_queue_delay_wall_s':self.transport.max_delay_s}
            return status

        def publish(self, status):
            if rclpy.ok():
                self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))
                self.stop_pub.publish(Bool(data=status['stop_requested']))
                self.safety_pub.publish(String(data=json.dumps({**status,'mode':acceptance['controller_kind'],'source':'scan_slam'},allow_nan=False)))

        def check_integrity(self):
            slam_ok,slam_nodes=self.publishers(SLAM_TOPIC,'demo_slam_odom_adapter')
            command_ok,command_nodes=self.publishers(COMMAND_TOPIC,'demo_navigation')
            scope_ok=True
            if acceptance.get('experimental'):
                health_ok,_=self.publishers('/demo/teacher/navigation/sensor_health','teacher_navigation_sensor_gate')
                anchor_ok,_=self.publishers('/demo/teacher/navigation/anchor','teacher_navigation_request')
                scope_ok=health_ok and anchor_ok
                if acceptance['controller_kind']=='champ':
                    execution_ok,_=self.publishers('/demo/champ/execution_health','champ_reviewed_command_reader')
                    scope_ok=scope_ok and execution_ok
            current=check_acceptance(acceptance_path)
            if current['sha256']!=acceptance['sha256']:raise RuntimeError('Acceptance file changed during navigation; review and restart')
            return {'graph':{'slam':slam_ok,'command':command_ok,
                    'details':{'slam_publishers':slam_nodes,'command_publishers':command_nodes}},'scope_ok':scope_ok}

        def tick(self):
            self.sync_clock();now=time.monotonic();checked=self.checker.result
            if checked is not None:
                if checked['error']:self.gate.failed='Integrity check failed: '+checked['error']
                else:
                    self.gate.graph=checked['data']['graph'];self.gate.scope_graph_ok=checked['data']['scope_ok']
                    self.gate.graph_wall=checked['started_wall']
            if self.transport.error:self.gate.failed=self.transport.error
            status = self.envelope(now)
            try:
                self.transport.submit(status)
            except RuntimeError as error:
                self.gate.failed = f'Command file write failed: {error}'
                status = self.envelope(now)
                self.publish(status)
                raise RuntimeError(self.gate.failed) from error
            # A normal health heartbeat is 20Hz. State/stop edges publish
            # immediately; repeated already-zero updates do not flood DDS.
            published=(status['state'],status['stop_requested'])
            if now-self.last_status_wall>=.05 or published!=self.last_published_state:
                self.publish(status);self.last_status_wall=now;self.last_published_state=published

        def shutdown_stop(self):
            if getattr(self,'shutdown_started',False):return
            self.shutdown_started=True
            self.gate.failed = 'Navigation command bridge stopped'
            status = self.envelope(time.monotonic())
            try:
                self.transport.close(status)
            finally:
                self.publish(status)
                self.checker.close()
    return Bridge()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--acceptance', type=Path, default=ROOT / 'runs/acceptance.json')
    parser.add_argument('--command-file', type=Path,
                        default=None)
    parser.add_argument('--prepare', action='store_true', help='Review the gate and print an UNVERIFIED receipt; start no ROS nodes')
    args, ros_args = parser.parse_known_args()
    try:
        acceptance = check_acceptance(args.acceptance)
        refusal = None
    except RuntimeError as error:
        acceptance, refusal = None, str(error)
    if args.command_file is None:
        args.command_file=Path(acceptance['run_dir'])/'navigation_command.json'if acceptance and acceptance.get('experimental')else Path(os.environ.get('TEACHER_COMMAND_FILE',str(ROOT/'runs/nav_command.json')))
    if acceptance and acceptance.get('experimental') and args.command_file.resolve()!=Path(acceptance['run_dir'])/'navigation_command.json':
        refusal='Finite navigation command output must belong to this run: navigation_command.json'
    if args.prepare:
        print(json.dumps({'status': 'prepared_unverified', 'navigation': 'unverified', 'starts_ros': False,
                          'writes_commands': False, 'runtime_eligible': acceptance is not None, 'refusal': refusal,
                          'acceptance': acceptance, 'command_file': str(args.command_file.resolve()),
                          'slam_source': SLAM_TOPIC, 'command_source': COMMAND_TOPIC, 'timeout_s': TIMEOUT,
                          'scope': 'Code/interface preparation only; does not verify navigation or physical stopping'},
                         ensure_ascii=False, indent=2))
        return 0
    if refusal:
        print(refusal)
        return 2
    # Imports and ROS initialization happen only AFTER the frozen acceptance gate.
    writer = CommandFile(args.command_file)
    node = None
    try:
        import rclpy
        rclpy.init(args=ros_args)
        node = make_node(rclpy, writer, args.acceptance, acceptance)
        node.tick()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            from owned_guardian import bounded_shutdown_signals
            with bounded_shutdown_signals():
                node.shutdown_stop()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
    finally:
        from owned_guardian import bounded_shutdown_signals
        with bounded_shutdown_signals():writer.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
