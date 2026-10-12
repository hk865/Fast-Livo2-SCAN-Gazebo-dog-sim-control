#!/usr/bin/env python3
"""Read-only paired-QoS actual cloud receipts. No decoding, publishing or control."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def message_info(info):
    if info is None:
        return None
    data = {}
    for name in ('source_timestamp', 'received_timestamp', 'reception_timestamp',
                 'publication_sequence_number', 'reception_sequence_number'):
        value = getattr(info, name, None)
        data[name] = int(value) if value is not None else None
    gid = getattr(info, 'publisher_gid', None)
    try:
        data['publisher_gid_hex'] = bytes(gid).hex() if gid is not None else None
    except (TypeError, ValueError):
        data['publisher_gid_hex'] = None
    return data


class Receipts:
    """Bounded single writer: immutable metadata lines, explicit overflow/error."""
    def __init__(self, path, capacity=8192):
        self.path = Path(path)
        self.queue = queue.Queue(capacity)
        self.error = None
        self.submitted = 0
        self.written = 0
        self.closed = False
        self.thread = threading.Thread(target=self._run, name='cloud-qos-probe-evidence', daemon=True)
        self.thread.start()

    def append(self, data):
        if self.closed or self.error:
            raise RuntimeError(self.error or 'Probe receipt writer closed')
        line = json.dumps(data, allow_nan=False, separators=(',', ':')) + '\n'
        try:
            self.queue.put_nowait(line)
        except queue.Full:
            self.error = 'Probe receipt queue overflow; evidence is incomplete'
            raise RuntimeError(self.error)
        self.submitted += 1

    def _run(self):
        stream = None
        try:
            stream = self.path.open('x')
            while True:
                line = self.queue.get()
                try:
                    if line is None:
                        break
                    if self.error is None:
                        stream.write(line)
                        self.written += 1
                finally:
                    self.queue.task_done()
            stream.flush()
        except Exception as exc:
            self.error = f'Probe receipt write failed: {type(exc).__name__}: {exc}'
        finally:
            if stream is not None:
                stream.close()

    def close(self):
        self.closed = True
        if self.thread.is_alive():
            try:
                self.queue.put(None, timeout=2.)
            except queue.Full:
                self.error = self.error or 'Probe receipt queue could not close'
            self.thread.join(timeout=5.)
        if self.thread.is_alive():
            self.error = self.error or 'Probe receipt writer did not drain'
        if self.error or self.submitted != self.written:
            raise RuntimeError(self.error or 'Probe receipt submitted/written mismatch')


class Metadata:
    def __init__(self, writer):
        self.writer = writer
        self.counts = {name: 0 for name in ('cloud_reliable', 'cloud_best_effort', 'body_odom', 'clock')}
        self.clock_ns = None
        self.clock_wall = None
        self.first_clock_ns = None
        self.clock_regressions = 0

    def receive(self, name, msg, info=None):
        wall = time.monotonic()
        if name == 'clock':
            original = stamp_ns(msg.clock)
            if self.clock_ns is not None and original < self.clock_ns:
                self.clock_regressions += 1
                raise RuntimeError('Observed actual simulation clock moved backwards')
            self.clock_ns = original
            self.clock_wall = wall
            if self.first_clock_ns is None:
                self.first_clock_ns = original
            frame = None
        else:
            original = stamp_ns(msg.header.stamp)
            frame = msg.header.frame_id
        self.counts[name] += 1
        row = {'schema': 'actual_cloud_paired_qos_receipt/v1', 'stream': name,
               'stream_sequence': self.counts[name], 'original_stamp_ns': original,
               'frame_id': frame, 'received_monotonic_wall': wall,
               'received_ros_clock_ns': self.clock_ns,
               'received_clock_wall_age_s': None if self.clock_wall is None else wall - self.clock_wall,
               'message_info': message_info(info), 'decoded': False,
               'ground_truth_navigation_used': False}
        if name.startswith('cloud_'):
            row.update(topic='/cloud_registered_full', width=int(msg.width), height=int(msg.height),
                       point_count_slots=int(msg.width) * int(msg.height), point_step=int(msg.point_step),
                       row_step=int(msg.row_step), data_bytes=len(msg.data), is_dense=bool(msg.is_dense),
                       is_bigendian=bool(msg.is_bigendian), fields=[{
                           'name': f.name, 'offset': int(f.offset), 'datatype': int(f.datatype),
                           'count': int(f.count)} for f in msg.fields],
                       qos={'history': 'KEEP_LAST', 'depth': 1,
                            'reliability': 'RELIABLE' if name == 'cloud_reliable' else 'BEST_EFFORT'},
                       payload_copied_or_hashed=False)
        elif name == 'body_odom':
            row.update(topic='/demo/slam/body_odom', child_frame_id=msg.child_frame_id,
                       qos={'history': 'KEEP_LAST', 'depth': 1, 'reliability': 'BEST_EFFORT'})
        else:
            row.update(topic='/clock', qos={'history': 'KEEP_LAST', 'depth': 1, 'reliability': 'BEST_EFFORT'})
        self.writer.append(row)


def atomic(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def verify_freeze(path):
    data = json.loads(path.read_text())
    for name, expected in data['source_hashes'].items():
        if digest(HERE / name) != expected:
            raise RuntimeError('Probe source/configuration changed: ' + name)
    return data


def endpoint_metadata(node):
    rows = {}
    for topic in ('/cloud_registered_full', '/demo/slam/body_odom', '/clock'):
        rows[topic] = {}
        for kind, method in (('publishers', node.get_publishers_info_by_topic),
                             ('subscribers', node.get_subscriptions_info_by_topic)):
            values = []
            for item in method(topic):
                qos = item.qos_profile
                values.append({'node_name': item.node_name, 'node_namespace': item.node_namespace,
                    'topic_type': item.topic_type, 'endpoint_gid_hex': bytes(item.endpoint_gid).hex(),
                    'qos': {'history': str(qos.history), 'depth': int(qos.depth),
                            'reliability': str(qos.reliability), 'durability': str(qos.durability)}})
            rows[topic][kind] = values
    return rows


def self_test(output):
    from types import SimpleNamespace as NS
    output.mkdir(parents=True, exist_ok=False)
    checks = {}
    path = output / 'mock_receipts.jsonl'
    writer = Receipts(path)
    meta = Metadata(writer)
    large_ns = 9_007_199_254_740_993
    clock = NS(clock=NS(sec=large_ns // 1_000_000_000, nanosec=large_ns % 1_000_000_000))
    header = NS(stamp=clock.clock, frame_id='camera_init')
    class Payload:
        def __len__(self): return 472000
        def __iter__(self): raise AssertionError('Payload must never be copied, hashed or decoded')
    cloud = NS(header=header, width=14750, height=1, point_step=32, row_step=472000,
               is_dense=False, is_bigendian=False, data=Payload(),
               fields=[NS(name='x', offset=0, datatype=7, count=1)])
    info = NS(source_timestamp=4, received_timestamp=5, publication_sequence_number=11,
              reception_sequence_number=12, publisher_gid=bytes([1, 2, 3]))
    meta.receive('clock', clock)
    meta.receive('cloud_reliable', cloud, info)
    meta.receive('cloud_best_effort', cloud, info)
    meta.receive('body_odom', NS(header=header, child_frame_id='base_link'), info)
    writer.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    checks['integer_stamp_above_float_precision_preserved'] = all(x['original_stamp_ns'] == large_ns for x in rows)
    checks['paired_identical_metadata_except_qos_and_receipt'] = all(rows[1][k] == rows[2][k] for k in ('original_stamp_ns', 'frame_id', 'width', 'height', 'fields', 'data_bytes', 'message_info'))
    checks['clock_latest_explicit_no_extra_timesource'] = all(x['received_ros_clock_ns'] == large_ns for x in rows)
    checks['payload_not_read_copied_hashed_or_decoded'] = all(not x['decoded'] for x in rows)
    checks['actual_qos_distinction'] = rows[1]['qos']['reliability'] == 'RELIABLE' and rows[2]['qos']['reliability'] == 'BEST_EFFORT'
    checks['message_info_original_sequence'] = rows[1]['message_info']['publication_sequence_number'] == 11
    checks['all_four_streams_recorded_and_drained'] = writer.submitted == writer.written == 4 and writer.error is None
    full = Receipts.__new__(Receipts)
    full.closed = False
    full.error = None
    full.submitted = 0
    full.queue = queue.Queue(1)
    full.queue.put('occupied')
    try:
        full.append({'value': 1})
    except RuntimeError:
        checks['bounded_queue_overflow_is_explicit'] = 'overflow' in full.error and full.submitted == 0
    try:
        writer.append({'value': 1})
    except RuntimeError:
        checks['closed_writer_rejects_additional_records'] = True
    try:
        meta.receive('clock', NS(clock=NS(sec=0, nanosec=1)))
    except RuntimeError:
        checks['clock_regression_fails'] = meta.clock_regressions == 1
    bad = Receipts(output / 'bad_nan.jsonl')
    try:
        bad.append({'value': float('nan')})
    except ValueError:
        checks['nonfinite_metadata_rejected'] = bad.submitted == 0
    bad.close()
    error_writer = Receipts(output / 'bad_queue.jsonl')
    error_writer.error = 'injected writer error'
    try:
        error_writer.append({'value': 1})
    except RuntimeError:
        checks['writer_error_rejects_callback_receipt'] = True
    try:
        error_writer.close()
    except RuntimeError:
        checks['writer_error_requires_failed_close'] = not error_writer.thread.is_alive()
    import ast
    tree = ast.parse(Path(__file__).read_text())
    checks['no_ros_publisher_service_client_or_actuation'] = not any(isinstance(n, ast.Attribute) and n.attr in ('create_publisher', 'create_client', 'create_service', 'publish') for n in ast.walk(tree))
    import inspect
    def callback(msg, info, stream='cloud_reliable'): return stream
    try:
        inspect.signature(callback).bind(object())
    except TypeError:
        inspect.signature(callback).bind(object(), object())
        checks['rclpy_two_argument_message_info_callback_compatible'] = True
    atomic(output / 'offline_receipt.json', {'status': 'passed' if all(checks.values()) else 'failed',
           'checks': checks, 'source_sha256': digest(__file__), 'live_ros_nodes_started': 0,
           'simulation_started': False, 'navigation_validation': 'unverified'})
    return 0 if all(checks.values()) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-sim', type=float, default=60.)
    parser.add_argument('--wall-deadline', type=float, default=90.)
    parser.add_argument('--ros-domain', type=int, default=79)
    parser.add_argument('--freeze', type=Path, default=HERE / 'freeze.json')
    parser.add_argument('--self-test', action='store_true')
    args, ros = parser.parse_known_args()
    if args.self_test:
        return self_test(args.output)
    if args.duration_sim != 60. or args.wall_deadline != 90. or args.ros_domain != 79:
        raise ValueError('Frozen probe requires60sim seconds,90wall seconds,ROSdomain79')
    freeze = verify_freeze(args.freeze)
    freeze_sha256 = digest(args.freeze)
    if os.environ.get('ROS_DOMAIN_ID', '79') != '79':
        raise RuntimeError('Probe must join only the authorized ROS domain79')
    os.environ['ROS_DOMAIN_ID'] = '79'
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'sources').mkdir()
    for name in freeze['source_hashes']:
        (output / 'sources' / name).write_bytes((HERE / name).read_bytes())
    (output / 'sources' / 'freeze.json').write_bytes(args.freeze.read_bytes())
    writer = Receipts(output / 'receipts.jsonl')
    meta = Metadata(writer)
    started_wall = time.monotonic()
    reason = None
    interrupted = {'signal': None}
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda sig, frame: interrupted.update(signal=sig))
    node = None
    error = None
    observed_sim_s = None
    source_refs = {str(HERE / name): expected for name, expected in freeze['source_hashes'].items()}
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)
        node = Node('teacher_actual_cloud_paired_qos_probe')
        # One explicit depth1 Clock subscriber; do not enable a second ROS TimeSource.
        qos = lambda reliable: QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
        subscriptions = []
        for name, typ, topic, reliable in (
            ('cloud_reliable', PointCloud2, '/cloud_registered_full', True),
            ('cloud_best_effort', PointCloud2, '/cloud_registered_full', False),
            ('body_odom', Odometry, '/demo/slam/body_odom', False),
            ('clock', Clock, '/clock', False)):
            def receive(msg, info, stream=name):
                meta.receive(stream, msg, info)
            subscriptions.append(node.create_subscription(typ, topic, receive, qos(reliable)))
        atomic(output / 'ready.json', {'pid': os.getpid(), 'ros_domain': 79,
            'started_monotonic_wall': started_wall, 'source_hashes': source_refs,
            'freeze_sha256': freeze_sha256, 'no_publishers_or_control': True})
        env_names = ('RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
                     'ROS_AUTOMATIC_DISCOVERY_RANGE', 'CYCLONEDDS_URI',
                     'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                     'RMW_FASTRTPS_USE_QOS_FROM_XML', 'RMW_FASTRTPS_PUBLICATION_MODE',
                     'FASTDDS_BUILTIN_TRANSPORTS', 'FASTDDS_STATISTICS')
        env_names = sorted(set(env_names) | {name for name in os.environ
            if name.startswith(('RMW_', 'FASTDDS_', 'FASTRTPS_', 'CYCLONEDDS_'))})
        from rclpy.utilities import get_rmw_implementation_identifier
        atomic(output / 'environment.json', {'environment': {name: os.environ.get(name) for name in env_names},
            'actual_observer_rmw_implementation': get_rmw_implementation_identifier(),
            'scope': 'This process only; not a claim about other nodes middleware or transport'})
        # Actual loaded middleware libraries of this observer only, never a claim about other nodes.
        (output / 'observer_proc_maps.txt').write_text(Path('/proc/self/maps').read_text())
        print(json.dumps({'probe_ready': str(output), 'pid': os.getpid()}), flush=True)
        next_graph_wall = 0.
        while True:
            if interrupted['signal'] is not None:
                reason = 'interrupted'
                break
            if writer.error:
                raise RuntimeError(writer.error)
            if time.monotonic() - started_wall >= args.wall_deadline:
                reason = 'wall_deadline_without_complete_sim_duration'
                break
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() >= next_graph_wall:
                graph_start_wall = time.monotonic()
                writer.append({'schema': 'actual_cloud_paired_qos_graph/v1',
                    'started_monotonic_wall': graph_start_wall, 'ros_clock_ns': meta.clock_ns,
                    'endpoints': endpoint_metadata(node), 'finished_monotonic_wall': time.monotonic()})
                next_graph_wall = time.monotonic() + 2.
            if meta.first_clock_ns is not None:
                observed_sim_s = (meta.clock_ns - meta.first_clock_ns) / 1e9
                if observed_sim_s >= args.duration_sim:
                    reason = 'observed_complete_sim_duration'
                    break
    except BaseException as exc:
        error = type(exc).__name__ + ': ' + str(exc)
        reason = 'observer_exception'
    finally:
        try:
            writer.close()
        except Exception as exc:
            error = error or type(exc).__name__ + ': ' + str(exc)
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                error = error or type(exc).__name__ + ': ' + str(exc)
        if 'rclpy' in locals():
            try:
                rclpy.try_shutdown()
            except Exception as exc:
                error = error or type(exc).__name__ + ': ' + str(exc)
        source_mismatches = [name for name, expected in freeze['source_hashes'].items()
            if not (HERE / name).is_file() or digest(HERE / name) != expected]
        if source_mismatches or digest(args.freeze) != freeze_sha256:
            error = error or 'Observer source/freeze changed during observation'
        completed = (reason == 'observed_complete_sim_duration' and error is None
                     and all(meta.counts.values()) and meta.clock_regressions == 0)
        result = {'schema': 'actual_cloud_paired_qos_result/v1',
            'status': 'observation_complete' if completed else 'failed', 'reason': reason,
            'error': error, 'queue_error': writer.error, 'counts': meta.counts,
            'records_submitted': writer.submitted, 'records_written': writer.written,
            'records_drained': writer.submitted == writer.written and not writer.thread.is_alive(),
            'pid': os.getpid(), 'ros_domain': 79, 'first_clock_ns': meta.first_clock_ns,
            'last_clock_ns': meta.clock_ns, 'observed_sim_duration_s': observed_sim_s,
            'wall_duration_s': time.monotonic() - started_wall,
            'interrupt_signal': interrupted['signal'], 'source_hashes': source_refs,
            'freeze_sha256': freeze_sha256, 'source_mismatches': source_mismatches,
            'navigation_validation': 'unverified',
            'actuation_or_navigation_command_sent': False, 'payload_decoded_or_copied': False,
            'pairing_limitation': 'Paired readers share one lightweight process and publisher; reliable reader may affect publisher transport and does not prove baseline loss mechanism',
            'complete_does_not_mean_cloud_delivery_or_navigation_passed': True}
        atomic(output / 'probe_result.json', result)
    return 0 if completed else (128 + interrupted['signal'] if interrupted['signal'] else 2)


if __name__ == '__main__':
    raise SystemExit(main())
