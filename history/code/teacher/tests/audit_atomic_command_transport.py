#!/usr/bin/env python3
"""Offline source-isolated atomic command transport review; no ROS or signals.

Only AST-selected actual transport classes/consumer function are executed.
All writes are in the new report directory; live runtime files are read only.
"""
from __future__ import annotations
import argparse
import ast
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
import types
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def selected_source(path, names):
    source = path.read_text()
    tree = ast.parse(source, filename=str(path))
    body = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names]
    if {node.name for node in body} != set(names):
        raise RuntimeError('Missing requested source definitions')
    return ast.fix_missing_locations(ast.Module(body=body, type_ignores=[]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    sources = out / 'sources'
    sources.mkdir()
    files = {
        'bridge.py': ROOT / 'navigation/bridge.py',
        'runtime_io.py': ROOT / 'navigation/runtime_io.py',
        'worker.py': ROOT / 'policy/worker.py',
        'executed_audit.py': Path(__file__).resolve(),
    }
    for name, path in files.items():
        shutil.copyfile(path, sources / name)
    ns = dict(fcntl=fcntl, json=json, math=math, os=os, Path=Path, tempfile=tempfile,
              threading=threading, time=time, hashlib=hashlib, np=np)
    for name, defs in [('bridge.py', ['CommandFile']), ('runtime_io.py', ['LatestCommandWriter']),
                       ('worker.py', ['navigation_command'])]:
        exec(compile(selected_source(sources / name, defs), str(sources / name), 'exec'), ns)
    CommandFile, LatestCommandWriter, consumer = (ns[name] for name in ('CommandFile', 'LatestCommandWriter', 'navigation_command'))
    command_ast = selected_source(sources / 'bridge.py', ['CommandFile'])
    assert not any(isinstance(node, ast.Attribute) and node.attr == 'fsync' for node in ast.walk(command_ast))
    checks = {}
    details = {}

    def envelope(sequence, padding, wall=None, stop=False):
        wall = time.monotonic() if wall is None else wall
        return {'schema_version': 1, 'mode': 'scan_slam', 'source': 'scan_slam',
                'sequence': sequence, 'acceptance': {'sha256': 'offline-only'},
                'monotonic_wall': wall, 'sim_time': 100.,
                'stamp': {'monotonic_wall': wall, 'ros_sim_time_ns': 100000000000},
                'healthy': True, 'stop_requested': stop, 'command': [0., 0., 0.] if stop else [.135, 0., -.01],
                'padding': 'x' * padding, 'padding_size': padding}

    # Actual same-directory NamedTemporaryFile + close + replace, with concurrent readers.
    target = out / 'concurrent_command.json'
    writer = CommandFile(target)
    writer.write(envelope(1, 8192))
    before_lock = target.read_bytes()
    try:
        duplicate = CommandFile(target)
    except RuntimeError:
        checks['exclusive_writer_lock_rejects_second_owner'] = True
    else:
        duplicate.close()
        checks['exclusive_writer_lock_rejects_second_owner'] = False
    assert target.read_bytes() == before_lock
    seen = []
    reader_errors = []
    stop = threading.Event()

    def reader():
        local = []
        while not stop.is_set():
            try:
                raw = target.read_bytes()
                value = json.loads(raw)
                assert value['padding'] == 'x' * value['padding_size']
                assert 1 <= value['sequence'] <= 201
                assert value['stamp']['monotonic_wall'] == value['monotonic_wall']
                local.append(value['sequence'])
            except Exception as exc:
                reader_errors.append(f'{type(exc).__name__}: {exc}')
        seen.extend(local)

    readers = [threading.Thread(target=reader) for _ in range(4)]
    for thread in readers:
        thread.start()
    begin = time.monotonic()
    for sequence in range(2, 202):
        writer.write(envelope(sequence, 131072 if sequence % 2 else 8))
    elapsed = time.monotonic() - begin
    stop.set()
    for thread in readers:
        thread.join()
    checks['concurrent_reads_are_complete_old_or_new_json'] = bool(seen) and not reader_errors
    checks['final_atomic_payload_matches_last_envelope'] = json.loads(target.read_text())['sequence'] == 201
    details['concurrency'] = {'readers': 4, 'writes': 200, 'complete_reads': len(seen),
        'distinct_sequences_seen': len(set(seen)), 'errors': reader_errors, 'elapsed_wall_s': elapsed,
        'alternating_padding_bytes': [8, 131072], 'benchmark_claim': False}

    # Serialization failure is before temp creation; injected flush/rename faults retain old destination.
    stable = target.read_bytes()
    bad = envelope(202, 1)
    bad['command'][0] = float('nan')
    try:
        writer.write(bad)
    except ValueError:
        checks['nonfinite_serialization_rejected_without_replacing_old_command'] = target.read_bytes() == stable
    else:
        checks['nonfinite_serialization_rejected_without_replacing_old_command'] = False

    actual_tempfile = ns['tempfile']

    class FlushFailure:
        def __init__(self, **kwargs):
            self.inner = tempfile.NamedTemporaryFile(**kwargs)
            self.name = self.inner.name
        def __enter__(self):
            self.inner.__enter__()
            return self
        def __exit__(self, *args):
            return self.inner.__exit__(*args)
        def write(self, value):
            return self.inner.write(value)
        def flush(self):
            raise OSError('offline injected flush failure')

    ns['tempfile'] = types.SimpleNamespace(NamedTemporaryFile=FlushFailure)
    try:
        writer.write(envelope(202, 1))
    except OSError:
        checks['flush_failure_retains_complete_old_destination'] = target.read_bytes() == stable
    else:
        checks['flush_failure_retains_complete_old_destination'] = False
    finally:
        ns['tempfile'] = actual_tempfile

    class RenameFailure:
        def __init__(self, name):
            self.real = Path(name)
        def replace(self, destination):
            raise OSError('offline injected replace failure')
        def unlink(self, **kwargs):
            return self.real.unlink(**kwargs)

    ns['Path'] = RenameFailure
    try:
        writer.write(envelope(202, 1))
    except OSError:
        checks['rename_failure_retains_complete_old_destination'] = target.read_bytes() == stable
    else:
        checks['rename_failure_retains_complete_old_destination'] = False
    finally:
        ns['Path'] = Path
    checks['failed_write_temp_files_cleaned'] = not list(out.glob('.concurrent_command.json.*'))
    writer.close()

    # Deliberately stall first write; pending motion is replaced by latest controlled stop.
    delayed_path = out / 'delayed_command.json'
    underlying = CommandFile(delayed_path)

    class DelayedWriter:
        def __init__(self):
            self.started = threading.Event()
            self.release = threading.Event()
            self.calls = 0
        def write(self, value):
            self.calls += 1
            if self.calls == 1:
                self.started.set()
                if not self.release.wait(2.):
                    raise RuntimeError('offline timeout')
            underlying.write(value)

    delayed = DelayedWriter()
    history = out / 'actually_written_history.jsonl'
    transport = LatestCommandWriter(delayed, history)
    first = envelope(1, 0)
    transport.submit(first)
    assert delayed.started.wait(1.)
    pending_values = [envelope(seq, 0) for seq in range(2, 101)]
    for value in pending_values:
        transport.submit(value)
    final = envelope(101, 0, stop=True)
    transport.submit(final)
    # Wall TTL alone must expire, even with unchanged fresh simulation clock.
    time.sleep(.36)
    delayed.release.set()
    transport.close()
    written = [json.loads(line) for line in history.read_text().splitlines()]
    checks['bounded_latest_queue_writes_only_inflight_and_latest_stop'] = [v['sequence'] for v in written] == [1, 101]
    checks['latest_zero_request_supersedes_pending_motion'] = json.loads(delayed_path.read_text())['command'] == [0., 0., 0.]
    checks['transport_preserves_original_envelope_and_source_stamps'] = all(
        all(row[key] == original[key] for key in original) for row, original in zip(written, [first, final]))
    checks['superseded_sequences_are_explicit'] = transport.superseded == 99 and transport.written == 2
    checks['append_history_records_actual_writes_and_transport_delays'] = all(
        row['transport_written_monotonic_wall'] >= row['transport_write_started_monotonic_wall']
        >= row['monotonic_wall'] for row in written)
    # Read the delayed first envelope via ACTUAL consumer code, using its original wall time.
    underlying.write(first)
    sequence = {}
    command, expired, reason = consumer(delayed_path, 100., 'offline-only', sequence)
    checks['late_motion_delivery_still_expires_at_unchanged_wall_300ms_ttl'] = (
        bool(expired) and command.tolist() == [0., 0., 0.] and sequence['wall_age_s'] > .3
        and sequence['sim_age_s'] == 0. and sequence['read_status'] == 'valid_but_unhealthy_or_stale')
    late_result = {'expired': bool(expired), 'reason': reason, 'command': command.tolist(), **sequence}
    fresh = envelope(2, 0)
    underlying.write(fresh)
    fresh_seq = {}
    command, expired, reason = consumer(delayed_path, 100., 'offline-only', fresh_seq)
    checks['fresh_motion_is_accepted_after_delayed_stale_envelope'] = not expired and command.tolist() == fresh['command'] and fresh_seq['read_status'] == 'accepted'
    # Simulation TTL independently rejects an otherwise wall-fresh envelope.
    sim_seq = {}
    command, expired, reason = consumer(delayed_path, 100.31, 'offline-only', sim_seq)
    checks['simulation_300ms_ttl_also_unchanged'] = bool(expired) and command.tolist() == [0., 0., 0.] and sim_seq['sim_age_s'] > .3
    details['delayed_transport'] = {'actually_written_sequences': [v['sequence'] for v in written],
        'superseded_pending_envelopes': transport.superseded, 'write_count': transport.written,
        'late_motion_consumer_result': late_result, 'original_first_envelope': first,
        'original_latest_stop_envelope': final}
    underlying.close()

    result = {'schema': 'teacher_atomic_transport_offline_review/v1', 'passed': all(checks.values()),
        'source_hashes': {name: sha(sources / name) for name in files},
        'execution': 'Offline AST-selected actual classes/function; no ROS/Kit/Gazebo imports or signals',
        'checks': checks, 'details': details,
        'boundaries': [
            'No power-loss durability guarantee; file close/rename do not fsync data or directory.',
            'Same-directory atomic replacement gives complete old/new JSON to ordinary concurrent readers on this local filesystem.',
            'Old file fsync without directory fsync also did not guarantee rename persistence across power loss.',
            'Serialization, filesystem metadata and append history writes may still delay transport; no live latency or physical stop proof.',
            'Fault injection raises Python exceptions before successful rename; no live process crash/power cut was executed.',
            'Consumer emits zero velocity request on stale input; actual Teacher parking requires independent physical verification.',
        ]}
    (out / 'review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': result['passed'], 'checks': len(checks), 'complete_reads': len(seen),
                      'output': str(out), 'review_sha256': sha(out / 'review.json')}))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
