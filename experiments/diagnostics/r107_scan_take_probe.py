"""Observe existing SCAN take/execute boundaries; never take or parse a message.

Install AFTER R99, detach BEFORE R99. MessageOnly, callback group, executor,
QoS and original exception behavior remain unchanged. No runtime file writes.
"""
from functools import wraps
import hashlib
import inspect
import json
import os
from pathlib import Path
import time
from types import MethodType

import experiments.diagnostics.r107_scan_transport_probe as probe

TOPICS = {'/demo/navigation/reference_attempt': 'attempt',
          '/demo/navigation/bspline': 'spline',
          '/demo/navigation/trajectory_metadata': 'metadata'}
_binding = None


def _observe(recorder, admission, stage, kind, **details):
    try:
        recorder.mark(admission, stage, kind=kind, **details)
    except Exception:
        recorder.errors += 1


def _attach(node, executor, subscriptions):
    """Already verified public objects; also used by explicit software fixtures."""
    global _binding
    admission = node._r88_admission
    recorder = admission.transport_probe
    original_take = executor._take_subscription
    had_take = '_take_subscription' in vars(executor)
    originals = [(sub, sub.callback, kind) for sub, kind in subscriptions]
    counter = [0]
    callbacks = []
    receipt = dict(enabled=True, installed=False, message_info_used=False,
                   source_timestamp_is_send_monotonic=False,
                   original_take_calls=0, no_message=0, original_take_errors=0,
                   original_callback_errors=0, observation_errors=0,
                   detached=False, detach_conflicts=0)

    def note_error():
        receipt['observation_errors'] += 1

    def callback_wrapper(original, kind):
        @wraps(original)
        def observed(msg):
            context = probe.TAKE_CONTEXT.get()
            if context is not None:
                try:
                    context[3] = time.monotonic_ns()
                    context[4] = time.thread_time_ns()
                    recorder.duration('take_to_callback', context[3] - context[2])
                except Exception:
                    note_error()
            _observe(recorder, admission, 18, kind)
            returned = False
            try:
                value = original(msg)
                returned = True
                return value
            except BaseException:
                receipt['original_callback_errors'] += 1
                raise
            finally:
                _observe(recorder, admission, 19, kind, outcome=1 if returned else -1)
        return observed

    @wraps(original_take)
    def observed_take(self, incoming):
        match = next((kind for sub, _, kind in originals if incoming is sub), None)
        if match is None:
            return original_take(incoming)
        counter[0] += 1
        context = [counter[0], time.monotonic_ns(), -1, -1, -1]
        token = probe.TAKE_CONTEXT.set(context)
        receipt['original_take_calls'] += 1
        execute = None
        try:
            _observe(recorder, admission, 16, match)
            try:
                execute = original_take(incoming)
            except BaseException:
                receipt['original_take_errors'] += 1
                raise
            finally:
                context[2] = time.monotonic_ns()
                try:
                    recorder.duration('take_call', context[2] - context[1])
                except Exception:
                    note_error()
                _observe(recorder, admission, 17, match, outcome=1 if execute is not None else 0)
        finally:
            probe.TAKE_CONTEXT.reset(token)
        if execute is None:
            receipt['no_message'] += 1
            return None

        @wraps(execute)
        async def observed_execute():
            token = probe.TAKE_CONTEXT.set(context)
            try:
                return await execute()
            finally:
                probe.TAKE_CONTEXT.reset(token)
        return observed_execute

    wrapped_take = MethodType(observed_take, executor)
    try:
        for sub, original, kind in originals:
            wrapped = callback_wrapper(original, kind)
            sub.callback = wrapped
            callbacks.append((sub, original, wrapped))
        executor._take_subscription = wrapped_take
    except BaseException:
        for sub, original, _ in reversed(callbacks):
            sub.callback = original
        raise
    receipt['installed'] = True
    _binding = (executor, original_take, had_take, wrapped_take, callbacks, receipt)
    return receipt


def install(node, executor):
    selection = os.environ.get(probe.ENVIRONMENT_KEY, '0')
    if selection not in ('0', '1'):
        raise ValueError('R107_SCAN_TRANSPORT_PROBE must be 0 or 1')
    if selection == '0':
        return dict(enabled=False, installed=False)
    if _binding is not None:
        raise RuntimeError('R107 already installed')
    import rclpy.executors as execution
    import rclpy.subscription as subscription
    import rclpy.time_source as time_source
    import experiments.diagnostics.r99_clock_probe as clock_probe
    sources = {}
    for module in (execution, subscription, time_source):
        path = Path(module.__file__).resolve()
        sha = hashlib.sha256(path.read_bytes()).hexdigest()
        if sha != clock_probe.EXPECTED_SHA256.get(path.name):
            raise RuntimeError('R107 unsupported pinned Jazzy source: '+path.name)
        sources[path.name] = dict(path=str(path), sha256=sha)
    take = executor._take_subscription
    clock_binding = clock_probe._binding
    expected = (execution.Executor._take_subscription if clock_binding is None
                else clock_binding[6].__func__)
    if (not inspect.ismethod(take) or take.__self__ is not executor
            or take.__func__ is not expected
            or (clock_binding is not None and clock_binding[3] is not executor)):
        raise RuntimeError('R107 requires original or this executor R99 take wrapper')
    admission = getattr(node, '_r88_admission', None)
    if admission is None or not isinstance(admission.transport_probe, probe.Recorder):
        raise RuntimeError('R107 admission recorder unavailable')
    found = []
    for topic, kind in TOPICS.items():
        matches = [sub for sub in node.subscriptions if sub.topic_name == topic]
        if len(matches) != 1:
            raise RuntimeError('R107 requires one original subscription: '+topic)
        sub = matches[0]
        if (type(sub) is not subscription.Subscription or sub.raw
                or sub._callback_type is not subscription.Subscription.CallbackType.MessageOnly
                or inspect.iscoroutinefunction(sub.callback)):
            raise RuntimeError('R107 requires pinned synchronous MessageOnly subscription')
        found.append((sub, kind))
    if len({id(sub.callback_group) for sub, _ in found}) != 1:
        raise RuntimeError('R107 SCAN sources must retain the common original callback group')
    receipt = _attach(node, executor, found)
    receipt.update(sources=sources, installed_after_r99=clock_binding is not None,
                   clock='CLOCK_MONOTONIC', no_payload_parse_or_hash=True)
    # Setup only, before executor.add_node/spin. This is NOT an admission/PASS receipt.
    files = (Path(__file__), Path(probe.__file__),
             Path(__file__).with_name('r88_async_admission.py'),
             Path(__file__).with_name('controller.py'))
    receipt['source_bindings'] = {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in files}
    receipt['process_identity'] = dict(admission.transport_probe.identity)
    receipt['monotonic_clock_implementation'] = time.get_clock_info('monotonic').implementation
    receipt['buffer_bytes'] = len(admission.transport_probe.buffer) * admission.transport_probe.buffer.itemsize
    receipt['setup_only_no_navigation_PASS'] = True
    (Path(node.run) / 'r107_scan_take_install.json').write_text(
        json.dumps(receipt, sort_keys=True, separators=(',', ':')) + '\n')
    return receipt


def detach():
    global _binding
    if _binding is None:
        return
    executor, original_take, had_take, wrapped_take, callbacks, receipt = _binding
    for sub, original, wrapped in reversed(callbacks):
        if sub.callback is wrapped:
            sub.callback = original
        else:
            receipt['detach_conflicts'] += 1
    if vars(executor).get('_take_subscription') is wrapped_take:
        if had_take:
            executor._take_subscription = original_take
        else:
            del executor._take_subscription
    else:
        receipt['detach_conflicts'] += 1
    receipt['detached'] = True
    _binding = None
