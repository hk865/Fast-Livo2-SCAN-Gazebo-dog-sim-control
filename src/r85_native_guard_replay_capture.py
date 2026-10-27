"""Explicit eligibility for the existing bounded native guard evidence buffer.

This module never evaluates a guard, changes a command, supplies a cloud, or
changes a capture budget. The caller must retain the original GuardReplayBuffer
implementation and its original 64 MiB retained / 48 MiB persisted limits.
"""
from pathlib import Path

CONTRACT = dict(schema='r85_native_guard_replay_capture/v1', enabled=True,
    observation_only=True, retained_array_payload_limit_B=64*1024**2,
    persisted_logical_and_allocated_limit_B=48*1024**2, complete_window_ns=2_000_000_000)


def enabled(node):
    profile = node.profile
    if profile.get('r85_session', {}).get('enabled') is not True:
        return False
    if profile.get('r85_task', {}).get('kind') != 'f1_to_f3':
        return False
    config = profile.get('r85_native_guard_replay_capture')
    if not isinstance(config, dict) or config.get('enabled') is not True:
        return False
    if config != CONTRACT:
        raise ValueError('R85 guard capture must preserve original complete-window and byte budgets')
    return node.request_id == Path(node.run).name+':r85:f1_to_f3:1'
