"""Offline-only equivalent-computation candidate. Never imported by production.

The geometry algorithms, point order, predicates, thresholds, and rejection
reasons are unchanged. Only exact binary64 scalar representation and redundant
serialization within an immutable certification job are changed.
"""
from contextvars import ContextVar
import hashlib
from pathlib import Path

import numpy as np

EXPECTED_FROZEN_SHA256 = '386e62167f66fe593d3aee538ae162a58abdb01cbc08da32e9433058de795aee'


def hull_binary64(points):
    # np.float64 -> Python float preserves binary64 values. tolist() converts
    # once in C, avoiding hundreds of thousands of NumPy scalar operations.
    # The sorted unique order and <= 0 collinear rejection are unchanged.
    pts = sorted(set(map(tuple, np.asarray(points).tolist())))
    if len(pts) < 3:
        return []
    lower = []
    for p in pts:
        while len(lower) >= 2:
            a, b = lower[-2], lower[-1]
            if (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]) > 0:
                break
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2:
            a, b = upper[-2], upper[-1]
            if (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]) > 0:
                break
            upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def install(core):
    """Patch only this isolated in-memory copy of the pinned frozen module.

    Context-local map caching is valid only for the owned, immutable job input.
    No cache is carried between calls; current time/source ages are not cached.
    The original hash function still generates every serialized byte.
    """
    if hashlib.sha256(Path(core.__file__).read_bytes()).hexdigest() != EXPECTED_FROZEN_SHA256:
        raise ValueError('Offline candidate requires the reviewed frozen source')
    original_sha = core._sha
    original_certify = core.certify_corridor
    context = ContextVar('offline_corridor_immutable_snapshot_cache', default=None)

    def sha_once(value):
        active = context.get()
        if active is not None and value is active['snapshot']:
            if active['digest'] is None:
                active['digest'] = original_sha(value)
            return active['digest']
        return original_sha(value)

    def certify_once(path_receipt, map_snapshot, support_snapshot, actual_state, limits, now_ns):
        token = context.set(dict(snapshot=map_snapshot, digest=None))
        try:
            return original_certify(path_receipt, map_snapshot, support_snapshot, actual_state, limits, now_ns)
        finally:
            context.reset(token)

    core._hull = hull_binary64
    core._sha = sha_once
    core.certify_corridor = certify_once
    return core
