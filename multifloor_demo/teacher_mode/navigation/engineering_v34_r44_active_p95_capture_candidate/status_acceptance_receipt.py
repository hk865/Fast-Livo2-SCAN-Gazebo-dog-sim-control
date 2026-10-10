"""Run-local, recoverable projection of a verified static status receipt only.

This does not authorize commands, replace bridge safety, or verify the scope.
The caller supplies the complete return value of the original verify_scope.
Control/worker source checks and full command acceptance remain unchanged.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import stat


SCHEMA = 'teacher_navigation_status_static_acceptance/v1'
FILENAME = 'navigation_status_static_acceptance.json'
MAX_BYTES = 256 * 1024
REFERENCE_KEYS = {'path', 'sha256', 'run_dir', 'receipt_ref'}
REF_KEYS = {'schema', 'file', 'bytes', 'file_sha256', 'receipt_sha256'}


def canonical(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                      separators=(',', ':')).encode('utf-8')


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def _read_bounded(path):
    # A fixed run-local regular file; never follow an arbitrary reference/symlink.
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        st = os.fstat(stream.fileno())
        if (not stat.S_ISREG(st.st_mode) or st.st_uid != os.geteuid()
                or st.st_size > MAX_BYTES or st.st_blocks * 512 > MAX_BYTES):
            raise ValueError('Static receipt is not an owned bounded regular file')
        raw = stream.read(MAX_BYTES + 1)
    if len(raw) > MAX_BYTES:
        raise ValueError('Static receipt exceeds bound')
    return raw


def _identity(receipt, run):
    if not isinstance(receipt, dict):
        raise ValueError('Complete original verified receipt required')
    if (receipt.get('run_dir') != str(run)
            or receipt.get('path') != str(run / 'navigation_scope.json')
            or not isinstance(receipt.get('sha256'), str)
            or len(receipt['sha256']) != 64
            or any(c not in '0123456789abcdef' for c in receipt['sha256'])):
        raise ValueError('Static receipt scope/run identity mismatch')


def restore_acceptance(acceptance, run):
    """Recover the exact original JSON receipt, or reject an invalid reference.

    Inline receipts are returned unchanged. This reader is for audits/consumers
    needing the full static receipt, never a substitute for original safety gates.
    """
    if not isinstance(acceptance, dict) or 'receipt_ref' not in acceptance:
        return acceptance
    run = Path(run).resolve()
    _identity(acceptance, run)
    ref = acceptance['receipt_ref']
    if (set(acceptance) != REFERENCE_KEYS or not isinstance(ref, dict)
            or set(ref) != REF_KEYS or ref['schema'] != SCHEMA
            or ref['file'] != FILENAME or type(ref['bytes']) is not int
            or not 0 < ref['bytes'] <= MAX_BYTES):
        raise ValueError('Unknown static receipt reference')
    raw = _read_bounded(run / FILENAME)
    if len(raw) != ref['bytes'] or sha256(raw) != ref['file_sha256']:
        raise ValueError('Static receipt bytes/hash mismatch')
    receipt = json.loads(raw)
    _identity(receipt, run)
    if (sha256(canonical(receipt)) != ref['receipt_sha256']
            or any(receipt.get(k) != acceptance[k] for k in ('path', 'sha256', 'run_dir'))):
        raise ValueError('Full static receipt hash/scope mismatch')
    return receipt


def restore_status(status, run):
    """Return a copy with only a projected bridge acceptance restored."""
    bridge = status.get('execution_bridge_safety')
    if not isinstance(bridge, dict):
        return status
    acceptance = bridge.get('acceptance')
    if not isinstance(acceptance, dict) or 'receipt_ref' not in acceptance:
        return status
    restored = dict(status)
    restored['execution_bridge_safety'] = dict(bridge)
    restored['execution_bridge_safety']['acceptance'] = restore_acceptance(acceptance, run)
    return restored


class StatusAcceptanceReceipt:
    """Project only when the actual bridge receipt equals the original verified one.

    One exclusive file creation at the first matching publication. Every later
    projection still compares the entire actual receipt and rechecks the saved
    file; no cached/omitted worker checks or status reads are introduced. A file
    collision, save failure, or any mismatch leaves that original status intact.
    """
    def __init__(self, run, verified_receipt, *, enabled=False):
        self.enabled = enabled is True
        self.saved = False
        self.save_failed = False
        self.fallback_reason = None
        self.run = Path(run).resolve()
        self.receipt = None
        self.receipt_bytes = None
        self.reference = None
        if not self.enabled:
            return
        try:
            self.receipt = copy.deepcopy(verified_receipt)
            _identity(self.receipt, self.run)
            self.receipt_bytes = canonical(self.receipt)
            full = self.receipt_bytes + b'\n'
            if len(full) > MAX_BYTES:
                raise ValueError('Full original static receipt exceeds bound')
            self.reference = {k: self.receipt[k] for k in ('path', 'sha256', 'run_dir')}
            self.reference['receipt_ref'] = dict(schema=SCHEMA, file=FILENAME,
                bytes=len(full), file_sha256=sha256(full), receipt_sha256=sha256(self.receipt_bytes))
        except (OSError, ValueError, TypeError, KeyError, OverflowError, RecursionError) as error:
            self.enabled = False
            self.fallback_reason = type(error).__name__ + ': ' + str(error)

    def _save_once(self):
        if self.save_failed:
            raise ValueError('Previous exclusive receipt save failed')
        if self.saved:
            return
        try:
            fd = os.open(self.run / FILENAME,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, 'wb') as stream:
                stream.write(self.receipt_bytes + b'\n')
                stream.flush()
                os.fsync(stream.fileno())
            # Reference emission only after complete close/read/hash/value check.
            if canonical(restore_acceptance(self.reference, self.run)) != self.receipt_bytes:
                raise ValueError('Saved receipt differs from original verified return')
            self.saved = True
        except (OSError, ValueError, TypeError, KeyError, OverflowError, RecursionError):
            self.save_failed = True
            raise

    def project(self, status):
        if not self.enabled:
            return status
        try:
            bridge = status.get('execution_bridge_safety')
            if not isinstance(bridge, dict) or 'acceptance' not in bridge:
                return status
            # Canonical comparison also distinguishes bool/int/float JSON types.
            if canonical(bridge['acceptance']) != self.receipt_bytes:
                self.fallback_reason = 'Actual bridge receipt differs from complete verified receipt'
                return status
            self._save_once()
            if _read_bounded(self.run / FILENAME) != self.receipt_bytes + b'\n':
                raise ValueError('Saved static receipt no longer matches original')
            projected = dict(status)
            projected['execution_bridge_safety'] = dict(bridge)
            projected['execution_bridge_safety']['acceptance'] = copy.deepcopy(self.reference)
            return projected
        except (OSError, ValueError, TypeError, KeyError, OverflowError, RecursionError) as error:
            self.fallback_reason = type(error).__name__ + ': ' + str(error)
            return status
