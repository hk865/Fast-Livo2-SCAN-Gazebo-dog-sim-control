"""Bounded archive close after native physics has stopped; no process signals.

Teacher inference, IPC responses, PD and owned-group escalation stay unchanged.
This helper is called by the owner only after its existing Gazebo cleanup.
"""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import re
import time
import zipfile

ARCHIVE_GRACE_S = 30.0
MAX_SAMPLES = 75_001  # Frozen original 1500 native seconds at 50 Hz, plus one.
IDENTITY_KEYS = ('pid', 'start_ticks', 'pgid', 'session', 'boot_id')


def await_archive_close(worker, saved, physics_cleanup, identity, validate_members,
                        *, abort_reason=lambda: None, clock=time.monotonic,
                        sleep=time.sleep):
    """Observe normal EOF/close, then let the caller use its original escalation.

    An expired wall/storage/guardian gate cancels the grace. A missing identity
    or still-live physics group cannot earn extra waiting time. Never signals.
    """
    started = clock()
    receipt = dict(schema='go2_worker_archive_close/v1', maximum_grace_s=ARCHIVE_GRACE_S,
                   wait_started_monotonic=started, waited_wall_s=0.0,
                   sent_signals=[], natural_worker_exit=False, reason=None)

    def finish(reason, rc=None):
        receipt.update(reason=reason, waited_wall_s=max(0.0, clock()-started),
                       worker_returncode=rc, natural_worker_exit=rc is not None)
        return receipt

    if worker is None:
        return finish('worker_not_started')
    if saved is None:
        return finish('worker_identity_unavailable')
    if not (physics_cleanup.get('started') is True
            and physics_cleanup.get('returncode') is not None
            and physics_cleanup.get('remaining_owned_group_members') == []
            and not physics_cleanup.get('identity_mismatch')
            and not physics_cleanup.get('identity_unavailable')):
        return finish('native_physics_stop_not_confirmed')
    deadline = started + ARCHIVE_GRACE_S
    while True:
        try:
            aborted = abort_reason()
            if aborted:
                return finish('grace_cancelled: '+str(aborted), worker.poll())
            leader = identity(worker.pid)
            if leader and any(leader.get(k) != saved.get(k) for k in IDENTITY_KEYS):
                return finish('worker_identity_mismatch')
            validate_members(saved)
            rc = worker.poll()
            if rc is not None:
                return finish('natural_worker_exit_observed', rc)
            if leader is None:
                return finish('live_worker_identity_unavailable')
            now = clock()
            if now >= deadline:
                return finish('archive_grace_expired')
            sleep(min(0.05, deadline-now))
        except Exception as error:
            return finish('archive_wait_rejected: '+type(error).__name__+': '+str(error))


def _hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_worker_archive(run):
    """Validate both array shapes, lengths and ZIP CRC without importing Actor.

    Does not repair or write an archive. Bounds come from the frozen 1500 s
    run, not from untrusted ZIP lengths. Header parsing never executes code.
    """
    run = Path(run)
    result = dict(schema='go2_worker_archive_integrity/v1', verified=False,
                  samples=None, arrays={}, files={}, error=None,
                  artifact_written_or_repaired=False)
    try:
        for name in ('worker_result.json', 'observations_actions.npz'):
            path = run/name
            if not path.is_file() or path.is_symlink():
                raise ValueError(name+' is missing or not a regular owned artifact')
            result['files'][name] = dict(bytes=path.stat().st_size)
        if result['files']['worker_result.json']['bytes'] > 1024*1024:
            raise ValueError('worker result exceeds metadata bound')
        if result['files']['observations_actions.npz']['bytes'] > 160_000_000:
            raise ValueError('worker archive exceeds frozen array bound')
        for name in result['files']:
            result['files'][name]['sha256'] = _hash(run/name)
        metadata = json.loads((run/'worker_result.json').read_text())
        samples = metadata.get('samples')
        if (type(samples) is not int or not 0 < samples <= MAX_SAMPLES
                or metadata.get('completed') is not True):
            raise ValueError('invalid completed/sample metadata')
        result.update(samples=samples, worker_fault=metadata.get('fault'))
        with zipfile.ZipFile(run/'observations_actions.npz') as archive:
            names = archive.namelist()
            if sorted(names) != ['actions.npy', 'observations.npy']:
                raise ValueError('archive members are missing, duplicated or unexpected')
            for name, width in [('observations.npy', 247), ('actions.npy', 12)]:
                entry = archive.getinfo(name)
                if entry.file_size > samples*width*8+4108:
                    raise ValueError(name+' exceeds bounded array length')
                with archive.open(entry) as stream:
                    magic = stream.read(8)
                    if magic[:6] != b'\x93NUMPY' or magic[6:] not in (b'\x01\x00', b'\x02\x00', b'\x03\x00'):
                        raise ValueError('invalid NPY magic/version')
                    size_bytes = 2 if magic[6] == 1 else 4
                    raw_length = stream.read(size_bytes)
                    if len(raw_length) != size_bytes:
                        raise ValueError('truncated NPY header length')
                    length = int.from_bytes(raw_length, 'little')
                    if not 0 < length <= 4096:
                        raise ValueError('NPY header exceeds bound')
                    raw_header = stream.read(length)
                    if len(raw_header) != length:
                        raise ValueError('truncated NPY header')
                    header = ast.literal_eval(raw_header.decode('utf-8' if magic[6] == 3 else 'latin1'))
                    if (set(header) != {'descr', 'fortran_order', 'shape'}
                            or type(header['shape']) is not tuple
                            or len(header['shape']) != 2
                            or any(type(dimension) is not int for dimension in header['shape'])
                            or header['shape'] != (samples, width)
                            or header['fortran_order'] is not False
                            or not isinstance(header['descr'], str)
                            or not re.fullmatch(r'[<>=|]f[48]', header['descr'])):
                        raise ValueError(name+' array contract differs')
                    expected = samples*width*int(header['descr'][-1])
                    if entry.file_size != 8+size_bytes+length+expected:
                        raise ValueError(name+' declared array length differs')
                    count = 0
                    while True:
                        chunk = stream.read(1024*1024)
                        if not chunk:
                            break
                        count += len(chunk)
                    if count != expected:
                        raise ValueError(name+' payload is truncated')
                    result['arrays'][name] = dict(shape=list(header['shape']),
                                                 dtype=header['descr'], payload_bytes=count,
                                                 crc32=entry.CRC, crc_verified=True)
        result['verified'] = True
    except Exception as error:
        result['error'] = type(error).__name__+': '+str(error)
    return result
