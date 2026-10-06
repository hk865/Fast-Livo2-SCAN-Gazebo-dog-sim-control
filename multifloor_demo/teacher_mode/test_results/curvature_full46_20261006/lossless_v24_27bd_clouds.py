#!/usr/bin/env python3
"""Fixed interrupted V24_27bd .bin/.npy archive, full verification, and separate guarded pruning."""
import argparse
import contextlib
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import tarfile
import time

RUN = Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_173509_closed_loop_cascade_clock_hold_curvature_on_V24_full46_r1_27bd')
CLOUD = RUN / 'navigation_cloud_arrays'
DEFAULT_ARCHIVE = Path('/var/tmp/go2_teacher_curvature_20261006_archives/v24_27bd')
CHUNK = 1024 * 1024
NAME = re.compile(r'\d+_\d+\.(bin|npy)\Z')
SCHEMA = 'teacher_v24_27bd_lossless_cloud_archive/v1'


def require(ok, why):
    if not ok:
        raise RuntimeError(why)


def real_dir(path):
    path = Path(os.path.abspath(path))
    for parent in reversed([path, *path.parents]):
        s = parent.lstat()
        require(stat.S_ISDIR(s.st_mode) and not stat.S_ISLNK(s.st_mode), f'non-real directory: {parent}')
    return path


def ident(s):
    return dict(dev=s.st_dev, ino=s.st_ino, size=s.st_size, mtime_ns=s.st_mtime_ns,
                ctime_ns=s.st_ctime_ns, nlink=s.st_nlink)


def source_stat(path):
    s = path.lstat()
    require(stat.S_ISREG(s.st_mode) and s.st_nlink == 1, f'not single-link regular file: {path}')
    return s


@contextlib.contextmanager
def source_open(path, expected=None):
    before = source_stat(path)
    if expected is not None:
        require(ident(before) == expected, f'source identity changed: {path}')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd, 'rb') as f:
        require(ident(os.fstat(f.fileno())) == ident(before), f'open race: {path}')
        yield f, before
        require(ident(os.fstat(f.fileno())) == ident(before), f'source changed while reading: {path}')
    require(ident(source_stat(path)) == ident(before), f'source replaced while reading: {path}')


def hash_stream(f):
    h, n = hashlib.sha256(), 0
    while True:
        b = f.read(CHUNK)
        if not b:
            return h.hexdigest(), n
        h.update(b)
        n += len(b)


def file_hash(path):
    with source_open(path) as (f, _):
        return hash_stream(f)[0]


def atomic_json(path, data):
    tmp = path.with_suffix(path.suffix + '.partial')
    with open(tmp, 'x', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    require(not path.exists(), f'refusing overwrite: {path}')
    os.rename(tmp, path)


def live_identity(pid, ticks):
    try:
        t = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return int(t[19]) == int(ticks)  # starttime is Linux stat field 22
    except FileNotFoundError:
        return False


INTERRUPT_ERROR = 'InterruptedError: Runner interrupted; cleanup is restricted to saved owned process groups'
REQUIRED_ROLES = {'worker', 'bridge', 'capture', 'navigation_stack', 'gazebo'}


def closed_run():
    """Prove saved process identities have ended; this is never a runtime/navigation PASS."""
    real_dir(RUN)
    real_dir(CLOUD)
    names = ['run_result.json', 'runtime_manifest.json', 'pipeline_normal_stop.json',
             'owned_processes_started.json', 'slam_loaded_binary.json']
    docs = {n: json.loads((RUN / n).read_text()) for n in names}
    result, runtime, stop, started, loaded = [docs[n] for n in names]
    worker = RUN / 'worker_result.json'
    require(not worker.exists() and not worker.is_symlink(), '27bd worker-result missing state changed')
    require(result.get('run') == str(RUN) and result.get('runtime_error') == INTERRUPT_ERROR and
            result.get('status') == 'failed', 'not exact interrupted 27bd result')
    require(runtime.get('run') == str(RUN) and runtime.get('error') == INTERRUPT_ERROR and
            runtime.get('runtime_status') == 'failed' and runtime.get('all_owned_and_children_clean') is False,
            'not original failed interrupted runtime; do not replace it with PASS')
    require(result.get('runtime_manifest_sha256') == file_hash(RUN / 'runtime_manifest.json'),
            'run-result runtime manifest hash mismatch')
    require(stop.get('run') == str(RUN) and stop.get('normal_completed') is True and
            stop.get('error') is None and stop.get('gate_errors') == [], 'normal-stop gate not clean')
    require(runtime.get('pipeline_normal_completed') is True and
            runtime.get('pipeline_normal_stop_sha256') == file_hash(RUN / 'pipeline_normal_stop.json'),
            'runtime normal-stop binding mismatch')
    q = stop['summary']
    require(q.get('schema') == 'staged_input_pipeline_v19/v1' and q.get('mode') == 'staged' and
            q.get('normal_completed') is True and q.get('context_valid_at_drain') is True and
            q.get('failure') == '' and q.get('uncommitted_packets') == [], 'pipeline drain not clean')
    require(type(q.get('accepted')) is int and q['accepted'] > 0 and
            q['accepted'] == q['delivered'] == q['committed'], 'accepted/delivered/committed differ')
    require(all(q.get(k) == 0 for k in ['pending', 'inflight', 'ready', 'bytes', 'canceled',
                                      'rejected_capacity', 'closed_rejections']), 'pipeline pending/rejected nonzero')
    owners = runtime.get('owned_processes', [])
    require(len(owners) == 5 and {x.get('role') for x in owners} == REQUIRED_ROLES and
            set(started) == REQUIRED_ROLES, 'five-role ownership closure missing')
    require(len({x['pid'] for x in owners}) == 5, 'duplicate owner PID')
    cleanup = runtime.get('cleanup', {})
    require(set(cleanup) == REQUIRED_ROLES, 'missing saved role cleanup receipt')
    for owner in owners:
        role = owner['role']; original = started[role]; c = cleanup[role]
        require(all(type(owner.get(k)) is int and owner[k] > 0 and owner[k] == original.get(k)
                    for k in ['pid', 'pgid', 'start_ticks']), 'saved owner identity changed: ' + role)
        require(c.get('started') is True and c.get('remaining_owned_group_members') == [] and
                not c.get('identity_mismatch') and not c.get('identity_unavailable'), 'owned cleanup incomplete: ' + role)
        require(not live_identity(owner['pid'], owner['start_ticks']), 'saved owned process still alive: ' + role)
    proof = runtime.get('launch_child_exit_evidence', {})
    children = proof.get('children', [])
    require(proof.get('expected_children') == proof.get('started_children') == 13 and
            len(children) == 13 and len({x.get('pid') for x in children}) == 13 and
            proof.get('died') == [] and proof.get('all_expected_children_clean') is True and
            all(x.get('clean_exit') is True for x in children), '13 launch child exit closure missing')
    # No child start-ticks were archived in this receipt: conservatively reject any current PID presence.
    require(all(type(x.get('pid')) is int and x['pid'] > 0 and
                not Path('/proc', str(x['pid'])).exists() for x in children), 'recorded launch child PID currently exists')
    mapping = loaded.get('mapping_identity', {}); saved = stop['saved_identity']
    require(loaded.get('run') == str(RUN) and loaded.get('verified') is True and
            all(mapping.get(k) == saved.get(k) for k in ['pid', 'pgid', 'start_ticks']) and
            saved['pid'] in {x['pid'] for x in children}, 'loaded SLAM / pipeline identity differs')
    require(not live_identity(saved['pid'], saved['start_ticks']), 'saved pipeline process still alive')
    return {n: file_hash(RUN / n) for n in names}


class DigestReader:
    def __init__(self, f):
        self.f, self.h, self.n = f, hashlib.sha256(), 0

    def read(self, n):
        b = self.f.read(n)
        self.h.update(b)
        self.n += len(b)
        return b


def verify_archive(archive, entries):
    require(archive.name == 'v24_27bd_cloud_arrays.tar.zst', 'unexpected archive name')
    p = subprocess.Popen(['/usr/bin/zstd', '-q', '-d', '-c', str(archive)], stdout=subprocess.PIPE)
    seen = 0
    try:
        with tarfile.open(fileobj=p.stdout, mode='r|') as tf:
            for member in tf:
                require(seen < len(entries), 'extra archive member')
                e = entries[seen]
                require(member.isreg() and member.name == e['name'] and member.size == e['size'],
                        f'archive member identity/size mismatch: {member.name}')
                h, n = hash_stream(tf.extractfile(member))
                require(h == e['sha256'] and n == e['size'], f'decompressed byte mismatch: {member.name}')
                seen += 1
        # Drain remaining padding; consume the stream so zstd exit status is meaningful.
        while p.stdout.read(CHUNK):
            pass
        require(p.wait() == 0, 'zstd decompression failed')
        require(seen == len(entries), 'missing archive members')
    finally:
        p.stdout.close()
        if p.poll() is None:
            p.terminate()
            p.wait()


def archive(root, audit_receipt):
    manifest = root / 'MANIFEST.json'
    require(not manifest.exists() and not (root / 'v24_27bd_cloud_arrays.tar.zst').exists(), 'archive already exists')
    metadata = closed_run()
    audit_receipt = Path(audit_receipt).absolute()
    source_stat(audit_receipt)
    audit_sha = file_hash(audit_receipt)  # Operator must first review this independent receipt.
    paths = sorted(CLOUD.iterdir())
    require(paths and all(NAME.fullmatch(p.name) for p in paths), 'unexpected cloud filename/type')
    entries = []
    source_bytes = sum(source_stat(path).st_size for path in paths)
    import shutil
    require(shutil.disk_usage(root).free > source_bytes + 256 * 1024 * 1024, 'archive filesystem lacks worst-case free space')
    tmp = root / 'v24_27bd_cloud_arrays.tar.zst.partial'
    require(not tmp.exists(), 'unfinished archive exists; inspect it first')
    with open(tmp, 'xb') as compressed:
        p = subprocess.Popen(['/usr/bin/zstd', '-q', '-1', '-T1', '-c'], stdin=subprocess.PIPE, stdout=compressed)
        try:
            with tarfile.open(fileobj=p.stdin, mode='w|') as tf:
                for path in paths:
                    with source_open(path) as (f, s):
                        reader = DigestReader(f)
                        info = tarfile.TarInfo(path.name)
                        info.size, info.mode, info.mtime = s.st_size, 0o600, s.st_mtime
                        tf.addfile(info, reader)
                        require(reader.n == s.st_size, f'incomplete source read: {path}')
                        entries.append(dict(name=path.name, size=s.st_size, allocated_bytes=s.st_blocks * 512,
                                            source_identity=ident(s), sha256=reader.h.hexdigest()))
            p.stdin.close()
            require(p.wait() == 0, 'zstd compression failed')
        finally:
            if not p.stdin.closed:
                p.stdin.close()
            if p.poll() is None:
                p.terminate()
                p.wait()
        compressed.flush()
        os.fsync(compressed.fileno())
    final = root / 'v24_27bd_cloud_arrays.tar.zst'
    os.rename(tmp, final)
    verify_archive(final, entries)
    require(closed_run() == metadata, 'run metadata changed during archival')
    require([p.name for p in sorted(CLOUD.iterdir())] == [e['name'] for e in entries], 'source directory changed')
    for e in entries:
        require(ident(source_stat(CLOUD / e['name'])) == e['source_identity'], 'source changed after archive read')
    gross, allocated, compressed_bytes = sum(e['size'] for e in entries), sum(e['allocated_bytes'] for e in entries), final.stat().st_size
    data = dict(schema=SCHEMA, run=str(RUN), source_directory=str(CLOUD), archive=str(final),
                archive_sha256=file_hash(final), archive_bytes=compressed_bytes, gross_original_bytes=gross,
                gross_original_allocated_bytes=allocated, global_net_logical_bytes=gross-compressed_bytes,
                source_device=CLOUD.stat().st_dev, archive_device=root.stat().st_dev,
                home_net_logical_bytes=gross-compressed_bytes if CLOUD.stat().st_dev == root.stat().st_dev else gross,
                verification='ALL_FILES_STREAM_DECOMPRESSED_SHA256_AND_SIZE_MATCH',
                independent_audit_receipt=str(audit_receipt), independent_audit_receipt_sha256=audit_sha,
                run_metadata_sha256=metadata, entries=entries, originals_deleted=False,
                worker_result_missing=True, runtime_clean_pass=False, navigation_pass=False,
                close_proof='EXACT_INTERRUPTED_RUNTIME_ALL_SAVED_IDENTITIES_ENDED_PIPELINE_DRAINED',
                targeted_audit_receipt=str(audit_receipt), targeted_audit_receipt_sha256=audit_sha)
    atomic_json(manifest, data)
    print(json.dumps({k: v for k, v in data.items() if k != 'entries'}, indent=2))


def load_manifest(root):
    with source_open(root / 'MANIFEST.json') as (f, _):
        m = json.load(f)
    require(m['schema'] == SCHEMA and m['run'] == str(RUN) and m['source_directory'] == str(CLOUD), 'foreign manifest')
    require(m['verification'] == 'ALL_FILES_STREAM_DECOMPRESSED_SHA256_AND_SIZE_MATCH', 'archive not verified')
    require(m['archive'] == str(root / 'v24_27bd_cloud_arrays.tar.zst'), 'foreign archive path')
    es = m['entries']
    require(es and all(NAME.fullmatch(e['name']) for e in es) and len({e['name'] for e in es}) == len(es), 'invalid manifest names')
    require(sum(e['size'] for e in es) == m['gross_original_bytes'], 'manifest size mismatch')
    require(file_hash(Path(m['archive'])) == m['archive_sha256'], 'archive changed')
    return m


def prune(root):
    require(not (root / 'PRUNE_RECEIPT.json').exists(), 'pruning already completed')
    m = load_manifest(root)
    require(closed_run() == m['run_metadata_sha256'], 'run metadata changed since archival')
    require(file_hash(Path(m['independent_audit_receipt'])) == m['independent_audit_receipt_sha256'], 'audit receipt changed')
    journal = root / 'PRUNE_JOURNAL.jsonl'
    done = {}
    if journal.exists():
        source_stat(journal)
        for line in journal.read_text().splitlines():
            e = json.loads(line)
            require(e['name'] not in done and e['status'] == 'deleted', 'invalid prune journal')
            done[e['name']] = e
    require(set(done) <= {e['name'] for e in m['entries']}, 'foreign prune journal')
    for e in m['entries']:
        path = CLOUD / e['name']
        if e['name'] in done:
            require(not path.exists() and not path.is_symlink(), 'journaled source reappeared')
        else:
            require(path.exists(), 'source absent without successful prune journal')
            require(ident(source_stat(path)) == e['source_identity'], f'source identity changed: {path}')
    # Re-decompress before any deletion. The archive SHA and source identities are also checked.
    verify_archive(Path(m['archive']), m['entries'])
    with open(journal, 'a', encoding='utf-8') as j:
        for e in m['entries']:
            if e['name'] in done:
                continue
            path = CLOUD / e['name']
            with source_open(path, e['source_identity']) as (f, _):
                h, n = hash_stream(f)
                require(h == e['sha256'] and n == e['size'], f'original bytes changed: {path}')
            # Final identity check immediately precedes the exact individual unlink.
            require(ident(source_stat(path)) == e['source_identity'], 'source identity changed before unlink')
            path.unlink()
            j.write(json.dumps(dict(name=e['name'], size=e['size'], sha256=e['sha256'], status='deleted',
                                    monotonic_wall_ns=time.monotonic_ns())) + '\n')
            j.flush()
            os.fsync(j.fileno())
    require(not list(CLOUD.iterdir()), 'unexpected files remain; do not remove them automatically')
    receipt = root / 'PRUNE_RECEIPT.json'
    require(not receipt.exists(), 'prune receipt already exists')
    atomic_json(receipt, dict(schema=SCHEMA, status='VERIFIED_ORIGINALS_PRUNED', run=str(RUN),
                             manifest_sha256=file_hash(root / 'MANIFEST.json'), count=len(m['entries']),
                             gross_original_bytes=m['gross_original_bytes'], archive_bytes=m['archive_bytes'],
                             global_net_logical_bytes=m['global_net_logical_bytes'],
                             home_net_logical_bytes=m['home_net_logical_bytes']))
    print(receipt)


def restore(root):
    m = load_manifest(root)
    real_dir(CLOUD)
    require(set(p.name for p in CLOUD.iterdir()) <= set(e['name'] for e in m['entries']), 'restore refuses foreign source files')
    verify_archive(Path(m['archive']), m['entries'])
    p = subprocess.Popen(['/usr/bin/zstd', '-q', '-d', '-c', m['archive']], stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=p.stdout, mode='r|') as tf:
            for member, e in zip(tf, m['entries']):
                require(member.name == e['name'] and member.isreg() and member.size == e['size'], 'restore member mismatch')
                path = CLOUD / e['name']
                if path.exists() or path.is_symlink():
                    with source_open(path) as (existing, _):
                        h_existing, n_existing = hash_stream(existing)
                    require(h_existing == e['sha256'] and n_existing == e['size'], f'refusing overwrite: {path}')
                    continue
                h, n = hashlib.sha256(), 0
                tmp = CLOUD / (e['name'] + '.restore_partial')
                with open(tmp, 'xb') as out, tf.extractfile(member) as src:
                    while True:
                        b = src.read(CHUNK)
                        if not b:
                            break
                        out.write(b); h.update(b); n += len(b)
                    out.flush(); os.fsync(out.fileno())
                require(h.hexdigest() == e['sha256'] and n == e['size'], f'restore mismatch: {path}')
                require(not path.exists() and not path.is_symlink(), f'restore target appeared: {path}')
                os.rename(tmp, path)
                os.utime(path, ns=(e['source_identity']['mtime_ns'], e['source_identity']['mtime_ns']))
        while p.stdout.read(CHUNK):
            pass
        require(p.wait() == 0, 'restore decompression failed')
    finally:
        p.stdout.close()
        if p.poll() is None:
            p.terminate(); p.wait()
    print(f'Restored {len(m["entries"])} original files; no original runtime/config/manifest files changed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['preflight', 'archive', 'prune', 'restore'])
    parser.add_argument('--archive-root', type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument('--audit-complete', action='store_true', help='Operator confirms the independent V24 interrupted audit has finished; required before archive/prune.')
    parser.add_argument('--audit-receipt', type=Path, help='Final independent V24 interrupted audit receipt, recorded by SHA; required for archive.')
    args = parser.parse_args()
    if args.command == 'preflight':
        metadata = closed_run()
        print(json.dumps(dict(schema=SCHEMA, run=str(RUN),
            status='CLOSED_INTERRUPTED_NOT_RUNTIME_OR_NAVIGATION_PASS', worker_result_missing=True,
            runtime_clean_pass=False, navigation_pass=False, run_metadata_sha256=metadata,
            targeted_audit='PENDING_OPERATOR_COMPLETION_RECEIPT_REQUIRED_BEFORE_ARCHIVE_PRUNE'), indent=2))
        return
    require(args.command == 'restore' or args.audit_complete, 'V24 interrupted independent audit must finish first')
    root = args.archive_root.absolute()
    require(root != CLOUD and CLOUD not in root.parents and RUN not in root.parents, 'archive must be outside original run')
    if not root.exists():
        real_dir(root.parent)
        root.mkdir(mode=0o700)
    real_dir(root)
    require(root.stat().st_uid == os.getuid() and stat.S_IMODE(root.stat().st_mode) == 0o700, 'archive root must be owned private 0700')
    lock = os.open(root / '.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        require(stat.S_ISREG(os.fstat(lock).st_mode) and os.fstat(lock).st_nlink == 1, 'unsafe lock file')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.command == 'archive':
            require(args.audit_receipt is not None, '--audit-receipt required')
            archive(root, args.audit_receipt)
        elif args.command == 'prune':
            prune(root)
        else:
            restore(root)
    finally:
        os.close(lock)


if __name__ == '__main__':
    main()
