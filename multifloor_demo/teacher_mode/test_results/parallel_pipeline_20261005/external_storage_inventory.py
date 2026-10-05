#!/usr/bin/env python3
"""Seal this campaign's explicitly owned external run storage after writers exit.

The project package inventories this manifest and run aliases. External payloads
remain in the recorded canonical directory; they are not claimed to be embedded
in the project package. No historical run is relocated or modified.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import stat

EXTERNAL = Path('/var/tmp/go2_teacher_parallel_20261005')
TEACHER = Path(__file__).resolve().parents[2]


def signature(path):
    s = path.lstat()
    return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)


def digest(path):
    before = signature(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        opened = os.fstat(fd)
        if not stat.S_ISREG(opened.st_mode):
            raise RuntimeError('Not a regular external evidence file: ' + str(path))
        if (opened.st_dev, opened.st_ino, opened.st_size,
                opened.st_mtime_ns, opened.st_ctime_ns) != before:
            raise RuntimeError('External evidence replaced during opening: ' + str(path))
        h = hashlib.sha256()
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
                h.update(block)
        if before != signature(path):
            raise RuntimeError('Writer still active: ' + str(path))
        return h.hexdigest(), before
    finally:
        os.close(fd)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--write', action='store_true')
    args = ap.parse_args()
    info = EXTERNAL.lstat()
    assert stat.S_ISDIR(info.st_mode) and not EXTERNAL.is_symlink()
    assert info.st_uid == os.getuid() and stat.S_IMODE(info.st_mode) == 0o700
    if not args.write:
        print(json.dumps({'mode': 'prepare_only', 'root': str(EXTERNAL),
                          'write_after_all_runtime_and_audit_writers_exit': True}))
        return
    output = Path(__file__).with_name('EXTERNAL_STORAGE_MANIFEST.json')
    if output.exists():
        raise RuntimeError('Refuse overwriting a sealed external inventory')
    entries, links, runs, endpoints = [], [], [], []
    fingerprints = {EXTERNAL: signature(EXTERNAL)}
    for run in sorted(EXTERNAL.iterdir()):
        assert run.is_dir() and not run.is_symlink() and run.stat().st_uid == os.getuid()
        alias = TEACHER / 'runs' / run.name
        assert alias.is_symlink() and alias.resolve() == run
        contract = json.loads((run / 'run_storage_contract.json').read_text())
        assert contract['canonical_run_dir'] == str(run)
        assert contract['project_run_alias'] == str(alias)
        runs.append({'canonical_run': str(run), 'project_alias': str(alias)})
        for directory, dirs, names in os.walk(run, followlinks=False):
            base = Path(directory)
            fingerprints[base] = signature(base)
            keep = []
            for name in sorted(dirs):
                p = base / name
                if p.is_symlink():
                    links.append({'path': str(p.relative_to(EXTERNAL)), 'target': os.readlink(p)})
                    fingerprints[p] = signature(p)
                else:
                    keep.append(name)
            dirs[:] = keep
            for name in sorted(names):
                p = base / name
                if p.is_symlink():
                    links.append({'path': str(p.relative_to(EXTERNAL)), 'target': os.readlink(p)})
                    fingerprints[p] = signature(p)
                elif p.is_file():
                    if name.endswith('.tmp'):
                        raise RuntimeError('Unfinished output: ' + str(p))
                    h, fp = digest(p)
                    fingerprints[p] = fp
                    entries.append({'path': str(p.relative_to(EXTERNAL)),
                                    'size_bytes': fp[2], 'sha256': h})
                else:
                    fp = signature(p)
                    fingerprints[p] = fp
                    mode = p.lstat().st_mode
                    endpoints.append({'path': str(p.relative_to(EXTERNAL)),
                                      'type': 'socket' if stat.S_ISSOCK(mode) else 'nonregular',
                                      'hashed': False})
        print(json.dumps({'inventoried_run': run.name, 'files_so_far': len(entries)}), flush=True)
    for p, fp in fingerprints.items():
        if signature(p) != fp:
            raise RuntimeError('External evidence changed during inventory: ' + str(p))
    result = {'schema': 'go2_owned_external_storage_inventory/v1',
              'created_UTC': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'canonical_storage_root': str(EXTERNAL), 'owner_uid': os.getuid(),
              'root_mode': '0700', 'simulation_only': True,
              'historical_runs_relocated': False, 'payloads_embedded_in_project': False,
              'runs': runs, 'files': entries, 'symlinks_not_followed': links,
              'nonregular_endpoints_not_hashed': endpoints,
              'total_size_bytes': sum(e['size_bytes'] for e in entries),
              'file_count': len(entries)}
    with output.open('x') as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'manifest': str(output), 'sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
                      'file_count': len(entries), 'total_size_bytes': result['total_size_bytes']}))


if __name__ == '__main__':
    main()
