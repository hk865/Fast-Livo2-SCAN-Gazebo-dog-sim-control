"""Lossless, same-stream dictionaries for three optional history archives.

Live files/messages and the shared writer's command transport are unchanged.
Every original JSON value is retained; this is not a field-dropping projection.
"""
from pathlib import Path
import copy
import hashlib
import json
import threading
from owned_guardian import bounded_shutdown_signals

FORMAT = 'v33_dictionary_jsonl/v1'
STATIC_PATHS = {
    'navigation_command_history.jsonl': [('acceptance',)],
    'navigation_status.jsonl': [('goals_definitions',), ('execution_bridge_safety', 'acceptance')],
    'mission46_status_history.jsonl': [('current_goals',), ('navigation', 'goals_definitions'),
        ('navigation', 'execution_bridge_safety', 'acceptance'), ('heading_alignment',), ('goal_region',)],
}


def canonical(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
        separators=(',', ':')).encode('utf-8')


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def configured(profile):
    value = profile.get('engineering_recording', {}).get('compact_histories', {'enabled': False})
    if not isinstance(value, dict) or type(value.get('enabled', False)) is not bool:
        raise ValueError('Explicit boolean compact history configuration required')
    if value.get('enabled', False):
        if value != dict(enabled=True, codec=FORMAT, streams=list(STATIC_PATHS),
                original_JSON_values_preserved=True, live_inputs_unchanged=True):
            raise ValueError('Only exact lossless three-stream history configuration allowed')
        return True
    return False


def context(run):
    run = Path(run)
    return {name: hashlib.sha256((run / name).read_bytes()).hexdigest()
        for name in ('input_profile.json', 'source_manifest.json')}


def compact_path(path):
    path = Path(path)
    return path.with_name(path.stem + '.compact.jsonl')


class Codec:
    def __init__(self, name, sources):
        if name not in STATIC_PATHS:
            raise ValueError('Unsupported compact stream')
        self.name = name; self.sources = dict(sources); self.seen = set()
        self.count = 0; self.chain = hashlib.sha256(); self.started = False; self.sealed = False

    def encode(self, data):
        if self.sealed: raise ValueError('Compact archive already closed')
        checksum = digest(data); projected = copy.deepcopy(data); records = []
        if not self.started:
            records.append(dict(kind='header', format=FORMAT, original_filename=self.name,
                source_sha256=self.sources, fidelity='all original JSON values; no whitespace identity claimed'))
            self.started = True
        references = []
        for keys in STATIC_PATHS[self.name]:
            parent = projected
            for key in keys[:-1]:
                parent = parent.get(key) if isinstance(parent, dict) else None
            if not isinstance(parent, dict) or keys[-1] not in parent or parent[keys[-1]] is None:
                continue
            value = parent[keys[-1]]; content = digest(value)
            if content not in self.seen:
                records.append(dict(kind='dictionary', sha256=content, value=value)); self.seen.add(content)
            parent[keys[-1]] = None
            references.append(dict(path=list(keys), sha256=content))
        self.count += 1; self.chain.update(checksum.encode('ascii'))
        records.append(dict(kind='frame', index=self.count, original_record_sha256=checksum,
            data=projected, references=references))
        return ''.join(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n' for row in records)

    def footer(self):
        if self.sealed: return ''
        self.sealed = True
        return json.dumps(dict(kind='footer', frames=self.count, record_chain_sha256=self.chain.hexdigest(),
            complete_archive=True, navigation_success=False)) + '\n'


def decode(lines, *, require_complete=True, expected_sources=None,
        expected_original_filename=None, integrity=None):
    """Strict streaming decoder; incomplete prefixes need explicit opt-in."""
    values = {}; count = 0; chain = hashlib.sha256(); header = None; footer = False
    report = integrity if integrity is not None else {}
    report.update(format=FORMAT, restored_frames=0, footer_present=False,
        truncated_tail_record=False, formal_acceptance_allowed=False,
        archive_integrity_complete=False, source_identity_checked=False)
    for line in lines:
        try: row = json.loads(line)
        except json.JSONDecodeError:
            if not require_complete and not line.endswith('\n'):
                report['truncated_tail_record'] = True
                return
            raise
        if footer: raise ValueError('Data after compact footer')
        kind = row.get('kind')
        if header is None:
            if kind != 'header' or row.get('format') != FORMAT or row.get('original_filename') not in STATIC_PATHS:
                raise ValueError('Missing compact archive header')
            if expected_sources is not None and row.get('source_sha256') != expected_sources:
                raise ValueError('Compact archive source identity mismatch')
            if expected_original_filename is not None and row.get('original_filename') != expected_original_filename:
                raise ValueError('Compact archive stream identity mismatch')
            header = row; report['source_identity_checked'] = expected_sources is not None
            continue
        if kind == 'dictionary':
            if digest(row['value']) != row['sha256'] or row['sha256'] in values:
                raise ValueError('Corrupt or duplicate dictionary entry')
            values[row['sha256']] = row['value']
        elif kind == 'frame':
            if row['index'] != count + 1: raise ValueError('Missing or reordered compact frame')
            data = copy.deepcopy(row['data']); used = set()
            for reference in row['references']:
                keys = tuple(reference['path'])
                if keys not in STATIC_PATHS[header['original_filename']] or keys in used:
                    raise ValueError('Unexpected or repeated archive reference')
                used.add(keys); parent = data
                for key in keys[:-1]: parent = parent[key]
                if parent[keys[-1]] is not None: raise ValueError('Archive reference overwrites data')
                parent[keys[-1]] = copy.deepcopy(values[reference['sha256']])
            checksum = digest(data)
            if checksum != row['original_record_sha256']: raise ValueError('Original record digest mismatch')
            count += 1; chain.update(checksum.encode('ascii'))
            report['restored_frames'] = count
            yield data
        elif kind == 'footer':
            if row.get('complete_archive') is not True or row.get('frames') != count or row.get('record_chain_sha256') != chain.hexdigest():
                raise ValueError('Incomplete or corrupt compact footer')
            footer = True; report['footer_present'] = True
        else: raise ValueError('Unknown compact archive record')
    if header is None or require_complete and not footer: raise ValueError('Compact archive incomplete')
    report['source_identity_checked'] = expected_sources is not None
    report['archive_integrity_complete'] = footer


def open_history(path, *, require_complete=True, integrity=None):
    path = Path(path)
    if path.exists() and compact_path(path).exists():
        raise ValueError('Ambiguous raw and compact histories')
    if path.exists():
        with path.open() as stream:
            for line in stream: yield json.loads(line)
    else:
        with compact_path(path).open() as stream:
            yield from decode(stream, require_complete=require_complete,
                expected_sources=context(path.parent), expected_original_filename=path.name, integrity=integrity)


def make_evidence_writer(run, profile):
    from runtime_io import EvidenceWriter
    if not configured(profile): return EvidenceWriter()
    sources = context(run)
    class ArchiveWriter(EvidenceWriter):
        def __init__(self):
            self.codecs = {}; self.archive_lock = threading.Lock(); self.archive_closed=False; self.close_error=None; super().__init__()
        def append(self, path, data):
            path = Path(path)
            if path.name not in STATIC_PATHS: return super().append(path, data)
            if path.parent.resolve() != Path(run).resolve(): raise ValueError('Wrong compact run directory')
            with self.archive_lock:
                if path not in self.codecs:
                    if path.exists() or compact_path(path).exists(): raise FileExistsError('History already exists')
                    self.codecs[path] = Codec(path.name, sources)
                codec = self.codecs[path]
                line = codec.encode(data); target = compact_path(path)
                def write():
                    with target.open('a') as stream: stream.write(line)
                self.enqueue(write)
        def close(self):
            if self.archive_closed:return
            if self.close_error:raise RuntimeError(self.close_error)
            with bounded_shutdown_signals():
                try:
                    super().close()  # Same original drain deadline; failed queues never seal.
                    for path, codec in self.codecs.items():
                        with compact_path(path).open('a') as stream: stream.write(codec.footer())
                    self.archive_closed=True
                except Exception as error:
                    self.error=self.close_error='Compact archive close failed: '+type(error).__name__+': '+str(error)
                    raise RuntimeError(self.error) from error
    return ArchiveWriter()


class ArchiveHistory:
    def __init__(self, path, sources):
        self.path = Path(path); self.codec = Codec(self.path.name, sources)
        if self.path.exists() or compact_path(self.path).exists(): raise FileExistsError('History already exists')
    def open(self, mode):
        if mode != 'a': raise ValueError('Archive history only appends')
        archive = self
        class Stream:
            def __enter__(self):
                self.file = compact_path(archive.path).open('a'); return self
            def write(self, line):
                self.file.write(archive.codec.encode(json.loads(line)))
                return len(line)
            def __exit__(self, *args): self.file.close()
        return Stream()
    def finish(self):
        if self.codec.started:
            with compact_path(self.path).open('a') as stream: stream.write(self.codec.footer())


def make_command_writer(writer, history, acceptance):
    from runtime_io import LatestCommandWriter
    if not configured(acceptance.get('profile', {})): return LatestCommandWriter(writer, history)
    if history is None or Path(history).name != 'navigation_command_history.jsonl':
        raise ValueError('Exact command history required for compact engineering mode')
    archive = ArchiveHistory(history, context(Path(history).parent))
    class ArchiveCommandWriter(LatestCommandWriter):
        # _run, submit, pending/zero replacement, actual write and timings are inherited verbatim.
        def __init__(self):
            super().__init__(writer, None); self.history = archive  # Before the first submit.
            self.archive_closed=False;self.close_error=None
        def close(self, final_envelope=None):
            if self.archive_closed:return
            if self.close_error:raise RuntimeError(self.close_error)
            with bounded_shutdown_signals():
                try:
                    super().close(final_envelope)
                    archive.finish();self.archive_closed=True
                except Exception as error:
                    self.error=self.close_error='Compact command archive close failed: '+type(error).__name__+': '+str(error)
                    raise RuntimeError(self.error) from error
    return ArchiveCommandWriter()
