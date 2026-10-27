"""Same-read owner observation and bounded replay; no ROS or actor operations.

The capsule records one successful original verifier observation. It is an
owned byte receipt, not a signature or a replacement acceptance predicate.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import json
import os
from pathlib import Path

SCHEMA = 'r85_dynamic_owner_verified_sources/v1'
BINDING_SCHEMA = 'r85_dynamic_owner_verified_sources_binding/v1'
CAPSULE_NAME = 'r85_dynamic_owner_verified_sources.json'
MAX_CAPSULE_BYTES = 24*1024**2
SOURCE_NAMES = dict(verifier='r85_dynamic_obstacle_verifier.py',
                    evidence_io='r85_dynamic_evidence_io.py', owner='r85_request_runtime.py')
JSON_NAMES = dict(contract='r85_dynamic_task_contract.json', runtime_state='r85_dynamic_obstacle_state.json')
LOG_ROLES = ('journal', 'native_guard', 'pretrigger_monitoring')


def _require(ok, reason):
    if not ok:
        raise ValueError(reason)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _sources(verifier, owner_source):
    # Module locations are descriptive; source bytes are never launch authority.
    return dict(verifier=dict(path=str(Path(verifier.__file__).resolve())),
                evidence_io=dict(path=str(Path(__file__).resolve())),
                owner=dict(path=str(Path(owner_source).resolve())))






class CaptureIO:
    """Observe the original read calls once, without rereading mutable state."""
    def __init__(self, run, request_id, verifier_module, scope, *, owner_source):
        self.run = Path(run).resolve()
        self.request_id = request_id
        self.verifier = verifier_module
        self.scope = copy.deepcopy(scope)
        _require(request_id == self.run.name+':r85:f1_to_f3:1', 'Foreign capture request')
        self.source_files = _sources(verifier_module, owner_source)
        self.json_sources = {}
        self.log_sources = {}
        self._seal_attempted = False

    def read_json(self, path, expected=None):
        path = Path(path).resolve()
        roles = [k for k, name in JSON_NAMES.items() if path == self.run/name]
        _require(len(roles) == 1, 'Unexpected captured JSON path')
        role = roles[0]
        _require(role not in self.json_sources and not self._seal_attempted, 'Duplicated/late JSON capture')

        def observe(original_path, raw, doc, binding):
            _require(Path(original_path).resolve() == path and binding == dict(
                path=str(path), sha256=_sha(raw), bytes=len(raw)), 'Same-read JSON binding differs')
            # raw is the exact local bytes object from the original _read_json.
            self.json_sources[role] = dict(source=copy.deepcopy(binding), raw=raw)

        return self.verifier._read_json(path, expected, observe_raw=observe)

    def lines(self, run, filename):
        _require(Path(run).resolve() == self.run and not self._seal_attempted, 'Foreign/late log capture')
        path = self.verifier._inside(self.run, filename)
        _require(str(path) not in self.log_sources, 'Duplicated log capture')
        rows, source = self.verifier._lines(self.run, filename)
        self.log_sources[str(path)] = copy.deepcopy(source)
        return rows, source

    def seal(self, owner_result):
        _require(not self._seal_attempted, 'Capsule seal already attempted')
        self._seal_attempted = True
        _require(owner_result.get('passed') is True and owner_result.get('state') == 'passed',
                 'Only original successful verification can be sealed')
        _require(owner_result.get('run_id') == self.run.name and owner_result.get('request_id') == self.request_id,
                 'Foreign owner result')
        _require(set(self.json_sources) == set(JSON_NAMES), 'Missing/duplicated JSON observations')
        receipt = owner_result['source_receipt']
        _require(set(receipt) == set(JSON_NAMES)|set(LOG_ROLES), 'Original source receipt shape differs')
        for role in JSON_NAMES:
            _require(receipt[role] == self.json_sources[role]['source'], 'Owner JSON receipt differs: '+role)
        _require(len(self.log_sources) == 3 and {v['path'] for v in self.log_sources.values()} ==
                 {receipt[k]['path'] for k in LOG_ROLES}, 'Missing/duplicated log observations')
        for role in LOG_ROLES:
            _require(receipt[role] == self.log_sources.get(receipt[role]['path']), 'Owner log receipt differs: '+role)
        current = _sources(self.verifier, self.source_files['owner']['path'])
        state = self.json_sources['runtime_state']
        capsule = dict(schema=SCHEMA, run_id=self.run.name, request_id=self.request_id, run_dir=str(self.run),
            owner_result_canonical_sha256=_sha(_canonical(owner_result).encode()), source_files=current,
            json_sources=dict(contract=dict(source=self.json_sources['contract']['source']),
                runtime_state=dict(source=state['source'], raw_base64=base64.b64encode(state['raw']).decode('ascii'))),
            log_sources={k: copy.deepcopy(receipt[k]) for k in LOG_ROLES},
            storage_accounting=dict(location='same_owned_run', maximum_capsule_bytes=MAX_CAPSULE_BYTES,
                existing_round_and_session_storage_budget_unchanged=True),
            authority='owned_same_read_observation_not_signature_or_new_acceptance_predicate')
        raw = (_canonical(capsule)+'\n').encode()
        _require(len(raw) <= MAX_CAPSULE_BYTES, 'Capsule byte budget exceeded')
        path = self.run/CAPSULE_NAME
        # Exclusive, read-only at creation; an incomplete write can never be
        # retried or silently replaced by another successful observation.
        fd = os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW, 0o444)
        with os.fdopen(fd, 'wb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        _require(path.read_bytes() == raw, 'Capsule durability readback differs')
        return dict(schema=BINDING_SCHEMA, path=str(path), sha256=_sha(raw), bytes=len(raw))


class _CapturedJSONPath:
    """Read-only path view so the original JSON parser/checks see sealed bytes."""
    def __init__(self, path, raw):
        self.path, self.raw = path, raw
    def is_file(self):
        return True
    def read_bytes(self):
        return self.raw
    def resolve(self):
        return self.path
    def __str__(self):
        return str(self.path)


class ReplayIO:
    """Replay exact state bytes and original real-log prefixes after closure.

Original source paths resolve through this run's manifest and source archive;
future installed source edits do not rewrite an old run's source identity.
"""
    def __init__(self, run, capsule_binding, verifier_module, scope=None, owner_result=None):
        self.run = Path(run).resolve()
        self.verifier = verifier_module
        _require(isinstance(capsule_binding, dict) and capsule_binding.get('schema') == BINDING_SCHEMA,
                 'Missing capsule binding')
        path = self.run/CAPSULE_NAME
        _require(Path(capsule_binding['path']) == path and not path.is_symlink(), 'Foreign capsule path')
        _require(path.is_file() and path.stat().st_size <= MAX_CAPSULE_BYTES, 'Missing/oversize capsule')
        raw = path.read_bytes()
        _require(len(raw) <= MAX_CAPSULE_BYTES and type(capsule_binding['bytes']) is int and
                 len(raw) == capsule_binding['bytes'] and _sha(raw) == capsule_binding['sha256'], 'Capsule bytes/SHA differ')
        self.capsule = json.loads(raw)
        _canonical(self.capsule)
        c = self.capsule
        _require(c.get('schema') == SCHEMA and c.get('run_id') == self.run.name and c.get('run_dir') == str(self.run)
                 and c.get('request_id') == self.run.name+':r85:f1_to_f3:1', 'Foreign capsule run/request')
        _require(isinstance(owner_result, dict) and owner_result.get('passed') is True and
                 owner_result.get('run_id') == self.run.name and owner_result.get('request_id') == c['request_id'] and
                 _sha(_canonical(owner_result).encode()) == c['owner_result_canonical_sha256'], 'Owner result/capsule SHA differs')
        if scope is None:
            scope, _ = verifier_module._read_json(self.run/'navigation_scope.json')
        self.source_archive_bindings = self._archive_sources(scope)
        _require(set(c['json_sources']) == set(JSON_NAMES) and set(c['log_sources']) == set(LOG_ROLES),
                 'Capsule source roles differ')
        receipt = owner_result['source_receipt']
        _require(set(receipt) == set(JSON_NAMES)|set(LOG_ROLES), 'Saved source receipt shape differs')
        for role, name in JSON_NAMES.items():
            source = c['json_sources'][role]['source']
            _require(source == receipt[role] and source['path'] == str(self.run/name), 'Foreign sealed JSON source')
        _require(all(c['log_sources'][k] == receipt[k] for k in LOG_ROLES), 'Saved prefix receipt differs')
        _require(len({c['log_sources'][k]['path'] for k in LOG_ROLES}) == 3, 'Duplicated sealed log source')
        for role, filename in dict(journal='r85_dynamic_journal.jsonl', native_guard='navigation_guard_history.jsonl').items():
            _require(c['log_sources'][role]['path'] == str(self.run/filename), 'Foreign sealed log role')
        state = c['json_sources']['runtime_state']
        state_raw = base64.b64decode(state['raw_base64'], validate=True)
        _require(len(state_raw) <= 16*1024**2 and len(state_raw) == state['source']['bytes'] and
                 _sha(state_raw) == state['source']['sha256'], 'Sealed state bytes/SHA differ')
        self.state_raw = state_raw
        self.used_json, self.used_logs = set(), set()
        self.report = dict(schema='r85_dynamic_bounded_replay_observation/v1', capsule_binding=copy.deepcopy(capsule_binding),
            source_archive_bindings=self.source_archive_bindings, current_closed_state=None,
            current_closed_source_failure=None, late_append={})
        try:
            doc, source = verifier_module._read_json(self.run/JSON_NAMES['runtime_state'])
            self.report['current_closed_state'] = dict(source=source,
                differs_from_owner_read=source != state['source'], runtime_failure=doc.get('failure'))
            if doc.get('failure') is not None:
                self.report['current_closed_source_failure'] = 'Current runtime state failure: '+str(doc['failure'])
        except (OSError, ValueError, TypeError, AttributeError) as error:
            self.report['current_closed_source_failure'] = type(error).__name__+': '+str(error)

    def _archive_sources(self, scope):
        # Replay checks actual recorded data, with the current project implementation.
        # It makes no claim to re-execute an archived source version.
        return {}

    def read_json(self, path, expected=None):
        path = Path(path).resolve()
        roles = [k for k, name in JSON_NAMES.items() if path == self.run/name]
        _require(len(roles) == 1 and roles[0] not in self.used_json, 'Unexpected/duplicated replay JSON')
        role = roles[0]
        self.used_json.add(role)
        source = self.capsule['json_sources'][role]['source']
        _require(expected is None or expected == source['sha256'], 'Replay expected JSON SHA differs')
        view = _CapturedJSONPath(path, self.state_raw) if role == 'runtime_state' else path
        doc, binding = self.verifier._read_json(view, source['sha256'])
        _require(binding == source, 'Replay JSON receipt differs')
        if role == 'contract':
            pretrigger = self.verifier._inside(self.run, doc['pretrigger_history_file'])
            _require(str(pretrigger) == self.capsule['log_sources']['pretrigger_monitoring']['path'], 'Foreign pretrigger source')
        return doc, binding

    def lines(self, run, filename):
        v = self.verifier
        _require(Path(run).resolve() == self.run, 'Foreign replay run')
        path = v._inside(self.run, filename)
        sources = self.capsule['log_sources']
        roles = [k for k, source in sources.items() if source['path'] == str(path)]
        _require(len(roles) == 1 and roles[0] not in self.used_logs, 'Unexpected/duplicated replay log')
        role = roles[0]
        self.used_logs.add(role)
        source = sources[role]
        length = source['complete_prefix_bytes']
        _require(type(length) is int and 0 <= length <= v.MAX_LOG_BYTES and v._hex(source['complete_prefix_sha256']),
                 'Malformed bounded prefix')
        final_size = path.stat().st_size
        _require(final_size <= v.MAX_LOG_BYTES, 'Log byte budget exceeded')
        _require(final_size >= length, 'Final log shorter than owner prefix')
        rows, offset, digest = [], 0, hashlib.sha256()
        with path.open('rb') as stream:
            while offset < length:
                raw = stream.readline(min(v.MAX_LINE_BYTES+1, length-offset))
                _require(raw and len(raw) <= v.MAX_LINE_BYTES, 'Source line byte budget exceeded/truncated prefix')
                _require(raw.endswith(b'\n'), 'Sealed log prefix ends inside a source line')
                d = json.loads(raw)
                v._canonical(d)
                rows.append(dict(data=d, source_file=str(path), source_offset=offset,
                    source_length=len(raw), source_line_sha256=v._sha(raw)))
                offset += len(raw)
                digest.update(raw)
                _require(len(rows) <= v.MAX_RECORDS, 'Source record budget exceeded')
        binding = dict(path=str(path), complete_prefix_bytes=offset, complete_prefix_sha256=digest.hexdigest())
        _require(binding == source, 'Actual log prefix bytes/SHA differ')
        # Original predicates call _verify_line again on each selected real-file
        # row. These descriptors therefore retain their exact real offsets/SHA.
        self.report['late_append'][role] = dict(final_file_bytes=final_size, owner_complete_prefix_bytes=length,
            bytes_after_owner_prefix=final_size-length, appended_after_owner_prefix=final_size > length)
        return rows, binding

    def assert_complete(self, replay_result, owner_result):
        _require(self.used_json == set(JSON_NAMES) and self.used_logs == set(LOG_ROLES), 'Replay source consumption incomplete')
        _require(replay_result == owner_result, 'Entire dynamic verifier receipt differs from owner saved result')
        return copy.deepcopy(self.report)
