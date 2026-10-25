"""Owned immutable input boundary for asynchronous original qualification.

No ROS, node, Actor, source-clock synthesis or acceptance predicate lives here.
The main callback reads two bounded JSON byte strings and at most one line-size
of each log tail. The worker hashes/parses only those fixed complete prefixes.
Original paths and original verifier receipt fields are retained verbatim.
"""
from __future__ import annotations
import base64
import copy
import hashlib
import json
import os
from pathlib import Path
import stat
import time
import uuid

JSON_NAMES={'contract':'r85_dynamic_task_contract.json',
            'runtime_state':'r85_dynamic_obstacle_state.json'}
MAX_JSON_BYTES=16*1024**2
MAX_CAPTURE_BYTES=48*1024**2


def _require(ok,reason):
    if not ok:raise ValueError(reason)


def _sha(raw):return hashlib.sha256(raw).hexdigest()


class FrozenSnapshot:
    def __init__(self,run):
        self.run=run;self.json_sources={};self.logs={}
        self.captured_monotonic_wall=time.monotonic();self.closed=False
    def close(self):
        if self.closed:return
        self.closed=True;first_error=None
        for entry in self.logs.values():
            try:os.close(entry['fd'])
            except OSError as error:
                if first_error is None:first_error=error
        if first_error is not None:raise first_error


def _read_json_bytes(path):
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    with os.fdopen(fd,'rb')as stream:
        before=os.fstat(stream.fileno())
        _require(stat.S_ISREG(before.st_mode) and before.st_size<=MAX_JSON_BYTES,
                 'frozen_JSON_type_or_budget')
        raw=stream.read(MAX_JSON_BYTES+1);after=os.fstat(stream.fileno())
    _require(len(raw)<=MAX_JSON_BYTES and len(raw)==before.st_size
             and (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)==
                 (after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns),
             'frozen_JSON_changed_during_read')
    return raw


def freeze_inputs(verifier,run):
    """Main callback: bounded real reads, fixed complete-prefix lengths, no log hash."""
    run=Path(run).resolve();snapshot=FrozenSnapshot(run)
    try:
        for role,name in JSON_NAMES.items():
            path=verifier._inside(run,name)
            snapshot.json_sources[role]={'path':str(path),'raw':_read_json_bytes(path)}
        contract=json.loads(snapshot.json_sources['contract']['raw'])
        names=['r85_dynamic_journal.jsonl',contract['pretrigger_history_file'],
               'navigation_guard_history.jsonl','known_scene_bundles.jsonl',
               'known_scene_controller_inputs.jsonl']
        for name in names:
            path=verifier._inside(run,name)
            _require(str(path)not in snapshot.logs,'duplicate_frozen_log_path')
            fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
            try:
                info=os.fstat(fd)
                _require(stat.S_ISREG(info.st_mode) and info.st_size<=verifier.MAX_LOG_BYTES,
                         'frozen_log_type_or_budget')
                size=info.st_size;length=min(size,verifier.MAX_LINE_BYTES)
                tail=os.pread(fd,length,size-length)
                _require(len(tail)==length,'frozen_log_shortened_during_tail_read')
                last=tail.rfind(b'\n')
                _require(last>=0 or size<verifier.MAX_LINE_BYTES,
                         'frozen_log_partial_line_exceeds_budget')
                complete=0 if last<0 else size-length+last+1
                current=os.fstat(fd)
                _require(current.st_size>=size,'frozen_log_shortened_during_capture')
                snapshot.logs[str(path)]={'fd':fd,'dev':info.st_dev,'ino':info.st_ino,
                    'frozen_size':size,'complete_prefix_bytes':complete}
            except BaseException:
                os.close(fd);raise
        _require(len(snapshot.logs)==5,'frozen_five_log_boundary_missing')
        return snapshot
    except BaseException:
        snapshot.close();raise


class _JSONView:
    def __init__(self,path,raw):self.path=Path(path);self.raw=raw
    def is_file(self):return True
    def read_bytes(self):return self.raw
    def resolve(self):return self.path


class FrozenIO:
    """Worker IO with original source paths, raw line SHA and fixed prefix receipts."""
    def __init__(self,verifier,snapshot,*,load_prefixes=True):
        _require(not snapshot.closed,'frozen_snapshot_closed')
        self.verifier=verifier;self.snapshot=snapshot;self.rows={};self.receipts={}
        self.json_used=set();self.logs_used=set()
        if load_prefixes:self.load_prefixes()
    def load_prefixes(self):
        _require(not self.rows and not self.receipts,'frozen_prefix_already_loaded')
        for path,entry in self.snapshot.logs.items():
            self.rows[path],self.receipts[path]=self._prefix(path,entry)
    def _identity(self,path,entry):
        info=os.fstat(entry['fd']);current=os.stat(path,follow_symlinks=False)
        _require(stat.S_ISREG(current.st_mode)
            and (info.st_dev,info.st_ino)==(entry['dev'],entry['ino'])
            and (current.st_dev,current.st_ino)==(entry['dev'],entry['ino'])
            and info.st_size>=entry['frozen_size'] and current.st_size>=entry['frozen_size'],
            'frozen_log_replaced_or_shortened')
    def _prefix(self,path,entry):
        self._identity(path,entry);rows=[];offset=0;digest=hashlib.sha256()
        length=entry['complete_prefix_bytes']
        with os.fdopen(os.dup(entry['fd']),'rb')as stream:
            stream.seek(0)
            while offset<length:
                raw=stream.readline(min(self.verifier.MAX_LINE_BYTES+1,length-offset))
                _require(raw and len(raw)<=self.verifier.MAX_LINE_BYTES and raw.endswith(b'\n'),
                         'frozen_prefix_truncated_or_partial')
                data=json.loads(raw);self.verifier._canonical(data)
                rows.append(dict(data=data,source_file=path,source_offset=offset,
                    source_length=len(raw),source_line_sha256=_sha(raw)))
                offset+=len(raw);digest.update(raw)
                _require(len(rows)<=self.verifier.MAX_RECORDS,'frozen_prefix_record_budget')
        self._identity(path,entry)
        _require(offset==length,'frozen_prefix_length_changed')
        return rows,dict(path=path,complete_prefix_bytes=length,
                         complete_prefix_sha256=digest.hexdigest())
    def read_json(self,path,expected=None):
        path=str(Path(path).resolve())
        roles=[k for k,v in self.snapshot.json_sources.items()if v['path']==path]
        _require(len(roles)==1 and roles[0]not in self.json_used,'foreign_or_duplicate_frozen_JSON')
        role=roles[0];self.json_used.add(role)
        return self.verifier._read_json(_JSONView(path,self.snapshot.json_sources[role]['raw']),expected)
    def lines(self,run,filename):
        _require(Path(run).resolve()==self.snapshot.run,'foreign_frozen_run')
        path=str(self.verifier._inside(self.snapshot.run,filename))
        _require(path in self.rows and path not in self.logs_used,'foreign_or_duplicate_frozen_log')
        self.logs_used.add(path)
        return self.rows[path],copy.deepcopy(self.receipts[path])
    def identity_prefixes(self):
        return {role:copy.deepcopy(self.receipts[str(self.snapshot.run/name)])for role,name in
            [('bundles','known_scene_bundles.jsonl'),('controller_inputs','known_scene_controller_inputs.jsonl')]}
    def persist(self,*,boundary_capture=None):
        doc=dict(schema='r85_async_frozen_qualification_inputs/v1',run_dir=str(self.snapshot.run),
            captured_monotonic_wall=self.snapshot.captured_monotonic_wall,
            json_sources={role:dict(path=v['path'],sha256=_sha(v['raw']),bytes=len(v['raw']),
                raw_base64=base64.b64encode(v['raw']).decode('ascii'))for role,v in self.snapshot.json_sources.items()},
            log_sources=copy.deepcopy(self.receipts),
            frozen_log_sizes={p:v['frozen_size']for p,v in self.snapshot.logs.items()},
            frozen_complete_prefix_bytes={p:v['complete_prefix_bytes']for p,v in self.snapshot.logs.items()},
            boundary_capture=copy.deepcopy(boundary_capture),
            prefix_receipts_complete=len(self.receipts)==5,
            log_storage='Original owned run source prefixes retained in place; receipts bind exact bytes. No copied or reconstructed observations.',
            dynamic_PASS=False,navigation_PASS=False)
        raw=(json.dumps(doc,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
        _require(len(raw)<=MAX_CAPTURE_BYTES,'frozen_capture_budget')
        path=self.snapshot.run/('r85_frozen_qualification_inputs_'+uuid.uuid4().hex+'.json')
        fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o444)
        with os.fdopen(fd,'wb')as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
        _require(path.read_bytes()==raw,'frozen_capture_readback_changed')
        directory=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(directory)
        finally:os.close(directory)
        return dict(path=str(path),sha256=_sha(raw),bytes=len(raw))


def qualify_frozen(verifier,completed_scope,snapshot,request_id,profile,scope,handoff_source_ns):
    """Worker only; unchanged original verifier, then unchanged fixed-prefix witness."""
    started=time.monotonic()
    try:
        io=FrozenIO(verifier,snapshot,load_prefixes=False)
        boundary=io.persist()  # Preserve original raw state even if worker prefix validation fails.
        io.load_prefixes();capture=io.persist(boundary_capture=boundary)
        result=verifier.verify_result(snapshot.run,request_id,profile,scope,evidence_io=io)
        _require(result.get('passed')is True and result.get('state')=='passed'
            and result['verification'].get('allowed')is True,
            'original_physical_history_not_complete:'+str(result.get('failure_reason')))
        _require(io.json_used=={'contract','runtime_state'} and len(io.logs_used)==3,
                 'frozen_original_verifier_consumption_incomplete')
        state=snapshot.json_sources['runtime_state']['raw']
        _require(_sha(state)==result['source_receipt']['runtime_state']['sha256'],
                 'frozen_runtime_same_read_receipt_changed')
        stable=completed_scope.stable_selected_witness(verifier,snapshot.run,result,handoff_source_ns,
                                                       identity_prefixes=io.identity_prefixes())
        return dict(schema='r85_completed_history_retention_qualification/v1',
            qualified_for_history_retention_only=True,passed=False,dynamic_PASS=False,navigation_PASS=False,
            original_single_scope_dynamic_PASS=False,identity_contract_changed=True,
            original_physical_verifier_result=result,stable_witness=stable,
            original_runtime_state_raw_base64=base64.b64encode(state).decode('ascii'),
            actual_monotonic_duration_s=time.monotonic()-started,
            acceptance_deferred_until_endpoint_parking_map=True,frozen_input_capture=capture)
    finally:snapshot.close()
