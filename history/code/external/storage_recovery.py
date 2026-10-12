"""One explicitly reviewed recovery epoch; historical storage is never refunded.

The authoritative journal is append-only, fsynced and serialized with flock.
Unknown missing roots, another boot, changed baseline or missing history fail.
"""
from pathlib import Path
import fcntl
import hashlib
import json
import os
import time


def canonical(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'))+'\n').encode()


def stable_session(session):
    return hashlib.sha256(canonical({k:v for k,v in session.items() if k!='roots'})).hexdigest()


def advance(previous, known, filesystems, original_charge):
    """Charge positive observed increments per channel; never cancel frees."""
    current={str(f['device']):int(f['available_bytes']) for f in filesystems}
    if set(current)!=set(previous['available_bytes']):
        raise RuntimeError('Recovery filesystem identity changed')
    logical=previous['logical_increments']+max(0,known['logical_bytes']-previous['known_logical'])
    allocated=previous['allocated_increments']+max(0,known['allocated_bytes']-previous['known_allocated'])
    filesystem=previous['filesystem_increments']+sum(max(0,previous['available_bytes'][dev]-v) for dev,v in current.items())
    charged=max(previous['charged_bytes'],previous['historical_floor_bytes']+max(logical,allocated,filesystem),original_charge)
    return dict(recovery_id=previous['recovery_id'],sequence=previous['sequence']+1,
        historical_floor_bytes=previous['historical_floor_bytes'],charged_bytes=charged,
        known_logical=int(known['logical_bytes']),known_allocated=int(known['allocated_bytes']),
        available_bytes=current,logical_increments=logical,allocated_increments=allocated,
        filesystem_increments=filesystem)


def verify_contract(contract, session, boot_id):
    if contract['schema']!='R39_storage_reboot_recovery/v1':
        raise RuntimeError('Unknown storage recovery contract')
    if contract['boot_id']!=boot_id:
        raise RuntimeError('Recovery belongs to another boot')
    if stable_session(session)!=contract['original_session_stable_sha256']:
        raise RuntimeError('Original storage baseline or limits changed')
    if not set(contract['original_registered_roots']).issubset(session['roots']):
        raise RuntimeError('Historical registered output root removed')
    allowed=contract['verified_missing_roots']
    if not allowed or not set(allowed).issubset(contract['original_registered_roots']):
        raise RuntimeError('Unregistered missing-root exemption')
    if contract['historical_floor_bytes']<47_772_774_400:
        raise RuntimeError('Historical cumulative charge was refunded')
    for row in contract['historical_evidence']:
        if hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()!=row['sha256']:
            raise RuntimeError('Recovery historical evidence changed: '+row['path'])
    return tuple(allowed)


def read_journal(contract):
    p=Path(contract['journal_path']);st=p.stat()
    if [st.st_dev,st.st_ino]!=contract['journal_identity']:
        raise RuntimeError('Authoritative recovery journal replaced')
    raw=p.read_bytes()
    if not raw or len(raw)>contract['journal_hard_cap_bytes'] or not raw.endswith(b'\n'):
        raise RuntimeError('Recovery journal absent, oversized or partial')
    lines=raw.splitlines(keepends=True)
    if hashlib.sha256(lines[0]).hexdigest()!=contract['initial_journal_record_sha256']:
        raise RuntimeError('Recovery historical anchor changed')
    previous=None;digest=None
    for line in lines:
        row=json.loads(line)
        state=row['state']
        if row['previous_sha256']!=digest or state['recovery_id']!=contract['recovery_id']:
            raise RuntimeError('Recovery journal chain changed')
        if previous is not None:
            if state['sequence']!=previous['sequence']+1:
                raise RuntimeError('Recovery sequence lost')
            for key in ('charged_bytes','logical_increments','allocated_increments','filesystem_increments'):
                if state[key]<previous[key]:raise RuntimeError('Recovery cumulative charge regressed')
        if state['historical_floor_bytes']!=contract['historical_floor_bytes'] or state['charged_bytes']<contract['historical_floor_bytes']:
            raise RuntimeError('Historical storage charge changed')
        previous=state;digest=hashlib.sha256(line).hexdigest()
    seal=json.loads(Path(contract['checkpoint_path']).read_text())
    if seal!=dict(sequence=previous['sequence'],tail_sha256=digest,charged_bytes=previous['charged_bytes']):
        raise RuntimeError('Recovery journal truncated or checkpoint inconsistent')
    return previous,digest,len(raw)


def observe_recovery(here, session, usage, capacity, conservative_growth):
    contract_path=Path(here)/'STORAGE_REBOOT_RECOVERY_CONTRACT.json'
    if not contract_path.exists():
        raise RuntimeError('Required R39 recovery contract lost')
    contract=json.loads(contract_path.read_text())
    lock_path=Path(contract['lock_path'])
    # Existing immutable lock inode; never manufacture a missing journal/lock.
    with lock_path.open('r+b') as lock:
        st=os.fstat(lock.fileno())
        if [st.st_dev,st.st_ino]!=contract['lock_identity']:
            raise RuntimeError('Recovery accounting lock replaced')
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX)
        current=lock_path.stat()
        if [current.st_dev,current.st_ino]!=contract['lock_identity']:
            raise RuntimeError('Recovery lock path changed while acquiring lock')
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        allowed=verify_contract(contract,session,boot_id)
        previous,digest,size=read_journal(contract)
        known=usage(session['roots'],allowed_missing=allowed)
        filesystems=[capacity('/home'),capacity('/tmp')]
        initial={x['device']:x for x in session['initial_filesystems']}
        if set(initial)!={x['device'] for x in filesystems}:
            raise RuntimeError('Original filesystem identity changed')
        original=conservative_growth(known,session['initial_usage'],sum(max(0,initial[x['device']]['available_bytes']-x['available_bytes']) for x in filesystems))
        state=advance(previous,known,filesystems,original)
        row=canonical(dict(previous_sha256=digest,state=state))
        if size+len(row)>contract['journal_hard_cap_bytes']:
            raise RuntimeError('Recovery journal cap exceeded')
        with Path(contract['journal_path']).open('ab',buffering=0) as journal:
            if journal.write(row)!=len(row):raise RuntimeError('Partial recovery accounting write')
            os.fsync(journal.fileno())
        checkpoint=Path(contract['checkpoint_path'])
        temporary=checkpoint.with_name(checkpoint.name+'.'+str(os.getpid())+'.tmp')
        seal=canonical(dict(sequence=state['sequence'],tail_sha256=hashlib.sha256(row).hexdigest(),charged_bytes=state['charged_bytes']))
        with temporary.open('xb',buffering=0) as stream:
            if stream.write(seal)!=len(seal):raise RuntimeError('Partial accounting checkpoint')
            os.fsync(stream.fileno())
        os.replace(temporary,checkpoint)
        directory=os.open(str(checkpoint.parent),os.O_DIRECTORY)
        try:os.fsync(directory)
        finally:os.close(directory)
        return known,filesystems,dict(contract_path=str(contract_path),
            historical_floor_bytes=state['historical_floor_bytes'],charged_bytes=state['charged_bytes'],
            sequence=state['sequence'],missing_roots=[p for p in allowed if not Path(p).is_dir()],
            original_history_verified_restored=False,source='append-only fsynced recovery accounting',
            new_logical_increments=state['logical_increments'],new_allocated_increments=state['allocated_increments'],
            new_filesystem_increments=state['filesystem_increments'])
