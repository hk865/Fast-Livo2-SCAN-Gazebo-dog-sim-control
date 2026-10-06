"""Explicit owned external run storage; immutable canonical paths from creation."""
import json, os, re, stat
from pathlib import Path
EXTERNAL_ROOT=Path(os.environ.get('TEACHER_RUN_STORAGE_ROOT','/var/tmp/go2_teacher_portable_'+str(os.getuid())))
SCHEMA='teacher_owned_run_storage/v1'
def capacity(path):
    s=os.statvfs(path)
    return {'path':str(path),'available_bytes':s.f_bavail*s.f_frsize,'total_bytes':s.f_blocks*s.f_frsize}
def owned_root(path,create=False):
    path=Path(path)
    if path!=EXTERNAL_ROOT:raise RuntimeError('External storage must be the explicitly owned fixed root')
    if create:path.mkdir(mode=0o700,parents=False,exist_ok=True)
    info=path.lstat()
    if not stat.S_ISDIR(info.st_mode) or path.is_symlink() or info.st_uid!=os.getuid():
        raise RuntimeError('External run root must be an owned directory, never a symlink')
    if create:path.chmod(0o700)
    if stat.S_IMODE(path.stat().st_mode)!=0o700:raise RuntimeError('External run root must have mode0700')
    return path

def create_run(project_root,name,requested=None):
    project_root=Path(project_root).resolve();(project_root/'runs').mkdir(parents=True,exist_ok=True);alias=project_root/'runs'/name
    if not re.fullmatch(r'[A-Za-z0-9_-]+',name):raise RuntimeError('Invalid unique run name')
    if os.path.lexists(alias):raise RuntimeError('Refuse existing project run or alias')
    external=requested is not None
    storage=owned_root(Path(requested),create=True) if external else alias.parent
    run=storage/name
    run.mkdir(mode=0o700)
    if external:alias.symlink_to(run,target_is_directory=True)
    contract={'schema':SCHEMA,'simulation_only':True,'external_enabled':external,
        'canonical_run_dir':str(run.resolve()),'project_run_alias':str(alias),
        'storage_root':str(storage.resolve()),'owner_uid':os.getuid(),
        'root_is_symlink':False,'external_root_mode': '0700' if external else None,
        'capacity_before':{'storage':capacity(storage),'project':capacity(alias.parent)},
        'historical_runs_relocated':False,'all_runtime_arrays_inside_canonical_run':True,
        'no_old_run_overwrite':True,'claim':'Location and launch provenance only; data completeness separately evaluated'}
    (run/'run_storage_contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    return run.resolve(),contract

def verify_run(run,project_root):
    run=Path(run).resolve();project=Path(project_root).resolve()
    if run.parent==(project/'runs').resolve():
        if (run/'run_storage_contract.json').is_file():
            d=json.loads((run/'run_storage_contract.json').read_text())
            if d.get('schema')!=SCHEMA or d.get('external_enabled') is not False or d.get('canonical_run_dir')!=str(run):
                raise RuntimeError('Wrong default run storage contract')
        return True
    owned_root(run.parent)
    if not run.is_dir() or run.is_symlink() or run.stat().st_uid!=os.getuid():raise RuntimeError('Invalid owned canonical run')
    d=json.loads((run/'run_storage_contract.json').read_text());alias=project/'runs'/run.name
    if (d.get('schema')!=SCHEMA or d.get('simulation_only')is not True or d.get('external_enabled') is not True
        or d.get('canonical_run_dir')!=str(run) or d.get('project_run_alias')!=str(alias)
        or d.get('storage_root')!=str(EXTERNAL_ROOT) or d.get('owner_uid')!=os.getuid()
        or not alias.is_symlink() or alias.resolve()!=run):raise RuntimeError('External canonical/alias contract mismatch')
    return True
