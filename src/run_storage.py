"""Create owned run directories inside the current project."""
import json,os,re
from pathlib import Path

def capacity(path):
    s=os.statvfs(path)
    return dict(path=str(path),available_bytes=s.f_bavail*s.f_frsize,total_bytes=s.f_blocks*s.f_frsize)
def create_run(project_root,name,requested=None):
    project=Path(project_root).resolve()
    if requested is not None:raise ValueError('Runs must stay inside the current project')
    if not re.fullmatch(r'[A-Za-z0-9_-]+',name):raise ValueError('Invalid unique run name')
    run=project/'runs'/name
    run.mkdir(mode=0o700)
    receipt=dict(simulation_only=True,external_enabled=False,canonical_run_dir=str(run),
        project_run_alias=str(run),storage_root=str(run.parent),owner_uid=os.getuid(),
        capacity_before=dict(storage=capacity(run.parent),project=capacity(project)),
        historical_runs_relocated=False,no_old_run_overwrite=True)
    (run/'run_storage_contract.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return run,receipt
def verify_run(run,project_root):
    run=Path(run);project=Path(project_root).resolve()
    if (run.is_symlink() or not run.is_dir() or run.resolve().parent!=project/'runs'
        or run.stat().st_uid!=os.getuid()):raise ValueError('Run directory must belong to this project/user')
    return True
