"""Current retained metadata census; no symlink traversal or historical floor."""
from pathlib import Path
import os,stat,time,json
B=Path('/opt/experiment/history/2026-10-07/task/go2_engineering_v33')
TM=Path('/opt/experiment/workspace/multifloor_demo/teacher_mode')
EXTERNAL=Path('/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs')

def declared_roots():
    d=json.loads((B/'STORAGE_SESSION.json').read_text())
    roots=[Path(__file__).resolve().parent.parent,B,TM,EXTERNAL,*map(Path,d.get('historical_registered_roots',d['roots'])),
           *map(Path,d.get('retained_roots',[]))]
    for parent in ('/tmp','/var/tmp'):
        with os.scandir(parent)as entries:
            roots.extend(Path(e.path)for e in entries if 'go2'in e.name.lower())
    return roots

def usage(paths,*,allowed_missing=()):
    started=time.monotonic();roots=[];missing=[]
    for p in sorted(set(Path(os.path.abspath(p))for p in paths),key=lambda p:(len(p.parts),str(p))):
        if any(p==r or p.is_relative_to(r)for r in roots):continue
        try:p.lstat()
        except FileNotFoundError:missing.append(str(p));continue
        roots.append(p)
    seen=set();logical=allocated=files=directories=links=duplicates=0;errors=[];vanished=[]
    pending=[(str(p),None)for p in roots]
    while pending:
        path,entry=pending.pop()
        try:st=os.lstat(path)if entry is None else entry.stat(follow_symlinks=False)
        except FileNotFoundError:vanished.append(path);continue
        except OSError as e:errors.append(dict(path=path,error=type(e).__name__+': '+str(e)));continue
        identity=st.st_dev,st.st_ino
        if identity in seen:duplicates+=1;continue
        seen.add(identity);allocated+=st.st_blocks*512
        if stat.S_ISDIR(st.st_mode):
            directories+=1
            try:
                with os.scandir(path)as entries:pending.extend((e.path,e)for e in entries)
            except OSError as e:errors.append(dict(path=path,error=type(e).__name__+': '+str(e)))
        elif stat.S_ISREG(st.st_mode):files+=1;logical+=st.st_size
        elif stat.S_ISLNK(st.st_mode):links+=1;logical+=st.st_size
    return dict(logical_bytes=logical,allocated_bytes=allocated,current_retained_budget_bytes=max(logical,allocated),
        files=files,directories=directories,symlinks=links,objects=len(seen),duplicate_inode_entries=duplicates,
        roots=[str(p)for p in roots],missing_roots=missing,scan_errors=errors,transient_vanished_paths=vanished,
        elapsed_s=time.monotonic()-started,non_atomic_live_census=True,symlinks_followed=False,
        historical_counter_used_as_capacity_floor=False)

def current():return usage(declared_roots())
