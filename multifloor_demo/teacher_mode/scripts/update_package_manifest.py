#!/usr/bin/env python3
"""Inventory the completed simulation package without running any controller.

Default prints a prepare-only description. --write is intended AFTER all runs
and reports finish. It archives the prior manifest's exact bytes, hashes current
sources/docs/evidence, and copies scoped levels from current_status.json only.
No ROS, Gazebo, robot, training, process-signal or acceptance-writing API is used.
"""
from __future__ import annotations
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile

ROOT=Path(__file__).resolve().parents[1]
FROZEN_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
SKIP_DIRS={'__pycache__','build','.git','.pytest_cache','.mypy_cache','.ruff_cache'}
HISTORY=Path('test_results/package_manifest_history')
COMPILED=('simulation/build/libteacher_actuator.so','navigation/dynamic/build/obstacle_pose_observer',
          'pid_comparison/champ_mode/native/build/libchamp_native_observer.so')


def fingerprint(path):
    s=path.stat(follow_symlinks=False)
    return (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns)


def digest(path):
    """Hash one regular immutable file; fail if a producer changes/replaces it."""
    path=Path(path);before=fingerprint(path)
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    try:
        st=os.fstat(fd)
        if not stat.S_ISREG(st.st_mode):raise RuntimeError('Not a regular evidence file: '+str(path))
        if (st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns)!=before:raise RuntimeError('File changed before hashing: '+str(path))
        h=hashlib.sha256()
        with os.fdopen(fd,'rb',closefd=False)as stream:
            for block in iter(lambda:stream.read(8*1024*1024),b''):h.update(block)
        after=os.fstat(fd)
        if (after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns)!=before or fingerprint(path)!=before:
            raise RuntimeError('Evidence still changing; finish writers before package update: '+str(path))
        return h.hexdigest(),st.st_size,before
    finally:os.close(fd)


def category(relative):
    first=relative.parts[0]
    if first in ('docs','runs','test_results','plots'):return first
    if '/build/'in str(relative):return 'compiled_artifacts'
    return 'current_sources_and_configuration'


def inventory(root):
    entries=[];links=[];fingerprints={};excluded=[]
    for directory,dirs,names in os.walk(root,followlinks=False):
        base=Path(directory)
        fingerprints[base]=fingerprint(base)
        kept=[]
        for name in sorted(dirs):
            path=base/name;rel=path.relative_to(root)
            if path.is_symlink():
                links.append({'path':str(rel),'target':os.readlink(path),'followed':False});continue
            if name in SKIP_DIRS or rel==HISTORY:
                excluded.append({'path':str(rel),'reason':'temporary build/cache or self-manifest history'});continue
            kept.append(name)
        dirs[:]=kept
        for name in sorted(names):
            path=base/name;rel=path.relative_to(root)
            if path.is_symlink():
                links.append({'path':str(rel),'target':os.readlink(path),'followed':False});continue
            if rel==Path('PACKAGE_MANIFEST.json')or name.endswith(('.tmp','.swp','.lock','~')):
                excluded.append({'path':str(rel),'reason':'current output or transient file'});continue
            if not path.is_file():
                excluded.append({'path':str(rel),'reason':'non-regular endpoint'});continue
            h,size,fp=digest(path);fingerprints[path]=fp
            entries.append({'path':str(rel),'size_bytes':size,'sha256':h,'category':category(rel)})
    # Build machinery is excluded, but the exact executed actuator and passive
    # pose-observer binaries are essential package inputs, not temporary files.
    for name in COMPILED:
        path=root/name
        if path.is_file():
            h,size,fp=digest(path);fingerprints[path]=fp
            entries.append({'path':name,'size_bytes':size,'sha256':h,'category':'compiled_artifacts'})
    return sorted(entries,key=lambda x:x['path']),links,excluded,fingerprints


def json_file(path):
    return json.loads(Path(path).read_text())


def recorded_runtime_identity(run,cache=None):
    """Bind a named preparation's actual logs to its own immutable run inputs.

    This is an inventory identity check, not an outcome gate: nonzero/unfinished
    process exits and failed physics receipts can still describe an actual run.
    Original paths in metadata are provenance, never replacements for archived
    paths. An offline copy retaining another run's paths cannot be upgraded.
    """
    run=Path(run).resolve();cache={} if cache is None else cache
    basis={'method':'self_bound_actual_native_runtime_v1','verified':False,
           'outcome_pass_inferred':False,'checks':{},'errors':[]}
    def require(name,ok,detail=None):
        basis['checks'][name]={'passed':bool(ok),'detail':detail}
        if not ok:raise ValueError(name)
    def hash_file(path):
        path=Path(path);fp=fingerprint(path)
        key=(str(path),fp)
        if key not in cache:cache[key]=digest(path)[0]
        return cache[key]
    def local_regular(path):
        path=Path(path)
        return path.is_absolute() and not path.is_symlink() and path.resolve().is_relative_to(run) and path.is_file()
    try:
        native=run/'actuator.jsonl';teacher=run/'telemetry.jsonl';runtime_path=run/'runtime_manifest.json'
        require('actual_nonempty_native_and_teacher_logs',all(local_regular(p) and p.stat().st_size>0 for p in (native,teacher)))
        require('self_runtime_manifest',local_regular(runtime_path))
        runtime=json_file(runtime_path)
        require('runtime_manifest_mapping',isinstance(runtime,dict))
        if 'run' in runtime:require('declared_run_identity',Path(runtime['run']).resolve()==run)
        owned=runtime.get('owned_processes',[])
        require('owned_worker_and_gazebo_graph',isinstance(owned,list) and
            all(isinstance(p,dict) for p in owned) and
            len([p for p in owned if p.get('role')=='worker'])==1 and
            len([p for p in owned if p.get('role')=='gazebo'])==1 and
            all(type(p.get('pid')) is int and p['pid']>0 for p in owned if p.get('role') in ('worker','gazebo')) and
            len({p.get('pid') for p in owned if p.get('role') in ('worker','gazebo')})==2,
            [{'role':p.get('role'),'pid':p.get('pid'),'returncode':p.get('returncode')} for p in owned if isinstance(p,dict)] if isinstance(owned,list) else None)
        require('exclusive_runtime_writer',runtime.get('exclusive_writer')=='teacher_sim::TeacherActuator',runtime.get('exclusive_writer'))
        # Native contract and plugin graph occur before the first physics step.
        # Read only the actual prefix, avoiding a second full 200Hz-log parse.
        contracts=[];owners=[]
        with native.open() as stream:
            for line in stream:
                row=json.loads(line)
                if not isinstance(row,dict):raise ValueError('Native prefix is not an object')
                if row.get('kind')=='actuator_contract':contracts.append(row)
                elif row.get('kind')=='model_plugin_ownership':owners.append(row)
                elif row.get('kind')=='physics_step':break
        plugins=owners[0].get('plugins',[]) if len(owners)==1 else []
        require('actual_plugin_graph_schema',isinstance(plugins,list) and all(isinstance(p,dict) for p in plugins))
        teacher_plugins=[p for p in plugins if 'teacheractuator' in (p.get('name','')+' '+p.get('filename','')).lower() or 'teacher_actuator' in (p.get('name','')+' '+p.get('filename','')).lower()]
        require('unique_actual_native_contract_and_plugin_graph',len(contracts)==1 and len(owners)==1 and
            contracts[0].get('writer')=='teacher_sim::TeacherActuator sole JointForceCmd writer' and
            owners[0].get('passed') is True and owners[0].get('teacher_writers')==1 and len(teacher_plugins)==1)
        manifest_path=run/'source_manifest.json'
        require('source_manifest_self_hash',local_regular(manifest_path) and runtime.get('source_manifest_sha256')==hash_file(manifest_path),
                runtime.get('source_manifest_sha256'))
        manifest=json_file(manifest_path);require('nonempty_run_source_manifest',isinstance(manifest,dict) and bool(manifest))
        snapshots=[];bad=[]
        for key,item in manifest.items():
            if isinstance(item,str):path=run/'sources'/key;expected=item
            elif isinstance(item,dict):
                path=Path(item.get('path',''));expected=item.get('sha256')
                if not path.is_absolute():path=run/path
            else:bad.append(str(key)+':schema');continue
            if not local_regular(path):bad.append(str(key)+':outside_self_run_or_not_regular');continue
            generated=isinstance(item,dict) and item.get('generated_input') is True
            if not path.resolve().is_relative_to(run/'sources') and not generated:
                bad.append(str(key)+':not_archived_source_or_declared_input');continue
            if not isinstance(expected,str) or len(expected)!=64 or hash_file(path)!=expected:
                bad.append(str(key)+':SHA');continue
            snapshots.append((path,expected))
        require('every_source_archive_binds_this_run_and_hash',not bad,{'verified_entries':len(snapshots),'errors':bad})
        world=run/'world.sdf'
        require('actual_world_bound_to_source_manifest',any(path.resolve()==world and expected==hash_file(world) for path,expected in snapshots))
        native_sha=runtime.get('native_plugin_sha256')
        require('native_binary_hash_matches_executed_archive',isinstance(native_sha,str) and len(native_sha)==64 and
            any(path.name.endswith('libteacher_actuator.so') and expected==native_sha for path,expected in snapshots),native_sha)
        policy_path=run/'policy_manifest.json';require('self_policy_manifest',local_regular(policy_path))
        policy=json_file(policy_path)
        require('policy_manifest_mapping',isinstance(policy,dict))
        model=runtime.get('model_sha256',runtime.get('frozen_model_sha256',runtime.get('checkpoint_sha256')))
        require('runtime_and_policy_frozen_model_identity',model==FROZEN_SHA and policy.get('checkpoint_sha256')==FROZEN_SHA,model)
        checkpoint=Path(policy.get('checkpoint',''))
        require('frozen_checkpoint_bytes_match_model_identity',checkpoint.is_absolute() and checkpoint.is_file() and
            not checkpoint.is_symlink() and hash_file(checkpoint)==FROZEN_SHA,str(checkpoint))
        basis['verified']=True
    except (ValueError,TypeError,KeyError,OSError,RuntimeError) as exc:
        basis['errors'].append(str(exc))
    return basis


def run_index(root,entries):
    by_path={item['path']:item for item in entries};rows=[];identity_cache={}
    for run in sorted((root/'runs').iterdir()):
        if not run.is_dir()or run.is_symlink():continue
        # Names never count as actual trials. A prepared world/scope alone is
        # explicitly excluded even if its schema grants permission to test.
        preparation=any(word in run.name.lower()for word in ('offline','freeze','prepare'))
        native=run/'actuator.jsonl';teacher=run/'telemetry.jsonl';runtime=run/'runtime_manifest.json'
        physical=(native.is_file()and native.stat().st_size>0 and teacher.is_file()and teacher.stat().st_size>0)
        classification=('configuration_or_offline_preparation'if preparation else
                        'actual_gazebo_runtime_recorded'if physical and runtime.is_file()else
                        'actual_gazebo_partial_evidence'if physical else'non_runtime_reference_or_preparation')
        basis={'method':'legacy_non_preparation_log_presence' if not preparation else 'name_without_self_bound_runtime',
               'preparation_word_in_name':preparation,'outcome_pass_inferred':False}
        if preparation and physical and runtime.is_file():
            basis=recorded_runtime_identity(run,identity_cache)
            basis['preparation_word_in_name']=True
            if basis['verified']:classification='actual_gazebo_runtime_recorded'
        summary=[]
        for path in sorted(run.glob('summary*.json')):
            entry=by_path.get(str(path.relative_to(root)))
            if entry is None:continue
            data=json_file(path)
            summary.append({'path':entry['path'],'sha256':entry['sha256'],'status':data.get('status'),
                'levels':data.get('levels'),'meaning':'Original scoped receipt, never promoted to overall package pass'})
        rows.append({'run_id':run.name,'classification':classification,
            'classification_basis':basis,
            'counted_as_actual_runtime':classification=='actual_gazebo_runtime_recorded',
            'counted_as_complete_pass':False,'runtime_manifest_present':runtime.is_file(),
            'actual_physics_and_teacher_logs_present':physical,'summary_receipts':summary})
    return rows


def verify_protected(old):
    result={};fingerprints={}
    for name,evidence in old.get('protected_assets',{}).items():
        expected=evidence.get('expected_sha256')
        if not isinstance(expected,str)or len(expected)!=64:raise RuntimeError('Original protected asset lacks baseline hash: '+name)
        actual,size,fp=digest(name);fingerprints[Path(name)]=fp
        result[name]={'expected_sha256':expected,'actual_sha256':actual,'size_bytes':size,'unchanged':actual==expected}
        if actual!=expected:raise RuntimeError('Protected original asset changed: '+name)
    if not result:raise RuntimeError('Prior package manifest must preserve original asset baselines')
    return result,fingerprints


def atomic_write(path,content):
    tmp=None
    try:
        with tempfile.NamedTemporaryFile('wb',dir=path.parent,prefix='.'+path.name+'.',suffix='.tmp',delete=False)as stream:
            tmp=Path(stream.name);stream.write(content);stream.flush();os.fsync(stream.fileno())
        tmp.replace(path)
        # A one-time archival manifest can request directory durability. This
        # is separate from the latency-sensitive transient command transport.
        fd=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
    finally:
        if tmp is not None:tmp.unlink(missing_ok=True)


def build_and_write(root,status_path):
    target=root/'PACKAGE_MANIFEST.json';old_raw=target.read_bytes();old=json.loads(old_raw)
    old_sha=hashlib.sha256(old_raw).hexdigest();verified_old_sha,_,initial_target=digest(target)
    if verified_old_sha!=old_sha:raise RuntimeError('Prior manifest changed during read')
    status_raw=status_path.read_bytes();status=json.loads(status_raw)
    if status.get('finalized')is not True:raise RuntimeError('current_status.json is not finalized; complete all evidence and reports before formal package write')
    scopes=status.get('levels_by_scope',status.get('scopes'))
    if not isinstance(scopes,dict)or not scopes:raise RuntimeError('current_status.json requires explicit nonempty levels_by_scope or scopes; do not infer them from historical acceptance')
    status_sha,status_size,status_fp=digest(status_path)
    if status_sha!=hashlib.sha256(status_raw).hexdigest():raise RuntimeError('Current scoped status changed during read')
    checkpoint=Path(old['checkpoint']);cp_sha,cp_size,cp_fp=digest(checkpoint)
    if cp_sha!=FROZEN_SHA or old.get('checkpoint_sha256')!=FROZEN_SHA:raise RuntimeError('Frozen checkpoint identity differs')
    acceptance=root/'runs/acceptance.json';accept_sha,accept_size,accept_fp=digest(acceptance)
    original_acceptance_sha=old.get('original_acceptance_sha256',old.get('acceptance_sha256'))
    if accept_sha!=original_acceptance_sha:raise RuntimeError('Historical overall acceptance changed; this inventory never rewrites or upgrades it')
    protected,protected_fp=verify_protected(old)
    archive_hashes={};archive_fp={}
    for name,expected in old.get('archive_hashes',{}).items():
        actual,size,fp=digest(name);archive_fp[Path(name)]=fp
        if actual!=expected:raise RuntimeError('Frozen training archive changed: '+name)
        archive_hashes[name]=actual
    entries,links,excluded,fps=inventory(root)
    fps.update(protected_fp);fps.update(archive_fp);fps.update({target:initial_target,status_path:status_fp,
        checkpoint:cp_fp,acceptance:accept_fp})
    rows=run_index(root,entries)
    now=datetime.datetime.now(datetime.timezone.utc);stamp=now.strftime('%Y%m%dT%H%M%SZ')
    previous_relative=HISTORY/('PACKAGE_MANIFEST_'+stamp+'_'+old_sha[:16]+'.json')
    groups={}
    for item in entries:
        group=groups.setdefault(item['category'],{'files':0,'size_bytes':0});group['files']+=1;group['size_bytes']+=item['size_bytes']
    source_pairs=[[x['path'],x['sha256']]for x in entries if x['category']in ('current_sources_and_configuration','docs','compiled_artifacts')]
    # Preserve all prior fields and their exact bytes in history. Legacy fields
    # are retained in legacy_metadata with their original timestamp/context;
    # scoped current conclusions come exclusively from the status source.
    result={'schema_version':2,'timestamp':now.isoformat(),'scope':'Simulation-only frozen Teacher package inventory; actual outcomes remain independently scoped',
        'checkpoint':str(checkpoint),'checkpoint_sha256':cp_sha,'checkpoint_size_bytes':cp_size,
        'archive_hashes':archive_hashes,'protected_assets':protected,
        'acceptance_sha256':accept_sha,'original_acceptance_sha256':original_acceptance_sha,
        'historical_acceptance_unchanged':True,'historical_acceptance_levels':json_file(acceptance).get('levels'),
        'current_status':{'path':str(status_path.relative_to(root))if status_path.is_relative_to(root)else str(status_path),
            'sha256':status_sha,'size_bytes':status_size,'content':status},
        'levels_by_scope':scopes,'source_files':source_pairs,'file_inventory':entries,
        'inventory_totals':groups,'symlinks_not_followed':links,'excluded_paths':excluded,'run_inventory':rows,
        'actual_runtime_runs':sum(x['counted_as_actual_runtime']for x in rows),
        'partial_evidence_runs':sum(x['classification']=='actual_gazebo_partial_evidence'for x in rows),
        'configuration_or_offline_preparations':sum(x['classification']=='configuration_or_offline_preparation'for x in rows),
        'prior_package_manifest':str(previous_relative),'prior_package_manifest_sha256':old_sha,'prior_package_manifest_size_bytes':len(old_raw),
        'legacy_metadata':{k:v for k,v in old.items()if k not in ('file_inventory','run_inventory','legacy_metadata','source_files')},
        'legacy_source_files':old.get('source_files',[]),
        'preserved_processes_historical_snapshot':old.get('preserved_processes',old.get('preserved_processes_historical_snapshot')),
        'preserved_processes_snapshot_timestamp':old.get('timestamp'),
        'process_scope':'Historical process receipts retained; this generator neither probes nor controls processes, and does not claim a prior alive flag is current',
        'manifest_self_inclusion':'Current root manifest and manifest-history directory excluded; historical per-run source snapshots remain inventoried',
        'source_snapshots_note':'Current source inventory never replaces per-run executed snapshots, evaluator provenance corrections, failures or independent scoped receipts',
        'meaning':'Inventory only; no automatic pass, no navigation permission, no acceptance update, no actual robot deployment claim'}
    by_path={x['path']:x for x in entries}
    if COMPILED[0]in by_path:result['plugin_sha256']=by_path[COMPILED[0]]['sha256']
    result['historical_plugin_sha256']=old.get('historical_plugin_sha256',old.get('plugin_sha256'))
    for path,fp in fps.items():
        if fingerprint(path)!=fp:raise RuntimeError('Evidence changed during inventory; package was not written: '+str(path))
    previous=root/previous_relative;previous.parent.mkdir(parents=True,exist_ok=True)
    if previous.exists():
        if previous.read_bytes()!=old_raw:raise RuntimeError('Historical manifest archive collision')
    else:
        with previous.open('xb')as stream:stream.write(old_raw);stream.flush();os.fsync(stream.fileno())
    atomic_write(target,(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n').encode())
    return {'schema_version':2,'status':'inventory_written','manifest':str(target),'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
        'files':len(entries),'previous_manifest':str(previous),'acceptance_unchanged':True,'levels_by_scope':scopes}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--write',action='store_true')
    p.add_argument('--current-status',type=Path,default=ROOT/'current_status.json');args=p.parse_args()
    if not args.write:
        print(json.dumps({'status':'prepared_only','writes_manifest':False,'starts_ros':False,'starts_simulation':False,
            'usage':'Run --write after all actual runs, reports and current_status.json are complete',
            'current_status':str(args.current_status),'output':str(ROOT/'PACKAGE_MANIFEST.json')}));return
    status_path=args.current_status.resolve()
    with (ROOT/'.PACKAGE_MANIFEST.lock').open('a')as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        result=build_and_write(ROOT,status_path)
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
