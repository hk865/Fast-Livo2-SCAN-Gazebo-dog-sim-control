#!/usr/bin/env python3
"""Explicitly scoped raw-data purge. Default writes a reviewable plan, never deletes.

User authorization: 2026-10-06: 原始的数据就删除吧，太占地方了.
Only this task's physical-run recordings are eligible. Models, source snapshots,
reports, config, training, camera_mode and finite numerical fixtures are excluded.
An immutable prior inventory SHA is provenance, not a fresh raw-file rehash.
"""
import argparse,datetime,gzip,hashlib,json,os,stat,subprocess,time
from pathlib import Path
WORKSPACE=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_')
TEACHER=WORKSPACE/'multifloor_demo/teacher_mode'
ROOTS=[TEACHER/'runs',Path('/var/tmp/go2_teacher_simulation_20261005'),Path('/var/tmp/go2_teacher_parallel_20261005')]
REPO=Path(__file__).resolve().parents[1]
REMOTE='git@github.com:hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control.git'
OUT=REPO/'maintenance/private_execution'
AUTH='原始的数据就删除吧，太占地方了'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb')as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def sig(p):
 s=p.lstat();return [s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns,s.st_mode,s.st_nlink,s.st_blocks]
def candidate(rel):
 if len(rel.parts)<2 or 'sources' in rel.parts:return False
 if any(x in rel.parts for x in ['navigation_cloud_arrays','frames','vehicle_rgb']):return True
 n=rel.name
 return n.endswith('.jsonl') or n=='observations_actions.npz' or (n=='records.bin' and 'fastlivo_diagnostics' in rel.parts) or (n.startswith('colored_map.') and n.endswith('.bin'))
def inventory():
 manifests=[TEACHER/'PACKAGE_MANIFEST.json',TEACHER/'test_results/lidar_density_rate_20261005/EXTERNAL_STORAGE_MANIFEST.json',TEACHER/'test_results/parallel_pipeline_20261005/EXTERNAL_STORAGE_MANIFEST.json']
 known={};proof=[]
 for i,p in enumerate(manifests):
  d=json.loads(p.read_text());proof.append({'path':str(p),'sha256':sha(p)})
  entries=d['file_inventory'] if i==0 else d['files']
  base=TEACHER if i==0 else ROOTS[i]
  for e in entries:known[str(base/e['path'])]={'sha256':e['sha256'],'size_bytes':e['size_bytes'],'inventory':i}
 return known,proof
def plan():
 OUT.mkdir(parents=True,exist_ok=True);target=OUT/'PURGE_PLAN.json'
 if target.exists():raise RuntimeError('Plan exists; never replace a reviewed plan')
 known,proof=inventory();items=[];unknown=[]
 for root in ROOTS:
  assert root.is_dir() and not root.is_symlink()
  for base,ds,ns in os.walk(root,followlinks=False):
   ds[:]=[n for n in ds if n!='sources' and not Path(base,n).is_symlink()]
   for n in ns:
    p=Path(base,n);rel=p.relative_to(root)
    if not candidate(rel) or p.is_symlink():continue
    s=sig(p)
    if not stat.S_ISREG(s[5]):continue
    entry=known.get(str(p))
    if entry is None or entry['size_bytes']!=s[2]:unknown.append({'path':str(p),'reason':'not bound by exact size prior inventory'});continue
    items.append({'path':str(p),'root':str(root),'relative':str(rel),'stat_before':s,**entry})
 d={'schema':'authorized_teacher_raw_purge_plan/v1','created_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),'authorization_quote':AUTH,'remote':REMOTE,'roots':list(map(str,ROOTS)),'prior_inventory':proof,'items':items,'skipped_unbound':unknown,'files':len(items),'logical_bytes':sum(x['size_bytes']for x in items),'protected':['source snapshots','models','all RL_for_unitree','camera_mode','reports/config/summary receipts','test_results finite fixtures'],'raw_reaudit_will_be_unavailable':True}
 target.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'plan':str(target),'sha256':sha(target),'files':d['files'],'logical_bytes':d['logical_bytes'],'unbound_skipped':len(unknown)},ensure_ascii=False))
def free():
 return {str(p):os.statvfs(p).f_bavail*os.statvfs(p).f_frsize for p in [TEACHER,ROOTS[1]]}
def execute(expected,commit,evidence_manifest):
 path=OUT/'PURGE_PLAN.json';assert sha(path)==expected,'plan checksum mismatch'
 d=json.loads(path.read_text());assert d['authorization_quote']==AUTH and d['roots']==list(map(str,ROOTS))
 assert evidence_manifest.is_file(),'preserved evidence manifest missing'
 refs=subprocess.check_output(['git','ls-remote',REMOTE,'refs/heads/main'],text=True).split()
 assert refs and refs[0]==commit,'repository main is not the verified preserved commit'
 for item in d['prior_inventory']:assert sha(Path(item['path']))==item['sha256'],'prior manifest changed'
 journal=OUT/'PURGE_JOURNAL.jsonl';before=free();started=datetime.datetime.now(datetime.timezone.utc).isoformat();deleted=0;total=0
 with journal.open('x')as f:
  for row in d['items']:
   p=Path(row['path']);root=Path(row['root']);rel=p.relative_to(root)
   assert root in ROOTS and not root.is_symlink() and candidate(rel)
   assert not p.is_symlink() and sig(p)==row['stat_before'],'raw file changed: '+str(p)
   assert p.resolve()==p,'symlink parent not permitted'
   # Preserve summaries/source/config directories; unlink only exact planned files.
   p.unlink();deleted+=1;total+=row['size_bytes']
   f.write(json.dumps({'path':str(p),'bytes':row['size_bytes'],'prior_sha256':row['sha256'],'deleted_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat()},ensure_ascii=False)+'\n')
   if deleted%1000==0:f.flush();os.fsync(f.fileno());print(json.dumps({'deleted_files':deleted,'logical_bytes':total}),flush=True)
  f.flush();os.fsync(f.fileno())
 after=free();receipt={'schema':'authorized_teacher_raw_purge_receipt/v1','status':'completed','authorization_quote':AUTH,'began_UTC':started,'ended_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),'plan_sha256':expected,'journal_sha256':sha(journal),'deleted_files':deleted,'deleted_logical_bytes':total,'free_bytes_before':before,'free_bytes_after':after,'remote':REMOTE,'preserved_remote_commit':commit,'evidence_manifest':str(evidence_manifest),'evidence_manifest_sha256':sha(evidence_manifest),'raw_available_for_historical_complete_reaudit':False,'original_prior_manifests_modified':False,'models_training_camera_mode_touched':False,'scope':'Selected raw files only in three Teacher run roots; source snapshots/reports/finite fixtures retained'}
 (OUT/'PURGE_RECEIPT.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps(receipt,ensure_ascii=False))
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute-plan-sha');p.add_argument('--preserved-commit');p.add_argument('--evidence-manifest',type=Path);a=p.parse_args()
 if a.execute_plan_sha:
  if not a.preserved_commit or not a.evidence_manifest:p.error('execution requires verified commit and preserved evidence manifest')
  execute(a.execute_plan_sha,a.preserved_commit,a.evidence_manifest)
 else:plan()
