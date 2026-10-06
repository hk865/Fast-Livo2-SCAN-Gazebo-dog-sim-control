#!/usr/bin/env python3
"""Observer-only finally fault probes on owned mock children, no ROS/sim."""
from pathlib import Path
from types import SimpleNamespace as NS
import ast,hashlib,json,subprocess,sys,tempfile
ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation';sys.path.insert(0,str(NAV))
from dynamic.observer_preroll import ActualCloudPreRoll,finish_observer_evidence
from runtime_io import EvidenceWriter
checks={}
def check(n,c):
 checks[n]={'passed':bool(c)}
 if not c:raise AssertionError(n)
with tempfile.TemporaryDirectory()as tmp:
 run=Path(tmp);directory=run/'dynamic_sensor_evidence';directory.mkdir()
 node=NS(evidence=EvidenceWriter(),pre_roll=ActualCloudPreRoll(),observer_error=None,directory=directory,
  pre_roll_flushed=True,saved_xyz={},counts={'cloud':2})
 node.evidence.append(run/'ordered.jsonl',{'n':1});node.evidence.append(run/'ordered.jsonl',{'n':2})
 finish_observer_evidence(node,run,{'kind':'fixture_end','role':'sensor_observer'})
 check('success_records_order_and_end_preserved',[json.loads(x)['n']for x in (run/'ordered.jsonl').read_text().splitlines()]==[1,2]
  and json.loads((run/'obstacle_motion_history.jsonl').read_text())['kind']=='fixture_end')
 manifest=json.loads((run/'dynamic_sensor_evidence_manifest.json').read_text())
 check('normal_drained_success_metadata_not_false_navigation_pass',manifest['status']=='recorded_unverified'and manifest['cleanup_errors']==[]and not manifest['navigation_input'])
 child=r'''
from pathlib import Path
from types import SimpleNamespace as NS
import sys
sys.path.insert(0,sys.argv[1])
from dynamic.observer_preroll import ActualCloudPreRoll,finish_observer_evidence
run=Path(sys.argv[2]);mode=sys.argv[3];directory=run/'dynamic_sensor_evidence';directory.mkdir()
class Writer:
 def __init__(self):self.error='queue overflow'if mode=='queue_error'else None
 def append(self,path,row):
  if mode in ('queue_error','end_append'):raise RuntimeError('fixture append fault')
  path.write_text('actual mock end\n')
 def close(self):
  (run/'close_attempted').write_text('yes')
  if mode in ('queue_error','drain'):raise RuntimeError('drain fault')
node=NS(evidence=Writer(),pre_roll=ActualCloudPreRoll(),observer_error=None,directory=directory,
 pre_roll_flushed=True,saved_xyz={},counts={})
if mode=='ring_overflow':node.pre_roll.error='pre-roll capacity overflow'
if mode=='manifest_error':(run/'dynamic_sensor_evidence_manifest.json').mkdir()
finish_observer_evidence(node,run,{'kind':'fixture_end'})
'''
 for mode in ('queue_error','end_append','drain','ring_overflow','manifest_error'):
  case=run/mode;case.mkdir()
  process=subprocess.run([sys.executable,'-c',child,str(NAV),str(case),mode],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  check(mode+'_propagates_nonzero_owned_mock_exit',process.returncode!=0)
  check(mode+'_still_attempts_writer_close',(case/'close_attempted').exists())
  if mode!='manifest_error':
   m=json.loads((case/'dynamic_sensor_evidence_manifest.json').read_text())
   check(mode+'_writes_explicit_failed_manifest',m['status']=='failed'and bool(m['cleanup_errors']))
source=NAV/'dynamic/observer_preroll.py';tree=ast.parse(source.read_text())
prior=ROOT/'test_results/navigation_v45_observer_preroll_freeze.json';old=json.loads(prior.read_text())
oldring=ROOT/'test_results/observer_preroll_independent_review_20261004/snapshots/observer_preroll.py'
# Prior independent snapshot paths vary; use its source-hash snapshot catalog.
snapshots=list((ROOT/'test_results/observer_preroll_independent_review_20261004').rglob('*.py'))
oldring=next((p for p in snapshots if hashlib.sha256(p.read_bytes()).hexdigest()==old['source_hashes']['navigation/dynamic/observer_preroll.py']),None)
if oldring is None:raise RuntimeError('Original observer ring snapshot not found')
oldtree=ast.parse(oldring.read_text())
check('original_ring_class_AST_unchanged',ast.dump(next(x for x in tree.body if getattr(x,'name',None)=='ActualCloudPreRoll'),include_attributes=False)==ast.dump(next(x for x in oldtree.body if getattr(x,'name',None)=='ActualCloudPreRoll'),include_attributes=False))
check('all_V44_control_profile_guard_physics_runner_sources_unchanged',all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==v for n,v in old['source_hashes'].items()if n not in ('navigation/dynamic/dynamic_obstacle.py','navigation/dynamic/observer_preroll.py')))
receipt={'schema':1,'status':'passed','checks':checks,'owned_mock_children':5,'actual_ROS_started':False,'Gazebo_started':False,
 'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [source,NAV/'dynamic/dynamic_obstacle.py',Path(__file__)]},
 'limitation':'Filesystem manifest-write failure propagates nonzero; no code can promise a written manifest when the filesystem rejects its path'}
out=Path(__file__).with_name('offline_checks.json');out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':'passed','checks':len(checks),'receipt':str(out)}))
