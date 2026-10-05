#!/usr/bin/env python3
"""Actual observer finalization AST, synthetic failure injection only."""
from pathlib import Path
import ast,copy,json,hashlib,datetime
from types import SimpleNamespace
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 if (OUT/'cleanup_review.json').exists():raise RuntimeError('Do not overwrite evidence')
 src=OUT/'sources/ring.py';namespace={};exec(compile(src.read_text(),str(src),'exec'),namespace)
 finish=namespace['finish_observer_evidence'];Ring=namespace['ActualCloudPreRoll'];checks={};results=[]
 class Writer:
  def __init__(self,append_error=False,close_error=False,existing_error=None):self.error=existing_error;self.append_error=append_error;self.close_error=close_error;self.appended=[];self.closed=0
  def append(self,p,d):
   if self.append_error:raise RuntimeError('injected append error')
   self.appended.append((str(p),copy.deepcopy(d)))
  def close(self):
   self.closed+=1
   if self.close_error:raise RuntimeError('injected close error')
 def make(name,**kwargs):
  run=OUT/('cleanup_case_'+name);run.mkdir(exist_ok=False);directory=run/'dynamic_sensor_evidence';directory.mkdir();(directory/'original_123.npz').write_bytes(b'original immutable evidence')
  n=SimpleNamespace(evidence=Writer(**kwargs),pre_roll=Ring(),observer_error=None,directory=directory,pre_roll_flushed=True,saved_xyz={('cloud',123):'oldhash'},counts={'cloud':10},program=SimpleNamespace(phase='blocking',failure=None),destroyed=False)
  n.destroy_node=lambda:setattr(n,'destroyed',True);return n,run
 end={'kind':'fixture_end','role':'sensor_observer','phase':'blocking','failure':None,'counts':{'cloud':10},'navigation_validation':'unverified'}
 cases=[('normal',{}),('append_error',{'append_error':True}),('close_error',{'close_error':True}),('append_and_close_error',{'append_error':True,'close_error':True}),('already_queue_error',{'append_error':True,'close_error':True,'existing_error':'queue overflow'})]
 for name,kwargs in cases:
  n,run=make(name,**kwargs);before=sha(n.directory/'original_123.npz');thrown=None
  try:finish(n,run,end)
  except RuntimeError as error:thrown=str(error)
  m=json.loads((run/'dynamic_sensor_evidence_manifest.json').read_text());bad=name!='normal'
  checks[name+'_manifest_status_and_exception_not_swallowed']=m['status']==('failed' if bad else 'recorded_unverified') and bool(thrown)==bad
  checks[name+'_close_always_attempted_and_evidence_not_changed']=n.evidence.closed==1 and sha(n.directory/'original_123.npz')==before and m['files']['dynamic_sensor_evidence/original_123.npz']==before
  if name=='normal':checks['normal_fixture_end_payload_exact']=n.evidence.appended[0][1]==end and not m['cleanup_errors']
  if name=='already_queue_error':checks['queue_failure_manifest_preserves_explicit_error']=m['queue_error']=='queue overflow' and len(m['cleanup_errors'])==3
  results.append({'case':name,'exception':thrown,'manifest_sha256':sha(run/'dynamic_sensor_evidence_manifest.json'),'status':m['status']})
 for kind in ['ring','observer']:
  n,run=make(kind+'_existing_error')
  if kind=='ring':n.pre_roll.error='actual ring capacity overflow'
  else:n.observer_error='actual archived source conflict'
  try:finish(n,run,end)
  except RuntimeError:thrown=True
  else:thrown=False
  m=json.loads((run/'dynamic_sensor_evidence_manifest.json').read_text())
  checks[kind+'_existing_failure_manifest_and_nonzero_exception']=thrown and m['status']=='failed' and n.evidence.closed==1
 # Execute the original finalbody from main, preserving inherited callback exceptions.
 tree=ast.parse((OUT/'sources/observer.py').read_text());mainfn=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name=='main');finaltry=next(x for x in mainfn.body if isinstance(x,ast.Try)and x.finalbody)
 fn=ast.FunctionDef(name='actual_finally',args=ast.arguments(posonlyargs=[],args=[],kwonlyargs=[],kw_defaults=[],defaults=[]),body=copy.deepcopy(finaltry.finalbody),decorator_list=[])
 env={'finish_observer_evidence':finish};exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(OUT/'sources/observer.py'),'exec'),env)
 def context(name,role,**kwargs):
  n,run=make(name,**kwargs);shutdown=[];env.update(node=n,run=run,args=SimpleNamespace(role=role),rclpy=SimpleNamespace(try_shutdown=lambda:shutdown.append(True)));return n,run,shutdown
 n,run,sd=context('actual_finalbody_append_error','sensor_observer',append_error=True)
 try:env['actual_finally']()
 except RuntimeError:thrown=True
 else:thrown=False
 checks['actual_finalbody_failure_still_destroys_shutsdown_and_propagates']=thrown and n.destroyed and sd==[True] and json.loads((run/'dynamic_sensor_evidence_manifest.json').read_text())['status']=='failed'
 n,run,sd=context('actual_finalbody_normal','sensor_observer');env['actual_finally']();checks['actual_finalbody_normal_destroys_shutsdown_no_success_promotion']=n.destroyed and sd==[True] and json.loads((run/'dynamic_sensor_evidence_manifest.json').read_text())['status']=='recorded_unverified'
 n,run,sd=context('original_callback_exception','sensor_observer')
 try:
  try:raise ValueError('original callback failure')
  finally:env['actual_finally']()
 except ValueError as e:preserved=str(e)=='original callback failure'
 else:preserved=False
 checks['original_callback_failure_not_swallowed_by_normal_finalization']=preserved and n.destroyed and sd==[True]
 n,run,sd=context('unchanged_mover_normal','mover');env['actual_finally']();checks['mover_original_append_close_destroy_only_no_observer_manifest']=n.evidence.closed==1 and len(n.evidence.appended)==1 and n.destroyed and sd==[True] and not(run/'dynamic_sensor_evidence_manifest.json').exists()
 n,run,sd=context('unchanged_mover_close_error','mover',close_error=True)
 try:env['actual_finally']()
 except RuntimeError:thrown=True
 else:thrown=False
 checks['mover_original_close_exception_propagates_with_destroy']=thrown and n.destroyed and sd==[True] and not(run/'dynamic_sensor_evidence_manifest.json').exists()
 # Cleanup only writes evidence paths; no command or truth mutations.
 body=next(x for x in ast.parse(src.read_text()).body if isinstance(x,ast.FunctionDef)and x.name=='finish_observer_evidence')
 checks['finalization_no_publisher_service_signal_or_NAVcommand']=not any(isinstance(x,ast.Call)and isinstance(x.func,ast.Attribute)and x.func.attr in ('publish','call_async','kill','send_signal','create_publisher') for x in ast.walk(body))
 x={'asof_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'passed':all(checks.values()),'checks':checks,'cases':results,'actual_navigation_coverage_pass_asserted':False,'source_hashes':{'snapshot_ring':sha(src),'current_ring':sha(ROOT/'navigation/dynamic/observer_preroll.py'),'snapshot_observer':sha(OUT/'sources/observer.py'),'current_observer':sha(ROOT/'navigation/dynamic/dynamic_obstacle.py')},'script_sha256':sha(__file__),'limits':['Synthetic error injection; cannot guarantee failed manifest persistence if the audit destination filesystem itself is unwritable. That failure must remain nonzero/missing evidence, never pass.','Only observer evidence finalization changes; original mover behavior and V44 control must remain frozen.']}
 (OUT/'cleanup_review.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps({'passed':x['passed'],'checks':checks,'sha256':sha(OUT/'cleanup_review.json')}))
 if not x['passed']:raise SystemExit(1)
if __name__=='__main__':main()
