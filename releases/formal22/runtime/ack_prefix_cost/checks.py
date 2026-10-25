#!/usr/bin/env python3
"""4 bounded real-prefix software sequences. No controller/ROS construction."""
from pathlib import Path
import ast,collections,copy,hashlib,json,os,subprocess,sys,time,types,tempfile
from contextlib import contextmanager
from unittest.mock import patch
H=Path(__file__).resolve().parent;W=H.parent;C=W/'candidate';sys.path.insert(0,str(C));import numpy as np
R=Path('/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261024T151836Z_closed_loop_R85_compact_protocol_02_round27_b077')
BASE='7fc8cfe8150d8d41447d828f3669dce865e53316'
oldtext=subprocess.check_output(['git','-C',str(W),'show',BASE+':candidate/controller.py'],text=True);newtext=(C/'controller.py').read_text();oldtree=ast.parse(oldtext);newtree=ast.parse(newtext);oldclass=next(n for n in oldtree.body if isinstance(n,ast.ClassDef)and n.name=='PIDNavigation');newclass=next(n for n in newtree.body if isinstance(n,ast.ClassDef)and n.name=='PIDNavigation')
COUNTS=collections.Counter();real_loads=json.loads;real_sha=hashlib.sha256
class J:
 @staticmethod
 def loads(raw,*a,**k):COUNTS['json_loads']+=1;COUNTS['json_input_bytes']+=len(raw);return real_loads(raw,*a,**k)
class HH:
 @staticmethod
 def sha256(raw=b''):COUNTS['sha256']+=1;COUNTS['hash_input_bytes']+=len(raw);return real_sha(raw)
def make_class(node):
 methods=[]
 for n in node.body:
  if isinstance(n,ast.FunctionDef)and n.name in['_decode_ack_entries_reusing_lines','_read_ack_entries_cached','get_ack']:
   n=copy.deepcopy(n);n.decorator_list=[];methods.append(n)
 cls=ast.ClassDef(name='Prefix',bases=[],keywords=[],body=methods,decorator_list=[]);mod=ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[]));ns={'json':J,'hashlib':HH,'os':os,'copy':copy,'OrderedDict':collections.OrderedDict,'_r96_cpu':types.SimpleNamespace(PROBE=None)};exec(compile(mod,str(C/'controller.py'),'exec'),ns);return ns['Prefix']
Old,New=make_class(oldclass),make_class(newclass)
# Execute only the actual new post-verified-scope binding block, never init/Node.
init=next(n for n in newclass.body if isinstance(n,ast.FunctionDef)and n.name=='__init__');binding=next(n for n in init.body if isinstance(n,ast.If)and 'handoff_scope_sha256' in ast.unparse(n.test));fn=ast.FunctionDef(name='bind_scope',args=ast.arguments(posonlyargs=[],args=[ast.arg(arg='self')],kwonlyargs=[],kw_defaults=[],defaults=[]),body=[copy.deepcopy(binding)],decorator_list=[]);ns={'hashlib':HH};exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(C/'controller.py'),'exec'),ns);bind_scope=ns['bind_scope']
core=ast.parse((C/'cascade_core.py').read_text());corecls=next(n for n in core.body if isinstance(n,ast.ClassDef)and n.name=='Controller');fresh=copy.deepcopy(next(n for n in corecls.body if isinstance(n,ast.FunctionDef)and n.name=='_fresh'));integer=copy.deepcopy(next(n for n in core.body if isinstance(n,ast.FunctionDef)and n.name=='integer'));ns={'np':np};exec(compile(ast.fix_missing_locations(ast.Module(body=[integer,fresh],type_ignores=[])),str(C/'cascade_core.py'),'exec'),ns);original_fresh=ns['_fresh']
raw=(R/'closed_loop_executor_ack.json').read_bytes();scope=(R/'navigation_scope.json').read_bytes();profile=real_loads((W.parents[1]/'trials/compact_protocol_02/PROFILE.json').read_text());base_rows=real_loads(raw)['entries'];pose=base_rows[-1]['stamp_ns'];scope_sha=real_sha(scope).hexdigest();assert profile['r85_active_turn_braking']=={'enabled':False}
results={};cost={};first_counts={}
with tempfile.TemporaryDirectory(prefix='ack_prefix_',dir=H)as td:
 run=Path(td);ackpath=run/'closed_loop_executor_ack.json';scopepath=run/'navigation_scope.json';scopepath.write_bytes(scope)
 def atomic(data):
  p=run/'next_ack.tmp';p.write_bytes(data);p.replace(ackpath)
 def encode(rows):return ('{"entries":['+',\n'.join(json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False)for x in rows)+'],"schema":"teacher_known_velocity_ack/v1"}\n').encode()
 def node(cls,active=False):
  x=cls();x.run=run;x.profile=profile;x.feedback={'stamp_ns':pose};x.heading_gate=types.SimpleNamespace(active_braking_enabled=active);x.handoff_scope_sha256=scope_sha if cls is Old or active else None
  if cls is New:bind_scope(x)
  return x
 real_rt,real_rb,real_open,real_stat=Path.read_text,Path.read_bytes,Path.open,Path.stat
 def rt(p,*a,**k):
  value=real_rt(p,*a,**k)
  if p==ackpath:COUNTS['ACK_read_text']+=1;COUNTS['ACK_read_bytes']+=len(value.encode('utf-8'))
  return value
 def rb(p,*a,**k):
  value=real_rb(p,*a,**k)
  if p==scopepath:COUNTS['scope_read_bytes_calls']+=1;COUNTS['scope_read_bytes']+=len(value)
  return value
 class Reader:
  def __init__(self,f):self.f=f
  def __enter__(self):self.f.__enter__();return self
  def __exit__(self,*a):return self.f.__exit__(*a)
  def fileno(self):return self.f.fileno()
  def read(self,*a):
   b=self.f.read(*a);COUNTS['ACK_binary_reads']+=1;COUNTS['ACK_read_bytes']+=len(b);return b
 def op(p,*a,**k):
  f=real_open(p,*a,**k)
  return Reader(f)if p==ackpath and(a and a[0]=='rb')else f
 def pst(p,*a,**k):
  if p==ackpath:COUNTS['ACK_stat']+=1
  return real_stat(p,*a,**k)
 with patch.object(Path,'read_text',rt),patch.object(Path,'read_bytes',rb),patch.object(Path,'open',op),patch.object(Path,'stat',pst):
  atomic(raw);o,n=node(Old),node(New)
  for p in[pose,base_rows[0]['stamp_ns'],pose-1,pose+1]:assert o.get_ack(p)==n.get_ack(p)
  selected=n.get_ack();selected['sequence']=-1;assert n.get_ack()==o.get_ack();results['1_real_closed_ACK_equal_and_cache_alias_private']='PASS'
  atomic(raw);before=ackpath.stat().st_ino;assert o.get_ack()==n.get_ack();atomic(raw);assert ackpath.stat().st_ino!=before;COUNTS.clear();assert o.get_ack()==n.get_ack();first_counts['new_inode_same_payload_combined_old_new']=dict(COUNTS)
  rows=copy.deepcopy(base_rows);rows[-1]['sequence']+=1;atomic(encode(rows));assert o.get_ack()==n.get_ack()==rows[-1]
  for bad in[b'{"entries":',b'{"entries":null}',b'{"entries":{}}',raw[:-17],b'{"entries":['+json.dumps(base_rows[0]).encode()+b',\n{"stamp_ns":'+b'],"schema":"teacher_known_velocity_ack/v1"}\n']:
   atomic(bad);assert o.get_ack()is None and n.get_ack()is None
  ackpath.unlink();assert o.get_ack()is None and n.get_ack()is None;atomic(raw);assert o.get_ack()==n.get_ack();results['2_atomic_new_inode_updated_ACK_bad_JSON_halfline_missing_recovery']='PASS'
  row=copy.deepcopy(base_rows[-1]);row['source_scope_sha256']='f'*64;atomic(encode([row]));assert o.get_ack()is None and n.get_ack()is None
  row=copy.deepcopy(base_rows[-1]);row['stamp_ns']=pose+1;atomic(encode([row]));assert o.get_ack()is None and n.get_ack()is None
  older=copy.deepcopy(base_rows[-1]);older['stamp_ns']=pose-1;atomic(encode([older,row]));assert o.get_ack()==n.get_ack()==older
  older['stamp_ns']=pose-300_000_001;atomic(encode([older]));assert o.get_ack()==n.get_ack()==older
  try:original_fresh(types.SimpleNamespace(minimal_simulation_baseline=True),n.get_ack(),pose,older['received_wall_ns'])
  except ValueError as exc:assert str(exc)=='Source stale or future'
  else:raise AssertionError('original age gate lost')
  results['3_scope_future_causal_older_selection_and_original300ms_rejection']='PASS'
  # Original canonical full-array fallback format must stay semantically equal.
  atomic(json.dumps({'entries':base_rows,'schema':'teacher_known_velocity_ack/v1'},separators=(',',':')).encode());assert o.get_ack()==n.get_ack();atomic(raw)
  # Cold source binding measured separately, using actual __init__ block.
  COUNTS.clear();n=node(New);first_counts['new_init_binding']=dict(COUNTS);assert n.handoff_scope_sha256==scope_sha;COUNTS.clear();assert n.get_ack()==base_rows[-1];first_counts['new_cold_get_ack']=dict(COUNTS)
  for label,cls in [('old',Old),('new',New)]:
   atomic(raw);x=node(cls);x.get_ack();COUNTS.clear();wall=time.monotonic_ns();cpu=time.thread_time_ns()
   for _ in range(200):assert x.get_ack()==base_rows[-1]
   cost[label+'_unchanged200']={'wall_ns':time.monotonic_ns()-wall,'thread_CPU_ns':time.thread_time_ns()-cpu,'operations':dict(COUNTS)}
  # Each iteration rotates inode and supplies a new last ACK row. Fixture I/O
  # occurs outside measured function spans; both arms consume identical bytes.
  for label,cls in [('old',Old),('new',New)]:
   atomic(raw);x=node(cls);x.get_ack();COUNTS.clear();wc=cc=0;expected=copy.deepcopy(base_rows)
   for k in range(50):
    expected[-1]['sequence']=base_rows[-1]['sequence']+k+1;atomic(encode(expected));wall=time.monotonic_ns();cpu=time.thread_time_ns();got=x.get_ack();wc+=time.monotonic_ns()-wall;cc+=time.thread_time_ns()-cpu;assert got==expected[-1]
   cost[label+'_new_inode50']={'wall_ns':wc,'thread_CPU_ns':cc,'operations':dict(COUNTS)}
  results['4_same_file_fallback_init_binding_and_bounded_component_cost']='PASS'
# Assert production diff has only init/get_ack changes, and no original guard math changed.
def normalized(tree):
 tree=copy.deepcopy(tree)
 for n in tree.body:
  if isinstance(n,ast.ClassDef)and n.name=='PIDNavigation':n.body=[m for m in n.body if not(isinstance(m,ast.FunctionDef)and m.name in['__init__','get_ack'])]
 return ast.dump(tree,include_attributes=False)
assert normalized(oldtree)==normalized(newtree)
assert subprocess.check_output(['git','-C',str(W),'show',BASE+':candidate/cascade_core.py'])==(C/'cascade_core.py').read_bytes()
receipt={'schema':'ACK_prefix_actual_method_software/v1','base_commit':BASE,'PASS':True,'groups':results,'NoNode':True,'original_closed_run_unchanged':True,'source_fixture':{'run':str(R),'ACK_bytes':len(raw),'ACK_SHA256':real_sha(raw).hexdigest(),'scope_bytes':len(scope),'scope_SHA256':scope_sha,'profile_non_braking':True},'counts':first_counts,'cost':cost,'cost_limit':'Offline hot prefix only; fixed closed ACK/50 scalar sequence updates and OS page cache, no executor/ROS/Actor load or real-time/navigation benefit claim. Writes excluded from timed get_ack. Instrumented operation counters add bookkeeping overhead, so measured numbers are not production latency.','actual_production_changed_methods':['PIDNavigation.__init__ scalar binding','PIDNavigation.get_ack parsing source/selected-copy'],'cascade_math_and_all_other_controller_methods_unchanged':True,'candidate_source_sha256':real_sha(newtext.encode()).hexdigest(),'base_source_sha256':real_sha(oldtext.encode()).hexdigest()}
(H/'CHECKS.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt,indent=2))
