# Publication 2026-10-13: local paths redacted; numeric/control behavior retained; local admission is not transferred.
"""One sole strict non-GT full-route owner. Fresh once latch, original safety, normal close."""
from pathlib import Path
import argparse,datetime as dt,hashlib,importlib.util,json,os,signal,sys,time,types,uuid
S=Path(__file__).resolve().parent
C=Path('./multifloor_demo/teacher_mode/navigation/engineering_v63_r83_parking_hysteresis_candidate')
AUTHORITY=json.loads((S/"AUTHORITY.json").read_text())

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,d):
 with Path(p).open('x')as f:json.dump(d,f,indent=2,allow_nan=False);f.write('\n')
def module(p,name):
 spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m

def require_no_simulation():
 if dt.datetime.now(dt.timezone.utc)>=dt.datetime.fromisoformat(AUTHORITY['deadline_UTC']):raise RuntimeError('Overnight deadline: no new simulation start')
 out=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:a=p.joinpath('cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
  except OSError:continue
  if (any(Path(x).name=='gz'for x in a)and'sim'in a)or any(Path(x).name in ('fastlivo_mapping','ipc_request_window.py','shadow.launch.py','stack.launch.py')for x in a):out.append(dict(pid=int(p.name),argv=a[:8]))
 if out:raise RuntimeError('Existing simulation roles prevent serial start: '+str(out))

def signal_event(run,kind,target,sig):
 record=dict(kind=kind,target=target,signal=int(sig),monotonic_wall=time.monotonic())
 with(run/'R77_SIGNAL_EVENTS.jsonl').open('a')as f:f.write(json.dumps(dict(event='attempt',**record))+'\n')
 (os.kill if kind=='pid'else os.killpg)(target,sig)
 with(run/'R77_SIGNAL_EVENTS.jsonl').open('a')as f:f.write(json.dumps(dict(event='sent',**record))+'\n')

def load():
 sys.path[:0]=[str(S),str(C)];guard=module(S/'storage_guard.py','storage_guard')
 base=module(C/'run.py','R77_actual_strict_runner')
 from probe_stop import diagnostic_stop
 holder=dict(run=None)
 ns=dict(base.__dict__,S=S,diagnostic_stop=diagnostic_stop,TRACE_RUN=holder,
  trace_signal=lambda kind,target,sig:signal_event(holder['run'],kind,target,sig))
 exec(compile((S/'stop_owned_traced.py').read_text(),str(S/'stop_owned_traced.py'),'exec'),ns)
 exec(compile((S/'diagnostic_execute.py').read_text(),str(S/'diagnostic_execute.py'),'exec'),ns)
 return base,guard,ns['execute']

def verify_stage():
 if AUTHORITY.get('T0_revision_authorized')is not True or AUTHORITY.get('maximum_simulation_attempts')!=1:
  raise RuntimeError('Explicit R80 one-run/new T0 authorization absent')
 d=json.loads((S/'RUNTIME_BINDINGS.json').read_text())
 for p,h in d['bindings'].items():
  if sha(p)!=h:raise RuntimeError('R80 reviewed runtime input changed: '+p)
 d=json.loads((S/'FRESH_CPU_RECEIPT.json').read_text())
 if d.get('tests_pass')is not True or d.get('actual_candidate')!=str(C)or d.get('business_workflows_verified')!=3:
  raise RuntimeError('R80 fresh cross-module workflows incomplete')
 for p,h in d['source_sha256'].items():
  if sha(p)!=h:raise RuntimeError('R80 tested source changed: '+p)
 for name in ('INDEPENDENT_PARKING_REVIEW.json','INDEPENDENT_INPUT_EVIDENCE_REVIEW.json','INDEPENDENT_OWNER_BUDGET_REVIEW.json'):
  r=json.loads((S/name).read_text())
  if r.get('review_passed')is not True or r.get('blocking_findings')!=[]:
   raise RuntimeError('R80 independent review incomplete: '+name)
  for p,h in r['source_sha256'].items():
   if sha(p)!=h:raise RuntimeError('R80 reviewed input changed: '+p)
 for p,h in json.loads((S/'SDK_BINDINGS.json').read_text())['freeze'].items():
  if sha(p)!=h:raise RuntimeError('Protected SDK input changed: '+p)

def inspect_owner():
 base,guard,runner=load()
 if Path(guard.__file__).resolve()!=S/'storage_guard.py'or sys.modules['storage_guard']is not guard:
  raise RuntimeError('Actual owner does not load the current-retained guard')
 if guard.HARD_BYTES!=130_000_000_000 or guard.ROUND_HARD_BYTES!=10_000_000_000:
  raise RuntimeError('Actual owner budget constants differ')
 # This imports the actual candidate proxy, without prepare, simulation,
 # ROS initialization, model loading, watchdog or signaling.
 proxy=module(C/'storage_guard.py','R80_candidate_guard_proxy_inspection')
 if Path(proxy.base.__file__).resolve()!=S/'storage_guard.py':raise RuntimeError('Candidate guard proxy differs')
 d=dict(schema='R80_actual_owner_import_budget_binding/v1',passed=True,
  owner=str(S/'run.py'),actual_guard_file=str(Path(guard.__file__).resolve()),
  candidate_proxy_actual_guard=str(Path(proxy.base.__file__).resolve()),
  actual_runner_file=runner.__code__.co_filename,base_file=base.__file__,
  source_sha256={str(p):sha(p)for p in (S/'run.py',S/'storage_guard.py',S/'retained_usage.py',
   S/'diagnostic_execute.py',S/'owned_guardian.py',C/'storage_guard.py')},
  simulation=False,watchdog_started=False,signals_sent=False,Actor_loaded=False)
 (S/'OWNER_BUDGET_LOAD_CHECK.json').write_text(json.dumps(d,indent=2)+'\n')
 print(json.dumps(d),flush=True)

def prepare():
 require_no_simulation();verify_stage()
 for n in ('ALLOCATED_RUN.json','PREPARED_RUN.json','ONE_SIMULATION_LATCH.json'):
  if(S/n).exists():raise RuntimeError('R71 already allocated/prepared/consumed')
 base,guard,_=load();base.resource_guard(base.resource());base.require_private_domain(126)
 from run_storage import create_run
 run,location=create_run(base.ROOT,dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_closed_loop_R83_parking_hysteresis_'+uuid.uuid4().hex[:4])
 write(S/'ALLOCATED_RUN.json',dict(run=str(run),location=location));guard.register(run)
 args=types.SimpleNamespace(profile='engineering_nav46',label='R83_parking_hysteresis',shadow_sample_contract=None,surface_validity=0,
  cpu_python=Path('./external/local_machine/isaacsim/kit/python/bin/python3'),domain=126,real_time_factor=1.)
 plan=base.prepare(run,args);plan.update(wall_budget_s=4590.,autonomous_navigation_verified=False)
 plan['navigation_stack_diagnostic_environment'].update(FASTLIVO_BOUNDARY_TIMING='1',FASTLIVO_BOUNDARY_TIMING_FILE=str(run/'R77_native_boundary_timing.jsonl'),FASTLIVO_DIAGNOSTIC_DIR='',FASTLIVO_FRAME_TIMING_FROM_NS='650000000000')
 plan['R77_strict_identity_handoff']=dict(source_dir=str(S),max_native_s=1500,max_acquisition_wall_s=4350,
  output_hard_B=10000000000,output_soft_stop_B=6800000000,strict_original_quality_source_safety_and_coverage_gates=True,
  approved_parking_contract='ordinary_zero_ack_fixed5',keep_legacy_arrival_window_diagnostic=True,continuation_authority_sha256=sha(S/'AUTHORITY.json'),
  actual_source_point_identity_required=True,pose_commit_required=True,identity_predicate_revision='actual_source_ID_coverage/v1',
  GT_carrier_allowed=False,DSH_used=False,fresh_full_route_required=True)
 (run/'runtime_plan.json').write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n')
 freeze=dict(json.loads((C/'R33_STARTUP_PREFLIGHT.json').read_text())['verified_inputs_sha256'])
 freeze.update(json.loads((S/'RUNTIME_BINDINGS.json').read_text())['bindings'])
 freeze.update({str(S/n):sha(S/n)for n in ('RUNTIME_BINDINGS.json','FRESH_CPU_RECEIPT.json',
  'INDEPENDENT_PARKING_REVIEW.json','INDEPENDENT_INPUT_EVIDENCE_REVIEW.json','INDEPENDENT_OWNER_BUDGET_REVIEW.json')})
 freeze.update({str(p):sha(p)for p in run.rglob('*')if p.is_file()and '__pycache__'not in p.parts})
 write(S/'PREPARED_RUN.json',dict(run=str(run),freeze=freeze,actual_runtime_started=False,navigation_ground_truth_used=False,DSH_used=False))
 write(S/'SOURCE_FREEZE.json',dict(bindings=freeze,prepared_run_sha256=sha(S/'PREPARED_RUN.json'),strict_full_autonomous_PASS=False))
 print(json.dumps(dict(prepared_run=str(run),frozen=len(freeze))),flush=True)

def execute():
 require_no_simulation();verify_stage();d=json.loads((S/'PREPARED_RUN.json').read_text());run=Path(d['run']);plan=json.loads((run/'runtime_plan.json').read_text())
 frozen=json.loads((S/'SOURCE_FREEZE.json').read_text())
 if sha(S/'PREPARED_RUN.json')!=frozen['prepared_run_sha256']:raise RuntimeError('R71 preparation changed')
 for p,h in d['freeze'].items():
  if sha(p)!=h:raise RuntimeError('R71 frozen input changed: '+p)
 base,guard,runner=load();base.resource_guard(base.resource());base.require_private_domain(126)
 from pid_scope import verify_scope
 verify_scope(run/'navigation_scope.json');guard.observe(run,plan['profile'],starting=True)
 require_no_simulation()  # Recheck UTC after all scope/budget/hash checks, immediately before latch.
 write(S/'ONE_SIMULATION_LATCH.json',dict(run=str(run),maximum_attempts=1,navigation_ground_truth_used=False,
  authorization=str(S/'AUTHORITY.json'),utc=dt.datetime.now(dt.timezone.utc).isoformat()))
 error=runner(run,plan)
 actual={}
 try:actual=json.loads((run/'runtime_manifest.json').read_text())
 except OSError:pass
 write(S/'RUN_OUTCOME.json',dict(run=str(run),error=error,full_autonomous_navigation_verified=False,
  runtime_closure_completed=actual.get('R77_runtime_closure_completed'),
  cleanup_errors=actual.get('cleanup_errors',[]),typed_capture_close_verified=actual.get('typed_capture_close_verified'),pipeline_normal_completed=actual.get('pipeline_normal_completed'),
  actor_archive_verified=actual.get('worker_archive_integrity',{}),simulation_attempts=1,navigation_ground_truth_used=False))
 print(json.dumps(dict(run=str(run),error=error,full_autonomous_navigation_verified=False)),flush=True)
 return 0 if actual.get('R77_runtime_closure_completed')is True and error is None else 2

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');p.add_argument('--execute',action='store_true');p.add_argument('--inspect',action='store_true');a=p.parse_args()
 if a.inspect:
  if a.prepare or a.execute:p.error('Inspection cannot prepare/execute')
  inspect_owner();raise SystemExit(0)
 if a.prepare==a.execute:p.error('Choose prepare or execute')
 if a.prepare:prepare()
 else:raise SystemExit(execute())
