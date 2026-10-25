"""Offline-only deployment proposal. Never deploy, prepare, inspect host or execute.

Consumes explicit SHA-pinned inputs; emits proposed file contents into a JSON
receipt only. The existing runner/admission/closure lifecycle remains separate.
"""
from pathlib import Path
import argparse,ast,copy,datetime as dt,hashlib,json,subprocess,sys
A=Path(__file__).resolve().parent
TARGETS={
 'R107':dict(worktree='queue_timing',implementation='3ae0d0786df5ed5dc27f06f41c86f44464681a07',base='59f96ab341fa63045e02d97baa30edacd0a5a0ef',template='terminal_owner',source_head='158ea3a68a5307cbcdbc8fa0b657c08eae86b760'),
 'R108':dict(worktree='dynamic_evidence',implementation='d99c2d1f9bd7ab44899b32048bf1f3afbae648a5',base='64a6dfe0898c197272f520dd1fc092589cef0a96',template='prequalification_hold')}
LIMITS=dict(maximum_retained_bytes=170000000000,maximum_round_bytes=10000000000,
 additional_reserved_bytes=10000000000,maximum_runtime_wall_s=2400,maximum_cleanup_wall_s=300)

def require(value,reason):
 if not value:raise ValueError(reason)
def digest(raw):return hashlib.sha256(raw).hexdigest()
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for chunk in iter(lambda:f.read(1024**2),b''):h.update(chunk)
 return h.hexdigest()
def document(path):
 p=Path(path);require(p.stat().st_size<=4*1024**2,'oversized JSON input')
 return json.loads(p.read_bytes())
def pinned(pin):
 p=Path(pin['path']).resolve();require(p.is_file() and sha(p)==pin['sha256'],'input SHA mismatch: '+str(p));return p
def git(root,*args):return subprocess.check_output(['git',*args],cwd=root,text=True).strip()
def literal(path,name):
 for n in ast.parse(Path(path).read_text()).body:
  if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets):return ast.literal_eval(n.value)
 raise ValueError('missing literal '+name)
def jbytes(v):return (json.dumps(v,indent=2)+'\n').encode()

def check_authority(authority,deadline,domain,kind,now,planned_start):
 require(type(domain)is int and 0<=domain<=232,'FastDDS domain outside supported 0..232 range')
 require(authority.get('schema')=='r_next_round_external_authority/v1','external authority schema missing')
 require(authority.get('direction')==kind and authority.get('deadline_utc')==deadline,'explicit deadline/direction authority mismatch')
 require(authority.get('deadline_basis')=='parent_authorized_single_round_administration','new single-round administrative authority required')
 require(authority.get('automatic_renewal')is False and authority.get('automatic_retry')is False,'no automatic renewal/retry')
 require(all(type(authority.get(k))is int and authority[k]==v for k,v in LIMITS.items()),'resource scope differs')
 start=dt.datetime.fromisoformat(planned_start);end=dt.datetime.fromisoformat(deadline)
 require(start.tzinfo is not None and end.tzinfo is not None and end==start+dt.timedelta(seconds=2700),
         'deadline must equal explicit planned start plus 2400+300 seconds')
 require(authority.get('planned_start_UTC')==planned_start,'planned start authority mismatch')
 require(now<=start,'planned start has passed; no automatic deadline shift')

def resource_preview(C,kind,deadline,authority_pin):
 old_source=(C/'r85_resource_renewal.py').read_text()
 old_proof=document(C/'R105_RESOURCE_RENEWAL_UNTIL20261023T1500_170GB.json')
 proof=copy.deepcopy(old_proof);proof['deadline_utc']=deadline
 proof['kind']='explicit_'+kind+'_single_round_administration_170GB_v1'
 proof['source_authority'].append(authority_pin)
 name=kind+'_RESOURCE_ADMIN_SINGLE_ROUND.json';proof_raw=jbytes(proof)
 updates={'R105_CONFIG':{'enabled':True,'kind':proof['kind']},
          'R105_PROOF_SHA256':digest(proof_raw),'R105_EXPECTED_METADATA':proof}
 tree=ast.parse(old_source);lines=old_source.splitlines(keepends=True);edits=[]
 for n in tree.body:
  if isinstance(n,ast.Assign):
   names=[x.id for x in n.targets if isinstance(x,ast.Name)]
   for key in names:
    if key in updates:edits.append((n.lineno,n.end_lineno,key+'='+repr(updates[key])+'\n'))
    if key=='R105_PROOF_PATH':edits.append((n.lineno,n.end_lineno,"R105_PROOF_PATH=HERE/"+repr(name)+'\n'))
  if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call):
   target=ast.unparse(n.value.func)
   if target in ("R105_EXPECTED_METADATA.update", "R105_EXPECTED_METADATA['source_authority'].append"):
    edits.append((n.lineno,n.end_lineno,''))
 for first,last,value in sorted(edits,reverse=True):lines[first-1:last]=[value]
 source=''.join(lines)
 anchor="R105_CONFIG="
 require(source.count(anchor)==1,'resource administrative constants anchor differs')
 basis=kind+'_parent_authorized_single_round_administration'
 source=source.replace(anchor,'DEADLINE_UTC='+repr(deadline)+' if R105_RETAINED_BUDGET_ENABLED else DEADLINE_UTC\n'
  +'BASIS='+repr(basis)+' if R105_RETAINED_BUDGET_ENABLED else BASIS\n'+anchor)
 def functions(text):return [ast.dump(n,include_attributes=False) for n in ast.parse(text).body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef))]
 require(functions(source)==functions(old_source),'resource function AST changed')
 require({k:v for k,v in proof.items() if k not in ('deadline_utc','kind','source_authority')}==
         {k:v for k,v in old_proof.items() if k not in ('deadline_utc','kind','source_authority')},'numeric resource protection changed')
 require(proof['source_authority'][:-1]==old_proof['source_authority'],'legacy authority chain changed')
 return name,proof_raw,source,basis

def source_audit(kind,review_pin,metadata_review_pin=None):
 cfg=TARGETS[kind];root=A/'worktrees'/cfg['worktree'];C=root/'candidate'
 source_head=cfg.get('source_head',cfg['implementation'])
 require(git(root,'rev-parse','HEAD')==source_head,'explicit source HEAD differs')
 require(not git(root,'status','--porcelain'),'implementation is not clean')
 review_path=pinned(review_pin);review=document(review_path)
 commit=review.get('implementation_commit',review.get('implementation'))
 require(commit==cfg['implementation'],'independent review implementation mismatch')
 require(review.get('passed')is True or review.get('software_review_passed')is True,'independent review not passed')
 hashes=review.get('verified_file_sha256',review.get('source_sha256',{}))
 changed=git(root,'diff','--name-only',cfg['base'],cfg['implementation']).splitlines()
 exact={};metadata={};metadata_receipt=None
 if kind=='R107':
  allowed={'R107_PREPARED_CANDIDATE.zh-CN.md','scan_transport_runtime/BUILD_PLAN.json',
   'scan_transport_runtime/COMPONENT_BUILD_RECEIPT.json','scan_transport_runtime/build_private_scan.py'}
  delta=set(git(root,'diff','--name-only',cfg['implementation'],source_head).splitlines())
  require(delta==allowed,'explicit metadata-only HEAD file delta differs')
  require(not git(root,'diff','--name-only',cfg['implementation'],source_head,'--','candidate',
    'scan_transport_runtime/scan_ws/src'),'business/C++ source differs under metadata HEAD')
  require(metadata_review_pin is not None,'independent metadata-head review required')
  mp=pinned(metadata_review_pin);mr=document(mp)
  require(mr.get('passed')is True and mr.get('source_head')==source_head
   and mr.get('business_implementation')==cfg['implementation'],'metadata-head review identity differs')
  mh=mr.get('verified_file_sha256',{})
  for rel in sorted(delta):
   require(rel in mh and sha(root/rel)==mh[rel],'unreviewed metadata file: '+rel)
   metadata[rel]=mh[rel]
  metadata_receipt=dict(path=str(mp),sha256=sha(mp))
 for rel in changed:
  p=root/rel;expected=hashes.get(rel,hashes.get(Path(rel).name) if rel.startswith('candidate/') else None)
  original=subprocess.check_output(['git','show',cfg['implementation']+':'+rel],cwd=root)
  require(expected is not None and digest(original)==expected,'business review original bytes differ: '+rel)
  require(rel in metadata or (p.is_file() and sha(p)==expected),'unreviewed current business file: '+rel)
  exact[rel]=expected
 return cfg,root,C,dict(review=dict(path=str(review_path),sha256=sha(review_path)),
  source_head=source_head,business_implementation=cfg['implementation'],changed_file_sha256=exact,
  metadata_review=metadata_receipt,metadata_file_sha256=metadata,business_cpp_diff_empty=True)

def proposal(spec):
 kind=spec['kind'];cfg,root,C,audit=source_audit(kind,spec['review'],spec.get('metadata_review'))
 authority_path=pinned(spec['authority']);authority=document(authority_path)
 check_authority(authority,spec['deadline_utc'],spec['domain'],kind,dt.datetime.now(dt.timezone.utc),spec['planned_start_UTC'])
 trial=Path(spec['new_trial']).resolve()
 require(trial.parent==A/'trials' and not trial.exists(),'trial must be a new direct child; never overwrite history')
 admin_pin=dict(path=str(authority_path),sha256=sha(authority_path),bytes=authority_path.stat().st_size)
 proof_name,proof_raw,resource_source,resource_basis=resource_preview(C,kind,spec['deadline_utc'],admin_pin)
 proof=C/proof_name
 template=A/'worktrees'/cfg['template']/'candidate'
 require(pinned(spec['binding_template'])==template/'r86_runtime_binding.py','wrong baseline-specific binding template')
 require(pinned(spec['profile_template'])==A/'trials'/cfg['template']/'PROFILE.json','wrong baseline-specific profile template')
 binding=(template/'r86_runtime_binding.py').read_text()
 require("sys.modules['__main__' if name=='controller' else name]" in binding,'actual controller __main__ binding absent')
 require('r105_terminal_parking_owner' in binding and 'renewal.configuration(node.profile)' in binding,'terminal/resource actual binding absent')
 require(('prequalification_environment=' in binding)==(kind=='R108'),'R106 template policy differs')
 if kind=='R107':
  anchor="('r102_', 'r103_', 'r104_', 'r105_')"
  require(binding.count(anchor)==1,'auxiliary source binding anchor differs')
  binding=binding.replace(anchor,"('r102_', 'r103_', 'r104_', 'r105_', 'r107_')")
 ast.parse(binding)
 # Full replacement SCAN contract must itself be independently reviewed. The
 # script never fabricates build/ELF receipt fields from a binary's existence.
 scan_path=pinned(spec['scan_contract']);scan=document(scan_path)
 scan_review_path=pinned(spec['scan_review']);sr=document(scan_review_path)
 require(sr.get('passed')is True and sr.get('implementation_commit')==cfg.get('source_head',cfg['implementation']),'SCAN independent build review differs')
 require(sr.get('contract_sha256')==sha(scan_path),'SCAN contract not covered by build review')
 required={str(Path(scan[k]).resolve()) for k in ('executable','config','build_receipt')}
 build_files=sr.get('verified_file_sha256',{})
 require(required<=set(build_files),'SCAN ELF/config/build receipt not all reviewed')
 for name,h in build_files.items():
  path=Path(name)
  if not path.is_absolute():
   path=(root/path).resolve();require(path.is_relative_to(root),'relative review path escapes worktree')
  require(sha(path)==h,'SCAN build input changed: '+name)
 if kind=='R107':
  own=(root/'scan_transport_runtime').resolve()
  require(all(Path(p).is_relative_to(own) for p in required),'R107 still bound to old SCAN deployment')
  require(sr.get('r107_cpp_probe_in_actual_elf')is True,'R107 C++ probe ELF provenance absent')
 require(scan.get('actual_loaded_binary_witness_required')is True,'actual loaded SCAN witness still required')
 scan['source_root']=str(C)
 policy=document(C/'POLICY_SELECTION_CONTRACT.json');policy['candidate_origin']=str(C/'policy_candidate/worker.py')
 profile=document(spec['profile_template']['path'])
 require(profile['r85_session']['maximum_retained_bytes']==170000000000 and profile['raw_storage_budget']['max_run_bytes']==170000000000,'profile 170GB mismatch')
 require(profile.get('r86_expired_prefix',{}).get('enabled',False)is False,'S1 must remain OFF')
 require(profile['r85_dynamic_event_contract']['completed_same_geometry_preservation']is False,'completed-history policy must remain OFF')
 profile['r85_session'].update(scan_workspace_contract=str(C/'R73_SCAN_RUNTIME_CONTRACT.json'),
  scan_workspace_contract_sha256=digest(jbytes(scan)),resource_renewal_proof=str(proof),resource_renewal_proof_sha256=digest(proof_raw),deadline_utc=spec['deadline_utc'])
 profile['r85_resource_renewal']['kind']=json.loads(proof_raw)['kind']
 profile['diagnostic_observe_only'].update(user_authorized_work_deadline_utc=spec['deadline_utc'],work_deadline_basis=resource_basis)
 worker=(C/'worker.py').read_text();ipc=(C/'ipc_request_window.py').read_text()
 require('worker_executor_sources_loaded.json' in worker and 'initialize_command_reader' in worker and 'import worker' in ipc,'actual Actor entry/loaded source receipt chain missing')
 env=dict(R99_CLOCK_FEEDBACK='0',R103_SCAN_SHARED_STORAGE='0',R97_DIAGNOSTIC_QUEUE_BYTES='2097152',
  R103_IMU_SAFETY_INTAKE='1',R103_EARLY_REQUEST_ACK='1',R103_EXHAUSTED_DRIVE_REPLACEMENT='1',
  R104_PARKING_CAUSAL_JOIN='1',R105_TERMINAL_PARKING_OWNER='1',R105_RETAINED_BUDGET_170GB='1',
  R106_PREQUALIFICATION_SOURCE_HOLD='1' if kind=='R108' else '0',R107_SCAN_TRANSPORT_PROBE='1' if kind=='R107' else '0')
 files={'candidate/'+proof_name:proof_raw.decode(),'candidate/r85_resource_renewal.py':resource_source,
  'candidate/R73_SCAN_RUNTIME_CONTRACT.json':jbytes(scan).decode(),
  'candidate/POLICY_SELECTION_CONTRACT.json':jbytes(policy).decode(),
  'candidate/r86_runtime_binding.py':binding,'trial/PROFILE.json':jbytes(profile).decode()}
 return dict(schema='r_next_round_offline_proposal/v1',kind=kind,implementation=cfg['implementation'],source_head=cfg.get('source_head',cfg['implementation']),base=cfg['base'],
  worktree=str(root),new_trial=str(trial),domain=spec['domain'],authority=dict(path=str(authority_path),sha256=sha(authority_path)),
  planned_start_UTC=spec['planned_start_UTC'],deadline_utc=spec['deadline_utc'],source_audit=audit,environment_delta=env,
  proposed_files={p:dict(sha256=digest(s.encode()),content=s) for p,s in files.items()},
  actual_runtime_loading_verified=False,deployed=False,prepared=False,simulation_started=False,
  status='OFFLINE_PROPOSAL_ONLY_NOT_READY_TO_EXECUTE',remaining=[
   'Independent review of generated binding/profile/source delta before root applies to a fresh deployment commit',
   'New direction dispatch/authority inputs for existing launch/close lifecycle; retain original lock, admission, one-shot intent and closure checks',
   'Fresh admission must establish private domain and physical resource availability; this static range check is not admission',
   'Prepared navigation_scope/runtime_plan/source_manifest: worker entry, actual worker and executor_command origin/snapshot double-key SHA',
   'Before-spin actual controller module/source and instance checks; R107 probe installation receipt when selected',
   'Runtime worker_executor_sources_loaded.json, Actor identity/CRC, and actual loaded SCAN executable/config/build witness'])

def self_test():
 now=dt.datetime(2026,10,23,10,tzinfo=dt.timezone.utc);planned='2026-10-23T14:15:00+00:00';deadline='2026-10-23T15:00:00+00:00'
 authority=dict(LIMITS,schema='r_next_round_external_authority/v1',direction='R107',deadline_utc=deadline,
  deadline_basis='parent_authorized_single_round_administration',planned_start_UTC=planned,automatic_renewal=False,automatic_retry=False)
 check_authority(authority,deadline,224,'R107',now,planned)
 cases=[(authority,deadline,233,'R107',now),(authority,deadline,224,'R108',now),
  (authority,deadline,224,'R107',now+dt.timedelta(hours=5)),
  (dict(authority,deadline_basis='administrative-only'),deadline,224,'R107',now),
  (dict(authority,maximum_retained_bytes=160000000000),deadline,224,'R107',now),
  (dict(authority,automatic_renewal=True),deadline,224,'R107',now)]
 for args in cases:
  try:check_authority(*args,planned)
  except ValueError:continue
  raise AssertionError('negative authority case accepted')
 C=A/'worktrees/dynamic_evidence/candidate'
 before=sha(C/'r85_resource_renewal.py')
 name,raw,source,basis=resource_preview(C,'R108',deadline,dict(path='/nonexistent/software_fixture_only.json',sha256='0'*64,bytes=0))
 require(sha(C/'r85_resource_renewal.py')==before,'dry-run mutated source')
 require(name=='R108_RESOURCE_ADMIN_SINGLE_ROUND.json' and json.loads(raw)['deadline_utc']==deadline,'resource preview identity differs')
 return dict(passed=True,checks=10,scope='authority/domain guards and resource pure generation/AST/numeric preservation only; no deployment/runtime PASS',mutation=False)

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--spec');p.add_argument('--output');p.add_argument('--self-test',action='store_true');args=p.parse_args()
 require(args.self_test or args.spec,'explicit specification required; no automatic defaults for authority/deadline')
 try:result=self_test() if args.self_test else proposal(document(args.spec))
 except Exception as error:
  result=dict(status='BLOCKED',deployed=False,prepared=False,simulation_started=False,reason=type(error).__name__+': '+str(error))
  print(json.dumps(result,ensure_ascii=False,indent=2));return 2
 if args.output:
  # Only a new receipt; never write proposal content to candidate or trial.
  out=Path(args.output).resolve();require(out.parent==A,'receipt must be a new R26 top-level file')
  with out.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
 else:print(json.dumps(result,ensure_ascii=False,indent=2))
 return 0
if __name__=='__main__':sys.exit(main())
