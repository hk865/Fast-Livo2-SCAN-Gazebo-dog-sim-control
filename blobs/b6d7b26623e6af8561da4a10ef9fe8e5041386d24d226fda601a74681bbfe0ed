"""Strict source-equivalence declaration for one C51 bounded integration copy.
No control or safety predicates are replaced. This creates no runtime or files.
"""
from pathlib import Path
import hashlib,json,sys,os,time
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def contract(here):
    here=Path(here).resolve();d=json.loads((here/'C51_REFACTOR_CONTRACT.json').read_text())
    if (d.get('schema')!='R69_explicit_strict_identity_handoff_admission/v1' or d['candidate_root']!=str(here)
        or d.get('historical_runtime_PASS_inherited')is not False
        or d.get('autonomous_navigation_verified')is not False
        or d.get('REPORT_mode')is not False or d.get('maximum_simulation_attempts')!=1):
        raise RuntimeError('R69 diagnostic identity/scope differs')
    return d

def process_identity(pid):
    text=Path(f'/proc/{pid}/stat').read_text();f=text[text.rfind(')')+2:].split()
    return dict(pid=int(pid),state=f[0],ppid=int(f[1]),pgid=int(f[2]),session=int(f[3]),
        start_ticks=int(f[19]),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
def validate_launch_records(d,entry,argv,env,prepared,latch,plan,plan_digest,me,saved,owner,parent,permit,heartbeat,parent_command,now):
    stage=Path(d['bounded_stage']);here=Path(d['candidate_root']);run=prepared['run']
    identity_keys=('pid','start_ticks','pgid','session','boot_id')
    same=lambda a,b:all(a.get(k)==b.get(k)for k in identity_keys)
    checks=[
      str(entry)==d['allowed_launch_entry']['path'],
      argv==[str(entry),'launch',str(here/'stack.launch.py'),'run_dir:='+run],
      env.get('DEMO_RUN_DIR')==run and env.get('ROS_DOMAIN_ID')=='118' and env.get('GZ_PARTITION')=='teacher_closed_loop_'+Path(run).name,
      latch.get('run')==run and latch.get('maximum_attempts')==1 and latch.get('navigation_ground_truth_used')is False,
      prepared['freeze'].get(str(Path(run)/'runtime_plan.json'))==plan_digest,
      plan.get('run')==run and plan.get('profile_path')==str(here/'profiles/engineering_nav46.json'),
      plan.get('simulation_only')is True and plan.get('real_robot')is False and plan.get('navigation_ground_truth_used')is False and plan.get('autonomous_navigation_verified')is False,
      plan.get('ros_domain')==118 and plan.get('wall_budget_s')==4590.,
      plan.get('R69_strict_identity_handoff',{}).get('source_dir')==str(stage),
      plan.get('R69_strict_identity_handoff',{}).get('max_native_s')==1500 and plan.get('R69_strict_identity_handoff',{}).get('max_acquisition_wall_s')==4350,
      plan.get('R69_strict_identity_handoff',{}).get('output_hard_B')==10000000000 and plan.get('R69_strict_identity_handoff',{}).get('strict_original_gates')is True,
      same(me,saved)and me.get('state')!='Z' and me.get('pid')==me.get('pgid')==me.get('session'),
      saved.get('owned_run')==run and saved.get('role')=='navigation_stack',
      saved.get('ownership_token')==owner.get('token')==permit.get('token') and permit.get('identity')==saved,
      same(parent,owner['parent'])and parent.get('state')!='Z' and me.get('ppid')==parent.get('pid'),
      'navigation_stack'in owner.get('allowed_roles',[])and now<owner['wall_deadline'],
      parent_command[1:]==['-B',str(stage/'run.py'),'--execute'],
      heartbeat.get('state')=='RUNNING' and 0<=now-heartbeat['monotonic_wall']<3.,
      plan.get('R69_strict_identity_handoff',{}).get('continuation_authority_sha256')==sha(stage/'AUTHORITY.json'),
      0<owner['wall_deadline']-now<=4590+240,
    ]
    if not all(checks):raise RuntimeError('C51 bounded launch identity/context differs: '+str([i for i,x in enumerate(checks)if not x]))
def require_bounded_launch(d,entry):
    if str(entry)!=d['allowed_launch_entry']['path'] or sha(entry)!=d['allowed_launch_entry']['sha256']:
        raise RuntimeError('C51 admission requires the source-bound bounded owner; direct legacy run entry refused')
    stage=Path(d['bounded_stage']);prepared=json.loads((stage/'PREPARED_RUN.json').read_text())
    latch=json.loads((stage/'ONE_SIMULATION_LATCH.json').read_text());run=Path(prepared['run']).resolve()
    if str(run)!=prepared['run']:raise RuntimeError('C51 prepared canonical run differs')
    plan=json.loads((run/'runtime_plan.json').read_text());me=process_identity(os.getpid())
    saved=json.loads((run/'owned_roles/navigation_stack.identity.json').read_text())
    owner=json.loads((run/'guardian_owner.json').read_text());parent=process_identity(owner['parent']['pid'])
    permit=json.loads((run/'owned_roles/navigation_stack.permit.json').read_text())
    heartbeat=json.loads((run/'guardian_heartbeat.json').read_text())
    parent_command=Path(f"/proc/{parent['pid']}/cmdline").read_bytes().decode().strip('\0').split('\0')
    validate_launch_records(d,entry,sys.argv,os.environ,prepared,latch,plan,sha(run/'runtime_plan.json'),
        me,saved,owner,parent,permit,heartbeat,parent_command,time.monotonic())
    for path,digest in prepared['freeze'].items():
        if sha(path)!=digest:raise RuntimeError('C51 bounded launch frozen input changed: '+path)
def require_bounded_entry(here):
    d=contract(here);entry=Path(sys.argv[0]).resolve()
    if str(entry)in d['allowed_owner_entries']:
        if sha(entry)!=d['allowed_owner_entries'][str(entry)]:raise RuntimeError('C51 bounded owner entry changed')
        return
    require_bounded_launch(d,entry)
def verify_source_equivalence(here):
    here=Path(here).resolve();d=contract(here)
    manifest_path=Path(d['candidate_manifest'])
    if sha(manifest_path)!=d['candidate_manifest_sha256']:raise RuntimeError('R69 manifest changed')
    manifest=json.loads(manifest_path.read_text());baseline=Path(manifest['baseline']).resolve()
    before=json.loads(Path(d['baseline_manifest']).read_text())
    if sha(d['baseline_manifest'])!=d['baseline_manifest_sha256']:raise RuntimeError('R69 baseline receipt changed')
    for name,digest in before['baseline_files'].items():
        if sha(baseline/name)!=digest:raise RuntimeError('Protected R57 source changed: '+name)
    prior=json.loads((baseline/'R33_STARTUP_PREFLIGHT.json').read_text())
    for name,digest in prior['verified_inputs_sha256'].items():
        if sha(name)!=digest:raise RuntimeError('Protected R57 external source changed: '+name)
    if (baseline!=here.with_name('engineering_v48_r68_delivery_diagnostic_candidate')
            or before.get('baseline')!=str(baseline) or before.get('candidate')!=str(here)):
        raise RuntimeError('R69 baseline identity differs')
    excluded=set(manifest['excluded_gate_files'])
    if excluded!={'C51_REFACTOR_CONTRACT.json','R33_STARTUP_PREFLIGHT.json','R33_STARTUP_SOURCE_REVIEW.json'}:
        raise RuntimeError('R69 excluded gate set differs')
    if set(manifest['sources'])&excluded:raise RuntimeError('R69 gate/source sets overlap')
    changed_set={name for name,digest in manifest['sources'].items()
                 if digest!=before['baseline_files'].get(name)}
    if changed_set!=set(manifest['declared_changed_files']):raise RuntimeError('R69 declared delta differs')
    actual={str(p.relative_to(here))for p in here.rglob('*')if p.is_file()and'__pycache__'not in p.parts and'.git'not in p.parts}
    if actual!=set(manifest['sources'])|excluded:raise RuntimeError('R69 unlisted files')
    rows={}
    for name,digest in manifest['sources'].items():
        if sha(here/name)!=digest:raise RuntimeError('R69 candidate source changed: '+name)
        changed=digest!=before['baseline_files'].get(name)
        if changed and name not in set(manifest['declared_changed_files']):raise RuntimeError('R69 undeclared runtime edit: '+name)
        rows[name]=dict(baseline=str(baseline/name),candidate=str(here/name),sha256=digest,
            baseline_sha256=before['baseline_files'].get(name),declared_change='Declared prospective source identity/path handoff repair' if changed else 'Exact R57 bytes retained')
    for name,digest in d['stage_input_sha256'].items():
        if sha(name)!=digest:raise RuntimeError('R69 stage input changed: '+name)
    if sha(d['fresh_CPU_receipt'])!=d['fresh_CPU_receipt_sha256']:raise RuntimeError('R69 CPU receipt changed')
    cpu=json.loads(Path(d['fresh_CPU_receipt']).read_text())
    if cpu.get('tests_pass')is not True or cpu.get('actual_candidate')!=str(here):raise RuntimeError('R69 CPU checks incomplete')
    for name,digest in cpu['source_sha256'].items():
        if sha(name)!=digest:raise RuntimeError('R69 CPU source changed: '+name)
    return rows

