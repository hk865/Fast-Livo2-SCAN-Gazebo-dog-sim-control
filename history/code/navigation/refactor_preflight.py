"""Strict source-equivalence declaration for one C51 bounded integration copy.
No control or safety predicates are replaced. This creates no runtime or files.
"""
from pathlib import Path
import hashlib,json,sys,os,time
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def contract(here):
    here=Path(here).resolve();d=json.loads((here/'C51_REFACTOR_CONTRACT.json').read_text())
    if (d.get('schema')!='C51_bounded_source_equivalence/v1' or d['candidate_root']!=str(here)
        or d.get('runtime_algorithm_changes')is not False or d.get('REPORT_mode')is not False
        or d.get('historical_runtime_PASS_inherited')is not False or d.get('maximum_simulation_attempts')!=1):
        raise RuntimeError('C51 admission identity/scope differs')
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
      env.get('DEMO_RUN_DIR')==run and env.get('ROS_DOMAIN_ID')=='90' and env.get('GZ_PARTITION')=='teacher_closed_loop_'+Path(run).name,
      latch.get('run')==run and latch.get('maximum_attempts')==1,
      prepared['freeze'].get(str(Path(run)/'runtime_plan.json'))==plan_digest,
      plan.get('run')==run and plan.get('profile_path')==str(here/'profiles/engineering_nav46.json'),
      plan.get('simulation_only')is True and plan.get('real_robot')is False and plan.get('navigation_ground_truth_used')is False,
      plan.get('ros_domain')==90 and plan.get('wall_budget_s')==180.,
      plan.get('R55_shutdown_probe',{}).get('source_dir')==str(stage),
      plan.get('R55_shutdown_probe',{}).get('max_native_s')==45 and plan.get('R55_shutdown_probe',{}).get('max_acquisition_wall_s')==120,
      plan.get('R55_shutdown_probe',{}).get('output_hard_B')==2147483648 and plan.get('R55_shutdown_probe',{}).get('strict_original_gates')is True,
      same(me,saved)and me.get('state')!='Z' and me.get('pid')==me.get('pgid')==me.get('session'),
      saved.get('owned_run')==run and saved.get('role')=='navigation_stack',
      saved.get('ownership_token')==owner.get('token')==permit.get('token') and permit.get('identity')==saved,
      same(parent,owner['parent'])and parent.get('state')!='Z' and me.get('ppid')==parent.get('pid'),
      'navigation_stack'in owner.get('allowed_roles',[])and now<owner['wall_deadline'],
      parent_command[1:]==['-B',str(stage/'probe_owner.py'),'run'],
      heartbeat.get('state')=='RUNNING' and 0<=now-heartbeat['monotonic_wall']<3.,
    ]
    if not all(checks):raise RuntimeError('C51 bounded launch identity/context differs: '+str([i for i,x in enumerate(checks)if not x]))
def require_bounded_launch(d,entry):
    if str(entry)!=d['allowed_launch_entry']['path'] or sha(entry)!=d['allowed_launch_entry']['sha256']:
        raise RuntimeError('C51 admission requires the source-bound bounded owner; direct legacy run entry refused')
    stage=Path(d['bounded_stage']);prepared=json.loads((stage/'PREPARED.json').read_text())
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
    manifest=Path(d['sealed_C51_manifest'])
    if sha(manifest)!=d['sealed_C51_manifest_sha256']:
        raise RuntimeError('Sealed C51 manifest changed')
    m=json.loads(manifest.read_text());sealed=Path(m['candidate'])
    for name,digest in m['sources'].items():
        if sha(sealed/name)!=digest:raise RuntimeError('Sealed C51 source changed: '+name)
        if name not in d['admission_only_files'] and sha(here/name)!=digest:
            raise RuntimeError('C51 integration source differs: '+name)
    actual={str(p.relative_to(here))for p in here.rglob('*')if p.is_file()and'__pycache__'not in p.parts and'.git'not in p.parts}
    if actual!=set(m['sources'])|set(d['new_admission_files']):
        raise RuntimeError('C51 integration has unlisted files')
    for name,digest in d['admission_source_sha256'].items():
        if sha(here/name)!=digest:raise RuntimeError('C51 admission source changed: '+name)
    restored_runtime=(here/'known_scene_runtime.py').read_text().replace(
        'from collision_minimal_capture import CollisionObservedP95SourceFrames as P95SourceFrames',
        'from p95_capture import P95SourceFrames')
    if restored_runtime!=(sealed/'known_scene_runtime.py').read_text():
        raise RuntimeError('C51 diagnostic observer changed runtime beyond its import alias')
    restored_adapter=(here/'known_scene_adapter.py').read_text().replace(
        "(getattr(self.diagnostic_capture, 'capture_needed', False) or getattr(self.diagnostic_capture, 'first_collision_evidence_needed', False))",
        "getattr(self.diagnostic_capture, 'capture_needed', False)")
    if restored_adapter!=(sealed/'known_scene_adapter.py').read_text():
        raise RuntimeError('C51 diagnostic evidence request changed adapter beyond its pin flag')
    expected_policy=json.loads((sealed/'POLICY_SELECTION_CONTRACT.json').read_text())
    expected_policy['candidate_origin']=str(here/'policy_candidate/worker.py')
    if json.loads((here/'POLICY_SELECTION_CONTRACT.json').read_text())!=expected_policy:
        raise RuntimeError('C51 private policy binding changed beyond candidate path')
    baseline=json.loads(Path(d['original_baseline_receipt']).read_text())
    original=Path(baseline['baseline'])
    for name,digest in baseline['baseline_files'].items():
        if sha(original/name)!=digest:raise RuntimeError('Original C44 source changed: '+name)
    for name,digest in d['stage_input_sha256'].items():
        if sha(name)!=digest:raise RuntimeError('C51 bounded stage input changed: '+name)
    receipt=Path(d['fresh_CPU_receipt'])
    cpu=json.loads(receipt.read_text())
    if (cpu.get('schema')!='R33_startup_finite_CPU_receipt/v1' or cpu.get('tests_pass')is not True
        or cpu.get('failures')!=0 or cpu.get('actual_candidate')!=str(here)
        or cpu.get('comparisons')!=260 or cpu.get('full_module_AST_checks')!=9):
        raise RuntimeError('C51 actual-candidate CPU validation incomplete')
    for name,digest in cpu['source_sha256'].items():
        path=Path(name)if Path(name).is_absolute()else here/name
        if sha(path)!=digest:raise RuntimeError('C51 CPU checked source changed: '+str(path))
    declared=json.loads(Path(d['changed_files_receipt']).read_text())['changed']
    return {name:dict(baseline=str(original/name),candidate=str(here/name),sha256=sha(here/name),
        baseline_sha256=sha(original/name),declared_change='Fresh full-module AST and CPU behavior equivalence; original predicates retained')
        for name in declared}
