"""Finite original Guardian/lifecycle sequence checks; no process or ROS launch."""
from pathlib import Path
import argparse
import ast
import contextlib
import copy
import hashlib
import io
import json
import signal
import sys
import tempfile
from unittest.mock import patch

sys.dont_write_bytecode=True
import owned_guardian as guardian
import pipeline_lifecycle as pipeline

# Execute the actual approved exception class only; do not import/call its census.
base=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-07/task/go2_engineering_v33/R83_PARKING_HYSTERESIS_STAGE/storage_guard.py')
node=next(n for n in ast.parse(base.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='StorageStop')
namespace={};exec(compile(ast.Module(body=[node],type_ignores=[]),str(base),'exec'),namespace)
StorageStop=namespace['StorageStop']


def write(path,value):
    path.write_text(json.dumps(value,indent=2)+'\n')


class Clock:
    def __init__(self):self.value=100.
    def now(self):return self.value
    def sleep(self,seconds):self.value+=seconds


def fixture(root):
    run=root.resolve();(run/'owned_roles').mkdir();(run/'fastlivo_debug').mkdir()
    parent=dict(pid=900001,start_ticks=1,pgid=900001,session=900001,boot_id=guardian.BOOT,state='R')
    role=dict(pid=900002,start_ticks=2,pgid=900002,session=900002,boot_id=guardian.BOOT,
              state='R',ppid=parent['pid'],role='navigation_stack',ownership_token='fixture',owned_run=str(run))
    mapper=dict(pid=900003,start_ticks=3,pgid=role['pgid'],session=role['session'],state='R')
    source=Path(guardian.__file__).resolve().parent/'pipeline_lifecycle.py'
    write(run/'source_manifest.json',{str(source):pipeline.sha(source)})
    plan=dict(profile=dict(r85_session=dict(enabled=True)),source_manifest_sha256=pipeline.sha(run/'source_manifest.json'))
    write(run/'runtime_plan.json',plan)
    owner=dict(parent=parent,token='fixture',allowed_roles=['navigation_stack'],wall_deadline=1000.,
               heartbeat_timeout_s=10.,parent_cleanup_grace_s=60.,runtime_plan_sha256=pipeline.sha(run/'runtime_plan.json'))
    write(run/'guardian_owner.json',owner);write(run/'runner_heartbeat.json',dict(monotonic_wall=100.))
    write(run/'owned_roles/navigation_stack.identity.json',role)
    launch=run/'navigation_stack_effective_environment.json';write(launch,dict(DEMO_RUN_DIR=str(run)))
    witness=dict(verified=True,mapping_identity=mapper,source_bound_run_dir=str(run),
        launch_environment_receipt_file=str(launch),launch_environment_receipt_sha256=pipeline.sha(launch))
    write(run/'slam_loaded_binary.json',witness)
    summary=dict(schema='staged_input_pipeline_v19/v1',normal_completed=True,context_valid_at_drain=True,
        accepted=3,delivered=3,committed=3,pending=0,inflight=0,ready=0,bytes=0,canceled=0,
        rejected_capacity=0,closed_rejections=0,failure='',uncommitted_packets=[])
    write(run/'fastlivo_debug/pipeline_v19_summary.json',summary)
    normal=dict(schema='teacher_pipeline_v19_stop_request/v1',run=str(run),normal_completed=True,
        signal_sent=True,signal=int(signal.SIGUSR1),error=None,gate_errors=[],saved_identity=mapper,
        started_monotonic_wall=90.,sent_monotonic_wall=90.1,ended_monotonic_wall=91.,
        summary_sha256=pipeline.sha(run/'fastlivo_debug/pipeline_v19_summary.json'),summary=summary)
    write(run/'pipeline_normal_stop.json',normal)
    release=dict(parent=parent,monotonic_wall=99.)
    observation=dict(schema='r85_current_retained_and_round_storage/v1',monotonic_wall=98.,
        stop_reasons=['fresh_round_predictive_stop'],current_retained_budget_bytes=80000000000,
        R85_round_budget=dict(charged_B=2000000000,predictive_rate_B_s=70000000,
            projected_full_stop_B=11211088640,predictive_soft_stop=True,hard_stop=False))
    return owner,role,release,observation


def sequence(root,kind):
    owner,role,release,observation=fixture(root)
    clock=Clock();signals=[];cleanup=[];pending=False
    if kind=='normal_release':write(root/'guardian_release.json',release)
    if kind=='evidence_collision':write(root/'guardian_stop_cause.json',dict(existing_evidence=True))
    def storage(previous):
        nonlocal pending
        if kind=='stale_parent':release['parent']={**release['parent'],'start_ticks':999}
        if kind=='stale_role':
            changed=json.loads((root/'owned_roles/navigation_stack.identity.json').read_text())
            changed['ownership_token']='foreign';write(root/'owned_roles/navigation_stack.identity.json',changed)
        if kind=='incomplete_summary':
            s=json.loads((root/'fastlivo_debug/pipeline_v19_summary.json').read_text());s['pending']=1
            write(root/'fastlivo_debug/pipeline_v19_summary.json',s)
            p=json.loads((root/'pipeline_normal_stop.json').read_text());p['summary']=s
            p['summary_sha256']=pipeline.sha(root/'fastlivo_debug/pipeline_v19_summary.json');write(root/'pipeline_normal_stop.json',p)
        if kind=='wrong_summary_hash':
            p=json.loads((root/'pipeline_normal_stop.json').read_text());p['summary_sha256']='0'*64;write(root/'pipeline_normal_stop.json',p)
        if kind=='wrong_mapper_identity':
            p=json.loads((root/'pipeline_normal_stop.json').read_text());p['saved_identity']['start_ticks']=999;write(root/'pipeline_normal_stop.json',p)
        if kind=='wrong_run_binding':
            w=json.loads((root/'slam_loaded_binary.json').read_text());w['source_bound_run_dir']='/tmp/foreign_run';write(root/'slam_loaded_binary.json',w)
        if kind=='pending_owned':pending=True
        write(root/'guardian_release.json',release)
        raise StorageStop(copy.deepcopy(observation))
    def identity(pid):
        if pid==owner['parent']['pid']:return owner['parent']
        if pid==900004:return dict(pid=900004,start_ticks=4,pgid=900004,session=900004,boot_id=guardian.BOOT,state='R')
        return None  # Mapper is already gone; original request must not signal it.
    def validated(saved):return [role] if pending else []
    def stop(saved,**kwargs):
        nonlocal pending
        cleanup.append(saved['role']);pending=False
        return dict(sent_signals=[],remaining_members=[])
    with patch.object(guardian.os,'getpid',return_value=900004),patch.object(guardian,'ident',side_effect=identity), \
         patch.object(guardian,'validated_members',side_effect=validated),patch.object(guardian,'stop_group',side_effect=stop), \
         patch.object(guardian,'members',return_value=[]),patch.object(guardian.signal,'signal'), \
         patch.object(guardian.os,'kill',side_effect=lambda pid,sig:signals.append([pid,int(sig)])), \
         patch.object(guardian.time,'monotonic',side_effect=clock.now),patch.object(guardian.time,'sleep',side_effect=clock.sleep), \
         contextlib.redirect_stderr(io.StringIO()) as stderr:
        rc=guardian._watch(root,storage_check=storage)
    result=json.loads((root/'guardian_result.json').read_text())
    good=(kind=='normal_release' and rc==0 and result['status']=='parent_cleanup_verified' and not signals and not cleanup)
    if kind!='normal_release':
        good=(rc==1 and result['reason'].startswith('StorageStop:') and result['resource_observation']==observation
              and all(sig==int(signal.SIGTERM) and pid==owner['parent']['pid'] for pid,sig in signals))
        if kind in ('stop_then_release','evidence_collision'):
            good=good and result['status']=='parent_cleanup_verified_after_stop' and result['cleanup_verified'] is True
            good=good and result['pipeline']['existing_normal_stop_reused'] is True and not cleanup
            good=good and result['pipeline']['signal_sent'] is False
        else:
            good=good and result['status']=='recovered_failed' and result['cleanup_verified'] is False
            good=good and bool(result['parent_release_proof_error']) and cleanup==['navigation_stack']
            good=good and result['pipeline']['normal_completed'] is False and result['pipeline']['signal_sent'] is False
        if kind=='evidence_collision':
            good=good and json.loads((root/'guardian_stop_cause.json').read_text())==dict(existing_evidence=True)
            good=good and 'FileExistsError' in result['stop_evidence_error'] and 'write_failed' in stderr.getvalue()
        else:
            cause=json.loads((root/'guardian_stop_cause.json').read_text())
            good=good and cause['observation']==observation and cause['caught_monotonic_wall']==100.
    return good,result


def close_contract(root,result):
    root.mkdir();(root/'owned_roles').mkdir()
    obj=object.__new__(guardian.Guardian)
    obj.run=root;obj.parent=dict(pid=1,start_ticks=1,pgid=1,session=1,boot_id=guardian.BOOT)
    obj.children={};obj.saved=dict(pid=2);obj.error=None
    class Done:
        def set(self):pass
        def join(self,**kwargs):pass
    class Proc:
        returncode=1
        def wait(self,**kwargs):write(root/'guardian_result.json',result)
    obj.done=Done();obj.thread=Done();obj.proc=Proc();obj.log=io.StringIO()
    try:obj.close()
    except guardian.GuardianOperationalStop as error:
        return error.observation==result and 'StorageStop:' in str(error) and 'cleanup_verified=True' in str(error),str(error)
    return False,'No typed stop raised'


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    checks={};details={}
    with tempfile.TemporaryDirectory(prefix='r118_guardian_contract_') as temporary:
        parent=Path(temporary)
        cases=['normal_release','stop_then_release','stale_parent','stale_role','incomplete_summary',
               'wrong_summary_hash','wrong_mapper_identity','wrong_run_binding','pending_owned','evidence_collision']
        for kind in cases:
            root=parent/kind;root.mkdir();checks[kind],details[kind]=sequence(root,kind)
        checks['typed_close_keeps_resource_stop_fatal'],message=close_contract(parent/'close_contract',details['stop_then_release'])
        # Execute the real E formal runtime gate unchanged; it must reject the
        # typed resource error even while cleanup_verified is true.
        audit=Path(__file__).resolve().parents[2]/'acceptance/candidate/r118_formal_acceptance_audit.py'
        node=next(n for n in ast.walk(ast.parse(audit.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='runtime')
        def require(value,reason):
            if not value:raise ValueError(reason)
        ns=dict(require=require,outcome=dict(runtime_error=message),manifest=dict(error=message))
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(audit),'exec'),ns)
        try:ns['runtime']()
        except ValueError as error:checks['unchanged_E_formal_runtime_gate_rejects_stop']=str(error)=='Runtime error recorded'
        else:checks['unchanged_E_formal_runtime_gate_rejects_stop']=False
    report=dict(schema='r118_guardian_cleanup_contract_checks/v1',passed=all(checks.values()),checks=checks,
        details=details,typed_close_error=message,formal_runtime_gate_AST_unchanged=True,
        formal_audit_sha256=pipeline.sha(audit),
        formal_runtime_gate_AST_sha256=hashlib.sha256(ast.dump(node,include_attributes=False).encode()).hexdigest(),
        no_subprocess_or_ROS_or_model=True,storage_evaluation_or_CRC_invoked=False,
        source_sha256={str(Path(__file__).resolve().parent/name):pipeline.sha(Path(__file__).resolve().parent/name)
                       for name in ('owned_guardian.py','pipeline_lifecycle.py','r118_guardian_release_checks.py')})
    with a.output.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(passed=report['passed'],checks=checks,typed_close_error=message),indent=2))
    if not report['passed']:raise SystemExit(2)


if __name__=='__main__':main()
