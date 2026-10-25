"""Request orderly shutdown from the exact owned, previously witnessed mapper.

The mapper retains its ROS context while stopping admission and draining accepted
packets. Emergency group cleanup stays in the runner and never becomes a PASS.
"""
from pathlib import Path
import hashlib
import json
import math
import os
import signal
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_summary(summary):
    errors=[]
    if summary.get('schema')!='staged_input_pipeline_v19/v1':errors.append('wrong_schema')
    for key in ('normal_completed','context_valid_at_drain'):
        if summary.get(key)is not True:errors.append(key)
    counts=[summary.get(key)for key in ('accepted','delivered','committed')]
    if any(type(x)is not int or x<0 for x in counts)or len(set(counts))!=1:
        errors.append('accepted_delivered_committed')
    for key in ('pending','inflight','ready','bytes','canceled','rejected_capacity','closed_rejections'):
        if type(summary.get(key))is not int or summary[key]!=0:errors.append(key)
    if summary.get('failure')!='':errors.append('failure')
    if summary.get('uncommitted_packets')!=[]:errors.append('uncommitted_packets')
    return errors


def reuse_normal_stop(run, witness, navigation_identity, release_wall):
    """Read a source-bound completed drain proof; never signal a departed PID."""
    run=Path(run).resolve()
    source=run/'pipeline_normal_stop.json'
    receipt=json.loads(source.read_text())
    target=run/'fastlivo_debug/pipeline_v19_summary.json'
    if not witness or witness.get('verified') is not True:
        raise RuntimeError('No verified owned SLAM binary witness for completed drain')
    saved=witness['mapping_identity']
    recorded=receipt.get('saved_identity',{})
    if any(type(saved.get(k)) is not int or type(recorded.get(k)) is not int or
           saved[k]!=recorded[k] for k in ('pid','start_ticks','pgid','session')):
        raise RuntimeError('Completed drain mapper identity differs')
    if type(navigation_identity.get('pgid')) is not int or saved['pgid']!=navigation_identity['pgid']:
        raise RuntimeError('Completed drain mapper is outside owned navigation group')
    launch=run/'navigation_stack_effective_environment.json'
    if (witness.get('source_bound_run_dir')!=str(run)
            or witness.get('launch_environment_receipt_file')!=str(launch)
            or witness.get('launch_environment_receipt_sha256')!=sha(launch)
            or json.loads(launch.read_text()).get('DEMO_RUN_DIR')!=str(run)):
        raise RuntimeError('Completed drain source-bound launch directory differs')
    times=[receipt.get(k) for k in ('started_monotonic_wall','sent_monotonic_wall','ended_monotonic_wall')]+[release_wall]
    if any(type(x) not in (int,float) or not math.isfinite(x) for x in times) or times!=sorted(times):
        raise RuntimeError('Completed drain/release ordering differs')
    if (receipt.get('schema')!='teacher_pipeline_v19_stop_request/v1' or receipt.get('run')!=str(run)
            or receipt.get('normal_completed') is not True or receipt.get('signal_sent') is not True
            or type(receipt.get('signal')) is not int or receipt['signal']!=int(signal.SIGUSR1)
            or receipt.get('error') is not None or receipt.get('gate_errors')!=[]):
        raise RuntimeError('Existing normal stop request is not a completed proof')
    summary=json.loads(target.read_text())
    if (receipt.get('summary_sha256')!=sha(target) or check_summary(summary)
            or json.dumps(receipt.get('summary'),sort_keys=True,allow_nan=False)!=json.dumps(summary,sort_keys=True,allow_nan=False)):
        raise RuntimeError('Completed drain summary hash/contract differs')
    return dict(schema='teacher_pipeline_v19_stop_request/v1',run=str(run),
        normal_completed=True,signal_sent=False,error=None,gate_errors=[],summary=summary,
        summary_sha256=sha(target),saved_identity=saved,existing_normal_stop_reused=True,
        original_receipt=dict(path=str(source),sha256=sha(source)),navigation_verified=False)


def request_normal_stop(run, witness, navigation_identity, identity_fn, timeout_s=15.,
                        receipt_name='pipeline_normal_stop.json'):
    """No process name search, no process-group signal, no PID reuse acceptance."""
    if receipt_name not in ('pipeline_normal_stop.json','pipeline_guardian_stop.json'):
        raise ValueError('Unexpected lifecycle receipt path')
    run=Path(run).resolve()
    target=run/'fastlivo_debug/pipeline_v19_summary.json'
    receipt={'schema':'teacher_pipeline_v19_stop_request/v1','run':str(run),
             'signal_sent':False,'normal_completed':False,
             'started_monotonic_wall':time.monotonic(),'error':None}
    try:
        if not witness or witness.get('verified')is not True:
            raise RuntimeError('No verified owned SLAM binary witness')
        saved=witness['mapping_identity'];current=identity_fn(saved['pid'])
        receipt['saved_identity']=saved
        if not current or any(current.get(k)!=saved.get(k)for k in ('pid','start_ticks','pgid')):
            raise RuntimeError('Mapper exited or process identity changed before orderly stop')
        if current['pgid']!=navigation_identity['pgid']:
            raise RuntimeError('Mapper is outside the owned navigation group')
        exe=Path('/proc')/str(saved['pid'])/'exe'
        if os.readlink(exe)!=witness['actual_executable_path']or sha(exe)!=witness['actual_executable_sha256']:
            raise RuntimeError('Owned mapper executable changed')
        launch=run/'navigation_stack_effective_environment.json'
        if (witness.get('source_bound_run_dir')!=str(run)
                or witness.get('launch_environment_receipt_file')!=str(launch)
                or witness.get('launch_environment_receipt_sha256')!=sha(launch)
                or json.loads(launch.read_text()).get('DEMO_RUN_DIR')!=str(run)):
            raise RuntimeError('Mapper source-bound launch run directory differs from owned run')
        receipt.update(run_directory_proof='frozen actual owned launch environment and source-bound mapper witness',
                       process_environment_directly_read=False)
        if target.exists():raise RuntimeError('Unexpected pre-existing pipeline shutdown summary')
        os.kill(saved['pid'],signal.SIGUSR1)
        receipt.update(signal_sent=True,signal=int(signal.SIGUSR1),sent_monotonic_wall=time.monotonic())
        deadline=time.monotonic()+timeout_s
        while time.monotonic()<deadline:
            current=identity_fn(saved['pid'])
            if current and any(current.get(k)!=saved.get(k)for k in ('pid','start_ticks','pgid')):
                raise RuntimeError('Mapper identity changed while waiting for drain')
            if not current or current.get('state')=='Z':break
            time.sleep(.02)
        else:raise TimeoutError('Owned mapper did not finish the normal drain within15s')
        summary=json.loads(target.read_text())
        errors=check_summary(summary)
        receipt.update(summary_sha256=sha(target),summary=summary,gate_errors=errors)
        if errors:raise RuntimeError('Normal pipeline lifecycle failed: '+','.join(errors))
        receipt['normal_completed']=True
    except Exception as error:
        receipt['error']=type(error).__name__+': '+str(error)
    receipt['ended_monotonic_wall']=time.monotonic()
    with(run/receipt_name).open('x')as stream:
        json.dump(receipt,stream,indent=2);stream.write('\n')
    return receipt
