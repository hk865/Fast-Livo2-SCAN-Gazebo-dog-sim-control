"""Request orderly shutdown from the exact owned, previously witnessed mapper.

The mapper retains its ROS context while stopping admission and draining accepted
packets. Emergency group cleanup stays in the runner and never becomes a PASS.
"""
from pathlib import Path
import hashlib
import json
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
