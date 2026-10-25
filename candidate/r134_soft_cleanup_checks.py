"""Real D11 soft-stop values through Guardian wait/heartbeat/archive contracts."""
from pathlib import Path
import argparse
import copy
import datetime as dt
import io
import json
import types
from unittest.mock import patch

import owned_guardian as guardian
import pipeline_lifecycle as pipeline
import r118_guardian_release_checks as prior
import r122_native_reap_checks as native
import worker_archive_shutdown as archive

D11=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261024T093405Z_closed_loop_R85_R128_D11_closer_dynamic_trigger_round27_4555')
OBS=json.loads((D11/'guardian_stop_cause.json').read_text())['observation']
START=108596.857256319


def sequence(root,kind='soft'):
    owner,role,release,_=prior.fixture(root)
    roles={'navigation_stack':role}
    for name,pid in (('gazebo',900005),('worker',900006)):
        value=dict(role,pid=pid,pgid=pid,session=pid,start_ticks=pid,role=name)
        roles[name]=value;prior.write(root/'owned_roles'/(name+'.identity.json'),value)
    owner.update(allowed_roles=list(roles),wall_deadline=START+1000.)
    plan=json.loads((root/'runtime_plan.json').read_text())
    plan['profile']['r85_session']['deadline_utc']=(dt.datetime.now(dt.timezone.utc)+dt.timedelta(hours=1)).isoformat()
    prior.write(root/'runtime_plan.json',plan);owner['runtime_plan_sha256']=pipeline.sha(root/'runtime_plan.json')
    prior.write(root/'guardian_owner.json',owner);prior.write(root/'runner_heartbeat.json',dict(monotonic_wall=START))
    clock=types.SimpleNamespace(t=START)
    state=dict(first=True,recovered=False);signals=[];stops=[];beats=[];samples=[]
    observation=copy.deepcopy(OBS)
    if kind=='hard_initial':
        observation['stop_reasons'].append('fresh_round_hard_stop')
        observation['R85_round_budget'].update(charged_B=10000000001,hard_stop=True)
    atomic=guardian.atomic
    def record(path,value,**kwargs):
        if Path(path).name=='guardian_heartbeat.json':beats.append(copy.deepcopy(value))
        return atomic(path,value,**kwargs)
    def members(saved):
        elapsed=clock.t-START
        if state['recovered']:return []
        if kind=='foreign' and elapsed>=1. and saved['role']=='gazebo':raise RuntimeError('fixture foreign owned identity')
        if saved['role']=='navigation_stack':return []
        if saved['role']=='gazebo':return [saved] if elapsed<5. else []
        return [saved] if elapsed<8. else []
    def identity(pid):
        if pid==owner['parent']['pid']:return owner['parent']
        if pid==900004:return dict(pid=900004,start_ticks=4,pgid=900004,session=900004,boot_id=guardian.BOOT,state='R')
        return None
    def sleep(seconds):
        clock.t+=seconds
        prior.write(root/'runner_heartbeat.json',dict(monotonic_wall=clock.t))
        if clock.t-START>=8. and not (root/'guardian_release.json').exists():
            prior.write(root/'guardian_release.json',dict(parent=owner['parent'],monotonic_wall=clock.t))
    def storage(previous):
        current=copy.deepcopy(observation)
        if not state['first']:
            current['monotonic_wall']=clock.t
            if kind=='hard_later':
                current['stop_reasons'].append('fresh_round_hard_stop')
                current['R85_round_budget'].update(charged_B=10000000001,hard_stop=True)
            if kind=='FS_later':current['filesystems'][0]['available_bytes']=1;current['stop_reasons'].append('filesystem_reserve_/home')
            if kind=='callback_overdue':sleep(4.)
        state['first']=False;samples.append(copy.deepcopy(current))
        raise prior.StorageStop(current)
    def stop(saved,**kwargs):
        stops.append(saved['role']);state['recovered']=True
        return dict(sent_signals=[],remaining_members=[])
    with patch.object(guardian,'atomic',side_effect=record),patch.object(guardian.os,'getpid',return_value=900004), \
         patch.object(guardian,'ident',side_effect=identity),patch.object(guardian,'validated_members',side_effect=members), \
         patch.object(guardian,'stop_group',side_effect=stop),patch.object(guardian,'members',return_value=[]), \
         patch.object(guardian.signal,'signal'),patch.object(guardian.os,'kill',side_effect=lambda p,s:signals.append([p,int(s)])), \
         patch.object(guardian.time,'monotonic',side_effect=lambda:clock.t),patch.object(guardian.time,'sleep',side_effect=sleep):
        rc=guardian._watch(root,storage_check=storage)
    result=json.loads((root/'guardian_result.json').read_text())
    return dict(returncode=rc,result=result,heartbeats=beats,storage_samples=samples,role_stops=stops,signals=signals)


def monitor(root,beats,*,lost=False):
    # Replay actual heartbeats produced by _watch through the original monitor.
    obj=object.__new__(guardian.Guardian);obj.run=root;obj.parent=dict(pid=1);obj.error=None
    current=[START];sent=[];real_read=Path.read_text
    class Done:
        def wait(self,seconds):current[0]+=seconds;return current[0]>START+7.5
        def is_set(self):return False
    obj.done=Done();obj.proc=types.SimpleNamespace(poll=lambda:None)
    def reading(path,*args,**kwargs):
        if path==root/'guardian_heartbeat.json':
            limit=min(current[0],START+1.) if lost else current[0]
            value=next((row for row in reversed(beats) if row['monotonic_wall']<=limit),beats[0])
            return json.dumps(value)
        return real_read(path,*args,**kwargs)
    with patch.object(Path,'read_text',reading),patch.object(guardian.time,'monotonic',side_effect=lambda:current[0]), \
         patch.object(guardian.os,'kill',side_effect=lambda p,s:sent.append([p,int(s)])):
        obj._heartbeat()
    return obj.error,sent


def original_archive(monitor_error=None,*,unknown=False,stuck=False):
    world=native.World(gone=START+5.,waitable=START+5.)
    world.t=START+5.;world.worker_done=START+(50. if stuck else 8.)
    cleanup=dict(started=True,returncode=0,remaining_owned_group_members=[])
    callback=world.abort(dict(gazebo=cleanup),runner_deadline=START+100.)
    # Use the exact unchanged callback, including its genuine guardian error gate.
    callback.__globals__['guardian']=types.SimpleNamespace(error=monitor_error)
    identity=world.identity if not unknown else lambda pid:None
    worker=world.worker if not unknown else types.SimpleNamespace(pid=200,poll=lambda:None)
    result=archive.await_archive_close(worker,world.worker.saved,cleanup,identity,world.validated,
        abort_reason=callback,clock=world.now,sleep=world.advance,maximum_wait_s=30.,absolute_deadline=START+35.)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    checks={};details={}
    with prior.tempfile.TemporaryDirectory(prefix='r134_soft_cleanup_') as temporary:
        parent=Path(temporary)
        for kind in ('soft','hard_initial','hard_later','FS_later','foreign','callback_overdue'):
            root=parent/kind;root.mkdir();details[kind]=sequence(root,kind)
        soft=details['soft'];root=parent/'soft'
        error,sent=monitor(root,soft['heartbeats'])
        lost_error,lost_signals=monitor(root,soft['heartbeats'],lost=True)
        checks['D11_real_soft_values_enter_bounded_owner_cleanup']=(soft['returncode']==1 and
            soft['result']['status']=='parent_cleanup_verified_after_stop' and soft['result']['cleanup_verified'] is True and
            soft['result']['resource_observation']==OBS and soft['result']['soft_stop_owner_wait']['released'] is True)
        checks['observed_projection_and_rate_never_rewritten']=all(
            row['R85_round_budget']['predictive_rate_B_s']==OBS['R85_round_budget']['predictive_rate_B_s'] and
            row['R85_round_budget']['projected_full_stop_B']==10336310953 for row in soft['storage_samples'])
        checks['soft_wait_no_false_guardian_stale_through_parent_monitor']=error is None and not sent
        checks['guard_really_lost_still_stale_and_requests_stop']=lost_error=='Guardian heartbeat stale' and bool(lost_signals)
        checks['no_quiescent_or_returncode_success_invented']=all(row.get('archive_eligibility_granted') is False for row in
            soft['heartbeats'] if row['state']=='STOPPING_SOFT')
        details['parent_archive']=original_archive(error)
        checks['original_30s_archive_interface_can_observe_natural_worker_exit']=(
            details['parent_archive']['natural_worker_exit'] is True and details['parent_archive']['worker_returncode']==0 and
            details['parent_archive']['maximum_grace_s']==30. and details['parent_archive']['sent_signals']==[])
        details['lost_guard_archive']=original_archive(lost_error)
        checks['real_guard_error_still_aborts_original_archive']=details['lost_guard_archive']['reason'].startswith('grace_cancelled: owned guardian:')
        details['unknown_worker_archive']=original_archive(unknown=True)
        checks['unknown_worker_rc_does_not_become_archive_success']=not details['unknown_worker_archive']['natural_worker_exit'] and details['unknown_worker_archive']['worker_returncode'] is None
        details['stuck_worker_archive']=original_archive(stuck=True)
        checks['worker_not_exited_retains_original_30s_timeout']=details['stuck_worker_archive']['reason']=='archive_grace_expired' and not details['stuck_worker_archive']['natural_worker_exit']
        checks['initial_hard_stop_never_enters_soft_wait']=not any(row['state']=='STOPPING_SOFT' for row in details['hard_initial']['heartbeats']) and details['hard_initial']['returncode']==1
        for kind in ('hard_later','FS_later','foreign','callback_overdue'):
            d=details[kind];checks[kind+'_aborts_owner_wait_and_preserves_failure']=(d['returncode']==1 and
                d['result']['status']=='recovered_failed' and bool(d['role_stops']) and
                any(row['state']=='STOPPING_ABORT' for row in d['heartbeats']))
        checks['fresh_hard_and_FS_observation_values_retained']=all(
            json.loads((parent/kind/'r134_soft_stop_storage_last.json').read_text())==details[kind]['storage_samples'][-1]
            for kind in ('hard_later','FS_later'))
        checks['typed_close_preserves_resource_failure_after_cleanup'],message=prior.close_contract(parent/'close',soft['result'])
    report=dict(schema='r134_soft_stop_owner_cleanup_contract_checks/v1',passed=all(checks.values()),checks=checks,
        details=details,typed_close_error=message,real_D11_observation_source=str(D11/'guardian_stop_cause.json'),
        actual_Runtime_or_ROS_or_model_or_OS_signals_or_CRC=False,
        all_followup_samples_and_process_lifecycles_are_explicit_software_fixtures=True,navigation_PASS=False)
    with a.output.open('x')as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps(dict(passed=report['passed'],checks=checks),indent=2))
    if not report['passed']:raise SystemExit(2)


if __name__=='__main__':main()
