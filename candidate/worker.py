#!/usr/bin/env python3
"""Explicit Teacher/Student CPU modes with actual SLAM/SCAN source and causal ACK.

Native proxies supply Actor fields; Student adds explicitly privileged COM velocity
PID correction only. Native position never owns navigation/route/arrival.
"""
import r118_source_age_probe as _r118_source_age
import argparse,collections,hashlib,importlib.util,json,os,sys,time
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True

def canonical(x):return json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def atomic(p,x):
    tmp=p.with_suffix('.worker.tmp');tmp.write_text(canonical(x)+'\n');tmp.replace(p)

def atomic_ack_rows(p, encoded_rows):
    # Each entry is the completed bind_causal_ack canonical snapshot already
    # given to ACK history. Strings detach all mutable pending/telemetry aliases.
    content='{"entries":['+',\n'.join(encoded_rows)+'],"schema":"teacher_known_velocity_ack/v1"}\n'
    tmp=p.with_suffix('.worker.tmp');tmp.write_text(content);tmp.replace(p)

def r85_task_completion(run_id,profile,task):
    """Typed terminal owner result only; status=succeeded is not fixed5 PASS."""
    if profile.get('r85_session',{}).get('enabled')is not True:return None
    kind=profile['r85_task']['kind'];rid=run_id+':r85:'+kind+':1'
    if (not isinstance(task,dict) or task.get('schema')!='r85_pct_scan_student_task_result/v1'
        or task.get('run_id')!=run_id or task.get('request_id')!=rid or task.get('kind')!=kind
        or task.get('navigation_ground_truth_used')is not False):
        raise ValueError('R85 terminal task result identity/source differs')
    state=task.get('state')
    if state not in ('succeeded','failed'):return None
    if state=='succeeded':
        if task.get('parking_pass')is not True:raise ValueError('R85 success lacks actual first-ACK fixed5 parking')
        if task.get('dynamic_required')is True and task.get('dynamic_pass')is not True:
            raise ValueError('R85 success lacks required actual dynamic verification')
        if kind=='business_chain_prefix':
            if task.get('short_chain_validation_only')is not True or task.get('full_F3_PASS')is not False:
                raise ValueError('R85 short chain was promoted to full F3 success')
        elif kind=='f1_to_f3':
            if (task.get('full_F3_PASS')is not True or task.get('short_chain_validation_only')is not False
                or task.get('dynamic_required')is not True or task.get('dynamic_pass')is not True):
                raise ValueError('R85 full task lacks actual full/dynamic result')
        else:raise ValueError('Unknown R85 task boundary')
    return dict(schema=task['schema'],run_id=run_id,request_id=rid,kind=kind,state=state,
        parking_pass=task.get('parking_pass')is True,full_F3_PASS=task.get('full_F3_PASS')is True,
        short_chain_validation_only=task.get('short_chain_validation_only')is True,
        navigation_ground_truth_used=False,owner_result_only_not_a_new_acceptance_gate=True)

def initialize_command_reader(run,scope,scope_sha,limits,profile):
    """One startup proof of the actual worker and constructed reader sources."""
    from pathlib import Path as SourcePath
    import json as source_json
    from executor_command import CommandReader
    reader=CommandReader(run.name,scope_sha,limits,profile=profile)
    run=SourcePath(run).resolve()
    scope_path=run/'navigation_scope.json'
    snapshot_path=run/'navigation_source_snapshots.json';manifest_path=run/'source_manifest.json'
    plan_path=run/'runtime_plan.json'
    snapshots=source_json.loads(snapshot_path.read_text());manifest=source_json.loads(manifest_path.read_text())
    plan=source_json.loads(plan_path.read_text())
    if (scope.get('run_dir')!=str(run) or sha(scope_path)!=scope_sha
        or source_json.loads(scope_path.read_text())!=scope or plan.get('run')!=str(run)
        or plan.get('scope_sha256')!=scope_sha or plan.get('source_manifest_sha256')!=sha(manifest_path)):
        raise RuntimeError('Actual executor startup scope/runtime plan differs')
    references=scope['references']
    if any(references.get(str(path))!=sha(path) for path in (snapshot_path,manifest_path)):
        raise RuntimeError('Actual executor snapshot maps not frozen by scope')
    entry_module=sys.modules['__main__'];entry=SourcePath(entry_module.__file__).resolve()
    argv=plan['runtime_commands']['worker']
    planned=[SourcePath(value).resolve() for value in argv if isinstance(value,str) and value.endswith('.py')]
    if (len(planned)!=1 or planned[0]!=entry or SourcePath(sys.argv[0]).resolve()!=entry
        or '--run' not in argv or argv[argv.index('--run')+1]!=str(run)):
        raise RuntimeError('Actual worker entry differs from runtime plan')
    worker_module=sys.modules[main.__module__];reader_class=type(reader)
    reader_module=sys.modules[reader_class.__module__]
    actual=dict(entry=entry,worker=SourcePath(worker_module.__file__).resolve(),
        executor_command=SourcePath(reader_module.__file__).resolve())
    if (main.__globals__ is not vars(worker_module) or worker_module.main is not main
        or SourcePath(main.__code__.co_filename).resolve()!=actual['worker']
        or actual['worker']!=entry.parent/'worker.py' or actual['worker']!=SourcePath(__file__).resolve()
        or reader_class is not CommandReader or reader_module.CommandReader is not reader_class
        or reader_class.__init__.__globals__ is not vars(reader_module)
        or any(SourcePath(method.__code__.co_filename).resolve()!=actual['executor_command']
            for method in (reader_class.__init__,reader_class.read,reader_class.before_observation))
        or actual['executor_command']!=entry.parent/'executor_command.py'):
        raise RuntimeError('Actual worker/CommandReader function or module path differs')
    modules={}
    for role,path in actual.items():
        origin=str(path);mapping=snapshots[origin];snapshot=SourcePath(mapping['snapshot']).resolve()
        if not snapshot.is_relative_to(run/'sources') or snapshot.name!=path.name:
            raise RuntimeError('Actual executor source snapshot outside run archive')
        digest=sha(path);relative=str(snapshot.relative_to(run/'sources'))
        if (references.get(origin)!=digest or references.get(str(snapshot))!=digest
            or mapping.get('sha256')!=digest or manifest.get(origin)!=digest
            or manifest.get(relative)!=digest or sha(snapshot)!=digest):
            raise RuntimeError('Actual executor loaded source/archive hash differs: '+role)
        modules[role]=dict(actual_module_file=origin,actual_sha256=digest,
            snapshot=str(snapshot),snapshot_sha256=digest,manifest_key=relative)
    age_binding=_r118_source_age.bind('reader',run)
    receipt=dict(r118_source_age_binding=age_binding,schema='actual_worker_executor_sources_loaded/v1',run_id=run.name,
        pid=os.getpid(),ppid=os.getppid(),observed_monotonic_wall_ns=time.monotonic_ns(),
        entry_argv=list(sys.argv),python_executable=sys.executable,scope_sha256=scope_sha,
        runtime_plan_sha256=sha(plan_path),source_manifest_sha256=sha(manifest_path),
        navigation_source_snapshots_sha256=sha(snapshot_path),modules=modules,
        actual_worker_function=dict(module=main.__module__,qualname=main.__qualname__,
            code_filename=main.__code__.co_filename),actual_reader_class=dict(
            module=reader_class.__module__,qualname=reader_class.__qualname__),
        actual_reader_constructed=True,before_legacy_main=True,
        Actor_model_or_physical_PASS_claimed=False)
    # Exclusive creation makes this a once-only startup receipt, never a frame probe.
    with (run/'worker_executor_sources_loaded.json').open('x') as stream:
        stream.write(source_json.dumps(receipt,sort_keys=True,allow_nan=False)+'\n')
    return reader

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--socket',required=True);a=ap.parse_args()
    run=a.run.resolve();scope_path=run/'navigation_scope.json';scope=json.loads(scope_path.read_text());scope_sha=sha(scope_path)
    actor_kind=scope.get('actor_kind','teacher')
    teacher_sha='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
    student_checkpoint='7b59b8c7eafc9ffbc2dc32aaa2df3dadec3269d039338e4f93217bc73af66fd9'
    student_export='d8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185'
    if actor_kind not in ('teacher','student'):raise RuntimeError('Unknown actual Actor kind')
    if (scope.get('allowed') is not True or scope.get('navigation_ground_truth_used') is not False
        or scope.get('schema')!=actor_kind+'_closed_loop_navigation_scope/v1'
        or scope.get('controller_kind')!=actor_kind
        or scope.get('checkpoint_sha256')!=(teacher_sha if actor_kind=='teacher' else student_checkpoint)):
        raise RuntimeError('Invalid explicit closed-loop executor scope')
    if actor_kind=='student' and (scope.get('export_sha256')!=student_export
        or scope['profile'].get('actor_kind')!='student' or scope['profile'].get('controller_kind')!='student'):
        raise RuntimeError('Student export/profile identity differs')
    sys.path.append('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/workspace/multifloor_demo/teacher_mode/navigation')
    from evidence_runtime_v32.worker_logging import RecordingContext
    from ipc_evidence import BoundedHistory, HistorySink, TimedSink, FrameTiming, TimedSocketModule
    # Optional payload budget: raw command8MiB + height8MiB + histories20MiB.
    # Live telemetry/state/ACK are separate, mandatory consumers and stay sync.
    recording=RecordingContext(run,maximum_command_bytes=8*1024*1024,maximum_height_bytes=8*1024*1024)
    timing_writer=BoundedHistory(run/'ipc_frame_timing.jsonl',maximum_file_bytes=40*1024*1024)
    command_writer=BoundedHistory(run/'closed_loop_execution_commands.jsonl',maximum_file_bytes=8*1024*1024)
    ack_writer=BoundedHistory(run/'closed_loop_executor_ack.jsonl',maximum_file_bytes=4*1024*1024)
    timer=FrameTiming(timing_writer)
    p=scope['profile'];limits=np.asarray(p['cascade']['command_limits']);fingerprints={}
    from student_lower_loop_ack import configuration as lower_ack_configuration, annotate_pending, bind_causal_ack
    lower_ack_config=lower_ack_configuration(p['cascade'].get('r85_student_lower_loop_ack'))
    if lower_ack_config is not None and actor_kind!='student':
        raise ValueError('R85 Student lower-loop ACK contract requires actual Student actor')
    for name,h in scope['references'].items():
        if sha(name)!=h:raise RuntimeError('Frozen source mismatch '+name)
        st=Path(name).stat();fingerprints[name]=(st.st_size,st.st_mtime_ns)
    for name in [str(scope_path)]:
        st=Path(name).stat();fingerprints[name]=(st.st_size,st.st_mtime_ns)
    sys.path.insert(0,str(run/'sources/policy'))
    spec=importlib.util.spec_from_file_location('frozen_cpu_closed_loop_teacher',run/'sources/policy/worker.py')
    legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
    from policy_selection import record_actual_loaded
    loaded_sources=record_actual_loaded(run,legacy)
    student=None
    if actor_kind=='student':
        binding=loaded_sources['selected_binding']
        adapter_path=Path(binding['student_adapter_snapshot'])
        spec=importlib.util.spec_from_file_location('frozen_student_actor_adapter',adapter_path)
        adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
        student=adapter.StudentRuntime(run,scope,binding,limits)
    provider=None
    if p.get('terrain_layer_switch') is not None or p.get('mission46_required'):
        if p.get('mission46_required'):
            from mission46_terrain import MissionTerrainProvider as SwitchingTerrainProvider
        else:
            from terrain_provider import SwitchingTerrainProvider
        original_map=legacy.TerrainHeightMap
        observation=sys.modules[legacy.build_observation.__module__]
        try:provider=SwitchingTerrainProvider(run,scope,original_map,observation)
        except Exception as error:
            with (run/'terrain_provider_initialization_failure.json').open('x') as out:
                out.write(canonical({'schema':'teacher_actor_terrain_switch/v1','status':'failed',
                    'stage':'initialization','error':type(error).__name__+': '+str(error),
                    'navigation_ground_truth_used':False})+'\n')
            raise
        class ActorTerrainFactory:
            @staticmethod
            def from_sdf(path,include_models=None,excluded_models=('go2','teacher_go2','moving_obstacle')):
                if include_models is None:
                    # Native clearance/contact safety is independent of Actor
                    # terrain selection and retains all static collisions.
                    return original_map.from_sdf(path,excluded_models=excluded_models)
                if (Path(path).resolve()!=provider.world or list(include_models)!=provider.initial_models or
                    tuple(excluded_models)!=('go2','teacher_go2','moving_obstacle')):
                    raise ValueError('Unexpected Actor terrain constructor; no geometry fallback allowed')
                return provider
        legacy.TerrainHeightMap=ActorTerrainFactory
        original_build=legacy.build_observation
        def build_observation(*args,**kwargs):
            obs,extra=original_build(*args,**kwargs)
            extra.update(actor_terrain_provider=provider.evidence())
            return obs,extra
        legacy.build_observation=build_observation
    recording.install_height_path(legacy)
    evidence_path=legacy.Path
    class TimingPath(evidence_path):
        def open(self,*args,**kwargs):
            mode=args[0] if args else kwargs.get('mode','r')
            stream=super().open(*args,**kwargs)
            if self.name=='telemetry.jsonl' and mode=='w' and self.parent.resolve()==run:
                return TimedSink(stream,timer,'telemetry_write')
            return stream
    legacy.Path=TimingPath
    observation_builder=legacy.build_observation
    def timed_observation(*args,**kwargs):
        with timer.measure('observation'):
            obs,extra=observation_builder(*args,**kwargs)
            if student is not None:
                student.observation_frame(args[0],args[1],args[2],obs,kwargs.get('noisy',False))
            return obs,extra
    legacy.build_observation=timed_observation
    actor_loader=legacy.load_actor
    def timed_actor_loader():
        actor=actor_loader() if student is None else student.load_actor(legacy.torch)
        class Actor:
            def __call__(self,*args,**kwargs):
                with timer.measure('inference'):return actor(*args,**kwargs)
        return Actor()
    legacy.load_actor=timed_actor_loader
    legacy.socket=TimedSocketModule(legacy.socket,timer)
    original_recv,original_atomic=legacy.recv_exact,legacy.atomic_json
    latest={'clock_ns':None,'pending_ack':None,'metadata':None,'evidence':None,'completion_elapsed_s':None}
    reader=initialize_command_reader(run,scope,scope_sha,limits,p)
    legacy.ordinary_zero_input_adapter=reader.before_observation
    if student is not None:
        original_prepare_velocity_input=legacy.prepare_velocity_input
        def student_prepare_velocity_input(cmd,request,input_clock_ns,ordinary_zero_adapter=None):
            return student.prepare_command(cmd,request,input_clock_ns,ordinary_zero_adapter,
                latest['evidence'],run.name,original_prepare_velocity_input)
        legacy.prepare_velocity_input=student_prepare_velocity_input
    acks=collections.deque(maxlen=32);ack_seq=0
    log=HistorySink(command_writer,timer,'command_history_enqueue')
    ack_log=HistorySink(ack_writer,timer,'ack_history_enqueue')

    def receive(conn,n):
        nonlocal ack_seq
        if n==512:timer.begin_receive()
        try:
            with timer.measure('state_read'):data=original_recv(conn,n)
        except BaseException as error:
            timer.finish('state_read_failed',error)
            raise
        if n==512:
            native_state=np.frombuffer(data,dtype='<f8')
            clock=round(float(native_state[0])*1e9)
            if student is not None:student.native_frame(native_state)
            timer.received(clock)
            if provider is not None:
                with timer.measure('terrain_native_observation'):provider.observe_state(native_state)
            latest['clock_ns']=clock;pending=latest['pending_ack']
            if pending is not None and clock>pending['stamp_ns']:
                # Receiving the next actuator request proves the preceding
                # policy response completed. Only known commands are exposed.
                ack_seq+=1
                row={**pending,'sequence':ack_seq,'ack_native_clock_ns':clock,
                    'received_wall_ns':time.monotonic_ns(),'navigation_ground_truth_used':False,
                    'source':'previous actual CPU Actor velocity input acknowledged by next native request',
                    'contains_position_or_attitude':False}
                if lower_ack_config is not None:bind_causal_ack(row,lower_ack_config)
                encoded_ack=canonical(row)
                acks.append(encoded_ack);ack_log.write(encoded_ack+'\n')
                with timer.measure('causal_ack_publish'):
                    atomic_ack_rows(run/'closed_loop_executor_ack.json',acks)
                latest['pending_ack']=None
        return data

    def read_command():
        started=time.monotonic()
        try:
            with timer.measure('command_file_read'):raw=(run/'navigation_command.json').read_bytes()
            command,rejected,record=reader.read(raw,latest['clock_ns'],time.monotonic())
            record['reader_validation_sample_monotonic_wall']=record['read_monotonic_wall']
            record['read_monotonic_wall']=started
            return command,rejected,record
        except OSError as error:
            return reader.reject_frame(latest['clock_ns'],type(error).__name__+': '+str(error))

    def request(test,t):
        changed=[]
        with timer.measure('frozen_source_stat'):
            for name,fp in fingerprints.items():
                try:
                    st=os.stat(name)
                    if (st.st_size,st.st_mtime_ns)!=fp:changed.append(name)
                except OSError:changed.append(name)
        if changed:
            cmd,rejected,attempt=reader.reject_frame(latest['clock_ns'],'Frozen source changed')
            attempt['paths']=changed
        else:
            with timer.measure('command_read_validate'):cmd,rejected,attempt=read_command()
        timer.command_binding(attempt)
        if provider is not None and not changed:
            if provider.check_switch():
                cmd=np.zeros(3);rejected=True;reader.revoke()
                attempt.update(execution_override='actor_terrain_switch_protection',
                    reason='Actor terrain switch refused: '+provider.failure if provider.failure else 'Waiting for causal original connector arrival clock')
        evidence={'sim_time':float(t),'physics_clock_ns':latest['clock_ns'],'command':cmd.tolist(),
            'rejected':rejected,'reason':attempt['reason'],'actual_read_attempt':attempt,
            'actual_original_envelope':attempt.get('decoded_envelope'),'navigation_ground_truth_used':False}
        if provider is not None:evidence['actor_terrain_provider']=provider.evidence()
        latest['evidence']=evidence;log.write(canonical(recording.project_evidence(evidence))+'\n')
        # Completion/hold is declared by actual SLAM/IMU source, never native
        # position. First declaration is immutable and cannot shift later.
        try:
            status=json.loads((run/'navigation_status.json').read_text());park=status.get('cascade_parking') or {}
            if p.get('mission46_required'):
                mission=json.loads((run/'mission46_status.json').read_text())
                complete=(mission.get('run_id')==run.name and mission.get('navigation_ground_truth_used') is False
                    and (mission.get('stage')=='failed' or
                         mission.get('stage')=='completed' and mission.get('functional_sequence_completed') is True))
                if complete and latest['completion_elapsed_s'] is None:
                    latest['completion_elapsed_s']=float(t)
            elif p.get('r85_session',{}).get('enabled')is True:
                task=json.loads((run/'r85_task_result.json').read_text())
                terminal=r85_task_completion(run.name,p,task)
                if terminal is not None and latest['completion_elapsed_s']is None:
                    latest['completion_elapsed_s']=float(t)
                    latest['r85_completion_observation']=terminal
            else:
                terrain_ready=provider is None or (provider.switched and provider.failure is None)
                if terrain_ready and status.get('state')=='succeeded' and park.get('mode')=='active_hold' and latest['completion_elapsed_s'] is None:
                    latest['completion_elapsed_s']=float(t)
            if latest['completion_elapsed_s'] is not None and latest['metadata'] is not None:
                latest['metadata']['test_duration_s']=min(float(p['duration_s']),latest['completion_elapsed_s']+7.)
        except (OSError,ValueError,TypeError):pass
        return cmd,'initializing' if t<.1 else 'tracking' if np.any(cmd) else 'stop_transition'

    original_request=request
    def timed_request(test,t):
        with timer.measure('command_request_and_status'):return original_request(test,t)

    class IsolatedJSON:
        def __getattr__(self,name):return getattr(json,name)
        def dumps(self,obj,**kwargs):
            if isinstance(obj,dict) and 'world_sim_time' in obj and 'command_expired' in obj:
                obj=dict(obj);e=latest['evidence'] or {}
                if student is not None:obj=student.telemetry(obj)
                else:obj['actor_kind']='teacher'
                obj.update(command_expired=e.get('rejected',True),command_reason=e.get('reason','no input'),
                    command_source=('actual SLAM/IMU fixed-goal terminal reference' if
                        ((e.get('actual_original_envelope') or {}).get('controller_source') or {}).get('reference_kind')=='terminal_fixed_goal/v1'
                        else 'actual SLAM/IMU cascade with original SCAN path'),closed_loop_read_evidence=e,
                    outer_navigation_ground_truth_used=False,privileged_actor_observations=True)
                if provider is not None:obj['actor_terrain_provider']=provider.evidence()
                latest['pending_ack']={'stamp_ns':round(obj['world_sim_time']*1e9),
                    'applied_command_body':obj['command'],'requested_command_body':obj['requested'],
                    'actor_kind':actor_kind,'source_scope_sha256':scope_sha,
                    'actual_checkpoint_sha256':scope['checkpoint_sha256']}
                if student is not None:
                    latest['pending_ack'].update(actual_actor_input=dict(student.input_receipt),
                        actual_export_sha256=student_export,student_contract_sha256=sha(student.contract_path))
                    if lower_ack_config is not None:
                        annotate_pending(latest['pending_ack'],obj['student_pid'],lower_ack_config,limits)
                zero=obj.get('ordinary_zero_before_actor_observation')
                if (zero is not None and zero.get('observation_command_exact_zero')is True
                    and zero.get('actor_inferred_this_frame')is True and not obj.get('fault')
                    and obj['command']==[0.,0.,0.] and obj['requested']==[0.,0.,0.]):
                    if zero['input_clock_ns']!=latest['pending_ack']['stamp_ns']:
                        raise RuntimeError('Ordinary zero ACK input clock mismatch')
                    latest['pending_ack']['ordinary_zero_before_actor_observation']=dict(zero)
                obj=recording.project_telemetry(obj)
            elif isinstance(obj,dict) and 'height_scan_raw' in obj:
                obj=recording.project_height(obj)
            with timer.measure('json_serialization'):
                return json.dumps(obj,**kwargs)

    def metadata(path,obj):
        if path.name=='policy_manifest.json':
            obj.update(test='closed_loop_SLAM_SCAN_cascade',command_source='actual SLAM/IMU cascade with SCAN guarded path',
                high_level_source_is_actual_SLAM=True,navigation_truth_used=False,actor_observations_remain_privileged=True,
                privileged_actor_observation_dimensions=232,test_duration_s=float(p['duration_s']),
                source_timeout_s=.3,source_scope_sha256=scope_sha,legacy_motion_evaluation_applicable=False,
                command_start_seconds=None,command_end_s=None,acceptance_is_global=False)
            obj['actor_kind']=actor_kind
            if student is not None:obj.update(student.metadata())
            latest['metadata']=obj
            if provider is not None:obj['actor_terrain_layer_switch']=provider.contract()
        if path.name=='worker_result.json':obj.update(controller_hold_observed_worker_elapsed_s=latest['completion_elapsed_s'],
            navigation_ground_truth_used=False,actor_observations_remain_privileged=True,last_closed_loop_read=latest['evidence'])
        if path.name=='worker_result.json':
            obj.update(actor_kind=actor_kind,r85_task_completion_observation=latest.get('r85_completion_observation'))
            if student is not None:
                obj.update(student_export_sha256=student_export,student_actual_inference_count=student.inference_count,
                    student_pid_new_reset_count=student.pid.new_reset_count,
                    student_pid_old_would_reset_count=student.pid.old_would_reset_count)
        if path.name=='worker_result.json' and provider is not None:
            obj.update(actor_terrain_provider=provider.evidence(),actor_terrain_switch_failed=provider.failure is not None)
        with timer.measure('state_publish' if path.name=='state.json' else 'metadata_publish'):
            original_atomic(path,obj)

    legacy.recv_exact=receive;legacy.requested=timed_request;legacy.atomic_json=metadata;legacy.json=IsolatedJSON()
    legacy.duration=lambda _:float(p['duration_s'])
    sys.argv=[str(run/'sources/policy/worker.py'),'--run',str(run),'--test','forward','--socket',a.socket,
        '--terrain-target-manifest',str(run/'terrain_target_manifest.json')]
    try:legacy.main()
    finally:
        _r118_source_age.finish(run)
        if student is not None:student.close()
        log.close();ack_log.close()
        if provider is not None:provider.close()
        timer.finish('worker_exit_without_response')
        recording_result=recording.close()
        result=dict(schema='teacher_ipc_evidence_runtime/v1',
            native_watchdog_ms=200,source_deadline_ms=300,native_parking_verified=False,
            actor_or_navigation_algorithm_changed=student is not None,actor_kind=actor_kind,
            navigation_algorithm_changed=False,
            synchronous_native_telemetry_and_state_preserved=True,
            synchronous_causal_ack_snapshot_preserved=True,
            optional_history_only=True,optional_total_file_byte_limit=68*1024*1024,
            native_telemetry_file_byte_limit=None,
            complete_original_command_audit=recording_result['original_commands']['capture_complete'],
            timing=timing_writer.close(),command_history=command_writer.close(),ack_history=ack_writer.close(),
            actual_original_commands=recording_result['original_commands'],height_sources=recording_result['height_sources'])
        atomic(run/'ipc_evidence_result.json',result)

if __name__=='__main__':main()
