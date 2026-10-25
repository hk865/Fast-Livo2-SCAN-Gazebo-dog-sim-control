#!/usr/bin/env python3
"""CPU Teacher: actual SLAM/SCAN velocity only; causal known-command ack.

Native truth supplies Actor fields and offline diagnostics. Its position and
attitude never enter navigation, route/arrival or the command acknowledgment.
"""
import argparse,collections,hashlib,importlib.util,json,sys,time
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True

def canonical(x):return json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def atomic(p,x):
    tmp=p.with_suffix('.worker.tmp');tmp.write_text(canonical(x)+'\n');tmp.replace(p)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--socket',required=True);a=ap.parse_args()
    run=a.run.resolve();scope_path=run/'navigation_scope.json';scope=json.loads(scope_path.read_text());scope_sha=sha(scope_path)
    if (scope.get('schema')!='teacher_closed_loop_navigation_scope/v1' or scope.get('allowed') is not True
        or scope.get('navigation_ground_truth_used') is not False or scope.get('controller_kind')!='teacher'
        or scope.get('checkpoint_sha256')!='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'):
        raise RuntimeError('Invalid closed-loop executor scope')
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
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
    for name,h in scope['references'].items():
        if sha(name)!=h:raise RuntimeError('Frozen source mismatch '+name)
        st=Path(name).stat();fingerprints[name]=(st.st_size,st.st_mtime_ns)
    for name in [str(scope_path)]:
        st=Path(name).stat();fingerprints[name]=(st.st_size,st.st_mtime_ns)
    sys.path.insert(0,str(run/'sources/policy'))
    spec=importlib.util.spec_from_file_location('frozen_cpu_closed_loop_teacher',run/'sources/policy/worker.py')
    legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
    from policy_selection import record_actual_loaded
    record_actual_loaded(run,legacy)
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
        with timer.measure('observation'):return observation_builder(*args,**kwargs)
    legacy.build_observation=timed_observation
    actor_loader=legacy.load_actor
    def timed_actor_loader():
        actor=actor_loader()
        class Actor:
            def __call__(self,*args,**kwargs):
                with timer.measure('inference'):return actor(*args,**kwargs)
        return Actor()
    legacy.load_actor=timed_actor_loader
    legacy.socket=TimedSocketModule(legacy.socket,timer)
    original_recv,original_atomic=legacy.recv_exact,legacy.atomic_json
    latest={'clock_ns':None,'pending_ack':None,'metadata':None,'evidence':None,'completion_elapsed_s':None}
    from executor_command import CommandReader
    reader=CommandReader(run.name,scope_sha,limits)
    legacy.ordinary_zero_input_adapter=reader.before_observation
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
                acks.append(row);ack_log.write(canonical(row)+'\n')
                with timer.measure('causal_ack_publish'):
                    atomic(run/'closed_loop_executor_ack.json',{'schema':'teacher_known_velocity_ack/v1','entries':list(acks)})
                latest['pending_ack']=None
        return data

    def read_command():
        started=time.monotonic()
        try:
            with timer.measure('command_file_read'):raw=(run/'navigation_command.json').read_bytes()
            command,rejected,record=reader.read(raw,latest['clock_ns'],time.monotonic())
            record['read_monotonic_wall']=started
            return command,rejected,record
        except OSError as error:
            return reader.reject_frame(latest['clock_ns'],type(error).__name__+': '+str(error))

    def request(test,t):
        changed=[]
        for name,fp in fingerprints.items():
            try:
                st=Path(name).stat()
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
                obj.update(command_expired=e.get('rejected',True),command_reason=e.get('reason','no input'),
                    command_source='actual SLAM/IMU cascade with original SCAN path',closed_loop_read_evidence=e,
                    outer_navigation_ground_truth_used=False,privileged_actor_observations=True)
                if provider is not None:obj['actor_terrain_provider']=provider.evidence()
                latest['pending_ack']={'stamp_ns':round(obj['world_sim_time']*1e9),
                    'applied_command_body':obj['command'],'requested_command_body':obj['requested']}
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
            latest['metadata']=obj
            if provider is not None:obj['actor_terrain_layer_switch']=provider.contract()
        if path.name=='worker_result.json':obj.update(controller_hold_observed_worker_elapsed_s=latest['completion_elapsed_s'],
            navigation_ground_truth_used=False,actor_observations_remain_privileged=True,last_closed_loop_read=latest['evidence'])
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
        log.close();ack_log.close()
        if provider is not None:provider.close()
        timer.finish('worker_exit_without_response')
        recording_result=recording.close()
        result=dict(schema='teacher_ipc_evidence_runtime/v1',
            native_watchdog_ms=200,source_deadline_ms=300,native_parking_verified=False,
            actor_or_navigation_algorithm_changed=False,
            synchronous_native_telemetry_and_state_preserved=True,
            synchronous_causal_ack_snapshot_preserved=True,
            optional_history_only=True,optional_total_file_byte_limit=68*1024*1024,
            native_telemetry_file_byte_limit=None,
            complete_original_command_audit=recording_result['original_commands']['capture_complete'],
            timing=timing_writer.close(),command_history=command_writer.close(),ack_history=ack_writer.close(),
            actual_original_commands=recording_result['original_commands'],height_sources=recording_result['height_sources'])
        atomic(run/'ipc_evidence_result.json',result)

if __name__=='__main__':main()
