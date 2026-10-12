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
    p=scope['profile'];limits=np.asarray(p['cascade']['command_limits']);fingerprints={}
    for name,h in scope['references'].items():
        if sha(name)!=h:raise RuntimeError('Frozen source mismatch '+name)
        st=Path(name).stat();fingerprints[name]=(st.st_size,st.st_mtime_ns)
    for name in [str(scope_path)]:
        st=Path(name).stat();fingerprints[name]=(st.st_size,st.st_mtime_ns)
    sys.path.insert(0,str(run/'sources/policy'))
    spec=importlib.util.spec_from_file_location('frozen_cpu_closed_loop_teacher',run/'sources/policy/worker.py')
    legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
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
    original_recv,original_atomic=legacy.recv_exact,legacy.atomic_json
    latest={'clock_ns':None,'pending_ack':None,'metadata':None,'evidence':None,'completion_elapsed_s':None}
    acks=collections.deque(maxlen=32);last_seq=0;last_hash=None;ack_seq=0
    log=(run/'closed_loop_execution_commands.jsonl').open('w',buffering=1)
    ack_log=(run/'closed_loop_executor_ack.jsonl').open('w',buffering=1)

    def receive(conn,n):
        nonlocal ack_seq
        data=original_recv(conn,n)
        if n==512:
            native_state=np.frombuffer(data,dtype='<f8')
            if provider is not None:provider.observe_state(native_state)
            clock=round(float(native_state[0])*1e9)
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
                atomic(run/'closed_loop_executor_ack.json',{'schema':'teacher_known_velocity_ack/v1','entries':list(acks)})
                latest['pending_ack']=None
        return data

    def read_command():
        nonlocal last_seq,last_hash
        clock=latest['clock_ns'];started=time.monotonic();raw=None;data=None
        record={'read_clock_ns':clock,'read_monotonic_wall':started,'raw_utf8':None,'raw_bytes_sha256':None,'decoded_envelope':None}
        try:
            raw=(run/'navigation_command.json').read_bytes();record['raw_bytes_sha256']=hashlib.sha256(raw).hexdigest()
            record['raw_utf8']=raw.decode('utf-8');data=json.loads(record['raw_utf8']);record['decoded_envelope']=data
            if data.get('schema_version')!=1 or data.get('mode')!='scan_slam' or data.get('source')!='scan_slam':raise ValueError('Unknown command contract')
            if data.get('acceptance',{}).get('sha256')!=scope_sha:raise ValueError('Scope mismatch')
            seq=data['sequence'];h=hashlib.sha256(canonical(data).encode()).hexdigest()
            if type(seq) is not int or seq<1 or seq<last_seq or seq==last_seq and h!=last_hash:raise ValueError('Command sequence changed or reversed')
            values=np.asarray(data['command'],dtype=float)
            if values.shape!=(3,) or not np.isfinite(values).all() or (abs(values)>limits+1e-10).any():raise ValueError('Malformed or out-of-range velocity')
            sim_age=clock/1e9-float(data['sim_time']);wall_age=time.monotonic()-float(data['monotonic_wall'])
            if not(-.05<=sim_age<=.3 and 0<=wall_age<=.3):raise ValueError('Producer dual300ms expired')
            lineage=data.get('controller_source')
            if np.any(values):
                if not isinstance(lineage,dict) or lineage.get('navigation_ground_truth_used') is not False:raise ValueError('No original controller source')
                pose=lineage['source_pose_stamp_ns'];gyro=lineage['paired_imu_stamp_ns'];control=lineage['control_stamp_ns']
                if not all(type(x) is int for x in (pose,gyro,control)):raise ValueError('Invalid source integer stamps')
                if not(0<=clock-pose<=300_000_000 and 0<=pose-gyro<=20_000_000 and pose<=control<=clock+50_000_000):raise ValueError('Source age/causality rejected')
                if not(0<=time.monotonic()-lineage['source_pose_received_monotonic_wall']<=.3
                       and 0<=time.monotonic()-lineage['paired_imu_received_monotonic_wall']<=.3):raise ValueError('Original source wall age expired')
                if not np.allclose(values,lineage['command_after_slew'],atol=1e-12,rtol=0):raise ValueError('Command lost guarded source binding')
            last_seq,last_hash=seq,h
            if data.get('healthy') is not True:
                record.update(status='rejected',reason='SLAM/SCAN input unhealthy');return np.zeros(3),True,record
            if data.get('stop_requested') is True:values=np.zeros(3)
            record.update(status='accepted',reason='Fresh actual SLAM/IMU/SCAN velocity',producer_sim_age_s=sim_age,producer_wall_age_s=wall_age)
            return values,False,record
        except (OSError,ValueError,KeyError,TypeError,OverflowError) as e:
            record.update(status='rejected',reason=type(e).__name__+': '+str(e));return np.zeros(3),True,record

    def request(test,t):
        changed=[]
        for name,fp in fingerprints.items():
            try:
                st=Path(name).stat()
                if (st.st_size,st.st_mtime_ns)!=fp:changed.append(name)
            except OSError:changed.append(name)
        if changed:cmd,rejected,attempt=np.zeros(3),True,{'status':'rejected','reason':'Frozen source changed','paths':changed}
        else:cmd,rejected,attempt=read_command()
        if provider is not None and not changed:
            if provider.check_switch():
                cmd=np.zeros(3);rejected=True
                attempt.update(execution_override='actor_terrain_switch_protection',
                    reason='Actor terrain switch refused: '+provider.failure if provider.failure else 'Waiting for causal original connector arrival clock')
        evidence={'sim_time':float(t),'physics_clock_ns':latest['clock_ns'],'command':cmd.tolist(),
            'rejected':rejected,'reason':attempt['reason'],'actual_read_attempt':attempt,
            'actual_original_envelope':attempt.get('decoded_envelope'),'navigation_ground_truth_used':False}
        if provider is not None:evidence['actor_terrain_provider']=provider.evidence()
        latest['evidence']=evidence;log.write(canonical(evidence)+'\n')
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
        original_atomic(path,obj)

    legacy.recv_exact=receive;legacy.requested=request;legacy.atomic_json=metadata;legacy.json=IsolatedJSON()
    legacy.duration=lambda _:float(p['duration_s'])
    sys.argv=[str(run/'sources/policy/worker.py'),'--run',str(run),'--test','forward','--socket',a.socket,
        '--terrain-target-manifest',str(run/'terrain_target_manifest.json')]
    try:legacy.main()
    finally:
        log.close();ack_log.close()
        if provider is not None:provider.close()

if __name__=='__main__':main()
