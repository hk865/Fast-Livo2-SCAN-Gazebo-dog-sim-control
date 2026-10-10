#!/usr/bin/env python3
# Public learning snapshot 2026-10-10: privacy/path metadata or documentation links adjusted; control algorithms and numeric gates unchanged.
"""Frozen CPU actor and explicit stop/timeout state machine. Simulation only."""
import argparse, hashlib, json, math, os, socket, time, sys
from pathlib import Path
import numpy as np
import torch
from observation import TerrainHeightMap, build_observation

ROOT=Path(__file__).resolve().parents[1]
CHECKPOINT=Path('external/weights/teacher.pt')
SHA=''
QDEFAULT=np.array([-.1,.8,-1.5,.1,.8,-1.5,-.1,1.,-1.5,.1,1.,-1.5])
COMMANDS={'stand':[0,0,0],'forward':[.3,0,0],'backward':[-.3,0,0],
          'left':[0,.2,0],'right':[0,-.2,0],'turn_positive':[0,0,.3],'turn_negative':[0,0,-.3],
          'ramp_up':[.3,0,0],'ramp_down':[.3,0,0],'step05':[.3,0,0],'step10':[.3,0,0],
          'step05_continue':[.3,0,0],'step10_continue':[.3,0,0],
          'walk_stop':[.3,0,0],'command_timeout':[.3,0,0], 'switch':[.3,0,0],
          'invalid_output':[.3,0,0],'worker_disconnect':[.3,0,0],'navigation':[0,0,0]}

def load_actor():
    torch.set_num_threads(1); torch.set_num_interop_threads(1)
    digest=hashlib.sha256(CHECKPOINT.read_bytes()).hexdigest()
    if digest!=SHA:raise RuntimeError('Frozen checkpoint SHA mismatch')
    payload=torch.load(CHECKPOINT,map_location='cpu',weights_only=True)
    actor=torch.nn.Sequential(torch.nn.Linear(247,512),torch.nn.ELU(),torch.nn.Linear(512,256),torch.nn.ELU(),
                              torch.nn.Linear(256,128),torch.nn.ELU(),torch.nn.Linear(128,12)).eval()
    state=payload['actor_state_dict']
    actor.load_state_dict({k.removeprefix('mlp.'):v for k,v in state.items()if k.startswith('mlp.')},strict=True)
    if not all(torch.isfinite(v).all()for v in actor.state_dict().values()):raise RuntimeError('Nonfinite weights')
    return actor

def requested(test,t):
    if t<3:return np.zeros(3), 'initializing' if t<.1 else 'teacher_stand'
    if test=='stand':return np.zeros(3),'teacher_stand'
    if test=='switch':
        if t<8:cmd=[.3,0,0]
        elif t<13:cmd=[0,.2,0]
        elif t<18:cmd=[0,0,-.3]
        else:cmd=[0,0,0]
    else:cmd=COMMANDS[test]if t<(22 if test.endswith('_continue')else 14 if test.startswith('step')else 11) else [0,0,0]
    return np.array(cmd,dtype=float),'tracking' if any(cmd) else 'stop_transition'

def duration(test):return 600. if test=='navigation' else 30. if test.endswith('_continue') else 26. if test=='switch' else 25. if test.startswith('step') else 15. if test=='stand' else 18.

def load_schedule(path):
    if path is None:return None
    data=json.loads(path.read_text())
    end=float(data['duration_s']);steps=data['commands']
    if not math.isfinite(end) or not 5<=end<=600 or not isinstance(steps,list) or not steps:
        raise ValueError('Schedule requires 5..600 seconds and explicit commands')
    previous=-1.
    for step in steps:
        start=float(step['start_s']);command=np.asarray(step['command'],dtype=float)
        if not math.isfinite(start) or not previous<start<end or command.shape!=(3,) or not np.isfinite(command).all():
            raise ValueError('Invalid or unordered schedule command')
        if (abs(command)>np.asarray([.3,.2,.3])+1e-8).any():raise ValueError('Schedule exceeds bounded command limits')
        previous=start
    if float(steps[0]['start_s'])!=0 or any(steps[0]['command']) or any(steps[-1]['command']):
        raise ValueError('Schedule must begin and end with zero velocity command')
    completion=data.get('completion_stop')
    if completion:
        if completion.get('axis')!='x' or completion.get('comparison')not in ('le','ge'):
            raise ValueError('Physical test completion supports an explicit x threshold only')
        values=[float(completion[k])for k in ('threshold','minimum_active_s','settle_s')]
        if not all(math.isfinite(v)for v in values) or values[1]<3 or not 5<=values[2]<=15:
            raise ValueError('Invalid physical completion bounds')
    return data

def scheduled_request(schedule,t):
    step=schedule['commands'][0]
    for candidate in schedule['commands']:
        if float(candidate['start_s'])>t:break
        step=candidate
    command=np.asarray(step['command'],dtype=float)
    return command,'initializing'if t<.1 else 'tracking'if any(command)else 'stop_transition'

def sensor_terms(run,state,obs,broker_sha):
    data=json.loads((run/'sensor_feedback.json').read_text());now=time.monotonic();stamp_ns=round((float(state[0])-.005)*1e9)
    if data.get('schema_version')!=1 or data.get('run')!=str(run.resolve()) or data.get('broker_alive')is not True:
        raise ValueError('Sensor broker is absent, dead or belongs to a different run')
    if data.get('script_sha256')!=broker_sha or data.get('application_publishers')!=[]:
        raise ValueError('Sensor broker source or read-only contract differs')
    if not 0<=now-float(data['broker_monotonic_wall'])<=.3:raise ValueError('Sensor broker wall timeout')
    selected={}
    for key in ('imu','joints'):
        graph=data['publisher_graph'].get(key,[])
        if len(graph)!=1 or graph[0].get('node_name')!='ros_gz_bridge':raise ValueError('Actual sensor publisher is not unique Gazebo bridge')
        eligible=[v for v in data['histories'][key]if 0<=stamp_ns-int(v['stamp_ns'])<=25000000 and 0<=now-float(v['received_monotonic_wall'])<=.3]
        if not eligible:
            history=data['histories'][key];latest_stamp=int(history[-1]['stamp_ns'])if history else None
            latest_wall_age=now-float(history[-1]['received_monotonic_wall'])if history else None
            atomic_json(run/'sensor_feedback_rejection.json',{'schema_version':1,'reference_ns':stamp_ns,
                'read_monotonic_wall':now,'rejected_source':key,'broker_payload':data,
                'candidate_ages':[{'stamp_ns':int(v['stamp_ns']),'age_sim_s':(stamp_ns-int(v['stamp_ns']))*1e-9,
                    'age_wall_s':now-float(v['received_monotonic_wall'])}for v in history],
                'maximum_sim_age_s':.025,'maximum_wall_age_s':.3,'truth_fallback':False})
            raise ValueError('No causal fresh actual '+key+' sample; reference_ns='+str(stamp_ns)+' latest_ns='+str(latest_stamp)+' age_sim_s='+str((stamp_ns-latest_stamp)*1e-9 if latest_stamp is not None else None)+' age_wall_s='+str(latest_wall_age)+' broker_sequence='+str(data['broker_sequence']))
        selected[key]=max(eligible,key=lambda v:int(v['stamp_ns']))
    imu,joints=selected['imu'],selected['joints'];gyro=np.asarray(imu['gyro_body']);gravity=np.asarray(imu['gravity_body']);q=np.asarray(joints['q']);qd=np.asarray(joints['qd'])
    if gyro.shape!=(3,)or gravity.shape!=(3,)or q.shape!=(12,)or qd.shape!=(12,)or not all(np.isfinite(v).all()for v in (gyro,gravity,q,qd)):
        raise ValueError('Actual sensor vector shape or finite check failed')
    expected=[f'{leg}_{part}_joint'for leg in ('rf','lf','rh','lh')for part in ('hip','upper_leg','lower_leg')]
    if joints['ordered_names']!=expected or abs(np.linalg.norm(gravity)-1)>.001:raise ValueError('Actual sensor mapping or gravity contract failed')
    out=obs.copy();out[3:6]=np.clip(gyro,-100,100)*.2;out[6:9]=np.clip(gravity,-100,100);out[12:24]=np.clip(q-QDEFAULT,-100,100);out[24:36]=np.clip(qd,-100,100)*.05
    evidence={'effective_physics_stamp_ns':stamp_ns,'broker_sequence':data['broker_sequence'],'broker_sha256':broker_sha,
        'raw_terms':{'gyro_body':gyro.tolist(),'gravity_body':gravity.tolist(),'q':q.tolist(),'qd':qd.tolist()},
        'sources':{key:{'stamp_ns':v['stamp_ns'],'age_sim_s':(stamp_ns-int(v['stamp_ns']))*1e-9,'age_wall_s':now-float(v['received_monotonic_wall']),
                       'source_topic':v['source_topic'],'frame':v['frame']}for key,v in selected.items()},
        'replaced_dimensions':[3,6,12,24],'dimension_count':30,'actor_input_changed':True,'truth_fallback':False}
    return out,evidence

def navigation_command(path,world_time,acceptance_sha,sequence_state=None):
    """Read only SLAM/SCAN-gated command envelopes; stale input parks via policy."""
    if sequence_state is not None:sequence_state.update(read_monotonic_wall=time.monotonic(),read_status='rejected_or_missing')
    try:
        data=json.loads(path.read_text())
        if data.get('schema_version')!=1:raise ValueError('Unsupported command schema')
        if data.get('source')!='scan_slam' or data.get('mode')!='scan_slam':raise ValueError('Unexpected command source')
        if data.get('acceptance',{}).get('sha256')!=acceptance_sha:raise ValueError('Acceptance mismatch')
        sequence=data.get('sequence')
        if isinstance(sequence,bool) or not isinstance(sequence,int) or sequence<1:raise ValueError('Invalid command sequence')
        envelope_hash=hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()
        if sequence_state is not None:
            previous=sequence_state.get('sequence',0)
            if sequence<previous or (sequence==previous and envelope_hash!=sequence_state.get('hash')):
                raise ValueError('Command sequence moved backward or changed')
        wall_age=time.monotonic()-float(data['monotonic_wall']);sim_age=world_time-float(data['sim_time'])
        values=np.asarray(data['command'],dtype=float)
        if values.shape!=(3,)or not np.isfinite(values).all():raise ValueError('Invalid command values')
        if not isinstance(data.get('stop_requested'),bool):raise ValueError('Missing stop state')
        if sequence_state is not None:sequence_state.update(sequence=sequence,hash=envelope_hash,source=data['source'],envelope_sim_time=float(data['sim_time']),
            envelope_monotonic_wall=float(data['monotonic_wall']),wall_age_s=wall_age,sim_age_s=sim_age,healthy=data.get('healthy'),stop_requested=data['stop_requested'],read_status='valid_but_unhealthy_or_stale')
        if data.get('healthy')is not True or not (-.05<=wall_age<=.3 and -.05<=sim_age<=.3):
            return np.zeros(3),True,'SLAM/SCAN input stale or unhealthy'
        if (abs(values)>np.array([.3,.2,.3])+1e-8).any():raise ValueError('Command exceeds reviewed bounds')
        if sequence_state is not None:sequence_state['read_status']='accepted'
        if data['stop_requested']:return np.zeros(3),False,'SLAM/SCAN requested controlled stop'
        return values,False,'Fresh SLAM/SCAN command'
    except FileNotFoundError:return np.zeros(3),True,'Waiting for SLAM/SCAN command file'
    except (OSError,ValueError,KeyError,TypeError,OverflowError):return np.zeros(3),True,'Rejected malformed SLAM/SCAN command'
def recv_exact(conn,n):
    buf=b''
    while len(buf)<n:
        data=conn.recv(n-len(buf))
        if not data:raise EOFError('Gazebo actuator disconnected')
        buf+=data
    return buf
def atomic_json(p,obj):
    tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(obj,ensure_ascii=False,allow_nan=False));tmp.replace(p)
def rpy(q):
    w,x,y,z=q
    return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(np.clip(2*(w*y-z*x),-1,1)),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--test',choices=list(COMMANDS),required=True)
    p.add_argument('--socket',required=True);p.add_argument('--noisy',action='store_true');p.add_argument('--seed',type=int,default=42)
    p.add_argument('--command-file',type=Path);p.add_argument('--acceptance',type=Path,default=ROOT/'runs/acceptance.json')
    p.add_argument('--schedule',type=Path)
    p.add_argument('--terrain-target-manifest',type=Path)
    p.add_argument('--navigation-duration',type=float,default=120.)
    p.add_argument('--sensor-feedback',action='store_true')
    args=p.parse_args();schedule=load_schedule(args.schedule);actor=load_actor();args.run.mkdir(parents=True,exist_ok=True)
    if schedule and args.test=='navigation':raise ValueError('A scripted schedule cannot replace SLAM navigation')
    acceptance=None
    if args.test=='navigation':
        if not 30<=args.navigation_duration<=300:raise ValueError('Navigation experiment duration must be 30..300 seconds')
        if args.command_file is None:raise ValueError('Navigation requires a reviewed command file')
        sys.path.insert(0,str(ROOT/'navigation'))
        from bridge import check_acceptance
        acceptance=check_acceptance(args.acceptance)
    safety_terrain=TerrainHeightMap.from_sdf(args.run/'world.sdf');terrain=safety_terrain
    terrain_manifest=None
    if args.terrain_target_manifest:
        terrain_manifest=json.loads(args.terrain_target_manifest.read_text());selected=terrain_manifest['include_models']
        if not isinstance(selected,list)or not selected or any(not isinstance(v,str)for v in selected)or len(set(selected))!=len(selected):
            raise ValueError('Terrain target manifest requires explicit unique model names')
        available={shape.name.split('/')[0]for shape in safety_terrain.shapes}
        if set(selected)-available:raise ValueError('Requested terrain model absent from actual static collisions')
        terrain=TerrainHeightMap.from_sdf(args.run/'world.sdf',include_models=selected)
    rng=np.random.default_rng(args.seed)
    last_action=np.zeros(12);cmd=np.zeros(3);fault=None;first_t=None;settled_frames=0
    last_command_t=0.;last_request=np.zeros(3);navigation_sequence={};completion_time=None
    metadata={'checkpoint':str(CHECKPOINT),'checkpoint_sha256':SHA,'inference_device':'cpu','torch_threads':1,
              'observation_noise':'teacher_noisy matched uniform' if args.noisy else 'clean teacher_full evaluation',
              'observation_source':'Gazebo privileged COM twist, quaternion, joint physics, applied DCMotor torque, static SDF terrain raycast',
              'navigation_truth_used':False,'action_clip':None,'test':args.test,'seed':args.seed,
              'stop_strategy':'slew command to zero and continue Teacher closed loop; never freeze joint targets or replace action with zero',
              'bootstrap_PD_seconds':.1,'command_start_seconds':3.,'command_slew_acceleration':[.6,.6,.8],
              'physics_state_phase':'Gazebo PreUpdate reads previous Physics result; effective state time = world_sim_time - 0.005s',
              'test_duration_s':duration(args.test),'command_end_s':22. if args.test.endswith('_continue')else 14. if args.test.startswith('step')else 11.}
    if args.test.endswith('_continue'):
        metadata['functional_step_protocol']='tests/step_functional_protocol.json'
        metadata['functional_step_scope']='Step ascent and continued forward walking; world base height gain is diagnostic only'
    if schedule:
        metadata.update(test_duration_s=float(schedule['duration_s']),command_start_seconds=next((float(v['start_s'])for v in schedule['commands']if any(v['command'])),None),
                        command_end_s=float(schedule['commands'][-1]['start_s']),schedule=str(args.schedule),schedule_sha256=hashlib.sha256(args.schedule.read_bytes()).hexdigest(),
                        protocol_scope=schedule.get('scope','independent bounded simulation protocol'),legacy_motion_evaluation_applicable=False)
        metadata['physical_test_completion_source']='privileged base x only for test termination; no position servo or navigation localization'if schedule.get('completion_stop')else None
    if acceptance:metadata.update(command_source='SLAM/SCAN command file',acceptance=acceptance,test_duration_s=args.navigation_duration,legacy_motion_evaluation_applicable=False)
    broker_sha=hashlib.sha256((args.run/'sources/scripts/sensor_feedback.py').read_bytes()).hexdigest()if args.sensor_feedback else None
    if args.sensor_feedback:
        if args.noisy:raise ValueError('Sensor replacement A/B uses clean evaluation only')
        metadata.update(observation_source='Gazebo privileged COM velocity, applied torque and static terrain; actual ROS IMU gyro/gravity and named physical joints q/qd after t=3s',
                        actual_sensor_replacement={'activation_s':3.,'dimensions':30,'script_sha256':broker_sha,'maximum_sim_age_s':.025,'maximum_wall_age_s':.3,
                                                   'causal_reference':'world_sim_time-.005','on_failure':'native damping; no privileged fallback',
                                                   'remaining_dimensions_not_replaced':217,'remaining_privileged_dimensions':190,'controller_known_dimensions':27})
    if terrain_manifest:
        metadata['terrain_target_manifest']={'path':str(args.terrain_target_manifest),'sha256':hashlib.sha256(args.terrain_target_manifest.read_bytes()).hexdigest(),
            'definition':terrain_manifest,'selected_collision_names':[shape.name for shape in terrain.shapes],
            'excluded_collision_names':[shape.name for shape in safety_terrain.shapes if shape.name not in {v.name for v in terrain.shapes}],
            'world_sha256':terrain.source_sha256,'safety_source':'all static collisions, ray start at body height',
            'equivalence':'Explicit deployment terrain provider; frozen grid/yaw/+20/clip retained. Terrain target geometry is not strict training equivalence.'}
    atomic_json(args.run/'policy_manifest.json',metadata)
    sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);sock.bind(args.socket);sock.listen(1);sock.settimeout(90)
    (args.run/'worker_ready').write_text(str(os.getpid()))
    stream=(args.run/'telemetry.jsonl').open('w',buffering=1);scan_stream=(args.run/'height_scan_sources.jsonl').open('w',buffering=1)if terrain_manifest else None;observations=[];actions=[];latencies=[];sample_count=0;last_row=None
    try:
        conn,_=sock.accept();conn.settimeout(10)
        with conn,torch.inference_mode():
            while True:
                s=np.frombuffer(recv_exact(conn,64*8),dtype='<f8').copy()
                if first_t is None:first_t=s[0]
                t=float(s[0]-first_t);request,phase=scheduled_request(schedule,t)if schedule else requested(args.test,t)
                if schedule and schedule.get('completion_stop'):
                    criterion=schedule['completion_stop'];crossed=float(s[1])<=float(criterion['threshold'])if criterion['comparison']=='le'else float(s[1])>=float(criterion['threshold'])
                    if completion_time is None and t>=float(criterion['minimum_active_s'])and crossed:completion_time=t
                    if completion_time is not None:request=np.zeros(3);phase='stop_transition'
                # Model a command producer that dies at t=7s. Its last command
                # really expires after .30s, then follows the same stop path.
                if args.test=='navigation':
                    current_acceptance=check_acceptance(args.acceptance)
                    if current_acceptance['sha256']!=acceptance['sha256']:raise RuntimeError('Acceptance changed during navigation')
                    request,command_expired,command_reason=navigation_command(args.command_file,s[0],acceptance['sha256'],navigation_sequence)
                    phase='tracking'if any(request)else 'stop_transition'
                    last_command_t=t if not command_expired else last_command_t
                elif args.test!='command_timeout' or t<7:
                    last_command_t=t;last_request=request.copy()
                if args.test!='navigation':
                    command_expired=t-last_command_t>.3;command_reason='scripted command producer TTL'
                    if command_expired:request=np.zeros(3);phase='stop_transition'
                    else:request=last_request.copy()
                cmd+=np.clip(request-cmd,-np.array([.6,.6,.8])*.02,np.array([.6,.6,.8])*.02)
                angles=rpy(s[4:8]);contacts=s[50:55].tolist()
                obs,extra=build_observation(s,cmd,last_action,terrain,noisy=args.noisy,rng=rng)
                sensor_evidence=None
                if args.sensor_feedback and t>=3:
                    try:obs,sensor_evidence=sensor_terms(args.run,s,obs,broker_sha)
                    except (OSError,ValueError,KeyError,TypeError)as error:
                        fault=fault or 'actual_sensor_source_rejected: '+str(error);phase='fault_damping_sensor_source';obs=np.full(247,np.nan,dtype=np.float32)
                if scan_stream:scan_stream.write(json.dumps({'sim_time':t,**extra},allow_nan=False)+'\n')
                start=time.perf_counter_ns();action=actor(torch.from_numpy(np.asarray(obs,dtype=np.float32)).reshape(1,247)).numpy()[0]if np.isfinite(obs).all()else last_action.copy()
                latency=(time.perf_counter_ns()-start)/1e6;latencies.append(latency)
                if not np.isfinite(action).all() or not np.isfinite(obs).all():fault=fault or 'nonfinite_policy'
                if s[56]>0:fault=fault or f'actuator_fault_{int(s[56])}'
                # Clearance is privileged independent safety evidence, not a navigation pose.
                # Safety clearance casts from the body downward, while the
                # Teacher scan keeps the archived +20m ray-start semantics.
                ground=float(safety_terrain.height(s[None,1:3],ray_start_z=s[3])[0])
                if t>1.5 and (max(abs(angles[0]),abs(angles[1]))>.8 or s[3]-ground<.15):fault=fault or 'fallen_or_low_clearance'
                if t>1.5 and contacts[0]>0:fault=fault or 'body_contact'
                target=QDEFAULT+.25*action
                if t<.1:target=QDEFAULT.copy();phase='initializing';last_action=np.zeros(12)
                else:last_action=action.copy()
                # Parking keeps the learned posture feedback active. Freezing
                # a single support target was tested, failed, and is archived.
                # A zero VELOCITY COMMAND is never treated as a zero ACTION.
                if phase=='stop_transition' and np.linalg.norm(cmd)<1e-6:
                    settled_frames=settled_frames+1 if all(v>0 for v in contacts[1:]) and np.linalg.norm(s[8:10])<.08 and abs(s[13])<.1 else 0
                    if settled_frames>=15:phase='teacher_zero_command_stand'
                else:settled_frames=0
                if args.test=='invalid_output' and t>=7:target[0]=float('nan');phase='fault_injection'
                done=t>=metadata['test_duration_s'] or (completion_time is not None and t>=completion_time+float(schedule['completion_stop']['settle_s'])) or (fault is not None and t>=2)
                response=np.zeros(16,dtype='<f8');response[:12]=target;response[12]=1 if fault else 0;response[13]=1 if done else 0
                row={'sim_time':t,'world_sim_time':float(s[0]),'command':cmd.tolist(),'requested':request.tolist(),
                     'state_physics_world_time':float(s[0]-.005),
                     'measured':[float(s[8]),float(s[9]),float(s[13])], 'body_lin_vel':s[8:11].tolist(),
                     'body_ang_vel':s[11:14].tolist(),'position':s[1:4].tolist(),'quaternion_wxyz':s[4:8].tolist(),
                     'rpy':angles,'contacts':dict(zip(['body','FR','FL','RR','RL'],contacts)),
                     'q':s[14:26].tolist(),'qd':s[26:38].tolist(),'applied_torque':s[38:50].tolist(),
                     'q_target':target.tolist()if np.isfinite(target).all()else None,'action':action.tolist(),
                     'policy_target':(QDEFAULT+.25*action).tolist(),
                     'state':phase,'fault':fault,'inference_ms':latency,'body_clearance':float(s[3]-ground),
                     'height_scan_min':float(np.min(obs[60:]))if np.isfinite(obs).all()else None,'height_scan_max':float(np.max(obs[60:]))if np.isfinite(obs).all()else None,
                     'height_scan_overhead_points':int(extra.get('height_scan_overhead_count',0)),
                     'command_expired':command_expired,'command_age_sim_s':t-last_command_t,
                     'command_source':metadata.get('command_source','scripted simulation test'),'command_reason':command_reason,
                     'physical_test_completion_time_s':completion_time,
                     'navigation_envelope':navigation_sequence.copy()if args.test=='navigation'else None,
                     'sensor_feedback_used':sensor_evidence,'actor_inferred_this_frame':bool(np.isfinite(obs).all()),
                     'observation_source':metadata['observation_source']}
                sample_count+=1;last_row=row;stream.write(json.dumps(row,allow_nan=False)+'\n')
                if sample_count%10==0 or done:atomic_json(args.run/'state.json',row)
                observations.append(obs);actions.append(action)
                conn.sendall(response.tobytes())
                if args.test=='worker_disconnect' and t>=7:break
                if done:break
    except Exception as e:
        fault=fault or f'{type(e).__name__}: {e}'
    finally:
        stream.close();sock.close()
        if scan_stream:scan_stream.close()
        try:Path(args.socket).unlink()
        except FileNotFoundError:pass
        np.savez_compressed(args.run/'observations_actions.npz',observations=np.asarray(observations),actions=np.asarray(actions))
        atomic_json(args.run/'worker_result.json',{'completed':bool(sample_count),'fault':fault,'samples':sample_count,
            'physical_test_completion_time_s':completion_time,
            'last_sim_time':last_row['sim_time']if sample_count else None,'cpu_forward_ms':{'p50':float(np.median(latencies)),'p95':float(np.percentile(latencies,95)),'max':float(max(latencies))}if latencies else {}})
if __name__=='__main__':main()
