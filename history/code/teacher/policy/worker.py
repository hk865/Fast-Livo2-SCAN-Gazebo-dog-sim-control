#!/usr/bin/env python3
"""Frozen CPU actor and explicit stop/timeout state machine. Simulation only."""
import argparse, hashlib, json, math, os, socket, time, sys
from pathlib import Path
import numpy as np
import torch
from observation import TerrainHeightMap, build_observation

ROOT=Path(__file__).resolve().parents[1]
CHECKPOINT=Path('/__LOCAL_HOME__/projects/1.Project/RL_for_unitree/logs/rsl_rl/rl_unitree_go2_aer_height_distribution/2026-10-01_17-23-23_STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A-FORMAL-4096ENV-3000ITER-SEED42/model_1000.pt')
SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
QDEFAULT=np.array([-.1,.8,-1.5,.1,.8,-1.5,-.1,1.,-1.5,.1,1.,-1.5])
COMMANDS={'stand':[0,0,0],'forward':[.3,0,0],'backward':[-.3,0,0],
          'left':[0,.2,0],'right':[0,-.2,0],'turn_positive':[0,0,.3],'turn_negative':[0,0,-.3],
          'ramp_up':[.3,0,0],'ramp_down':[.3,0,0],'step05':[.3,0,0],'step10':[.3,0,0],
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
    else:cmd=COMMANDS[test]if t<(14 if test.startswith('step')else 11) else [0,0,0]
    return np.array(cmd,dtype=float),'tracking' if any(cmd) else 'stop_transition'

def duration(test):return 600. if test=='navigation' else 26. if test=='switch' else 25. if test.startswith('step') else 15. if test=='stand' else 18.

def navigation_command(path,world_time,acceptance_sha):
    """Read only SLAM/SCAN-gated command envelopes; stale input parks via policy."""
    try:
        data=json.loads(path.read_text())
        if data.get('source')!='scan_slam' or data.get('mode')!='scan_slam':raise ValueError('Unexpected command source')
        if data.get('acceptance',{}).get('sha256')!=acceptance_sha:raise ValueError('Acceptance mismatch')
        wall_age=time.monotonic()-float(data['monotonic_wall']);sim_age=world_time-float(data['sim_time'])
        values=np.asarray(data['command'],dtype=float)
        if values.shape!=(3,)or not np.isfinite(values).all():raise ValueError('Invalid command values')
        if data.get('healthy')is not True or not (-.05<=wall_age<=.3 and -.05<=sim_age<=.3):
            return np.zeros(3),True,'SLAM/SCAN input stale or unhealthy'
        if (abs(values)>np.array([.3,.2,.3])+1e-8).any():raise ValueError('Command exceeds reviewed bounds')
        return values,False,'Fresh SLAM/SCAN command'
    except FileNotFoundError:return np.zeros(3),True,'Waiting for SLAM/SCAN command file'
    except (ValueError,KeyError,TypeError):return np.zeros(3),True,'Rejected malformed SLAM/SCAN command'
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
    args=p.parse_args();actor=load_actor();args.run.mkdir(parents=True,exist_ok=True)
    acceptance=None
    if args.test=='navigation':
        if args.command_file is None:raise ValueError('Navigation requires a reviewed command file')
        sys.path.insert(0,str(ROOT/'navigation'))
        from bridge import check_acceptance
        acceptance=check_acceptance(args.acceptance)
    terrain=TerrainHeightMap.from_sdf(args.run/'world.sdf');rng=np.random.default_rng(args.seed)
    last_action=np.zeros(12);cmd=np.zeros(3);fault=None;first_t=None;settled_frames=0
    last_command_t=0.;last_request=np.zeros(3)
    metadata={'checkpoint':str(CHECKPOINT),'checkpoint_sha256':SHA,'inference_device':'cpu','torch_threads':1,
              'observation_noise':'teacher_noisy matched uniform' if args.noisy else 'clean teacher_full evaluation',
              'observation_source':'Gazebo privileged COM twist, quaternion, joint physics, applied DCMotor torque, static SDF terrain raycast',
              'navigation_truth_used':False,'action_clip':None,'test':args.test,'seed':args.seed,
              'stop_strategy':'slew command to zero and continue Teacher closed loop; never freeze joint targets or replace action with zero',
              'bootstrap_PD_seconds':.1,'command_start_seconds':3.,'command_slew_acceleration':[.6,.6,.8],
              'test_duration_s':duration(args.test),'command_end_s':14. if args.test.startswith('step')else 11.}
    if acceptance:metadata.update(command_source='SLAM/SCAN command file',acceptance=acceptance)
    atomic_json(args.run/'policy_manifest.json',metadata)
    sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);sock.bind(args.socket);sock.listen(1);sock.settimeout(90)
    (args.run/'worker_ready').write_text(str(os.getpid()))
    stream=(args.run/'telemetry.jsonl').open('w',buffering=1);observations=[];actions=[];latencies=[];rows=[]
    try:
        conn,_=sock.accept();conn.settimeout(10)
        with conn,torch.inference_mode():
            while True:
                s=np.frombuffer(recv_exact(conn,64*8),dtype='<f8').copy()
                if first_t is None:first_t=s[0]
                t=float(s[0]-first_t);request,phase=requested(args.test,t)
                # Model a command producer that dies at t=7s. Its last command
                # really expires after .30s, then follows the same stop path.
                if args.test=='navigation':
                    request,command_expired,command_reason=navigation_command(args.command_file,s[0],acceptance['sha256'])
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
                start=time.perf_counter_ns();action=actor(torch.from_numpy(np.asarray(obs,dtype=np.float32)).reshape(1,247)).numpy()[0]
                latency=(time.perf_counter_ns()-start)/1e6;latencies.append(latency)
                if not np.isfinite(action).all() or not np.isfinite(obs).all():fault=fault or 'nonfinite_policy'
                if s[56]>0:fault=fault or f'actuator_fault_{int(s[56])}'
                # Clearance is privileged independent safety evidence, not a navigation pose.
                # Safety clearance casts from the body downward, while the
                # Teacher scan keeps the archived +20m ray-start semantics.
                ground=float(terrain.height(s[None,1:3],ray_start_z=s[3])[0])
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
                done=t>=duration(args.test) or (fault is not None and t>=2)
                response=np.zeros(16,dtype='<f8');response[:12]=target;response[12]=1 if fault else 0;response[13]=1 if done else 0
                row={'sim_time':t,'world_sim_time':float(s[0]),'command':cmd.tolist(),'requested':request.tolist(),
                     'measured':[float(s[8]),float(s[9]),float(s[13])], 'body_lin_vel':s[8:11].tolist(),
                     'body_ang_vel':s[11:14].tolist(),'position':s[1:4].tolist(),'quaternion_wxyz':s[4:8].tolist(),
                     'rpy':angles,'contacts':dict(zip(['body','FR','FL','RR','RL'],contacts)),
                     'q':s[14:26].tolist(),'qd':s[26:38].tolist(),'applied_torque':s[38:50].tolist(),
                     'q_target':target.tolist()if np.isfinite(target).all()else None,'action':action.tolist(),
                     'policy_target':(QDEFAULT+.25*action).tolist(),
                     'state':phase,'fault':fault,'inference_ms':latency,'body_clearance':float(s[3]-ground),
                     'height_scan_min':float(np.min(obs[60:])),'height_scan_max':float(np.max(obs[60:])),
                     'height_scan_overhead_points':int(extra.get('height_scan_overhead_count',0)),
                     'command_expired':command_expired,'command_age_sim_s':t-last_command_t,
                     'command_source':metadata.get('command_source','scripted simulation test'),'command_reason':command_reason,
                     'observation_source':metadata['observation_source']}
                rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n')
                if len(rows)%10==0 or done:atomic_json(args.run/'state.json',row)
                observations.append(obs);actions.append(action)
                conn.sendall(response.tobytes())
                if args.test=='worker_disconnect' and t>=7:break
                if done:break
    except Exception as e:
        fault=fault or f'{type(e).__name__}: {e}'
    finally:
        stream.close();sock.close()
        try:Path(args.socket).unlink()
        except FileNotFoundError:pass
        np.savez_compressed(args.run/'observations_actions.npz',observations=np.asarray(observations),actions=np.asarray(actions))
        atomic_json(args.run/'worker_result.json',{'completed':bool(rows),'fault':fault,'samples':len(rows),
            'last_sim_time':rows[-1]['sim_time']if rows else None,'cpu_forward_ms':{'p50':float(np.median(latencies)),'p95':float(np.percentile(latencies,95)),'max':float(max(latencies))}if latencies else {}})
if __name__=='__main__':main()
