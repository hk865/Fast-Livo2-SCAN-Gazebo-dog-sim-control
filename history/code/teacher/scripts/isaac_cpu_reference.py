#!/usr/bin/env python3
"""CPU-only physics/policy replay, frozen Teacher, no optimization or training.

Creates a separate single-env flat diagnostic simulation. Startup/reset
randomization, pushes and actor corruption are disabled. This is a nominal
reference, not a reproduction of the noisy randomized training distribution.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import sys
import time

RL = Path('/__LOCAL_HOME__/projects/1.Project/RL_for_unitree')
sys.path.insert(0, str(RL / 'source/rl_unitree'))
import gymnasium as gym
import numpy as np
import torch
import isaaclab_tasks
from isaaclab_tasks.utils import add_launcher_args, launch_simulation, resolve_task_config, setup_preset_cli
from rl_unitree.tasks import register
from isaaclab_rl.rsl_rl import handle_deprecated_rsl_rl_cfg

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--checkpoint', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--duration', type=float, default=10.)
parser.add_argument('--only-stand', action='store_true')
parser.add_argument('--schedule', type=Path, help='JSON cases with duration, segments[start,end,command,mode], optional command_slew xyz per second')
add_launcher_args(parser)
args, hydra_args = setup_preset_cli(parser)
if args.device != 'cpu':
    raise RuntimeError('This reference explicitly requires --device cpu')
sys.argv = [sys.argv[0]] + hydra_args
register()
torch.set_num_threads(1)
args.output = args.output.resolve()
args.checkpoint = args.checkpoint.resolve()
args.output.mkdir(parents=True, exist_ok=False)
schedule = json.loads(args.schedule.read_text()) if args.schedule else None
if schedule is not None:
    for case in schedule['cases']:
        end = 0.
        for segment in case['segments']:
            if abs(float(segment['start']) - end) > 1e-8 or float(segment['end']) <= end:
                raise ValueError('Schedule segments must continuously cover a case from time0')
            end = float(segment['end'])
            vector = np.asarray(segment['command'], dtype=float)
            if vector.shape != (3,) or not np.isfinite(vector).all():
                raise ValueError('Schedule command must be finite3')
            if segment.get('mode', 'teacher') not in ('teacher', 'pd_init'):
                raise ValueError('Schedule mode is teacher or explicit pd_init')
        if abs(end - float(case['duration'])) > 1e-8:
            raise ValueError('Schedule duration must equal its final segment end')
        if 'command_slew' in case:
            slew = np.asarray(case['command_slew'], dtype=float)
            if slew.shape != (3,) or not np.isfinite(slew).all() or (slew <= 0).any():
                raise ValueError('Schedule command_slew must be finite positive3')
sha = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
if sha != 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34':
    raise RuntimeError('Frozen checkpoint SHA mismatch')
config, agent = resolve_task_config('RL-Unitree-Go2-Flat-FullObservation-AER-GaitGeometry-v22', 'rsl_rl_cfg_entry_point')
config.sim.device = 'cpu'
config.scene.num_envs = 1
config.scene.terrain.terrain_generator.num_rows = 1
config.scene.terrain.terrain_generator.num_cols = 1
config.scene.terrain.terrain_generator.border_width = 100.
config.scene.terrain.max_init_terrain_level = 0
config.scene.terrain.terrain_generator.use_cache = False
longest_duration = max(c['duration'] for c in schedule['cases']) if schedule else args.duration
config.episode_length_s = max(20., longest_duration + 2.)
config.seed = 42
for group in ('policy', 'teacher_noisy', 'teacher_full'):
    getattr(config.observations, group).enable_corruption = False
config.events.push_robot = None
config.events.base_external_force_torque = None
material = config.events.physics_material.params
material.update(static_friction_range=(1., 1.), dynamic_friction_range=(1., 1.), restitution_range=(0., 0.))
config.events.add_base_mass.params['mass_distribution_params'] = (1., 1.)
config.events.base_com.params['com_range'] = {a:(0.,0.) for a in ('x','y','z')}
config.events.actuator_gains.params['stiffness_distribution_params'] = (1., 1.)
config.events.actuator_gains.params['damping_distribution_params'] = (1., 1.)
config.events.reset_base.params['pose_range'] = {a:(0.,0.) for a in ('x','y','yaw')}
config.events.reset_base.params['velocity_range'] = {a:(0.,0.) for a in ('x','y','z','roll','pitch','yaw')}
config.events.reset_robot_joints.params['position_range'] = (1., 1.)
config.events.reset_robot_joints.params['velocity_range'] = (0., 0.)
config.commands.base_velocity.heading_command = False
config.commands.base_velocity.rel_heading_envs = 0.
config.commands.base_velocity.rel_standing_envs = 0.
config.commands.base_velocity.resampling_time_range = (1000., 1000.)
actor = torch.nn.Sequential(torch.nn.Linear(247,512),torch.nn.ELU(),torch.nn.Linear(512,256),torch.nn.ELU(),torch.nn.Linear(256,128),torch.nn.ELU(),torch.nn.Linear(128,12))
checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
actor.load_state_dict({k.removeprefix('mlp.'):v for k,v in checkpoint['actor_state_dict'].items() if k.startswith('mlp.')}, strict=True)
actor.eval()
protocol = {'checkpoint':str(args.checkpoint), 'checkpoint_sha256':sha,
            'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'task':'RL-Unitree-Go2-Flat-FullObservation-AER-GaitGeometry-v22',
            'policy_device':'cpu', 'physics_device':'cpu', 'num_envs':1,
            'diagnostic_conditions':'nominal dynamics, reset default at rest, no pushes/no observation noise, flat mesh, single seed42',
            'criterion':{'fall':'termination before timeout or roll/pitch >0.8rad or base clearance<0.16m', 'velocity_rmse_xy_max_mps':.15, 'velocity_rmse_yaw_max_radps':.2, 'stand_drift_max_m':.15},
            'physics_dt':config.sim.dt, 'decimation':config.decimation,
            'schedule':schedule, 'schedule_sha256':hashlib.sha256(args.schedule.read_bytes()).hexdigest() if args.schedule else None,
            'no_training':True, 'started_at':time.time()}
(args.output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
cases=[{'name':name,'command':command,'duration':args.duration} for name,command in [('stand',[0.,0.,0.]),('forward',[.3,0.,0.]),('backward',[-.3,0.,0.]),('left',[0.,.2,0.]),('right',[0.,-.2,0.]),('yaw_positive',[0.,0.,.3]),('yaw_negative',[0.,0.,-.3]),('switch',None)]]
if schedule is not None: cases=schedule['cases']
if args.only_stand: cases=[case for case in cases if case['name']=='stand']
results=[]
try:
    with launch_simulation(config, args):
        env=gym.make(protocol['task'],cfg=config)
        base=env.unwrapped
        robot=base.scene['robot']
        cmd=base.command_manager.get_term('base_velocity')
        action_term=base.action_manager.get_term('joint_pos')
        ids=action_term._joint_ids
        contact=base.scene.sensors['contact_forces']
        def command(value):
            cmd.vel_command_b.copy_(torch.tensor([value],dtype=torch.float32,device='cpu'))
            cmd.time_left.fill_(1000.)
            cmd.is_standing_env.fill_(False)
            cmd.is_heading_env.fill_(False)
        for case in cases:
            name, value = case['name'], case.get('command')
            env.reset()
            start_pos=robot.data.root_link_pos_w.torch[0].numpy().copy()
            trace=[]; failed=False
            n=int(round(case['duration']/base.step_dt))
            previous_command=np.zeros(3)
            for index in range(n):
                t=index*base.step_dt
                target=value
                control_mode='teacher'
                if 'segments' in case:
                    segment=next(s for s in case['segments'] if s['start']<=t<s['end'])
                    target=segment['command']
                    control_mode=segment.get('mode','teacher')
                elif target is None:
                    target=[0.,0.,0.] if t<2. or t>=8. else ([.3,0.,0.] if t<5. else [-.3,0.,0.])
                requested_target=np.asarray(target,dtype=float)
                target=requested_target.copy()
                if 'command_slew' in case:
                    delta=np.asarray(case['command_slew'])*base.step_dt
                    target=previous_command+np.clip(target-previous_command,-delta,delta)
                previous_command=target.copy()
                command(target)
                obs=base.observation_manager.compute_group('teacher_full', update_history=True)
                with torch.inference_mode(): action=actor(obs) if control_mode=='teacher' else torch.zeros((1,12),device='cpu')
                if not torch.isfinite(obs).all() or not torch.isfinite(action).all():
                    raise RuntimeError('Nonfinite Teacher inference')
                pos=robot.data.root_link_pos_w.torch[0].numpy().copy()
                quat=robot.data.root_link_quat_w.torch[0].numpy().copy() # xyzw
                vx=robot.data.root_lin_vel_b.torch[0].numpy().copy()
                wz=robot.data.root_ang_vel_b.torch[0].numpy().copy()
                x,y,z,w=quat
                roll=float(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)))
                pitch=float(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
                q=robot.data.joint_pos.torch[0,ids].numpy().copy()
                qd=robot.data.joint_vel.torch[0,ids].numpy().copy()
                torque=robot.data.applied_torque.torch[0,ids].numpy().copy()
                ray=base.scene.sensors['height_scanner'].data.ray_hits_w.torch[0,:,2].numpy().copy()
                clearance=float(pos[2]-np.median(ray[np.isfinite(ray)])) if np.isfinite(ray).any() else None
                cf=contact.data.net_forces_w.torch[0].numpy().copy()
                trace.append({'t':t,'command':target.tolist(),'requested_command':requested_target.tolist(),'controller_mode':control_mode,'position':pos.tolist(),'quaternion_xyzw':quat.tolist(),
                    'linear_velocity_body_com':vx.tolist(),'angular_velocity_body':wz.tolist(),
                    'roll':roll,'pitch':pitch,'clearance':clearance,'q':q.tolist(),'qd':qd.tolist(),
                    'torque':torque.tolist(),'action':action[0].numpy().tolist(),
                    'q_target':(action_term._offset[0]+action_term._scale*action[0]).numpy().tolist(),
                    'height_scan':obs[0,60:].numpy().tolist(),'observation247':obs[0].numpy().tolist(),'observed_command':obs[0,9:12].numpy().tolist(),'last_action_observed':obs[0,48:60].numpy().tolist(),'height_scan_raw':(base.scene.sensors['height_scanner'].data.pos_w.torch[0,2]-base.scene.sensors['height_scanner'].data.ray_hits_w.torch[0,:,2]-.5).numpy().tolist(),'contact_forces':cf.tolist()})
                _,_,terminated,truncated,_=env.step(action)
                if terminated.item() or max(abs(roll),abs(pitch))>.8 or (clearance is not None and clearance<.16):
                    failed=True; break
            (args.output/f'{name}_trace.json').write_text(json.dumps(trace,separators=(',',':'))+'\n')
            measured=[row for row in trace if row['t']>=1.]
            errs=np.asarray([np.r_[np.asarray(row['linear_velocity_body_com'])[:2]-np.asarray(row['command'])[:2],row['angular_velocity_body'][2]-row['command'][2]] for row in measured])
            rmse=np.sqrt(np.mean(errs**2,axis=0)).tolist() if len(errs) else None
            displacement=float(np.linalg.norm(np.asarray(trace[-1]['position'])[:2]-start_pos[:2]))
            passed=not failed and rmse is not None and np.linalg.norm(rmse[:2])<=.15 and rmse[2]<=.2 and (name!='stand' or displacement<=.15)
            result={'name':name,'command':value,'failed':failed,'duration_recorded':len(trace)*base.step_dt,'rmse_vx_vy_wz':rmse,'displacement_xy':displacement,'mean_actual_vx_vy_wz':np.mean([np.r_[row['linear_velocity_body_com'][:2],row['angular_velocity_body'][2]] for row in measured],axis=0).tolist() if measured else None,'max_abs_roll':max(abs(row['roll']) for row in trace),'max_abs_pitch':max(abs(row['pitch']) for row in trace),'passed':passed}
            results.append(result)
            (args.output/'results.json').write_text(json.dumps({'status':'running','results':results,'protocol':protocol},indent=2)+'\n')
            print('CASE',json.dumps(result),flush=True)
        status='completed'
        (args.output/'results.json').write_text(json.dumps({'status':status,'results':results,'protocol':protocol},indent=2)+'\n')
        env.close()
    status='completed'
except Exception as error:
    status='infrastructure_failed'
    (args.output/'failure.json').write_text(json.dumps({'error':repr(error)},indent=2)+'\n')
    raise
finally:
    (args.output/'results.json').write_text(json.dumps({'status':locals().get('status','infrastructure_failed'),'results':results,'protocol':protocol},indent=2)+'\n')
