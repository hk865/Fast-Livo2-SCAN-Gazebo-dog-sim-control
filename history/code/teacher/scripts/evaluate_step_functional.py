#!/usr/bin/env python3
"""Independent low-step functional audit. World-Z gain is diagnostic only.

Uses actual native foot collision pairs plus SDF kinematics, never navigation
truth. Historical protocol/evaluator/results are preserved without alteration.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LEGS = {'FR':'rf','FL':'lf','RR':'rh','RL':'lh'}
PROTOCOL = {
    'scope':'5/10 cm single raised tread entry and continued walking; not stairs or full obstacle crossing',
    'duration_s':30.,'command_start_s':3.,'command_end_s':22.,'stop_window_s':[27.,30.],
    'command_vx_mps':.3,'minimum_continuous_foot_tread_support_s':.1,
    'body_entry_past_front_m':.5,'minimum_continued_progress_m':1.,
    'minimum_late_forward_velocity_mps':.02,'minimum_late_positive_velocity_fraction':.8,
    'max_abs_roll_pitch_rad':.65,'min_support_clearance_m':.18,
    'stop_translation_drift_m':.15,'stop_yaw_drift_rad':.2,
    'stop_linear_rms_mps':.08,'stop_yaw_rate_rms_radps':.1,
    'foot_top_geometric_tolerance_m':.015,
    'contact_top_z_tolerance_m':.005,'minimum_abs_contact_normal_z':.8,
    'body_world_height_gain_is_pass_criterion':False,
}


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def clean(x):
    if isinstance(x,np.ndarray):return clean(x.tolist())
    if isinstance(x,np.generic):return clean(x.item())
    if isinstance(x,dict):return {str(k):clean(v)for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [clean(v)for v in x]
    if isinstance(x,float)and not math.isfinite(x):return None
    return x
def read(path):return json.loads(Path(path).read_text())
def lines(path):return [json.loads(s)for s in Path(path).read_text().splitlines()if s.strip()]
def check(passed,**evidence):return {'passed':bool(passed),**evidence}
def array(rows,key,width=None):
    a=np.asarray([r.get(key)for r in rows],dtype=float)
    expected=(len(rows),)if width is None else(len(rows),width)
    if a.shape!=expected or not np.isfinite(a).all():raise ValueError(f'Invalid finite {key} shape: {a.shape}, expected {expected}')
    return a


def rpy_rotation(v):
    r,p,y=v;cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def pose(element):
    p=element.find('pose');v=np.fromstring(p.text,sep=' ')if p is not None else np.zeros(6)
    if v.shape!=(6,)or not np.isfinite(v).all():raise ValueError('Invalid SDF pose')
    return v[:3],rpy_rotation(v[3:])
def quaternion_rotations(q):
    if not np.allclose(np.sum(q*q,axis=1),1.,atol=1e-3):raise ValueError('Nonunit native base quaternion')
    w,x,y,z=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)
def axis_rotations(axis,angles):
    axis=axis/np.linalg.norm(axis);x,y,z=axis;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    c=np.cos(angles)[:,None,None];s=np.sin(angles)[:,None,None]
    return c*np.eye(3)+(1-c)*np.outer(axis,axis)+s*k


def foot_centers(world,steps,joint_order):
    """Reconstruct real spherical collision centers from measured q and base pose."""
    model=world.find("world/model[@name='go2']")
    if model is None:raise ValueError('go2 model missing')
    q=array(steps,'q',12);base=array(steps,'position',3);rot=quaternion_rotations(array(steps,'quaternion_wxyz',4))
    centers={};descriptions={}
    for leg,short in LEGS.items():
        p=base.copy();r=rot.copy();parent='base_link';chain=[]
        for part in ['hip','upper_leg','lower_leg']:
            name=f'{short}_{part}_joint';joint=model.find(f"joint[@name='{name}']")
            child=joint.findtext('child');jp=joint.find('pose')
            if joint.findtext('parent')!=parent or jp is None or jp.get('relative_to')!=parent:raise ValueError(f'Unsupported SDF joint frame {name}')
            xyz,rr=pose(joint);p+=np.einsum('nij,j->ni',r,xyz);r=r@rr
            axis=joint.find('axis/xyz')
            if axis.get('expressed_in'):raise ValueError('Nonlocal joint axis requires explicit frame resolution')
            r=r@axis_rotations(np.fromstring(axis.text,sep=' '),q[:,joint_order.index(name)])
            link=model.find(f"link[@name='{child}']");lp=link.find('pose')
            if lp is not None and lp.get('relative_to')!=name:raise ValueError(f'Unsupported child frame {child}')
            xyz,rr=pose(link);p+=np.einsum('nij,j->ni',r,xyz);r=r@rr;parent=child;chain.append(name)
        collisions=[c for c in link.findall('collision')if f'{short}_foot_link_collision' in c.get('name','')]
        if len(collisions)!=1:raise ValueError('Expected exactly one physical foot collision')
        col=collisions[0];radius=float(col.findtext('geometry/sphere/radius'))
        if col.find('pose')is not None and col.find('pose').get('relative_to'):raise ValueError('Nonlocal foot collision pose')
        xyz,_=pose(col);centers[leg]=p+np.einsum('nij,j->ni',r,xyz)
        descriptions[leg]={'collision':f'go2::{parent}::{col.get("name")}','radius_m':radius,'chain':chain}
    return centers,descriptions


def longest_span(mask,t,max_gap=.0075):
    best=0.;start=None;end=None;best_pair=None
    for i,valid in enumerate(mask):
        if valid:
            if start is None or(i and t[i]-t[i-1]>max_gap):start=float(t[i])
            end=float(t[i]);span=end-start
            if span>best:best=span;best_pair=[start,end]
        else:start=end=None
    return best,best_pair

def first_confirmation(mask,t,duration,max_gap):
    start=None
    for i,valid in enumerate(mask):
        if not valid:start=None;continue
        if start is None or(i and t[i]-t[i-1]>max_gap):start=float(t[i])
        if t[i]-start>=duration-1e-8:return float(t[i])
    return None


def evaluate(run):
    run=Path(run).resolve();checks={};metrics={};errors=[];name=run.name;legacy={};sources={}
    protocol=dict(PROTOCOL);frozen_protocol={};protocol_path=None
    try:
        protocol_path=run/'sources/tests/step_functional_protocol.json'
        if not protocol_path.exists():protocol_path=ROOT/'tests/step_functional_protocol.json'
        frozen_protocol=read(protocol_path);a=frozen_protocol['ascent'];co=frozen_protocol['continuation'];ti=frozen_protocol['timing'];sa=frozen_protocol['safety'];ss=frozen_protocol['stand_stop'];fi=frozen_protocol['fixture']
        protocol.update(duration_s=ti['end_s'],command_start_s=ti['command_start_s'],command_end_s=ti['forward_command_end_s'],stop_window_s=[ti['stand_stop_start_s'],ti['end_s']],
            command_vx_mps=frozen_protocol['command']['body_vx_mps'],minimum_continuous_foot_tread_support_s=a['each_foot_continuous_top_support_s'],
            body_entry_past_front_m=co['base_start_after_front_m'],minimum_continued_progress_m=co['min_forward_progress_m'],
            minimum_late_forward_velocity_mps=co['minimum_world_forward_mean_mps'],minimum_late_positive_velocity_fraction=co['minimum_positive_window_fraction'],
            max_abs_roll_pitch_rad=sa['max_abs_roll_pitch_rad'],min_support_clearance_m=sa['minimum_body_above_support_m'],
            stop_translation_drift_m=ss['translation_drift_m'],stop_yaw_drift_rad=ss['yaw_drift_rad'],stop_linear_rms_mps=ss['linear_rms_mps'],stop_yaw_rate_rms_radps=ss['yaw_rate_rms_radps'],
            contact_top_z_tolerance_m=a['top_contact_z_tolerance_m'],minimum_abs_contact_normal_z=a['minimum_abs_normal_z'],
            max_native_gap_s=a['max_native_sample_gap_s'],contact_xy_margin_m=a['contact_xy_margin_m'],continuation_min_duration_s=co['min_duration_s'],rolling_window_s=co['rolling_window_s'],maximum_unsupported_s=co['maximum_unsupported_s'])
        command_start=protocol['command_start_s'];command_end=protocol['command_end_s'];stop_start,stop_end=protocol['stop_window_s'];native_gap=protocol['max_native_gap_s']
        legacy=read(run/'summary.json')if(run/'summary.json').exists()else {}
        checks['frozen_native_interface']=check(legacy.get('levels',{}).get('interface')=='passed',
            interface_status=legacy.get('levels',{}).get('interface'),legacy_motion_not_used_as_functional_gate=True,
            interface_failed_checks=[k for k,v in legacy.get('iface_checks',{}).items()if not v.get('passed')])
        meta=read(run/'policy_manifest.json');name=meta['test'];runtime=read(run/'runtime_manifest.json');result=read(run/'worker_result.json')
        native=lines(run/'actuator.jsonl');steps=[r for r in native if r.get('kind')=='physics_step'];rows=lines(run/'telemetry.jsonl')
        contracts=[r for r in native if r.get('kind')=='actuator_contract'];contract=contracts[0]
        if name not in ['step05_continue','step10_continue']:raise ValueError('This evaluator only accepts independent continue fixtures')
        world=ET.parse(run/'world.sdf').getroot();platform=world.find("world/model[@name='teacher_low_step']")
        if platform is None:raise ValueError('Actual raised tread collision absent')
        pp,pr=pose(platform);link=platform.find('link');lp,lr=pose(link);col=link.find('collision');cp,cr=pose(col)
        center=pp+pr@lp+pr@lr@cp;rotation=pr@lr@cr;size=np.fromstring(col.findtext('geometry/box/size'),sep=' ')
        if not np.allclose(rotation,np.eye(3),atol=1e-10):raise ValueError('Fixture must be an axis-aligned raised tread')
        front,back=center[0]-size[0]/2,center[0]+size[0]/2;ymin,ymax=center[1]-size[1]/2,center[1]+size[1]/2;top=center[2]+size[2]/2
        tread_name=f'teacher_low_step::{link.get("name")}::{col.get("name")}'
        checks['fixture']=check(np.allclose([front,back,center[1],size[1],top],[fi['front_x_m'],fi['back_x_m'],fi['center_y_m'],fi['width_m'],fi['height_m'][0 if name.startswith('step05')else 1]],atol=1e-8),front_x_m=front,back_x_m=back,tread_y_limits_m=[ymin,ymax],top_z_m=top,collision=tread_name)
        offset=rows[0]['world_sim_time']-rows[0]['sim_time'];nt=array(steps,'t')-offset;t=array(rows,'sim_time');worldt=array(rows,'world_sim_time')
        pos=array(steps,'position',3);vel=array(rows,'measured',3);cmd=array(rows,'command',3);requested=array(rows,'requested',3);rpy=array(rows,'rpy',3)
        checks['complete_continuous_runtime']=check(len(contracts)==1 and result.get('completed')is True and not result.get('fault')and not runtime.get('error')and not any(r.get('kind')=='fault'for r in native)and not any(r.get('fault')for r in steps)and steps[-1].get('terminating')is True and all(p.get('returncode')==0 for p in runtime.get('owned_processes',[])[:2])and t[-1]>=protocol['duration_s']-.02 and np.all(np.diff(t)>0)and np.max(np.diff(t))<=1/frozen_protocol['policy_hz']+1e-8 and np.all(np.diff(nt)>0)and np.max(np.diff(nt))<=native_gap,
            duration_s=t[-1],policy_samples=len(rows),physics_samples=len(steps),native_termination=steps[-1].get('terminating'),worker_fault=result.get('fault'),runner_error=runtime.get('error'),main_exit_codes=[p.get('returncode')for p in runtime.get('owned_processes',[])[:2]])
        active=np.array([r.get('state')not in ['initializing','support_capture','support_hold']for r in rows])&(t>=.1)
        targets=array(rows,'q_target',12);actions=array(rows,'action',12);default=np.asarray(contract['initial_q'])
        execution_error=float(np.max(abs(targets[active]-(default+.25*actions[active]))))if active.any()else None
        npz=np.load(run/'observations_actions.npz');obs=npz['observations'];acts=npz['actions']
        checks['continuous_cpu_teacher']=check(meta.get('inference_device')=='cpu'and meta.get('checkpoint_sha256')=='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'and obs.shape==(len(rows),247)and acts.shape==(len(rows),12)and np.isfinite(obs).all()and np.isfinite(acts).all()and execution_error is not None and execution_error<1e-6 and not any(r.get('state')in ['support_capture','support_hold']for r in rows),target_action_max_error_rad=execution_error,bootstrap_excluded_s=.1,legacy_interface=legacy.get('levels',{}).get('interface'))
        before=(t>=command_start)&(t<command_end);stop=(t>=stop_start)&(t<=stop_end+1e-8)
        required_forward=[frozen_protocol['command'][k]for k in ['body_vx_mps','body_vy_mps','yaw_radps']]
        checks['command_profile']=check(before.sum()>=(command_end-command_start)*frozen_protocol['policy_hz']-2 and stop.sum()>=(stop_end-stop_start)*frozen_protocol['policy_hz']-2 and np.allclose(requested[before],required_forward,atol=1e-8)and np.max(abs(cmd[stop]))<1e-6,
            forward_interval_s=[command_start,command_end],stop_interval_s=[stop_start,stop_end],forward_samples=int(before.sum()),stop_samples=int(stop.sum()))
        centers,feet=foot_centers(world,steps,contract['joint_order']);support={};entry={}
        for group,(leg,short)in enumerate(LEGS.items(),1):
            c=centers[leg];foot=feet[leg];contact=[];top_contacts=[];missing_normals=0;missing_positions=0;fallback=[]
            for row in steps:
                pair=False;qualified=False;top_position=False
                for p in row.get('contact_pairs',[]):
                    if p.get('group')!=group or set([p.get('a'),p.get('b')])!=set([foot['collision'],tread_name])or p.get('points',0)<=0:continue
                    pair=True
                    points=np.asarray(p.get('positions_world',[]),dtype=float);normals=np.asarray(p.get('normals_world',[]),dtype=float)
                    if points.ndim!=2 or points.shape!=(p['points'],3)or not np.isfinite(points).all():missing_positions+=1;continue
                    margin=protocol['contact_xy_margin_m']
                    top_mask=(abs(points[:,2]-top)<=protocol['contact_top_z_tolerance_m'])&(points[:,0]>=front+margin)&(points[:,0]<=back-margin)&(points[:,1]>=ymin+margin)&(points[:,1]<=ymax-margin)
                    top_position=top_position or bool(top_mask.any())
                    if normals.shape!=points.shape or not np.isfinite(normals).all():
                        missing_normals+=1
                        continue
                    qualified=qualified or bool((top_mask&(abs(normals[:,2])>=protocol['minimum_abs_contact_normal_z'])).any())
                contact.append(pair);top_contacts.append(qualified);fallback.append(top_position)
            contact=np.asarray(contact);top_contacts=np.asarray(top_contacts)
            inside=(c[:,0]>=front)&(c[:,0]<=back)&(c[:,1]>=ymin)&(c[:,1]<=ymax)
            on_top=abs(c[:,2]-foot['radius_m']-top)<=protocol['foot_top_geometric_tolerance_m']
            supporting=top_contacts&inside&on_top;support[leg]=supporting
            crossings=np.flatnonzero((c[:-1,0]<front)&(c[1:,0]>=front)&(c[1:,0]>c[:-1,0]))+1
            entries=[int(i)for i in crossings if ymin<=c[i,1]<=ymax and nt[i]>=command_start and nt[i]<command_end]
            entered=np.zeros(len(nt),bool)
            if entries:entered[entries[0]:]=True
            ascent_active=(nt>=command_start)&(nt<command_end);ascent_support=supporting&entered&ascent_active
            span,span_times=longest_span(ascent_support,nt,native_gap)
            confirmation=first_confirmation(ascent_support,nt,protocol['minimum_continuous_foot_tread_support_s'],native_gap);entry[leg]=confirmation
            fallback_span,_=longest_span(np.asarray(fallback)&inside&on_top&entered&ascent_active,nt,native_gap)
            stop_native=(nt>=stop_start)&(nt<=stop_end+1e-8);stop_span,stop_span_times=longest_span(supporting&stop_native,nt,native_gap)
            foot.update(entry_time_s=float(nt[entries[0]])if entries else None,entry_center_world_m=c[entries[0]].tolist()if entries else None,
                        tread_contact_pair_samples=int(contact.sum()),verified_top_support_samples=int(supporting.sum()),max_continuous_tread_support_s=span,support_interval_s=span_times,
                        top_position_with_missing_normals_samples=missing_normals,position_and_collider_geometry_fallback_span_s=fallback_span,
                        tread_pair_missing_position_samples=missing_positions,
                        contact_normal_source='actual gz.msgs.Contact normal; missing normals never guessed; fallback diagnostic does not satisfy strict support',
                        first_active_support_confirmed_time_s=confirmation,
                        stop_tread_support_samples=int((supporting&stop_native).sum()),max_continuous_stop_tread_support_s=stop_span,stop_support_interval_s=stop_span_times)
            checks[f'{leg}_front_entry_and_real_tread_support']=check(bool(entries)and confirmation is not None and span>=protocol['minimum_continuous_foot_tread_support_s']-1e-8,**foot)
            checks[f'{leg}_stopped_on_real_tread']=check(stop_span>=protocol['minimum_continuous_foot_tread_support_s']-1e-8 and bool(supporting[-1]),**{k:v for k,v in foot.items()if k.startswith('stop')or k.startswith('max_continuous_stop')})
            checks[f'{leg}_contact_geometry_complete']=check(missing_positions==0 and missing_normals==0,missing_positions=missing_positions,missing_normals=missing_normals,raw_wrench_not_used_as_inferred_load=True)
        all_support=np.all(np.column_stack([support[k]for k in LEGS]),axis=1)
        simultaneous_span,simultaneous_interval=longest_span(all_support&(nt>=stop_start)&(nt<=stop_end+1e-8),nt,native_gap)
        simultaneous_required=ss.get('simultaneous_four_foot_top_support_s',protocol['minimum_continuous_foot_tread_support_s'])
        checks['four_feet_simultaneous_top_stop_support']=check(simultaneous_span>=simultaneous_required-1e-8 and bool(all_support[-1]),continuous_span_s=simultaneous_span,required_span_s=simultaneous_required,interval_s=simultaneous_interval,final_sample_all_four_on_top=bool(all_support[-1]))
        # At most a 1-step cached-state phase difference; all positions are native measured states.
        drive=(nt>=command_start)&(nt<command_end);all_confirmed=max(entry.values())if all(v is not None for v in entry.values())else None
        past=np.flatnonzero(drive&(pos[:,0]>=front+protocol['body_entry_past_front_m'])&(pos[:,1]>=ymin)&(pos[:,1]<=ymax)&(nt>=all_confirmed))if all_confirmed is not None else []
        late_start=None;progress=None;late_v=None;fraction=None;platform_ratio=None;continued_duration=None;unsupported_duration=None;rolling_count=0
        world_v=np.einsum('nij,nj->ni',quaternion_rotations(array(steps,'quaternion_wxyz',4)),array(steps,'body_lin_vel_com',3))
        if len(past):
            begin=past[0];end=np.flatnonzero(drive)[-1];progress=float(pos[end,0]-pos[begin,0]);late_start=float(nt[begin]);continued_duration=float(nt[end]-nt[begin])
            continuation=drive&(nt>=nt[begin]);platform_ratio=float(((pos[continuation,0]>=front)&(pos[continuation,0]<=back)&(pos[continuation,1]>=ymin)&(pos[continuation,1]<=ymax)).mean())
            window=max(1,round(protocol['rolling_window_s']/frozen_protocol['physics_dt_s']));vv=world_v[continuation,0]
            rolling=np.convolve(vv,np.ones(window)/window,mode='valid')if len(vv)>=window else []
            rolling_count=len(rolling);late_v=float(np.mean(vv))if len(vv)else None;fraction=float(np.mean(np.asarray(rolling)>protocol['minimum_late_forward_velocity_mps']))if len(rolling)else None
            supported_any=np.any(np.column_stack([support[k]for k in LEGS]),axis=1)
            unsupported_duration,_=longest_span((~supported_any)&continuation,nt,native_gap)
        metrics['continued_motion']={'edge_plus_half_m_first_time_s':float(nt[past[0]])if len(past)else None,'all_four_feet_active_ascent_confirmed_time_s':all_confirmed,'continued_x_progress_m':progress,'late_interval_s':[late_start,command_end],
             'late_forward_velocity_mean_mps':late_v,'late_positive_velocity_fraction':fraction,'on_platform_body_samples_fraction':platform_ratio,
             'continued_duration_s':continued_duration,'rolling_velocity_windows':rolling_count,'rolling_window_s':protocol['rolling_window_s'],'maximum_no_foot_top_support_s':unsupported_duration,
             'positive_fraction_definition':'fraction of all 0.5 s rolling world-frame COM vx means above protocol minimum; mean of entire continuation is diagnostic',
             'measured_position_source':'native Gazebo state, diagnostic simulation evaluation only'}
        checks['continued_teacher_forward_on_tread']=check(progress is not None and progress>=protocol['minimum_continued_progress_m']and continued_duration>=protocol['continuation_min_duration_s']and fraction is not None and fraction>=protocol['minimum_late_positive_velocity_fraction']and platform_ratio==1. and unsupported_duration<=protocol['maximum_unsupported_s'],**metrics['continued_motion'])
        # Recompute base clearance from actual supporting collision surface below it,
        # independent of world-Z gain and never using an overhead ray origin.
        sys.path.insert(0,str(run/'sources/policy')if(run/'sources/policy/observation.py').exists()else str(ROOT/'policy'))
        from observation import TerrainHeightMap
        terrain=TerrainHeightMap.from_sdf(run/'world.sdf');ground=terrain.height(pos[:,:2],ray_start_z=pos[:,2]);clear=pos[:,2]-ground
        safety_start=ti['bootstrap_pd_s'];safety_mask=nt>=safety_start;contacts=np.asarray([r.get('contacts')for r in steps],dtype=float)
        w,x,y,z=array(steps,'quaternion_wxyz',4).T
        native_attitude=np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1))])
        max_attitude=float(abs(native_attitude[safety_mask]).max());minimum_clear=float(clear[safety_mask].min())
        metrics['safety']={'max_abs_roll_pitch_rad':max_attitude,'min_support_clearance_m':minimum_clear,'body_contact_samples':int((contacts[safety_mask,0]>0).sum()),
            'unknown_contact_samples':int((contacts[safety_mask]<0).sum()),'safety_start_s':safety_start,'attitude_source':'200 Hz native quaternion',
            'clearance_definition':'base-link origin world z minus actual collision surface directly below base; safety only'}
        checks['safety']=check(max_attitude<=protocol['max_abs_roll_pitch_rad']and minimum_clear>=protocol['min_support_clearance_m']and not(contacts[safety_mask,0]>0).any()and not(contacts[safety_mask]<0).any(),**metrics['safety'])
        tp=array(rows,'position',3);yaw=np.unwrap(rpy[:,2]);stop_pos=tp[stop];stop_yaw=yaw[stop]
        drift=float(np.linalg.norm(stop_pos[:,:2]-stop_pos[0,:2],axis=1).max())if len(stop_pos)else None;yaw_drift=float(abs(stop_yaw-stop_yaw[0]).max())if len(stop_yaw)else None
        rms=np.sqrt(np.mean(vel[stop]**2,axis=0))if stop.any()else None
        metrics['stop']={'window_s':[stop_start,stop_end],'translation_drift_m':drift,'yaw_drift_rad':yaw_drift,'velocity_rms':rms,'samples':int(stop.sum()),'strategy':'continuous Teacher at zero velocity command'}
        checks['teacher_zero_command_stop']=check(drift is not None and drift<=protocol['stop_translation_drift_m']and yaw_drift<=protocol['stop_yaw_drift_rad']and max(rms[:2])<=protocol['stop_linear_rms_mps']and rms[2]<=protocol['stop_yaw_rate_rms_radps']and np.max(abs(cmd[stop]))<1e-6,**metrics['stop'])
        metrics['feet']=feet;initial=(t>=2)&(t<=3);ending=stop
        metrics['body_world_height_gain_diagnostic_m']=float(np.median(tp[ending,2])-np.median(tp[initial,2]))if ending.any()and initial.any()else None
        metrics['body_world_height_gain_used_for_pass']=False
        metrics['command_timing']={'duration_s':float(t[-1]),'forward_end_s':command_end,'stop_evaluation_start_s':stop_start}
        np.savez_compressed(run/'functional_feet.npz',sim_time=nt,base_position=pos,support_clearance=clear,
                            **{f'{leg}_collision_center':centers[leg]for leg in LEGS},**{f'{leg}_real_tread_support':support[leg]for leg in LEGS})
        sources={str(p.relative_to(run)):digest(p)for p in [run/'world.sdf',run/'telemetry.jsonl',run/'actuator.jsonl',run/'runtime_manifest.json',run/'policy_manifest.json',run/'source_manifest.json']}
    except Exception as error:errors.append(f'{type(error).__name__}: {error}');checks['evidence_readable']=check(False,error=errors[-1])
    failed=[k for k,v in checks.items()if not v['passed']];status='passed'if checks and not failed and not errors else 'failed'
    reasons=failed+errors;summary={'schema_version':1,'test':name,'status':status,'functional_status':status,
        'levels':{'interface':legacy.get('levels',{}).get('interface','unverified'),'functional_step':status,'sim2sim':'unverified','navigation':'unverified','real_robot':'unverified'},
        'tests':[{'name':name,'status':status,'reason':'; '.join(reasons),'metrics':metrics}], 'checks':checks,'metrics':metrics,
        'protocol':frozen_protocol,'criteria_flattened':protocol,'protocol_source':str(protocol_path),'protocol_sha256':digest(protocol_path)if protocol_path else None,
        'evaluator_sha256':digest(__file__),'source_hashes':sources,'legacy_summary':{'levels':legacy.get('levels'),'tests':legacy.get('tests')},
        'scope':'Independent extended-tread functional retest; body height gain is diagnostic, not an acceptance gate; privileged evaluation is not SLAM navigation proof'}
    summary=clean(summary);(run/'summary_functional.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n');return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',type=Path,nargs='+');args=p.parse_args()
    for run in args.runs:
        s=evaluate(run);print(json.dumps({'run':str(run),'test':s['test'],'status':s['status'],'reason':s['tests'][0]['reason']},ensure_ascii=False),flush=True)
if __name__=='__main__':main()
