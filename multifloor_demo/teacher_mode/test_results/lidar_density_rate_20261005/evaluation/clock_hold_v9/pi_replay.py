"""Original frozen independent PI equations; explicit hold events are added.
No controller core module is imported or called. Common helpers retain their
original numerical meanings and are supplied from the frozen run evaluator.
"""
import math
import numpy as np

def ack_and_PI_hold_audit(run,pid,poses,execution,profile,holds):
    """Independent command-ack joins and PI recurrence, not old PID terms."""
    ackrows=rows(run/'closed_loop_executor_ack.jsonl');ai=original_index(ackrows)
    byseq={r['sequence']:r for r in ackrows};pi=original_index(poses)
    frames={round(r['world_sim_time']*1e9):r for r in execution}
    errors=[];previous=-1;lastgoal=None;previous_pose=None
    integral=np.zeros(3);hold_integral=np.zeros(3);filtered=None;inner=None
    offset=finite(profile['cascade']['base_com_offset'],3)
    reset_count={};updates=0;blocked_limit=0;blocked_ack=0;maximum=0.
    for ack in ackrows:
        stamp=ns(ack['stamp_ns']);seq=ns(ack['sequence']);confirmation=ns(ack['ack_native_clock_ns'])
        original=frames.get(stamp)
        if seq<=previous or confirmation<=stamp:errors.append([seq,'Ack identity/next native request is invalid'])
        previous=seq
        if original is None:errors.append([seq,'Ack lacks actual Teacher frame']);continue
        if not np.allclose(ack['applied_command_body'],original['command'],atol=1e-12,rtol=0) or not np.allclose(ack['requested_command_body'],original['requested'],atol=1e-12,rtol=0):errors.append([seq,'Ack changed real requested/applied Teacher input'])
        if ack.get('contains_position_or_attitude') is not False or ack.get('navigation_ground_truth_used') is not False:errors.append([seq,'Known-command ack contains truth/navigation state'])
    events=sorted([(r['compute_monotonic_wall'],0,'pid',r)for r in pid]+[(r['compute_monotonic_wall'],1,'hold',r)for r in holds],key=lambda x:(x[0],x[1]))
    for _,_,event_type,r in events:
        if event_type=='hold':
            before=r['frozen_control_before'];after=r['frozen_control_after']
            for key,own in [('velocity_integral',integral),('hold_integral',hold_integral)]:
                if before.get(key)is not None:
                    maximum=max(maximum,float(np.max(abs(np.asarray(before[key])-own))))
            if before.get('math_pose_stamp_ns')!=previous_pose:
                errors.append(['hold',r['sequence'],'Before-hold math source differs from independently replayed state'])
            if r['protected_or_stale_reset']:
                integral[:]=0;hold_integral[:]=0;filtered=inner=None;previous_pose=None
            for key,own in [('velocity_integral',integral),('hold_integral',hold_integral)]:
                if after.get(key)is not None:
                    maximum=max(maximum,float(np.max(abs(np.asarray(after[key])-own))))
            if after.get('math_pose_stamp_ns')!=previous_pose:
                errors.append(['hold',r['sequence'],'After-hold math source differs from unchanged/protected replay state'])
            continue
        c=r.get('cascade',{});mode=c.get('mode',r.get('mode'));goal=c.get('fixed_goal_sha256');seq=r['sequence']
        if goal!=lastgoal:
            integral[:]=0;hold_integral[:]=0;filtered=inner=None;previous_pose=None;lastgoal=goal
        if c.get('protection_active') or mode in('protect','recovering'):
            integral[:]=0;hold_integral[:]=0;filtered=inner=None;previous_pose=None
            if np.any(finite(c.get('command_body',[0,0,0]),3)):errors.append([seq,'Protected core requests nonzero correction'])
            continue
        if c.get('controller_updated') is not True:continue
        updates+=1;source=pi.get(c.get('feedback_pose_stamp_ns'))
        if source is None:errors.append([seq,'Math source missing']);continue
        pns=source['stamp_ns'];dt=0. if previous_pose is None else (pns-previous_pose)/1e9
        if dt<0 or abs(dt-c['header_dt_s'])>1e-9:errors.append([seq,'PI dt is not new original source gap'])
        previous_pose=pns
        R=rotation(np.asarray(source['quaternion'])[[3,0,1,2]][None])[0];rp=angles(np.asarray(source['quaternion'])[[3,0,1,2]][None])[0]
        bodygyro=finite(r['imu']['angular_velocity_body'],3);origin=finite(source['body_velocity'],3);com=origin+np.cross(bodygyro,offset)
        yawdot=(math.sin(rp[0])*bodygyro[1]+math.cos(rp[0])*bodygyro[2])/math.cos(rp[1]);measurement=np.r_[ (R@origin)[:2],yawdot];actual=np.r_[com[:2],bodygyro[2]]
        filtered=measurement.copy() if filtered is None else filtered+dt/(.2+dt)*(measurement-filtered)
        inner=actual.copy() if inner is None else inner+dt/(.1+dt)*(actual-inner)
        vpi=c.get('velocity_PI')
        if not isinstance(vpi,dict):raise MissingEvidence('Actual new velocity_PI recurrence absent')
        ack=byseq.get(vpi['ack_sequence'])
        if ack is None:raise MissingEvidence('Exact original executed command ack absent')
        if ack['stamp_ns']>pns or pns-ack['stamp_ns']>300_000_000 or ack['received_wall_ns']>c['compute_wall_ns'] or c['compute_wall_ns']-ack['received_wall_ns']>300_000_000:errors.append([seq,'Executed command ack future/stale to actual source'])
        residual=finite(ack['requested_command_body'],3)-finite(ack['applied_command_body'],3)
        own=hold_integral if mode in('capture','active_hold') else integral
        if c.get('capture_entry_integral_reset'):
            if mode!='capture':errors.append([seq,'Unexpected integral reset outside capture entry'])
            own[:]=0;reset_count[goal]=reset_count.get(goal,0)+1
        if mode in('capture','active_hold'):
            limits=np.array([.15,.07,.10] if mode=='capture' else [.06,.04,.10])
            fixed=c['fixed_goal'];goalxy=np.asarray(fixed['position_world_xyz'])[:2]
            e=goalxy-np.asarray(source['position'])[:2];P=(.8 if mode=='capture' else .18)*np.sign(e)*np.maximum(abs(e)-.003,0.)
            D=-.2*filtered[:2];worldxy=P+D;cap=.05 if mode=='capture' else .025
            if np.linalg.norm(worldxy)>cap:worldxy*=cap/np.linalg.norm(worldxy)
            refworld=np.r_[worldxy,0.];yawerror=float(wrap(fixed['heading_rad']-rp[2]));yp=.65*math.copysign(max(abs(yawerror)-.005,0.),yawerror);yd=-.18*filtered[2];wref=float(np.clip(yp+yd,-.07,.07))
            if not np.allclose(c['position_terms']['P'],P,atol=1e-9,rtol=0) or not np.allclose(c['position_terms']['D'],D,atol=1e-9,rtol=0):errors.append([seq,'Fixed-target phase PD differs from prospective capture/hold law'])
        else:
            limits=finite(profile['cascade']['command_limits'],3);tangent=finite(c['local_horizontal_tangent'],2);normal=np.array([-tangent[1],tangent[0]])
            if abs(np.linalg.norm(tangent)-1)>1e-8:errors.append([seq,'Invalid actual SCAN tangent'])
            remaining=float(c['remaining_horizontal_arc_m']);speed=min(float(profile['cascade']['desired_speed']),.9*max(remaining,0.),math.sqrt(.7*max(remaining-.05,0.)))
            lat=np.clip(-.8*c['error_cross_m']-.2*(filtered[:2]@normal),-.2,.2)
            refworld=np.r_[speed*tangent+lat*normal,speed*c['local_grade_dz_ds']]
            if mode!='drive':refworld[:]=0.
            yp=1.3*float(wrap(c['reference_yaw_rad']-rp[2]));yd=-.15*filtered[2];wref=yp+yd
        body_wref=(wref*math.cos(rp[1])-math.sin(rp[0])*bodygyro[1])/math.cos(rp[0])
        target=np.r_[(R.T@refworld+np.cross([0,0,body_wref],offset))[:2],body_wref]
        error=target-inner;active_dt=dt if mode in('drive','turn','capture','active_hold') else 0.
        candidate=np.clip(own+active_dt*error,-.5,.5)
        if mode=='turn':candidate[:2]=own[:2]
        kp=np.array([.6,.4,.3]);ki=np.array([.5,.3,.2]);raw=target+kp*error+ki*candidate;limited=bounded_command(raw,limits,mode=='drive')
        block_limits=error*(raw-limited)>1e-9;block_ack=error*residual>1e-9
        candidate=np.where(block_limits|block_ack,own,candidate);raw=target+kp*error+ki*candidate;limited=bounded_command(raw,limits,mode=='drive')
        own[:]=candidate
        pairs=[(c['measured_COM_velocity_body'],com),(c['measured_origin_velocity_world'],R@origin),(vpi['filtered_actual_body'],inner),
               (vpi['error'],error),(vpi['P'],kp*error),(vpi['I'],ki*candidate),(vpi['integral_state'],candidate),
               (vpi['acknowledged_downstream_residual'],residual),(c['raw_command_body'],raw),
               (c['command_limits_body'],limits)]
        for observed,expected in pairs:maximum=max(maximum,float(np.max(abs(np.asarray(observed)-expected))))
        if not np.array_equal(vpi['blocked_limits'],block_limits) or not np.array_equal(vpi['blocked_actual_ack'],block_ack):errors.append([seq,'PI anti-windup blocked flags differ'])
        if abs(active_dt-c['integral_dt_s'])>1e-9:errors.append([seq,'Held/non-drive integration dt differs'])
        if mode=='turn':limited[:2]=0.
        elif mode in('pre_turn','settle','path_end_hold'):limited[:]=0.
        maximum=max(maximum,float(np.max(abs(np.asarray(c['command_body'])-limited))))
        if mode in('drive','turn','capture','active_hold'):
            maximum=max(maximum,float(np.max(abs(np.asarray(c['reference_COM_velocity_body'])-target))))
        blocked_limit+=int(block_limits.sum());blocked_ack+=int(block_ack.sum())
    if maximum>1e-8:errors.append(['maximum mathematical replay error',maximum])
    if any(v!=1 for v in reset_count.values()):errors.append(['Capture integral resets',reset_count])
    return check(bool(updates) and bool(ackrows) and not errors,actual_original_acks=len(ackrows),
                 actual_fresh_math_updates=updates,maximum_replay_absolute_error=maximum,errors=errors,
                 capture_entry_integral_resets=reset_count,PI_axis_blocked_count=blocked_limit,PI_actual_ack_blocked_count=blocked_ack,
                 explicit_hold_events_replayed=len(holds),
                 outer_projection_independence='Projection geometry requires separate original SCAN path-record replay',
                 actual_ack_is_known_command_not_truth_feedback=True)
