"""Unselected pure Student-yaw P/D fade candidate; no actuator or stop authority.

Consume only the in-process pre_zero_unload object already authorized by the
original NativeUnload/prepare_command path. Event hashes are identity, not proof.
This module does not create events, ACKs, publication receipts or source clocks.
"""
import copy
import math

PROFILE_KEY='r85_student_yaw_pd_fade'
CONFIG={'enabled':True,'kind':'accepted_pre_zero_event_yaw_PD_fade_v1','maximum_alpha_rate_per_s':1.0}
SCHEMA='student_yaw_PD_fade_candidate/v1'

def _number(v):
    return type(v)in(int,float)and math.isfinite(v)

def configuration(profile,unload_helper):
    if not isinstance(profile,dict):raise ValueError('Profile dictionary required')
    option=profile.get(PROFILE_KEY)
    if option is None:return None
    if isinstance(option,dict)and set(option)=={'enabled'}and option['enabled']is False:return None
    if (not isinstance(option,dict)or set(option)!=set(CONFIG)or option['enabled']is not True
            or option['kind']!=CONFIG['kind']or not _number(option['maximum_alpha_rate_per_s'])
            or option['maximum_alpha_rate_per_s']!=1.):
        raise ValueError('Exact explicit yaw P/D fade configuration required')
    if unload_helper.configuration(profile)is None:
        raise ValueError('Existing explicit coordinated pre-zero unload required')
    return copy.deepcopy(CONFIG)

class StudentYawPDFade:
    """Only owns alpha in [0,1]. Never receives or mutates PID I/D states.

    Active current owned event -> alpha toward zero, otherwise toward one.
    A valid pre-expiry source arriving after native expiry is restore-only;
    it is not a new worker fault or a deadline extension. Invalid authorization
    raises before mutation; caller must retain original zero/protection behavior.
    """
    def __init__(self,config,unload_helper=None,run_id=None,scope_sha256=None):
        if config is not None and (not isinstance(config,dict)or set(config)!=set(CONFIG)
                or config['enabled']is not True or config['kind']!=CONFIG['kind']
                or not _number(config['maximum_alpha_rate_per_s'])
                or config['maximum_alpha_rate_per_s']!=1.):raise ValueError('Configuration differs')
        if config is not None and (not isinstance(run_id,str)or not run_id
               ):
            raise ValueError('Bound run and original scope required')
        self.config=copy.deepcopy(config);self.helper=unload_helper
        self.run_id=run_id;self.scope=scope_sha256;self.alpha=1.
        self.last_clock=None;self.last_frame=None;self.last_diagnostic=None

    def step(self,dt_s,input_clock_ns,input_frame_serial,motion_epoch,
             authoritative_zero,pre_zero_unload=None):
        if self.config is None:
            return {'schema':SCHEMA,'enabled':False,'alpha':1.,'new_control_math':False}
        if (not _number(dt_s)or not .019999<=dt_s<=.020001
                or type(input_clock_ns)is not int or input_clock_ns<0
                or type(input_frame_serial)is not int or input_frame_serial<=0
                or type(authoritative_zero)is not bool):
            raise ValueError('Original typed fresh 50Hz input required')
        if self.last_clock is not None and (input_clock_ns-self.last_clock!=20_000_000
                or input_frame_serial-self.last_frame!=1):
            raise ValueError('Repeated/gapped native input cannot advance fade')
        event=None;source_clock=None;active=False
        reason='original_exact_zero_priority_restore'if authoritative_zero else'no_owned_event_restore'
        # Original exact-zero output/reset takes priority, without event credit.
        if not authoritative_zero and pre_zero_unload is not None:
            q=pre_zero_unload
            if (not isinstance(q,dict)or set(q)!={'helper','authorization','input_clock_ns','input_frame_serial'}
                    or q['helper']is not self.helper or q['input_clock_ns']!=input_clock_ns
                    or q['input_frame_serial']!=input_frame_serial):
                raise ValueError('Original internal unload/frame binding differs')
            a=q['authorization']
            if not isinstance(a,dict)or set(a)!={'event','path_sha256','goal_sha256','source_control_clock_ns'}:
                raise ValueError('Original authorization shape differs')
            source_clock=a['source_control_clock_ns']
            if type(source_clock)is not int or not 0<=input_clock_ns-source_clock<=300_000_000:
                raise ValueError('Accepted source clock missing, future or stale')
            event=self.helper.validate_event(a['event'],self.run_id,self.scope,source_clock)
            c=event['context'];expected_epoch=c['request_id']+':waypoint:'+str(c['waypoint_index'])
            if (motion_epoch!=expected_epoch or not self.helper.valid_sha(a['path_sha256'])
                    or a['goal_sha256']!=event['goal_sha256']):
                raise ValueError('Accepted event epoch/path/goal differs')
            active=event['brake_entry_clock_ns']<=input_clock_ns<event['expires_clock_ns']
            reason='active_owned_event_fade'if active else'accepted_late_source_restore_only'
        before=self.alpha;target=0. if active else 1.
        maximum_change=self.config['maximum_alpha_rate_per_s']*dt_s
        after=max(target,before-maximum_change)if before>target else min(target,before+maximum_change)
        assert 0.<=after<=1.
        diagnostic=dict(schema=SCHEMA,enabled=True,new_control_math=True,
            alpha_before=before,alpha=after,target_alpha=target,maximum_alpha_rate_per_s=1.,dt_s=dt_s,
            active_current_owned_event=active,reason=reason,
            event_id=None if event is None else event['event_id'],
            event_expires_clock_ns=None if event is None else event['expires_clock_ns'],
            event_validated_at_source_clock=event is not None,source_control_clock_ns=source_clock,
            run_id=self.run_id,scope_sha256=self.scope,motion_epoch=motion_epoch,
            input_clock_ns=input_clock_ns,input_frame_serial=input_frame_serial,
            original_exact_zero_priority=authoritative_zero,
            PID_I_or_D_state_modified_by_helper=False,stop_or_ACK_credit=False)
        self.alpha=after;self.last_clock=input_clock_ns;self.last_frame=input_frame_serial
        self.last_diagnostic=diagnostic
        return copy.deepcopy(diagnostic)

    def gain_vectors(self,kp,kd):
        if self.config is None:return kp,kd
        if (len(kp)!=3 or len(kd)!=3 or float(kp[2])!=.5 or float(kd[2])!=.015
                or not all(math.isfinite(float(v))for v in list(kp)+list(kd))):
            raise ValueError('Original Student yaw .5/.015 gains required; do not compose gain candidates')
        p=list(kp);d=list(kd);p[2]=.5*self.alpha;d[2]=.015*self.alpha
        return p,d
