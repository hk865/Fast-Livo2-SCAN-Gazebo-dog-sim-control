"""Excluded geometry hypothesis; never imported by production NAV.

Returns a heading reference/diagnostic only. No velocity, ROS, GT, guard override,
arrival or task completion. A future caller must still use all existing guards
and the original pre-turn/settle state machine. Values below are experimental.
"""
import math


def angle(value):
    return math.atan2(math.sin(value),math.cos(value))


class BoundedTurnReference:
    def __init__(self, reference_key, seed_heading, seed_stamp_ns, *,
                 rate_rad_s=.06, max_offset_rad=.30,
                 persistence_ns=350_000_000, coherence_rad=.05,
                 max_observation_gap_ns=200_000_000):
        if not isinstance(reference_key,tuple) or len(reference_key)!=4:
            raise ValueError('key must bind request, goal, trajectory, reference stamp')
        if any(not math.isfinite(v) or v<=0 for v in (rate_rad_s,max_offset_rad,coherence_rad)):
            raise ValueError('finite positive experimental bounds required')
        if not math.isfinite(seed_heading) or type(seed_stamp_ns) is not int:
            raise ValueError('finite heading and native integer stamp required')
        if persistence_ns<=0 or max_observation_gap_ns<=0:
            raise ValueError('positive observation bounds required')
        self.key=reference_key;self.seed=angle(seed_heading);self.reference=self.seed
        self.last_stamp_ns=seed_stamp_ns;self.stable_sign=0;self.stable_since_ns=None
        self.rate=rate_rad_s;self.max_offset=max_offset_rad;self.persistence_ns=persistence_ns
        self.coherence=coherence_rad;self.max_gap_ns=max_observation_gap_ns

    def snapshot(self, reason, current_heading=None, yaw=None, eligible=False):
        current_error=None if current_heading is None or yaw is None else angle(current_heading-yaw)
        reference_error=None if yaw is None else angle(self.reference-yaw)
        return dict(reason=reason,reference_heading=self.reference,
            reference_offset=angle(self.reference-self.seed),
            reference_error=reference_error,current_path_error=current_error,
            # Original pure-turn .10 threshold; current route's original .20
            # gate is an additional condition, never a looser completion bound.
            can_start_original_settle=bool(eligible and abs(reference_error)<.10 and abs(current_error)<=.20),
            needs_fresh_plan=reason in {'reference_identity_changed','direction_outside_bound'},
            caller_must_hold=reason in {'invalid_input','protected_or_not_fresh','observation_gap',
                                       'reference_identity_changed','direction_outside_bound'})

    def update(self, stamp_ns, current_heading, yaw, reference_key, *, protected=False, fresh=True):
        if type(stamp_ns) is not int or not all(math.isfinite(x) for x in (current_heading,yaw)):
            return self.snapshot('invalid_input')
        if reference_key!=self.key:
            return self.snapshot('reference_identity_changed',current_heading,yaw)
        if protected or not fresh:
            self.stable_sign=0;self.stable_since_ns=None
            return self.snapshot('protected_or_not_fresh',current_heading,yaw)
        if stamp_ns<=self.last_stamp_ns:
            return self.snapshot('no_new_native_odom',current_heading,yaw)
        elapsed_ns=stamp_ns-self.last_stamp_ns;self.last_stamp_ns=stamp_ns
        if elapsed_ns>self.max_gap_ns:
            self.stable_sign=0;self.stable_since_ns=None
            return self.snapshot('observation_gap',current_heading,yaw)
        offset=angle(current_heading-self.seed)
        if abs(offset)>self.max_offset:
            self.stable_sign=0;self.stable_since_ns=None
            return self.snapshot('direction_outside_bound',current_heading,yaw)
        allowed=False
        if abs(offset)<=self.coherence:
            # Return toward initial reference gradually; alternating small sway
            # cannot establish an accumulating steering correction.
            self.stable_sign=0;self.stable_since_ns=None;desired_offset=0.;allowed=True
        else:
            sign=1 if offset>0 else -1
            if sign!=self.stable_sign:
                self.stable_sign=sign;self.stable_since_ns=stamp_ns
            desired_offset=offset
            allowed=stamp_ns-self.stable_since_ns>=self.persistence_ns
        if allowed:
            limit=self.rate*elapsed_ns/1e9
            error=angle(self.seed+desired_offset-self.reference)
            self.reference=angle(self.reference+max(-limit,min(limit,error)))
        return self.snapshot('bounded_update' if allowed else 'awaiting_direction_consistency',
                             current_heading,yaw,eligible=True)
