"""Typed simulation speed → fixed dynamic encounter lead; no robot control.

The original selector preserves its 16–18 s nominal lead. Explicit R128 uses
observed approach calibration without a nominal timing or stopping guarantee.
Obstacle motion, source freshness and all collision gates stay original.
"""
import hashlib
import json
from pathlib import Path
import r109_speed_tiers as speed

KEY = 'r117_dynamic_speed_lead'
SCHEMA = 'r117_typed_dynamic_cruise_lead/v1'

# Explicit input-timing experiment; neither robot speed nor a stopping bound.
APPROACH_KEY = 'r128_observed_approach_trigger'
APPROACH_SCHEMA = 'r128_observed_approach_trigger/v1'
APPROACH_RUN = '20261024T081822Z_closed_loop_R85_R125_B10_B05_business_control_round27_f019'


def _observed_approach(profile):
    selected = speed.configuration(profile)
    if selected is None or selected['cruise_mps'] != .5:
        raise ValueError('r128_observed_approach_requires_original_typed_point5')
    expected=dict(schema=APPROACH_SCHEMA,enabled=True,lead_window_m=[2.4,2.7])
    if profile[APPROACH_KEY]!=expected:raise ValueError('Dynamic approach configuration invalid')
    return dict(schema=APPROACH_SCHEMA, speed_tier=dict(selected),
        lead_window_m=[2.4, 2.7], original_nominal_lead_window_m=[8., 9.],
        obstacle_enter_s=8.4, physical_stopping_bound_claimed=False,
        nominal_cruise_16_to_18s_claimed=False,
        observation=dict(run_id=APPROACH_RUN,
            measured_arc_source_interval_ns=[115965000000, 152365000000],
            measured_mean_arc_mps=.0936029544929574,
            local_trailing_fit_source_ns=174805000000,
            local_trailing_8s_arc_mps=.1395287052757022,
            geometry_hypothesis_is_actual_LiDAR_or_braking_proof=False),
        selected_external_profile=dict(expected))


def configuration(profile):
    maximum = profile.get('max_speed_mps')
    if APPROACH_KEY in profile:
        return _observed_approach(profile)
    if speed.KEY in profile:
        selected = speed.configuration(profile)
        if selected['cruise_mps'] == .5:
            return dict(schema=SCHEMA, speed_tier=dict(selected),
                original_cruise_mps=.2, original_lead_window_m=[3.2,3.6],
                nominal_cruise_lead_s=[16.,18.], lead_window_m=[8.,9.],
                obstacle_enter_s=8.4, physical_stopping_bound_claimed=False)
    # Same original numeric predicate and error for absent/0.2 selection.
    if type(maximum) not in (int,float) or not 0<float(maximum)<=.2:
        raise ValueError('fixed_dynamic_lead_budget_requires_original_max_speed_at_most_point2')
    return None


def lead_window(setting):
    return (3.2,3.6) if setting is None else tuple(setting['lead_window_m'])


def validate_contract(contract, profile=None):
    """Caller supplies actual scoped profile; contract cannot select its budget."""
    setting = None if profile is None else configuration(profile)
    expected = list(lead_window(setting))
    trigger = contract.get('trigger', {})
    if setting is None:
        if KEY in contract:
            raise ValueError('dynamic_speed_lead_requires_matching_external_typed_profile')
        return (3.2,3.6)  # Original verifier never trusted the declared window.
    elif json.dumps(contract.get(KEY),sort_keys=True,allow_nan=False) != json.dumps(setting,sort_keys=True,allow_nan=False):
        raise ValueError('dynamic_speed_lead_contract_differs_from_external_typed_profile')
    if trigger.get('lead_window_m') != expected or any(type(v)is not float for v in trigger.get('lead_window_m', [])):
        raise ValueError('dynamic_trigger_lead_window_differs_from_selected_speed')
    return lead_window(setting)
