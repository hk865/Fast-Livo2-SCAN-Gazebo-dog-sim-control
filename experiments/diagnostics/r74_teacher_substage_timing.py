"""Explicit default-off R74 timing opt-in for the isolated minimal simulation."""
KEY = 'r74_teacher_substage_timing'
SCHEMA = 'r74_bounded_teacher_substage_timing/v1'


def enabled(profile):
    if type(profile) is not dict:
        raise ValueError('R74 timing profile must be a JSON object')
    if KEY not in profile:
        return False
    value = profile[KEY]
    if (type(value) is not dict or set(value) != {'enabled', 'schema'}
            or type(value['enabled']) is not bool
            or type(value['schema']) is not str or value['schema'] != SCHEMA):
        raise ValueError('Exact typed R74 bounded timing option required')
    from experiments.candidates.minimal_simulation_baseline import enabled as minimal_enabled
    if not minimal_enabled(profile):
        raise ValueError('R74 timing option requires the isolated minimal simulation opt-in')
    return value['enabled']
