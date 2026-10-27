"""Diagnostic opt-in for latest queued corrected metadata; no control authority."""

KEY = 'r74_corrected_metadata_latest_only'
SCHEMA = 'r74_corrected_metadata_keep_last1_delivery/v1'


def subscription_depth(profile):
    """Preserve original integer depth10 unless the strict diagnostic option selects1."""
    if type(profile) is not dict:
        raise ValueError('Corrected metadata QoS profile must be an object')
    if KEY not in profile:
        return 10
    option = profile[KEY]
    if (type(option) is not dict or set(option) != {'enabled', 'schema'} or
            type(option['enabled']) is not bool or type(option['schema']) is not str or
            option['schema'] != SCHEMA):
        raise ValueError('Exact typed latest corrected metadata option required')
    if not option['enabled']:
        return 10
    from diagnostic_observation import strict_formal_enabled
    if strict_formal_enabled(profile):
        return 1
    from r74_corrected_delivery_witness import enabled as witness_enabled
    if not witness_enabled(profile):
        raise ValueError('Latest corrected metadata requires isolated measured diagnostic scope')
    return 1
