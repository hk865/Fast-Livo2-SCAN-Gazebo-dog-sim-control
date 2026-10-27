"""R74 setup-only public /clock callback-group routing for minimal simulation.

No clock producer, timestamp transformation, timer, private TimeSource access or
per-message observer is installed here. The original TimeSource callback/QoS
and its strict time/source consumers keep their values and rules.
"""
from minimal_simulation_baseline import enabled as minimal_baseline_enabled

KEY = 'r74_independent_ros_clock'
CONFIG = {'enabled': True, 'schema': 'r74_independent_clock_callback_group/v1'}


def enabled(profile):
    if not isinstance(profile, dict) or KEY not in profile:
        return False
    option = profile[KEY]
    if (type(option) is not dict or set(option) != set(CONFIG)
            or type(option.get('enabled')) is not bool
            or type(option.get('schema')) is not str or option != CONFIG):
        raise ValueError('Exact typed R74 independent ROS clock option required')
    from diagnostic_observation import strict_formal_enabled
    if strict_formal_enabled(profile):
        return True
    if not minimal_baseline_enabled(profile):
        raise ValueError('Independent ROS clock requires existing minimal simulation opt-in')
    return True


def subscription_options(node, topic, options):
    # Other topics and unselected ordinary /clock calls pass through unchanged.
    if topic != '/clock' or not enabled(getattr(node, 'profile', None)):
        return options
    if options.get('callback_group') is not None:
        raise ValueError('Do not replace an explicitly grouped clock subscription')
    group = getattr(node, '_r74_clock_callback_group', None)
    if group is None:
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        group = MutuallyExclusiveCallbackGroup()
        node._r74_clock_callback_group = group
        node._r74_clock_setup_receipt = dict(
            schema=CONFIG['schema'], enabled=True, topic='/clock',
            callback_group='dedicated_MutuallyExclusiveCallbackGroup',
            setup_only=True, clock_messages_observed=False,
            original_callback_QoS_timestamps_preserved=True,
            no_new_subscription_or_clock_producer=True)
    routed = dict(options)
    routed['callback_group'] = group
    return routed
