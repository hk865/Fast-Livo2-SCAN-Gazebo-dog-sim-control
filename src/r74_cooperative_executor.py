"""Default-off R74 public spin_once/zero-yield simulation experiment.

This is not a generally equivalent replacement for Executor.spin. The selected
executor has one sole main-loop owner; public spin_once releases is_spinning
between iterations and during sleep(0). No second or nested spinner is allowed.
"""
import threading
import time

KEY = 'r74_cooperative_executor'
SCHEMA = 'r74_spin_once_zero_yield/v1'


def enabled(profile):
    if type(profile) is not dict:
        raise ValueError('Cooperative executor profile must be a JSON object')
    if KEY not in profile:
        return False
    value = profile[KEY]
    if (type(value) is not dict or set(value) != {'enabled', 'schema'}
            or type(value['enabled']) is not bool
            or type(value['schema']) is not str or value['schema'] != SCHEMA):
        raise ValueError('Exact typed R74 cooperative executor option required')
    if not value['enabled']:
        return False
    from diagnostic_observation import strict_formal_enabled
    if strict_formal_enabled(profile):
        return True
    from minimal_simulation_baseline import enabled as minimal_enabled
    from r74_teacher_substage_timing import enabled as timing_enabled
    if not minimal_enabled(profile) or not timing_enabled(profile):
        raise ValueError('Cooperative executor requires minimal simulation and selected R74 timing')
    return True


def make_executor(base, profile, num_threads=2):
    if not enabled(profile):
        return base(num_threads=num_threads)

    class CooperativeExecutor(base):
        def __init__(self, *args, **kwargs):
            self._r74_cooperative_stop = threading.Event()
            super().__init__(*args, **kwargs)

        def spin(self):
            while self.context.ok() and not self._r74_cooperative_stop.is_set():
                self.spin_once()
                time.sleep(0)

        def shutdown(self, *args, **kwargs):
            self._r74_cooperative_stop.set()
            return super().shutdown(*args, **kwargs)

    return CooperativeExecutor(num_threads=num_threads)
