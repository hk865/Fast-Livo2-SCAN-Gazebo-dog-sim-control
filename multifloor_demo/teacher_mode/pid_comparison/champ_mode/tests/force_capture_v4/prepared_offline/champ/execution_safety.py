"""Combine the independent IMU stop gate and joint-adapter health.

Returning joint references is a command handoff, not an IMU tilt hold. The
adapter keeps the physical actuator at zero until its native nominal ACK.
"""
import math
import os


def adapter_settings():
    """Default production enablement, with explicit historical A/B overrides."""
    explicit = os.environ.get('DEMO_JOINT_STOP_ADAPTER')
    test_value = os.environ.get('DEMO_TEST_JOINT_STOP_ADAPTER')
    if explicit not in {None, '0', '1'} or test_value not in {None, '0', '1'}:
        raise ValueError('Joint stop adapter override must be 0 or 1')
    legacy = explicit is None and test_value == '1'
    enabled = (explicit if explicit is not None else (test_value or '1')) != '0'
    prefix = '/demo/test' if legacy else '/demo/control'
    return dict(enabled=enabled, legacy_test_topics=legacy,
                actuator_topic=prefix+'/actuator_cmd_vel',
                raw_topic=prefix+'/joint_reference/raw',
                status_topic=prefix+'/joint_stop_safety')


class JointAdapterHealth:
    STATES = {'calibrating', 'idle', 'walk', 'wait_native_zero', 'returning', 'failed'}

    def __init__(self, required=True, timeout=.30):
        self.required = required
        self.timeout = timeout
        self.data = None
        self.wall = None
        self.calibrated = False
        self.failure = None

    def update(self, data, wall):
        if (not isinstance(data, dict) or data.get('state') not in self.STATES
                or type(data.get('nominal_calibrated')) is not bool
                or type(data.get('failed')) is not bool or not math.isfinite(wall)
                or (data.get('state') == 'calibrating' and data.get('nominal_calibrated'))):
            self.failure = self.failure or 'invalid_joint_adapter_status'
            return
        self.data = dict(data)
        self.wall = wall
        self.calibrated |= data['nominal_calibrated']
        if data['failed'] or data['state'] == 'failed':
            self.failure = self.failure or str(data.get('reason', 'joint_adapter_failed'))
        elif self.calibrated and not data['nominal_calibrated']:
            self.failure = self.failure or 'joint_adapter_lost_nominal'

    def status(self, imu_state, imu_reason, wall):
        if not self.required:
            return imu_state, imu_reason
        if self.wall is not None and wall-self.wall > self.timeout and self.calibrated:
            self.failure = self.failure or 'joint_adapter_status_timeout'
        if imu_state == 'failed':
            return imu_state, imu_reason
        if self.failure:
            return 'failed', self.failure
        if (self.data is None or self.wall is None or wall-self.wall > self.timeout
                or not self.calibrated):
            return 'hold', 'waiting_for_joint_adapter_nominal'
        return imu_state, imu_reason
