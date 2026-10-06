#!/usr/bin/env python3
"""Excluded test-only NAV entrypoint; production source stays byte-for-byte .04.

Physical owner must validate mixed-motion .08 response before this candidate is
executed. Both pure-turn limit and all arrival/safety/obstacle contracts remain
the actual production implementation. No node other than actual NAV drives.
"""
import hashlib
import json
import os
from pathlib import Path
import sys

EXPECTED_CORE_SHA256='98eb4702af719b92109f30f2b3f8f0c9cf161ce3cf67fc3062797b64d6df2646'
EXPECTED_CONTROLLER_SHA256='852e90731670651b7a000a71f7514418b02b43961c966a72b605b585a6397dcb'


class TestStatusPublisher:
    """Add test provenance only; forward the actual production status unchanged."""
    def __init__(self, actual_publisher, core):
        self.actual_publisher=actual_publisher
        self.core=core

    def publish(self,msg):
        data=json.loads(msg.data)
        data['test_only_motion_override']=dict(
            active_walk_yaw_cap=self.core.MAX_WALK_YAW_RATE,
            production_walk_yaw_cap=.04,walk_proportional_gain=.5,
            pure_turn_cap=self.core.MAX_TURN_RATE,
            source='excluded staging wrapper; production NAV source unchanged')
        msg.data=json.dumps(data,ensure_ascii=False)
        self.actual_publisher.publish(msg)


def configure():
    root=Path(os.environ['DEMO_TEST_ROOT']).resolve()
    out=Path(os.environ['DEMO_RUN_DIR']).resolve()
    cap=float(os.environ['DEMO_TEST_NAV_WALK_YAW_CAP'])
    if cap not in (.04,.08):
        raise ValueError('Only the predeclared baseline/candidate cap is permitted')
    hashes={name:hashlib.sha256((root/'navigation'/name).read_bytes()).hexdigest()
            for name in ['control_core.py','controller.py']}
    if hashes['control_core.py']!=EXPECTED_CORE_SHA256 or hashes['controller.py']!=EXPECTED_CONTROLLER_SHA256:
        raise RuntimeError('Production NAV differs from this frozen single-variable candidate')
    sys.path.insert(0,str(root/'navigation'))
    import control_core
    if control_core.MAX_WALK_YAW_RATE!=.04 or control_core.MAX_TURN_RATE!=.12:
        raise RuntimeError('Unexpected production motion limits')
    # Both follow_trajectory and HeadingGate resolve this same module global.
    # Their complete production bodies and acceleration/brake logic are intact.
    control_core.MAX_WALK_YAW_RATE=cap
    receipt=dict(scope=__doc__,active_walk_yaw_cap=cap,production_walk_yaw_cap=.04,
        walk_proportional_gain=.5,pure_turn_cap=control_core.MAX_TURN_RATE,
        production_source_SHA256=hashes,wrapper_SHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        unchanged=dict(raw_arrival_m=.22,stable_arrival_s=.4,joint_arrival_m=.30,
            waypoint_timeout_s=90,persistent_heading_error_rad=.20,persistent_heading_s=.5,
            severe_heading_error_rad=.55,tilt_hold_rad=.30,tilt_fail_rad=.50,
            tilt_resume_rad=.18,tilt_resume_stable_s=.8,linear_speed=.12,
            linear_acceleration=.15,yaw_acceleration=.25,box_timing=[8,20,8]))
    tmp=out/'navigation_strength_override.tmp'
    tmp.write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
    tmp.replace(out/'navigation_strength_override.json')
    return control_core


def main():
    core=configure()
    import controller
    production_navigation=controller.Navigation
    class TestStrengthNavigation(production_navigation):
        def __init__(self):
            super().__init__()
            self.status_pub=TestStatusPublisher(self.status_pub,core)
    # Production main owns initialization, spin, exact stop and archive cleanup.
    # The subclass only adds the current test constant to emitted diagnostics.
    controller.Navigation=TestStrengthNavigation
    controller.main()


if __name__=='__main__':
    main()
