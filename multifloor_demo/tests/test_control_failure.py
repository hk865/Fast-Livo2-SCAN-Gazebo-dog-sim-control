#!/usr/bin/env python3
"""Real mission callback with transport-free fixtures, not a physics test."""
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from mission_server import Coordinator
from mission.state_machine import Mission, ACTIVE_STAGES


class ControlFailureTests(unittest.TestCase):
    def fixture(self,stage):
        actions=[]
        fake=SimpleNamespace(lock=threading.RLock(),mission=Mission(stage=stage),
            execution_safety={},touch=lambda name:None,elapsed=lambda:12.,apply=actions.extend)
        return fake,actions

    def test_every_active_stage_fails_and_stops_immediately(self):
        for stage in ACTIVE_STAGES:
            with self.subTest(stage=stage):
                fake,actions=self.fixture(stage)
                Coordinator.on_control_safety(fake,SimpleNamespace(data=json.dumps(
                    dict(state='failed',reason='fixture_adapter_timeout'))))
                self.assertEqual(fake.mission.stage,'failed')
                self.assertEqual(actions,[dict(kind='stop_navigation')])
                self.assertEqual(fake.execution_safety['reason'],'fixture_adapter_timeout')

    def test_late_failure_does_not_change_terminal_result(self):
        for stage in ['idle','completed','failed','stopped']:
            with self.subTest(stage=stage):
                fake,actions=self.fixture(stage)
                Coordinator.on_control_safety(fake,SimpleNamespace(data=json.dumps(dict(state='failed',reason='fixture'))))
                self.assertEqual(fake.mission.stage,stage)
                self.assertFalse(actions)

    def test_returning_adapter_ready_does_not_fail_mission(self):
        fake,actions=self.fixture('exploring')
        Coordinator.on_control_safety(fake,SimpleNamespace(data=json.dumps(
            dict(state='ready',joint_adapter=dict(state='returning')))))
        self.assertEqual(fake.mission.stage,'exploring')
        self.assertFalse(actions)


if __name__=='__main__':unittest.main(verbosity=2)
