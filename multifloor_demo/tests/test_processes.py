#!/usr/bin/env python3
"""Real child-process checks for run isolation and bounded cleanup."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mission.processes import finish_owned_process, group_running


class ProcessTests(unittest.TestCase):
    def test_interrupt_cleans_owned_session(self):
        proc = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], start_new_session=True)
        proc.send_signal(signal.SIGINT)
        finish_owned_process(proc, grace=.2, terminate_timeout=.2, kill_timeout=1.)
        self.assertIsNotNone(proc.returncode)
        self.assertFalse(group_running(proc.pid))

    def test_stubborn_child_is_killed_without_affecting_caller(self):
        code = 'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); print("ready",flush=True); time.sleep(60)'
        proc = subprocess.Popen([sys.executable, '-c', code], start_new_session=True, stdout=subprocess.PIPE, text=True)
        self.assertEqual(proc.stdout.readline().strip(), 'ready')
        proc.send_signal(signal.SIGINT)
        started = time.monotonic()
        finish_owned_process(proc, grace=.1, terminate_timeout=.1, kill_timeout=1.)
        self.assertEqual(proc.returncode, -signal.SIGKILL)
        self.assertFalse(group_running(proc.pid))
        self.assertLess(time.monotonic()-started, 2.)
        self.assertTrue(group_running(os.getpgrp()))
        proc.stdout.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
