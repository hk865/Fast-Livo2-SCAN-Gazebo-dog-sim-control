"""Bounded shutdown for the isolated process session owned by one demo run."""
import os
from pathlib import Path
import signal
import subprocess
import time


def group_running(group):
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == group and fields[0] not in {'Z', 'X'}:
                return True
        except (OSError, ValueError, IndexError):
            continue
    return False


def wait_group(group, timeout):
    deadline = time.monotonic()+timeout
    while group_running(group):
        if time.monotonic() >= deadline:
            return False
        time.sleep(.05)
    return True


def finish_owned_process(proc, grace=35., terminate_timeout=8., kill_timeout=5.):
    """Called after SIGINT; never signals the server or another run's session."""
    group = proc.pid  # Popen(start_new_session=True) creates this owned group.
    if group == os.getpgrp():
        raise ValueError('refusing to stop the caller process group')
    if proc.poll() is None and os.getpgid(proc.pid) != group:
        raise ValueError('demo process was not launched in an isolated session')
    for timeout, next_signal in ((grace, signal.SIGTERM), (terminate_timeout, signal.SIGKILL)):
        if wait_group(group, timeout):
            break
        try:
            os.killpg(group, next_signal)
        except ProcessLookupError:
            break
    if not wait_group(group, kill_timeout):
        raise TimeoutError('owned simulation process group did not exit')
    try:
        proc.wait(timeout=kill_timeout)
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError('owned launch process did not exit') from exc
