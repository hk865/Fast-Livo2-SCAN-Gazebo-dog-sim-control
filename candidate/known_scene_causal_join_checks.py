"""Deterministic timestamp/queue regression checks; no ROS or simulation.

These isolate selection with opaque message objects and a stub geometry check.
They do not certify source geometry, navigation, or a physical parking result.
"""
import json
import threading
import time
import numpy as np
from known_scene_adapter import ExactSourceJoin, MatcherWorker

ROLES = (*ExactSourceJoin.ROLES, *ExactSourceJoin.META_ROLES, *ExactSourceJoin.AUX_ROLES)
WALL = 100_000_000_000


def fixture():
    join = ExactSourceJoin({})
    join._check = lambda ns, slot: dict(source_ns=ns, body_points=np.zeros((1, 3)),
        T_odom_body=np.eye(4), provenance={}, source_messages={r: p[0] for r, p in slot.items()})
    return join


def add(join, ns, wall=WALL):
    join.slots[ns] = {role: (object(), wall) for role in ROLES}


def main():
    checks = []
    # Captured times from the observed failure; payloads are opaque fixtures.
    join = fixture(); add(join, 49_830_000_000); add(join, 49_865_000_000)
    future = dict(join.slots[49_865_000_000])
    result = join.pop_latest(49_850_000_000, WALL)
    assert result['source_ns'] == 49_830_000_000
    assert join.slots[49_865_000_000] == future
    assert result['join_selection']['excluded_future_complete_count'] == 1
    assert result['join_selection']['maximum_complete_source_ns'] == 49_865_000_000
    checks.append('newest_future_does_not_replace_causal_source')

    join = fixture(); add(join, 4_001_000_000)
    before = {n: dict(s) for n, s in join.slots.items()}
    for _ in range(3):
        assert join.pop_latest(4_000_000_000, WALL) is None
        assert join.slots == before and join.last_completed_ns is None
    assert join.pop_latest(4_001_000_000, WALL)['source_ns'] == 4_001_000_000
    checks.append('future_only_waits_without_consumption_then_clock_catches_up')

    for clock, reason in [(4_000_000_000, None), (7_000_000_000, None),
                           (7_000_000_001, 'source_age')]:
        join = fixture(); add(join, 4_000_000_000)
        assert join.pop_latest(clock, WALL).get('rejected') == reason
    checks.append('original_source_age_exact_boundaries')

    for received in [WALL - 3_000_000_001, WALL + 1]:
        join = fixture(); add(join, 4_000_000_000, received)
        assert join.pop_latest(4_000_000_000, WALL)['rejected'] == 'wall_age'
    join = fixture(); add(join, 4_000_000_000, WALL - 3_000_000_000)
    assert join.pop_latest(4_000_000_000, WALL).get('rejected') is None
    checks.append('old_and_future_wall_receipts_still_rejected')

    join = fixture(); join.last_completed_ns = 3_000_000_000
    add(join, 3_999_999_999); add(join, 5_000_000_000)
    before = {n: dict(s) for n, s in join.slots.items()}
    assert join.pop_latest(4_000_000_000, WALL) is None
    assert join.slots == before and join.last_completed_ns == 3_000_000_000
    add(join, 4_000_000_000)
    assert join.pop_latest(4_000_000_000, WALL)['source_ns'] == 4_000_000_000
    assert 5_000_000_000 in join.slots
    checks.append('one_second_interval_unchanged_future_never_advances_watermark')

    join = fixture(); ready = threading.Event(); outcome = []
    join.lock.acquire()
    def waiting_pop():
        ready.set(); outcome.append(join.pop_latest(4_000_000_000))
    thread = threading.Thread(target=waiting_pop)
    thread.start(); assert ready.wait(1); time.sleep(.01)
    received = time.monotonic_ns(); add(join, 4_000_000_000, received)
    join.lock.release(); thread.join(1)
    assert not thread.is_alive() and outcome[0].get('rejected') is None
    assert outcome[0]['join_selection']['guard_wall_ns'] >= received
    checks.append('wall_clock_sampled_after_lock_wait_and_selected_receipts')

    join = fixture(); add(join, 49_830_000_000, time.monotonic_ns())
    entered, release = threading.Event(), threading.Event()
    pop = join.pop_latest
    def delayed_pop(clock_ns):
        entered.set(); assert release.wait(1)
        return pop(clock_ns)
    join.pop_latest = delayed_pop
    match_threads = []
    class Matcher:
        def match(self, ns, *args, **kwargs):
            match_threads.append(threading.current_thread().name)
            return dict(source_ns=ns, accepted=True, evidence_verified=True)
    worker = MatcherWorker(Matcher())
    try:
        worker.submit_join(join, 49_850_000_000, np.eye(4), parent_generation=7)
        assert entered.wait(1)
        add(join, 49_865_000_000, time.monotonic_ns()); release.set()
        result = None
        until = time.monotonic() + 1
        while result is None and time.monotonic() < until:
            result = worker.poll(); time.sleep(.001)
        assert result is not None and result['source_ns'] == 49_830_000_000
        assert result['parent_generation'] == 7
        assert result['join_selection']['captured_clock_ns'] == 49_850_000_000
        assert result['join_selection']['excluded_future_complete_count'] == 1
        assert result['join_selection']['submitted_monotonic_ns'] <= result['worker_started_wall_ns']
        assert match_threads == ['known-scene-match']
    finally:
        release.set(); worker.close()
    assert not worker.thread.is_alive()
    checks.append('real_worker_delay_keeps_captured_clock_and_geometry_off_thread')
    print(json.dumps(dict(passed=True, checks=checks, count=len(checks),
        opaque_payload_selection_fixture=True, ROS_Node_started=False,
        geometry_certified=False, navigation_PASS=False), indent=2))


if __name__ == '__main__':
    main()
