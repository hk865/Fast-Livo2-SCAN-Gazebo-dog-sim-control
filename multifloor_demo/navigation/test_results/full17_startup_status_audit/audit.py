#!/usr/bin/env python3
"""Read the immutable Full17 records; no ROS, publication, or control imports."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUN = ROOT / 'runs/20261001_232612_ad2317'
OUTPUT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def largest_gap(entries):
    if len(entries) < 2:
        return None
    prior, following = max(zip(entries, entries[1:]), key=lambda pair: pair[1]['wall']-pair[0]['wall'])
    return {'wall_gap_s': following['wall']-prior['wall'], 'prior': prior, 'following': following}


def main():
    mission = json.loads((RUN / 'mission.json').read_text())
    gate = json.loads((RUN / 'startup_gate.json').read_text())
    adapter = rows(RUN / 'joint_stop_adapter.jsonl')
    feedback = rows(RUN / 'feedback_navigation.jsonl')
    imu = [row for row in rows(RUN / 'sensor_audit.jsonl') if row['source'] == 'imu']
    postwarm = [row for row in imu if row['stamp'] >= 10.0]
    ordered_gap = largest_gap([row for row in adapter if row['sim'] >= 10.0])
    kind_gap = {kind: largest_gap([row for row in adapter if row['kind'] == kind and row['sim'] >= 10.0])
                for kind in ('raw_target', 'filtered_target', 'requested_safe', 'actual_champ_command')}
    reference = next(row['positions'] for row in adapter if 'positions' in row)
    zeros = [row for row in adapter if row['kind'] in ('requested_safe', 'actual_champ_command')]
    final_safety = mission['execution_safety']
    nav = [{key: row.get(key) for key in ('wall_elapsed', 'sim_time', 'navigation')}
           for row in feedback if 'navigation' in row]
    source_names = ['simulation/control_bridge.py', 'simulation/execution_safety.py',
                    'simulation/joint_reference_adapter.py', 'scripts/stack.launch.py',
                    'scripts/wait_sensors.py', 'scripts/sensor_startup_gate.py',
                    'scripts/mission_server.py', 'navigation/controller.py']
    frozen = json.loads((ROOT / 'test_results/full17_freeze/source_manifest.json').read_text())
    frozen_files = frozen['sha256']
    sources = {name: {'current_sha256': digest(ROOT / name),
                     'frozen_sha256': frozen_files.get(name)} for name in source_names}
    for value in sources.values():
        value['matches_frozen'] = value['current_sha256'] == value['frozen_sha256']
    report = {
        'scope': 'Read-only Full17 startup records and frozen source. No ROS nodes, tests, builds, physics, or control.',
        'run_id': RUN.name,
        'original_outcome': {'stage': mission['stage'], 'message': mission['message'],
                             'events': mission['events'], 'current_request': mission['current_request'],
                             'origin': mission['origin']},
        'source_receipts': sources,
        'input_sha256': {name: digest(RUN / name) for name in (
            'mission.json', 'startup_gate.json', 'joint_stop_adapter.jsonl',
            'feedback_navigation.jsonl', 'sensor_audit.jsonl', 'stack.log',
            'acceptance.json', 'root_terminal_provenance.json')},
        'gate': gate,
        'actual_thresholds': {
            'adapter_status_timeout_wall_s': .30,
            'bridge_command_timeout_wall_s': .40,
            'adapter_safe_command_timeout_wall_s': .25,
            'all_preserved': True,
            'source': 'JointAdapterHealth default, Bridge tick and Adapter watchdog'},
        'adapter_callback_evidence': {
            'last_to_first_any_logged_event_gap': ordered_gap,
            'post_10_kind_largest_gaps': kind_gap,
            'own_failure_rows': [row for row in adapter if row['kind'] == 'failure' or row['state'] == 'failed'],
            'actual_and_requested_command_rows': len(zeros),
            'all_six_command_values_exact_zero': all(all(value == 0.0 for value in row['value']) for row in zeros),
            'raw_and_filtered_target_rows': sum('positions' in row for row in adapter),
            'all_nominal_positions_exact_equal': all(row['positions'] == reference for row in adapter if 'positions' in row),
            'shutdown': adapter[-1],
            'timestamp_limit': 'wall is adapter-relative monotonic; sim is cached /clock. This is callback/log evidence, not publisher entry/return.',
        },
        'independent_sensor_evidence': {
            'imu_count': len(imu),
            'first_stamp': imu[0]['stamp'], 'last_stamp': imu[-1]['stamp'],
            'all_imu_stamp_steps_1ms': all(abs(b['stamp']-a['stamp']-.001) < 1e-9 for a,b in zip(imu,imu[1:])),
            'post_10_wall_max_gap_s': max(b['wall_monotonic']-a['wall_monotonic'] for a,b in zip(postwarm,postwarm[1:])),
            'during_adapter_cached_clock_gap_13_843_to_14_178': {
                'count': sum(13.843 <= row['stamp'] <= 14.178 for row in imu),
                'statement': 'Independent original IMU continued; adapter clock cache jumped from 13.848 to 14.178 after its callback gap.'},
        },
        'final_bridge_safety': final_safety,
        'official_nav_status_samples': nav,
        'recording_limits': {
            'control_audit_bytes': (RUN / 'control_audit.jsonl').stat().st_size,
            'navigation_audit_bytes': (RUN / 'navigation_audit.jsonl').stat().st_size,
            'trajectories_bytes': (RUN / 'feedback_navigation_trajectories.jsonl').stat().st_size,
            'bridge_first_failure_callback_not_recorded': True,
            'adapter_status_publisher_enter_return_not_recorded': True,
            'scheduler_or_native_syscall_durations_not_recorded': True},
        'causal_classification': {
            'proved': [
                'Initial static-sensor/ready-bridge gate passed before heavy modules launched.',
                'No navigation request, SLAM body odom, path or goal-region arrival was produced.',
                'Adapter remained idle with no own failure record; commands were exact zero and joint targets constant.',
                'Different adapter callback classes jointly failed to log progress for 337ms while independent IMU continued.',
                'Bridge joint health latched adapter_status_timeout; a later fresh healthy adapter status did not clear that latch.',
                'Mission independently consumed failed bridge status and ended startup at 20.366wall seconds.'
            ],
            'not_proved': [
                'Specific blocking rcl_publish topic, DDS duration, log-write duration, or OS scheduling cause.',
                'Exact bridge first timeout instant or whether IMU callback/timer/new-status callback first latched it.',
                'Actual JTC target delivery time/effort during callback gap; only adapter-side returned-publish log is available.',
                'CM regression, NAV/SCAN steering fault, region failure, or motion-response correctness.'
            ],
            'plausible_mechanisms_to_separate': [
                'One synchronous publish call blocks the adapter single-threaded executor, preventing status/clock/raw/safe callbacks.',
                'The adapter process is not scheduled or waits in executor while other processes keep running.',
                'Adapter callback-side serialization/file write stalls; no native write duration was captured.'
            ],
            'pure_status_delivery_lag_insufficient': 'Would not alone account for all independent adapter callback/log streams stalling and cached clock catching up.',
        },
        'minimal_next_observation_only': {
            'keep_limits_and_control_unchanged': True,
            'native_boundary': 'Bounded in-memory rcl_publish entry/return hook for adapter actuator Twist, filtered JointTrajectory and joint_stop_safety String; verify topic/type/publisher handle.',
            'status_consume_boundary': 'Bounded status callback wall-entry receipt in Bridge with original source sim/counters; do not refresh watchdog from observer.',
            'interpretation': 'If the adapter publish entry spans the whole blank then isolate publishing; if no such call spans it, inspect process scheduling/native write boundary. No widening TTL or ignoring failed latch.'
        },
        'terminal_provenance': json.loads((RUN / 'root_terminal_provenance.json').read_text()),
    }
    (OUTPUT / 'result.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'result': str(OUTPUT / 'result.json'), 'frozen_sources_match': all(x['matches_frozen'] for x in sources.values()),
                      'any_logged_event_gap_s': ordered_gap['wall_gap_s'],
                      'imu_post10_wall_max_gap_s': report['independent_sensor_evidence']['post_10_wall_max_gap_s'],
                      'own_adapter_failures': len(report['adapter_callback_evidence']['own_failure_rows'])}, indent=2))


if __name__ == '__main__':
    main()
