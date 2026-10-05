#!/usr/bin/env python3
"""Append-only status-payload lineage proof; no change to original ramp acceptance."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True
OBSERVATION_FIELDS = {'monotonic_wall', 'ros_sim_time'}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def match_payload(payload, observed):
    """No source field may be removed or changed; only two observer fields are extra."""
    if set(observed) - set(payload) != OBSERVATION_FIELDS:
        return False
    if not all(k in observed and canonical(observed[k]) == canonical(v)
               for k, v in payload.items()):
        return False
    return all(type(observed[k]) in (int, float) and math.isfinite(observed[k])
               for k in OBSERVATION_FIELDS)


def analyze(run):
    run = Path(run).resolve()
    common = read(run / 'summary_closed_loop_cascade_independent.json')
    ramp = read(run / 'summary_closed_loop_ramp_independent.json')
    assert common['evaluator_sha256'] == '074b468581de568264f558e38f82ce25e809094843c6f86e2816130babf8099e'
    input_sources = {}
    for name in ('summary_closed_loop_cascade_independent.json',
                 'summary_closed_loop_ramp_independent.json', 'navigation_scope.json',
                 'navigation_request.json', 'navigation_status.jsonl',
                 'navigation_slam_poses.jsonl', 'terrain_provider_events.jsonl',
                 'terrain_provider_result.json'):
        path = run / name
        input_sources[str(path)] = digest(path)
    # The original common auditor already joined the original source region headers.
    # Recheck all sources used here against its untouched verified input hash chain.
    for name, sha in common['verified_input_source_sha256'].items():
        path = Path(name)
        if not path.is_absolute():
            path = run / path
        resolved = str(path.resolve())
        if resolved in input_sources:
            assert input_sources[resolved] == sha, ('Original input changed', name)
    events = [json.loads(line) for line in (run / 'terrain_provider_events.jsonl').read_text().splitlines()]
    switches = [e for e in events if e.get('event') == 'switched_once']
    result = read(run / 'terrain_provider_result.json')
    assert len(switches) == 1 and result['status'] == 'switched' and result['failure'] is None
    event = switches[0]
    assert event['navigation_ground_truth_used'] is False
    decoded = {}
    for name in ('status_read', 'request_read'):
        raw = event[name]['raw_utf8'].encode('utf-8')
        assert hashlib.sha256(raw).hexdigest() == event[name]['raw_bytes_sha256']
        decoded[name] = json.loads(raw)
    assert decoded['request_read'] == read(run / 'navigation_request.json')
    payload = decoded['status_read']
    matches = []
    with (run / 'navigation_status.jsonl').open('rb') as f:
        for index, raw in enumerate(f, 1):
            observed = json.loads(raw)
            if match_payload(payload, observed):
                matches.append({'line_number': index,
                                'original_history_line_sha256': hashlib.sha256(raw).hexdigest(),
                                'observer_metadata': {k: observed[k] for k in sorted(OBSERVATION_FIELDS)},
                                'projected_original_payload_sha256': hashlib.sha256(canonical({k: observed[k] for k in payload}).encode()).hexdigest()})
    assert matches, 'No complete original source payload in observer history'
    arrival = event['original_region_arrival']
    scope = read(run / 'navigation_scope.json')
    switch_contract = scope['profile']['terrain_layer_switch']
    assert switch_contract['completed_goal_id'] == arrival['goal_id'] == 'connector_mid'
    assert arrival in payload['region_arrivals']
    assert hashlib.sha256(canonical(arrival).encode()).hexdigest() == event['region_arrival_sha256']
    assert arrival['request_id'] == decoded['request_read']['request_id'] == event['request_id']
    assert arrival['goals_definition_sha256'] == payload['goals_definition_sha256'] == event['goals_definition_sha256']
    assert arrival['protected'] is False and arrival['region_inside'] is True and arrival['control_region_inside'] is True
    stamp, clock, effective = arrival['stamp_ns'], event['native_request_clock_ns'], event['native_state_effective_clock_ns']
    assert all(type(v) is int for v in (stamp, clock, effective))
    assert stamp <= clock and effective == clock - 5_000_000
    original_arrivals = common['checks']['all_original_SLAM_3D_region_arrivals']['arrivals']
    verified = [a for a in original_arrivals if a.get('goal_id') == 'connector_mid']
    assert len(verified) == 1 and verified[0]['passed'] is True
    assert verified[0]['original_begin_end_ns'] == [arrival['start_stamp_ns'], arrival['stamp_ns']]
    assert arrival['dwell_ns'] >= 600_000_000
    original_gate = ramp['checks']['original_source_authorized_causal_terrain_layer_switch']
    assert original_gate['status'] == 'unverified'
    return {
        'schema': 'teacher_actor_provider_status_source_appendix/v1',
        'status': 'passed', 'run': str(run),
        'scope': 'Only original provider status/request bytes and measured connector source lineage; no original acceptance override',
        'checks': {
            'original_read_UTF8_and_SHA': True,
            'all_original_status_fields_exact_without_source_drops': True,
            'only_explicit_observer_metadata_extra': True,
            'original_common_verified_3D_connector_headers_and_causal_native_clock': True},
        'status_payload_field_count': len(payload),
        'original_status_read_raw_bytes_sha256': event['status_read']['raw_bytes_sha256'],
        'canonical_original_status_payload_sha256': hashlib.sha256(canonical(payload).encode()).hexdigest(),
        'matched_original_history_records': matches,
        'original_connector_arrival': arrival,
        'native_request_clock_ns': clock, 'native_state_effective_clock_ns': effective,
        'causal_source_age_s': (clock - stamp) / 1e9,
        'original_common_status': common['status'], 'original_ramp_status': ramp['status'],
        'original_provider_gate_unchanged': original_gate,
        'navigation_motion_status_override': False,
        'full_187_geometry_replay_in_this_appendix': False,
        'navigation_ground_truth_used': False,
        'verified_input_source_sha256': input_sources,
        'appendix_source_sha256': digest(__file__),
        'limitations': ['The frozen ramp auditor compared observer envelopes with original status payloads including two observer-only metadata fields.',
                        'This supplement demonstrates complete original status fields and unchanged causal connector receipt only; it does not label either upper-ramp completion or final parking passed.',
                        'Original common/ramp FAILED and original switch UNVERIFIED remain unchanged.']}


def self_test():
    p = {'source_header_ns': 123, 'pose': [1., 2., 3.], 'healthy': True}
    row = dict(p, monotonic_wall=4.5, ros_sim_time=1.0)
    assert match_payload(p, row)
    assert not match_payload(p, dict(row, source_header_ns=124))
    assert not match_payload(p, dict(row, pose=[1., 2., 3.1]))
    assert not match_payload(p, dict(row, extra_source='invented'))
    assert not match_payload(p, {k: v for k, v in row.items() if k != 'pose'})
    print('5 source-wrapper positive/negative checks passed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.run is None or args.output is None:
        parser.error('--run and --output required')
    data = analyze(args.run)
    with args.output.open('x') as f:
        json.dump(data, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write('\n')
    print(json.dumps({'status': data['status'], 'output': str(args.output.resolve()),
                      'sha256': digest(args.output)}))


if __name__ == '__main__':
    main()
