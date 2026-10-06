#!/usr/bin/env python3
"""Offline actual paired-QoS DDS evidence; never launches ROS or changes acceptance."""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve()
CAPACITY = {'shm512k_01': 524288, 'shm64m_01': 67108864}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def rows(path):
    if not Path(path).exists():
        return []
    with Path(path).open() as f:
        return [json.loads(line) for line in f if line.strip()]


def distribution(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {'count': 0, 'min': None, 'median': None, 'p95': None, 'max': None}
    return {'count': len(a), 'min': float(a.min()), 'median': float(np.median(a)),
            'p95': float(np.percentile(a, 95)), 'max': float(a.max())}


def intervals(clock_ns, mask):
    """Actual sampled clock runs; no hypothetical deliveries or gap interpolation."""
    result = []
    start = None
    for i, active in enumerate(mask):
        if active and start is None:
            start = i
        if start is not None and (not active or i == len(mask) - 1):
            end = i if not active else i + 1
            last = min(end, len(clock_ns) - 1)
            result.append({'start_clock_ns': int(clock_ns[start]),
                           'end_clock_ns': int(clock_ns[last]),
                           'duration_sim_s': float((clock_ns[last] - clock_ns[start]) / 1e9),
                           'actual_clock_samples': end - start})
            start = None
    return result


def freshness_replay(stream, clock):
    """Causal cache replay at each actual received clock, not a runtime gate receipt."""
    cw = np.array([r['received_monotonic_wall'] for r in clock], float)
    ct = np.array([r['original_stamp_ns'] for r in clock], np.int64)
    rw = np.array([r['received_monotonic_wall'] for r in stream], float)
    rt = np.array([r['original_stamp_ns'] for r in stream], np.int64)
    ix = np.searchsorted(rw, cw, side='right') - 1
    valid = ix >= 0
    sim_age = np.full(len(cw), np.nan)
    wall_age = np.full(len(cw), np.nan)
    sim_age[valid] = (ct[valid] - rt[ix[valid]]) / 1e9
    wall_age[valid] = cw[valid] - rw[ix[valid]]
    fresh = valid & (sim_age >= -.05) & (sim_age < .3) & (wall_age >= -.05) & (wall_age < .3)
    active = valid
    sim_invalid = active & ((sim_age < -.05) | (sim_age >= .3))
    wall_invalid = active & ((wall_age < -.05) | (wall_age >= .3))
    stale = active & ~fresh
    episode = intervals(ct, stale)
    weighted = np.r_[np.diff(ct) / 1e9, 0.]
    active_duration = float(weighted[active].sum())
    return {'audit_kind': 'offline causal observer cache replay, not actual navigation gate',
            'reference_clock': 'original integer /clock received before this observer callback',
            'original_dual_age_limits_s': [-.05, .3],
            'upper_limit_inclusive_invalid': True,
            'clock_samples': len(clock), 'samples_before_first_delivery': int((~valid).sum()),
            'post_first_delivery_clock_samples': int(active.sum()),
            'post_first_delivery_stale_samples': int(stale.sum()),
            'source_age_invalid_samples': int(sim_invalid.sum()),
            'receive_wall_age_invalid_samples': int(wall_invalid.sum()),
            'post_first_delivery_stale_sample_fraction': float(stale.sum() / active.sum()) if active.any() else None,
            'post_first_delivery_sim_duration_s': active_duration,
            'sample_interval_stale_duration_sim_s': float(weighted[stale].sum()),
            'maximum_stale_episode_sim_s': max((x['duration_sim_s'] for x in episode), default=0.),
            'post_first_delivery_300ms_continuity': 'passed' if active.any() and not stale.any() else 'failed',
            'stale_episodes': episode,
            'source_sim_age_s': distribution(sim_age[active]),
            'callback_cache_wall_age_s': distribution(wall_age[active])}, (ct / 1e9, sim_age, wall_age)


def stream_metrics(stream, clock, capacity):
    t = np.array([r['original_stamp_ns'] for r in stream], np.int64)
    w = np.array([r['received_monotonic_wall'] for r in stream], float)
    age = [(r['received_ros_clock_ns'] - r['original_stamp_ns']) / 1e9 for r in stream
           if r.get('received_ros_clock_ns') is not None]
    replay, array = freshness_replay(stream, clock)
    payload = [r['data_bytes'] for r in stream if 'data_bytes' in r]
    info_keys = ['source_timestamp', 'received_timestamp', 'reception_timestamp',
                 'publication_sequence_number', 'reception_sequence_number', 'publisher_gid_hex']
    result = {'callbacks': len(stream), 'unique_original_header_stamps': len(set(map(int, t))),
              'duplicate_header_callbacks': len(t) - len(set(map(int, t))),
              'header_regressions': int((np.diff(t) < 0).sum()),
              'first_header_stamp_ns': int(t[0]) if len(t) else None,
              'last_header_stamp_ns': int(t[-1]) if len(t) else None,
              'header_gap_sim_s': distribution(np.diff(t) / 1e9),
              'callback_gap_wall_s': distribution(np.diff(w)),
              'callback_gap_received_sim_s': distribution(np.diff([r['received_ros_clock_ns'] for r in stream]) / 1e9),
              'source_age_at_callback_sim_s': distribution(age),
              'source_age_invalid_at_callback': sum(a < -.05 or a >= .3 for a in age),
              'frames': dict(Counter(r.get('frame_id') for r in stream)),
              'message_info_available_counts': {k: sum((r.get('message_info') or {}).get(k) is not None for r in stream) for k in info_keys},
              'freshness_replay': replay}
    if payload:
        result['PointCloud2'] = {'data_bytes': distribution(payload),
            'data_bytes_exceed_512KiB': sum(x > 524288 for x in payload),
            'data_bytes_exceed_configured_segment': sum(x > capacity for x in payload),
            'point_step_bytes': sorted(set(r['point_step'] for r in stream)),
            'row_stride_matches_data_bytes': all(r['row_step'] * r['height'] == r['data_bytes'] for r in stream),
            'fields': stream[0]['fields'], 'serialized_total_bytes': 'unverified; data_bytes is payload only'}
    return result, array


def exact_pair(left, right, metadata=False):
    a = {int(r['original_stamp_ns']): r for r in left}
    b = {int(r['original_stamp_ns']): r for r in right}
    common = sorted(a.keys() & b.keys())
    fields = ['frame_id', 'width', 'height', 'point_step', 'row_step', 'data_bytes', 'fields', 'is_dense', 'is_bigendian']
    mismatched = [s for s in common if metadata and any(a[s].get(k) != b[s].get(k) for k in fields)]
    return {'matching': 'exact original integer header stamp; no rounding/tolerance matching',
            'common_original_stamps': len(common), 'left_only_stamps': len(a.keys() - b.keys()),
            'right_only_stamps': len(b.keys() - a.keys()), 'paired_metadata_mismatches': mismatched,
            'right_receive_wall_minus_left_s': distribution([b[s]['received_monotonic_wall'] - a[s]['received_monotonic_wall'] for s in common])}


def source_hashes(probe, result):
    archive = probe / 'sources'
    checked, missing, mismatched = [], [], []
    for source, expected in result['source_hashes'].items():
        file = archive / Path(source).name
        if not file.exists():
            missing.append(str(file))
        elif sha(file) != expected:
            mismatched.append(str(file))
        else:
            checked.append(str(file))
    freeze = archive / 'freeze.json'
    if not freeze.exists() or sha(freeze) != result['freeze_sha256']:
        mismatched.append(str(freeze))
    return {'passed': not missing and not mismatched, 'verified_count': len(checked),
            'missing': missing, 'mismatched': mismatched, 'archive_only': True}


def normalize_xml(path, omit_capacity=False, run_name=None):
    node = ET.fromstring(Path(path).read_text())
    def walk(n):
        text = (n.text or '').strip()
        if omit_capacity and n.tag.split('}')[-1] == 'segment_size':
            text = 'CAPACITY_ONLY'
        if run_name:
            text = text.replace(run_name, '{RUN}')
        return (n.tag, sorted(n.attrib.items()), text, tuple(walk(x) for x in n))
    return walk(node)


def analyze_case(parent):
    parent = parent.resolve()
    probe = parent / 'probe'
    link = read_json(parent / 'actual_run_link_and_process_snapshot.json')
    run = Path(link['run'])
    result = read_json(probe / 'probe_result.json')
    runtime = read_json(run / 'runtime_manifest.json')
    exit_receipt = read_json(parent / 'root_owned_probe_exit.json')
    manifest = read_json(parent / 'cloud_transport_manifest.json')
    capacity = manifest['segment_capacity_bytes']
    receipt_rows = rows(probe / 'receipts.jsonl')
    streams = {name: [r for r in receipt_rows if r.get('stream') == name]
               for name in ('cloud_reliable', 'cloud_best_effort', 'body_odom', 'clock')}
    clocks = streams['clock']
    metrics, arrays = {}, {}
    for name in ('cloud_reliable', 'cloud_best_effort', 'body_odom'):
        metrics[name], arrays[name] = stream_metrics(streams[name], clocks, capacity)
    sensor = defaultdict(list)
    for r in rows(run / 'sensor_shadow' / 'sensor_samples.jsonl'):
        if r.get('source') in ('cloud', 'slam'):
            sensor[r['source']].append({'original_stamp_ns': r['stamp_ns'],
                'received_monotonic_wall': r['received_monotonic_wall'],
                'received_ros_clock_ns': None, 'frame_id': r['frame']})
    telemetry = rows(run / 'telemetry.jsonl')
    source_audit = source_hashes(probe, result)
    checks = {
        'observer_completed_full_60sim_seconds': result['status'] == 'observation_complete' and result['observed_sim_duration_s'] >= 60,
        'four_actual_streams_and_counts_match': all(len(v) == result['counts'][k] and len(v) for k, v in streams.items()),
        'receipts_drained_no_error': result['error'] is None and result['queue_error'] is None and result['records_drained'] and len(receipt_rows) == result['records_written'] == result['records_submitted'],
        'original_clock_strictly_increasing': all(b['original_stamp_ns'] > a['original_stamp_ns'] for a, b in zip(clocks, clocks[1:])),
        'probe_source_archive_matches_run_freeze': source_audit['passed'],
        'main_six_owned_and_probe_exited_zero': len(runtime['owned_processes']) == 6 and all(p['returncode'] == 0 for p in runtime['owned_processes']) and runtime['error'] is None and exit_receipt['returncode'] == 0 and exit_receipt['process_exited'],
        'main_and_probe_XML_exact_bytes': sha(run / 'cloud_transport.xml') == sha(parent / 'cloud_transport.xml') == manifest['xml_sha256'],
        'scripted_zero_command_CPU_teacher_fixture': bool(telemetry) and all(all(v == 0 for v in r['command']) and all(v == 0 for v in r['requested']) for r in telemetry) and runtime['rendering']['actor_device'] == 'cpu' and not runtime['real_robot'] and runtime['training_process_untouched'],
        'paired_cloud_metadata_exact_for_matching_headers': not exact_pair(streams['cloud_reliable'], streams['cloud_best_effort'], True)['paired_metadata_mismatches'],
    }
    graph = [r for r in receipt_rows if 'endpoints' in r]
    publisher_sets = {}
    for topic in ('/cloud_registered_full', '/demo/slam/body_odom', '/clock'):
        pub = {}
        for g in graph:
            for p in g['endpoints'].get(topic, {}).get('publishers', []):
                pub[p['endpoint_gid_hex']] = p
        publisher_sets[topic] = list(pub.values())
    log = (run / 'navigation_stack.log').read_text()
    slices = [{'logged_camera_slice_ns': int(Decimal(t) * Decimal(1_000_000_000)), 'points': int(n)}
              for t, n in re.findall(r'DEMO_LIDAR_SLICE\] camera=([0-9.]+).*? points=(\d+)', log)]
    archive = read_json(run / 'map_metadata.json')
    paths = [probe / 'receipts.jsonl', probe / 'probe_result.json', probe / 'sources/freeze.json',
             probe / 'environment.json', probe / 'observer_proc_maps.txt', parent / 'actual_run_link_and_process_snapshot.json',
             parent / 'cloud_transport.xml', parent / 'cloud_transport_manifest.json', parent / 'root_owned_probe_exit.json',
             run / 'runtime_manifest.json', run / 'telemetry.jsonl', run / 'map_metadata.json',
             run / 'navigation_stack.log', run / 'sensor_shadow/sensor_samples.jsonl', run / 'policy_manifest.json',
             run / 'independent_protocol.json', run / 'world.sdf', run / 'summary.json']
    return {'schema': 'independent_actual_cloud_transport_case/v1', 'case': parent.name,
        'run': str(run), 'probe': str(probe), 'status': 'observation_evidence_verified' if all(checks.values()) else 'failed',
        'checks': {k: {'status': 'passed' if v else 'failed', 'passed': bool(v)} for k, v in checks.items()},
        'capacity_bytes': capacity, 'metrics': metrics,
        'pairs': {'reliable_to_best_effort': exact_pair(streams['cloud_reliable'], streams['cloud_best_effort'], True),
                  'reliable_to_body_odom': exact_pair(streams['cloud_reliable'], streams['body_odom']),
                  'reliable_to_shadow_cloud': exact_pair(streams['cloud_reliable'], sensor['cloud']),
                  'body_odom_to_shadow_slam': exact_pair(streams['body_odom'], sensor['slam'])},
        'source_audit': source_audit, 'actual_graph_publishers': publisher_sets,
        'actual_process_evidence': {'main_runtime': runtime['owned_processes'], 'probe_pid': result['pid'],
            'probe_exit': exit_receipt['returncode'], 'main_live_maps_snapshot_processes': len(link['actual_owned_process_snapshot']),
            'observer_own_actual_environment': read_json(probe / 'environment.json'),
            'observer_loaded_rmw_fastrtps_cpp': 'librmw_fastrtps_cpp.so' in (probe / 'observer_proc_maps.txt').read_text(),
            'B_missing_main_maps_not_imputed': not bool(link['actual_owned_process_snapshot']),
            'configured_transport_is_not_per_message_transport_proof': True},
        'SLAM': {'processed_camera_slice_logs': len(slices), 'first_last_processed_slice_ns': [slices[0]['logged_camera_slice_ns'], slices[-1]['logged_camera_slice_ns']] if slices else None,
            'publish_call_integer_receipts_available': False,
            'scope': 'DEMO_LIDAR_SLICE is input processing, not a complete publication counter or exact original header receipt'},
        'map_archive_final_receipt': {'counts': archive['counts'], 'stamps': archive['stamps'],
            'source_topic': archive['source_topic'], 'error': archive['error'],
            'per_callback_integer_pairing': 'unverified; only final metadata counts are retained'},
        'controller_and_gate': {'status': 'unverified', 'reason': 'Stand fixture starts SLAM without route controller or sensor gate; no navigation callbacks or gate receipts'},
        'navigation_acceptance': 'unverified', 'DDS_root_cause': 'unverified',
        'actual_inputs': {str(p): sha(p) for p in paths}, 'analyzer_sha256': sha(HERE)}, arrays


def comparison(base, write=True):
    cases = []
    figure_arrays = []
    for name in CAPACITY:
        case, array = analyze_case(base / name)
        cases.append(case)
        figure_arrays.append(array)
    a, b = cases
    ra, rb = Path(a['run']), Path(b['run'])
    ma, mb = read_json(base / a['case'] / 'cloud_transport_manifest.json'), read_json(base / b['case'] / 'cloud_transport_manifest.json')
    env = lambda m: {k: v for k, v in m['environment'].items() if k != 'FASTRTPS_DEFAULT_PROFILES_FILE'}
    checks = {
        'both_actual_evidence_complete': all(c['status'] == 'observation_evidence_verified' for c in cases),
        'only_XML_segment_capacity_differs': normalize_xml(base / a['case'] / 'cloud_transport.xml', True) == normalize_xml(base / b['case'] / 'cloud_transport.xml', True),
        'actual_capacity_values_are_512KiB_and_64MiB': [c['capacity_bytes'] for c in cases] == [524288, 67108864],
        'same_declared_run_and_probe_effective_environment_except_file_path': env(ma) == env(mb),
        'same_world_after_run_path_normalization': normalize_xml(ra / 'world.sdf', run_name=ra.name) == normalize_xml(rb / 'world.sdf', run_name=rb.name),
        'same_immutable_60sec_zero_command_schedule': sha(ra / 'independent_protocol.json') == sha(rb / 'independent_protocol.json'),
        'same_probe_source_freeze': read_json(base / a['case'] / 'probe/probe_result.json')['freeze_sha256'] == read_json(base / b['case'] / 'probe/probe_result.json')['freeze_sha256'],
    }
    result = {'schema': 'independent_actual_cloud_transport_comparison/v1',
        'status': 'comparison_complete' if all(checks.values()) else 'comparison_incomplete',
        'checks': {k: {'status': 'passed' if v else 'failed', 'passed': bool(v)} for k, v in checks.items()},
        'cases': cases, 'analyzer_sha256': sha(HERE),
        'scope': 'One sequential 60sim zero-command stand per capacity; same explicit loopback UDP+SHM configuration; actual paired QoS and original integer stamp evidence',
        'limitations': ['No route controller or gate runs, so this is not PID motion/navigation acceptance.',
            'Only one A/B repetition; shared reliable reader can affect publisher backpressure.',
            'All actual MessageInfo values remain null where unavailable; header agreement is not a DDS publication-sequence guarantee.',
            'B main-process maps snapshot was late and empty; main environment comes from run manifest, probe own environment/maps were captured live.',
            'Segment configuration and mappings do not identify transport used for every message.',
            'Historical LOCALHOST_ONLY=1 and implicit transport differ in discovery/transport construction and cannot isolate capacity alone.',
            'SLAM slice logs and archive final counts do not establish a complete exact publication log.',
            'The 300ms cache replay uses actual clock/callback ordering, not the absent runtime navigation gate.'],
        'navigation': 'unverified', 'motion_transport_causal_conclusion': 'unverified'}
    if write:
        for case in cases:
            (base / case['case'] / 'summary_cloud_transport_independent.json').write_text(json.dumps(case, indent=2, allow_nan=False) + '\n')
        (base / 'summary_cloud_transport_comparison_independent.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        plot(base, cases, figure_arrays)
    return result


def plot(base, cases, arrays):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10})
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
    colors = {'cloud_reliable': '#205bc2', 'cloud_best_effort': '#d16b00', 'body_odom': '#31915e'}
    labels = {'cloud_reliable': 'Reliable cloud', 'cloud_best_effort': 'Best-effort cloud', 'body_odom': 'SLAM body odom'}
    for j, (case, arr) in enumerate(zip(cases, arrays)):
        receipts = rows(Path(case['probe']) / 'receipts.jsonl')
        for k, color in colors.items():
            stream = [r for r in receipts if r.get('stream') == k]
            t = np.array([r['original_stamp_ns'] for r in stream]) / 1e9
            y = {'cloud_reliable': 2, 'cloud_best_effort': 1, 'body_odom': 0}[k]
            axes[0, j].plot(t, np.full(len(t), y), '|', color=color, markersize=7, label=f'{labels[k]} n={len(t)}')
            ct, sim_age, wall_age = arr[k]
            axes[1, j].plot(ct, sim_age, color=color, linewidth=.8, label=labels[k])
            axes[2, j].plot(ct, wall_age, color=color, linewidth=.8)
        axes[0, j].set_title(f"{case['capacity_bytes'] / 1024 / 1024:g} MiB configured SHM segment")
        axes[0, j].set_yticks([0, 1, 2], ['Body odom', 'BE cloud', 'Reliable cloud'])
        axes[0, j].legend(loc='upper left', fontsize=8)
        for i in (1, 2):
            axes[i, j].axhline(.3, linestyle='--', color='#a11', linewidth=1, label='Original 300 ms limit')
        axes[1, j].set_ylabel('Latest delivered header age (sim s)')
        axes[2, j].set_ylabel('Latest callback cache age (wall s)')
        axes[2, j].set_xlabel('Original received /clock (sim s)')
        for i in range(3):
            axes[i, j].set_xlim(0, 60.01)
            axes[i, j].grid(alpha=.2)
    fig.suptitle('Actual paired-QoS delivery and causal 300 ms cache replay — stand only\nNo route controller/gate; clock samples before first delivery excluded from cache curves', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .94))
    image = base / 'cloud_transport_actual_comparison.png'
    fig.savefig(image, dpi=150)
    plt.close(fig)
    (base / 'cloud_transport_actual_figure_manifest.json').write_text(json.dumps({
        'schema': 'scientific_actual_figure/v1', 'figure': str(image), 'figure_sha256': sha(image),
        'analyzer_sha256': sha(HERE), 'raw_inputs': {path: digest for c in cases for path, digest in c['actual_inputs'].items()},
        'no_fabricated_or_interpolated_deliveries': True, 'visual_QA': 'pending',
        'meaning': 'Metadata event marks and causal age at original received clocks; not a navigation gate receipt'}, indent=2) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, default=ROOT / 'test_results/cloud_transport_actual_20261004')
    p.add_argument('--no-write', action='store_true')
    args = p.parse_args()
    result = comparison(args.base.resolve(), not args.no_write)
    print(json.dumps({'status': result['status'], 'checks': result['checks'],
        'cases': [{k: c[k] for k in ('case', 'status', 'capacity_bytes')} for c in result['cases']]}, indent=2))


if __name__ == '__main__':
    main()
