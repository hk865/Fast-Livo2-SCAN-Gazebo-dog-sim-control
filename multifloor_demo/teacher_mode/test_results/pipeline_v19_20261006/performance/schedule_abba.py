#!/usr/bin/env python3
"""Make a prospective independent-process schedule; never run a benchmark."""
import argparse
import hashlib
import json
from pathlib import Path


def schedule(scenes, baseline, candidate, processes_per_variant=32):
    if processes_per_variant < 32 or processes_per_variant % 4:
        raise ValueError('At least 32 processes per variant/scene, multiple of four required')
    rows = []
    # Four calls per block gives two independent processes for each variant.
    for scene in scenes:
        for block in range(processes_per_variant // 2):
            order = (baseline, candidate, candidate, baseline) if block % 2 == 0 else (candidate, baseline, baseline, candidate)
            for slot, mode in enumerate(order):
                rows.append({'independent_process_index': len(rows), 'scene': scene,
                             'pair': [baseline, candidate], 'block': block, 'slot': slot,
                             'mode': mode, 'warmup_event_or_iteration_count': 10,
                             'measured_event_or_iteration_count': 30})
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenes', nargs='+', required=True)
    p.add_argument('--candidate', choices=('rx_decode', 'staged'), required=True)
    p.add_argument('--processes-per-variant', type=int, default=32)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    rows = schedule(args.scenes, 'serial', args.candidate, args.processes_per_variant)
    result = {'schema': 'go2_pipeline_v19_independent_process_schedule/v1',
              'status': 'prepared_not_executed', 'independent_processes': len(rows),
              'minimum_processes_per_scene_per_mode': args.processes_per_variant,
              'input_corpus_and_runner_sources_must_be_frozen_before_execution': True,
              'live_ROSbag_multi_topic_order_is_not_exact_production_replay': True,
              'rows': rows}
    with args.out.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps({'out': str(args.out), 'independent_processes': len(rows),
                      'sha256': hashlib.sha256(args.out.read_bytes()).hexdigest()}))


if __name__ == '__main__':
    raise SystemExit(main())
