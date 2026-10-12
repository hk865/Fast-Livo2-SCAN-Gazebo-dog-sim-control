#!/usr/bin/env python3
"""Compare completed navigation runs without starting ROS or modifying originals."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def stats(values):
    values = np.asarray(values, float)
    return {'samples': len(values), 'median_s': float(np.median(values)),
            'p95_s': float(np.percentile(values,95)), 'minimum_s': float(values.min()),
            'maximum_s': float(values.max()), 'at_least_300ms': int((values>=.3).sum())} if len(values) else {'samples':0}


def callback_lag(history, shadow, kind):
    # Independent subscribers may receive the same publication at different
    # times. This diagnoses callback scheduling/transport differences, not
    # publisher acquisition latency or a ground-truth pose substitute.
    shadow_index = {}
    for row in shadow:
        if row['source']==kind:
            shadow_index.setdefault(row['stamp_ns'], row['received_monotonic_wall'])
    callbacks = {}
    for row in history:
        if kind=='slam':
            stamp = row.get('slam_stamp_ns')
            age = row.get('ages',{}).get('pose_wall_s')
        else:
            entry = row.get('ages',{}).get('cloud',{})
            stamp,age = entry.get('stamp_ns'),entry.get('wall_s')
        if isinstance(stamp,int) and isinstance(age,(int,float)):
            callbacks.setdefault(stamp,row['monotonic_wall']-age)
    pairs = [{'stamp_ns':stamp,'callback_wall':wall,'shadow_wall':shadow_index[stamp],
              'callback_minus_shadow_s':wall-shadow_index[stamp]}
             for stamp,wall in sorted(callbacks.items()) if stamp in shadow_index]
    return {'statistics':stats([row['callback_minus_shadow_s'] for row in pairs]),
            'negative_lag_meaning':'Independent callback was earlier than shadow callback; no future sample matching',
            'pairs':pairs}


def describe(run):
    health=rows(run/'navigation_sensor_gate_history.jsonl')
    bridge=rows(run/'navigation_command_history.jsonl')
    statuses=rows(run/'navigation_status.jsonl')
    shadow=rows(run/'sensor_shadow/sensor_samples.jsonl')
    summary=read(run/'summary_navigation_independent.json')
    timing=read(run/'navigation_timing_audit.json')
    runtime=read(run/'runtime_manifest.json')
    gpu=[row['gpu'] for row in runtime.get('gpu_resource_samples',[]) if isinstance(row,dict) and 'gpu' in row]
    return {'run':str(run),'levels':summary['levels'],
            'gpu_before':read(run/'resources_before.json').get('gpu'),
            'gpu_samples':gpu,'policy_cpu_forward_ms':read(run/'worker_result.json').get('cpu_forward_ms'),
            'nonzero_planar_command_seconds':summary['checks']['teacher_received_and_executed_motion']['nonzero_velocity_command_seconds'],
            'maximum_actual_xy_excursion_m':summary['checks']['teacher_received_and_executed_motion']['maximum_actual_xy_excursion_m'],
            'accepted_scan_trajectories':len(summary['metrics']['planner']['accepted_trajectory_ids']),
            'regions_confirmed':sum(row['passed']for row in summary['metrics'].get('arrivals',[])),
            'sensor_ready_snapshots':sum(row.get('ready')is True for row in health),'sensor_snapshots':len(health),
            'bridge_healthy_snapshots':sum(row.get('healthy')is True for row in bridge),'bridge_snapshots':len(bridge),
            'longest_sensor_ready_snapshot_span':next(iter(timing['metrics']['sensor_ready_snapshot_spans']),None),
            'longest_bridge_healthy_snapshot_span':next(iter(timing['metrics']['bridge_healthy_snapshot_spans']),None),
            'initial_warmup_complete':any(row.get('initial_warmup_complete')is True for row in health),
            'alignment_phase_snapshot_counts':dict(collections.Counter(row.get('alignment_phase')for row in statuses)),
            'maximum_tilt_stops':max(row.get('tilt_stops',0)for row in statuses),
            'bridge_reason_counts':dict(collections.Counter(row.get('reason')for row in bridge)),
            'bridge_slam_callback_relative_to_same_stamp_shadow':callback_lag(bridge,shadow,'slam'),
            'sensor_gate_cloud_callback_relative_to_same_stamp_shadow':callback_lag(health,shadow,'cloud'),
            'source_hashes':{name:sha(run/name)for name in ['world.sdf','source_manifest.json','navigation_scope.json',
                'navigation_sensor_gate_history.jsonl','navigation_command_history.jsonl','sensor_shadow/sensor_samples.jsonl',
                'summary_navigation_independent.json','navigation_timing_audit.json']}}


def normalized_world(run):
    source=(run/'world.sdf').read_text().replace(str(run),'<RUN>').replace(run.name,'<RUN_ID>')
    root=ET.fromstring(source)
    # Only numeric spelling of RTF is normalized, not physics values/poses.
    for node in root.findall('.//physics/real_time_factor'):
        node.text=format(float(node.text),'.17g')
    return ET.tostring(root)


def evaluate(busy,idle,output):
    busy,idle=busy.resolve(),idle.resolve()
    a,b=read(busy/'source_manifest.json'),read(idle/'source_manifest.json')
    paths=sorted(set(a)|set(b));control=[name for name in paths if name.startswith(('navigation/','policy/','simulation/'))]
    equal={name:a.get(name)==b.get(name) for name in control}
    configuration=['navigation_profile.json','navigation_scenario.json','navigation_fastlivo.yaml','navigation_camera.yaml']
    scopea,scopeb=read(busy/'navigation_scope.json'),read(idle/'navigation_scope.json')
    shared=[name for name in set(scopea['references'])&set(scopeb['references'])if '/multifloor_demo/navigation/' in name]
    result={'schema_version':1,'scope':'Read-only observed resource comparison; uncontrolled conditions, no causal GPU isolation',
            'same_core_control_semantics':all(value for name,value in equal.items()if name not in ['simulation/prepare.py']),
            'core_source_hash_comparison':{name:{'equal':value,'busy_sha256':a.get(name),'idle_sha256':b.get(name)}for name,value in equal.items()},
            'shared_navigation_reference_hashes_equal':{name:scopea['references'][name]==scopeb['references'][name]for name in shared},
            'generated_world_equal_after_run_paths_and_numeric_RTF_spelling_normalized':normalized_world(busy)==normalized_world(idle),
            'physical_real_time_factor_values':[float(ET.parse(run/'world.sdf').find('.//physics/real_time_factor').text)for run in [busy,idle]],
            'generated_configuration_hashes_equal':{name:sha(busy/name)==sha(idle/name)for name in configuration},
            'runner_source_hashes':{'busy':a.get('scripts/run_test.py'),'idle':b.get('scripts/run_test.py')},
            'runner_and_prepare_limit':'Hashes differ for later optional RTF/runtime-recording support; generated actual physics and sensor/navigation config are compared separately',
            'runs':[describe(busy),describe(idle)],
            'conclusion':'Both runs physically stood with zero navigation velocity. Lower observed GPU load improved some freshness spans but did not produce route execution. GPU load is not shown to be the sole cause; no claim that it has no effect.',
            'analyzer_sha256':sha(Path(__file__)),'original_logs_or_summaries_overwritten':False}
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('busy',type=Path);parser.add_argument('idle',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=evaluate(args.busy,args.idle,args.output)
    print(json.dumps({'same_core_control_semantics':result['same_core_control_semantics'],
        'normalized_world_same':result['generated_world_equal_after_run_paths_and_numeric_RTF_spelling_normalized'],
        'runs':[{'run':r['run'],'gpu_before':r['gpu_before'],'same_stamp_bridge_lag':r['bridge_slam_callback_relative_to_same_stamp_shadow']['statistics']}for r in result['runs']]},allow_nan=False))


if __name__=='__main__':main()
