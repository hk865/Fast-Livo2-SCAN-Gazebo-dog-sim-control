#!/usr/bin/env python3
"""Offline actual CM motion response, using the unchanged passive observer.

Native integer body/truth headers are decoded from CDR without ROS init/nodes.
Adapter-applied Twist timing is the same historical cached-clock reconstruction
used in Full15, because Twist has no Header. GT only assesses the observer.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import struct
import sys

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
BASE = ROOT / 'simulation/test_results'
sys.path.insert(0, str(ROOT / 'navigation/test_results/motion_window_comparison'))
import compare_windows as shared


def capture(path):
    with path.open('rb') as handle:
        if handle.read(12) != b'STARTUPCDR1\n':
            raise ValueError('unexpected capture format')
        while size := handle.read(8):
            if len(size) != 8:
                raise ValueError('truncated capture size')
            n, m = struct.unpack('<II', size)
            metadata, cdr = handle.read(n), handle.read(m)
            if len(metadata) != n or len(cdr) != m:
                raise ValueError('truncated capture record')
            yield json.loads(metadata), cdr


def pose(message, metadata):
    stamp = message.header.stamp.sec*shared.NS + message.header.stamp.nanosec
    if stamp != metadata['header_stamp_ns']:
        raise ValueError('native CDR/header stamp mismatch')
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return dict(stamp_ns=stamp, stamp=stamp/shared.NS,
                p=[p.x,p.y,p.z], q=[q.x,q.y,q.z,q.w],
                captured_callback_wall_ns=metadata['wall_monotonic_ns'])


def normalize(kind):
    original = BASE / ('20261001_final_cm_'+kind)
    target = HERE / ('native_inputs_'+kind)
    target.mkdir(exist_ok=False)
    result = json.loads((original/'motion_result.json').read_text())
    decoded = []
    for metadata, cdr in capture(original/'actuator/actuator_startup.cdrlog'):
        if cdr and metadata['kind'] in ('slam', 'truth'):
            item = pose(deserialize_message(cdr, Odometry), metadata)
            item['source'] = metadata['kind']
            decoded.append(item)
    ps = sorted([row for row in decoded if row['source']=='slam'],key=lambda row:row['stamp_ns'])
    gt = {row['stamp_ns']:row for row in decoded if row['source']=='truth'}
    times = np.asarray(sorted(gt),dtype='int64')
    truth = [gt[int(t)] for t in times]
    if kind == 'fourturn_v2':
        start = round(result['events'][0]['stamp']*shared.NS)
        end = round(result['events'][-1]['stamp']*shared.NS)
        # Exactly one initial body/truth alignment before motion; never refit.
        first = ps[0]
        paired = shared.truth_pose(first['stamp_ns'],times,truth,np.eye(3),np.zeros(3))
        if paired is None:
            raise ValueError('first native body pose lacks bounded truth bracket')
        Rws = Rotation.from_quat(paired[1]).as_matrix() @ Rotation.from_quat(first['q']).as_matrix().T
        tws = np.asarray(paired[0])-Rws@np.asarray(first['p'])
        alignment = dict(stamp_ns=first['stamp_ns'],rotation_world_from_slam=Rws.tolist(),
                         translation_world_from_slam=tws.tolist(),scope='one first-pose SE3; GT evaluation only')
        contexts=[]
        for event in result['events']:
            if event.get('phase')=='pre_turn':
                contexts.append(dict(status=dict(request_id='fourturn-component',waypoint_index=event['segment'],
                    accepted_trajectory_id=None,steering=dict(stamp=event['stamp']))))
    else:
        start,end=map(lambda t:round(t*shared.NS),result['independent_truth_evaluation_interval'])
        original_alignment=result['initial_fixed_SE3_evaluation_only']
        alignment=dict(original_alignment)
        alignment['translation_world_from_slam']=alignment.pop('translation')
        contexts=result['statuses']
    for row in decoded:
        # Shared analysis recognizes these active labels. This normalization
        # declares only the original component's active interval, not a mission.
        row['stage']='navigating' if start<=row['stamp_ns']<=end else 'outside_component'
    with (target/'pose_audit.jsonl').open('x') as handle:
        for row in sorted(decoded,key=lambda row:row['stamp_ns']):
            handle.write(json.dumps(row)+'\n')
    with (target/'navigation_audit.jsonl').open('x') as handle:
        for row in contexts:
            handle.write(json.dumps(row)+'\n')
    for filename in ('joint_stop_adapter.jsonl','source_snapshot.tar.gz'):
        (target/filename).symlink_to(original/filename)
    (target/'alignment.json').write_text(json.dumps({'initial_SE3':alignment},indent=2)+'\n')
    receipt=dict(scope=__doc__,original_run=str(original),active_start_ns=start,active_end_ns=end,
        original_component_passed=result['passed'],original_checks=result['checks'],
        normalization_stage='navigating means original component-active interval; not full Demo mission',
        native_body_pose_count=len(ps),native_truth_pose_count=len(gt),
        alignment_once=True,truth_only_evaluation=True,
        actual_command_time='same Full15 method: adapter cached sim float -> ns; Twist has no native Header',
        fourturn_context='fixed target-heading segment, not a SCAN trajectory' if kind=='fourturn_v2' else None,
        input_sha256={str(original/f):shared.sha(original/f) for f in ['motion_result.json','actuator/actuator_startup.cdrlog','joint_stop_adapter.jsonl','source_snapshot.tar.gz','runtime_manifest.json','process_cleanup.json']},
        no_ROS_init_no_control_no_physics=True)
    (target/'normalization_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return target


def summarize(output):
    report=json.loads((output/'result.json').read_text())
    result={}
    for label,w in report['windows'].items():
        result[label]=dict(counts=w['counts'],reasons=w['reasons'],
            episode_summary=w['opposite_episodes_summary'],
            walk_valid=w['statistics'].get('walk_valid'),turn_valid=w['statistics'].get('turn_valid'))
    long_evidence=[]
    e=json.loads((output/'episodes_0.6s.json').read_text())
    for event in e['opposite_body_GT_confirmed_consistent_command_sign']['episodes']:
        if event['at_least_one_nominal_gait_cycle']:
            long_evidence.append(event)
    rows=[json.loads(line) for line in (output/'snapshots_0.6s.jsonl').open()]
    result['long_0p6_opposite_evidence']=long_evidence
    result['gt_confirms_each_valid_0p6_opposite']=[x for x in rows if x['classifications']['opposite_body_GT_confirmed_consistent_command_sign']]
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    return {label:dict(counts=value['counts'],episode_summary=value['episode_summary'])
            for label,value in result.items() if label.endswith('s')}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('kind',choices=('fourturn_v2','slope','dynamic'))
    args=parser.parse_args()
    normalized=normalize(args.kind)
    output=HERE/('comparison_'+args.kind)
    shared.compare(normalized,output,normalized/'alignment.json')
    summary=summarize(output)
    (output/'analysis_receipt.json').write_text(json.dumps(dict(
        script_sha256=shared.sha(Path(__file__)),shared_analysis_sha256=shared.sha(Path(shared.__file__)),
        shared_observer_sha256=shared.sha(shared.OBSERVER),result_sha256=shared.sha(output/'result.json'),
        summary_sha256=shared.sha(output/'summary.json'),normalization_sha256=shared.sha(normalized/'normalization_receipt.json'),
        same_fit_gates_windows_and_cycle_rule_as_full15=True,no_feedback_generated=True),indent=2)+'\n')
    print(json.dumps({'kind':args.kind,'summary':summary},indent=2))


if __name__=='__main__':
    main()
