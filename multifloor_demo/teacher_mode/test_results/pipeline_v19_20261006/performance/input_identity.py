#!/usr/bin/env python3
"""Offline captured-input identity utilities; no ROS graph, launch or signals."""
import hashlib
import json
from collections import Counter
from pathlib import Path

SCHEMA = 'go2_pipeline_v19_captured_input_index/v1'
IDENTITY = ('topic', 'type', 'source_ns', 'serialized_bytes', 'cdr_sha256')
TOPICS = {
    '/demo/slam/lidar_filtered': 'sensor_msgs/msg/PointCloud2',
    '/demo/teacher/slam/imu': 'sensor_msgs/msg/Imu',
    '/demo/teacher/slam/image': 'sensor_msgs/msg/Image',
    '/clock': 'rosgraph_msgs/msg/Clock',
}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def identity(row):
    if row.get('topic') not in TOPICS or row.get('type') != TOPICS[row['topic']]:
        raise ValueError('Unknown topic/type or changed captured-input contract')
    for key in ('source_ns', 'serialized_bytes'):
        if type(row.get(key)) is not int or row[key] < 0:
            raise ValueError('Invalid unsigned integer ' + key)
    digest = row.get('cdr_sha256')
    if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
        raise ValueError('Invalid complete captured CDR digest')
    return tuple(row[key] for key in IDENTITY)


def load_index(path):
    path = Path(path).resolve()
    rows = []
    with path.open() as stream:
        for line in stream:
            row = json.loads(line)
            if row.get('schema') != SCHEMA or row.get('bag_record_index') != len(rows):
                raise ValueError('Foreign schema, missing row or discontinuous bag record order')
            identity(row)
            if type(row.get('bag_received_timestamp_ns')) is not int or row['bag_received_timestamp_ns'] < 0:
                raise ValueError('Invalid bag timestamp')
            rows.append(row)
    if not rows:
        raise ValueError('Empty index is not an input-identity PASS')
    return rows


def summary(rows):
    identities = [identity(row) for row in rows]
    return {
        'record_count': len(rows),
        'topic_counts': dict(Counter(row['topic'] for row in rows)),
        'serialized_bytes': sum(row['serialized_bytes'] for row in rows),
        'ordered_identity_sha256': hashlib.sha256(canonical(identities)).hexdigest(),
        'multiset_identity_sha256': hashlib.sha256(canonical(sorted(identities))).hexdigest(),
        'complete_topic_set': set(row['topic'] for row in rows) == set(TOPICS),
    }


def compare(left, right):
    a, b = [identity(row) for row in left], [identity(row) for row in right]
    exact, same_multiset = a == b, Counter(a) == Counter(b)
    complete = summary(left)['complete_topic_set'] and summary(right)['complete_topic_set']
    return {
        'schema': 'go2_pipeline_v19_captured_input_comparison/v1',
        'status': 'passed_captured_identity_only' if exact and complete else 'failed',
        'same_complete_CDR_multiset': same_multiset,
        'same_bag_record_order': exact,
        'all_required_topics_present': complete,
        'left': summary(left), 'right': summary(right),
        'original_production_callback_order_proven': False,
        'initial_estimator_map_state_proven': False,
        'actual_numeric_or_performance_PASS': False,
    }
