#!/usr/bin/env python3
"""Index a NEW actual bag offline; never start ROS nodes or alter the bag."""
import argparse
import hashlib
import json
from pathlib import Path
from input_identity import SCHEMA, TOPICS, canonical, sha, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--storage-id', choices=('mcap', 'sqlite3'), default='mcap')
    parser.add_argument('--out', type=Path, required=True, help='New directory; refuses existing output')
    args = parser.parse_args()
    bag, out = args.bag.resolve(), args.out.resolve()
    if not bag.is_dir() or not (bag / 'metadata.yaml').is_file():
        raise ValueError('Completed new bag directory/metadata required')
    if out.exists() or bag == out or bag in out.parents:
        raise ValueError('Output must be a new separate directory; do not modify captured bag')
    # Imports are offline message/CDR APIs, not rclpy.init or a ROS graph.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    metadata = rosbag2_py.MetadataIo().read_metadata(str(bag))
    if metadata.storage_identifier != args.storage_id:
        raise ValueError('Actual metadata storage differs from selected reader')
    if metadata.compression_mode not in ('', 'none'):
        raise ValueError('Use native MCAP chunk compression; outer compression would require a separate derivative workspace')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=args.storage_id),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if types != TOPICS:
        raise ValueError('Bag topic/type set must exactly match frontend-input-plus-clock contract')
    message_types = {topic: get_message(typ) for topic, typ in types.items()}
    out.mkdir(parents=True)
    rows = []
    with (out / 'captured_inputs.jsonl').open('x') as stream:
        while reader.has_next():
            topic, raw, received = reader.read_next()
            message = deserialize_message(raw, message_types[topic])
            stamp = message.clock if topic == '/clock' else message.header.stamp
            row = {'schema': SCHEMA, 'bag_record_index': len(rows), 'topic': topic, 'type': types[topic],
                   'source_ns': int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
                   'serialized_bytes': len(raw), 'cdr_sha256': hashlib.sha256(raw).hexdigest(),
                   'bag_received_timestamp_ns': int(received)}
            if topic != '/clock': row['frame_id'] = message.header.frame_id
            stream.write(canonical(row).decode() + '\n')
            rows.append(row)
    result = {
        'schema': 'go2_pipeline_v19_captured_input_manifest/v1', 'bag': str(bag),
        'storage_id': args.storage_id, 'metadata_sha256': sha(bag / 'metadata.yaml'),
        'index_sha256': sha(out / 'captured_inputs.jsonl'), **summary(rows),
        'bag_timestamp_domain': 'Recorder stored receive timestamp; not sensor acquisition wall clock',
        'bag_record_order_is_original_mapper_callback_order': False,
        'complete_corpus_available': True, 'raw_budget_bytes': 6 * 1024**3,
        'bag_files_bytes': sum(p.stat().st_size for p in bag.iterdir() if p.is_file()),
        'actual_numeric_or_performance_PASS': False,
    }
    result['topic_header_stats'] = {}
    for topic in TOPICS:
        stamps = [row['source_ns'] for row in rows if row['topic'] == topic]
        distinct = list(dict.fromkeys(stamps))
        result['topic_header_stats'][topic] = {
            'count': len(stamps), 'unique_stamps': len(distinct),
            'first_ns': stamps[0] if stamps else None, 'last_ns': stamps[-1] if stamps else None,
            'minimum_ns': min(stamps) if stamps else None, 'maximum_ns': max(stamps) if stamps else None,
            'duplicates': len(stamps) - len(distinct),
            'backward_transitions': sum(b < a for a, b in zip(stamps, stamps[1:])),
            'Hz': (len(distinct) - 1) * 1e9 / (distinct[-1] - distinct[0]) if len(distinct) > 1 and distinct[-1] > distinct[0] else None,
        }
    result['bag_message_count_metadata'] = metadata.message_count
    result['all_metadata_messages_indexed'] = len(rows) == metadata.message_count
    result['within_raw_budget'] = result['bag_files_bytes'] <= result['raw_budget_bytes']
    with (out / 'CAPTURED_INPUT_MANIFEST.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write('\n')
    reader.close()
    print(json.dumps({'out': str(out), 'record_count': len(rows), 'within_raw_budget': result['within_raw_budget']}))
    return 0 if result['within_raw_budget'] and result['complete_topic_set'] and result['all_metadata_messages_indexed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
