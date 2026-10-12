"""Offline replay of the TWO existing real R7 sensor captures; no ROS init."""
from pathlib import Path
import hashlib
import json
import struct
import time
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from known_scene_matcher import KnownSceneMatcher, SceneReference, MatchPolicy
from known_scene_adapter import ExactSourceJoin, odometry_transform, pointcloud_xyz

RUN = Path('/__LOCAL_HOME__/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261007_231900_closed_loop_cascade_clock_hold_R7_F2_two_groups_24MiB_cbfc')
STAGE = Path(__file__).resolve().parent
REFERENCE_SHA = 'd98466feb013e0d30b4f087e3b35f7e647480d2acc458c5aac1d223f0ff6c37d'
WORLD_SHA = '90ac15b8b8ec45ee234088e602ca63f5b9d2d85d0cba3b2d861efb3addbfe203'


def load_reference():
    return SceneReference.load(STAGE / 'REFERENCE_GEOMETRY.json', REFERENCE_SHA, WORLD_SHA)


def load_capture(label):
    folder = RUN / 'bounded_shadow_samples' / label
    bundle = json.loads((folder / 'bundle.json').read_text())
    for name, binding in bundle['files'].items():
        raw = (folder / name).read_bytes()
        assert len(raw) == binding['bytes']
        assert hashlib.sha256(raw).hexdigest() == binding['sha256']
    messages = dict(raw=deserialize_message((folder / 'raw.cdr').read_bytes(), PointCloud2),
                    adapted=deserialize_message((folder / 'filtered.cdr').read_bytes(), PointCloud2))
    with (folder / 'sync.bin').open('rb') as f:
        assert f.read(16) == b'GO2BOUNDED001LE\0'
        while True:
            header = f.read(16)
            if not header:
                break
            m, n = struct.unpack('<QQ', header)
            assert m < 8192 and n < 8192
            meta = json.loads(f.read(m))
            raw = f.read(n)
            assert hashlib.sha256(raw).hexdigest() == meta['body_sha256']
            if meta['role'] in ('body', 'mapper'):
                messages[meta['role']] = deserialize_message(raw, Odometry)
    # The full registered CDR was not retained. Only reconstruct a pointcloud
    # carrier for the *saved* canonical XYZ, and label it offline fixture.
    xyz = np.load(folder / 'registered_xyz.npy', allow_pickle=False)
    assert hashlib.sha256(np.ascontiguousarray(xyz, dtype='<f8').tobytes()).hexdigest() == bundle['registered_canonical_xyz_sha256']
    registered = PointCloud2()
    registered.header = messages['mapper'].header
    registered.height, registered.width = 1, len(xyz)
    registered.point_step, registered.row_step = 12, 12 * len(xyz)
    registered.is_bigendian, registered.is_dense = False, True
    from sensor_msgs.msg import PointField
    registered.fields = [PointField(name=name, offset=4 * i, datatype=7, count=1)
                         for i, name in enumerate(('x', 'y', 'z'))]
    registered.data = np.ascontiguousarray(xyz, dtype='<f4').tobytes()
    messages['registered'] = registered
    join = ExactSourceJoin(dict(valid=False, reasons=['historical_loaded_producer_witness_missing']),
                           interval_ns=0)
    now = time.monotonic_ns()
    for role, msg in messages.items():
        assert join.put(role, msg, now)
    job = join.pop_latest(bundle['source_ns'], now)
    assert job is not None and not job.get('rejected'), job
    return job, bundle


def replay():
    reference = load_reference()
    matcher = KnownSceneMatcher(reference)
    started = time.monotonic()
    reports = []
    baseline_c = np.eye(4)
    baseline_c[2, 3] = .3
    for label in ('F2_before_drop', 'F2_after_drop'):
        job, bundle = load_capture(label)
        result = matcher.match(job['source_ns'], job['body_points'], job['T_odom_body'],
                               current_c=baseline_c,
                               provenance=job['provenance'])
        result['label'] = label
        result['saved_registered_CDR_available'] = False
        result['offline_registered_xyz_carrier_only'] = True
        result['public_scene_frame_origin_world_m'] = [0., 0., .3]
        if 'T_world_body' in result:
            result['navigation_body_height_m'] = result['T_world_body'][2][3] - .3
        reports.append(result)
    return dict(schema='v34_known_scene_R7_real_sensor_replay/v1',
                source_run=str(RUN), duration_s=time.monotonic() - started,
                original_R7_time_evidence_admitted=False, runtime_adoption=False,
                results=reports)


if __name__ == '__main__':
    report = replay()
    path = STAGE / 'R7_REAL_SENSOR_REPLAY_V2.json'
    with path.open('x') as f:
        json.dump(report, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'results'}))
    for result in report['results']:
        print(json.dumps(dict(label=result['label'], reason=result['reason'],
                              duration_s=result['duration_s'], accepted=result['accepted'],
                              candidates=[dict(z=q['T_world_body'][2][3],
                                               rms_m=q['rms_capped_m'],
                                               rays=q['rays'], quality=q['quality_passed'],
                                               rejected=q['rejections']) for q in result['candidates']]), allow_nan=False))
