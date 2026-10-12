#!/usr/bin/env python3
"""Read completed CameraInfo/RGB archives; no renderer or controller is started."""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(run):
    views = {}
    for view in ('overview', 'vehicle'):
        info = run / 'camera_info' / (view + '.jsonl')
        rows = [json.loads(line) for line in info.open() if line.strip()]
        pixels = run if view == 'overview' else run / 'vehicle_rgb'
        stamps = {(r['stamp_sec'], r['stamp_nsec']) for r in rows}
        unmatched = []
        frames = sorted((pixels / 'frames').glob('*.jpg'))
        for frame in frames:
            try:
                stamp = tuple(int(x) for x in frame.stem.split('_'))
            except ValueError:
                stamp = None
            if stamp not in stamps:
                unmatched.append(frame.name)
        last_source = pixels / 'frame_source.json'
        source = json.loads(last_source.read_text())
        final = rows[-1]
        dimensions_match = all(r['width'] == 640 and r['height'] == 480 for r in rows)
        zero_d = all(len(r['d']) == 5 and all(abs(x) < 1e-12 for x in r['d']) for r in rows)
        identity_r = all(r['r'] == [1., 0., 0., 0., 1., 0., 0., 0., 1.] for r in rows)
        last_pair = (source['stamp_sec'], source['stamp_nsec']) in stamps
        views[view] = dict(camera_info_samples=len(rows), archived_frames=len(frames),
            all_image_stamp_pairs_present=not unmatched, unmatched_frame_files=unmatched,
            last_frame_stamp_pair_present=last_pair, all_dimensions_640x480=dimensions_match,
            all_D_zero=zero_d, all_R_identity=identity_r, last_K=final['k'], last_D=final['d'],
            calculated_horizontal_fov_deg=math.degrees(2 * math.atan(320 / final['k'][0])),
            source=final['source'], topic=final['topic'], frame=final['frame'],
            camera_info_sha256=sha(info), last_frame_sha256=sha(pixels / 'frame.jpg'),
            last_frame_source_sha256=sha(last_source),
            passed=bool(rows and frames and not unmatched and last_pair and dimensions_match and zero_d and identity_r))
    return dict(run=str(run), views=views, passed=all(v['passed'] for v in views.values()))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = dict(schema=1, recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='Actual simulated pinhole CameraInfo/image correspondence; not a hardware lens calibration',
        source_sha256=sha(Path(__file__)), runs=[audit(run.resolve()) for run in args.runs])
    result['status'] = 'passed' if all(r['passed'] for r in result['runs']) else 'failed'
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'status': result['status'], 'runs': len(result['runs']), 'output': str(args.output)}))
