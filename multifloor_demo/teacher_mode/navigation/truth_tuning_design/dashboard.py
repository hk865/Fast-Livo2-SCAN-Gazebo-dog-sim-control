#!/usr/bin/env python3
"""Read-only archived truth-calibration evidence viewer. No ROS/control imports."""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
from urllib.parse import parse_qs, urlsplit

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def rows(path):
    result = []
    try:
        with path.open() as stream:
            for line in stream:
                try:
                    result.append(json.loads(line))
                except ValueError:
                    # A running producer may have an unfinished final row.
                    continue
    except OSError:
        pass
    return result


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def curve_evidence(run):
    path=run/'summary_curve_independent.json'
    receipt=read_json(path)
    valid=isinstance(receipt,dict) and receipt.get('schema')=='independent_truth_teacher_continuous_curve_tracking/v1'
    matches=False
    if valid:
        inputs=receipt.get('verified_input_source_sha256',{})
        matches=bool(inputs) and all(digest(Path(name))==expected for name,expected in inputs.items())
        ancestor=receipt.get('ancestor_common',{})
        matches=matches and ancestor.get('sha256')==digest(run/'summary_truth_pid.json')
    return receipt,{'path':str(path),'sha256':digest(path),'schema_and_input_chain_matches':valid and matches,
        'recorded_status':receipt.get('status','unverified') if valid else 'unverified',
        'scope':'Continuous curve physical acceptance remains separate from generic flat motion acceptance.'}


def active_hold_evidence(run):
    path=run/'summary_active_hold_independent.json'
    receipt=read_json(path)
    valid=(isinstance(receipt,dict) and
           receipt.get('schema') in ('independent_truth_teacher_active_endpoint_hold/v2','independent_truth_teacher_active_endpoint_hold/v4') and
           receipt.get('status') in ('passed','failed','unverified'))
    matches=False
    if valid:
        inputs=receipt.get('verified_input_source_sha256',{})
        common=receipt.get('ancestors',{}).get('common',{})
        matches=(bool(inputs) and all(digest(Path(name))==expected for name,expected in inputs.items()) and
                 common.get('sha256')==digest(run/'summary_truth_pid.json') and
                 common.get('status')==(read_json(run/'summary_truth_pid.json') or {}).get('status'))
    return receipt,{'path':str(path),'sha256':digest(path),
        'schema_and_input_chain_matches':bool(valid and matches),
        'recorded_status':receipt.get('status','unverified') if valid else 'unverified',
        'scope':'New fixed endpoint active pose hold; small nonzero Teacher velocity commands permitted. Original zero-command parking status is preserved.'}


def combined_evidence(run, common, terrain):
    path = run / 'summary_truth_motion_combined.json'
    receipt = read_json(path)
    ancestors = receipt.get('ancestors', {}) if isinstance(receipt, dict) else {}
    verification = {}
    if isinstance(receipt, dict):
        for name, original, filename in (('common', common, 'summary_truth_pid.json'),
                                          ('terrain', terrain, 'summary_truth_terrain.json')):
            ancestor = ancestors.get(name)
            if name == 'terrain' and receipt.get('terrain_kind') == 'flat':
                verification[name] = {'applicable': False, 'matches': True}
                continue
            actual_hash = digest(run / filename)
            matches = (isinstance(ancestor, dict) and isinstance(original, dict) and
                       ancestor.get('sha256') == actual_hash and
                       ancestor.get('status') == original.get('status'))
            verification[name] = {'applicable': True, 'matches': matches,
                                  'declared_sha256': ancestor.get('sha256') if isinstance(ancestor, dict) else None,
                                  'current_sha256': actual_hash,
                                  'original_status': original.get('status') if isinstance(original, dict) else None}
    schema_valid = (isinstance(receipt, dict) and
                    receipt.get('schema') == 'truth_teacher_motion_combined_receipt/v1' and
                    receipt.get('status') in ('passed', 'failed', 'unverified') and
                    receipt.get('navigation_ground_truth_used') is True and
                    receipt.get('SLAM_navigation_verified') is False and
                    receipt.get('real_robot_verified') is False)
    return receipt, {'path': str(path), 'sha256': digest(path),
                     'original_status': receipt.get('status', 'unverified') if isinstance(receipt, dict) else 'unverified',
                     'available': isinstance(receipt, dict),
                     'schema_and_scope_valid': schema_valid,
                     'ancestor_chain_matches': schema_valid and bool(verification) and all(v['matches'] for v in verification.values()),
                     'ancestor_verification': verification,
                     'scope': 'Append-only truth motion conclusion; original common/terrain statuses remain separate.',
                     'overwrites_original_receipts': False}


def frame_inventory(run, camera):
    directory = run / 'vehicle_rgb' if camera == 'vehicle' else run
    output = []
    for path in (directory / 'frames').glob('*.jpg'):
        match = re.fullmatch(r'(\d+)_(\d{9})\.jpg', path.name)
        if match:
            output.append({'name': path.name, 'stamp_s': int(match[1]) + int(match[2]) / 1e9})
    output.sort(key=lambda row: row['stamp_s'])
    return {'frames': output, 'source': read_json(directory / 'frame_source.json'),
            'camera_info': read_json(run / 'camera_info' / (camera + '.json'))}


def run_payload(run):
    profile = read_json(run / 'truth_profile.json')
    summary = read_json(run / 'summary_truth_pid.json')
    telemetry = rows(run / 'telemetry.jsonl')
    control = rows(run / 'control.jsonl')
    # Every entry is one original Actor50Hz frame, without synthetic interpolation.
    data = []
    for row in telemetry:
        try:
            data.append([row['sim_time'], row['state_physics_world_time'],
                         *row['position'], *row['body_lin_vel'], *row['body_ang_vel'],
                         *row['rpy'], *row['command'], *row['requested']])
        except (KeyError, TypeError):
            continue
    references = [{key: row.get(key) for key in
                   ('elapsed_s', 'feedback_time_s', 'controller_updated', 'mode', 'segment',
                    'reference_yaw', 'reference_velocity_world', 'v_reference_body',
                    'error_cross', 'error_yaw', 'command_body')}
                  for row in control]
    # An append-only terrain receipt remains separate from the common v2 result.
    # Its original status must never overwrite the run summary or upgrade flat.
    terrain_path = run / 'summary_truth_terrain.json'
    terrain = read_json(terrain_path)
    combined, combined_meta = combined_evidence(run, summary, terrain)
    curve,curve_meta=curve_evidence(run)
    active_hold,active_hold_meta=active_hold_evidence(run)
    return {'run': run.name, 'profile': profile, 'summary': summary,
            'summary_active_hold_independent':active_hold,'active_hold_evidence':active_hold_meta,
            'summary_curve_independent':curve,'curve_evidence':curve_meta,
            'summary_truth_motion_combined': combined,
            'combined_evidence': combined_meta,
            'summary_truth_terrain': terrain,
            'terrain_evidence': {'path': str(terrain_path),
                                 'sha256': digest(terrain_path),
                                 'status': terrain.get('status', 'unverified') if isinstance(terrain, dict) else 'unverified',
                                 'available': isinstance(terrain, dict),
                                 'overwrites_common_benchmark': False},
            'protocol': read_json(run / 'sources/truth/protocol.json'),
            'policy': read_json(run / 'policy_manifest.json'),
            'runtime': read_json(run / 'runtime_manifest.json'),
            'sources': read_json(run / 'source_manifest.json'),
            'hashes': {name: digest(run / name) for name in
                       ('truth_profile.json', 'summary_truth_pid.json', 'source_manifest.json')},
            'field_order': ['elapsed_s', 'physical_world_s', 'x', 'y', 'z',
                            'COM_vx_body', 'COM_vy_body', 'COM_vz_body',
                            'omega_x_body', 'omega_y_body', 'omega_z_body',
                            'roll', 'pitch', 'yaw', 'actor_cmd_vx', 'actor_cmd_vy',
                            'actor_cmd_wz', 'requested_vx', 'requested_vy', 'requested_wz'],
            'telemetry': data, 'control': references,
            'cameras': {camera: frame_inventory(run, camera) for camera in ('overview', 'vehicle')},
            'scope': 'Gazebo 真值反馈仿真标定；不计入 SLAM 融合、导航集成或真机验证。',
            'archive_only': True,
            'provenance': '原始 telemetry / control 日志与 Gazebo RGB 归档；未插值、未生成替代画面。'}


def make_handler(root):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, payload, content_type='application/json; charset=utf-8', status=200):
            if not isinstance(payload, bytes):
                payload = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

        def run_directory(self, query):
            name = query.get('run', [''])[0]
            if not re.fullmatch(r'[A-Za-z0-9_-]+', name) or '_truth_pid_' not in name:
                raise ValueError('Invalid truth run')
            directory = (root / 'runs' / name).resolve()
            if directory.parent != (root / 'runs').resolve() or not (directory / 'truth_profile.json').is_file():
                raise ValueError('Unknown truth run')
            return directory

        def do_GET(self):
            request = urlsplit(self.path)
            query = parse_qs(request.query)
            try:
                if request.path in ('/', '/index.html'):
                    self.send((HERE / 'dashboard.html').read_bytes(), 'text/html; charset=utf-8')
                elif request.path == '/api/runs':
                    inventory = []
                    for profile_path in (root / 'runs').glob('*_truth_pid_*/truth_profile.json'):
                        run = profile_path.parent
                        profile, summary = read_json(profile_path), read_json(run / 'summary_truth_pid.json')
                        terrain = read_json(run / 'summary_truth_terrain.json')
                        combined, combined_meta = combined_evidence(run, summary, terrain)
                        curve,curve_meta=curve_evidence(run)
                        active_hold,active_hold_meta=active_hold_evidence(run)
                        inventory.append({'name': run.name, 'profile': profile,
                                          'active_hold_status':active_hold_meta['recorded_status'],
                                          'active_hold_chain_matches':active_hold_meta['schema_and_input_chain_matches'],
                                          'status': summary.get('status') if summary else 'pending',
                                          'terrain_status': terrain.get('status') if terrain else 'unverified',
                                          'combined_status': combined.get('status') if combined else None,
                                          'combined_chain_matches': combined_meta['ancestor_chain_matches'],
                                          'curve_status':curve_meta['recorded_status'],
                                          'curve_chain_matches':curve_meta['schema_and_input_chain_matches'],
                                          'score': summary.get('score') if summary else None})
                    self.send(sorted(inventory, key=lambda row: row['name'], reverse=True))
                elif request.path == '/api/run':
                    self.send(run_payload(self.run_directory(query)))
                elif request.path == '/api/frame':
                    run = self.run_directory(query)
                    camera = query.get('camera', ['overview'])[0]
                    if camera not in ('overview', 'vehicle'):
                        raise ValueError('Unknown camera')
                    name = query.get('name', [''])[0]
                    if not re.fullmatch(r'\d+_\d{9}\.jpg', name):
                        raise ValueError('Invalid archived frame')
                    path = (run / 'vehicle_rgb' if camera == 'vehicle' else run) / 'frames' / name
                    self.send(path.read_bytes(), 'image/jpeg')
                else:
                    self.send({'error': 'Unknown read-only view'}, status=404)
            except (OSError, ValueError) as error:
                self.send({'error': str(error)}, status=400)

    return Handler


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8769)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('Invalid local port')
    print(json.dumps({'url': 'http://127.0.0.1:' + str(args.port), 'read_only': True,
                      'truth_feedback_calibration': True, 'SLAM_navigation_claim': False}), flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(ROOT)).serve_forever()
