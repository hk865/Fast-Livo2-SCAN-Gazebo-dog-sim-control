"""Pure file/handler checks: no socket, HTTP server, ROS or simulator starts."""
import argparse
import json
from pathlib import Path
from serve_r85 import View, handler

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path, required=True)
args = parser.parse_args()
view = View(args.run)
state = view.state()
result = view.read('r85_task_result.json')
assert state['acceptance'] == (result or None)
assert state['counts'] == view.read('map_metadata.json').get('counts', {})
assert state['ages'] == view.read('map_metadata.json').get('ages', {})
assert view.poses.offset == view.file('navigation_slam_poses.jsonl').stat().st_size
offset = view.poses.offset
assert view.state()['pose'] == state['pose'] and view.poses.offset == offset
assert state['trace'] and state['yaw'] is not None
if result.get('passed') is False:
    assert state['stage'] == 'failed'
elif result.get('passed') is True:
    assert state['stage'] == 'completed'

Handler = handler(view)
request = object.__new__(Handler)
captured = []
request.send = lambda body, mime='application/json', status=200: captured.append((status, mime, body))
for path in ['/', '/api/state', '/api/acceptance', '/api/frame', '/api/map/map_metadata.json']:
    request.path = path
    request.do_GET()
    assert captured[-1][0] == 200, path
name = view.read('map_metadata.json')['binary_filename']
request.path = '/api/map/' + name
request.do_GET()
assert captured[-1][0] == 200 and len(captured[-1][2]) % 16 == 0
for path in ['/api/mission', '/api/map/../../runtime_plan.json', '/api/map/colored_map.bad.bin']:
    request.path = path
    request.do_GET()
    assert captured[-1][0] == 404, path
request.do_POST()
assert captured[-1][0] == 405
html = Path(__file__).with_name('index.html').read_text()
assert "method:'POST'" not in html and "$('start')" not in html
print(json.dumps(dict(passed=True, run_id=view.run.name, stage=state['stage'],
                     displayed_trace_points=len(state['trace']), accepted_plan_points=len(state['planned_path']),
                     map_points=state['map'].get('point_count'), no_socket_or_ROS_started=True), indent=2))
