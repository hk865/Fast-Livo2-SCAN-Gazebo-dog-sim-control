#!/usr/bin/env python3
"""Loopback, GET-only viewer for actual SLAM fixed-route transfer evidence.

No ROS, simulator, process-control, command writer or training imports. The
viewer never modifies runs. An independent receipt is shown with its own SHA
and current input identity, without upgrading the original runner summary.
"""
import argparse
from collections import Counter
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
from urllib.parse import parse_qs, urlsplit

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
CACHE={}


def read_json(path):
    try:return json.loads(Path(path).read_text())
    except (OSError,ValueError):return None


def rows(path):
    result=[]
    try:
        with Path(path).open() as stream:
            for line in stream:
                try:result.append(json.loads(line))
                except ValueError:pass  # A live final partial row is not evidence.
    except OSError:pass
    return result


def sha(path):
    p=Path(path)
    try:
        st=p.stat();key=(str(p.resolve()),st.st_size,st.st_mtime_ns)
        if key not in CACHE:
            h=hashlib.sha256()
            with p.open('rb') as stream:
                for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
            CACHE[key]=h.hexdigest()
        return CACHE[key]
    except OSError:return None


def frames(run,camera):
    directory=run/'vehicle_rgb' if camera=='vehicle' else run
    inventory=[]
    for p in (directory/'frames').glob('*.jpg'):
        match=re.fullmatch(r'(\d+)_(\d{9})\.jpg',p.name)
        if match:inventory.append({'name':p.name,'stamp_s':int(match[1])+int(match[2])/1e9})
    inventory.sort(key=lambda r:r['stamp_s'])
    return {'frames':inventory,'latest_source':read_json(directory/'frame_source.json'),
        'actual_camera_info':read_json(run/'camera_info'/(camera+'.json'))}


def receipt_evidence(run):
    path=run/'summary_slam_transfer_independent.json';receipt=read_json(path)
    if not isinstance(receipt,dict):return {'available':False,'status':'unverified','path':str(path)}
    schema=(receipt.get('schema')=='independent_actual_SLAM_fixed_route_teacher_transfer/v1'
        and receipt.get('navigation_ground_truth_used') is False and receipt.get('real_robot_verified') is False
        and receipt.get('SCAN_navigation_verified') is False and receipt.get('full_multifloor_navigation_verified') is False
        and receipt.get('score') is None)
    conflicts=[];missing=[]
    for name,digest in receipt.get('input_sha256',{}).items():
        actual=sha(name)
        if actual is None:missing.append(name)
        elif actual!=digest:conflicts.append(name)
    match=schema and bool(receipt.get('input_sha256')) and not conflicts and not missing
    return {'available':True,'path':str(path),'sha256':sha(path),'receipt':receipt,
        'schema_scope_valid':schema,'current_original_input_chain_matches':match,
        'missing_input_paths':missing,'changed_input_paths':conflicts,
        'status':receipt.get('status') if match else 'unverified'}


def run_index(root):
    output=[]
    for run in root.iterdir():
        if not run.is_dir() or run.is_symlink() or not (run/'slam_execution.json').is_file():continue
        execution=read_json(run/'slam_execution.json')
        if not isinstance(execution,dict) or execution.get('schema')!='teacher_slam_fixed_route_execution/v1':continue
        runtime=read_json(run/'runtime_manifest.json');original=read_json(run/'summary_slam_fixed_route.json')
        independent=read_json(run/'summary_slam_transfer_independent.json')
        output.append({'id':run.name,'runtime_status':None if runtime is None else runtime.get('runtime_status'),
            'runtime_error':None if runtime is None else runtime.get('error'),
            'original_runner_status':'unverified' if original is None else original.get('status'),
            'independent_recorded_status':'unverified' if independent is None else independent.get('status'),
            'independent_status_requires_input_chain_check':True})
    return sorted(output,key=lambda r:r['id'],reverse=True)


def payload(run):
    folder=run/'SLAM_fixed_route';poses=rows(folder/'adapter_slam_poses.jsonl');imus=rows(folder/'adapter_IMU_inputs.jsonl')
    controls=rows(folder/'adapter_controller_updates.jsonl');commands=rows(folder/'adapter_commands.jsonl')
    attempts=rows(run/'slam_execution_commands.jsonl');telemetry=rows(run/'telemetry.jsonl')
    anchors=rows(folder/'adapter_route_anchor.jsonl');source=read_json(folder/'adapter_source_receipt.json')
    rejected=Counter(str(r.get('reason','missing reason')) for r in attempts if r.get('rejected') is True)
    source_rejected=Counter((str(r.get('source'))+': '+str(r.get('reason'))) for r in rows(folder/'adapter_rejections.jsonl'))
    updates=[r for r in controls if isinstance(r.get('core_original_row'),dict) and r['core_original_row'].get('controller_updated') is True]
    ns=[r['envelope'].get('control_pose_stamp_ns') for r in updates]
    rate=None if len(ns)<2 or ns[-1]<=ns[0] else (len(ns)-1)*1e9/(ns[-1]-ns[0])
    trace=[]
    for r in telemetry:
        try:trace.append([r['world_sim_time'],r['state_physics_world_time'],*r['body_lin_vel'],*r['body_ang_vel'],*r['command'],*r['requested'],r.get('fault')])
        except (KeyError,TypeError):pass
    return {'id':run.name,'absolute_run_path':str(run),'runtime':read_json(run/'runtime_manifest.json'),
        'original_runner_summary':read_json(run/'summary_slam_fixed_route.json'),
        'independent':receipt_evidence(run),'execution':read_json(run/'slam_execution.json'),
        'binding':read_json(run/'slam_binding.json'),'profile':read_json(run/'frozen_controller_profile.json'),
        'source_receipt':source,'writer':read_json(folder/'adapter_writer_receipt.json'),
        'cleanup':read_json(folder/'adapter_cleanup_receipt.json'),'anchor_rows':anchors,
        'source_SLAM_points':[{k:r.get(k) for k in ('stamp_ns','accepted','position','quaternion_xyzw','received_monotonic_wall')} for r in poses],
        'actual_rate':{'math_update_rows':len(updates),'all_new_source_controller_rows':len(controls),
            'source_math_rate_hz':rate,'distinct_math_headers':len(set(ns)),
            'selected_calibration_hz':None if source is None else source.get('selected_calibration_feedback_hz'),
            'mapped_SLAM_ceiling_hz':10,'accepted_SLAM_headers':sum(r.get('accepted') is True for r in poses),
            'accepted_IMU_headers':sum(r.get('accepted') is True for r in imus),
            'IMU_orientation_unavailable_headers':sum(r.get('orientation_available') is False for r in imus)},
        'executor_read_rejections':dict(rejected),'producer_source_rejections':dict(source_rejected),
        'executor_read_count':len(attempts),'producer_heartbeat_count':len(commands),
        'native_COM_and_actual_Teacher_commands':trace,
        'overview':frames(run,'overview'),'vehicle':frames(run,'vehicle'),
        'claims':{'outer_ground_truth_used':False,'actor_privileged_observations':True,
            'actor_privileged_dimension_count':232,'actual_SLAM_route_frame':'camera_init',
            'absolute_scene_centerline_registration':'unverified','SCAN':'unverified','full_multifloor':'unverified','hardware':'unverified',
            'COM_plot_is_offline_physics_diagnostic_not_SLAM_navigation_input':True}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8770)
    parser.add_argument('--runs',type=Path,default=ROOT/'runs')
    args=parser.parse_args();root=args.runs.resolve()
    if not root.is_dir():raise ValueError('Actual runs directory missing')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,data,ctype='application/json; charset=utf-8',code=200):
            self.send_response(code);self.send_header('Content-Type',ctype);self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def do_GET(self):
            parsed=urlsplit(self.path);query=parse_qs(parsed.query)
            if parsed.path in ('/','/index.html'):
                return self.send((HERE/'slam_transfer_dashboard.html').read_bytes(),'text/html; charset=utf-8')
            if parsed.path=='/api/runs':return self.send(json.dumps(run_index(root),ensure_ascii=False).encode())
            run_id=query.get('run',[''])[0]
            if not re.fullmatch(r'[A-Za-z0-9_.-]+',run_id) or run_id not in {r['id'] for r in run_index(root)}:
                return self.send(b'Unknown actual run','text/plain',404)
            run=(root/run_id).resolve()
            if run.parent!=root:return self.send(b'Forbidden','text/plain',403)
            if parsed.path=='/api/run':return self.send(json.dumps(payload(run),ensure_ascii=False,allow_nan=False).encode())
            if parsed.path=='/frame':
                camera=query.get('camera',['overview'])[0];name=query.get('name',['latest'])[0]
                if camera not in ('overview','vehicle'):return self.send(b'Unknown camera','text/plain',404)
                directory=run/'vehicle_rgb' if camera=='vehicle' else run
                if name=='latest':path=directory/'frame.jpg'
                elif re.fullmatch(r'\d+_\d{9}\.jpg',name):path=directory/'frames'/name
                else:return self.send(b'Unknown actual frame','text/plain',404)
                if not path.is_file():return self.send(b'Actual frame unavailable','text/plain',404)
                return self.send(path.read_bytes(),'image/jpeg')
            self.send(b'Not found','text/plain',404)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print('Read-only actual SLAM transfer viewer: http://127.0.0.1:'+str(args.port),flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()


if __name__=='__main__':main()
