#!/usr/bin/env python3
"""Read-only local dashboard for recorded Teacher Gazebo tests.

No ROS publisher, simulation control, or training process is created here.
"""
from __future__ import annotations

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlsplit


ROOT = Path(__file__).resolve().parents[1]


def finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def vector(value, size=3):
    if isinstance(value, dict):
        if isinstance(value.get('linear'), dict):
            linear, angular = value['linear'], value.get('angular', {})
            value = [linear.get('x'), linear.get('y'), angular.get('z')]
        else:
            keys = ('x', 'y', 'z') if any(k in value for k in ('x', 'y', 'z')) else ('vx', 'vy', 'wz')
            value = [value.get(k) for k in keys]
    if not isinstance(value, (list, tuple)) or len(value) < size:
        return None
    values = [finite_number(v) for v in value[:size]]
    return values if all(v is not None for v in values) else None


def first(row, names):
    for name in names:
        value = row.get(name)
        if value is not None:
            return value
    return None


def normalize(row):
    """Keep only display fields; never synthesize missing measurements."""
    if not isinstance(row, dict):
        return None
    stamp = finite_number(first(row, ('sim_time', 'sim', 't', 'time', 'stamp', 'clock')))
    if stamp is None and finite_number(row.get('stamp_ns')) is not None:
        stamp = float(row['stamp_ns']) / 1e9
    if stamp is None:
        return None
    command = vector(first(row, ('command', 'cmd', 'cmd_vel', 'requested', 'requested_command')))
    measured = vector(first(row, ('measured', 'measured_velocity', 'velocity_body', 'actual_velocity')))
    if measured is None:
        linear = vector(first(row, ('base_lin_vel', 'body_lin_vel', 'linear_velocity_body')))
        angular = vector(first(row, ('base_ang_vel', 'body_ang_vel', 'angular_velocity_body')))
        if linear is not None and angular is not None:
            measured = [linear[0], linear[1], angular[2]]
    position = vector(first(row, ('position', 'p', 'base_position', 'truth_position', 'ground_truth_position')))
    if position is None and isinstance(row.get('pose'), dict):
        position = vector(row['pose'].get('position'))
    rpy = vector(first(row, ('rpy', 'roll_pitch_yaw', 'attitude')))
    if rpy is None:
        roll, pitch, yaw = (finite_number(row.get(k)) for k in ('roll', 'pitch', 'yaw'))
        if None not in (roll, pitch, yaw):
            rpy = [roll, pitch, yaw]
    return {
        't': stamp, 'command': command, 'measured': measured, 'position': position,
        'rpy': rpy, 'contacts': first(row, ('contacts', 'foot_contacts', 'contact')),
        'state': first(row, ('state', 'mode', 'controller_state')),
        'test': first(row, ('test', 'test_name', 'case', 'phase')),
        'reason': first(row, ('reason', 'failure', 'error', 'fault')),
    }


def read_json(path):
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {'data': value}
    except (OSError, ValueError):
        return {}


def pid_receipt_view(directory):
    original = read_json(directory / 'summary_pid_navigation_independent.json')
    correction = read_json(directory / 'display_correction.json')
    selected = original
    verified = False
    if (correction.get('schema') == 'pid_analysis_display_correction/v1'
            and correction.get('original_filename') == 'summary_pid_navigation_independent.json'
            and correction.get('corrected_filename') in ('summary_pid_navigation_independent.corrected_v1.json',
                'summary_pid_navigation_independent.corrected_v2.json')
            and all(correction.get(k) is True for k in ('rawinputhashes_unchanged', 'analysis_only',
                'protocol_thresholds_unchanged', 'old_receipt_kept'))):
        corrected = read_json(directory / correction['corrected_filename'])
        version = 'v2' if correction['corrected_filename'].endswith('.corrected_v2.json') else 'v1'
        fix_path = ROOT / f'tests/pid_navigation/independent_receipt_fix_{version}.json'
        try:
            verified = bool(corrected and original and
                hashlib.sha256((directory / correction['original_filename']).read_bytes()).hexdigest() == correction['original_summary_sha256'] and
                hashlib.sha256((directory / correction['corrected_filename']).read_bytes()).hexdigest() == correction['corrected_summary_sha256'] and
                hashlib.sha256(fix_path.read_bytes()).hexdigest() == correction['fix_receipt_sha256'] and
                original.get('input_hashes') == corrected.get('input_hashes') and
                original.get('protocol') == corrected.get('protocol'))
        except (OSError, KeyError):
            verified = False
        if verified:
            selected = corrected
    return selected, original, {**correction, 'display_hashes_verified': verified} if correction else {}


def closed_loop_receipt(directory, filename, schema):
    """Select only this run's authored receipt; retain the original payload.

    This display check does not replay the evaluator or declare absent evidence
    successful. A copied receipt from another run and an internally inconsistent
    PASS remain unverified even when a filename looks authoritative.
    """
    path = directory / filename
    raw = read_json(path)
    if not raw:
        return {}
    errors = []
    if raw.get('schema') != schema:
        errors.append('Receipt schema does not match this result type')
    try:
        same_run = Path(raw.get('run', '')).resolve() == directory.resolve()
    except (TypeError, ValueError, OSError):
        same_run = False
    if not same_run:
        errors.append('Receipt belongs to another run')
    scope = raw.get('scope') or {}
    if schema != 'teacher_closed_loop_runtime_summary/v1':
        if not isinstance(scope, dict) or scope.get('navigation_ground_truth_used') is not False:
            errors.append('Actual sensor navigation source declaration is missing')
        checks = raw.get('checks')
        if not isinstance(checks, dict) or not checks:
            errors.append('Independent checks are missing')
            checks = {}
    else:
        checks = {}
    counts = {'passed': 0, 'failed': 0, 'unverified': 0, 'total': len(checks)}
    for item in checks.values():
        if isinstance(item, dict) and item.get('passed') is True and item.get('status') == 'passed':
            counts['passed'] += 1
        elif isinstance(item, dict) and item.get('passed') is False and item.get('status') == 'failed':
            counts['failed'] += 1
        else:
            counts['unverified'] += 1
    claimed = raw.get('status')
    if claimed == 'passed' and (not checks or counts['passed'] != counts['total']):
        errors.append('Claimed PASS contains missing or non-passing checks')
    if claimed == 'failed' and schema != 'teacher_closed_loop_runtime_summary/v1' and not counts['failed']:
        errors.append('Claimed FAIL has no explicit failed independent check')
    if claimed not in ('passed', 'failed', 'unverified'):
        errors.append('Independent result status is unsupported')
    try:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        errors.append('Receipt changed while reading')
        digest = None
    return {'filename': filename, 'sha256': digest, 'raw': raw,
            'valid_for_selected_run': not errors, 'validation_errors': errors,
            'status': claimed if not errors else 'unverified', 'counts': counts}


def closed_loop_receipt_view(directory):
    common = closed_loop_receipt(directory, 'summary_closed_loop_cascade_independent.json',
        'independent_actual_SLAM_SCAN_cascade_navigation/v1')
    ramp = closed_loop_receipt(directory, 'summary_closed_loop_ramp_independent.json',
        'independent_actual_SLAM_SCAN_complete_ramp_navigation/v1')
    runtime = closed_loop_receipt(directory, 'summary_closed_loop_navigation.json',
        'teacher_closed_loop_runtime_summary/v1')
    pilots = [closed_loop_receipt(directory, path.name,
        'independent_actual_SLAM_SCAN_complete_ramp_navigation/v1')
        for path in sorted(directory.glob('summary_closed_loop_ramp_independent.pilot_*.json'))]
    pilots = [receipt for receipt in pilots if receipt]
    scope = read_json(directory / 'navigation_scope.json')
    if not any((common, ramp, runtime, pilots)) and scope.get('schema') != 'teacher_closed_loop_navigation_scope/v1':
        return {}
    selected, kind = (ramp, 'ramp_independent') if ramp else (common, 'cascade_independent') if common else ({}, 'runtime_unverified')
    if selected:
        summary = dict(selected['raw'])
        if not selected['valid_for_selected_run']:
            # Render one explicit evidence rejection, rather than the foreign
            # or internally inconsistent receipt's apparent passing checks.
            summary = {'status': 'unverified', 'checks': {'selected_run_receipt_integrity': {
                'status': 'unverified', 'passed': None,
                'reason': '; '.join(selected['validation_errors'])}}}
        display_status = selected['status']
        counts = selected['counts'] if selected['valid_for_selected_run'] else {'passed': 0, 'failed': 0, 'unverified': 1, 'total': 1}
        note = '本轮独立验收；仅对应所选 run 的冻结场景、来源和判据。最终进程清理状态不替代验收。'
    else:
        # A clean runtime, a succeeded controller status, a pilot diagnostic,
        # and historical global/truth-PID acceptance cannot certify navigation.
        display_status = 'unverified'
        counts = {'passed': 0, 'failed': 0, 'unverified': 1, 'total': 1}
        summary = {'status': 'unverified', 'checks': {'closed_loop_independent_acceptance_pending': {
            'status': 'unverified', 'passed': None,
            'reason': '本轮真实 SLAM / SCAN 闭环尚无正式独立验收；模型接口、运行结束和历史结果不能代替。'}}}
        note = '运行记录待独立验收；当前不声明导航、完整坡道或多层通过。'
    return {'schema': 'teacher_closed_loop_dashboard_selection/v1',
            'status': display_status, 'counts': counts, 'selection_kind': kind,
            'selected_receipt_filename': selected.get('filename'),
            'selected_receipt_sha256': selected.get('sha256'),
            'selected_validation_summary': summary, 'display_note': note,
            'common': common, 'ramp': ramp, 'runtime': runtime,
            'pilot_diagnostics': pilots,
            'navigation_ground_truth_used': False,
            'historical_results_are_not_this_run_acceptance': True,
            'shutdown_status_is_not_independent_acceptance': True}


# The historical page is kept unchanged on disk. This additive read-only
# renderer understands the new receipt types without altering archived pages,
# camera selection, D/K display, RGB sources, or the actual SLAM/SCAN plot.
CLOSED_LOOP_PAGE_SCRIPT = r'''
<script>
(() => {
 const originalShow = show;
 const closedLoopNames = {
  execution_phase_status_and_cleanup_boundary:'运动阶段与进程清理边界',
  frozen_scope_and_archived_sources:'冻结范围、源码与存档哈希',
  actual_causal_SLAM_IMU_cascade_updates:'实际 SLAM / IMU 因果配对与反馈更新',
  actual_checked_SCAN_trajectory_payloads:'实际 SCAN 样条、原始数组与关联',
  fixed_goal_and_original_SCAN_bounded_projection:'固定目标与 SCAN 路径投影',
  actual_executor_ack_and_cascade_PI_COM_PD_replay:'实际执行回执与速度 PI / 路径 PD 重放',
  actual_raw_cloud_bytes_fields_filtering:'原始点云字节、字段与过滤来源',
  actual_cascade_movement_guard_raw_geometry:'实际最终命令方向的点云碰撞检查',
  actual_command_original_read_dual_TTL_slew:'实际读取、双时钟 300 ms 超时与命令过渡',
  all_original_SLAM_3D_region_arrivals:'全部目标区域的原始 SLAM 三维到达',
  runtime_CPU_single_writer_complete_and_drained:'CPU 推理、唯一执行器与完整退出',
  native_continuous_200Hz:'原始 200 Hz 物理记录连续性',
  native_force_velocity_and_support:'实际力矩、关节速度与足部支持',
  exclusive_Teacher_CPU_identity:'冻结 Teacher 身份与 CPU 唯一执行权',
  actual_sensor_graph_actor247_CPU_joint_execution:'实际传感器链、247 维 Actor 与关节执行',
  independent_offline_actual_route_speed_heading:'离线实测路线、速度与航向',
  new_first_declared_active_hold_fixed_5s:'首次声明后的固定 5 秒主动停车',
  strict_nonflat_complete_contact_geometry:'完整坡道接触几何（需专门验收）',
  all_unchanged_original_common_gates:'全部原闭环共同判据保持并通过',
  prospective_exact_ramp_contract:'事前冻结的完整坡道验收协议',
  original_source_authorized_causal_terrain_layer_switch:'实际 SLAM 来源触发的特权高度层切换',
  complete_original_required_ramp_evidence:'完整坡道、目的区域与停车证据',
  closed_loop_independent_acceptance_pending:'本轮正式独立验收待完成',
  selected_run_receipt_integrity:'所选 run 的收据来源与一致性'
 };
 const originalCheckName = checkName;
 checkName = name => closedLoopNames[name] || originalCheckName(name);
 const originalCheckDetail = checkDetail;
 checkDetail = item => {
  if (!Object.hasOwn(closedLoopNames,item.name) && !String(item.name).startsWith('leg_') && !String(item.name).startsWith('native_recheck_')) return originalCheckDetail(item);
  return item.reason || item.failure_reason || item.metrics?.reason || item.notes ||
   (status(item)[1]==='pass'?'本轮原始来源与事前判据的独立核验通过':status(item)[1]==='fail'?'本轮预定判据未通过；展开查看原始实测指标':'本轮尚无完整证明；不计为通过');
 };
 const closedLoopCaption = data => {
  const result=data.closed_loop_result;if(!result||!Object.keys(result).length)return;
  const nav=data.navigation||{},state=nav.state||{};
  $('navigationCaption').textContent=`实际 SLAM ${nav.poses?.records||0} 条 · SCAN ${nav.scan?.hash_verified?'样条 '+nav.scan.trajectory_id+'（原始数组哈希已核对）':'暂无已核对样条'} · 本轮独立验收：${status(result.status)[0]}。原始最后运行状态：${state.status??state.state??'等待状态'}（可能包含进程清理，不替代验收）。此图不使用 Gazebo 真值；规划曲线不代表已实际到达。`;
 };
 const originalNavigationPlot=navigationPlot;
 navigationPlot=()=>{originalNavigationPlot();if(current)closedLoopCaption(current);};
 show = data => {
  originalShow(data);
  const result=data.closed_loop_result;
  if (!result || !Object.keys(result).length) {const old=$('closedLoopReceipts');if(old)old.hidden=true;return;}
  const selected=result.selected_validation_summary||{};
  showTests(selected);
  $('legacyPanel').open=true;
  $('legacyTitle').textContent=result.selection_kind==='ramp_independent'?'本轮真实 SLAM / SCAN 完整坡道独立验收':'本轮真实 SLAM / SCAN 闭环独立验收';
  $('mainTitle').textContent='Teacher · 真实 SLAM / SCAN 闭环';
  $('mainSubtitle').textContent='实际传感器路线反馈、路径 PD 与速度 PI · CPU 推理 · 所选 run 的独立收据';
  $('pidCorrectionNote').hidden=false;
  $('pidCorrectionNote').textContent=result.display_note+(result.selected_receipt_filename?' 来源：'+result.selected_receipt_filename+' · SHA256 '+result.selected_receipt_sha256:'');
  closedLoopCaption(data);
  let panel=$('closedLoopReceipts');
  if(!panel){panel=document.createElement('section');panel.id='closedLoopReceipts';panel.className='panel tests';$('legacyPanel').after(panel);}
  panel.hidden=false;panel.replaceChildren();
  const heading=document.createElement('div');heading.className='panelhead';
  const title=document.createElement('h2');title.textContent='本轮收据与有限结论';heading.append(title);panel.append(heading);
  const rows=[['闭环共同判据',result.common],['正式完整坡道验收',result.ramp],
   ...(result.pilot_diagnostics||[]).map(r=>['坡道先导诊断（不升级通过）',r]),['运行记录（不认证导航）',result.runtime]].filter(([,r])=>r&&Object.keys(r).length);
  for(const [label,r] of rows){const block=document.createElement('details'),head=document.createElement('summary');
   let value=r.status;if(label.includes('不升级通过')&&value==='passed')value='unverified';if(label.includes('不认证导航'))value='unverified';
   head.textContent=label+' · '+status(value)[0]+' · '+r.filename;block.append(head);
   const note=document.createElement('p');note.className='caption';const c=r.counts||{};
   note.textContent=(c.total?`${c.total} 项：${c.passed} 通过 / ${c.failed} 失败 / ${c.unverified} 未验证。 `:'')+'SHA256 '+(r.sha256||'未核对')+(r.validation_errors?.length?'；显示完整性拒绝：'+r.validation_errors.join('；'):'')+(label.includes('不升级通过')?'；缺少事前协议的先导记录不能作为正式通过。':'');block.append(note);
   const raw=document.createElement('pre');raw.textContent=JSON.stringify(r.raw,null,2);block.append(raw);panel.append(block);
  }
  const source=document.createElement('p');source.className='caption';source.textContent='导航反馈：实际 SLAM / IMU / 点云与实际 SCAN。Gazebo 真值仅供离线运动验收；Actor 仍有 232 维特权输入，15 维为已知命令与上一动作。完整三层、自动原地图配准、动态障碍及真机仅以各自正式收据为准。';panel.append(source);
  const raw=$('raw');let original={};try{original=JSON.parse(raw.textContent);}catch(e){}raw.textContent=JSON.stringify({...original,closed_loop_result:result},null,2);
 };
})();
</script>
'''


def dashboard_page():
    page = (ROOT / 'web/index.html').read_text(encoding='utf-8')
    return page.replace('</body>', CLOSED_LOOP_PAGE_SCRIPT + '\n</body>').encode('utf-8')


def safe_json(value):
    """Nonfinite diagnostic values stay missing instead of breaking the UI."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    return value


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.entries = {}

    def read(self, path):
        with self.lock:
            try:
                stat = path.stat()
            except OSError:
                return {'rows': [], 'trajectory': [], 'latest': None, 'records': 0,
                        'invalid_records': 0, 'age_s': None, 'loading': False}
            key = str(path)
            entry = self.entries.get(key)
            identity = (stat.st_dev, stat.st_ino)
            if entry is None or entry['identity'] != identity or stat.st_size < entry['offset']:
                entry = {'identity': identity, 'offset': 0, 'partial': b'',
                         'rows': deque(maxlen=6000), 'trajectory': [], 'records': 0,
                         'invalid_records': 0, 'latest': None}
                self.entries[key] = entry
                # Bound memory when the user browses many archived runs.
                while len(self.entries) > 6:
                    self.entries.pop(next(iter(self.entries)))
            try:
                with path.open('rb') as stream:
                    stream.seek(entry['offset'])
                    chunk = stream.read(8 * 1024 * 1024)
                    entry['offset'] = stream.tell()
            except OSError:
                chunk = b''
            lines = (entry['partial'] + chunk).split(b'\n')
            entry['partial'] = lines.pop()
            # A partially written line is completed on the next poll.
            if len(entry['partial']) > 8 * 1024 * 1024:
                entry['partial'] = b''
                entry['invalid_records'] += 1
            for line in lines:
                if not line.strip():
                    continue
                try:
                    row = normalize(json.loads(line))
                except (ValueError, UnicodeDecodeError):
                    row = None
                if row is None:
                    entry['invalid_records'] += 1
                    continue
                entry['records'] += 1
                entry['latest'] = row
                entry['rows'].append(row)
                position = row['position']
                if position is not None:
                    point = [position[0], position[1], row['t']]
                    path_rows = entry['trajectory']
                    if not path_rows or math.hypot(point[0] - path_rows[-1][0], point[1] - path_rows[-1][1]) >= .015:
                        path_rows.append(point)
                    if len(path_rows) > 4000:
                        entry['trajectory'] = path_rows[::2]
            rows = list(entry['rows'])
            stride = max(1, math.ceil(len(rows) / 2000))
            displayed = rows[::stride]
            if rows and (not displayed or displayed[-1] is not rows[-1]):
                displayed.append(rows[-1])
            return {'rows': displayed, 'trajectory': entry['trajectory'], 'latest': entry['latest'],
                    'records': entry['records'], 'invalid_records': entry['invalid_records'],
                'age_s': None if entry['offset'] < stat.st_size else max(0., time.time() - stat.st_mtime),
                    'loading': entry['offset'] < stat.st_size}


class Dashboard:
    def __init__(self, runs):
        self.runs = runs.resolve()
        self.telemetry = Telemetry()
        self.plan_cache = {}

    def navigation_view(self, directory):
        """Only measured SLAM and an archived actual SCAN payload are drawn."""
        poses = self.telemetry.read(directory / 'navigation_slam_poses.jsonl')
        request = read_json(directory / 'navigation_request.json')
        result = {'poses': poses, 'request': request,
                  'state': read_json(directory / 'navigation_status.json'),
                  'scan': {}, 'source': 'Actual sensor SLAM in camera_init; no Gazebo truth input'}
        files = sorted((directory / 'navigation_trajectories').glob('*_trajectory_*.json'))
        if not files:
            return result
        record = files[-1]
        try:
            key = (str(record), record.stat().st_mtime_ns)
            if key not in self.plan_cache:
                import numpy as np
                data = read_json(record)
                archive = self.inside(record.parent / data['array_file'])
                if archive is None or hashlib.sha256(archive.read_bytes()).hexdigest() != data['array_sha256']:
                    raise ValueError('Actual SCAN array hash mismatch')
                with np.load(archive, allow_pickle=False) as saved:
                    samples = saved['samples']
                    if samples.ndim != 2 or samples.shape[1] != 3 or not np.isfinite(samples).all():
                        raise ValueError('Actual SCAN samples malformed')
                    stride = max(1, math.ceil(len(samples) / 2000))
                    points = samples[::stride].tolist()
                self.plan_cache[key] = {'trajectory_id': data['trajectory_id'],
                    'waypoint_index': data['waypoint_index'], 'points': points,
                    'array_sha256': data['array_sha256'], 'source': data['source'],
                    'file': record.name, 'hash_verified': True}
                while len(self.plan_cache) > 6:
                    self.plan_cache.pop(next(iter(self.plan_cache)))
            result['scan'] = self.plan_cache[key]
        except (OSError, ValueError, KeyError, ImportError):
            result['scan'] = {'error': 'Actual SCAN payload not available or hash not verified'}
        return result

    def inside(self, path):
        try:
            resolved = path.resolve()
            resolved.relative_to(self.runs)
            return resolved
        except (ValueError, OSError):
            return None

    def directories(self):
        try:
            children = list(self.runs.iterdir())
        except OSError:
            return []
        result = []
        for child in children:
            resolved = self.inside(child)
            if child.name != 'latest' and resolved is not None and resolved.is_dir():
                try:
                    modified = child.stat().st_mtime
                    summary = read_json(child / 'summary.json')
                    state = read_json(child / 'state.json')
                    closed_loop = closed_loop_receipt_view(child)
                    result.append({'id': child.name, 'modified_at': modified,
                                   'status': closed_loop.get('status') if closed_loop else summary.get('status', state.get('state', state.get('status'))),
                                   'tests': closed_loop['counts']['total'] if closed_loop else len(summary.get('tests', [])) if isinstance(summary.get('tests'), (dict, list)) else 0})
                except OSError:
                    continue
        return sorted(result, key=lambda item: item['modified_at'], reverse=True)

    def resolve(self, run):
        if run == 'latest':
            latest = self.inside(self.runs / 'latest')
            if latest is not None and latest.is_dir():
                return latest
            directories = self.directories()
            return self.inside(self.runs / directories[0]['id']) if directories else None
        if not run or run in {'.', '..'} or '/' in run or '\\' in run:
            return None
        path = self.inside(self.runs / run)
        return path if path is not None and path.is_dir() else None

    @staticmethod
    def frame(directory):
        candidates = [p for p in (directory / 'frame.jpg', directory / 'frame.png') if p.is_file()]
        return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None

    def snapshot(self, run):
        directory = self.resolve(run)
        if directory is None:
            return {'available': False, 'run_id': None, 'state': {}, 'summary': {},
                    'telemetry': {'rows': [], 'trajectory': [], 'latest': None}, 'frame': {'available': False},
                    'acceptance': read_json(self.runs / 'acceptance.json')}
        state_path = directory / 'state.json'
        frame = self.frame(directory)
        vehicle_frame = self.frame(directory / 'vehicle_rgb')
        try:
            state_age = max(0., time.time() - state_path.stat().st_mtime)
        except OSError:
            state_age = None
        original_functional = read_json(directory / 'summary_functional.json')
        reproduced = read_json(directory / 'summary_functional.reproduced.json')
        correction = read_json(directory / 'provenance_correction.json')
        corrected = bool(reproduced and correction.get('status') == 'reproduced_identical'
            and correction.get('all_checks_identical') is True and correction.get('all_metrics_identical') is True
            and original_functional.get('checks') == reproduced.get('checks')
            and original_functional.get('metrics') == reproduced.get('metrics'))
        ramp_original = read_json(directory / 'summary_full_ramp.json')
        stop_addendum = read_json(directory / 'full_ramp_stop_status_addendum.json')
        ramp_display = ramp_original
        if ramp_original and stop_addendum and stop_addendum.get('original_summary_sha256') == hashlib.sha256((directory / 'summary_full_ramp.json').read_bytes()).hexdigest():
            ramp_display = {**ramp_original, 'checks': {**ramp_original.get('checks', {}),
                'teacher_stop_on_destination_landing': stop_addendum['teacher_stop_on_destination_landing']},
                'display_note': 'Parking status uses separately archived clarification; original full-ramp receipt retained'}
        pid_display, pid_original, pid_correction = pid_receipt_view(directory)
        return {'available': True, 'run_id': directory.name, 'selected': run,
                'state': read_json(state_path), 'state_age_s': state_age,
                'summary': read_json(directory / 'summary.json'),
                'step_functional': reproduced if corrected else original_functional,
                'step_functional_original': original_functional,
                'step_provenance': correction,
                'step_functional_source': 'summary_functional.reproduced.json' if corrected else 'summary_functional.json',
                'step_safety': read_json(directory / 'supplemental_native_safety.json'),
                'full_ramp': ramp_display,
                'full_ramp_original': ramp_original,
                'full_ramp_stop_addendum': stop_addendum,
                'navigation_summary': read_json(directory / 'summary_navigation_independent.json'),
                'closed_loop_result': closed_loop_receipt_view(directory),
                'pid_navigation': pid_display,
                'pid_navigation_original': pid_original,
                'pid_navigation_correction': pid_correction,
                'pid_scope': read_json(directory / 'navigation_scope.json') if (directory / 'pid_profile_input.json').is_file() else {},
                'targeted_ttl': read_json(directory / 'summary_ttl_dropout_independent.json'),
                'dynamic_obstacle': read_json(directory / 'summary_dynamic_obstacle_independent.json'),
                'runtime_processes': read_json(directory / 'runtime_manifest.json').get('owned_processes', []),
                'navigation': self.navigation_view(directory),
                'replay': {'available': (directory / 'frame_replay.mp4').is_file() or (directory / 'frame_replay.gif').is_file(),
                           'mp4': (directory / 'frame_replay.mp4').is_file(),
                           'gif': (directory / 'frame_replay.gif').is_file(),
                           'manifest': read_json(directory / 'frame_replay_manifest.json') or read_json(directory / 'step_retest_evidence_manifest.json'),
                           'scope': 'Archived actual Gazebo RGB in sensor timestamp order; no interpolated or synthesized frames'},
                'step_plot': (directory / 'step_functional.png').is_file(),
                'acceptance': read_json(self.runs / 'acceptance.json'),
                'current_validation': read_json(self.runs.parent / 'current_status.json'),
                'telemetry': self.telemetry.read(directory / 'telemetry.jsonl'),
                'frame': {'available': frame is not None,
                          'age_s': max(0., time.time() - frame.stat().st_mtime) if frame else None,
                          'filename': frame.name if frame else None},
                'camera_info': {kind: read_json(directory / 'camera_info' / (kind + '.json')) for kind in ('overview','vehicle')},
                'vehicle_frame': {'available': vehicle_frame is not None,
                                  'age_s': max(0., time.time() - vehicle_frame.stat().st_mtime) if vehicle_frame else None,
                                  'filename': vehicle_frame.name if vehicle_frame else None},
                'diagnostic_source': 'Gazebo simulation truth for motion diagnostics; not a navigation input',
                'read_only': True}


def handler_for(dashboard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            # Polling should not bury simulation diagnostics in HTTP logs.
            if len(args) < 2 or str(args[1]) not in {'200', '304'}:
                super().log_message(format, *args)

        def send(self, data, content_type='application/json; charset=utf-8', status=200, headers=None):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def json(self, value, status=200):
            data = json.dumps(safe_json(value), ensure_ascii=False, allow_nan=False, default=str).encode('utf-8')
            self.send(data, status=status)

        def media(self, path, content_type):
            if path is None or dashboard.inside(path) is None or not path.is_file():
                self.json({'error': 'No recorded media'}, 404)
                return
            try:
                size = path.stat().st_size
                start, end, status = 0, size - 1, 200
                headers = {'Accept-Ranges': 'bytes'}
                value = self.headers.get('Range')
                if value:
                    # One byte range is enough for read-only HTML video seeking.
                    import re
                    match = re.fullmatch(r'bytes=(\d*)-(\d*)', value.strip())
                    if not match or not any(match.groups()):
                        self.send(b'', content_type, 416, {'Content-Range': f'bytes */{size}'})
                        return
                    first, last = match.groups()
                    if first:
                        start = int(first); end = min(size - 1, int(last)) if last else size - 1
                    else:
                        start = max(0, size - int(last))
                    if start > end or start >= size:
                        self.send(b'', content_type, 416, {'Content-Range': f'bytes */{size}'})
                        return
                    status = 206; headers['Content-Range'] = f'bytes {start}-{end}/{size}'
                with path.open('rb') as stream:
                    stream.seek(start); payload = stream.read(end - start + 1)
                self.send(payload, content_type, status, headers)
            except OSError:
                self.json({'error': 'Recorded media changed while reading'}, 404)

        def do_GET(self):
            parsed = urlsplit(self.path)
            run = parse_qs(parsed.query).get('run', ['latest'])[0]
            if parsed.path in {'/', '/index.html'}:
                try:
                    self.send(dashboard_page(), 'text/html; charset=utf-8')
                except OSError:
                    self.json({'error': 'Dashboard page is unavailable'}, 404)
            elif parsed.path == '/api/runs':
                self.json({'runs': dashboard.directories(), 'read_only': True})
            elif parsed.path == '/api/run':
                self.json(dashboard.snapshot(run))
            elif parsed.path == '/api/acceptance':
                self.json(read_json(dashboard.runs / 'acceptance.json'))
            elif parsed.path == '/api/frame':
                directory = dashboard.resolve(run)
                camera = parse_qs(parsed.query).get('camera', ['overview'])[0]
                if camera not in ('overview','vehicle'):
                    self.json({'error': 'Unknown camera'}, 400)
                    return
                frame = dashboard.frame(directory / 'vehicle_rgb' if camera == 'vehicle' else directory) if directory else None
                if frame is None or dashboard.inside(frame) is None:
                    self.json({'error': 'No recorded Gazebo frame'}, 404)
                    return
                try:
                    self.send(frame.read_bytes(), 'image/png' if frame.suffix == '.png' else 'image/jpeg')
                except OSError:
                    self.json({'error': 'Frame changed while reading'}, 404)
            elif parsed.path == '/api/replay':
                directory = dashboard.resolve(run)
                kind = parse_qs(parsed.query).get('kind', ['mp4'])[0]
                if kind not in {'mp4', 'gif'}:
                    self.json({'error': 'Unsupported recorded media type'}, 400)
                    return
                self.media(directory / ('frame_replay.' + kind) if directory else None, 'video/mp4' if kind == 'mp4' else 'image/gif')
            elif parsed.path == '/api/step_plot':
                directory = dashboard.resolve(run)
                self.media(directory / 'step_functional.png' if directory else None, 'image/png')
            else:
                self.json({'error': 'Not found'}, 404)

        def do_POST(self):
            self.json({'error': 'This dashboard is read-only'}, 405)

        do_PUT = do_POST
        do_DELETE = do_POST
        do_PATCH = do_POST
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8768)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--runs', type=Path, default=ROOT / 'runs')
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), handler_for(Dashboard(args.runs)))
    print(f'Teacher read-only dashboard: http://{args.host}:{args.port}/', flush=True)
    print(f'Recordings: {args.runs.resolve()}', flush=True)
    try:
        server.serve_forever(poll_interval=.3)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
