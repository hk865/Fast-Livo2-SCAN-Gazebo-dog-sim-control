#!/usr/bin/env python3
"""Report every recorded sustained-step retest, without rerunning the policy.

Default is a read-only plan. --write exports the report, real telemetry figures,
and a timestamped replay from captured JPEGs. It never changes acceptance.json,
legacy summaries, source images, or simulation state.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
CASES = ('step05_continue', 'step10_continue')
FROZEN_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'


def read(path):
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def sha(path):
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def clean(value):
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def dump(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(clean(value), ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def status(value):
    if value is True or value == 'passed':
        return 'passed'
    if value is False or value == 'failed':
        return 'failed'
    return 'unverified'


def telemetry(path):
    rows, invalid = [], 0
    try:
        with path.open() as stream:
            for line in stream:
                try:
                    row = json.loads(line)
                except ValueError:
                    invalid += 1; continue
                if isinstance(row, dict) and finite(row.get('sim_time')):
                    rows.append(row)
                else:
                    invalid += 1
    except OSError:
        pass
    return rows, invalid


def captured_frames(directory):
    rows, invalid = [], []
    for path in (directory / 'frames').glob('*.jpg'):
        match = re.fullmatch(r'(\d+)_(\d{9})\.jpg', path.name)
        if match:
            rows.append({'file': path, 'sensor_stamp_s': int(match[1]) + int(match[2]) / 1e9,
                         'sha256': sha(path)})
        else:
            invalid.append(path.name)
    rows.sort(key=lambda row: row['sensor_stamp_s'])
    return rows, invalid


def fixture(directory):
    try:
        root = ET.parse(directory / 'world.sdf').getroot()
        step = root.find("world/model[@name='teacher_low_step']")
        if step is None:
            return {}
        pose = list(map(float, step.findtext('pose').split()))
        size = list(map(float, step.findtext('link/collision/geometry/box/size').split()))
        return {'source': 'Actual run world.sdf teacher_low_step collision', 'pose_xyz_rpy': pose, 'size_xyz': size,
                'front_x_m': pose[0] - size[0]/2, 'rear_x_m': pose[0] + size[0]/2,
                'top_z_m': pose[2] + size[2]/2, 'world_sha256': sha(directory / 'world.sdf')}
    except (OSError, ValueError, AttributeError, ET.ParseError):
        return {}


def resource_receipt(directory, runtime):
    before, after = read(directory / 'resources_before.json'), read(directory / 'resources_after.json')
    memory = []
    for row in runtime.get('gpu_resource_samples', []):
        match = re.search(r'(\d+)\s*MiB', row.get('gpu', ''))
        if match:
            memory.append(int(match[1]))
    peak = None
    try:
        for line in (directory / 'process_resources.jsonl').read_text().splitlines():
            row = json.loads(line)
            total = 0
            for entry in row.get('owned_processes', '').splitlines()[1:]:
                fields = entry.split(None, 5)
                if len(fields) == 6:
                    total += int(fields[4])
            peak = max(peak or 0, total)
    except (OSError, ValueError, TypeError):
        pass
    return {'scope': 'Actual resource snapshots for owned retest processes; shared GPU usage includes ongoing training',
            'actor_device': read(directory / 'policy_manifest.json').get('inference_device'),
            'gpu_before': before.get('gpu'), 'gpu_after': after.get('gpu'),
            'gpu_total_peak_mb': max(memory) if memory else None, 'owned_process_rss_peak_kib': peak,
            'source_sha256': {name: sha(directory / name) for name in ('resources_before.json', 'resources_after.json', 'process_resources.jsonl', 'rendering_manifest.json')}}


def collect(runs, protocol_path):
    entries = []
    for directory in sorted(runs.iterdir() if runs.exists() else []):
        if not directory.is_dir() or directory.is_symlink():
            continue
        policy = read(directory / 'policy_manifest.json')
        case = next((name for name in CASES if policy.get('test') == name or ('_' + name + '_') in directory.name), None)
        if case is None:
            continue
        legacy, functional, runtime = (read(directory / name) for name in ('summary.json', 'summary_functional.json', 'runtime_manifest.json'))
        frames, malformed = captured_frames(directory)
        actual_status = status(functional.get('functional_status', functional.get('status', functional.get('levels', {}).get('functional_step'))))
        codes = runtime.get('owned_processes', [])
        complete = bool(functional and runtime and len(codes) >= 2 and all(item.get('returncode') is not None for item in codes))
        interface = status(functional.get('levels', {}).get('interface', legacy.get('levels', {}).get('interface')))
        frozen_protocol = sha(protocol_path)
        archived_protocol = sha(directory / 'sources/tests/step_functional_protocol.json')
        evaluated_protocol = functional.get('protocol_file_sha256', functional.get('protocol_sha256'))
        evaluator_sha = functional.get('evaluator_sha256')
        receipt_valid = bool(frozen_protocol and archived_protocol == frozen_protocol
            and evaluated_protocol == frozen_protocol
            and evaluator_sha and evaluator_sha == sha(directory / 'sources/scripts/evaluate_step_functional.py')
            and policy.get('checkpoint_sha256') == FROZEN_SHA and policy.get('inference_device') == 'cpu')
        entries.append({'run_id': directory.name, 'directory': str(directory.resolve()), 'case': case,
            'functional_status': actual_status, 'interface': interface, 'complete': complete,
            'legacy_motion': status(legacy.get('levels', {}).get('motion')), 'legacy_summary': legacy,
            'summary_functional': functional, 'runtime': runtime, 'policy': policy,
            'asset': read(directory / 'asset_manifest.json'), 'fixture': fixture(directory),
            'frozen_receipt_verified': receipt_valid,
            'protocol_receipt': {'expected_file_sha256': frozen_protocol, 'archived_file_sha256': archived_protocol,
                'functional_file_sha256': evaluated_protocol, 'actual_evaluator_sha256': evaluator_sha,
                'archived_evaluator_sha256': sha(directory / 'sources/scripts/evaluate_step_functional.py')},
            'resources': resource_receipt(directory, runtime), 'captured_frame_count': len(frames),
            'malformed_frame_names': malformed, 'frame_source': read(directory / 'frame_source.json'),
            'source_sha256': {name: sha(directory / name) for name in ('summary.json', 'summary_functional.json', 'runtime_manifest.json',
                'policy_manifest.json', 'asset_manifest.json', 'source_manifest.json', 'world.sdf', 'telemetry.jsonl', 'actuator.jsonl', 'frame_source.json', 'functional_feet.npz')},
            'functional_protocol_sha256': sha(protocol_path), 'artifacts': {}})
    return entries


def replay(directory):
    """Encode actual captured pixels; no overlays, resize, interpolation or new frames."""
    from PIL import Image
    rows, malformed = captured_frames(directory)
    receipt = {'source': 'Actual Gazebo RGB archived by capture.py', 'frame_count': len(rows), 'frames':
               [{'file': str(row['file'].resolve()), 'sensor_stamp_s': row['sensor_stamp_s'], 'sha256': row['sha256']} for row in rows],
        'malformed_frame_names': malformed, 'interpolation': False, 'synthetic_frames': False,
        'modifications': 'Codec encoding only. Original JPEGs unchanged. No crops, resize, color enhancement, overlays or generated frames.'}
    if len(rows) < 2:
        return {**receipt, 'status': 'unverified_insufficient_captured_frames'}
    periods = [b['sensor_stamp_s'] - a['sensor_stamp_s'] for a, b in zip(rows, rows[1:])]
    if min(periods) <= 0:
        return {**receipt, 'status': 'failed_nonincreasing_sensor_stamps'}
    durations = periods + [statistics.median(periods)]
    receipt.update(first_sensor_stamp_s=rows[0]['sensor_stamp_s'], last_sensor_stamp_s=rows[-1]['sensor_stamp_s'],
        sensor_period_min_s=min(periods), sensor_period_max_s=max(periods),
        final_frame_display_tail_s=durations[-1], final_tail_note='Last real captured frame displayed for one median capture period; no later measurement implied.')
    images, dimensions = [], set()
    try:
        for row in rows:
            with Image.open(row['file']) as image:
                dimensions.add(image.size)
                images.append(image.convert('RGB').quantize(colors=256))
        if len(dimensions) != 1:
            return {**receipt, 'status': 'failed_mixed_frame_dimensions'}
        target = directory / 'frame_replay.gif'
        durations_ms = [max(10, round(value * 1000 / 10) * 10) for value in durations]
        images[0].save(target, save_all=True, append_images=images[1:], duration=durations_ms,
                       loop=0, optimize=False, disposal=2)
        receipt['gif'] = {'file': str(target.resolve()), 'sha256': sha(target), 'time_quantization_s': .01,
                          'encoding': 'GIF palette quantization, original image dimensions; nearest 10ms durations'}
    except (OSError, ValueError) as error:
        receipt['gif_error'] = str(error)
    finally:
        for image in images:
            image.close()
    ffmpeg = shutil.which('ffmpeg')
    if ffmpeg:
        concat = directory / 'frame_replay.concat.txt'
        lines = ['ffconcat version 1.0']
        for row, duration in zip(rows, durations):
            escaped = str(row['file'].resolve()).replace("'", "'\\''")
            lines += [f"file '{escaped}'", 'option framerate 1000', f'duration {duration:.9f}']
        # Concat needs a final repeated input to retain the last real frame's display interval.
        lines += [lines[-3], 'option framerate 1000']
        concat.write_text('\n'.join(lines) + '\n')
        target = directory / 'frame_replay.mp4'
        command = [ffmpeg, '-hide_banner', '-loglevel', 'warning', '-y', '-f', 'concat', '-safe', '0', '-i', str(concat),
                   '-an', '-fps_mode', 'vfr', '-c:v', 'libx264', '-threads', '1', '-pix_fmt', 'yuv420p', '-crf', '18',
                   '-enc_time_base', '1:1000', '-video_track_timescale', '1000000', '-movflags', '+faststart', str(target)]
        try:
            result = subprocess.run(command, text=True, capture_output=True, timeout=60)
            (directory / 'frame_replay_encoder.log').write_text(result.stderr)
            if result.returncode == 0 and target.is_file():
                receipt['mp4'] = {'file': str(target.resolve()), 'sha256': sha(target), 'concat_sha256': sha(concat),
                    'encoder': 'CPU libx264 CRF18, VFR, original dimensions. Final frame repeated solely to retain its documented display duration.'}
                ffprobe = shutil.which('ffprobe')
                if ffprobe:
                    probe = subprocess.run([ffprobe, '-v', 'error', '-select_streams', 'v:0', '-count_frames',
                        '-show_entries', 'stream=width,height,nb_read_frames,time_base:format=duration', '-of', 'json', str(target)],
                        text=True, capture_output=True, timeout=30)
                    if probe.returncode == 0:
                        receipt['mp4']['probe'] = json.loads(probe.stdout)
            else:
                receipt['mp4_error'] = result.stderr[-3000:]
        except (OSError, subprocess.TimeoutExpired, ValueError) as error:
            receipt['mp4_error'] = str(error)
    else:
        receipt['mp4_error'] = 'ffmpeg unavailable; actual GIF and JPEG evidence retained'
    receipt['status'] = 'encoded_actual_captures' if 'gif' in receipt or 'mp4' in receipt else 'failed_encoding'
    return receipt


def plot(directory, functional, geometry):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    rows, invalid = telemetry(directory / 'telemetry.jsonl')
    if len(rows) < 2:
        return {'status': 'unverified_missing_telemetry', 'invalid_rows': invalid}
    t = np.asarray([row['sim_time'] for row in rows])
    def column(field, index=None):
        values = []
        for row in rows:
            value = row.get(field)
            if index is not None:
                value = value[index] if isinstance(value, list) and len(value) > index else None
            values.append(value if finite(value) else np.nan)
        return np.asarray(values)
    figure, axes = plt.subplots(3, 2, figsize=(13, 11), layout='constrained')
    axes = axes.ravel()
    axes[0].plot(t, column('command', 0), 'k--', lw=1.3, label='Actual command vx')
    axes[0].plot(t, column('measured', 0), lw=1.1, label='Recorded body vx')
    axes[0].set(title='Forward command and measured velocity', ylabel='vx [m/s]')
    for leg in ('FR', 'FL', 'RR', 'RL'):
        contact = np.asarray([row.get('contacts', {}).get(leg, np.nan) for row in rows])
        axes[1].plot(t, contact, lw=.9, label=leg)
    axes[1].set(title='Recorded native foot contact force', ylabel='Force [N]')
    axes[2].plot(t, column('body_clearance'), lw=1.1, label='Below-body clearance')
    axes[2].set(title='Measured support clearance (not world height gain)', ylabel='Clearance [m]')
    position = np.column_stack([column('position', index) for index in range(3)])
    axes[3].plot(position[:, 0], position[:, 1], lw=1.3, label='Recorded Gazebo truth')
    if geometry:
        pose, size = geometry['pose_xyz_rpy'], geometry['size_xyz']
        corners = np.asarray([[-size[0]/2, -size[1]/2], [size[0]/2, -size[1]/2], [size[0]/2, size[1]/2], [-size[0]/2, size[1]/2]])
        rotation = np.asarray([[math.cos(pose[5]), -math.sin(pose[5])], [math.sin(pose[5]), math.cos(pose[5])]])
        axes[3].add_patch(Polygon(corners @ rotation.T + pose[:2], facecolor='#e9c17b', alpha=.18, label='Actual SDF tread footprint'))
    axes[3].set(title='Actual world x/y trajectory (truth diagnostic)', xlabel='World x [m]', ylabel='World y [m]')
    axes[3].set_aspect('equal', adjustable='datalim')
    for index, name in enumerate(('roll', 'pitch', 'yaw')):
        axes[4].plot(t, column('rpy', index), lw=1, label=name)
    axes[4].set(title='Recorded body attitude', ylabel='Angle [rad]')
    for field, label in [('applied_torque', 'Applied actuator command'), ('q_target', 'Joint position target')]:
        values = [max(abs(v) for v in row[field]) if isinstance(row.get(field), list) and row[field] and all(finite(v) for v in row[field]) else np.nan for row in rows]
        target = axes[5] if field == 'applied_torque' else axes[5].twinx()
        target.plot(t, values, lw=1, color='tab:blue' if field == 'applied_torque' else 'tab:orange', label=label)
        target.set_ylabel('max |torque command| [Nm]' if field == 'applied_torque' else 'max |joint target| [rad]')
        target.legend(loc='upper left' if field == 'applied_torque' else 'upper right', fontsize=8)
    axes[5].set_title('Actual actuator commands (simulation)')
    support_file = directory / 'functional_feet.npz'
    if support_file.exists():
        # Recorded evaluator arrays derive from actual 5ms contact points/normals,
        # native states and FK; they are not a new re-evaluation or policy input.
        with np.load(support_file) as native:
            axes[2].plot(native['sim_time'], native['support_clearance'], lw=.8, alpha=.7, label='Native 5ms support-clearance audit')
        axes[2].axhline(.18, color='#b84e4e', ls=':', lw=1, label='Frozen safety minimum .18m')
    for index, axis in enumerate(axes):
        axis.grid(alpha=.2)
        if index != 3:
            axis.set_xlabel('Policy simulation time [s]')
        if index != 5:
            axis.legend(fontsize=8)
    label = functional.get('functional_status', functional.get('status', 'unverified'))
    figure.suptitle(f"Go2 sustained-step retest | {directory.name} | functional={label}\nActual recorded data; simulation truth is not SLAM navigation", fontsize=12)
    output = directory / 'step_functional.png'
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return {'status': 'rendered_recorded_telemetry', 'file': str(output.resolve()), 'sha256': sha(output), 'rows': len(rows),
            'invalid_rows': invalid, 'telemetry_sha256': sha(directory / 'telemetry.jsonl'), 'native_support_sha256': sha(support_file),
            'scope': 'Actual simulation diagnostics, not a reconstructed or fabricated route'}


def markdown(report):
    lines = ['# 5 / 10 cm 持续上台阶重测', '', f"本次单项结果：**{report['status']}**。要求两种高度各3独立轮；共收集{len(report['runs'])}轮，所有轮次与失败保留。", '',
        '冻结Teacher不重新训练，CPU推理。速度命令在3–22秒持续前进，随后闭环停车至30秒。实际相机帧、原始telemetry、原生接触收据与哈希逐轮保存。', '',
        '测试平台前沿x=7 m、宽1.4 m及高度0.05/0.10 m保留；台面由原x=7..9延长至x=7..13，以让上台后继续行走而不在测试末尾下台。它是新fixture，不能当成原2 m台面相同条件。是低台阶登上台面，不是完整障碍跨越或真实楼梯。', '',
        '本次功能复核使用预先冻结的足部上台支持、持续运动、安全与停车判据。world body height gain仅为诊断，原数值门summary.json逐轮原样保留，不被新判据覆盖。原42轮验收、原起点场景失败、Sim2Sim与导航结论保持原记录。', '',
        '| 运行 | 单项interface | 功能复核 | 旧数值motion | 完整结束 | 冻结收据核对 | 实际画面数 |', '|---|---|---|---|---|---|---:|']
    for row in report['runs']:
        lines.append(f"| {row['run_id']} | {row['interface']} | {row['functional_status']} | {row['legacy_motion']} | {row['complete']} | {row['frozen_receipt_verified']} | {row['captured_frame_count']} |")
    for row in report['runs']:
        directory = Path(row['directory'])
        functional = row['summary_functional']
        lines += ['', f"## {row['run_id']}", '', '功能逐项收据：', '']
        checks = functional.get('checks', {})
        for name, check in checks.items():
            lines.append(f"- {name}: `{json.dumps(check, ensure_ascii=False, allow_nan=False)}`")
        if not checks:
            lines.append('- 未记录功能逐项收据，保持未验证。')
        metrics = functional.get('metrics', {})
        if metrics:
            lines += ['', '原始功能指标：', '', '```json', json.dumps(metrics, ensure_ascii=False, indent=2), '```']
        lines += ['', '原数值门失败/通过记录：', '', '```json', json.dumps(row['legacy_summary'].get('tests', []), ensure_ascii=False, indent=2), '```']
        artifacts = row.get('artifacts', {})
        for key in ('mp4', 'gif'):
            media = artifacts.get('replay', {}).get(key, {})
            if media.get('file'):
                lines.append(f"\n[实际{key.upper()}回放]({media['file']})")
        if artifacts.get('plot', {}).get('file'):
            lines.append(f"\n![实际运动、接触、间隙与轨迹]({artifacts['plot']['file']})")
        lines += ['', f"逐轮图像/模型/配置/资源SHA与所有sensor stamps：[证据清单]({directory / 'step_retest_evidence_manifest.json'})。", '',
                  '回放按原始相机sensor stamp排列；采样间隔内保持上一实际帧，不插帧、不加字幕、不生成/修饰场景。GIF仅做格式必需的调色板编码，MP4仅做CPU视频编码；原JPEG完整保留。末帧显示尾时长单独记录，不能被误当之后的仿真测量。']
    lines += ['', '## 旧42轮中的六次台阶结果继续保留', '',
        '旧5cm/10cm每类3轮的原summary按原数值判据保留。历史接触和继续进度另有只读审计，旧数据缺顶面接触XYZ/normal，不能追认本轮功能通过。', '',
        '| 旧正式运行 | 原interface / motion | 原失败原因 |', '|---|---|---|']
    for old in report['historical_step_runs']:
        lines.append(f"| [{old['run_id']}]({old['summary_path']}) | {old['levels'].get('interface')} / {old['levels'].get('motion')} | {old['reason'].replace('|','/')} |")
    lines += ['', f"历史功能只读审计：[原记录]({ROOT / 'test_results/step_historical_functional_review_20261003.json'})。", '',
        f"原全局验收levels：`{report['global_acceptance_levels']}`；本脚本没有写入原acceptance.json、旧REPORT或summary。导航未执行，真机未验证。", '',
        f"功能协议SHA256：`{report['functional_protocol_sha256']}`；模型SHA256：`{FROZEN_SHA}`。", '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, default=ROOT / 'runs')
    parser.add_argument('--protocol', type=Path, default=ROOT / 'tests/step_functional_protocol.json')
    parser.add_argument('--report', type=Path, default=ROOT / 'docs/STEP_RETEST_REPORT.md')
    parser.add_argument('--manifest', type=Path, default=ROOT / 'test_results/step_retest_report_manifest.json')
    parser.add_argument('--write', action='store_true', help='Export actual recorded evidence and report; no reevaluation or acceptance update')
    args = parser.parse_args()
    entries = collect(args.runs.resolve(), args.protocol)
    count = Counter(row['case'] for row in entries)
    identities = [tuple(item.get('pid') for item in row['runtime'].get('owned_processes', []) if item.get('role') in ('worker', 'gazebo')) for row in entries]
    independent = bool(identities) and len(identities) == len(set(identities)) and all(len(value) == 2 and all(isinstance(v, int) for v in value) for value in identities)
    complete = len(entries) == 6 and all(count[name] == 3 for name in CASES) and all(row['complete'] for row in entries) and independent
    failed = any(row['functional_status'] == 'failed' or row['interface'] == 'failed' for row in entries)
    campaign_status = 'failed' if failed else 'passed' if complete and all(row['functional_status'] == row['interface'] == 'passed' and row['frozen_receipt_verified'] for row in entries) else 'unverified'
    acceptance = read(args.runs / 'acceptance.json')
    historical = []
    for case in ('step05', 'step10'):
        for run_id in acceptance.get('campaign', {}).get('cases', {}).get(case, {}).get('runs', []):
            directory = args.runs / run_id
            summary = read(directory / 'summary.json')
            historical.append({'run_id': run_id, 'summary_path': str((directory / 'summary.json').resolve()),
                'sha256': sha(directory / 'summary.json'), 'levels': summary.get('levels', {}),
                'reason': '; '.join(item.get('reason', '') for item in summary.get('tests', [])), 'tests': summary.get('tests', [])})
    report = {'schema_version': 1, 'status': campaign_status, 'scope': 'Sustained 5/10cm step functional retest only; not overall acceptance',
        'generated_at_local': datetime.now(timezone(timedelta(hours=8))).isoformat(),
        'expected_rounds': 6, 'case_counts': dict(count), 'exact_six_coverage': complete, 'independent_processes': independent,
        'functional_protocol': read(args.protocol), 'functional_protocol_sha256': sha(args.protocol), 'checkpoint_sha256': FROZEN_SHA,
        'global_acceptance_levels': acceptance.get('levels', {}), 'global_acceptance_sha256': sha(args.runs / 'acceptance.json'),
        'historical_step_runs': historical, 'historical_review_sha256': sha(ROOT / 'test_results/step_historical_functional_review_20261003.json'),
        'script_sha256': sha(Path(__file__)), 'source_images_modified': False, 'acceptance_written': False, 'runs': entries}
    if args.write:
        for entry in entries:
            directory = Path(entry['directory'])
            entry['artifacts'] = {'replay': replay(directory), 'plot': plot(directory, entry['summary_functional'], entry['fixture'])}
            dump(directory / 'step_retest_evidence_manifest.json', {'schema_version': 1, 'run_id': entry['run_id'],
                'replay': entry['artifacts']['replay'], 'plot': entry['artifacts']['plot'], 'resources': entry['resources'],
                'source_sha256': entry['source_sha256'], 'fixture': entry['fixture'], 'script_sha256': report['script_sha256'],
                'functional_protocol_sha256': report['functional_protocol_sha256'], 'checkpoint_sha256': entry['policy'].get('checkpoint_sha256'),
                'functional_status': entry['functional_status'], 'legacy_motion': entry['legacy_motion'], 'scope': report['scope']})
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(markdown(clean(report)))
        dump(args.manifest, report)
    print(json.dumps({'status': campaign_status, 'writes': args.write, 'rounds': len(entries), 'expected': 6,
        'case_counts': dict(count), 'independent_processes': independent, 'complete': complete,
        'runs': [{'id': row['run_id'], 'functional': row['functional_status'], 'interface': row['interface'],
                  'legacy_motion': row['legacy_motion'], 'frames': row['captured_frame_count']} for row in entries],
        'global_levels_unchanged': report['global_acceptance_levels']}, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
