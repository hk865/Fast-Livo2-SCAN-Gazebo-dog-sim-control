#!/usr/bin/env python3
"""Read original experiment files and create a bounded, explicitly partial archive.

No original files are changed. Full-log SHA values come from sealed inventories,
not from hashing multi-GB raw data again. Samples are visualization material.
"""
import argparse
import gzip
import hashlib
import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OLD = Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode')
DEST = Path(__file__).resolve().parent
LIMIT_FILE = 20_000_000
LIMIT_TOTAL = 200_000_000
RUNS = {
    'V12_fbcb': Path('/var/tmp/go2_teacher_simulation_20261005/20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb'),
    'V18_prefix_ac02': Path('/var/tmp/go2_teacher_parallel_20261005/20261005_232930_closed_loop_cascade_clock_hold_combined_v18_prefix_r1_ac02'),
    'V18_full_ab53': Path('/var/tmp/go2_teacher_parallel_20261005/20261005_234102_closed_loop_cascade_clock_hold_combined_v18_full_r1_ab53'),
    'V17_efde': Path('/var/tmp/go2_teacher_parallel_20261005/20261005_234627_closed_loop_cascade_clock_hold_ingress_v17_smoke_r1_efde'),
    'height_failure_6fdf': OLD/'runs/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf',
}
records = []
references = {}
manifest_metadata = []
skipped = []


def digest(data):
    return hashlib.sha256(data).hexdigest()


def register(path, *, source=None, purpose, transform='byte_copy', source_sha=None):
    data = path.read_bytes()
    assert len(data) < LIMIT_FILE, (path, len(data))
    ref = references.get(str(source.resolve())) if source else None
    row = {'path': str(path.relative_to(DEST)), 'size_bytes': len(data),
           'sha256': digest(data), 'purpose': purpose, 'transform': transform,
           'source_path': str(source) if source else None,
           'source_size_bytes': source.stat().st_size if source else None,
           'source_sha256_reference': source_sha or (ref or {}).get('sha256'),
           'source_sha256_reference_inventory': (ref or {}).get('inventory'),
           'source_sha256_reference_is_rehash': False}
    if transform == 'byte_copy' and row['source_sha256_reference']:
        row['matches_historical_inventory_sha256'] = row['sha256'] == row['source_sha256_reference']
    records.append(row)


def write_json(path, data, *, purpose, source=None, transform='derived_json'):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    register(path, source=source, purpose=purpose, transform=transform)


def copy(source, relative, purpose):
    if not source.is_file():
        skipped.append({'source_path': str(source), 'reason': 'not_present'})
        return
    if source.stat().st_size >= LIMIT_FILE:
        skipped.append({'source_path': str(source), 'reason': 'file_size_limit', 'size_bytes': source.stat().st_size})
        return
    target = DEST/relative
    if target.exists():
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(source.read_bytes())
    register(target, source=source, purpose=purpose)


def old_copy(path, category='reports', purpose='Historical report, verdict, provenance or derived visualization; not a substitute for raw records'):
    copy(path, Path(category)/path.relative_to(OLD), purpose)


def load_inventories():
    sources = [('PACKAGE_MANIFEST', OLD/'PACKAGE_MANIFEST.json', OLD, 'file_inventory'),
               ('EXTERNAL_SIMULATION_MANIFEST', OLD/'test_results/lidar_density_rate_20261005/EXTERNAL_STORAGE_MANIFEST.json', Path('/var/tmp/go2_teacher_simulation_20261005'), 'files'),
               ('EXTERNAL_PARALLEL_MANIFEST', OLD/'test_results/parallel_pipeline_20261005/EXTERNAL_STORAGE_MANIFEST.json', Path('/var/tmp/go2_teacher_parallel_20261005'), 'files')]
    for label, source, root, field in sources:
        raw = source.read_bytes()
        source_sha = digest(raw)
        obj = json.loads(raw)
        for row in obj[field]:
            p = root/row['path']
            references[str(p.resolve())] = {'sha256': row['sha256'], 'size_bytes': row['size_bytes'], 'inventory': label}
        target = DEST/'history_manifests'/f'{label}.json.gz'
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('wb') as f:
            with gzip.GzipFile(filename='', mode='wb', fileobj=f, compresslevel=6, mtime=0) as z:
                z.write(raw)
        assert gzip.decompress(target.read_bytes()) == raw
        register(target, source=source, purpose='Immutable historical byte inventory, losslessly compressed; listed raw bytes are not embedded', transform='gzip_original_json', source_sha=source_sha)
        manifest_metadata.append({'label': label, 'source_path': str(source), 'source_sha256': source_sha,
                                  'source_size_bytes': len(raw), 'compressed_path': str(target.relative_to(DEST)),
                                  'historical_inventory_not_current_availability': True})
        print('inventory', label, len(raw), '->', target.stat().st_size, flush=True)


def copy_reports_and_proofs():
    for p in sorted((OLD/'docs').glob('*.md')):
        old_copy(p)
    for name in ['README.md','REPORT.md','current_status.json']:
        old_copy(OLD/name)
    groups = {
        'test_results/parallel_pipeline_20261005/evaluation': ('.md', '.json', '.png', '.csv'),
        'test_results/lidar_density_rate_20261005/evaluation': ('.md',),
        'test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic': ('.md','.json','.png','.npz'),
        'test_results/full_ramp_audit_20261004': ('.json','.png','.jpg'),
        'test_results/truth_pid_campaign_20261004/actual_motion_overview': ('.json','.png'),
        'test_results/curvature_campaign_20261005/curve_analysis': ('.md','.json','.png','.csv'),
        'test_results/parallel_pipeline_20261005/pipeline_runtime_audit': ('.md','.json'),
    }
    # Never include whole decoded arrays, full raw export fixtures or benchmark binaries.
    exclude_parts = {'real_lio_rows_v12','real_lio_rows_v12_format16','semantic_runs','logs','raw','finite_fixtures','benchmark_outputs'}
    for name, suffixes in groups.items():
        root = OLD/name
        for p in sorted(root.rglob('*')):
            if not p.is_file() or p.suffix not in suffixes or any(s in exclude_parts for s in p.relative_to(root).parts):
                continue
            if p.stat().st_size > 3_000_000:
                continue
            old_copy(p)
    for rel in [
        'test_results/lidar_density_rate_20261005/evaluation/comparison_all_v8/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf',
        'test_results/parallel_pipeline_20261005/combined_v18',
        'test_results/slam_transfer_v3_20261005/v5_aggregate',
    ]:
        root = OLD/rel
        for p in sorted(root.rglob('*')):
            if p.is_file() and p.suffix in ['.md','.json','.png','.csv'] and p.stat().st_size < 2_000_000 and not any(x in p.parts for x in ['logs','finite_fixtures']):
                old_copy(p)
    proof_files = [
        'test_results/parallel_pipeline_20261005/lio_v15/confirmatory_results.json',
        'test_results/parallel_pipeline_20261005/lio_v15/confirmatory_summary.json',
        'test_results/parallel_pipeline_20261005/lio_v15/finite_fixture_comparison.json',
        'test_results/parallel_pipeline_20261005/lio_v15/real_numeric_comparison.json',
        'test_results/parallel_pipeline_20261005/lio_v15/build_receipt.json',
        'test_results/parallel_pipeline_20261005/lio_v15/source_strip_proof.json',
        'test_results/parallel_pipeline_20261005/lio_v15/source_equivalence.json',
        'test_results/parallel_pipeline_20261005/vio_v16/benchmark_confirmatory32.json',
        'test_results/parallel_pipeline_20261005/vio_v16/numeric_comparison.json',
        'test_results/parallel_pipeline_20261005/vio_v16/fixture_build_receipt.json',
        'test_results/parallel_pipeline_20261005/vio_v16/team_partition_proof.json',
        'test_results/truth_pid_campaign_20261004/selection_receipt_v3.json',
        'navigation/truth_tuning_design/campaign_ABC_analysis_final.md',
        'test_results/curvature_campaign_20261005/analysis/final_/20261004T181443_819883Z/summary_radius_campaign.json',
        'test_results/curvature_campaign_20261005/analysis/final_/20261004T181443_819883Z/selected_minimum_receipts.json',
        'test_results/curvature_campaign_20261005/boundary_analysis/final_/20261004T182346_462506Z/summary_radius_boundary_campaign.json',
        'test_results/curvature_campaign_20261005/boundary_analysis/final_/20261004T182346_462506Z/boundary_receipts.json',
        'test_results/curvature_campaign_20261005/sweep_outer_launcher_receipt_v1.json',
        'test_results/step_historical_functional_review_20261003.json',
        'test_results/step_retest_visual_review_20261003.json',
        'test_results/step_camera_supplement_equivalence.json',
        'test_results/step_functional_evaluator_source_audit_20261003/provenance_correction.json',
        'test_results/step_functional_evaluator_source_audit_20261003/contact_geometry_independent_audit.json',
        'test_results/sensor_replacement_audit_20261004_v1.json',
        'test_results/sensor_replacement_audit_20261004_v2.json',
        'test_results/sensor_replacement_audit_20261004_idle_v3.json',
        'test_results/sensor_replacement_idle_runtime_20261004_v2.json',
    ]
    for rel in proof_files:
        old_copy(OLD/rel, 'reproduction_checks')


def copy_run_receipts():
    for label, run in RUNS.items():
        for p in sorted(run.glob('*.json')):
            if p.stat().st_size < 2_000_000:
                copy(p, Path('receipts')/label/p.name, 'Original frozen run metadata/verdict or source binding; full raw files excluded')
        for sub in ['camera_info','vehicle_rgb/camera_info']:
            for p in sorted((run/sub).glob('*.json'))[:3]:
                copy(p, Path('receipts')/label/sub/p.name, 'Actual simulation camera metadata, limited representative selection')
        for sub in ['frames','vehicle_rgb/frames']:
            frames = list((run/sub).glob('*.jpg'))
            def frame_time(p):
                try:
                    sec, nano = p.stem.split('_'); return float(sec)+int(nano)*1e-9
                except ValueError:
                    return math.inf
            frames.sort(key=frame_time)
            if frames:
                for idx in sorted({0,len(frames)//2,len(frames)-1}):
                    p = frames[idx]
                    copy(p, Path('figures')/label/sub/p.name, 'Three actual camera frames selected by timestamp (first/middle/last); no claim robot is visible in every frame')
    prefixes = ['20261003_232320','20261003_232357','20261003_232434','20261003_232511','20261003_232549','20261003_232626','20261003_233510','20261003_233547',
                '20261004_010011','20261004_010157','20261004_081027','20261004_013552','20261004_013614','20261004_015357','20261004_015446','20261004_015511',
                '20261005_023202','20261005_023511','20261005_084153','20261005_145640','20261005_153109',
                '20261005_192400','20261005_192802','20261005_193528','20261005_195524','20261005_200025','20261005_203630']
    for prefix in prefixes:
        for run in (OLD/'runs').glob(prefix+'*'):
            for p in sorted(run.glob('*.json')):
                if p.stat().st_size < 1_000_000 and (p.name.startswith('summary') or any(k in p.name for k in ['receipt','source_manifest','runtime_manifest','scope','acceptance','stop_status'])):
                    copy(p, Path('receipts/history')/run.name/p.name, 'Historical success/failure and provenance; raw data and unchanged payload arrays excluded')


def ref_for(path):
    r = references.get(str(path.resolve()))
    assert r is not None and r['size_bytes'] == path.stat().st_size, ('sealed reference missing or size mismatch',path,r)
    return {'source_path':str(path), 'source_size_bytes':path.stat().st_size,
            'source_sha256_reference':r['sha256'], 'historical_inventory':r['inventory'],
            'source_sha256_not_recomputed':True}


def sample_jsonl(source, target, fields, time_field):
    reference = ref_for(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    last_bucket = None
    last = None
    source_rows = 0
    sampled = []
    expression = re.compile(r'"'+time_field+r'"\s*:\s*([-+0-9.eE]+)')
    with source.open() as f, target.open('w') as out:
        for line_number, line in enumerate(f,1):
            source_rows += 1
            m = expression.search(line[:200])
            if not m:
                raise ValueError(f'Time field is not in original row header: {source}:{line_number}')
            t = float(m.group(1))*(1e-9 if time_field == 'stamp_ns' else 1)
            bucket = math.floor((t+1e-8)/0.2)
            last = (line_number,line,t)
            if bucket == last_bucket:
                continue
            row = json.loads(line)
            compact = {'source_row_1based':line_number, 'sample_time_s':t,
                       'fields_missing_in_source_row':[key for key in fields if key not in row]}
            compact.update({k:row.get(k) for k in fields})
            sampled.append(compact)
            out.write(json.dumps(compact,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
            last_bucket = bucket
        # Retain the final endpoint even when it lies in the final 0.2s bin.
        if last and sampled[-1]['source_row_1based'] != last[0]:
            row = json.loads(last[1])
            compact = {'source_row_1based':last[0], 'sample_time_s':last[2],
                       'extra_final_endpoint':True, 'fields_missing_in_source_row':[k for k in fields if k not in row]}
            compact.update({k:row.get(k) for k in fields})
            sampled.append(compact)
            out.write(json.dumps(compact,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
    register(target, source=source, purpose='First original row in each 0.2s time bin plus final endpoint; visualization only, not full-rate safety/TTL/replay evidence', transform='partial_fields_0p2s_time_bins')
    return sampled, {**reference, 'source_rows':source_rows, 'selected_rows':len(sampled),
                     'selection':'first source row in floor((time+1e-8)/0.2); additionally retain final endpoint',
                     'time_field':time_field, 'retained_fields':fields,
                     'full_raw_reaudit_supported_by_this_sample':False}


def plots(label, telemetry, slam):
    T=np.array([r['sample_time_s'] for r in telemetry]); P=np.array([r['position'] for r in telemetry]);
    S=np.array([r['sample_time_s'] for r in slam]); Q=np.array([r['position'] for r in slam]);
    fig, ax=plt.subplots(1,3,figsize=(13,3.9))
    for a,p,ts,title in [(ax[0],P,T,'Native world-frame body origin (offline only)'),(ax[1],Q,S,'Actual SLAM camera_init body origin')]:
        a.plot(p[:,0],p[:,1],lw=1);a.scatter(p[0,0],p[0,1],c='g',s=25,label='start');a.scatter(p[-1,0],p[-1,1],c='r',s=25,label='end');a.set_aspect('equal',adjustable='datalim');a.set(xlabel='x (m)',ylabel='y (m)',title=title);a.grid(alpha=.3);a.legend(fontsize=7)
    ax[2].plot(T,P[:,2]-P[0,2],label='native z - first z');ax[2].plot(S,Q[:,2]-Q[0,2],label='SLAM z - first z');ax[2].set(xlabel='respective logged time (s)',ylabel='relative height (m)',title='Only scalar initial-z offsets removed');ax[2].legend(fontsize=7);ax[2].grid(alpha=.3)
    fig.suptitle(label+' | 0.2s visualization, coordinate frames are not registered',fontsize=10)
    fig.tight_layout();target=DEST/'figures'/label/'sampled_trajectory.png';target.parent.mkdir(parents=True,exist_ok=True);fig.savefig(target,dpi=130);plt.close(fig)
    register(target,purpose='Plot of downsampled positions; native truth used only offline; frame offsets must not be interpreted as localization error',transform='matplotlib_from_sampled_jsonl')
    cmd=np.array([r['command'] for r in telemetry]);v=np.array([r['body_lin_vel'] for r in telemetry]);w=np.array([r['body_ang_vel'] for r in telemetry]);
    fig,ax=plt.subplots(3,1,figsize=(11,7),sharex=True)
    for i,(actual,title) in enumerate([(v[:,0],'Body linear x (m/s)'),(v[:,1],'Body linear y (m/s)'),(w[:,2],'Body angular z (rad/s), not Euler yaw rate')]):
        ax[i].plot(T,cmd[:,i],label='Teacher command',lw=1);ax[i].plot(T,actual,label='Native measured',lw=.8);ax[i].set_ylabel(title,fontsize=8);ax[i].grid(alpha=.3);ax[i].legend(fontsize=8)
    ax[-1].set_xlabel('Teacher sim_time (s)');fig.suptitle(label+' | 0.2s samples; high-frequency peaks and guard timing may be omitted',fontsize=10);fig.tight_layout();target=DEST/'figures'/label/'sampled_command_velocity.png';fig.savefig(target,dpi=130);plt.close(fig)
    register(target,purpose='Sampled command versus native body-frame velocity; not a re-estimate of formal full-rate verdicts',transform='matplotlib_from_sampled_jsonl')


def sample_runs():
    fields=['sim_time','world_sim_time','command','body_lin_vel','body_ang_vel','position','rpy','contacts','fault','q_target','applied_torque','state','command_expired','command_source','observation_source']
    sfields=['stamp_ns','frame_id','child_frame_id','position','quaternion','body_velocity','body_angular_velocity','received_monotonic_wall','callback_ros_clock_ns','sim_age_at_callback_s','body_gyro','gyro_stamp_s']
    for label,run in RUNS.items():
        print('sampling',label,flush=True)
        telemetry,tmeta=sample_jsonl(run/'telemetry.jsonl',DEST/'curves'/label/'telemetry_0p2s.jsonl',fields,'sim_time')
        slam,smeta=sample_jsonl(run/'navigation_slam_poses.jsonl',DEST/'curves'/label/'navigation_slam_poses_0p2s.jsonl',sfields,'stamp_ns')
        write_json(DEST/'curves'/label/'SAMPLE_PROVENANCE.json',{'schema':'go2_compact_visualization_samples/v1','run_dir':str(run),'label':label,'sample_interval_s':0.2,'telemetry':tmeta,'SLAM':smeta,'warnings':['Partial fields and downsampling discard observations, actions and transient safety/TTL/contact events.','Original native world and camera_init SLAM frames are not registered by these samples.','SHA values are quoted from immutable sealed inventories; large source data were not rehashed.','This evidence archive cannot reconstruct or re-audit full original records after their deletion.']},purpose='Explicit source hash references, selected row counts and limitations')
        plots(label,telemetry,slam)
        print('sampled',label,len(telemetry),len(slam),flush=True)


def finish():
    index='''# 精简实验记录 / Compact experiment evidence

本目录保存已有实验的原始小型收据、报告、配置来源、派生曲线及有限真实画面。`history_manifests/*.json.gz` 是历史全字节清单的无损压缩副本，列出文件不代表那些文件仍存在。

用户已要求删除大体积原始数据。删除后，本目录不能从原始点云、诊断、物理和策略逐样本日志重新执行历史完整验收。保存源码可以重新运行新实验；不能还原已删除的历史输入。`EVIDENCE_MANIFEST.json` 记录本目录每个文件的实际SHA、原来源、尺寸和用途；旧原始数据SHA来自删除前封存清单，不是新的完整重审。

## 关键结论与位置

|记录|范围|精简内容|
|---|---|---|
|V12_fbcb|单次实际32区域、两条12m坡道、首固定5s停车联合通过|`receipts/V12_fbcb/`，旧common/v2/ramp未验证状态保留；新metadata-v2修正独立通过|
|V18_prefix_ac02|210s有限前缀、25/32；不能冒充完整导航|`receipts/V18_prefix_ac02/`|
|V18_full_ab53|单次264.6s实际32区域、两条完整12m坡道和停车联合通过|`receipts/V18_full_ab53/`，原收据与metadata-v2独立结论分别保留|
|V17_efde|60s前缀8/32，严格管线失败|`receipts/V17_efde/`；accepted31615、committed31614、canceled1，不豁免尾包|
|height_failure_6fdf|旧真实SLAM高度故障诊断|`receipts/height_failure_6fdf/`及保留的高度派生图和指标|

完整报告复制在 `reports/docs/`；较新的独立报告复制在 `reports/test_results/parallel_pipeline_20261005/evaluation/{actual_v18_full,actual_v18_prefix,actual_v17_smoke}/`。V12完整报告在 `reports/test_results/lidar_density_rate_20261005/evaluation/FULL_REPORT.md`。

`curves/<label>/` 有每0.2s时间桶首行加最终端点的有限字段JSONL；`figures/<label>/` 有轨迹/速度PNG和三张实际相机画面。采样可能遗漏跌倒、尖峰、接触、TTL失联等瞬态，不能用它重新认证原安全门。原SLAM camera_init与native world画在独立坐标面板；相对高度只去除各自首z，没有SE3注册，也不是定位误差图。

Teacher导航仍使用实际SLAM/IMU及SCAN，Actor保留232维特权仿真输入；15维是命令与上一动作。这里的三层连接是坡道，不能称真实楼梯。未证明随机鲁棒性、动态障碍整线、全部传感器替换Actor、真实机器人或新控制器完整Isaac跨仿真等价。

旧5/10cm持续登台各3次功能通过，旧机身增高比例与兼容summary失败保留。真值PID标定、曲率tight-S失败、初版完整开环坡道失败、IMU/关节末帧新鲜度失败和各次20/30Hz实验不因最终单轮通过而抹去。

## 构建本目录

`assemble_compact_evidence.py` 只读原目录，复制小型证据并抽样；需要原数据尚未删除、Python NumPy/Matplotlib。脚本保留旧机器绝对路径用于历史溯源，不是新机器运行器。新实验复现应使用仓库的独立构建和启动说明。
'''
    p=DEST/'INDEX.md';p.write_text(index);register(p,purpose='Entry point and explicit partial-evidence/deletion boundary',transform='new_documentation')
    register(Path(__file__),purpose='Read-only assembly and sampling script for this compact historical archive',transform='assembly_source')
    total=sum(p.stat().st_size for p in DEST.rglob('*') if p.is_file())
    assert total < LIMIT_TOTAL, ('total budget exceeded',total)
    summary={'schema':'go2_compact_evidence_manifest/v1','created_UTC':datetime.now(timezone.utc).isoformat(),'source_project':str(OLD),'file_limit_bytes_exclusive':LIMIT_FILE,'total_limit_bytes_exclusive':LIMIT_TOTAL,'measured_files_before_manifest':len(records),'measured_bytes_before_manifest':total,'files':records,'historical_inventory_copies':manifest_metadata,'key_runs':{k:str(v)for k,v in RUNS.items()},'explicit_limits':{'large_raw_payloads_embedded':False,'historical_raw_deletion_authorized_by_user':True,'historical_raw_deleted_by_this_script':False,'full_original_raw_reaudit_supported':False,'sampled_curves_are_complete_safety_evidence':False,'copied_receipts_are_claims_generated_before_raw_deletion':True},'skipped_sources':skipped}
    p=DEST/'EVIDENCE_MANIFEST.json';p.write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    total=sum(p.stat().st_size for p in DEST.rglob('*')if p.is_file())
    assert total<LIMIT_TOTAL
    assert max(p.stat().st_size for p in DEST.rglob('*')if p.is_file())<LIMIT_FILE
    print(json.dumps({'completed':True,'files':len(list(DEST.rglob('*'))),'total_bytes':total,'manifest_sha256':digest((DEST/'EVIDENCE_MANIFEST.json').read_bytes())},indent=2),flush=True)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--assemble',action='store_true',required=True);parser.parse_args()
    assert not (DEST/'EVIDENCE_MANIFEST.json').exists(), 'Refuse replacing an existing completed archive'
    load_inventories();copy_reports_and_proofs();copy_run_receipts();sample_runs();finish()


if __name__=='__main__':
    main()
