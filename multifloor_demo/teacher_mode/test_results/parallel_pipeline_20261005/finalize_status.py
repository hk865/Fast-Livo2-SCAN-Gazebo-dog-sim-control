#!/usr/bin/env python3
"""Publish only scoped, already-written independent outcomes after all writers exit."""
from pathlib import Path
import datetime, hashlib, json, os
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
FULL=Path('/var/tmp/go2_teacher_parallel_20261005/20261005_234102_closed_loop_cascade_clock_hold_combined_v18_full_r1_ab53')
PIPE=Path('/var/tmp/go2_teacher_parallel_20261005/20261005_234627_closed_loop_cascade_clock_hold_ingress_v17_smoke_r1_efde')
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    full_receipt=FULL/'summary_closed_loop_terrain_metadata_join_independent_v2.json'
    pipe_receipt=HERE/'pipeline_runtime_audit/V17_efde_RUNTIME_RECEIPT.json'
    assert read(full_receipt)['status']=='passed'
    assert read(pipe_receipt)['pipeline_integrity_status']=='failed'
    external=HERE/'EXTERNAL_STORAGE_MANIFEST.json'
    ext=read(external)
    for run in [FULL,PIPE]:assert read(run/'runtime_manifest.json')['all_owned_and_children_clean'] is True
    prior=HERE/'prior_status/current_status.json'
    assert sha(prior)=='11e7f3cae10bc73914f3a5e0a8b7c45d6cdcb5d82ba0654656f563e2faa76e87'
    d=read(prior)
    d['updated_at']=datetime.datetime.now(datetime.timezone.utc).isoformat()
    d['prior_current_status_before_parallel_pipeline']={'path':str(prior.relative_to(ROOT)),'sha256':sha(prior)}
    d['scope']='V15/V16 finite numerical/performance validation; V18 actual static32 dual12m-up-ramp scoped pass; V17 actual60s ordered-ingress lifecycle FAILED (one canceled item); no end-to-end speedup proven'
    d['finalized']=True
    d['levels_by_scope']['queue_and_algorithm_pipeline_design_before_v15']=d['levels_by_scope']['queue_and_algorithm_pipeline_candidates']
    d['levels_by_scope']['queue_and_algorithm_pipeline_candidates']={'status':'unverified','description':'部分已实际实现：V15/V16有限算子通过、V18指定静态导航通过、V17队列生命周期失败；分块正规方程与IMU区间Delta/FQ扫描仍未实施，不能合并成全部候选通过。','implemented':True,'all_candidates_implemented':False}
    d['levels_by_scope']['actual_SLAM_SCAN_static32_dual_ramp_v18']={'status':'passed','description':'V18组合版单次真实SLAM/SCAN静态32区域、两条完整12m上坡及首固定5s停车；新增metadata-v2联合所有原门，原UNVERIFIED保持。','actual_runs':1,'run':FULL.name,'canonical_run':str(FULL),'receipt':str(full_receipt),'receipt_sha256':sha(full_receipt),'regions_reached':32,'regions_total':32,'up_ramps_m':[12,12],'parking_xy_drift_m':0.00214,'parking_yaw_drift_rad':0.004682,'navigation_ground_truth_used':False,'actor_privileged_dimensions':232,'dynamic_obstacles_current_configuration_verified':False,'new_matched_Isaac_closed_loop_verified':False,'end_to_end_speedup_proven':False}
    d['levels_by_scope']['finite_navigation']={'status':'passed','description':'V18本轮真实SLAM/SCAN静态32区域、双12m上行坡道与固定5s停车单次通过；其他范围另列'}
    d['levels_by_scope']['parallel_compute_kernel']={'status':'passed','description':'V15行4线程和V16块1线程有限数值/性能门通过；VIO4尾延迟门失败；实际整链无显著加速证据'}
    d['levels_by_scope']['ordered_ingress_pipeline']={'status':'failed','description':'V17实际60s：31615接收、31614提交、退出取消1条；已提交源/顺序/容量通过，零取消门失败'}
    d['display_note']='V18独立组合版：真实SLAM/IMU+SCAN静态32/32区域、双12m上行坡道与首5s停车通过，XY漂移2.14mm；新metadata-v2联合原门，旧UNVERIFIED不改写。SLAM30.296Hz，实际数学17.974Hz；控制行位姿源年龄max80ms。局部算子改善未转化为已证明的整链加速。V17独立队列60s退出取消1条，严格失败。Actor232维特权+15维已知输入；当前动态障碍、原46区、全感知Actor、新律完整Sim2Sim和真机未验证。'
    d['navigation_ground_truth_used']=False
    d['all_evidence_writers_completed']=True
    d['active_work']={'scope':'Independent queue and same-iteration algorithm parallel optimization and actual simulation tests','status':'completed_scoped_experiments_with_preserved_failures','new_queue_and_algorithm_candidates_implemented':True,'full32_mission_verified':True,'single_trial_only':True,'pipeline_lifecycle_verified':False,'end_to_end_speedup_proven':False,'handoff':'test_results/parallel_pipeline_20261005/README.md','truth_navigation_used':False,'next_work':'New independent queue shutdown drain/trace candidate, identical-input full frontend replay and saturation tests; current experiment is closed with failures preserved','package_inventory':'Separate PACKAGE_MANIFEST is regenerated after this status'}
    d['active_closed_loop_campaign']={'status':'completed_single_scoped_V18_static32_pass_and_V17_pipeline_failed','current_run':FULL.name,'canonical_run':str(FULL),'current_configuration':'combined_compute_v18/combined_lio4_vio1_l64_r30_c30_600','navigation_ground_truth_used':False,'Actor_privileged_dimensions':232,'Actor_hz':50,'native_PD_hz':200,'actual_feedback_hz':30.29583,'actual_controller_math_hz':17.97408,'heartbeat_wall_hz':20,'current_navigation_verified':True,'last_runtime_completed_s':264.60,'last_Actor_samples':13231,'last_worker_fault':None,'implementation':'Independent LIO Jacobian rows4 + VIO reordered patches1; IMU/LIO/VIO single state owner; V17 ingress separately failed lifecycle','pipeline_run':PIPE.name,'pipeline_integrity_status':'failed','combined_pipeline_and_compute_run':False}
    d['current_controller_frequency_by_scope']['V18_static32']={'SLAM_actual_header_hz':30.29583,'math_actual_mean_sim_hz':17.97408,'ticker_wall_hz':20,'Teacher_sim_hz':50,'native_PD_sim_hz':200}
    d['prior_external_run_storage_before_parallel_pipeline']=d['external_run_storage']
    d['external_run_storage']={'root':ext['canonical_storage_root'],'project_runs_aliases':True,'payload_embedded_in_project':False,'manifest':str(external.relative_to(ROOT)),'manifest_sha256':sha(external),'regular_files':ext['file_count'],'total_size_bytes':ext['total_size_bytes'],'copy_requires_external_payload':True,'prior_root_retained':'/var/tmp/go2_teacher_simulation_20261005','old_manifest_unchanged':True,'package_run_index_scope':'Project alias symlinks are not followed; external payload is separately bound by this immutable inventory'}
    d['parallel_pipeline_preserved_failures']={'VIO4_performance':'p95 regressions and insufficient selected median gain','V15_historical_HTH_reconstruction':'3-4e-8 discrepancy unresolved; finite fresh outputs byte-equal','V18_prefix':'25/32 at210s; original incomplete/full parking gates failed/unverified','V17_actual_lifecycle':'accepted31615, committed31614, canceled1; canceled type/source/time unknown','end_to_end_speedup':'not proven by same-scene different-trajectory runs','viewer_session_exit143':'source stable; cause unverified; restored dedicated local read-only server'}
    evidence=[HERE/'evaluation/actual_v17_smoke/SMOKE_ORIGINAL_GUARD_AND_STRICT_PIPELINE_COVERAGE.json',full_receipt,pipe_receipt,HERE/'EXTERNAL_STORAGE_MANIFEST.json',HERE/'evaluation/actual_v18_full/FULL_ORIGINAL_SCOPE_COVERAGE.json',HERE/'evaluation/actual_v18_prefix/PREFIX_MANDATORY_COVERAGE.json',HERE/'performance_actual/full_process_observation.json',HERE/'performance_actual/performance.json']
    for p in evidence:d['evidence_sha256'][str(p)]=sha(p)
    reports=[ROOT/'README.md',HERE/'README.md',HERE/'IMPLEMENTATION_AND_BOTTLENECKS.md',HERE/'performance_actual/REPORT.md',HERE/'evaluation/actual_v18_full/FULL_REPORT.md',HERE/'pipeline_runtime_audit/actual_efde/README.md']
    reports += list((HERE/'evaluation/actual_v17_smoke').glob('*.md'))
    for p in reports:d['current_reports'][str(p)]=sha(p)
    view=read(HERE/'viewer/restart_after_session_exit143.json')
    assert Path(f"/proc/{view['pid']}/stat").read_text().split()[21]==view['start_ticks']
    d['readonly_viewer']={'PID':view['pid'],'start_ticks':view['start_ticks'],'port':8768,'bind':'127.0.0.1','source_sha256':view['source_sha256'],'actual_browser_verified':True,'source_audit_card_passed':True,'old_receipts_unverified_preserved':True,'new_storage_tests':10,'publication_tests':26,'metadata_tests':20,'old_storage_tests':10,'v14_tests':3,'physics_controller_running':False,'log_outside_sealed_package':view['log']}
    target=ROOT/'current_status.json';temp=target.with_suffix('.json.tmp')
    temp.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');os.replace(temp,target)
    print(json.dumps({'status':str(target),'sha256':sha(target),'full_navigation':'passed_scoped','pipeline':'failed','all_writers_exited':True}))
if __name__=='__main__':main()
