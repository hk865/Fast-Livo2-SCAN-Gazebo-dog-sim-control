from pathlib import Path
import datetime,hashlib,json,sys
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2];HERE=ROOT/'navigation/ingress_pipeline_v17';E=OUT.parent/'evaluation'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
semantic_path=E/'v17_semantic_production_fixtures_precision17/semantic_receipt.json'
semantic=json.loads(semantic_path.read_text());build_path=semantic_path.parent/'build_receipt.json';build=json.loads(build_path.read_text())
assert semantic['status']=='PASS_LIMITED_SEMANTICS'
assert len(semantic['case_pairs'])==8 and all(x['passed'] is True for x in semantic['case_pairs'])
assert semantic['negative_guard']['passed'] is True
assert len(semantic['processes'])==16 and all(x['returncode']==0 and x['raw_writer_final_lossless'] is True for x in semantic['processes'])
assert sha(build_path)==semantic['build_receipt_sha256']
assert sha(build['library'])==build['library_sha256']
for name,value in build['source_sha256'].items():assert sha(name)==value,name
assert 'PASS capacity_count' in (OUT/'queue_fixture_sanitized.log').read_text()
assert 'ERROR:' not in (OUT/'queue_fixture_sanitized.log').read_text()
assert '2 packages finished' in (OUT/'build.log').read_text()
proof=json.loads((OUT/'controller_source_proof.json').read_text());assert len(proof['files'])==23
for record in proof['files']:
 assert sha(HERE/record['file'])==record['sha256']
 assert sha(ROOT/'navigation/lidar_sampling_v12'/record['file'])==record['sha256']
checks={name:True for name in ('independent_build','bounded_queue_sanitized','production_packet_semantics','single_estimator_owner_source_audit','unchanged_original_controller','new_timing_semantics_explicit','actual_experiment_prospective_scope')}
files=list(HERE.glob('*.py'))+list((HERE/'profiles').glob('*.json'))+[HERE/'INGRESS_PIPELINE_CONTRACT.json',semantic_path,build_path,
 E/'v17_initial_source_audit/AUDIT.json',E/'v17_deferred_source_audit/AUDIT.json',E/'PROSPECTIVE_V17_60S_SMOKE_CRITERIA.json',
 OUT/'controller_source_proof.json',OUT/'queue_fixture.cpp',OUT/'queue_fixture_sanitized',OUT/'queue_fixture_sanitized.log',OUT/'build.sh',OUT/'build.log',Path(__file__)]
files += [p for p in (HERE/'slam_ws/src').rglob('*') if p.is_file()]
files += [HERE/'slam_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',HERE/'slam_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
files += [HERE/'slam_ws/build'/package/name for package in ('fast_livo2_core','fast_livo2_ros') for name in ('compile_commands.json','CMakeCache.txt')]
receipt={'schema':'limited_ordered_ingress_preflight/v17','status':'PASS_LIMITED_PURE_CHECKS','UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'checks':checks,'sha256':{str(p.resolve()):sha(p) for p in files},
 'scope':'Only finite selected production callback semantics, source audit, bounds/lifetime unit checks and matching build. Authorizes a bounded experiment, never certifies estimator equivalence, throughput, actual source freshness or navigation.',
 'actual_physics_test_pass':False,'actual_queue_performance_pass':False,'full_route_pass':False,
 'source_audit_followup':'Final source differs from second source-audit snapshot only by explicit literal HILTI configuration validation; fresh actual negative fixture proves that restriction.',
 'model_sha256':'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'}
p=HERE/'INGRESS_PIPELINE_PREFLIGHT.json';p.write_text(json.dumps(receipt,indent=2)+'\n')
sys.path.insert(0,str(HERE));from pipeline_preflight import verify_pipeline_preflight
verify_pipeline_preflight(json.loads((HERE/'profiles/l64_r30_c30_ingress_smoke60.json').read_text()))
print(json.dumps({'preflight_sha256':sha(p),'bound_files':len(files),'core_sha256':sha(build['library']),'status':receipt['status']}))
