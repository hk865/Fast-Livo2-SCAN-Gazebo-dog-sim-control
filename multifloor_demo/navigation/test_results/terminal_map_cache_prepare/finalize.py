"""Read-only current API/archive mismatch provenance and excluded patch receipt."""
import ast,hashlib,json,time,urllib.request
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
run=ROOT/'runs/20261002_001559_882ddb'
with urllib.request.urlopen('http://127.0.0.1:8767/api/state',timeout=2) as response:
    state=json.load(response)
with urllib.request.urlopen('http://127.0.0.1:8767/api/map/map_metadata.json',timeout=2) as response:
    api_metadata=json.load(response)
file_metadata=json.loads((run/'map_metadata.json').read_text())
excerpt={k:state.get(k) for k in ('run_id','stage','process_running','stopping','elapsed_s','map')}
(HERE/'api_state_excerpt.json').write_text(json.dumps(excerpt,indent=2)+'\n')
(HERE/'api_metadata.json').write_text(json.dumps(api_metadata,indent=2)+'\n')
source=ROOT/'scripts/mission_server.py'
old=ast.parse(source.read_text());new=ast.parse((HERE/'mission_server_candidate.py').read_text())
def methods(t):
    c=next(n for n in t.body if isinstance(n,ast.ClassDef) and n.name=='Coordinator')
    return {n.name:ast.dump(n) for n in c.body if isinstance(n,ast.FunctionDef)}
a,b=methods(old),methods(new)
changed=[k for k in a if a[k]!=b[k]];added=[k for k in b if k not in a]
frozen=json.loads((ROOT/'test_results/full18_freeze/source_manifest.json').read_text())['sha256']
source_changes=[n for n,h in frozen.items() if sha(ROOT/n)!=h]
files=('mission_server_candidate.py','source_before.py','server_map_cache.patch','test_methods.py','test_http.py',
       'contract_result.json','http_result.json','api_state_excerpt.json','api_metadata.json')
contract=json.loads((HERE/'contract_result.json').read_text());http=json.loads((HERE/'http_result.json').read_text())
checks=dict(current_API_same_original_run=state.get('run_id')==run.name,
    original_terminal_FAILURE_kept=state.get('stage')=='failed' and json.loads((run/'acceptance.json').read_text())['passed'] is False,
    actual_cached52_and_file53_reproduced=state['map']['revision']==52 and state['map']['point_count']==141557 and file_metadata['revision']==53 and file_metadata['point_count']==142199,
    API_map_serves_actual_final_file=api_metadata==file_metadata,
    final_immutable_binary_complete=(run/file_metadata['binary_filename']).stat().st_size==16*file_metadata['point_count'],
    only_target_Coordinator_methods_changed=changed==['on_map','finish_stop'] and added==['refresh_final_map'],
    actual_proposed_method_tests=contract['passed'],actual_local_HTTP_whitelist_tests=http['passed'],
    original269_collected_unchanged=not source_changes)
result=dict(scope=__doc__,ready_for_parent_review=all(checks.values()),checks=checks,
    observed_at_wall_ns=time.time_ns(),original_run=str(run),
    original_cache_counts={'revision':state['map']['revision'],'points':state['map']['point_count']},
    final_archive_counts={'revision':file_metadata['revision'],'points':file_metadata['point_count']},
    cause={'map_archive_final_snapshot':'slam/map_archive.py finally calls snapshot after ROS context closure; it writes metadata/binary but publishes status only if context.ok.',
           'coordinator_cache':'Only on_map copies ROS status into map_status; finish_stop originally never reloads committed final metadata.',
           'UI_sources':'show uses state.map.point_count for header; loadMap directly GETs immutable final map metadata and binary for canvas.'},
    candidate={'owned_shutdown_only':True,'same_proc_run_id_required':True,'metadata_binary_size_validation':True,
               'no_touch_of_map_or_sensor_freshness':True,'same_run_old_revision_DDS_rejected':True,
               'precise_README_GET_whitelist':True,'no_root_runs_or_tests_exposure':True,
               'no_original_FAIL_or_save_acceptance_modified':True},
    method_contract=contract,http_contract=http,production_modified=False,ROS_nodes_started=False,
    physics_started=False,actual_HTTP_server_owned_clean=http['owned_HTTP_thread_clean'],
    source_changes=source_changes,sha256={n:sha(HERE/n) for n in files},
    original_source_sha256=sha(source),map_archive_source_sha256=sha(ROOT/'slam/map_archive.py'),
    original_final_metadata_sha256=sha(run/'map_metadata.json'),original_acceptance_sha256=sha(run/'acceptance.json'),
    limitations=['The final file contains the last observed sensor health, not a live sensor heartbeat; last/map receive freshness is deliberately unchanged.',
                 'The minimal final reload requires successful owned-process cleanup. Missing/invalid metadata keeps cached status and cannot declare map saved or acceptance successful.',
                 'No server restart or integration occurred. HTTP tests use temporary permitted/private files and the exact proposed Handler body; ROS methods use no middleware.',
                 'The same-run highwater check is for the actual MapArchive revision-bearing status schema. Broader legacy no-run/no-revision ROS map publishers were not changed into a new accepted protocol.'])
(HERE/'receipt.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(ready=result['ready_for_parent_review'],checks=checks,receipt_sha256=sha(HERE/'receipt.json'),candidate_sha256=sha(HERE/'mission_server_candidate.py'))))
raise SystemExit(not result['ready_for_parent_review'])
