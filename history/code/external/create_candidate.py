"""Single authorized continuation; preserve R39 and original hard gates."""
from pathlib import Path
import datetime as dt
import difflib
import hashlib
import json
import shutil

S = Path(__file__).resolve().parent
B = Path('/__LOCAL_HOME__/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/engineering_v34_r39_p1_association')
C = B.with_name('engineering_v34_r40_continue_original_route')
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not C.exists()
before = {str(p.relative_to(B)): sha(p) for p in B.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
shutil.copytree(B, C, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
p = C/'first_goal_stop.py'
s = p.read_text()
s = s.replace('1_000_000_000', '5_000_000_000')
s = s.replace('120_000_000_000', '1_500_000_000_000').replace('native120s_P1_diagnostic_total_cutoff','native1500s_original_route_total_cutoff')
s = s.replace('wall_elapsed >= 900:', 'wall_elapsed >= 9090:').replace('wall900s_P1_diagnostic_total_cutoff','wall9090s_original_route_total_cutoff')
old = '''    # Only after all original native/mission/navigation failures and caps.
    # The separate event window begins at the actual navigation_f1_f3 mission.
    from event_aware_source_capture import diagnostic_stop_hint
    hint=diagnostic_stop_hint(run,clock_ns)
    if hint is not None:return hint
'''
assert s.count(old) == 1
s = s.replace(old, '''    # Event sampling is evidence only. Completion or expiry must not stop the
    # autonomous route; rejected updates remain rejected by known_scene_runtime.
''')
p.write_text(s)
p = C/'run.py'
s = p.read_text().replace('latest_engineering_v34_r39_p1_association', 'latest_engineering_v34_r40_continue_original_route')
s = s.replace('root authorized P1 original association failure capture; no shadow scoring', 'root authorized R40 autonomous original-route continuation; bounded P1 capture is evidence only; no shadow scoring')
p.write_text(s)
p = C/'POLICY_SELECTION_CONTRACT.json'
d = json.loads(p.read_text()); d['candidate_origin'] = str(C/'policy_candidate/worker.py')
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
p = C/'R33_SHORT_STARTUP_CONTRACT.json'
d = json.loads(p.read_text())
d.update(schema='R40_continue_original_autonomous_route_contract/v1',candidate_root=str(C),source_stage=str(S),max_native_seconds=1500,max_wall_seconds=9090,new_output_cap_bytes=5_000_000_000,phase='continue_original46_without_sampling_soft_stop',policy_selection_contract=str(C/'POLICY_SELECTION_CONTRACT.json'),authorization_quote='所以我认为对于仿真来说不影响，如果继续校验可能是浪费时间',authorization_user_message='Sentinel_239acb653ffc8191815d1663aeffb3a7',authorization_assistant_message=None,sampling_completion_or_expiry_stops_route=False,rejected_localization_update_skipped_without_admission=True,original_correction_freshness_gate_preserved=True,GT_carrier_allowed=False,new_authority_file=str(S/'AUTHORITY.json'),new_authority_sha256=sha(S/'AUTHORITY.json'),base_R39_source_root=str(B),base_R39_source_gate_sha256=sha(B/'R33_STARTUP_PREFLIGHT.json'))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
p = C/'EVENT_CAPTURE_CONTRACT_TEMPLATE.json'
d = json.loads(p.read_text())
for item in d['source_bindings']:
    origin = Path(item['path'])
    if origin.is_relative_to(B):
        item['path'] = str(C/origin.relative_to(B))
        assert sha(item['path']) == item['sha256']
d['authorization'].update(user_message='Sentinel_239acb653ffc8191815d1663aeffb3a7',delegation='R40 autonomous original-route continuation; sampling completion is evidence only')
d['runtime_budget'].update(native_seconds=1500,wall_seconds=9090,new_output_cap_bytes=5_000_000_000)
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
changed = {}
for rel,h in before.items():
 assert sha(B/rel) == h, rel
 now = sha(C/rel)
 if now != h: changed[rel] = dict(before=h,after=now)
assert set(changed) == {'first_goal_stop.py','run.py','POLICY_SELECTION_CONTRACT.json','R33_SHORT_STARTUP_CONTRACT.json','EVENT_CAPTURE_CONTRACT_TEMPLATE.json'}
diff = ''.join(''.join(difflib.unified_diff((B/r).read_text().splitlines(True),(C/r).read_text().splitlines(True),fromfile='R39/'+r,tofile='R40/'+r)) for r in changed)
(S/'CANDIDATE_DIFF.patch').write_text(diff)
receipt = dict(schema='R40_isolated_copy_receipt/v1',utc=dt.datetime.now(dt.timezone.utc).isoformat(),base=str(B),candidate=str(C),base_sha256=before,changed=changed,all_other_files_exact=True,old_R39_unmodified=True,profiles_unchanged=True,Actor_PD_localization_controller_communication_unchanged=True,source_gate_stale_until_new_freeze=True,new_runtime_started=False,navigation_PASS=False)
(S/'ISOLATED_CANDIDATE_COPY.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({'candidate':str(C),'changes':changed,'files':len(before)}))
