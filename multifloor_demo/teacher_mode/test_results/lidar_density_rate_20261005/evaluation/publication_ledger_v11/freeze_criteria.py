#!/usr/bin/env python3
"""Freeze an additive, pre-physics publication audit contract.
No existing criterion, numerical acceptance gate or runtime source is edited.
"""
from __future__ import annotations
import argparse,copy,datetime,hashlib,json,os
from pathlib import Path

HERE=Path(__file__).resolve().parent
PARENT_SHA='9c2b5f2ff6a8f6525df662411ca51976ee3aa6196661f3b774b8a30bf4259f65'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def removed_observer_sha(p):
    kept=[];inside=False
    for line in p.read_text().splitlines(keepends=True):
        if 'PUBLICATION_OBSERVER_BEGIN'in line:
            if inside:raise ValueError('Nested observer block')
            inside=True;continue
        if 'PUBLICATION_OBSERVER_END'in line:
            if not inside:raise ValueError('Unmatched observer block')
            inside=False;continue
        if not inside:kept.append(line)
    if inside:raise ValueError('Unclosed observer block')
    return hashlib.sha256(''.join(kept).encode()).hexdigest()
def freeze(source):
    parent=HERE.parent/'clock_hold_v9/PROSPECTIVE_CRITERIA.json'
    if sha(parent)!=PARENT_SHA:raise ValueError('Original prospective criterion changed')
    d=copy.deepcopy(json.loads(parent.read_text()));d['schema']='prospective_v11_publication_ledger_independent_audit/v2'
    d['created_UTC']=datetime.datetime.now(datetime.timezone.utc).isoformat()
    d['parent_prospective_criteria_sha256']=PARENT_SHA
    d['parent_prospective_criterion']=str(parent)
    d['ledger_receipt_schema']='independent_actual_SLAM_SCAN_publication_ledger_navigation/v2'
    d['scope']+=['Every actual producer command publication, including asynchronous protection zeros outside controller math, must have a complete original ledger',
        'Producer ROS clock is the actual unchanged last_command_time anchor. Topic receive clocks cannot substitute for publisher clocks',
        'No old failed/unverified V8/V9/V10 receipt is upgraded by this future producer evidence extension']
    d['additional_checks']+=['actual_control_publication_ledger_complete_and_original']
    d['required_extra_check_keys']=['prospective_clock_hold_criteria_and_archived_sources',*d['additional_checks']]
    d['publication_ledger_gates']={
        'schema':'teacher_actual_command_publication/v1','file':'navigation_command_publications.jsonl',
        'complete_original_sequence_starts_at':1,'record_count_equal_writer_expected_and_original_published_counter':True,
        'all_PID_rows_have_exactly_one_actual_publication':True,'all_hold_rows_have_exact_original_publication_sequence':True,
        'actual_publisher_commands_form_complete_chain':True,'outside_PID_publications_must_be_exact_zero':True,
        'producer_last_command_time_anchor_equals_original_publish_ros_clock':True,
        'no_receive_or_estimated_publication_clocks_permitted':True,
        'producer_wall_publication_order_monotonic':True,'producer_command_and_state_clocks_not_changed_by_observer':True,
        'previous_slew_state_includes_every_actual_PID_hold_and_callback_zero':True,
        'source_snapshots_must_bind_exact_pre_test_producer_and_observation_helper':True,
        'numeric_replay_max_abs_error':d['hold_gates']['numeric_replay_max_abs_error'],
        'slew_limits_mps2_radps2':d['hold_gates']['slew_limits_mps2_radps2'],
        'original_route_arrival_TTL_parking_safety_and_PI_numerical_gates_unchanged':True,
        'producer_fields':['sequence','entry_monotonic_wall_ns','publish_started_monotonic_wall_ns','monotonic_wall_ns',
            'publish_ros_clock_ns','command_before','last_command_time_before','last_publication_ros_clock_ns_before',
            'command_after','last_command_time_after','commands_published_count','associated_pid_sequence','trigger','stopped_argument','state_before',
            'state_after','heading_phase','heading_reference','protection_flags_before','protection_flags_after','navigation_ground_truth_used']}
    names=['controller.py','clock_hold.py','cascade_core.py','publication_ledger.py','PUBLICATION_LEDGER_PREFLIGHT.json','PUBLICATION_LEDGER_CONTRACT.json']
    paths=[source/n for n in names]
    if any(not p.is_file()for p in paths):raise ValueError('Producer/helper/preflight has not been finalized')
    preflight=json.loads((source/'PUBLICATION_LEDGER_PREFLIGHT.json').read_text())
    if (preflight.get('status')!='PASS_LIMITED_PRODUCTION_AST'or preflight.get('tests_run')!=41
            or preflight.get('simulation_started')is not False or preflight.get('ROS_started')is not False):
        raise ValueError('Publisher prospective preflight not passed before physics')
    old={Path(k).name:v for k,v in json.loads(parent.read_text())['pre_test_source_sha256'].items()}
    removed={name:removed_observer_sha(source/name)for name in ('controller.py','clock_hold.py')}
    if any(removed[name]!=old[name]for name in removed)or sha(source/'cascade_core.py')!=old['cascade_core.py']:
        raise ValueError('Publication observer did not preserve original V9 controller/hold/core bytes')
    d['original_execution_sha256']={name:old[name]for name in ('controller.py','clock_hold.py','cascade_core.py')}
    d['observation_removed_execution_sha256']=removed
    d['pre_test_source_sha256']={str(p.resolve()):sha(p)for p in paths}
    common=next((k,v)for k,v in json.loads(parent.read_text())['pre_test_source_sha256'].items()if Path(k).name=='evaluate_closed_loop.py')
    d['pre_test_source_sha256'][common[0]]=common[1]
    return d
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);a=p.parse_args()
    d=freeze(a.source.resolve());out=HERE/'PROSPECTIVE_CRITERIA.json'
    with out.open('x')as f:f.write(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    digest=sha(out);(HERE/'PROSPECTIVE_CRITERIA.sha256').write_text(digest+'  '+out.name+'\n')
    print(json.dumps({'path':str(out),'sha256':digest,'extra_check_keys':d['required_extra_check_keys'],'pre_test_source_sha256':d['pre_test_source_sha256']},indent=2))
if __name__=='__main__':main()
