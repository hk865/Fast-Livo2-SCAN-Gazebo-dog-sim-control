#!/usr/bin/env python3
"""Closed-source extension of the immutable metadata-join v1 reader.

Exact required check sets, receipt schemas, single permitted PI replacement,
all ancestor input hashes and v2 publication/hold hashes are enforced. Numeric
criteria and physical data are unchanged. The old four receipts remain intact.
"""
from __future__ import annotations
import argparse,copy,hashlib,json,os,tempfile
from pathlib import Path
import audit_terrain_metadata as original
SCHEMA='independent_exact_terrain_status_payload_metadata_join/v2'
CRITERION_SHA='756ad4c3f78a68f42483c9d016a7caa49e2bcbfd22f41595de4ac1675acb480c'
V1_SOURCE_SHA='1d464d0a70e9b7a5484a9535727683cd8118743a2d2282afa792ff302cb06c53'
STRICT='strict_nonflat_complete_contact_geometry';CAUSAL='original_source_authorized_causal_terrain_layer_switch'
REPLACED='actual_executor_ack_and_cascade_PI_COM_PD_replay'
EXTRA_PI='actual_executor_ack_and_PI_replay_with_explicit_hold_resets'
def passed(value):return value.get('status')=='passed'and value.get('passed')is True
def required_ramp():
    names={'all_unchanged_original_common_gates','prospective_exact_ramp_contract','actual_native_iteration_phase_continuity','actual_native_COM_and_origin_translation_consistency',
        'original_named_toe_world_contact_XYZ_normals_available',CAUSAL,'unchanged_first_active_hold_on_final_actual_landing'}
    names|={'native_recheck_'+name for name in('native_continuous_200Hz','native_physical_safety','native_force_velocity_and_support','exclusive_Teacher_CPU_identity')}
    suffixes=['complete_original_12m_fixture','actual_COM_entryoutside_full_axis_exitoutside','all_named_feet_ordered_actual_top_support','first_destination_landing_continuous_support','all_twelve_metre_bins_actual_support','whole_traversal_real_support_and_axis_safety','actual_original_SLAM_destination_region_on_correct_physical_floor']
    return names|{'leg_'+str(i)+'_'+name+'_'+suffix for i,name in enumerate(['ramp_12','ramp_23'])for suffix in suffixes}
def exact_gates(common,v2,ramp,common_required,extra_required):
    schemas={original.COMMON:'independent_actual_SLAM_SCAN_cascade_navigation/v1',original.V2:'independent_actual_SLAM_SCAN_publication_ledger_navigation/v2',original.RAMP:'independent_actual_SLAM_SCAN_complete_ramp_navigation/v1'}
    for name,receipt in[(original.COMMON,common),(original.V2,v2),(original.RAMP,ramp)]:
        if receipt.get('schema')!=schemas[name]:raise ValueError('Wrong original receipt schema: '+name)
    if set(common['checks'])!=common_required:raise ValueError('Original common mandatory check set differs')
    if set(v2['checks'])!=common_required|extra_required:raise ValueError('Original publication-v2 mandatory check set differs')
    if set(ramp['checks'])!=required_ramp():raise ValueError('Original full-ramp mandatory check set differs')
    if v2.get('explicit_replaced_common_checks')!=[REPLACED]:raise ValueError('The only permitted PI replacement differs')
    if original.canonical(v2.get('raw_common_checks'))!=original.canonical(common['checks']):raise ValueError('Stored original common checks differ')
    for key in common_required-{REPLACED}:
        if original.canonical(v2['checks'][key])!=original.canonical(common['checks'][key]):raise ValueError('Unpermitted original check replacement '+key)
    if original.canonical(v2['checks'][REPLACED])!=original.canonical(v2['checks'][EXTRA_PI]):raise ValueError('Explicit PI replacement differs from independent full-publication PI check')
    for receipt in(common,v2):
        value=receipt['checks'][STRICT]
        if value.get('status')!='unverified'or value.get('passed')is not None:raise ValueError('Original explicit nonflat applicability differs')
        if any(not passed(v)for k,v in receipt['checks'].items()if k!=STRICT):raise ValueError('Required common/v2 gate is failed or unverified')
    value=ramp['checks'][CAUSAL]
    if value.get('status')!='unverified'or value.get('passed')is not None or value.get('reason')!='MissingEvidence: Original provider status read not joined to raw status history':raise ValueError('Original ramp unverified cause is not the known exact archival-row mismatch')
    if any(not passed(v)for k,v in ramp['checks'].items()if k!=CAUSAL):raise ValueError('Required ramp gate is failed or unverified')
    if common.get('status')!='unverified'or v2.get('status')!='unverified'or ramp.get('status')!='unverified':raise ValueError('Original overall statuses differ from explicit applicability-only evidence')
    return dict(common_required_check_count=len(common_required),v2_required_check_count=len(common_required|extra_required),ramp_required_check_count=len(required_ramp()),
        common_required_check_keys=sorted(common_required),v2_extra_check_keys=sorted(extra_required),ramp_required_check_keys=sorted(required_ramp()),
        all_required_passed_flags_verified=True,exact_original_common_payload_preserved=True,only_permitted_replaced_check=REPLACED)
def verify_map(run,name,mapping):
    if not isinstance(mapping,dict)or not mapping:raise ValueError('Missing source/input binding map '+name)
    verified={}
    for key,digest in mapping.items():
        raw=Path(key);p=(raw if raw.is_absolute()else run/raw).resolve()
        if not p.is_relative_to(run)or not p.is_file():raise ValueError('Bound source missing/outside real run: '+name+' '+key)
        actual=original.sha(p)
        if actual!=digest:raise ValueError('Original ancestor source bytes changed: '+name+' '+key)
        verified[str(p)]=actual
    return verified
def closed_bindings(run):
    receipts={name:original.read(run/name)for name in(original.COMMON,original.V2,original.RAMP)};common,v2,ramp=(receipts[name]for name in(original.COMMON,original.V2,original.RAMP))
    if original.sha(Path(original.__file__))!=V1_SOURCE_SHA:raise ValueError('Immutable v1 metadata correction source changed')
    criterion=Path(v2['criteria_binding_source']).resolve()
    if not criterion.is_relative_to(run/'sources')or original.sha(criterion)!=CRITERION_SHA:raise ValueError('Frozen pre-test v2 criterion differs')
    contract=original.read(criterion)
    if v2.get('criteria_sha256')!=CRITERION_SHA or v2.get('parent_prospective_criteria_sha256')!=contract['parent_prospective_criteria_sha256']:raise ValueError('Original v2 criterion lineage differs')
    if v2.get('unchanged_common_numerical_criteria')!=contract['unchanged_common_numerical_criteria']or contract['permitted_replaced_common_checks']!=[REPLACED]:raise ValueError('Original numerical criteria or permitted replacement changed')
    snapshots=original.read(run/'navigation_source_snapshots.json');found=[d for name,d in snapshots.items()if Path(name).name=='evaluate_ramp.py']
    if len(found)!=1:raise ValueError('Unique original frozen ramp helper missing')
    path=Path(found[0]['snapshot']).resolve()
    if not path.is_relative_to(run/'sources')or original.sha(path)!=found[0]['sha256']or original.sha(path)!=ramp['evaluator_sha256']:raise ValueError('Frozen ramp helper source differs')
    helper=original.module(path,'closed_metadata_original_ramp');common_required=set(helper.REQUIRED_COMMON_CHECKS)|{helper.APPLICABILITY};extra_required=set(contract['required_extra_check_keys'])
    if len(common_required)!=19 or len(extra_required)!=7 or len(required_ramp())!=25:raise ValueError('Frozen required-set cardinalities differ')
    gates=exact_gates(common,v2,ramp,common_required,extra_required);maps={}
    if v2['verified_input_source_sha256']!=common['verified_input_source_sha256']:raise ValueError('Inherited original common binding map differs')
    for receipt_name,fields in[(original.COMMON,['verified_input_source_sha256']),(original.V2,['verified_input_source_sha256','clock_hold_verified_input_source_sha256','source_bindings']),(original.RAMP,['verified_input_source_sha256'])]:
        for field in fields:
            key=receipt_name+':'+field;maps[key]=verify_map(run,key,receipts[receipt_name].get(field))
    if v2['raw_common_receipt_sha256']!=original.sha(run/original.COMMON)or ramp['ancestors']['common']['sha256']!=original.sha(run/original.COMMON):raise ValueError('Ancestor original receipt SHA differs')
    return receipts,dict(exact_original_check_sets=gates,criterion_snapshot=str(criterion),criterion_sha256=CRITERION_SHA,required_common_set_source=str(path),
        required_common_set_source_sha256=original.sha(path),all_three_ancestor_binding_maps_rehashed=maps,immutable_v1_reader_sha256=V1_SOURCE_SHA)
def evaluate(run):
    run=run.resolve();_,closure=closed_bindings(run);data=original.evaluate(run)
    data.update(schema=SCHEMA,evaluator_sha256=original.sha(Path(__file__)),source_and_check_closure=closure,
        earlier_metadata_v1_receipt=dict(path=str(run/'summary_closed_loop_terrain_metadata_join_independent.json'),sha256=original.sha(run/'summary_closed_loop_terrain_metadata_join_independent.json'),
            status=original.read(run/'summary_closed_loop_terrain_metadata_join_independent.json')['status'],scope='Preserved v1 bookkeeping proof; v2 closes all ancestor source maps and exact mandatory gate sets'))
    data['checks']['all_three_ancestor_byte_bindings_and_exact_mandatory_check_sets']=original.check(True,**closure)
    data['verified_input_source_sha256'][str(run/'summary_closed_loop_terrain_metadata_join_independent.json')]=original.sha(run/'summary_closed_loop_terrain_metadata_join_independent.json')
    return data
def self_test():
    old=original.self_test();common_required={'source',STRICT,REPLACED};extra={EXTRA_PI,'actual_control_publication_ledger_complete_and_original'}
    ok=lambda:dict(status='passed',passed=True);unknown=lambda:dict(status='unverified',passed=None)
    common=dict(schema='independent_actual_SLAM_SCAN_cascade_navigation/v1',status='unverified',checks={k:ok()for k in common_required});common['checks'][STRICT]=unknown()
    v2=dict(schema='independent_actual_SLAM_SCAN_publication_ledger_navigation/v2',status='unverified',checks={**copy.deepcopy(common['checks']),**{k:ok()for k in extra}},explicit_replaced_common_checks=[REPLACED],raw_common_checks=copy.deepcopy(common['checks']))
    ramp=dict(schema='independent_actual_SLAM_SCAN_complete_ramp_navigation/v1',status='unverified',checks={k:ok()for k in required_ramp()});ramp['checks'][CAUSAL]={**unknown(),'reason':'MissingEvidence: Original provider status read not joined to raw status history'}
    exact_gates(common,v2,ramp,common_required,extra)
    bad=copy.deepcopy(v2);del bad['checks']['actual_control_publication_ledger_complete_and_original']
    try:exact_gates(common,bad,ramp,common_required,extra)
    except ValueError:pass
    else:raise AssertionError('Missing actual-publication gate became PASS')
    bad=copy.deepcopy(common);del bad['checks']['source']
    try:exact_gates(bad,v2,ramp,common_required,extra)
    except ValueError:pass
    else:raise AssertionError('Missing mandatory common gate became PASS')
    with tempfile.TemporaryDirectory(prefix='terrain_metadata_closed_sources_')as folder:
        root=Path(folder);p=root/'navigation_command_publications.jsonl';p.write_text('original ledger\n');digest=original.sha(p);verify_map(root,'v2 appended inputs',{str(p):digest});p.write_text('changed ledger\n')
        try:verify_map(root,'v2 appended inputs',{str(p):digest})
        except ValueError:pass
        else:raise AssertionError('Changed original additional ledger bytes became PASS')
    return dict(status='passed',meaningful_negative_groups=old['meaningful_negative_groups']+3,physical_operations=0)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path);p.add_argument('--self-test',action='store_true');a=p.parse_args()
    if a.self_test:print(json.dumps(self_test()));return
    if not a.run:p.error('--run required')
    d=evaluate(a.run);run=a.run.resolve();target=run/'summary_closed_loop_terrain_metadata_join_independent_v2.json';body=json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if target.exists():raise ValueError('Immutable v2 metadata-join receipt exists')
    temp=run/('.terrain_metadata_join_v2_'+str(os.getpid())+'.tmp');temp.write_text(body)
    try:os.link(temp,target)
    finally:temp.unlink()
    print(body)
if __name__=='__main__':main()
