#!/usr/bin/env python3
"""Independent exact payload join for the frozen status archive metadata.

Append-only receipt. Original common/v2/ramp and source bytes stay unchanged.
Only two producer-proven archival fields are removed in a virtual history view;
the original terrain-switch helper then executes every other original condition.
"""
from __future__ import annotations
import argparse,ast,hashlib,importlib.util,json,math,os,copy
from decimal import Decimal
from pathlib import Path
SCHEMA='independent_exact_terrain_status_payload_metadata_join/v1'
FIELDS={'monotonic_wall','ros_sim_time'}
COMMON='summary_closed_loop_cascade_independent.json';V2='summary_closed_loop_clock_hold_independent.json';RAMP='summary_closed_loop_ramp_independent.json'
COMMON_SHA='074b468581de568264f558e38f82ce25e809094843c6f86e2816130babf8099e'
def sha(p):
    h=hashlib.sha256()
    with p.open('rb')as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(line)for line in p.open()if line.strip()]
def canonical(d):return json.dumps(d,sort_keys=True,separators=(',',':'),allow_nan=False)
def check(value,**values):return dict(status='passed'if value is True else'failed'if value is False else'unverified',passed=value,**values)
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
def producer_proof(source):
    tree=ast.parse(source.read_text());klass=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='RecordedPublisher');method=next(n for n in klass.body if isinstance(n,ast.FunctionDef)and n.name=='publish')
    def named_call(n,name):return isinstance(n,ast.Expr)and isinstance(n.value,ast.Call)and isinstance(n.value.func,ast.Attribute)and n.value.func.attr==name
    app=[(i,n.value)for i,n in enumerate(method.body)if named_call(n,'append')and len(n.value.args)==2 and isinstance(n.value.args[0],ast.BinOp)and isinstance(n.value.args[0].right,ast.Constant)and n.value.args[0].right.value=='navigation_status.jsonl']
    atomic=[(i,n.value)for i,n in enumerate(method.body)if named_call(n,'atomic')and len(n.value.args)==2 and isinstance(n.value.args[0],ast.BinOp)and isinstance(n.value.args[0].right,ast.Constant)and n.value.args[0].right.value=='navigation_status.json']
    if len(app)!=1 or len(atomic)!=1:raise ValueError('Frozen status producer does not have unique original append/atomic calls')
    index,a=app[0];ai,b=atomic[0];value=a.args[1]
    if ai!=index+1 or not isinstance(b.args[1],ast.Name)or b.args[1].id!='data':raise ValueError('Original status payload differs or mutates between archive and file')
    if not isinstance(value,ast.Dict)or len(value.keys)!=3 or value.keys[0]is not None or not isinstance(value.values[0],ast.Name)or value.values[0].id!='data':raise ValueError('Archive payload is not the original whole data expansion')
    extra={k.value for k in value.keys[1:]if isinstance(k,ast.Constant)}
    if extra!=FIELDS:raise ValueError('Unexpected archive field additions')
    expected_wall=ast.parse('time.monotonic()',mode='eval').body
    expected_clock=ast.parse('self.node.get_clock().now().nanoseconds/1e9',mode='eval').body
    actual={k.value:v for k,v in zip(value.keys[1:],value.values[1:])}
    if ast.dump(actual['monotonic_wall'])!=ast.dump(expected_wall)or ast.dump(actual['ros_sim_time'])!=ast.dump(expected_clock):raise ValueError('Archival metadata clocks do not match frozen source')
    return dict(snapshot=str(source),sha256=sha(source),original_append_lineno=a.lineno,original_atomic_lineno=b.lineno,
        archival_fields_only=sorted(FIELDS),whole_payload='Original **data exact expansion, followed by atomic JSON file of the same data; no mutation between calls',
        exact_archive_AST=ast.dump(a,include_attributes=False),exact_original_file_AST=ast.dump(b,include_attributes=False))
def exact_join(status,history,event):
    if FIELDS&set(status):raise ValueError('Cannot remove metadata fields already present in the original status payload')
    matches=[]
    for index,row in enumerate(history):
        if set(row)-set(status)!=FIELDS or set(status)-set(row):continue
        if canonical({k:row[k]for k in status})==canonical(status):matches.append((index,row))
    if len(matches)!=1:raise ValueError('Original status must match exactly one history payload; observed '+str(len(matches)))
    index,row=matches[0];start=event['status_read']['read_started_monotonic_wall'];end=event['status_read']['read_completed_monotonic_wall'];recorded=event['recorded_monotonic_wall'];wall=row['monotonic_wall']
    if not all(type(v)in(int,float)and math.isfinite(v)for v in(wall,start,end,recorded)):raise ValueError('Nonfinite original causal wall clocks')
    if not wall<=start<=end<=recorded:raise ValueError('Archival status/read/actual switch wall ordering is noncausal')
    stamp=Decimal(str(row['ros_sim_time']))*Decimal(1_000_000_000)
    if stamp!=stamp.to_integral_value()or not event['arrival_stamp_ns']<=int(stamp)<=event['native_request_clock_ns']:raise ValueError('Original region/status/native source clock ordering is noncausal')
    return dict(original_history_index=index,unique_original_payload_matches=1,removed_archival_fields=sorted(FIELDS),removed_archival_values={k:row[k]for k in FIELDS},
        original_payload_field_count=len(status),all_original_payload_fields_equal=True,original_payload_sha256=hashlib.sha256(canonical(status).encode()).hexdigest(),
        original_whole_history_row_sha256=hashlib.sha256(canonical(row).encode()).hexdigest(),
        original_causal_wall_order_s=[wall,start,end,recorded],original_arrival_status_native_clock_ns=[event['arrival_stamp_ns'],int(stamp),event['native_request_clock_ns']],
        source_receipts_or_clock_freshness_refreshed=False)
def other_gates(common,v2,ramp):
    applicability='strict_nonflat_complete_contact_geometry';causal='original_source_authorized_causal_terrain_layer_switch'
    if common['checks'].get(applicability,{}).get('status')!='unverified'or v2['checks'].get(applicability,{}).get('status')!='unverified':raise ValueError('Original nonflat applicability differs')
    if any(v['status']!='passed'for k,v in common['checks'].items()if k!=applicability):raise ValueError('Another original common gate is not passed')
    if any(v['status']!='passed'for k,v in v2['checks'].items()if k!=applicability):raise ValueError('Another original full-publication gate is not passed')
    if any(v['status']!='passed'for k,v in ramp['checks'].items()if k!=causal):raise ValueError('Another original ramp gate is not passed')
    expected=['all_unchanged_original_common_gates','prospective_exact_ramp_contract','unchanged_first_active_hold_on_final_actual_landing']
    suffixes=['complete_original_12m_fixture','actual_COM_entryoutside_full_axis_exitoutside','all_named_feet_ordered_actual_top_support','first_destination_landing_continuous_support','all_twelve_metre_bins_actual_support','whole_traversal_real_support_and_axis_safety','actual_original_SLAM_destination_region_on_correct_physical_floor']
    expected += ['leg_'+str(i)+'_'+name+'_'+suffix for i,name in enumerate(['ramp_12','ramp_23'])for suffix in suffixes]
    if any(k not in ramp['checks']for k in expected):raise ValueError('Mandatory complete two-ramp gates missing')
    arrivals=common['checks']['all_original_SLAM_3D_region_arrivals']['arrivals']
    if len(arrivals)!=32 or any(a['status']!='passed'for a in arrivals):raise ValueError('All32 original arrivals required')
    return dict(original_common_nonflat_applicability_only='unverified',original_ramp_terrain_causal_only=ramp['checks'][causal],all_other_gates_passed=True,
        original_required_ramp_gates=expected,all32_arrivals_preserved=True,fixed_first_five_second_parking_preserved=True)
def evaluate(run):
    run=run.resolve();checks={};proofs={};receipts={name:read(run/name)for name in(COMMON,V2,RAMP)}
    for name,d in receipts.items():
        if Path(d['run']).resolve()!=run:raise ValueError('Foreign original receipt')
    if receipts[COMMON]['evaluator_sha256']!=COMMON_SHA or receipts[V2]['raw_common_receipt_sha256']!=sha(run/COMMON):raise ValueError('Original common/v2 ancestry changed')
    if receipts[V2].get('source_bindings_verified')is not True or receipts[V2].get('unchanged_common_checks_verified')is not True:raise ValueError('Original v2 binding/replay preservation not verified')
    if receipts[RAMP]['ancestors']['common']['sha256']!=sha(run/COMMON):raise ValueError('Original ramp ancestry changed')
    try:proofs['all_original_gates_preserved']=other_gates(receipts[COMMON],receipts[V2],receipts[RAMP]);checks['all_other_original_common_v2_ramp_numeric_gates']=check(True,**proofs['all_original_gates_preserved'])
    except ValueError as e:checks['all_other_original_common_v2_ramp_numeric_gates']=check(False,reason=str(e))
    snapshots=read(run/'navigation_source_snapshots.json');sources={}
    for basename in('teacher_wrapper.py','evaluate_ramp.py','evaluate_closed_loop.py'):
        found=[(name,data)for name,data in snapshots.items()if Path(name).name==basename]
        if len(found)!=1:raise ValueError('No unique frozen '+basename)
        name,data=found[0];p=Path(data['snapshot']).resolve()
        if not p.is_relative_to(run/'sources')or sha(p)!=data['sha256']:raise ValueError('Archived source hash differs')
        sources[basename]=p
    if receipts[RAMP]['evaluator_sha256']!=sha(sources['evaluate_ramp.py'])or sha(sources['evaluate_closed_loop.py'])!=COMMON_SHA:raise ValueError('Original frozen evaluator bytes differ')
    for name,digest in receipts[COMMON]['verified_input_source_sha256'].items():
        p=Path(name).resolve()
        if not p.is_relative_to(run)or not p.is_file()or sha(p)!=digest:raise ValueError('Original common source/input bytes changed: '+name)
    proofs['frozen_status_producer']=producer_proof(sources['teacher_wrapper.py']);checks['frozen_exact_archival_metadata_addition']=check(True,**proofs['frozen_status_producer'])
    event_list=rows(run/'terrain_provider_events.jsonl');switched=[e for e in event_list if e['event']=='switched_once']
    if len(switched)!=1:raise ValueError('Original single terrain switch missing')
    event=switched[0];raw=event['status_read']['raw_utf8'].encode();status=json.loads(raw);history=rows(run/'navigation_status.jsonl')
    if hashlib.sha256(raw).hexdigest()!=event['status_read']['raw_bytes_sha256']:raise ValueError('Provider original read byte SHA differs')
    try:proofs['exact_original_status_join']=exact_join(status,history,event);checks['exact_unique_full_payload_and_original_causal_join']=check(True,**proofs['exact_original_status_join'])
    except ValueError as e:checks['exact_unique_full_payload_and_original_causal_join']=check(False,reason=str(e))
    ramp=module(sources['evaluate_ramp.py'],'metadata_original_ramp');base=module(sources['evaluate_closed_loop.py'],'metadata_original_common')
    original_lines=ramp.lines
    def exact_virtual_lines(path):
        original=original_lines(path)
        if Path(path).resolve()==(run/'navigation_status.jsonl').resolve():return [{k:v for k,v in row.items()if k not in FIELDS}for row in original]
        return original
    if checks['exact_unique_full_payload_and_original_causal_join']['passed']:
        ramp.lines=exact_virtual_lines
        try:result=ramp.terrain_switch_audit(run,base.native_state(run),rows(run/'telemetry.jsonl'),receipts[RAMP]['prospective_contract'],base)
        finally:ramp.lines=original_lines
        checks['all_original_terrain_switch_ray_and_native_conditions']=result
    else:checks['all_original_terrain_switch_ray_and_native_conditions']=check(None,reason='Exact causal payload join refused; remaining helper not invoked')
    expected_source_hashes={str(p):sha(p)for p in sources.values()}
    input_names=[COMMON,V2,RAMP,'navigation_scope.json','navigation_source_snapshots.json','navigation_status.jsonl','terrain_provider_events.jsonl','terrain_provider_result.json','ramp_acceptance_contract.json','navigation_request.json','telemetry.jsonl','actuator.jsonl','terrain_target_manifest.json','alternate_terrain_target_manifest.json']
    bound={str(run/name):sha(run/name)for name in input_names};bound.update(expected_source_hashes)
    state='failed'if any(c['status']=='failed'for c in checks.values())else'unverified'if any(c['status']=='unverified'for c in checks.values())else'passed'
    return dict(schema=SCHEMA,run=str(run),status=state,passed=state=='passed',checks=checks,proofs=proofs,
        original_receipts={name:dict(path=str(run/name),sha256=sha(run/name),status=receipts[name]['status'])for name in receipts},
        verified_input_source_sha256=bound,evaluator_sha256=sha(Path(__file__)),
        correction_scope='Archive bookkeeping only: exact same whole original status payload plus frozen-producer two archival fields. No numeric/ray/phase/arrival/parking/source TTL/control criterion changes; original three receipts immutable.',
        retrospective_reader_correction_explicit=True,original_ramp_status_not_relabelled=True,navigation_ground_truth_used=False,
        limitations=['Static registered32-region route and two ramps plus first5s active hold only; no dynamic-obstacle or real-robot inference.','Actor observation inputs remain privileged and separate from sensor SLAM navigation feedback.'])
def self_test():
    status={'state':'running','nested':{'value':1},'arrival':'connector_mid'};row={**copy.deepcopy(status),'monotonic_wall':1.,'ros_sim_time':2.};event=dict(status_read=dict(read_started_monotonic_wall=1.1,read_completed_monotonic_wall=1.2),recorded_monotonic_wall=1.3,arrival_stamp_ns=1_000_000_000,native_request_clock_ns=3_000_000_000)
    exact_join(status,[row],event)
    bad_cases=[]
    changed=copy.deepcopy(row);changed['nested']['value']=2;bad_cases.append(('real_payload_difference',[changed],event))
    missing=copy.deepcopy(row);del missing['arrival'];bad_cases.append(('missing_original_field',[missing],event))
    bad_cases.append(('duplicate_exact_payload',[row,row],event));wrong=copy.deepcopy(event);wrong['status_read']['read_started_monotonic_wall']=.9;bad_cases.append(('noncausal_wall',[row],wrong))
    wrong=copy.deepcopy(event);wrong['native_request_clock_ns']=1_500_000_000;bad_cases.append(('noncausal_source_clock',[row],wrong))
    for name,history,e in bad_cases:
        try:exact_join(status,history,e)
        except ValueError:continue
        raise AssertionError('False PASS '+name)
    common={'checks':{'strict_nonflat_complete_contact_geometry':{'status':'unverified'},'source':{'status':'passed'}}};v2=copy.deepcopy(common);ramp={'checks':{'original_source_authorized_causal_terrain_layer_switch':{'status':'unverified'},'native':{'status':'failed'}}}
    try:other_gates(common,v2,ramp)
    except ValueError:pass
    else:raise AssertionError('Original other ramp failure became PASS')
    return dict(status='passed',meaningful_negative_groups=6,physical_operations=0)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path);p.add_argument('--self-test',action='store_true');a=p.parse_args()
    if a.self_test:print(json.dumps(self_test()));return
    if not a.run:p.error('--run required')
    d=evaluate(a.run);run=a.run.resolve();target=run/'summary_closed_loop_terrain_metadata_join_independent.json';body=json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if target.exists():raise ValueError('Immutable independent metadata-join receipt already exists')
    temp=run/('.terrain_metadata_join_'+str(os.getpid())+'.tmp');temp.write_text(body)
    try:os.link(temp,target)
    finally:temp.unlink()
    print(body)
if __name__=='__main__':main()
