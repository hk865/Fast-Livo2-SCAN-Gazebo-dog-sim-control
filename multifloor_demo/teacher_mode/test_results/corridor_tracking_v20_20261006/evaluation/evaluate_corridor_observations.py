#!/usr/bin/env python3
"""Independent bounded-file audit of V20 corridor shadow observations.

Read archived inputs, snapshots, certificates and the frozen pure source only.
Never read simulator truth or infer free/support from absent LiDAR returns.
Replay is refused until runtime_manifest.json exists. Outputs go outside run/.
"""
import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics
import struct
import sys
import time

RUNTIME_FIELDS={'observation','recorded_clock_ns','active_path_id_at_receipt','control_authority'}
MAX_SNAPSHOT_BYTES=2_000_000


def digest(raw):return hashlib.sha256(raw).hexdigest()


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()


def file_sha(path):
    h=hashlib.sha256()
    with path.open('rb')as source:
        for raw in iter(lambda:source.read(1<<20),b''):h.update(raw)
    return h.hexdigest()


def rows(path,errors):
    if not path.exists():return
    with path.open('rb')as source:
        offset=0
        for number,raw in enumerate(source,1):
            ref=dict(file=str(path),line=number,offset=offset,length=len(raw),sha256=digest(raw));offset+=len(raw)
            try:
                if not raw.endswith(b'\n'):raise ValueError('incomplete final line')
                value=json.loads(raw)
                if not isinstance(value,dict):raise ValueError('nonobject row')
                yield value,ref
            except (ValueError,UnicodeError)as error:errors.append(dict(reason='invalid_jsonl',source=ref,error=str(error)))


def distribution(values):
    values=sorted(x for x in values if isinstance(x,(float,int))and math.isfinite(x))
    if not values:return dict(count=0)
    q=lambda p:values[min(len(values)-1,int(round((len(values)-1)*p)))]
    return dict(count=len(values),min=values[0],median=statistics.median(values),p95=q(.95),max=values[-1],mean=statistics.fmean(values))


def load_frozen_core(run):
    manifest=json.loads((run/'source_manifest.json').read_text())
    key='navigation/corridor_tracking_v20/corridor.py'
    path=run/'sources'/key
    if not path.exists()or key not in manifest:raise ValueError('frozen corridor source missing')
    if file_sha(path)!=manifest[key]:raise ValueError('frozen corridor source hash mismatch')
    spec=importlib.util.spec_from_file_location('audit_frozen_v20_corridor',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module,dict(path=str(path),sha256=file_sha(path),manifest_key=key)


def expect(condition,reason,reference,errors,detail=None):
    if not condition:
        errors.append(dict(reason=reason,source=reference,detail=detail));return False
    return True


def pure_result(certificate):return {k:v for k,v in certificate.items()if k not in RUNTIME_FIELDS}


def audit(run,replay='sample',sample_limit=20):
    run=run.resolve();finished=(run/'runtime_manifest.json').exists()
    if replay!='none'and not finished:raise RuntimeError('Replay requires a finished run; use --replay none for bounded observation audit.')
    errors=[];warnings=[];metrics=defaultdict(list);counters=defaultdict(Counter)
    try:core,source=load_frozen_core(run)
    except Exception as error:core=None;source=dict(error=str(error));errors.append(dict(reason='frozen_core_unavailable',detail=str(error)))
    certificates=list(rows(run/'corridor_certificates.jsonl',errors));inputs=list(rows(run/'corridor_inputs.jsonl',errors))
    by_sequence={}
    for cert,ref in certificates:
        observation=cert.get('observation',{});sequence=observation.get('sequence')
        if sequence in by_sequence:errors.append(dict(reason='duplicate_certificate_sequence',source=ref,detail=sequence))
        by_sequence[sequence]=(cert,ref)
        counters['certificate_status'][str(cert.get('status'))]+=1
        counters['certificate_reason'][str(cert.get('reason'))]+=1
        counters['interval_failures'].update(cert.get('failure_counts',{}))
        counters['width_rejections'].update(cert.get('width_rejection_counts',{}))
        support=cert.get('support_diagnostic',{})
        counters['support_cell_failures'].update(support.get('failure_counts',{}))
        if support: counters['support_model_status'][str(support.get('status'))]+=1
        if support.get('observed_planar_cells')is not None:metrics['observed_planar_cells'].append(support['observed_planar_cells'])
        expect(cert.get('shadow_only')is True and cert.get('control_authority')is False,
            'certificate_not_shadow_only',ref,errors)
        start=observation.get('compute_started_wall_ns');end=observation.get('compute_finished_wall_ns')
        received=observation.get('snapshot_received_wall_ns')
        if isinstance(start,int)and isinstance(end,int):
            metrics['compute_wall_ms'].append((end-start)/1e6)
            if end<start:errors.append(dict(reason='backward_compute_wall_clock',source=ref))
        if isinstance(received,int)and isinstance(start,int):metrics['worker_queue_wall_ms'].append((start-received)/1e6)
        if cert.get('status')=='certified':
            for key in ('left_m','right_m','stopping_distance_m'):metrics[key].append(cert.get(key))
            pure=pure_result(cert);expected=pure.pop('certificate_sha256',None)
            expect(expected==digest(canonical(pure)),'certificate_self_hash_mismatch',ref,errors)
            valid=cert.get('valid_until_ns');recorded=cert.get('recorded_clock_ns')
            if isinstance(valid,int)and isinstance(recorded,int):
                metrics['valid_margin_at_recorded_clock_ms'].append((valid-recorded)/1e6)
                counters['certified_expired_at_receipt'][str(recorded>valid)]+=1
            else:counters['certified_expiry_receipt_unavailable']['count']+=1
        active=cert.get('active_path_id_at_receipt')
        if active is not None:counters['path_changed_by_result_receipt'][str(active!=observation.get('path_id'))]+=1
    selected=[];replayed=[];replay_reasons=set();snapshot_receipts=[]
    for sequence,(entry,ref)in enumerate(inputs,1):
        path=entry.get('path',{});state=entry.get('actual_state',{});limits=entry.get('limits',{})
        now=entry.get('compute_request_clock_ns')
        try:
            file=(run/entry['snapshot_file']).resolve()
            if not file.is_relative_to(run):raise ValueError('snapshot path escapes run')
            with gzip.open(file,'rb')as compressed:raw=compressed.read(MAX_SNAPSHOT_BYTES+1)
            if len(raw)>MAX_SNAPSHOT_BYTES:raise ValueError('snapshot exceeds bounded payload')
            snapshot=json.loads(raw)
            expect(digest(raw)==entry['snapshot_sha256'],'snapshot_raw_hash_mismatch',ref,errors)
            snapshot_receipts.append(dict(sequence=sequence,file=str(file),gzip_sha256=file_sha(file),
                raw_sha256=digest(raw),raw_bytes=len(raw),gzip_bytes=file.stat().st_size))
        except Exception as error:
            errors.append(dict(reason='snapshot_unreadable',source=ref,detail=str(error)));continue
        expect(entry.get('control_authority')is False and entry.get('navigation_ground_truth_used')is False,
            'input_provenance_or_authority_invalid',ref,errors)
        if snapshot.get('complete'):
            expect(snapshot.get('navigation_ground_truth_used')is False,'snapshot_navigation_truth_flag_invalid',ref,errors)
        elif snapshot.get('navigation_ground_truth_used')is not False:
            # V20's refusal payload omits this flag. The pure reader safely
            # refuses it as invalid provenance; preserve the exporter reason.
            counters['incomplete_snapshot_metadata_issue']['missing_navigation_truth_flag']+=1
        counters['snapshot_complete'][str(snapshot.get('complete'))]+=1
        counters['snapshot_reason'][str(snapshot.get('reason'))]+=1
        counters['surface_complete'][str(snapshot.get('surface_points_complete'))]+=1
        for name in ('stamp_ns','source_cloud_stamp_ns','source_sensor_pose_stamp_ns','request_stamp_ns'):
            stamp=snapshot.get(name)
            if isinstance(stamp,int)and isinstance(now,int):metrics['age_at_request_ms:'+name].append((now-stamp)/1e6)
        raw_states=snapshot.get('states','');ages=snapshot.get('cell_observation_age_ms',[])
        if isinstance(raw_states,str):
            counts=Counter(raw_states);counters['snapshot_voxel_observations'].update(counts)
            for key,name in (('0','unknown'),('1','free'),('2','occupied')):metrics['snapshot_'+name+'_count'].append(counts[key])
            if raw_states:metrics['snapshot_free_fraction'].append(counts['1']/len(raw_states))
        else:errors.append(dict(reason='snapshot_states_not_string',source=ref));raw_states=''
        if len(ages)==len(raw_states)and raw_states and isinstance(now,int):
            delta=now-snapshot['stamp_ns'];deadline=limits.get('cell_max_age_ns',300_000_000)
            stale=sum(1 for flag,age in zip(raw_states,ages)if flag=='1'and(age<0 or age*1_000_000+delta>deadline))
            metrics['snapshot_stale_free_count'].append(stale)
            metrics['snapshot_stale_free_fraction'].append(stale/max(1,raw_states.count('1')))
        if isinstance(snapshot.get('surface_points_xyz'),list):metrics['surface_point_count'].append(len(snapshot['surface_points_xyz']))
        if sequence not in by_sequence:
            errors.append(dict(reason='input_without_completed_certificate',source=ref,detail=sequence));continue
        cert,cert_ref=by_sequence[sequence];observation=cert.get('observation',{});binding=cert.get('binding',{})
        recorded=cert.get('recorded_clock_ns')
        if isinstance(recorded,int):
            for name,source_stamp,limit in (
                    ('cloud',snapshot.get('source_cloud_stamp_ns'),limits.get('map_max_age_ns',300_000_000)),
                    ('pose',state.get('stamp_ns'),limits.get('state_max_age_ns',300_000_000))):
                if isinstance(source_stamp,int):
                    age=recorded-source_stamp;metrics[name+'_age_at_result_receipt_ms'].append(age/1e6)
                    counters[name+'_expired_at_result_receipt'][str(age>limit)]+=1
            metrics['source_clock_advance_during_async_ms'].append((recorded-now)/1e6)
        counters['status_by_route_layer'][str(path.get('layer_id'))+'|'+str(cert.get('status'))]+=1
        for key,expected in dict(snapshot_sha256=entry['snapshot_sha256'],map_revision=snapshot.get('revision'),
                path_id=path.get('path_id'),path_sha256=path.get('path_sha256'),frame_id=path.get('frame_id'),
                layer_id=path.get('layer_id'),source_pose_stamp_ns=state.get('stamp_ns'),
                requested_clock_ns=now,position_world_xyz=state.get('position_world_xyz'),
                progress_m=state.get('progress_m')).items():
            expect(observation.get(key)==expected,'observation_binding_mismatch:'+key,cert_ref,errors)
        if binding:
            point_hash=digest(b''.join(struct.pack('<ddd',*point)for point in path['points_xyz']))
            expected_bindings=dict(path_id=path['path_id'],path_sha256=path['path_sha256'],
                points_float64_sha256=point_hash,frame_id=path['frame_id'],layer_id=path['layer_id'],
                map_revision=snapshot.get('revision'),map_snapshot_sha256=digest(canonical(snapshot)),evaluated_at_ns=now)
            if core is not None:expected_bindings['limits_sha256']=digest(canonical({**core.DEFAULT_LIMITS,**limits}))
            for key,expected in expected_bindings.items():expect(binding.get(key)==expected,'certificate_binding_mismatch:'+key,cert_ref,errors)
        metrics['snapshot_payload_bytes'].append(len(raw));metrics['snapshot_gzip_bytes'].append(file.stat().st_size)
        reason_key=(cert.get('status'),cert.get('reason'),tuple(sorted(cert.get('failure_counts',{}))))
        pick=replay=='all'or(replay=='sample'and(len(selected)<sample_limit)and(reason_key not in replay_reasons or sequence%25==1))
        if pick:
            selected.append(sequence);replay_reasons.add(reason_key)
            if core is None:continue
            started=time.monotonic_ns()
            actual=core.certify_corridor(path,snapshot,None,state,limits,now)
            elapsed=(time.monotonic_ns()-started)/1e6
            expected=pure_result(cert);exact=canonical(actual)==canonical(expected)
            changed=[key for key in sorted(set(actual)|set(expected))if actual.get(key)!=expected.get(key)]
            receipt=dict(sequence=sequence,exact_pure_result_match=exact,changed_fields=changed,
                offline_compute_wall_ms=elapsed,input_source=ref,certificate_source=cert_ref,
                expected_status=cert.get('status'),replayed_status=actual.get('status'),
                expected_reason=cert.get('reason'),replayed_reason=actual.get('reason'))
            replayed.append(receipt)
            if not exact:errors.append(dict(reason='frozen_pure_replay_mismatch',source=cert_ref,detail=receipt))
    counters['input_rejection_reason'].update(str(row.get('reason'))for row,_ in rows(run/'corridor_input_rejections.jsonl',errors))
    worker=[row for row,_ in rows(run/'corridor_worker_receipt.jsonl',errors)]
    last_worker=worker[-1]if worker else None
    if finished and last_worker:
        expect(last_worker.get('submitted')==len(inputs),'worker_submitted_input_count_mismatch',{},errors,last_worker)
        expect(last_worker.get('completed')==len(certificates),'worker_completed_certificate_count_mismatch',{},errors,last_worker)
    elif finished:warnings.append('No worker close receipt; shutdown count reconciliation unverified.')
    extra=set(by_sequence)-set(range(1,len(inputs)+1))
    if extra:errors.append(dict(reason='certificates_without_input_sequence',detail=sorted(extra,key=str)))
    analysis=[]
    interval=counters['interval_failures'];support=counters['support_cell_failures']
    if interval.get('unknown_body_sweep'):analysis.append('Unknown swept cells are a measured-map coverage gap; no free space was inferred from absence.')
    if interval.get('stale_free_body_sweep'):analysis.append('Previously free swept cells lack sufficiently recent individual observations; a fresh global cloud does not refresh those cells.')
    if any('hull'in key or'gap'in key or'insufficient'in key for key in support):
        analysis.append('Support evidence is rejected by point coverage/edge/gap conditions; finite samples do not establish an unobserved floor.')
    if any('residual'in key or'slope'in key or'height'in key for key in support):
        analysis.append('Some local measured surfaces fail the configured planar, slope or body-height model; inspect retained clouds before changing any assumption.')
    if any(row.get('reason')=='frozen_pure_replay_mismatch'for row in errors):
        analysis.append('Exact frozen pure replay mismatch is an implementation/evidence consistency issue, not evidence of insufficient sensing.')
    if counters['certificate_reason'].get('actual_state_outside_corridor'):
        analysis.append('Actual-state/corridor mismatch needs geometry/progress review; it cannot be attributed to LiDAR coverage alone.')
    if metrics['compute_wall_ms']and max(metrics['compute_wall_ms'])>300:
        analysis.append('Some asynchronous computations exceed 300 ms wall time; actual source-clock expiry at receipt must be checked separately. Wall duration includes scheduling/GIL, not isolated CPU cost.')
    status='FAILED_INTEGRITY'if errors else'OBSERVATIONS_VERIFIED'
    if not finished:status='PARTIAL_RUNNING_OBSERVATION'
    consumed={}
    for name in ('corridor_inputs.jsonl','corridor_certificates.jsonl','corridor_worker_receipt.jsonl',
            'corridor_input_rejections.jsonl','source_manifest.json','runtime_manifest.json'):
        p=run/name
        if p.exists():consumed[name]=dict(sha256=file_sha(p),bytes=p.stat().st_size)
    return dict(schema='teacher_corridor_independent_observation_audit/v1',run=str(run),run_finished=finished,
        status=status,inputs=len(inputs),certificates=len(certificates),frozen_source=source,
        counters={key:dict(value)for key,value in counters.items()},
        distributions={key:distribution(value)for key,value in metrics.items()},worker_receipt=last_worker,
        busy_diagnostic_snapshots=None if last_worker is None else last_worker.get('busy_diagnostic_snapshots'),
        replay=dict(mode=replay,selected=selected,completed=len(replayed),all_selected_exact=all(x['exact_pure_result_match']for x in replayed)if replayed else None,receipts=replayed),
        interpretation=analysis,errors=errors,warnings=warnings,consumed_files=consumed,
        snapshot_receipts=snapshot_receipts,navigation_acceptance='UNVERIFIED',control_authority=False,
        exclusions=['No simulator truth consumed','No new free/support observations created',
            'No actuator/physics replay','Snapshot voxel counts include the exported AABB, not just the body sweep',
            'A geometric certified result is not a guarantee of foot contact, braking performance or locomotion stability'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--replay',choices=('none','sample','all'),default='sample')
    parser.add_argument('--sample-limit',type=int,default=20)
    args=parser.parse_args()
    output=args.output or Path(__file__).resolve().parent/(args.run.name+'_CORRIDOR_AUDIT.json')
    if output.resolve().is_relative_to(args.run.resolve()):raise SystemExit('Audit output must be outside the immutable run.')
    report=audit(args.run,args.replay,args.sample_limit)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),status=report['status'],inputs=report['inputs'],certificates=report['certificates'],
        statuses=report['counters'].get('certificate_status'),errors=len(report['errors']),replays=report['replay']['completed']),ensure_ascii=False))
    return int(report['status']=='FAILED_INTEGRITY')


if __name__=='__main__':raise SystemExit(main())
