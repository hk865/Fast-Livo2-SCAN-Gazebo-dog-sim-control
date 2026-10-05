#!/usr/bin/env python3
"""Append a source-verified truth-motion receipt without changing ancestors.

This combines already executed independent receipts. It cannot execute physics,
ROS, policy inference, training, or a command writer. It never assigns a score
or converts simulator-feedback success into SLAM navigation success.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import re
from pathlib import Path

MODEL_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
PROTOCOL_SHA = 'e2308b7b7585f933081cf741bd8670e950607cea495c3f4eca293e33ae7796a8'
EVALUATOR_SHA = '7e386d004e2d1187753609b73f6b9f419c417aa35c1a30147d28385ffcd9fb8a'
TERRAIN_SHA = '035600c3b00f0f9ab7fc2df28be1ec794d02b3bf9073ab07d205ca9a5dde96f5'
COMMON_SCHEMA = 'independent_truth_teacher_pid_benchmark/v2'
TERRAIN_SCHEMA = 'truth_teacher_actual_terrain_receipt/v1'
COMMON_CHECKS = ('physical_time_and_telemetry_coverage', 'original_control_clock_and_feedback_causality',
                 'causal_control_reference_coverage', 'native_physics_coverage_and_state_pairing',
                 'flat_or_requested_route_distance', 'drive_heading', 'stable_real_COM_speed',
                 'as_recorded_physical_safety', 'ordered_route_and_final_goal',
                 'declared_completion_and_termination', 'fixed_final_goal_parking', 'actual_runtime_receipt')
APPLICABILITY = 'strict_terrain_support_applicability'
SHARED_REQUIRED = ('truth_profile.json', 'telemetry.jsonl', 'control.jsonl', 'actuator.jsonl',
                   'worker_result.json', 'runtime_manifest.json', 'world.sdf',
                   'sources/truth/evaluate.py', 'sources/truth/protocol.json',
                   'sources/policy/observation.py', 'sources/policy/contract.json')


def status(value):
    return 'unverified' if value is None else 'passed' if bool(value) else 'failed'


def final_status(checks):
    values = [row['status'] for row in checks.values()]
    return 'failed' if 'failed' in values else 'passed' if values and all(v=='passed' for v in values) else 'unverified'


def required_status(checks, required):
    missing = [k for k in required if k not in checks or not isinstance(checks[k], dict) or checks[k].get('status') not in ('passed', 'failed', 'unverified')]
    failed = [k for k in required if isinstance(checks.get(k), dict) and checks[k].get('status')=='failed']
    unver = [k for k in required if isinstance(checks.get(k), dict) and checks[k].get('status')=='unverified']
    value = False if failed else None if missing or unver else True
    return {'status': status(value), 'required': list(required), 'missing': missing, 'failed': failed, 'unverified': unver}


def decision_checks(common, terrain, kind):
    """Pure decision logic used by offline negative tests; no IO or re-evaluation."""
    out = {}
    def check(key, value, **evidence):
        out[key] = {'status': status(value), **evidence}
    if common is None:
        check('complete_original_common_receipt', None, reason='Original common receipt unavailable')
        return out
    check('supported_common_receipt_schema', True if common.get('schema')==COMMON_SCHEMA else None,
          original_schema=common.get('schema'), supported_schema=COMMON_SCHEMA, old_pilot_not_promoted=True)
    cc = common.get('checks', {})
    if not isinstance(cc, dict): cc = {}
    out['all_required_common_motion_checks'] = required_status(cc, COMMON_CHECKS)
    extra_failed = [k for k, v in cc.items() if k not in COMMON_CHECKS and k not in (APPLICABILITY,'complete_parseable_required_evidence') and isinstance(v, dict) and v.get('status')=='failed']
    # An old parse-only interface pilot lacks the entire v2 schema. It remains
    # unverified. Any actual additional failure in a supported v2 receipt fails.
    check('no_additional_common_failure', not extra_failed if common.get('schema')==COMMON_SCHEMA else None,
          additional_failed_checks=extra_failed)
    check('complete_common_evidence_parse', None if cc.get('complete_parseable_required_evidence', {}).get('status')=='failed' else True,
          original_parse_check=cc.get('complete_parseable_required_evidence'),
          missing_or_unparseable_evidence_can_never_be_promoted_to_pass=True)
    applicable = cc.get(APPLICABILITY, {}).get('status')
    if kind=='flat':
        check('flat_original_direct_pass', common.get('status')=='passed' and applicable=='passed'
              if common.get('schema')==COMMON_SCHEMA and applicable in ('passed','failed','unverified') else None,
              original_status=common.get('status'), original_applicability=applicable,
              rule='Flat requires its original complete common receipt to pass directly')
        return out
    if kind not in ('ramp','step'):
        check('supported_prospective_terrain_kind', None, original_kind=kind)
        return out
    check('only_unimplemented_support_gate_may_be_replaced', applicable=='unverified' and common.get('status')=='failed'
          if common.get('schema')==COMMON_SCHEMA and applicable in ('passed','failed','unverified') else None,
          original_status=common.get('status'), original_applicability=applicable,
          rule='Only the v2 unverified support-applicability gate is replaced; no failed motion gate is waived')
    if terrain is None:
        check('complete_independent_terrain_receipt', None, reason='Independent actual-contact geometry receipt unavailable')
        return out
    check('supported_terrain_receipt_schema', True if terrain.get('schema')==TERRAIN_SCHEMA else None,
          original_schema=terrain.get('schema'), supported_schema=TERRAIN_SCHEMA)
    required = ['unchanged_common_truth_benchmark', 'native_cadence_and_unassisted_contract',
                'actual_contact_geometry_available', 'no_extended_all_feet_unsupported',
                'final_simultaneous_four_foot_destination_support']
    if kind=='ramp':
        required += ['complete_original_ramp_fixture', 'each_foot_ordered_landing_ramp_destination_support',
                     'entire_twelve_meter_contact_coverage', 'full_ramp_and_landings_forward_motion']
    else:
        required += ['continued_step_fixture', 'ascent_then_continued_actual_forward_walking']
        required += [leg+'_front_entry_and_true_tread_support' for leg in ('FR','FL','RR','RL')]
    tc = terrain.get('checks', {})
    if not isinstance(tc, dict): tc = {}
    out['all_required_actual_terrain_checks'] = required_status(tc, required)
    extra_bad = [k for k, v in tc.items() if isinstance(v, dict) and v.get('status')!='passed']
    check('independent_geometry_original_pass', terrain.get('status')=='passed' and terrain.get('strict_terrain_motion_verified') is True and
          terrain.get('terrain_kind')==kind and not extra_bad if terrain.get('schema')==TERRAIN_SCHEMA else None,
          original_status=terrain.get('status'), strict_geometry_verified=terrain.get('strict_terrain_motion_verified'),
          original_kind=terrain.get('terrain_kind'), nonpassed_geometry_checks=extra_bad)
    copied = terrain.get('common_benchmark_checks')
    filtered = {k:v for k,v in cc.items() if k!=APPLICABILITY}
    check('geometry_common_ancestor_checks_exact', copied==filtered if isinstance(copied,dict) else None,
          rule='Full copied common checks including values/status must equal the original common receipt')
    return out


class SourceVerifier:
    def __init__(self):
        self.cache = {}; self.missing = []; self.mismatches = []; self.verified = {}

    def digest(self, path):
        path = Path(path).resolve()
        if path in self.cache: return self.cache[path]
        if not path.is_file():
            self.missing.append(str(path)); return None
        before = path.stat(); h = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024), b''):h.update(block)
        after = path.stat()
        if (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns):
            self.mismatches.append({'path':str(path),'reason':'Source changed while hashed'})
        value = h.hexdigest(); self.cache[path]=value
        return value

    def verify(self, path, expected):
        canonical = Path(path).resolve(); actual = self.digest(canonical)
        if not isinstance(expected,str) or not re.fullmatch('[0-9a-f]{64}',expected):
            self.missing.append('Invalid/missing expected SHA256: '+str(canonical)); return None
        if actual is None:return None
        if actual!=expected:
            self.mismatches.append({'path':str(canonical),'expected':expected,'actual':actual});return False
        self.verified[str(canonical)] = actual;return True

    def source_map(self, run, value, required):
        if not isinstance(value,dict):
            self.missing.append('Ancestor input_source_sha256 is unavailable');return {}
        normalized = {}
        for raw, digest in value.items():
            path = Path(raw)
            if not path.is_absolute() or not path.resolve().is_relative_to(run):
                self.mismatches.append({'path':str(path),'reason':'Ancestor input must name canonical absolute source inside this run'});continue
            path = path.resolve()
            if str(path) in normalized and normalized[str(path)]!=digest:
                self.mismatches.append({'path':str(path),'reason':'Conflicting SHA256 for one source'})
            normalized[str(path)] = digest
            self.verify(path,digest)
        for name in required:
            if str((run/name).resolve()) not in normalized:self.missing.append('Missing ancestor source hash: '+name)
        return normalized


def compose(run, write=True):
    run=Path(run).resolve(); output=run/'summary_truth_motion_combined.json'
    if not run.is_dir(): raise ValueError('Existing completed run directory required')
    if write and output.exists(): raise ValueError('Refusing to replace prior combined evidence')
    verifier=SourceVerifier(); checks={}; docs={}; ancestors={}
    result={'schema':'truth_teacher_motion_combined_receipt/v1','run':str(run),'status':'unverified',
            'scope':'Frozen Teacher with simulator-truth outer velocity control: simulation motion benchmark only',
            'navigation_ground_truth_used':True,'SLAM_navigation_verified':False,'real_robot_verified':False,
            'strict_terrain_motion_verified':False,'score':None,'checks':checks,'ancestors':ancestors,
            'composition_rule':'Flat: original common receipt passes directly. Ramp/step: every original common motion gate except the v2 unimplemented support-applicability gate passes AND independent actual terrain geometry passes with identical source lineage. No original failure is overwritten.'}
    def check(name,value,**evidence):checks[name]={'status':status(value),**evidence}
    def document(name):
        path=run/name
        if not path.is_file():verifier.missing.append(str(path));return None
        try:return json.loads(path.read_text())
        except (ValueError,UnicodeError) as error:
            check('valid_json_'+name,False,reason=str(error));return None
    try:
        for name in ('truth_profile.json','runtime_manifest.json','policy_manifest.json','source_manifest.json',
                     'asset_manifest.json','sources/policy/contract.json','summary_truth_pid.json'):
            docs[name]=document(name)
        profile=docs['truth_profile.json'] or {}; common=docs['summary_truth_pid.json']
        terrain_text=str(profile.get('terrain',''))
        kind='flat' if terrain_text in ('flat','flat_short','flat_long','flat_roundtrip') else profile.get('terrain_acceptance',{}).get('kind')
        if kind is None:kind='ramp' if terrain_text.startswith('ramp') else 'step' if terrain_text.startswith('step') else None
        result['terrain_kind']=kind
        terrain=None
        if kind in ('ramp','step'):terrain=document('summary_truth_terrain.json')
        checks.update(decision_checks(common,terrain,kind))
        for name,data in (('common',common),('terrain',terrain)):
            filename='summary_truth_pid.json' if name=='common' else 'summary_truth_terrain.json'
            ancestors[name]=None if data is None else {'path':str(run/filename),'sha256':verifier.digest(run/filename),
                'status':data.get('status'),'schema':data.get('schema'),'checks':data.get('checks'),
                'input_source_sha256':data.get('input_source_sha256')}
            if data is not None:
                check(name+'_belongs_to_this_run',data.get('run')==str(run),ancestor_run=data.get('run'),expected_run=str(run))
                check(name+'_truth_scope_preserved',data.get('navigation_ground_truth_used') is True and
                      data.get('SLAM_navigation_verified') is False and data.get('real_robot_verified') is False,
                      navigation_ground_truth_used=data.get('navigation_ground_truth_used'),SLAM_navigation_verified=data.get('SLAM_navigation_verified'))
        cm=verifier.source_map(run,common.get('input_source_sha256') if common else None,SHARED_REQUIRED)
        tm={}
        if kind in ('ramp','step'):
            tm=verifier.source_map(run,terrain.get('input_source_sha256') if terrain else None,
                                   (*SHARED_REQUIRED,'sources/truth/terrain_receipt.py'))
            pairs=[(name,cm.get(str((run/name).resolve())),tm.get(str((run/name).resolve())))for name in SHARED_REQUIRED]
            different=[name for name,a,b in pairs if a is not None and b is not None and a!=b]
            missing=[name for name,a,b in pairs if a is None or b is None]
            check('common_and_geometry_identical_raw_source_lineage',False if different else None if missing else True,
                  conflicting_sources=different,missing_binding_sources=missing,required_shared_sources=list(SHARED_REQUIRED))
        supported=common is not None and common.get('schema')==COMMON_SCHEMA
        pinned=[('sources/truth/protocol.json',PROTOCOL_SHA),('sources/truth/evaluate.py',EVALUATOR_SHA)]
        if kind in ('ramp','step'):pinned.append(('sources/truth/terrain_receipt.py',TERRAIN_SHA))
        pinned_values=[verifier.verify(run/name,digest) for name,digest in pinned] if supported else []
        check('exact_frozen_acceptance_and_analyzer_sources',all(v is True for v in pinned_values) if supported and all(v is not None for v in pinned_values) else None,
              expected_sources=dict(pinned),old_unsupported_pilot_not_mapped_to_current_protocol=not supported)
        runtime=docs['runtime_manifest.json'] or {}; source_manifest=docs['source_manifest.json']
        if isinstance(source_manifest,dict):
            values=[]
            for relative,digest in source_manifest.items():
                path=(run/'sources'/relative).resolve()
                if Path(relative).is_absolute() or not path.is_relative_to(run/'sources'):
                    verifier.mismatches.append({'path':str(path),'reason':'Invalid archive-relative source path'});values.append(False)
                else:values.append(verifier.verify(path,digest))
            check('all_archived_executed_sources_intact',all(v is True for v in values) if values and all(v is not None for v in values) else None,
                  archived_files=len(source_manifest),source_manifest=source_manifest)
        else:check('all_archived_executed_sources_intact',None,reason='Missing source manifest')
        proof=[verifier.verify(run/'source_manifest.json',runtime.get('source_manifest_sha256')),
               verifier.verify(run/'truth_profile.json',runtime.get('truth_profile_sha256'))]
        check('runtime_binds_archived_sources_and_profile',all(v is True for v in proof) if all(v is not None for v in proof) else None,
              runtime_source_manifest_sha256=runtime.get('source_manifest_sha256'),runtime_profile_sha256=runtime.get('truth_profile_sha256'))
        asset=docs['asset_manifest.json'] or {}
        world_ok=verifier.verify(run/'world.sdf',asset.get('world_sha256'))
        check('actual_world_matches_asset_receipt',world_ok,asset_world_sha256=asset.get('world_sha256'))
        policy=docs['policy_manifest.json'] or {}; contract=docs['sources/policy/contract.json'] or {}
        model_path=contract.get('checkpoint')
        model_expected=[policy.get('checkpoint_sha256'),contract.get('checkpoint_sha256'),runtime.get('frozen_model_sha256')]
        if profile.get('model_sha256') is not None:model_expected.append(profile['model_sha256'])
        model_present=bool(model_path) and policy.get('checkpoint')==model_path and all(v is not None for v in model_expected)
        model_actual=verifier.verify(Path(model_path),MODEL_SHA) if model_path else None
        check('frozen_model_identity_and_cpu_policy',all(v==MODEL_SHA for v in model_expected) and model_actual is True and
              policy.get('inference_device')=='cpu' and policy.get('torch_threads')==1 if model_present and model_actual is not None else None,
              checkpoint_path=model_path,expected_sha256=MODEL_SHA,actual_sha256=verifier.cache.get(Path(model_path).resolve()) if model_path else None,
              metadata_hashes=model_expected,inference_device=policy.get('inference_device'),torch_threads=policy.get('torch_threads'))
        archive=contract.get('archive_hashes')
        archive_values=[verifier.verify(Path(path),digest) for path,digest in archive.items()] if isinstance(archive,dict) else []
        check('frozen_training_config_source_identity',all(v is True for v in archive_values) if len(archive_values)>=2 and all(v is not None for v in archive_values) else None,
              archived_config_sha256=archive)
        provider=policy.get('terrain_target_manifest')
        if profile.get('terrain_target_manifest'):
            if isinstance(provider,dict):
                pp=provider.get('path');pv=verifier.verify(Path(pp),provider.get('sha256')) if pp else None
                check('selected_policy_terrain_provider_bound',pv if pp and Path(pp).resolve().is_relative_to(run) and provider.get('world_sha256')==verifier.digest(run/'world.sdf') else False,
                      policy_provider=provider,strict_training_geometry_equivalence=False)
            else:check('selected_policy_terrain_provider_bound',None,reason='Declared policy terrain provider lacks actual receipt')
        check('truth_only_runtime_scope',runtime.get('uses_truth_for_control') is True and runtime.get('counts_as_SLAM_navigation') is False and
              profile.get('uses_truth_for_control') is True and profile.get('counts_as_SLAM_navigation') is False,
              truth_runtime_source=runtime.get('scope'))
        result['source_provenance']={
            'profile':{'path':str(run/'truth_profile.json'),'sha256':verifier.digest(run/'truth_profile.json'),'definition':profile},
            'protocol':{'path':str(run/'sources/truth/protocol.json'),'sha256':verifier.digest(run/'sources/truth/protocol.json'),'expected_sha256':PROTOCOL_SHA},
            'world':{'path':str(run/'world.sdf'),'sha256':verifier.digest(run/'world.sdf')},
            'model':{'path':model_path,'expected_sha256':MODEL_SHA,'actual_sha256':verifier.cache.get(Path(model_path).resolve()) if model_path else None},
            'policy_manifest':{'path':str(run/'policy_manifest.json'),'sha256':verifier.digest(run/'policy_manifest.json'),'definition':policy},
            'runtime':{'path':str(run/'runtime_manifest.json'),'sha256':verifier.digest(run/'runtime_manifest.json'),'definition':runtime,
                       'native_binary_sha256_recorded_at_execution':runtime.get('native_plugin_sha256'),
                       'native_binary_provenance_scope':'Execution receipt hash; original archived C++ source verified separately, no claim of an archived DSO if absent'},
            'archived_sources':{'path':str(run/'source_manifest.json'),'sha256':verifier.digest(run/'source_manifest.json'),'entries':source_manifest}}
    except (OSError,KeyError,TypeError,ValueError) as error:
        check('complete_parseable_composition_evidence',None,reason=type(error).__name__+': '+str(error))
    check('all_original_source_hashes_match',False if verifier.mismatches else None if verifier.missing else True,
          mismatches=verifier.mismatches,missing=sorted(set(verifier.missing)),no_source_or_ancestor_rewritten=True)
    result['status']=final_status(checks)
    result['strict_terrain_motion_verified']=result['status']=='passed' and result.get('terrain_kind') in ('ramp','step')
    result['failed_checks']=[k for k,v in checks.items() if v['status']=='failed']
    result['unverified_checks']=[k for k,v in checks.items() if v['status']=='unverified']
    result['verified_input_source_sha256']=verifier.verified
    result['composer_sha256']=verifier.digest(Path(__file__))
    if write:
        with output.open('x') as stream:json.dump(result,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True,nargs='+')
    args=parser.parse_args()
    for run in args.run:
        result=compose(run)
        print(json.dumps({'run':str(run.resolve()),'status':result['status'],'strict_terrain_motion_verified':result['strict_terrain_motion_verified'],
                          'score':None,'SLAM_navigation_verified':False,'failed_checks':result['failed_checks'],'unverified_checks':result['unverified_checks']}))


if __name__=='__main__':main()
