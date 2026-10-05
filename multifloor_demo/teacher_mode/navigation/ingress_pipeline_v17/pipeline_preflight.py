"""Fail closed until the new pipeline's own limited checks are byte-bound.

This permits a bounded experiment; it never asserts throughput/navigation PASS.
"""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
REQUIRED=frozenset(('independent_build','bounded_queue_sanitized','production_packet_semantics',
                    'single_estimator_owner_source_audit','unchanged_original_controller',
                    'new_timing_semantics_explicit','actual_experiment_prospective_scope'))

def verify_pipeline_preflight(profile):
    receipt=json.loads((HERE/'INGRESS_PIPELINE_PREFLIGHT.json').read_text())
    if receipt.get('schema')!='limited_ordered_ingress_preflight/v17' or receipt.get('status')!='PASS_LIMITED_PURE_CHECKS':
        raise RuntimeError('V17 pure pipeline checks are not independently complete')
    if profile.get('ingress_pipeline')!={'schema':'single_owner_ordered_ingress/v17','enabled':True}:
        raise RuntimeError('Explicit V17 enabled profile is required')
    if profile.get('duration_s') not in (60,210):
        raise RuntimeError('V17 permits only frozen bounded experiments, not an unvalidated full route')
    if set(receipt.get('checks',{}))!=REQUIRED or not all(v is True for v in receipt['checks'].values()):
        raise RuntimeError('Missing/failed limited pipeline check')
    bindings=receipt.get('sha256',{})
    if not bindings or str((HERE/'INGRESS_PIPELINE_CONTRACT.json').resolve()) not in bindings:
        raise RuntimeError('Pipeline contract is not bound')
    for path,expected in bindings.items():
        actual=Path(path)
        if not actual.is_absolute() or not actual.is_file() or hashlib.sha256(actual.read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Pipeline source/build/test evidence changed: '+path)
    return receipt
