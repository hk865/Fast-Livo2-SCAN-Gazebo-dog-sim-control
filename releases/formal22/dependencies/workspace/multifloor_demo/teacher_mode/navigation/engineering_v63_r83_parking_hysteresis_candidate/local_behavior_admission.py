"""Strict exact local source delta and finite behavior receipt validation."""
from pathlib import Path
import hashlib,json
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify_local_behavior(here,contract):
    here=Path(here);p=Path(contract['behavior_patch_receipt']);d=json.loads(p.read_text())
    if d['candidate']!=str(here) or d['original_predicates_unchanged']is not True:
        raise RuntimeError('R57 local behavior identity differs')
    baseline=Path(d['baseline'])
    for name,patches in d['patches'].items():
        if sha(baseline/name)!=d['baseline_sha256'][name] or sha(here/name)!=d['candidate_sha256'][name]:
            raise RuntimeError('R57 local behavior source changed: '+name)
        text=(here/name).read_text()
        for patch in reversed(patches):
            if text.count(patch['after'])!=1:raise RuntimeError('R57 local delta ambiguous: '+name)
            text=text.replace(patch['after'],patch['before'])
        if text!=(baseline/name).read_text():raise RuntimeError('R57 unrelated behavior delta: '+name)
    if sha(here/'known_scene_hold_guard.py')!=d['new_module_sha256']:
        raise RuntimeError('R57 observation helper changed')
    cpu_path=Path(contract['behavior_cpu_receipt'])
    if sha(cpu_path)!=contract['behavior_cpu_receipt_sha256']:
        raise RuntimeError('R57 local CPU receipt changed')
    cpu=json.loads(cpu_path.read_text())
    if cpu.get('passed')is not True or cpu.get('simulation_started')is not False:
        raise RuntimeError('R57 local CPU proof incomplete')
    for name,digest in cpu['source_sha256'].items():
        if sha(name)!=digest:raise RuntimeError('R57 local CPU checked input changed: '+name)
    return True
