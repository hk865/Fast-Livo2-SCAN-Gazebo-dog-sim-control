"""Read-only actual prepared closure and byte-freeze verification before launch."""
from pathlib import Path
import datetime,hashlib,json,sys
S=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
C=Path(read(S/'COPY_RECEIPT.json')['candidate']);R=Path(read(S/'PREPARED_RUN.json')['run'])
sys.path[:0]=[str(S),str(C)]
from pid_scope import prepared_dependency_files,verify_scope
scope=read(R/'navigation_scope.json');profile=scope['profile'];refs=scope['references']
snap=read(R/'navigation_source_snapshots.json');manifest=read(R/'source_manifest.json')
deps=prepared_dependency_files(R,profile);assert len(deps)==29
rows=[]
for p in deps:
 name=str(p);h=sha(p);row=snap[name];archive=Path(row['snapshot']);rel=str(archive.relative_to(R/'sources'))
 values=[h,refs[name],row['sha256'],sha(archive),refs[str(archive)],manifest[name],manifest[rel]]
 assert len(set(values))==1,(name,values)
 rows.append(dict(source=name,sha256=h,snapshot=str(archive),source_scope_snapshot_manifest_closed=True))
for n in ['terrain_target_manifest.json','mission46_runtime_interfaces.json']:
 assert (R/n).is_file(),n
verify_scope(R/'navigation_scope.json')
freeze=read(S/'SOURCE_FREEZE.json');mismatch=[p for p,h in freeze['bindings'].items()if not Path(p).is_file()or sha(p)!=h]
assert not mismatch,mismatch
record=dict(schema='R82_actual_full_prepare_dependency_closure/v1',UTC=datetime.datetime.now(datetime.timezone.utc).isoformat(),
 passed=True,run=str(R),candidate=str(C),dependency_count=len(rows),dependencies=rows,
 frozen_inputs=len(freeze['bindings']),frozen_mismatches=mismatch,
 normal_complete_prepare=True,old_failed_partial_prepare_reused=False,
 source_freeze_sha256=sha(S/'SOURCE_FREEZE.json'),scope_sha256=sha(R/'navigation_scope.json'),
 no_simulation_or_model_started_by_check=True)
with(S/'PREPARED_DEPENDENCY_CLOSURE.json').open('x')as f:json.dump(record,f,indent=2);f.write('\n')
print(json.dumps(dict(passed=True,dependencies=len(rows),freeze_inputs=len(freeze['bindings']),mismatches=0,run=str(R))))
