"""Current project policy and model locations; actual interfaces are validated by the actor."""
from pathlib import Path
import hashlib,json,sys
from project_config import ROOT,CONFIG,path
HERE=Path(__file__).resolve().parent
STUDENT_EXPORT_SHA='d8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185'
STUDENT_CHECKPOINT_SHA='7b59b8c7eafc9ffbc2dc32aaa2df3dadec3269d039338e4f93217bc73af66fd9'
TEACHER_CHECKPOINT_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def selection():
    worker=HERE/'policy_candidate/worker.py'
    if not worker.is_file():raise FileNotFoundError(worker)
    return dict(candidate_origin=str(worker),selected_destination='policy/worker.py')
def student_sources():
    files=dict(student_adapter=HERE/'student_actor_adapter.py',student_contract=CONFIG,student_export=path('student_model'))
    for value in files.values():
        if not value.is_file():raise FileNotFoundError(value)
    return files
def source_files():return [Path(selection()['candidate_origin']),*student_sources().values(),HERE/'policy_candidate/observation.py']
def archive_relative(original,root,data):
    original=Path(original).resolve()
    if original==Path(data['candidate_origin']):return Path('policy/worker.py')
    if original.parent==HERE/'policy_candidate':return Path('policy')/original.name
    return Path('navigation')/original.name
def binding(run):
    result=dict(candidate_origin=selection()['candidate_origin'],selected_snapshot=selection()['candidate_origin'])
    for role,origin in student_sources().items():
        result.update({role+'_origin':str(origin),role+'_snapshot':str(origin)})
    return result
def record_actual_loaded(run,legacy):
    observation=sys.modules[legacy.build_observation.__module__]
    paths=dict(worker=str(Path(legacy.__file__).resolve()),observation=str(Path(observation.__file__).resolve()))
    value=dict(actor_kind='student',actual_paths=paths,selected_binding=binding(run),actual_actor_load_pending=True)
    (Path(run)/'selected_policy_loaded.json').write_text(json.dumps(value,indent=2)+'\n')
    return value
