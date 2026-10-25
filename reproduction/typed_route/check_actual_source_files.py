from pathlib import Path
import json,os,sys,time,hashlib,traceback
base=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024');w=base/'worktrees/published_formal22_repro';o=w/'reproduction/typed_route';spec=json.loads((base/'trials/published_repro_03/SPECIFICATION.json').read_text());env=spec['environment'];os.environ.update({k:str(v) for k,v in env.items()});sys.path.insert(0,str(w/'candidate'));prof=json.loads(Path('/tmp/published_repro_04_source_closure_profile.json').read_text());r=dict(schema='published_r85_actual_source_files_check/v1',profile='/tmp/published_repro_04_source_closure_profile.json',environment_source=str(base/'trials/published_repro_03/SPECIFICATION.json'),no_ROS_Node_Gazebo_started=True)
start=time.monotonic()
try:
 import pid_scope
 sources=pid_scope.r85_source_files(prof);r['r85_source_files']=[dict(path=str(p),exists=p.is_file()) for p in sources]
 files=pid_scope.runtime_files(prof);r['runtime_files']=[dict(path=str(p),exists=p.is_file()) for p in files];r['missing']=[str(p) for p in files if not p.is_file()];r['passed']=not r['missing'];r['runtime_files_count']=len(files);r['r85_source_files_count']=len(sources)
except Exception as e:
 r.update(passed=False,error=repr(e),traceback=traceback.format_exc())
r['wall_s']=time.monotonic()-start;(o/'ACTUAL_SOURCE_FILES_CHECK.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ['r85_source_files','runtime_files']},indent=2))
