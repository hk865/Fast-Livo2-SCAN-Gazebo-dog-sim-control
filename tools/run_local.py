#!/usr/bin/env python3
"""New clone experiment entry; execute only after this host's fresh finite gate."""
from pathlib import Path
if not __debug__:
 raise RuntimeError("Optimized Python (-O/PYTHONOPTIMIZE) is forbidden for portable validation")
import argparse,os,subprocess,sys
from portable_common import REPO,NAMES,model_path,MODEL_SHA,sha

def main():
 p=argparse.ArgumentParser();p.add_argument('--variant',choices=['v18','v17'],required=True);p.add_argument('--profile',required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--cpu-python',type=Path,required=True);p.add_argument('--label',default='portable');p.add_argument('--domain',type=int,default=86);p.add_argument('--storage-root',type=Path);p.add_argument('--execute',action='store_true');a=p.parse_args()
 model=a.model.expanduser().resolve();cpu=a.cpu_python.expanduser().resolve()
 if sha(model)!=MODEL_SHA:raise RuntimeError('Frozen Teacher SHA256 differs')
 env={**os.environ,'TEACHER_REPO_ROOT':str(REPO),'TEACHER_MODEL_CHECKPOINT':str(model),'TEACHER_CPU_PYTHON':str(cpu),'PYTHONDONTWRITEBYTECODE':'1'}
 command=['/usr/bin/python3','-B',str(REPO/'multifloor_demo/teacher_mode/navigation'/NAMES[a.variant]/'run.py'),'--profile',a.profile,'--label',a.label,'--domain',str(a.domain),'--cpu-python',str(cpu)]
 if a.storage_root:
  root=a.storage_root.expanduser().absolute();env['TEACHER_RUN_STORAGE_ROOT']=str(root);command+=['--run-storage-root',str(root)]
 if not a.execute:command+=['--prepare-only']
 print('Local experimental '+('EXECUTE'if a.execute else'PREPARE_ONLY')+'; historical PASS not inherited',flush=True)
 result=subprocess.run(command,env=env,cwd=REPO)
 raise SystemExit(result.returncode)
if __name__=='__main__':main()
