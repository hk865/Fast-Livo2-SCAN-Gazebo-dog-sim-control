#!/usr/bin/env python3
import hashlib,json,os,subprocess,sys,time
from pathlib import Path
root=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode')
profile=sys.argv[1]
if profile not in ('shm_512k','shm_64m'):raise ValueError(profile)
folder=root/'test_results/cloud_transport_actual_20261004'/sys.argv[2]
folder.mkdir(exist_ok=False)
helper=root/'tests/cloud_transport_probe_20261004/prepare_transport.py'
subprocess.run(['/usr/bin/python3',str(helper),'--run',str(folder),'--profile',profile],check=True,stdout=(folder/'prepare.log').open('w'))
manifest=json.loads((folder/'cloud_transport_manifest.json').read_text())
env=os.environ.copy()
for key in manifest['remove_environment_keys']:env.pop(key,None)
env.update(manifest['environment'])
env.update(ROS_DOMAIN_ID='79',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
observer=root/'tests/cloud_transport_probe_20261004/observer.py'
output=folder/'probe'
start=time.monotonic()
with (folder/'observer.log').open('x') as log:
 proc=subprocess.Popen(['/usr/bin/python3',str(observer),'--output',str(output)],env=env,stdout=log,stderr=subprocess.STDOUT)
 receipt={'schema':'root_owned_paired_qos_probe/v1','pid':proc.pid,'command':proc.args,'environment':{key:env.get(key) for key in set(manifest['environment'])|{'ROS_DOMAIN_ID','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'}},'removed_environment_keys':manifest['remove_environment_keys'],'manifest_sha256':hashlib.sha256((folder/'cloud_transport_manifest.json').read_bytes()).hexdigest(),'launcher_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'started_monotonic_wall':start,'separate_readonly_role':True,'actuator_or_navigation_publisher':False}
 (folder/'root_owned_probe_started.json').write_text(json.dumps(receipt,indent=2)+'\n')
 print(json.dumps({'root_owned_probe':str(folder),'pid':proc.pid}),flush=True)
 result=proc.wait()
 receipt.update(returncode=result,finished_monotonic_wall=time.monotonic(),process_exited=True)
 (folder/'root_owned_probe_exit.json').write_text(json.dumps(receipt,indent=2)+'\n')
 print(json.dumps({'probe_exit':result,'folder':str(folder)}),flush=True)
 raise SystemExit(result)
