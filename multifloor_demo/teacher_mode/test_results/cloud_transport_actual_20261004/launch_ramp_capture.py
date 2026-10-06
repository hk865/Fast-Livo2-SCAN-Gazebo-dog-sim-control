#!/usr/bin/env python3
"""Root-owned read-only ramp capture; inherit the actual run transport only."""
import hashlib,json,os,subprocess,sys,time
from pathlib import Path
root=Path(__file__).resolve().parents[2]
run=Path(sys.argv[1]).resolve()
folder=Path(sys.argv[2]).resolve()
if run.parent!=(root/'runs').resolve():raise ValueError('Actual teacher run required')
folder.mkdir(parents=True,exist_ok=False)
manifest_path=run/'cloud_transport_manifest.json'
manifest=json.loads(manifest_path.read_text())
env=os.environ.copy()
for key in manifest['remove_environment_keys']:env.pop(key,None)
env.update(manifest['environment'])
env.update(ROS_DOMAIN_ID='79',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
observer=root/'navigation/ramp_registration/observer.py'
command=['/usr/bin/python3',str(observer),'--source-run',str(run),'--output',str(folder/'capture'),
         '--duration-s','60','--wall-timeout-s','180','--environment-manifest',str(manifest_path)]
start=time.monotonic()
with (folder/'observer.log').open('x') as log:
    proc=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    receipt={'schema':'root_owned_actual_ramp_capture/v1','run':str(run),'pid':proc.pid,
             'command':command,'environment':{k:env.get(k)for k in set(manifest['environment'])|{'ROS_DOMAIN_ID','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'}},
             'removed_environment_keys':manifest['remove_environment_keys'],
             'manifest_sha256':hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
             'launcher_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             'started_monotonic_wall':start,'readonly':True,'navigation_or_actuator_publisher':False}
    (folder/'root_owned_capture_started.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps({'root_owned_capture':str(folder),'pid':proc.pid}),flush=True)
    result=proc.wait()
    receipt.update(returncode=result,finished_monotonic_wall=time.monotonic(),process_exited=True)
    (folder/'root_owned_capture_exit.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps({'capture_exit':result,'folder':str(folder)}),flush=True)
    raise SystemExit(result)
