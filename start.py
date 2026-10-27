#!/usr/bin/env python3
"""Start/check the one project configuration. ROS/system dependencies are external."""
from pathlib import Path
import argparse,datetime as dt,json,os,sys,uuid
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src'))
from project_config import read,path

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='Validate files/config/imports only; no ROS nodes or simulation')
    parser.add_argument('--prepare-only',action='store_true',help='Generate fresh runtime inputs; no ROS nodes or simulation')
    parser.add_argument('--domain',type=int,default=231)
    parser.add_argument('--label',default='navigation')
    args=parser.parse_args()
    if not 1<=args.domain<=232:parser.error('Domain must be 1..232')
    os.chdir(ROOT)
    profile=read()
    # A new bounded wall budget is a runtime safety setting, not a historical
    # permission, version or contract. Never renewed after starting a run.
    profile['r85_session']['deadline_utc']=(dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=2700)).isoformat()
    for k,v in profile['features'].items():os.environ[k]=v
    os.environ['PYTHONDONTWRITEBYTECODE']='1'
    import run
    run.validate_profile(profile)
    from slam_workspace import verify_slam_workspace
    from scan_workspace import scan_contract
    verify_slam_workspace();scan_contract(profile)
    from dds_environment import library_path
    library_path()
    for name in ['student_model','actuator','actor_python']:
        if not path(name).is_file():raise FileNotFoundError(name+': '+str(path(name)))
    if not (ROOT/'src/run.py').is_file():raise FileNotFoundError('Project runner missing')
    if args.check:
        print(json.dumps(dict(passed=True,project=str(ROOT),config='config/navigation.json',basic_configuration_validated=True,complete_launch_validation=False,simulation_started=False)))
        return
    state=ROOT/'.local/launches';state.mkdir(parents=True,exist_ok=True)
    selected=state/(dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:6]+'.json')
    with selected.open('x')as f:json.dump(profile,f,indent=2);f.write('\n')
    (ROOT/'runs').mkdir(exist_ok=True)
    if not args.prepare_only and os.getsid(0)!=os.getpid():os.setsid()
    argv=['/usr/bin/python3','-B',str(ROOT/'src/run.py'),'--profile',str(selected),'--label',args.label,'--domain',str(args.domain),'--real-time-factor','1.0','--surface-validity','0','--cpu-python',str(path('actor_python'))]
    if args.prepare_only:argv.append('--prepare-only')
    os.execvpe(argv[0],argv,os.environ)

if __name__=='__main__':main()
