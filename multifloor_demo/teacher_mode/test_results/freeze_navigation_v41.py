#!/usr/bin/env python3
"""Read-only runtime-source freeze and launch construction, no LaunchService."""
from pathlib import Path
import datetime,hashlib,importlib.util,json,os,sys
from launch import LaunchContext
from launch.actions import OpaqueFunction
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
root=Path(__file__).resolve().parents[1];nav=root/'navigation';sys.path.insert(0,str(nav))
from scoped_profile import verify_scope
from dynamic.dynamic_obstacle import verify_scope as verify_dynamic
from dynamic.prepare import validate_asset
spec=importlib.util.spec_from_file_location('teacher_nav_launch',nav/'stack.launch.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
req=SetEntityPose.Request();req.entity.name='moving_obstacle';req.entity.type=Entity.MODEL;req.pose.orientation.w=1.
rows=[]
for kind,run_id in [('flat','20261004_navigation_v41_freeze_flat'),('dynamic','20261004_navigation_v41a_freeze_dynamic')]:
    run=root/'runs'/run_id;os.environ.update(ROS_DOMAIN_ID='79',GZ_PARTITION='teacher_'+run.name,DEMO_RUN_DIR=str(run))
    result=verify_scope(run/'navigation_scope.json')
    if kind=='dynamic':verify_dynamic(run,run/'dynamic_protocol.json');validate_asset(run)
    ctx=LaunchContext();ctx.launch_configurations['run_dir']=str(run)
    ld=module.generate_launch_description();op=[a for a in ld.entities if isinstance(a,OpaqueFunction)][0]
    actions=op.execute(ctx);rows.append({'scope':kind,'run_dir':str(run),'scope_sha256':result['sha256'],
        'experiment':result['experiment'],'constructed_actions':len(actions),'starts_ROS':False,'starts_simulation':False})
checks={}
for name,path in [('nav_boundary',root/'test_results/navigation_v41_offline_checks.json'),
                  ('teacher_controller_methods',root/'test_results/navigation_v41_controller_method_checks.json'),
                  ('dynamic_fixture',root/'test_results/navigation_v41_dynamic_offline_checks.json'),
                  ('atomic_transport',root/'test_results/atomic_transport_review_20261004/review.json')]:
    data=json.loads(path.read_text());status=data.get('status','passed'if data.get('passed')is True else 'failed')
    checks[name]={'receipt':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'status':status,'count':len(data.get('checks',[]))}
    if status!='passed':raise RuntimeError('Offline check not passed: '+name)
files=sorted(nav.glob('*.py'))+sorted((nav/'dynamic').glob('*.py'))+sorted((nav/'dynamic').glob('*.json'))
files += [nav/'flat_relative_roundtrip.json',nav/'dynamic/obstacle_pose_observer.cpp',nav/'dynamic/CMakeLists.txt',
    nav/'dynamic/build/obstacle_pose_observer',root/'policy/worker.py',root/'policy/observation.py',root/'policy/contract.json',
    root/'simulation/build/libteacher_actuator.so',root/'simulation/prepare.py',root/'scripts/run_test.py',root/'runs/acceptance.json']
sourcehashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()for p in files}
receipt={'schema':1,'version':'navigation_v4.1_atomic_transport_and_explicit_dynamic_fixture',
    'status':'offline_ready_runtime_unverified','frozen_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'starts_ros':False,'starts_simulation':False,'offline_receipts':checks,'prepared_scopes':rows,'source_hashes':sourcehashes,
    'global_acceptance_preserved':json.loads((root/'runs/acceptance.json').read_text())['levels'],
    'transport_change':'only transient CommandFile fsync removed; flush/close/atomic replace and original wall/sim stamp/300msTTL/sequence/superseded semantics preserved',
    'physics_actor_control_changes':False,'dynamic_claim':'unverified until actual physical obstruction, parking, withdrawal and measured SLAM/SCAN recovery',
    'ordinary_flat_ready':True,'dynamic_flat_ready':True,'launchservice_started':False,'existing_failure_receipts_overwritten':False}
out=root/'test_results/navigation_v41_freeze.json';out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':receipt['status'],'freeze':str(out),'freeze_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),
    'protocol_sha256':sourcehashes['navigation/dynamic/protocol.json'],'runner_sha256':sourcehashes['scripts/run_test.py'],
    'checks':checks,'prepared_scopes':rows},indent=2))
