"""Root-only finite OFF/ON real publisher component, no navigation or model.

Uses the exact four original publisher-construction AST statements. Each mode
loads in a fresh process. Root owns the private ROS domain and runtime lock.
Parent uses a 4s timeout per process; the pair is bounded below 10s by root.
"""
import argparse,ast,copy,hashlib,json,os,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
NAMES=('r119_metadata_async.py','r119_metadata_async.xml','r119_writer_qos.cpp',
       'r119_writer_qos.so','r119_writer_qos_build.json')


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def child(out,transport_manifest):
    import r119_metadata_async as mode
    domain=os.environ.get('ROS_DOMAIN_ID')
    if domain is None or not 1<=int(domain)<=232:
        raise RuntimeError('Root must select the private ROS domain')
    out.mkdir(parents=True,exist_ok=False)
    origin=Path(transport_manifest).resolve();transport=json.loads(origin.read_bytes())
    inherited=dict(transport['environment'])
    for key in transport['remove_environment_keys']:os.environ.pop(key,None)
    os.environ.update(inherited)
    fixture=dict(transport,run=str(out))
    fixture_path=out/'cloud_transport_manifest.json'
    fixture_path.write_text(json.dumps(fixture,indent=2)+'\n')
    artifact_names=NAMES+(('r119_body_async.xml',)if mode.BODY_ENABLED else ())
    refs={str(HERE/name):digest(HERE/name)for name in artifact_names}
    refs.update(transport['source_hashes'])
    refs.update({str(origin):digest(origin),str(fixture_path):digest(fixture_path)})
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
        value=os.environ.get(key,'')
        if value:refs[str(Path(value).resolve())]=digest(value)
    (out/'navigation_scope.json').write_text(json.dumps({'references':refs}))
    setup=mode.configure(out)
    import rclpy
    from rclpy.node import Node
    from rclpy.context import Context
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import get_rmw_implementation_identifier
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    if get_rmw_implementation_identifier()!='rmw_fastrtps_cpp':raise RuntimeError('Actual RMW differs')
    context=Context();node=None;start=time.monotonic()
    try:
        rclpy.init(context=context)
        node=Node('r119_metadata_publisher_component',context=context)
        tree=ast.parse((HERE/'known_scene_runtime.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='main')
        cls=next(n for n in main.body if isinstance(n,ast.ClassDef)and n.name=='Runtime')
        init=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='__init__')
        names={'self.pubs','self.cloud_pub','self.bundle_pub','self.status_pub'}
        statements=[copy.deepcopy(n)for n in init.body if isinstance(n,ast.Assign)and any(ast.unparse(t)in names for t in n.targets)]
        assert len(statements)==4
        env=dict(self=node,Odometry=Odometry,PointCloud2=PointCloud2,String=String,
                 qos_profile_sensor_data=qos_profile_sensor_data)
        exec(compile(ast.Module(body=statements,type_ignores=[]),str(HERE/'known_scene_runtime.py'),'exec'),env)
        pubs=dict(metadata=node.bundle_pub,body=node.pubs['body'],lidar=node.pubs['lidar'],
            cloud=node.cloud_pub,status=node.status_pub)
        actual={name:mode.read_writer(pub)for name,pub in pubs.items()}
        for name,q in actual.items():
            assert q['publish_mode']==(1 if (name=='metadata'and mode.ENABLED)or(name=='body'and mode.BODY_ENABLED)else 0),(name,q)
        durations={}
        for name,pub in pubs.items():
            wall=[];cpu=[]
            for _ in range(50):
                msg=pub.msg_type()
                begin,begin_cpu=time.monotonic_ns(),time.thread_time_ns()
                pub.publish(msg)
                wall.append(time.monotonic_ns()-begin);cpu.append(time.thread_time_ns()-begin_cpu)
            durations[name]=dict(calls=len(wall),wall_sum_ns=sum(wall),wall_max_ns=max(wall),
                thread_cpu_sum_ns=sum(cpu),thread_cpu_max_ns=max(cpu),
                messages_are_component_type_fixtures=True,publish_return_is_not_delivery=True)
        if mode.ENABLED:mode.verify(node,out)
        result=dict(schema='r119_actual_publisher_component/v1',selected=mode.ENABLED,body_selected=mode.BODY_ENABLED,
            transport_manifest_origin=dict(path=str(origin),sha256=digest(origin)),
            component_fixture_only_run_field_changed=True,
            transport_environment_before=inherited,
            transport_environment_after={k:os.environ.get(k)for k in inherited},
            actual_RMW=get_rmw_implementation_identifier(),setup=setup,actual=actual,
            original_publisher_AST=ast.unparse(ast.Module(body=statements,type_ignores=[])),
            original_source_sha256=digest(HERE/'known_scene_runtime.py'),
            bindings={str(HERE/name):digest(HERE/name)for name in artifact_names},durations=durations,
            ROS_DOMAIN_ID=int(domain),wall_s=time.monotonic()-start,
            native_writer_selection_verified=True,navigation_or_cause_PASS=False,
            caveat='No matched subscriber, congestion, 331ms reproduction or navigation claim')
        (out/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown(context=context)


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--child',action='store_true');p.add_argument('--body-pair',action='store_true');p.add_argument('--transport-manifest',type=Path,required=True);a=p.parse_args()
    if a.child:return child(a.out.resolve(),a.transport_manifest)
    if a.out.exists():raise RuntimeError('Fresh component output required')
    a.out.mkdir(parents=True)
    begin=time.monotonic()
    for selected,name in [('0','OFF'),('1','ON')]:
        env=dict(os.environ,R119_METADATA_ASYNC='1'if a.body_pair else selected,
            R119_BODY_ASYNC=selected if a.body_pair else '0',PYTHONDONTWRITEBYTECODE='1')
        subprocess.run([sys.executable,str(Path(__file__).resolve()),'--out',str(a.out/name),'--transport-manifest',str(a.transport_manifest.resolve()),'--child'],
            env=env,check=True,timeout=4)
    off,on=(json.loads((a.out/name/'RESULT.json').read_bytes())for name in ('OFF','ON'))
    assert off['original_publisher_AST']==on['original_publisher_AST']
    assert off['transport_environment_before']==on['transport_environment_before']
    if a.body_pair:
        assert off['selected']and on['selected']and not off['body_selected']and on['body_selected']
    else:assert off['transport_environment_after']==off['transport_environment_before']
    mode_changes={k:[before,on['transport_environment_after'][k]]for k,before in on['transport_environment_before'].items() if before!=on['transport_environment_after'][k]}
    assert mode_changes=={'RMW_FASTRTPS_PUBLICATION_MODE':['SYNCHRONOUS','AUTO']}
    assert on['setup']['transport_baseline']['actual_environment_verified']
    if a.body_pair:
        assert off['transport_environment_after']==on['transport_environment_after']
        assert off['setup']['transport_baseline']['actual_environment_verified']
    differences={}
    for name in off['actual']:
        changes={key:[before,on['actual'][name][key]]for key,before in off['actual'][name].items()
                 if before!=on['actual'][name][key]}
        differences[name]=changes
        assert changes==({'publish_mode':[0,1]}if name==('body'if a.body_pair else'metadata')else {}),(name,changes)
    result=dict(schema='r119_OFF_ON_actual_writer_comparison/v1',actual_mode_PASS=True,body_only_pair=a.body_pair,
        metadata_ASYNC_preserved=bool(a.body_pair),
        transport_environment_changes=mode_changes,
        inherited_transport_environment=off['transport_environment_before'],
        transport_manifest_origin=off['transport_manifest_origin'],
        differences=differences,wall_s=time.monotonic()-begin,per_branch_timeout_s=4,
        unchanged='reliability/durability/history/depth/resource limits/memory/data-sharing/blocking time',
        source_sha256=digest(HERE/'known_scene_runtime.py'),navigation_or_cause_PASS=False)
    (a.out/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
