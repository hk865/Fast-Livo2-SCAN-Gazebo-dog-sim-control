"""Real ROS Node construction/close only, private domain210, no simulator/spin."""
import argparse,ast,copy,hashlib,json,os,shutil,sys,tempfile,time
from pathlib import Path
P=Path(__file__).resolve().parents[3]/'navigation/pipeline_v19'
sys.path.insert(0,str(P))
import rclpy
from rclpy.node import Node
import mission46_runtime as runtime
import mission46_obstacle_runtime as obstacle

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,d):Path(p).write_text(json.dumps(d,indent=2)+'\n')
def clone_sources(source, run):
    scope=copy.deepcopy(json.loads((source/'navigation_scope.json').read_text()))
    profile=json.loads((source/'navigation_profile.json').read_text())
    scope['run_dir']=str(run);scope['profile']=profile
    for file in runtime.required_runtime_files():scope['references'][str(file.resolve())]=sha(file)
    for name in ('sensor_contract.json',):
        shutil.copyfile(source/name,run/name);scope['references'][str(run/name)]=sha(run/name)
    snapshots=json.loads((source/'navigation_source_snapshots.json').read_text())
    for file, row in snapshots.items():
        if Path(file).name=='heading_alignment.py' and Path(file).parent.name=='slam':
            snapshot=run/'sources/heading_alignment.py';snapshot.parent.mkdir();shutil.copyfile(row['snapshot'],snapshot)
            row['snapshot']=str(snapshot);scope['references'][str(snapshot)]=sha(snapshot)
    write(run/'navigation_source_snapshots.json',snapshots);scope['references'][str(run/'navigation_source_snapshots.json')]=sha(run/'navigation_source_snapshots.json')
    interfaces=json.loads((source/'mission46_runtime_interfaces.json').read_text())
    for row in interfaces.values():row['implementation_sha256']=sha(row['module']);scope['references'][str(Path(row['module']).resolve())]=row['implementation_sha256']
    write(run/'mission46_runtime_interfaces.json',interfaces);scope['references'][str(run/'mission46_runtime_interfaces.json')]=sha(run/'mission46_runtime_interfaces.json')
    write(run/'navigation_scope.json',scope);write(run/'navigation_profile.json',profile)
    return scope,profile

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source-run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='210':raise RuntimeError('Require private domain210')
    checks={};fixtures=[]
    for filename in ('mission46_runtime.py','mission46_obstacle_runtime.py'):
        tree=ast.parse((P/filename).read_text());attrs={n.attr for n in ast.walk(tree)if isinstance(n,ast.Attribute)and isinstance(n.ctx,ast.Store)and isinstance(n.value,ast.Name)and n.value.id=='self'}
        collisions=sorted(attrs.intersection(dir(Node)));checks[filename+'_Node_attribute_collisions']=collisions
        if collisions:raise AssertionError(collisions)
    with tempfile.TemporaryDirectory(prefix='teacher_mission46_ros_construct_')as folder:
        base=Path(folder)
        for context_shutdown_first in (False,True):
            run=base/('valid_context'if not context_shutdown_first else'invalid_context');run.mkdir()
            scope,profile=clone_sources(a.source_run.resolve(),run)
            rclpy.init(args=[]);node=None;obs=None
            try:
                node=runtime.build_node(run,scope,profile);obs=obstacle.build_node(run)
                pub_topics=[p.topic_name for p in node.publishers]+[p.topic_name for p in obs.publishers]
                assert not any('cmd_vel'in p or'joint'in p or'action'in p for p in pub_topics)
                assert node.guard_tail.path==run/'navigation_guard_history.jsonl'
                subscription_topics=[sub.topic_name for sub in node.subscriptions]
                publisher_topics=[pub.topic_name for pub in node.publishers]
                assert set(['/demo/slam/body_odom','/livox/imu','/demo/navigation/status','/demo/control/safety','/demo/slam/map_status','/demo/obstacle/state']).issubset(subscription_topics)
                assert '/demo/teacher/mission46/control' in publisher_topics and '/demo/navigation/request' in publisher_topics
                node.started=True;node.apply(node.mission.start(run.name,wall_s=time.monotonic(),sim_ns=0))
                if context_shutdown_first:rclpy.shutdown()
                node.close();obs.close()
                state=json.loads((run/'mission46_status.json').read_text())
                assert state['stage']=='failed' and state['functional_sequence_completed']is False
                assert state['navigation_ground_truth_used']is False
                assert node.writer.error is None and obs.writer.error is None
                fixtures.append(dict(context_shutdown_first=context_shutdown_first,coordinator_constructed=True,obstacle_constructed=True,
                    subscription_topics=subscription_topics,publisher_topics=publisher_topics,closed_without_exception=True,state_stage=state['stage'],terminal_receipt_sha256=sha(run/'mission46_status.json'),
                    unexpected_command_publisher=False))
            finally:
                if obs:obs.destroy_node()
                if node:node.destroy_node()
                rclpy.try_shutdown()
    receipt=dict(schema='teacher_mission46_real_ros_construct_close/v1',passed=True,domain=210,spun=False,simulator_started=False,
        actor_started=False,source_run=str(a.source_run.resolve()),source_run_modified=False,temporary_scope_hashes_rebound_to_current_sources=True,
        source_sha256={str(f):sha(f)for f in runtime.required_runtime_files()},attribute_audit=checks,fixtures=fixtures,
        limitation='Real ROS construction/close covers API lifecycle only; no sensor initialization, actual movement or navigation has been verified.')
    write(a.output,receipt);print(json.dumps(receipt,indent=2))
if __name__=='__main__':main()
