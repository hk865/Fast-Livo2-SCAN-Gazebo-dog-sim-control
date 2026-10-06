#!/usr/bin/env python3
"""Isolated own Python mock processes and direct launch command construction.

Uses the installed ros2run function's AST, never starts ROS/bridge/Gazebo.
"""
import ast,hashlib,json,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
API=Path('/opt/ros/jazzy/lib/python3.12/site-packages/ros2run/api/__init__.py')
PY=sys.executable

def wait_file(path,timeout=3.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if path.exists():return
        time.sleep(.01)
    raise RuntimeError('Mock child not ready')


def state(pid):
    try:return Path('/proc')/str(pid)/'stat'
    except OSError:return None


def running(pid):
    try:return (Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()[0]not in ('Z','X','x')
    except OSError:return False


def main():
    checks={};details={}
    def check(name,condition):
        checks[name]=bool(condition)
        if not condition:raise AssertionError(name)
    before=(HERE/'stack.launch.v41.py').read_text();after=(ROOT/'navigation/stack.launch.py').read_text()
    old="""                ExecuteProcess(cmd=['ros2','run','ros_gz_bridge','parameter_bridge',
                    '/world/teacher_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose'],output='screen'),"""
    new="""                Node(package='ros_gz_bridge',executable='parameter_bridge',
                    arguments=['/world/teacher_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose'],output='screen'),"""
    check('only_service_bridge_launch_target_changed',after.replace(new,old)==before and after.count(new)==1)
    function=[n for n in ast.parse(API.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='run_executable'][0]
    function_code=ast.unparse(function)
    # Two newly-created mock sessions. Signals are sent only to Popen-owned
    # handles/PIDs; no name matching, shared groups or actual ROS processes.
    with tempfile.TemporaryDirectory(prefix='teacher_mock_cleanup_')as tmp:
        tmp=Path(tmp);child=tmp/'mock_service.py';wrapper=tmp/'mock_ros2run.py'
        child.write_text("import json,os,signal,sys,time\nfrom pathlib import Path\np=Path(sys.argv[1]);p.write_text(str(os.getpid()))\ndef end(s,f):\n Path(sys.argv[2]).write_text(json.dumps({'signal':s,'pid':os.getpid()}));sys.exit(0)\nsignal.signal(signal.SIGINT,end)\nwhile True:time.sleep(.01)\n")
        wrapper.write_text('import os,signal,subprocess,sys\n'+function_code+'\nsys.exit(run_executable(path=sys.executable,argv=sys.argv[1:]))\n')
        pidfile=tmp/'wrapper_child.pid';receipt=tmp/'wrapper_child.sigint'
        proc=subprocess.Popen([PY,str(wrapper),str(child),str(pidfile),str(receipt)],start_new_session=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        try:
            wait_file(pidfile);child_pid=int(pidfile.read_text());os.kill(proc.pid,signal.SIGINT);time.sleep(.35)
            check('installed_ros2run_does_not_forward_parent_only_sigint',proc.poll()is None and running(child_pid)and not receipt.exists())
            os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=2.)
            check('wrapper_requires_owned_group_fallback_in_mock',proc.returncode==-signal.SIGTERM)
            details['wrapper']={'returncode':proc.returncode,'child_received_SIGINT':receipt.exists(),'scope':'Owned mock processes only'}
        finally:
            if proc.poll()is None:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2.)
        pidfile=tmp/'direct.pid';receipt=tmp/'direct.sigint'
        proc=subprocess.Popen([PY,str(child),str(pidfile),str(receipt)],start_new_session=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        try:
            wait_file(pidfile);begin=time.monotonic();os.kill(proc.pid,signal.SIGINT);proc.wait(timeout=2.);elapsed=time.monotonic()-begin
            check('direct_process_receives_launch_target_sigint',json.loads(receipt.read_text())['signal']==signal.SIGINT)
            check('direct_process_mock_exits_cleanly_without_escalation',proc.returncode==0 and elapsed<1.)
            details['direct']={'returncode':proc.returncode,'elapsed_wall_s':elapsed,'scope':'Owned mock process only; actual ROS cleanup remains unverified'}
        finally:
            if proc.poll()is None:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2.)
    from launch import LaunchContext
    from launch.substitutions import TextSubstitution
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    node=Node(package='ros_gz_bridge',executable='parameter_bridge',
        arguments=['/world/teacher_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose'],output='screen')
    context=LaunchContext();resolved=[perform_substitutions(context,c)for c in node.cmd]
    check('launch_resolves_direct_installed_service_bridge_binary',resolved[0]=='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge')
    check('service_argument_and_world_unchanged',resolved[1]=='/world/teacher_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose')
    check('no_ros2_cli_wrapper_in_new_command',not any(x=='ros2'for x in resolved))
    details['resolved_command']=resolved
    result={'schema':1,'status':'passed','checks':checks,'details':details,
        'actual_ROS_nodes_started':False,'Gazebo_started':False,'live_process_signals':False,
        'mock_process_signals':True,'runtime_cleanup_validation':'unverified_until_new_actual_run',
        'installed_ros2run_sha256':hashlib.sha256(API.read_bytes()).hexdigest(),
        'old_stack_sha256':hashlib.sha256(before.encode()).hexdigest(),
        'new_stack_sha256':hashlib.sha256(after.encode()).hexdigest()}
    (HERE/'offline_cleanup_receipt.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
