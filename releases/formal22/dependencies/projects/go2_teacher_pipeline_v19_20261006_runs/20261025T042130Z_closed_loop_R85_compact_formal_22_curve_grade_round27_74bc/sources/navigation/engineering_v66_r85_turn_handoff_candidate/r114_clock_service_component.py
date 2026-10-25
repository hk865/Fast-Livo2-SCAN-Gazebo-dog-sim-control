"""Root-run finite real ROS component; no Gazebo/model/navigation admission claim.

Parent runs OFF and ON in separate processes with immutable import selectors.
Actual PID on_odom delegates to an explicit Teacher sentinel; no downstream
Teacher/physical/parking success is claimed. R86 bundle is a scalar fixture.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent


def child(out):
    import rclpy
    from rclpy.node import Node
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.clock import ROSClock, JumpThreshold
    from rclpy.duration import Duration
    from rclpy.impl.implementation_singleton import rclpy_implementation as impl
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from scan_planner_msgs.msg import Bspline
    from collections import deque
    from types import SimpleNamespace, MethodType
    import controller
    import r114_clock_service as service_module
    import r99_clock_probe as clock_probe
    import r107_scan_take_probe as scan_probe
    import r107_scan_transport_probe as transport

    selected = os.environ['R114_CLOCK_SERVICE'] == '1'
    assert service_module.CLOCK_SERVICE_ENABLED is selected
    assert controller.CLOCK_SERVICE_ENABLED is selected
    root = HERE.parents[2]
    profile = json.loads((root/'trials/poll_yield/PROFILE.json').read_text())
    context = Context();rclpy.init(context=context)
    checks, bodies, timings = {}, [], []
    node = executor = None
    class TeacherReached(Exception):pass
    original_teacher = controller.TeacherNavigation.on_odom
    original_sensor = controller._sensor_callback_odom
    def sentinel(owner, msg):raise TeacherReached()
    controller.TeacherNavigation.on_odom = sentinel
    controller._sensor_callback_odom = lambda *a, **k: None
    class Fixture(controller.PIDNavigation):
        def __init__(self):
            self.profile=profile;self.run=out
            Node.__init__(self,'r114_clock_component',context=context)
            self.set_parameters([Parameter('use_sim_time',value=True)])
            self._r86_prepared_mailbox=object();self._r99_policy=None
            self._r86_committing_bundle=None;self._r104_parking_causal_join=None
            self.pose_stamp=97_880_000_000;self.imu_history=deque();self.imu_source_buffer=None
            self.gyro_body=self.gyro_stamp=self.gyro_wall=None
            self.heading_gate=SimpleNamespace(quiet_target_enabled=False)
        def body(self,msg):
            source=int(self.stamp(msg));self._r86_committing_bundle=SimpleNamespace(source_ns=source)
            begin,cpu=time.monotonic_ns(),time.thread_time_ns()
            result='future_rejected'
            try:
                controller.PIDNavigation.on_odom(self,msg)
            except TeacherReached:result='delegated_to_teacher_sentinel'
            except BaseException as error:result=type(error).__name__+':'+str(error)
            finally:
                self._r86_committing_bundle=None
                timings.append(dict(wall_ns=time.monotonic_ns()-begin,cpu_ns=time.thread_time_ns()-cpu))
                bodies.append(dict(source_ns=source,clock_ns=self.get_clock().now().nanoseconds,result=result))
    def wait_ready(sub):
        with sub.handle:
            waitset=impl.WaitSet(1,0,0,0,0,0,context.handle)
            waitset.clear_entities();waitset.add_subscription(sub.handle);waitset.wait(2_000_000_000)
            assert sub.handle.pointer in waitset.get_ready_entities('subscription'),'DDS message not ready within 2s'
    def wait_match(pub):
        deadline=time.monotonic()+2
        while pub.get_subscription_count()<1 and time.monotonic()<deadline:time.sleep(.005)
        assert pub.get_subscription_count()>=1,'DDS discovery not matched within 2s'
    def dispatch(sub, task=None):
        task=executor._make_handler(sub,node,executor._take_subscription) if task is None else task
        executor._executor.submit(task).result(timeout=2)
        assert task.done(),'original handler unexpectedly suspended'
        task.result()
    def clock_message(ns):
        msg=Clock();msg.clock.sec=ns//10**9;msg.clock.nanosec=ns%10**9
        clockpub.publish(msg);wait_ready(clocksub)
    def body_message(ns):
        msg=Odometry();msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
        bodypub.publish(msg);wait_ready(bodysub);dispatch(bodysub)
        return bodies[-1]['result']
    result = None
    try:
        node=Fixture();executor=MultiThreadedExecutor(num_threads=2,context=context)
        for topic,kind in scan_probe.TOPICS.items():
            node.create_subscription(Bspline if kind=='spline' else String,topic,lambda msg:None,1)
        node._r88_admission=SimpleNamespace(transport_probe=transport.Recorder(node))
        clock_probe.install(node,executor);scan_probe.install(node,executor)
        node._r114_clock_service=service_module.install(node,executor)
        executor.add_node(node)
        clocksub=next(s for s in node.subscriptions if s.topic_name=='/clock')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        clockpub=node.create_publisher(Clock,'/clock',qos)
        bodysub=node.create_subscription(Odometry,'/r114_component/body',node.body,qos)
        bodypub=node.create_publisher(Odometry,'/r114_component/body',qos)
        wait_match(clockpub);wait_match(bodypub)
        extra=ROSClock();node._time_source.attach_clock(extra)
        jumps=[]
        jump=node.get_clock().create_jump_callback(JumpThreshold(min_forward=None,min_backward=Duration(nanoseconds=-1),on_clock_change=False),post_callback=lambda event:jumps.append(event.delta.nanoseconds))
        clock_message(97_950_000_000);dispatch(clocksub)
        clock_message(98_055_000_000)
        queued=executor._make_handler(clocksub,node,executor._take_subscription)
        before_count=clock_probe.PROBE.callback_serial
        observed=body_message(97_980_000_000)
        checks['actual_pid_future_gate']=observed==('delegated_to_teacher_sentinel' if selected else 'future_rejected')
        checks['queued_marker_preserved']=clocksub._executor_event is True
        count_after_body=clock_probe.PROBE.callback_serial
        dispatch(clocksub,queued)
        checks['queued_handler_no_duplicate_update']=(clock_probe.PROBE.callback_serial-before_count==1 and clocksub._executor_event is False and clocksub.callback_group.can_execute(clocksub))
        checks['inline_probe_callback_once']=count_after_body-before_count==int(selected)
        checks['all_original_associated_clocks']=extra.now().nanoseconds==node.get_clock().now().nanoseconds==98_055_000_000
        checks['empty_preserves_future_rejection']=body_message(98_080_000_000)=='future_rejected'
        clock_message(98_100_000_000)
        assert clocksub.callback_group.beginning_execution(clocksub)
        try:checks['busy_preserves_future_rejection']=body_message(98_080_000_000)=='future_rejected'
        finally:clocksub.callback_group.ending_execution(clocksub);executor.wake()
        dispatch(clocksub)
        entered,release=threading.Event(),threading.Event()
        def pause_before_update():
            entered.set()
            if not release.wait(2):raise RuntimeError('component pause exceeded 2s')
        paused_jump=node.get_clock().create_jump_callback(
            JumpThreshold(min_forward=Duration(nanoseconds=1),min_backward=None,on_clock_change=False),pre_callback=pause_before_update)
        clock_message(98_150_000_000)
        inflight=executor._make_handler(clocksub,node,executor._take_subscription)
        worker=executor._executor.submit(inflight)
        try:
            assert entered.wait(1),'original clock never reached pre-update jump callback'
            checks['already_taken_clock_remains_group_busy']=body_message(98_120_000_000)=='future_rejected'
        finally:
            release.set();worker.result(timeout=2);inflight.result();paused_jump.unregister()
        checks['inflight_update_completes_once']=node.get_clock().now().nanoseconds==98_150_000_000 and clocksub.callback_group.can_execute(clocksub)
        clock_message(97_900_000_000)
        # OFF has no inline service: perform its original handler before asking
        # the original gate to observe the rollback. ON services it at the body.
        if not selected:dispatch(clocksub)
        checks['rollback_preserves_original_future_gate']=body_message(98_120_000_000)=='future_rejected'
        checks['original_jump_and_multiple_clocks']=bool(jumps) and jumps[-1]<0 and extra.now().nanoseconds==node.get_clock().now().nanoseconds==97_900_000_000
        checks['clock_group_free']=clocksub.callback_group.can_execute(clocksub)
        checks['probe_context_unwound']=clock_probe._TAKE.get() is None
        if selected:
            svc=node._r114_clock_service
            def jump_error():raise ValueError('explicit_component_jump_error')
            failing_jump=node.get_clock().create_jump_callback(
                JumpThreshold(min_forward=Duration(nanoseconds=1),min_backward=None,on_clock_change=False),pre_callback=jump_error)
            try:
                clock_message(98_200_000_000)
                observed=body_message(200_000_000_000)
                checks['original_clock_callback_exception_releases_group']=('explicit_component_jump_error' in observed and clocksub.callback_group.can_execute(clocksub) and clock_probe._TAKE.get() is None)
            finally:failing_jump.unregister()
            # Explicit unexpected-suspend injection: not a claim about real TimeSource.
            oldtake,svctake=executor._take_subscription,svc.take
            closed=[]
            class YieldOnce:
                def __await__(self):yield 'unexpected'
            def inject(self,sub):
                if sub is not clocksub:return oldtake(sub)
                async def execute():
                    token=clock_probe._TAKE.set(('injected',))
                    try:await YieldOnce()
                    finally:clock_probe._TAKE.reset(token);closed.append(True)
                return execute
            executor._take_subscription=MethodType(inject,executor);svc.take=executor._take_subscription
            try:
                observed=body_message(200_000_000_000)
                checks['unexpected_suspend_fail_closed_and_unwound']=('unexpectedly suspended' in observed and closed==[True] and clock_probe._TAKE.get() is None and clocksub.callback_group.can_execute(clocksub))
            finally:executor._take_subscription=oldtake;svc.take=svctake
            msg=Odometry();msg.header.stamp.sec=200;msg.header.stamp.nanosec=0
            bodypub.publish(msg);wait_ready(bodysub)
            pending=executor._make_handler(bodysub,node,executor._take_subscription)
            executor._is_shutdown=True
            try:
                dispatch(bodysub,pending)
                checks['shutdown_no_take_original_rejection']=bodies[-1]['result']=='future_rejected'
            finally:executor._is_shutdown=False
        assert all(checks.values()),checks
        result=dict(passed=True,selected=selected,checks=checks,bodies=bodies,single_call_timings=timings,
            service=None if node._r114_clock_service is None else node._r114_clock_service.receipt(),
            real_rclpy_entities=True,actual_PID_method=True,Teacher_boundary_is_sentinel=True,
            R86_bundle_is_scalar_fixture=True,downstream_navigation_acceptance_claimed=False,
            pid=os.getpid(),module_path=service_module.__file__,helper_sha256=hashlib.sha256(Path(service_module.__file__).read_bytes()).hexdigest())
    finally:
        controller.TeacherNavigation.on_odom=original_teacher;controller._sensor_callback_odom=original_sensor
        if executor is not None:
            executor.shutdown(timeout_sec=2);executor._executor.shutdown(wait=True)
        scan_probe.detach();clock_probe.detach()
        if node is not None:Node.destroy_node(node)
        context.try_shutdown()
    if result is not None:
        result['executor_threads_joined']=not any(t.is_alive() for t in executor._executor._threads)
        result['remaining_python_threads']=[t.name for t in threading.enumerate() if t is not threading.main_thread()]
        assert result['executor_threads_joined'] and not result['remaining_python_threads']
        (out/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);parser.add_argument('--child',action='store_true');args=parser.parse_args()
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    assert os.environ.get('ROS_DOMAIN_ID')=='218'
    if args.child:return child(out)
    start=time.monotonic();results=[]
    for value in ('0','1'):
        target=out/('off' if value=='0' else 'on')
        env=dict(os.environ,R114_CLOCK_SERVICE=value,R99_CLOCK_FEEDBACK='0',R99_CLOCK_PROBE='1',R102_CLOCK_READ_INTERVAL_PROBE='1',R107_SCAN_TRANSPORT_PROBE='1',GO2_R73_MINIMAL_SIMULATION='1',PYTHONDONTWRITEBYTECODE='1')
        with (out/(value+'.log')).open('x') as log:
            process=subprocess.run([sys.executable,'-B',str(Path(__file__).resolve()),'--child','--out',str(target)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=15)
        assert process.returncode==0,(value,process.returncode)
        results.append(json.loads((target/'RESULT.json').read_text()))
    (out/'RESULT.json').write_text(json.dumps(dict(passed=all(r['passed'] for r in results),wall_s=time.monotonic()-start,serial_children=2,child_pids=[r['pid'] for r in results],both_processes_exited=True,results=results),indent=2)+'\n')


if __name__=='__main__':main()
