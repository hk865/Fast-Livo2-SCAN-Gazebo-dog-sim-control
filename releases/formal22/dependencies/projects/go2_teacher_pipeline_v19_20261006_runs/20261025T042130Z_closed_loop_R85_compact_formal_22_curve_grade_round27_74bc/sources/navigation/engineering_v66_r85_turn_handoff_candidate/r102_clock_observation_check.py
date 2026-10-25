"""Finite no-ROS checks of the actual R99 storage/wrappers and PID gate prefix.

Explicit execution only. No rclpy import, process launch, GC policy change, or
simulation. Setup compatibility/real DDS/TimeSource remain for owner validation.
"""
import argparse
import ast
import asyncio
from collections import deque
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import statistics
import sys
import time
import tracemalloc
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent


def load(option, interval):
    keys = dict(R99_CLOCK_PROBE=option, R102_CLOCK_READ_INTERVAL_PROBE=interval,
                R99_CLOCK_FEEDBACK='0')
    prior = {k: os.environ.get(k) for k in keys}
    try:
        os.environ.update(keys)
        spec = importlib.util.spec_from_file_location('r102_check_clock', HERE/'r99_clock_probe.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for k, v in prior.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


class Node:
    def __init__(self, run):
        self.value = 0
        self.calls = 0
        self.run = run
        self.clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.value))

    def get_clock(self):
        return self.clock

    def original(self, msg):
        self.calls += 1
        self.value = msg.clock.sec*1_000_000_000 + msg.clock.nanosec
        return self.value


class Executor:
    def __init__(self):
        self.pending = None
        self.takes = 0

    def _take_subscription(self, sub):
        self.takes += 1
        if self.pending is None:
            return None
        msg, self.pending = self.pending, None

        async def execute():
            return sub.callback(msg)
        return execute


def attach(module, run):
    node, executor = Node(run), Executor()
    sub = SimpleNamespace(callback=node.original, topic_name='/clock')
    node.subscriptions = [sub]
    module._attach(node, executor, sub, {'fixture': 'no ROS setup compatibility'})
    return node, executor, sub


def gate_prefix(module):
    tree = ast.parse((HERE/'controller.py').read_text())
    cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name=='PIDNavigation')
    fn = next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name=='on_odom')
    gate = next(i for i,x in enumerate(fn.body) if isinstance(x,ast.If)
                and ast.unparse(x.test)=='stamp > clock')
    fn.body = fn.body[:gate+1] + [ast.Return(value=ast.Constant(value='continued'))]
    capture = []
    env = dict(_r99_clock_probe=module, time=time,
               _sensor_callback_odom=lambda *a,**k: capture.append((a[1:6],k)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),
                 str(HERE/'controller.py'), 'exec'),env)
    return env['on_odom'], capture


def behavior(run):
    evidence = []
    for interval in ('0','1'):
        m=load('1',interval)
        n,e,s=attach(m,run)
        async def cycle():
            for value in (1_000_000_000,1_010_000_000):
                e.pending=SimpleNamespace(clock=SimpleNamespace(sec=1,nanosec=value-1_000_000_000))
                assert await e._take_subscription(s)()==value
        asyncio.run(cycle())
        assert e.takes==2 and n.calls==2
        assert e._take_subscription(s) is None
        n.stamp=lambda msg:msg
        n.pose_stamp=900_000_000
        gate,capture=gate_prefix(m)
        assert gate(n,1_000_000_000)=='continued'
        assert gate(n,1_030_000_000) is None
        assert n.pose_stamp==900_000_000 and len(capture)==1
        assert m.PROBE.rows[-1][m.CLOCK_COLUMNS.index('original_clock_after_ns')]==1_010_000_000
        notes=[dict(zip(m.NOTE_COLUMNS,r)) for r in m.PROBE.notes]
        assert notes[-1]['original_future_predicate']
        assert notes[-1]['original_clock_read_interval_valid']==(interval=='1')
        assert list(m.PROBE.windows)==['first_future']
        assert m.PROBE.unmatched_callbacks==0
        # Original business exceptions must escape unchanged.
        def failure(msg):
            raise LookupError('original')
        try:
            m.PROBE.callback(failure,SimpleNamespace(clock=SimpleNamespace(sec=1,nanosec=0)))
        except LookupError as ex:
            assert str(ex)=='original'
        else:
            raise AssertionError('Original error was hidden')
        owner=m.PROBE.owner
        assert owner['node_clock_id']==id(n.get_clock()) and owner['pid']==os.getpid()
        m.detach()
        assert s.callback==n.original and '_take_subscription' not in vars(e)
        evidence.append(dict(interval=interval, takes=e.takes, original_calls=n.calls,
                             notes=notes, owner=owner, restored=True,
                             original_exception_propagated=True))
    for value in ('true','2',''):
        try:
            load('0',value)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid interval selection accepted')
    off=load('0','0')
    assert off.read_before(object()) is None
    off.note(object(),2,1,0,0)
    assert off.PROBE is None
    keys=('R99_CLOCK_PROBE','R102_CLOCK_READ_INTERVAL_PROBE','R99_CLOCK_FEEDBACK')
    prior={k:os.environ.get(k) for k in keys}
    try:
        os.environ.update(dict(zip(keys,('0','0','0'))))
        assert off.install(object(),object())['enabled'] is False
        for flags in (('0','1','0'),('1','1','1')):
            bad=load(flags[0],flags[1])
            os.environ.update(dict(zip(keys,flags)))
            try:
                bad.install(object(),object())
            except RuntimeError as ex:
                assert 'requires R99 probe ON and feedback wait OFF' in str(ex)
            else:
                raise AssertionError('Unsupported interval/wait selection accepted')
    finally:
        for k,v in prior.items():
            if v is None:
                os.environ.pop(k,None)
            else:
                os.environ[k]=v
    return evidence


def retained_size(value):
    """Count owned scalar/container graph only; no weakref target/node/module."""
    seen=set()
    def visit(x):
        if id(x) in seen:
            return 0
        seen.add(id(x))
        size=sys.getsizeof(x)
        if isinstance(x,dict):
            size+=sum(visit(k)+visit(v) for k,v in x.items())
        elif isinstance(x,(tuple,list,deque)):
            size+=sum(visit(v) for v in x)
        return size
    return visit(vars(value))


def worst_storage(run):
    m=load('1','1')
    tracemalloc.start()
    n,e,s=attach(m,run)
    p=m.PROBE
    msg=SimpleNamespace(clock=SimpleNamespace(sec=100,nanosec=0))
    total=3*m.CAPACITY+2*m.POST+100
    freeze_cost=[]
    for i in range(total):
        msg.clock.nanosec=i*5_000_000 % 1_000_000_000
        token=m._TAKE.set(p.take_after(p.take_before(), object()))
        try:
            p.callback(n.original,msg)
        finally:
            m._TAKE.reset(token)
        # The actual note path, never a large payload, every callback to maximize notes.
        before=m.read_before(n)
        p.note(n.value,n.value,time.monotonic_ns(),n.value-1,before)
        if i==m.CAPACITY-1:
            wall,cpu=time.monotonic_ns(),time.thread_time_ns()
            p.freeze('first_future',p.notes[-1],'bounded_storage_fixture')
            freeze_cost.append(dict(kind='first_future',thread_cpu_ns=time.thread_time_ns()-cpu,
                                    wall_ns=time.monotonic_ns()-wall))
        if i==2*m.CAPACITY+m.POST-1:
            wall,cpu=time.monotonic_ns(),time.thread_time_ns()
            m.hard_fault(n,'x'*240)
            freeze_cost.append(dict(kind='first_hard_fault',thread_cpu_ns=time.thread_time_ns()-cpu,
                                    wall_ns=time.monotonic_ns()-wall))
    current,peak=tracemalloc.get_traced_memory()
    owned=retained_size(p)
    tracemalloc.stop()
    assert set(p.windows)=={'first_future','first_hard_fault'}
    assert all(len(w['rows'])==m.CAPACITY and len(w['post_rows'])==m.POST
               and len(w['notes'])==m.NOTE_CAPACITY and len(w['post_notes'])==m.NOTE_POST
               for w in p.windows.values())
    assert peak<=4*1024**2 and owned<=4*1024**2
    status=p.status()
    assert status['dropped']==status['note_dropped']==status['unmatched_callbacks']==0
    wall,cpu=time.monotonic_ns(),time.thread_time_ns()
    flushed=m.finish(n)
    flush_cpu=time.thread_time_ns()-cpu
    flush_wall=time.monotonic_ns()-wall
    assert flushed['flushed']
    assert Path(flushed['path']).stat().st_size<=20*1024**2
    d=json.loads(Path(flushed['path']).read_text())
    assert len(d['windows'])==2 and len(d['rows'])==m.CAPACITY
    # Deliberately lower only this fixture's file bound to exercise a bounded partial.
    small=run/'limit_fixture'
    small.mkdir()
    m2=load('1','1');n2,_,_=attach(m2,small)
    m2.MAX_OUTPUT_BYTES=200
    limited=m2.finish(n2)
    assert not limited['flushed'] and limited['error']=='OverflowError'
    assert not (small/'r99_clock_timeline.json').exists()
    assert (small/'r99_clock_timeline.json.partial').stat().st_size<=200
    return dict(clock_capacity=m.CAPACITY,note_capacity=m.NOTE_CAPACITY,
                clock_post=m.POST,note_post=m.NOTE_POST,trace_current_bytes=current,
                trace_peak_bytes=peak,owned_graph_bytes=owned,fill_callbacks=total,
                two_disjoint_freezes=True,recording=status,flush=flushed,
                freeze_cost_with_tracemalloc=freeze_cost,
                flush_wall_ns=flush_wall,flush_thread_cpu_ns=flush_cpu,
                output_cap_failure=limited,measurement_includes='recorder attach/fill/freeze; excludes flush JSON and tracemalloc internal memory')


def binding_scope():
    """Execute the actual binding loops with real local module bytes, no Node."""
    source=HERE/'r86_runtime_binding.py'
    fn=next(x for x in ast.parse(source.read_text()).body
            if isinstance(x,ast.FunctionDef) and x.name=='write_binding')
    loops=[x for x in fn.body if isinstance(x,ast.For) and
           ast.unparse(x.iter) in ('sorted(sys.modules)','paths.items()')]
    assert len(loops)==2
    code=compile(ast.fix_missing_locations(ast.Module(body=loops,type_ignores=[])),str(source),'exec')
    spec=importlib.util.spec_from_file_location('r102_clock_observation_check',Path(__file__))
    actual=importlib.util.module_from_spec(spec);spec.loader.exec_module(actual)
    m=load('1','1')
    modules={'r99_clock_probe':m,'r102_clock_observation_check':actual}
    refs={str(HERE/(n+'.py')):hashlib.sha256((HERE/(n+'.py')).read_bytes()).hexdigest()
          for n in modules}
    def verify(modules,refs):
        env=dict(sys=SimpleNamespace(modules=modules),here=HERE,Path=Path,hashlib=hashlib,
                 references=refs,records={},paths={'r99_clock_probe':HERE/'r99_clock_probe.py'})
        exec(code,env)
        return env['records']
    records=verify(modules,refs)
    assert set(records)==set(modules)
    bad=dict(modules)
    bad['r102_clock_observation_check']=SimpleNamespace(__file__=str(HERE.parent/'bind_r102_worktree.py'))
    try:
        verify(bad,refs)
    except RuntimeError as ex:
        assert 'actual imported module path differs' in str(ex)
    else:
        raise AssertionError('Foreign module path accepted')
    badrefs=dict(refs);badrefs[str(HERE/'r102_clock_observation_check.py')]='0'*64
    try:
        verify(modules,badrefs)
    except RuntimeError as ex:
        assert 'absent from frozen scope' in str(ex)
    else:
        raise AssertionError('Changed module hash accepted')
    return dict(actual_source_and_scope_loops=True,modules=records,
                foreign_path_rejected=True,wrong_scope_hash_rejected=True,
                full_write_binding_not_exercised=True)


def cost(run):
    modules={key:load(*flags) for key,flags in
             [('OFF',('0','0')),('R99_ONLY',('1','0')),('R99_INTERVAL',('1','1'))]}
    nodes={};executors={};subscriptions={}
    for key,m in modules.items():
        if key=='OFF':
            nodes[key]=Node(run)
            executors[key]=Executor()
            subscriptions[key]=SimpleNamespace(callback=nodes[key].original)
        else:
            nodes[key],executors[key],subscriptions[key]=attach(m,run)
    trials=[]
    calls=2000
    for pair in range(6):
        order=['OFF','R99_ONLY','R99_INTERVAL']
        if pair%2:
            order.reverse()
        for key in order:
            m,n=modules[key],nodes[key]
            n.value=1_000_000_000
            fn,_=gate_prefix(m)
            n.stamp=lambda msg:msg
            n.pose_stamp=900_000_000
            for _ in range(100):
                fn(n,1_000_000_000)
            wall,cpu=time.monotonic_ns(),time.thread_time_ns()
            for _ in range(calls):
                assert fn(n,1_000_000_000)=='continued'
            dc=time.thread_time_ns()-cpu;dw=time.monotonic_ns()-wall
            trials.append(dict(pair=pair,variant=key,scope='actual_PID_gate_prefix',
                               wall_per_call_ns=dw/calls,cpu_per_call_ns=dc/calls))
    msg=SimpleNamespace(clock=SimpleNamespace(sec=1,nanosec=0))
    async def callback_trials():
        for pair in range(6):
            order=['OFF','R99_ONLY','R99_INTERVAL']
            if pair%2:
                order.reverse()
            for key in order:
                e,s=executors[key],subscriptions[key]
                for _ in range(100):
                    e.pending=msg
                    await e._take_subscription(s)()
                wall,cpu=time.monotonic_ns(),time.thread_time_ns()
                for _ in range(calls):
                    e.pending=msg
                    assert await e._take_subscription(s)()==1_000_000_000
                dc=time.thread_time_ns()-cpu;dw=time.monotonic_ns()-wall
                trials.append(dict(pair=pair,variant=key,scope='actual_R99_take_execute_callback_wrappers',
                                   wall_per_call_ns=dw/calls,cpu_per_call_ns=dc/calls))
    asyncio.run(callback_trials())
    summaries={scope:{key:{field:statistics.median(r[field] for r in trials
                         if r['variant']==key and r['scope']==scope)
                    for field in ('wall_per_call_ns','cpu_per_call_ns')} for key in modules}
               for scope in ('actual_PID_gate_prefix','actual_R99_take_execute_callback_wrappers')}
    for m in modules.values():
        m.detach()
    return dict(calls_per_variant_pair=calls,pairs=6,warmup=100,trials=trials,
                medians=summaries,limits=['Fake original callback/executor exercise the actual observer wrappers; no ROS/DDS/take scheduling or control math',
                    'GC policy unchanged; no production tail bound inferred',
                    'OFF includes original gate clock read and disabled note function'])


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    run=args.output/'storage_fixture'
    run.mkdir()
    result=dict(schema='r102_clock_observation_offline_check/v1',
                python=sys.version,real_ros_started=False,simulation_started=False,
                gc_policy_changed=False,source={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (HERE/'controller.py',HERE/'r99_clock_probe.py',HERE/'r86_runtime_binding.py',Path(__file__))})
    result['behavior']=behavior(run)
    result['binding_scope']=binding_scope()
    result['storage']=worst_storage(run)
    result['cost']=cost(run)
    result['passed']=True
    target=args.output/'OFFLINE_RESULT.json'
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=True,result=str(target),storage_bytes=result['storage']['trace_peak_bytes'],
                         output_bytes=result['storage']['flush']['output_bytes'],cost=result['cost']['medians'])))


if __name__=='__main__':
    main()
