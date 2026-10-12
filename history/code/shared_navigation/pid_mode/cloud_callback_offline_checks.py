#!/usr/bin/env python3
"""Actual cloud callback branch parity; no ROS node, transport or sim startup."""
import argparse,collections,hashlib,importlib.util,json,math,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE))
from controller import PIDNavigation,base
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


def run_checks(output):
    archive=ROOT/'test_results/pid_navigation_v3_baseline_archive_20261004/navigation/pid_mode/teacher_wrapper.py'
    spec=importlib.util.spec_from_file_location('pid_v2_cloud_baseline',archive);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    checks=[];clock_ns=1_000_000_000
    def check(name,value):
        checks.append({'name':name,'passed':bool(value)})
        if not value:raise AssertionError(name)
    def msg(points,stamp=clock_ns,frame='camera_init'):
        return point_cloud2.create_cloud_xyz32(Header(stamp=Time(sec=stamp//1_000_000_000,nanosec=stamp%1_000_000_000),frame_id=frame),np.asarray(points,dtype=np.float32).reshape(-1,3))
    def fake(cls,history=None,stamp=-1,publish_error=False):
        f=cls.__new__(cls);rows=[];publishes=[]
        f.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=clock_ns))
        f.cloud_stamp=stamp;f.cloud_timeout=.3;f.cloud_updated=-math.inf;f.cloud=None;f.cloud_input_context=None;f.self_filtered_points=0
        f.pose_history=collections.deque([(clock_ns,np.zeros(3),np.eye(3))]if history is None else history)
        f.counts={'cloud':0};f.message='';f.run=Path('/offline/does_not_exist');f.guard_cloud_receipt=None;f.guard_cloud=None;f.cloud_callback_records=0;f.pending_cloud_callback_row=None
        f.evidence=SimpleNamespace(append=lambda p,r:rows.append((p.name,dict(r))))
        def publish(m):
            if publish_error:raise RuntimeError('offline publisher error')
            publishes.append(m)
        f.scan_cloud_pub=SimpleNamespace(publish=publish)
        return f,rows,publishes
    normal=[[2,2,0],[3,2,.5],[2,3,1]]
    cases=[('accepted',msg(normal),{},'accepted'),('duplicate',msg(normal),{'stamp':clock_ns},'rejected_duplicate_or_backward_header'),
        ('backward',msg(normal,clock_ns-1),{'stamp':clock_ns},'rejected_duplicate_or_backward_header'),
        ('stale',msg(normal,clock_ns-300_000_000),{},'rejected_header_age'),('future',msg(normal,clock_ns+50_000_001),{},'rejected_header_age'),
        ('wrong_frame',msg(normal,frame='odom'),{},'rejected_frame_id'),('empty',msg([]),{},'rejected_no_finite_points'),
        ('nan',msg([[float('nan'),0,0]]),{},'rejected_no_finite_points'),('no_pose',msg(normal),{'history':[]},'rejected_no_slam_pose_history'),
        ('pose_gap',msg(normal),{'history':[(clock_ns-150_000_001,np.zeros(3),np.eye(3))]},'rejected_no_nearby_slam_pose'),
        ('all_self_returns',msg([[0,0,.15]]),{},'rejected_all_points_self_filtered')]
    for name,message,kwargs,expected in cases:
        previous,oldrows,oldpub=fake(old.TeacherNavigation,**kwargs);current,rows,pub=fake(PIDNavigation,**kwargs)
        old.TeacherNavigation.on_cloud(previous,message);PIDNavigation.on_cloud(current,message)
        callbacks=[r for n,r in rows if n=='navigation_cloud_callbacks.jsonl'];check(name+' single callback receipt',len(callbacks)==1)
        receipt=callbacks[0];check(name+' actual branch reason',receipt['reason']==expected)
        check(name+' exact original header',receipt['producer_stamp_ns']==message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec)
        check(name+' finite JSON and original receive clock',json.dumps(receipt,allow_nan=False)and receipt['received_ros_clock_ns']==clock_ns and receipt['finished_monotonic_wall']>=receipt['received_monotonic_wall'] and receipt['callback_duration_wall_s']>=0)
        same=current.cloud_stamp==previous.cloud_stamp and current.counts==previous.counts and current.message==previous.message and len(pub)==len(oldpub)
        if previous.cloud is not None:same=same and np.array_equal(current.cloud,previous.cloud)and current.self_filtered_points==previous.self_filtered_points
        check(name+' V2 decision/payload/forwarding parity',same)
    previous,oldrows,oldpub=fake(old.TeacherNavigation,publish_error=True);current,rows,pub=fake(PIDNavigation,publish_error=True)
    errors=[]
    for f,fn in [(previous,old.TeacherNavigation.on_cloud),(current,PIDNavigation.on_cloud)]:
        try:fn(f,msg(normal))
        except RuntimeError as e:errors.append(str(e))
    check('live publisher exception still propagates',errors==['offline publisher error']*2)
    receipt=next(r for n,r in rows if n=='navigation_cloud_callbacks.jsonl');check('callback exception receipt preserves failure',receipt['reason']=='callback_exception'and receipt['accepted']is False)
    # Count the actual decoder calls; audit must never decode/retry a cloud itself.
    original=point_cloud2.read_points_numpy;calls=[]
    def counted(*args,**kwargs):calls.append(1);return original(*args,**kwargs)
    point_cloud2.read_points_numpy=counted
    try:
        current,rows,pub=fake(PIDNavigation);PIDNavigation.on_cloud(current,msg(normal));check('accepted actual cloud exactly one decoder call',len(calls)==1)
        calls.clear();current,rows,pub=fake(PIDNavigation);PIDNavigation.on_cloud(current,msg(normal,clock_ns-300_000_000));check('stale header no decode or retry',len(calls)==0)
    finally:point_cloud2.read_points_numpy=original
    # All protected/control source is byte-identical aside from explicit read-only
    # audit calls and approved V3 acceleration/profile metadata.
    import ast
    current=(HERE/'shared_controller.py').read_text();prior=(archive.parent/'shared_controller.py').read_text()
    tree=ast.parse(current)
    class RemoveAudit(ast.NodeTransformer):
        def visit_Expr(self,node):
            if isinstance(node.value,ast.Call)and isinstance(node.value.func,ast.Attribute)and node.value.func.attr=='note_cloud_decision':return None
            return self.generic_visit(node)
    check('all shared protection/control AST exactly V2 excluding read-only audit hooks',ast.dump(RemoveAudit().visit(tree),include_attributes=False)==ast.dump(ast.parse(prior),include_attributes=False))
    priorprofiles=archive.parent/'profiles'
    for p in(HERE/'profiles').glob('*.json'):
        now=json.loads(p.read_text());before=json.loads((priorprofiles/p.name).read_text());check(p.stem+' gains/TTL/fence/limits/regions unchanged',now['pid']['slew_acceleration']==[.3,.3,.25]and all(now[k]==v for k,v in before.items()if k not in ('experiment','pid'))and all(now['pid'][k]==v for k,v in before['pid'].items()if k!='slew_acceleration'))
    report={'schema':'pid_cloud_callback_offline_checks/v1','status':'passed','check_count':len(checks),'checks':checks,
        'actual_physics_started':False,'ROS_initialized':False,'message_types_constructed_only':True,
        'source_hashes':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in HERE.glob('*.py')}}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'status':'passed','checks':len(checks),'output':str(output)}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run_checks(a.output.resolve())
