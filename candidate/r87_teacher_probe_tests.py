"""Offline checks against the actual pre-probe AST; no ROS or physical launch."""
import ast
import copy
from contextlib import contextmanager
from pathlib import Path
import math
import subprocess
from types import SimpleNamespace as NS
import unittest

HERE=Path(__file__).resolve().parent
BASELINE='ebec30cae924e1fd4487709ffe9dd4396277cec6'


def original(filename):
    return subprocess.check_output(['git','show',BASELINE+':candidate/'+filename],cwd=HERE,text=True)


def method(source,cls,name):
    parent=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name==cls)
    return copy.deepcopy(next(n for n in parent.body if isinstance(n,ast.FunctionDef) and n.name==name))


def diagnostic_call(node):
    return (isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)
        and isinstance(node.func.value,ast.Name) and node.func.value.id=='_r87_diag')


class WithoutDiagnostics(ast.NodeTransformer):
    def visit_With(self,node):
        node=self.generic_visit(node)
        if len(node.items)==1 and diagnostic_call(node.items[0].context_expr):return node.body
        return node
    def visit_Expr(self,node):
        return None if diagnostic_call(node.value) else self.generic_visit(node)
    def visit_FunctionDef(self,node):
        node=self.generic_visit(node)
        node.decorator_list=[d for d in node.decorator_list if not diagnostic_call(d)]
        return node


def same_ast(actual, expected, label):
    if ast.dump(actual,include_attributes=False)!=ast.dump(expected,include_attributes=False):
        raise AssertionError('Unexpected R88 AST at '+label)


def statement(text):
    return ast.parse(text).body[0]


class NormalizeR88(ast.NodeTransformer):
    """Prove the exact added branches before selecting the default-off path.

    This is deliberately not a generic constant folder: any changed branch,
    argument, extra side effect or missing wrapper must fail the comparison.
    Async-enabled behavior is covered by r88_admission_tests separately.
    """
    START="""if asynchronous is not None and physical_result is None:
    return asynchronous.start(msg,metadata,received_wall_ns=received_wall_ns,admission_started=started)
"""
    PHYSICAL="""if physical_result is None:
    physical=self.r85_physical_gate.certify(self,value,metadata,binding)
elif physical_result.get('error'):
    physical=dict(schema='r85_actual_scan_physical_execution_gate/v1',allowed=False,
        reason='r88_physical_worker_rejected',error=physical_result['error'])
else:
    physical=self.r85_physical_gate.complete(self,value,metadata,binding,
        physical_result['prepared'],physical_result['proof'])
"""
    def __init__(self):
        self.counts=dict(started=0,owner=0,start=0,physical=0,receipt=0)

    def admission(self,node):
        expected=statement('def _admit_matched_spline(self,msg,metadata,received_wall_ns=None,*,admission_started=None,physical_result=None): pass')
        same_ast(node.args,expected.args,'admission optional defaults')
        node.args.kwonlyargs=[];node.args.kw_defaults=[]
        result=self.visit(node)
        if self.counts!=dict(started=1,owner=1,start=1,physical=1,receipt=1):
            raise AssertionError('R88 normalization coverage mismatch: '+str(self.counts))
        return result

    def visit_Assign(self,node):
        names=[target.id for target in node.targets if isinstance(target,ast.Name)]
        if names==['started']:
            same_ast(node,statement('started=time.monotonic() if admission_started is None else admission_started'),
                     'original/default admission clock')
            self.counts['started']+=1
            return statement('started=time.monotonic()')
        if names==['asynchronous']:
            same_ast(node,statement("asynchronous=getattr(self,'_r88_admission',None)"),'default-off owner')
            self.counts['owner']+=1
            return None
        return self.generic_visit(node)

    def visit_If(self,node):
        names={value.id for value in ast.walk(node.test) if isinstance(value,ast.Name)}
        if 'asynchronous' in names:
            same_ast(node,statement(self.START),'async submission branch')
            self.counts['start']+=1
            return None  # asynchronous is None in the original disabled route.
        if 'physical_result' in names:
            same_ast(node,statement(self.PHYSICAL),'physical proof dispatch')
            self.counts['physical']+=1
            return node.body  # physical_result is None uses the exact old call.
        return self.generic_visit(node)

    def visit_Call(self,node):
        if isinstance(node.func,ast.Name) and node.func.id=='defer_future_path':
            expected=statement('defer_future_path(self,msg,metadata,started=started,received_wall_ns=received_wall_ns)').value
            same_ast(node,expected,'explicit original-receipt forwarding')
            self.counts['receipt']+=1
            node.keywords=[item for item in node.keywords if item.arg!='received_wall_ns']
        return self.generic_visit(node)

    @staticmethod
    def protocol(node,role):
        expected=statement(f"""def {node.name}(self,msg):
    asynchronous=getattr(self,'_r88_admission',None)
    if asynchronous is not None:
        return asynchronous.receive({role!r},msg,super().{node.name})
    return super().{node.name}(msg)
""")
        same_ast(node,expected,'protocol '+node.name)
        node.body=node.body[-1:]
        return node


class Probe:
    def __init__(self,enabled):self.enabled=enabled;self.rows=[]
    def mark(self,stage,**fields):
        if self.enabled:self.rows.append(dict(stage=stage,**fields))
    def anomaly(self,reason,**fields):
        if self.enabled:self.rows.append(dict(reason=reason,**fields))
    @contextmanager
    def span(self,*args,**kwargs):yield
    def observe(self,stage):return lambda fn:fn


class BaseReached(Exception):pass


def isolated_teacher(source,enabled=True):
    odom=method(source,'TeacherNavigation','on_odom')
    cls=ast.ClassDef(name='TeacherNavigation',bases=[ast.Name(id='Base',ctx=ast.Load())],
        keywords=[],body=[odom],decorator_list=[])
    class Base:
        def on_odom(self,msg):
            self.base_calls+=1
            raise BaseReached
    probe=Probe(enabled);calls=dict(clock=0,now=0,wall=0,stamp=0,finite=[])
    def finite(value):calls['finite'].append(value);return math.isfinite(value)
    def wall():calls['wall']+=1;return 100.0
    scope=dict(Base=Base,_r87_diag=probe,math=NS(isfinite=finite),time=NS(monotonic=wall))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'<actual-teacher-odom>','exec'),scope)
    node=scope['TeacherNavigation']();node.base_calls=0;node.pose_stamp=650_000_000;node.pose_timeout=.3
    def now():calls['now']+=1;return NS(nanoseconds=1_000_000_000)
    def get_clock():calls['clock']+=1;return NS(now=now)
    def stamp(msg):calls['stamp']+=1;return msg.header.source_ns
    node.get_clock=get_clock;node.stamp=stamp
    return node,probe,calls


def message():
    return NS(header=NS(frame_id='camera_init',source_ns=900_000_000),child_frame_id='demo_slam_body',
        pose=NS(pose=NS(position=NS(x=1.,y=2.,z=3.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),
        twist=NS(twist=NS(linear=NS(x=.1,y=.2,z=.3),angular=NS(x=.01,y=.02,z=.03))))


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.teacher_old=original('teacher_wrapper.py');cls.teacher_new=(HERE/'teacher_wrapper.py').read_text()
        cls.controller_old=original('controller.py');cls.controller_new=(HERE/'controller.py').read_text()

    def test_scan_admission_ast_identical_after_removing_endpoints(self):
        old=method(self.controller_old,'PIDNavigation','_admit_matched_spline')
        new=WithoutDiagnostics().visit(method(self.controller_new,'PIDNavigation','_admit_matched_spline'))
        new=NormalizeR88().admission(new)
        self.assertEqual(ast.dump(old,include_attributes=False),ast.dump(new,include_attributes=False))

    def test_r88_original_receipt_forwarding_is_explicit_and_preserves_default(self):
        # This intentional R88 change is asserted independently, never hidden
        # inside the admission equivalence stripper above.
        old=next(n for n in ast.parse(original('pending_path_commit.py')).body
                 if isinstance(n,ast.FunctionDef) and n.name=='defer_future')
        new=next(n for n in ast.parse((HERE/'pending_path_commit.py').read_text()).body
                 if isinstance(n,ast.FunctionDef) and n.name=='defer_future')
        same_ast(new.args,statement('def defer_future(node,msg,metadata,*,started=None,received_wall_ns=None): pass').args,
                 'pending receipt argument/default')
        expressions=[n for n in ast.walk(new) if isinstance(n,ast.keyword) and n.arg=='received_wall_ns']
        self.assertEqual(len(expressions),1)
        selected=expressions[0].value
        expected=ast.parse('time.monotonic_ns() if received_wall_ns is None else received_wall_ns',mode='eval').body
        same_ast(selected,expected,'pending original receipt selection')
        for received,expected_value,expected_calls in ((None,42,1),(123,123,0),(0,0,0)):
            calls=[]
            def now():calls.append(True);return 42
            value=eval(compile(ast.Expression(body=copy.deepcopy(selected)),'<receipt-expression>','eval'),
                       dict(time=NS(monotonic_ns=now),received_wall_ns=received))
            self.assertEqual((value,len(calls)),(expected_value,expected_calls))
        new.args=copy.deepcopy(old.args)
        expressions[0].value=ast.parse('time.monotonic_ns()',mode='eval').body
        self.assertEqual(ast.dump(old,include_attributes=False),ast.dump(new,include_attributes=False))

    def test_r88_normalizer_rejects_changed_dispatch_or_budget(self):
        for before,after in (
                ('admission_started=started)','admission_started=time.monotonic())'),
                ('if admission_started is None else admission_started','if admission_started is None else 0'),
                ("reason='r88_physical_worker_rejected'","reason='ignored_worker_error'"),
                ('started=started,received_wall_ns=received_wall_ns):return',
                 'started=started,received_wall_ns=time.monotonic_ns()):return')):
            self.assertIn(before,self.controller_new)
            mutated=self.controller_new.replace(before,after,1)
            node=WithoutDiagnostics().visit(method(mutated,'PIDNavigation','_admit_matched_spline'))
            with self.assertRaises(AssertionError,msg=before):NormalizeR88().admission(node)

    def test_native_guard_ast_identical_after_removing_endpoints(self):
        old=method(self.teacher_old,'TeacherNavigation','record_native_guard')
        new=WithoutDiagnostics().visit(method(self.teacher_new,'TeacherNavigation','record_native_guard'))
        self.assertEqual(ast.dump(old,include_attributes=False),ast.dump(new,include_attributes=False))

    def test_teacher_predicates_and_order_are_the_original_six(self):
        old=method(self.teacher_old,'TeacherNavigation','on_odom')
        gate=next(n for n in old.body if isinstance(n,ast.If))
        new=method(self.teacher_new,'TeacherNavigation','on_odom')
        branch=next(n for n in new.body if isinstance(n,ast.If));predicates=[]
        while branch:
            predicates.append(branch.test)
            branch=branch.orelse[0] if branch.orelse else None
        self.assertEqual([ast.dump(n) for n in gate.test.values],[ast.dump(n) for n in predicates])

    def test_actual_teacher_outcomes_clocks_and_short_circuit_on_off(self):
        cases=[('accepted',{},None),('age_exact_timeout',{'stamp':700_000_000},'header_age'),
            ('age_inside_timeout',{'stamp':700_000_001},None),('age_older',{'stamp':699_999_999},'header_age'),
            ('future_at_lower_boundary',{'stamp':1_050_000_000},None),
            ('future_beyond_lower_boundary',{'stamp':1_050_000_001},'header_age'),
            ('equal',{'stamp':650_000_000},'source_not_advancing'),
            ('backward',{'stamp':649_999_999},'source_not_advancing'),
            ('frame',{'frame':'other','stamp':700_000_000,'quat':float('nan')},'frame_id'),
            ('child',{'child':'other','stamp':700_000_000,'quat':float('nan')},'child_frame_id'),
            ('quaternion',{'quat':2.},'quaternion_norm'),
            ('quaternion_nan',{'quat':float('nan')},'quaternion_norm'),
            ('position_nan',{'position':float('nan')},'nonfinite_pose_or_twist'),
            ('twist_inf',{'twist':float('inf')},'nonfinite_pose_or_twist')]
        for name,changes,leaf in cases:
            observations=[]
            for source,enabled in ((self.teacher_old,False),(self.teacher_new,False),(self.teacher_new,True)):
                node,probe,calls=isolated_teacher(source,enabled);msg=message()
                if 'stamp' in changes:msg.header.source_ns=changes['stamp']
                if 'frame' in changes:msg.header.frame_id=changes['frame']
                if 'child' in changes:msg.child_frame_id=changes['child']
                if 'quat' in changes:msg.pose.pose.orientation.w=changes['quat']
                if 'position' in changes:msg.pose.pose.position.x=changes['position']
                if 'twist' in changes:msg.twist.twist.angular.z=changes['twist']
                try:node.on_odom(msg);outcome='reject'
                except BaseReached:outcome='accepted_to_shared'
                observations.append((outcome,node.base_calls,dict(calls,finite=[repr(v) for v in calls['finite']])))
                self.assertEqual((calls['clock'],calls['now'],calls['wall'],calls['stamp']),(1,1,1,1),name)
                if source is self.teacher_new and enabled:
                    decision=next(r for r in probe.rows if r.get('stage')=='teacher.odom.decision')
                    self.assertEqual(decision['rejection'],leaf,name)
                    self.assertEqual(decision['decision_ros_ns'],1_000_000_000,name)
                    self.assertEqual(decision['header_age_ns'],1_000_000_000-msg.header.source_ns,name)
            self.assertEqual(observations[0],observations[1],name)
            self.assertEqual(observations[0],observations[2],name)

    def test_teacher_accepted_tail_is_unchanged(self):
        old=method(self.teacher_old,'TeacherNavigation','on_odom')
        new=WithoutDiagnostics().visit(method(self.teacher_new,'TeacherNavigation','on_odom'))
        def after_shared(function):
            index=next(i for i,n in enumerate(function.body) if isinstance(n,ast.Expr)
                and isinstance(n.value,ast.Call) and isinstance(n.value.func,ast.Attribute)
                and n.value.func.attr=='on_odom')
            return [ast.dump(n) for n in function.body[index+1:]]
        self.assertEqual(after_shared(old),after_shared(new))

    def test_protocol_overrides_only_delegate_once(self):
        for name,role in (('on_trajectory_metadata','metadata'),('on_spline','spline'),('on_reference_attempt','attempt')):
            new=WithoutDiagnostics().visit(method(self.controller_new,'PIDNavigation',name))
            new=NormalizeR88.protocol(new,role)
            self.assertEqual(len(new.body),1)
            self.assertIsInstance(new.body[0],ast.Return)
            call=new.body[0].value
            self.assertEqual(call.func.attr,name)
            self.assertEqual([a.id for a in call.args],['msg'])


if __name__=='__main__':unittest.main()
