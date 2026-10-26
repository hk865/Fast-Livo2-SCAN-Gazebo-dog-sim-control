"""Real terminal owner -> R86 intake -> publication -> typed Student ACK -> T0.

No node, model or physics is started. The existing accepted region/ROS/native
boundaries remain the explicit fixtures; production control methods run intact.
"""
import ast,copy,json,os,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
os.environ['GO2_R73_MINIMAL_SIMULATION']='1'
# Its two unused legacy-baseline comparisons refer to Git objects absent from
# the published clone. Load all real fixture/control methods, excluding only
# those top-level baseline fetches; this check does not run baseline comparisons.
# External historical fixture is explicit; its old local path is not published.
# Missing fixture/profile is a test prerequisite failure, never synthetic PASS.
SOURCE_RUN=Path(os.environ['PARKING_TERMINAL_SOURCE_RUN']).resolve()
PROFILE_PATH=Path(os.environ['PARKING_TERMINAL_PROFILE']).resolve()
fixture_path=Path(__file__).with_name('r105_terminal_parking_owner_checks.py')
tree=ast.parse(fixture_path.read_text())
excluded={n.lineno for n in tree.body if isinstance(n,ast.Assign)
          and any(isinstance(t,ast.Name)and t.id in ('BASE','OLDZERO')for t in n.targets)}
tree.body=[n for n in tree.body if n.lineno not in excluded]
# Reuse an available archived final-region receipt; source/Actor completion
# remains an explicit synthetic boundary, never a physics replay or PASS repair.
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='RUN' for t in node.targets):
        node.value=ast.parse('Path('+repr(str(SOURCE_RUN))+')',mode='eval').body
flow=types.ModuleType('_pose10hz_terminal_fixture');flow.__file__=str(fixture_path)
exec(compile(ast.fix_missing_locations(tree),str(fixture_path),'exec'),flow.__dict__)
from parking_pose10hz import CONFIG
from parking_contract import SPEC
from r86_prepared_feedback import check_negative_parking

class TerminalFlowCheck(unittest.TestCase):
    def test_actual_terminal_control_publication_and_Student_ACK_make_one_fixed_T0(self):
        # Explicit matching published ACK identity; archived region facts may
        # use an earlier contract hash. Actor completion remains synthetic.
        profile_path=PROFILE_PATH
        profile=json.loads(profile_path.read_text())
        profile['ordinary_terminal_parking_contract']=copy.deepcopy(SPEC)
        profile['cascade']['parking_pose10hz']=copy.deepcopy(CONFIG)
        original=flow.inputs
        def inputs(*args,**kwargs):
            f,i,a,clock,wall=original(*args,**kwargs)
            # PIDNavigation.on_odom's actual accepted feedback shape/clocks.
            f['callback_ros_clock_ns']=clock
            return f,i,a,clock,wall
        pid_class=flow.CorrectionOnlyPID
        def pid(*args,**kwargs):return pid_class(*args,**kwargs,speed_tier=profile['r109_speed_tier'])
        with patch.object(flow,'PROFILE',profile),patch.object(flow,'inputs',inputs),\
             patch.object(flow,'CONFIG',profile['cascade']['r85_student_lower_loop_ack']),\
             patch.object(flow,'CorrectionOnlyPID',pid),tempfile.TemporaryDirectory()as folder:
            n=flow.NodeFixture(folder);n.finish_region()
            self.assertEqual(n.state,'succeeded')
            base_update=n.update
            def update(*args,**kwargs):
                base_update(*args,**kwargs)
                # The real queue hook runs before the terminal control tick.
                check_negative_parking(n)
            n.update=update
            for k in range(1,23):n.step(flow.START+k*50_000_000,k+1)
            owner=n.terminal_parking_owner
            self.assertTrue(owner.pose10hz);self.assertEqual(owner.stage,'zero_hold',
                dict(owner=owner.evidence(),reason=n.cascade.protection_reason,row=n.cascade.row,
                     capture_ready=owner.capture_grid.ready,capture_since=owner.capture_grid.since))
            self.assertIsNone(owner.first_actor_zero_ack)
            source=json.loads((n.run/'cascade_command_source.json').read_text())
            self.assertEqual(source['command_after_slew'],[0.,0.,0.])
            # Reuses real CommandReader / StudentRuntime observation / ACK validators.
            anchor=flow.Checks().qualify_node(n)
            self.assertEqual(anchor['proof']['parking_rule'],SPEC['schema'])
            self.assertEqual(anchor['end_stamp_ns']-anchor['start_stamp_ns'],5_000_000_000)
            first=copy.deepcopy(owner.first_actor_zero_ack);window=owner.pose_grid
            for k in range(1,102):
                src=json.loads((n.run/'cascade_command_source.json').read_text())
                stamp=n.pose_stamp+20_000_000;wall=flow.WALL+(stamp-flow.START)
                envelope=dict(schema_version=1,mode='scan_slam',source='scan_slam',acceptance=dict(sha256='a'*64),
                    sequence=k+1,command=[0.,0.,0.],sim_time=stamp/1e9,monotonic_wall=wall/1e9,healthy=True,
                    controller_source=src,ordinary_terminal_zero_event=src['ordinary_terminal_zero_event'])
                reader=flow.CommandReader(n.run.name,'a'*64,[.6,.2,.6])
                _,rejected,_=reader.read(json.dumps(envelope).encode(),stamp,wall/1e9)
                self.assertFalse(rejected)
                ack,_,_=flow.student(stamp,102+k,reader,envelope)
                n.step(n.pose_stamp+50_000_000,102+k,ack=ack)
                if n.cascade.failure_latched:break
            self.assertIs(owner.pose_grid,window)
            self.assertEqual(owner.first_actor_zero_ack,first)
            self.assertTrue(window.evidence()['passed'])
            self.assertEqual(len(window.samples),51)
            self.assertEqual(n.state,'succeeded')

if __name__=='__main__':unittest.main()
