# Publication 2026-10-13: local paths redacted; numeric/control behavior retained; local admission is not transferred.
"""CPU-only protocol and evidence tests; no Torch/Gazebo/ROS/GPU/native motion."""
import ast
import importlib.util
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

from ipc_evidence import BoundedHistory, FrameTiming, TimedConnection


POLICY=Path(__file__).resolve().parents[2]/'policy/worker.py'
if not POLICY.exists():
    POLICY=Path('./multifloor_demo/teacher_mode/policy/worker.py')
tree=ast.parse(POLICY.read_text())
node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='recv_exact')
env={};exec(compile(ast.Module(body=[node],type_ignores=[]),str(POLICY),'exec'),env)
recv_exact=env['recv_exact']


class WriterTests(unittest.TestCase):
    def test_exact_written_payload_and_limit_receipt(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'h.jsonl';w=BoundedHistory(p,maximum_file_bytes=8)
            self.assertTrue(w.submit(b'1234567\n',sequence=17))
            self.assertFalse(w.submit(b'x\n',sequence=18))
            r=w.close();self.assertEqual(p.read_bytes(),b'1234567\n')
            self.assertEqual((r['attempted'],r['written'],r['omitted']),(2,1,1))
            self.assertTrue(r['byte_limit_reached']);self.assertFalse(r['capture_complete'])
            self.assertEqual(r['last_attempt_sequence'],18)

    def test_pending_bounds_include_blocked_inflight(self):
        with tempfile.TemporaryDirectory() as td:
            w=BoundedHistory(Path(td)/'h',maximum_file_bytes=1024,
                maximum_queue_bytes=6,maximum_queue_records=1)
            gate=threading.Event();entered=threading.Event();original=w.stream
            class Blocked:
                def write(self,b):entered.set();gate.wait(2);return original.write(b)
                def flush(self):original.flush()
                def close(self):original.close()
            w.stream=Blocked();self.assertTrue(w.submit(b'12345\n'))
            self.assertTrue(entered.wait(1));start=time.monotonic()
            self.assertFalse(w.submit(b'x\n'));self.assertLess(time.monotonic()-start,.05)
            self.assertEqual(w.snapshot()['pending_records'],1)
            gate.set();r=w.close();self.assertTrue(r['queue_limit_reached'])
            self.assertLessEqual(r['maximum_pending_bytes'],6)

    def test_writer_failure_is_not_capture_success(self):
        with tempfile.TemporaryDirectory() as td:
            w=BoundedHistory(Path(td)/'h',maximum_file_bytes=1024);original=w.stream
            class Failed:
                def write(self,b):raise OSError('injected disk failure')
                def flush(self):pass
                def close(self):original.close()
            w.stream=Failed();w.submit(b'evidence\n')
            r=w.close();self.assertTrue(r['writer_io_failed'])
            self.assertEqual(r['written'],0);self.assertEqual(r['omitted'],1)
            self.assertFalse(r['capture_complete'])

    def test_oversized_record_explicitly_omitted(self):
        with tempfile.TemporaryDirectory() as td:
            w=BoundedHistory(Path(td)/'h',maximum_file_bytes=1024,maximum_record_bytes=4)
            self.assertFalse(w.submit(b'12345'));r=w.close()
            self.assertEqual(r['oversize_records'],1);self.assertEqual(r['omitted'],1)


class ProtocolTests(unittest.TestCase):
    def run_case(self, sender, *, timeout=.5, send_response=True):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'timing';w=BoundedHistory(p,maximum_file_bytes=64*1024);timer=FrameTiming(w)
            raw,peer=socket.socketpair();raw.settimeout(timeout)
            conn=TimedConnection(raw,timer);thread=threading.Thread(target=sender,args=(peer,));thread.start()
            timer.begin_receive();error=None;received=None
            try:
                with timer.measure('state_read'):received=recv_exact(conn,512)
                timer.received(1_000_000_000)
                if send_response:conn.sendall(b'R'*128)
                else:timer.finish('diagnostic_read_only')
            except BaseException as exc:
                error=exc;timer.finish('state_read_failed',exc)
            finally:
                raw.close();thread.join(2);peer.close();receipt=w.close()
            self.assertFalse(thread.is_alive());self.assertTrue(receipt['capture_complete'])
            rows=[json.loads(line) for line in p.read_text().splitlines()]
            return received,error,rows

    def test_fragmented_frame_exact_and_response_unchanged(self):
        replies=[]
        def send(peer):
            peer.sendall(b'A'*97);time.sleep(.01);peer.sendall(b'A'*415)
            replies.append(recv_exact(peer,128))
        received,error,rows=self.run_case(send)
        self.assertEqual(received,b'A'*512);self.assertIsNone(error)
        self.assertEqual(replies,[b'R'*128]);self.assertTrue(rows[0]['response_sent'])
        self.assertGreater(rows[0]['stages_ns']['state_read'],5_000_000)

    def test_partial_disconnect_not_fabricated_complete_frame(self):
        def send(peer):peer.sendall(b'A'*192);peer.close()
        received,error,rows=self.run_case(send)
        self.assertIsNone(received);self.assertIsInstance(error,EOFError)
        self.assertFalse(rows[0]['response_sent']);self.assertIn('EOFError',rows[0]['error'])

    def test_python_read_timeout_recorded_without_new_deadline(self):
        def send(peer):time.sleep(.08)
        received,error,rows=self.run_case(send,timeout=.03)
        self.assertIsNone(received);self.assertIsInstance(error,TimeoutError)
        self.assertEqual(rows[0]['native_deadline_ms'],200)
        self.assertFalse(rows[0]['native_parking_verified'])

    def test_delay_over_native_budget_visible_not_claimed_parking(self):
        def send(peer):time.sleep(.22);peer.sendall(b'A'*512)
        _,error,rows=self.run_case(send,send_response=False)
        self.assertIsNone(error);self.assertGreater(rows[0]['stages_ns']['state_read'],200_000_000)
        # The observer deliberately does not implement/relax the native watchdog.
        self.assertFalse(rows[0]['native_parking_verified'])

    def test_broken_pipe_raises_and_records_send_failure(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'timing';w=BoundedHistory(p,maximum_file_bytes=64*1024);timer=FrameTiming(w)
            raw,peer=socket.socketpair();conn=TimedConnection(raw,timer)
            timer.begin_receive();timer.received(1_000_000_000);peer.close()
            with self.assertRaises(BrokenPipeError):conn.sendall(b'R'*128)
            raw.close();w.close();row=json.loads(p.read_text())
            self.assertEqual(row['outcome'],'response_send_failed');self.assertFalse(row['response_sent'])
            self.assertEqual(row['stage_errors']['response_send'],1)

    def test_cycle_and_read_interval_include_after_response_delay(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'timing';w=BoundedHistory(p,maximum_file_bytes=64*1024);timer=FrameTiming(w)
            timer.begin_receive();timer.received(1);timer.finish('response_sent',response_sent=True)
            time.sleep(.01) # models work between responses and the next recv call
            timer.begin_receive();timer.received(2);timer.finish('response_sent',response_sent=True)
            w.close();rows=[json.loads(x) for x in p.read_text().splitlines()]
            self.assertGreater(rows[1]['state_read_interval_ns'],5_000_000)
            self.assertGreater(rows[1]['after_previous_send_to_receive_start_ns'],5_000_000)
            self.assertGreater(rows[1]['previous_send_to_end_ns'],5_000_000)


class WorkerIntegration(unittest.TestCase):
    def test_live_ack_and_every_telemetry_row_preserved_with_stale_rejection(self):
        # Real wrapper + Unix socket, stub policy only. No Actor/physics acceptance claim.
        import hashlib
        import numpy as np
        worker=Path(__file__).with_name('worker.py')
        if not worker.exists():worker=Path(__file__).with_name('worker.after.py')
        spec=importlib.util.spec_from_file_location('ipc_wrapper_under_test',worker)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        fake='''import json, socket\nfrom pathlib import Path\nimport numpy as np\ndef atomic_json(p,obj):\n p.write_text(json.dumps(obj))\ndef recv_exact(conn,n):\n b=b''\n while len(b)<n:\n  x=conn.recv(n-len(b))\n  if not x:raise EOFError('disconnected')\n  b+=x\n return b\ndef load_actor():return lambda obs: np.zeros(12)\ndef build_observation(*args,**kwargs):return np.zeros(247),{}\ndef requested(*args):return np.zeros(3),'stop'\ndef main():\n import sys\n run=Path(sys.argv[sys.argv.index('--run')+1]);name=sys.argv[sys.argv.index('--socket')+1]\n atomic_json(run/'policy_manifest.json',{})\n server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(name);server.listen(1)\n actor=load_actor()\n stream=(run/'telemetry.jsonl').open('w',buffering=1)\n try:\n  conn,_=server.accept()\n  with conn:\n   for index in range(2):\n    state=np.frombuffer(recv_exact(conn,512),dtype='<f8')\n    cmd,phase=requested('forward',index*.02);obs,extra=build_observation(state,cmd)\n    actor(obs)\n    row={'sim_time':index*.02,'world_sim_time':float(state[0]),'command':cmd.tolist(),'requested':cmd.tolist(),'command_expired':False,'body_clearance':.3,'contacts':{'body':0,'FR':1,'FL':1,'RR':1,'RL':1},'fault':None,'actor_inferred_this_frame':True}\n    stream.write(json.dumps(row)+'\\n');atomic_json(run/'state.json',row)\n    conn.sendall(np.zeros(16,dtype='<f8').tobytes())\n finally:\n  stream.close();server.close();atomic_json(run/'worker_result.json',{'completed':True,'fault':None,'samples':2})\n'''
        with tempfile.TemporaryDirectory() as td:
            run=Path(td);(run/'sources/policy').mkdir(parents=True)
            (run/'sources/policy/worker.py').write_text(fake)
            scope=dict(schema='teacher_closed_loop_navigation_scope/v1',allowed=True,
                navigation_ground_truth_used=False,controller_kind='teacher',
                checkpoint_sha256='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34',
                profile={'cascade':{'command_limits':[.3,.2,.3]},'duration_s':60},references={})
            scope_path=run/'navigation_scope.json';scope_path.write_text(json.dumps(scope))
            scope_sha=hashlib.sha256(scope_path.read_bytes()).hexdigest()
            (run/'navigation_status.json').write_text('{"state":"running"}')
            name=str(run/'sock');responses=[];client_errors=[]
            def client():
                try:
                    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as peer:
                        peer.settimeout(2)
                        for _ in range(200):
                            try:peer.connect(name);break
                            except FileNotFoundError:time.sleep(.005)
                        else:raise RuntimeError('stub server never ready')
                        for index in range(2):
                            world=1.+index*.02
                            command=dict(schema_version=1,mode='scan_slam',source='scan_slam',
                                acceptance={'sha256':scope_sha},sequence=index+1,command=[0,0,0],
                                sim_time=world,monotonic_wall=time.monotonic()-(1 if index==1 else 0),healthy=True)
                            (run/'navigation_command.json').write_text(json.dumps(command))
                            state=np.zeros(64,dtype='<f8');state[0]=world;peer.sendall(state.tobytes())
                            responses.append(recv_exact(peer,128))
                except BaseException as error:client_errors.append(error)
            thread=threading.Thread(target=client);thread.start();old_argv=sys.argv[:];old_path=sys.path[:]
            try:
                sys.argv=[str(worker),'--run',str(run),'--socket',name];module.main()
            finally:
                sys.argv=old_argv;sys.path[:]=old_path;thread.join(3)
            self.assertEqual(client_errors,[]);self.assertEqual(len(responses),2)
            self.assertEqual(responses,[bytes(128),bytes(128)])
            rows=[json.loads(x) for x in (run/'telemetry.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),2);self.assertFalse(rows[0]['command_expired'])
            self.assertTrue(rows[1]['command_expired'])
            self.assertEqual(rows[1]['contacts'],{'body':0,'FR':1,'FL':1,'RR':1,'RL':1})
            ack=json.loads((run/'closed_loop_executor_ack.json').read_text())
            self.assertEqual(len(ack['entries']),1);self.assertEqual(ack['entries'][0]['stamp_ns'],1_000_000_000)
            self.assertEqual(ack['entries'][0]['ack_native_clock_ns'],1_020_000_000)
            timing=[json.loads(x) for x in (run/'ipc_frame_timing.jsonl').read_text().splitlines()]
            self.assertEqual(len(timing),2);self.assertEqual(timing[1]['command_validation_status'],'rejected')
            for key in ('command_file_read','observation','inference','telemetry_write','state_publish','response_send'):
                self.assertIn(key,timing[1]['stages_ns'])
            result=json.loads((run/'ipc_evidence_result.json').read_text())
            self.assertTrue(result['timing']['capture_complete']);self.assertFalse(result['native_parking_verified'])



class FreshnessAndAckRegression(unittest.TestCase):
    """Real wrapper and stub Actor; finite synthetic peer, no native acceptance.

    Validation returns are checked with nonzero commands. These tests do not
    model the frozen Actor's slew/posture behavior or certify native damping.
    """
    fixture=r'''import json, socket
from pathlib import Path
import numpy as np
import ipc_fixture_hooks as hooks
def atomic_json(path,value):path.write_text(json.dumps(value))
def recv_exact(conn,size):
 data=b''
 while len(data)<size:
  part=conn.recv(size-len(data))
  if not part:raise EOFError('fixture native peer disconnected')
  data+=part
 return data
def load_actor():
 def actor(obs):
  if hooks.actor_hook is not None:hooks.actor_hook(hooks.frame)
  return np.zeros(12)
 return actor
def build_observation(*args,**kwargs):return np.zeros(247),{}
def requested(*args):return np.zeros(3),'stop'
def main():
 import sys
 run=Path(sys.argv[sys.argv.index('--run')+1]);name=sys.argv[sys.argv.index('--socket')+1]
 atomic_json(run/'policy_manifest.json',{})
 server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(name);server.listen(1)
 actor=load_actor();samples=0;fault=None
 stream=(run/'telemetry.jsonl').open('w',buffering=1)
 try:
  conn,_=server.accept()
  with conn:
   conn.settimeout(2)
   for index in range(hooks.frames):
    state=np.frombuffer(recv_exact(conn,512),dtype='<f8');hooks.frame=index
    cmd,phase=requested('forward',index*.02);obs,extra=build_observation(state,cmd);actor(obs)
    row={'sim_time':index*.02,'world_sim_time':float(state[0]),'command':cmd.tolist(),'requested':cmd.tolist(),
         'command_expired':False,'body_clearance':.3,'contacts':{'body':0,'FR':1,'FL':1,'RR':1,'RL':1},
         'fault':None,'actor_inferred_this_frame':True}
    stream.write(json.dumps(row)+'\n');atomic_json(run/'state.json',row);samples+=1
    conn.sendall(np.zeros(16,dtype='<f8').tobytes())
 except Exception as error:fault=type(error).__name__+': '+str(error)
 finally:
  stream.close();server.close();atomic_json(run/'worker_result.json',{'completed':bool(samples),'fault':fault,'samples':samples})
'''

    @staticmethod
    def command(scope_sha,world,sequence,values):
        clock=round(world*1e9);wall=time.monotonic()
        return dict(schema_version=1,mode='scan_slam',source='scan_slam',
            acceptance={'sha256':scope_sha},sequence=sequence,command=values,
            sim_time=world,monotonic_wall=wall,healthy=True,
            controller_source=dict(navigation_ground_truth_used=False,
                source_pose_stamp_ns=clock,paired_imu_stamp_ns=clock-5_000_000,
                control_stamp_ns=clock,source_pose_received_monotonic_wall=wall,
                paired_imu_received_monotonic_wall=wall,command_after_slew=values))

    @staticmethod
    def state(peer,run,command,world):
        import numpy as np
        (run/'navigation_command.json').write_text(json.dumps(command))
        state=np.zeros(64,dtype='<f8');state[0]=world;peer.sendall(state.tobytes())

    def run_fixture(self,client,*,frames,actor_hook=None):
        """Every socket and temporary file is owned and closed by this fixture."""
        import hashlib
        worker=Path(__file__).with_name('worker.py')
        spec=importlib.util.spec_from_file_location('freshness_wrapper_under_test',worker)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        hooks=types.ModuleType('ipc_fixture_hooks');hooks.frames=frames;hooks.actor_hook=actor_hook;hooks.frame=None
        with tempfile.TemporaryDirectory(prefix='v33_ipc_regression_',dir='/tmp') as td:
            run=Path(td);(run/'sources/policy').mkdir(parents=True)
            (run/'sources/policy/worker.py').write_text(self.fixture)
            scope=dict(schema='teacher_closed_loop_navigation_scope/v1',allowed=True,
                navigation_ground_truth_used=False,controller_kind='teacher',
                checkpoint_sha256='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34',
                profile={'cascade':{'command_limits':[.3,.2,.3]},'duration_s':60},references={})
            scope_path=run/'navigation_scope.json';scope_path.write_text(json.dumps(scope))
            scope_sha=hashlib.sha256(scope_path.read_bytes()).hexdigest()
            (run/'navigation_status.json').write_text('{"state":"running"}')
            name=str(run/'sock');client_errors=[]
            def connected_client():
                try:
                    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as peer:
                        peer.settimeout(2)
                        for _ in range(200):
                            try:peer.connect(name);break
                            except FileNotFoundError:time.sleep(.005)
                        else:raise RuntimeError('fixture server never ready')
                        client(peer,run,scope_sha)
                except BaseException as error:client_errors.append(error)
            old_argv=sys.argv[:];old_path=sys.path[:]
            thread=threading.Thread(target=connected_client)
            try:
                with patch.dict(sys.modules,{'ipc_fixture_hooks':hooks}):
                    thread.start()
                    sys.argv=[str(worker),'--run',str(run),'--socket',name];module.main()
                    thread.join(3)
            finally:
                sys.argv=old_argv;sys.path[:]=old_path
            self.assertFalse(thread.is_alive());self.assertEqual(client_errors,[])
            result={}
            for filename in ['telemetry.jsonl','ipc_frame_timing.jsonl','closed_loop_executor_ack.jsonl',
                             'closed_loop_executor_ack.json','ipc_evidence_result.json','worker_result.json']:
                p=run/filename
                result[filename]=([json.loads(line) for line in p.read_text().splitlines()]
                    if filename.endswith('.jsonl') else json.loads(p.read_text())) if p.exists() else None
            self.assertTrue(result['ipc_evidence_result.json']['timing']['capture_complete'])
            self.assertFalse(result['ipc_evidence_result.json']['native_parking_verified'])
            return result

    def rejected_second_frame(self,mutate):
        def client(peer,run,scope_sha):
            first=self.command(scope_sha,1.,1,[.12,0,0]);self.state(peer,run,first,1.);recv_exact(peer,128)
            second=self.command(scope_sha,1.02,2,[.15,0,0]);mutate(second)
            self.state(peer,run,second,1.02);recv_exact(peer,128)
        result=self.run_fixture(client,frames=2)
        rows=result['telemetry.jsonl'];self.assertEqual(rows[0]['requested'],[.12,0,0])
        self.assertFalse(rows[0]['command_expired']);self.assertEqual(rows[1]['requested'],[0,0,0])
        self.assertTrue(rows[1]['command_expired'])
        timing=result['ipc_frame_timing.jsonl'];self.assertEqual(timing[1]['command_validation_status'],'rejected')
        acks=result['closed_loop_executor_ack.json']['entries'];self.assertEqual(len(acks),1)
        self.assertEqual(acks[0]['applied_command_body'],[.12,0,0])
        return result

    def test_nonzero_producer_expiry_in_either_clock_returns_zero(self):
        for key in ['sim_time','monotonic_wall']:
            with self.subTest(clock=key):
                self.rejected_second_frame(lambda c:c.__setitem__(key,c[key]-1.))

    def test_nonzero_original_source_expiry_returns_zero(self):
        for key in ['source_pose_received_monotonic_wall','paired_imu_received_monotonic_wall']:
            with self.subTest(source=key):
                def mutate(c):c['controller_source'][key]-=1.
                self.rejected_second_frame(mutate)

        with self.subTest(source='pose_sim_stamp'):
            def mutate(c):
                c['controller_source']['source_pose_stamp_ns']-=1_000_000_000
                c['controller_source']['paired_imu_stamp_ns']-=1_000_000_000
            result=self.rejected_second_frame(mutate)
            self.assertIn('Source age/causality rejected',
                result['ipc_frame_timing.jsonl'][1]['command_validation_reason'])

    def test_mutated_or_reversed_sequence_does_not_replay_last_nonzero(self):
        def client(peer,run,scope_sha):
            for index,sequence in enumerate([7,7,6]):
                world=1.+index*.02
                cmd=self.command(scope_sha,world,sequence,[.12 if index==0 else .15,0,0])
                self.state(peer,run,cmd,world);recv_exact(peer,128)
        result=self.run_fixture(client,frames=3);rows=result['telemetry.jsonl']
        self.assertEqual([r['requested'] for r in rows],[[.12,0,0],[0,0,0],[0,0,0]])
        self.assertEqual([r['command_expired'] for r in rows],[False,True,True])

    def test_partial_next_state_cannot_ack_previous_response(self):
        def client(peer,run,scope_sha):
            self.state(peer,run,self.command(scope_sha,1.,1,[.12,0,0]),1.)
            recv_exact(peer,128);peer.sendall(bytes(192));peer.shutdown(socket.SHUT_RDWR)
        result=self.run_fixture(client,frames=2);timing=result['ipc_frame_timing.jsonl']
        self.assertEqual([r['outcome'] for r in timing],['response_sent','state_read_failed'])
        self.assertIsNone(result['closed_loop_executor_ack.json'])
        self.assertEqual(result['closed_loop_executor_ack.jsonl'],[])
        self.assertIn('EOFError',result['worker_result.json']['fault'])

    def test_synthetic_200ms_peer_timeout_cannot_ack_late_response(self):
        entered=threading.Event();closed=threading.Event();accepted=[]
        def actor_hook(frame):
            if frame==1:
                entered.set()
                if not closed.wait(2):raise RuntimeError('synthetic peer did not close')
        def client(peer,run,scope_sha):
            self.state(peer,run,self.command(scope_sha,1.,1,[.12,0,0]),1.)
            accepted.append(recv_exact(peer,128))
            self.state(peer,run,self.command(scope_sha,1.02,2,[.15,0,0]),1.02)
            if not entered.wait(1):raise RuntimeError('Actor stub did not reach delay')
            # Synthetic peer only: this does not certify teacher_actuator.cpp.
            peer.settimeout(.2)
            try:
                with self.assertRaises(TimeoutError):recv_exact(peer,128)
            finally:
                peer.shutdown(socket.SHUT_RDWR);peer.close();closed.set()
        result=self.run_fixture(client,frames=2,actor_hook=actor_hook)
        self.assertEqual(accepted,[bytes(128)])
        timing=result['ipc_frame_timing.jsonl'];self.assertEqual(timing[-1]['outcome'],'response_send_failed')
        self.assertFalse(timing[-1]['response_sent'])
        self.assertGreater(timing[-1]['stages_ns']['inference'],200_000_000)
        acks=result['closed_loop_executor_ack.json']['entries'];self.assertEqual(len(acks),1)
        self.assertEqual(acks[0]['applied_command_body'],[.12,0,0])
        self.assertEqual(acks[0]['stamp_ns'],1_000_000_000)
        self.assertIn('BrokenPipeError',result['worker_result.json']['fault'])


if __name__=='__main__':unittest.main(verbosity=2)
