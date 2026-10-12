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
import unittest

from ipc_evidence import BoundedHistory, FrameTiming, TimedConnection


POLICY=Path(__file__).resolve().parents[2]/'policy/worker.py'
if not POLICY.exists():
    POLICY=Path('/__LOCAL_HOME__/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/policy/worker.py')
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


if __name__=='__main__':unittest.main(verbosity=2)
