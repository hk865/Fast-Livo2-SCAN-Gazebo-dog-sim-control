"""Finite original-predicate tests; no ROS, processes or model execution."""
import copy
from pathlib import Path
import unittest
from r111_scan_startup_identity import await_startup,SAMPLE_LIMIT

class Clock:
    def __init__(self):self.value=0.
    def __call__(self):return self.value
    def sleep(self,s):self.value+=s

class Child:
    pid=123
    def poll(self):return None

class Checks(unittest.TestCase):
    def fixture(self):
        clock=Clock();child=Child();binary=Path('/tmp/r111_fixture_only_binary')
        identity=dict(pid=123,ppid=12,pgid=12,start_ticks=45,exe=str(binary))
        nonce='a'*32
        command=[str(binary),'--ros-args','-p','fsm.child_odometry_consumption_ack:=true',
            '-p','fsm.child_odometry_ack_nonce:='+nonce,'-p','fsm.reference_protocol_ack:=true']
        raw=b'\0'.join(x.encode()for x in command)+b'\0';records=[];owned=[]
        args=dict(child=child,binary=binary,digest='hash',supervisor_pid=12,nonce=nonce,
            expected_command=command,require_reference=True,deadline=5.,identity=lambda _:dict(identity),
            record=records.append,provisional=owned.append,monotonic=clock,pause=clock.sleep,
            read_argv=lambda:raw,hash_exe=lambda:'hash')
        return args,raw,records,owned,clock,identity

    def test_empty_and_prefix_converge_without_accepting_them(self):
        for transient in (b'',b'/tmp/r111_fixture_only_binary\0--ros-args\0'):
            a,raw,records,owned,clock,_=self.fixture();items=iter([transient,raw,raw]);a['read_argv']=lambda:next(items)
            result=await_startup(**a);self.assertEqual(result,owned[0]);self.assertTrue(records[-1]['accepted'])
            self.assertEqual(records[-1]['samples'][0]['reason'],'argv_predicate_not_satisfied')
            self.assertGreater(clock(),0);self.assertLess(clock(),5)

    def test_permanent_empty_exact_budget_and_evidence_bound(self):
        a,_,records,owned,clock,_=self.fixture();a['read_argv']=lambda:b''
        with self.assertRaisesRegex(RuntimeError,'deadline'):await_startup(**a)
        self.assertEqual(clock(),5.);self.assertTrue(owned);p=records[-1]
        self.assertFalse(p['accepted']);self.assertGreater(p['sample_count'],100);self.assertEqual(len(p['samples']),SAMPLE_LIMIT)
        self.assertEqual(p['samples'][0]['sequence'],1);self.assertEqual(p['samples'][-1]['sequence'],p['sample_count'])
        self.assertEqual(p['expected_nonce'],'a'*32)

    def test_wrong_duplicate_and_reference_flag_fail_immediately(self):
        for kind in ('wrong','duplicate','reference','conflicting_nonce','conflicting_flag'):
            a,raw,records,owned,clock,_=self.fixture()
            bad=raw.replace(b'a'*32,b'b'*32)if kind=='wrong' else raw+b'fsm.child_odometry_ack_nonce:'+b'=' + b'a'*32+b'\0' if kind=='duplicate' else raw.replace(b'fsm.reference_protocol_ack:=true',b'fsm.reference_protocol_ack:=false')
            if kind=='conflicting_nonce':bad=raw+b'fsm.child_odometry_ack_nonce:='+b'b'*32+b'\0'
            if kind=='conflicting_flag':bad=raw+b'fsm.child_odometry_consumption_ack:=false\0'
            a['read_argv']=lambda:bad
            with self.assertRaises(ValueError):await_startup(**a)
            self.assertEqual(clock(),0);self.assertTrue(owned);self.assertFalse(records[-1]['accepted'])
            self.assertEqual(bytes.fromhex(records[-1]['samples'][-1]['actual_argv_prefix_hex']),bad)

    def test_unrelated_argv_difference_preserves_original_acceptance(self):
        a,raw,records,_,_,_=self.fixture();a['read_argv']=lambda:raw+b'unrelated_original_accepted_argument\0'
        await_startup(**a);self.assertTrue(records[-1]['accepted']);self.assertFalse(records[-1]['full_argv_equality_required'])

    def test_identity_hash_exit_and_hash_deadline_fail(self):
        for kind in ('identity','hash','exit','latehash'):
            a,_,records,owned,clock,ident=self.fixture()
            if kind=='identity':
                count=[0]
                def current(_):
                    count[0]+=1;return dict(ident,start_ticks=46)if count[0]>1 else dict(ident)
                a['identity']=current
            elif kind=='hash':a['hash_exe']=lambda:'wrong'
            elif kind=='latehash':
                def late():clock.value=5.;return 'hash'
                a['hash_exe']=late
            else:
                calls=[0]
                def poll():calls[0]+=1;return 1 if calls[0]>1 else None
                a['child'].poll=poll
            with self.assertRaises((ValueError,RuntimeError)):await_startup(**a)
            self.assertFalse(records[-1]['accepted']);self.assertTrue(owned)

    def test_final_read_mismatch_is_retained_and_rejected(self):
        a,raw,records,_,_,_=self.fixture();bad=raw.replace(b'a'*32,b'b'*32);items=iter([raw,bad]);a['read_argv']=lambda:next(items)
        with self.assertRaises(ValueError):await_startup(**a)
        self.assertEqual(bytes.fromhex(records[-1]['samples'][-1]['actual_argv_prefix_hex']),bad)

    def test_nonempty_partial_wrong_is_not_retryable(self):
        a,_,records,_,clock,_=self.fixture();a['read_argv']=lambda:b'/wrong-executable\0'
        with self.assertRaises(ValueError):await_startup(**a)
        self.assertEqual(clock(),0);self.assertFalse(records[-1]['accepted'])

if __name__=='__main__':unittest.main(verbosity=2)
