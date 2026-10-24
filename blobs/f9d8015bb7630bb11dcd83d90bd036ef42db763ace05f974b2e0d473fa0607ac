"""Finite CPU negatives for historical charge, loss and concurrent accounting."""
from pathlib import Path
import concurrent.futures
import copy
import datetime
import hashlib
import importlib.util
import json
import os
import resource
import sys
import tempfile
import time
import unittest

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import storage_recovery as rec
import storage_guard as new
spec=importlib.util.spec_from_file_location('original_guard',HERE/'BEFORE_storage_guard.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
FLOOR=47_772_774_400


def fixture():
    path=Path(tempfile.mkdtemp(prefix='case_',dir=HERE/'test_fixtures'))
    present=path/'present';present.mkdir();missing=path/'lost'
    session=dict(roots=[str(present),str(missing)],initial_usage=dict(logical_bytes=100,allocated_bytes=100),
        initial_filesystems=[dict(device=1,available_bytes=10000),dict(device=2,available_bytes=10000)],
        hard_bytes=80_000_000_000,soft_bytes=new.SOFT_BYTES,cleanup_reserve_bytes=512*1024**2)
    known=dict(logical_bytes=100,allocated_bytes=100)
    fs=[dict(device=1,available_bytes=10000),dict(device=2,available_bytes=10000)]
    state=dict(recovery_id='finite_fixture',sequence=0,historical_floor_bytes=FLOOR,charged_bytes=FLOOR,
        known_logical=100,known_allocated=100,available_bytes={'1':10000,'2':10000},
        logical_increments=0,allocated_increments=0,filesystem_increments=0)
    raw=rec.canonical(dict(previous_sha256=None,state=state))
    journal=path/'journal.jsonl';journal.write_bytes(raw);lock=path/'lock';lock.touch()
    js=journal.stat();ls=lock.stat();checkpoint=path/'checkpoint.json'
    checkpoint.write_bytes(rec.canonical(dict(sequence=0,tail_sha256=hashlib.sha256(raw).hexdigest(),charged_bytes=FLOOR)))
    contract=dict(schema='R39_storage_reboot_recovery/v1',recovery_id='finite_fixture',boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        original_session_stable_sha256=rec.stable_session(session),original_registered_roots=session['roots'],
        verified_missing_roots=[str(missing)],historical_floor_bytes=FLOOR,historical_evidence=[],
        journal_path=str(journal),journal_identity=[js.st_dev,js.st_ino],lock_path=str(lock),lock_identity=[ls.st_dev,ls.st_ino],
        checkpoint_path=str(checkpoint),initial_journal_record_sha256=hashlib.sha256(raw).hexdigest(),journal_hard_cap_bytes=4*1024**2)
    (path/'STORAGE_REBOOT_RECOVERY_CONTRACT.json').write_text(json.dumps(contract))
    return path,session,contract,known,fs


def observe(f,known=None,fs=None):
    p,s,c,k,a=f;k=k if known is None else known;a=a if fs is None else fs
    def usage(paths,allowed_missing=()):
        new.usage(paths,allowed_missing=allowed_missing)
        return k
    def capacity(path):return a[0 if path=='/home' else 1]
    return rec.observe_recovery(p,s,usage,capacity,new.conservative_growth)


class RecoveryTests(unittest.TestCase):
    def test_01_missing_history_keeps_floor_and_unknown_root_rejects(self):
        f=fixture();_,_,r=observe(f);self.assertEqual(r['charged_bytes'],FLOOR)
        self.assertFalse(Path(f[2]['verified_missing_roots'][0]).exists())
        f[1]['roots'].append(str(f[0]/'other_lost'))
        with self.assertRaisesRegex(RuntimeError,'Storage output root absent'):observe(f)

    def test_02_free_then_write_is_charged_again_without_refund(self):
        f=fixture();_,_,a=observe(f,dict(logical_bytes=150,allocated_bytes=180))
        _,_,b=observe(f,dict(logical_bytes=10,allocated_bytes=10))
        _,_,c=observe(f,dict(logical_bytes=15,allocated_bytes=20))
        self.assertEqual(a['charged_bytes'],FLOOR+80)
        self.assertEqual(b['charged_bytes'],FLOOR+80)
        self.assertEqual(c['charged_bytes'],FLOOR+90)

    def test_03_filesystem_free_cannot_cancel_other_device_growth(self):
        f=fixture();_,_,r=observe(f,fs=[dict(device=1,available_bytes=11000),dict(device=2,available_bytes=9993)])
        self.assertEqual(r['charged_bytes'],FLOOR+7)
        _,_,r=observe(f,fs=[dict(device=1,available_bytes=10990),dict(device=2,available_bytes=20000)])
        self.assertEqual(r['charged_bytes'],FLOOR+17)

    def test_04_returning_history_is_counted_and_never_claimed_restored(self):
        f=fixture();root=Path(f[2]['verified_missing_roots'][0]);root.mkdir();(root/'record').write_bytes(b'valid newly recovered bytes')
        k=new.usage(f[1]['roots']);_,_,r=observe(f,k)
        self.assertGreater(r['charged_bytes'],FLOOR)
        self.assertEqual(r['missing_roots'],[])
        self.assertFalse(r['original_history_verified_restored'])

    def test_05_new_boot_changed_baseline_and_removed_registration_reject(self):
        f=fixture();c=f[2];s=f[1]
        with self.assertRaisesRegex(RuntimeError,'another boot'):rec.verify_contract(c,s,'different')
        changed=copy.deepcopy(s);changed['hard_bytes']=81_000_000_000
        with self.assertRaisesRegex(RuntimeError,'baseline or limits changed'):rec.verify_contract(c,changed,c['boot_id'])
        changed=copy.deepcopy(s);changed['roots']=changed['roots'][:1]
        with self.assertRaisesRegex(RuntimeError,'root removed'):rec.verify_contract(c,changed,c['boot_id'])

    def test_06_partial_and_complete_line_journal_truncation_reject(self):
        f=fixture();j=Path(f[2]['journal_path']);first=j.read_bytes();observe(f,dict(logical_bytes=500,allocated_bytes=800))
        with j.open('r+b') as stream:stream.truncate(len(first))
        with self.assertRaisesRegex(RuntimeError,'checkpoint inconsistent'):observe(f)
        f=fixture();j=Path(f[2]['journal_path'])
        with j.open('ab') as stream:stream.write(b'{partial')
        with self.assertRaisesRegex(RuntimeError,'partial'):observe(f)

    def test_07_missing_journal_checkpoint_contract_and_replaced_lock_reject(self):
        for component in ['journal_path','checkpoint_path','lock_path']:
            f=fixture();p=Path(f[2][component]);p.rename(p.with_suffix('.preserved'))
            with self.assertRaises(FileNotFoundError):observe(f)
        f=fixture();p=f[0]/'STORAGE_REBOOT_RECOVERY_CONTRACT.json';p.rename(p.with_suffix('.preserved'))
        with self.assertRaisesRegex(RuntimeError,'contract lost'):observe(f)
        f=fixture();p=Path(f[2]['lock_path']);p.rename(p.with_suffix('.preserved'));p.touch()
        with self.assertRaisesRegex(RuntimeError,'lock replaced'):observe(f)

    def test_08_threads_serialized_and_no_lost_updates(self):
        f=fixture()
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            result=list(pool.map(lambda _:observe(f,dict(logical_bytes=200,allocated_bytes=300))[2],range(40)))
        state,_,_=rec.read_journal(f[2])
        self.assertEqual(state['sequence'],40)
        self.assertEqual(state['charged_bytes'],FLOOR+200)
        self.assertEqual(sorted(r['sequence'] for r in result),list(range(1,41)))

    def test_09_processes_serialized_and_no_lost_updates(self):
        import multiprocessing
        f=fixture();ctx=multiprocessing.get_context('fork')
        def worker():
            for _ in range(10):observe(f,dict(logical_bytes=200,allocated_bytes=300))
        workers=[ctx.Process(target=worker) for _ in range(3)]
        for w in workers:w.start()
        for w in workers:w.join(10);self.assertEqual(w.exitcode,0)
        state,_,_=rec.read_journal(f[2]);self.assertEqual(state['sequence'],30);self.assertEqual(state['charged_bytes'],FLOOR+200)

    def test_10_original_soft_reserve_predictive_limits_identical(self):
        import ast
        a=ast.parse((HERE/'BEFORE_storage_guard.py').read_text());b=ast.parse((HERE/'storage_guard.py').read_text())
        def node(t,name):return next(n for n in t.body if isinstance(n,ast.FunctionDef) and n.name==name)
        self.assertEqual(ast.dump(node(a,'check')),ast.dump(node(b,'check')))
        for value in ['HARD_BYTES','SOFT_BYTES','HOME_RESERVE','ROOT_RESERVE','MIN_PREDICTIVE_RATE','MAX_SINGLE_ATOMIC_WRITE']:self.assertEqual(getattr(old,value),getattr(new,value))
        for charged in [FLOOR,new.SOFT_BYTES-1,new.SOFT_BYTES,new.HARD_BYTES+1]:
            sample=dict(conservative_session_growth_bytes=charged,monotonic_wall=1,filesystems=[dict(path='/home',available_bytes=new.HOME_RESERVE),dict(path='/tmp',available_bytes=new.ROOT_RESERVE)])
            self.assertEqual(old.check(copy.deepcopy(sample)),new.check(copy.deepcopy(sample)))

    def test_11_counter_cap_and_new_filesystem_reject(self):
        f=fixture();f[2]['journal_hard_cap_bytes']=10;(f[0]/'STORAGE_REBOOT_RECOVERY_CONTRACT.json').write_text(json.dumps(f[2]))
        with self.assertRaisesRegex(RuntimeError,'oversized'):observe(f)
        f=fixture()
        with self.assertRaisesRegex(RuntimeError,'filesystem identity changed'):observe(f,fs=[dict(device=1,available_bytes=10000),dict(device=3,available_bytes=10000)])


if __name__=='__main__':
    resource.setrlimit(resource.RLIMIT_CPU,(45,45));resource.setrlimit(resource.RLIMIT_AS,(1024**3,1024**3))
    (HERE/'test_fixtures').mkdir(exist_ok=True)
    start=time.monotonic();suite=unittest.defaultTestLoader.loadTestsFromTestCase(RecoveryTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    receipt=dict(schema='R33_startup_finite_CPU_receipt/v1',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),tests_pass=result.wasSuccessful(),tests_run=result.testsRun,failures=len(result.failures)+len(result.errors),
        wall_s=time.monotonic()-start,source_sha256={str(HERE/n):sha(HERE/n) for n in ['storage_recovery.py','storage_guard.py','corridor_preflight.py','recovery_tests.py','BEFORE_storage_guard.py']},
        failure_details=[dict(test=str(case),traceback=trace) for case,trace in [*result.errors,*result.failures]],GPU=False,ROS=False,simulation=False,navigation_admission_changed=False)
    with (HERE/'RECOVERY_FINITE_TEST_RECEIPT.json').open('x') as stream:stream.write(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt));sys.exit(0 if result.wasSuccessful() else 1)
