"""Finite pure tests; no simulation, no large files."""
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from storage_guard import usage,check,SOFT_BYTES,HOME_RESERVE,ROOT_RESERVE,Watchdog,StorageStop,conservative_growth

class StorageTests(unittest.TestCase):
    def row(self,growth=0):
        return dict(monotonic_wall=10.,conservative_session_growth_bytes=growth,
            filesystems=[dict(path='/home',available_bytes=HOME_RESERVE+1),
                         dict(path='/tmp',available_bytes=ROOT_RESERVE+1)])
    def test_soft_before_hard(self):
        self.assertIn('whole_session_soft_stop_3p5GiB',check(self.row(SOFT_BYTES))['stop_reasons'])
    def test_home_reserve(self):
        r=self.row();r['filesystems'][0]['available_bytes']-=2
        self.assertIn('filesystem_reserve_/home',check(r)['stop_reasons'])
    def test_root_reserve(self):
        r=self.row();r['filesystems'][1]['available_bytes']-=2
        self.assertIn('filesystem_reserve_/tmp',check(r)['stop_reasons'])
    def test_predicts_drain(self):
        r=self.row(SOFT_BYTES-1000);prev=self.row(SOFT_BYTES-1100);prev['monotonic_wall']=9.
        self.assertIn('projected_stop_grace_would_exhaust_soft_budget',check(r,prev)['stop_reasons'])
    def test_long_cleanup_bound(self):
        r=self.row(SOFT_BYTES-10000);prev=self.row(SOFT_BYTES-10100);prev['monotonic_wall']=9.
        self.assertIn('projected_stop_grace_would_exhaust_soft_budget',check(r,prev)['stop_reasons'])
    def test_unknown_writes_also_include_initial_seed(self):
        initial=dict(logical_bytes=3*1024**3,allocated_bytes=2*1024**3)
        self.assertEqual(conservative_growth(initial,initial,2*1024**3),5*1024**3)
    def test_first_observation_already_reserves_history_peak_and_atomic_write(self):
        r=check(self.row(2*1024**3))
        self.assertGreater(r['predictive_growth_allowance_bytes_per_s'],16_721_875)
        self.assertIn('projected_stop_grace_would_exhaust_soft_budget',r['stop_reasons'])
    def test_overlap_symlink_and_hardlink_not_double_counted(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'a').mkdir();(p/'a/x').write_bytes(b'1234567')
            (p/'link').symlink_to(p/'a/x');(p/'hard').hardlink_to(p/'a/x')
            self.assertEqual(usage([p,p/'a'])['logical_bytes'],7)
            self.assertEqual(usage([p,p/'a'])['files'],1)
    def test_watchdog_initial_log_io_failure_stops(self):
        with tempfile.TemporaryDirectory() as tmp:
            calls=[];w=Watchdog(tmp,{},lambda:calls.append('only_runner'));w.last={}
            with patch.object(Path,'open',side_effect=OSError('injected write failure')):
                w._loop()
            self.assertEqual(calls,['only_runner'])
            self.assertIn('stop receipt unavailable',w.error)
    def test_stop_receipt_failure_cannot_prevent_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            calls=[];w=Watchdog(tmp,{},lambda:calls.append('only_runner'));w.last={}
            w.done.wait=lambda seconds:False
            with patch('storage_guard.observe',side_effect=StorageStop({'stop_reasons':['budget']})), \
                 patch.object(Path,'write_text',side_effect=OSError('injected full disk')):
                w._loop()
            self.assertEqual(calls,['only_runner'])
    def test_close_prevents_late_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            calls=[];w=Watchdog(tmp,{},lambda:calls.append('only_runner'));w.last={};w.done.set()
            with patch.object(Path,'open',side_effect=OSError('after close')):w._loop()
            self.assertEqual(calls,[])

if __name__=='__main__':unittest.main()
