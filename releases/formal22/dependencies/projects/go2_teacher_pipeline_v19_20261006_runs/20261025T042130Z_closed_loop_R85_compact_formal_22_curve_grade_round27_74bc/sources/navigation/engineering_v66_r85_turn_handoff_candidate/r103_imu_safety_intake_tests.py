"""Real mailbox, FIFO callback and saved Teacher guard contracts; no ROS run."""
import ast
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from collections import deque
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

DEMO = Path('/opt/experiment/workspace/multifloor_demo')
sys.path.insert(0, str(DEMO/'navigation'))
from control_core import RawImuTilt, body_tilt
from imu_causal_buffer import CausalImuBuffer
from r103_imu_safety_intake import SafetyIntake, WINDOWS, enabled


def production_class():
    path = Path(__file__).with_name('teacher_wrapper.py')
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TeacherNavigation')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('apply_tilt_guard','on_imu')]
    pid_path = path.with_name('controller.py')
    pid_cls = next(n for n in ast.parse(pid_path.read_text()).body
                   if isinstance(n, ast.ClassDef) and n.name == 'PIDNavigation')
    pid_method = next(n for n in pid_cls.body if isinstance(n, ast.FunctionDef) and n.name == 'on_imu')
    scope = dict(body_tilt=body_tilt, raw_imu_fresh=lambda n,w,s,f:f,
        pose_cloud_fresh=lambda n,k,w,s,f:f, wall_stale=lambda n,k,w,r,t,f:f,
        reset_transition=lambda n,r:None, math=math, np=np, time=time)
    classes = [ast.ClassDef(name='TeacherNavigation',bases=[],keywords=[],body=methods,decorator_list=[]),
        ast.ClassDef(name='PIDNavigation',bases=[ast.Name(id='TeacherNavigation',ctx=ast.Load())],
            keywords=[],body=[pid_method],decorator_list=[])]
    exec(compile(ast.fix_missing_locations(ast.Module(body=classes, type_ignores=[])), str(path), 'exec'), scope)
    return scope['PIDNavigation']


PID_CLASS = production_class()
GUARD = PID_CLASS.apply_tilt_guard


def node():
    n = PID_CLASS()
    n.__dict__.update(imu_body_rotation=np.eye(3), imu_reference_rotation=np.eye(3),
        raw_imu=RawImuTilt(np.eye(3)), counts={'commands':0,'imu':0}, state='running',
        rotation=np.eye(3), max_tilt=0., bridge_safety={'state':'ready'},
        evidence=NS(error=None,append=lambda *a:None), run=Path('/tmp/unused-r103-fixture'),
        tilt_hold=False, tilt_stops=0,
        tilt_clear_since=None, tilt_source=None, pose_updated=10.,
        pose_timeout=.3, bridge_updated=10., samples='unchanged',
        gyro_body=['untouched'], gyro_stamp=123, gyro_wall=456,
        imu_history=deque(maxlen=512),imu_records=0,command=[.1,0.,0.])
    n.reset_region_arrival=lambda reason:None
    n.publications=[]
    n.publish_command=lambda:n.publications.append('zero')
    n.publish_status=lambda:None
    n.stamp = lambda m:m.stamp_ns
    n.clock_ns = 10_000_000_000
    n.get_clock = lambda:NS(now=lambda:NS(nanoseconds=n.clock_ns))
    return n


def message(stamp_ns, tilt=0., gyro=0., available=True):
    return NS(stamp_ns=stamp_ns,
        orientation=NS(x=math.sin(tilt/2),y=0.,z=0.,w=math.cos(tilt/2)),
        angular_velocity=NS(x=gyro,y=0.,z=0.),
        orientation_covariance=[0. if available else -1.]+[0.]*8,
        header=NS(frame_id='imu_link'))


class Contracts(unittest.TestCase):
    def fifo(self, n, setting='1'):
        with patch.dict(os.environ, {'R103_IMU_SAFETY_INTAKE':setting}):
            n.imu_source_buffer = CausalImuBuffer(n, RawImuTilt)
        return n.imu_source_buffer

    def callback(self, n, buffer, stamp, wall, tilt=0., gyro=0., available=True):
        n.clock_ns = stamp
        with patch('imu_causal_buffer.time.monotonic_ns',return_value=wall):
            buffer.callback(message(stamp,tilt,gyro,available), {})

    def original_callback(self,n,stamp,wall,tilt=0.,gyro=0.):
        n.clock_ns=max(n.clock_ns,stamp)
        n.pose_updated=n.bridge_updated=wall/1e9
        with patch('time.monotonic',return_value=wall/1e9),patch('time.monotonic_ns',return_value=wall):
            n.on_imu(message(stamp,tilt,gyro))

    def test_real_E_window_recovers_using_actual_receipt(self):
        n=node();b=self.fifo(n)
        n.raw_imu.update([0,0,0,1],31.485,22545.268908947,31.485)
        self.callback(n,b,31_685_000_000,22_545_551_042_487)
        old_gyro=(n.gyro_body,n.gyro_stamp,n.gyro_wall)
        now=22545.637610109;n.pose_updated=n.bridge_updated=now
        self.assertFalse(n.raw_imu.fresh(now,31.740))
        self.assertFalse(GUARD(n,now,31.740))
        self.assertEqual((n.raw_imu.stamp,n.raw_imu.updated),(31.685,22545.551042487))
        self.assertEqual((n.gyro_body,n.gyro_stamp,n.gyro_wall),old_gyro)
        self.assertEqual(b.safety_intake.counts['stale_to_fresh'],1)

    def test_stale_and_future_wall_receipts_never_restore_freshness(self):
        for stamp,wall,now,clock in [(10.,10.,10.3,10.3),(10.,10.1,10.,10.)]:
            n=node();b=self.fifo(n);self.callback(n,b,round(stamp*1e9),round(wall*1e9))
            self.assertTrue(GUARD(n,now,clock));self.assertFalse(math.isfinite(n.raw_imu.stamp))

    def test_slot_replacement_duplicate_and_out_of_order_do_not_retime(self):
        n=node();b=self.fifo(n)
        self.callback(n,b,10_000_000_000,10_000_000_000)
        self.callback(n,b,10_010_000_000,10_010_000_000)
        self.callback(n,b,10_010_000_000,10_020_000_000)
        self.callback(n,b,10_005_000_000,10_025_000_000)
        GUARD(n,10.03,10.03)
        self.assertEqual((n.raw_imu.stamp,n.raw_imu.updated),(10.01,10.01))
        GUARD(n,10.04,10.04);self.assertEqual(n.raw_imu.updated,10.01)
        self.assertEqual(b.safety_intake.counts['consumed'],1)

    def test_invalid_orientation_is_not_offered(self):
        n=node();b=self.fifo(n)
        self.callback(n,b,10_000_000_000,10_000_000_000,available=False)
        self.assertIsNone(b.safety_intake.latest)

    def test_valid_dangerous_orientation_with_nonfinite_gyro_is_protected(self):
        n=node();b=self.fifo(n)
        self.callback(n,b,10_000_000_000,10_000_000_000,.6,float('nan'))
        self.assertEqual(len(b.history),0)
        self.assertTrue(GUARD(n,10.01,10.01));self.assertEqual(n.state,'failed')

    def test_B1_original_severe_is_not_overwritten_by_pending_fifo_level(self):
        n=node();b=self.fifo(n)
        self.original_callback(n,10_000_000_000,10_000_000_000)
        self.callback(n,b,10_020_000_000,10_020_000_000)
        self.original_callback(n,10_010_000_000,10_030_000_000,.6)
        self.assertEqual(n.state,'failed');self.assertEqual(n.publications,['zero'])
        self.assertEqual(b.safety_intake.original_raw_imu.stamp,10.01)
        self.assertEqual(n.raw_imu.stamp,10.02)
        self.assertEqual(n.imu_records,2)
        self.assertEqual(n.imu_history[-1]['stamp_ns'],10_010_000_000)

    def test_startup_original_freshness_does_not_wait_for_running_guard(self):
        n=node();b=self.fifo(n);n.state='idle'
        original=b.safety_intake.original_raw_imu
        self.original_callback(n,10_000_000_000,10_000_000_000)
        self.assertIs(n.raw_imu,original);self.assertTrue(n.raw_imu.fresh(10.01,10.01))
        self.assertEqual(n.imu_records,1)
        n.state='running';GUARD(n,10.01,10.01)
        self.assertIsNot(n.raw_imu,original);self.assertTrue(n.raw_imu.fresh(10.01,10.01))

    def test_fifo_only_severe_survives_original_level_callback(self):
        n=node();b=self.fifo(n)
        self.callback(n,b,10_000_000_000,10_000_000_000,.6)
        self.original_callback(n,10_010_000_000,10_010_000_000)
        self.assertEqual(n.state,'failed');self.assertEqual(n.publications,['zero'])

    def test_B1_prior_fifo_consumption_cannot_advance_original_watermark(self):
        n=node();b=self.fifo(n)
        self.original_callback(n,10_000_000_000,10_000_000_000)
        self.callback(n,b,10_020_000_000,10_020_000_000)
        GUARD(n,10.025,10.025)
        self.original_callback(n,10_010_000_000,10_030_000_000,.6)
        self.assertEqual(n.state,'failed');self.assertEqual(n.publications,['zero'])
        self.assertEqual(b.safety_intake.original_raw_imu.last_rejected,None)
        self.assertEqual(n.gyro_stamp,10.01);self.assertEqual(n.imu_records,2)

    def test_same_stamp_ports_preserve_original_gyro_history_and_receipt(self):
        for fifo_first in (True,False):
            n=node();b=self.fifo(n)
            if fifo_first:
                self.callback(n,b,10_000_000_000,10_000_000_000)
                GUARD(n,10.001,10.001)
            self.original_callback(n,10_000_000_000,10_010_000_000,gyro=.7)
            if not fifo_first:self.callback(n,b,10_000_000_000,10_020_000_000)
            GUARD(n,10.03,10.03)
            self.assertEqual(n.imu_records,1);self.assertEqual(n.gyro_body,[.7,0.,0.])
            self.assertEqual(n.gyro_wall,10.01)
            self.assertEqual(b.safety_intake.original_raw_imu.updated,10.01)
            self.assertEqual(n.imu_history[0]['received_wall_ns'],10_010_000_000)
            receipt=n.raw_imu.updated
            self.original_callback(n,10_000_000_000,10_040_000_000,gyro=.8)
            self.assertEqual(n.imu_records,1);self.assertEqual(n.gyro_body,[.7,0.,0.])
            self.assertEqual(n.raw_imu.updated,receipt)
            # The rejected original duplicate must not borrow the healthy
            # reading view's rejection field in the existing timing receipt.
            original_rejection=b.safety_intake.original_raw_imu.last_rejected
            self.assertEqual(original_rejection,'repeated or out-of-order IMU stamp')
            self.assertEqual(b.recent[-1]['raw_rejection'],original_rejection)
            self.assertIsNone(n.raw_imu.last_rejected)

    def test_original_staleness_remains_visible_after_merged_view_is_fresh(self):
        n=node();b=self.fifo(n);n.publication_records=17
        self.original_callback(n,10_000_000_000,10_000_000_000)
        self.callback(n,b,10_200_000_000,10_200_000_000)
        n.pose_updated=n.bridge_updated=10.21;GUARD(n,10.21,10.21)
        self.callback(n,b,10_300_000_000,10_300_000_000)
        n.pose_updated=n.bridge_updated=10.31;GUARD(n,10.31,10.31)
        row=b.safety_intake.windows[-1]
        self.assertTrue(row['before_raw_scalar_fresh'])
        self.assertFalse(row['original_scalar_fresh'])
        self.assertTrue(row['sample_scalar_fresh']);self.assertTrue(row['merged_scalar_fresh'])
        self.assertEqual((row['original_stamp_ns'],row['original_receipt_ns']),(10_000_000_000,10_000_000_000))
        self.assertEqual((row['source_stamp_ns'],row['callback_wall_ns']),(10_300_000_000,10_300_000_000))
        self.assertEqual(row['r73_publication_sequence_before'],17)
        self.assertEqual(b.safety_intake.counts['old_port_stale_new_view_fresh'],1)
        receipt=n.raw_imu.updated;GUARD(n,10.32,10.32)
        self.assertFalse(b.safety_intake.windows[-1]['selected_this_guard'])
        self.assertEqual(n.raw_imu.updated,receipt)
        self.assertEqual(b.safety_intake.counts['old_port_stale_new_view_fresh'],2)

    def test_B2_intermediate_point18_to_point30_resets_recovery(self):
        # Quaternion->acos rounds nominal .18 just below .18 on this host;
        # exact scalar thresholds are exercised separately below.
        for tilt in (.180001,.25,.299999,.30):
            n=node();b=self.fifo(n);n.tilt_hold=True;n.tilt_clear_since=10.
            self.callback(n,b,10_790_000_000,10_790_000_000,tilt)
            self.callback(n,b,10_800_000_000,10_800_000_000,0.)
            n.pose_updated=n.bridge_updated=10.81
            self.assertTrue(GUARD(n,10.81,10.81));self.assertTrue(n.tilt_hold)
            self.assertIsNone(n.tilt_clear_since)
            for now in (10.9,11.3,11.69):
                self.callback(n,b,round(now*1e9),round(now*1e9))
                n.pose_updated=n.bridge_updated=now
                GUARD(n,now,now);self.assertTrue(n.tilt_hold)
            self.callback(n,b,11_710_000_000,11_710_000_000)
            n.pose_updated=n.bridge_updated=11.71
            GUARD(n,11.71,11.71);self.assertFalse(n.tilt_hold)

    def test_exact_scalar_thresholds_keep_original_comparisons(self):
        for tilt in (.179999,.18,.299999,.30,.499999,.50):
            n=node();b=self.fifo(n);n.tilt_hold=True;n.tilt_clear_since=9.
            b.safety_intake.offer(1,10_000_000_000,10_000_000_000,10_000_000_000,tilt)
            b.safety_intake.offer(2,10_010_000_000,10_010_000_000,10_010_000_000,0.)
            GUARD(n,10.02,10.02)
            self.assertEqual(n.state=='failed',tilt>=.50)
            if tilt < .50:
                self.assertEqual(n.tilt_hold,tilt>=.18)
                if tilt>=.18:self.assertIsNone(n.tilt_clear_since)

    def test_original_recovery_interruption_survives_newer_fifo_level(self):
        n=node();b=self.fifo(n);n.tilt_hold=True;n.tilt_clear_since=10.
        self.callback(n,b,10_800_000_000,10_800_000_000)
        self.original_callback(n,10_790_000_000,10_810_000_000,.25)
        self.assertTrue(n.tilt_hold);self.assertIsNone(n.tilt_clear_since)

    def test_future_header_and_source_expiry_match_original_fresh_predicate(self):
        for stamp in (10.099999,10.10,10.100001,9.75,9.749999):
            n=node();b=self.fifo(n);n.clock_ns=10_000_000_000
            m=message(round(stamp*1e9));original=RawImuTilt(np.eye(3))
            accepted=original.update([0.,0.,0.,1.],m.stamp_ns/1e9,10.,10.)
            with patch('imu_causal_buffer.time.monotonic_ns',return_value=10_000_000_000):
                b.callback(m,{})
            GUARD(n,10.,10.)
            self.assertEqual(n.raw_imu.fresh(10.,10.),accepted and original.fresh(10.,10.))

    def test_intermediate_severe_survives_latest_level_and_is_consumed_once(self):
        n=node();b=self.fifo(n)
        self.callback(n,b,10_000_000_000,10_000_000_000,.6)
        self.callback(n,b,10_010_000_000,10_010_000_000,0.)
        self.assertTrue(GUARD(n,10.02,10.02));self.assertEqual(n.state,'failed')
        self.assertEqual(n.raw_imu.tilt,0.)
        self.assertEqual(b.safety_intake.counts['severe_delivered'],1)
        self.assertIsNone(b.safety_intake.consume(n,10.03,10.03))

    def test_high_peak_preserves_original_point_eight_second_recovery(self):
        n=node();b=self.fifo(n)
        self.callback(n,b,10_000_000_000,10_000_000_000,.35)
        self.callback(n,b,10_010_000_000,10_010_000_000,0.)
        self.assertTrue(GUARD(n,10.02,10.02));self.assertTrue(n.tilt_hold)
        for now in (10.1,10.5,10.89):
            self.callback(n,b,round(now*1e9),round(now*1e9))
            n.pose_updated=n.bridge_updated=now
            self.assertTrue(GUARD(n,now,now));self.assertTrue(n.tilt_hold)
        self.callback(n,b,10_910_000_000,10_910_000_000)
        n.pose_updated=n.bridge_updated=10.91
        self.assertTrue(GUARD(n,10.91,10.91));self.assertFalse(n.tilt_hold)

    def test_off_callback_and_guard_keep_original_safety_state(self):
        n=node();b=self.fifo(n,'0')
        self.callback(n,b,10_000_000_000,10_000_000_000,.6)
        self.assertIsNone(b.safety_intake)
        self.assertEqual(len(b.history),1)
        self.assertTrue(GUARD(n,10.01,10.01));self.assertEqual(n.state,'running')
        self.assertFalse(math.isfinite(n.raw_imu.stamp))
        with patch.dict(os.environ,{'R103_IMU_SAFETY_INTAKE':'bad'}):
            self.assertRaises(ValueError,enabled)

    def test_concurrent_mailbox_is_coherent_and_receipt_is_bounded(self):
        n=node();s=SafetyIntake();errors=[]
        def producer():
            for seq in range(1,3001):
                s.offer(seq,seq*1_000_000,seq*1_000_000,seq*1_000_000,.35)
        t=threading.Thread(target=producer);t.start()
        while t.is_alive():
            with s._lock:
                sample=s.latest
            if sample:
                s.consume(n,sample[1]/1e9,sample[1]/1e9)
        t.join()
        for seq in range(3001,3101):
            s.offer(seq,seq*1_000_000,seq*1_000_000,seq*1_000_000,.35)
            s.consume(n,seq/1000,seq/1000)
        r=s.receipt();self.assertLessEqual(len(r['windows']),WINDOWS)
        self.assertLess(len(json.dumps(r,allow_nan=False)),32768)
        for w in r['windows']:
            self.assertEqual(w['source_stamp_ns'],w['sequence']*1_000_000)
            self.assertEqual(w['source_stamp_ns'],w['callback_wall_ns'])
            self.assertEqual(w['source_stamp_ns'],w['callback_clock_ns'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
