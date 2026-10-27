import hashlib,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import known_scene_hold_guard as capture

class MultipleStopCaptureTests(unittest.TestCase):
    def test_lossless_storage_and_float64_fallback(self):
        x=np.array([[.1,.2,.3]],dtype='<f4').astype('<f8')
        packed=capture._lossless_storage_cloud(x)
        self.assertEqual(packed.dtype,np.dtype('<f4'))
        np.testing.assert_array_equal(packed.astype('<f8'),x)
        y=np.array([[1/3,1/7,1/11]],dtype='<f8')
        self.assertIs(capture._lossless_storage_cloud(y),y)

    def fixture(self,run):
        node=SimpleNamespace(run=run,guard_cloud=np.array([[1.,2.,3.]],dtype='<f4'),
            evidence=SimpleNamespace(enqueue=lambda f:f()))
        def row(seq):
            return dict(sequence=seq,request_id='current',compute_ros_clock_ns=seq*100000000,
                control_pose_stamp_ns=seq*100000000,cloud_header_stamp_ns=seq*100000000,
                cloud_frame_id='camera_init',filtered_xyz_float64_sha256=hashlib.sha256(node.guard_cloud.astype('<f8').tobytes()).hexdigest(),
                trajectory_id=seq,reference_stamp=[1,2],route_float64_sha256='a'*64,
                union_result=dict(blocked=True),obstacle_hold_before=False)
        return node,row

    def test_all_stop_manifests_are_immutable_and_replayable(self):
        with tempfile.TemporaryDirectory() as t:
            node,row=self.fixture(Path(t))
            for sequence in (10,20,30):
                buffer=capture.GuardReplayBuffer(node)
                buffer.remember(row(sequence));buffer.persist()
            self.assertEqual(len(list((Path(t)/'r57_dynamic_guard_events').glob('*.json'))),3)
            for sequence in (10,20,30):
                p=Path(t)/capture.replay_payload(Path(t),row(sequence))
                np.testing.assert_array_equal(np.load(p),node.guard_cloud)
            self.assertTrue(capture.dynamic_payload_ready(Path(t)))
            with self.assertRaisesRegex(ValueError,'missing'):
                capture.replay_payload(Path(t),row(40))
            changed=row(10);changed['trajectory_id']=99
            with self.assertRaisesRegex(ValueError,'binding differs'):
                capture.replay_payload(Path(t),changed)

    def test_multiple_stops_share_the_original_storage_budget(self):
        with tempfile.TemporaryDirectory() as t:
            node,row=self.fixture(Path(t))
            first=capture.GuardReplayBuffer(node);first.remember(row(10));first.persist()
            reserved=node._guard_replay_reserved_B
            second=capture.GuardReplayBuffer(node);second.remember(row(20))
            with patch.object(capture,'ARRAY_LIMIT_B',reserved):
                with self.assertRaisesRegex(ValueError,'shared allowance'):
                    second.persist()
            self.assertFalse((Path(t)/'r57_dynamic_guard_events/20.json').exists())

if __name__=='__main__':unittest.main()
