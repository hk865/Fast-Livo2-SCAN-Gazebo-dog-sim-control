#!/usr/bin/env python3
"""Regression tests for frame, real-RGB parsing and immutable map contracts."""
import json
import struct
import sys
import tempfile
import time
import unittest
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from geometry import transform_state, body_twist, cloud_records, shifted_pose_covariance
from map_archive import MapArchive, VoxelArchive, BROWSER_DTYPE, pcd_bytes, atomic_write

ZERO = {'body_xyz': [0, 0, 0], 'body_rpy': [0, 0, 0]}


def fixture_cloud(endian='<', point_step=24, row_padding=8):
    width, height = 2, 2
    row_step = width * point_step + row_padding
    data = bytearray(row_step * height)
    rgb = [0xFF0000, 0x00FF00, 0x0000FF, 0x123456]
    for i, packed in enumerate(rgb):
        base = (i // width) * row_step + (i % width) * point_step
        struct.pack_into(endian+'fff', data, base, i+.1, i+.2, i+.3)
        struct.pack_into(endian+'I', data, base+16, packed)
    fields = [SimpleNamespace(name=n, offset=i*4, datatype=7, count=1) for i,n in enumerate(['x','y','z'])]
    fields.append(SimpleNamespace(name='rgb', offset=16, datatype=7, count=1))
    return SimpleNamespace(fields=fields, is_bigendian=endian=='>', point_step=point_step,
                           row_step=row_step, data=bytes(data), width=width, height=height)


class FrameTests(unittest.TestCase):
    def test_rotating_offset_prevents_old_five_centimetre_tf_bug(self):
        imu = {'body_xyz': [0,0,.05], 'body_rpy': [0,0,0]}
        r = Rotation.from_euler('y',90,degrees=True)
        p, _ = transform_state([1,2,3],r.as_quat(),imu,ZERO)
        np.testing.assert_allclose(p,[.95,2,3],atol=1e-10)

    def test_general_rotated_imu_sensor_chain(self):
        imu={'body_xyz':[.1,.2,.3],'body_rpy':[.1,.2,.3]}
        sensor={'body_xyz':[.4,-.2,.1],'body_rpy':[-.4,.1,-.2]}
        rwb=Rotation.from_euler('xyz',[.3,-.2,.5]).as_matrix()
        rbi=Rotation.from_euler('xyz',imu['body_rpy']).as_matrix()
        pwi=np.array([3,4,5])+rwb@imu['body_xyz']
        p,r=transform_state(pwi,Rotation.from_matrix(rwb@rbi).as_quat(),imu,sensor)
        np.testing.assert_allclose(p,np.array([3,4,5])+rwb@sensor['body_xyz'])
        np.testing.assert_allclose(r,rwb@Rotation.from_euler('xyz',sensor['body_rpy']).as_matrix())

    def test_twist_current_child_frame_and_lever_arm(self):
        sensor={'body_xyz':[.2,0,.1177],'body_rpy':[0,0,0]}
        a=transform_state([0,0,0],[0,0,0,1],ZERO,sensor)
        b=transform_state([0,0,0],Rotation.from_euler('z',.1).as_quat(),ZERO,sensor)
        v,w=body_twist(a,b,.1)
        np.testing.assert_allclose(w,[0,0,1],atol=1e-10)
        self.assertAlmostEqual(v[1],.2*np.sin(.1)/.1)
        with self.assertRaises(ValueError): body_twist(a,b,0)
        with self.assertRaises(ValueError): body_twist(a,b,-.1)

    def test_config_exactly_matches_scenario_extrinsics_and_intrinsics(self):
        root=Path(__file__).resolve().parents[1]
        sensors=json.loads((root.parent/'simulation/scenario.json').read_text())['sensors']
        p=yaml.safe_load((root/'fastlivo.yaml').read_text())['/**']['ros__parameters']
        i,l,c=[sensors[n] for n in ['imu','lidar','camera']]
        ri,rl,rc=[Rotation.from_euler('xyz',s['body_rpy']).as_matrix() for s in [i,l,c]]
        np.testing.assert_allclose(p['extrin_calib']['extrinsic_T'],ri.T@(np.array(l['body_xyz'])-i['body_xyz']))
        np.testing.assert_allclose(np.array(p['extrin_calib']['Rcl']).reshape(3,3),rc.T@rl,atol=1e-10)
        np.testing.assert_allclose(p['extrin_calib']['Pcl'],rc.T@(np.array(l['body_xyz'])-c['body_xyz']),atol=1e-10)
        cam=yaml.safe_load((root/'camera.yaml').read_text())['/**']['ros__parameters']
        self.assertAlmostEqual(cam['cam_fx'],c['width']/2/np.tan(c['hfov']/2))
        self.assertFalse(p['publish']['publish_base_tf'])
        self.assertEqual(p['common']['img_en'],1)
        self.assertFalse(p['common']['use_gps'])

    def test_orientation_uncertainty_propagates_through_lidar_lever_arm(self):
        cov=np.zeros((6,6));cov[5,5]=.04
        result=np.array(shifted_pose_covariance(cov.ravel(),[.2,0,0])).reshape(6,6)
        self.assertAlmostEqual(result[1,1],.04*.2*.2)
        self.assertAlmostEqual(result[1,5],.04*.2)


class MapTests(unittest.TestCase):
    def test_full_route_default_capacity_preserves_eight_centimetre_resolution(self):
        archive=VoxelArchive()
        self.assertEqual(archive.max_points,8000000)
        self.assertEqual(archive.voxel_size,.08)

    def test_actual_rgb_preserved_with_row_padding_and_both_endiannesses(self):
        for endian in ['<','>']:
            xyz,rgb=cloud_records(fixture_cloud(endian))
            np.testing.assert_allclose(xyz[:,0],[.1,1.1,2.1,3.1])
            np.testing.assert_array_equal(rgb,[[255,0,0],[0,255,0],[0,0,255],[18,52,86]])

    def test_rejects_uncoloured_and_truncated_cloud(self):
        msg=fixture_cloud();msg.fields.pop()
        with self.assertRaisesRegex(ValueError,'no camera'): cloud_records(msg)
        msg=fixture_cloud();msg.data=msg.data[:-1]
        with self.assertRaisesRegex(ValueError,'truncated'): cloud_records(msg)

    def test_binary_contract_voxel_replacement_and_capacity(self):
        archive=VoxelArchive(.1,2)
        archive.add(np.array([[0,0,0],[.01,0,0],[1,2,3]],dtype='f4'),np.array([[12,34,56],[1,2,3],[255,0,128]],dtype='u1'))
        self.assertEqual(len(archive.voxels),2)
        archive.add(np.array([[.02,0,0],[9,9,9]],dtype='f4'),np.array([[9,8,7],[1,2,3]],dtype='u1'))
        self.assertEqual(archive.rejected_capacity,1)
        records=archive.records()
        self.assertEqual(BROWSER_DTYPE.itemsize,16)
        np.testing.assert_array_equal(records['rgb'][0],[9,8,7])
        self.assertEqual(records.tobytes()[12:15],bytes([9,8,7]))
        payload=pcd_bytes(records).split(b'DATA binary\n',1)[1]
        self.assertEqual(struct.unpack_from('<I',payload,12)[0],0x090807)

    def test_atomic_snapshot_is_readable_without_temporary_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'map_metadata.json'
            atomic_write(path,b'{"revision":1}')
            atomic_write(path,b'{"revision":2}')
            self.assertEqual(json.loads(path.read_text())['revision'],2)
            self.assertFalse(path.with_name(path.name+'.tmp').exists())

    def test_replayed_sensor_timestamp_cannot_refresh_health(self):
        holder=SimpleNamespace(stamps={},received={},counts=Counter())
        message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)))
        self.assertTrue(MapArchive.observe(holder,'camera',message))
        original=holder.received['camera']
        self.assertFalse(MapArchive.observe(holder,'camera',message))
        self.assertEqual(holder.received['camera'],original)
        message.header.stamp.sec=9
        self.assertFalse(MapArchive.observe(holder,'camera',message))
        self.assertEqual(holder.counts['camera'],1)
        self.assertEqual(holder.counts['rejected_camera_timestamp'],2)

    def test_save_refuses_stale_or_capacity_truncated_map_and_records_health(self):
        with tempfile.TemporaryDirectory() as directory:
            archive=VoxelArchive(.1,2)
            archive.add(np.array([[0,0,0]],dtype='f4'),np.array([[9,8,7]],dtype='u1'))
            ages={name:.1 for name in ('odom','lidar','imu','full_cloud','camera','colored_cloud')}
            health={'slam_healthy':True,'camera_healthy':True,'message_age':.1,'error':None,
                    'ages':ages,'stamps':{name:10. for name in ages}}
            holder=SimpleNamespace(directory=Path(directory),archive=archive,stamps={'colored_cloud':10.},
                metadata=lambda:health,snapshot=lambda:None,save_state={},last_error=None)
            response=SimpleNamespace(success=False,message='')
            health['camera_healthy']=False
            MapArchive.save(holder,None,response)
            self.assertFalse(response.success)
            self.assertFalse((Path(directory)/'colored_map.pcd').exists())
            health['camera_healthy']=True;archive.rejected_capacity=1
            MapArchive.save(holder,None,response)
            self.assertFalse(response.success)
            self.assertIn('capacity',response.message)
            archive.rejected_capacity=0
            MapArchive.save(holder,None,response)
            self.assertTrue(response.success)
            self.assertTrue(holder.save_state['complete'])
            self.assertEqual(holder.save_state['healthy_sensor_evidence']['stamps']['camera'],10.)
            self.assertEqual(holder.save_state['last_observation_stamp'],10.)

    def test_saved_pcd_and_evidence_remain_fixed_when_navigation_grows_live_map(self):
        with tempfile.TemporaryDirectory() as directory:
            archive=VoxelArchive(.1,100)
            archive.add(np.array([[0,0,0]],dtype='f4'),np.array([[9,8,7]],dtype='u1'))
            sources=('odom','lidar','imu','full_cloud','camera','colored_cloud')
            holder=SimpleNamespace(directory=Path(directory),archive=archive,
                received={name:time.monotonic() for name in sources},
                stamps={name:10. for name in sources},counts=Counter({name:1 for name in sources}),
                revision=1,binary_filename='colored_map.000001.bin',bounds=None,
                started_at=time.time(),snapshot=lambda:None,save_state={},last_error=None,dirty=False)
            holder.metadata=lambda:MapArchive.metadata(holder)
            holder.observe=lambda name,msg:MapArchive.observe(holder,name,msg)
            response=SimpleNamespace(success=False,message='')
            MapArchive.save(holder,None,response)
            self.assertTrue(response.success)
            original_state=json.loads(json.dumps(holder.save_state))
            filename=Path(directory)/'colored_map.pcd'
            original_pcd=filename.read_bytes()
            self.assertIn(b'\nPOINTS 1\n',original_pcd)
            payload=original_pcd.split(b'DATA binary\n',1)[1]
            self.assertEqual(struct.unpack_from('<I',payload,12)[0],0x090807)

            # Exercise the real subsequent cloud callback and timestamp update,
            # as the navigation phase does after the one map-save request.
            cloud=fixture_cloud()
            cloud.header=SimpleNamespace(frame_id='camera_init',stamp=SimpleNamespace(sec=20,nanosec=0))
            MapArchive.cloud(holder,cloud)
            MapArchive.observe(holder,'camera',cloud)
            live=MapArchive.metadata(holder)
            self.assertEqual(live['point_count'],5)
            self.assertEqual(len(archive.records()),5)
            self.assertEqual(live['stamps']['colored_cloud'],20.)
            self.assertEqual(live['stamps']['camera'],20.)
            self.assertEqual(live['save'],original_state)
            self.assertEqual(live['save']['point_count'],len(payload)//16)
            self.assertNotEqual(live['point_count'],live['save']['point_count'])
            self.assertEqual(live['save']['healthy_sensor_evidence']['stamps']['camera'],10.)
            self.assertEqual(filename.read_bytes(),original_pcd)


if __name__=='__main__': unittest.main()
