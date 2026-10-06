#!/usr/bin/env python3
import struct
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from self_echo_filter import filter_records,self_mask


class SelfEchoTests(unittest.TestCase):
    def test_only_known_trunk_and_leg_envelope_removed(self):
        points=np.array([[0,0,0],[.4,.2,-.2],[.5,0,0],[0,.3,-.2],[0,0,-.4],[.25,0,.25]])
        np.testing.assert_array_equal(self_mask(points),[True,True,False,False,False,False])

    def test_fixed_lidar_mount_rotation_and_complete_field_bytes(self):
        lidar={'body_xyz':[.2,0,.1177],'body_rpy':[0,0,np.pi/2]}
        body=np.array([[0,0,0],[1,0,0],[0,.4,-.2],[.4,.2,-.2]])
        rotation=Rotation.from_euler('xyz',lidar['body_rpy']).as_matrix()
        xyz=(body-np.array(lidar['body_xyz']))@rotation
        for endian in ('<','>'):
            rows=[struct.pack(endian+'fffI',*point,0x123400+i) for i,point in enumerate(xyz)]
            msg=SimpleNamespace(width=2,height=2,point_step=16,row_step=36,is_bigendian=endian=='>',
                data=rows[0]+rows[1]+b'pad!'+rows[2]+rows[3]+b'pad!',
                fields=[SimpleNamespace(name=name,datatype=7,count=1,offset=i*4) for i,name in enumerate(['x','y','z'])])
            payload,removed,total=filter_records(msg,lidar)
            self.assertEqual((removed,total),(2,4))
            self.assertEqual(payload,rows[1]+rows[2])

    def test_rejects_bad_geometry_layout(self):
        msg=SimpleNamespace(fields=[])
        with self.assertRaisesRegex(ValueError,'scalar float32'):filter_records(msg,{'body_xyz':[0,0,0],'body_rpy':[0,0,0]})


if __name__=='__main__':unittest.main()
