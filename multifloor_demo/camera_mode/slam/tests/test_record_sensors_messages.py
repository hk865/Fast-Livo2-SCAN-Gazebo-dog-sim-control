"""Execute the actual recorder methods using constructed installed ROS messages.

No rclpy.init, Node, executor, publisher or physics is created.
"""
import ast
import io
import json
from pathlib import Path
import time
import unittest

import numpy as np
from sensor_msgs.msg import Imu, Image, PointCloud2, PointField

SOURCE = Path(__file__).resolve().parents[1]/'record_sensors.py'
tree = ast.parse(SOURCE.read_text())
methods = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == 'SensorAudit')
selected = [n for n in methods.body if isinstance(n, ast.FunctionDef) and n.name in ('write','imu','lidar','image')]
actual = ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='ActualRecorderMethods', bases=[],
    keywords=[], body=selected, decorator_list=[])], type_ignores=[]))
namespace = dict(json=json, time=time)
exec(compile(actual, str(SOURCE), 'exec'), namespace)


class ActualMessageRecorderTests(unittest.TestCase):
    def setUp(self):
        self.node = namespace['ActualRecorderMethods']()
        self.node.stream = io.StringIO()
        self.node.previous = {}

    def stamped(self, msg, nanosec=123456789):
        msg.header.stamp.sec = 17
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = 'actual_frame'
        return msg

    def row(self):
        return json.loads(self.node.stream.getvalue().splitlines()[-1])

    def test_actual_imu_numpy_covariance_available_and_unavailable(self):
        msg = self.stamped(Imu())
        self.assertIsInstance(msg.orientation_covariance[0] != -1, np.bool_)
        msg.orientation.w = 1.
        msg.linear_acceleration.z = 9.81
        self.node.imu(msg)
        row = self.row()
        self.assertIs(row['orientation_available'], True)
        self.assertEqual(row['stamp_ns'], 17123456789)
        self.assertEqual(row['quaternion'], [0.,0.,0.,1.])
        self.assertEqual(row['acceleration'], [0.,0.,9.81])
        msg.header.stamp.nanosec += 1_000_000
        msg.orientation_covariance[0] = -1.
        self.node.imu(msg)
        row = self.row()
        self.assertIs(row['orientation_available'], False)
        self.assertEqual(row['stamp_delta_ns'], 1_000_000)

    def test_actual_lidar_fields_and_camera_payload_count_serialize(self):
        cloud = self.stamped(PointCloud2())
        cloud.width, cloud.height, cloud.point_step, cloud.row_step = 2,1,16,32
        cloud.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)]
        cloud.data = bytes(range(32))
        self.node.lidar(cloud)
        row = self.row()
        self.assertEqual(row['fields'], ['x'])
        self.assertEqual(row['actual_payload_bytes'], 32)
        self.assertEqual(row['points'], 2)
        image = self.stamped(Image())
        image.width, image.height, image.step = 2,2,6
        image.encoding = 'rgb8'
        image.data = bytes(range(12))
        self.node.image(image)
        row = self.row()
        self.assertEqual(row['actual_payload_bytes'], 12)
        self.assertEqual(row['encoding'], 'rgb8')
        self.assertEqual(row['stamp_ns'], 17123456789)


if __name__ == '__main__':
    unittest.main()
