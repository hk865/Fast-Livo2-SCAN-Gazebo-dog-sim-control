"""Small pure differential tests; no archived-job reruns or ROS imports."""
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np
from candidate import hull_binary64, EXPECTED_FROZEN_SHA256

SOURCE = Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_130405_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r1_39ea/sources/navigation/corridor_tracking_v20/corridor.py')
if hashlib.sha256(SOURCE.read_bytes()).hexdigest() != EXPECTED_FROZEN_SHA256:
    raise RuntimeError('Frozen hull source changed')
spec = importlib.util.spec_from_file_location('reference_hull', SOURCE)
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


class HullEquivalence(unittest.TestCase):
    def same(self, points):
        points = np.asarray(points, dtype=np.float64).reshape((-1, 2))
        a, b = reference._hull(points), hull_binary64(points)
        self.assertEqual(a, b)
        # Signed zeros and output numeric serialization must also remain exact.
        self.assertEqual(json.dumps(a), json.dumps(b))

    def test_degenerate_and_boundary_sets(self):
        for points in ([], [[0., 0.]], [[0., 0.]]*8,
                [[-0., 0.], [0., -0.], [1., 0.], [1., 1.], [0., 1.]],
                [[x, x] for x in range(-8, 9)],
                [[x, np.nextafter(float(x), float('inf'))] for x in range(-8, 9)]):
            self.same(points)

    def test_seeded_finite_float64_sets(self):
        rng = np.random.default_rng(20261006)
        for size in (3, 6, 12, 32):
            for _ in range(20):
                points = rng.uniform(-.3, .3, (size, 2))
                self.same(points)
                self.same(np.vstack((points, points[:2])))


if __name__ == '__main__':
    unittest.main()
