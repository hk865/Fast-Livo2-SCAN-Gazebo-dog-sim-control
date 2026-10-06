"""Optional sensor-frame sensitivity; all orientation inputs are logged sensors.

The original map and first observer use yaw-only axes. This separate offline
test evaluates full up-axis calibration, without changing either reference.
"""
import numpy as np


def expected_plane_height(initial_floor_at_anchor_xy, anchor_xy, body_xy,
                          elevation, camera_init_map_up):
    n=np.asarray(camera_init_map_up,dtype=np.float64)
    if n.shape!=(3,) or not np.isfinite(n).all() or np.linalg.norm(n)<1e-8:
        raise ValueError('invalid observed map up')
    n=n/np.linalg.norm(n)
    if n[2]<.98:raise ValueError('up calibration not reliable')
    anchor=np.asarray(anchor_xy,dtype=float);xy=np.asarray(body_xy,dtype=float)
    return float(initial_floor_at_anchor_xy + (elevation-n[:2]@(xy-anchor))/n[2])


def check():
    return {'horizontal_identity':abs(expected_plane_height(-.3,[0,0],[1,2],1.2,[0,0,1])-.9)<1e-12,
            'tilt_coordinate_effect':abs(expected_plane_height(-.3,[0,0],[0,10],1.2,[0,.01,.99995])-(-.3+1.1/.99995))<1e-7}
