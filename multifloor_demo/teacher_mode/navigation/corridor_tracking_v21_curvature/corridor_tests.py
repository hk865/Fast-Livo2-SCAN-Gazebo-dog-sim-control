"""Finite synthetic rejection checks, never evidence of physical safety."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from corridor import certify_corridor, derive_measured_support, request_prefix


def fixture():
    now=10_000_000_000;res=.08;origin=np.array([-15,-14,0]);shape=np.array([48,28,16])
    points=np.array([[0.,0.,.7],[1.5,0.,.7]])
    path=dict(points_xyz=points.tolist(),path_id='fixture:1',path_sha256=hashlib.sha256(points.tobytes()).hexdigest(),
        stamp_ns=now-100_000_000,request_stamp_ns=now-10_000_000,frame_id='camera_init',layer_id='floor_fixture')
    xy=np.array([[x,y]for x in np.arange(-1.16,2.6,.04)for y in np.arange(-1.08,1.09,.04)])
    surface=np.c_[xy,np.full(len(xy),.3)]
    snapshot=dict(schema='teacher_scan_local_snapshot/v1',source='actual_registered_lidar_raycast',
        frame_id='camera_init',revision=3,stamp_ns=now,source_cloud_stamp_ns=now,
        source_sensor_pose_stamp_ns=now,request_stamp_ns=path['request_stamp_ns'],complete=True,
        resolution_m=res,origin_index_xyz=origin.tolist(),shape_xyz=shape.tolist(),
        states='1'*int(np.prod(shape)),cell_observation_age_ms=[0]*int(np.prod(shape)),
        surface_points_xyz=surface.tolist(),surface_points_complete=True,
        raw_occupancy_not_robot_inflated=True,navigation_ground_truth_used=False)
    state=dict(position_world_xyz=[.15,0.,.7],yaw_rad=0.,speed_mps=.2,progress_m=.15,stamp_ns=now)
    return path,snapshot,state,{},now


def alter_cells(snapshot, fn, value):
    shape=np.array(snapshot['shape_xyz']);origin=np.array(snapshot['origin_index_xyz']);res=snapshot['resolution_m']
    states=np.array(list(snapshot['states'])).reshape(tuple(shape))
    for local in np.ndindex(*shape):
        point=(origin+local+.5)*res
        if fn(point):states[local]=value
    snapshot['states']=''.join(states.ravel())


class CorridorTests(unittest.TestCase):
    def evaluate(self,fixture_data):
        p,m,s,l,n=fixture_data
        return certify_corridor(p,m,None,s,l,n)

    def test_supported_free_fixture_and_no_mutation(self):
        data=fixture();before=copy.deepcopy(data);result=self.evaluate(data)
        self.assertEqual(result['status'],'certified',result.get('first_rejection',result))
        self.assertTrue(result['shadow_only']);self.assertEqual(data,before)
        self.assertGreater(result['left_m'],0);self.assertGreater(result['right_m'],0)
        self.assertLessEqual(result['valid_until_ns'],data[-1]+300_000_000)

    def test_unknown_is_not_free(self):
        data=fixture();alter_cells(data[1],lambda p:.5<p[0]<.7 and abs(p[1])<.1 and .55<p[2]<.8,'0')
        result=self.evaluate(data);self.assertEqual(result['status'],'unavailable')
        self.assertIn('unknown_body_sweep',result['failure_counts'])

    def test_obstacle_and_asymmetric_corridor(self):
        data=fixture();alter_cells(data[1],lambda p:p[1]>.70 and p[2]>.5,'2')
        result=self.evaluate(data);self.assertEqual(result['status'],'certified',result)
        self.assertLess(result['left_m'],result['right_m'])
        alter_cells(data[1],lambda p:.6<p[0]<.8 and abs(p[1])<.15 and .55<p[2]<.8,'2')
        result=self.evaluate(data);self.assertEqual(result['status'],'blocked')

    def test_rotation_sweep_is_not_center_point_check(self):
        data=fixture();alter_cells(data[1],lambda p:p[1]>.4 and p[2]>.5,'2')
        self.assertEqual(self.evaluate(data)['status'],'blocked')
        data[3]['yaw_half_range_rad']=.01
        result=self.evaluate(data);self.assertEqual(result['status'],'certified',result)
        data[2]['yaw_rad']=math.pi/2
        self.assertEqual(self.evaluate(data)['reason'],'actual_yaw_outside_verified_sweep')

    def test_missing_stale_future_and_incomplete_map(self):
        p,m,s,l,n=fixture()
        self.assertEqual(certify_corridor(p,None,None,s,l,n)['status'],'unavailable')
        for field in ('stamp_ns','source_cloud_stamp_ns','source_sensor_pose_stamp_ns'):
            x=copy.deepcopy(m);x[field]=n-300_000_001
            self.assertEqual(certify_corridor(p,x,None,s,l,n)['status'],'unavailable')
            x[field]=n+1
            self.assertEqual(certify_corridor(p,x,None,s,l,n)['status'],'unavailable')
        x=copy.deepcopy(m);x['complete']=False
        self.assertEqual(certify_corridor(p,x,None,s,l,n)['status'],'unavailable')

    def test_free_cell_observation_age_cannot_be_refreshed_by_new_snapshot(self):
        data=fixture();data[1]['cell_observation_age_ms']=[301]*len(data[1]['states'])
        result=self.evaluate(data)
        self.assertEqual(result['status'],'unavailable');self.assertIn('stale_free_body_sweep',result['failure_counts'])

    def test_surface_absence_truncation_floor_alias_and_edge(self):
        data=fixture();data[1]['surface_points_complete']=False
        self.assertEqual(self.evaluate(data)['status'],'unavailable')
        data=fixture();data[1]['surface_points_xyz']=[]
        self.assertEqual(self.evaluate(data)['status'],'unavailable')
        data=fixture();data[1]['surface_points_xyz']=[[x,y,z+2.]for x,y,z in data[1]['surface_points_xyz']]
        result=self.evaluate(data);self.assertEqual(result['status'],'unavailable')
        self.assertGreater(result['support_diagnostic']['failure_counts'].get('insufficient_measured_points',0),0)
        data=fixture();data[1]['surface_points_xyz']=[p for p in data[1]['surface_points_xyz']if p[1]>.1]
        self.assertEqual(self.evaluate(data)['status'],'unavailable')

    def test_rough_surface_and_slope_rejected(self):
        data=fixture();data[1]['surface_points_xyz']=[[x,y,z+(.05 if i%2 else -.05)]
            for i,(x,y,z)in enumerate(data[1]['surface_points_xyz'])]
        result=self.evaluate(data);self.assertEqual(result['status'],'unavailable')
        self.assertGreater(result['support_diagnostic']['failure_counts'].get('support_plane_residual',0),0)
        data=fixture();data[1]['surface_points_xyz']=[[x,y,z+.55*y]for x,y,z in data[1]['surface_points_xyz']]
        result=self.evaluate(data);self.assertEqual(result['status'],'unavailable')
        self.assertGreater(result['support_diagnostic']['failure_counts'].get('support_slope',0),0)

    def test_stopping_distance_and_pose_progress_binding(self):
        data=fixture();data[2]['speed_mps']=1.
        self.assertEqual(self.evaluate(data)['reason'],'verified_prefix_shorter_than_stopping_distance')
        data=fixture();data[2]['position_world_xyz'][0]=1.4
        self.assertEqual(self.evaluate(data)['reason'],'actual_state_outside_corridor')

    def test_bad_bindings_and_limits(self):
        data=fixture();data[1]['request_stamp_ns']+=1
        self.assertEqual(self.evaluate(data)['reason'],'map_request_binding_mismatch')
        data=fixture();data[3]['map_max_age_ns']=400_000_000
        self.assertEqual(self.evaluate(data)['status'],'unavailable')
        data=fixture();data[3]['body_radius_m']=.01
        self.assertEqual(self.evaluate(data)['status'],'unavailable')
        data=fixture();data[1]['states']=data[1]['states'][:-1]
        self.assertEqual(self.evaluate(data)['status'],'unavailable')

    def test_support_cannot_be_reused_across_map_or_layer(self):
        p,m,s,l,n=fixture();support=derive_measured_support(m,p,l)
        m['revision']+=1
        self.assertEqual(certify_corridor(p,m,support,s,l,n)['status'],'unavailable')
        m['revision']-=1;p['layer_id']='other'
        self.assertEqual(certify_corridor(p,m,support,s,l,n)['status'],'unavailable')

    def test_request_prefix_bounded_and_at_actual_progress(self):
        p,_,_,_,_=fixture();prefix=request_prefix(p,.2,.7)
        self.assertTrue(np.allclose(prefix[0],[.2,0,.7]));self.assertTrue(np.allclose(prefix[-1],[.9,0,.7]))
        self.assertLessEqual(len(prefix),128)


if __name__=='__main__':
    unittest.main(verbosity=2)
