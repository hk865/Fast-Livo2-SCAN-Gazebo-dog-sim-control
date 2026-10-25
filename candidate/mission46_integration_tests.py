"""Finite fail-closed mission/controller tests; not navigation acceptance."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import sys
import numpy as np
from mission46_guard import mission_hold


class MissionGuardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.run = Path(self.tmp.name)
        self.control = dict(run_id=self.run.name, current_request='test:exploration:1',
                            sim_ns=1000000000, monotonic_wall=10., hold=False,
                            navigation_ground_truth_used=False)
        self.arrival = dict(goal_id='exploration:11', stamp_ns=900000000)
        self.evidence = self.run/'evidence.json'
        self.evidence.write_text('{}\n')
        self.ack = dict(run_id=self.run.name, request_id=self.control['current_request'],
                        goal_id=self.arrival['goal_id'], arrival_receipt=self.arrival,
                        goals_definition_sha256='goalhash', from_layer='lower12', to_layer='upper23',
                        passed=True, navigation_ground_truth_used=False, effective_sim_ns=950000000,
                        source_evidence_file=str(self.evidence),
                        source_evidence_sha256=hashlib.sha256(self.evidence.read_bytes()).hexdigest())
        self.write()

    def tearDown(self):
        self.tmp.cleanup()

    def write(self):
        (self.run/'mission46_control.json').write_text(json.dumps(self.control))
        (self.run/'mission46_terrain_ack.json').write_text(json.dumps(self.ack))

    def gate(self, arrivals=None, now=1000000000, wall=10.):
        return mission_hold(self.run, self.control['current_request'],
                            [self.arrival] if arrivals is None else arrivals, 'goalhash', now, wall)[0]

    def test_fresh_release_and_connector_ack(self):
        self.assertFalse(self.gate([]));self.assertFalse(self.gate())

    def test_source_expiry_or_future(self):
        for now, wall in ((1300000001, 10.), (999999999, 10.), (1000000000, 10.301), (1000000000, 9.9)):
            with self.subTest(now=now, wall=wall):self.assertTrue(self.gate(now=now, wall=wall))

    def test_explicit_hold(self):
        self.control['hold']=True;self.write();self.assertTrue(self.gate([]))

    def test_missing_source_declaration(self):
        del self.control['navigation_ground_truth_used'];self.write();self.assertTrue(self.gate([]))

    def test_missing_ack(self):
        (self.run/'mission46_terrain_ack.json').unlink();self.assertTrue(self.gate())

    def test_wrong_request_layer_goal_or_future_ack(self):
        original=copy.deepcopy(self.ack)
        for k,v in (('request_id','wrong'),('from_layer','upper23'),('goal_id','wrong'),
                    ('effective_sim_ns',1000000001),('navigation_ground_truth_used',True),('passed',False)):
            with self.subTest(k=k):
                self.ack=copy.deepcopy(original);self.ack[k]=v;self.write();self.assertTrue(self.gate())

    def test_evidence_tamper(self):
        self.evidence.write_text('{"tampered":true}');self.assertTrue(self.gate())

    def test_geometry_at_origin_and_shared_landing(self):
        root=Path(__file__).resolve().parents[2]
        sys.path.insert(0,str(root/'policy'))
        from observation import TerrainHeightMap,SCAN_GRID_XY
        world=root.parent/'simulation/generated/three_floors.sdf'
        lower=TerrainHeightMap.from_sdf(world,include_models=['floor_1','ramp_12','floor_2'])
        upper=TerrainHeightMap.from_sdf(world,include_models=['floor_2','ramp_23','floor_3'])
        origin=np.column_stack((SCAN_GRID_XY,np.full(187,20.3)))
        z,names=lower.raycast(origin)
        self.assertTrue(np.isfinite(z).all());self.assertTrue(np.allclose(z,0,rtol=0,atol=1e-9))
        self.assertTrue(all(x.split('/')[0]=='floor_1' for x in names))
        for yaw in (0.,np.pi/2,np.pi,-np.pi/2):
            c,s=np.cos(yaw),np.sin(yaw)
            xy=SCAN_GRID_XY@np.array([[c,s],[-s,c]])+[16,4.5]
            starts=np.column_stack((xy,np.full(187,21.6)))
            a,an=lower.raycast(starts);b,bn=upper.raycast(starts)
            self.assertTrue(np.isfinite(a).all() and np.isfinite(b).all())
            self.assertLessEqual(float(np.max(np.abs(a-b))),1e-9)
            self.assertTrue(all(n.split('/')[0]=='floor_2' for n in an+bn))

    def test_provider_bidirectional_and_bad_landing(self):
        root=Path(__file__).resolve().parents[2]
        sys.path.insert(0,str(root/'policy'));sys.path.insert(0,str(root.parent/'navigation'))
        import observation,goal_regions
        from mission46_terrain import MissionTerrainProvider,TRANSITIONS,LAYERS
        def fixture():
            p=MissionTerrainProvider.__new__(MissionTerrainProvider)
            p.run=self.run;p.profile={'experiment':'finite_only'};p.failure=None;p.current_id='initial'
            p.state=np.zeros(64);p.state[0]=1.;p.state[1:4]=[16,4.5,1.6];p.state[4]=1.
            p.world=root.parent/'simulation/generated/three_floors.sdf'
            p.maps={k:observation.TerrainHeightMap.from_sdf(p.world,include_models=models) for k,models in
                    [('initial',['floor_1','ramp_12','floor_2']),('alternate',['floor_2','ramp_23','floor_3'])]}
            p.observation=observation;p.goal_regions=goal_regions;p.completed_transitions=[]
            p.manifest_hashes={'initial':'fixture','alternate':'fixture'};p._record=lambda row:row
            return p
        def request(p, phase):
            gid,before,after=TRANSITIONS[phase]
            rid=self.run.name+':'+gid.split(':')[0]+':1'
            doc=dict(schema_version=2,request_id=rid,frame_id='camera_init',goals=[dict(goal_id=gid,
                center=[16.,4.5,1.6],arrival=dict(type='disc_prism',radius_m=.35,height_half_span_m=.1,
                dwell_sim_s=.4,control_band=dict(type='disc_prism',radius_m=.25,height_half_span_m=.1)),timeout_sim_s=90.)])
            _,goals=goal_regions.parse_request(doc);gh=goal_regions.definitions_sha256(goals)
            arrival=dict(goal_id=gid,request_id=rid,goals_definition_sha256=gh,
                stamp_ns=900000000,start_stamp_ns=400000000,dwell_ns=500000000,max_observation_gap_ns=50000000,
                protected=False,reason='arrived',region_inside=True,control_region_inside=True,raw_position=[16.,4.5,1.6])
            status=dict(request_id=rid,state='running',waypoint_index=1,total=1,frame_id='camera_init',
                goals_definition_sha256=gh,goals_definitions=[g.definition() for g in goals],region_arrivals=[arrival],
                navigation_ground_truth_used=False,mode='teacher',scope='finite_only')
            intent=dict(phase=phase,run_id=self.run.name,request_id=rid,goal_id=gid,
                goals_definition_sha256=gh,arrival_receipt=arrival,from_layer=before,to_layer=after)
            for name,d in [('navigation_request.json',doc),('navigation_status.json',status),('mission46_terrain_request.json',intent)]:
                (self.run/name).write_text(json.dumps(d))
            return after
        p=fixture()
        for phase in ('exploring','returning','navigating'):
            after=request(p,phase);self.assertFalse(p.check_switch(),p.failure)
            self.assertEqual(p.current_id,LAYERS[after]);self.assertFalse(p.check_switch())
        self.assertEqual(len(p.completed_transitions),3)
        q=fixture();request(q,'exploring');q.state[1]=8.
        self.assertTrue(q.check_switch());self.assertIsNotNone(q.failure)
        self.assertEqual(q.current_id,'initial')


if __name__=='__main__':unittest.main()
