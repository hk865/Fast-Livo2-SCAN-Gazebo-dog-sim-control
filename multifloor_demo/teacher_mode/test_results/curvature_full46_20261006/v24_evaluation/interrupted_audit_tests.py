import copy,json,tempfile,unittest
from pathlib import Path
import audit_interrupted_v24 as partial

class PartialChecks(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.audit=partial.adapter.load_adapter()
 def fixture(self):
  raw=dict(goal_id='exploration:9',center=[0.,0.,1.2],arrival=dict(type='oriented_box',dwell_sim_s=.4,half_extents_m=[.35,.3,.1],control_band=dict(type='oriented_box',half_extents_m=[.25,.2,.07])),timeout_sim_s=90.)
  g=self.audit.parse_goal(raw);poses={t:dict(position=[0.,0.,1.2],provenance={})for t in (1_000_000_000,1_200_000_000,1_400_000_000)}
  r=dict(request_id='r',goal_id=g.goal_id,goals_definition_sha256='h',waypoint_index=9,start_stamp_ns=1_000_000_000,stamp_ns=1_400_000_000,goal_activated_ros_clock_ns=0,dwell_ns=400_000_000,raw_position=[0.,0.,1.2],arrival_definition=g.definition()['arrival'],control_arrival_definition=g.control_arrival_definition(),protected=False,region_inside=True,control_region_inside=True,reason='arrived')
  return raw,r,poses
 def check(self,raw,r,poses):return partial.verify_dwell(self.audit,raw,r,poses,sorted(poses),'r','h')
 def test_original_3D_dwell_pass(self):
  a,r,p=self.fixture();self.assertTrue(self.check(a,r,p)['passed'])
 def test_height_error_does_not_pass_even_at_correct_XY(self):
  a,r,p=self.fixture();p[1_200_000_000]['position'][2]=1.38
  self.assertFalse(self.check(a,r,p)['checks']['original_3D_control_region'])
 def test_gap_greater_than_original_200ms_fails(self):
  a,r,p=self.fixture();p.pop(1_200_000_000)
  self.assertFalse(self.check(a,r,p)['checks']['source_gap'])
 def test_missing_boundary_or_terminal_raw_pose_fails(self):
  for kind in ('start','end','raw'):
   a,r,p=self.fixture()
   if kind=='start':p.pop(1_000_000_000)
   elif kind=='end':p.pop(1_400_000_000)
   else:r['raw_position']=[.01,0.,1.2]
   self.assertFalse(self.check(a,r,p)['passed'])
 def test_short_dwell_or_original_deadline_failure(self):
  a,r,p=self.fixture();r['start_stamp_ns']=1_200_000_000;r['dwell_ns']=200_000_000
  self.assertFalse(self.check(a,r,p)['checks']['original_dwell'])
  a,r,p=self.fixture();r['goal_activated_ros_clock_ns']=-90_000_000_000
  self.assertFalse(self.check(a,r,p)['checks']['original_deadline'])
 def test_wrong_original_geometry_or_identity_fails(self):
  for field,value in (('arrival_definition',{}),('request_id','foreign'),('protected',True)):
   a,r,p=self.fixture();r[field]=value;self.assertFalse(self.check(a,r,p)['passed'])
 def test_stream_hash_collected_in_same_pass(self):
  with tempfile.TemporaryDirectory()as d:
   p=Path(d)/'rows.jsonl';p.write_text('{"x": 1}\n\n{"x": 2}\n');receipts={};rows=list(partial.rows_once(p,receipts))
   self.assertEqual(len(rows),2);self.assertEqual(receipts[str(p)]['read_passes'],1);self.assertEqual(receipts[str(p)]['sha256'],partial.sha(p));self.assertEqual(rows[1][1]['source_offset'],10)
if __name__=='__main__':unittest.main(verbosity=2)
