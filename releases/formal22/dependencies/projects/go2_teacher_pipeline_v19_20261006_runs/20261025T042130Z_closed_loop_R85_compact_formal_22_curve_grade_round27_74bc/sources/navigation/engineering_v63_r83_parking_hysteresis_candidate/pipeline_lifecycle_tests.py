import copy,unittest
from pipeline_lifecycle import check_summary
class LifecycleTests(unittest.TestCase):
 def good(self):return dict(schema='staged_input_pipeline_v19/v1',normal_completed=True,context_valid_at_drain=True,accepted=12,delivered=12,committed=12,pending=0,inflight=0,ready=0,bytes=0,canceled=0,rejected_capacity=0,closed_rejections=0,failure='',uncommitted_packets=[])
 def test_complete(self):self.assertEqual(check_summary(self.good()),[])
 def test_each_missing_field(self):
  for key in self.good():
   d=self.good();d.pop(key)
   with self.subTest(key=key):self.assertTrue(check_summary(d))
 def test_original_v17_cancel_not_waived(self):
  d=self.good();d.update(accepted=31615,delivered=31614,committed=31614,canceled=1)
  self.assertIn('accepted_delivered_committed',check_summary(d));self.assertIn('canceled',check_summary(d))
 def test_producer_flag_does_not_override_reader(self):
  for key in ('pending','inflight','ready','bytes','canceled','rejected_capacity','closed_rejections'):
   d=self.good();d[key]=1
   with self.subTest(key=key):self.assertIn(key,check_summary(d))
 def test_early_context_shutdown(self):
  d=self.good();d['context_valid_at_drain']=False;self.assertIn('context_valid_at_drain',check_summary(d))
 def test_event_identity_and_failures(self):
  d=self.good();d['uncommitted_packets']=[dict(seq=13,kind=0,reason='abort')];self.assertIn('uncommitted_packets',check_summary(d))
  d=self.good();d['failure']='decoder';self.assertIn('failure',check_summary(d))
 def test_bool_is_not_count(self):
  d=self.good();d.update(accepted=True,delivered=True,committed=True);self.assertIn('accepted_delivered_committed',check_summary(d))
if __name__=='__main__':unittest.main()
