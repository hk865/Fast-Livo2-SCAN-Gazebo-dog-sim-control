"""An observation hook must not turn a sent request into an unacknowledged retry."""
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from planning.reference.automatic_reference_runtime import AutomaticReferenceRuntime
from planning.admission import path_handoff

class Publisher:
    def __init__(self):self.messages=[]
    def get_subscription_count(self):return 1
    def publish(self,message):self.messages.append(message)
class Evidence:
    def append(self,*args,**kwargs):pass

class ReplanAck(unittest.TestCase):
    def owner(self):
        runtime=AutomaticReferenceRuntime.__new__(AutomaticReferenceRuntime)
        runtime.run=Path('/tmp/replan-ack-test');runtime.replan_sequence=0
        runtime.reference={'sent_reference':{'stamp':[1,0]}}
        runtime._ready_scope=lambda owner:((2,123,1),2_000_000_000,1_990_000_000)
        runtime.load=lambda epoch,generation:{}
        registered=[]
        association=NS(observed_reference_revision=1,reference_revision=1,
            note_local_replan_request=registered.append,reference_stamp=[1,0],accepted=object())
        runtime.association=association
        owner=NS(run=runtime.run,reference_pub=Publisher(),local_replan_pub=Publisher(),evidence=Evidence(),
            make_local_replan_message=lambda *args:args,trajectory_association=association,
            reference_requests=1,automatic_reference=runtime,request_id='run:goal',waypoint_index=0,
            known_scene_inputs={'generation':1},continuous_route=None,pose_stamp=1_990_000_000,
            heading_gate=NS(phase='drive'),cascade=NS(capture_stamp=None,path_id='path1',spatial_config={'arc_length_m':.2}),
            path_handoff_early_requested_for=None)
        owner.request_plan=lambda:runtime.request_plan(owner)
        return owner,runtime,registered
    def test_publish_returns_ack(self):
        owner,runtime,registered=self.owner()
        with patch('planning.reference.automatic_reference_runtime.EARLY_REQUEST_ACK_ENABLED',True):
            self.assertIs(runtime.request_plan(owner),True)
        self.assertEqual(registered,[1]);self.assertEqual(len(owner.local_replan_pub.messages),1)
    def test_same_path_tail_requests_once(self):
        owner,runtime,registered=self.owner()
        reference=dict(remaining_m=.1,endpoint_is_fixed_goal=False)
        with patch('planning.reference.automatic_reference_runtime.EARLY_REQUEST_ACK_ENABLED',True), \
             patch.object(path_handoff,'EARLY_REQUEST_ACK_ENABLED',True), \
             patch.object(path_handoff,'context',return_value={'allowed':True}), \
             patch('planning.admission.pending_path_commit.waiting',return_value=False):
            path_handoff.early_request(owner,reference)
            path_handoff.early_request(owner,reference)
        self.assertEqual(registered,[1]);self.assertEqual(len(owner.local_replan_pub.messages),1)
if __name__=='__main__':unittest.main()
