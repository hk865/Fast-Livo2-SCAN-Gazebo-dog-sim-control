"""Evidence adequacy checks: omitted callback zero must never pass exact replay."""
import copy,importlib.util,json,tempfile,unittest
from pathlib import Path

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('publication_audit',HERE/'audit_publication_ledger.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)

class EvidenceTests(unittest.TestCase):
    def fixture(self):
        ledger=[]
        def pub(clock,before,after,pidseq,trigger):
            prior=ledger[-1]if ledger else None;seq=len(ledger)+1
            ledger.append(dict(schema='teacher_actual_command_publication/v1',sequence=seq,
                entry_monotonic_wall_ns=100_000_000_000+clock,publish_started_monotonic_wall_ns=100_000_000_000+clock+1000,
                monotonic_wall_ns=100_000_000_000+clock+2000,publish_ros_clock_ns=clock,
                command_before=before,command_after=after,commands_published_count=seq,last_command_time_before=None if prior is None else prior['last_command_time_after'],
                last_publication_ros_clock_ns_before=None if prior is None else prior['publish_ros_clock_ns'],last_command_time_after=clock/1e9,
                associated_pid_sequence=pidseq,trigger=trigger,stopped_argument=pidseq is None,heading_phase='drive',heading_reference=0.,state_before='running',state_after='running',
                protection_flags_before={},protection_flags_after={},navigation_ground_truth_used=False))
        pub(0,[0,0,0],[0,0,0],None,'control')
        pub(1_000_000_000,[0,0,0],[.06,0,0],1,'control')
        pub(1_040_000_000,[.06,0,0],[0,0,0],None,'on_imu')
        pub(1_100_000_000,[0,0,0],[.036,0,0],2,'control')
        pid=[dict(sequence=1,compute_ros_clock_ns=1_000_000_000,publish_ros_clock_ns=1_000_000_000,compute_monotonic_wall=101.,
                desired_body_command=[.2,0,0],prepared_after_slew_command=[.06,0,0],command_after_slew=[.06,0,0],cascade={'mode':'drive'},stopped=False),
            dict(sequence=2,compute_ros_clock_ns=1_100_000_000,publish_ros_clock_ns=1_100_000_000,compute_monotonic_wall=101.1,
                desired_body_command=[.2,0,0],prepared_after_slew_command=[.036,0,0],command_after_slew=[.036,0,0],cascade={'mode':'drive'},stopped=False)]
        return pid,ledger
    def evaluate(self,pid,ledger,count=4):
        with tempfile.TemporaryDirectory()as td:
            r=Path(td);(r/'navigation_pid_writer_receipt.json').write_text(json.dumps({'expected_command_publication_records':count,'final_commands_published_count':4}))
            return audit.publication_ledger_audit(r,pid,[],ledger)
    def test_actual_callback_zero_restores_exact_slew_replay(self):
        p,l=self.fixture();integrity,slew=self.evaluate(p,l)
        self.assertTrue(integrity['passed']);self.assertTrue(slew['passed'])
        self.assertFalse(audit.publication_slew_PID_hold_only(p,[])['passed'])
    def test_missing_callback_evidence_does_not_pass(self):
        p,l=self.fixture();l.pop(2);integrity,slew=self.evaluate(p,l)
        self.assertFalse(integrity['passed']);self.assertFalse(slew['passed'])
    def test_receive_clock_cannot_replace_publisher_anchor(self):
        p,l=self.fixture();l[2]['publish_ros_clock_ns']+=5_000_000
        integrity,slew=self.evaluate(p,l);self.assertFalse(integrity['passed']);self.assertFalse(slew['passed'])
    def test_nonzero_without_math_row_is_rejected(self):
        p,l=self.fixture();l[2]['command_after']=[.01,0,0]
        integrity,slew=self.evaluate(p,l);self.assertFalse(integrity['passed']);self.assertFalse(slew['passed'])
    def test_writer_self_consistency_cannot_override_original_published_counter(self):
        p,l=self.fixture();l.pop(2)
        for i,d in enumerate(l):d['sequence']=i+1
        integrity,slew=self.evaluate(p,l,count=3);self.assertFalse(integrity['passed']);self.assertFalse(slew['passed'])

if __name__=='__main__':unittest.main()
