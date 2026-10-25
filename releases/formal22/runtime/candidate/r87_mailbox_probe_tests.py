"""No-node checks of observable mailbox boundaries and disabled probe behavior."""
import threading
import unittest
from unittest.mock import patch

import r87_diagnostic_probe as diag
import r86_prepared_feedback as mailbox
from point_provenance import verify_corrected
from r86_prepared_feedback_tests import feed, packet


class MailboxDiagnosticTests(unittest.TestCase):
    def test_disabled_probe_never_samples_or_formats_scope(self):
        with patch.object(diag, 'RECORDER', None), \
             patch.object(diag, 'sample', side_effect=AssertionError('disabled sample')), \
             patch.object(mailbox, '_scope_text', side_effect=AssertionError('disabled scope')):
            box = mailbox.ros_mailbox(verify_corrected)
            box.set_scope(b'a' * 32)
            feed(box, packet())
            batch = box.take_batch()
            self.assertEqual(len(batch), 1)
            self.assertEqual(batch[0].scopes, (b'a' * 32,) * 3)
            box.committed(batch[0], 123)
            box.release_batch(batch)
            self.assertEqual(box.snapshot()['retained_bytes'], 0)

    def test_enqueue_brackets_observe_real_queue_and_batch_is_fixed(self):
        recorder = diag.Recorder(capacity=4096)
        with patch.object(diag, 'RECORDER', recorder):
            box = mailbox.ros_mailbox(verify_corrected)
            box.set_scope(b'b' * 32)
            observed_queue_sizes = []
            original_mark = diag.mark
            def inspect(stage, source_ns=None, **fields):
                if stage in ('mailbox.ready_append_before', 'mailbox.ready_append_after'):
                    observed_queue_sizes.append((stage, source_ns, len(box.ready)))
                return original_mark(stage, source_ns, **fields)
            with patch.object(diag, 'mark', side_effect=inspect):
                feed(box, packet(1_000_000_000, 1))
                feed(box, packet(1_100_000_000, 2))
                first = box.take_batch()
                feed(box, packet(1_200_000_000, 3))
                second = box.take_batch()
            self.assertEqual([row[2] for row in observed_queue_sizes], [0, 1, 1, 2, 0, 1])
            self.assertEqual([b.source_ns for b in first], [1_000_000_000, 1_100_000_000])
            self.assertEqual([b.source_ns for b in second], [1_200_000_000])
            members = [r for r in recorder.rows if r['stage'] == 'mailbox.batch_member']
            self.assertEqual([(r['batch_id'], r['index'], r['source_ns']) for r in members],
                [(1, 0, 1_000_000_000), (1, 1, 1_100_000_000), (2, 0, 1_200_000_000)])
            for row in members:
                self.assertEqual(row['scope_body'], (b'b' * 32).hex())
                self.assertFalse(any(isinstance(v, bytes) for v in row.values()))
            self.assertEqual(recorder.error_count, 0)
            box.release_batch(first)
            box.release_batch(second)
            self.assertEqual(box.snapshot()['retained_bytes'], 0)

    def test_lock_wait_boundary_precedes_actual_lock_acquisition(self):
        recorder = diag.Recorder(capacity=512)
        waiting = threading.Event()
        finished = threading.Event()
        with patch.object(diag, 'RECORDER', recorder):
            box = mailbox.ros_mailbox(verify_corrected)
            original_mark = diag.mark
            def signal(stage, source_ns=None, **fields):
                result = original_mark(stage, source_ns, **fields)
                if stage == 'mailbox.take_batch.wait':
                    waiting.set()
                return result
            def consume():
                box.take_batch()
                finished.set()
            with patch.object(diag, 'mark', side_effect=signal):
                box.lock.acquire()
                worker = threading.Thread(target=consume)
                worker.start()
                try:
                    self.assertTrue(waiting.wait(2))
                    self.assertFalse(finished.is_set())
                    self.assertFalse(any(r['stage'] == 'mailbox.take_batch.acquired'
                                         for r in recorder.rows))
                finally:
                    box.lock.release()
                    worker.join(2)
                self.assertFalse(worker.is_alive())
            stages = [r['stage'] for r in recorder.rows
                      if r['stage'].startswith('mailbox.take_batch.')]
            self.assertEqual(stages, ['mailbox.take_batch.wait',
                'mailbox.take_batch.acquired', 'mailbox.take_batch.release_before',
                'mailbox.take_batch.release_after'])

    def test_fatal_mailbox_fault_freezes_existing_diagnostic_window(self):
        recorder = diag.Recorder(capacity=4096)
        with patch.object(diag, 'RECORDER', recorder):
            box = mailbox.ros_mailbox(verify_corrected)
            feed(box, packet(bad_hash=True))
            self.assertIsNone(box.fault_reason())
            self.assertIsNone(recorder.frozen)
            box.fail('diagnostic_fixture_fault')
            self.assertEqual(recorder.trigger['reason'], 'diagnostic_fixture_fault')
            self.assertTrue(any(r['stage'] == 'mailbox.verify_rejected' for r in recorder.frozen))
            frozen = recorder.frozen
            box.fail('second_fault')
            self.assertIs(recorder.frozen, frozen)
            self.assertEqual(box.fault_reason(), 'diagnostic_fixture_fault')


if __name__ == '__main__':
    unittest.main(verbosity=2)
