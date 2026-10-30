import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from control.safety.contact_mailbox import ContactMailbox, ContactPublisher, PACKET


class ContactFlows(unittest.TestCase):
    def test_nonblocking_missing_receiver_and_original_stamp_freshness(self):
        with tempfile.TemporaryDirectory() as directory:
            endpoint=Path(directory)/'contacts.sock';sender=ContactPublisher(endpoint)
            self.assertFalse(sender.publish(1_000_000_000,[0,1,0,1,0]))
            receiver=ContactMailbox(endpoint)
            try:
                self.assertTrue(sender.publish(1_000_000_000,[0,1,0,1,0]))
                receiver.drain();self.assertTrue(receiver.fresh(1_020_000_000))
                receipt=receiver.receipt_wall_ns
                sender.publish(1_000_000_000,[10,0,0,0,0]);receiver.drain()
                self.assertEqual(receiver.receipt_wall_ns,receipt)
                self.assertEqual(receiver.contacts,(0,1,0,1,0))
                self.assertFalse(receiver.fresh(1_300_000_000))
                with patch('control.safety.contact_mailbox.time.monotonic_ns',return_value=receipt+300_000_000):
                    self.assertFalse(receiver.fresh(1_020_000_000))
                sender.publish(1_020_000_000,[0,1,0,1,0],fault=True);receiver.drain()
                self.assertFalse(receiver.fresh(1_020_000_000))
                self.assertEqual(PACKET.size,56)
            finally:receiver.close();sender.close()


if __name__=='__main__':unittest.main()
