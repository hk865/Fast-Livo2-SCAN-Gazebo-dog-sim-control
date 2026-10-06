import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from event_archive import EventArchive


def wait(predicate):
    deadline=time.monotonic()+3.
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('archive did not finish')


class ArchiveTest(unittest.TestCase):
    def test_exact_numpy_atomic_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            archive=EventArchive(directory)
            cloud=np.array([[.651234567, .1, -.1]],dtype=np.float32)
            cloud.setflags(write=False)
            archive.enqueue({'cloud':cloud},{'blocked':np.bool_(True),'stamp':np.int64(10000000001)})
            wait(lambda:archive.completed==1)
            path=Path(archive.status()['path']); document=json.loads(path.read_text())
            binary=path.with_name(document['cloud_file'])
            self.assertEqual(document['stamp'],10000000001)
            self.assertIs(document['blocked'],True)
            self.assertEqual(document['cloud_sha256'],hashlib.sha256(binary.read_bytes()).hexdigest())
            self.assertTrue(np.array_equal(np.load(binary)['cloud'],cloud))
            self.assertEqual(list(path.parent.glob('*.tmp.*')),[])
            self.assertTrue(archive.close())

    def test_no_environment_isolates_instances(self):
        with patch.dict(os.environ,{},clear=True):
            a,b=EventArchive(),EventArchive()
            for obj in (a,b):obj.enqueue({'cloud':np.zeros((1,3))},{})
            wait(lambda:a.completed==b.completed==1)
            self.assertNotEqual(a.root,b.root)
            self.assertTrue(str(a.root).startswith(tempfile.gettempdir()))
            self.assertTrue(a.close());self.assertTrue(b.close())

    def test_failure_has_no_partial_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            invalid=Path(directory)/'file';invalid.write_text('not a directory')
            archive=EventArchive(str(invalid))
            archive.enqueue({'cloud':np.zeros((1,3))},{})
            wait(lambda:archive.status().get('error'))
            self.assertEqual(archive.completed,0)
            self.assertIn('NotADirectoryError',archive.status()['error'])
            self.assertTrue(archive.close())

    def test_queue_is_bounded_and_nonblocking(self):
        with tempfile.TemporaryDirectory() as directory:
            archive=EventArchive(directory,capacity=1)
            gate=threading.Event(); entered=threading.Event(); original=archive._write
            def delayed(*args):
                entered.set();gate.wait(2.);return original(*args)
            archive._write=delayed
            archive.enqueue({'cloud':np.zeros((1,3))},{})
            self.assertTrue(entered.wait(1.))
            archive.enqueue({'cloud':np.ones((1,3))},{})
            start=time.monotonic();self.assertIsNone(archive.enqueue({'cloud':np.ones((1,3))*2},{}))
            self.assertLess(time.monotonic()-start,.05)
            self.assertEqual(archive.dropped,1)
            gate.set();wait(lambda:archive.completed==2)
            self.assertTrue(archive.close())


if __name__=='__main__':unittest.main()
