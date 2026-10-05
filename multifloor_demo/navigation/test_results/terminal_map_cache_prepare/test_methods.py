"""Real proposed Coordinator methods, no ROS context or physics."""
import ast,copy,json,tempfile,threading,types,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
tree=ast.parse((HERE/'mission_server_candidate.py').read_text())
cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='Coordinator')
methods=[copy.deepcopy(x) for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in ('refresh_final_map','on_map','finish_stop')]
ns={'json':json,'finish_owned_process':lambda proc:None}
exec(compile(ast.fix_missing_locations(ast.Module(body=methods,type_ignores=[])),'actual_proposed_methods','exec'),ns)

class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.dir=Path(self.tmp.name)/'run_current';self.dir.mkdir()
        self.n=types.SimpleNamespace(run_dir=self.dir,mission=types.SimpleNamespace(run_id='run_current',stage='failed'),
            map_status=dict(run_id='run_current',revision=52,point_count=10),
            last={'map':123.},acceptance={'passed':False},stopping=True,
            lock=threading.RLock(),action_lock=threading.Lock(),proc=object(),proc_log=None)
        self.n.touch=lambda name:self.n.last.update({name:999.})
        for name in ('refresh_final_map','on_map','finish_stop'):
            setattr(self.n,name,types.MethodType(ns[name],self.n))
    def tearDown(self):self.tmp.cleanup()
    def file(self,**overrides):
        x=dict(run_id='run_current',revision=53,point_count=12,binary_filename='colored_map.000053.bin',
               format={'point_step':16},save={'requested':False,'complete':False})
        x.update(overrides);(self.dir/'map_metadata.json').write_text(json.dumps(x))
        (self.dir/'colored_map.000053.bin').write_bytes(bytes(12*16));return x
    def test_owned_finish_refreshes_final_archive_without_freshness_or_acceptance(self):
        wanted=self.file();self.n.finish_stop(self.n.proc)
        self.assertEqual(self.n.map_status,wanted);self.assertFalse(self.n.stopping)
        self.assertEqual(self.n.last,{'map':123.});self.assertEqual(self.n.acceptance,{'passed':False})
        self.assertEqual(self.n.mission.stage,'failed');self.assertFalse(self.n.map_status['save']['complete'])
    def test_old_proc_cannot_refresh_new_run(self):
        self.file();self.n.finish_stop(object());self.assertEqual(self.n.map_status['revision'],52);self.assertTrue(self.n.stopping)
    def test_wrong_run_metadata_preserves_cached_data(self):
        self.file(run_id='other');self.assertFalse(self.n.refresh_final_map());self.assertEqual(self.n.map_status['revision'],52)
    def test_missing_or_short_binary_rejected(self):
        self.file();(self.dir/'colored_map.000053.bin').write_bytes(bytes(10))
        self.assertFalse(self.n.refresh_final_map());self.assertEqual(self.n.map_status['point_count'],10)
    def test_unsafe_filename_malformed_object_and_boolean_revision_rejected(self):
        self.file(binary_filename='../other.bin');self.assertFalse(self.n.refresh_final_map())
        (self.dir/'map_metadata.json').write_text('null');self.assertFalse(self.n.refresh_final_map())
        self.file(revision=True);self.assertFalse(self.n.refresh_final_map())
    def test_final_file_cannot_regress_cache(self):
        self.file();self.n.map_status['revision']=54;self.assertFalse(self.n.refresh_final_map())
        self.assertEqual(self.n.map_status['revision'],54)
    def test_late_old_DDS_revision_cannot_undo_final_refresh(self):
        self.file();self.assertTrue(self.n.refresh_final_map())
        self.n.on_map(types.SimpleNamespace(data=json.dumps(dict(run_id='run_current',revision=52,point_count=10))))
        self.assertEqual(self.n.map_status['revision'],53);self.assertEqual(self.n.last,{'map':123.})
    def test_newer_live_DDS_revision_still_updates_cache_and_touch(self):
        self.n.on_map(types.SimpleNamespace(data=json.dumps(dict(run_id='run_current',revision=54,point_count=13))))
        self.assertEqual(self.n.map_status['revision'],54);self.assertEqual(self.n.last,{'map':999.})
    def test_empty_archive_does_not_require_binary_and_non_object_DDS_ignored(self):
        self.file(revision=0,point_count=0,binary_filename=None);self.n.map_status={}
        self.assertTrue(self.n.refresh_final_map());self.n.on_map(types.SimpleNamespace(data='null'))
        self.assertEqual(self.n.map_status['point_count'],0)
    def test_read_only_original_full18_final_archive_53_replaces_cached52(self):
        demo=HERE.parents[2];directory=demo/'runs/20261002_001559_882ddb'
        self.n.run_dir=directory;self.n.mission.run_id=directory.name
        self.n.map_status={'run_id':directory.name,'revision':52,'point_count':141557}
        self.assertTrue(self.n.refresh_final_map())
        self.assertEqual((self.n.map_status['revision'],self.n.map_status['point_count']),(53,142199))
        self.assertEqual(self.n.last,{'map':123.});self.assertEqual(self.n.acceptance,{'passed':False})

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    (HERE/'contract_result.json').write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
        failures=len(result.failures),errors=len(result.errors),scope=__doc__),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
