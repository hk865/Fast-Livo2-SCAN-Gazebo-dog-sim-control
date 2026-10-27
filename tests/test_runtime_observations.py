"""Actual observation interfaces stay mandatory; source/version pins do not."""
from pathlib import Path
import copy,json,sys,tempfile,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from known_scene_adapter import validate_runtime_witness

class RuntimeObservationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        roles=['gz_sensors_gpu_lidar','gz_rendering','gz_sim_sensors','ros_gz_bridge']
        producer=[]
        for i,role in enumerate(roles):
            library='/no-source-install/'+role+'.so';maps=self.root/(role+'.maps');maps.write_text('actual-observation '+library+'\n')
            producer.append(dict(role=role,path=library,observed_pid=100+i,load_observation='proc_maps',load_observation_path=str(maps)))
        mapper=dict(pid=10,start_ticks=20);processes=dict(mapper=mapper)
        for i,role in enumerate(['source','adapter']):
            source='/no-source-install/'+role+'.py';observation=self.root/(role+'.cmdline');observation.write_bytes(('python3\0'+source+'\0--run\0test').encode())
            processes[role]=dict(pid=11+i,start_ticks=21+i,source_path=source,observation=str(observation))
        loaded=self.root/'loaded.json';loaded.write_text(json.dumps(dict(verified=True,mapping_identity=mapper)))
        self.witness=dict(schema='go2_known_scene_runtime_witness/v1',run_id='test',producer_model='gz_gpu_lidar_instantaneous_header_snapshot',actual_loaded_libraries_verified=True,header_snapshot_time_verified=True,post_lio_single_owner_publication_verified=True,no_robot_truth_pose=True,producer_bindings=producer,explicit_point_commit_required=True,identity_fingerprint_model='go2_ros_semantic/v1',identity_producer_process_bindings=processes,loaded_SLAM_receipt_path=str(loaded))
    def test_static_sources_hashes_and_versions_are_not_inputs(self):
        self.assertTrue(validate_runtime_witness(self.witness,'test')['valid'])
        self.assertNotIn('stage_source_bindings',self.witness)
    def test_loaded_role_cannot_be_asserted_without_observation(self):
        w=copy.deepcopy(self.witness);w['producer_bindings'][0]['load_observation_path']=str(self.root/'missing')
        self.assertFalse(validate_runtime_witness(w,'test')['valid'])
    def test_wrong_library_in_actual_maps_is_rejected(self):
        w=copy.deepcopy(self.witness);Path(w['producer_bindings'][0]['load_observation_path']).write_text('another-library')
        self.assertFalse(validate_runtime_witness(w,'test')['valid'])
    def test_wrong_producer_command_is_rejected(self):
        w=copy.deepcopy(self.witness);Path(w['identity_producer_process_bindings']['source']['observation']).write_text('other-script')
        self.assertFalse(validate_runtime_witness(w,'test')['valid'])
    def test_foreign_run_is_rejected(self):
        self.assertFalse(validate_runtime_witness(self.witness,'foreign')['valid'])
    def test_actual_mapper_process_is_required(self):
        w=copy.deepcopy(self.witness);w['identity_producer_process_bindings']['mapper']['pid']=999
        self.assertFalse(validate_runtime_witness(w,'test')['valid'])
    def test_point_commit_protocol_cannot_be_disabled(self):
        w=copy.deepcopy(self.witness);w['explicit_point_commit_required']=False
        self.assertFalse(validate_runtime_witness(w,'test')['valid'])

if __name__=='__main__':unittest.main()
