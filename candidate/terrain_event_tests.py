"""Fresh production terrain-switch/worker-guard finite replay, not navigation PASS."""
import ast,copy,hashlib,importlib.util,io,json,sys,tempfile,time,types,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];DEMO=ROOT.parent;OLD=HERE.parent/'corridor_tracking_v24_recovery_replan'
sys.path.insert(0,str(DEMO));sys.path.insert(0,str(ROOT))
from policy import observation
from mission46_terrain import MissionTerrainProvider,TRANSITIONS
from terrain_provider import SwitchingTerrainProvider
from navigation import goal_regions
WORLD=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_175947_closed_loop_cascade_clock_hold_curvature_on_V24_full46_r2_isolated_a87e/world.sdf')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def actual_worker_guard(provider, command=(.2,.0,.1)):
    # Exact actual request prefix through its provider override/evidence; no fake actuator or inference.
    tree=ast.parse((HERE/'worker.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='main')
    request=next(n for n in ast.walk(main)if isinstance(n,ast.FunctionDef)and n.name=='request')
    end=next(i for i,n in enumerate(request.body)if isinstance(n,ast.Expr)and 'log.write' in ast.unparse(n))
    nodes=copy.deepcopy(request.body[:end+1]);nodes.append(ast.parse('return cmd,rejected,evidence').body[0])
    fn=ast.FunctionDef(name='production_request_guard',args=copy.deepcopy(request.args),body=nodes,decorator_list=[])
    ns=dict(np=np,Path=Path,fingerprints={},provider=provider,
        read_command=lambda:(np.asarray(command),False,{'reason':'finite_fresh_original_command_fixture'}),
        latest={'clock_ns':round(provider.state[0]*1e9)},log=io.StringIO(),
        canonical=lambda x:json.dumps(x,sort_keys=True,allow_nan=False))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'worker_actual_terrain_guard_prefix','exec'),ns)
    return ns['production_request_guard'](None,provider.state[0])

class TerrainEventTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.run=Path(self.tmp.name)/'finiteV25';self.run.mkdir()
        (self.run/'world.sdf').write_bytes(WORLD.read_bytes())
        refs={}
        for name,source in [('terrain_target_manifest.json',ROOT/'tests/terrain_targets/lower12.json'),
                            ('alternate_terrain_target_manifest.json',ROOT/'tests/terrain_targets/upper23.json')]:
            dst=self.run/name;dst.write_bytes(source.read_bytes());refs[str(source.resolve())]=sha(source);refs[str(dst)]=sha(dst)
        archived=self.run/'sources/multifloor_demo/navigation/goal_regions.py';archived.parent.mkdir(parents=True);archived.write_bytes((DEMO/'navigation/goal_regions.py').read_bytes())
        snaps=self.run/'navigation_source_snapshots.json';snaps.write_text(json.dumps({str(DEMO/'navigation/goal_regions.py'):{'snapshot':str(archived),'sha256':sha(archived)}}))
        refs.update({str(self.run/'world.sdf'):sha(self.run/'world.sdf'),str(snaps):sha(snaps),str(archived):sha(archived)})
        self.profile=json.loads((HERE/'profiles/curvature_original46_on.json').read_text())
        self.provider=MissionTerrainProvider(self.run,{'profile':self.profile,'references':refs},observation.TerrainHeightMap,observation)
        self.clock=100.;self.native=np.zeros(64);self.native[0]=self.clock;self.native[1:4]=[16.,4.5,1.5];self.native[4]=1.
        self.provider.observe_state(self.native)
    def tearDown(self):self.provider.close();self.tmp.cleanup()
    def prepare(self,phase):
        gid,before,after=TRANSITIONS[phase];route={'exploring':'exploration','returning':'return_origin','navigating':'navigation_f1_f3'}[phase]
        rid=self.run.name+':'+phase+':1';request=dict(schema_version=2,request_id=rid,frame_id='camera_init',goals=self.profile['original_scenario']['route_goals'][route])
        _,goals=goal_regions.parse_request(request);index=next(i for i,g in enumerate(goals)if g.goal_id==gid);gh=goal_regions.definitions_sha256(goals)
        arrivals=[]
        for i,g in enumerate(goals[:index+1]):
            stamp=round((self.clock-10+(i+1)*.5)*1e9);dwell=round(g.dwell_sim_s*1e9)
            arrivals.append(dict(request_id=rid,goal_id=g.goal_id,waypoint_index=i,goals_definition_sha256=gh,
                stamp_ns=stamp,start_stamp_ns=stamp-dwell,dwell_ns=dwell,max_observation_gap_ns=200000000,
                region_inside=True,control_region_inside=True,protected=False,reason='arrived',raw_position=list(g.center),
                arrival_definition=g.definition()['arrival'],control_arrival_definition=g.control_arrival_definition()))
        status=dict(request_id=rid,state='running',waypoint_index=index+1,mode='teacher',frame_id='camera_init',scope=self.profile['experiment'],
            navigation_ground_truth_used=False,goals_definition_sha256=gh,goals_definitions=[g.definition()for g in goals],region_arrivals=arrivals)
        intent=dict(run_id=self.run.name,request_id=rid,goal_id=gid,phase=phase,from_layer=before,to_layer=after,
            goals_definition_sha256=gh,arrival_receipt=arrivals[-1])
        self.write('navigation_request.json',request);self.write('navigation_status.json',status);self.write('mission46_terrain_request.json',intent)
        return status,intent
    def write(self,name,obj):(self.run/name).write_text(json.dumps(obj,allow_nan=False))
    def test_original_bug_after_safe187_switch_reproduces_latch_and_worker_zero(self):
        spec=importlib.util.spec_from_file_location('frozen_v24_terrain_old',OLD/'terrain_provider.py');old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
        self.provider._record=types.MethodType(old.SwitchingTerrainProvider._record,self.provider)
        self.prepare('exploring');cmd,rejected,e=actual_worker_guard(self.provider)
        self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0]);self.assertIn('multiple values',self.provider.failure)
        self.assertEqual(self.provider.current_id,'alternate');self.assertEqual(len(self.provider.completed_transitions),1)
    def test_safe_production_three_switches_and_worker_nonzero_restored(self):
        for phase,current in [('exploring','alternate'),('returning','initial'),('navigating','alternate')]:
            with self.subTest(phase=phase):
                self.prepare(phase);cmd,rejected,e=actual_worker_guard(self.provider)
                self.assertFalse(rejected);np.testing.assert_array_equal(cmd,[.2,0,.1]);self.assertIsNone(self.provider.failure)
                self.assertEqual(self.provider.current_id,current);self.assertEqual(len(self.provider.completed_transitions),['exploring','returning','navigating'].index(phase)+1)
                ack=json.loads((self.run/'mission46_terrain_ack.json').read_text());self.assertTrue(ack['passed']);self.assertEqual(ack['ray_count'],187)
                self.assertEqual(ack['maximum_ray_difference_m'],0);self.assertFalse(ack['navigation_ground_truth_used'])
                self.assertEqual(sha(Path(ack['source_evidence_file'])),ack['source_evidence_sha256'])
                self.clock+=20;self.native[0]=self.clock;self.provider.observe_state(self.native)
        rows=[json.loads(x)for x in (self.run/'terrain_provider_events.jsonl').read_text().splitlines()]
        self.assertEqual([r['event']for r in rows].count('mission46_switched'),3)
        self.assertEqual([r['sequence']for r in rows],list(range(1,len(rows)+1)))
    def test_duplicate_false_and_payload_metadata_preserved_authority_not_overridden(self):
        before=self.provider.event_sequence
        with patch('terrain_provider.time.monotonic',return_value=123.):
            row=self.provider._record(dict(navigation_ground_truth_used=False,sequence=-5,recorded_monotonic_wall=-9.,metadata={'keep':1}))
        self.assertEqual(row['sequence'],before+1);self.assertEqual(row['recorded_monotonic_wall'],123.)
        self.assertIs(row['navigation_ground_truth_used'],False);self.assertEqual(row['metadata'],{'keep':1})
    def test_unsafe_truth_event_refused_before_counter_write(self):
        count=self.provider.event_sequence;length=(self.run/'terrain_provider_events.jsonl').stat().st_size
        for value in (True,None,0,np.bool_(False)):
            with self.subTest(value=value),self.assertRaises(ValueError):self.provider._record(dict(navigation_ground_truth_used=value))
        self.assertEqual(count,self.provider.event_sequence);self.assertEqual(length,(self.run/'terrain_provider_events.jsonl').stat().st_size)
    def test_unsafe_status_truth_latches_zero(self):
        status,_=self.prepare('exploring');status['navigation_ground_truth_used']=True;self.write('navigation_status.json',status)
        cmd,rejected,_=actual_worker_guard(self.provider);self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0]);self.assertIsNotNone(self.provider.failure)
    def test_bad187_height_latches_and_stays_zero_after_repair(self):
        self.prepare('exploring');original=self.provider.maps['alternate'].raycast
        def bad(starts):z,n=original(starts);z[0]+=1e-5;return z,n
        with patch.object(self.provider.maps['alternate'],'raycast',side_effect=bad):
            cmd,rejected,_=actual_worker_guard(self.provider);self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0])
        cmd,rejected,_=actual_worker_guard(self.provider);self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0]);self.assertEqual(self.provider.current_id,'initial');self.assertEqual(self.provider.completed_transitions,[])
    def test_bad187_count_latches_zero(self):
        self.prepare('exploring');original=self.provider.maps['alternate'].raycast
        def bad(starts):z,n=original(starts);return z[:-1],n[:-1]
        with patch.object(self.provider.maps['alternate'],'raycast',side_effect=bad):
            cmd,rejected,_=actual_worker_guard(self.provider);self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0])
    def test_nonfinite187_ray_latches_zero(self):
        self.prepare('exploring');original=self.provider.maps['alternate'].raycast
        def bad(starts):z,n=original(starts);z[0]=np.inf;return z,n
        with patch.object(self.provider.maps['alternate'],'raycast',side_effect=bad):
            cmd,rejected,_=actual_worker_guard(self.provider);self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0])
    def test_wrong_original_transition_order_latches_zero(self):
        self.prepare('returning');cmd,rejected,_=actual_worker_guard(self.provider);self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0])
    def test_early_native_clock_holds_and_causal_release_recovers(self):
        _,intent=self.prepare('exploring');stamp=intent['arrival_receipt']['stamp_ns'];self.native[0]=(stamp-25000000)/1e9;self.provider.observe_state(self.native)
        cmd,rejected,_=actual_worker_guard(self.provider);self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0]);self.assertIsNone(self.provider.failure)
        self.native[0]=stamp/1e9;self.provider.observe_state(self.native);cmd,rejected,_=actual_worker_guard(self.provider);self.assertFalse(rejected);np.testing.assert_array_equal(cmd,[.2,0,.1])
    def test_duplicate_completed_intent_does_not_switch_or_reject_again(self):
        self.prepare('exploring');self.assertFalse(self.provider.check_switch());seq=self.provider.event_sequence
        self.assertFalse(self.provider.check_switch());self.assertEqual(seq,self.provider.event_sequence);self.assertEqual(len(self.provider.completed_transitions),1)
    def test_request_hash_mismatch_latches_zero(self):
        status,_=self.prepare('exploring');status['goals_definition_sha256']='0'*64;self.write('navigation_status.json',status)
        cmd,rejected,_=actual_worker_guard(self.provider);self.assertTrue(rejected);np.testing.assert_array_equal(cmd,[0,0,0])
    def test_production_AST_delta_only_record_and_original_worker_terrain_math_unchanged(self):
        for name in ('mission46_terrain.py','mission46_obstacle_runtime.py','worker.py','controller.py','cascade_core.py','spatial_reference.py','shared_controller.py','replan_policy.py','clock_hold.py','mission46.py','route_fence.py','mission46_runtime.py','mission46_runtime_evidence.py'):
            self.assertEqual((HERE/name).read_bytes(),(OLD/name).read_bytes(),name)
        a,b=ast.parse((HERE/'terrain_provider.py').read_text()),ast.parse((OLD/'terrain_provider.py').read_text())
        for tree in (a,b):
            cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='SwitchingTerrainProvider')
            cls.body=[n for n in cls.body if not(isinstance(n,ast.FunctionDef)and n.name=='_record')]
        self.assertEqual(ast.dump(a),ast.dump(b))
    def test_actual_obstacle_payloads_do_not_duplicate_authoritative_record_keys(self):
        tree=ast.parse((HERE/'mission46_obstacle_runtime.py').read_text());cls=next(n for n in ast.walk(tree)if isinstance(n,ast.ClassDef)and n.name=='Obstacle')
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='tick')
        sent=next(n.value for n in ast.walk(tick)if isinstance(n,ast.Assign)and any(ast.unparse(t)=='self.sent'for t in n.targets)and isinstance(n.value,ast.Call))
        sent_keys={x.arg for x in sent.keywords};reserved={'sequence','run_id','request_id','navigation_ground_truth_used'}
        for call in [n for n in ast.walk(tick)if isinstance(n,ast.Call)and ast.unparse(n.func)=='self.record']:
            payload=call.args[0];keys={k.arg for k in payload.keywords if k.arg};self.assertFalse(keys&reserved)
            if any(k.arg is None for k in payload.keywords):self.assertFalse(sent_keys&reserved)

if __name__=='__main__':unittest.main()
