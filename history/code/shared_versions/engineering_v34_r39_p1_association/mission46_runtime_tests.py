"""Finite synthetic source/negative/lifecycle tests, never actual mission evidence."""
from __future__ import annotations
import ast
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
import mission46_runtime_evidence as e
from mission46_runtime import atomic_exact, load_interfaces
from mission46_obstacle_runtime import target_at

HERE=Path(__file__).resolve().parent


def native(t=0):
    return dict(state_physics_world_time=t,world_sim_time=t,body_lin_vel=[0.,0.,0.],
        body_ang_vel=[0.,0.,0.],rpy=[0.,0.,0.],qd=[0.]*12,applied_torque=[0.]*12,
        q_target=[0.]*12,fault=None,actor_inferred_this_frame=True,body_clearance=.3,
        contacts=dict(body=0,FR=1,FL=1,RR=1,RL=1),command=[0.,0.,0.],command_expired=False)


def compile_method(name, run):
    """Compile the unchanged production method body without constructing ROS."""
    tree=ast.parse((HERE/'mission46_runtime.py').read_text())
    cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='Coordinator')
    method=copy.deepcopy(next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name))
    module=ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[]))
    namespace=dict(run=run,json=json,copy=copy,atomic_exact=atomic_exact,ACTIVE_STAGES={'exploring'},
        time=__import__('time'),Bool=lambda **k:k,String=lambda **k:k)
    exec(compile(module,str(HERE/'mission46_runtime.py'),'exec'),namespace)
    return namespace[name]


class Fixture(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.run=Path(self.tmp.name)
    def tearDown(self):self.tmp.cleanup()
    def write(self,name,data):
        p=self.run/name;p.write_bytes(e.json_bytes(data));return p
    def rgb(self):
        p=self.run/'colored_map.pcd';arr=np.zeros(501,dtype=[('xyz','<f4',(3,)),('rgb','<u4')]);arr['rgb']=np.arange(501)%16
        header=b'VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH 501\nHEIGHT 1\nPOINTS 501\nDATA binary\n'
        p.write_bytes(header+arr.tobytes())
        meta=dict(run_id=self.run.name,frame_id='camera_init',ground_truth_used=False,reference_map_loaded=False,
            source_topic='/cloud_registered',color_source='actual /camera/image_color',observed_rgb_samples=501,capacity_rejections=0,
            counts={k:1 for k in ('camera','lidar','imu','odom','colored_cloud')},
            save=dict(filename=str(p),complete=True,saved_at=11.,point_count=501,
                healthy_sensor_evidence=dict(error=None,slam_healthy=True,camera_healthy=True,
                    ages={k:.1 for k in ('odom','lidar','imu','full_cloud','camera','colored_cloud')})))
        self.write('map_metadata.json',meta);return meta
    def terrain(self):
        pending=dict(run_id=self.run.name,request_id='actual_rid',goal_id='exploration:11',
            goals_definition_sha256='f'*64,from_layer='lower12',to_layer='upper23',arrival_receipt=dict(stamp_ns=10))
        refs={};mh={}
        for name in ('world.sdf','terrain_target_manifest.json','alternate_terrain_target_manifest.json'):
            p=self.run/name;p.write_bytes(b'frozen synthetic '+name.encode());refs[str(p)]=e.raw_sha(p.read_bytes())
        mh=dict(initial=refs[str(self.run/'terrain_target_manifest.json')],alternate=refs[str(self.run/'alternate_terrain_target_manifest.json')])
        read=dict(raw_utf8='{}\n',raw_bytes_sha256=e.raw_sha(b'{}\n'))
        proof=dict(intent=dict(kind='terrain_switch',**pending),ray_starts=[[0.,0.,3.]]*187,
            ray_heights_before=[1.2]*187,ray_heights_after=[1.2]*187,collision_names_before=['floor_2/collision']*187,
            collision_names_after=['floor_2/collision']*187,native_clock=20,effective_sim_ns=20,
            navigation_ground_truth_used=False,world_sha256=refs[str(self.run/'world.sdf')],manifest_sha256=mh,
            actual_original_navigation_status=dict(request_id='actual_rid',goals_definition_sha256='f'*64,
                region_arrivals=[pending['arrival_receipt']],navigation_ground_truth_used=False,state='running'),
            status_read=read,request_read=read,intent_read=read)
        p=self.write('proof.json',proof)
        ack=dict(pending,source_evidence_file=str(p),source_evidence_sha256=e.raw_sha(p.read_bytes()),effective_sim_ns=20)
        return dict(references=refs),ack,pending,proof
    def terrain_mutation(self,change):
        scope,ack,pending,proof=self.terrain();change(proof);p=self.write('proof.json',proof)
        ack['source_evidence_sha256']=e.raw_sha(p.read_bytes())
        with self.assertRaises(ValueError):e.verify_terrain(self.run,scope,ack,pending,30)
    def parking(self):
        p=self.run/'telemetry.jsonl';p.write_bytes(b''.join(e.json_bytes(native(i*.02)) for i in range(251)))
        records=e.FileTail(p,e.native_projection).poll()
        poses=[dict(stamp_ns=i*100_000_000,position=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.],
                received_sim_ns=i*100_000_000,received_wall_ns=i*100_000_000,source_received_wall_ns=i*100_000_000)
               for i in range(51)]
        statuses=[dict(received_sim_ns=i*100_000_000,data=dict(state='succeeded',request_id='r',
                cascade_parking=dict(mode='active_hold'),execution_bridge_safety=dict(state='ready'),
                tilt_hold=False,obstacle_hold=False)) for i in range(51)]
        return poses,records,statuses,dict(position=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.])

    def test_tail_partial_is_not_accepted(self):
        p=self.run/'source.jsonl';p.write_bytes(b'{"a":1}\n{"a":2')
        reader=e.FileTail(p);self.assertEqual(len(reader.poll()),1);self.assertEqual(reader.poll(),[])
        with p.open('ab') as f:f.write(b'}\n')
        self.assertEqual(reader.poll()[0]['data'],{'a':2})
    def test_tail_truncation_rejected(self):
        p=self.write('source.jsonl',dict(a=1));reader=e.FileTail(p);reader.poll();p.write_bytes(b'')
        with self.assertRaises(ValueError):reader.poll()
    def test_projection_keeps_exact_serialized_binding(self):
        row=dict(native(),large_unused={'x':list(range(1000))});p=self.write('source.jsonl',row)
        r=e.FileTail(p,e.native_projection).poll()[0];e.verify_line(r)
        self.assertNotIn('large_unused',r['data']);self.assertEqual(r['source_length'],p.stat().st_size)
    def test_source_change_rejected_even_outside_projection(self):
        p=self.write('source.jsonl',dict(native(),unused=1));r=e.FileTail(p,e.native_projection).poll()[0]
        p.write_bytes(p.read_bytes().replace(b'"unused":1',b'"unused":2'))
        with self.assertRaises(ValueError):e.verify_line(r)
    def test_seal_refuses_changed_receipt(self):
        e.seal(self.run,'x.json',dict(a=1));e.seal(self.run,'x.json',dict(a=1))
        with self.assertRaises(ValueError):e.seal(self.run,'x.json',dict(a=2))
    def test_evidence_path_escape_rejected(self):
        with self.assertRaises(ValueError):e.inside(self.run,Path(__file__))
    def test_native_standing_requires_actual_actor_and_contact(self):
        self.assertTrue(e.native_safe(native(),True))
        for change in (dict(actor_inferred_this_frame=False),dict(body_clearance=.17),dict(body_lin_vel=[.031,0,0]),
                       dict(applied_torque=[24.]*12),dict(contacts=dict(body=1,FR=1,FL=1)),dict(qd=[float('nan')]*12),dict(command=[float('nan'),0,0]),dict(body_clearance=float('nan'))):
            with self.subTest(change=change):self.assertFalse(e.native_safe(dict(native(),**change),True))
    def test_native_timestamp_uses_original_physics_clock(self):
        self.assertEqual(e.native_ns(dict(native(12),sim_time=1)),12_000_000_000)
        with self.assertRaises(ValueError):e.native_ns(dict(sim_time=1))
    def test_actual_pcd_saved_receipt(self):
        self.rgb();r=e.verify_rgb(self.run,'r',dict(success=True,message='saved'),10)
        self.assertTrue(r['passed']);self.assertEqual(r['point_count'],501);self.assertEqual(r['unique_colors'],16)
    def test_map_service_failure_not_success_flags(self):
        self.rgb()
        with self.assertRaises(ValueError):e.verify_rgb(self.run,'r',dict(success=False,message='failed'),10)
    def test_map_foreign_run_rejected(self):
        meta=self.rgb();meta['run_id']='other';self.write('map_metadata.json',meta)
        with self.assertRaises(ValueError):e.verify_rgb(self.run,'r',dict(success=True,message=''),10)
    def test_old_save_rejected(self):
        self.rgb()
        with self.assertRaises(ValueError):e.verify_rgb(self.run,'r',dict(success=True,message=''),12)
    def test_map_stale_source_rejected(self):
        meta=self.rgb();meta['save']['healthy_sensor_evidence']['ages']['imu']=2.;self.write('map_metadata.json',meta)
        with self.assertRaises(ValueError):e.verify_rgb(self.run,'r',dict(success=True,message=''),10)
    def test_map_zero_colors_rejected(self):
        meta=self.rgb();p=self.run/'colored_map.pcd';raw=p.read_bytes();h,b=raw.split(b'DATA binary\n');arr=np.frombuffer(b,dtype=[('xyz','<f4',(3,)),('rgb','<u4')]).copy();arr['rgb']=0;p.write_bytes(h+b'DATA binary\n'+arr.tobytes())
        with self.assertRaises(ValueError):e.verify_rgb(self.run,'r',dict(success=True,message=''),10)
    def test_actual_187_ray_receipt(self):
        scope,ack,pending,_=self.terrain();r=e.verify_terrain(self.run,scope,ack,pending,30)
        self.assertTrue(r['passed']);self.assertEqual(r['ray_count'],187)
    def test_terrain_one_ray_diff_rejected(self):self.terrain_mutation(lambda p:p['ray_heights_after'].__setitem__(10,1.2001))
    def test_terrain_wrong_floor_rejected(self):self.terrain_mutation(lambda p:p['collision_names_after'].__setitem__(0,'floor_3/collision'))
    def test_terrain_future_effect_rejected(self):self.terrain_mutation(lambda p:p.update(native_clock=31,effective_sim_ns=31))
    def test_terrain_corrupt_source_read_rejected(self):self.terrain_mutation(lambda p:p['status_read'].update(raw_bytes_sha256='0'*64))
    def test_first5s_actual_parking(self):
        args=self.parking();r=e.verify_parking(self.run,'r',0,*args);self.assertTrue(r['passed']);self.assertEqual(r['end_stamp_ns'],5_000_000_000)
    def test_first5s_drift_rejected(self):
        poses,n,s,o=self.parking();poses[-1]['position'][0]=.051
        with self.assertRaises(ValueError):e.verify_parking(self.run,'r',0,poses,n,s,o)
    def test_first5s_missing_native_coverage_rejected(self):
        poses,n,s,o=self.parking()
        with self.assertRaises(ValueError):e.verify_parking(self.run,'r',0,poses,n[10:],s,o)
    def test_first5s_duplicate_physics_clock_rejected(self):
        poses,n,s,o=self.parking();n[1]=copy.deepcopy(n[0])
        with self.assertRaises(ValueError):e.verify_parking(self.run,'r',0,poses,n,s,o)
    def test_first5s_wrong_controller_mode_rejected(self):
        poses,n,s,o=self.parking();s[-1]['data']['cascade_parking']['mode']='zero_action'
        with self.assertRaises(ValueError):e.verify_parking(self.run,'r',0,poses,n,s,o)
    def test_interface_config_requires_scope_and_current_module_hash(self):
        p=self.write('module.py',dict(example=1));modulehash=e.raw_sha(p.read_bytes())
        config=self.write('mission46_runtime_interfaces.json',dict(x=dict(module=str(p),implementation_sha256=modulehash,supported=True)))
        scope=dict(references={str(p):modulehash,str(config):e.raw_sha(config.read_bytes())})
        self.assertEqual(load_interfaces(self.run,scope)['x']['implementation_sha256'],modulehash)
        p.write_bytes(b'changed')
        with self.assertRaises(ValueError):load_interfaces(self.run,scope)
    def test_original_dynamic_motion_piecewise(self):
        expected=((0,[1.,4.,.6],'entering'),(8,[1.,2.,.6],'blocking'),(28,[1.,2.,.6],'leaving'),(36,[1.,0.,.6],'clear'))
        for t,p,phase in expected:self.assertEqual(target_at(t),(p,phase))
        with self.assertRaises(ValueError):target_at(-.1)
    def test_production_apply_no_ros_publish_after_context_shutdown(self):
        rows=[];pub=SimpleNamespace(publish=lambda _:self.fail('ROS publish invoked with invalid context'))
        node=SimpleNamespace(writer=SimpleNamespace(append=lambda *a:rows.append(a)),context=SimpleNamespace(ok=lambda:False),
            clock_ns=lambda:123,stop_pub=pub,obstacle_pub=pub,terrain_pub=pub)
        compile_method('apply',self.run)(node,[dict(kind='stop_navigation'),dict(kind='obstacle',enabled=False),dict(kind='terrain_switch',request_id='r')])
        self.assertEqual(len(rows),3);self.assertTrue((self.run/'mission46_terrain_request.json').exists())
    def test_production_close_always_drains_writer_even_if_pool_fails(self):
        events=[]
        def pool(**_):events.append('pool');raise ValueError('test shutdown error')
        node=SimpleNamespace(started=True,mission=SimpleNamespace(original=SimpleNamespace(stage='completed')),
            publish_state=lambda *a:events.append('status'),clock_ns=lambda:123,
            pool=SimpleNamespace(shutdown=pool),writer=SimpleNamespace(close=lambda:events.append('writer')))
        with self.assertRaises(ValueError):compile_method('close',self.run)(node)
        self.assertEqual(events,['status','pool','writer'])
    def test_production_close_preserves_incomplete_mission_failure(self):
        events=[];node=SimpleNamespace(started=True,mission=SimpleNamespace(original=SimpleNamespace(stage='exploring')),
            fail=lambda m:events.append(('fail',m)),publish_state=lambda *a:events.append('status'),clock_ns=lambda:123,
            pool=SimpleNamespace(shutdown=lambda **k:events.append('pool')),writer=SimpleNamespace(close=lambda:events.append('writer')))
        compile_method('close',self.run)(node)
        self.assertEqual(events[0][0],'fail');self.assertEqual(events[-2:],['pool','writer'])


if __name__=='__main__':unittest.main(verbosity=2)
