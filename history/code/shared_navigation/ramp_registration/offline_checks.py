#!/usr/bin/env python3
"""Synthetic offline method checks only. No ROS initialization or simulation."""
from __future__ import annotations
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil
import time
import os
from unittest.mock import patch
import numpy as np
from cloud import decode_cloud,rotation,GravityCalibration
from geometry import register,_side_face_evidence,DEFAULTS
from capture import CaptureWriter,finish,temporal_admission,atomic_json
from analyze import analyze_capture
from observer import environment
HERE=Path(__file__).resolve().parent


def synthetic_points():
    pts=[]
    for x in np.arange(0,10.01,.1):
        for y in np.arange(-1,1.01,.1):pts.append([x,y,0 if x<2 else .6 if x>8 else .1*(x-2)])
    for x in np.arange(0,10.01,.2):
        for y in np.arange(-1.4,1.41,.2):pts.extend([[x,y,-.3],[x,y,2.2]])
    for x in np.arange(2,8.01,.05):
        for y in [-1.03,1.03]:
            for d in [.03,.06,.10,.14]:pts.append([x,y,.1*(x-2)-d])
    return np.asarray(pts,dtype=np.float64)


def row(name,stamp,wall):
    return {'stream':name,'original_stamp_ns':stamp,'frame_id':'imu_link'if name=='imu'else'camera_init',
        'received_monotonic_wall':wall,'received_ros_clock_ns':stamp+10_000_000,'received_clock_wall_age_s':.005,
        'position':[1.,0.,.4],'quaternion':[0.,0.,0.,1.],'body_linear_velocity':[0.,0.,0.],
        'child_frame_id':'demo_slam_body'if name=='body_odom'else'aft_mapped',
        'linear_acceleration':[0.,0.,9.81],'angular_velocity':[0.,0.,0.],
        'pose_covariance':[0.]*36,'twist_covariance':[.04 if k%7==0 else 0. for k in range(36)]}


def check_all(output):
    output=Path(output);output.mkdir(exist_ok=False,parents=True);checks=[]
    def test(name,fn):
        try:
            value=fn();passed=bool(value);checks.append({'name':name,'passed':passed})
        except Exception as exc:checks.append({'name':name,'passed':False,'error':type(exc).__name__+': '+str(exc)})
    def raises(fn):
        try:fn()
        except (ValueError,RuntimeError):return True
        return False
    for path in HERE.glob('*.py'):test('compile_'+path.name,lambda p=path:bool(compile(p.read_text(),str(p),'exec')))
    meta={'width':2,'height':2,'point_step':48,'row_step':112,'is_bigendian':False,'is_dense':False,
        'fields':[{'name':n,'offset':o,'datatype':k,'count':c}for n,o,k,c in
                  [('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',16,7,1),('normal',20,7,3),('ring',40,4,1)]]}
    raw=bytearray([171]*(meta['height']*meta['row_step']))
    for h in range(2):
        for w in range(2):
            offset=h*112+w*48;values=np.array([h*2+w,w,h],dtype='<f4');raw[offset:offset+12]=values.tobytes()
            raw[offset+16:offset+20]=np.array([.75],dtype='<f4').tobytes()
            raw[offset+20:offset+32]=np.array([0,0,99],dtype='<f4').tobytes()
            raw[offset+40:offset+42]=np.array([321],dtype='<u2').tobytes()
    raw[112:116]=np.array([np.nan],dtype='<f4').tobytes()
    fields,xyz,idx=decode_cloud(meta,bytes(raw))
    test('organized_row_padding_and_finite_source_indices',lambda:np.array_equal(idx,[0,1,3])and np.array_equal(xyz[:,0],[0,1,3]))
    test('original_structured_float_and_integer_fields',lambda:fields['x'].dtype==np.dtype('<f4')and np.all(fields['ring']==321)and np.all(fields['normal'][:,2]==99))
    original_points=b''.join(bytes(raw[h*112+w*48:h*112+(w+1)*48])for h in range(2)for w in range(2))
    test('original_point_padding_bytes_preserved',lambda:fields.tobytes()==original_points)
    big={'width':1,'height':1,'point_step':32,'row_step':32,'is_bigendian':True,'fields':[
        {'name':n,'offset':o,'datatype':8,'count':1}for n,o in [('z',0),('x',8),('y',16)]]}
    payload=np.asarray([3.25,1.25,2.25,123.],dtype='>f8').tobytes();bf,bxyz,bi=decode_cloud(big,payload)
    test('big_endian_float64_arbitrary_field_order',lambda:np.array_equal(bxyz,[[1.25,2.25,3.25]])and bf['z'].dtype==np.dtype('>f8'))
    invalid=dict(meta,fields=meta['fields']+[dict(meta['fields'][0])])
    test('duplicate_fields_rejected',lambda:raises(lambda:decode_cloud(invalid,bytes(raw))))
    invalid2=dict(meta,row_step=10)
    test('malformed_stride_rejected',lambda:raises(lambda:decode_cloud(invalid2,bytes(raw))))
    test('empty_package_rejected',lambda:raises(lambda:decode_cloud(dict(meta,width=0),b'')))
    base=row('full_cloud',1_000_000_000,10.);pose=row('body_odom',1_000_000_000,9.99)
    test('fresh_actual_stamp_admitted',lambda:not temporal_admission(base,pose))
    test('300ms_old_header_rejected',lambda:'cloud_header_not_fresh_300ms'in temporal_admission(dict(base,received_ros_clock_ns=1_300_000_000),pose))
    test('300ms_pose_wall_rejected',lambda:'body_pose_wall_not_fresh_300ms'in temporal_admission(base,dict(pose,received_monotonic_wall=9.7)))
    test('151ms_pose_pair_rejected',lambda:'no_actual_body_pose_within_150ms'in temporal_admission(base,dict(pose,original_stamp_ns=849_000_000)))
    test('wrong_frame_rejected',lambda:'non_camera_init_cloud'in temporal_admission(dict(base,frame_id='world'),pose))
    axis=np.asarray([1.,2.,3.]);axis/=np.linalg.norm(axis);q=np.r_[axis*np.sin(.7),np.cos(.7)];R=rotation(q)
    g=GravityCalibration()
    for i in range(71):
        ns=i*10_000_000;imu=row('imu',ns,10+i*.01);ip=row('imu_slam_odom',(i//10)*100_000_000,10+(i//10)*.1)
        bp=row('body_odom',ip['original_stamp_ns'],ip['received_monotonic_wall']);ip['quaternion']=q.tolist()
        g.observe(imu,ip,bp)
    test('up_from_actual_accelerometer_and_slam_rotation',lambda:g.result is not None and np.allclose(g.result['up_camera_init'],R@[0,0,1],atol=1e-12))
    frozen=g.result.copy();imu=row('imu',9_000_000_000,19.);imu['linear_acceleration']=[9.81,0,0]
    g.observe(imu,row('imu_slam_odom',9_000_000_000,19.),row('body_odom',9_000_000_000,19.))
    test('gravity_frozen_once_dynamic_not_recalibrated',lambda:g.result==frozen)
    bad=GravityCalibration(min_samples=1,min_span_ns=0,min_pose_samples=1);imu=row('imu',0,1.);imu['frame_id']='world'
    test('unmatched_imu_frame_cannot_initialize_gravity',lambda:bad.observe(imu,row('imu_slam_odom',0,1.),row('body_odom',0,1.))is None)
    bad2=GravityCalibration(min_samples=1,min_span_ns=0,min_pose_samples=1)
    test('future_source_pose_cannot_initialize_gravity',lambda:bad2.observe(row('imu',0,1.),row('imu_slam_odom',1,1.),row('body_odom',0,1.))is None)
    pts=synthetic_points();anchor=np.array([1.,0.,.4]);registered=register(pts,[0,0,1],anchor)
    atomic_json(output/'synthetic_geometry.json',registered)
    test('synthetic_double_edge_two_platform_full_candidate',lambda:registered['status']=='full_candidate'and registered['full_route']is not None)
    platform_heights=[p['center'][2]for p in registered['support_planes']if p['kind']=='horizontal_platform_candidate']
    test('both_lower_and_overhead_layers_retained',lambda:any(abs(h+.3)<.02 for h in platform_heights)and any(abs(h-2.2)<.02 for h in platform_heights))
    selected=next(p for p in registered['support_planes']if p['plane_id']==registered['current_support_platform_id'])
    test('current_layer_neither_lowest_nor_highest',lambda:abs(selected['center'][2])<.02)
    on_ramp=register(pts,[0,0,1],[3.2,0,.52])
    current_plane=next(p for p in on_ramp['support_planes']if p['plane_id']==on_ramp['current_support_plane_id'])
    test('current_support_can_be_actual_ramp_not_only_platform',lambda:current_plane['kind']=='ramp_candidate'and on_ramp['full_route_eligible'])
    translated=register(pts@R.T+[8,-5,2],R@[0,0,1],anchor@R.T+[8,-5,2])
    target=np.asarray(registered['full_route']['surface_centerline_camera_init'])@R.T+[8,-5,2]
    test('rigid_rotation_translation_full_route_invariance',lambda:translated['status']=='full_candidate'and np.allclose(translated['full_route']['surface_centerline_camera_init'],target,atol=1e-8))
    one_side=pts[pts[:,1]<1.01]
    test('one_actual_side_only_cannot_output_full_route',lambda:not register(one_side,[0,0,1],anchor)['full_route_eligible'])
    cropped=pts[pts[:,0]<6]
    test('unseen_exit_platform_is_partial',lambda:register(cropped,[0,0,1],anchor)['full_route']is None)
    faces_removed=pts[(np.abs(pts[:,1])<1.02)|(pts[:,2]==-.3)|(pts[:,2]==2.2)]
    test('hull_plus_lower_layer_without_faces_not_complete_edges',lambda:register(faces_removed,[0,0,1],anchor)['full_route']is None)
    uu,vv=np.meshgrid(np.arange(0,1.01,.05),[-1.04,-1.02,-1.]);uu=uu.ravel();vv=vv.ravel();hh=np.zeros(len(uu));depth=.36*uu+.03
    face=_side_face_evidence(uu,vv,hh,np.ones(len(uu),dtype=bool),depth,DEFAULTS)
    test('steep_ramp_relative_depth_of_horizontal_floor_not_vertical_face',lambda:face['depth_span_m']>.06 and not face['verified_near_vertical_face'])
    test('color_cloud_exploration_cannot_claim_full',lambda:register(pts,[0,0,1],anchor,source_kind='exploratory_color_cloud')['full_route']is None)
    test('incomplete_archive_cannot_claim_full',lambda:register(pts,[0,0,1],anchor,complete_archive=False)['full_route']is None)
    observer_tree=ast.parse((HERE/'observer.py').read_text());calls=[n.func.attr for n in ast.walk(observer_tree)if isinstance(n,ast.Call)and isinstance(n.func,ast.Attribute)]
    test('observer_no_publisher_service_client_actuation',lambda:not({'create_publisher','create_client','create_service','publish'}&set(calls)))
    observer_class=next(n for n in ast.walk(observer_tree)if isinstance(n,ast.ClassDef)and n.name=='Observer')
    publisher_override=next(n for n in observer_class.body if isinstance(n,ast.FunctionDef)and n.name=='create_publisher')
    ns={'RuntimeError':RuntimeError};exec(compile(ast.Module(body=[publisher_override],type_ignores=[]),'actual_create_publisher_override','exec'),ns)
    class LocalOnly:
        def LocalParameterEvents(self):return 'local bookkeeping only'
    test('read_only_node_only_local_parameter_event_sink',lambda:ns['create_publisher'](LocalOnly(),type('ParameterEvent',(),{}),'/parameter_events')=='local bookkeeping only')
    test('read_only_node_forbids_every_external_publisher',lambda:raises(lambda:ns['create_publisher'](LocalOnly(),type('Twist',(),{}),'/demo/cmd_vel')))
    xml=output/'transport.xml';xml.write_text('offline XML placeholder, never parsed/started by DDS')
    envdata={'schema':'cloud_transport_experiment/v1','xml_path':str(xml.resolve()),'xml_sha256':hashlib.sha256(xml.read_bytes()).hexdigest(),
        'environment':{'ROS_DOMAIN_ID':'79','FASTRTPS_DEFAULT_PROFILES_FILE':str(xml.resolve())},
        'remove_environment_keys':['FASTRTPS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI'],'source_hashes':{}}
    envpath=output/'transport_manifest.json';atomic_json(envpath,envdata)
    with patch.dict(os.environ,envdata['environment'],clear=True):
        test('transport_removed_then_canonically_replaced_key_accepted',lambda:environment(envpath)['ROS_DOMAIN_ID']=='79')
    with patch.dict(os.environ,dict(envdata['environment'],CYCLONEDDS_URI='unwanted'),clear=True):
        test('transport_removed_not_replaced_key_rejected',lambda:raises(lambda:environment(envpath)))
    mainfunc=next(n for n in observer_tree.body if isinstance(n,ast.FunctionDef)and n.name=='main')
    maintry=[n for n in mainfunc.body if isinstance(n,ast.Try)][0]
    protected_calls=[ast.unparse(n.func)for n in ast.walk(ast.Module(body=maintry.body,type_ignores=[]))if isinstance(n,ast.Call)]
    test('sources_copy_and_ros_init_inside_failed_manifest_cleanup_try',lambda:'shutil.copyfile'in protected_calls and 'rclpy.init'in protected_calls)
    capture=output/'capture';w=CaptureWriter(capture);(capture/'sources').mkdir()
    names=['cloud.py','geometry.py','capture.py','analyze.py'];source_hashes={}
    for name in names:
        shutil.copyfile(HERE/name,capture/'sources'/name);source_hashes[name]=hashlib.sha256((HERE/name).read_bytes()).hexdigest()
    start=time.monotonic()-2
    for i in range(71):
        ns=i*10_000_000;wall=start+i*.01
        if i%10==0:
            w.append(row('body_odom',ns,wall));w.append(row('imu_slam_odom',ns,wall))
        w.append(row('imu',ns,wall+.001))
    cloud_row=row('full_cloud',700_000_000,start+.705);cloud_row.update(meta);w.append(cloud_row,bytes(raw))
    manifest=finish(w,{'schema':'actual_ramp_cloud_capture/v1','source_freeze':{'source_hashes':source_hashes}})
    test('actual_writer_drains_and_records_original_payload',lambda:manifest['writer_drained']and manifest['submitted']==manifest['processed']and manifest['clouds'][0]['raw_payload_sha256']==hashlib.sha256(raw).hexdigest())
    with np.load(capture/manifest['clouds'][0]['archive_file'],allow_pickle=False)as data:
        test('NPZ_original_raw_and_structured_fields_roundtrip',lambda:data['raw'].tobytes()==bytes(raw)and np.array_equal(data['fields']['ring'],fields['ring']))
    analysis=analyze_capture(capture,output/'analysis_no_full')
    test('actual_offline_analyzer_hashes_and_no_small_cloud_full_claim',lambda:analysis['full_route']is None and analysis['registration_control_enabled']is False)
    original_archive=capture/manifest['clouds'][0]['archive_file'];original_bytes=original_archive.read_bytes()
    original_archive.write_bytes(original_bytes+b'changed')
    test('source_archive_tamper_refused',lambda:raises(lambda:analyze_capture(capture,output/'analysis_tampered')))
    original_archive.write_bytes(original_bytes)
    empty_capture=output/'empty_capture';ew=CaptureWriter(empty_capture)
    empty_row=row('full_cloud',1_000_000_000,10.);empty_row.update(dict(meta,width=0,height=0,row_step=0));ew.append(empty_row,b'')
    em=finish(ew,{'schema':'actual_ramp_cloud_capture/v1'})
    test('invalid_empty_cloud_kept_and_explicitly_rejected',lambda:len(em['clouds'])==1 and not em['clouds'][0]['geometry_admitted']and em['clouds'][0]['decode_error']is not None)
    error_capture=output/'error_capture';fw=CaptureWriter(error_capture,max_queue_bytes=1)
    failed_append=raises(lambda:fw.append(row('body_odom',0,1.)))
    failed_finish=raises(lambda:finish(fw,{'schema':'actual_ramp_cloud_capture/v1'}))
    fm=json.loads((error_capture/'capture_manifest.json').read_text())
    test('overflow_failed_manifest_drain_and_exception',lambda:failed_append and failed_finish and fm['status']=='failed'and fm['writer_drained'])
    receipt={'schema':'ramp_registration_offline_checks/v1','ROS_started':False,'simulation_started':False,
        'synthetic_geometry_not_actual_evidence':True,'checks':checks,'passed':all(c['passed']for c in checks),
        'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in HERE.glob('*.py')},
        'navigation_verified':False}
    atomic_json(output/'checks.json',receipt);return receipt


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=check_all(a.output)
    print(json.dumps({'checks':len(r['checks']),'passed':r['passed'],'failures':[c for c in r['checks']if not c['passed']]}))
    if not r['passed']:raise SystemExit(1)


if __name__=='__main__':main()
