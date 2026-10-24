"""Read-only source/physical verification used by the Mission46 ROS adapter.

Native telemetry is allowed only for initialization safety and final physical
stopping. It is never a navigation pose or a terrain-selection coordinate.
All receipts are derived from real serialized records/files, not caller flags.
"""
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np

from mission46 import INIT_CHECKS, MAP_CHECKS, TERRAIN_CHECKS, DYNAMIC_CHECKS, PARK_CHECKS
from mission46_profile import FROZEN_SHA, ORIGINAL_SCENARIO_SHA, SPAWN


def raw_sha(raw):
    return hashlib.sha256(raw).hexdigest()


def json_bytes(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False,
                       allow_nan=False) + '\n').encode()


def inside(run, value):
    p = Path(value)
    if not p.is_absolute(): p = Path(run) / p
    p = p.resolve()
    if not p.is_relative_to(Path(run).resolve()) or not p.is_file():
        raise ValueError('Required actual evidence is missing or outside this run: ' + str(p))
    return p


def checked_json(run, value, expected_sha=None):
    p = inside(run, value);raw = p.read_bytes()
    if expected_sha is not None and raw_sha(raw) != expected_sha:
        raise ValueError('Actual evidence hash differs: ' + str(p))
    return json.loads(raw), dict(file=str(p), sha256=raw_sha(raw), bytes=len(raw))


def seal(run, basename, data):
    """Create an immutable small evidence document; refuse different rewrites."""
    p = Path(run) / basename;raw = json_bytes(data)
    if p.exists():
        if p.read_bytes() != raw: raise ValueError('Refusing to replace sealed evidence: ' + str(p))
    else:
        with p.open('xb') as stream: stream.write(raw)
    return dict(source_evidence_file=str(p.resolve()), source_evidence_sha256=raw_sha(raw))


def receipt(run, schema, checks, evidence, **fields):
    return dict(schema=schema, run_id=Path(run).name, binding_verified=True, passed=True,
                checks={k:dict(status='passed',passed=True) for k in checks}, **evidence, **fields)


class FileTail:
    """Bounded incremental exact-line reader; partial lines stay unread."""
    def __init__(self, path, project=None):
        self.path=Path(path);self.offset=0;self.identity=None;self.last_size=0;self.project=project
    def poll(self, max_bytes=2*1024**2):
        if not self.path.exists(): return []
        st=self.path.stat();identity=(st.st_dev,st.st_ino)
        if self.identity is not None and (identity!=self.identity or st.st_size<self.offset):
            raise ValueError('Actual source was replaced/truncated: ' + str(self.path))
        self.identity=identity;self.last_size=st.st_size
        with self.path.open('rb') as f:
            f.seek(self.offset);raw=f.read(max_bytes)
        end=raw.rfind(b'\n')+1
        if not end:return []
        records=[];cursor=self.offset
        for line in raw[:end].splitlines(keepends=True):
            full=json.loads(line);data=full if self.project is None else self.project(full)
            record=dict(data=data,source_file=str(self.path.resolve()),source_offset=cursor,
                source_length=len(line),source_line_sha256=raw_sha(line))
            if self.project is not None:record['source_projection_fields']=list(data)
            records.append(record)
            cursor+=len(line)
        self.offset+=end
        return records


def verify_line(record):
    p=Path(record['source_file'])
    with p.open('rb') as f:
        f.seek(record['source_offset']);raw=f.read(record['source_length'])
    full=json.loads(raw)
    expected=full if 'source_projection_fields' not in record else {k:full.get(k) for k in record['source_projection_fields']}
    if raw_sha(raw)!=record['source_line_sha256'] or expected!=record['data']:
        raise ValueError('Exact serialized source-line binding failed')


NATIVE_FIELDS=('world_sim_time','state_physics_world_time','body_lin_vel','body_ang_vel','rpy',
               'qd','applied_torque','q_target','fault','actor_inferred_this_frame','body_clearance',
               'contacts','command','command_expired')


def native_projection(row):return {k:row.get(k) for k in NATIVE_FIELDS}


def status_projection(row):
    fields=('request_id','state','obstacle_hold','obstacle_stops','obstacle_resumes','aligned_obstacle_resumes',
            'command','tilt_hold','accepted_trajectory_id','pose_age','cloud_age','raw_imu_age')
    d={k:row.get(k) for k in fields}
    d['cascade_parking']={'mode':(row.get('cascade_parking')or{}).get('mode')}
    d['execution_bridge_safety']={'state':(row.get('execution_bridge_safety')or{}).get('state')}
    for field in ('known_scene_localization','known_scene_runtime_status','known_scene_scan_epoch'):
        d[field]=copy.deepcopy(row.get(field))
    return d


def native_ns(row):
    value=row.get('state_physics_world_time', row.get('world_sim_time'))
    if type(value) not in (int,float) or not math.isfinite(value):
        raise ValueError('Native record has no original physical timestamp')
    return round(value*1e9)


def native_safe(row, standing=False):
    if row.get('fault') is not None or row.get('actor_inferred_this_frame') is not True:
        return False
    try:
        rpy=np.asarray(row['rpy'],dtype=float);v=np.asarray(row['body_lin_vel'],dtype=float)
        w=np.asarray(row['body_ang_vel'],dtype=float);qd=np.asarray(row['qd'],dtype=float)
        tau=np.asarray(row['applied_torque'],dtype=float);target=np.asarray(row['q_target'],dtype=float)
        if any(not np.isfinite(x).all() for x in (rpy,v,w,qd,tau,target)):return False
        if rpy.shape!=(3,) or v.shape!=(3,) or w.shape!=(3,) or qd.shape!=(12,) or tau.shape!=(12,) or target.shape!=(12,):return False
        if (not math.isfinite(row['body_clearance']) or not math.isfinite(row['contacts']['body'])
                or max(abs(rpy[:2]))>.65 or row['body_clearance']<.18 or row['contacts']['body']>0
                or max(abs(qd))>30.001 or max(abs(tau))>23.50001):return False
        command=np.asarray(row['command'],dtype=float)
        if command.shape!=(3,) or not np.isfinite(command).all():return False
        if standing and (np.linalg.norm(v[:2])>.03 or np.linalg.norm(w)>.05 or
                         sum(row['contacts'].get(k,0)>0 for k in ('FR','FL','RR','RL'))<2 or
                         np.linalg.norm(row['command'])>1e-10):return False
        return True
    except (ValueError,TypeError,KeyError):return False


def verify_initialization(run, scope, pose_rows, imu_rows, native_records, registration, sensor_health):
    """Verify actual stationary input histories and fresh produced SLAM.

    This does not claim to inspect the estimator's hidden bias state. The
    estimator config/source must enable its separately frozen stationary gate.
    """
    import yaml
    config=inside(run,'navigation_fastlivo.yaml')
    if scope['references'].get(str(config))!=raw_sha(config.read_bytes()):
        raise ValueError('Actual initialization configuration changed after source freeze')
    cfg=yaml.safe_load(config.read_text())['/**']['ros__parameters']['imu']
    if cfg.get('stationary_initialization_en')is not True:
        raise ValueError('Actual estimator stationary initialization gate is not enabled')
    required=int(cfg['imu_int_frame']);span=float(cfg.get('init_min_span',2.9))
    if len(imu_rows)<required or len(pose_rows)<10 or len(native_records)<150:
        raise ValueError('Actual initialization source windows are incomplete')
    imus=list(imu_rows)
    stamps=np.array([r['stamp_ns'] for r in imus],dtype=np.int64)
    gyro=np.asarray([r['angular_velocity_sensor'] for r in imus],dtype=float)
    acc=np.asarray([r['linear_acceleration_sensor'] for r in imus],dtype=float)
    if (stamps[-1]-stamps[0]<round(span*1e9) or np.any(np.diff(stamps)<=0)
            or max(np.diff(stamps))>round(float(cfg.get('init_max_sample_gap',.03))*1e9)
            or not np.isfinite(gyro).all() or not np.isfinite(acc).all()
            or np.max(np.linalg.norm(gyro,axis=1))>float(cfg.get('init_max_gyro_norm',.03))
            or np.max(abs(np.linalg.norm(acc,axis=1)-9.81))>float(cfg.get('init_max_acc_norm_error',1.2))
            or max(np.std(acc,axis=0))>float(cfg.get('init_max_acc_axis_std',.4))):
        raise ValueError('Actual raw IMU history is not a complete stationary initialization window')
    pose_stamps=[r['stamp_ns'] for r in pose_rows]
    if (pose_stamps[-1]-pose_stamps[0]<2_900_000_000 or max(np.diff(pose_stamps))>200_000_000
            or any(np.linalg.norm(r['body_velocity'])>.03 for r in pose_rows)
            or np.max(np.linalg.norm(np.array([r['position'] for r in pose_rows])-pose_rows[0]['position'],axis=1))>.08):
        raise ValueError('Actual SLAM body did not remain stably stopped')
    ns=[native_ns(r['data']) for r in native_records]
    if ns[-1]-ns[0]<2_900_000_000 or max(np.diff(ns))>30_000_000 or any(not native_safe(r['data'],True) for r in native_records):
        raise ValueError('Actual Teacher did not stand safely and continuously at original spawn')
    for r in native_records:verify_line(r)
    asset,asset_source=checked_json(run,'asset_manifest.json')
    if (scope['references'].get(asset_source['file'])!=asset_source['sha256'] or
            asset.get('spawn')!=SPAWN or asset.get('exclusive_writer')!='teacher_sim::TeacherActuator'):
        raise ValueError('Actual original-spawn sole Teacher actuator contract differs')
    if (scope.get('checkpoint_sha256')!=FROZEN_SHA or scope.get('navigation_ground_truth_used')is not False
            or sensor_health.get('ready')is not True or sensor_health.get('ground_truth_navigation_used')is not False
            or registration.get('ground_truth_navigation_used')is not False or registration.get('status')!='frozen'):
        raise ValueError('Actual initialization/registration source contract is not ready')
    proof=dict(actual_pose_records=list(pose_rows),actual_imu_records=imus,native_safety_source_records=native_records,
        asset=asset,asset_source=asset_source,stationary_estimator_config=cfg,
        estimator_config_sha256=raw_sha(config.read_bytes()),scene_axis_registration=registration,
        sensor_health=sensor_health,native_use='standing safety only; never navigation coordinates',
        estimator_hidden_bias_state_inspected=False,navigation_ground_truth_used=False)
    evidence=seal(run,'mission46_initialization_evidence.json',proof)
    return receipt(run,'teacher_original_origin_initialization/v1',INIT_CHECKS,evidence,
        checkpoint_sha256=FROZEN_SHA,original_scenario_sha256=ORIGINAL_SCENARIO_SHA,
        spawn=SPAWN,navigation_ground_truth_used=False)


def verify_terrain(run, scope, ack, pending, now_ns):
    proof, source=checked_json(run,ack['source_evidence_file'],ack['source_evidence_sha256'])
    if proof.get('intent') != pending:
        # Producer receives the action kind in its persisted intent.
        expected=dict(kind='terrain_switch',**pending)
        if proof.get('intent') != expected:raise ValueError('Provider proof has a different serialized mission intent')
    for k in ('run_id','request_id','goal_id','goals_definition_sha256','from_layer','to_layer','arrival_receipt'):
        if ack.get(k)!=pending[k]:raise ValueError('Terrain receipt binding differs: '+k)
    a=np.asarray(proof['ray_heights_before'],dtype=float);b=np.asarray(proof['ray_heights_after'],dtype=float)
    starts=np.asarray(proof['ray_starts'],dtype=float)
    if a.shape!=(187,) or b.shape!=(187,) or starts.shape!=(187,3) or not all(np.isfinite(x).all() for x in (a,b,starts)):
        raise ValueError('Provider proof lacks all actual187 ray inputs/outputs')
    delta=float(np.max(np.abs(a-b)))
    if delta>1e-9 or any(len(proof[k])!=187 or any(str(n).split('/')[0]!='floor_2' for n in proof[k])
                          for k in ('collision_names_before','collision_names_after')):
        raise ValueError('Actor layers are not identical on the shared floor2 landing')
    clock=proof['native_clock'];effective=proof['effective_sim_ns']
    if (type(clock)is not int or effective!=clock or ack['effective_sim_ns']!=clock
            or not pending['arrival_receipt']['stamp_ns']<=clock<=now_ns
            or proof.get('navigation_ground_truth_used')is not False):
        raise ValueError('Provider application lacks actual causal native clock')
    world=inside(run,'world.sdf')
    expected=scope['references'].get(str(world))
    if not expected or proof['world_sha256']!=expected or raw_sha(world.read_bytes())!=expected:
        raise ValueError('Provider world is not the frozen actual SDF')
    manifests={'initial':'terrain_target_manifest.json','alternate':'alternate_terrain_target_manifest.json'}
    for key, name in manifests.items():
        p=inside(run,name);h=raw_sha(p.read_bytes())
        if proof['manifest_sha256'].get(key)!=h or scope['references'].get(str(p))!=h:
            raise ValueError('Provider terrain manifest source differs')
    status=proof['actual_original_navigation_status']
    if (status.get('request_id')!=pending['request_id'] or
            status.get('goals_definition_sha256')!=pending['goals_definition_sha256'] or
            pending['arrival_receipt'] not in status.get('region_arrivals',[]) or
            status.get('navigation_ground_truth_used')is not False or status.get('state')!='running'):
        raise ValueError('Provider selection has no original actual SLAM status binding')
    for name in ('status_read','request_read','intent_read'):
        read=proof[name];raw=read.get('raw_utf8')
        if not isinstance(raw,str) or raw_sha(raw.encode())!=read.get('raw_bytes_sha256'):
            raise ValueError('Original producer file-read evidence is missing or corrupted')
    result=copy.deepcopy(ack)
    result.update(binding_verified=True,passed=True,ray_count=187,maximum_ray_difference_m=delta,
                  checks={k:dict(status='passed',passed=True) for k in TERRAIN_CHECKS},
                  runtime_verified_source=source)
    return result


def read_pcd(path):
    raw=Path(path).read_bytes()
    header,payload=raw.split(b'DATA binary\n',1);fields={}
    for line in header.decode('ascii').splitlines():
        if line and not line.startswith('#'):
            parts=line.split();fields[parts[0]]=parts[1:]
    count=int(fields['POINTS'][0])
    if (fields.get('FIELDS')!=['x','y','z','rgb'] or fields.get('SIZE')!=['4']*4
            or fields.get('TYPE',[])[:3]!=['F']*3 or fields['TYPE'][3] not in ('F','U')
            or len(payload)!=count*16):raise ValueError('Actual saved map PCD structure/count differs')
    points=np.frombuffer(payload,dtype=[('xyz','<f4',(3,)),('rgb','<u4')])
    if not np.isfinite(points['xyz']).all():raise ValueError('Saved map contains invalid geometry')
    return dict(point_count=count,unique_colors=int(len(np.unique(points['rgb']&0xffffff))),
                file_sha256=raw_sha(raw),bytes=len(raw))


def verify_rgb(run, request_id, service_result, requested_epoch_s):
    if not service_result['success']:raise ValueError('Actual map save service failed: '+service_result['message'])
    meta,meta_source=checked_json(run,'map_metadata.json');save=meta.get('save',{})
    p=inside(run,save['filename'])
    if p.parent!=Path(run).resolve():raise ValueError('Saved RGB PCD must belong to this actual run')
    stats=read_pcd(p);health=save.get('healthy_sensor_evidence',{});ages=health.get('ages',{})
    if (meta.get('run_id')!=Path(run).name or meta.get('frame_id')!='camera_init'
            or meta.get('ground_truth_used')is not False or meta.get('reference_map_loaded')is not False
            or meta.get('source_topic')!='/cloud_registered' or '/camera/image_color' not in meta.get('color_source','')
            or save.get('complete')is not True or save.get('saved_at',-math.inf)<requested_epoch_s
            or stats['point_count']<500 or stats['point_count']!=save.get('point_count')
            or stats['unique_colors']<8 or meta.get('observed_rgb_samples',0)<500
            or meta.get('capacity_rejections')!=0 or health.get('error')is not None
            or health.get('slam_healthy')is not True or health.get('camera_healthy')is not True
            or not all(meta.get('counts',{}).get(k,0)>0 for k in ('camera','lidar','imu','odom','colored_cloud'))
            or not all(type(ages.get(k))in(int,float) and math.isfinite(ages[k]) and 0<=ages[k]<2
                       for k in ('odom','lidar','imu','full_cloud','camera','colored_cloud'))):
        raise ValueError('Actual current-run RGB map/count/color/provenance/freshness gate failed')
    proof=dict(metadata=meta,metadata_source=meta_source,actual_pcd=stats,
               service_response=service_result,save_requested_epoch_s=requested_epoch_s,
               navigation_ground_truth_used=False)
    evidence=seal(run,'mission46_rgb_save_evidence.json',proof)
    return receipt(run,'teacher_mission46_rgb_save/v1',MAP_CHECKS,evidence,
        request_id=request_id,point_count=stats['point_count'],unique_colors=stats['unique_colors'],
        observed_rgb_samples=meta['observed_rgb_samples'],capacity_rejections=0,
        ground_truth_used=False,reference_map_loaded=False,source_topic=meta['source_topic'],
        color_source=meta['color_source'],saved_file=str(p),saved_file_sha256=stats['file_sha256'],
        sensor_ages_s=ages)


def verify_dynamic(run, request_id, status_rows, pose_rows, guard_records, obstacle_records):
    """Replay actual guard inputs and require physical service ACKs/recovery.

    SetEntityPose ACK is explicitly the service execution evidence; this is
    not an independent Gazebo pose/info observer. All original raw records
    remain available for the final independent evaluator to check that limit.
    """
    from simulation.obstacle_trigger import SceneTrigger
    from continuous_scene_trigger import scene_trigger_view
    import sys
    from mission46_profile import DEMO
    if str(DEMO/'navigation') not in sys.path:sys.path.append(str(DEMO/'navigation'))
    from control_core import steering_obstacle_ahead
    from guard_audit import NativeGuardAudit, result_json
    records=[r for r in obstacle_records if r['data'].get('request_id')==request_id]
    triggers=[r for r in records if r['data'].get('operation')=='trigger_armed']
    acks=[r for r in records if r['data'].get('operation')=='SetEntityPose_response' and r['data'].get('success')is True]
    if len(triggers)!=1:raise ValueError('Exactly one actual original dynamic scene trigger required')
    trigger=triggers[0]['data'];scene=SceneTrigger()
    trigger_view,adapter=scene_trigger_view(trigger['mission_state'])
    if adapter["continuous_route"] and (
            adapter["applied"] is not True or trigger.get("trigger_scene_view")!=trigger_view
            or trigger.get("trigger_view_adapter")!=adapter):
        raise ValueError("Actual continuous trigger lacks replayable original pass receipt/view binding")
    scene.observe(trigger_view)
    if not scene.nearby(trigger['actual_slam_pose']['position'][:2]) or not scene.ready:
        raise ValueError('Actual dynamic trigger does not replay original SceneTrigger')
    if (trigger['mission_state']['current_request']!=request_id or
            trigger.get('navigation_ground_truth_used')is not False or
            trigger['actual_slam_pose'].get('ground_truth_used')is not False):
        raise ValueError('Dynamic trigger source identity/SLAM provenance differs')
    phases={r['data']['phase'] for r in acks}
    if not {'entering','blocking','leaving','clear'}.issubset(phases):
        raise ValueError('Actual obstacle service has not completed original enter/block/leave/clear phases')
    for r in triggers+acks:
        verify_line(r)
    blocking=[r for r in acks if r['data']['phase']=='blocking'];clear=[r for r in acks if r['data']['phase']=='clear']
    if (blocking[-1]['data']['request_sim_ns']-blocking[0]['data']['request_sim_ns']<19_000_000_000
            or any(r['data'].get('entity')!='moving_obstacle' or r['data'].get('service')!='/world/teacher_demo/set_pose'
                   or r['data']['response_sim_ns']<r['data']['request_sim_ns']
                   or r['data']['position'][0]!=1. or r['data']['position'][2]!=.6 for r in acks)
            or any(abs(r['data']['position'][1]-2.)>1e-9 for r in blocking)
            or any(abs(r['data']['position'][1])>1e-9 for r in clear)):
        raise ValueError('Actual obstacle ACK geometry/duration/causality differs')
    statuses=[r for r in status_rows if r['data'].get('request_id')==request_id]
    held=[r for r in statuses if r['data'].get('obstacle_hold')is True]
    if not held:raise ValueError('Actual controller did not stop for the moving obstacle')
    edge=held[0]['received_sim_ns']
    guards=[r for r in guard_records if r['data'].get('request_id')==request_id]
    preroll=[r for r in guards if edge-350_000_000<=r['data']['compute_ros_clock_ns']<=edge]
    if len(preroll)<3 or preroll[-1]['data']['compute_ros_clock_ns']-preroll[0]['data']['compute_ros_clock_ns']<300_000_000:
        raise ValueError('Actual dynamic hold lacks the full300ms native SCAN pre-roll')
    last_held=held[-1]['received_sim_ns']
    resumed=[r for r in statuses if r['received_sim_ns']>last_held and not r['data'].get('obstacle_hold')
             and r['data'].get('aligned_obstacle_resumes',0)>0 and r['data'].get('obstacle_resumes',0)>0]
    if not resumed:raise ValueError('Actual controller has not completed aligned obstacle recovery')
    resume=resumed[0]['received_sim_ns']
    cleared=[r for r in guards if resume-1_350_000_000<=r['data']['compute_ros_clock_ns']<=resume
             and r['data']['union_result']['blocked']is False]
    if len(cleared)<10 or cleared[-1]['data']['compute_ros_clock_ns']-cleared[0]['data']['compute_ros_clock_ns']<1_000_000_000:
        raise ValueError('Actual recovery lacks one continuous clear guard second')
    window=preroll+cleared
    for rows in (preroll,cleared):
        clocks=[r['data']['compute_ros_clock_ns'] for r in rows]
        if any(x<=0 or x>300_000_000 for x in np.diff(clocks)):raise ValueError('Actual obstacle guard continuity failed')
    blocked=[r for r in guards if edge-350_000_000<=r['data']['compute_ros_clock_ns']<=edge+350_000_000
             and r['data']['union_result']['blocked']is True]
    if not blocked:raise ValueError('No actual native collision corridor blockage at hold edge')
    replay_rows=[blocked[0],*cleared]
    replay=[]
    for r in window:
        d=r['data'];verify_line(r);clock=d['compute_ros_clock_ns']
        if (not 0<=clock-d['cloud_header_stamp_ns']<=300_000_000 or
                not 0<=clock-d['control_pose_stamp_ns']<=300_000_000 or
                d.get('cloud_frame_id')!='camera_init'):
            raise ValueError('Actual obstacle guard source is stale or in another frame')
    for r in replay_rows:
        d=r['data'];array=inside(run,d['cloud_array_file']);cloud=np.load(array,allow_pickle=False)
        if raw_sha(np.asarray(cloud,dtype='<f8').tobytes())!=d['filtered_xyz_float64_sha256']:
            raise ValueError('Actual guard cloud payload hash differs')
        native=NativeGuardAudit(steering_obstacle_ahead)
        result=native(cloud,np.asarray(d['control_pose']),np.asarray(d['checked_target']),
                      np.asarray(d['steering_direction']),np.asarray(d['route']))
        if (result_json(result)!=d['union_result'] or result_json(native.calls[0][1])!=d['goal_corridor_result']
                or result_json(native.calls[1][1])!=d['motion_corridor_result']):
            raise ValueError('Actual native two-corridor guard does not replay')
        replay.append(dict(guard_source=r,cloud_file=str(array),cloud_file_sha256=raw_sha(array.read_bytes()),
                           replay_union=result_json(result)))
    stopped=[r for r in pose_rows if edge<=r['stamp_ns']<=last_held
             and np.linalg.norm(r['body_velocity'][:2])<=.03 and abs(r['body_angular_velocity'][2])<=.05
             and r.get('body_gyro')is not None and np.linalg.norm(r['body_gyro'])<=.05]
    # Search a contiguous measured stop window, not a timer or desired zero.
    good=[];stop_window=None
    for r in stopped:
        if good and r['stamp_ns']-good[-1]['stamp_ns']>200_000_000:good=[]
        good.append(r)
        if len(good)>=3 and good[-1]['stamp_ns']-good[0]['stamp_ns']>=300_000_000:stop_window=list(good);break
    moving=[r for r in pose_rows if r['stamp_ns']>=resume]
    if stop_window is None or len(moving)<3 or not any(np.linalg.norm(r['body_velocity'][:2])>.025 for r in moving):
        raise ValueError('Actual SLAM/IMU stop and actual moving recovery are incomplete')
    progress=max(np.linalg.norm(np.asarray(r['position'][:2])-moving[0]['position'][:2]) for r in moving)
    if progress<.08:raise ValueError('Recovery has no actual SLAM displacement beyond8cm')
    if any(any(abs(v)>1e-10 for v in r['data'].get('command',[])) for r in held):
        raise ValueError('Actual obstacle hold status contains a nonzero command')
    proof=dict(request_id=request_id,actual_trigger=triggers[0],actual_service_acknowledgments=acks,
        measured_stop_window=stop_window,recovery_actual_poses=moving,native_guard_pre_roll=preroll,
        continuous_clear_guard=cleared,native_guard_replay=replay,actual_controller_held_statuses=held,
        actual_recovered_statuses=resumed,recovery_progress_m=float(progress),navigation_ground_truth_used=False,
        actual_entity_evidence='SetEntityPose service execution ACK; independent pose/info observation not collected here')
    evidence=seal(run,'mission46_dynamic_evidence.json',proof)
    return receipt(run,'teacher_mission46_dynamic_obstacle/v1',DYNAMIC_CHECKS,evidence,
        request_id=request_id,navigation_ground_truth_used=False,
        actual_entity_evidence_scope=proof['actual_entity_evidence'])


def verify_parking(run, request_id, start_ns, pose_rows, native_records, status_rows, origin):
    end=start_ns+5_000_000_000
    poses=[r for r in pose_rows if start_ns<=r['stamp_ns']<=end]
    natives=[r for r in native_records if start_ns<=native_ns(r['data'])<=end]
    statuses=[r for r in status_rows if start_ns<=r['received_sim_ns']<=end]
    if len(poses)<25 or len(natives)<200 or len(statuses)<20:
        raise ValueError('First5s hold has insufficient actual pose/native/status records')
    profile_path=Path(run)/'navigation_profile.json'
    assisted=profile_path.exists() and json.loads(profile_path.read_text()).get('known_scene_localization_contract',{}).get('enabled')
    if assisted:
        generations=set()
        for row in statuses:
            data=row['data'];m=data.get('known_scene_localization') or {};s=data.get('known_scene_runtime_status') or {};e=data.get('known_scene_scan_epoch') or {}
            if (not m.get('control_allowed') or m.get('correction_stale') or m.get('correction_transition')
                    or not s.get('initialized') or s.get('correction_transition') or not e.get('ready')
                    or not (m.get('generation')==s.get('generation')==e.get('generation'))):
                raise ValueError('Known-scene parking requires a fresh committed pose/cloud/planner epoch, never a correction hold')
            generations.add(m['generation'])
        if len(generations)!=1:raise ValueError('Parking5s crosses a localization correction epoch')
    def cover(stamps,gap):
        return stamps[0]-start_ns<=gap and end-stamps[-1]<=gap and np.all(np.diff(stamps)>0) and max(np.diff(stamps))<=gap
    if not cover([r['stamp_ns'] for r in poses],200_000_000) or not cover([native_ns(r['data']) for r in natives],30_000_000):
        raise ValueError('Actual first5s parking source coverage/gap failed')
    p=np.asarray([r['position'] for r in poses]);q=np.asarray([r['quaternion_xyzw'] for r in poses]);base=np.asarray(origin['position'])
    yaw=np.arctan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2))
    q0=origin['quaternion_xyzw'];yaw0=math.atan2(2*(q0[3]*q0[2]+q0[0]*q0[1]),1-2*(q0[1]**2+q0[2]**2))
    xy=float(np.max(np.linalg.norm(p[:,:2]-base[:2],axis=1)))
    yd=float(np.max(np.abs(np.arctan2(np.sin(yaw-yaw0),np.cos(yaw-yaw0)))))
    vv=max(float(np.linalg.norm(r['data']['body_lin_vel'][:2])) for r in natives)
    wz=max(abs(r['data']['body_ang_vel'][2]) for r in natives)
    for r in natives:verify_line(r)
    angular=np.unwrap([r['data']['rpy'][2] for r in natives]);ts=np.array([native_ns(r['data']) for r in natives])/1e9
    yawdot=float(np.max(np.abs(np.diff(angular)/np.diff(ts))))
    if (xy>.05 or yd>.1 or vv>.08 or wz>.1 or yawdot>.1
            or any(not native_safe(r['data']) or r['data'].get('command_expired')is not False for r in natives)
            or any(r['data'].get('state')!='succeeded' or r['data'].get('request_id')!=request_id
                   or (r['data'].get('cascade_parking')or{}).get('mode')!='active_hold'
                   or r['data'].get('tilt_hold') or r['data'].get('obstacle_hold')
                   or (r['data'].get('execution_bridge_safety')or{}).get('state')!='ready' for r in statuses)
            or any(not 0<=r['received_sim_ns']-r['stamp_ns']<=300_000_000
                   or not 0<=r['received_wall_ns']-r['source_received_wall_ns']<=300_000_000 for r in poses)):
        raise ValueError('Actual first5s parking drift/native/source/protection gate failed')
    proof=dict(interval=[start_ns,end],actual_slam_poses=poses,native_source_records=natives,
        actual_controller_statuses=statuses,origin=origin,
        metrics=dict(slam_xy_drift_max_m=xy,slam_yaw_drift_max_rad=yd,native_planar_speed_max_mps=vv,
                     native_wz_max_radps=wz,native_yaw_dot_max_radps=yawdot),
        navigation_ground_truth_used=False,native_use='offline parking safety only')
    evidence=seal(run,'mission46_final_parking_evidence.json',proof)
    return receipt(run,'teacher_mission46_final_parking/v1',PARK_CHECKS,evidence,
                   request_id=request_id,start_stamp_ns=start_ns,end_stamp_ns=end,metrics=proof['metrics'])
