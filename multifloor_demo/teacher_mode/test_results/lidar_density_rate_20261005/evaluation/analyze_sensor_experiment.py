#!/usr/bin/env python3
"""Read-only LiDAR density/rate comparison; no acceptance gates are changed.

Configured beams, actual valid relay points, preprocessing, LIO/VIO headers and
controller mathematical updates are distinct. Native state is OFFLINE ONLY.
"""
from __future__ import annotations
import argparse,csv,hashlib,json,struct
from collections import Counter,defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HEADER=struct.Struct('<7Q')
MAGIC=b'FLIVODIAG0001LE\0'
READER_SHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()

def rows(path):
    with Path(path).open() as f:
        for line in f:
            if line.strip():yield json.loads(line)

def read(path):return json.loads(Path(path).read_text())
def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def dist(a):
    a=np.asarray(a,float);a=a[np.isfinite(a)]
    return {'count':len(a),'min':float(a.min()),'p50':float(np.median(a)),
        'p95':float(np.quantile(a,.95)),'max':float(a.max())}if len(a)else{'count':0}
def rate(stamps):
    s=np.asarray(stamps,np.int64);d=np.diff(s)*1e-9;unique=np.unique(s)
    return {'count':len(s),'unique_headers':len(unique),'first_last_s':[float(s[0]*1e-9),float(s[-1]*1e-9)]if len(s)else None,
        'span_average_hz':float((len(unique)-1)/(unique[-1]-unique[0])*1e9)if len(unique)>1 else None,
        'median_dt_hz':float(1/np.median(d[d>0]))if np.any(d>0)else None,
        'source_gap_s':dist(d),'duplicate_headers':int(np.sum(d==0)),
        'backward_headers':int(np.sum(d<0)),'over_300ms_source_gaps':int(np.sum(d>.3))}

def frozen_schema(run,name):
    d=read(run/'navigation_source_snapshots.json')
    candidates=[r for p,r in d.items()if Path(p).name==name]
    if len(candidates)!=1:raise ValueError('No unique frozen schema '+name)
    row=candidates[0];p=Path(row['snapshot']).resolve()
    if not p.is_relative_to((run/'sources').resolve())or sha(p)!=row['sha256']:
        raise ValueError('Frozen schema path/hash differs')
    return read(p),{'path':str(p),'sha256':row['sha256']}

def field(layout,name):return next(f for f in layout if f['name']==name)
def value(a,layout,name):
    f=field(layout,name);v=a[f['offset']:f['offset']+f['count']]
    return v.reshape(f['shape']or[])
def table(a,layout,width,prefix):
    count=int(a[prefix-2]);actual_width=int(a[prefix-1])
    if actual_width!=width or len(a)!=prefix+count*width:raise ValueError('Table width differs')
    return a[prefix:].reshape(count,width)

def native_and_slam(run):
    # Native Teacher PreUpdate reads prior physics result; the native t-dt phase
    # is retained. Neither this alignment nor native positions feed navigation.
    native=np.array([[d['t']-d['dt'],*d['position'],*d['quaternion_wxyz'],
        *d['body_lin_vel_origin'],*d['body_ang_vel'],d['contacts'][0],d['fault']]
        for d in rows(run/'actuator.jsonl')if d.get('kind')=='physics_step'],float)
    slam=np.array([[d['stamp_ns']*1e-9,*d['position'],*d['quaternion'],
        *d['body_velocity'],d['sim_age_at_callback_s']]
        for d in rows(run/'navigation_slam_poses.jsonl')],float)
    if not len(native)or not len(slam):raise ValueError('Missing actual native/SLAM data')
    reg=read(run/'navigation_scene_axis_registration.json');pair=reg['exact_paired_sample_records'][-1]
    anchor_t=pair['slam_stamp_ns']*1e-9;axis=np.array(reg['heading_receipt']['rotation_camera_init_from_world'])
    p0=np.array([np.interp(anchor_t,native[:,0],native[:,j])for j in (1,2,3)])
    anchor=np.array(pair['slam_source']['position']);aligned=(native[:,1:4]-p0)@axis.T+anchor
    return native,slam,aligned,axis,anchor_t

def window_physical(n,s,aligned,start,end):
    nn=n[(n[:,0]>start)&(n[:,0]<=end)];ss=s[(s[:,0]>start)&(s[:,0]<=end)]
    if len(nn)<2 or len(ss)<2:return {'window_s':[start,end],'status':'insufficient_samples'}
    rr=Rotation.from_quat(nn[:,[5,6,7,4]]);rpy=rr.as_euler('xyz');wv=rr.apply(nn[:,8:11])
    ns=np.array([np.interp(ss[:,0],n[:,0],aligned[:,j])for j in range(3)]).T
    ze=ss[:,3]-ns[:,2]
    return {'window_start_exclusive_end_inclusive_s':[start,end],
        'native_sample_count':len(nn),'SLAM_sample_count':len(ss),
        'native_delta_z_m':float(nn[-1,3]-nn[0,3]),'native_z_range_m':float(np.ptp(nn[:,3])),
        'SLAM_delta_z_m':float(ss[-1,3]-ss[0,3]),'SLAM_native_relative_z_error_start_end_m':[float(ze[0]),float(ze[-1])],
        'SLAM_native_relative_z_error_max_abs_m':float(np.abs(ze).max()),
        'native_roll_pitch_abs_max_rad':np.max(np.abs(rpy[:,:2]),axis=0).tolist(),
        'native_world_vz_mps':dist(wv[:,2]),'native_body_contact_samples':int(np.sum(nn[:,14]>0)),
        'native_fault_samples':int(np.sum(nn[:,15]!=0)),
        'SLAM_source_callback_age_s':dist(ss[:,11])}

def east_wall(run):
    # Obtain literal frozen SDF wall geometry, rather than assuming an old plane
    # ID denotes the same entity in another run. This demo box is axis-aligned.
    world=ET.parse(run/'world.sdf').getroot()
    for model in world.findall('.//model'):
        if model.get('name')=='wall_18.5':
            pose=np.array([float(x)for x in model.findtext('pose','0 0 0 0 0 0').split()])
            size=np.array([float(x)for x in model.findtext('.//visual/geometry/box/size').split()])
            if not np.allclose(pose[3:],0):raise ValueError('Wall rotation unsupported')
            return {'model':'wall_18.5','inner_x_m':float(pose[0]-size[0]/2),
                'y_bounds_m':[float(pose[1]-size[1]/2),float(pose[1]+size[1]/2)],
                'z_bounds_m':[float(pose[2]-size[2]/2),float(pose[2]+size[2]/2)]}
    raise ValueError('No literal east-wall box in frozen SDF')

def diagnostic_summary(run,native,axis):
    schema,provenance=frozen_schema(run,'lio_diagnostic_schema.json')
    writer=read(run/'fastlivo_diagnostics/writer_stats.json')
    detail_begin=float(writer['detail_begin_s']);detail_end=float(writer['detail_end_s'])
    snapshots=read(run/'navigation_source_snapshots.json')
    trim_names=[n for n in snapshots if Path(n).name=='LOGGING_ACCELERATION_CONTRACT.json']
    trim_contract=trim_provenance=None
    if trim_names:trim_contract,trim_provenance=frozen_schema(run,'LOGGING_ACCELERATION_CONTRACT.json')
    def timing_window(t):
        return ('startup_0_10'if t<10 else 'pre_detail_10_'+format(detail_begin,'g')if t<detail_begin
            else 'detail_'+format(detail_begin,'g')+'_'+format(detail_end,'g')if t<=detail_end
            else 'post_detail_'+format(detail_end,'g')+'_end')
    layouts={k:schema['kinds'][str(k)].get('fields',schema['kinds'][str(k)].get('row_fields'))for k in (100,101,103)}
    wall=east_wall(run);sensor=read(run/'sensor_contract.json')['lidar']
    ext=Rotation.from_euler('xyz',sensor['body_rpy']);trans=np.array(sensor['body_xyz'])
    stamps=defaultdict(list);quantities=defaultdict(list);counts=Counter();geometry=[];receipts=defaultdict(list)
    phase_start={};phase_propagated={};timings=defaultdict(list);phase_begin_wall=defaultdict(list)
    latest_imu_stamp=None;internal_backlog=defaultdict(list)
    selected=hashlib.sha256();sampled_seconds=set();plane_sample_seconds=set();plane_events=[]
    index=run/'fastlivo_diagnostics/index.csv';binary=run/'fastlivo_diagnostics/records.bin'
    with binary.open('rb')as b,index.open()as listing:
        if b.read(16)!=MAGIC:raise ValueError('Binary magic differs')
        for row in csv.DictReader(listing):
            kind=int(row['kind']);stamp=int(row['stamp_ns']);it=int(row['iteration']);t=stamp*1e-9;counts[kind]+=1
            if kind==1:
                stamps['raw_IMU_callback'].append(stamp);latest_imu_stamp=stamp;continue
            if kind==201:
                if int(row['n_values'])!=3 and (not stamps['actual_VIO_optimizer_frames']or stamps['actual_VIO_optimizer_frames'][-1]!=stamp):
                    stamps['actual_VIO_optimizer_frames'].append(stamp)
                continue
            relevant=kind in (2,3,4,10,11,12,13,100)
            if kind==101 and it==0 and detail_begin<=t<=detail_end and int(t)not in sampled_seconds:
                relevant=True;sampled_seconds.add(int(t))
            if kind==103 and detail_begin<=t<=detail_end and int(t)not in plane_sample_seconds:
                relevant=True;plane_sample_seconds.add(int(t))
            if not relevant:continue
            offset=int(row['offset']);b.seek(offset);header=b.read(56);h=HEADER.unpack(header)
            a=np.frombuffer(b.read(int(row['n_values'])*8),dtype='<f8')
            if h[0]!=kind or h[2]!=stamp or len(a)!=h[-1]:raise ValueError('Binary/index mismatch')
            selected.update(header);selected.update(a.tobytes())
            if kind==2:
                stamps['actual_FASTLIVO_valid_PointCloud2_callback'].append(stamp)
                receipts['actual_FASTLIVO_valid_PointCloud2_callback'].append(float(a[0]))
                quantities['valid_FASTLIVO_input_points'].append(a[3]*a[4])
            elif kind==3:
                stamps['actual_image_callback'].append(stamp);receipts['actual_image_callback'].append(float(a[0]))
            elif kind==4:quantities['post_preprocess_points_detail_window'].append(a[0])
            elif kind==10:
                stamps['before_Process2_stage_'+str(h[3])].append(stamp);phase_start[h[1]]=float(a[0])
                phase_begin_wall[str(h[3])].append(float(a[0]))
                window=timing_window(t)
                if latest_imu_stamp is not None:internal_backlog[(str(h[3]),window)].append((latest_imu_stamp-stamp)*1e-9)
            elif kind==11:phase_propagated[h[1]]=float(a[0])
            elif kind in (12,13):
                label='VIO'if kind==12 else'LIO'
                if a[0]:stamps[label+'_handler_after_processed'].append(stamp)
                window=timing_window(t)
                if h[1]in phase_start:timings[(label,window,'phase_total_wall_s')].append(float(a[1])-phase_start[h[1]])
                if h[1]in phase_propagated:timings[(label,window,'observation_wall_s')].append(float(a[1])-phase_propagated[h[1]])
            elif kind==100 and it==0:
                stamps['actual_LIO_optimizer_frames'].append(stamp)
                quantities['first_LIO_iteration_downsampled_points'].append(float(value(a,layouts[100],'downsampled_count')))
                quantities['first_LIO_iteration_effective_constraints'].append(float(value(a,layouts[100],'effective_count')))
            elif kind==103:
                tab=table(a,layouts[103],schema['kinds']['103']['row_width'],4)
                for r in tab:
                    if float(value(r,layouts[103],'is_plane')):
                        ev=np.array([float(value(r,layouts[103],f))for f in ('min_eigenvalue','mid_eigenvalue','max_eigenvalue')])
                        norm=value(r,layouts[103],'normal');plane_events.append([t,*ev,float(abs(norm[2])),float(value(r,layouts[103],'update_enabled'))])
            elif kind==101:
                tab=table(a,layouts[101],schema['kinds']['101']['row_width'],3)
                fpoint=field(layouts[101],'point_lidar');fpnorm=field(layouts[101],'normal');fmid=field(layouts[101],'plane_mid_eigenvalue');fmax=field(layouts[101],'plane_max_eigenvalue')
                j=int(np.clip(np.searchsorted(native[:,0],t),0,len(native)-1));prev=max(0,j-1)
                if abs(native[prev,0]-t)<abs(native[j,0]-t):j=prev
                body=Rotation.from_quat(native[j,[5,6,7,4]])
                world_points=body.apply(ext.apply(tab[:,fpoint['offset']:fpoint['offset']+3])+trans)+native[j,1:4]
                # Coordinates are estimator-undistorted lidar points. The 6cm
                # support proximity is diagnostic classification only, never
                # a navigation acceptance threshold or new measurement gate.
                mask=(abs(world_points[:,0]-wall['inner_x_m'])<=.06)&(world_points[:,1]>=wall['y_bounds_m'][0])&(world_points[:,1]<=wall['y_bounds_m'][1])&(world_points[:,2]>=wall['z_bounds_m'][0])&(world_points[:,2]<=wall['z_bounds_m'][1])
                normal=tab[:,fpnorm['offset']:fpnorm['offset']+3]@axis
                angle=np.rad2deg(np.arccos(np.clip(abs(normal[:,0])/np.linalg.norm(normal,axis=1),0,1)))
                ratios=tab[:,fmid['offset']]/np.maximum(tab[:,fmax['offset']],1e-30)
                geometry.append({'stamp_ns':stamp,'actual_constraints':len(tab),'native_body_pose_nearest_gap_s':abs(native[j,0]-t),
                    'east_wall_constraints':int(mask.sum()),'east_wall_normal_error_degrees':dist(angle[mask]),
                    'east_wall_normal_error_over60deg_constraints':int(np.sum(mask&(angle>60))),
                    'east_wall_historical_mid_over_max':dist(ratios[mask]),
                    'east_wall_weak2D_mid_over_max_under005_constraints':int(np.sum(mask&(ratios<.05))),
                    'east_wall_near_horizontal_normal_constraints':int(np.sum(mask&(np.abs(normal[:,2])>=.8)))})
    initial=np.array(plane_events)
    rates={k:rate(v)for k,v in stamps.items()}
    if trim_contract is not None:
        for name in ('actual_LIO_optimizer_frames','actual_VIO_optimizer_frames'):
            rates.setdefault(name,rate([])).update(collection_scope='actual writer detail window only',
                detail_window_sim_s=[detail_begin,detail_end],whole_run_optimizer_frequency_status='not_collected',
                missing_outside_window_is_not_zero=True)
    return {'rates':rates,'point_counts':{k:dist(v)for k,v in quantities.items()},
        'diagnostic_collection_coverage':{'actual_writer_detail_window_sim_s':[detail_begin,detail_end],
            'frozen_logging_trim_contract':trim_contract,'frozen_logging_trim_provenance':trim_provenance,
            'kind100_constraint_counts_outside_detail_status':'not_collected'if trim_contract is not None else'collected_by_frozen_original_logger',
            'VIO_kind201_optimizer_frames_outside_detail_status':'not_collected'if trim_contract is not None else'collected_by_frozen_original_logger',
            'outside_window_processed_handler_evidence':'kind12/13 retain actual processing flags and solve wall times; separate from unavailable kind100/201 detailed optimizer evidence'},
        'callback_wall_receipt_rates':{k:{'callback_count':len(v),
            'wall_span_average_hz':float((len(v)-1)/(v[-1]-v[0]))if len(v)>1 else None,
            'wall_gap_s':dist(np.diff(v)),
            'over_300ms_wall_gaps':int(np.sum(np.diff(v)>.3))}for k,v in receipts.items()},
        'actual_phase_begin_wall_progress':{stage:{'phases':len(v),
            'wall_span_average_hz':float((len(v)-1)/(v[-1]-v[0]))if len(v)>1 else None,
            'begin_to_same_stage_begin_wall_gap_s':dist(np.diff(v))}for stage,v in phase_begin_wall.items()},
        'observed_latest_raw_IMU_header_minus_phase_target_s':{stage:{window:dist(v)
            for(ss,window),v in internal_backlog.items()if ss==stage}for stage in phase_begin_wall},
        'stage_timing_scope':'Recorded phase_total ends immediately after StateEstimation/visual solve; excludes later voxel-map updates, colored-cloud/image/path publication, dispatch/wait and other phase costs. Begin-to-same-stage wall gap includes these and waiting; not exclusive optimizer CPU cost. Backlog uses latest raw IMU callback observed before kind10, not Gazebo truth.',
        'actual_estimator_stage_wall_latency':{label:{window:{name:{**dist(v),
            'over_300ms_count':int(np.sum(np.array(v)>.3)),
            'over_configured_lidar_period_count':int(np.sum(np.array(v)>1/sensor['rate_hz']))}
            for(ll,ww,name),v in timings.items()if ll==label and ww==window}
            for window in sorted({ww for ll,ww,name in timings if ll==label})}
            for label in ('LIO','VIO')},
        'record_kind_counts':dict(counts),'frozen_lio_schema':provenance,
        'index_sha256':sha(index),'selected_read_diagnostic_sha256':selected.hexdigest(),
        'literal_SDF_east_wall':wall,'sampled_east_wall_matching_geometry':geometry,
        'geometry_sampling':'first iteration, first actual frame in each integer sim second within actual writer detail window; no cross-run plane ID matching',
        'sampled_detail_window_plane_event_spectrum':{'actual_detail_window_sim_s':[detail_begin,detail_end],'event_rows':len(initial),
            'mid_over_max':dist(initial[:,2]/np.maximum(initial[:,3],1e-30))if len(initial)else None,
            'near_horizontal_normal_fraction':float(np.mean(initial[:,4]>=.8))if len(initial)else None,
            'meaning':'first event record in each integer second of actual detail window; event rows may revisit same plane, descriptive not unique plane count. Initial0–10s map events are not logged; historical spectra appear in actual accepted constraint rows.'},
        'writer_stats':writer}

def analyze(run,out):
    run=run.resolve();out.mkdir(parents=True,exist_ok=True)
    n,s,aligned,axis,anchor_t=native_and_slam(run)
    command=list(rows(run/'navigation_command_history.jsonl'))
    # Cleanup after the final physics sample naturally expires inputs and can
    # append failed bridge rows. Keep them separate from physical-time failure.
    failed=next((r for r in command if r.get('state')=='failed'and r['sim_time']<=n[-1,0]+1e-6),None)
    cleanup_failed=next((r for r in command if r.get('state')=='failed'and r['sim_time']>n[-1,0]+1e-6),None)
    first_nav_failed=next((r for r in rows(run/'navigation_status.jsonl')if r.get('state')=='failed'and r.get('ros_sim_time',0)<=n[-1,0]+1e-6),None)
    ft=failed.get('slam_stamp_ns',0)*1e-9 if failed and failed.get('slam_stamp_ns')else None
    failure_window_clock_source='first physical-time bridge failure original SLAM header'if ft is not None else None
    if ft is None and first_nav_failed is not None:
        ft=first_nav_failed['ros_sim_time'];failure_window_clock_source='first physical-time navigation failed status ROS clock (not original SLAM header)'
    windows={'whole_post_anchor':(anchor_t,float(s[-1,0])),'baseline_fault_window_131_134':(131.,134.)}
    if ft is not None:windows['5s_before_actual_first_failure']=(max(anchor_t,ft-5),ft)
    else:windows['last_5s_no_failed_guard']=(max(anchor_t,float(s[-1,0])-5),float(s[-1,0]))
    status=read(run/'navigation_status.json');profile=read(run/'navigation_profile.json');sensor=read(run/'sensor_contract.json')
    resource=list(rows(run/'owned_resources.jsonl'));mathstamps=[];tickerstamps=[];ages=defaultdict(list);active_ages=defaultdict(list)
    for r in rows(run/'navigation_pid_history.jsonl'):
        tickerstamps.append(r['compute_ros_clock_ns'])
        if r.get('cascade',{}).get('controller_updated'):mathstamps.append(r['source_pose_stamp_ns'])
        clock=r['compute_ros_clock_ns'];p=r.get('source_pose_stamp_ns');g=r.get('paired_imu_stamp_ns')
        tickages={}
        if p is not None:tickages['pose_sim_age_s']=(clock-p)*1e-9
        if g is not None:tickages['paired_IMU_sim_age_s']=(clock-g)*1e-9
        if r.get('feedback'):tickages['pose_wall_age_s']=r['compute_monotonic_wall']-r['feedback']['received_wall_ns']*1e-9
        for k,v in tickages.items():
            ages[k].append(v)
            if r.get('cascade',{}).get('protection_active')is not True:active_ages[k].append(v)
    ds=diagnostic_summary(run,n,axis)
    result={'schema':'actual_lidar_density_rate_comparison/v1','run':str(run),
        'truth_is_offline_only':True,'navigation_ground_truth_used':read(run/'navigation_scope.json')['navigation_ground_truth_used'],
        'reader_sha256':READER_SHA256,'sensor_contract':sensor,
        'configured_beams_per_frame':sensor['lidar']['samples']['horizontal']*sensor['lidar']['samples']['vertical'],
        'configured_beams_per_sim_second':sensor['lidar']['samples']['horizontal']*sensor['lidar']['samples']['vertical']*sensor['lidar']['rate_hz'],
        'configured_sensor_rates_hz':{k:sensor[k]['rate_hz']for k in ('lidar','camera','imu')},
        'declared_duration_s':profile['duration_s'],'actual_native_end_s':float(n[-1,0]),
        'offline_alignment_anchor_s':anchor_t,'physical_windows':{k:window_physical(n,s,aligned,*v)for k,v in windows.items()},
        'first_failed_guard':{k:failed.get(k)for k in ('sim_time','slam_stamp_ns','reason','registered_route_fence','ages')}if failed else None,
        'first_cleanup_bridge_failed_row':{k:cleanup_failed.get(k)for k in ('sim_time','slam_stamp_ns','reason')}if cleanup_failed else None,
        'dynamic_failure_window_clock_source':failure_window_clock_source,
        'region_arrivals':len(status.get('region_arrivals',[])),'total_regions':status.get('total'),
        'final_navigation_state':status['state'],'worker_result':read(run/'worker_result.json'),
        'first_navigation_failed_status':{k:first_nav_failed.get(k)for k in ('ros_sim_time','message','reason','failure','waypoint_index','total','pose')}if first_nav_failed else None,
        'runtime_completed':read(run/'runtime_manifest.json').get('runtime_status'),
        'whole_run_native_safety':{'samples':len(n),'body_contact_samples':int(np.sum(n[:,14]>0)),'fault_samples':int(np.sum(n[:,15]!=0)),
            'native_source_rate':rate(np.rint(n[:,0]*1e9).astype(np.int64))},
        'actual_navigation_SLAM_source_rate':rate(np.rint(s[:,0]*1e9).astype(np.int64)),
        'actual_controller_math_update_rate':rate(mathstamps),'actual_controller_ticker_rate':rate(tickerstamps),
        'controller_ages_all_modes':{k:{**dist(v),'over_300ms_count':int(np.sum(np.array(v)>.3))}for k,v in ages.items()},
        'controller_ages_without_protection':{k:{**dist(v),'over_300ms_count':int(np.sum(np.array(v)>.3))}for k,v in active_ages.items()},
        'shared_resources':{'logical_cpus':resource[0]['logical_cpus'],
            'load1':dist([r['load_1_5_15'][0]for r in resource]),
            'GPU_used_MiB':dist([r['gpu']['used_MiB']for r in resource if'gpu'in r]),
            'GPU_utilization_percent':dist([r['gpu']['utilization_percent']for r in resource if'gpu'in r]),
            'RAM_available_GiB':dist([r['memory_KiB']['MemAvailable']/1024**2 for r in resource]),
            'scope':'shared resources; not exclusive Teacher/Gazebo/SLAM GPU/CPU consumption'},
        'diagnostics':ds,
        'source_hashes':{name:sha(run/name)for name in ('sensor_contract.json','world.sdf','navigation_profile.json','navigation_fastlivo.yaml','policy_manifest.json','navigation_source_snapshots.json','navigation_scene_axis_registration.json')},
        'acceptance':'Descriptive independent comparison only. Common/ramp frozen evaluators retain all original thresholds; finite prefix without full32-region completion cannot pass navigation.'}
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    np.savetxt(out/'native_SLAM_aligned_trace.csv',np.c_[n[:,0],n[:,1:4],aligned,n[:,14:16]],delimiter=',',header='t,native_x,native_y,native_z,SLAM_aligned_native_x,SLAM_aligned_native_y,SLAM_aligned_native_z,body_contact,fault',comments='')
    np.savetxt(out/'SLAM_body_trace.csv',s,delimiter=',',header='t,x,y,z,qx,qy,qz,qw,vx_body,vy_body,vz_body,callback_age',comments='')
    fig,axes=plt.subplots(3,1,figsize=(13,10),sharex=True)
    axes[0].plot(n[:,0],aligned[:,2],label='Native body (OFFLINE aligned)');axes[0].plot(s[:,0],s[:,3],label='Actual SLAM body');axes[0].set_ylabel('Height [m]');axes[0].legend()
    native_at_s=np.interp(s[:,0],n[:,0],aligned[:,2]);axes[1].plot(s[:,0],s[:,3]-native_at_s);axes[1].set_ylabel('SLAM-native relative z [m]')
    cc=np.array([[r['sim_time'],*r['command']]for r in command]);axes[2].plot(cc[:,0],cc[:,1],label='Teacher vx request');axes[2].plot(cc[:,0],cc[:,3],label='Teacher wz request');axes[2].legend();axes[2].set_xlabel('Actual source/simulation time [s]')
    for ax in axes:
        ax.grid(alpha=.2)
        if ft is not None:ax.axvline(ft,color='red',ls=':')
    lid=sensor['lidar'];fig.suptitle(f"{lid['samples']['vertical']}x{lid['samples']['horizontal']} @{lid['rate_hz']}Hz LiDAR; camera {sensor['camera']['rate_hz']}Hz; actual navigation sources only")
    fig.tight_layout();fig.savefig(out/'height_commands_full.png',dpi=150)
    for ax in axes:ax.set_xlim(110,min(175,n[-1,0]))
    fig.savefig(out/'height_commands_original_fault_window.png',dpi=150);plt.close(fig)
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,action='append',required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True);reports=[]
    for run in args.run:reports.append(analyze(run,args.output/run.name))
    (args.output/'comparison.json').write_text(json.dumps({'schema':'lidar_density_rate_comparison_batch/v1','runs':reports},indent=2,allow_nan=False)+'\n')
    print(json.dumps([{'run':r['run'],'actual_lidar_rate':r['diagnostics']['rates'].get('actual_FASTLIVO_valid_PointCloud2_callback'),
        'LIO_rate':r['diagnostics']['rates'].get('actual_LIO_optimizer_frames'),'VIO_rate':r['diagnostics']['rates'].get('actual_VIO_optimizer_frames'),
        'controller_math_rate':r['actual_controller_math_update_rate'],'regions':str(r['region_arrivals'])+'/'+str(r['total_regions']),
        'first_failed_guard':r['first_failed_guard'],'physical_window131_134':r['physical_windows']['baseline_fault_window_131_134']}for r in reports],indent=2))

if __name__=='__main__':main()
