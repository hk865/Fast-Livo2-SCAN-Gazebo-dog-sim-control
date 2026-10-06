"""Independent actual receipt audit. Never imports or reruns frozen analyze/register."""
from collections import Counter
import hashlib,json,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[2]
RUN=ROOT/'runs/20261004_150720_stand_ramp12_registration_stand_v1_r1_2071'
CAP=BASE/'capture';REG=BASE/'registration'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return json.loads(Path(p).read_text())
def lines(p):return [json.loads(x)for x in Path(p).read_text().splitlines()if x.strip()]
def stats(values):
 a=np.asarray(values,float);a=a[np.isfinite(a)]
 return {'n':len(a),'min':float(a.min()),'p50':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(a.max())}if len(a)else {'n':0}
checks=[]
def check(name,condition,**metrics):checks.append({'name':name,'passed':bool(condition),**metrics})
manifest=load(CAP/'capture_manifest.json');reg=load(REG/'registration.json');runtime=load(RUN/'runtime_manifest.json')
streams={s:lines(CAP/(s+'.jsonl'))for s in ('imu','imu_slam_odom','body_odom','full_cloud')}
cloud=streams['full_cloud'];admitted=[x for x in cloud if x['geometry_admitted']]
check('capture_complete_drained_no_error',manifest['status']=='observation_complete'and manifest['writer_drained']and not manifest['errors']and manifest['submitted']==manifest['processed'])
check('JSONL_manifest_counts_identical',all(len(v)==manifest['counts'][k]for k,v in streams.items())and cloud==manifest['clouds'])
check('readonly_actual_interfaces',manifest['actual_node_interface_counts']=={'publishers':0,'services':0,'clients':0,'subscriptions':5}and manifest['control_outputs']==0 and not manifest['truth_topics_subscribed'])
check('producer_configuration_hashes',all(sha(p)==h and sha(CAP/'producer_configuration'/Path(p).name)==h for p,h in manifest['producer_configuration']['source_hashes'].items()))
check('frozen_core_live_and_archived_hashes',all(sha(CAP/'sources'/n)==h and sha(ROOT/'navigation/ramp_registration'/n)==h for n,h in manifest['source_freeze']['source_hashes'].items()))
check('archived_dependency_hashes',all(sha(CAP/'sources/dependencies'/n)==r['sha256'] for n,r in manifest['source_freeze']['source_contract_refs'].items()))
check('frozen_registration_content_and_file_receipts',sha(REG/'registration.json')==load(REG/'receipt_hash.json')['sha256']and hashlib.sha256(json.dumps({k:v for k,v in reg.items()if k!='registration_content_sha256'},sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()==reg['registration_content_sha256'])
check('analysis_references_and_snapshots',all(sha(p)==h for p,h in reg['input_source_hashes'].items())and all(sha(p)==h and sha(REG/'sources'/Path(p).name)==h for p,h in reg['analysis_source_hashes'].items()))
source=load(RUN/'source_manifest.json')
check('actual_run_archived_core_helper_runner',all(source[str(p.relative_to(ROOT))]==sha(p)==sha(RUN/'sources'/p.relative_to(ROOT))for p in [ROOT/'scripts/run_test.py',ROOT/'navigation/ramp_registration/producer_profile.py',*[ROOT/'navigation/ramp_registration'/n for n in manifest['source_freeze']['source_hashes']]]))
check('six_main_owned_exit_zero',len(runtime['owned_processes'])==6 and all(x['returncode']==0 for x in runtime['owned_processes'])and runtime['error']is None)
log=(RUN/'navigation_stack.log').read_text();started=re.findall(r'process started with pid \[(\d+)\]',log);clean=re.findall(r'process has finished cleanly \[pid (\d+)\]',log)
check('six_actual_SLAM_children_clean',len(started)==6 and Counter(started)==Counter(clean),started_pids=started,clean_pids=clean)
proc=load(RUN/'root_owned_runtime_process_snapshot.json')['processes'];commands=[p['command']for p in proc]
mapper=[p for p in proc if str(ROOT.parent/'slam/ros2_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping')in p['command']]
check('actual_mapper_run_configuration_and_parent_identity',len(mapper)==1 and str(RUN/'navigation_fastlivo.yaml')in mapper[0]['command'] and str(RUN/'navigation_camera.yaml')in mapper[0]['command'] and mapper[0]['ppid']==next(p['pid']for p in runtime['owned_processes']if p['role']=='navigation_stack'))
exit=load(BASE/'root_owned_capture_exit.json');check('observer_owned_identity_and_exit_zero',exit['returncode']==0 and exit['pid']==manifest['pid'] and str(ROOT/'navigation/ramp_registration/observer.py')in exit['command'] and str(RUN)in exit['command'])
transport=load(RUN/'cloud_transport_manifest.json')
check('actual_observer_transport_environment',manifest['rmw_implementation_identifier']=='rmw_fastrtps_cpp'and manifest['environment']['ROS_DOMAIN_ID']=='79'and all(manifest['environment'].get(k)==v for k,v in transport['environment'].items())and manifest['environment']['environment_manifest']['sha256']==sha(RUN/'cloud_transport_manifest.json'))
check('no_navigation_executed_or_claimed',runtime['test']=='stand'and runtime['slam_registration_only']['navigation_started']is False and not reg['registration_control_enabled']and not reg['navigation_request_emitted']and not reg['full_route_eligible']and reg['full_route']is None)
raw_errors=[];padding_different=0;named_fields_ok=0;points=0;bytes_total=0
for row in cloud:
 path=CAP/row['archive_file']
 if sha(path)!=row['archive_sha256']:raw_errors.append([row['sequence'],'archive_hash'])
 with np.load(path,allow_pickle=False)as f:
  raw=f['raw'].tobytes(); xyz=f['xyz'];fields=f['fields'];n=row['width']*row['height']
  if hashlib.sha256(raw).hexdigest()!=row['raw_payload_sha256']or len(raw)!=row['raw_payload_bytes']:raw_errors.append([row['sequence'],'raw_payload'])
  if hashlib.sha256(xyz.tobytes()).hexdigest()!=row['decoded_xyz_sha256']:raw_errors.append([row['sequence'],'xyz_hash'])
  expected=[];same=True
  for info in row['fields']:
   # Actual stand2071 all scalar float32; explicit guard prevents assumed layouts.
   if info['datatype']!=7 or info['count']!=1:raw_errors.append([row['sequence'],'unexpected_field_schema']);continue
   view=np.ndarray((row['height'],row['width']),dtype='>f4'if row['is_bigendian']else'<f4',buffer=raw,offset=info['offset'],strides=(row['row_step'],row['point_step'])).reshape(-1)
   same&=(view.tobytes()==fields[info['name']].tobytes())
   if info['name']in('x','y','z'):expected.append((info['name'],view.astype(np.float64)))
  values=np.column_stack([dict(expected)[name]for name in ('x','y','z')]);finite=np.isfinite(values).all(axis=1)
  if not same:raw_errors.append([row['sequence'],'named_fields_changed'])
  if not np.array_equal(np.flatnonzero(finite),f['finite_source_indices'])or not np.array_equal(values[finite],xyz):raw_errors.append([row['sequence'],'XYZ_original_redecode'])
  named_fields_ok+=int(same);padding_different+=int(fields.tobytes()!=raw);points+=len(xyz);bytes_total+=len(raw)
check('all571_actual_raw_archives_fields_XYZ_verified',not raw_errors,archives=len(cloud),finite_point_observations=points,original_payload_bytes=bytes_total,errors=raw_errors)
check('no_decode_error_or_nonfinite_points',all(x['decode_error']is None and x['nonfinite_points']==0 for x in cloud))
check('strict_increasing_cloud_headers',all(b['original_stamp_ns']>a['original_stamp_ns']for a,b in zip(cloud,cloud[1:])))
check('admitted_cloud_original_freshness',all(-50_000_000<=x['received_ros_clock_ns']-x['original_stamp_ns']<300_000_000 and 0<=x['received_clock_wall_age_s']<.3 and abs(x['original_stamp_ns']-x['body_pose']['original_stamp_ns'])<=150_000_000 and 0<=x['received_monotonic_wall']-x['body_pose']['received_monotonic_wall']<.3 for x in admitted))
byimu={x['original_stamp_ns']:x for x in streams['imu']};byip={x['original_stamp_ns']:x for x in streams['imu_slam_odom']};bybp={x['original_stamp_ns']:x for x in streams['body_odom']}
g=manifest['gravity'];ups=[];gaps=[];wallgaps=[];speeds=[];gyros=[];accnorm=[];gerror=[]
for entry in g['calibration_sample_receipts']:
 i=byimu[entry['imu_stamp_ns']];ip=byip[entry['slam_imu_pose_stamp_ns']];pair=i['gravity_pairing'];bp=bybp[pair['body_pose_stamp_ns']]
 a=np.asarray(i['linear_acceleration']);norm=np.linalg.norm(a);v=Rotation.from_quat(ip['quaternion']).apply(a)/norm
 if not np.allclose(v,entry['transformed_up_camera_init'],rtol=0,atol=1e-14):gerror.append(entry['imu_stamp_ns'])
 ups.append(v);gaps.extend([(i['original_stamp_ns']-ip['original_stamp_ns'])*1e-9,(i['original_stamp_ns']-bp['original_stamp_ns'])*1e-9]);wallgaps.extend([i['received_monotonic_wall']-ip['received_monotonic_wall'],i['received_monotonic_wall']-bp['received_monotonic_wall']]);speeds.append(np.linalg.norm(bp['body_linear_velocity']));gyros.append(np.linalg.norm(i['angular_velocity']));accnorm.append(norm)
mean=np.mean(ups,axis=0);mean/=np.linalg.norm(mean);spread=np.max(np.arccos(np.clip(np.asarray(ups)@mean,-1,1)))
check('actual_gravity_all100_pairing_recomputed',not gerror and np.max(np.abs(mean-g['up_camera_init']))<1e-14 and max(gaps)<=.15 and min(gaps)>=0 and min(wallgaps)>=0 and max(wallgaps)<.3 and max(speeds)<=.05 and max(gyros)<=.1 and min(accnorm)>=9 and max(accnorm)<=10.6 and g['samples']>=50 and g['last_stamp_ns']-g['first_stamp_ns']>=500_000_000 and g['unique_slam_imu_pose_samples']>=4 and spread<=np.deg2rad(3),mean_max_error=float(np.max(np.abs(mean-g['up_camera_init']))),source_gap_s=stats(gaps),receive_pair_gap_s=stats(wallgaps),maximum_body_speed_mps=max(speeds),maximum_gyro_radps=max(gyros),acceleration_norm_mps2=stats(accnorm))
with np.load(REG/'observed_xyz.npz',allow_pickle=False)as f:xyz=f['xyz']
anchor=np.array(reg['body_anchor']);up=np.array(reg['up_camera_init']);q=manifest['body_anchor']['quaternion'];forward=Rotation.from_quat(q).apply([1,0,0]);forward-=forward@up*up;forward/=np.linalg.norm(forward);lateral=np.cross(up,forward)
d=xyz-anchor;uvh=np.column_stack([d@forward,d@lateral,d@up]);radius=np.linalg.norm(uvh[:,:2],axis=1);support_band=(uvh[:,2]>-.65)&(uvh[:,2]<-.15)
# Secondary candidate search only in an actually observed local body-relative ROI.
# It neither certifies double edges/platforms nor outputs any route.
roi=(radius<=3)&support_band;local=xyz[roi];rng=np.random.default_rng(204);best=None;count=0
for _ in range(800):
 a,b,c=local[rng.choice(len(local),3,replace=False)];normal=np.cross(b-a,c-a);norm=np.linalg.norm(normal)
 if norm<1e-8:continue
 normal/=norm
 if normal@up<0:normal=-normal
 tilt=np.rad2deg(np.arccos(np.clip(normal@up,-1,1)))
 if not 3<=tilt<=20:continue
 mask=np.abs((local-a)@normal)<=.025
 if mask.sum()>count:count=int(mask.sum());best=(a,normal,mask)
diag=None
if best is not None:
 core=local[best[2]];center=core.mean(axis=0);_,_,vh=np.linalg.svd(core-center,full_matrices=False);normal=vh[-1]
 if normal@up<0:normal=-normal
 residual=np.abs((local-center)@normal);mask=residual<=.025
 diag={'status':'diagnostic_candidate_only','source':'actual local XYZ, frozen actual SLAM body anchor, measured up','ROI_horizontal_radius_m':3.,'ROI_body_height_band_m':[-.65,-.15],'ROI_points':len(local),'observed_inliers':int(mask.sum()),'normal_camera_init':normal.tolist(),'center_camera_init':center.tolist(),'tilt_deg':float(np.rad2deg(np.arccos(np.clip(normal@up,-1,1)))),'rms_m':float(np.sqrt(np.mean(residual[mask]**2))),'route_emitted':False,'full_ramp_certified':False,'scan_field_or_world_geometry_used':False}
plt.rcParams.update({'font.size':10})
fig,ax=plt.subplots(2,2,figsize=(12,8));sample=np.arange(0,len(xyz),max(1,len(xyz)//22000));sc=ax[0,0].scatter(uvh[sample,0],uvh[sample,1],c=uvh[sample,2],s=2,cmap='viridis',vmin=-.5,vmax=2.5);ax[0,0].scatter([0],[0],c='red',marker='x',s=65);ax[0,0].set_aspect('equal');ax[0,0].set(xlabel='Actual initial SLAM body forward projection [m]',ylabel='Actual initial SLAM body lateral projection [m]',title='Measured XYZ only; no route / no SDF');fig.colorbar(sc,ax=ax[0,0],label='Height along measured gravity-up [m]')
ax[0,1].scatter(uvh[roi,0],uvh[roi,2],s=2,alpha=.3,label='Actual local support-height band');ax[0,1].axhline(0,c='red',label='SLAM body anchor');ax[0,1].set(xlabel='Body forward projection [m]',ylabel='Measured-up height relative to anchor [m]',title='Local observations; secondary fit is diagnostic');ax[0,1].legend(loc='best')
ctime=np.array([x['original_stamp_ns']*1e-9 for x in cloud]);age=[(x['received_ros_clock_ns']-x['original_stamp_ns'])*1e-9 for x in cloud];gap=[abs(x['original_stamp_ns']-x['body_pose']['original_stamp_ns'])*1e-9 for x in cloud];ax[1,0].plot(ctime,age,label='Original source age');ax[1,0].plot(ctime,gap,label='Body/source pair gap');ax[1,0].axhline(.3,c='red',ls='--',label='Source freshness limit');ax[1,0].set(xlabel='Original actual cloud stamp [sim s]',ylabel='Seconds',title='No refreshed source timestamps');ax[1,0].legend()
ax[1,1].bar(range(10),[np.rad2deg(p['tilt_rad'])for p in reg['support_planes']],label='Frozen candidate inclination');ax[1,1].axhline(1.5,c='red',ls='--',label='Horizontal cap');ax[1,1].set(xlabel='Frozen support plane candidate ID',ylabel='Inclination to measured up [degrees]',title='Frozen max_planes=10 reached; no ramp retained');ax[1,1].legend()
fig.tight_layout();fig.savefig(BASE/'actual_cloud_and_freshness.png',dpi=170);plt.close(fig)
rejections=Counter(r for row in cloud for r in row['geometry_rejection_reasons'])
result={'schema':'independent_actual_ramp_capture_audit/v1','scope':'stand2071 only; no motion/complete ramp/navigation pass','status':'passed'if all(c['passed']for c in checks)else'failed','checks':checks,'actual_registration_status':reg['status'],'registration_rejection_reasons':reg['rejection_reasons'],'full_route_eligible':False,'navigation_verified':False,
 'capture':{k:manifest[k]for k in ('first_actual_clock_ns','last_actual_clock_ns','duration_sim_s','status','counts','submitted','processed','writer_drained','peak_queue_bytes','archive_bytes','recorded_clouds','admitted_clouds','actual_node_interface_counts')},
 'timing':{stream:{'original_header_interval_s':stats(np.diff([x['original_stamp_ns']for x in rows])*1e-9),'callback_interval_wall_s':stats(np.diff([x['received_monotonic_wall']for x in rows])),'queue_delay_wall_s':stats([x['background_queue_delay_wall_s']for x in rows])}for stream,rows in streams.items()},
 'cloud_header_age_s':stats(age),'cloud_body_source_pair_gap_s':stats(gap),'cloud_body_receive_age_wall_s':stats([x['received_monotonic_wall']-x['body_pose']['received_monotonic_wall']for x in cloud]),'body_header_future_to_cloud_count':sum(x['body_pose']['original_stamp_ns']>x['original_stamp_ns']for x in cloud),'cloud_rejections':dict(rejections),
 'raw_layout':{'point_step':sorted({x['point_step']for x in cloud}),'fields':cloud[0]['fields'],'payload_bytes':stats([x['raw_payload_bytes']for x in cloud]),'named_fields_exact_archives':named_fields_ok,'structured_padding_differs_archives':padding_different,'padding_limit':'Original raw payload bytes are authoritative and fully hash verified; structured named fields match. NumPy structured serialization padding is not a preserved raw message and must not replace raw.'},
 'actual_gravity':{'up_camera_init':g['up_camera_init'],'samples':g['samples'],'unique_slam_imu_pose_samples':g['unique_slam_imu_pose_samples'],'first_stamp_ns':g['first_stamp_ns'],'last_stamp_ns':g['last_stamp_ns'],'max_axis_spread_rad':spread,'recomputed_pair_count':len(ups),'actual_accelerometer_and_raw_SLAM_quaternion_used':True,'IMU_orientation_or_world_Z_used':False},
 'geometry_diagnostic':{'frozen_support_planes':reg['support_planes'],'max_plane_budget_exhausted':len(reg['support_planes'])==reg['configuration']['max_planes'],'frozen_ramp_candidates':len(reg['ramp_candidates']),'retained_actual_points':len(xyz),'actual_point_extents_camera_init_m':[xyz.min(axis=0).tolist(),xyz.max(axis=0).tolist()],'local_current_support_points_within045m':int(np.sum(support_band&(radius<.45))),'nearest_observed_support_band_horizontal_distance_m':float(np.min(radius[support_band])),'support_band_counts_by_radius_m':{str(r):int(np.sum(support_band&(radius<r)))for r in [.45,.6,1,2,3]},'secondary_local_plane_diagnostic':diag,'explanation':'Frozen algorithm is horizontal-first with one shared 10-plane budget. It stops at that budget with 10 horizontal candidates. This does not establish that the measured cloud contains no ramp-like returns. Local radius .45 has no plausible support-height observations; hence the frozen current-support-layer gate fails.'},
 'processes':{'root_live_snapshot_process_count':len(proc),'observer_pid':manifest['pid'],'observer_exit':exit['returncode'],'main_owned':runtime['owned_processes'],'SLAM_started_pids':started,'SLAM_clean_pids':clean,'single_mapper_actual_run_args':mapper[0]['command'],'DDS_sequence_metadata':'All captured MessageInfo fields null; no per-DDS-sequence loss claim'},
 'prior9b70':'Configuration preparation refused old flat spawn gate before actual physics; retained as prepare failure, not movement failure.',
 'sources':{str(p):sha(p)for p in [Path(__file__),CAP/'capture_manifest.json',CAP/'gravity.json',REG/'registration.json',REG/'receipt_hash.json',REG/'observed_xyz.npz',RUN/'runtime_manifest.json',RUN/'root_owned_runtime_process_snapshot.json',RUN/'navigation_stack.log',RUN/'source_manifest.json',BASE/'root_owned_capture_started.json',BASE/'root_owned_capture_exit.json',*[CAP/(s+'.jsonl')for s in streams]]}}
(BASE/'AUDIT.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps({'status':result['status'],'checks':len(checks),'failures':[x['name']for x in checks if not x['passed']],'registration':reg['status'],'rejections':dict(rejections),'local_diagnostic':diag},ensure_ascii=False))
