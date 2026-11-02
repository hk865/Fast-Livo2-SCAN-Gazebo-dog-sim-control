"""Offline fixed-frame GT scoring. Never imported by localization or control."""
from pathlib import Path
import argparse,collections,csv,json
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

def rows(path):
 if not path.exists():return[]
 return[json.loads(line)for line in path.open()if line.strip()]
def distribution(values):
 values=np.asarray(values,dtype=float)
 return None if not len(values)else dict(p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),maximum=float(np.max(values)))
def correction_latency(poses,decisions,applications):
 if applications:
  return dict(method='Explicit owner application event timestamps; simulation source age and wall publication delay are separate',applications=len(applications),source_to_apply_sim_ms=distribution([r['source_to_apply_sim_ms']for r in applications]),backend_publish_to_apply_wall_ms=distribution([r['backend_publish_to_apply_wall_ms']for r in applications]))
 first={}
 for p in poses:
  if p['generation']>0:first.setdefault(p['generation'],p)
 accepted=[r for r in decisions if r.get('accepted')and'C_world_odom'in r]
 delays=[];ambiguous=0;unmatched=0
 for generation,p in first.items():
  c=np.asarray(p['T_world_body'])@np.linalg.inv(np.asarray(p['raw_T_odom_body']))
  matches=[r for r in accepted if r['source_ns']<=p['stamp_ns']and np.allclose(c,r['C_world_odom'],atol=1e-8,rtol=0)]
  if len(matches)==1:delays.append((p['stamp_ns']-matches[0]['source_ns'])/1e6)
  elif matches:ambiguous+=1
  else:unmatched+=1
 return dict(method='Offline exact-matrix association of each first corrected pose generation to backend source; measures first output rather than timer application',associated=len(delays),ambiguous_associations=ambiguous,unmatched=unmatched,source_to_first_corrected_output_sim_ms=distribution(delays))

def score(run):
 setup=json.loads((run/'experiment_setup.json').read_text());poses=rows(run/'slam_poses.jsonl');truth=rows(run/'native_pose_source.jsonl');origin=np.asarray(setup['navigation_world_origin']);result=dict(run=run.name,mode=setup.get('navigation_mode'),GT_used_only_in_this_offline_score=True,navigation_ground_truth_used=setup.get('navigation_ground_truth_used'),pose_samples=len(poses))
 if truth and poses:
  stamps=np.asarray([r['stamp_ns']for r in truth],dtype=np.int64);position=np.asarray([r['position']for r in truth]);q=np.asarray([r['quaternion_wxyz'][1:]+r['quaternion_wxyz'][:1]for r in truth]);rotation=Slerp((stamps-stamps[0])/1e9,Rotation.from_quat(q));raw_error=[];error=[];angles=[];ages=[];locations=[]
  for r in poses:
   stamp=r['stamp_ns']
   if not stamps[0]<=stamp<=stamps[-1]:continue
   p=np.array([np.interp(stamp,stamps,position[:,i])for i in range(3)]);T=np.asarray(r['T_world_body']);raw=np.asarray(r['raw_T_odom_body']);error.append(float(np.linalg.norm(T[:3,3]-p)));raw_error.append(float(np.linalg.norm(raw[:3,3]+origin-p)));angles.append(float((Rotation.from_matrix(T[:3,:3]).inv()*rotation([(stamp-stamps[0])/1e9])[0]).magnitude()));ages.append(r['source_age_ms']);locations.append((stamp,p))
  result.update(corrected_position_error_m=distribution(error),raw_configured_prior_position_error_m=distribution(raw_error),corrected_orientation_error_rad=distribution(angles),source_age_ms=distribution(ages),source_age_over_300ms=int(np.sum(np.asarray(ages)>300)),position_RMSE_m=float(np.sqrt(np.mean(np.square(error))))if error else None,raw_position_RMSE_m=float(np.sqrt(np.mean(np.square(raw_error))))if raw_error else None,final_correction_generation=poses[-1]['generation'])
 for filename,name in [('backend_spool/bridge_status.json','bridge'),('backend_status.json','backend'),('slam_status.json','localization')]:
  p=run/filename
  if p.exists():result[name]=json.loads(p.read_text())
 history=list((run/'backend_history').glob('epoch_*/history.json'));result['history_keyframes']=0;result['persisted_submaps']=0;stages=collections.Counter()
 for p in history:
  h=json.loads(p.read_text());result['history_keyframes']+=len(h['keyframes']);result['persisted_submaps']+=len(h.get('submap_assets',[]));stages.update(k['final_stage']for k in h['keyframes'])
 result['keyframe_final_stages']=dict(stages)
 resources=run/'process_resources.csv'
 if resources.exists():
  usage=collections.defaultdict(list);rss=collections.defaultdict(list)
  for r in csv.DictReader(resources.open()):usage[r['role']].append(float(r['cpu_cores_interval']));rss[r['role']].append(float(r['rss_mib']))
  result['resources']={k:dict(cpu_cores=distribution(v),peak_rss_mib=max(rss[k]))for k,v in usage.items()}
 decisions=rows(run/'backend_results.jsonl')
 if decisions:result['correction_decisions']=dict(processed=len(decisions),accepted=sum(r.get('accepted')is True for r in decisions),reasons=dict(collections.Counter(r.get('reason')for r in decisions)),registration_ms=distribution([r['registration_elapsed_s']*1000 for r in decisions if 'registration_elapsed_s'in r]),backend_total_ms=distribution([r['backend_wall_s']*1000 for r in decisions if 'backend_wall_s'in r]),queue_wait_ms=distribution([r['queue_wait_s']*1000 for r in decisions if 'queue_wait_s'in r]))
 result['correction_latency']=correction_latency(poses,decisions,rows(run/'correction_applications.jsonl'))
 p=run/'slam_backend_score.json';p.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');return result
if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path);args=parser.parse_args();print(json.dumps(score(args.run),ensure_ascii=False,indent=2))
