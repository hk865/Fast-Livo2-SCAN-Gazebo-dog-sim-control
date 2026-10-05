#!/usr/bin/env python3
"""Excluded, offline evidence reader; no ROS, control or source mutation."""
import collections, hashlib, json, math, pathlib, sys
import numpy as np

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def audit(p):
 r=json.loads((p/'first_eight_result.json').read_text());lo,hi=r['independent_truth_evaluation_interval'];lo_ns=round(lo*1e9);hi_ns=round(hi*1e9)
 samples=collections.defaultdict(list);counts=collections.Counter();names={};invalid=collections.Counter();max_qd={};max_tilt=(0.,None,None);cmd=[];latest_imu_ns=None
 with (p/'balance_inputs.jsonl').open() as f:
  for line in f:
   a=json.loads(line);k=a['kind'];counts[k]+=1;n=a['stamp_ns'];d=a['data'];w=a['wall_monotonic']
   if k=='imu':
    latest_imu_ns=n
    if lo_ns<=n<=hi_ns:
     x,y,z,qw=d['q'];norm=x*x+y*y+z*z+qw*qw;roll=math.atan2(2*(qw*x+y*z),norm-2*(x*x+y*y));pitch=math.asin(max(-1.,min(1.,2*(qw*y-z*x)/norm)));tilt=max(abs(roll),abs(pitch))
     if tilt>max_tilt[0]:max_tilt=(tilt,n,d['q'])
   if k in ('imu','joints','legacy_joints') and lo_ns<=n<=hi_ns:
    samples[k].append((n,w))
    if k!='imu':
     names.setdefault(k,d['names']);invalid[k]+=int(d['names']!=names[k] or len(d['positions'])!=12 or len(d['velocities'])!=12 or not all(math.isfinite(x) for x in d['positions']+d['velocities']))
     max_qd[k]=max(max_qd.get(k,0),max(abs(x) for x in d['velocities']))
   if k=='actual_actuator_command' and latest_imu_ns is not None and lo_ns<=latest_imu_ns<=hi_ns:cmd.append((latest_imu_ns,w,d['value']))
 arrays={k:np.asarray(v) for k,v in samples.items()};stats={}
 for k,a in arrays.items():
  ns=np.array([int(x[0]) for x in samples[k]],dtype='int64');wall=a[:,1];dn=np.diff(ns);dw=np.diff(wall);j=int(np.argmax(dw));h=int(np.argmax(dn));expected=1_000_000 if k!='legacy_joints' else None
  stats[k]=dict(count=len(ns),first_ns=int(ns[0]),last_ns=int(ns[-1]),header_delta_ns=dict(minimum=int(dn.min()),maximum=int(dn.max()),zero=int((dn==0).sum()),backward=int((dn<0).sum()),non_1ms=int((dn!=1_000_000).sum()) if expected else None),wall_gap_s=dict(maximum=float(dw.max()),p99=float(np.quantile(dw,.99)),max_interval_ns=[int(ns[j]),int(ns[j+1])],max_header_interval_ns=[int(ns[h]),int(ns[h+1])]),schema_invalid=int(invalid[k]) if k!='imu' else None,max_observed_velocity_rad_s=max_qd.get(k))
 old=arrays['legacy_joints'];new=arrays['joints'];imu=arrays['imu'];old_dw=np.diff(old[:,1]);intervals=[]
 for j in np.flatnonzero(old_dw>.030):
  a,b=old[j],old[j+1];ni=np.flatnonzero((new[:,1]>a[1])&(new[:,1]<b[1]));ii=np.flatnonzero((imu[:,1]>a[1])&(imu[:,1]<b[1]));intervals.append(dict(legacy_header_ns=[int(a[0]),int(b[0])],wall_gap_s=float(b[1]-a[1]),new_joint_callbacks_between=int(len(ni)),imu_callbacks_between=int(len(ii)),new_header_range_ns=None if not len(ni) else [int(new[ni[0],0]),int(new[ni[-1],0])]))
 kinds=collections.Counter();jt=[]
 with (p/'joint_stop_adapter.jsonl').open() as f:
  for line in f:
   a=json.loads(line);kinds[a['kind']]+=1
   if a['kind']=='actual_champ_command' and lo<=a.get('sim',-1)<=hi:jt.append((a['sim'],a['wall']))
 actualcmd=np.asarray([(x[0],x[1]) for x in cmd]);actualgaps=np.diff(actualcmd[:,1]);jointgaps=np.diff(np.array(jt)[:,1]) if len(jt)>1 else []
 qn_read=json.loads((p/'first8_manifest.json').read_text())
 return dict(scope='Read-only actual callback/header continuity. Callback delay does not uniquely identify upstream publisher stall. New measured q freshness is not proof of CM/JTC/actuator uninterrupted delivery.',run=p.name,original_result_sha256=sha(p/'first_eight_result.json'),actual_domain_ns=[lo_ns,hi_ns],counts_all=counts,actual_raw_IMU_safety=dict(metric='max(abs(roll),abs(pitch)); identical to ImuSafetyGate',maximum_rad=max_tilt[0],stamp_ns=max_tilt[1],quaternion=max_tilt[2],recorded_active_max=r['active_max_imu_tilt']),streams=stats,legacy_wall_gaps_above_30ms=intervals,actual_body_command_callback=dict(count=len(cmd),maximum_wall_gap_s=float(actualgaps.max()),topic_scope='Actual CHAMP body twist output observed by passive recorder, not joint actuator receipt'),adapter_log=dict(kinds=kinds,actual_champ_command_count=len(jt),max_logged_wall_gap_s=None if not len(jointgaps) else float(max(jointgaps))),boundary_limitations=dict(actual_published_joint_references_logged=True,JTC_reference_and_effort_recorded=False,hardware_or_sim_actuator_receipt_recorded=False,CM_publish_stall_root_cause_proven=False),measured_topic=qn_read.get('measured_input'))
if __name__=='__main__':
 p=pathlib.Path(sys.argv[1]);out=pathlib.Path(sys.argv[2]);r=audit(p);out.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({k:r[k] for k in ['run','actual_raw_IMU_safety','streams','actual_body_command_callback','adapter_log','boundary_limitations']},indent=2));print('legacy_gaps_above30ms',len(r['legacy_wall_gaps_above_30ms']))
