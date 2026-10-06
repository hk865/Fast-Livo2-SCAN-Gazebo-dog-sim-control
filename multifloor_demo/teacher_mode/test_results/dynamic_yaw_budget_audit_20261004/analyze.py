#!/usr/bin/env python3
"""Read-only comparison of executed turn commands, body response and NAV phases."""
from collections import Counter
from pathlib import Path
import hashlib,json,math
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
IDS=['20261004_082258_navigation_slam_scan_roundtrip_v4_r1_9c33',
'20261004_082715_navigation_slam_scan_roundtrip_v4_confirm_r1_1c57',
'20261004_083026_navigation_slam_scan_roundtrip_v4_confirm_r2_bd19',
'20261004_091122_navigation_slam_scan_atomic_transport_v41_r1_dba1',
'20261004_091513_navigation_slam_scan_dynamic_flat_v41_r1_6614',
'20261004_092935_navigation_slam_scan_dynamic_flat_v42_cleanup_r1_10c7']

def load(p):return [json.loads(x)for x in p.read_text().splitlines()if x.strip()]
def yaw(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def integral(t,a):return float(np.sum(a[:-1]*np.diff(t)))if len(t)>1 else 0.
def timing(t,mask):return integral(t,mask.astype(float))
def segments(t,mask):
    if not len(t):return []
    diff=np.diff(np.r_[False,mask,False].astype(int));starts=np.where(diff==1)[0];ends=np.where(diff==-1)[0]
    return [{'start_s':float(t[s]),'end_s':float(t[min(e,len(t)-1)]),'duration_s':float(t[min(e,len(t)-1)]-t[s])}for s,e in zip(starts,ends)]

results=[]
for rid in IDS:
    run=ROOT/'runs'/rid;nav=load(run/'navigation_status.jsonl');telem=load(run/'telemetry.jsonl');poses=load(run/'navigation_slam_poses.jsonl')
    returning=[n for n in nav if n.get('waypoint_index')==1]
    if not returning:raise RuntimeError('No return stage: '+rid)
    start=float(returning[0]['ros_sim_time']);terminal=next((n for n in nav if n['ros_sim_time']>=start and n['state']in ('succeeded','failed')),returning[-1]);end=float(terminal['ros_sim_time'])
    nrows=[n for n in nav if start<=n['ros_sim_time']<=end];nt=np.array([n['ros_sim_time']for n in nrows]);phases=[n.get('teacher_transition',{}).get('phase','missing')for n in nrows]
    phase_durations={p:timing(nt,np.array([x==p for x in phases]))for p in set(phases)}
    phase_changes=[{'sim_s':float(nt[i]),'before':phases[i-1],'after':phases[i]}for i in range(1,len(phases))if phases[i]!=phases[i-1]]
    tr=[r for r in telem if start<=r['world_sim_time']<=end];t=np.array([r['world_sim_time']for r in tr]);cmd=np.array([r['command']for r in tr]);req=np.array([r['requested']for r in tr]);actual=np.array([r['measured']for r in tr]);position=np.array([r['position']for r in tr]);physical_yaw=np.unwrap(np.array([r['rpy'][2]for r in tr]))
    accepted=np.array([r.get('navigation_envelope',{}).get('read_status')=='accepted'for r in tr]);expired=np.array([bool(r['command_expired'])for r in tr]);turn=(np.abs(cmd[:,2])>=.01)&(np.linalg.norm(cmd[:,:2],axis=1)<.005);drive=np.linalg.norm(cmd[:,:2],axis=1)>=.005;saturation=np.abs(cmd[:,2])>=.119
    turn_seg=segments(t,turn);drive_seg=segments(t,drive);expire_seg=segments(t,expired)
    errors=np.array([n.get('steering',{}).get('error',math.nan)if n.get('steering')else math.nan for n in nrows])
    p=[x for x in poses if start<=x['stamp_ns']/1e9<=end];pt=np.array([x['stamp_ns']/1e9 for x in p]);py=np.unwrap(np.array([yaw(x['quaternion'])for x in p]));pxy=np.array([x['position'][:2]for x in p])
    def value(mask):
        return {'samples':int(mask.sum()),'duration_s':timing(t,mask),'command_wz_mean_radps':float(np.mean(cmd[mask,2]))if mask.any()else None,
         'actual_body_wz_mean_radps':float(np.mean(actual[mask,2]))if mask.any()else None,
         'actual_body_wz_median_radps':float(np.median(actual[mask,2]))if mask.any()else None,
         'actual_wz_integral_rad':integral(t,actual[:,2]*mask),'command_wz_integral_rad':integral(t,cmd[:,2]*mask)}
    result={'run_id':rid,'return_start_s':start,'return_terminal_s':end,'return_duration_s':end-start,'return_terminal_state':terminal['state'],
     'phase_duration_estimate_s':phase_durations,'phase_changes':phase_changes,'first_return_drive_command_s':drive_seg[0]['start_s']if drive_seg else None,
     'turn':value(turn),'turn_saturated_at_012':value(turn&saturation),'drive':value(drive),'expired_duration_s':timing(t,expired),
     'accepted_duration_s':timing(t,accepted),'zero_cmd_duration_s':timing(t,np.linalg.norm(cmd,axis=1)<1e-9),
     'turn_command_segments':turn_seg,'expired_segments':expire_seg,'bridge_recoveries':int(np.sum(np.diff(expired.astype(int))==-1)),
     'physical_yaw_change_rad':float(physical_yaw[-1]-physical_yaw[0]),'slam_yaw_change_rad':float(py[-1]-py[0]),
     'physical_turn_xy_displacement_m':float(np.linalg.norm(position[np.where(turn)[0][-1],:2]-position[np.where(turn)[0][0],:2]))if turn.any()else None,
     'physical_return_total_xy_path_length_m':float(np.sum(np.linalg.norm(np.diff(position[:,:2],axis=0),axis=1))),
     'physical_return_displacement_xy_m':(position[-1,:2]-position[0,:2]).tolist(),'slam_return_displacement_xy_m':(pxy[-1]-pxy[0]).tolist(),
     'heading_error_start_rad':float(errors[np.where(np.isfinite(errors))[0][0]]),'heading_error_end_rad':float(errors[np.where(np.isfinite(errors))[0][-1]]),
     'final_return_center_error_m':terminal.get('region_arrival_evidence',{}).get('center_error_m'),
     'source_hashes':{name:hashlib.sha256((run/name).read_bytes()).hexdigest()for name in('navigation_status.jsonl','navigation_slam_poses.jsonl','telemetry.jsonl')},
     'scope_note':'SLAM used for navigation; physics/world position and angular rate only diagnosis. Durations from50Hz telemetry; phase left-hold estimates fromstatus samples'}
    results.append(result)
result={'schema':1,'scope':'Read-only actual source/heading budget comparison','status':'analyzed_no_runtime_changes','runs':results,
'future_parameter_change':'unverified until new prospective protocol/actual trials; no altered90s deadlines or old outcomes'}
out=Path(__file__).with_name('comparison.json');out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
for r in results:print(json.dumps({k:v for k,v in r.items()if k not in('turn_command_segments','expired_segments','source_hashes','phase_changes')}))
