#!/usr/bin/env python3
"""Read immutable motion evidence; no ROS, control output or GT input.

Historic recorder stamps are floats; derived integer ns only organize replay,
not claims that native integer stamps were captured. Fresh proposed observers
must consume actual ROS sec/nanosec directly.
"""
import bisect,hashlib,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3]
RUN=ROOT/'simulation/test_results/20261001_balance_pair2_a_disabled'
OUT=Path(__file__).with_name('actual_velocity_response.json')

def rows(p):
    for l in p.open():yield json.loads(l)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def category(c):
    if c[0]>.015:return 'walk'
    if abs(c[2])>.001:return 'turn'
    return 'zero'
def distribution(v):
    a=np.asarray(v)
    if not len(a):return {'count':0}
    return dict(count=len(a),median=np.median(a,axis=0).tolist(),p05=np.quantile(a,.05,axis=0).tolist(),
                p95=np.quantile(a,.95,axis=0).tolist(),mean=np.mean(a,axis=0).tolist())

def main():
    commands={float(r['sim']):[r['value'][0],r['value'][1],r['value'][5]] for r in rows(RUN/'joint_stop_adapter.jsonl') if r['kind']=='actual_champ_command'}
    times=sorted(commands);cs=np.array([commands[t] for t in times]);modes=[category(c) for c in cs]
    changes=[times[i] for i in range(1,len(times)) if modes[i]!=modes[i-1]]
    evidence={}
    starts={}
    for r in rows(RUN/'feedback_navigation.jsonl'):
        n=r.get('navigation',{})
        if n.get('state')=='running':starts.setdefault(n['waypoint_index'],r['sim_time'])
        if 'slam' in r:evidence[r['slam']['stamp']]=r['slam']
    odom=sorted(evidence.values(),key=lambda r:r['stamp'])
    native=[];errors=[];sparse=0;duplicates=0;stats={m:[] for m in ('walk','turn','zero')}
    per_index={}
    for a,b in zip(odom,odom[1:]):
        t=float(b['stamp']);dt=(round(t*1e9)-round(a['stamp']*1e9))/1e9
        if not .08<=dt<=.12:sparse+=1;continue
        if not t>=starts.get(0,float('inf')) or t>=starts.get(8,float('inf')):continue
        R=Rotation.from_quat(b['quaternion']).as_matrix();Ra=Rotation.from_quat(a['quaternion']).as_matrix()
        v=R.T@(np.array(b['pose'])-np.array(a['pose']))/dt
        w=R.T@Rotation.from_matrix(R@Ra.T).as_rotvec()/dt
        yaw=math.atan2(R[1,0],R[0,0]);ya=math.atan2(Ra[1,0],Ra[0,0])
        heading_rate=math.atan2(math.sin(yaw-ya),math.cos(yaw-ya))/dt
        errors.append([np.linalg.norm(v-b['twist_body']),np.linalg.norm(w-b['angular_body'])])
        j=bisect.bisect_right(times,t)-1
        if j<0:continue
        mode=modes[j]
        # Remove actual command/stop transitions from steady-response samples.
        k=bisect.bisect_left(changes,t)
        adjacent=changes[max(0,k-1):k+1]
        if any(abs(x-t)<.5 for x in adjacent):continue
        cmd=cs[j]
        data=list(v)+list(w)+[heading_rate]+list(cmd)
        stats[mode].append(data)
        idx=max((i for i,s in starts.items() if i<8 and s<=t),default=None)
        per_index.setdefault(str(idx),{m:[] for m in stats})[mode].append(data)
    columns=['measured_body_vx','measured_body_vy','measured_body_vz','measured_body_wx','measured_body_wy','measured_body_wz','measured_world_heading_rate','actual_CHAMP_vx','actual_CHAMP_vy','actual_CHAMP_yaw']
    result=dict(scope=__doc__,run=str(RUN.relative_to(ROOT)),original_result_sha256=sha(RUN/'first_eight_result.json'),
                feedback_sha256=sha(RUN/'feedback_navigation.jsonl'),actual_command_sha256=sha(RUN/'joint_stop_adapter.jsonl'),
                source_sha256={s:sha(ROOT/s) for s in ('slam/geometry.py','slam/odom_adapter.py','navigation/controller.py')},
                ground_truth_consumed=False,unique_odom_messages=len(odom),intervals_not_80_to_120ms=sparse,
                stamp_source='Historic recorder float sec; derived round(stamp*1e9), not native stamp evidence.',
                reconstructed_vs_message_body_twist_error=distribution(errors),
                columns=columns,steady_actual_modes={m:distribution(a) for m,a in stats.items()},
                per_waypoint_steady_modes={i:{m:distribution(v) for m,v in a.items()} for i,a in per_index.items()},
                limitations=['Raw finite difference includes gait sway and LIO correction; nominal covariance is not an actual calibrated confidence.',
                            'Recorder may miss source messages; only adjacent nominal 100ms pairs used, no source loss claim.',
                            'Actual command mode excludes .5s transitions; steady statistics do not quantify transient overshoot.',
                            'Single completed A was strictly FAIL for active tilt; this observer audit does not change it.'])
    OUT.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('unique_odom_messages','reconstructed_vs_message_body_twist_error','columns','steady_actual_modes')},indent=2))
if __name__=='__main__':main()
