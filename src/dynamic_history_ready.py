"""Submission readiness only; original full dynamic verifier retains authority."""
import numpy as np

def dynamic_history_ready(run_name, request_id, now_ns, status_rows, pose_rows):
    """Check the arrived frozen history using original stop/recovery predicates.

    No request identity is invented for pose rows. The caller supplies its own
    current-run history, unchanged, to this check and the original verifier.
    This never constructs a receipt, moves a robot or repairs a source stamp.
    """
    if (not isinstance(request_id,str) or not request_id.startswith(str(run_name)+':')
            or type(now_ns)is not int or now_ns<=0):return False
    statuses=[r for r in status_rows if r['data'].get('request_id')==request_id]
    if any(type(r['received_sim_ns'])is not int or not 0<r['received_sim_ns']<=now_ns for r in statuses):return False
    held=[r for r in statuses if r['data'].get('obstacle_hold')is True]
    if not held:return False
    edge=held[0]['received_sim_ns'];last_held=held[-1]['received_sim_ns']
    resumed=[r for r in statuses if r['received_sim_ns']>last_held and not r['data'].get('obstacle_hold')
             and r['data'].get('aligned_obstacle_resumes',0)>0 and r['data'].get('obstacle_resumes',0)>0]
    if not resumed:return False
    resume=resumed[0]['received_sim_ns']
    for r in pose_rows:
        if (type(r['stamp_ns'])is not int or type(r['received_sim_ns'])is not int
                or not 0<r['stamp_ns']<=r['received_sim_ns']<=now_ns):return False
        for key in ('position','body_velocity','body_angular_velocity'):
            if not np.all(np.isfinite(np.asarray(r[key],dtype=float))):return False
        if r.get('body_gyro')is not None and not np.all(np.isfinite(np.asarray(r['body_gyro'],dtype=float))):return False
    stopped=[r for r in pose_rows if edge<=r['stamp_ns']<=last_held
             and np.linalg.norm(r['body_velocity'][:2])<=.03 and abs(r['body_angular_velocity'][2])<=.05
             and r.get('body_gyro')is not None and np.linalg.norm(r['body_gyro'])<=.05]
    good=[];stop_window=None
    for r in stopped:
        if good and r['stamp_ns']-good[-1]['stamp_ns']>200_000_000:good=[]
        good.append(r)
        if len(good)>=3 and good[-1]['stamp_ns']-good[0]['stamp_ns']>=300_000_000:stop_window=list(good);break
    moving=[r for r in pose_rows if r['stamp_ns']>=resume]
    if stop_window is None or len(moving)<3 or not any(np.linalg.norm(r['body_velocity'][:2])>.025 for r in moving):return False
    progress=max(np.linalg.norm(np.asarray(r['position'][:2])-moving[0]['position'][:2]) for r in moving)
    return bool(progress>=.08)
