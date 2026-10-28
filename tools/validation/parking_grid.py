"""Independent fixed-anchor sampling check, with strictly distinct source stamps."""
def causal_grid(poses,start_ns,origin):
    initial=dict(t=origin['stamp_ns']/1e9,p=origin['position'],q=origin['quaternion_xyzw'])
    rows=[];errors=[];previous=None
    for index in range(51):
        target=start_ns+index*100_000_000
        row=initial if index==0 else next((p for p in reversed(poses) if round(p['t']*1e9)<=target),None)
        stamp=None if row is None else round(row['t']*1e9)
        if stamp is None:errors.append('missing_causal_sample')
        elif previous is not None and stamp<=previous:errors.append('reused_or_reversed_pose_header')
        elif not 0<=target-stamp<=300_000_000:errors.append('stale_sensor')
        if stamp is not None:previous=stamp
        rows.append((target,row))
    return rows,sorted(set(errors))
