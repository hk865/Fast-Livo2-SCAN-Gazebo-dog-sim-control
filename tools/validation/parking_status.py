"""Original fixed-window control coverage, separate from the 10Hz pose grid.

Only statuses actually received inside [T0,T1] are examined. The first/last
boundary gap and every internal gap must be <=300ms. A pre-T0 deceleration
status is not evidence about the fixed zero hold and must not be carried in.
"""

def check_window(rows,start_ns,end_ns,request_id):
    selected=[r for r in rows if start_ns<=round(r['t']*1e9)<=end_ns]
    errors=[];stamps=[round(r['t']*1e9)for r in selected]
    if not stamps:return dict(passed=False,errors=['missing_control_status'],samples=0)
    gaps=[stamps[0]-start_ns,end_ns-stamps[-1],*[b-a for a,b in zip(stamps,stamps[1:])]]
    if any(gap<0 or gap>300_000_000 for gap in gaps):errors.append('status_coverage_gap')
    generations=set()
    for row in selected:
        d=row['data'];parking=d.get('cascade_parking')or{};owner=parking.get('ordinary_parking')or{}
        command=d.get('command')
        if not isinstance(command,list)or len(command)!=3 or any(value!=0. for value in command):errors.append('nonzero_hold_command')
        if (d.get('request_id')!=request_id or d.get('state')!='succeeded'or parking.get('mode')!='ordinary_zero_hold'
                or parking.get('protection_active')or parking.get('failure_latched')or owner.get('failure')
                or d.get('ordinary_terminal_failure_latched')or d.get('tilt_hold')or d.get('obstacle_hold')
                or(d.get('execution_bridge_safety')or{}).get('state')!='ready'):errors.append('hold_interrupted')
        m=d.get('known_scene_localization')or{};s=d.get('known_scene_runtime_status')or{};epoch=d.get('known_scene_scan_epoch')or{}
        if (not m.get('control_allowed')or m.get('correction_stale')or m.get('correction_transition')
                or not s.get('initialized')or s.get('correction_transition')or not epoch.get('ready')
                or not m.get('generation')==s.get('generation')==epoch.get('generation')):errors.append('coordinate_source_unavailable')
        else:generations.add(m['generation'])
    if len(generations)>1:errors.append('coordinate_generation_changed')
    return dict(passed=not errors,errors=sorted(set(errors)),samples=len(selected),first_received_ns=stamps[0],last_received_ns=stamps[-1],max_gap_ns=max(gaps))
