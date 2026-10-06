"""Test-only actual held-command intervals; never use GT or change control.

4Hz status is an observation, not the exact command-time obstacle flag. End an
interval only on an immutable current-goal reference committed by SCAN, with a
real resume counter and actual accepted trajectory used for steering. A later
native guard on that trajectory also documents use during immediate re-block.
"""
import math


def stamp(value):
    if not isinstance(value,(list,tuple)) or len(value)!=2 or any(type(x) is not int for x in value):
        return None
    if value[0]<0 or not 0<=value[1]<1_000_000_000:
        return None
    return value[0]+value[1]*1e-9


def same_goal(a,b):
    try:
        return len(a)==len(b)==3 and all(math.isfinite(float(x)) and math.isfinite(float(y))
            and abs(float(x)-float(y))<=1e-6 for x,y in zip(a,b))
    except (TypeError,ValueError):
        return False


def audit_held_commands(native_events,statuses,metadata,commands,request_id,goals):
    active=[r for r in statuses if r['status'].get('request_id')==request_id]
    events=sorted((r['event'] for r in native_events if r['event'].get('edge_request_id')==request_id
        and r['event'].get('request_id')==request_id),key=lambda e:e['obstacle_edge_stamp'])
    end=max((r['sim'] for r in active),default=0.)
    reports=[]
    for ordinal,event in enumerate(events,1):
        start=event['obstacle_edge_stamp'];index=event.get('edge_waypoint_index')
        candidates=[]
        valid_event=(type(index) is int and 0<=index<len(goals)
            and event.get('waypoint_index')==index and event.get('guard_result',[False])[0] is True
            and event.get('zero_command')==[0.,0.,0.] and event.get('zero_command_stamp')==start)
        if valid_event:
            for row in active:
                s=row['status'];steer=s.get('steering') or {}
                ref=s.get('trajectory_reference_stamp');ref_time=stamp(ref)
                if (s.get('waypoint_index')!=index or s.get('obstacle_resumes')!=ordinal
                    or s.get('state')!='running' or s.get('obstacle_stops',0)<ordinal
                    or ref_time is None or ref_time<=start or ref_time>row['sim']
                    or ref==event.get('reference_stamp')
                    or s.get('accepted_trajectory_id') is None
                    or steer.get('trajectory_id')!=s['accepted_trajectory_id']
                    or steer.get('stamp',-1)<ref_time):
                    continue
                for mrow in metadata:
                    m=mrow['metadata'];trajectory=m.get('trajectory') or {}
                    if (m.get('reference_stamp')==ref and same_goal(m.get('body_goal'),goals[index])
                        and trajectory.get('traj_id')==s['accepted_trajectory_id']
                        and mrow.get('received_sim',math.inf)<=row['sim']):
                        # The immutable metadata was paired/accepted and is
                        # actually used by this production steering callback.
                        candidates.append(dict(reference_stamp=ref,reference_time=ref_time,
                            actual_status_sim=row['sim'],actual_steering_stamp=steer['stamp'],
                            accepted_trajectory_id=s['accepted_trajectory_id'],
                            actual_resume_counter=s['obstacle_resumes'],body_goal=m['body_goal']))
        proof=min(candidates,key=lambda c:c['reference_time']) if candidates else None
        until=proof['reference_time'] if proof else end
        samples=[c for c in commands if c.get('nav_request_id')==request_id
            and c.get('nav_index')==index and start<=c['sim']<until]
        bad=[c for c in samples if c['command']!=[0.,0.,0.]]
        sources={c['source'] for c in samples}
        reports.append(dict(event_id=event.get('event_id'),start=start,end=until,
            valid_native_zero_edge=valid_event,confirmed_same_goal_executed_reference=proof,
            exact_zero_samples=len(samples)-len(bad),total_samples=len(samples),nonzero_samples=bad,
            both_command_sources={'requested','safe'}<=sources,
            passed=bool(valid_event and samples and not bad and {'requested','safe'}<=sources)))
    expected_stops=max((r['status'].get('obstacle_stops',0) for r in active),default=0)
    return dict(scope=__doc__,expected_actual_stops=expected_stops,committed_native_edges=len(events),
        intervals=reports,passed=bool(events and len(events)==expected_stops and all(r['passed'] for r in reports)),
        caveat='Receipt timestamps and finite command samples are preserved. Reference execution proof does not infer arrival, physical progress or safety from metadata alone.')
