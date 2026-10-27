"""Read-only R85 dynamic evidence verifier. No ROS, simulator or model imports.

Replays the unchanged R83 two-corridor guard and stop/recovery predicates.
The R85 crossing geometry is a separate, fixed protocol, not an R83 scene PASS.
An ACK, independent entity observation, and guard stop are separate evidence;
their bounded association is not a counterfactual object-causation proof.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import r117_dynamic_speed_lead as speed_lead

BINDING_KEYS = ('run_id', 'reference_id', 'reference_sha256', 'request_id',
                'epoch', 'localization_generation', 'source_route_sha256')
PROTOCOL = 'r85_f1_f3_fixed_crossing/v1'
ACTOR_KEY='r85_actor_choreography'
ACTOR_SELECTION='independent_posttrigger_world_motion/v2'
ACTOR_PROTOCOL='r85_f1_f3_fixed_crossing_world_motion/v2'
ACTOR_MOTION_CONTRACT=dict(schema=ACTOR_SELECTION,
    trigger_fresh_identity_and_lead_window_unchanged=True,
    posttrigger_navigation_ready_and_mission_freshness_required=False,
    same_trigger_actor_fresh_observation_required=True,
    original_service_ACK_failure_cancel_and_owner_stop_required=True,
    fixed_geometry_speed_and_real_stop_resume_recovery_required=True)

def actor_choreography(profile):
    if ACTOR_KEY not in profile:return None
    value=profile[ACTOR_KEY]
    if type(value)is not str or value!=ACTOR_SELECTION:
        raise ValueError('explicit_actor_choreography_selection_differs')
    return dict(ACTOR_MOTION_CONTRACT)


PARK = np.array([17., 5.7, 1.8])
BLOCK = np.array([14.9, 5.7, 1.8])
SIZE = np.array([.5, .5, 1.2])
MAX_LOG_BYTES = 256*1024**2
MAX_LINE_BYTES = 2*1024**2
MAX_RECORDS = 500_000


class MissingEvidence(ValueError):
    pass


def _require(ok, message, *, missing=False):
    if not ok:
        raise (MissingEvidence if missing else ValueError)(message)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _hex(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def _integer(value, minimum=0):
    return type(value) is int and value >= minimum


def _vector(value, length=3):
    a = np.asarray(value, dtype=float)
    _require(a.shape == (length,) and np.isfinite(a).all(), 'Malformed/nonfinite vector')
    return a


def _inside(run, filename):
    p = Path(filename)
    if not p.is_absolute():
        p = run/p
    p = p.resolve()
    _require(p.is_relative_to(run), 'Evidence file escapes this run')
    _require(p.is_file(), 'Missing evidence file: '+str(p), missing=True)
    return p


def _read_json(path, expected=None, *, observe_raw=None):
    _require(path.is_file(), 'Missing JSON source: '+str(path), missing=True)
    raw = path.read_bytes()
    _require(len(raw) <= 16*1024**2, 'JSON byte budget exceeded')
    binding = dict(path=str(path.resolve()), sha256=_sha(raw), bytes=len(raw))
    if expected is not None:
        _require(_hex(expected) and binding['sha256'] == expected, 'JSON source SHA mismatch: '+str(path))
    doc = json.loads(raw)
    _canonical(doc)
    if observe_raw is not None:
        observe_raw(path, raw, doc, binding)
    return doc, binding


def _lines(run, filename):
    path = _inside(run, filename)
    _require(path.stat().st_size <= MAX_LOG_BYTES, 'Log byte budget exceeded')
    rows, offset, digest = [], 0, hashlib.sha256()
    with path.open('rb') as stream:
        for raw in stream:
            _require(len(raw) <= MAX_LINE_BYTES, 'Source line byte budget exceeded')
            if not raw.endswith(b'\n'):
                break  # An in-progress partial line is not an observation.
            d = json.loads(raw)
            _canonical(d)
            rows.append(dict(data=d, source_file=str(path), source_offset=offset,
                source_length=len(raw), source_line_sha256=_sha(raw)))
            offset += len(raw); digest.update(raw)
            _require(len(rows) <= MAX_RECORDS, 'Source record budget exceeded')
    return rows, dict(path=str(path), complete_prefix_bytes=offset, complete_prefix_sha256=digest.hexdigest())


def _verify_line(record):
    with Path(record['source_file']).open('rb') as stream:
        stream.seek(record['source_offset']); raw = stream.read(record['source_length'])
    _require(_sha(raw) == record['source_line_sha256'], 'Serialized source line changed')
    parsed = json.loads(raw)
    expected = record.get('_journal_raw', record['data'])
    _require(_canonical(parsed) == _canonical(expected), 'Source line/data projection mismatch')


def _modules():
    import control_core, guard_audit, dynamic_stop_event, known_scene_hold_guard
    modules = dict(core=control_core, audit=guard_audit, stop=dynamic_stop_event,
                   payload=known_scene_hold_guard)
    return modules, {key: dict(path=module.__file__) for key, module in modules.items()}


def _same_binding(value, expected):
    _require(isinstance(value, dict), 'Missing causal binding')
    for key in BINDING_KEYS:
        _require(type(value.get(key)) is type(expected[key]) and value[key] == expected[key],
                 'Foreign/conflicting causal binding: '+key)


def _projection(points, point):
    delta = np.diff(points, axis=0); lengths = np.linalg.norm(delta, axis=1)
    _require(np.isfinite(lengths).all() and np.any(lengths > 0), 'Degenerate source route')
    t = np.clip(np.sum((point-points[:-1])*delta, axis=1)/np.maximum(lengths**2, 1e-24), 0., 1.)
    residual = np.linalg.norm(point-(points[:-1]+t[:, None]*delta), axis=1)
    index = int(np.argmin(residual))
    arc = float(np.r_[0., np.cumsum(lengths)][index]+t[index]*lengths[index])
    return arc, float(residual[index]), index


def _target_at(elapsed,scene=None):
    from dynamic_scene import target_at
    position,phase=target_at(elapsed,scene)
    return np.asarray(position,float),phase


def _strict_clocks(rows, key, gap, minimum_count, minimum_span, label):
    _require(len(rows) >= minimum_count, label+' record count incomplete', missing=True)
    clocks = [r['data'][key] for r in rows]
    _require(all(_integer(v) for v in clocks), label+' clock type invalid')
    _require(all(0 < b-a <= gap for a, b in zip(clocks, clocks[1:])), label+' continuity invalid')
    _require(clocks[-1]-clocks[0] >= minimum_span, label+' duration incomplete', missing=True)


def select_clear_release(guards, event, aligned_resume_ns):
    """Bind clear evidence to this stop's native release, before aligned drive."""
    _require(_integer(aligned_resume_ns), 'Actual aligned recovery clock invalid')
    active = [r for r in guards if event['clock_ns'] < r['data']['compute_ros_clock_ns'] <= aligned_resume_ns]
    _require(active, 'Actual stop-to-release guards missing', missing=True)
    releases = []
    stop_rows = [r for r in guards if r['data'].get('sequence') == event['sequence']]
    _require(len(stop_rows) == 1, 'Actual clear release stop source missing or duplicated')
    previous = stop_rows[0]['data']
    for row in active:
        g = row['data']
        _require(_integer(g.get('sequence'),1) and _integer(g.get('compute_ros_clock_ns'))
            and g.get('request_id') == event['request_id']
            and type(g.get('waypoint_index')) is int and g['waypoint_index'] == event['waypoint_index']
            and type(g.get('obstacle_stops_before')) is int and g['obstacle_stops_before'] == event['stop_count']
            and type(g.get('obstacle_hold_before')) is bool and type(g.get('obstacle_hold_after')) is bool
            and type(g.get('obstacle_resumes_after')) is int and g['obstacle_resumes_after'] >= 0,
            'Actual clear release mixes stop events or malformed counters')
        if g['obstacle_hold_before'] is True and g['obstacle_hold_after'] is False:
            _require(previous is not None and type(previous.get('obstacle_resumes_after')) is int
                and g['obstacle_resumes_after'] == previous['obstacle_resumes_after'] + 1
                and g['sequence'] == previous['sequence'] + 1
                and g.get('previous_actual_guard_clock_ns') == previous['compute_ros_clock_ns']
                and g['union_result']['blocked'] is False,
                'Actual clear release counter/source transition differs')
            releases.append(row)
        else:
            _require(type(previous.get('obstacle_resumes_after')) is int
                and g['obstacle_resumes_after'] == previous['obstacle_resumes_after'],
                'Actual clear release counter changed without release')
        if 'obstacle_stops_after' in g:
            _require(type(g['obstacle_stops_after']) is int and g['obstacle_stops_after'] == event['stop_count'],
                'Actual clear release crosses another counted stop')
        if releases:
            _require(g['union_result']['blocked'] is False and g['obstacle_hold_after'] is False,
                'Actual clear release followed by another blockage/hold')
        previous = g
    _require(len(releases) == 1, 'Actual unique native clear release missing or duplicated', missing=not releases)
    release = releases[0]
    _require(release['data']['compute_ros_clock_ns'] > event['clock_ns'], 'Actual clear release precedes stop')
    return release


def select_clear_window(guards, event, aligned_resume_ns):
    release=select_clear_release(guards,event,aligned_resume_ns)
    g=release['data'];start=g.get('clear_start_before_s')
    _require(type(start) in (int,float) and math.isfinite(start) and start>=0
        and type(g.get('clear_start_after_s')) in (int,float) and g['clear_start_after_s']==start,
        'Actual native clear timer source invalid')
    start_ns=round(start*1e9);end=g['compute_ros_clock_ns']
    _require(event['clock_ns']<start_ns<=end,'Actual native clear timer outside stop/release')
    observed=[r for r in guards if event['clock_ns']<r['data']['compute_ros_clock_ns']<=end
        and type(r['data'].get('clear_start_after_s')) in (int,float)
        and r['data']['clear_start_after_s']==start]
    _require(observed,'Actual native clear timer observations missing',missing=True)
    first=observed[0]['data']
    # A cached-clear control callback may initialize the timer between native
    # evaluations. The first same-timer guard is evidence, not an invented origin.
    _require(_integer(first.get('after_control_ros_clock_ns'))
        and start_ns<=first['after_control_ros_clock_ns'],
        'Actual native clear timer observation clock differs')
    timer=[r for r in guards if first['sequence']<=r['data']['sequence']<=g['sequence']]
    _require(all(type(r['data'].get('clear_start_after_s')) in (int,float)
        and r['data']['clear_start_after_s']==start and r['data']['union_result']['blocked'] is False
        and r['data'].get('obstacle_hold_before') is True for r in timer),
        'Actual native clear timer reset or blockage')
    _require(all(type(r['data'].get('clear_start_before_s')) in (int,float)
        and math.isfinite(r['data']['clear_start_before_s'])
        and r['data']['clear_start_before_s']==start for r in timer[1:]),
        'Actual native clear timer interrupted before evaluation')
    _strict_clocks(timer,'compute_ros_clock_ns',300_000_000,1,0,'Actual native clear timer')
    _require(all(b['data']['sequence']==a['data']['sequence']+1
        and b['data'].get('previous_actual_guard_clock_ns')==a['data']['compute_ros_clock_ns']
        for a,b in zip(timer,timer[1:])), 'Actual native clear timer source records have a gap')
    lower=end-1_350_000_000
    return release,[r for r in timer if lower<=r['data']['compute_ros_clock_ns']<=end]


def _native_stamp(header):
    stamp = header.get('stamp', {})
    values = []
    for value in (stamp.get('sec', 0), stamp.get('nsec', stamp.get('nanosec', 0))):
        if type(value) is int:
            values.append(value)
        elif isinstance(value, str) and value.isascii() and value.isdigit():
            values.append(int(value))
        else:
            raise ValueError('Malformed native observer stamp')
    sec, nsec = values
    _require(sec >= 0 and 0 <= nsec < 1_000_000_000, 'Malformed native observer stamp')
    return sec*1_000_000_000+nsec


def _pretrigger_history(evidence, contract, binding, points, refdoc, registration, scene=None):
    """Read distinct monitoring sources; never migrate an old-scope event."""
    from dynamic_scene import configuration as scene_configuration
    PARK=np.asarray((scene_configuration()if scene is None else scene)['park_world_xyz'],float)
    rows = evidence.get('pretrigger_records')
    _require(isinstance(rows, list) and rows, 'Actual pretrigger monitoring history missing', missing=True)
    _require(contract.get('scope_locked_at_trigger') is True and _integer(contract.get('trigger_sim_ns')),
             'Dynamic scope was not frozen at actual trigger')
    trigger_ns = contract['trigger_sim_ns']
    previous = None; active = None; selected = []; all_rows = {}
    for row in rows:
        raw = row.get('_journal_raw', row['data']); _verify_line(row)
        _require(raw.get('schema') == 'r85_dynamic_pretrigger_record/v1', 'Wrong pretrigger monitoring schema')
        seq, ns, wall = raw['sequence'], raw['sim_ns'], raw['monotonic_wall']
        _require(_integer(seq, 1) and _integer(ns) and ns <= trigger_ns
            and type(wall) in (int, float) and math.isfinite(wall), 'Pretrigger monitoring clock invalid')
        if previous is None:_require(seq == 1, 'Pretrigger monitoring prefix missing')
        else:_require(seq == previous[0]+1 and ns >= previous[1] and wall >= previous[2], 'Pretrigger source continuity differs')
        previous = seq, ns, wall
        b = raw['binding']; kind = raw['kind']; data = raw['data']
        _require(isinstance(b, dict), 'Pretrigger binding missing')
        for key in ('run_id', 'request_id', 'source_route_sha256'):
            _require(type(b.get(key)) is type(binding[key]) and b[key] == binding[key], 'Pretrigger immutable task/typed binding changed: '+key)
        _require(_integer(b.get('epoch'), 1) and _integer(b.get('localization_generation'), 1)
            and _hex(b.get('reference_sha256')) and isinstance(b.get('reference_id'), str) and b['reference_id'],
            'Pretrigger actual scope types invalid')
        if kind == 'reference_binding':
            if active is not None:
                _require(b['epoch'] > active['epoch'] and b['localization_generation'] >= active['localization_generation'],
                         'Pretrigger planner epoch did not advance')
            _require(_sha(data['reference_raw_utf8'].encode()) == b['reference_sha256']
                and _canonical(json.loads(data['reference_raw_utf8'])) == _canonical(data['document']), 'Pretrigger reference snapshot bytes differ')
            doc = data['document']; reg = data['registration_document']
            for key in ('run_id', 'reference_id', 'epoch', 'localization_generation', 'source_route_sha256'):
                _require(type(doc.get(key)) is type(b[key]) and doc[key] == b[key], 'Pretrigger reference identity differs: '+key)
            _require(doc.get('schema') == 'r85_pct_scan_global_reference/v1'
                and doc.get('frame_id') == 'camera_init' and doc.get('reference_scope') == 'full_route'
                and doc.get('navigation_ground_truth_used') is False
                and doc.get('T_navigation_world') == refdoc['T_navigation_world']
                and np.array_equal(np.asarray(doc['body_points'], float), points)
                and Path(doc['source_route_file']).resolve() == Path(contract['typed_source_file']).resolve(),
                'Pretrigger full typed/reference geometry changed')
            _require(_sha(data['registration_raw_utf8'].encode()) == data['registration_sha256']
                and _canonical(json.loads(data['registration_raw_utf8'])) == _canonical(reg), 'Pretrigger registration snapshot bytes differ')
            _require(reg.get('schema') == 'r85_reference_source_registration/v1'
                and reg.get('run_id') == b['run_id'] and reg.get('reference_id') == b['reference_id']
                and reg.get('reference_file_sha256') == b['reference_sha256']
                and reg.get('source_route_sha256') == b['source_route_sha256']
                and type(reg.get('localization_generation')) is int and reg['localization_generation'] == b['localization_generation']
                and reg.get('navigation_ground_truth_used') is False
                and reg.get('producer') == 'root_authorized_PCT_route_owner'
                and all(reg.get(k) == registration[k] for k in ('run_id',)),
                'Pretrigger source registration semantics differ')
            active = dict(b)
        else:
            _require(active is not None, 'Pretrigger callback preceded actual source selection')
            _same_binding(b, active)
        if kind in ('service_request', 'service_ack'):
            _require(data.get('phase') == 'parked' and data.get('entity') == 'moving_obstacle'
                and data.get('service') == '/world/teacher_demo/set_pose'
                and np.array_equal(_vector(data['position_world_xyz']), PARK), 'Actor moved before trigger')
        _require(kind not in ('trigger','reference_lock'), 'Locked event appeared in pretrigger monitoring history')
        projected = dict(row, data=data, _journal_raw=raw, received_sim_ns=ns, monitoring_sequence=seq)
        all_rows[seq] = projected
        if b == binding:selected.append(projected)
    _same_binding(active, binding)
    arms = contract.get('arming_pretrigger_sequences')
    expected = dict(reference_binding='reference_binding', actual_pose='actual_pose', mission_state='mission_state',
                    entity_observation='entity_observation', park_service_ack='service_ack')
    _require(isinstance(arms, dict) and set(arms) == set(expected), 'Trigger arming source references missing')
    for key, kind in expected.items():
        seq = arms[key]
        _require(_integer(seq, 1) and seq in all_rows, 'Actual arming callback missing: '+key, missing=True)
        r = all_rows[seq]; _same_binding(r['_journal_raw']['binding'], binding)
        _require(r['_journal_raw']['kind'] == kind, 'Arming callback kind differs: '+key)
    start = all_rows[arms['reference_binding']]['received_sim_ns']
    for key in ('actual_pose','entity_observation'):
        r = all_rows[arms[key]]
        _require(start <= r['data']['stamp_ns'] <= trigger_ns and trigger_ns-r['data']['stamp_ns'] <= 300_000_000,
                 'Old/stale actual pose reused after pretrigger scope selection: '+key)
    ack = all_rows[arms['park_service_ack']]['data']
    _require(ack.get('success') is True and start <= ack['request_sim_ns'] <= ack['response_sim_ns'] <= trigger_ns
        and trigger_ns-ack['response_sim_ns'] <= 300_000_000, 'Old/stale park ACK reused after pretrigger scope selection')
    return selected



def _guard_route(g, points, refdoc, ref_binding, route_sha, run):
    if 'route' in g:
        return np.asarray(g['route'],float)  # Original history compatibility.
    reference=g.get('route_reference')
    _require(isinstance(reference,dict) and reference.get('schema')=='program_global_guard_route/v1',
             'Actual native guard route reference missing',missing=True)
    _require(_inside(run,reference.get('reference_file','')).resolve()==Path(ref_binding['path']).resolve()
        and reference.get('reference_file_sha256')==ref_binding['sha256']
        and reference.get('reference_id')==refdoc['reference_id']
        and reference.get('source_route_sha256')==refdoc['source_route_sha256']
        and reference.get('frame_id')=='camera_init' and reference.get('point_count')==len(points)
        and type(reference.get('route_id'))is int and reference['route_id']>0
        and type(reference.get('route_version'))is int and reference['route_version']>0
        and reference.get('route_float64_sha256')==g.get('route_float64_sha256')==route_sha,
        'Actual native guard versioned route binding differs')
    return points  # Already validated full typed/reference points, no new file read.


def _coordinate_segments(evidence,contract,profile,refdoc,run,registration):
    from r85_dynamic_obstacle_runtime import COORDINATE_WINDOW,COORDINATE_SELECTION,validate_coordinate_handoff_edge
    _require(contract.get('event_lifetime_contract')==COORDINATE_WINDOW
        and contract.get('navigation_segment_policy')==COORDINATE_SELECTION,'Coordinate handoff contract differs')
    rows=[r for r in evidence['records'] if r.get('_journal_raw',r['data']).get('kind')=='navigation_segment_boundary']
    _require(len(rows)<=2,'Coordinate handoff bound exceeded')
    previous=contract['binding'];previous_doc=refdoc;cut=-1;segments=[]
    for index,row in enumerate(rows,1):
        _verify_line(row);raw=row.get('_journal_raw',row['data']);edge=raw['data']
        _same_binding(raw['binding'],contract['binding']);_same_binding(edge['old_binding'],previous)
        _require(edge['switch_index']==index and edge['old_reference']==previous_doc
            and edge['cut_sim_ns']>cut and 0<=raw['sim_ns']-edge['cut_sim_ns']<=300_000_000,
            'Coordinate edge order/old reference differs')
        validate_coordinate_handoff_edge(edge)
        reference=edge['new_reference'];b=edge['new_binding']
        doc,rb=_read_json(_inside(run,reference['reference_file']),b['reference_sha256'])
        reg,rg=_read_json(_inside(run,reference['registration_file']),reference['registration_sha256'])
        _require(doc==reference['document'] and reg==reference['registration_document']
            and _sha(reference['reference_raw_utf8'].encode())==rb['sha256']
            and _sha(reference['registration_raw_utf8'].encode())==rg['sha256'],'Coordinate version bytes differ')
        for key in ('run_id','reference_id','epoch','localization_generation','source_route_sha256'):
            _require(type(doc.get(key))is type(b[key]) and doc[key]==b[key],'Coordinate version identity differs: '+key)
        _require(reg.get('reference_file_sha256')==b['reference_sha256'] and reg.get('reference_id')==b['reference_id']
            and reg.get('localization_generation')==b['localization_generation'] and reg.get('run_id')==b['run_id']
            and reg.get('source_route_sha256')==b['source_route_sha256']
            and reg.get('producer')=='root_authorized_PCT_route_owner','Coordinate registration differs')
        _require(all(type(reg.get(k))is type(registration.get(k)) and reg.get(k)==registration.get(k)
            for k in ('navigation_ground_truth_used',)),
            'Coordinate owner/task differs')
        segments.append(dict(edge=edge,sequence=raw['sequence'],cut=edge['cut_sim_ns'],binding=b,reference=doc,reference_source=rb))
        previous=b;previous_doc=doc;cut=edge['cut_sim_ns']
    return dict(coordinate_chain=segments,binding=previous,anchor=contract['binding'],trigger_cut=contract['trigger_sim_ns'])


def _coordinate_at(segment,*,sequence=None,clock=None):
    selected=None
    for item in segment['coordinate_chain']:
        if (sequence is not None and sequence>item['sequence']) or (clock is not None and clock>=item['cut']):selected=item
    return selected


def _coordinate_body_sha(value):
    return type(value)is str and len(value)==64 and all(c in '0123456789abcdef'for c in value)


def _coordinate_pose_sources(run,evidence,segment):
    # One bounded streaming pass over the existing producer's own exact-body
    # witnesses. An Odometry callback itself carries no localization generation.
    wanted={(r.get('_journal_raw',r['data'])['data'].get('stamp_ns'),
        r.get('_journal_raw',r['data'])['data'].get('corrected_body_message_sha256'))
        for r in evidence['records'] if r.get('_journal_raw',r['data']).get('kind')=='actual_pose'
        and _integer(r.get('_journal_raw',r['data'])['data'].get('stamp_ns'))
        and _coordinate_body_sha(r.get('_journal_raw',r['data'])['data'].get('corrected_body_message_sha256'))}
    found={};path=_inside(run,'known_scene_bundles.jsonl');digest=hashlib.sha256()
    with path.open('rb') as stream:
        count=0;prefix_bytes=0
        while True:
            raw=stream.readline(MAX_LINE_BYTES+1)
            if not raw:break
            count+=1;_require(count<=MAX_RECORDS and len(raw)<=MAX_LINE_BYTES and raw.endswith(b'\n'),
                'Coordinate producer bundle witness bound/incomplete')
            digest.update(raw);prefix_bytes+=len(raw)
            row=json.loads(raw);key=(row.get('source_ns'),row.get('corrected_body_message_sha256'))
            if key not in wanted:continue
            _require(row.get('run_id')==run.name and row.get('schema')=='known_scene_navigation_bundle/v1'
                and type(row.get('generation'))is int and row['generation']>0,
                'Coordinate pose witness producer identity malformed')
            _require(key not in found,'Coordinate pose witness duplicate/ambiguous')
            found[key]=row['generation']
    segment['pose_sources']=found
    return dict(path=str(path),source_prefix_sha256=digest.hexdigest(),source_prefix_bytes=prefix_bytes,matched=len(found),requested=len(wanted))


def _coordinate_recovery_rows(poses,segment,resume):
    latest=segment['coordinate_chain'][-1] if segment['coordinate_chain'] else None
    binding=segment['anchor'] if latest is None else latest['binding']
    cut=segment.get('trigger_cut',0) if latest is None else latest['cut']
    return [row for row in poses if _integer(row['data'].get('stamp_ns'))
        and row['data']['stamp_ns']>=max(resume,cut)
        and segment.get('pose_sources',{}).get((row['data']['stamp_ns'],row['data'].get('corrected_body_message_sha256')))==binding['localization_generation']]


def _coordinate_guard_reference(g,segment,refdoc,refsource):
    selected=_coordinate_at(segment,clock=g.get('compute_ros_clock_ns',-1))
    if selected is None:return refdoc,refsource
    if 'route' in g:
        route=np.asarray(g['route'],float)
        expected=np.asarray(selected['reference']['body_points'],float)
        _require(route.shape==expected.shape and np.isfinite(route).all()
            and np.array_equal(route,expected),
            'Coordinate guard actual route geometry differs from current reference')
    else:
        stamp=g.get('reference_stamp');reference=g.get('route_reference')or{}
        _require(type(stamp)in(list,tuple) and len(stamp)==2 and all(type(n)is int for n in stamp)
            and stamp[0]>=0 and 0<=stamp[1]<1_000_000_000
            and stamp[0]*1_000_000_000+stamp[1]==reference.get('route_id'),
            'Coordinate guard reference stamp differs from actual route id')
    cut=selected['cut']
    _require(all(_integer(g.get(k)) and cut<=g[k]<=g['compute_ros_clock_ns']
        for k in ('control_pose_stamp_ns','cloud_header_stamp_ns','cloud_filtering_body_stamp_ns'))
        and type(g.get('compute_monotonic_wall'))in(int,float)
        and g['compute_monotonic_wall']>selected['edge']['boundary_wall'],
        'Coordinate guard uses old/unproven source after boundary')
    return selected['reference'],selected['reference_source']


def _navigation_segment(evidence,contract,profile,refdoc,run,registration):
    from r85_dynamic_obstacle_runtime import navigation_segments_enabled,SEGMENT_WINDOW,SEGMENT_SELECTION,validate_navigation_segment_edge,coordinate_handoffs_enabled
    if coordinate_handoffs_enabled(profile):return _coordinate_segments(evidence,contract,profile,refdoc,run,registration)
    if not navigation_segments_enabled(profile):return None
    _require(contract.get('event_lifetime_contract')==SEGMENT_WINDOW and contract.get('navigation_segment_policy')==SEGMENT_SELECTION,
        'Explicit world-event/navigation-segment contract missing')
    rows=[r for r in evidence['records'] if r.get('_journal_raw',r['data']).get('kind')=='navigation_segment_boundary']
    _require(len(rows)<=1,'More than one pre-stop navigation segment edge')
    if not rows:return None
    row=rows[0];_verify_line(row);raw=row.get('_journal_raw',row['data']);edge=raw['data']
    _same_binding(raw['binding'],contract['binding']);_same_binding(edge['old_binding'],contract['binding'])
    _require(edge['old_reference']==refdoc and 0<=raw['sim_ns']-edge['cut_sim_ns']<=300_000_000 and raw['sequence']>1,
        'Navigation edge does not preserve original event/version')
    validate_navigation_segment_edge(edge)
    reference=edge['new_reference'];b=edge['new_binding']
    doc,rb=_read_json(_inside(run,reference['reference_file']),b['reference_sha256'])
    reg,rg=_read_json(_inside(run,reference['registration_file']),reference['registration_sha256'])
    _require(doc==reference['document'] and reg==reference['registration_document']
        and _sha(reference['reference_raw_utf8'].encode())==rb['sha256']
        and _sha(reference['registration_raw_utf8'].encode())==rg['sha256'], 'Navigation edge real version bytes differ')
    for key in ('run_id','reference_id','epoch','localization_generation','source_route_sha256'):
        _require(type(doc.get(key))is type(b[key]) and doc[key]==b[key],'New navigation reference identity differs: '+key)
    _require(reg.get('reference_file_sha256')==b['reference_sha256'] and reg.get('reference_id')==b['reference_id']
        and reg.get('localization_generation')==b['localization_generation']
        and reg.get('run_id')==b['run_id'] and reg.get('source_route_sha256')==b['source_route_sha256']
        and reg.get('producer')=='root_authorized_PCT_route_owner','New navigation source registration differs')
    _require(all(type(reg.get(k))is type(registration.get(k)) and reg.get(k)==registration.get(k)
        for k in ('navigation_ground_truth_used',)),
        'Navigation owner/task changed across world event')
    # Source-owner/task bytes remain independently bound, not an alias of old IDs.
    # Inspect the full source set before selection can drop old-source guards.
    # A late/same-ROS-tick old guard is still an old stop fact, even if no
    # status survived. Only an actual new version plus new sources may stop.
    points=np.asarray(doc['body_points'],float);route_sha=_sha(np.asarray(points,dtype='<f8').tobytes())
    for row in evidence['guards']:
        g=row['data']
        proof=((g.get('union_result')or{}).get('blocked')is True
            or g.get('obstacle_hold_before')is True or g.get('obstacle_hold_after')is True
            or any(type(g.get(k))is int and g[k]>0 for k in ('obstacle_stops_before','obstacle_stops_after')))
        if not proof:continue
        clock=g.get('compute_ros_clock_ns');_require(_integer(clock),'Blocked/stop guard has no exact source clock')
        if clock<contract['trigger_sim_ns']:continue
        _verify_line(row)
        _require(g.get('request_id')==b['request_id'] and clock>=edge['cut_sim_ns']
            and all(_integer(g.get(k)) and edge['cut_sim_ns']<=g[k]<=clock
                for k in ('control_pose_stamp_ns','cloud_header_stamp_ns','cloud_filtering_body_stamp_ns'))
            and type(g.get('compute_monotonic_wall'))in(int,float) and math.isfinite(g['compute_monotonic_wall'])
            and g['compute_monotonic_wall']>edge['boundary_wall'],
            'Blocked/stop guard is old or unproven navigation at handoff')
        _require('route'not in g,'Blocked/stop guard lacks actual new route-version identity')
        _guard_route(g,points,doc,rb,route_sha,run)
        stamp=g.get('reference_stamp');reference=g['route_reference']
        _require(type(stamp)in(list,tuple) and len(stamp)==2 and all(type(n)is int for n in stamp)
            and stamp[0]>=0 and 0<=stamp[1]<1_000_000_000
            and stamp[0]*1_000_000_000+stamp[1]==reference['route_id'],
            'Blocked/stop guard local reference differs from actual new route version')
    return dict(edge=edge,sequence=raw['sequence'],cut=edge['cut_sim_ns'],binding=b,reference=doc,reference_source=rb)


def _new_navigation_row(raw,segment):
    if segment is None:return True
    if 'coordinate_chain'in segment:
        from r85_dynamic_obstacle_runtime import coordinate_navigation_source,coordinate_status_matches
        d=raw['data'];kind=raw['kind'];source=coordinate_navigation_source(kind,d)
        _require(raw.get('navigation_source_identity')==source,'Coordinate immutable payload identity differs')
        selected=_coordinate_at(segment,sequence=raw['sequence'])
        binding=segment['anchor'] if selected is None else selected['binding']
        cut=segment.get('trigger_cut',0) if selected is None else selected['cut']
        if kind=='actual_pose':
            stamp=d.get('stamp_ns');fingerprint=d.get('corrected_body_message_sha256')
            if not _integer(stamp) or not _coordinate_body_sha(fingerprint):return False
            key=(stamp,fingerprint)
            # Select by the original body source stamp and producer generation,
            # not by callback arrival relative to a journal boundary.
            pose_segment=_coordinate_at(segment,clock=stamp) if _integer(stamp) else None
            expected=segment['anchor'] if pose_segment is None else pose_segment['binding']
            if pose_segment is not None and raw['sequence']<=pose_segment['sequence']:return False
            return (_integer(stamp) and segment.get('pose_sources',{}).get(key)==expected['localization_generation'])
        if kind=='status':
            if not coordinate_status_matches(d,binding):return False
            stamp=(d.get('known_scene_localization')or{}).get('source_ns')
            return _integer(stamp) and stamp>=cut
        if kind=='mission_state':
            ref=d.get('r85_reference')or{}
            if not (type(ref.get('epoch'))is int and ref['epoch']==binding['epoch']
                    and type(ref.get('localization_generation'))is int and ref['localization_generation']==binding['localization_generation']
                    and ref.get('reference_id')==binding['reference_id']):return False
            return _integer(d.get('source_actual_pose_stamp_ns')) and d['source_actual_pose_stamp_ns']>=cut
        return False
    d=raw['data'];kind=raw['kind'];cut=segment['cut']
    if raw['sequence']<segment['sequence']:
        if kind=='status':
            _require(d.get('obstacle_hold')is not True and d.get('dynamic_stop_event')is None
                and not any(type(d.get(k))is int and d[k]>0 for k in ('obstacle_stops','obstacle_resumes','aligned_obstacle_resumes')),
                'Old navigation segment already contains dynamic proof')
        return False
    if kind=='actual_pose':return _integer(d.get('stamp_ns')) and d['stamp_ns']>=cut
    if kind=='mission_state':
        ref=d.get('r85_reference')or{};b=segment['binding']
        _require(ref.get('reference_id')==b['reference_id'] and ref.get('epoch')==b['epoch']
            and ref.get('localization_generation')==b['localization_generation'], 'New navigation mission scope differs')
        return _integer(d.get('source_actual_pose_stamp_ns')) and d['source_actual_pose_stamp_ns']>=cut
    if kind=='status':
        e=d.get('known_scene_scan_epoch')or{};b=segment['binding']
        _require(e.get('planner_epoch')==b['epoch'] and e.get('localization_generation')==b['localization_generation'],
            'New navigation status scope differs')
        source=(d.get('known_scene_localization')or{}).get('source_ns')
        if not _integer(source) or source<cut:
            _require(d.get('obstacle_hold')is not True and d.get('dynamic_stop_event')is None,'Stop used a pre-handoff source')
            return False
    return True


def _verify_segment_control_ACKs(run,guards,cut,request_id):
    from compact_archives import open_history
    expected={g['data'].get('pid_sequence'):g['data'] for g in guards}
    _require(None not in expected,'Selected new navigation guards lack actual PID links')
    found={};integrity={}
    for row in open_history(run/'navigation_pid_history.jsonl',integrity=integrity):
        seq=row.get('sequence')
        if seq not in expected:continue
        g=expected[seq];ack=row.get('ack')or{}
        _require(row.get('request_id')==request_id and row.get('compute_ros_clock_ns')==g.get('pid_compute_ros_clock_ns')
            and row.get('source_pose_stamp_ns',-1)>=cut and row.get('paired_imu_stamp_ns',-1)>=cut
            and type(ack.get('stamp_ns'))is int and ack['stamp_ns']>=cut,
            'New navigation guard/PID/actual control ACK crossed old segment')
        found[seq]=True
    _require(set(found)==set(expected) and integrity.get('archive_integrity_complete')is True,
        'Actual new navigation PID/ACK receipt missing or incomplete',missing=True)



def _world_stop_windows(held,guards,window,request_id,stop_module):
    """Verify every real stop/release; only the completed final stop gets recovery credit."""
    groups=[];seen=set()
    for row in held:
        event=row['data'].get('dynamic_stop_event')
        _require(isinstance(event,dict),'Actual held status lacks a stop event')
        key=_canonical(event)
        if not groups or groups[-1]['key']!=key:
            _require(key not in seen,'Actual stop event returned after a later stop')
            seen.add(key);groups.append(dict(key=key,event=event,held=[]))
        groups[-1]['held'].append(row)
    _require(groups,'Actual obstacle stop missing',missing=True)
    if len(groups)>1:
        _require(isinstance(window,dict) and window.get('stop_event')==groups[-1]['event'],
            'Completed world event does not identify its actual final stop')
    for index,group in enumerate(groups):
        event=group['event']
        _require(event.get('request_id')==request_id and event.get('stop_count')==index+1,
            'Actual stop counters are foreign, missing or out of order')
        pre,stop,event=stop_module.select_event_preroll(request_id,group['held'],guards,_verify_line)
        _strict_clocks(pre,'compute_ros_clock_ns',300_000_000,3,300_000_000,'Native pre-roll')
        end=groups[index+1]['event']['clock_ns'] if index+1<len(groups) else float('inf')
        release=[r for r in guards if event['clock_ns']<r['data']['compute_ros_clock_ns']<end
            and r['data'].get('obstacle_hold_before') is True and r['data'].get('obstacle_hold_after') is False
            and r['data'].get('obstacle_stops_after')==event['stop_count']]
        _require(len(release)==1,'Actual stop has no unique native clear release',missing=not release)
        release,clear=select_clear_window(guards,event,release[0]['data']['compute_ros_clock_ns'])
        _strict_clocks(clear,'compute_ros_clock_ns',300_000_000,10,1_000_000_000,'Continuous clear guard')
        _require(all(r['data']['union_result']['blocked'] is False for r in clear),
            'An actual stop clear interval contains a blocked guard')
        group.update(preroll=pre,stop_guard=stop,event=event,release=release,cleared=clear)
    return groups

def _measured_stop(rows,first,last):
    good=[]
    for row in rows:
        p=row['data']
        if not first<=p['stamp_ns']<=last:continue
        imu_valid=_integer(p.get('imu_stamp_ns')) and abs(p['stamp_ns']-p['imu_stamp_ns'])<=300_000_000 and p.get('body_gyro') is not None
        stopped=imu_valid and np.linalg.norm(_vector(p['body_velocity'])[:2])<=.03 and abs(_vector(p['body_angular_velocity'])[2])<=.05 and np.linalg.norm(_vector(p['body_gyro']))<=.05
        if not stopped:good=[];continue
        if good and p['stamp_ns']-good[-1]['data']['stamp_ns']>200_000_000:good=[]
        good.append(row)
        if len(good)>=3 and p['stamp_ns']-good[0]['data']['stamp_ns']>=300_000_000:return good
    raise MissingEvidence('Actual contiguous measured stop incomplete')

def validate_r85_dynamic_evidence(evidence_doc, task_contract=None, *, profile=None):
    """Recompute real source evidence. Incomplete input waits; invalid input fails.

    evidence_doc is assembled by verify_result from actual journal/guard files;
    it contains records with exact source-line references, not trusted booleans.
    A caller-provided task_contract is mandatory for an external task binding.
    """
    out = dict(schema='r85_dynamic_obstacle_verification/v1', allowed=False, passed=False,
        state='waiting', checks={}, navigation_PASS=False, full_F3_PASS=False,
        physical_navigation_ground_truth_used=False,
        causal_attribution_scope='Bound native guard→stop token and independently observed obstacle spatiotemporal association; no counterfactual or LiDAR object-identity proof',
        existing_passed_flags_used=False)
    checks = out['checks']
    from dynamic_scene import configuration as scene_configuration
    scene=scene_configuration(profile)
    PARK,BLOCK,SIZE=(np.asarray(scene[k],float)for k in ('park_world_xyz','blocking_world_xyz','obstacle_size_m'))
    try:
        _require(isinstance(task_contract, dict), 'External dynamic task contract missing', missing=True)
        _require(task_contract.get('schema') == 'r85_dynamic_task_contract/v1', 'Wrong dynamic task contract schema')
        lead_window = speed_lead.validate_contract(task_contract, profile)
        _require(evidence_doc.get('schema') == 'r85_dynamic_obstacle_evidence/v1', 'Wrong dynamic evidence schema')
        binding = task_contract['binding']
        _require(set(BINDING_KEYS) <= set(binding), 'Incomplete expected identity binding')
        _require(all(isinstance(binding[k], str) and binding[k] for k in ('run_id','reference_id','request_id'))
            and all(_hex(binding[k]) for k in ('reference_sha256','source_route_sha256'))
            and _integer(binding['epoch'], 1) and _integer(binding['localization_generation'], 1), 'Invalid expected identity types')
        run = Path(evidence_doc['run_dir']).resolve()
        _require(run.is_dir() and run.name == binding['run_id'], 'Run directory/identity mismatch')
        _same_binding(evidence_doc['binding'], binding)
        out.update(run_id=binding['run_id'], request_id=binding['request_id'], binding=binding)
        motion_contract=actor_choreography(profile or {})
        _require(task_contract.get('protocol') == (ACTOR_PROTOCOL if motion_contract is not None else PROTOCOL),
            'Wrong explicitly selected R85 actor choreography protocol')
        if motion_contract is not None:
            _require(task_contract.get('actor_motion_contract')==motion_contract, 'Independent world-motion contract differs')
        else:
            _require('actor_motion_contract' not in task_contract, 'Legacy actor cannot claim new world-motion contract')
        _require(task_contract.get('entity', 'moving_obstacle') == 'moving_obstacle'
            and task_contract.get('service', '/world/teacher_demo/set_pose') == '/world/teacher_demo/set_pose'
            and task_contract.get('entity_pose_topic', '/world/teacher_demo/pose/info') == '/world/teacher_demo/pose/info',
            'Entity/service/independent observer protocol differs')
        for key, fixed in (('park_position_world_xyz', PARK), ('blocking_position_world_xyz', BLOCK), ('obstacle_size_m', SIZE)):
            if key in task_contract:
                _require(np.array_equal(_vector(task_contract[key]), fixed), 'Fixed protocol geometry differs: '+key)
        _require(task_contract.get('protocol_parameters') == dict(speed_mps=.25, enter_s=8.4,
            blocking_s=20., leave_s=8.4, retreat_same_side=True), 'Fixed crossing phase parameters differ')
        _require(task_contract.get('phase_positions_world_xyz') == dict(entering=PARK.tolist(), blocking=BLOCK.tolist(),
            leaving=PARK.tolist(), clear=PARK.tolist()), 'Fixed crossing phase positions differ')
        _require(task_contract.get('privileged_actor_pose') is True
            and task_contract.get('navigation_ground_truth_used') is False
            and task_contract.get('robot_pose_service_called') is False, 'Actor privilege/navigation boundary differs')
        rootdoc, root_binding = _read_json(_inside(run, task_contract['task_contract_file']), task_contract['task_contract_sha256'])
        _require(rootdoc.get('schema') == 'r85_pct_scan_student_task/v1' and rootdoc.get('run_id') == run.name
            and rootdoc.get('kind') == 'f1_to_f3' and rootdoc.get('dynamic_required') is True
            and rootdoc.get('short_chain_validation_only') is False, 'Original scoped full-task dynamic contract differs')
        source_doc, source_binding = _read_json(_inside(run, task_contract['typed_source_file']), task_contract['typed_source_sha256'])
        _require(source_binding['sha256'] == binding['source_route_sha256'] == rootdoc['source_typed_route_sha256']
            and Path(rootdoc['source_typed_route_file']).resolve() == Path(source_binding['path']), 'Full typed source/task binding differs')
        _require(source_doc.get('schema') == 'r85_prepared_typed_global_reference/v1'
            and source_doc.get('reference_scope', 'full_route') == 'full_route', 'Full prepared typed source required')
        points = np.asarray(source_doc['body_points'], float)
        _require(points.ndim == 2 and points.shape[1] == 3 and 2 <= len(points) <= 20000 and np.isfinite(points).all(), 'Invalid typed route')
        index=scene['encounter_route_point_index']
        _require(index<len(points) and task_contract['trigger']['encounter_source_point_index']==index
            and np.allclose(points[index]+scene['body_to_world_translation'],task_contract['trigger']['encounter_body_world_xyz'],atol=1e-6,rtol=0),
            'Current route encounter geometry differs')
        transform = np.asarray(source_doc['T_navigation_world'], float)
        _require(np.array_equal(transform, np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-.3],[0,0,0,1]])), 'Frozen canonical transform differs')
        refdoc, ref_binding = _read_json(_inside(run, task_contract['reference_file']), binding['reference_sha256'])
        for key in ('run_id','reference_id','epoch','localization_generation'):
            _require(type(refdoc.get(key)) is type(binding[key]) and refdoc[key] == binding[key], 'Actual reference identity mismatch: '+key)
        _require(refdoc.get('schema') == 'r85_pct_scan_global_reference/v1'
            and refdoc.get('frame_id') == 'camera_init' and refdoc.get('reference_scope') == 'full_route'
            and refdoc.get('navigation_ground_truth_used') is False
            and np.array_equal(np.asarray(refdoc['T_navigation_world'], float), transform)
            and refdoc.get('source_route_sha256') == binding['source_route_sha256']
            and Path(refdoc['source_route_file']).resolve() == Path(source_binding['path'])
            and np.array_equal(np.asarray(refdoc['body_points'], float), points), 'Reference/full typed source mismatch')
        runtime_doc_binding = dict(module='r85_dynamic_obstacle_runtime')
        registration, registration_binding = _read_json(_inside(run, task_contract['registration_file']), task_contract['registration_sha256'])
        _require(registration.get('schema') == 'r85_reference_source_registration/v1'
            and registration.get('run_id') == run.name and registration.get('reference_file_sha256') == binding['reference_sha256']
            and registration.get('reference_id') == binding['reference_id']
            and registration.get('source_route_sha256') == binding['source_route_sha256']
            and type(registration.get('localization_generation')) is int
            and registration['localization_generation'] == binding['localization_generation']
            and registration.get('navigation_ground_truth_used') is False
            and registration.get('producer') == 'root_authorized_PCT_route_owner'
            and Path(registration['task_source_file']).resolve() == Path(root_binding['path']), 'Actual request registration semantics differ')
        checks['full_task_and_source_bindings'] = True
        guard_route_sha=_sha(np.asarray(points,dtype='<f8').tobytes())
        modules, helper_sources = _modules()
        records = evidence_doc['records']
        segment=_navigation_segment(evidence_doc,task_contract,profile,refdoc,run,registration)
        coordinate=segment is not None and 'coordinate_chain'in segment
        if coordinate:checks['coordinate_pose_producer_witness']=_coordinate_pose_sources(run,evidence_doc,segment)
        guard_refdoc=refdoc if segment is None or coordinate else segment['reference']
        guard_ref_binding=ref_binding if segment is None or coordinate else segment['reference_source']
        if coordinate:
            for row in evidence_doc['guards']:
                g=row['data']
                proof=((g.get('union_result')or{}).get('blocked')is True or g.get('obstacle_hold_before')is True
                    or g.get('obstacle_hold_after')is True or any(type(g.get(k))is int and g[k]>0 for k in ('obstacle_stops_before','obstacle_stops_after')))
                if not proof or g.get('request_id')!=binding['request_id']:continue
                clock=g.get('compute_ros_clock_ns');_require(_integer(clock),'Coordinate proof guard source clock missing')
                if clock<task_contract['trigger_sim_ns']:continue
                _verify_line(row);gd,gb=_coordinate_guard_reference(g,segment,refdoc,ref_binding)
                _guard_route(g,points,gd,gb,guard_route_sha,run)
        _require(isinstance(records, list) and len(records) <= MAX_RECORDS, 'Journal record budget invalid')
        by_kind = {}
        previous_sequence, previous_wall, previous_sim = None, None, None
        for row in records:
            raw = row.get('_journal_raw', row['data'])
            _require(raw.get('schema') == 'r85_dynamic_obstacle_record/v1', 'Wrong journal record schema')
            expected_binding=(segment['binding'] if segment is not None and not coordinate and raw['kind'] in ('status','actual_pose','mission_state') and raw['sequence']>segment['sequence'] else binding)
            if coordinate and raw['kind']in ('status','actual_pose','mission_state'):
                expected_binding=binding # Immutable world anchor; actual payload identity is separate.
            _same_binding(raw['binding'], expected_binding)
            seq, sim, wall = raw['sequence'], raw['sim_ns'], raw['monotonic_wall']
            _require(_integer(seq) and _integer(sim) and type(wall) in (int,float) and math.isfinite(wall), 'Journal clock/sequence malformed')
            _require(_integer(task_contract.get('trigger_sim_ns')) and sim >= task_contract['trigger_sim_ns'],
                     'Pretrigger record migrated into locked event journal')
            if previous_sequence is not None:
                _require(seq == previous_sequence+1 and sim >= previous_sim and wall >= previous_wall, 'Journal source order/continuity differs')
            else:
                _require(seq == 1, 'Journal source prefix missing')
            previous_sequence, previous_wall, previous_sim = seq, wall, sim
            _verify_line(row)
            projection = dict(row, data=raw['data'], _journal_raw=raw, received_sim_ns=sim)
            if raw['kind']not in ('status','actual_pose','mission_state') or _new_navigation_row(raw,segment):
                by_kind.setdefault(raw['kind'], []).append(projection)
        monitored = _pretrigger_history(evidence_doc, task_contract, binding, points, refdoc, registration,scene)
        for row in reversed(monitored):
            by_kind.setdefault(row['_journal_raw']['kind'], []).insert(0, row)
        locks = by_kind.get('reference_lock', [])
        _require(len(locks) == 1 and locks[0]['received_sim_ns'] == task_contract['trigger_sim_ns']
            and locks[0]['_journal_raw']['sequence'] == 1
            and locks[0]['data'].get('scope_locked_at_trigger') is True
            and locks[0]['data'].get('pretrigger_records_migrated') is False
            and locks[0]['data'].get('arming_pretrigger_sequences') == task_contract['arming_pretrigger_sequences'],
            'Actual trigger scope-lock snapshot missing/conflicting')
        _require(_canonical(locks[0]['data']['document']) == _canonical(refdoc)
            and _canonical(locks[0]['data']['registration_document']) == _canonical(registration)
            and locks[0]['data']['reference_sha256'] == binding['reference_sha256']
            and locks[0]['data']['registration_sha256'] == registration_binding['sha256'],
            'Actual trigger scope-lock source snapshot differs')
        reference_rows = by_kind.get('reference_binding', [])
        _require(len(reference_rows) >= 1, 'Actual reference-binding callback absent', missing=True)
        _require(len(reference_rows) == 1, 'Multiple/conflicting actual reference-binding callbacks')
        reference_row = reference_rows[0]['data']
        _require(reference_row['reference_sha256'] == ref_binding['sha256']
            and _canonical(reference_row['document']) == _canonical(refdoc)
            and reference_row['registration_sha256'] == registration_binding['sha256']
            and _canonical(reference_row['registration_document']) == _canonical(registration), 'Actual reference/registration snapshot differs')
        triggers = by_kind.get('trigger', [])
        _require(len(triggers) >= 1, 'Actual trigger not recorded', missing=True)
        _require(len(triggers) == 1, 'Conflicting/repeated dynamic trigger')
        trigger = triggers[0]; td = trigger['data']
        _require(trigger['received_sim_ns'] == task_contract['trigger_sim_ns'], 'Actual trigger/contract lock clock differs')
        tp = td['actual_pose']
        if segment is not None and not coordinate:
            _require(segment['edge']['world_event_trigger_sim_ns']==trigger['received_sim_ns']
                and segment['edge']['trigger_actor_identity']=={k:td['actual_park_observation'][k]for k in ('entity','entity_id')},'Original world trigger/actor was changed by navigation handoff')
        _require(tp.get('navigation_ground_truth_used') is False and tp.get('frame_id') == 'camera_init'
            and tp.get('source_topic') == '/demo/slam/body_odom', 'Trigger actual SLAM provenance differs')
        if coordinate:
            _require(all(item['edge']['world_event_trigger_sim_ns']==trigger['received_sim_ns']
                and item['edge']['trigger_actor_identity']=={k:td['actual_park_observation'][k]for k in ('entity','entity_id')}
                for item in segment['coordinate_chain']),'Coordinate handoff changed world trigger or actor')
        trigger_position = _vector(tp['position'])
        _require(_integer(tp['stamp_ns']) and 0 <= trigger['received_sim_ns']-tp['stamp_ns'] <= 300_000_000, 'Trigger SLAM source stale')
        matching_poses = [r for r in by_kind.get('actual_pose', []) if r['received_sim_ns'] <= trigger['received_sim_ns']
            and r['data'].get('stamp_ns') == tp['stamp_ns'] and _canonical(r['data']) == _canonical(tp)]
        _require(matching_poses, 'Trigger pose is not an exact actual SLAM callback snapshot', missing=True)
        obstacle_arc = float(np.linalg.norm(np.diff(points[:181], axis=0), axis=1).sum())
        actual_arc, path_error, _ = _projection(points, trigger_position)
        remaining = obstacle_arc-actual_arc
        _require(lead_window[0] <= remaining <= lead_window[1] and path_error <= .3,
                 'Actual trigger is outside fixed remaining-arclength/path-distance window')
        mission_rows = [r for r in by_kind.get('mission_state', [])
            if 0 <= trigger['received_sim_ns']-r['received_sim_ns'] <= 300_000_000]
        _require(mission_rows, 'Fresh actual mission-state callback at trigger absent', missing=True)
        mission = mission_rows[-1]['data']; mission_ref = mission['r85_reference']
        _require(mission.get('run_id') == run.name and mission.get('current_request') == binding['request_id']
            and mission.get('stage') == 'navigating' and mission.get('task_navigation_state') == 'running'
            and mission.get('navigation_ground_truth_used') is False, 'Actual trigger mission-state provenance differs')
        for key in ('reference_id','epoch','localization_generation','source_route_sha256'):
            _require(type(mission_ref.get(key)) is type(binding[key]) and mission_ref[key] == binding[key], 'Actual mission reference binding differs: '+key)
        _require(mission_ref['global_reference_sha256'] == binding['reference_sha256'], 'Actual mission reference SHA differs')
        _require(_canonical(td['actual_mission_state']) == _canonical(mission), 'Trigger mission-state snapshot differs from actual callback')
        _require(mission.get('schema') == 'r85_actual_single_task_mission_state/v1'
            and mission_ref.get('reference_scope') == 'full_route'
            and _integer(mission.get('ros_sim_time_ns'))
            and 0 <= trigger['received_sim_ns']-mission['ros_sim_time_ns'] <= 300_000_000
            and _integer(mission.get('source_actual_pose_stamp_ns'))
            and 0 <= trigger['received_sim_ns']-mission['source_actual_pose_stamp_ns'] <= 300_000_000,
            'Trigger actual task source clocks/scope differ')
        checks['actual_slam_trigger'] = dict(remaining_arclength_m=remaining, path_distance_m=path_error, obstacle_arc_m=obstacle_arc)
        acks = by_kind.get('service_ack', [])
        phases = {r['data'].get('phase') for r in acks}
        _require({'entering','blocking','leaving','clear'} <= phases, 'Actual ACK phase cycle incomplete', missing=True)
        phase_order = {'parked':-1, 'entering':0, 'blocking':1, 'leaving':2, 'clear':3}
        last_phase = -1
        previous_elapsed = -1.
        requests = by_kind.get('service_request', [])
        if motion_contract is not None:
            identity=td['actual_park_observation']['entity_id']
            for row in requests:
                d=row['data']
                if d.get('phase')=='parked':continue
                _require(d.get('actor_choreography')==ACTOR_SELECTION and d.get('world_motion_allowed')is True
                    and type(d.get('source_fresh'))is bool
                    and type(d.get('actor_entity_id'))is int and d['actor_entity_id']==identity,
                    'World-motion request lacks actual selected permit or trigger actor identity')
                prior=[r for r in by_kind.get('entity_observation', [])
                    if r['received_sim_ns']<=row['received_sim_ns']
                    and r['_journal_raw']['monotonic_wall']<=row['_journal_raw']['monotonic_wall']]
                _require(prior and 0<=row['received_sim_ns']-prior[-1]['data']['stamp_ns']<=300_000_000
                    and 0<=row['_journal_raw']['monotonic_wall']-prior[-1]['_journal_raw']['monotonic_wall']<.3
                    and prior[-1]['data'].get('entity_id')==identity,
                    'World-motion request lacks fresh independently observed trigger actor')
        park_acks = []
        for row in acks:
            a = row['data']; phase = a['phase']
            _require(phase in phase_order and phase_order[phase] >= last_phase, 'ACK phase order differs')
            last_phase = phase_order[phase]
            _require(a.get('success') is True and a.get('entity') == 'moving_obstacle'
                and a.get('service') == '/world/teacher_demo/set_pose', 'Actual entity/service ACK rejected')
            request, response = a['request_sim_ns'], a['response_sim_ns']
            _require(_integer(request) and _integer(response) and request <= response <= row['received_sim_ns'], 'Service ACK causal clock mismatch')
            matching = [r for r in requests if r['data'].get('request_sim_ns') == request]
            _require(len(matching) == 1 and matching[0]['received_sim_ns'] == request
                and all(a.get(k) == v for k,v in matching[0]['data'].items()), 'ACK does not bind one exact actual service request')
            pos = _vector(a['position_world_xyz'])
            _require(abs(pos[1]-5.7) <= 1e-9 and abs(pos[2]-1.8) <= 1e-9 and 14.9-1e-9 <= pos[0] <= 17.+1e-9, 'ACK fixed crossing geometry differs')
            if phase == 'parked':
                _require(response <= trigger['received_sim_ns'] and np.max(np.abs(pos-PARK)) <= 1e-9, 'Initial park ACK geometry/order differs')
                park_acks.append(row)
            else:
                _require(trigger['received_sim_ns'] <= request, 'Motion request preceded actual trigger')
                expected_position, expected_phase = _target_at(a['protocol_elapsed_s'],scene)
                _require(a['protocol_elapsed_s'] >= previous_elapsed and phase == expected_phase
                    and np.max(np.abs(pos-expected_position)) <= 1e-9, 'Actual ACK does not replay fixed phase/time target')
                _require(a['protocol_elapsed_s'] <= (request-trigger['received_sim_ns'])/1e9+1e-9,
                         'Protocol elapsed clock exceeds actual simulation time')
                previous_elapsed = a['protocol_elapsed_s']
        _require(park_acks, 'Initial actual park service ACK missing', missing=True)
        blocking = [r for r in acks if r['data']['phase'] == 'blocking']
        _require(blocking[-1]['data']['request_sim_ns']-blocking[0]['data']['request_sim_ns'] >= 19_000_000_000,
                 'Original blocking ACK duration incomplete', missing=True)
        observations = by_kind.get('entity_observation', [])
        _require(len(observations) >= 3, 'Independent entity pose/info observations missing', missing=True)
        for row in observations:
            d = row['data']
            _require(d.get('entity') == 'moving_obstacle' and d.get('topic') == '/world/teacher_demo/pose/info'
                and d.get('native_gz_pose_info') is True and d.get('source_message_type') == 'gz.msgs.Pose_V',
                'Entity observation is not independent native pose/info')
            _require(_integer(d['stamp_ns']) and 0 <= row['received_sim_ns']-d['stamp_ns'] <= 500_000_000, 'Independent entity observation stale')
            _require(_native_stamp(d['header']) == d['stamp_ns'], 'Native header/observation stamp projection differs')
            _vector(d['position_world_xyz'])
            native = d['raw_model_pose']
            if motion_contract is not None:
                _require(type(d.get('entity_id'))is int and d['entity_id']==td['actual_park_observation']['entity_id']
                    and native.get('id')==d['entity_id'], 'Independent world-motion actor identity changed')
            _require(native.get('name') == 'moving_obstacle', 'Native observer entity name differs')
            native_position = native.get('position', {})
            _require(np.array_equal(_vector([native_position.get(k, 0.) for k in ('x','y','z')]), _vector(d['position_world_xyz'])), 'Native observer position projection differs')
            orientation = native.get('orientation', {})
            q = _vector([orientation.get(k, 0.) for k in ('x','y','z','w')], 4)
            _require(np.linalg.norm(q[:3]) <= 1e-6 and abs(abs(q[3])-1.) <= 1e-6, 'Observed actor is not axis aligned')
        def observed(position, lower, upper):
            return [r for r in observations if lower <= r['data']['stamp_ns'] <= upper
                and np.linalg.norm(_vector(r['data']['position_world_xyz'])-position) <= .05]
        block_observed = observed(BLOCK, blocking[0]['data']['request_sim_ns']-500_000_000,
                                   blocking[-1]['data']['response_sim_ns']+500_000_000)
        clear = [r for r in acks if r['data']['phase'] == 'clear']
        clear_observed = observed(PARK, clear[0]['data']['request_sim_ns'], clear[-1]['received_sim_ns']+500_000_000)
        initial_park_observed = observed(PARK, park_acks[0]['data']['request_sim_ns'], trigger['received_sim_ns'])
        _require(initial_park_observed and 0 <= trigger['received_sim_ns']-initial_park_observed[-1]['data']['stamp_ns'] <= 300_000_000,
                 'Actual independent parked pose near trigger missing', missing=True)
        _require(len(block_observed) >= 2 and clear_observed, 'Independent actual blocking/clear observations incomplete', missing=True)
        _require(block_observed[-1]['data']['stamp_ns']-block_observed[0]['data']['stamp_ns'] >= 19_000_000_000,
                 'Independent actual blocking dwell incomplete', missing=True)
        # A pair of endpoints cannot manufacture an observed 19-second dwell.
        distinct_block = []
        for row in block_observed:
            if distinct_block and row['data']['stamp_ns'] == distinct_block[-1]['data']['stamp_ns']:
                continue
            distinct_block.append(row)
        _strict_clocks(distinct_block, 'stamp_ns', 500_000_000, 39, 19_000_000_000, 'Independent blocking pose')
        _require(any(_canonical(r['data']) == _canonical(td.get('actual_park_observation'))
            for r in initial_park_observed), 'Trigger parked snapshot is not an independent callback')
        for phase, direction in (('entering', -1.), ('leaving', 1.)):
            phase_acks = [r for r in acks if r['data']['phase'] == phase]
            lower = phase_acks[0]['data']['request_sim_ns']
            upper = phase_acks[-1]['data']['response_sim_ns']+500_000_000
            motion_rows = [r for r in observations if lower <= r['data']['stamp_ns'] <= upper]
            _require(len(motion_rows) >= 3, 'Independent '+phase+' observations incomplete', missing=True)
            _require(motion_rows[-1]['data']['stamp_ns']-motion_rows[0]['data']['stamp_ns'] >= 6_000_000_000,
                     'Independent '+phase+' duration incomplete', missing=True)
            positions = [_vector(r['data']['position_world_xyz']) for r in motion_rows]
            _require(all(abs(p[1]-5.7) <= .05 and abs(p[2]-1.8) <= .05 and 14.85 <= p[0] <= 17.05 for p in positions),
                     'Independent '+phase+' path leaves fixed crossing')
            _require(all(direction*(b[0]-a[0]) >= -.05 for a, b in zip(positions, positions[1:]))
                and direction*(positions[-1][0]-positions[0][0]) >= 1.5,
                'Independent '+phase+' displacement/order differs')
        checks['service_ack_and_independent_motion'] = True
        statuses = [r for r in by_kind.get('status', []) if r['data'].get('request_id') == binding['request_id'] and (segment is None or coordinate or r['received_sim_ns']>=segment['cut'])]
        held = [r for r in statuses if r['data'].get('obstacle_hold') is True]
        _require(held, 'Actual obstacle-held status missing', missing=True)
        for row in held:
            command = _vector(row['data']['command'])
            _require(np.max(np.abs(command)) <= 1e-10, 'Held command is nonzero')
        guards = [r for r in evidence_doc['guards'] if r['data'].get('request_id') == binding['request_id']]
        if segment is not None and not coordinate:
            guards=[r for r in guards if r['data'].get('compute_ros_clock_ns',-1)>=segment['cut']
                and r['data'].get('control_pose_stamp_ns',-1)>=segment['cut'] and r['data'].get('cloud_header_stamp_ns',-1)>=segment['cut']]
        _require(guards, 'Actual native guards missing', missing=True)
        stop_windows=_world_stop_windows(held,guards,evidence_doc.get('completed_event_window'),binding['request_id'],modules['stop'])
        held=stop_windows[-1]['held']
        extra_pre=[r for w in stop_windows[:-1] for r in w['preroll']]
        extra_stops=[w['stop_guard'] for w in stop_windows[:-1]]
        extra_clear=[r for w in stop_windows[:-1] for r in w['cleared']]
        try:
            preroll, stop_guard, event = modules['stop'].select_event_preroll(binding['request_id'], held, guards, _verify_line)
        except ValueError as error:
            if 'missing' in str(error) or 'lacks' in str(error):
                raise MissingEvidence(str(error)) from error
            raise
        _strict_clocks(preroll, 'compute_ros_clock_ns', 300_000_000, 3, 300_000_000, 'Native pre-roll')
        last_held = held[-1]['received_sim_ns']
        resumed = [r for r in statuses if r['received_sim_ns'] > last_held and r['data'].get('obstacle_hold') is False
            and type(r['data'].get('obstacle_resumes')) is int and r['data']['obstacle_resumes'] > 0
            and type(r['data'].get('aligned_obstacle_resumes')) is int and r['data']['aligned_obstacle_resumes'] > 0]
        _require(resumed, 'Actual aligned recovery status missing', missing=True)
        resume = resumed[0]['received_sim_ns']
        release, cleared = select_clear_window(guards, event, resume)
        clear_end = release['data']['compute_ros_clock_ns']
        _strict_clocks(cleared, 'compute_ros_clock_ns', 300_000_000, 10, 1_000_000_000, 'Continuous clear guard')
        _require(all(b['data'].get('sequence') == a['data'].get('sequence', -2)+1
            and b['data'].get('previous_actual_guard_clock_ns') == a['data']['compute_ros_clock_ns']
            for a,b in zip(cleared, cleared[1:])), 'Continuous clear native guard source records have a gap')
        _require(all(r['data']['union_result']['blocked'] is False for r in cleared), 'Recovery window contains blocked guard')
        if segment is not None and not coordinate:_verify_segment_control_ACKs(run,[*preroll,*extra_pre,stop_guard,*extra_stops,*cleared,*extra_clear],segment['cut'],binding['request_id'])
        if coordinate:
            for item in segment['coordinate_chain']:
                selected=[r for r in [*preroll,*extra_pre,stop_guard,*extra_stops,*cleared,*extra_clear] if _coordinate_at(segment,clock=r['data']['compute_ros_clock_ns'])is item]
                if selected:_verify_segment_control_ACKs(run,selected,item['cut'],binding['request_id'])
        replay_sources = []
        for row in preroll+extra_pre+cleared+extra_clear:
            _verify_line(row); g = row['data']; clock = g['compute_ros_clock_ns']
            if coordinate:
                gd,gb=_coordinate_guard_reference(g,segment,refdoc,ref_binding)
                _guard_route(g,points,gd,gb,guard_route_sha,run)
            _require(g['cloud_frame_id'] == 'camera_init' and 0 <= clock-g['cloud_header_stamp_ns'] <= 300_000_000
                and 0 <= clock-g['control_pose_stamp_ns'] <= 300_000_000, 'Actual native guard source stale/frame mismatch')
        for row in [stop_guard,*extra_stops,*cleared,*extra_clear]:
            _verify_line(row)
            g = row['data']; clock = g['compute_ros_clock_ns']
            if coordinate:guard_refdoc,guard_ref_binding=_coordinate_guard_reference(g,segment,refdoc,ref_binding)
            payload = modules['payload'].replay_payload(run, g)
            p = _inside(run, payload)
            _require(p.stat().st_size <= 64*1024**2, 'Guard payload exceeds original bounded budget')
            cloud = np.load(p, allow_pickle=False)
            _require(cloud.ndim == 2 and cloud.shape[1] == 3 and np.isfinite(cloud).all(), 'Invalid actual guard cloud payload')
            _require(_sha(np.asarray(cloud, dtype='<f8').tobytes()) == g['filtered_xyz_float64_sha256'], 'Actual guard cloud hash differs')
            native = modules['audit'].NativeGuardAudit(modules['core'].steering_obstacle_ahead)
            result = native(cloud, _vector(g['control_pose']), _vector(g['checked_target']),
                            _vector(g['steering_direction'], 2), _guard_route(g,points,guard_refdoc,guard_ref_binding,guard_route_sha,run))
            replay = modules['audit'].result_json
            _require(replay(result) == g['union_result'] and replay(native.calls[0][1]) == g['goal_corridor_result']
                and replay(native.calls[1][1]) == g['motion_corridor_result'], 'Unchanged actual native two-corridor guard does not replay')
            replay_sources.append(dict(path=str(p), sha256=_sha(p.read_bytes()), sequence=g['sequence']))
            if row is stop_guard or any(row is item for item in extra_stops):
                nearest = min(observations, key=lambda r: abs(r['data']['stamp_ns']-clock))
                _require(abs(nearest['data']['stamp_ns']-clock) <= 300_000_000, 'Independent entity pose near actual stop missing', missing=True)
                obstacle = _vector(nearest['data']['position_world_xyz'])
                world_cloud = cloud-np.array([0,0,-.3])
                nearby = cloud[np.all(np.abs(world_cloud-obstacle) <= SIZE/2+.05, axis=1)]
                associated = modules['core'].steering_obstacle_ahead(nearby, _vector(g['control_pose']),
                    _vector(g['checked_target']), _vector(g['steering_direction'], 2), _guard_route(g,points,guard_refdoc,guard_ref_binding,guard_route_sha,run))
                _require(associated[0] is True or bool(associated[0]), 'Stop cloud has no blocking returns spatially associated with independent entity observation')
        checks['native_cloud_guard_stop_token_and_recovery'] = dict(event=event, pre_roll_count=len(preroll), clear_count=len(cleared),
            clear_release_sequence=release['data']['sequence'], clear_release_clock_ns=clear_end,
            aligned_resume_sim_ns=resume)
        _require(trigger['received_sim_ns'] <= event['clock_ns'] <= clear[0]['data']['request_sim_ns'],
                 'Bound guard stop is outside actual actor motion/occupancy cycle')
        poses = []
        for row in by_kind.get('actual_pose', []):
            p = row['data']; stamp = p['stamp_ns']
            _require(_integer(stamp) and 0 <= row['received_sim_ns']-stamp <= 300_000_000
                and p.get('source_topic') == '/demo/slam/body_odom' and p.get('frame_id') == 'camera_init'
                and p.get('child_frame_id') == 'demo_slam_body' and p.get('navigation_ground_truth_used') is False,
                'Actual SLAM stop/recovery pose provenance differs')
            _vector(p['position']); _vector(p['body_velocity']); _vector(p['body_angular_velocity'])
            if poses and stamp == poses[-1]['data']['stamp_ns']:
                _require(all(p[k] == poses[-1]['data'][k] for k in ('position','body_velocity','body_angular_velocity')), 'Conflicting repeated SLAM pose')
                continue  # A repeated callback must never manufacture dwell.
            poses.append(row)
        _require(all(a['data']['stamp_ns'] < b['data']['stamp_ns'] for a,b in zip(poses, poses[1:])), 'Actual SLAM pose stamps repeat/backtrack')
        stop_window=_measured_stop(poses,held[0]['received_sim_ns'],last_held)
        for w in stop_windows[:-1]:
            w['measured_stop_rows']=len(_measured_stop(poses,w['held'][0]['received_sim_ns'],w['held'][-1]['received_sim_ns']))
        checks['all_actual_stop_release_windows']=[dict(event=w['event'],pre_roll_count=len(w['preroll']),clear_count=len(w['cleared']),clear_release_clock_ns=w['release']['data']['compute_ros_clock_ns'],held_status_count=len(w['held']),final_recovery_credit=i==len(stop_windows)-1)for i,w in enumerate(stop_windows)]
        moving = (_coordinate_recovery_rows(poses,segment,resume) if coordinate
            else [r for r in poses if r['data']['stamp_ns'] >= resume])
        _require(len(moving) >= 3, 'Actual recovery poses missing', missing=True)
        progress = max(float(np.linalg.norm(_vector(r['data']['position'])[:2]-_vector(moving[0]['data']['position'])[:2])) for r in moving)
        _require(any(np.linalg.norm(_vector(r['data']['body_velocity'])[:2]) > .025 for r in moving) and progress >= .08,
                 'Actual moving recovery beyond 8cm incomplete', missing=True)
        checks['measured_stop_and_recovery'] = dict(stop_count=len(stop_window), recovery_pose_count=len(moving), recovery_progress_m=progress)
        for row in [*records, *evidence_doc['pretrigger_records']]:
            _verify_line(row)
        if coordinate:checks['explicit_coordinate_handoffs']=dict(world_binding=binding,navigation_binding=segment['binding'],edges=[x['edge']for x in segment['coordinate_chain']],old_proof_identity_preserved=True)
        if segment is not None and not coordinate:checks['explicit_pre_stop_navigation_edge']=dict(world_binding=binding,navigation_binding=segment['binding'],cut_sim_ns=segment['cut'],old_navigation_stop_credit=False)
        out.update(allowed=True, passed=True, state='passed', failure_reason=None, original_R83_scene_PASS=False,
            source_receipt=dict(root_task=root_binding, typed_source=source_binding, actual_reference=ref_binding,
                runtime_protocol=runtime_doc_binding, registration=registration_binding, native_helpers=helper_sources, replay_payloads=replay_sources,
                journal=evidence_doc.get('journal_source'), pretrigger_monitoring=evidence_doc.get('pretrigger_source'),
                native_guard=evidence_doc.get('guard_source')))
    except MissingEvidence as error:
        out.update(allowed=False, passed=False, state='waiting', failure_reason=str(error))
    except Exception as error:
        out.update(allowed=False, passed=False, state='failed', failure_reason=type(error).__name__+': '+str(error))
    return out


def verify_result(run, request_id, profile, scope, *, evidence_io=None):
    """Owner hook: reread current real files on every call, never a saved PASS."""
    run = Path(run).resolve()
    out = dict(schema='r85_dynamic_task_result/v1', run_id=run.name, request_id=request_id,
        passed=False, state='waiting', failure_reason=None, navigation_PASS=False, full_F3_PASS=False)
    try:
        _require(isinstance(profile, dict) and profile.get('r85_task', {}).get('kind') == 'f1_to_f3', 'Full R85 task profile required')
        _require(isinstance(scope, dict), 'Original source scope mapping missing')
        speed_setting = speed_lead.configuration(profile)
        contract_path = _inside(run, 'r85_dynamic_task_contract.json')
        contract, contract_source = (_read_json(contract_path) if evidence_io is None
            else evidence_io.read_json(contract_path))
        _require(contract['binding']['request_id'] == request_id and contract['binding']['run_id'] == run.name, 'Foreign dynamic task contract')
        actual_state, state_source = (_read_json(_inside(run, 'r85_dynamic_obstacle_state.json')) if evidence_io is None
            else evidence_io.read_json(_inside(run, 'r85_dynamic_obstacle_state.json')))
        _require(actual_state.get('failure') is None, 'Actual obstacle runtime failure: '+str(actual_state.get('failure')))
        _same_binding(actual_state['binding'], contract['binding'])
        from r85_dynamic_obstacle_runtime import event_window_enabled,completed_event_window,EVENT_WINDOW,SEGMENT_WINDOW,COORDINATE_WINDOW,navigation_segments_enabled,navigation_segment_schema
        window=None
        if event_window_enabled(profile):
            event_schema=navigation_segment_schema(profile) if navigation_segments_enabled(profile) else EVENT_WINDOW
            _require(contract.get('event_lifetime_contract')==event_schema
                and contract.get('whole_task_single_scope_claimed')is False,
                'Explicit single completed event-window contract missing')
            _require(actual_state.get('diagnostic_protocol_cancelled')is False,
                'Cancelled dynamic event cannot become completed')
            window=completed_event_window(actual_state)
            _require(window is not None,'Original dynamic event not completed',missing=True)
        rows, journal_source = (_lines(run, 'r85_dynamic_journal.jsonl') if evidence_io is None
            else evidence_io.lines(run, 'r85_dynamic_journal.jsonl'))
        if window is not None:
            _require(rows and rows[-1]['data'].get('kind')=='event_completed_pending_verification'
                and rows[-1]['data'].get('sequence')==window['journal_last_sequence']
                and rows[-1]['data'].get('sim_ns')==window['completed_sim_ns']
                and rows[-1]['data'].get('data')==window,
                'Completed event journal endpoint differs or has later records')
        if window is not None and event_schema in (SEGMENT_WINDOW,COORDINATE_WINDOW):
            edges=[r['data']for r in rows if r['data'].get('kind')=='navigation_segment_boundary']
            _require([r['sequence']for r in edges]==window['navigation_boundary_sequences']
                and window['navigation_binding']==(edges[-1]['data']['new_binding']if edges else contract['binding']),
                'Completed navigation segment does not match actual journal edge')
        journal = []
        for row in rows:
            raw = row['data']
            journal.append(dict(row, _journal_raw=raw))
        guards, guard_source = (_lines(run, 'navigation_guard_history.jsonl') if evidence_io is None
            else evidence_io.lines(run, 'navigation_guard_history.jsonl'))
        if window is not None:
            # Later navigation guards belong to later routes, outside this
            # explicitly completed event. Source records remain on disk.
            guards=[r for r in guards if r['data']['compute_ros_clock_ns']<=window['completed_sim_ns']]
        pretrigger, pretrigger_source = (_lines(run, contract['pretrigger_history_file']) if evidence_io is None
            else evidence_io.lines(run, contract['pretrigger_history_file']))
        evidence = dict(schema='r85_dynamic_obstacle_evidence/v1', run_dir=str(run), binding=contract['binding'],
                        records=journal, guards=guards, journal_source=journal_source, guard_source=guard_source,
                        pretrigger_records=pretrigger, pretrigger_source=pretrigger_source,completed_event_window=window)
        receipt = validate_r85_dynamic_evidence(evidence, contract, profile=profile)
        if window is not None and receipt.get('state')=='waiting':
            # The completed event prefix is immutable: future callbacks cannot
            # fill missing historical observations. Preserve a terminal failure.
            receipt.update(state='failed',allowed=False,passed=False,
                failure_reason='Completed dynamic event evidence incomplete: '+str(receipt.get('failure_reason')))
        if window is not None:
            out.update(event_lifetime_contract=event_schema,event_window=window,
                whole_task_single_scope_claimed=False,original_nine_gate_verification=event_schema==EVENT_WINDOW,
                explicit_navigation_segment_identity=event_schema in (SEGMENT_WINDOW,COORDINATE_WINDOW),
                original_numeric_physical_stop_recovery_predicates=True)
        out.update(passed=receipt['allowed'], state=receipt['state'], failure_reason=receipt.get('failure_reason'),
            verification=receipt, source_receipt=dict(contract=contract_source, runtime_state=state_source,
                journal=journal_source, pretrigger_monitoring=pretrigger_source, native_guard=guard_source))
    except MissingEvidence as error:
        out.update(state='waiting', failure_reason=str(error))
    except Exception as error:
        out.update(state='failed', failure_reason=type(error).__name__+': '+str(error))
    return out
