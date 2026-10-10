"""Versioned trigger-only view of real ordered continuous-route coverage.

The actual controller state/index is never modified. Original SceneTrigger still
owns scene calibration, proximity and stage/request checks. This adapter merely
maps one validated ordinary pass to its original first-waypoint trigger meaning.
"""
import copy
import hashlib
import json
import math
import numpy as np
from navigation.goal_regions import contains, contains_control, definitions_sha256, parse_goal

SCHEMA = 'original46_continuous_scene_trigger_view/v1'
ROUTE = 'original46_continuous_route/v1'
PASS = 'original_region_ordered_pass/v1'


def sha(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),
                                    allow_nan=False).encode()).hexdigest()


def scene_trigger_view(state):
    """Return a separate view and deterministic replayable conversion proof.

    Invalid continuous evidence yields trigger index -1, never falls through to
    legacy index interpretation. Legacy mission states are copied unchanged.
    """
    view = copy.deepcopy(state)
    nav = state.get('navigation') if isinstance(state,dict) else None
    route = nav.get('continuous_route') if isinstance(nav,dict) else None
    proof = dict(schema=SCHEMA,applied=False,continuous_route=route is not None,
                 reason='legacy_trigger_view_unchanged',navigation_ground_truth_used=False)
    if route is None:
        return view,proof
    proof.update(reason='continuous_pass_evidence_not_ready',
                 actual_controller_waypoint_index=nav.get('waypoint_index'),
                 semantic_trigger_waypoint_index=-1,source_navigation_sha256=sha(nav))
    # Disable only the independent view; retain the complete actual navigation
    # state in the producer's original mission_state field and proof binding.
    view['navigation']['actual_controller_waypoint_index'] = nav.get('waypoint_index')
    view['navigation']['waypoint_index'] = -1
    view['navigation']['trigger_index_semantics'] = SCHEMA
    try:
        rid=state.get('current_request')
        if (state.get('stage') != 'navigating' or state.get('navigation_ground_truth_used') is not False
                or not isinstance(rid,str) or nav.get('request_id') != rid or nav.get('state') != 'running'
                or nav.get('navigation_ground_truth_used') is not False or nav.get('frame_id') != 'camera_init'
                or not isinstance(route,dict) or route.get('schema') != ROUTE
                or route.get('request_id') != rid or route.get('navigation_ground_truth_used') is not False
                or route.get('next_coverage_index') != 1 or type(route.get('next_coverage_index')) is not int
                or route.get('hard_target_index') != 7 or nav.get('waypoint_index') != 7
                or route.get('hard_stop_indices') != [7,13]
                or route.get('ordinary_pass_count') != 1 or route.get('original_dwell_count') != 0):
            return view,proof
        raw=nav.get('goals_definitions')
        if (not isinstance(raw,list) or len(raw) != 14 or state.get('current_goals') != raw
                or nav.get('total') != 14):
            return view,proof
        goals=tuple(parse_goal(g) for g in raw)
        if [g.goal_id for g in goals] != [f'navigation_f1_f3:{i}' for i in range(14)]:
            return view,proof
        goal_hash=definitions_sha256(goals)
        if (nav.get('goals_definition_sha256') != goal_hash or route.get('goals_definition_sha256') != goal_hash
                or state.get('current_goals_sha256') != goal_hash):
            return view,proof
        points=np.asarray(route.get('route_points_xyz'),dtype=float)
        if (points.shape != (15,3) or not np.isfinite(points).all()
                or not np.array_equal(points[1:],np.asarray([g.center for g in goals]))
                or sha(points.tolist()) != route.get('route_sha256')):
            return view,proof
        receipts=nav.get('region_arrivals')
        if not isinstance(receipts,list) or len(receipts) != 1 or not isinstance(receipts[0],dict):
            return view,proof
        r=receipts[0];g=goals[0]
        stamp,start,dwell,prior=(r.get(k) for k in ('stamp_ns','start_stamp_ns','dwell_ns','previous_observation_stamp_ns'))
        if (r.get('schema') != PASS or r.get('completion_semantics') != 'ordered_pass_without_dwell'
                or r.get('request_id') != rid or r.get('goal_id') != 'navigation_f1_f3:0'
                or r.get('waypoint_index') != 0 or type(r.get('waypoint_index')) is not int
                or r.get('goals_definition_sha256') != goal_hash
                or r.get('route_sha256') != route['route_sha256']
                or r.get('reason') != 'passed' or r.get('original_dwell_satisfied') is not False
                or r.get('navigation_ground_truth_used') is not False or r.get('protected') is not False
                or r.get('region_inside') is not True or r.get('control_region_inside') is not True
                or r.get('arrival_definition') != g.definition()['arrival']
                or r.get('control_arrival_definition') != g.control_arrival_definition()
                or r.get('original_route_surface_id') != g.route_surface_id
                or r.get('max_observation_gap_ns') != 200_000_000
                or any(type(v) is not int for v in (stamp,start,dwell,prior))
                or not 0<=prior<stamp or stamp-start != 0 or dwell != 0 or stamp-prior>200_000_000
                or not contains(g,r.get('raw_position')) or not contains_control(g,r.get('raw_position'))):
            return view,proof
        vertex=float(np.linalg.norm(points[1,:2]-points[0,:2]));p=r.get('route_projection') or {}
        arc=p.get('arc_m');progress=p.get('monotonic_progress_m')
        if (not math.isclose(r.get('route_vertex_arc_m',-1.),vertex,abs_tol=1e-8)
                or type(arc) not in (int,float) or type(progress) not in (int,float)
                or not math.isfinite(arc) or not math.isfinite(progress)
                or abs(arc-vertex)>.5 or progress<arc or progress>vertex+.25+1e-8):
            return view,proof
        proof.update(applied=True,reason='validated_first_original_region_ordered_pass',
            semantic_trigger_waypoint_index=1,source='actual_controller_continuous_coverage_and_original_pass_receipt',
            source_route_sha256=route['route_sha256'],source_goals_definition_sha256=goal_hash,
            source_pass_receipt_sha256=sha(r),source_pass_receipt=copy.deepcopy(r),
            original_region_geometry_unchanged=True,original_dwell_claimed=False)
        view['navigation']['waypoint_index']=1
        return view,proof
    except (ValueError,TypeError,KeyError,AttributeError,IndexError):
        proof['reason']='continuous_pass_evidence_invalid'
        return view,proof
