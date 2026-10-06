"""Freeze the scene obstacle trigger in the same SLAM frame as mission routes.

The scene anchor is the known spawn location. [1, 2, 0] is a world-axis
displacement from that anchor, not an absolute camera_init coordinate. The
mission supplies its one-time sensor-derived alignment and initial SLAM body
position. This module has no simulator pose or ground-truth input.
"""
import math


class SceneTrigger:
    def __init__(self):
        self.run_id = None
        self.stage = None
        self.point = None
        self.reference = None
        self.error = 'waiting for frozen mission scene calibration'
        self.navigation_ready = False
        self.navigation_index = None

    def observe(self, state):
        self.navigation_ready = False
        self.navigation_index = None
        if not isinstance(state, dict):
            self.error = 'mission state must be an object'
            return
        run_id = state.get('run_id')
        self.stage = state.get('stage')
        navigation = state.get('navigation')
        current_request = state.get('current_request')
        if isinstance(navigation, dict):
            self.navigation_index = navigation.get('waypoint_index')
            # Start the crossing only after the northward approach has actually
            # reached its first waypoint. Triggering during that approach can
            # finish the entire obstacle cycle before the eastward turn ends.
            self.navigation_ready = (
                isinstance(current_request, str) and bool(current_request)
                and navigation.get('request_id') == current_request
                and navigation.get('state') == 'running'
                and type(self.navigation_index) is int and self.navigation_index == 1)
        if not isinstance(run_id, str) or not run_id:
            self.error = 'mission run_id missing'
            return
        if run_id != self.run_id:
            self.run_id, self.point, self.reference = run_id, None, None
        origin = state.get('origin')
        alignment = state.get('heading_alignment')
        try:
            if not isinstance(alignment, dict) or alignment.get('ground_truth_used') is not False:
                raise ValueError('frozen sensor-derived heading alignment missing')
            if not isinstance(origin, (list, tuple)) or len(origin) != 3:
                raise ValueError('initial SLAM body origin missing')
            origin = tuple(float(v) for v in origin)
            angle = float(alignment['yaw_camera_init_from_world'])
            if not all(map(math.isfinite, (*origin, angle))):
                raise ValueError('non-finite mission scene calibration')
            reference = (*origin, angle)
            if self.reference is not None and reference != self.reference:
                raise ValueError('mission changed its frozen scene calibration')
            if self.reference is None:
                c, s = math.cos(angle), math.sin(angle)
                self.point = [origin[0]+c-2*s, origin[1]+s+2*c, origin[2]]
                self.reference = reference
            self.error = None
        except (ValueError, KeyError, TypeError) as exc:
            self.error = str(exc)

    @property
    def ready(self):
        return (self.stage == 'navigating' and self.navigation_ready
                and self.point is not None and self.error is None)

    def nearby(self, body_xy, radius=1.4):
        return self.ready and math.hypot(body_xy[0]-self.point[0], body_xy[1]-self.point[1]) < radius
