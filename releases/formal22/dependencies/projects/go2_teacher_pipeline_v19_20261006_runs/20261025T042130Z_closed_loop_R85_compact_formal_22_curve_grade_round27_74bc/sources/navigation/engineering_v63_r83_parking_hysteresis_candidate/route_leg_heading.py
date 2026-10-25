"""Immutable yaw reference from an activated, actual-SLAM mission leg.

No simulator state or SCAN geometry is used to choose this heading. The checked
SCAN target remains the XY PID target and its original heading remains evidence.
"""
from dataclasses import dataclass
import math
import numpy as np


class RouteLegError(ValueError):
    pass


def coordinates(value, name):
    try:array = np.asarray(value, dtype=float)
    except (ValueError,TypeError,OverflowError) as error:
        raise RouteLegError('Invalid actual-SLAM route leg ' + name) from error
    if array.shape != (3,) or not np.isfinite(array).all():
        raise RouteLegError('Invalid actual-SLAM route leg ' + name)
    return tuple(float(x) for x in array)


@dataclass(frozen=True)
class RouteLegReference:
    request_id: str
    waypoint_index: int
    start: tuple
    goal: tuple
    activation_pose_stamp_ns: int
    activation_ros_clock_ns: int
    activation_quaternion_xyzw: tuple
    heading: float

    def evidence(self):
        return {'schema': 'actual_slam_route_leg_heading/v1',
            'reference': 'route_leg', 'reference_source': 'frozen activated mission goal minus actual raw SLAM segment_start',
            'frame_id': 'camera_init', 'origin_source': '/demo/slam/body_odom',
            'goal_source': 'this run frozen SLAM-relative navigation_request.json goal center',
            'request_id': self.request_id, 'waypoint_index': self.waypoint_index,
            'leg_start': list(self.start), 'leg_goal': list(self.goal),
            'activation_pose_stamp_ns': self.activation_pose_stamp_ns,
            'activation_ros_clock_ns': self.activation_ros_clock_ns,
            'activation_quaternion_xyzw': list(self.activation_quaternion_xyzw),
            'heading_rad': self.heading, 'navigation_ground_truth_used': False}


class RouteLegLedger:
    def __init__(self):
        self.references = {}

    @staticmethod
    def key(request_id, waypoint_index):
        if (not isinstance(request_id, str) or not request_id
                or type(waypoint_index) is not int or waypoint_index < 0):
            raise RouteLegError('Invalid route leg request/waypoint identity')
        return request_id, waypoint_index

    def freeze(self, request_id, waypoint_index, start, goal, pose_stamp_ns,
               clock_ns, quaternion_xyzw):
        key = self.key(request_id, waypoint_index)
        start, goal = coordinates(start, 'start'), coordinates(goal, 'goal')
        if key in self.references:
            return self.get(request_id, waypoint_index, start, goal), False
        if (type(pose_stamp_ns) is not int or type(clock_ns) is not int or pose_stamp_ns < 0
                or not -50_000_000 <= clock_ns-pose_stamp_ns < 300_000_000):
            raise RouteLegError('Route leg activation lacks a fresh original SLAM header')
        try:q = np.asarray(quaternion_xyzw, dtype=float)
        except (ValueError,TypeError,OverflowError) as error:
            raise RouteLegError('Invalid actual SLAM activation quaternion') from error
        if q.shape != (4,) or not np.isfinite(q).all() or not .98 <= float(q @ q) <= 1.02:
            raise RouteLegError('Route leg activation lacks a valid actual SLAM quaternion')
        delta = np.asarray(goal[:2])-np.asarray(start[:2])
        norm=float(np.linalg.norm(delta))
        if not np.isfinite(delta).all() or not math.isfinite(norm) or norm <= 1e-6:
            raise RouteLegError('Degenerate actual-SLAM route leg; no heading is authorized')
        heading = math.atan2(float(delta[1]), float(delta[0]))
        reference = RouteLegReference(request_id, waypoint_index, start, goal,
            pose_stamp_ns, clock_ns, tuple(float(x) for x in q), heading)
        self.references[key] = reference
        return reference, True

    def get(self, request_id, waypoint_index, start, goal):
        key = self.key(request_id, waypoint_index)
        reference = self.references.get(key)
        if reference is None:
            raise RouteLegError('Route leg was not frozen at actual SLAM activation')
        if coordinates(start, 'start') != reference.start or coordinates(goal, 'goal') != reference.goal:
            raise RouteLegError('Frozen actual-SLAM route leg changed under the same identity')
        return reference


def apply_route_leg(reference, steering, yaw):
    """Change only yaw fields; retain the original checked target/corridor."""
    if not all(math.isfinite(float(x)) for x in (yaw,steering['heading'],steering['error'])):
        raise RouteLegError('Nonfinite actual SLAM yaw or original SCAN heading')
    result = dict(steering)
    result['scan_heading_rad'] = float(steering['heading'])
    result['scan_heading_error_rad'] = float(steering['error'])
    result['heading'] = reference.heading
    difference = reference.heading-yaw
    result['error'] = math.atan2(math.sin(difference), math.cos(difference))
    result['heading_reference'] = reference.evidence()
    return result
