"""Immutable routes relative to the first warmed actual SLAM pose and yaw."""
import math


def build_request(run_id,anchor,profile,arrival):
    if anchor.get('source')!='/demo/slam/body_odom'or anchor.get('ground_truth_navigation_used')is not False:
        raise ValueError('PID routes require a sensor-SLAM anchor')
    origin=list(anchor['origin']);yaw=anchor['yaw'];d=profile['distance_m']
    destination=[origin[0]+d*math.cos(yaw),origin[1]+d*math.sin(yaw),origin[2]+profile['route_height_delta_m']]
    goals=[{'goal_id':profile['selector']+'_destination','center':destination,'arrival':arrival,'timeout_sim_s':profile['goal_timeout_sim_s']}]
    if profile['return_to_origin']:
        goals.append({'goal_id':'return_origin','center':origin,'arrival':arrival,'timeout_sim_s':profile['goal_timeout_sim_s']})
    return {'schema_version':2,'request_id':run_id+'_sensor_slam_pid_route','frame_id':'camera_init','goals':goals}
