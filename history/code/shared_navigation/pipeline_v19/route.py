"""Immutable measured-SLAM routes; scene registration is a declared map prior."""
import hashlib,json,math


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)


def registered_points(anchor,profile):
    """Map static scene offsets into SLAM without a simulator pose input.

    The known spawn XY is a translation prior, not a measured localization.
    Vertical coordinates are support elevations relative to the initial floor;
    the measured initial body-origin height is added exactly once.
    """
    origin=anchor['origin'];yaw=anchor['scene_axis_registration']['heading_receipt']['yaw_camera_init_from_world']
    values=profile['route_world_points'];spawn=profile['spawn'];z0=profile['scene_initial_support_z']
    if (not isinstance(values,list) or not values or len(origin)!=3 or len(spawn)!=4 or
        not all(isinstance(v,(int,float))and not isinstance(v,bool)and math.isfinite(v)
                for v in [*origin,*spawn,z0,yaw])):raise ValueError('Invalid registered scene route coordinates')
    seen=set();mapped=[];c,s=math.cos(yaw),math.sin(yaw)
    for row in values:
        gid=row.get('goal_id');point=row.get('xyz')
        if (not isinstance(gid,str)or not gid or gid in seen or not isinstance(point,list)or len(point)!=3 or
            not all(isinstance(v,(int,float))and not isinstance(v,bool)and math.isfinite(v)and abs(v)<=1000 for v in point)):
            raise ValueError('Scene route requires unique goal IDs and finite xyz priors')
        seen.add(gid);dx,dy=point[0]-spawn[0],point[1]-spawn[1]
        mapped.append([origin[0]+c*dx-s*dy,origin[1]+s*dx+c*dy,origin[2]+point[2]-z0])
    return mapped


def bind_registered_route(anchor,profile):
    mapped=registered_points(anchor,profile)
    anchor.update(registered_route_camera_init_xyz=[list(anchor['origin']),*mapped],
        route_world_points=profile['route_world_points'],
        route_world_points_sha256=hashlib.sha256(canonical(profile['route_world_points']).encode()).hexdigest(),
        registered_route_points_sha256=hashlib.sha256(canonical([list(anchor['origin']),*mapped]).encode()).hexdigest(),
        known_spawn_xy_prior={'xy':list(profile['spawn'][:2]),'uncertainty_m':.15,
            'source':'fixed scene/map spawn prior; no live Gazebo pose or automatic point-cloud registration',
            'navigation_ground_truth_used':False},
        scene_initial_support_z=float(profile['scene_initial_support_z']),
        registered_route_reference='single sensor-derived R_camera_init_world yaw and measured initial body-origin translation')
    return mapped


def build_request(run_id,anchor,profile,arrival):
    if anchor.get('source')!='/demo/slam/body_odom'or anchor.get('ground_truth_navigation_used')is not False:
        raise ValueError('PID routes require a sensor-SLAM anchor')
    if profile.get('route_world_points') is not None:
        required={'type':'disc_prism','radius_m':.22,'height_half_span_m':.1,'dwell_sim_s':.6,
            'control_band':{'type':'disc_prism','radius_m':.17,'height_half_span_m':.1}}
        if canonical(arrival)!=canonical(required):raise ValueError('Registered route arrival thresholds changed')
        mapped=registered_points(anchor,profile)
        if anchor.get('registered_route_camera_init_xyz')!=[list(anchor['origin']),*mapped]:
            raise ValueError('Registered route differs from the frozen anchor')
        goals=[{'goal_id':row['goal_id'],'center':point,'arrival':arrival,'timeout_sim_s':90.}
               for row,point in zip(profile['route_world_points'],mapped)]
        return {'schema_version':2,'request_id':run_id+'_registered_scene_route','frame_id':'camera_init','goals':goals}
    origin=list(anchor['origin']);yaw=anchor['yaw'];d=profile['distance_m']
    destination=[origin[0]+d*math.cos(yaw),origin[1]+d*math.sin(yaw),origin[2]+profile['route_height_delta_m']]
    goals=[{'goal_id':profile['selector']+'_destination','center':destination,'arrival':arrival,'timeout_sim_s':profile['goal_timeout_sim_s']}]
    if profile['return_to_origin']:
        goals.append({'goal_id':'return_origin','center':origin,'arrival':arrival,'timeout_sim_s':profile['goal_timeout_sim_s']})
    return {'schema_version':2,'request_id':run_id+'_sensor_slam_pid_route','frame_id':'camera_init','goals':goals}
