"""Native SCAN safety envelope with bounded V34 local planning horizon, shared by epochs."""
def parameters(profile):
    horizon=float(profile['continuous_route_contract']['local_planning_horizon_m'])
    if horizon!=.65:raise ValueError('Only reviewed V34 bounded horizon permitted')
    return {
                    'use_sim_time':True,'fsm.navi_mode':3,'fsm.planning_horizon':horizon,'fsm.thresh_replan':.35,
                    'fsm.max_replan_fail_count':50,'fsm.measured_max_speed':profile['max_speed_mps'],
                    'fsm.measured_velocity_filter_tau':.4,'grid_map.frame_id':'camera_init',
                    'grid_map.sliding_map_frame_id':'teacher_scan_sliding_map','grid_map.sensor_type':'lidar',
                    'grid_map.cloud_is_world':True,'grid_map.need_extrinsic':False,'grid_map.resolution':.08,
                    'grid_map.body_height':.4,'grid_map.obstacles_inflation_z_down':.12,
                    'grid_map.obstacles_inflation_z_up':.12,'grid_map.double_cylinder_radius':.25,
                    'grid_map.double_cylinder_offset':.18,'grid_map.sliding_map_size_x':10.,
                    'grid_map.sliding_map_size_y':10.,'grid_map.sliding_map_size_z':5.,
                    'grid_map.local_update_range_x':5.,'grid_map.local_update_range_y':5.,'grid_map.local_update_range_z':2.5,
                    'manager.max_vel':profile['max_speed_mps'],'manager.max_acc':.15,'manager.planning_horizon':horizon,
                    'optimization.max_vel':profile['max_speed_mps'],'optimization.max_acc':.15}
