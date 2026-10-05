#!/usr/bin/env python3
"""Static geometry math only: no Actor, ROS, simulation or existing-file writes."""
import hashlib,importlib.util,json,math,sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent;T=OUT.parents[2];DEMO=T.parent
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    target=OUT/'height_provider_math.json'
    if target.exists():raise RuntimeError('Refuse overwrite')
    source=T/'policy/observation.py';spec=importlib.util.spec_from_file_location('static_provider_math',source)
    mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod)
    world=DEMO/'simulation/generated/three_floors.sdf'
    manifests={x:json.loads((T/f'tests/terrain_targets/{x}.json').read_text())for x in ('all_static','lower12','upper23')}
    maps={x:mod.TerrainHeightMap.from_sdf(world,include_models=v['include_models'])for x,v in manifests.items()}
    samples=[('lower_entry_spawn',1.2,2,.4,0),('lower_entry_settled',1.2,2,.3,0),
       ('lower_seam',2,2,.3,0),('lower_overhead_edge',2.8,2,.38,0),('lower_after_overhead',2.9,2,.39,0),
       ('lower_mid',8,2,.9,0),('lower_high',12,2,1.3,0),('lower_landing',14.8,2,1.5,0),
       ('connector_corner',16,2,1.5,math.pi/2),('connector_mid',16,4.5,1.5,math.pi/2),
       ('connector_upper_corner',16,7,1.5,math.pi),('upper_entry',14.8,7,1.5,math.pi),
       ('upper_mid',8,7,2.1,math.pi),('upper_high',4,7,2.5,math.pi),
       ('upper_landing',1.2,7,2.7,math.pi),('floor3',0,7,2.7,math.pi),('floor1_below_floor3',1.2,7,.3,0)]
    rows=[]
    for name,x,y,z,yaw in samples:
        state=np.zeros(64);state[1:4]=[x,y,z];state[4:8]=[math.cos(yaw/2),0,0,math.sin(yaw/2)];state[14:26]=mod.DEFAULT_Q
        values={}
        for provider,terrain in maps.items():
            obs,extra=mod.build_observation(state,np.zeros(3),np.zeros(12),terrain)
            counts={}
            for hit in extra['height_scan_hit_models']:
                key=hit.split('/')[0]if hit else'missing';counts[key]=counts.get(key,0)+1
            values[provider]={'hit_model_counts':counts,**{k:extra[k]for k in ('height_scan_valid_count','height_scan_invalid_count','height_scan_overhead_count','height_scan_clip_fraction','height_scan_raw_min','height_scan_raw_max')},
                             'scan_clipped_min':float(obs[-187:].min()),'scan_clipped_max':float(obs[-187:].max())}
        rows.append({'name':name,'hypothetical_base_xyz_yaw':[x,y,z,yaw],'not_actual_motion':True,'providers':values})
    # Check the full proposed switching guard region, allowing any yaw and ±.45m
    # tracking deviation about x16,y4.5, plus up to5cm placement uncertainty.
    maxdiff=0.;n=0;non_common=[]
    for x in (15.5,16,16.5):
      for y in (4,4.5,5):
       for yaw in np.linspace(-math.pi,math.pi,37):
        rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        xy=mod.SCAN_GRID_XY@rot.T+[x,y];starts=np.column_stack((xy,np.full(187,21.5)))
        hz0,n0=maps['lower12'].raycast(starts);hz1,n1=maps['upper23'].raycast(starts)
        diff=float(np.max(abs(hz0-hz1)));maxdiff=max(maxdiff,diff);n+=1
        if any(s is None or not s.startswith('floor_2/')for s in n0+n1):non_common.append([x,y,yaw])
    # Overhead on the original ramp12 centerline, a static sample grid only.
    overhead=[]
    for x in np.arange(1.2,14.80001,.1):
        z=.4+np.clip((x-2)*.1,0,1.2)
        xy=mod.SCAN_GRID_XY+[x,2];hz,_=maps['all_static'].raycast(np.column_stack((xy,np.full(187,z+20))))
        if (hz>z).any():overhead.append({'x_m':float(x),'overhead_rays':int((hz>z).sum())})
    dx=.8;dy=.5;extent=math.hypot(dx,dy)
    result={'schema':'static_multifloor_actor_height_provider_math/v1','actual_motion_or_policy_test':False,
       'sources':{str(p):sha(p)for p in [source,T/'policy/contract.json',world,*[T/f'tests/terrain_targets/{x}.json'for x in manifests]]},
       'scan':{'points':187,'local_x_range_m':[-.8,.8],'local_y_range_m':[-.5,.5],'spacing_m':.1,'yaw_only':True,
               'vertical_ray_start':'+20m from base origin','raw_formula':'base_z-hit_z-.5','clip':[-1,1],
               'maximum_axis_half_extent_any_yaw_m':extent},
       'sampled_hypothetical_poses':rows,
       'common_floor2_switch_region':{'nominal_scene_xy':[16,4.5],'tested_body_xy_box':[[15.5,16.5],[4,5]],
          'yaw_count':37,'pose_count':n,'maximum_hit_z_delta_lower_upper_m':maxdiff,'non_floor2_pose_count':len(non_common),
          'ray_x_bounds_conservative':[15.5-extent,16.5+extent],
          'ray_y_bounds_conservative':[4-extent,5+extent],
          'body_z_example':1.5,'scan_raw_on_common_floor2':-.2,
          'exact_current_frame_equality_required_at_runtime':True,'not_an_actual_navigation_arrival':True},
       'all_static_lower12_centerline_overhead_samples':overhead,
       'native_z_only_switch_is_invalid':'At base_z1.3 on lower ramp x12,y2, upper23 has no ramp12 hits; both layers share floor2. Height alone cannot decide the directed route transition.',
       'short_registered_prior_route':[[1.2,2,0],[2,2,0],[14,2,1.2],[14.8,2,1.2],[15.5,2,1.2],[15.5,4.5,1.2],[15.5,7,1.2],[14.8,7,1.2],[14,7,1.2],[2,7,2.4],[1.2,7,2.4]],
       'short_route_horizontal_length_m':33.6,
       'conservative_registered_prior_route':[[1.2,2,0],[2,2,0],[14,2,1.2],[14.8,2,1.2],[16,2,1.2],[16,4.5,1.2],[16,7,1.2],[14.8,7,1.2],[14,7,1.2],[2,7,2.4],[1.2,7,2.4]],
       'conservative_route_horizontal_length_m':34.6,
       'whole_leg_height_interpolation_error_max_m':.8*(1.2/13.6),
       'navigation_truth_used':False,'meaning':'Offline static prior math; source registration, dynamics, leg clearance and navigation are not tested.'}
    with target.open('x')as f:json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps({'file':str(target),'sha256':sha(target),'switch_maxdiff':maxdiff,'switch_non_common':len(non_common),'overhead_x':[r['x_m']for r in overhead]}))
if __name__=='__main__':main()
