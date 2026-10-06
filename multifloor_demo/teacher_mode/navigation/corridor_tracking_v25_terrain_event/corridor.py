"""Bounded geometric corridor evidence from measured SCAN map snapshots.

This is a shadow-only finite voxel and local planar surface model, not a proof of
foot contact, locomotion stability, or unobserved terrain. Unknown cells, missing
observations, stale sources and incomplete snapshots never mean free space.
No ROS, simulator, SDF, truth, wall clock, or controller mutation is used here.
"""
from collections import Counter
import hashlib
import json
import math

import numpy as np

SCHEMA = 'teacher_corridor_certificate/v1'
SNAPSHOT_SCHEMA = 'teacher_scan_local_snapshot/v1'
SUPPORT_SCHEMA = 'teacher_measured_planar_support/v1'
DEFAULT_LIMITS = dict(horizon_m=1.2, max_half_width_m=.30, width_step_m=.04,
    max_speed_mps=.3, brake_decel_mps2=.15, latency_s=.3, stop_margin_m=.10,
    body_radius_m=.25, body_offset_m=.18, body_below_m=.12, body_above_m=.12,
    position_uncertainty_m=.08, yaw_half_range_rad=math.pi,
    map_max_age_ns=300_000_000, cell_max_age_ns=300_000_000,
    support_max_age_ns=300_000_000, state_max_age_ns=300_000_000,
    sensor_pose_pair_max_ns=100_000_000, body_ground_height_m=.4,
    support_height_tolerance_m=.06, max_support_slope_rad=.35,
    support_patch_radius_m=.16, support_edge_margin_m=.01,
    support_max_residual_m=.02, support_max_gap_m=.06,
    support_min_points=6, interval_m=.08)


def _sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _limits(values):
    result = {**DEFAULT_LIMITS, **values}
    for key, value in result.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError('invalid_limit:' + key)
        if value < 0 or (key not in ('body_offset_m', 'position_uncertainty_m',
                                    'support_edge_margin_m') and value == 0):
            raise ValueError('invalid_limit:' + key)
    if result['horizon_m'] > 2.5 or result['max_half_width_m'] > .5:
        raise ValueError('unbounded_request')
    if not 0 < result['yaw_half_range_rad'] <= math.pi:
        raise ValueError('invalid_yaw_bound')
    if result['body_radius_m'] < .25 or result['body_offset_m'] < .18:
        raise ValueError('footprint_smaller_than_existing_SCAN')
    if (result['width_step_m'] < .01 or result['interval_m'] < .02 or
            not .05 <= result['support_patch_radius_m'] <= .30 or
            not 6 <= result['support_min_points'] <= 50):
        raise ValueError('unbounded_or_insufficient_sampling_configuration')
    for key in ('map_max_age_ns', 'cell_max_age_ns', 'support_max_age_ns', 'state_max_age_ns'):
        if result[key] > 300_000_000:
            raise ValueError('original_300ms_freshness_must_not_be_relaxed')
    return result


def _path(receipt):
    points = np.asarray(receipt['points_xyz'], dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not 2 <= len(points) <= 4096 or
            not np.isfinite(points).all() or receipt['frame_id'] != 'camera_init' or
            not isinstance(receipt['path_id'], str) or not receipt['path_id'] or
            not isinstance(receipt['layer_id'], str) or not receipt['layer_id'] or
            not isinstance(receipt['path_sha256'], str) or len(receipt['path_sha256']) != 64):
        raise ValueError('invalid_path_binding')
    lengths = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    if np.any(lengths < 1e-9):
        raise ValueError('degenerate_horizontal_path')
    return points, np.r_[0., np.cumsum(lengths)]


def _point(points, cumulative, distance):
    i = min(len(points)-2, max(0, int(np.searchsorted(cumulative, distance, side='right')-1)))
    t = np.clip((distance-cumulative[i])/(cumulative[i+1]-cumulative[i]), 0., 1.)
    return points[i] + t*(points[i+1]-points[i])


def request_prefix(path_receipt, progress_m, horizon_m=1.2):
    """Return measured-geometry prefix points for a nav_msgs/Path map request."""
    points, cumulative = _path(path_receipt)
    start = max(0., min(float(progress_m), cumulative[-1]))
    end = min(cumulative[-1], start+float(horizon_m))
    if end <= start:
        return []
    distances = np.linspace(start, end, min(128, max(2, math.ceil((end-start)/.04)+1)))
    return [_point(points, cumulative, s).tolist() for s in distances]


def _snapshot(snapshot, now_ns, limits):
    if snapshot is None:
        raise ValueError('map_snapshot_missing')
    if (snapshot.get('schema') != SNAPSHOT_SCHEMA or snapshot.get('frame_id') != 'camera_init' or
            snapshot.get('source') != 'actual_registered_lidar_raycast' or
            snapshot.get('navigation_ground_truth_used') is not False):
        raise ValueError('invalid_map_provenance')
    if not snapshot.get('complete'):
        raise ValueError('incomplete_map:' + str(snapshot.get('reason', 'unknown')))
    for key in ('stamp_ns', 'source_cloud_stamp_ns', 'source_sensor_pose_stamp_ns'):
        stamp = snapshot[key]
        if not isinstance(stamp, int) or not 0 < stamp <= now_ns:
            raise ValueError('map_source_future_or_missing:' + key)
        if now_ns-stamp > limits['map_max_age_ns']:
            raise ValueError('map_source_stale:' + key)
    if abs(snapshot['source_cloud_stamp_ns']-snapshot['source_sensor_pose_stamp_ns']) > limits['sensor_pose_pair_max_ns']:
        raise ValueError('map_sensor_pose_cloud_pair_gap')
    shape = np.asarray(snapshot['shape_xyz'])
    origin = np.asarray(snapshot['origin_index_xyz'])
    resolution = float(snapshot['resolution_m'])
    if (shape.shape != (3,) or origin.shape != (3,) or
            not np.issubdtype(shape.dtype, np.integer) or not np.issubdtype(origin.dtype, np.integer) or
            np.any(shape <= 0) or int(np.prod(shape)) > 65536 or not .01 <= resolution <= .20):
        raise ValueError('invalid_map_extent')
    states = snapshot['states']
    ages = np.asarray(snapshot['cell_observation_age_ms'])
    if (not isinstance(states, str) or len(states) != int(np.prod(shape)) or
            any(x not in '012' for x in states) or ages.shape != (len(states),) or
            not np.issubdtype(ages.dtype, np.integer) or np.any(ages < -1) or np.any(ages > 1_000_000_000_000)):
        raise ValueError('invalid_map_payload')
    if snapshot.get('raw_occupancy_not_robot_inflated') is not True:
        raise ValueError('raw_occupancy_required')
    return shape, origin, resolution, np.frombuffer(states.encode('ascii'), dtype=np.uint8).reshape(tuple(shape))-48, ages.reshape(tuple(shape))


def _hull(points):
    # Monotonic chain, with collinear boundary samples removed.
    pts = sorted(set(map(tuple, points)))
    if len(pts) < 3:
        return []
    cross = lambda a,b,c: (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2],lower[-1],p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2],upper[-1],p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def _contains(hull, point):
    return len(hull) >= 3 and all((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0]) >= -1e-10
        for a,b in zip(hull, hull[1:]+hull[:1]))


def _expected_body_z(xy, points):
    delta = np.diff(points, axis=0)
    t = np.clip(np.sum((xy-points[:-1,:2])*delta[:,:2], axis=1)/np.sum(delta[:,:2]**2, axis=1), 0., 1.)
    nearest = points[:-1]+t[:,None]*delta
    distance = np.linalg.norm(nearest[:,:2]-xy, axis=1)
    minimum = distance.min()
    candidates = nearest[distance <= minimum+1e-5,2]
    if np.ptp(candidates) > .06:
        raise ValueError('ambiguous_path_layer')
    return float(candidates.mean())


def derive_measured_support(map_snapshot, path_receipt, limits=None):
    """Build bounded observed planar patches, with per-cell rejection reasons.

    A route supplies only a narrow z window for selecting actual returns. It
    never creates ground. Four-corner convex-hull coverage, a 3x3 observed-hit
    gap bound, residual, slope and height checks define this finite model.
    """
    config = _limits(limits or {})
    points, _ = _path(path_receipt)
    report = dict(schema=SUPPORT_SCHEMA, status='unavailable', source='actual_measured_surface_points',
        frame_id=path_receipt['frame_id'], layer_id=path_receipt['layer_id'],
        path_id=path_receipt['path_id'], path_sha256=path_receipt['path_sha256'],
        map_revision=None if map_snapshot is None else map_snapshot.get('revision'),
        map_snapshot_sha256=None if map_snapshot is None else _sha(map_snapshot),
        source_cloud_stamp_ns=None if map_snapshot is None else map_snapshot.get('source_cloud_stamp_ns'),
        navigation_ground_truth_used=False, cells={}, failure_counts={},
        assumption='local sampled planar support; no foot-contact or dynamics guarantee')
    if map_snapshot is None or not map_snapshot.get('surface_points_complete'):
        report['reason']='support_surface_snapshot_incomplete'; return report
    surface = np.asarray(map_snapshot.get('surface_points_xyz', []), dtype=float)
    if (surface.ndim != 2 or surface.shape[1] != 3 or not 6 <= len(surface) <= 12000 or
            not np.isfinite(surface).all()):
        report['reason']='support_surface_points_missing_or_invalid'; return report
    res = float(map_snapshot['resolution_m'])
    # Work only inside the exported region; a request never grows from absence.
    origin = np.asarray(map_snapshot['origin_index_xyz'], int)
    shape = np.asarray(map_snapshot['shape_xyz'], int)
    pad = config['max_half_width_m']+config['body_radius_m']+config['body_offset_m']+config['position_uncertainty_m']
    lo = np.maximum(origin[:2], np.floor((points[:,:2].min(axis=0)-pad)/res).astype(int))
    hi = np.minimum(origin[:2]+shape[:2]-1, np.floor((points[:,:2].max(axis=0)+pad)/res).astype(int))
    if np.prod(np.maximum(hi-lo+1, 0)) > 4096:
        report['reason']='support_cell_budget_exceeded'; return report
    radius=config['support_patch_radius_m']; failures=Counter()
    bucket={}
    for index, point in enumerate(surface):
        key=tuple(np.floor(point[:2]/radius).astype(int)); bucket.setdefault(key, []).append(index)
    margin=config['support_edge_margin_m']
    for ix in range(int(lo[0]), int(hi[0])+1):
        for iy in range(int(lo[1]), int(hi[1])+1):
            center=(np.array([ix,iy],float)+.5)*res; key=f'{ix},{iy}'
            cell=dict(status='unavailable', index_xy=[ix,iy]); report['cells'][key]=cell
            def reject(reason):
                cell['reason']=reason; failures[reason]+=1
            try: body_z=_expected_body_z(center, points)
            except ValueError:
                reject('ambiguous_path_layer'); continue
            bz=np.floor(center/radius).astype(int)
            indices=[]
            for dx in (-1,0,1):
                for dy in (-1,0,1): indices.extend(bucket.get((bz[0]+dx,bz[1]+dy), []))
            local=surface[indices] if indices else np.empty((0,3))
            ground=body_z-config['body_ground_height_m']
            local=local[(np.linalg.norm(local[:,:2]-center,axis=1)<=radius) &
                (abs(local[:,2]-ground)<=config['support_height_tolerance_m']+radius*math.tan(config['max_support_slope_rad']))]
            cell['point_count']=len(local)
            if len(local)<config['support_min_points']:
                reject('insufficient_measured_points'); continue
            xy=local[:,:2]-center
            corners=np.array([[sx*(res/2+margin),sy*(res/2+margin)] for sx in (-1,1) for sy in (-1,1)])
            hull=_hull(xy)
            if not all(_contains(hull,p) for p in corners):
                reject('support_edge_or_incomplete_hull'); continue
            probes=np.array([[sx*res/2,sy*res/2] for sx in (-1,0,1) for sy in (-1,0,1)])
            gap=float(np.max(np.min(np.linalg.norm(probes[:,None,:]-xy[None,:,:],axis=2),axis=1)))
            cell['maximum_observation_gap_m']=gap
            if gap>config['support_max_gap_m']:
                reject('support_observation_gap'); continue
            A=np.c_[xy,np.ones(len(xy))]
            plane,_,rank,_=np.linalg.lstsq(A,local[:,2],rcond=None)
            if rank<3:
                reject('support_plane_rank_deficient'); continue
            residual=float(np.max(abs(A@plane-local[:,2])))
            slope=math.atan(float(np.linalg.norm(plane[:2])))
            cell.update(max_residual_m=residual,slope_rad=slope)
            if residual>config['support_max_residual_m']:
                reject('support_plane_residual'); continue
            if slope>config['max_support_slope_rad']:
                reject('support_slope'); continue
            if abs(plane[2]-ground)>config['support_height_tolerance_m']:
                reject('support_height'); continue
            cell.update(status='observed_planar',ground_z_m=float(plane[2]),
                plane_local_xy=[float(v)for v in plane],body_reference_z_m=body_z)
    report.update(status='measured_model',reason=None,failure_counts=dict(failures),
        observed_planar_cells=sum(c['status']=='observed_planar'for c in report['cells'].values()),
        resolution_m=res,model_limits_sha256=_sha(config))
    report['support_sha256']=_sha(report)
    return report


def _trig_max(angle, half, cosine):
    if half>=math.pi:
        return 1.
    shift=0. if cosine else math.pi/2
    values=[abs(math.cos(angle-half-shift)),abs(math.cos(angle+half-shift))]
    for k in range(-4,5):
        a=shift+k*math.pi
        if angle-half<=a<=angle+half: values.append(1.)
    return max(values)


def certify_corridor(path_receipt, map_snapshot, support_snapshot, actual_state, limits, now_ns):
    """Return certified/unavailable/blocked geometry evidence; never a command.

    `certified` is conditional on the stated voxel and measured-plane model.
    `shadow_only` stays true: the runtime must separately validate and authorize
    actuation. No previous certificate may replace a missing current result.
    """
    result=dict(schema=SCHEMA,status='unavailable',reason=None,shadow_only=True,
        navigation_ground_truth_used=False,intervals=[],failure_counts={},
        certificate_scope='finite voxel occupancy and observed planar support model; excludes foot-contact and dynamics')
    try:
        config=_limits(limits); points,cumulative=_path(path_receipt)
        if isinstance(now_ns,bool)or not isinstance(now_ns,int)or now_ns<=0:
            raise ValueError('invalid_clock')
        result['binding']=dict(path_id=path_receipt['path_id'],path_sha256=path_receipt['path_sha256'],
            points_float64_sha256=hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()).hexdigest(),
            frame_id=path_receipt['frame_id'],layer_id=path_receipt['layer_id'],
            map_revision=None if map_snapshot is None else map_snapshot.get('revision'),
            map_snapshot_sha256=None if map_snapshot is None else _sha(map_snapshot),
            limits_sha256=_sha(config),evaluated_at_ns=now_ns)
        shape,origin,res,states,ages=_snapshot(map_snapshot,now_ns,config)
        stamp=actual_state['stamp_ns']; pose=np.asarray(actual_state['position_world_xyz'],float)
        speed=float(actual_state['speed_mps']); yaw=float(actual_state['yaw_rad']); start=float(actual_state['progress_m'])
        if (not isinstance(stamp,int)or not 0<stamp<=now_ns or now_ns-stamp>config['state_max_age_ns'] or
                pose.shape!=(3,)or not np.isfinite(pose).all()or not all(map(math.isfinite,(speed,yaw,start)))or
                speed<0 or not 0<=start<cumulative[-1]or path_receipt['stamp_ns']>stamp):
            raise ValueError('invalid_or_stale_actual_state')
        if path_receipt.get('request_stamp_ns') is not None and path_receipt['request_stamp_ns']!=map_snapshot['request_stamp_ns']:
            raise ValueError('map_request_binding_mismatch')
        end=min(cumulative[-1],start+config['horizon_m'])
        braking_speed=max(speed,config['max_speed_mps'])
        stop=braking_speed*config['latency_s']+braking_speed**2/(2*config['brake_decel_mps2'])+config['stop_margin_m']
        result.update(s_start_m=start,s_end_m=float(end),stopping_distance_m=stop,
            stopping_speed_bound_mps=braking_speed,
            model_assumptions=dict(brake_decel_mps2=config['brake_decel_mps2'],
                brake_deceleration_physically_verified=False,latency_s=config['latency_s'],
                body_radius_m=config['body_radius_m'],body_offset_m=config['body_offset_m'],
                body_vertical_interval_m=[-config['body_below_m'],config['body_above_m']],
                position_uncertainty_m=config['position_uncertainty_m'],
                support='sampled local planar surface; foot contact and locomotion dynamics unverified'))
        if end-start<stop:
            raise ValueError('verified_prefix_shorter_than_stopping_distance')
        # Only a short prefix is fitted; using the whole route could alias floors.
        prefix=request_prefix(path_receipt,start,config['horizon_m'])
        local_receipt={**path_receipt,'points_xyz':prefix}
        support=derive_measured_support(map_snapshot,local_receipt,config)if support_snapshot is None else support_snapshot
        result['support_diagnostic']={k:v for k,v in support.items()if k!='cells'}
        if (support.get('schema')!=SUPPORT_SCHEMA or support.get('status')!='measured_model' or
                support.get('source')!='actual_measured_surface_points' or support.get('navigation_ground_truth_used')is not False or
                support.get('path_id')!=path_receipt['path_id']or support.get('path_sha256')!=path_receipt['path_sha256']or
                support.get('map_revision')!=map_snapshot['revision']or support.get('layer_id')!=path_receipt['layer_id']or
                support.get('map_snapshot_sha256')!=_sha(map_snapshot)or
                support.get('frame_id')!=path_receipt['frame_id']or support.get('model_limits_sha256')!=_sha(config)):
            raise ValueError('support_unavailable_or_binding_mismatch:'+str(support.get('reason')))
        support_age=now_ns-int(support['source_cloud_stamp_ns'])
        if not 0<=support_age<=config['support_max_age_ns']:
            raise ValueError('support_source_stale_or_future')
        failures=Counter(); width_failures=Counter(); occupied=False
        valid_until=min(map_snapshot['source_cloud_stamp_ns']+config['map_max_age_ns'],
            support['source_cloud_stamp_ns']+config['support_max_age_ns'],stamp+config['state_max_age_ns'])
        bounds=np.linspace(start,end,max(2,math.ceil((end-start)/config['interval_m'])+1))
        def check(lower,upper):
            nonlocal valid_until
            a=np.floor(lower/res).astype(int);b=np.floor(upper/res).astype(int)
            if np.any(a<origin)or np.any(b>=origin+shape):return 'outside_snapshot'
            sl=tuple(slice(int(a[i]-origin[i]),int(b[i]-origin[i]+1))for i in range(3))
            st=states[sl]; ag=ages[sl]
            if np.any(st==2):return 'occupied_body_sweep'
            if np.any(st!=1):return 'unknown_body_sweep'
            age_ns=ag*1_000_000+(now_ns-map_snapshot['stamp_ns'])
            if np.any(ag<0)or np.any(age_ns>config['cell_max_age_ns']):return 'stale_free_body_sweep'
            # Entire rectangular swept footprint needs an observed support model.
            for x in range(a[0],b[0]+1):
                for y in range(a[1],b[1]+1):
                    cell=support['cells'].get(f'{x},{y}')
                    if not cell or cell.get('status')!='observed_planar':
                        return 'support:'+('missing_cell'if not cell else cell.get('reason','unavailable'))
            valid_until=min(valid_until,now_ns+config['cell_max_age_ns']-int(age_ns.max()))
            return None
        for s0,s1 in zip(bounds[:-1],bounds[1:]):
            mid=(s0+s1)/2
            p0=_point(points,cumulative,max(start,mid-.1));p1=_point(points,cumulative,min(end,mid+.1))
            tangent=p1[:2]-p0[:2]; norm=float(np.linalg.norm(tangent))
            if norm<.01:raise ValueError('spatial_direction_unobservable')
            tangent/=norm;normal=np.array([-tangent[1],tangent[0]]);heading=math.atan2(tangent[1],tangent[0])
            sample=np.vstack([_point(points,cumulative,s0),points[(cumulative>s0)&(cumulative<s1)],_point(points,cumulative,s1)])
            center_lo=sample.min(axis=0);center_hi=sample.max(axis=0)
            ex=config['body_radius_m']+config['body_offset_m']*_trig_max(heading,config['yaw_half_range_rad'],True)
            ey=config['body_radius_m']+config['body_offset_m']*_trig_max(heading,config['yaw_half_range_rad'],False)
            uncertainty=config['position_uncertainty_m']
            def box(left,right):
                offsets=np.vstack([normal*left,-normal*right])
                lower=center_lo-np.array([ex+uncertainty,ey+uncertainty,config['body_below_m']+uncertainty])
                upper=center_hi+np.array([ex+uncertainty,ey+uncertainty,config['body_above_m']+uncertainty])
                lower[:2]+=offsets.min(axis=0);upper[:2]+=offsets.max(axis=0)
                return lower,upper
            reason=check(*box(0.,0.))
            if reason:
                failures[reason]+=1;occupied=occupied or reason=='occupied_body_sweep'
                if 'first_rejection'not in result:result['first_rejection']=dict(s_m=float(s0),reason=reason)
                continue
            left=right=0.
            for side in ('left','right'):
                for width in np.arange(config['width_step_m'],config['max_half_width_m']+1e-9,config['width_step_m']):
                    l,r=(float(width),right)if side=='left'else(left,float(width))
                    reason=check(*box(l,r))
                    if reason:width_failures[reason]+=1;break
                    left,right=l,r
            lower,upper=box(left,right)
            result['intervals'].append(dict(s_start_m=float(s0),s_end_m=float(s1),
                left_m=left,right_m=right,heading_rad=heading,yaw_half_range_rad=config['yaw_half_range_rad'],
                tangent_xy=tangent.tolist(),center_xyz=_point(points,cumulative,mid).tolist(),
                swept_aabb_min_xyz=lower.tolist(),swept_aabb_max_xyz=upper.tolist()))
        result.update(failure_counts=dict(failures),width_rejection_counts=dict(width_failures))
        if failures:
            result.update(status='blocked'if occupied else'unavailable',reason='prefix_not_fully_verified')
            return result
        first=result['intervals'][0];base=_point(points,cumulative,start);normal=np.array([-first['tangent_xy'][1],first['tangent_xy'][0]])
        lateral=float((pose[:2]-base[:2])@normal)
        longitudinal=float((pose[:2]-base[:2])@np.asarray(first['tangent_xy']))
        if (lateral>first['left_m']or -lateral>first['right_m']or abs(pose[2]-base[2])>uncertainty or
                abs(longitudinal)>uncertainty):
            result.update(status='blocked',reason='actual_state_outside_corridor');return result
        yaw_error=math.atan2(math.sin(yaw-first['heading_rad']),math.cos(yaw-first['heading_rad']))
        if abs(yaw_error)>config['yaw_half_range_rad']:
            result.update(status='blocked',reason='actual_yaw_outside_verified_sweep');return result
        result.update(status='certified',reason='finite_geometric_model_verified',
            left_m=min(x['left_m']for x in result['intervals']),right_m=min(x['right_m']for x in result['intervals']),
            valid_until_ns=int(valid_until),support_sha256=support['support_sha256'])
        result['certificate_sha256']=_sha(result)
        return result
    except (ValueError,TypeError,KeyError,IndexError,OverflowError)as exc:
        result['reason']=str(exc)or type(exc).__name__;return result
