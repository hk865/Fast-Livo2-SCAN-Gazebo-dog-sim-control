"""Synthetic geometry checks only: no policy, ROS, or simulator is imported."""
import json
import numpy as np
from plane_observer import fit_candidate_planes, select_support_plane, vertical_observation


def run():
    rng = np.random.default_rng(123)
    xy = rng.uniform(-2, 2, (1600, 2))
    def plane(z, gx=0.):
        return np.c_[xy, z+gx*xy[:, 0]+rng.normal(0, .002, len(xy))]
    checks = {}
    selected = select_support_plane(fit_candidate_planes(plane(-.3), [0, 0, 0]))
    checks['horizontal_support'] = selected['status'] == 'observed'
    checks['height_accuracy_5mm'] = abs(selected['selected']['height_at_body_xy']+.3) < .005
    candidates = fit_candidate_planes(np.concatenate([plane(-.3), plane(-1.5), plane(2.1)]), [0, 0, 0])
    s = select_support_plane(candidates)
    checks['lower_floor_and_ceiling_rejected_by_measured_clearance'] = s['status'] == 'observed' and abs(s['selected']['height_at_body_xy']+.3)<.005
    checks['lower_plane_preserved_as_rejected_candidate'] = any(x['body_origin_clearance']>1 for x in candidates)
    checks['upper_plane_preserved_as_rejected_candidate'] = any(x['body_origin_clearance']<0 for x in candidates)
    slope = select_support_plane(fit_candidate_planes(plane(-.3, .1), [0, 0, 0]))
    checks['ramp_normal_and_clearance'] = slope['status']=='observed' and abs(slope['selected']['tilt_rad']-np.arctan(.1))<.01
    shifted = select_support_plane(fit_candidate_planes(plane(.7), [0, 0, 1.]))
    obs = vertical_observation(shifted, -.3, 1.2)
    checks['measured_map_bias_not_body_goal_bias'] = abs(obs['delta_z']-.2)<.005
    ambiguous = [dict(selected['selected']),dict(selected['selected'])]
    ambiguous[1]['height_at_body_xy'] += .08
    checks['two_eligible_planes_rejected'] = select_support_plane(ambiguous)['status']=='unverified'
    checks['missing_plane_rejected'] = select_support_plane([])['status']=='unverified'
    far = select_support_plane(fit_candidate_planes(plane(-1.5),[0,0,0]))
    checks['no_near_ground_does_not_fallback'] = far['status']=='unverified'
    p = plane(-.3); original=p.copy();fit_candidate_planes(p,[0,0,0])
    checks['input_points_unchanged'] = np.array_equal(p,original)
    try: vertical_observation(selected, float('nan'), 1.2)
    except ValueError: checks['nonfinite_prior_rejected']=True
    else:checks['nonfinite_prior_rejected']=False
    checks={k:bool(v) for k,v in checks.items()}
    return dict(schema='pure_ground_plane_observer_checks/v1',status='passed' if all(checks.values()) else 'failed',checks=checks,check_count=len(checks),no_runtime_model_ROS_simulator=True)


if __name__=='__main__':
    result=run();print(json.dumps(result,indent=2));raise SystemExit(result['status']!='passed')
